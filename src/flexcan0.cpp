#include "flexcan0.hpp"
#include "error_report.hpp"

#include <Arduino.h>
#include <kinetis.h>
#include <atomic>

#if !defined(__MK20DX256__)
#error "This FlexCAN0 implementation is for Teensy 3.2 / MK20DX256 only."
#endif

namespace
{
constexpr uint32_t kSupportedBitrate = 500000u;
constexpr uint32_t kInitTimeoutUs = 1000u;
constexpr uint32_t kStandardDataFilterMask = 0xC0000000u; // Compare RTR and IDE.
constexpr uint8_t kDlcMask = 0x0Fu;
constexpr uint8_t kMaxMbIndex = 15u;
// FIFO occupies MB0..5, its eight Format-A filter words occupy MB6..7.
constexpr uint8_t kTxMb = 8u;
constexpr uint8_t kRxMb = 0u; // FIFO output, not a standalone RX mailbox.
constexpr uint32_t kAllMbFlags = 0x0000FFFFu;
constexpr uint32_t kFifoAvailable = 1u << 5;
constexpr uint32_t kFifoWarning = 1u << 6;
constexpr uint32_t kFifoOverflow = 1u << 7;
constexpr uint8_t kRxIsrBudget = 6u;
constexpr uint8_t kRxIrqPriority = 64u;

constexpr uintptr_t kCan0MbBase = 0x40024080u;
constexpr uintptr_t kCan0MbStride = 0x10u;
constexpr uintptr_t kMbCsOffset = 0x0u;
constexpr uintptr_t kMbIdOffset = 0x4u;
constexpr uintptr_t kMbWord0Offset = 0x8u;
constexpr uintptr_t kMbWord1Offset = 0xCu;

constexpr uint32_t kMcrMaxMbMask = 0x0000007Fu;
constexpr uint32_t kMcrIdamMask = 3u << 8;
constexpr uint32_t kMcrIrmq = 1u << 16;
constexpr uint32_t kMcrSrxDis = 0x00020000u;
constexpr uint32_t kMcrLpmAck = 0x00100000u;
constexpr uint32_t kMcrFrzAck = 0x01000000u;
constexpr uint32_t kMcrSoftRst = 0x02000000u;
constexpr uint32_t kMcrHalt = 0x10000000u;
constexpr uint32_t kMcrRfen = 0x20000000u;
constexpr uint32_t kMcrFrz = 0x40000000u;
constexpr uint32_t kMcrMdis = 0x80000000u;

constexpr uint32_t kCtrl1ClkSrc = 0x00002000u;
// 16 MHz OSCERCLK / (PRESDIV + 1 = 2) / 16 TQ = 500 kbit/s.
// PROPSEG=3 TQ, PSEG1=8 TQ and PSEG2=4 TQ give a 75% sample point.
constexpr uint32_t kCtrl1Timing500k = 0x017B0002u;

constexpr uint32_t kMbCodeShift = 24u;
constexpr uint32_t kMbCodeMask = 0x0F000000u;
constexpr uint32_t kMbDlcShift = 16u;
constexpr uint32_t kMbIdStdShift = 18u;
constexpr uint32_t kMbIde = 1u << 21;
constexpr uint32_t kMbRtr = 1u << 20;

constexpr uint8_t kMbCodeTxInactive = 0b1000u;
constexpr uint8_t kMbCodeTxAbort = 0b1001u;
constexpr uint8_t kMbCodeTxDataOnce = 0b1100u;
bool g_can_initialized = false;
static_assert(ATOMIC_INT_LOCK_FREE == 2, "RX ISR requires lock-free 32-bit atomics");
static_assert((CAN_RX_QUEUE_CAPACITY & (CAN_RX_QUEUE_CAPACITY - 1u)) == 0,
              "RX queue capacity must be a power of two");
struct RxEntry { CanFrame frame; uint32_t received_us; };
RxEntry g_rx_queue[CAN_RX_QUEUE_CAPACITY];
// One producer (CAN ISR), one consumer (main). Publish only complete copies.
std::atomic<uint32_t> g_rx_head{0}, g_rx_tail{0};
std::atomic<uint32_t> g_rx_overrun_reports_remaining{0};
std::atomic<uint32_t> g_fifo_overflows{0}, g_queue_overflows{0};
std::atomic<uint32_t> g_queue_high_water{0}, g_max_queue_age_us{0};

volatile uint32_t &mb_reg(uint8_t mb, uintptr_t offset)
{
    return *reinterpret_cast<volatile uint32_t *>(kCan0MbBase + (static_cast<uintptr_t>(mb) * kCan0MbStride) + offset);
}

volatile uint32_t &mb_cs(uint8_t mb)
{
    return mb_reg(mb, kMbCsOffset);
}

volatile uint32_t &mb_id(uint8_t mb)
{
    return mb_reg(mb, kMbIdOffset);
}

volatile uint32_t &mb_word0(uint8_t mb)
{
    return mb_reg(mb, kMbWord0Offset);
}

volatile uint32_t &mb_word1(uint8_t mb)
{
    return mb_reg(mb, kMbWord1Offset);
}

uint32_t mb_code(uint8_t code)
{
    return static_cast<uint32_t>(code) << kMbCodeShift;
}

uint8_t get_mb_code(uint8_t mb)
{
    return static_cast<uint8_t>((mb_cs(mb) & kMbCodeMask) >> kMbCodeShift);
}

uint32_t elapsed_us(uint32_t start)
{
    return static_cast<uint32_t>(micros() - start);
}

bool wait_until_set(volatile uint32_t &reg, uint32_t mask, uint32_t timeout_us)
{
    const uint32_t start = micros();
    while ((reg & mask) == 0u)
    {
        if (elapsed_us(start) >= timeout_us)
            return false;
    }
    return true;
}

bool wait_until_clear(volatile uint32_t &reg, uint32_t mask, uint32_t timeout_us)
{
    const uint32_t start = micros();
    while ((reg & mask) != 0u)
    {
        if (elapsed_us(start) >= timeout_us)
            return false;
    }
    return true;
}

uint32_t pack_word(const uint8_t *data, uint8_t start_index, uint8_t dlc)
{
    uint32_t word = 0u;
    for (uint8_t i = 0; i < 4u; ++i)
    {
        const uint8_t data_index = start_index + i;
        const uint8_t value = (data_index < dlc) ? data[data_index] : 0u;
        word |= static_cast<uint32_t>(value) << (24u - (8u * i));
    }
    return word;
}

void unpack_word(uint32_t word, uint8_t *data, uint8_t start_index, uint8_t dlc)
{
    for (uint8_t i = 0; i < 4u; ++i)
    {
        const uint8_t data_index = start_index + i;
        if (data_index < dlc)
            data[data_index] = static_cast<uint8_t>(word >> (24u - (8u * i)));
    }
}

void can_rx_isr()
{
    // FIFO is held until BUF5I is acknowledged. It does NOT use the standalone
    // mailbox BUSY/TIMER unlock protocol (K20 RM 44.4.6.3 and 44.4.7).
    const uint32_t flags = CAN0_IFLAG1;
    if (flags & kFifoOverflow) {
        g_fifo_overflows.fetch_add(1, std::memory_order_relaxed);
        g_rx_overrun_reports_remaining.store(ERROR_REPORT_REPEAT_COUNT, std::memory_order_relaxed);
    }
    const uint32_t errors = flags & (kFifoWarning | kFifoOverflow);
    if (errors) CAN0_IFLAG1 = errors; // W1C only observed RX flags, never TX MB8.

    // Bounded per entry. A still nonempty FIFO reasserts the IRQ after return.
    for (uint8_t i = 0; i < kRxIsrBudget && (CAN0_IFLAG1 & kFifoAvailable); ++i) {
        RxEntry entry = {};
        entry.received_us = micros();
        const uint32_t cs = mb_cs(kRxMb);
        entry.frame.id = static_cast<uint16_t>((mb_id(kRxMb) >> kMbIdStdShift) & CAN_STANDARD_ID_MAX);
        uint8_t dlc = static_cast<uint8_t>((cs >> kMbDlcShift) & kDlcMask);
        entry.frame.dlc = dlc > CAN_PAYLOAD_SIZE ? CAN_PAYLOAD_SIZE : dlc;
        unpack_word(mb_word0(kRxMb), entry.frame.data, 0, entry.frame.dlc);
        unpack_word(mb_word1(kRxMb), entry.frame.data, 4, entry.frame.dlc);
        CAN0_IFLAG1 = kFifoAvailable; // Pop only after the complete local copy.
        // Match the public standard-data-frame API, even if filtering is changed.
        if (cs & (kMbIde | kMbRtr)) continue;

        const uint32_t head = g_rx_head.load(std::memory_order_relaxed);
        const uint32_t tail = g_rx_tail.load(std::memory_order_acquire);
        const uint32_t used = head - tail;
        if (used >= CAN_RX_QUEUE_CAPACITY) {
            // Preserve queued order; drop newest, but drain HW to avoid IRQ storms.
            g_queue_overflows.fetch_add(1, std::memory_order_relaxed);
            g_rx_overrun_reports_remaining.store(ERROR_REPORT_REPEAT_COUNT, std::memory_order_relaxed);
            continue;
        }
        g_rx_queue[head & (CAN_RX_QUEUE_CAPACITY - 1u)] = entry;
        g_rx_head.store(head + 1u, std::memory_order_release);
        if (used + 1u > g_queue_high_water.load(std::memory_order_relaxed))
            g_queue_high_water.store(used + 1u, std::memory_order_relaxed);
    }
}

bool abort_tx_mb(uint32_t timeout_us)
{
    if (get_mb_code(kTxMb) == kMbCodeTxInactive)
        return true;

    const uint32_t flag = 1u << kTxMb;
    CAN0_IFLAG1 = flag;
    mb_cs(kTxMb) = mb_code(kMbCodeTxAbort);

    if (!wait_until_set(CAN0_IFLAG1, flag, timeout_us))
        return false;

    CAN0_IFLAG1 = flag;
    return get_mb_code(kTxMb) == kMbCodeTxInactive || get_mb_code(kTxMb) == kMbCodeTxAbort;
}
}

bool can_begin(uint32_t bitrate)
{
    NVIC_DISABLE_IRQ(IRQ_CAN_MESSAGE);
    NVIC_CLEAR_PENDING(IRQ_CAN_MESSAGE);
    g_can_initialized = false;
    g_rx_overrun_reports_remaining = 0;
    g_rx_head = 0; g_rx_tail = 0;
    g_fifo_overflows = 0; g_queue_overflows = 0;
    g_queue_high_water = 0; g_max_queue_age_us = 0;

    if (bitrate != kSupportedBitrate)
        return false;

    SIM_SCGC5 |= SIM_SCGC5_PORTA;
    SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
    OSC0_CR |= OSC_ERCLKEN;

    PORTA_PCR12 = PORT_PCR_MUX(2);
    PORTA_PCR13 = PORT_PCR_MUX(2);

    CAN0_CTRL1 &= ~kCtrl1ClkSrc;
    CAN0_MCR |= kMcrFrz;
    CAN0_MCR &= ~kMcrMdis;

    if (!wait_until_clear(CAN0_MCR, kMcrLpmAck, kInitTimeoutUs))
        return false;

    CAN0_MCR |= kMcrSoftRst;
    if (!wait_until_clear(CAN0_MCR, kMcrSoftRst, kInitTimeoutUs))
        return false;

    CAN0_MCR = (CAN0_MCR & ~(kMcrMaxMbMask | kMcrRfen | kMcrMdis | kMcrIdamMask | kMcrIrmq)) |
               kMcrFrz |
               kMcrHalt |
               kMcrSrxDis |
               kMaxMbIndex;

    if (!wait_until_set(CAN0_MCR, kMcrFrzAck, kInitTimeoutUs))
        return false;

    CAN0_CTRL1 = kCtrl1Timing500k;
    // RFFN=0: eight filter words; FIFO first; standard data frames only.
    CAN0_CTRL2 = 0u;
    CAN0_RXFGMASK = kStandardDataFilterMask; // Compare RTR/IDE, accept every standard ID.
    CAN0_RXMGMASK = 0u;
    CAN0_RX14MASK = 0u;
    CAN0_RX15MASK = 0u;
    CAN0_IMASK1 = 0u;
    CAN0_IFLAG1 = kAllMbFlags;

    for (uint8_t mb = 0u; mb <= kMaxMbIndex; ++mb)
    {
        mb_cs(mb) = 0u;
        mb_id(mb) = 0u;
        mb_word0(mb) = 0u;
        mb_word1(mb) = 0u;
    }

    // MB6..7 already contain eight zero Format-A filters (RTR=0, IDE=0).
    mb_cs(kTxMb) = mb_code(kMbCodeTxInactive);
    CAN0_MCR |= kMcrRfen;
    CAN0_IFLAG1 = kAllMbFlags;
    attachInterruptVector(IRQ_CAN_MESSAGE, can_rx_isr);
    NVIC_SET_PRIORITY(IRQ_CAN_MESSAGE, kRxIrqPriority);
    CAN0_IMASK1 = kFifoAvailable | kFifoWarning | kFifoOverflow;

    CAN0_MCR &= ~kMcrHalt;
    if (!wait_until_clear(CAN0_MCR, kMcrFrzAck, kInitTimeoutUs))
        return false;

    g_can_initialized = true;
    NVIC_ENABLE_IRQ(IRQ_CAN_MESSAGE);
    return true;
}

bool can_transmit(const CanFrame &frame, uint32_t timeout_us)
{
    if (!g_can_initialized || frame.id > CAN_STANDARD_ID_MAX || frame.dlc > CAN_PAYLOAD_SIZE)
        return false;

    const uint32_t flag = 1u << kTxMb;
    CAN0_IFLAG1 = flag;

    if (!abort_tx_mb(timeout_us))
        return false;

    mb_id(kTxMb) = (static_cast<uint32_t>(frame.id) & CAN_STANDARD_ID_MAX) << kMbIdStdShift;
    mb_word0(kTxMb) = pack_word(frame.data, 0u, frame.dlc);
    mb_word1(kTxMb) = pack_word(frame.data, 4u, frame.dlc);
    mb_cs(kTxMb) = mb_code(kMbCodeTxDataOnce) | (static_cast<uint32_t>(frame.dlc) << kMbDlcShift);

    if (!wait_until_set(CAN0_IFLAG1, flag, timeout_us))
        return false;

    CAN0_IFLAG1 = flag;
    return true;
}

bool can_receive(CanFrame &frame, uint32_t timeout_us)
{
    uint32_t received_us;
    return can_receive_timed(frame, received_us, timeout_us);
}

bool can_receive_timed(CanFrame &frame, uint32_t &received_us, uint32_t timeout_us)
{
    if (!g_can_initialized) return false;
    const uint32_t start = timeout_us ? micros() : 0u;
    do {
        const uint32_t tail = g_rx_tail.load(std::memory_order_relaxed);
        if (tail != g_rx_head.load(std::memory_order_acquire)) {
            const RxEntry entry = g_rx_queue[tail & (CAN_RX_QUEUE_CAPACITY - 1u)];
            g_rx_tail.store(tail + 1u, std::memory_order_release);
            frame = entry.frame;
            received_us = entry.received_us;
            const uint32_t age = micros() - received_us;
            if (age > g_max_queue_age_us.load(std::memory_order_relaxed))
                g_max_queue_age_us.store(age, std::memory_order_relaxed);
            return true;
        }
    } while (timeout_us && elapsed_us(start) < timeout_us);
    return false;
}

bool can_rx_overrun_detected()
{
    return g_rx_overrun_reports_remaining.load(std::memory_order_relaxed) > 0;
}

void can_rx_overrun_report_sent()
{
    // Do not lose an ISR's newly latched report while the main loop consumes one.
    uint32_t remaining = g_rx_overrun_reports_remaining.load(std::memory_order_relaxed);
    while (remaining && !g_rx_overrun_reports_remaining.compare_exchange_weak(
               remaining, remaining - 1u, std::memory_order_relaxed)) {}
}

void can_clear_rx_overrun_reports()
{
    g_rx_overrun_reports_remaining = 0;
}

bool can_is_initialized()
{
    return g_can_initialized;
}

bool can_rx_pending()
{
    return g_can_initialized &&
        (g_rx_head.load(std::memory_order_acquire) != g_rx_tail.load(std::memory_order_relaxed) ||
         (CAN0_IFLAG1 & kFifoAvailable) != 0u);
}

uint32_t can_rx_overrun_count()
{
    return g_fifo_overflows.load(std::memory_order_relaxed) +
           g_queue_overflows.load(std::memory_order_relaxed);
}

CanRxStats can_rx_stats()
{
    return {g_fifo_overflows.load(std::memory_order_relaxed),
            g_queue_overflows.load(std::memory_order_relaxed),
            g_queue_high_water.load(std::memory_order_relaxed),
            g_max_queue_age_us.load(std::memory_order_relaxed)};
}
