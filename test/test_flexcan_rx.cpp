// Actual driver with a FIFO pop/W1C model. No serial port or hardware access.
#define FLEXCAN0_HOST_TEST 1
#define __MK20DX256__ 1
#include <cstdio>
#include <deque>
#include "../src/flexcan0.cpp"

FakeCanFlags CAN0_IFLAG1;
volatile uint32_t fake_can_registers[14] = {};
bool fake_can_irq_enabled = false;
uint8_t fake_can_irq_priority = 0;
void (*fake_can_irq_handler)() = nullptr;
static uint32_t clock_us, timer_reads, inject_at;
static bool inject_on_clock, auto_init;
struct WireFrame { uint8_t pid, dlc; uint32_t extra_cs; };
static std::deque<WireFrame> fifo;

static void refresh_fifo()
{
    if (fifo.empty()) { CAN0_IFLAG1.value &= ~kFifoAvailable; return; }
    const WireFrame &f = fifo.front();
    mb_cs(kRxMb) = (static_cast<uint32_t>(f.dlc) << kMbDlcShift) | f.extra_cs;
    mb_id(kRxMb) = 0x701u << kMbIdStdShift;
    mb_word0(kRxMb) = (static_cast<uint32_t>(f.pid) << 24) | 0x001234u;
    mb_word1(kRxMb) = 0x56789ABCu;
    CAN0_IFLAG1.value |= kFifoAvailable;
}

void fake_can_flags_cleared(uint32_t mask)
{
    if ((mask & kFifoAvailable) && !fifo.empty()) fifo.pop_front();
    refresh_fifo(); // Next output is exposed only after acknowledgment.
}

static void arrive(uint8_t pid, uint8_t dlc = 8, uint32_t extra_cs = 0)
{
    fifo.push_back({pid, dlc, extra_cs});
    refresh_fifo();
}

uint32_t micros()
{
    ++clock_us;
    if (auto_init) {
        if (CAN0_MCR & kMcrSoftRst) CAN0_MCR &= ~kMcrSoftRst;
        if (!(CAN0_MCR & kMcrMdis)) CAN0_MCR &= ~kMcrLpmAck;
        if (CAN0_MCR & kMcrHalt) CAN0_MCR |= kMcrFrzAck;
        else CAN0_MCR &= ~kMcrFrzAck;
    }
    if (inject_on_clock && clock_us == inject_at) {
        inject_on_clock = false;
        arrive(201);
        if (fake_can_irq_enabled) fake_can_irq_handler();
    }
    return clock_us;
}

uint32_t fake_can_read_timer() { ++timer_reads; return 0; }

static void reset_model()
{
    fifo.clear();
    for (auto &r : fake_can_registers) r = 0;
    CAN0_IFLAG1.value = CAN0_IFLAG1.writes = 0;
    clock_us = timer_reads = 0;
    inject_at = 0;
    inject_on_clock = auto_init = false;
    fake_can_irq_enabled = true;
    fake_can_irq_handler = can_rx_isr;
    g_can_initialized = true;
    g_rx_head = 0; g_rx_tail = 0;
    g_fifo_overflows = 0; g_queue_overflows = 0;
    g_queue_high_water = 0; g_max_queue_age_us = 0;
    g_rx_overrun_reports_remaining = 0;
}

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); return false; \
} } while (0)

static bool check_init()
{
    reset_model(); auto_init = true;
    CHECK(can_begin(500000));
    CHECK(kTxMb == 8 && (CAN0_MCR & kMcrRfen));
    CHECK(!(CAN0_MCR & (kMcrIdamMask | kMcrIrmq | kMcrHalt)));
    CHECK(CAN0_CTRL2 == 0 && CAN0_RXFGMASK == 0xC0000000u);
    CHECK(CAN0_IMASK1 == (kFifoAvailable | kFifoWarning | kFifoOverflow));
    CHECK(fake_can_irq_enabled && fake_can_irq_priority == 64 && fake_can_irq_handler == can_rx_isr);
    for (uint8_t mb = 6; mb <= 7; ++mb)
        for (uint8_t word = 0; word < 4; ++word) CHECK(mb_reg(mb, word * 4) == 0);
    CHECK(get_mb_code(kTxMb) == kMbCodeTxInactive);
    CHECK(!can_begin(250000) && !can_is_initialized() && !fake_can_irq_enabled);
    return true;
}

static bool check_fifo_and_timestamps()
{
    reset_model();
    CAN0_IFLAG1.value = 1u << kTxMb;
    arrive(196); arrive(201);
    CHECK(can_rx_pending());
    CanFrame frame = {}; uint32_t received_us = 99;
    CHECK(!can_receive_timed(frame, received_us, 0)); // Consumer cannot touch HW.
    CHECK(fifo.size() == 2 && received_us == 99);
    clock_us = 100;
    can_rx_isr(); // CPU main loop remains busy; hardware has been emptied.
    CHECK(fifo.empty() && CAN0_IFLAG1.value == (1u << kTxMb));
    CHECK(g_rx_head == 2 && can_rx_pending() && timer_reads == 0);
    clock_us = 1000;
    CHECK(can_receive_timed(frame, received_us, 0) && frame.data[0] == 196);
    CHECK(received_us == 101 && frame.id == 0x701 && frame.dlc == 8);
    CHECK(frame.data[2] == 0x12 && frame.data[7] == 0xBC);
    CHECK(can_receive_timed(frame, received_us, 0) && frame.data[0] == 201 && received_us == 102);
    CHECK(!can_rx_pending() && !can_receive(frame, 0));
    CHECK(can_rx_stats().queue_high_water == 2 && can_rx_stats().max_queue_age_us == 900);

    arrive(196, 2); arrive(201, 8, kMbIde); arrive(201, 8, kMbRtr);
    can_rx_isr();
    CHECK(can_receive(frame, 0) && frame.dlc == 2 && frame.data[2] == 0 && frame.data[7] == 0);
    CHECK(!can_receive(frame, 0)); // No extended/remote frames masquerading as data.
    return true;
}

static bool check_capacity_loss_and_wrap()
{
    reset_model();
    // Use all 32 slots, including across unsigned producer/consumer wrap.
    g_rx_head = 0xFFFFFFF0u; g_rx_tail = 0xFFFFFFF0u;
    for (uint8_t i = 0; i < CAN_RX_QUEUE_CAPACITY + 2; ++i) { arrive(i); can_rx_isr(); }
    CHECK(can_rx_stats().queue_high_water == CAN_RX_QUEUE_CAPACITY);
    CHECK(can_rx_stats().queue_overflows == 2 && can_rx_overrun_count() == 2);
    CHECK(fifo.empty() && can_rx_overrun_detected());
    CanFrame frame = {};
    for (uint8_t i = 0; i < CAN_RX_QUEUE_CAPACITY; ++i)
        CHECK(can_receive(frame, 0) && frame.data[0] == i); // Drop newest, preserve order.
    CHECK(!can_rx_pending());
    CAN0_IFLAG1.value = kFifoOverflow | kFifoWarning | (1u << kTxMb);
    can_rx_isr();
    CHECK(can_rx_stats().fifo_overflows == 1 && can_rx_overrun_count() == 3);
    CHECK(CAN0_IFLAG1.value == (1u << kTxMb));
    for (int i = 0; i < ERROR_REPORT_REPEAT_COUNT; ++i) can_rx_overrun_report_sent();
    CHECK(!can_rx_overrun_detected());
    can_clear_rx_overrun_reports();
    CHECK(can_rx_overrun_count() == 3); // Report reset must not erase evidence.

    reset_model(); clock_us = 0xFFFFFFFDu;
    arrive(196); can_rx_isr(); clock_us = 3;
    uint32_t received_us = 0;
    CHECK(can_receive_timed(frame, received_us, 0) && received_us == 0xFFFFFFFEu);
    CHECK(can_rx_stats().max_queue_age_us == 6);
    return true;
}

static bool check_budget_and_wait()
{
    reset_model();
    // More than six entries models continuing arrivals while the ISR drains.
    for (uint8_t i = 0; i < 8; ++i) arrive(i);
    can_rx_isr();
    CHECK(g_rx_head == 6 && fifo.size() == 2 && (CAN0_IFLAG1 & kFifoAvailable));
    can_rx_isr(); CHECK(g_rx_head == 8 && fifo.empty());

    reset_model(); inject_on_clock = true; inject_at = 4;
    CanFrame frame = {}; uint32_t received_us = 0;
    CHECK(can_receive_timed(frame, received_us, 10) && frame.data[0] == 201 && received_us == 5);
    reset_model(); clock_us = 0xFFFFFFFCu;
    CHECK(!can_receive(frame, 5) && clock_us == 2); // Blocking timeout wraps correctly.
    return true;
}

int main()
{
    if (!check_init() || !check_fifo_and_timestamps() || !check_capacity_loss_and_wrap() ||
        !check_budget_and_wait()) return 1;
    std::puts("PASS: FlexCAN FIFO/IRQ init, ordered queue, timestamps, overflow, wrap and wait");
}
