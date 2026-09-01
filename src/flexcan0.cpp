#include "flexcan0.hpp"

#include <Arduino.h>
#include <kinetis.h>

#if !defined(__MK20DX256__)
#error "This FlexCAN0 implementation is for Teensy 3.2 / MK20DX256 only."
#endif

namespace
{
constexpr uint32_t kSupportedBitrate = 250000u;
constexpr uint8_t kMaxMbIndex = 15u;
constexpr uint8_t kTxMb = 0u;
constexpr uint8_t kRxMb = 1u;
constexpr uint32_t kAllMbFlags = 0x0000FFFFu;

constexpr uintptr_t kCan0MbBase = 0x40024080u;
constexpr uintptr_t kCan0MbStride = 0x10u;
constexpr uintptr_t kMbCsOffset = 0x0u;
constexpr uintptr_t kMbIdOffset = 0x4u;
constexpr uintptr_t kMbWord0Offset = 0x8u;
constexpr uintptr_t kMbWord1Offset = 0xCu;

constexpr uint32_t kMcrMaxMbMask = 0x0000007Fu;
constexpr uint32_t kMcrSrxDis = 0x00020000u;
constexpr uint32_t kMcrLpmAck = 0x00100000u;
constexpr uint32_t kMcrFrzAck = 0x01000000u;
constexpr uint32_t kMcrSoftRst = 0x02000000u;
constexpr uint32_t kMcrHalt = 0x10000000u;
constexpr uint32_t kMcrRfen = 0x20000000u;
constexpr uint32_t kMcrFrz = 0x40000000u;
constexpr uint32_t kMcrMdis = 0x80000000u;

constexpr uint32_t kCtrl1ClkSrc = 0x00002000u;
constexpr uint32_t kCtrl1Timing250k = 0x037B0002u;

constexpr uint32_t kMbCodeShift = 24u;
constexpr uint32_t kMbCodeMask = 0x0F000000u;
constexpr uint32_t kMbDlcShift = 16u;
constexpr uint32_t kMbIdStdShift = 18u;

constexpr uint8_t kMbCodeTxInactive = 0b1000u;
constexpr uint8_t kMbCodeTxAbort = 0b1001u;
constexpr uint8_t kMbCodeTxDataOnce = 0b1100u;
constexpr uint8_t kMbCodeRxFull = 0b0010u;
constexpr uint8_t kMbCodeRxEmpty = 0b0100u;
constexpr uint8_t kMbCodeRxOverrun = 0b0110u;

bool g_can_initialized = false;

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
    g_can_initialized = false;

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

    if (!wait_until_clear(CAN0_MCR, kMcrLpmAck, 1000u))
        return false;

    CAN0_MCR |= kMcrSoftRst;
    if (!wait_until_clear(CAN0_MCR, kMcrSoftRst, 1000u))
        return false;

    CAN0_MCR = (CAN0_MCR & ~(kMcrMaxMbMask | kMcrRfen | kMcrMdis)) |
               kMcrFrz |
               kMcrHalt |
               kMcrSrxDis |
               kMaxMbIndex;

    if (!wait_until_set(CAN0_MCR, kMcrFrzAck, 1000u))
        return false;

    CAN0_CTRL1 = kCtrl1Timing250k;
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

    mb_cs(kTxMb) = mb_code(kMbCodeTxInactive);
    mb_id(kRxMb) = 0u;
    mb_cs(kRxMb) = mb_code(kMbCodeRxEmpty);

    CAN0_MCR &= ~kMcrHalt;
    if (!wait_until_clear(CAN0_MCR, kMcrFrzAck, 1000u))
        return false;

    g_can_initialized = true;
    return true;
}

bool can_transmit(const CanFrame &frame, uint32_t timeout_us)
{
    if (!g_can_initialized || frame.id > 0x7FFu || frame.dlc > 8u)
        return false;

    const uint32_t flag = 1u << kTxMb;
    CAN0_IFLAG1 = flag;

    if (!abort_tx_mb(timeout_us))
        return false;

    mb_id(kTxMb) = (static_cast<uint32_t>(frame.id) & 0x7FFu) << kMbIdStdShift;
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
    if (!g_can_initialized)
        return false;

    const uint32_t flag = 1u << kRxMb;
    if (!wait_until_set(CAN0_IFLAG1, flag, timeout_us))
        return false;

    const uint32_t cs = mb_cs(kRxMb);
    const uint8_t code = static_cast<uint8_t>((cs & kMbCodeMask) >> kMbCodeShift);
    if (code != kMbCodeRxFull && code != kMbCodeRxOverrun)
    {
        CAN0_IFLAG1 = flag;
        mb_cs(kRxMb) = mb_code(kMbCodeRxEmpty);
        return false;
    }

    uint8_t dlc = static_cast<uint8_t>((cs >> kMbDlcShift) & 0x0Fu);
    if (dlc > 8u)
        dlc = 8u;

    frame.id = static_cast<uint16_t>((mb_id(kRxMb) >> kMbIdStdShift) & 0x7FFu);
    frame.dlc = dlc;
    for (uint8_t i = 0u; i < 8u; ++i)
        frame.data[i] = 0u;

    unpack_word(mb_word0(kRxMb), frame.data, 0u, frame.dlc);
    unpack_word(mb_word1(kRxMb), frame.data, 4u, frame.dlc);

    (void)CAN0_TIMER;
    CAN0_IFLAG1 = flag;
    mb_cs(kRxMb) = mb_code(kMbCodeRxEmpty);
    return true;
}

bool can_is_initialized()
{
    return g_can_initialized;
}
