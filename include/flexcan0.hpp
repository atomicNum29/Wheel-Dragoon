#pragma once

#include <stdint.h>

constexpr uint8_t CAN_PAYLOAD_SIZE = 8;
constexpr uint16_t CAN_STANDARD_ID_MAX = 0x7FF;

struct CanFrame
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_PAYLOAD_SIZE];
};

bool can_begin(uint32_t bitrate);
bool can_transmit(const CanFrame &frame, uint32_t timeout_us);
constexpr uint32_t CAN_RX_QUEUE_CAPACITY = 32u;
// Main-context consumers only. timeout_us=0 is nonblocking; ISR owns HW RX.
bool can_receive(CanFrame &frame, uint32_t timeout_us);
// Timestamp is micros() when the ISR reads FIFO output, not bus arrival time.
bool can_receive_timed(CanFrame &frame, uint32_t &received_us, uint32_t timeout_us);
struct CanRxStats {
    uint32_t fifo_overflows;
    uint32_t queue_overflows;
    uint32_t queue_high_water;
    uint32_t max_queue_age_us;
};
// Cumulative since can_begin(). Fields are individually atomic observations.
CanRxStats can_rx_stats();
// True while a FIFO/software queue loss has reports remaining.
bool can_rx_overrun_detected();
// Consume one overrun report after transmitting a canonical 0x82 packet.
void can_rx_overrun_report_sent();
// Clear historical RX overrun reports when starting a control session.
void can_clear_rx_overrun_reports();
bool can_is_initialized();
// Side-effect-free observer of both RAM queue and hardware FIFO pending state.
bool can_rx_pending();
// Sum of FIFO overflow observations and software queue drops since can_begin().
uint32_t can_rx_overrun_count();
