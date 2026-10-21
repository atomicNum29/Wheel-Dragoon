#pragma once

#include <stdint.h>

struct CanFrame
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
};

bool can_begin(uint32_t bitrate);
bool can_transmit(const CanFrame &frame, uint32_t timeout_us);
bool can_receive(CanFrame &frame, uint32_t timeout_us);
// True while a detected RX overrun has reports remaining.
bool can_rx_overrun_detected();
// Consume one overrun report after transmitting a canonical 0x82 packet.
void can_rx_overrun_report_sent();
bool can_is_initialized();
