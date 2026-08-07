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
bool can_is_initialized();

