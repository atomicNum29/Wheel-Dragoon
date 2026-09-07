#pragma once

#include <stdint.h>

#include "md200t_can.hpp"

// Automatic CAN traffic uses one 10 ms control cycle. The two driver command
// frames are sent back-to-back at phase 0; at most one poll is sent at phase 5.
constexpr uint32_t CAN_TX_COMMAND_PERIOD_US = 10000u;
constexpr uint32_t CAN_TX_POLL_PHASE_US = 5000u;
constexpr uint8_t CAN_TX_SUPERFRAME_CYCLES = 100u;

struct ScheduledPollRequest
{
    uint8_t driver_id;
    uint8_t pid;
};

// Return the optional polling request assigned to one cycle of the 1 s
// superframe. The diagonal driver/channel mapping stays explicit here.
inline bool scheduled_poll_for_cycle(uint8_t cycle, ScheduledPollRequest &request)
{
    cycle = static_cast<uint8_t>(cycle % CAN_TX_SUPERFRAME_CYCLES);

    if (cycle == 0u)
    {
        request = ScheduledPollRequest{MD200T_DRIVER_A_ID, MD200T_PID_VOLT_IN};
        return true;
    }
    if (cycle == 50u)
    {
        request = ScheduledPollRequest{MD200T_DRIVER_B_ID, MD200T_PID_VOLT_IN};
        return true;
    }

    switch (cycle % 10u)
    {
    case 1u:
        request = ScheduledPollRequest{MD200T_DRIVER_A_ID, MD200T_PID_MAIN_DATA}; // LF
        return true;
    case 2u:
        request = ScheduledPollRequest{MD200T_DRIVER_B_ID, MD200T_PID_MAIN_DATA}; // RF
        return true;
    case 3u:
        request = ScheduledPollRequest{MD200T_DRIVER_B_ID, MD200T_PID_MAIN_DATA2}; // LR
        return true;
    case 4u:
        request = ScheduledPollRequest{MD200T_DRIVER_A_ID, MD200T_PID_MAIN_DATA2}; // RR
        return true;
    default:
        return false;
    }
}
