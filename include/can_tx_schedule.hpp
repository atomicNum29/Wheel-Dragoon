#pragma once

#include <stdint.h>

#include "md200t_can.hpp"

// Every 10 ms cycle starts with the two back-to-back driver commands. Voltage
// has an optional 1 ms slot, followed by four spaced wheel polling slots. This
// leaves the main loop time to drain each response from the single RX mailbox.
constexpr uint32_t CAN_TX_COMMAND_PERIOD_US = 10000u;
constexpr uint8_t CAN_TX_SUPERFRAME_CYCLES = 100u;
constexpr uint8_t CAN_TX_MOTOR_POLL_COUNT = 4u;

// Change this one value to select how many four-wheel status polling rounds
// are generated per second. PID 207/174 remains fixed at 100 Hz.
constexpr uint8_t MD200T_STATUS_REQUEST_HZ = 100u;
static_assert(MD200T_STATUS_REQUEST_HZ >= 1u && MD200T_STATUS_REQUEST_HZ <= 100u,
              "MD200T_STATUS_REQUEST_HZ must be in the 1..100 Hz range");

constexpr uint8_t CAN_TX_WHEEL_LF_MASK = 1u << 0;
constexpr uint8_t CAN_TX_WHEEL_RF_MASK = 1u << 1;
constexpr uint8_t CAN_TX_WHEEL_LR_MASK = 1u << 2;
constexpr uint8_t CAN_TX_WHEEL_RR_MASK = 1u << 3;
constexpr uint8_t CAN_TX_ALL_WHEELS_MASK = CAN_TX_WHEEL_LF_MASK |
                                           CAN_TX_WHEEL_RF_MASK |
                                           CAN_TX_WHEEL_LR_MASK |
                                           CAN_TX_WHEEL_RR_MASK;

inline bool can_tx_wheel_round_complete(uint8_t requested_mask, uint8_t received_mask)
{
    return (requested_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK &&
           (received_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK;
}

// Distribute the requested number of rounds across the 100-cycle superframe.
// Cycle 0 is always selected, so control-mode startup obtains telemetry early.
inline bool scheduled_motor_poll_cycle(uint8_t cycle)
{
    cycle = static_cast<uint8_t>(cycle % CAN_TX_SUPERFRAME_CYCLES);
    const uint16_t phase = static_cast<uint16_t>(cycle) * MD200T_STATUS_REQUEST_HZ;
    return (phase % CAN_TX_SUPERFRAME_CYCLES) < MD200T_STATUS_REQUEST_HZ;
}

struct ScheduledPollRequest
{
    uint8_t driver_id;
    uint8_t pid;
    uint8_t wheel_mask;
    uint32_t phase_us;
};

// Driver voltages remain 1 Hz and are separated by 500 ms. This optional slot
// precedes the four motor requests in a selected wheel polling cycle.
inline bool scheduled_voltage_poll(uint8_t cycle, ScheduledPollRequest &request)
{
    cycle = static_cast<uint8_t>(cycle % CAN_TX_SUPERFRAME_CYCLES);

    if (cycle == 0u)
    {
        request = ScheduledPollRequest{MD200T_DRIVER_A_ID, MD200T_PID_VOLT_IN, 0u, 1000u};
        return true;
    }
    if (cycle == 50u)
    {
        request = ScheduledPollRequest{MD200T_DRIVER_B_ID, MD200T_PID_VOLT_IN, 0u, 1000u};
        return true;
    }
    return false;
}

// Every selected cycle requests all four wheels in the 0x82 serialization
// order. Mapping remains A/MOT1=LF, B/MOT1=RF, B/MOT2=LR, A/MOT2=RR.
inline bool scheduled_motor_poll(uint8_t poll_index, ScheduledPollRequest &request)
{
    switch (poll_index)
    {
    case 0u:
        request = ScheduledPollRequest{
            MD200T_DRIVER_A_ID, MD200T_PID_MAIN_DATA, CAN_TX_WHEEL_LF_MASK, 2000u};
        return true;
    case 1u:
        request = ScheduledPollRequest{
            MD200T_DRIVER_B_ID, MD200T_PID_MAIN_DATA, CAN_TX_WHEEL_RF_MASK, 3000u};
        return true;
    case 2u:
        request = ScheduledPollRequest{
            MD200T_DRIVER_B_ID, MD200T_PID_MAIN_DATA2, CAN_TX_WHEEL_LR_MASK, 4000u};
        return true;
    case 3u:
        request = ScheduledPollRequest{
            MD200T_DRIVER_A_ID, MD200T_PID_MAIN_DATA2, CAN_TX_WHEEL_RR_MASK, 5000u};
        return true;
    default:
        return false;
    }
}
