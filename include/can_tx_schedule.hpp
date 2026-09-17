#pragma once

#include <stdint.h>

#include "md200t_can.hpp"

// Every 10 ms cycle starts with the two back-to-back driver commands. The
// configured MD200T controllers return PID 193 and PID 200 in response to
// PID 207 or PID 174, so no separate wheel polling slots are required.
constexpr uint32_t CAN_TX_COMMAND_PERIOD_US = 10000u;
constexpr uint8_t CAN_TX_SUPERFRAME_CYCLES = 100u;

constexpr uint8_t CAN_TX_WHEEL_LF_MASK = 1u << 0;
constexpr uint8_t CAN_TX_WHEEL_RF_MASK = 1u << 1;
constexpr uint8_t CAN_TX_WHEEL_LR_MASK = 1u << 2;
constexpr uint8_t CAN_TX_WHEEL_RR_MASK = 1u << 3;
constexpr uint8_t CAN_TX_ALL_WHEELS_MASK = CAN_TX_WHEEL_LF_MASK |
                                           CAN_TX_WHEEL_RF_MASK |
                                           CAN_TX_WHEEL_LR_MASK |
                                           CAN_TX_WHEEL_RR_MASK;

inline bool can_tx_wheel_round_complete(uint8_t expected_mask, uint8_t received_mask)
{
    return (expected_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK &&
           (received_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK;
}

struct ScheduledVoltageRequest
{
    uint8_t driver_id;
    uint8_t pid;
    uint32_t phase_us;
};

// PID 207/174 does not return PID 143, so driver voltages remain explicit 1 Hz
// requests separated by 500 ms. The optional request follows the command pair.
inline bool scheduled_voltage_poll(uint8_t cycle, ScheduledVoltageRequest &request)
{
    cycle = static_cast<uint8_t>(cycle % CAN_TX_SUPERFRAME_CYCLES);

    if (cycle == 0u)
    {
        request = ScheduledVoltageRequest{MD200T_DRIVER_A_ID, MD200T_PID_VOLT_IN, 1000u};
        return true;
    }
    if (cycle == 50u)
    {
        request = ScheduledVoltageRequest{MD200T_DRIVER_B_ID, MD200T_PID_VOLT_IN, 1000u};
        return true;
    }
    return false;
}
