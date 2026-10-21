#pragma once

#include <stdint.h>

#include "md200t_can.hpp"

// Driver A starts each 10 ms cycle and Driver B follows at +3 ms. Each command
// owns a 3 ms window for its automatic PID 196/201 response pair, so no
// separate wheel polling slots are required.
constexpr uint32_t CAN_TX_COMMAND_PERIOD_US = 10000u;
constexpr uint32_t CAN_TX_DRIVER_B_PHASE_US = 3000u;
constexpr uint32_t CAN_TX_RESPONSE_TIMEOUT_US = 3000u;
constexpr uint32_t CAN_TX_VOLTAGE_PHASE_US = 6000u;
constexpr uint8_t CAN_TX_SUPERFRAME_CYCLES = 100u;

constexpr uint8_t CAN_TX_WHEEL_LF_MASK = 1u << 0;
constexpr uint8_t CAN_TX_WHEEL_RF_MASK = 1u << 1;
constexpr uint8_t CAN_TX_WHEEL_LR_MASK = 1u << 2;
constexpr uint8_t CAN_TX_WHEEL_RR_MASK = 1u << 3;
constexpr uint8_t CAN_TX_ALL_WHEELS_MASK = CAN_TX_WHEEL_LF_MASK |
                                           CAN_TX_WHEEL_RF_MASK |
                                           CAN_TX_WHEEL_LR_MASK |
                                           CAN_TX_WHEEL_RR_MASK;
constexpr uint8_t CAN_TX_DRIVER_A_WHEELS_MASK = CAN_TX_WHEEL_LF_MASK |
                                                CAN_TX_WHEEL_RR_MASK;
constexpr uint8_t CAN_TX_DRIVER_B_WHEELS_MASK = CAN_TX_WHEEL_RF_MASK |
                                                CAN_TX_WHEEL_LR_MASK;

inline bool can_tx_wheel_round_complete(uint8_t expected_mask, uint8_t received_mask)
{
    return (expected_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK &&
           (received_mask & CAN_TX_ALL_WHEELS_MASK) == CAN_TX_ALL_WHEELS_MASK;
}

inline bool can_tx_response_pair_complete(uint8_t expected_mask, uint8_t received_mask)
{
    return (received_mask & expected_mask) == expected_mask;
}

inline bool can_tx_response_window_expired(uint32_t tx_us, uint32_t now_us)
{
    return static_cast<uint32_t>(now_us - tx_us) >= CAN_TX_RESPONSE_TIMEOUT_US;
}

struct ScheduledVoltageRequest
{
    uint8_t driver_id;
    uint8_t pid;
    uint32_t phase_us;
};

// PID 207/174 does not return PID 143, so driver voltages remain explicit 1 Hz
// requests separated by 500 ms. The optional request follows both 3 ms response
// windows, leaving the rest of the 10 ms cycle idle.
inline bool scheduled_voltage_poll(uint8_t cycle, ScheduledVoltageRequest &request)
{
    cycle = static_cast<uint8_t>(cycle % CAN_TX_SUPERFRAME_CYCLES);

    if (cycle == 0u)
    {
        request = ScheduledVoltageRequest{
            MD200T_DRIVER_A_ID, MD200T_PID_VOLT_IN, CAN_TX_VOLTAGE_PHASE_US};
        return true;
    }
    if (cycle == 50u)
    {
        request = ScheduledVoltageRequest{
            MD200T_DRIVER_B_ID, MD200T_PID_VOLT_IN, CAN_TX_VOLTAGE_PHASE_US};
        return true;
    }
    return false;
}
