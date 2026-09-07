#pragma once

#include <stdint.h>
#include "flexcan0.hpp"

constexpr uint8_t MD200T_DRIVER_A_ID = 1u;
constexpr uint8_t MD200T_DRIVER_B_ID = 2u;
constexpr uint8_t MD200T_RESPONSE_MID = 7u;
constexpr uint8_t MD200T_PID_REQUEST_PID_DATA = 4u;
constexpr uint8_t MD200T_PID_VOLT_IN = 143u;
constexpr uint8_t MD200T_PID_MAIN_DATA = 193u;
constexpr uint8_t MD200T_PID_MAIN_DATA2 = 200u;

bool send_frame(const CanFrame &frame);
bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2);
bool md200t_torque_off(uint8_t driver_id);
// Request one PID through PID_REQ_PID_DATA (4); response handling is async.
bool md200t_request_pid_data(uint8_t driver_id, uint8_t target_pid);
// Decode an MDROBOT Standard response ID (MID 7) and return its driver ID.
bool md200t_decode_response_id(uint16_t standard_id, uint8_t &driver_id);
