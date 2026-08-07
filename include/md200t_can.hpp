#pragma once

#include <stdint.h>
#include "flexcan0.hpp"

constexpr uint8_t MD200T_DRIVER_A_ID = 1u;
constexpr uint8_t MD200T_DRIVER_B_ID = 2u;

bool send_frame(const CanFrame &frame);
bool md200t_set_motor1_rpm(uint8_t driver_id, int16_t rpm);
bool md200t_set_motor2_rpm(uint8_t driver_id, int16_t rpm);
bool md200t_torque_off(uint8_t driver_id);
