#pragma once
#include "control.hpp"

namespace rc_io {
constexpr int32_t PWM_CENTER_US = 1500;
constexpr int32_t PWM_DEADBAND_US = 50;
constexpr unsigned int PWM_MIN_US = 1000;
constexpr unsigned int PWM_MAX_US = 2000;
constexpr float PWM_ACTIVE_SPAN_US = PWM_MAX_US - PWM_CENTER_US - PWM_DEADBAND_US;
constexpr float THROTTLE_US_PER_MPS = 250.0f;
struct PwmInput { unsigned int velocity_us, steering_us; };
void initialize_inputs();
// Individual ISR-owned reads; no cross-channel atomic snapshot is implied.
DriveMode read_mode();
PwmInput read_pwm();
int32_t offset_with_deadband(unsigned int pulse_width_us);
void initialize_steering(const vehicle::MotionController &motion);
void write_steering(const vehicle::MotionController &motion);
} // namespace rc_io
