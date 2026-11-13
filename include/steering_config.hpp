#pragma once
#include "vehicle_kinematics.hpp"

namespace vehicle {
constexpr float servo_frequency_hz = 50.0f;
constexpr uint8_t servo_resolution_bits = 16;
constexpr uint32_t servo_duty_max = (1u << servo_resolution_bits) - 1u;
constexpr float microseconds_per_second = 1000000.0f;
inline int pwm_duty(int pulse_us) {
    return static_cast<int>(lroundf(pulse_us * servo_frequency_hz * servo_duty_max / microseconds_per_second));
}
constexpr uint8_t calibration_min_points = 3, calibration_max_points = 5;
constexpr int servo_valid_min_us = 500, servo_valid_max_us = 2500;
constexpr float rc_maximum_inner_angle = 70 * deg;
constexpr float rc_pivot_maximum_angular_velocity = 5.0f; // rad/s before common wheel RPM limiting
struct CalibrationPoint { float angle; int pulse_us; };
struct ServoCalibration {
    bool verified;
    uint8_t pin;
    int minimum_us, maximum_us;
    bool inverted; // Negate vehicle angle before looking up the PWM table.
    float minimum_angle, maximum_angle;
    CalibrationPoint points[calibration_max_points]; // increasing actuator angle; 0-degree point defines neutral
    uint8_t count;
};
inline bool valid_calibration(const ServoCalibration &c) {
    if (!c.verified || c.count < calibration_min_points || c.count > calibration_max_points || c.minimum_us < servo_valid_min_us ||
        c.maximum_us > servo_valid_max_us || c.minimum_us >= c.maximum_us ||
        !isfinite(c.minimum_angle) || !isfinite(c.maximum_angle) ||
        c.minimum_angle >= 0 || c.maximum_angle <= 0 ||
        c.minimum_angle < -rc_maximum_inner_angle || c.maximum_angle > rc_maximum_inner_angle) return false;
    bool neutral = false;
    for (uint8_t i=0;i<c.count;++i) {
        if (!isfinite(c.points[i].angle) || c.points[i].pulse_us < c.minimum_us ||
            c.points[i].pulse_us > c.maximum_us) return false;
        if (i && (c.points[i].angle <= c.points[i-1].angle ||
                  c.points[i].pulse_us <= c.points[i-1].pulse_us)) return false;
        if (c.points[i].angle == 0) neutral = true;
    }
    const float low = c.inverted ? -c.maximum_angle : c.minimum_angle;
    const float high = c.inverted ? -c.minimum_angle : c.maximum_angle;
    return neutral && c.points[0].angle <= low && c.points[c.count-1].angle >= high;
}
inline int pulse_for_angle(const ServoCalibration &c, float angle) {
    if (!valid_calibration(c) || !isfinite(angle)) return 0;
    angle = clamp(angle, c.minimum_angle, c.maximum_angle);
    if (c.inverted) angle = -angle;
    for (uint8_t i=1;i<c.count;++i) if (angle <= c.points[i].angle) {
        const auto &a=c.points[i-1]; const auto &b=c.points[i];
        return static_cast<int>(lroundf(clamp(a.pulse_us + (angle-a.angle) *
            (b.pulse_us-a.pulse_us)/(b.angle-a.angle), c.minimum_us, c.maximum_us)));
    }
    return c.maximum_us;
}
// User-measured vehicle angles (-70/0/+70 degrees), 2026-11-09:
// left 2100/1577/1044 us; right 1990/1443/908 us.
// Both outputs decrease with vehicle angle. Store increasing PWM against
// actuator angle and invert the vehicle angle before table lookup.
static const ServoCalibration left_servo = {
    // verified, pin, minimum_us, maximum_us, inverted, minimum_angle, maximum_angle
    true, 9, 1044, 2100, true, -rc_maximum_inner_angle, rc_maximum_inner_angle,
    {{-rc_maximum_inner_angle,1044},{0,1577},{rc_maximum_inner_angle,2100},{0,0},{0,0}}, 3};
static const ServoCalibration right_servo = {
    true, 10, 908, 1990, true, -rc_maximum_inner_angle, rc_maximum_inner_angle,
    {{-rc_maximum_inner_angle,908},{0,1443},{rc_maximum_inner_angle,1990},{0,0},{0,0}}, 3};
constexpr float mdh100_maximum_rpm = 400.0f;
struct MotionConfig {
    Geometry geometry;
    float rpm_limit = 200; // conservative commissioning cap, below MDH100 400 RPM specification
    float acceleration_rpm_s = 200;
    float deceleration_rpm_s = 400;
    float steering_rate = 180*deg; // maximum commanded rate; both axes share arrival time
    uint32_t settle_ms = 500;
    uint32_t startup_ms = 1500;
    uint32_t stopped_dwell_ms = 200;
};
} // namespace vehicle
