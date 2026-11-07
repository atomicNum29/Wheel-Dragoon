#pragma once
#include "vehicle_kinematics.hpp"

namespace vehicle {
constexpr float servo_frequency_hz = 50.0f;
inline int pwm_duty(int pulse_us) {
    return static_cast<int>(lroundf(pulse_us * servo_frequency_hz * 65535.0f / 1000000.0f));
}
struct CalibrationPoint { float angle; int pulse_us; };
struct ServoCalibration {
    bool verified;
    uint8_t pin;
    int minimum_us, maximum_us;
    bool inverted; // Negate vehicle angle before looking up the PWM table.
    float minimum_angle, maximum_angle;
    CalibrationPoint points[5]; // increasing actuator angle; 0-degree point defines neutral
    uint8_t count;
};
inline bool valid_calibration(const ServoCalibration &c) {
    if (!c.verified || c.count < 3 || c.count > 5 || c.minimum_us < 500 ||
        c.maximum_us > 2500 || c.minimum_us >= c.maximum_us ||
        !isfinite(c.minimum_angle) || !isfinite(c.maximum_angle) ||
        c.minimum_angle >= 0 || c.maximum_angle <= 0 ||
        c.minimum_angle < -70*deg || c.maximum_angle > 70*deg) return false;
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
// Provisional, user-enabled mapping for measurement, NOT measured wheel angles.
// Receiver observation: approximately +/-90 degrees. Assume 1000/1500/2000 us
// corresponds to -90/0/+90 degrees, giving rounded +/-70 endpoints 1111/1889 us.
// Both installed servos run opposite to the vehicle angle sign: invert at output.
// Neutral adjusted separately: left 1540 us, right 1420 us. Endpoints remain provisional.
// Measure each installed wheel independently and replace this table afterwards.
// Both verified flags gate output; true here permits the requested provisional use.
constexpr float rc_maximum_inner_angle = 70 * deg;
static const ServoCalibration left_servo = {
    // verified, pin, minimum_us, maximum_us, inverted, minimum_angle, maximum_angle
    true, 9, 1111, 1889, true, -70*deg, 70*deg,
    {{-70*deg,1111},{0,1540},{70*deg,1889},{0,0},{0,0}}, 3};
static const ServoCalibration right_servo = {
    true, 10, 1111, 1889, true, -70*deg, 70*deg,
    {{-70*deg,1111},{0,1420},{70*deg,1889},{0,0},{0,0}}, 3};
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
