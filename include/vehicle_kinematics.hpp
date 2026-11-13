#pragma once
#include <math.h>
#include <stdint.h>

namespace vehicle {
enum WheelIndex : uint8_t { LF = 0, RF = 1, LR = 2, RR = 3, WHEEL_COUNT = 4 };
constexpr uint8_t STEERING_COUNT = 2;
constexpr float MAX_LINEAR_SPEED_MPS = 2.0f;
constexpr float ANGLE_BOUND_TOLERANCE_RAD = 1e-6f;
constexpr float SECONDS_PER_MINUTE = 60.0f;
constexpr float pi = 3.14159265358979323846f;
constexpr float deg = pi / 180.0f;
inline float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
// MANUAL uses curvature even at zero speed; PIVOT uses angular velocity instead.
struct RcCommand {
    float linear_velocity; // m/s
    float curvature; // 1/m
    bool pivot;
    float angular_velocity; // rad/s about the rear-axle center, PIVOT only
    RcCommand(float v = 0, float k = 0, bool p = false, float w = 0)
        : linear_velocity(v), curvature(k), pivot(p), angular_velocity(w) {}
};
struct Geometry {
    float wheelbase = 0.420f;
    float front_track = 0.439f;
    float rear_track = 0.439f;
    float radius = 0.065f;
};
struct WheelTargets {
    float angle[STEERING_COUNT] = {}; // FL, FR, radians, commanded (never measured)
    float rpm[WHEEL_COUNT] = {};   // FL, FR, RL, RR; rolling signs, existing CAN polarity
    bool feasible = true;
};
inline float speed_to_rpm(float speed, const Geometry &g) {
    return speed * SECONDS_PER_MINUTE / (2.0f * pi * g.radius);
}
// Signed inner-wheel angle -> rear-axle-center curvature (1/m).
// The outer wheel gets its own Ackermann angle, not the same servo angle.
inline float curvature_from_inner_angle(float angle, const Geometry &g) {
    const float tangent = tanf(angle);
    return tangent / (g.wheelbase + g.front_track * 0.5f * fabsf(tangent));
}
// Reject the entire command. Never clamp one angle and retain inconsistent RPMs.
inline WheelTargets inverse(RcCommand cmd, const Geometry &g,
                            const float previous[STEERING_COUNT], const float minimum[STEERING_COUNT],
                            const float maximum[STEERING_COUNT]) {
    WheelTargets out;
    for (int i = 0; i < STEERING_COUNT; ++i) out.angle[i] = previous[i];
    const float v = cmd.linear_velocity, k = cmd.curvature;
    if (!isfinite(v) || !isfinite(k) || !isfinite(cmd.angular_velocity) || fabsf(v) > MAX_LINEAR_SPEED_MPS) {
        out.feasible = false;
        return out;
    }
    for (int i = 0; i < STEERING_COUNT; ++i) {
        // MANUAL: velocity per unit forward speed. PIVOT: per unit yaw rate.
        const float side = i == LF ? -1.0f : 1.0f;
        const float vx = cmd.pivot ? side * g.front_track / 2
                                   : 1.0f + k * side * g.front_track / 2;
        const float vy = cmd.pivot ? g.wheelbase : k * g.wheelbase;
        const float speed = hypotf(vx, vy);
        // Reverse left-front rolling direction to keep the pivot angle in range.
        const float angle = cmd.pivot && i == LF ? atan2f(-vy, -vx) : atan2f(vy, vx);
        // Allow only floating-point roundoff at an exactly commanded endpoint.
        if (angle < minimum[i] - ANGLE_BOUND_TOLERANCE_RAD || angle > maximum[i] + ANGLE_BOUND_TOLERANCE_RAD) out.feasible = false;
        out.angle[i] = clamp(angle, minimum[i], maximum[i]);
        out.rpm[i] = speed_to_rpm(speed * (cmd.pivot ? cmd.angular_velocity * side : v), g);
    }
    out.rpm[LR] = speed_to_rpm(cmd.pivot ? -cmd.angular_velocity*g.rear_track/2
                                        : v * (1.0f - k*g.rear_track/2), g);
    out.rpm[RR] = speed_to_rpm(cmd.pivot ? cmd.angular_velocity*g.rear_track/2
                                        : v * (1.0f + k*g.rear_track/2), g);
    if (!out.feasible) {
        for (int i=0; i < WHEEL_COUNT; ++i) out.rpm[i] = 0;
        for (int i=0; i < STEERING_COUNT; ++i) out.angle[i] = previous[i];
    }
    return out;
}
} // namespace vehicle
