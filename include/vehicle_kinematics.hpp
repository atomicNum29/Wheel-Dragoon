#pragma once
#include <math.h>
#include <stdint.h>

namespace vehicle {
constexpr float pi = 3.14159265358979323846f;
constexpr float deg = pi / 180.0f;
inline float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
// Manual RC command: curvature remains meaningful at zero speed.
struct RcCommand {
    float linear_velocity; // m/s
    float curvature; // 1/m
    bool pivot;
    float angular_velocity; // rad/s, used only for rear-axle-center pivot
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
    float angle[2] = {}; // FL, FR, radians, commanded (never measured)
    float rpm[4] = {};   // FL, FR, RL, RR; rolling signs, existing CAN polarity
    bool feasible = true;
};
inline float speed_to_rpm(float speed, const Geometry &g) {
    return speed * 60.0f / (2.0f * pi * g.radius);
}
// Signed inner-wheel angle -> rear-axle-center curvature (1/m).
// The outer wheel gets its own Ackermann angle, not the same servo angle.
inline float curvature_from_inner_angle(float angle, const Geometry &g) {
    const float tangent = tanf(angle);
    return tangent / (g.wheelbase + g.front_track * 0.5f * fabsf(tangent));
}
// Reject the entire command. Never clamp one angle and retain inconsistent RPMs.
inline WheelTargets inverse(RcCommand cmd, const Geometry &g,
                            const float previous[2], const float minimum[2],
                            const float maximum[2]) {
    WheelTargets out;
    for (int i = 0; i < 2; ++i) out.angle[i] = previous[i];
    const float v = cmd.linear_velocity, k = cmd.curvature;
    if (!isfinite(v) || !isfinite(k) || !isfinite(cmd.angular_velocity) || fabsf(v) > 2.0f) {
        out.feasible = false;
        return out;
    }
    for (int i = 0; i < 2; ++i) {
        // Unit forward path: signed v changes speed, not the steering angle.
        const float vx = cmd.pivot ? (i == 0 ? -1.0f : 1.0f) * g.front_track / 2
                                   : 1.0f - k * (i == 0 ? 1 : -1) * g.front_track / 2;
        const float vy = cmd.pivot ? g.wheelbase : k * g.wheelbase;
        const float speed = hypotf(vx, vy);
        // Reverse the left wheel's rolling direction to stay within servo travel.
        const float angle = cmd.pivot && i == 0 ? atan2f(-vy, -vx) : atan2f(vy, vx);
        // Allow only floating-point roundoff at an exactly commanded endpoint.
        if (angle < minimum[i] - 1e-6f || angle > maximum[i] + 1e-6f) out.feasible = false;
        out.angle[i] = clamp(angle, minimum[i], maximum[i]);
        out.rpm[i] = speed_to_rpm(speed * (cmd.pivot
            ? cmd.angular_velocity * (i == 0 ? -1.0f : 1.0f) : v), g);
    }
    out.rpm[2] = speed_to_rpm((cmd.pivot ? -cmd.angular_velocity*g.rear_track/2 : v * (1.0f - k*g.rear_track/2)), g);
    out.rpm[3] = speed_to_rpm((cmd.pivot ? cmd.angular_velocity*g.rear_track/2 : v * (1.0f + k*g.rear_track/2)), g);
    if (!out.feasible) {
        for (int i=0; i<4; ++i) out.rpm[i] = 0;
        for (int i=0; i<2; ++i) out.angle[i] = previous[i];
    }
    return out;
}
} // namespace vehicle
