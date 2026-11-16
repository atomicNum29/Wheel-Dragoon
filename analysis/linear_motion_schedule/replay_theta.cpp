// Host-only replay of the current v/center-angle API. No hardware interfaces.
#ifdef REPLAY_INSTRUMENTED
#include "instrumented_vehicle_motion.hpp"
#else
#include "vehicle_motion.hpp"
#endif
#include <cstdio>
#include <cstring>
#include <cstdlib>

int main(int argc, char **argv) {
    if (argc != 8 || (std::strcmp(argv[1], "step") && std::strcmp(argv[1], "ramp"))) return 2;
    const bool ramp = std::strcmp(argv[1], "ramp") == 0;
    const float v0 = std::strtof(argv[3], nullptr), v1 = std::strtof(argv[4], nullptr);
    const float theta0 = std::strtof(argv[5], nullptr), theta1 = std::strtof(argv[6], nullptr);
    const float duration = std::strtof(argv[7], nullptr);
    FILE *out = std::fopen(argv[2], "w");
    if (!out) return 3;
    vehicle::MotionController motion(vehicle::MotionConfig{}, vehicle::left_servo, vehicle::right_servo);
    for (uint32_t now = 0; now <= 10000; now += 10)
        motion.update(vehicle::RcCommand(v0, theta0), now, true);
    if (motion.transition != vehicle::Transition::DRIVING || !motion.feasible) return 4;
    // Independent final target: apply the same common RPM magnitude cap.
    const float lo[] = {motion.calibration[0].minimum_angle, motion.calibration[1].minimum_angle};
    const float hi[] = {motion.calibration[0].maximum_angle, motion.calibration[1].maximum_angle};
    auto target = vehicle::inverse(vehicle::RcCommand(v1, theta1), motion.config.geometry, motion.angle, lo, hi);
    if (!target.feasible) return 5;
    float peak = 0;
    for (float r : target.rpm) peak = fmaxf(peak, fabsf(r));
    if (peak > motion.config.rpm_limit)
        for (float &r : target.rpm) r = (r / peak) * motion.config.rpm_limit;
    std::fprintf(out, "time_s,request_v,request_theta_deg,generated_v,generated_theta_deg,LF_angle_deg,RF_angle_deg,LF_rpm,RF_rpm,LR_rpm,RR_rpm,transition,limited,feasible,target_angle_error_deg,target_rpm_error,attempts,rejected,evaluations,accepted\n");
    for (uint32_t tick = 0; tick <= 500; ++tick) {
        const float t = tick * 0.01f;
        const float progress = ramp ? fminf(t / duration, 1.0f) : (tick ? 1.0f : 0.0f);
        const float v = v0 + (v1 - v0) * progress;
        const float theta = theta0 + (theta1 - theta0) * progress;
        if (tick) motion.update(vehicle::RcCommand(v, theta), 10000 + tick * 10, true);
        const float conversion = vehicle::speed_to_rpm(1.0f, motion.config.geometry);
        const float generated_v = (motion.rpm[vehicle::LR] + motion.rpm[vehicle::RR]) / (2 * conversion);
        // Equivalent bicycle angle reconstructed from LF output, not measured feedback.
        const float tangent = tanf(motion.angle[0]);
        const float k = tangent / (motion.config.geometry.wheelbase + motion.config.geometry.front_track * 0.5f * tangent);
        const float generated_theta = atanf(k * motion.config.geometry.wheelbase);
        float angle_error = 0, rpm_error = 0;
        for (int i = 0; i < 2; ++i) angle_error = fmaxf(angle_error, fabsf(target.angle[i] - motion.angle[i]) / vehicle::deg);
        for (int i = 0; i < 4; ++i) rpm_error = fmaxf(rpm_error, fabsf(target.rpm[i] - motion.rpm[i]));
        unsigned attempts = 0, rejected = 0, evaluations = 0, accepted = 0;
#ifdef REPLAY_INSTRUMENTED
        if (tick) {
            attempts = motion.replay_attempts; rejected = motion.replay_rejected;
            evaluations = motion.replay_evaluations; accepted = motion.replay_accepted;
        }
#endif
        std::fprintf(out, "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%.9g,%.9g,%u,%u,%u,%u\n",
            tick * 0.01, double(v), double(theta / vehicle::deg), double(generated_v), double(generated_theta / vehicle::deg),
            double(motion.angle[0] / vehicle::deg), double(motion.angle[1] / vehicle::deg),
            double(motion.rpm[0]), double(motion.rpm[1]), double(motion.rpm[2]), double(motion.rpm[3]),
            unsigned(motion.transition), unsigned(motion.limited), unsigned(motion.feasible), double(angle_error), double(rpm_error),
            attempts, rejected, evaluations, accepted);
    }
    std::fclose(out);
    return 0;
}
