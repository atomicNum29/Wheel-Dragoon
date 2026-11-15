// Offline replay of the actual header implementation; no hardware interfaces.
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
    const float k0 = std::strtof(argv[5], nullptr), k1 = std::strtof(argv[6], nullptr);
    const float duration = std::strtof(argv[7], nullptr);
    FILE *out = std::fopen(argv[2], "w");
    if (!out) return 3;
    vehicle::MotionController motion(vehicle::MotionConfig{}, vehicle::left_servo, vehicle::right_servo);
    // Establish the requested initial DRIVING state through the public API.
    for (uint32_t now = 0; now <= 10000; now += 10)
        motion.update(vehicle::RcCommand(v0, k0), now, true);
    if (motion.transition != vehicle::Transition::DRIVING || !motion.feasible) return 4;
    std::fprintf(out, "time_s,request_v,request_k,generated_v,generated_k,LF_angle_deg,RF_angle_deg,LF_rpm,RF_rpm,LR_rpm,RR_rpm,transition,limited,feasible,attempts,rejected,evaluations,accepted\n");
    for (uint32_t tick = 0; tick <= 500; ++tick) {
        const float t = tick * 0.01f;
        const float progress = ramp ? fminf(t / duration, 1.0f) : (tick ? 1.0f : 0.0f);
        const float v = v0 + (v1 - v0) * progress, k = k0 + (k1 - k0) * progress;
        if (tick) motion.update(vehicle::RcCommand(v, k), 10000 + tick * 10, true);
        // Recover generated MANUAL v,k from rear RPMs (equal rear wheel radii).
        // The private scheduled state is neither exposed nor altered.
        const float conversion = vehicle::speed_to_rpm(1.0f, motion.config.geometry);
        const float sum = motion.rpm[vehicle::LR] + motion.rpm[vehicle::RR];
        const float generated_v = sum / (2.0f * conversion);
        // This also works for stationary steering in these MANUAL cases.
        const float tangent = tanf(motion.angle[0]);
        const float generated_k = tangent / (motion.config.geometry.wheelbase +
            0.5f * motion.config.geometry.front_track * tangent);
        unsigned attempts = 0, rejected = 0, evaluations = 0, accepted = 0;
#ifdef REPLAY_INSTRUMENTED
        if (tick) {
            attempts = motion.replay_attempts; rejected = motion.replay_rejected;
            evaluations = motion.replay_evaluations; accepted = motion.replay_accepted;
        }
#endif
        std::fprintf(out, "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u\n",
            tick * 0.01, double(v), double(k), double(generated_v), double(generated_k),
            double(motion.angle[0] / vehicle::deg), double(motion.angle[1] / vehicle::deg),
            double(motion.rpm[0]), double(motion.rpm[1]), double(motion.rpm[2]), double(motion.rpm[3]),
            unsigned(motion.transition), unsigned(motion.limited), unsigned(motion.feasible),
            attempts, rejected, evaluations, accepted);
    }
    std::fclose(out);
    return 0;
}
