// Host-only replay: stationary steering then acceleration, or the reverse order.
#ifdef REPLAY_INSTRUMENTED
#include "instrumented_vehicle_motion.hpp"
#else
#include "vehicle_motion.hpp"
#endif
#include <cstdio>
#include <cstring>

float progress(uint32_t ms, uint32_t onset, bool ramp) {
    if (ms < onset) return 0;
    return ramp ? fminf(static_cast<float>(ms - onset) / 1000.0f, 1.0f) : 1.0f;
}

int main(int argc, char **argv) {
    if (argc != 4 || (std::strcmp(argv[1], "steer_first") && std::strcmp(argv[1], "speed_first")) ||
        (std::strcmp(argv[2], "step") && std::strcmp(argv[2], "ramp"))) return 2;
    const bool steer_first = std::strcmp(argv[1], "steer_first") == 0;
    const bool ramp = std::strcmp(argv[2], "ramp") == 0;
    FILE *out = std::fopen(argv[3], "w");
    if (!out) return 3;
    vehicle::MotionController motion(vehicle::MotionConfig{}, vehicle::left_servo, vehicle::right_servo);
    for (uint32_t now = 0; now <= 10000; now += 10)
        motion.update(vehicle::RcCommand(0, 0), now, true);
    if (motion.transition != vehicle::Transition::DRIVING || !motion.feasible) return 4;
    std::fprintf(out, "time_s,request_v,request_k,generated_v,generated_k,LF_angle_deg,RF_angle_deg,LF_rpm,RF_rpm,LR_rpm,RR_rpm,transition,limited,feasible,attempts,rejected,evaluations,accepted,initial_fraction,accepted_fraction,first_angle_excess_rad,first_rpm_excess,first_peak_excess,first_reject_mask\n");
    for (uint32_t ms = 0; ms <= 5000; ms += 10) {
        const float v = 0.8f * progress(ms, steer_first ? 2000 : 500, ramp);
        const float k = 2.0f * progress(ms, steer_first ? 500 : 2000, ramp);
        if (ms) motion.update(vehicle::RcCommand(v, k), 10000 + ms, true);
        const float conversion = vehicle::speed_to_rpm(1.0f, motion.config.geometry);
        const float generated_v = (motion.rpm[vehicle::LR] + motion.rpm[vehicle::RR]) / (2.0f * conversion);
        const float tangent = tanf(motion.angle[0]);
        const float generated_k = tangent / (motion.config.geometry.wheelbase + 0.5f * motion.config.geometry.front_track * tangent);
        unsigned attempts = 0, rejected = 0, evaluations = 0, accepted = 0, mask = 0;
        double first = -1, last = -1, angle_excess = -1, rpm_excess = -1, peak_excess = -1;
#ifdef REPLAY_INSTRUMENTED
        if (ms) {
            attempts = motion.replay_attempts; rejected = motion.replay_rejected;
            evaluations = motion.replay_evaluations; accepted = motion.replay_accepted;
            first = motion.replay_initial; last = motion.replay_final;
            angle_excess = motion.replay_angle_excess; rpm_excess = motion.replay_rpm_excess;
            peak_excess = motion.replay_peak_excess; mask = motion.replay_mask;
        }
#endif
        std::fprintf(out, "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%.17g,%.17g,%.17g,%u\n",
            ms / 1000.0, double(v), double(k), double(generated_v), double(generated_k),
            double(motion.angle[0] / vehicle::deg), double(motion.angle[1] / vehicle::deg),
            double(motion.rpm[0]), double(motion.rpm[1]), double(motion.rpm[2]), double(motion.rpm[3]),
            unsigned(motion.transition), unsigned(motion.limited), unsigned(motion.feasible),
            attempts, rejected, evaluations, accepted, first, last, angle_excess, rpm_excess, peak_excess, mask);
    }
    std::fclose(out);
    return 0;
}
