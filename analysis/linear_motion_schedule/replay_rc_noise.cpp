// Scalar RC pulse replay. No Arduino, interrupt, CAN, or actuator emulation.
#ifdef REPLAY_INSTRUMENTED
#include "instrumented_vehicle_motion.hpp"
#else
#include "vehicle_motion.hpp"
#endif
#include "rc_replay_adapter.hpp"
#include <cstdio>

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    FILE *input = std::fopen(argv[1], "r"), *out = std::fopen(argv[2], "w");
    if (!input || !out) return 3;
    vehicle::MotionController motion(vehicle::MotionConfig{}, vehicle::left_servo, vehicle::right_servo);
    rc_io::PulseFilter velocity_filter(rc_io::PWM_CENTER_US), steering_filter(rc_io::PWM_CENTER_US);
    unsigned int filtered_v = rc_io::PWM_CENTER_US, filtered_k = rc_io::PWM_CENTER_US;
    unsigned int measured_v = filtered_v, measured_k = filtered_k;
    unsigned int dropped_v = 0, dropped_k = 0;
    char header[1024];
    if (!std::fgets(header, sizeof(header), input)) return 4;
    std::fprintf(out, "time_s,nominal_v,nominal_k,ideal_v_pwm_us,ideal_k_pwm_us,hand_v_us,hand_k_us,measurement_v_us,measurement_k_us,measured_v_pwm_us,measured_k_pwm_us,filtered_v_pwm_us,filtered_k_pwm_us,request_v,request_k,generated_v,generated_k,LF_angle_deg,RF_angle_deg,LF_rpm,RF_rpm,LR_rpm,RR_rpm,transition,limited,feasible,dropped_v_frames,dropped_k_frames,attempts,rejected,evaluations,accepted,initial_fraction,accepted_fraction,first_angle_excess_rad,first_rpm_excess,first_peak_excess,first_reject_mask\n");
    double nominal_v=0, nominal_k=0, ideal_v=1500, ideal_k=1500, hand_v=0, hand_k=0, error_v=0, error_k=0;
    for (uint32_t ms=0; ms<=15000; ms+=10) {
        if (ms%20 == 0) {
            unsigned int timestamp;
            const int n = std::fscanf(input, "%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%u,%u",
                &timestamp, &nominal_v, &nominal_k, &ideal_v, &ideal_k, &hand_v, &hand_k,
                &error_v, &error_k, &measured_v, &measured_k);
            if (n != 11 || timestamp != ms) return 5;
            // Match the existing scalar acceptance predicate (without emulating edges).
            if (measured_v <= rc_io::PWM_MAX_US) filtered_v = velocity_filter.push(measured_v);
            else ++dropped_v;
            if (measured_k <= rc_io::PWM_MAX_US) filtered_k = steering_filter.push(measured_k);
            else ++dropped_k;
        }
        const auto command = replay_rc_command(motion, filtered_v, filtered_k);
        motion.update(command, ms, true);
        if (ms < 10000) continue;
        if (ms == 10000 && (motion.transition != vehicle::Transition::DRIVING || !motion.feasible)) return 6;
        const float conversion = vehicle::speed_to_rpm(1.0f, motion.config.geometry);
        const float generated_v = (motion.rpm[vehicle::LR]+motion.rpm[vehicle::RR])/(2.0f*conversion);
        const float tangent = tanf(motion.angle[0]);
        const float generated_k = tangent/(motion.config.geometry.wheelbase + motion.config.geometry.front_track*.5f*tangent);
        unsigned attempts=0, rejected=0, evaluations=0, accepted=0, mask=0;
        double first=-1, last=-1, angle_excess=-1, rpm_excess=-1, peak_excess=-1;
#ifdef REPLAY_INSTRUMENTED
        attempts=motion.replay_attempts; rejected=motion.replay_rejected; evaluations=motion.replay_evaluations;
        accepted=motion.replay_accepted; mask=motion.replay_mask; first=motion.replay_initial; last=motion.replay_final;
        angle_excess=motion.replay_angle_excess; rpm_excess=motion.replay_rpm_excess; peak_excess=motion.replay_peak_excess;
#endif
        std::fprintf(out, "%.9g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%u,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%.17g,%.17g,%.17g,%u\n",
            (double(ms)-10000)/1000, nominal_v, nominal_k, ideal_v, ideal_k, hand_v, hand_k, error_v, error_k,
            measured_v, measured_k, filtered_v, filtered_k, double(command.linear_velocity), double(command.curvature),
            double(generated_v), double(generated_k), double(motion.angle[0]/vehicle::deg), double(motion.angle[1]/vehicle::deg),
            double(motion.rpm[0]), double(motion.rpm[1]), double(motion.rpm[2]), double(motion.rpm[3]),
            unsigned(motion.transition), unsigned(motion.limited), unsigned(motion.feasible), dropped_v, dropped_k,
            attempts, rejected, evaluations, accepted, first, last, angle_excess, rpm_excess, peak_excess, mask);
    }
    std::fclose(input); std::fclose(out);
    return 0;
}
