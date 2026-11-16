#pragma once
#include "steering_config.hpp"

namespace vehicle
{
    constexpr float ANGLE_REACHED_TOLERANCE_RAD = 0.00001f;
    constexpr float REVERSAL_RPM_PRODUCT_THRESHOLD = -0.01f;
    constexpr float SECONDS_PER_MILLISECOND = 0.001f;
    constexpr float MAX_UPDATE_DT_SECONDS = 0.05f;
    // A bounded search only reduces a step; it never commits an unchecked fallback.
    constexpr uint8_t MOTION_STEP_ATTEMPTS = 12;
    enum class Transition : uint8_t
    {
        STARTUP,
        DRIVING,
        BRAKING,
        STEERING,
        SETTLING
    };
    class MotionController
    {
    public:
        MotionConfig config;
        ServoCalibration calibration[STEERING_COUNT];
        Transition transition = Transition::STARTUP;
        float previous_angle[STEERING_COUNT] = {}, target_angle[STEERING_COUNT] = {}, angle[STEERING_COUNT] = {};
        float rpm[WHEEL_COUNT] = {};
        bool feasible = true, ready = false, limited = false;
        MotionController(const MotionConfig &c, const ServoCalibration &left, const ServoCalibration &right)
            : config(c), calibration{left, right} {}
        bool calibrated() const { return valid_calibration(calibration[0]) && valid_calibration(calibration[1]); }
        // Generated command, not measured motion or a CAN transmission ACK.
        // During direct two-axis pose transitions no single bicycle angle exists.
        bool applied_command(RcCommand &command) const
        {
            command = RcCommand{};
            if ((transition != Transition::DRIVING && transition != Transition::BRAKING) ||
                !scheduled_pose_valid || !ready || !calibrated() || !valid_configuration()) return false;
            command = scheduled;
            return true;
        }
        bool valid_configuration() const
        {
            const auto &g = config.geometry;
            return isfinite(g.wheelbase) && g.wheelbase > 0 &&
                   isfinite(g.front_track) && g.front_track > 0 &&
                   isfinite(g.rear_track) && g.rear_track > 0 &&
                   isfinite(g.radius) && g.radius > 0 &&
                   isfinite(config.rpm_limit) && config.rpm_limit > 0 &&
                   config.rpm_limit <= mdh100_maximum_rpm &&
                   isfinite(config.acceleration_rpm_s) && config.acceleration_rpm_s > 0 &&
                   isfinite(config.deceleration_rpm_s) && config.deceleration_rpm_s > 0 &&
                   isfinite(config.steering_rate) && config.steering_rate > 0 &&
                   config.startup_ms > 0 && config.settle_ms > 0 && config.stopped_dwell_ms > 0;
        }
        bool stopped() const
        {
            // Generated values reach exact zero in brake(); this is not a noisy
            // encoder measurement. Do not enter steering with residual RPMs.
            for (float r : rpm)
                if (r != 0)
                    return false;
            return true;
        }
        // Safety inhibition bypasses ordinary deceleration; existing CAN TQ-OFF handles it.
        void inhibit()
        {
            for (float &r : rpm)
                r = 0;
            scheduled.linear_velocity = 0;
            scheduled.angular_velocity = 0;
            transition = ready ? Transition::BRAKING : Transition::STARTUP;
            timer_active = false;
        }
        void update(RcCommand cmd, uint32_t now_ms, bool enabled)
        {
            const float dt = initialized ? fminf(static_cast<uint32_t>(now_ms - last_ms) * SECONDS_PER_MILLISECOND, MAX_UPDATE_DT_SECONDS) : 0;
            last_ms = now_ms;
            if (!initialized)
            {
                initialized = true;
                startup_at = now_ms;
            }
            if (!calibrated() || !valid_configuration())
            {
                ready = false;
                inhibit();
                return;
            }
            if (!ready)
            {
                if (static_cast<uint32_t>(now_ms - startup_at) < config.startup_ms)
                    return;
                ready = true;
                transition = Transition::BRAKING;
            }
            if (!enabled)
            {
                inhibit();
                return;
            }
            float lo[STEERING_COUNT], hi[STEERING_COUNT];
            for (int i = 0; i < STEERING_COUNT; ++i)
            {
                lo[i] = calibration[i].minimum_angle;
                hi[i] = calibration[i].maximum_angle;
            }
            limited = false;
            auto target = evaluate(cmd, lo, hi, limited);
            feasible = target.feasible;
            bool reversal = cmd.pivot != scheduled.pivot;
            for (int i = 0; i < STEERING_COUNT; ++i)
            {
                previous_angle[i] = angle[i];
                target_angle[i] = target.angle[i];
            }
            for (int i = 0; i < WHEEL_COUNT; ++i)
                reversal |= rpm[i] * target.rpm[i] < REVERSAL_RPM_PRODUCT_THRESHOLD;
            // Steering-angle changes alone never request braking.
            if (transition == Transition::DRIVING && (reversal || !feasible))
            {
                transition = Transition::BRAKING;
                timer_active = false;
            }
            if (transition == Transition::BRAKING)
            {
                brake(dt);
                if (!stopped())
                    return; // Hold steering until estimated drive deceleration completes.
                if (!elapsed(now_ms, config.stopped_dwell_ms))
                    return;
                transition = Transition::STEERING;
                timer_active = false;
            }
            // Rejected commands retain the generated angles while all four RPMs stop.
            if (!feasible)
            {
                timer_active = false;
                return;
            }
            if (transition == Transition::DRIVING)
            {
                advance_drive(cmd, target, lo, hi, dt);
                return;
            }
            // At standstill there is no rolling constraint. Interpolate the two
            // steering axes together, including incompatible MANUAL/PIVOT poses.
            // Keep all RPMs zero until the generated angles settle at the new pose.
            scheduled_pose_valid = false;
            float max_distance = 0;
            for (int i = 0; i < STEERING_COUNT; ++i)
                max_distance = fmaxf(max_distance, fabsf(target_angle[i] - angle[i]));
            const float fraction = max_distance > 0
                                       ? fminf(1.0f, config.steering_rate * dt / max_distance)
                                       : 1.0f;
            bool reached = true, changed = false;
            for (int i = 0; i < STEERING_COUNT; ++i)
            {
                if (fraction >= 1.0f)
                    angle[i] = target_angle[i];
                else
                    angle[i] += fraction * (target_angle[i] - angle[i]);
                reached &= fabsf(target_angle[i] - angle[i]) < ANGLE_REACHED_TOLERANCE_RAD;
                changed |= fabsf(previous_angle[i] - angle[i]) > ANGLE_REACHED_TOLERANCE_RAD;
            }
            if (transition == Transition::STEERING && reached)
            {
                transition = Transition::SETTLING;
                timer_active = false;
            }
            if (transition == Transition::SETTLING)
            {
                if (!reached || changed)
                {
                    transition = Transition::STEERING;
                    timer_active = false;
                }
                else if (elapsed(now_ms, config.settle_ms))
                {
                    scheduled = RcCommand(0, cmd.steering_angle, cmd.pivot, 0);
                    scheduled_pose_valid = true;
                    transition = Transition::DRIVING;
                    timer_active = false;
                }
            }
            if (transition == Transition::DRIVING)
                advance_drive(cmd, target, lo, hi, dt);
        }

    private:
        // This is the generated vehicle command, not a measured vehicle state.
        // MANUAL interpolates speed and virtual center steering angle. Curvature
        // is derived only when evaluating the coupled wheel outputs.
        // In DRIVING, angle[] and rpm[] always come from this same command.
        RcCommand scheduled;
        bool scheduled_pose_valid = false;
        bool initialized = false, timer_active = false;
        uint32_t last_ms = 0, startup_at = 0, timer_at = 0;
        bool elapsed(uint32_t now, uint32_t interval)
        {
            if (!timer_active)
            {
                timer_at = now;
                timer_active = true;
            }
            return static_cast<uint32_t>(now - timer_at) >= interval;
        }
        WheelTargets evaluate(RcCommand &cmd, const float lo[STEERING_COUNT],
                              const float hi[STEERING_COUNT], bool &rpm_limited) const
        {
            auto out = inverse(cmd, config.geometry, angle, lo, hi);
            if (!out.feasible)
                return out;
            float peak = 0;
            for (float r : out.rpm)
            {
                if (!isfinite(r))
                    out.feasible = false;
                peak = fmaxf(peak, fabsf(r));
            }
            if (!out.feasible)
            {
                for (float &r : out.rpm)
                    r = 0;
                for (int i = 0; i < STEERING_COUNT; ++i)
                    out.angle[i] = angle[i];
                return out;
            }
            if (peak > config.rpm_limit)
            {
                const float scale = config.rpm_limit / peak;
                // Scale the vehicle speed too, so the next step starts from the
                // limited state rather than from an unattained request.
                cmd.linear_velocity *= scale;
                cmd.angular_velocity *= scale;
                for (float &r : out.rpm)
                    r = (r / peak) * config.rpm_limit;
                rpm_limited = true;
            }
            return out;
        }
        float rpm_step_rate(float from, float to) const
        {
            if ((from < 0 && to > 0) || (from > 0 && to < 0))
                return fminf(config.acceleration_rpm_s, config.deceleration_rpm_s);
            return fabsf(to) > fabsf(from) ? config.acceleration_rpm_s : config.deceleration_rpm_s;
        }
        bool step_allowed(const WheelTargets &next, float dt) const
        {
            if (!next.feasible)
                return false;
            for (int i = 0; i < STEERING_COUNT; ++i)
                if (!isfinite(next.angle[i]) || fabsf(next.angle[i] - angle[i]) > config.steering_rate * dt)
                    return false;
            for (int i = 0; i < WHEEL_COUNT; ++i)
                if (!isfinite(next.rpm[i]) || fabsf(next.rpm[i]) > config.rpm_limit ||
                    fabsf(next.rpm[i] - rpm[i]) > rpm_step_rate(rpm[i], next.rpm[i]) * dt)
                    return false;
            return true;
        }
        void advance_drive(const RcCommand &goal, const WheelTargets &target,
                           const float lo[STEERING_COUNT], const float hi[STEERING_COUNT], float dt)
        {
            if (dt <= 0)
                return;
            // Start with the endpoint-derived progress bound. The mapping is
            // nonlinear, so validate the actual intermediate outputs before use.
            float fraction = 1;
            for (int i = 0; i < STEERING_COUNT; ++i)
            {
                const float distance = fabsf(target.angle[i] - angle[i]);
                if (distance > 0)
                    fraction = fminf(fraction, config.steering_rate * dt / distance);
            }
            for (int i = 0; i < WHEEL_COUNT; ++i)
            {
                const float distance = fabsf(target.rpm[i] - rpm[i]);
                if (distance > 0)
                    fraction = fminf(fraction, rpm_step_rate(rpm[i], target.rpm[i]) * dt / distance);
            }
            for (uint8_t attempt = 0; attempt < MOTION_STEP_ATTEMPTS; ++attempt)
            {
                RcCommand next = goal;
                WheelTargets outputs = target;
                bool intermediate_limited = false;
                if (fraction < 1)
                {
                    next.linear_velocity = scheduled.linear_velocity + (goal.linear_velocity - scheduled.linear_velocity) * fraction;
                    next.steering_angle = scheduled.steering_angle + (goal.steering_angle - scheduled.steering_angle) * fraction;
                    next.angular_velocity = scheduled.angular_velocity + (goal.angular_velocity - scheduled.angular_velocity) * fraction;
                    outputs = evaluate(next, lo, hi, intermediate_limited);
                }
                if (step_allowed(outputs, dt))
                {
                    scheduled = next;
                    for (int i = 0; i < STEERING_COUNT; ++i)
                        angle[i] = outputs.angle[i];
                    for (int i = 0; i < WHEEL_COUNT; ++i)
                        rpm[i] = outputs.rpm[i];
                    limited |= intermediate_limited;
                    return;
                }
                fraction *= 0.5f;
            }
            // Retain the entire previous output if no admissible step was found.
            // Never advance steering alone or clamp individual wheel RPMs.
        }
        void brake(float dt)
        {
            float peak = 0;
            for (float r : rpm)
                peak = fmaxf(peak, fabsf(r));
            const float remaining = peak > 0 ? 1.0f - fminf(1.0f, config.deceleration_rpm_s * dt / peak) : 0;
            for (float &r : rpm)
                r *= remaining;
            scheduled.linear_velocity *= remaining;
            scheduled.angular_velocity *= remaining;
        }
    };
} // namespace vehicle
