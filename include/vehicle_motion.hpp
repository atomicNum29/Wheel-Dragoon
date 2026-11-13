#pragma once
#include "steering_config.hpp"

namespace vehicle {
constexpr float STOPPED_RPM_TOLERANCE = 0.001f;
constexpr float ANGLE_REACHED_TOLERANCE_RAD = 0.00001f;
constexpr float REVERSAL_RPM_PRODUCT_THRESHOLD = -0.01f;
constexpr float SECONDS_PER_MILLISECOND = 0.001f;
constexpr float MAX_UPDATE_DT_SECONDS = 0.05f;
enum class Transition : uint8_t { STARTUP, DRIVING, BRAKING, STEERING, SETTLING };
class MotionController {
public:
    MotionConfig config;
    ServoCalibration calibration[STEERING_COUNT];
    Transition transition = Transition::STARTUP;
    float previous_angle[STEERING_COUNT] = {}, target_angle[STEERING_COUNT] = {}, angle[STEERING_COUNT] = {};
    float rpm[WHEEL_COUNT] = {};
    bool feasible = true, ready = false, limited = false;
    MotionController(const MotionConfig &c, const ServoCalibration &left, const ServoCalibration &right)
        : config(c), calibration{left,right} {}
    bool calibrated() const { return valid_calibration(calibration[0]) && valid_calibration(calibration[1]); }
    bool valid_configuration() const {
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
    bool stopped() const {
        for (float r:rpm) if (fabsf(r) > STOPPED_RPM_TOLERANCE) return false;
        return true;
    }
    // Safety inhibition bypasses ordinary deceleration; existing CAN TQ-OFF handles it.
    void inhibit() {
        for(float &r:rpm) r=0;
        transition = ready ? Transition::BRAKING : Transition::STARTUP;
        timer_active = false;
    }
    void update(RcCommand cmd, uint32_t now_ms, bool enabled) {
        const float dt = initialized ? fminf(static_cast<uint32_t>(now_ms-last_ms)*SECONDS_PER_MILLISECOND,MAX_UPDATE_DT_SECONDS) : 0;
        last_ms = now_ms;
        if (!initialized) { initialized=true; startup_at=now_ms; }
        if (!calibrated() || !valid_configuration()) { ready=false; inhibit(); return; }
        if (!ready) {
            if (static_cast<uint32_t>(now_ms-startup_at) < config.startup_ms) return;
            ready=true; transition=Transition::BRAKING;
        }
        if (!enabled) { inhibit(); return; }
        float lo[STEERING_COUNT],hi[STEERING_COUNT];
        for(int i=0;i < STEERING_COUNT;++i) { lo[i]=calibration[i].minimum_angle; hi[i]=calibration[i].maximum_angle; }
        auto target = inverse(cmd,config.geometry,angle,lo,hi);
        feasible=target.feasible;
        limited=false;
        float peak=0;
        for(float r:target.rpm) peak=fmaxf(peak,fabsf(r));
        if(peak>config.rpm_limit) {
            limited=true;
            for(float &r:target.rpm) r*=config.rpm_limit/peak; // common scaling preserves curvature
        }
        bool reversal=cmd.pivot != pivot_active;
        pivot_active=cmd.pivot;
        for(int i=0;i < STEERING_COUNT;++i) {
            target_angle[i]=target.angle[i];
            // Preserve upstream PIVOT behavior: set the fixed ICR immediately,
            // even at neutral yaw input and before the braking/dwell stages.
            // This bypasses commanded steering slew, not physical servo travel.
            if(cmd.pivot && feasible) {
                previous_angle[i]=angle[i];
                angle[i]=target_angle[i];
            }
        }
        for(int i=0;i < WHEEL_COUNT;++i) reversal |= rpm[i]*target.rpm[i]<REVERSAL_RPM_PRODUCT_THRESHOLD;
        // Steering-angle changes alone never request braking.
        if(transition==Transition::DRIVING && (reversal || !feasible)) {
            transition=Transition::BRAKING; timer_active=false;
        }
        if(transition==Transition::BRAKING) {
            ramp(nullptr,dt);
            if(!stopped()) return; // Hold steering until estimated drive deceleration completes.
            if(!elapsed(now_ms,config.stopped_dwell_ms)) return;
            transition=Transition::STEERING; timer_active=false;
        }
        // Rejected commands retain the generated angles while all four RPMs stop.
        if(!feasible) { timer_active=false; return; }
        // Shared progress makes both commanded angles arrive together for a
        // fixed target. The axis with the largest remaining travel sets the
        // duration; the other uses the same fraction of its own signed delta.
        // Recompute from current commands when a target changes mid-motion.
        float max_distance=0;
        for(int i=0;i < STEERING_COUNT;++i)
            max_distance=fmaxf(max_distance,fabsf(target_angle[i]-angle[i]));
        const float fraction=max_distance>0
            ? fminf(1.0f,config.steering_rate*dt/max_distance) : 1.0f;
        bool reached=true, changed=false;
        for(int i=0;i < STEERING_COUNT;++i) {
            previous_angle[i]=angle[i];
            if(fraction>=1.0f) angle[i]=target_angle[i];
            else angle[i]+=fraction*(target_angle[i]-angle[i]);
            reached &= fabsf(target_angle[i]-angle[i])<ANGLE_REACHED_TOLERANCE_RAD;
            changed |= fabsf(previous_angle[i]-angle[i])>ANGLE_REACHED_TOLERANCE_RAD;
        }
        if(transition==Transition::STEERING && reached) {
            transition=Transition::SETTLING; timer_active=false;
        }
        if(transition==Transition::SETTLING) {
            if(!reached || changed) { transition=Transition::STEERING; timer_active=false; }
            else if(elapsed(now_ms,config.settle_ms)) { transition=Transition::DRIVING; timer_active=false; }
        }
        if(transition==Transition::DRIVING) ramp(target.rpm,dt);
    }
private:
    bool initialized=false, timer_active=false, pivot_active=false;
    uint32_t last_ms=0, startup_at=0, timer_at=0;
    bool elapsed(uint32_t now,uint32_t interval) {
        if(!timer_active) { timer_at=now; timer_active=true; }
        return static_cast<uint32_t>(now-timer_at)>=interval;
    }
    void ramp(const float *target,float dt) {
        // One interpolation fraction keeps all wheel targets coordinated.
        float fraction=1;
        for(int i=0;i < WHEEL_COUNT;++i) {
            float goal=target?target[i]:0;
            float distance=fabsf(goal-rpm[i]);
            float rate=fabsf(goal)>fabsf(rpm[i])?config.acceleration_rpm_s:config.deceleration_rpm_s;
            if(distance>0) fraction=fminf(fraction,rate*dt/distance);
        }
        for(int i=0;i < WHEEL_COUNT;++i) rpm[i]+=fraction*((target?target[i]:0)-rpm[i]);
    }
};
} // namespace vehicle
