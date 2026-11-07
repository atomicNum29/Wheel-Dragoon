#include <cassert>
#include <cmath>
#include "vehicle_motion.hpp"
using namespace vehicle;
static ServoCalibration measured(bool right=false) {
    ServoCalibration c = right ? right_servo : left_servo;
    c.verified=true;
    c.inverted=false; // Synthetic fixture direction is independent of installed servos.
    c.minimum_angle=-70*deg; c.maximum_angle=70*deg;
    c.minimum_us=1000; c.maximum_us=2000;
    // Synthetic calibration fixtures; NOT measurements of real RDS51150s.
    c.points[0]={-70*deg,1000}; c.points[1]={0,1500}; c.points[2]={70*deg,2000};
    return c;
}
int main() {
    MotionConfig cfg;
    cfg.initial_mode=DriveMode::INDEPENDENT_STEERING;
    MotionController uncalibrated(cfg,left_servo,right_servo);
    uncalibrated.calibration[0].verified=false;
    uncalibrated.calibration[1].verified=false;
    uncalibrated.update({.5f,0},0,true);
    uncalibrated.update({.5f,0},5000,true);
    assert(!uncalibrated.ready && uncalibrated.stopped());
    assert(pulse_for_angle(uncalibrated.calibration[0],70*deg)==0);
    auto left=measured(),right=measured(true);
    right.inverted=true;
    assert(valid_calibration(left) && valid_calibration(right));
    assert(pulse_for_angle(left,0)==1500 && pulse_for_angle(right,0)==1500);
    assert(pulse_for_angle(left,70*deg)==2000 && pulse_for_angle(right,70*deg)==1000);
    left.count=5;
    left.points[0]={-70*deg,1000}; left.points[1]={-20*deg,1400};
    left.points[2]={0,1500}; left.points[3]={20*deg,1650}; left.points[4]={70*deg,2000};
    assert(pulse_for_angle(left,10*deg)==1575);
    auto bad=left; bad.points[3].angle=NAN; assert(!valid_calibration(bad));
    bad=left; bad.neutral_us=1600; assert(!valid_calibration(bad));
    MotionConfig excessive=cfg;
    excessive.rpm_limit=mdh100_maximum_rpm+1;
    MotionController invalid_config(excessive,left,right);
    invalid_config.update({.5f,0},0,true);
    invalid_config.update({.5f,0},5000,true);
    assert(!invalid_config.ready && invalid_config.stopped());
    MotionController m(cfg,left,right);
    uint32_t now=0;
    auto tick_enabled=[&](VehicleCommand cmd, bool enabled) {
        const float old_angle[2]={m.angle[0],m.angle[1]};
        const float old_rpm[4]={m.rpm[0],m.rpm[1],m.rpm[2],m.rpm[3]};
        now+=10; m.update(cmd,now,enabled);
        for(int i=0;i<2;++i) {
            assert(fabsf(m.angle[i]-old_angle[i])<=cfg.steering_rate*.01f+1e-6f);
            assert(fabsf(m.angle[i])<=70*deg);
        }
        if(enabled) for(int i=0;i<4;++i) {
            assert(fabsf(m.rpm[i]-old_rpm[i])<=cfg.deceleration_rpm_s*.01f+1e-4f);
            assert(fabsf(m.rpm[i])<=cfg.rpm_limit);
        }
    };
    auto tick=[&](VehicleCommand cmd) { tick_enabled(cmd,true); };
    for(int i=0;i<150;++i) tick({.5f,0});
    assert(!m.ready && m.stopped());
    for(int i=0;i<200;++i) tick({.5f,0});
    assert(m.ready && m.rpm[0]>70);
    assert(!m.request_mode(DriveMode::SKID_STEER,true)); // generated RPM still moving
    bool zero_seen=false;
    for(int i=0;i<300;++i) {
        tick({-.5f,0});
        zero_seen |= m.stopped();
        if(m.rpm[0]<0) assert(zero_seen);
        assert(fabsf(m.angle[0])<1e-6f);
    }
    assert(m.rpm[0]<-70);
    // Reversing wheel travel must still brake with steering held at zero RPM.
    const float held=m.angle[0];
    tick({.5f,1});
    assert(m.transition==Transition::BRAKING && m.angle[0]==held);
    for(int i=0;i<800;++i) {
        tick({.5f,1});
        if(m.transition==Transition::STEERING || m.transition==Transition::SETTLING) assert(m.stopped());
    }
    assert(m.transition==Transition::DRIVING && m.angle[0]>m.angle[1] && m.rpm[0]>0);
    for(int i=0;i<100;++i) tick({.5f,(i%2)?1.f:-1.f});
    // Steering target changes alone no longer request a stop.
    assert(m.transition==Transition::DRIVING && !m.stopped());
    for(int i=0;i<800;++i) tick({0,1});
    assert(m.rpm[0]<0 && m.rpm[1]>0 && m.angle[0]<0 && m.angle[1]>0);
    for(int i=0;i<400;++i) tick({.2195f,1});
    assert(!m.feasible && m.stopped());
    float old=m.angle[0];
    for(int i=0;i<100;++i) tick({0,0});
    assert(m.stopped() && fabsf(old-m.angle[0])<1e-6f);
    assert(!m.request_mode(DriveMode::SKID_STEER,false));
    assert(m.request_mode(DriveMode::SKID_STEER,true));
    for(int i=0;i<700;++i) tick({0,0});
    assert(m.mode==DriveMode::SKID_STEER && m.angle[0]==0 && m.angle[1]==0);
    for(int i=0;i<300;++i) tick({2,5});
    assert(m.limited);
    tick_enabled({0,0},false); assert(m.stopped()); // immediate inhibit, separate from normal ramp
    // Unsigned elapsed time survives wrap; long stalls cap motion increments at 50 ms.
    MotionController wrap(cfg,left,right);
    wrap.update({.5f,0},0xFFFFFF00u,true);
    wrap.update({.5f,0},0x00001000u,true);
    assert(wrap.ready && wrap.stopped());
}
