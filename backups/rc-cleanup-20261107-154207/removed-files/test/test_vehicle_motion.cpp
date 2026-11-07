#include <cassert>
#include <cmath>
#include <initializer_list>
#include "vehicle_kinematics.hpp"
using namespace vehicle;
static bool near(float a, float b, float eps=0.0001f) { return fabsf(a-b)<eps; }
int main() {
    Geometry g;
    float previous[2]={0,0}, lo[2]={-70*deg,-70*deg}, hi[2]={70*deg,70*deg};
    auto solve=[&](float v,float w) { return inverse({v,w},DriveMode::INDEPENDENT_STEERING,g,previous,lo,hi); };
    auto a=solve(.5f,0); assert(a.feasible && near(a.angle[0],0));
    for(float rpm:a.rpm) assert(near(rpm,speed_to_rpm(.5f,g)));
    auto b=solve(-.5f,0); for(int i=0;i<4;++i) assert(near(b.rpm[i],-a.rpm[i]));
    assert(near(b.angle[0],0));
    auto c=solve(.5f,1); assert(c.feasible && c.angle[0]>c.angle[1] && c.angle[1]>0);
    auto d=solve(0,1); assert(d.feasible && near(d.angle[0]/deg,-62.41f,.02f));
    assert(near(d.angle[1],-d.angle[0]) && d.rpm[0]<0 && d.rpm[2]<0 && d.rpm[1]>0 && d.rpm[3]>0);
    assert(near(d.rpm[0]/d.rpm[2],2.159f,.001f));
    auto e=solve(.2195f,1); assert(!e.feasible); for(float rpm:e.rpm) assert(rpm==0);
    previous[0]=.3f; previous[1]=-.2f;
    auto f=solve(0,0); assert(f.feasible && near(f.angle[0],previous[0]));
    for(float rpm:f.rpm) assert(rpm==0);
    assert(!solve(NAN,0).feasible && !solve(0,INFINITY).feasible);
    // Reconstruct each nonzero theoretical vector including reverse, without CAN sign assumptions.
    for(float v: {-.5f,0.f,.5f}) for(float w: {-1.f,0.f,1.f}) {
        auto t=solve(v,w); if(!t.feasible) continue;
        for(int i=0;i<2;++i) {
            const float speed=t.rpm[i]/speed_to_rpm(1,g);
            assert(near(speed*cosf(t.angle[i]),v-w*(i==0?1:-1)*g.front_track/2));
            assert(near(speed*sinf(t.angle[i]),w*g.wheelbase));
        }
    }
}
