#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace tricopter;
static constexpr float dt=.0005f;
static const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
struct Rig {
  CycloneControl control;
  SpinDynamicsModel spin;
  AttitudeReferenceConfig reference;
  SpinSafetyConfig safety;
  ControlOutput out{};
  float heading=0;
  Rig() { spin.maxRateDps=10; spin.maxAccelDps2=10; }
  void tick(unsigned s1,float actual=0,float physicalTarget=0,bool cut=false,float throttle=.3f) {
    heading=wrapDegrees(heading+actual*dt);
    const Quaternion q{std::cos(heading*radians*.5f),0,0,-std::sin(heading*radians*.5f)};
    control.applyAllocationFeedback(1,1,false,physicalTarget);
    const auto in=pilotInput(1000+static_cast<unsigned>(throttle*1000),1500,1500,
                             1500,s1,control.spinning());
    out=control.step(in,q,{0,0,actual},dt,true,cut,true,gains,&spin,&reference,&safety);
    assert(!out.fault);
  }
  void run(unsigned frames,unsigned s1,float actual=0,float target=0) {
    for(unsigned i=0;i<frames;++i) tick(s1,actual,target);
  }
};
static void earlyRequestMustNotConsumeFlight() {
  Rig r;
  // Captured failure: normal Yaw still requesting +20 while the airframe turns
  // the other way. CH6=1425 used to enter Spin and consume the abort latch.
  r.run(3000,1425,-10,20);
  assert(r.out.mode==FlightMode::Normal && !r.control.supervisor().inhibited());
  r.run(2500,2000,-10,20); // even an endpoint cannot enter while unsettled
  assert(r.out.mode==FlightMode::Normal && r.control.spinEntryState()==SpinEntryState::CenterRequired);
  r.run(3000,2000); // stability alone must not start a previously rejected request
  assert(r.out.mode==FlightMode::Normal && !r.control.supervisor().inhibited());
  r.run(1700,1500);
  assert(r.control.spinEntryState()==SpinEntryState::Ready);
  r.run(620,2000);
  assert(r.out.mode==FlightMode::Spin && r.out.yawTarget<0);
  assert(!r.control.supervisor().inhibited());
}
static void deliberateRequestAndSlew() {
  for(unsigned s1 : {1000u,2000u}) {
    Rig r; r.run(1700,1500);
    assert(r.control.spinEntryState()==SpinEntryState::Ready);
    r.run(200,s1); // 100 ms accidental excursion is not a mode change
    assert(r.out.mode==FlightMode::Normal);
    r.run(30,1500);
    r.run(590,s1);
    assert(r.out.mode==FlightMode::Normal);
    float previous=0;
    for(unsigned i=0;i<100;++i) {
      r.tick(s1);
      assert(std::fabs(r.out.yawTarget-previous)<=.0051f);
      previous=r.out.yawTarget;
    }
    assert(r.out.mode==FlightMode::Spin && !r.control.supervisor().inhibited());
    assert(s1==1000 ? r.out.yawTarget>0 : r.out.yawTarget<0);
  }
}
static void targetReadinessAndAbortStillApply() {
  Rig r; r.run(1700,1500,0,20); // measured rest alone does not prove a settled controller
  assert(r.control.spinEntryState()==SpinEntryState::Settling);
  r.run(1000,1000,0,20);
  assert(r.out.mode==FlightMode::Normal && !r.control.supervisor().inhibited());
  r.run(1700,1500); r.run(620,1000);
  assert(r.out.mode==FlightMode::Spin);
  r.run(550,1000,30,5); // real overspeed is still protected at the original dwell
  assert(r.control.supervisor().reasons()&spinOverspeed);
  assert(r.out.mode==FlightMode::Braking && r.out.motors.m2>0);
  assert(r.control.spinEntryState()==SpinEntryState::AbortLocked);
  r.tick(1000,0,0,false,0); // throttle alone never clears the abort
  r.tick(1000,0,0,true,0); // SA cut with displaced S1 also cannot clear it
  assert(r.control.supervisor().inhibited());
  r.tick(1500,0,0,true,0);
  assert(!r.control.supervisor().inhibited());
  r.run(2000,1000); // after idle, S1 must be qualified from center again
  assert(r.out.mode==FlightMode::Normal);
  r.run(1700,1500); r.run(620,1000);
  assert(r.out.mode==FlightMode::Spin);
}
static void vibrationMustNotPermanentlyBlockEntry() {
  Rig r;
  // Short alternating rate fluctuations within the original tracking guard
  // must not make an otherwise stationary nose permanently ineligible.
  for(unsigned i=0;i<2400;++i)
    r.tick(1500,7*std::sin(2*3.14159265f*8*i*dt));
  assert(r.control.spinEntryState()==SpinEntryState::Ready);
  for(unsigned i=0;i<620;++i)
    r.tick(2000,7*std::sin(2*3.14159265f*8*i*dt));
  assert(r.out.mode==FlightMode::Spin && !r.control.supervisor().inhibited());

  Rig moving; moving.run(1700,1500); moving.run(200,2000);
  moving.tick(2000,12); // a genuine instantaneous turn still rejects pending entry
  moving.run(2000,2000);
  assert(moving.out.mode==FlightMode::Normal &&
         moving.control.spinEntryState()==SpinEntryState::CenterRequired);
  assert(!moving.control.supervisor().inhibited());
}
int main() {
  earlyRequestMustNotConsumeFlight(); deliberateRequestAndSlew(); targetReadinessAndAbortStillApply();
  vibrationMustNotPermanentlyBlockEntry();
  std::puts("Captured early request, later deliberate entry, debounce, target readiness and unchanged abort/reset passed");
}
