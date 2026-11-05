#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace tricopter;
constexpr float dt=.0005f;
static const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
static Quaternion headingQ(float h) {
  return {std::cos(h*radians*.5f),0,0,-std::sin(h*radians*.5f)};
}
struct Rig {
  FlightParameters parameters;
  NormalYawConfig config;
  CycloneControl control;
  float heading=0;
  ControlOutput out{};
  Rig() {
    parameters.begin();config=parameters.config();config.ready=config.spinReady=true;
    config.spin.maxRateDps=15;
    for(unsigned i=0;i<1700;++i) tick({.5f,0,0,0,0,false});
  }
  ControlOutput tick(PilotInput in,float rate=0,bool cut=false) {
    heading=wrapDegrees(heading+rate*dt);
    out=control.step(in,headingQ(heading),{0,0,rate},dt,true,cut,true,gains,
        &config.spin,&config.reference,&config.safety);
    assert(!out.fault);
    control.applyAllocationFeedback(1,1,false,out.yawTarget);
    return out;
  }
  void start(PilotInput in) {
    for(unsigned i=0;i<620;++i) tick(in);
    assert(out.mode==FlightMode::Spin);
  }
};
static void stoppingMustBeMeasured() {
  Rig r;PilotInput in{.5f,0,0,0,2.f/3,true};
  r.start(in);
  for(unsigned i=0;i<4000;++i) r.tick(in,10);
  in.spin=0;in.spinRequested=false;
  for(unsigned i=0;i<16000;++i) {
    const auto out=r.tick(in,10);
    assert(out.mode==FlightMode::Braking && out.motors.m2>0);
  }
  assert(r.control.supervisor().reasons()&spinBrakingTimeout);
  assert(r.control.supervisor().inhibited());
  std::puts("Persistent 10 deg/s cannot exit braking; timeout latches without a motor cut");
}
static void dwellAndReacceleration() {
  Rig r;PilotInput in{.5f,0,0,0,.2f,true};
  r.start(in);
  for(unsigned i=0;i<1000;++i) r.tick(in,3);
  in.spin=0;in.spinRequested=false;
  // First bring only the target to zero. A moving airframe cannot build dwell.
  for(unsigned i=0;i<1000;++i) r.tick(in,2);
  for(unsigned i=0;i<800;++i) assert(r.tick(in).mode==FlightMode::Braking);
  r.tick(in,20); // instantaneous tracking-envelope breach resets the dwell
  for(unsigned i=0;i<960;++i) assert(r.tick(in).mode==FlightMode::Braking);
  for(unsigned i=0;i<80;++i) r.tick(in);
  assert(r.out.mode==FlightMode::Aligning);
  assert(r.tick(in,20).mode==FlightMode::Braking);
  for(unsigned i=0;i<960;++i) assert(r.tick(in).mode==FlightMode::Braking);
  assert(!r.control.supervisor().inhibited());
  std::puts("Stop dwell resets on an instantaneous envelope breach; alignment returns to braking");
}
static void vibrationAndResidualRotation() {
  for(float frequency : {5.f,20.f}) for(float residual : {0.f,6.f,-6.f}) {
    Rig r;PilotInput in{.5f,0,0,0,.2f,true};r.start(in);
    for(unsigned i=0;i<1000;++i) r.tick(in,3);
    in.spin=0;in.spinRequested=false;
    bool aligned=false,normal=false;
    for(unsigned i=0;i<16000;++i) {
      // Zero-mean ripple: peak heading excursion <=0.20 deg at 5/20 Hz.
      const float rate=residual+6*std::sin(2*3.14159265f*frequency*i*dt);
      const auto out=r.tick(in,rate);
      aligned=aligned || out.mode==FlightMode::Aligning;
      normal=normal || out.mode==FlightMode::Normal;
      if(residual!=0) assert(out.mode==FlightMode::Braking);
    }
    if(residual==0) assert(aligned && normal && !r.control.supervisor().inhibited());
    else assert(!aligned && !normal && (r.control.supervisor().reasons()&spinBrakingTimeout));
  }
  std::puts("Zero-mean vibration permits handoff; persistent +/-6 deg/s still blocks and times out");
}
static void boundedHandoff(float sign) {
  Rig r;PilotInput in{.5f,0,0,0,sign,true};
  r.start(in);
  Vec3 lastUp{0,0,-1};float peakReference=0,peakDirection=0;
  for(unsigned i=0;i<24000;++i) {
    if(i>=4000) {in.roll=.3f;in.pitch=.4f;}
    r.tick(in,sign*15); // consistent 180-degree phase
  }
  in.spin=0;in.spinRequested=false;
  bool aligning=false,normal=false;
  unsigned alignFrames=0;
  lastUp=r.out.desiredUp;
  for(unsigned i=0;i<32000;++i) {
    const auto out=r.tick(in);
    const float directionRate=vectorAngle(lastUp,out.desiredUp)*degrees/dt;
    assert(directionRate<=r.config.safety.tiltCommandDps+.03f);
    peakDirection=std::fmax(peakDirection,directionRate);lastUp=out.desiredUp;
    if(out.mode==FlightMode::Aligning) {
      aligning=true;++alignFrames;
      peakReference=std::fmax(peakReference,std::fabs(out.referenceRate));
      assert(std::fabs(out.referenceRate)<=r.config.safety.exitReferenceDps+.001f);
    }
    if(out.mode==FlightMode::Normal) {normal=true;break;}
  }
  assert(aligning && normal && alignFrames*dt>=8.9f);
  assert(!r.control.supervisor().inhibited());
  // A full right-stick reversal immediately after handoff is still bounded.
  in.roll=-1;in.pitch=-1;
  for(unsigned i=0;i<1000;++i) {
    const auto out=r.tick(in);
    assert(vectorAngle(lastUp,out.desiredUp)*degrees/dt<=r.config.safety.tiltCommandDps+.03f);
    lastUp=out.desiredUp;
  }
  std::printf("180-degree handoff sign %.0f: %.2f s alignment, reference <=%.3f deg/s, axis <=%.3f deg/s\n",
      sign,alignFrames*dt,peakReference,peakDirection);
}
static void yawDuringAlignment() {
  Rig r;PilotInput in{.5f,0,0,0,.2f,true};
  r.start(in);
  for(unsigned i=0;i<2000;++i) r.tick(in,3);
  in.spin=0;in.spinRequested=false;
  while(r.out.mode!=FlightMode::Aligning) r.tick(in);
  in.yaw=.5f;
  for(unsigned i=0;i<4000;++i) {
    const auto out=r.tick(in);
    assert(out.mode==FlightMode::Aligning && std::fabs(out.referenceRate-10)<.001f);
  }
  in.yaw=0;
  for(unsigned i=0;i<6000;++i) r.tick(in);
  assert(r.out.mode==FlightMode::Normal);
}
static void physicalRateTargetMustStop() {
  Rig r;PilotInput in{.5f,0,0,0,.2f,true};
  r.start(in);
  for(unsigned i=0;i<1000;++i) {
    r.tick(in);r.control.applyAllocationFeedback(1,1,false,5);
  }
  in.spin=0;in.spinRequested=false;
  for(unsigned i=0;i<4000;++i) {
    assert(r.tick(in).mode==FlightMode::Braking);
    r.control.applyAllocationFeedback(1,1,false,5);
  }
  for(unsigned i=0;i<1200;++i) {
    r.tick(in);r.control.applyAllocationFeedback(1,1,false,0);
  }
  assert(r.out.mode==FlightMode::Aligning && !r.control.supervisor().inhibited());
}
static void supervisorReasons() {
  const SpinSafetyConfig cfg;
  for(unsigned reason:{spinOverspeed,spinRateError,spinTiltError,spinAllocation,spinBrakingTimeout}) {
    SpinSupervisor supervisor;
    const auto frame=[&](bool exceed) {
      supervisor.update(cfg,true,15,0,
          exceed && reason==spinOverspeed ? 25 : (exceed && reason==spinRateError ? 10 : 0),
          exceed && reason==spinTiltError ? 10 : 0,
          exceed && reason==spinAllocation ? .8f : 1,1,false,
          exceed && reason==spinBrakingTimeout ? 8 : 0,7,dt);
    };
    if(reason!=spinBrakingTimeout) {
      for(unsigned i=0;i<100;++i) frame(true);
      frame(false);
      assert(!supervisor.inhibited());
    }
    for(unsigned i=0;i<2500;++i) frame(true);
    assert(supervisor.reasons()&reason);
    assert(supervisor.count()==1);
    frame(false);supervisor.idle();
    assert(supervisor.inhibited());
    supervisor.clear();assert(!supervisor.inhibited() && supervisor.count()==1);
  }
}
static void allocationAbortAndReset() {
  Rig r;NormalYawControl yaw;
  PilotInput in{.5f,0,0,0,.1f,true};
  r.start(in);
  for(unsigned i=0;i<1400;++i) {
    const auto refs=r.tick(in);
    YawControlRequest req;req.rateMode=refs.mode!=FlightMode::Normal;
    req.rateTargetDps=refs.yawTarget;req.attitude=headingQ(r.heading);req.gyroDps={0,0,0};
    const auto out=yaw.step(r.config,refs.motors,refs.heading,refs.yawRate,0,dt,true,req);
    assert(out.active && !out.fault && out.motors.m2>0 && out.motors.m4>0 && out.motors.m3>0);
    r.control.applyAllocationFeedback(.8f,1,true,out.rateTarget);
  }
  assert(r.control.supervisor().reasons()&spinAllocation);
  for(unsigned i=0;i<5000;++i) assert(r.tick(in).mode!=FlightMode::Spin);
  in.throttle=0;
  r.tick(in);assert(r.control.supervisor().inhibited()); // idle alone cannot clear
  r.tick(in,0,true);assert(r.control.supervisor().inhibited()); // S1 still displaced
  in.spinRequested=false; // Feature disabled: preserve the real, displaced S1 for reset.
  r.tick(in,0,true);assert(r.control.supervisor().inhibited());
  in.spin=0;in.spinRequested=false;
  r.tick(in,0,true);assert(!r.control.supervisor().inhibited());
  in.throttle=.5f;
  for(unsigned i=0;i<1700;++i) r.tick(in);
  in.throttle=.5f;in.spin=.1f;in.spinRequested=true;
  r.start(in);
}
static void collectiveBeforeYaw() {
  FlightParameters p;p.begin();const auto m=p.config().model;
  const float vertical=servoForTilt(m.rotor[2],0);
  const float servo=vertical-4;
  const Vec3 request{3*m.rotor[0].thrustN[4],0,0};
  const auto before=allocateFeasibleAt(m,request,2,servo);
  const auto after=allocateYaw(m,request,2,servo,.2f);
  assert(before.valid && after.valid && before.liftScale<1);
  assert(after.liftScale>before.liftScale);
  assert(std::fabs(after.servoDeg-vertical)<std::fabs(servo-vertical));
  assert(std::fabs(after.servoDeg-servo)<=.2001f);
  std::puts("Saturated allocation recovers collective before satisfying more yaw torque");
}
int main() {
  stoppingMustBeMeasured();dwellAndReacceleration();vibrationAndResidualRotation();
  boundedHandoff(1);boundedHandoff(-1);
  yawDuringAlignment();physicalRateTargetMustStop();supervisorReasons();
  allocationAbortAndReset();collectiveBeforeYaw();
  std::puts("Low-speed transition boundaries, pilot handoff, spin supervision and reset interlocks passed");
}
