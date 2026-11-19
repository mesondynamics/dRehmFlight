#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace tricopter;
static constexpr float dt=.0005f;
static const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};

struct Rig {
  FlightParameters parameters;
  CycloneControl control;
  NormalYawControl yaw;
  ControlOutput out{};
  NormalYawOutput physical{};
  float heading=0;
  Rig() {
    parameters.begin();
    assert(parameters.applyNormalHoverAtStartup(0x1881bf));
  }
  void tick(uint16_t s1,float measuredRate,uint16_t yawUs=1500,uint16_t throttle=1500,
            bool armed=true,bool cut=false,bool healthy=true) {
    uint16_t rc[]={yawUs,1500,throttle,1500,uint16_t(cut ? 2000 : 1000),s1},mapped[6]{};
    mapCrsfChannels(rc,mapped);
    auto pilot=pilotInput(mapped[0],mapped[1],mapped[2],mapped[3],mapped[5]);
    const auto &c=parameters.config();
    if(!c.spinEnabled || !c.spinReady) pilot.spinRequested=false;
    heading=wrapDegrees(heading+measuredRate*dt);
    const Quaternion q{std::cos(heading*radians*.5f),0,0,-std::sin(heading*radians*.5f)};
    const Vec3 gyro{0,0,measuredRate};
    out=control.step(pilot,q,gyro,dt,armed,cut,healthy,gains,&c.spin,&c.reference,&c.safety);
    YawControlRequest request;
    request.rateMode=out.mode!=FlightMode::Normal;
    request.rateTargetDps=out.yawTarget; request.attitude=q; request.gyroDps=gyro;
    physical=yaw.step(c,out.motors,out.heading,out.yawRate,pilot.yaw,dt,
                      armed && !cut && pilot.throttle>=.06f && !out.fault,request);
    if(physical.active)
      control.applyAllocationFeedback(physical.liftScale,physical.attitudeScale,
                                      physical.limited,physical.rateTarget);
  }
};

static void immediateRequestAndInhibition() {
  Rig r;
  assert(r.control.spinStatus(true)==SpinStatus::Idle);
  r.tick(1469,24,1800,1000,false,true);
  assert(r.out.mode==FlightMode::Normal && !r.physical.active);
  // First active frame. Yaw is held, measured heading rate is above the old
  // acquisition band and the allocator has not supplied a previous sample.
  r.tick(1469,24,1800);
  assert(r.out.mode==FlightMode::Spin && r.out.referenceRate<0 && r.physical.active);
  assert(r.control.spinStatus(true)==SpinStatus::Active && !r.control.supervisor().inhibited());
  r.tick(1470,24);
  assert(r.out.mode==FlightMode::Braking);
  r.tick(1531,24);
  assert(r.out.mode==FlightMode::Spin);
  r.tick(1530,24);
  assert(r.out.mode==FlightMode::Braking);
  // The aircraft-wide sensor fault still stops the active controller.
  r.tick(2000,0,1500,1500,true,false,false);
  assert(r.out.fault && r.out.mode==FlightMode::Normal && !r.physical.active);
}

static void fullTravelAndBraking() {
  Rig r;
  r.tick(1500,0); assert(r.control.spinStatus(true)==SpinStatus::Normal);
  assert(r.control.spinStatus(false)==SpinStatus::Disabled);
  // Supply tracked measured rates to isolate RC mapping, mode selection and
  // the actual allocator setpoint slew. This is a command-path regression.
  for(uint16_t s1 : {1000,1235,2000}) {
    for(unsigned i=0;i<50000;++i) {
      const float previous=r.physical.rateTarget;
      r.tick(s1,previous);
      assert(r.out.mode==FlightMode::Spin && !r.physical.fault && !r.control.supervisor().inhibited());
      assert(std::fabs(r.physical.rateTarget-previous)<=30.f*dt+.0001f);
      assert(std::fabs(r.physical.rateTarget)<=360.f);
    }
    const float expected=s1==1000 ? 360.f : (s1==1235 ? 180.f : -360.f);
    assert(std::fabs(r.physical.rateTarget-expected)<.001f);
  }
  for(unsigned i=0;i<2000;++i) {
    const float previous=r.physical.rateTarget;
    r.tick(1500,previous);
    assert(r.out.mode==FlightMode::Braking && r.physical.active);
    assert(r.physical.rateTarget>=previous && r.physical.rateTarget-previous<=30.f*dt+.0001f);
  }
}

static void overspeedLocksUntilCut() {
  Rig r;
  r.tick(2000,-390);
  assert(r.out.mode==FlightMode::Spin && std::fabs(r.out.yawTarget)<=360.f);
  for(unsigned i=0;i<510;++i) r.tick(2000,-390);
  assert((r.control.supervisor().reasons()&spinOverspeed)!=0);
  assert(r.out.mode==FlightMode::Braking && r.physical.active);
  r.tick(1000,0); assert(r.control.spinStatus(true)==SpinStatus::AbortLocked);
  r.tick(1500,0); r.tick(2000,0);
  assert(r.out.mode!=FlightMode::Spin && r.control.supervisor().inhibited());
  r.tick(2000,0,1500,1000,false,true); assert(r.control.supervisor().inhibited());
  r.tick(1500,0,1500,1000,false,true); assert(!r.control.supervisor().inhibited());
}

int main() {
  immediateRequestAndInhibition();
  fullTravelAndBraking();
  overspeedLocksUntilCut();
  std::puts("Direct CH6 selection, 60 RPM endpoints, slew, braking and airborne lock passed");
}
