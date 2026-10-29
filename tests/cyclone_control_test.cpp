#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
using namespace tricopter;
static const float dt = .0005f;
static const Quaternion level{1,0,0,0};
static const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
static const ControlGains noI{{.002f,0,.0005f},{.002f,0,.0005f},{.003f,0,.0000015f}};
static bool near(float a, float b, float eps=.00005f) { return std::fabs(a-b) < eps; }
static Quaternion multiply(Quaternion a, Quaternion b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
static Quaternion attitude(float roll, float pitch, float heading) {
  const float r=roll*radians/2, p=-pitch*radians/2, h=-heading*radians/2;
  return multiply(multiply({std::cos(h),0,0,std::sin(h)}, {std::cos(p),0,std::sin(p),0}),
                  {std::cos(r),std::sin(r),0,0});
}
static PilotInput input(unsigned r=1500, unsigned p=1500, unsigned y=1500, unsigned s=1500,
                        bool spinning=false, unsigned t=1500) {
  return pilotInput(t,r,p,y,s,spinning);
}
static ControlOutput tick(CycloneControl &c, PilotInput in, Quaternion q=level, Vec3 g={0,0,0},
                          const ControlGains &cg=gains) {
  return c.step(in,q,g,dt,true,false,true,cg);
}

static void radioAndNormal() {
  auto in=input(2000,2000,2000,1000);
  assert(in.roll == 1 && in.pitch == 1 && in.yaw == 1 && in.spin == 1 && in.spinRequested);
  in=input(1000,1000,1000,2000);
  assert(in.roll == -1 && in.pitch == -1 && in.yaw == -1 && in.spin == -1);
  for (unsigned v : {1480u,1500u,1520u}) {
    in=input(v,v,v);
    assert(in.roll == 0 && in.pitch == 0 && in.yaw == 0);
  }
  assert(!input(1500,1500,1500,1550).spinRequested);
  assert(input(1500,1500,1500,1551).spinRequested);
  assert(input(1500,1500,1500,1540,true).spinRequested);
  assert(!input(1500,1500,1500,1530,true).spinRequested);
  CycloneControl c;
  auto o=tick(c,input(2000));
  assert(o.motors.m2 > o.motors.m4 && o.roll > 0);
  o=tick(c,input(1500,2000));
  assert(o.pitch > 0 && o.motors.m3 > o.motors.m2);
  CycloneControl left, right, feedback;
  // Direction must be correct after the commanded ramp/filter settles; the
  // first sub-millisecond command intentionally stays inside the rate deadband.
  for (int i=0;i<200;++i) {
    tick(left,input(1500,1500,2000));
    tick(right,input(1500,1500,1000));
  }
  assert(tick(left,input(1500,1500,2000)).servoDeg < servoCenterDeg);
  assert(tick(right,input(1500,1500,1000)).servoDeg > servoCenterDeg);
  assert(tick(feedback,input(),level,{0,0,30}).servoDeg > servoCenterDeg);
  // Cut/idle reset all integrators and the spin reference.
  for (int i=0;i<4000;++i) tick(c,input(2000,2000,2000,1000));
  o=c.step(input(1500,1500,1500,1500,false,1000),level,{0,0,0},dt,false,true,true,gains);
  assert(o.servoDeg == servoCenterDeg && o.motors.m2 == 0 && o.mode == FlightMode::Normal);
  o=tick(c,input());
  assert(near(o.roll,0) && near(o.pitch,0) && near(o.yaw,0));
  o=c.step(input(2000,2000,2000,1000),level,{0,0,0},dt,true,true,true,gains);
  assert(o.motors.m2==0 && o.motors.m3==0 && o.servoDeg==servoCenterDeg);
}

static void phaseAndDamping() {
  for (float sign : {-1.f,1.f}) {
    CycloneControl c;
    const auto spin=input(1500,1500,1500,sign > 0 ? 1000 : 2000);
    for (int i=0;i<4000;++i) tick(c,spin,level,{0,0,0},noI);
    for (float phase : {0.f,90.f,179.9f,180.1f,270.f,360.f,720.f,-540.f}) {
      const Quaternion q=attitude(0,0,phase);
      auto o=tick(c,spin,q,{0,0,sign*360},noI);
      assert(near(o.roll,0) && near(o.pitch,0)); // pure self spin is not a tilt disturbance
      auto forward=spin; forward.pitch=.5f;
      o=tick(c,forward,q,{0,0,sign*360},noI);
      // Convert the actual actuator moments to world; they always tilt forward.
      const Vec3 torque=Rotation(q).world({o.roll,-o.pitch,0});
      assert(near(torque.x,0) && torque.y < -.01f);
      assert(near(torque.z,0));
    }
    // Fixed spatial tilt while spinning about its own thrust axis: no false damping.
    for (float phase : {0.f,90.f,180.f,270.f,359.f}) {
      auto forward=spin; forward.pitch=2.f/3;
      const Quaternion q=multiply(attitude(0,10,0),attitude(0,0,phase));
      const auto o=tick(c,forward,q,{0,0,sign*360},noI);
      assert(near(o.roll,0,.0001f) && near(o.pitch,0,.0001f));
    }
  }
  // Normal yaw with body-relative tilt held constant: remove reference rotation.
  for (float heading : {0.f,90.f,179.9f,-179.9f,-90.f}) {
    CycloneControl c;
    const Quaternion q=attitude(6,9,heading);
    const Vec3 b=Rotation(q).body({0,0,-120});
    PilotInput in{.5f,.2f,.3f,0,0,false};
    const auto o=tick(c,in,q,{b.x,-b.y,-b.z},noI);
    assert(near(o.yawRate,120,.0002f));
    assert(near(o.roll,0,.0001f) && near(o.pitch,0,.0001f));
  }
  // Reference turn crosses the wrap without a motor discontinuity or a speed change.
  CycloneControl c;
  auto in=input(1500,1500,2000,1000);
  auto o=tick(c,in,attitude(0,0,179.9f),{0,0,0},noI);
  float previous=o.reference;
  for (int i=0;i<6000;++i) {
    o=tick(c,in,attitude(0,0,179.9f),{0,0,0},noI);
    assert(near(wrapDegrees(o.reference-previous),pilotYawMaxDps*dt,.0001f));
    assert(near(o.roll,0) && near(o.pitch,0));
    previous=o.reference;
  }
  assert(near(o.yawTarget,360,.001f));
}

static void modesAndSaturation() {
  CycloneControl c;
  ControlOutput o{};
  float previous=0;
  for (int i=0;i<5000;++i) {
    o=tick(c,input(1500,1500,1500,1000));
    assert(std::fabs(o.yawTarget-previous) <= yawSlewDps2*dt+.0001f);
    previous=o.yawTarget;
  }
  assert(near(o.yawTarget,360,.001f) && near(o.tiltLimit,15));
  // Reverse through zero, bounded acceleration in both directions.
  bool crossedZero=false;
  for (int i=0;i<10000;++i) {
    o=tick(c,input(1500,1500,1500,2000));
    if (std::fabs(o.yawTarget)<.1f) crossedZero=true;
    assert(std::fabs(o.yawTarget-previous) <= yawSlewDps2*dt+.0001f);
    previous=o.yawTarget;
  }
  assert(crossedZero && near(o.yawTarget,-360,.001f));
  // Return to center while real spin persists: never exit based on CH6 alone.
  for (int i=0;i<7000;++i) o=tick(c,input(),level,{0,0,-100});
  assert(o.mode == FlightMode::Braking && near(o.yawTarget,0));
  for (int i=0;i<610;++i) o=tick(c,input());
  assert(o.mode == FlightMode::Aligning);
  o=tick(c,input(1500,1500,1500,1000));
  assert(o.mode == FlightMode::Spin); // re-entry during alignment
  for (int i=0;i<7000;++i) o=tick(c,input());
  assert(o.mode == FlightMode::Normal && near(o.reference,o.heading));
  // Holding a measured 100 deg/s during braking legitimately builds torque
  // trim at the lower gain. Returning throttle to idle must clear it before
  // the next spool-up; releasing the yaw stick alone must preserve hover trim.
  o=tick(c,input(1500,1500,1500,1500,false,1000));
  assert(o.servoDeg==servoCenterDeg && o.yawIntegral==0);
  o=tick(c,input());
  assert(std::fabs(o.yaw)<1e-6f);
  for (int i=0;i<20000;++i) o=tick(c,input(2000,2000,1500,1500,false,1999));
  assert(o.mixScale < 1);
  o=tick(c,input());
  assert(std::fabs(o.roll)<.001f && std::fabs(o.pitch)<.001f);
}

static void faultsAndFusion() {
  for (int which=0;which<7;++which) {
    CycloneControl c;
    Quaternion q=level;
    Vec3 g{0,0,0};
    if (which==0) q={0,0,0,0};
    if (which==1) q.w=std::numeric_limits<float>::quiet_NaN();
    if (which==2) g.z=std::numeric_limits<float>::infinity();
    if (which==5) q=attitude(180,0,0);
    if (which==6) q=attitude(0,90,0);
    auto o=c.step(input(),q,g,which==3 ? .1f : dt,true,false,which!=4,gains);
    assert(o.fault && o.motors.m2==0 && o.servoDeg==servoCenterDeg);
    const uint32_t reasons[]={faultQuaternion,faultQuaternion,faultGyroscope,faultTime,
                              faultSensor,faultInverted,faultHeadingSingularity};
    assert(c.faultReason() & reasons[which]);
    const uint32_t retainedReason=c.faultReason();
    assert(c.faultCount()==1);
    for (int i=0;i<1000;++i) o=tick(c,input());
    assert(o.fault); // healthy data alone cannot resume flight
    for (int i=0;i<1000;++i)
      o=c.step(input(1500,1500,1500,2000,false,1000),level,{0,0,0},dt,false,true,true,gains);
    assert(o.fault); // knob must be centered as well
    for (int i=0;i<610;++i)
      o=c.step(input(1500,1500,1500,1500,false,1000),level,{0,0,0},dt,false,true,true,gains);
    assert(!o.fault && o.motors.m2==0);
    assert(c.faultReason()==retainedReason && c.faultCount()==1);
    o=tick(c,input());
    assert(near(o.yaw,0) && o.mode==FlightMode::Normal);
    o=c.step(input(),level,{0,0,0},dt,true,false,false,gains);
    assert(o.fault && c.faultReason()==faultSensor && c.faultCount()==2);
  }
  Quaternion q=level;
  for (int i=0;i<20000;++i) assert(updateImu(q,{0,0,0},{0,0,1},dt,.04f));
  assert(near(q.w,1) && near(q.x,0)); // zero gradient is valid
  for (float rate : {-360.f,360.f}) {
    q=level;
    for (int i=0;i<10300;++i) assert(updateImu(q,{0,0,-rate},{0,0,1},dt,.04f));
    assert(near(wrapDegrees(Rotation(q).heading()-rate*10300*dt),0,.02f));
    assert(near(Rotation(q).bz.z,1));
  }
  assert(updateImu(q,{0,0,0},{2,0,1},dt,.04f)); // gate non-gravity acceleration
  assert(!updateImu(q,{0,0,0},{0,0,0},dt,.04f));
  q={0,0,0,0};
  assert(!updateImu(q,{0,0,0},{0,0,1},dt,.04f));
}
int main() {
  radioAndNormal();
  phaseAndDamping();
  modesAndSaturation();
  faultsAndFusion();
  std::puts("Cyclone radio, reference frames, modes, saturation, faults and fusion passed");
}
