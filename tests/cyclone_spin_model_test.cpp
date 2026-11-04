#include "yaw_model_fixture.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace tricopter;
static void laserChecks() {
  const LaserSyncConfig cfg;
  for (float direction : {-1.f,1.f}) {
    LaserSync sync;
    assert(!sync.update(false,0,45,100,cfg));
    assert(!sync.update(true,-direction*.1f,direction*45,1000,cfg));
    assert(sync.update(true,direction*.01f,direction*45,1500,cfg));
    assert(sync.pulseCount()==1 && sync.lastPulseUs()==1500);
    assert(sync.update(true,direction*.1f,direction*45,5000,cfg));
    assert(!sync.update(true,direction*.2f,direction*45,11500,cfg));
    assert(!sync.update(true,-direction*.01f,-direction*45,12000,cfg)); // no retrigger
    assert(!sync.update(true,-direction*179.9f,direction*45,200000,cfg));
    assert(!sync.update(true,direction*179.9f,direction*45,201000,cfg)); // wrap is not front
    assert(!sync.update(true,-direction*.1f,direction*45,250000,cfg));
    assert(sync.update(true,direction*.01f,direction*45,250500,cfg));
    assert(sync.pulseCount()==2);
    assert(!sync.update(false,0,45,251000,cfg)); // stop/fault cancels this tick
    assert(sync.pulseCount()==2);
  }
  LaserSync rollover;
  assert(!rollover.update(true,-.01f,45,UINT32_MAX-2000,cfg));
  assert(rollover.update(true,.01f,45,UINT32_MAX-1000,cfg));
  assert(rollover.update(true,.1f,45,3000,cfg));
  assert(!rollover.update(true,.2f,45,9000,cfg));
  assert(rollover.pulseCount()==1);
  LaserSync stopped;
  assert(!stopped.update(true,-.01f,0,1000,cfg));
  assert(!stopped.update(true,.01f,0,2000,cfg));
  assert(!stopped.update(true,std::numeric_limits<float>::quiet_NaN(),45,3000,cfg));
  assert(stopped.pulseCount()==0);
}

static void kinematicChecks() {
  SpinDynamicsModel spin;
  spin.viscousDampingNmS=.01f; spin.quadraticDampingNmS2=.002f;
  const float rate=.8f,accel=.4f,izz=.03f;
  assert(std::fabs(spin.feedforwardTorque(izz,rate,accel)-
      (izz*accel+.01f*rate+.002f*rate*rate))<1e-6f);
  assert(std::fabs(spin.feedforwardTorque(izz,-rate,-accel)+
                   spin.feedforwardTorque(izz,rate,accel))<1e-6f);
  for (float sign : {-1.f,1.f}) {
    Quaternion q{.96f,.15f,-.12f,.1f}; assert(q.normalize());
    const Vec3 omega{.3f,-.4f,sign*.8f};
    const HeadingDynamics k=headingDynamics(Rotation(q),omega);
    assert(k.valid);
    const float before=Rotation(q).headingRate(omega*degrees)*radians;
    const float eps=.0002f;
    assert(updateImu(q,{omega.x*degrees,-omega.y*degrees,-omega.z*degrees},
        Rotation(q).body({0,0,1}),eps,0));
    const float after=Rotation(q).headingRate(omega*degrees)*radians;
    assert(std::fabs((after-before)/eps-k.drift)<.002f);
    const Vec3 inertia{.03f,.04f,.0625f}, alpha{.2f,-.3f,.4f};
    const Vec3 gyro=gyroscopicTorque(inertia,omega);
    const Vec3 torque{inertia.x*alpha.x+gyro.x,inertia.y*alpha.y+gyro.y,
                      inertia.z*alpha.z+gyro.z};
    const Vec3 actual=angularAcceleration(inertia,omega,torque);
    assert(std::fabs(actual.x-alpha.x)<1e-5f && std::fabs(actual.y-alpha.y)<1e-5f &&
           std::fabs(actual.z-alpha.z)<1e-5f);
  }
  auto c=syntheticYawConfig();
  const auto limited=allocateYaw(c.model,{8,30,-25},.1f,90,1);
  assert(limited.valid && limited.limited && limited.attitudeScale<1);
  for (float thrust : limited.thrust) assert(thrust>=0);
}

static void unifiedPlant(float spinSpeed,float servoLag,float inertiaScale=1,float thrustScale=1,
                         bool combined=false,uint32_t expectedAbort=0) {
  // Independent Newton-Euler plant with actual calibrated linkage, motor lag,
  // 50 Hz servo sample/hold, 125-step motor quantization and unknown body drag.
  // No tether, rotating wings, fabricated IMU data capture or altitude control.
  FlightParameters parameters; parameters.begin();
  auto c=parameters.config(); c.ready=true; c.spinReady=true;
  c.spin.maxRateDps=spinSpeed; c.servoLag=combined ? .08f : servoLag;
  c.spin.maxAccelDps2=combined && !expectedAbort ? 10 : 30;
  c.model.ixx=.03f; c.model.iyy=.04f;
  auto physical=c.model;
  physical.ixx*=inertiaScale;physical.iyy*=inertiaScale;physical.izz*=inertiaScale;
  for(auto &r:physical.rotor) for(float &t:r.thrustN) t*=thrustScale;
  const Vec3 inertia=physical.inertia();
  const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
  CycloneControl reference; NormalYawControl control; LaserSync laser;
  Quaternion q{1,0,0,0}; Vec3 omega{0,0,0};
  const float throttle=motorCommand(c.model.rotor[0],c.model.massKg*9.81f/3);
  float thrust[3]{};
  for (unsigned i=0;i<3;++i) thrust[i]=physical.massKg*9.81f/3;
  float servo=servoForTilt(physical.rotor[2],0), servoHold=servo;
  float lastServo=servo, peakTilt=0, positiveError=0, negativeError=0;
  float finalRate=0, finalHeadingError=0;
  uint32_t abortReasons=0;float abortTime=0,peakRate=0;
  constexpr float dt=.0005f;
  NormalYawOutput out;
  ControlOutput refs{};
  const unsigned timeScale=combined ? 3 : 1; // Low-rate cases must complete full phase crossings.
  for (unsigned i=0;i<100000*timeScale;++i) {
    const float t=i*dt/timeScale;
    PilotInput pilot{throttle,0,0,0,0,false};
    if (t>=5 && t<17) { pilot.spin=1; pilot.spinRequested=true; }
    if (t>=17 && t<29) { pilot.spin=-1; pilot.spinRequested=true; }
    if (t>=9 && t<25) pilot.pitch=.2f; // world-referenced direction during spin
    if(combined && t>=9 && t<13) {pilot.roll=.2f;pilot.yaw=.3f;}
    if(combined && t>=21 && t<25) {pilot.roll=-.2f;pilot.yaw=-.3f;}
    if (t>=36 && t<38) pilot.yaw=.3f;
    // A legitimately excessive acceleration must cause a protected stop. Keep
    // S1 displaced to verify the abort latch prevents automatic spin re-entry.
    if(abortReasons) {pilot.roll=pilot.pitch=pilot.yaw=0;pilot.spin=1;pilot.spinRequested=true;}
    const Vec3 gyro=omega*degrees;
    refs=reference.step(pilot,q,gyro,dt,true,false,true,gains,&c.spin,&c.reference,&c.safety);
    assert(!refs.fault);
    if(reference.supervisor().inhibited() && !expectedAbort)
      std::fprintf(stderr,"Unexpected spin abort: Jx%.2f Tx%.2f at %.3fs, reasons=%u, mode=%u rate=%.3f target=%.3f tilt=%.3f\n",
          inertiaScale,thrustScale,i*dt,reference.supervisor().reasons(),unsigned(refs.mode),
          refs.yawRate,refs.yawTarget,refs.tiltErrorDeg);
    if(!expectedAbort) assert(!reference.supervisor().inhibited());
    if(reference.supervisor().inhibited()) {
      if(!abortReasons) abortTime=i*dt;
      abortReasons=reference.supervisor().reasons();
      assert(abortReasons&expectedAbort);
      assert(refs.mode!=FlightMode::Spin);
    }
    peakRate=std::fmax(peakRate,std::fabs(refs.yawRate));
    assert(refs.desiredTiltRate<c.safety.tiltCommandDps+.03f);
    YawControlRequest request;
    request.rateMode=refs.mode!=FlightMode::Normal;
    request.rateTargetDps=refs.yawTarget; request.attitude=q; request.gyroDps=gyro;
    out=control.step(c,refs.motors,refs.heading,refs.yawRate,pilot.yaw,dt,true,request);
    reference.applyAllocationFeedback(out.liftScale,out.attitudeScale,out.limited,out.rateTarget);
    assert(out.active && !out.fault);
    assert(std::fabs(out.servoDeg-lastServo)<=c.servoSlew*.0056f+.0001f);
    lastServo=out.servoDeg;
    for (float tilt : out.tilt) assert(std::fabs(tilt)<=c.model.tiltLimitDeg+.1f);
    if (i%40==0) servoHold=quantizedServoLogical(out.servoDeg);
    servo+=dt/(servoLag+dt)*(servoHold-servo);
    const float command[]={out.motors.m2,out.motors.m4,out.motors.m3};
    for (unsigned r=0;r<3;++r) {
      assert(command[r]>=0 && command[r]<=1);
      const float target=motorThrust(physical.rotor[r],std::floor(command[r]*125)/125);
      thrust[r]+=dt/(.04f+dt)*(target-thrust[r]);
    }
    Wrench w=wrench(physical,thrust,servo);
    w.torque.z-=.003f*omega.z; // true damping not given to controller
    if (t>=12 && t<12.25f) w.torque.x+=.004f;
    if (t>=22 && t<22.25f) w.torque.y-=.004f;
    const Vec3 alpha=angularAcceleration(inertia,omega,w.torque);
    omega=omega+alpha*dt;
    assert(updateImu(q,{omega.x*degrees,-omega.y*degrees,-omega.z*degrees},
        Rotation(q).body({0,0,1}),dt,0));
    const float tilt=std::acos(clamp(Rotation(q).bz.z,-1,1))*degrees;
    peakTilt=std::fmax(peakTilt,tilt);
    assert(tilt<12);
    if (t>=15 && t<17) positiveError=std::fmax(positiveError,std::fabs(refs.yawRate-spinSpeed));
    if (t>=27 && t<29) negativeError=std::fmax(negativeError,std::fabs(refs.yawRate+spinSpeed));
    laser.update(refs.mode==FlightMode::Spin,refs.phase,
        refs.yawRate-refs.referenceRate,i*500,c.laser);
    if (t>48) {
      finalRate=std::fmax(finalRate,std::fabs(refs.yawRate));
      finalHeadingError=std::fmax(finalHeadingError,std::fabs(wrapDegrees(out.headingTarget-refs.heading)));
    }
  }
  std::printf("Unified synthetic plant +/-%.0f deg/s accel%.0f, %.0f ms servo, Jx%.1f Tx%.1f combined=%d: tilt %.2f deg, rate errors %.2f/%.2f, final hold %.2f deg / %.2f deg/s, laser %u, abort %u at %.3fs, peak rate %.2f\n",
      spinSpeed,c.spin.maxAccelDps2,servoLag*1000,inertiaScale,thrustScale,combined,peakTilt,positiveError,negativeError,finalHeadingError,finalRate,laser.pulseCount(),abortReasons,abortTime,peakRate);
  assert(finalRate<5 && finalHeadingError<5 && refs.mode==FlightMode::Normal);
  if(expectedAbort) {
    assert(abortReasons&expectedAbort);
    assert(!(abortReasons&spinBrakingTimeout));
  } else {
    assert(positiveError<8 && negativeError<8 && laser.pulseCount()>0);
  }
}
static void physicalSaturationChecks() {
  CycloneControl limited,free;
  SpinDynamicsModel spin;
  AttitudeReferenceConfig cfg;
  const ControlGains integralOnly{{0,.003f,0},{0,.003f,0},defaultYawGains()};
  const PilotInput pilot{.5f,0,.3f,0,0,false};
  ControlOutput a{},b{};
  const ControlGains noIntegral{};
  // Settle the bounded thrust-direction reference before comparing equal
  // integration intervals; the actuator saturation is the subject here.
  for(unsigned i=0;i<1000;++i) {
    limited.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,noIntegral,&spin,&cfg);
    free.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,noIntegral,&spin,&cfg);
  }
  for (unsigned i=0;i<2000;++i) {
    a=limited.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,integralOnly,&spin,&cfg);
    b=free.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,integralOnly,&spin,&cfg);
    limited.applyAllocationLimits(true);
  }
  assert(!a.fault && !b.fault && a.mixScale==1 && b.mixScale==1);
  assert(std::fabs(a.motors.m3-.5f)<.0001f && std::fabs(b.motors.m3-.5f)>.01f);
  // When physical authority returns, integration resumes rather than latching.
  for (unsigned i=0;i<2000;++i) {
    a=limited.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,integralOnly,&spin,&cfg);
    limited.applyAllocationLimits(false);
  }
  assert(std::fabs(a.motors.m3-b.motors.m3)<.0001f);
}
static void spinEntryChecks() {
  CycloneControl reference;
  SpinDynamicsModel spin; spin.maxRateDps=45; spin.maxAccelDps2=30;
  AttitudeReferenceConfig cfg;
  const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
  PilotInput pilot{.5f,0,0,1,0,false};
  for (unsigned i=0;i<4000;++i)
    reference.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,gains,&spin,&cfg);
  pilot.yaw=0; pilot.spin=.1f; pilot.spinRequested=true;
  const auto entry=reference.step(pilot,{1,0,0,0},{0,0,0},.0005f,true,false,true,gains,&spin,&cfg);
  assert(entry.mode==FlightMode::Spin && entry.yawTarget>0 && entry.yawTarget<.02f);
}
int main() {
  laserChecks(); kinematicChecks(); physicalSaturationChecks(); spinEntryChecks();
  for (float speed : {45.f,90.f}) for (float lag : {.04f,.08f,.12f}) unifiedPlant(speed,lag);
  unifiedPlant(15,.16f,1,1,true);
  unifiedPlant(15,.16f,.5f,1,true);
  // Half the assumed inertia with 30 deg/s^2 feedforward really overspeeds;
  // require the supervisor to brake and hold, rather than suppressing the guard.
  unifiedPlant(15,.16f,.5f,1,true,spinOverspeed);
  unifiedPlant(15,.16f,2,1,true);
  for(float scale:{.7f,1.3f}) unifiedPlant(15,.16f,1,scale,true);
  std::puts("Unified model allocation, mode handoff, Euler coupling, phase marker and clock wrap passed");
}
