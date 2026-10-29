#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace tricopter;
constexpr float dt=.0005f;

static void groundVibrationAndRounding() {
  YawServoControl control;
  int servo=90, transitions=0;
  float maxOld=0, maxNew=0, lastMeasurement=0, previous=0;
  for (int i=0;i<20000;++i) {
    const float t=i*dt;
    const float throttle=clamp(t*.1f,0,.3f); // slow throttle increase, fixed yaw
    const float gyro=.2f+30*std::sin(6.2831853f*120*t)+8*std::sin(6.2831853f*35*t);
    const auto out=control.step(0,gyro,throttle,dt,defaultYawGains());
    assert(std::fabs(out.applied-previous)*180 <= yawServoSlewDegPerSec*dt+.00001f);
    previous=out.applied;
    // Previous stock gains and unfiltered derivative, before this fix.
    const float old=clamp(-.003f*gyro-.0000015f*(gyro-lastMeasurement)/dt,
                           -yawOutputLimit,yawOutputLimit);
    lastMeasurement=gyro;
    if (t>3) {
      maxOld=std::fmax(maxOld,std::fabs(old)*180);
      maxNew=std::fmax(maxNew,std::fabs(out.applied)*180);
      if (i%40==0) {
        const int next=servoCommandDeg(yawServoDeg(out.applied),servo);
        if (next!=servo) ++transitions;
        servo=next;
      }
    }
  }
  std::printf("Fixed-frame synthetic vibration: old excursion %.2f deg, filtered %.2f deg, PWM changes %d\n",
              maxOld,maxNew,transitions);
  assert(maxOld>20 && maxNew<.5f && transitions==0);
  assert(servoCommandDeg(89.999f,90)==90); // old int conversion produced 89
  assert(servoCommandDeg(90.001f,90)==90);
  assert(servoCommandDeg(90.55f,90)==90);
  assert(servoCommandDeg(90.66f,90)==91);
  assert(servoCommandDeg(90.45f,91)==91);
  assert(servoCommandDeg(90.34f,91)==90);
  assert(servoCommandDeg(89.34f,90)==89);
  assert(servoCommandDeg(90,50)==90); // cut must still center immediately
}

static void authorityLimitsAndRealYaw() {
  YawServoControl low, full;
  YawServoOutput lo{}, hi{};
  for (int i=0;i<4000;++i) {
    lo=low.step(0,20,.08f,dt,defaultYawGains());
    hi=full.step(0,20,.5f,dt,defaultYawGains());
  }
  assert(lo.authority==yawLowThrottleGain && hi.authority==1);
  assert(lo.integral==0); // no trim accumulation at low ground throttle
  assert(std::fabs(lo.applied)<std::fabs(hi.applied)*.25f);
  assert(hi.applied<0 && yawServoDeg(hi.applied)>servoCenterDeg); // oppose real CCW spin
  YawServoControl saturating;
  YawServoOutput out{};
  float previous=0;
  bool sawSlew=false;
  for (int i=0;i<20000;++i) {
    out=saturating.step(1000,0,.5f,dt,defaultYawGains());
    assert(std::fabs(out.applied-previous)*180<=yawServoSlewDegPerSec*dt+.00001f);
    previous=out.applied;
    assert(std::fabs(out.applied)<=yawOutputLimit+.000001f);
    sawSlew |= out.slewLimited;
  }
  assert(sawSlew && out.integral==0);
  for (int i=0;i<4000;++i) out=saturating.step(0,0,.5f,dt,defaultYawGains());
  assert(std::fabs(out.applied)*180<.01f);
  saturating.reset();
  out=saturating.step(0,.2f,.5f,dt,defaultYawGains());
  assert(out.applied==0 && out.integral==0); // small static gyro bias does not twitch servo
}

static void repeatedThrottleDoesNotCapMotors() {
  CycloneControl control;
  const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
  for (int cycle=0;cycle<3;++cycle) {
    for (unsigned throttle : {1000u,1060u,1150u,1300u,1500u,1800u,2000u}) {
      for (int i=0;i<100;++i) {
        const auto pilot=pilotInput(throttle,1500,1500,1500,1500,control.spinning());
        const auto out=control.step(pilot,{1,0,0,0},{0,0,20},dt,true,false,true,gains);
        assert(!out.fault);
        assert(std::fabs(out.motors.m2+out.motors.m3+out.motors.m4-3*pilot.throttle)<1e-5f);
        if (throttle==2000) assert(out.motors.m2==1 && out.motors.m3==1 && out.motors.m4==1);
      }
    }
  }
}
int main() {
  groundVibrationAndRounding();
  authorityLimitsAndRealYaw();
  repeatedThrottleDoesNotCapMotors();
  std::puts("Yaw noise, low-throttle authority, slew, PWM rounding, anti-windup and repeated throttle checks passed");
}
