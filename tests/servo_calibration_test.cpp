#include "../Versions/dRehmFlight_Teensy_BETA_1.3/TricopterConfig.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/ServoTiltCalibration.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/TricopterDynamics.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
  using namespace tricopter;
  // Every gate matters: an arm request denied for another reason must not
  // accidentally hand CH6's airborne spin command directly to the servo.
  for (bool ready : {false,true}) for (bool armed : {false,true})
    for (bool fault : {false,true}) for (unsigned cut : {1000u,1499u,1500u,2000u})
      assert(benchServoActive(ready,armed,cut,fault)==
             (ready && !armed && !fault && cut>=1500));
  assert(!fullRangeBenchTest);
  const int channels[]={1000,1125,1250,1375,1500,1625,1750,1875,2000};
  int last=0;
  for (int i=0;i<9;++i) {
    const float angle=benchServoDeg(channels[i]);
    assert(angle==50+10*i);
    const int applied=servoCommandDeg(angle,last);
    assert(applied==50+10*i);
    assert(servoPulseEstimateUs(applied)>=1063 && servoPulseEstimateUs(applied)<=1944);
    last=applied;
  }
  assert(servoPulseEstimateUs(90)==1504);
  assert(servoPulseCommandUs(90)==1504 && servoPulseCommandUs(50)==1063 && servoPulseCommandUs(130)==1945);
  assert(servoPulseCommandUs(90.1f)!=servoPulseCommandUs(90));
  assert(servoDuty16(90)<65536 && servoDuty16(50)<servoDuty16(130));
  assert(benchServoDeg(800)==50 && benchServoDeg(2200)==130);
  // User-supplied left-positive M3 measurements are explicitly converted to
  // FLU right-positive. The unmeasured software midpoint is not forced to zero.
  const int measuredPwm[]={1063,1187,1327,1605,1687,1820,1945};
  const float measuredTilt[]={55.9f,39.9f,20.5f,-16,-28.2f,-49.6f,-63.6f};
  for (unsigned i=0;i<7;++i)
    assert(std::fabs(servo_calibration::tiltAtPwm(measuredPwm[i])-measuredTilt[i])<1e-4f);
  assert(std::fabs(servo_calibration::tiltAtPwm(1483.1369863f))<1e-4f);
  assert(std::fabs(servo_calibration::tiltAtPwm(1504)+2.7392086f)<1e-4f);
  assert(servo_calibration::tiltAtPwm(500)==measuredTilt[0]);
  assert(servo_calibration::tiltAtPwm(2500)==measuredTilt[6]);
  RotorModel rotor{};
  for (unsigned i=0;i<9;++i) rotor.tiltDeg[i]=servo_calibration::defaultTilt(i);
  assert(tiltMappingValid(rotor));
  // Quantify the extra approximation from the fixed nine-knot runtime schema.
  // Full-range error is bounded; no measured samples are relabelled as fitted.
  float worst=0,normalWorst=0;
  for (int pwm=1063;pwm<=1945;++pwm) {
    const float truth=servo_calibration::tiltAtPwm(pwm);
    const float logical=(pwm-servoMinUs)*180.0f/(servoEffectiveMaxUs-servoMinUs);
    const float error=std::fabs(physicalTilt(rotor,logical)-truth);
    worst=std::fmax(worst,error);
    if (std::fabs(truth)<=30) normalWorst=std::fmax(normalWorst,error);
  }
  assert(worst<.8f && normalWorst<.3f);
  std::printf("Measured seven-point calibration: nine-knot max error %.4f deg, +/-30 deg region %.4f deg\n",
      worst,normalWorst);
  // Stopped controller outputs stay zero even when the spin knob is displaced.
  const ControlGains gains{{.01f,0,0},{.01f,0,0},defaultYawGains()};
  CycloneControl controller;
  for (int ch6 : channels) {
    const auto pilot=pilotInput(1000,1500,1500,1500,ch6,false);
    const auto out=controller.step(pilot,{1,0,0,0},{0,0,0},.0005f,false,true,true,gains);
    assert(out.motors.m2==0 && out.motors.m4==0 && out.motors.m3==0);
    assert(!canArm(true,1000,2000,ch6));
  }
  std::puts("SA-cut CH6 calibration gates, nine-point mapping, pulse estimates and motor stop passed");
}
