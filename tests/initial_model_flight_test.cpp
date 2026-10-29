#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>

// Model uncertainty regression, NOT evidence of this aircraft's stability.
// Exercise the flight defaults, measured nonlinear linkage and actual model
// allocator, against different thrust, inertia, reaction torque and servo lag.
int main() {
  using namespace tricopter;
  FlightParameters parameters; parameters.begin();
  auto c=parameters.config();
  assert(c.model.valid(false) && !c.ready && !c.model.calibrated);
  c.ready=true; c.initialModelEnabled=true;
  NormalYawControl idle;
  const auto stopped=idle.step(c,{0,0,0},25,0,0,.0005f,false);
  assert(!stopped.active && !stopped.fault);
  assert(std::fabs(physicalTilt(c.model.rotor[2],stopped.servoDeg))<.001f);
  assert(servoPulseCommandUs(stopped.servoDeg)==1483);
  float worstError=0,peakRate=0;
  unsigned cases=0;
  for (float inertiaScale : {.25f,1.f,2.f})
  for (float thrustScale : {.5f,1.f,2.f})
  for (float lag : {.04f,.08f,.12f}) {
    auto plant=c.model; plant.izz*=inertiaScale;
    for (auto &rotor : plant.rotor) {
      rotor.kappa*=1.5f;
      for (float &thrust : rotor.thrustN) thrust*=thrustScale;
    }
    NormalYawControl controller;
    float heading=178,rate=0,servo=stopped.servoDeg,servoTarget=servo;
    const float hover=c.model.massKg*9.81f/3;
    const float u=motorCommand(plant.rotor[0],hover);
    float thrust[]={hover,hover,hover};
    NormalYawOutput out;
    constexpr float dt=.0005f;
    for (unsigned tick=0;tick<48000;++tick) {
      const float t=tick*dt;
      const float stick=t>5 && t<7 ? .35f : (t>11 && t<13 ? -.35f : 0);
      out=controller.step(c,{u,u,u},heading,rate,stick,dt,true);
      assert(!out.fault && out.active);
      for (const auto &rotor : c.model.rotor)
        assert(std::fabs(physicalTilt(rotor,out.servoDeg))<=15.001f);
      if (tick%40==0) servoTarget=quantizedServoLogical(out.servoDeg);
      servo+=dt/(lag+dt)*(servoTarget-servo);
      const float commands[]={out.motors.m2,out.motors.m4,out.motors.m3};
      for (unsigned i=0;i<3;++i) {
        assert(commands[i]>=0 && commands[i]<=1);
        const float target=motorThrust(plant.rotor[i],std::floor(commands[i]*125)/125);
        thrust[i]+=dt/(.04f+dt)*(target-thrust[i]);
      }
      const auto w=wrench(plant,thrust,servo);
      const float disturbance=t>16 && t<16.5f ? .015f : 0;
      rate+=(w.torque.z+disturbance-.002f*rate*radians)/plant.izz*degrees*dt;
      heading=wrapDegrees(heading+rate*dt);
      peakRate=std::fmax(peakRate,std::fabs(rate));
    }
    const float error=std::fabs(wrapDegrees(out.headingTarget-heading));
    worstError=std::fmax(worstError,error);
    std::printf("Initial-model plant I x%.2f, thrust x%.1f, lag %.0f ms: final error %.2f deg, rate %.2f deg/s\n",
        inertiaScale,thrustScale,lag*1000,error,rate);
    assert(error<8 && std::fabs(rate)<8);
    ++cases;
  }
  assert(peakRate<70);
  std::printf("%u provisional-model regressions: worst final error %.2f deg, peak %.2f deg/s\n",
      cases,worstError,peakRate);
}
