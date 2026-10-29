#include "yaw_model_fixture.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <cstdlib>

int main() {
  std::setvbuf(stdout,nullptr,_IONBF,0);
  using namespace tricopter;
  for (float lag : {.04f,.08f,.12f}) for (float direction : {-1.f,1.f}) {
    auto c=syntheticYawConfig(); c.servoLag=lag;
    NormalYawControl control;
    float heading=178,rate=0,servo=90,servoTarget=90;
    float thrust[]={0,0,0};
    const float nominal=motorCommand(c.model.rotor[0],c.model.massKg*9.81f/3);
    NormalYawOutput out;
    float beforeTurn=0,afterTurn=0,maxRate=0;
    constexpr float dt=.0005f;
    for (unsigned tick=0;tick<36000;++tick) {
      const float t=tick*dt;
      const float stick=t>=3 && t<5 ? direction*.5f : 0;
      out=control.step(c,{nominal,nominal,nominal},heading,rate,stick,dt,true);
      assert(!out.fault && out.servoDeg>=50 && out.servoDeg<=130);
      assert(std::fabs(out.integral)<=c.integralLimit+.0001f);
      if (tick%40==0) servoTarget=quantizedServoLogical(out.servoDeg);
      servo+=dt/(lag+dt)*(servoTarget-servo);
      const float commands[]={out.motors.m2,out.motors.m4,out.motors.m3};
      for (unsigned i=0;i<3;++i) {
        assert(commands[i]>=0 && commands[i]<=1);
        const float quantized=std::floor(commands[i]*125)/125;
        thrust[i]+=dt/(.04f+dt)*(motorThrust(c.model.rotor[i],quantized)-thrust[i]);
      }
      const float disturbance=t>=8 && t<9 ? .025f : 0;
      const Wrench w=wrench(c.model,thrust,servo);
      rate+=(w.torque.z+disturbance-.003f*rate*radians)/c.model.izz*degrees*dt;
      heading=wrapDegrees(heading+rate*dt);
      if (std::getenv("DREHM_YAW_TEST_TRACE") && tick%3000==0)
        std::printf("t %.1f heading %.2f target %.2f rate %.2f servo %.2f I %.3f req %.4f est %.4f residual %.4f\n",
          t,heading,out.headingTarget,rate,out.servoDeg,out.integral,out.torqueRequested,out.torqueEstimated,out.residual);
      maxRate=std::fmax(maxRate,std::fabs(rate));
      if (tick==5999) beforeTurn=heading;
      if (tick==9999) afterTurn=heading;
    }
    const float finalError=std::fabs(wrapDegrees(out.headingTarget-heading));
    std::printf("Synthetic yaw lag %.0f ms: signed turn %.2f deg, final hold error %.2f deg, rate %.2f deg/s\n",
        lag*1000,wrapDegrees(afterTurn-beforeTurn),finalError,rate);
    assert(direction*wrapDegrees(afterTurn-beforeTurn)>15);
    assert(finalError<5 && std::fabs(rate)<5 && maxRate<80);
    const auto stopped=control.step(c,{0,0,0},heading,rate,0,dt,false);
    assert(stopped.servoDeg==90 && stopped.motors.m2==0);
  }
  auto c=syntheticYawConfig(); c.ready=false;
  NormalYawControl control;
  assert(control.step(c,{.4f,.4f,.4f},0,0,0,.0005f,true).fault);
  c=syntheticYawConfig();
  assert(control.step(c,{.4f,.4f,.4f},0,0,0,.02f,true).fault);
  // A collective/attitude step between 200 Hz yaw solves must reach the motors
  // on the next 500 us tick, with the requested lift/roll/pitch still satisfied.
  control.reset();
  const auto first=control.step(c,{.4f,.4f,.4f},0,0,0,.0005f,true);
  const MotorMix changed{.55f,.5f,.45f};
  const auto next=control.step(c,changed,0,0,0,.0005f,true);
  assert(!next.fault && first.servoDeg==next.servoDeg);
  assert(next.motors.m2>first.motors.m2+.05f);
  const float nominal[]={motorThrust(c.model.rotor[0],changed.m2),
    motorThrust(c.model.rotor[1],changed.m4),motorThrust(c.model.rotor[2],changed.m3)};
  const Wrench achieved=wrench(c.model,next.thrust,next.servoDeg);
  Vec3 expected{};
  for (unsigned i=0;i<3;++i) {
    expected.x+=nominal[i]; expected.y+=c.model.rotor[i].position.y*nominal[i];
    expected.z-=c.model.rotor[i].position.x*nominal[i];
  }
  assert(std::fabs(achieved.force.z-expected.x)<1e-4f);
  assert(std::fabs(achieved.torque.x-expected.y)<1e-5f);
  assert(std::fabs(achieved.torque.y-expected.z)<1e-5f);
  std::puts("Normal heading hold, CCW stick, wrap, servo/motor lag, quantization and disturbance recovery passed");
}
