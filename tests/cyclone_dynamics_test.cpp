#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace tricopter;

// Regression plant, NOT an identified airframe model. Symmetric inertia, motor
// lag, 50 Hz servo updates + first-order servo lag, drag and constant yaw torque.
// It exercises the actual controller and quaternion kinematics in both directions.
struct Plant {
  Quaternion q{1,0,0,0};
  Vec3 omega{0,0,0}; // FRD deg/s
  float rollMotor=0, pitchMotor=0, servo=0, servoCommand=0;
  unsigned frames=0;
  int servoLogical=90;
  float inertiaRatio=1; // Iz/Ix = Iz/Iy; 1=sphere, 2=flat symmetric frame
  void advance(ControlOutput out, float dt, float servoTau) {
    rollMotor += dt/.02f*(out.roll*out.mixScale-rollMotor);
    pitchMotor += dt/.02f*(out.pitch*out.mixScale-pitchMotor);
    if (frames++ % 40 == 0) { // 50 Hz with actual logical-degree quantization
      servoLogical=servoCommandDeg(out.servoDeg,servoLogical);
      servoCommand=(servoLogical-servoCenterDeg)/(yawServoDirection*180.0f);
    }
    servo += dt/servoTau*(servoCommand-servo);
    const Vec3 prev=omega;
    omega.x += dt*(10000*rollMotor-1.0f*prev.x+(1-inertiaRatio)*prev.y*prev.z*radians);
    omega.y += dt*(-10000*pitchMotor-1.0f*prev.y+(inertiaRatio-1)*prev.x*prev.z*radians);
    omega.z += dt*(-2000*servo-.12f*omega.z-35); // net prop reaction torque
    const Vec3 down=Rotation(q).body({0,0,1});
    assert(updateImu(q,omega,down,dt,0));
  }
};

int main() {
  const float dt=.0005f;
  const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},defaultYawGains()};
  for (float inertiaRatio : {1.f,2.f})
  for (float speed : {90.f,180.f,360.f})
  for (float servoTau : {.04f,.08f,.12f}) {
    CycloneControl controller;
    Plant plant;
    plant.inertiaRatio=inertiaRatio;
    ControlOutput out{};
    float worstTilt=0, normalResidual=0, positiveError=0, negativeError=0, exitResidual=0;
    for (int i=0;i<160000;++i) { // 80 seconds; slew, settle, reverse and brake
      const float t=i*dt;
      unsigned spin=1500;
      if (t>=15 && t<40) spin=1000; // positive spin
      if (t>=40 && t<65) spin=2000; // negative spin
      auto in=pilotInput(1500,1500,1500,1500,spin,controller.spinning());
      in.spin *= speed/spinMaxDps;
      const Vec3 gyro{plant.omega.x,-plant.omega.y,-plant.omega.z};
      out=controller.step(in,plant.q,gyro,dt,true,false,true,gains);
      assert(!out.fault);
      // Apply physical disturbances during ordinary and spinning flight.
      if (i==2000 || i==50000 || i==100000) { plant.omega.x+=20; plant.omega.y-=15; }
      plant.advance(out,dt,servoTau);
      const float tilt=std::acos(clamp(Rotation(plant.q).bz.z,-1,1))*degrees;
      worstTilt=std::fmax(worstTilt,tilt);
      assert(tilt<5); // never loses low-speed tilt stability in this plant
      if (t>10 && t<15) normalResidual=std::fmax(normalResidual,std::fabs(out.yawRate));
      if (t>35 && t<40) positiveError=std::fmax(positiveError,std::fabs(out.yawRate-speed));
      if (t>60 && t<65) negativeError=std::fmax(negativeError,std::fabs(out.yawRate+speed));
      if (t>75) exitResidual=std::fmax(exitResidual,std::fabs(out.yawRate));
    }
    std::printf("inertia %.0f, +/-%.0f deg/s, lag %.0f ms: tilt max %.2f deg, yaw residual %.2f, spin errors %.2f/%.2f, exit %.2f deg/s\n",
                inertiaRatio,speed,servoTau*1000,worstTilt,normalResidual,positiveError,negativeError,exitResidual);
    assert(normalResidual<5 && positiveError<std::fmax(10.f,speed*.1f) && negativeError<std::fmax(10.f,speed*.1f) && exitResidual<5);
    assert(out.mode==FlightMode::Normal);
    assert(out.servoDeg != servoCenterDeg); // active trim cancels the net motor torque
  }
}
