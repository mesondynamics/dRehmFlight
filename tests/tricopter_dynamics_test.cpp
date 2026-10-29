#include "yaw_model_fixture.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/ServoTiltCalibration.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
  using namespace tricopter;
  auto c=syntheticYawConfig(); assert(c.ready);
  const auto &m=c.model;
  const float thrust[]={4,4,4};
  const Wrench vertical=wrench(m,thrust,90);
  assert(std::fabs(vertical.force.z-12)<1e-5f);
  assert(std::fabs(vertical.torque.z+.06f)<1e-5f);
  assert(wrench(m,thrust,88).torque.z>vertical.torque.z);
  for (unsigned r=0;r<3;++r) {
    float one[]={0,0,0}; one[r]=4;
    assert(wrench(m,one,88).torque.z>wrench(m,one,90).torque.z);
    for (int j=0;j<=100;++j) {
      const float u=j*.01f;
      assert(std::fabs(motorCommand(m.rotor[r],motorThrust(m.rotor[r],u))-u)<1e-6f);
    }
  }
  const auto balanced=allocateYaw(m,{12,0,0},0,90,40);
  assert(balanced.valid && std::fabs(balanced.yawResidual)<1e-5f);
  // Keeping roll/pitch at zero also cancels the horizontal reaction torques,
  // so the exact solution has slightly unequal motor thrusts.
  const float a=.25f*(balanced.thrust[0]+balanced.thrust[1]+balanced.thrust[2]);
  const float reaction=.015f*(-balanced.thrust[0]+balanced.thrust[1]-balanced.thrust[2]);
  const float expected=90-std::atan(-reaction/a)*degrees/.5f;
  assert(std::fabs(balanced.servoDeg-expected)<.002f);
  assert(std::fabs(balanced.achieved.force.z-12)<1e-5f);
  assert(std::fabs(balanced.achieved.torque.x)<1e-5f && std::fabs(balanced.achieved.torque.y)<1e-5f);
  // Unequal arms/CG and independent nonlinear tilt curves still preserve the
  // exact lift, roll and pitch equations, including horizontal reaction torque.
  c.model.rotor[0].position.x+=.03f;
  c.model.rotor[1].position.y-=.02f;
  c.model.rotor[2].position.z=.04f;
  for (unsigned j=0;j<9;++j) c.model.rotor[1].tiltDeg[j]*=.9f;
  for (int i=0;i<200;++i) {
    const float servo=55+i%70;
    const auto a=allocateAt(c.model,{8,.03f,-.02f},.05f,servo);
    assert(a.valid && a.liftScale==1);
    assert(std::fabs(a.achieved.force.z-8)<1e-4f);
    assert(std::fabs(a.achieved.torque.x-.03f)<1e-5f);
    assert(std::fabs(a.achieved.torque.y+.02f)<1e-5f);
  }
  const auto limited=allocateYaw(m,{12,0,0},100,90,.3f);
  assert(limited.valid && limited.limited && std::fabs(limited.servoDeg-90)<=.30001f);
  const auto highLift=allocateAt(m,{40,0,0},0,90);
  assert(highLift.valid && highLift.liftScale<1);
  for (float t : highLift.thrust) assert(t<=9.00001f);
  assert(!allocateYaw(m,{0,0,0},0,90,1).valid);
  auto narrow=m; narrow.tiltLimitDeg=.05f;
  // Even with less than one derivative-probe step of physical travel, the
  // solver must move inward from a limit instead of probing outside travel.
  const auto boundary=allocateYaw(narrow,{12,0,0},-.06f,90.1f,.1f);
  assert(boundary.valid && boundary.servoDeg<90.1f);
  auto invalid=m; invalid.rotor[0].tiltDeg[1]=invalid.rotor[0].tiltDeg[0];
  assert(!invalid.valid());
  invalid=m; invalid.izz=0; assert(!invalid.valid());
  invalid=m; invalid.rotor[0].thrustN[2]=0; assert(!invalid.valid());
  // Measured mechanical travel may exceed 45 degrees; flight never inherits it.
  auto measured=m; measured.tiltLimitDeg=30;
  for (auto &rotor : measured.rotor)
    for (unsigned i=0;i<9;++i) rotor.tiltDeg[i]=servo_calibration::defaultTilt(i);
  assert(measured.valid());
  assert(!allocateAt(measured,{12,0,0},0,50).valid);
  assert(!allocateAt(measured,{12,0,0},0,130).valid);
  for (float yaw : {-100.f,0.f,100.f}) {
    const auto limited=allocateYaw(measured,{12,0,0},yaw,90,80);
    assert(limited.valid);
    for (const auto &rotor : measured.rotor)
      assert(std::fabs(physicalTilt(rotor,limited.servoDeg))<=30.001f);
  }
  measured.rotor[0].tiltDeg[0]=85.1f; assert(!measured.valid());
  const Vec3 accel=angularAcceleration({1,2,3},{0,0,0},{1,2,3});
  assert(accel.x==1 && accel.y==1 && accel.z==1);
  std::puts("Measured-angle geometry, reaction signs, nonlinear inverse thrust, exact allocation and limits passed");
}
