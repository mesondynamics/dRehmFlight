#include "../Versions/dRehmFlight_Teensy_BETA_1.3/HoverMeasurement.h"
#include <cassert>
#include <cmath>

using tricopter::HoverMeasurement;
using tricopter::Quaternion;
using tricopter::Vec3;
constexpr float dt = .0005f;
const Quaternion level{1,0,0,0};

static void checkProjection() {
  // Independent rotations: +30 deg pitch gives gravity a negative fusion X;
  // +30 deg roll gives gravity a positive fusion Y. Sensor X has opposite sign.
  const float c = std::cos(15*tricopter::radians), s = std::sin(15*tricopter::radians);
  const float cos30 = std::sqrt(.75f);
  for (int axis = 0; axis < 2; ++axis) {
    const Quaternion q = axis == 0 ? Quaternion{c,0,s,0} : Quaternion{c,s,0,0};
    const Vec3 accel = axis == 0 ? Vec3{.5f,0,cos30} : Vec3{0,.5f,cos30};
    HoverMeasurement m;
    m.update(true,q,accel,700,dt);
    assert(std::fabs(m.upG()-1) < 1e-6f && m.collectiveCode() == 700);
  }
  // Body pitched 30 deg and yawed 90 deg, world specific force includes
  // horizontal acceleration along the pitched body's horizontal heading.
  const float h = std::sqrt(.5f);
  const Quaternion turned{h*c,-h*s,h*s,h*c};
  HoverMeasurement m;
  m.update(true,turned,{.5f*1.2f-cos30*.3f,0,cos30*1.2f+.5f*.3f},680,dt);
  assert(std::fabs(m.upG()-1.2f) < 1e-6f);
  m.update(false,turned,{0,0,1},680,dt);
  assert(m.upG() == 0 && m.collectiveCode() == 0);
  m.update(true,level,{0,0,.95f},600,dt);
  assert(m.upG() == .95f && m.collectiveCode() == 600);
}

static float sinusoidGain(float hz) {
  HoverMeasurement m;
  double power = 0;
  for (int i=0; i<4000; ++i) {
    const float x = std::sin(6.2831853f*hz*dt*i);
    m.update(true,level,{0,0,1+.2f*x},700+100*x,dt);
    // Same amplitude and delay after rescaling both channels.
    assert(std::fabs((m.upG()-1)*500 - (m.collectiveCode()-700)) < .01f);
    if (i >= 2000) power += std::pow((m.upG()-1)/.2f,2);
  }
  return std::sqrt(2*power/2000);
}

int main() {
  checkProjection();
  assert(sinusoidGain(1) > .98f);
  assert(sinusoidGain(50) < .014f);
  assert(sinusoidGain(80) < .004f);
}
