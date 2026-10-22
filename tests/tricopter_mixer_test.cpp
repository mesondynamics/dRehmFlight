#include "../Versions/dRehmFlight_Teensy_BETA_1.3/TricopterConfig.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

static bool close(float a, float b) { return std::fabs(a - b) < 0.00001f; }

int main() {
  const auto level = tricopter::mix(0.4f, 0, 0);
  assert(close(level.m2, .4f) && close(level.m4, .4f) && close(level.m3, .4f));
  const auto rightRoll = tricopter::mix(.4f, .1f, 0);
  assert(rightRoll.m2 > .4f && rightRoll.m4 < .4f && close(rightRoll.m3, .4f));
  const auto noseDown = tricopter::mix(.4f, 0, .1f);
  assert(noseDown.m2 < .4f && noseDown.m4 < .4f && noseDown.m3 > .4f);
  // Unsaturated mixer preserves collective and produces independent, equal-gain roll/pitch moments.
  for (float r : {-0.1f, 0.0f, 0.1f}) for (float p : {-0.1f, 0.0f, 0.1f}) {
    const auto m = tricopter::mix(.5f, r, p);
    assert(close(m.m2 + m.m4 + m.m3, 1.5f));
    assert(close(tricopter::rollLever * (m.m2 - m.m4), 1.5f * r));
    assert(close(m.m3 - .5f * (m.m2 + m.m4), 1.5f * p));
  }
  // 270-degree shaft: logical 50..130 is nominally +/-60 degrees around neutral.
  assert(!tricopter::fullRangeBenchTest);
  assert(close(tricopter::benchServoDeg(500), 50));
  assert(close(tricopter::benchServoDeg(1000), 50));
  assert(close(tricopter::benchServoDeg(1500), 90));
  assert(close(tricopter::benchServoDeg(2000), 130));
  assert(close(tricopter::benchServoDeg(2500), 130));
  assert(close(tricopter::yawServoDeg(0), 90));
  assert(close(tricopter::yawServoDeg(10), 50));
  assert(close(tricopter::yawServoDeg(-10), 130));
  assert(close(tricopter::yawServoDeg(.1f), 72)); // Measured linkage: positive yaw decreases PWM.
  assert(close(tricopter::manualYawServoDeg(1500), 90));
  assert(close(tricopter::manualYawServoDeg(2000), 50)); // Left command -> M3 right.
  assert(close(tricopter::manualYawServoDeg(1000), 130)); // Right command -> M3 left.
  assert(close(tricopter::manualYawServoDeg(2500), 50));
  assert(close(tricopter::manualYawServoDeg(500), 130));
  // In manual mode even large opposite PID requests cannot move a centered CH4 servo.
  assert(close(tricopter::armedYawServoDeg(10, 1500), tricopter::yawFeedbackEnabled ? 50 : 90));
  assert(close(tricopter::armedYawServoDeg(-10, 1500), tricopter::yawFeedbackEnabled ? 130 : 90));
  assert(close(tricopter::armedYawServoDeg(0, 2000), tricopter::yawFeedbackEnabled ? 90 : 50));
  assert(tricopter::canArm(true, 1000, 1000, 1500));
  assert(!tricopter::canArm(false, 1000, 1000, 1500));
  assert(!tricopter::canArm(true, 1100, 1000, 1500));
  assert(!tricopter::canArm(true, 1000, 2000, 1500));
  assert(tricopter::canArm(true, 1000, 1000, 2000)); // CH6 only gates arming in full-range test mode.
  assert(tricopter::yawFromAetrUs(1000) == 2000); // left stick -> positive/CCW yaw
  assert(tricopter::yawFromAetrUs(1500) == 1500);
  assert(tricopter::yawFromAetrUs(2000) == 1000); // right stick -> negative/CW yaw
  std::puts("Tricopter geometry, servo limits, and arming checks passed");
}
