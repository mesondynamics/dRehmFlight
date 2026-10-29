#pragma once
#include "TricopterConfig.h"

namespace tricopter {
namespace servo_calibration {
struct Point { float pwmUs, tiltDeg; };

// M3 measured 2026-10-29. User's left-positive readings are negated into
// FLU/CCW tangential tilt (M3 right-positive). M2/M4 share this mapping by
// the user's explicit confirmation of the common geared linkage.
constexpr Point measured[] = {
  {1063,55.9f}, {1187,39.9f}, {1327,20.5f}, {1605,-16.0f},
  {1687,-28.2f}, {1820,-49.6f}, {1945,-63.6f}
};
constexpr unsigned pointCount=sizeof(measured)/sizeof(measured[0]);

// Preserve the measured knots as the source of truth. Clamp at measured
// endpoints; do not extrapolate or force software center to physical zero.
inline float tiltAtPwm(float pwmUs) {
  pwmUs=clamp(pwmUs,measured[0].pwmUs,measured[pointCount-1].pwmUs);
  for (unsigned i=0;i+1<pointCount;++i) {
    if (pwmUs<=measured[i+1].pwmUs) {
      const float fraction=(pwmUs-measured[i].pwmUs)/
          (measured[i+1].pwmUs-measured[i].pwmUs);
      return measured[i].tiltDeg+fraction*(measured[i+1].tiltDeg-measured[i].tiltDeg);
    }
  }
  return measured[pointCount-1].tiltDeg;
}

// Existing live parameters use nine evenly spaced logical-angle knots.
// Resample only during parameter initialization, outside the control loop.
inline float defaultTilt(unsigned point) {
  return tiltAtPwm(servoPulseCommandUs(50+10*point));
}
} // namespace servo_calibration
} // namespace tricopter
