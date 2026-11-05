#pragma once
#include "TricopterConfig.h"
#include "FlightMath.h"

namespace tricopter {
// Aircraft level reference measured with an independent level, motors stopped,
// 2026-11-05. These are RAW indicated angles at physical level, not commands.
constexpr float measuredLevelRollDeg=.04f;
constexpr float measuredLevelPitchDeg=-.15f;

inline Quaternion quaternionProduct(Quaternion a,Quaternion b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}

// Keep fusion in its sensor frame. Apply one constant rigid rotation at the
// controller boundary to BOTH attitude and angular velocity. In particular,
// subtracting Euler angles or applying a world-fixed pitch trim during spin
// would not represent the same mounting correction at every heading.
class LevelAlignment {
 public:
  bool set(float rawLevelRoll,float rawLevelPitch) {
    valid_=std::isfinite(rawLevelRoll) && std::isfinite(rawLevelPitch) &&
        std::fabs(rawLevelRoll)<=5 && std::fabs(rawLevelPitch)<=5;
    if (!valid_) return false;
    const float r=.5f*rawLevelRoll*radians, p=-.5f*rawLevelPitch*radians;
    // Raw sensor attitude at heading zero: Ry(-publicPitch) Rx(publicRoll).
    const Quaternion level{std::cos(p)*std::cos(r),std::cos(p)*std::sin(r),
        std::sin(p)*std::cos(r),-std::sin(p)*std::sin(r)};
    sensorFromBody_={level.w,-level.x,-level.y,-level.z};
    sensorToBody_=Rotation(level);
    return true;
  }
  bool valid() const { return valid_; }
  Quaternion attitude(Quaternion raw) const {
    return quaternionProduct(raw,sensorFromBody_);
  }
  Vec3 gyro(Vec3 rawPilotDps) const {
    const Vec3 frd=sensorToBody_.world({rawPilotDps.x,-rawPilotDps.y,-rawPilotDps.z});
    return {frd.x,-frd.y,-frd.z};
  }
 private:
  bool valid_=true;
  Quaternion sensorFromBody_{1,0,0,0};
  Rotation sensorToBody_{Quaternion{1,0,0,0}};
};
} // namespace tricopter
