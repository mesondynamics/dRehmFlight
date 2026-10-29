#pragma once
#include <cmath>

// Fusion coordinates: forward/right/down (the sketch feeds Gx,-Gy,-Gz).
// Public pilot angles: roll right, pitch forward, heading LEFT/CCW positive.
namespace tricopter {
constexpr float radians = 0.017453292519943295f;
constexpr float degrees = 57.29577951308232f;

inline float wrapDegrees(float angle) {
  angle = std::fmod(angle + 180.0f, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle - 180.0f;
}
inline float approach(float value, float target, float step) {
  return value + (target > value ? std::fmin(target - value, step) :
                                  -std::fmin(value - target, step));
}
struct Vec3 {
  float x, y, z;
  Vec3 operator+(Vec3 v) const { return {x+v.x, y+v.y, z+v.z}; }
  Vec3 operator-(Vec3 v) const { return {x-v.x, y-v.y, z-v.z}; }
  Vec3 operator*(float k) const { return {x*k, y*k, z*k}; }
  float dot(Vec3 v) const { return x*v.x + y*v.y + z*v.z; }
  Vec3 cross(Vec3 v) const { return {y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x}; }
  bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};
struct Quaternion {
  float w, x, y, z;
  bool normalize() {
    const float n = w*w+x*x+y*y+z*z;
    if (!std::isfinite(n) || n < 0.25f || n > 4.0f) return false;
    const float k = 1.0f / std::sqrt(n);
    w*=k; x*=k; y*=k; z*=k;
    return true;
  }
};
struct Rotation {
  Vec3 bx, by, bz; // Body basis vectors in world FRD coordinates.
  explicit Rotation(Quaternion q)
      : bx{1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z), 2*(q.x*q.z-q.w*q.y)},
        by{2*(q.x*q.y-q.w*q.z), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z+q.w*q.x)},
        bz{2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)} {}
  Vec3 world(Vec3 b) const { return bx*b.x + by*b.y + bz*b.z; }
  Vec3 body(Vec3 w) const { return {bx.dot(w), by.dot(w), bz.dot(w)}; }
  float heading() const { return -std::atan2(bx.y, bx.x)*degrees; }
  // Euler heading rate, not body GyroZ. Valid away from the pitch singularity.
  float headingRate(Vec3 pilotGyro) const {
    return (by.z*pilotGyro.y + bz.z*pilotGyro.z) / (1-bx.z*bx.z);
  }
};

// Checked Madgwick IMU step. A perfectly aligned gravity vector gives a zero
// gradient: skip its correction instead of dividing by zero. No magnetometer.
inline bool updateImu(Quaternion &q, Vec3 gyroFrd, Vec3 accel, float dt, float beta) {
  if (!gyroFrd.finite() || !accel.finite() || !std::isfinite(dt) || dt <= 0 || dt > .01f ||
      !std::isfinite(beta) || beta < 0 || !q.normalize()) return false;
  const Vec3 g = gyroFrd*radians;
  Quaternion d{-.5f*(q.x*g.x+q.y*g.y+q.z*g.z),
                .5f*(q.w*g.x+q.y*g.z-q.z*g.y),
                .5f*(q.w*g.y-q.x*g.z+q.z*g.x),
                .5f*(q.w*g.z+q.x*g.y-q.y*g.x)};
  const float an = accel.dot(accel);
  if (!std::isfinite(an) || an < 1e-8f) return false;
  // Reject strong translational/centripetal acceleration as a gravity reference.
  // Gyro propagation continues; this does not provide position/velocity feedback.
  if (an >= .64f && an <= 1.44f) {
    accel = accel*(1.0f/std::sqrt(an));
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    Quaternion s{4*w*(x*x+y*y)+2*y*accel.x-2*x*accel.y,
      4*x*z*z-2*z*accel.x+4*w*w*x-2*w*accel.y-4*x+8*x*(x*x+y*y)+4*x*accel.z,
      4*w*w*y+2*w*accel.x+4*y*z*z-2*z*accel.y-4*y+8*y*(x*x+y*y)+4*y*accel.z,
      4*x*x*z-2*x*accel.x+4*y*y*z-2*y*accel.y};
    const float sn=s.w*s.w+s.x*s.x+s.y*s.y+s.z*s.z;
    if (sn > 1e-12f) {
      const float k=beta/std::sqrt(sn);
      d.w-=k*s.w; d.x-=k*s.x; d.y-=k*s.y; d.z-=k*s.z;
    }
  }
  q.w+=d.w*dt; q.x+=d.x*dt; q.y+=d.y*dt; q.z+=d.z*dt;
  return q.normalize();
}
} // namespace tricopter
