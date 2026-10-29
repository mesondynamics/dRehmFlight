#pragma once
#include "TricopterConfig.h"
#include "FlightMath.h"
#include "YawServoControl.h"
#include <stdint.h>

namespace tricopter {
struct PilotInput {
  float throttle, roll, pitch, yaw, spin; // normalized; yaw/spin positive LEFT
  bool spinRequested;
};
// Caller supplies the sketch's internal T/R/P/Y channels (yaw already reversed).
inline PilotInput pilotInput(unsigned throttle, unsigned roll, unsigned pitch,
                             unsigned yaw, unsigned spin, bool wasSpinning) {
  const float offset = std::fabs(static_cast<float>(spin)-stickCenterUs);
  const bool requested = wasSpinning ? offset > spinExitUs : offset > spinEnterUs;
  return {clamp((static_cast<float>(throttle)-1000)/1000, 0, 1),
          rollDirection*centered(roll), pitchDirection*centered(pitch), centered(yaw),
          spinDirection*centered(spin, spinExitUs), requested};
}
struct ControlGains { AxisGains roll, pitch, yaw; };
enum ControlFaultReason : uint32_t {
  faultSensor=1, faultTime=2, faultQuaternion=4, faultGyroscope=8,
  faultPilotInput=16, faultHeadingSingularity=32, faultInverted=64
};
enum class FlightMode : unsigned { Normal=0, Spin=1, Braking=2, Aligning=3 };
struct ControlOutput {
  MotorMix motors;
  float roll, pitch, yaw, servoDeg;
  float heading, reference, phase, yawTarget, yawRate, mixScale, tiltLimit;
  float yawRequested, yawFilteredRate, yawAuthority, yawIntegral;
  bool yawSlewLimited;
  FlightMode mode;
  bool fault;
};

class CycloneControl {
 public:
  bool spinning() const { return spinSelected_; }
  bool faulted() const { return fault_; }
  uint32_t faultReason() const { return faultReason_; }
  uint32_t faultCount() const { return faultCount_; }
  ControlOutput step(PilotInput in, Quaternion q, Vec3 pilotGyro, float dt,
                     bool armed, bool cut, bool sensorHealthy, const ControlGains &gains) {
    ControlOutput out{};
    out.servoDeg = servoCenterDeg;
    out.mixScale = 1;
    const bool validTime = std::isfinite(dt) && dt > 0 && dt <= .01f;
    const bool validQuaternion=q.normalize();
    uint32_t reason=0;
    if (!sensorHealthy) reason|=faultSensor;
    if (!validTime) reason|=faultTime;
    if (!validQuaternion) reason|=faultQuaternion;
    if (!pilotGyro.finite()) reason|=faultGyroscope;
    if (!std::isfinite(in.throttle) || !std::isfinite(in.roll) || !std::isfinite(in.pitch) ||
        !std::isfinite(in.yaw) || !std::isfinite(in.spin)) reason|=faultPilotInput;
    // Heading is undefined when the forward axis is vertical. This upright-only
    // controller also rejects inversion (the antipodal thrust error has no unique axis).
    if (validQuaternion) {
      const Rotation r(q);
      if (!(1-r.bx.z*r.bx.z > .01f)) reason|=faultHeadingSingularity;
      if (!(r.bz.z > 0)) reason|=faultInverted;
    }
    const bool healthy=reason==0;
    if (!healthy) {
      // Preserve the entry reason through subsequent healthy samples and cut recovery.
      if (!fault_) { faultReason_=reason; ++faultCount_; }
      fault_ = true; recoveryTime_ = 0;
    }
    if (fault_) {
      if (healthy && cut && in.throttle < .05f && !in.spinRequested && in.spin == 0) {
        recoveryTime_ += dt;
        if (recoveryTime_ >= faultRecoverySeconds) { fault_ = false; recoveryTime_ = 0; }
      } else recoveryTime_ = 0;
      resetFlight(healthy ? Rotation(q).heading() : 0);
      out.fault = fault_;
      out.mode = FlightMode::Normal;
      out.tiltLimit = normalTiltDeg;
      return out; // even the recovery frame stays stopped; a new arm is required
    }
    const Rotation rotation(q);
    const float heading = rotation.heading();
    const float measuredYawRate = rotation.headingRate(pilotGyro);
    out.heading = heading;
    out.yawRate = measuredYawRate;
    if (!armed || cut || in.throttle < .06f) {
      resetFlight(heading);
      if (armed && !cut) out.motors = mix(in.throttle, 0, 0);
      out.reference = heading;
      out.mode = mode_;
      out.tiltLimit = tiltLimit_;
      return out;
    }
    const bool firstActive = !active_;
    if (firstActive) { resetFlight(heading); active_ = true; }
    const float oldReference = reference_;
    spinSelected_ = in.spinRequested;
    if (spinSelected_ && mode_ != FlightMode::Spin) {
      // Normal mode's reference always follows the nose; braking/aligning retain theirs.
      if (mode_ == FlightMode::Normal) reference_ = heading;
      mode_ = FlightMode::Spin;
      stoppedTime_ = 0;
    } else if (!spinSelected_ && mode_ == FlightMode::Spin) {
      mode_ = FlightMode::Braking;
      stoppedTime_ = 0;
    }

    float targetRate = 0;
    float referenceRate = 0;
    if (mode_ == FlightMode::Normal) {
      reference_ = heading;
      referenceRate = measuredYawRate;
      targetRate = in.yaw*pilotYawMaxDps;
    } else if (mode_ == FlightMode::Aligning) {
      alignTime_ = std::fmin(alignTime_+dt, exitBlendSeconds);
      alignPilotOffset_ += in.yaw*pilotYawMaxDps*dt;
      const float t = alignTime_/exitBlendSeconds;
      const float smooth = t*t*(3-2*t);
      reference_ = wrapDegrees(heading+(alignOffset_+alignPilotOffset_)*(1-smooth));
      referenceRate = wrapDegrees(reference_-oldReference)/dt;
      if (alignTime_ >= exitBlendSeconds) mode_ = FlightMode::Normal;
    } else {
      referenceRate = in.yaw*pilotYawMaxDps;
      reference_ = wrapDegrees(reference_+referenceRate*dt);
      if (mode_ == FlightMode::Spin) targetRate = in.spin*spinMaxDps;
    }
    yawRate_ = approach(yawRate_, targetRate, yawSlewDps2*dt);
    if (mode_ == FlightMode::Braking) {
      if (std::fabs(measuredYawRate) < spinStoppedDps && std::fabs(yawRate_) < .01f)
        stoppedTime_ += dt;
      else stoppedTime_ = 0;
      if (stoppedTime_ >= spinStoppedSeconds) {
        mode_ = FlightMode::Aligning;
        alignOffset_ = wrapDegrees(reference_-heading);
        alignPilotOffset_ = alignTime_ = 0;
      }
    }
    tiltLimit_ = approach(tiltLimit_, mode_ == FlightMode::Normal ? normalTiltDeg : spinTiltDeg,
                          tiltLimitSlewDps*dt);

    // Transport the stored integral torque as the reference axes turn. This
    // keeps the stored physical correction continuous when modes change.
    const float dr = wrapDegrees(reference_-oldReference)*radians;
    const float c=std::cos(dr), s=std::sin(dr);
    const float oldIr = integralRoll_, oldIp = integralPitch_;
    integralRoll_ = c*oldIr+s*oldIp;
    integralPitch_ = -s*oldIr+c*oldIp;

    const float h = reference_*radians;
    const Vec3 forward{std::cos(h), -std::sin(h), 0};
    const Vec3 right{std::sin(h), std::cos(h), 0};
    // Limit the stick vector radially, so diagonals cannot exceed the tilt limit.
    const float stickNorm = std::sqrt(in.roll*in.roll+in.pitch*in.pitch);
    const float k = tiltLimit_*radians / std::fmax(1.0f, stickNorm);
    const float roll = in.roll*k, pitch = in.pitch*k;
    const Vec3 desiredUp = forward*(std::sin(pitch)*std::cos(roll)) +
                          right*std::sin(roll) + Vec3{0,0,-std::cos(pitch)*std::cos(roll)};
    const Vec3 up = rotation.bz*(-1);
    Vec3 error = up.cross(desiredUp);
    const float en = std::sqrt(error.dot(error));
    error = error*(en > 1e-6f ? degrees*std::atan2(en, up.dot(desiredUp))/en : degrees);
    const float er = error.dot(forward), ep = -error.dot(right);

    // Project relative angular velocity perpendicular to the thrust axis.
    // A spin about body Z does not change the thrust direction and must not
    // masquerade as roll/pitch damping. Reference yaw is removed in world space.
    Vec3 relative = rotation.world({pilotGyro.x, -pilotGyro.y, -pilotGyro.z}) -
                    Vec3{0, 0, -referenceRate};
    relative = relative-up*relative.dot(up);
    const float rollBase = gains.roll.kp*er-gains.roll.kd*relative.dot(forward);
    const float pitchBase = gains.pitch.kp*ep+gains.pitch.kd*relative.dot(right);
    const float rateFraction = clamp(std::fabs(measuredYawRate)/spinMaxDps,0,1);
    const float integralScale = 1/(1+(1/spinTiltIntegralScale-1)*rateFraction*rateFraction);
    const float priorIr=integralRoll_, priorIp=integralPitch_;
    integralRoll_ = clamp(integralRoll_+gains.roll.ki*integralScale*er*dt, -tiltIntegralLimit, tiltIntegralLimit);
    integralPitch_ = clamp(integralPitch_+gains.pitch.ki*integralScale*ep*dt, -tiltIntegralLimit, tiltIntegralLimit);
    Vec3 bodyTorque = rotation.body(forward*(rollBase+integralRoll_) -
                                    right*(pitchBase+integralPitch_));
    LimitedMix mixed = mixLimited(in.throttle, bodyTorque.x, -bodyTorque.y);
    if (mixed.scale < 1) {
      if ((integralRoll_-priorIr)*(rollBase+integralRoll_) > 0) integralRoll_ = priorIr;
      if ((integralPitch_-priorIp)*(pitchBase+integralPitch_) > 0) integralPitch_ = priorIp;
      bodyTorque = rotation.body(forward*(rollBase+integralRoll_) -
                                 right*(pitchBase+integralPitch_));
      mixed = mixLimited(in.throttle, bodyTorque.x, -bodyTorque.y);
    }
    const YawServoOutput yaw=yawControl_.step(yawRate_,measuredYawRate,in.throttle,dt,gains.yaw);
    out.roll = bodyTorque.x;
    out.pitch = -bodyTorque.y;
    out.yaw = yaw.applied;
    out.yawRequested = yaw.requested;
    out.yawFilteredRate = yaw.rateFiltered;
    out.yawAuthority = yaw.authority;
    out.yawIntegral = yaw.integral;
    out.yawSlewLimited = yaw.slewLimited;
    out.servoDeg = yawServoDeg(out.yaw);
    out.motors = mixed.motors;
    out.mixScale = mixed.scale;
    out.reference = reference_;
    out.phase = wrapDegrees(heading-reference_);
    out.yawTarget = yawRate_;
    out.mode = mode_;
    out.tiltLimit = tiltLimit_;
    return out;
  }
 private:
  void resetFlight(float heading) {
    mode_ = FlightMode::Normal;
    reference_ = heading;
    spinSelected_ = active_ = false;
    yawRate_ = stoppedTime_ = alignTime_ = alignOffset_ = alignPilotOffset_ = 0;
    integralRoll_ = integralPitch_ = 0;
    yawControl_.reset();
    tiltLimit_ = normalTiltDeg;
  }
  FlightMode mode_ = FlightMode::Normal;
  bool spinSelected_ = false, active_ = false, fault_ = false;
  float reference_ = 0, yawRate_ = 0, tiltLimit_ = normalTiltDeg;
  float stoppedTime_ = 0, alignTime_ = 0, alignOffset_ = 0, alignPilotOffset_ = 0;
  float integralRoll_ = 0, integralPitch_ = 0;
  YawServoControl yawControl_;
  float recoveryTime_ = 0;
  uint32_t faultReason_=0, faultCount_=0;
};
} // namespace tricopter
