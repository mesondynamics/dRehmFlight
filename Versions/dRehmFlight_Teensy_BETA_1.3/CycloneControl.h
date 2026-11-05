#pragma once
#include "TricopterConfig.h"
#include "FlightMath.h"
#include "TricopterDynamics.h"
#include "YawServoControl.h"
#include "SpinSupervisor.h"
#include "SpinEntryGuard.h"
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
  float heading, reference, referenceRate, phase, yawTarget, yawRate, mixScale, tiltLimit;
  float yawRequested, yawFilteredRate, yawAuthority, yawIntegral;
  float yawTargetAccel, tiltErrorDeg;
  Vec3 desiredUp;
  float desiredTiltRate;
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
  const SpinSupervisor &supervisor() const { return supervisor_; }
  SpinEntryState spinEntryState() const {
    return supervisor_.inhibited() ? SpinEntryState::AbortLocked : entry_.state();
  }
  float brakingTime() const { return brakingTime_; }
  float stoppedDwell() const { return stoppedTime_; }
  float qualifiedRateDps() const { return stopRateTrend_; }
  ControlOutput step(PilotInput in, Quaternion q, Vec3 pilotGyro, float dt,
                     bool armed, bool cut, bool sensorHealthy, const ControlGains &gains) {
    return step(in,q,pilotGyro,dt,armed,cut,sensorHealthy,gains,nullptr,nullptr);
  }
  ControlOutput step(PilotInput in, Quaternion q, Vec3 pilotGyro, float dt,
                     bool armed, bool cut, bool sensorHealthy, const ControlGains &gains,
                     const SpinDynamicsModel *spinModel,
                     const AttitudeReferenceConfig *referenceConfig,
                     const SpinSafetyConfig *safetyConfig=nullptr) {
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
    if(healthy && cut && in.throttle<.05f && !in.spinRequested && in.spin==0)
      supervisor_.clear();
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
    const bool haveSpinModel=spinModel && spinModel->valid();
    const SpinSafetyConfig &safety=safetyConfig ? *safetyConfig : defaultSafety_;
    const float spinRateLimit=haveSpinModel ? spinModel->maxRateDps : spinMaxDps;
    const float spinAccelLimit=haveSpinModel ? spinModel->maxAccelDps2 : yawSlewDps2;
    const float normalTiltLimit=referenceConfig ? referenceConfig->normalTilt : normalTiltDeg;
    const float spinTiltLimit=referenceConfig ? referenceConfig->spinTilt : spinTiltDeg;
    const float referenceYawLimit=referenceConfig ? referenceConfig->generalizedYawMaxDps : pilotYawMaxDps;
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
    if (firstActive) {
      resetFlight(heading); active_ = true;
      desiredUp_=rotation.bz*(-1);
    }
    const float oldReference = reference_;
    // Allocation feedback is consumed once, from the preceding 500 us frame.
    const bool haveFeedback=feedbackValid_;
    feedbackValid_=false;
    const float currentTiltError=haveSpinModel ?
        vectorAngle(rotation.bz*(-1),desiredUp_)*degrees : 0;
    if(mode_==FlightMode::Braking) brakingTime_+=dt;
    if(haveSpinModel)
      supervisor_.update(safety,mode_!=FlightMode::Normal,spinRateLimit,
          haveFeedback ? allocatedRateTarget_ : yawRate_,measuredYawRate,
          currentTiltError,
          haveFeedback ? allocatedLiftScale_ : 1,
          haveFeedback ? allocatedAttitudeScale_ : 1,
          haveFeedback && allocatedYawLimited_,
          mode_==FlightMode::Braking ? brakingTime_ : 0,brakingBudget_,dt);
    // Require ordinary Yaw to have settled before changing its job to self-spin.
    // Use the actual physical controller's preceding target, not this manager's
    // legacy stick target. Keep its PI, servo state and limited ramp continuous.
    const float entryRate=std::fmin(5.f,.5f*spinRateLimit);
    const float entryError=std::fmin(5.f,.5f*safety.rateErrorDps);
    if(haveSpinModel && haveFeedback) {
      if(!entryRatesInitialized_) {
        qualificationRate_=stopRateTrend_=measuredYawRate; entryTargetRate_=allocatedRateTarget_;
        entryRatesInitialized_=true;
      } else {
        // Entry uses 2 Hz and stopped qualification uses 1 Hz. The flight PI
        // and supervisor retain their existing, faster measurements.
        const float alpha=lowpassAlpha(2,dt);
        qualificationRate_+=alpha*(measuredYawRate-qualificationRate_);
        stopRateTrend_+=lowpassAlpha(1,dt)*(measuredYawRate-stopRateTrend_);
        entryTargetRate_+=alpha*(allocatedRateTarget_-entryTargetRate_);
      }
    }
    // Match the existing overspeed envelope rather than treating a one-frame
    // excursion above the requested 10 deg/s as an entry fault. Qualification
    // and retention remain separate from the unchanged airborne supervisor.
    const float entryInstantLimit=spinRateLimit+safety.overspeedMarginDps;
    const bool entryEnvelope=haveFeedback &&
        std::fabs(measuredYawRate)<=entryInstantLimit &&
        std::fabs(qualificationRate_)<=std::fmin(8.f,spinRateLimit) &&
        std::fabs(entryTargetRate_)<=std::fmin(8.f,spinRateLimit) &&
        std::fabs(entryTargetRate_-qualificationRate_)<=std::fmin(8.f,safety.rateErrorDps) &&
        currentTiltError<=std::fmin(7.f,safety.tiltErrorDeg);
    const bool entrySettled=haveFeedback &&
        std::fabs(measuredYawRate)<=entryInstantLimit &&
        std::fabs(qualificationRate_)<=entryRate && std::fabs(entryTargetRate_)<=entryRate &&
        std::fabs(entryTargetRate_-qualificationRate_)<=entryError &&
        currentTiltError<=std::fmin(5.f,safety.tiltErrorDeg);
    spinSelected_ = haveSpinModel ?
        entry_.step(in.spinRequested,in.spin==0,std::fabs(in.yaw)<.001f,entrySettled,entryEnvelope,
                    mode_==FlightMode::Normal,supervisor_.inhibited(),dt) :
        (in.spinRequested && !supervisor_.inhibited());
    if (spinSelected_ && mode_ != FlightMode::Spin) {
      // Normal mode's reference always follows the nose; braking/aligning retain theirs.
      if (mode_ == FlightMode::Normal) {
        reference_ = heading;
        // Unified normal Yaw has its own limited heading/rate controller.
        // Do not inherit this manager's unused legacy 160-deg/s stick target
        // when entering a 10--45-deg/s spin. Begin at the measured rate;
        // the shared physical controller retains its own continuous ramp.
        if (haveSpinModel) yawRate_=measuredYawRate;
      }
      mode_ = FlightMode::Spin;
      stoppedTime_ = brakingTime_ = 0;
    } else if (!spinSelected_ && mode_ == FlightMode::Spin) {
      beginBraking(measuredYawRate,spinAccelLimit,safety);
    }
    const float stopRate=haveSpinModel ? safety.stoppedRate(spinRateLimit) : spinStoppedDps;
    // A half-second of every raw sample below 2 deg/s is not a meaningful
    // zero-speed test with motor vibration. Keep the same mean-rate limit and
    // dwell, but reject instantaneous excursions beyond the tracking envelope.
    // Persistent rotation still appears in the trend and cannot qualify.
    const bool useTrend=haveSpinModel && haveFeedback && entryRatesInitialized_;
    const float stopMeasurement=useTrend ? stopRateTrend_ : measuredYawRate;
    const bool stopEnvelope=!useTrend || std::fabs(measuredYawRate)<safety.rateErrorDps;
    const bool stopped=stopEnvelope && std::fabs(stopMeasurement)<stopRate &&
        std::fabs(yawRate_)<.01f && (!haveFeedback || std::fabs(allocatedRateTarget_)<.1f);
    if(mode_==FlightMode::Aligning &&
       (!stopEnvelope || std::fabs(stopMeasurement)>1.5f*stopRate ||
        (haveFeedback && std::fabs(allocatedRateTarget_)>=.1f)))
      beginBraking(measuredYawRate,spinAccelLimit,safety);

    float targetRate = 0;
    float referenceRate = 0;
    if (mode_ == FlightMode::Normal) {
      reference_ = heading;
      referenceRate = measuredYawRate;
      targetRate = in.yaw*pilotYawMaxDps;
    } else if (mode_ == FlightMode::Aligning) {
      stoppedTime_=stopped ? stoppedTime_+dt : 0;
      alignTime_ = std::fmin(alignTime_+dt, exitBlendSeconds);
      // While the pilot steers generalized Yaw, postpone the coordinate handoff.
      // Otherwise follow the shortest arc at a bounded azimuth rate. A 180 deg
      // phase error therefore takes at least 9 s at the default 20 deg/s.
      const bool yawCentered=std::fabs(in.yaw)<.001f;
      const float wantedStep=yawCentered ? wrapDegrees(heading-reference_) :
          in.yaw*referenceYawLimit*dt;
      const float step=clamp(wantedStep,-safety.exitReferenceDps*dt,safety.exitReferenceDps*dt);
      reference_=wrapDegrees(reference_+step);
      referenceRate=step/dt;
      if(alignTime_>=exitBlendSeconds &&
         stoppedTime_>=(haveSpinModel ? safety.stopDwellSeconds : spinStoppedSeconds) && yawCentered &&
         std::fabs(wrapDegrees(heading-reference_))<.001f)
        mode_=FlightMode::Normal;
    } else {
      referenceRate = in.yaw*referenceYawLimit;
      reference_ = wrapDegrees(reference_+referenceRate*dt);
      if (mode_ == FlightMode::Spin) targetRate = in.spin*spinRateLimit;
    }
    const float oldYawTarget=yawRate_;
    yawRate_ = approach(yawRate_, targetRate, spinAccelLimit*dt);
    if (mode_ == FlightMode::Braking) {
      if (stopped)
        stoppedTime_ += dt;
      else stoppedTime_ = 0;
      if (stoppedTime_ >= (haveSpinModel ? safety.stopDwellSeconds : spinStoppedSeconds)) {
        mode_ = FlightMode::Aligning;
        alignTime_=brakingTime_=0;
      }
    }
    tiltLimit_ = approach(tiltLimit_, mode_ == FlightMode::Normal ? normalTiltLimit : spinTiltLimit,
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
    const Vec3 wantedUp = forward*(std::sin(pitch)*std::cos(roll)) +
                         right*std::sin(roll) + Vec3{0,0,-std::cos(pitch)*std::cos(roll)};
    const Vec3 desiredUp=haveSpinModel ?
        slewDirection(desiredUp_,wantedUp,safety.tiltCommandDps*radians*dt) : wantedUp;
    out.desiredTiltRate=vectorAngle(desiredUp_,desiredUp)*degrees/dt;
    out.desiredUp=desiredUp;
    desiredUp_=desiredUp;
    const Vec3 up = rotation.bz*(-1);
    Vec3 error = up.cross(desiredUp);
    const float en = std::sqrt(error.dot(error));
    out.tiltErrorDeg=degrees*std::atan2(en,up.dot(desiredUp));
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
    // Keep the proven integral schedule referenced to 360 deg/s. A low spin
    // speed limit must not unexpectedly reduce normal-flight I bandwidth.
    const float rateFraction = clamp(std::fabs(measuredYawRate)/spinMaxDps,0,1);
    const float integralScale = 1/(1+(1/spinTiltIntegralScale-1)*rateFraction*rateFraction);
    const float priorIr=integralRoll_, priorIp=integralPitch_;
    priorIntegralRoll_=priorIr; priorIntegralPitch_=priorIp;
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
    // Legacy actuator-space Yaw is kept for isolated bench builds. The normal
    // firmware replaces it with the shared physical-torque allocator below
    // this class, in EVERY mode including braking/alignment.
    YawServoOutput yaw{};
    if (!spinModel) // unified flight path has one PI; do not run a shadow PI
      yaw=yawControl_.step(yawRate_,measuredYawRate,in.throttle,dt,gains.yaw);
    out.roll = bodyTorque.x;
    out.pitch = -bodyTorque.y;
    out.yaw = yaw.applied;
    out.yawRequested = yaw.requested;
    out.yawFilteredRate = yaw.rateFiltered;
    out.yawAuthority = yaw.authority;
    out.yawIntegral = yaw.integral;
    out.yawTargetAccel=(yawRate_-oldYawTarget)/dt;
    out.yawSlewLimited = yaw.slewLimited;
    requestedRoll_=rollBase+integralRoll_;
    requestedPitch_=pitchBase+integralPitch_;
    out.servoDeg = yawServoDeg(out.yaw);
    out.motors = mixed.motors;
    out.mixScale = mixed.scale;
    out.reference = reference_;
    out.referenceRate = referenceRate;
    out.phase = wrapDegrees(heading-reference_);
    out.yawTarget = yawRate_;
    out.mode = mode_;
    out.tiltLimit = tiltLimit_;
    return out;
  }
  // The nonlinear allocator runs after this reference controller. Roll back
  // only this frame's integral growth toward an unreachable transverse demand;
  // keep unwinding and keep the already-produced command continuous.
  void applyAllocationLimits(bool transverseLimited) {
    if (!active_ || !transverseLimited) return;
    if ((integralRoll_-priorIntegralRoll_)*requestedRoll_>0)
      integralRoll_=priorIntegralRoll_;
    if ((integralPitch_-priorIntegralPitch_)*requestedPitch_>0)
      integralPitch_=priorIntegralPitch_;
  }
  void applyAllocationFeedback(float liftScale,float attitudeScale,bool yawLimited,float rateTarget) {
    applyAllocationLimits(liftScale<.9999f || attitudeScale<.9999f);
    allocatedLiftScale_=liftScale; allocatedAttitudeScale_=attitudeScale;
    allocatedYawLimited_=yawLimited; allocatedRateTarget_=rateTarget;
    feedbackValid_=active_;
  }
 private:
  void beginBraking(float rate,float accel,const SpinSafetyConfig &safety) {
    mode_=FlightMode::Braking;
    stoppedTime_=alignTime_=brakingTime_=0;
    brakingBudget_=std::fmax(std::fabs(rate),std::fabs(yawRate_))/accel+safety.brakingSettleSeconds;
  }
  void resetFlight(float heading) {
    mode_ = FlightMode::Normal;
    reference_ = heading;
    spinSelected_ = active_ = false;
    yawRate_ = stoppedTime_ = alignTime_ = brakingTime_ = 0;
    brakingBudget_=120;
    feedbackValid_=false;
    desiredUp_={0,0,-1};
    supervisor_.idle(); // The abort latch is cleared only by the explicit cut-state check.
    entry_.reset();
    qualificationRate_=stopRateTrend_=entryTargetRate_=0; entryRatesInitialized_=false;
    integralRoll_ = integralPitch_ = 0;
    priorIntegralRoll_=priorIntegralPitch_=requestedRoll_=requestedPitch_=0;
    yawControl_.reset();
    tiltLimit_ = normalTiltDeg;
  }
  FlightMode mode_ = FlightMode::Normal;
  bool spinSelected_ = false, active_ = false, fault_ = false;
  float reference_ = 0, yawRate_ = 0, tiltLimit_ = normalTiltDeg;
  float stoppedTime_=0,alignTime_=0,brakingTime_=0,brakingBudget_=120;
  Vec3 desiredUp_{0,0,-1};
  SpinSafetyConfig defaultSafety_{};
  SpinSupervisor supervisor_;
  SpinEntryGuard entry_;
  float qualificationRate_=0,stopRateTrend_=0,entryTargetRate_=0;
  bool entryRatesInitialized_=false;
  bool feedbackValid_=false,allocatedYawLimited_=false;
  float allocatedLiftScale_=1,allocatedAttitudeScale_=1,allocatedRateTarget_=0;
  float integralRoll_ = 0, integralPitch_ = 0;
  float priorIntegralRoll_=0, priorIntegralPitch_=0, requestedRoll_=0, requestedPitch_=0;
  YawServoControl yawControl_;
  float recoveryTime_ = 0;
  uint32_t faultReason_=0, faultCount_=0;
};
} // namespace tricopter
