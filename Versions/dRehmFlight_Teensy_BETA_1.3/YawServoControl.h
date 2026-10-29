#pragma once
#include "TricopterConfig.h"
#include "FlightMath.h"

namespace tricopter {
struct AxisGains { float kp, ki, kd; }; // output per degree, degree-second, deg/s
inline AxisGains defaultYawGains() { return {.01f*yawKp, .01f*yawKi, .01f*yawKd}; }
inline float smoothFraction(float value, float start, float end) {
  const float t=clamp((value-start)/(end-start),0,1);
  return t*t*(3-2*t);
}
inline float lowpassAlpha(float hz, float dt) {
  const float k=6.28318530718f*hz*dt;
  return k/(1+k);
}
struct YawServoOutput {
  float applied, requested, rateFiltered, authority, integral;
  bool slewLimited;
};

// Separate the 2 kHz sensor/control loop from the bandwidth of the 50 Hz
// mechanical servo. State is reset on cut/idle/fault, not on stick recentering.
class YawServoControl {
 public:
  void reset() {
    initialized_=false;
    rate_=derivative_=integral_=filteredCommand_=applied_=0;
  }
  YawServoOutput step(float target, float measurement, float throttle, float dt, AxisGains gains) {
    const bool first=!initialized_;
    if (first) { rate_=measurement; initialized_=true; }
    const float previousRate=rate_;
    rate_ += lowpassAlpha(yawRateFilterHz,dt)*(measurement-rate_);
    const float derivative=first ? 0 : (rate_-previousRate)/dt;
    derivative_ += lowpassAlpha(yawDerivativeFilterHz,dt)*(derivative-derivative_);
    float error=target-rate_;
    // Continuous deadband, rather than alternating full corrections around zero.
    error = error > yawRateDeadbandDps ? error-yawRateDeadbandDps :
            (error < -yawRateDeadbandDps ? error+yawRateDeadbandDps : 0);
    const float authority=yawLowThrottleGain+(1-yawLowThrottleGain)*
        smoothFraction(throttle,yawFeedbackStartThrottle,yawFeedbackFullThrottle);
    const float integralWeight=smoothFraction(throttle,yawIntegralStartThrottle,yawFeedbackFullThrottle);
    const float base=gains.kp*error-gains.kd*derivative_;
    float proposed=clamp(integral_+gains.ki*error*dt*integralWeight,
                         -yawIntegralLimit,yawIntegralLimit);
    float requested=authority*(base+proposed);
    float filtered=filteredCommand_+lowpassAlpha(yawServoFilterHz,dt)*
        (clamp(requested,-yawOutputLimit,yawOutputLimit)-filteredCommand_);
    const float maxStep=yawServoSlewDegPerSec*dt/180.0f;
    bool slew=std::fabs(filtered-applied_)>maxStep;
    // Do not integrate further into an amplitude or rate limit. Keep the
    // accumulated torque trim when error changes sign or the demand is released.
    if ((std::fabs(requested)>yawOutputLimit || slew) &&
        (proposed-integral_)*(requested-applied_)>0) {
      proposed=integral_;
      requested=authority*(base+proposed);
      filtered=filteredCommand_+lowpassAlpha(yawServoFilterHz,dt)*
          (clamp(requested,-yawOutputLimit,yawOutputLimit)-filteredCommand_);
      slew=std::fabs(filtered-applied_)>maxStep;
    }
    integral_=proposed;
    filteredCommand_=filtered;
    applied_=approach(applied_,filteredCommand_,maxStep);
    return {applied_,requested,rate_,authority,integral_,slew};
  }
 private:
  bool initialized_=false;
  float rate_=0, derivative_=0, integral_=0, filteredCommand_=0, applied_=0;
};
} // namespace tricopter
