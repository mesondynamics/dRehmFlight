#pragma once
#include "TricopterDynamics.h"
#include <stdint.h>

namespace tricopter {
struct SpinSafetyConfig {
  float stopRateDps=2, stopDwellSeconds=.5f;
  float exitReferenceDps=20, tiltCommandDps=60;
  float overspeedMarginDps=5, overspeedDwellSeconds=.25f;
  float rateErrorDps=8, rateErrorDwellSeconds=1;
  float tiltErrorDeg=8, tiltErrorDwellSeconds=.5f;
  float allocationMinScale=.95f, allocationDwellSeconds=.5f;
  float brakingSettleSeconds=5; // Added to the captured ideal stopping time.
  bool valid() const {
    const float positive[]={stopRateDps,stopDwellSeconds,exitReferenceDps,tiltCommandDps,
      overspeedMarginDps,overspeedDwellSeconds,rateErrorDps,rateErrorDwellSeconds,
      tiltErrorDeg,tiltErrorDwellSeconds,allocationDwellSeconds,brakingSettleSeconds};
    for(float value:positive) if(!std::isfinite(value) || value<=0) return false;
    return std::isfinite(allocationMinScale) && allocationMinScale>=.5f && allocationMinScale<1;
  }
  float stoppedRate(float maxRate) const { return std::fmin(stopRateDps,.1f*maxRate); }
};
enum SpinAbortReason : uint32_t {
  spinOverspeed=1, spinRateError=2, spinTiltError=4,
  spinAllocation=8, spinBrakingTimeout=16
};

// A spin abort never disarms the aircraft. It requests the existing closed-loop
// braking path, keeps Roll/Pitch active, and inhibits another airborne spin.
class SpinSupervisor {
 public:
  uint32_t reasons() const { return reasons_; }
  uint32_t count() const { return count_; }
  bool inhibited() const { return reasons_!=0; }
  float overspeedTime() const { return overspeed_; }
  float rateErrorTime() const { return rateError_; }
  float tiltErrorTime() const { return tiltError_; }
  float allocationTime() const { return allocation_; }
  void clear() { reasons_=0; idle(); } // Caller requires SA cut, low throttle and centered S1.
  void idle() { overspeed_=rateError_=tiltError_=allocation_=0; }
  void update(const SpinSafetyConfig &c,bool enabled,float rateLimit,float targetRate,
              float measuredRate,float tiltError,float liftScale,float attitudeScale,
              bool yawLimited,float brakingTime,float brakingBudget,float dt) {
    if(!enabled) { idle(); return; }
    dwell(overspeed_,std::fabs(measuredRate)>rateLimit+c.overspeedMarginDps,dt);
    const float error=std::fabs(targetRate-measuredRate);
    dwell(rateError_,error>c.rateErrorDps,dt);
    dwell(tiltError_,tiltError>c.tiltErrorDeg,dt);
    dwell(allocation_,liftScale<c.allocationMinScale || attitudeScale<c.allocationMinScale ||
          (yawLimited && error>c.stoppedRate(rateLimit)),dt);
    uint32_t reason=0;
    if(overspeed_>=c.overspeedDwellSeconds) reason|=spinOverspeed;
    if(rateError_>=c.rateErrorDwellSeconds) reason|=spinRateError;
    if(tiltError_>=c.tiltErrorDwellSeconds) reason|=spinTiltError;
    if(allocation_>=c.allocationDwellSeconds) reason|=spinAllocation;
    if(brakingTime>brakingBudget) reason|=spinBrakingTimeout;
    if(reason && !reasons_) ++count_;
    reasons_|=reason;
  }
 private:
  static void dwell(float &timer,bool condition,float dt) {
    timer=condition ? std::fmin(timer+dt,120.f) : 0;
  }
  uint32_t reasons_=0,count_=0;
  float overspeed_=0,rateError_=0,tiltError_=0,allocation_=0;
};
} // namespace tricopter
