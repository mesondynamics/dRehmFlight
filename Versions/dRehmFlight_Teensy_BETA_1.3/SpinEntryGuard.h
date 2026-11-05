#pragma once
#include "TricopterConfig.h"
#include <cmath>

namespace tricopter {
// One extra integer STATE keeps the complete management frame within 512 B.
enum class SpinEntryState : unsigned {
  Idle=0, CenterRequired=1, Settling=2, Ready=3, Confirming=4,
  Running=5, AbortLocked=6, Disabled=7
};

// Qualify a deliberate entry without changing any airborne abort threshold.
// A request made before readiness is rejected until S1 has been centered again;
// it must never silently start later just because the aircraft settles.
class SpinEntryGuard {
 public:
  void reset() {
    centerTime_=stableTime_=requestTime_=0;
    centered_=false; state_=SpinEntryState::Idle;
  }
  SpinEntryState state() const { return state_; }
  bool step(bool requested,bool center,bool yawCentered,bool settled,
            bool normal,bool inhibited,float dt) {
    if(inhibited) {
      reset(); state_=SpinEntryState::AbortLocked; return false;
    }
    if(!normal) {
      reset(); state_=SpinEntryState::Running; return requested;
    }
    if(center) {
      centerTime_=std::fmin(centerTime_+dt,.3f);
      if(centerTime_>=.3f) centered_=true;
    } else centerTime_=0;
    const bool readyNow=yawCentered && settled;
    if(!readyNow) stableTime_=0;
    else if(centered_) stableTime_=std::fmin(stableTime_+dt,.5f);
    if(requested) {
      if(!centered_ || stableTime_<.5f || !readyNow) {
        reset(); state_=SpinEntryState::CenterRequired; return false;
      }
      requestTime_=std::fmin(requestTime_+dt,.3f);
      state_=requestTime_>=.3f ? SpinEntryState::Running : SpinEntryState::Confirming;
      return requestTime_>=.3f;
    }
    requestTime_=0;
    state_=!centered_ ? SpinEntryState::CenterRequired :
        (stableTime_<.5f ? SpinEntryState::Settling : SpinEntryState::Ready);
    return false;
  }
 private:
  float centerTime_=0,stableTime_=0,requestTime_=0;
  bool centered_=false;
  SpinEntryState state_=SpinEntryState::Idle;
};
} // namespace tricopter
