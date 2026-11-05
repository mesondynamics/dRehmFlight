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
    centerTime_=stableTime_=requestTime_=outsideTime_=0;
    centered_=ready_=false; state_=SpinEntryState::Idle;
  }
  SpinEntryState state() const { return state_; }
  bool step(bool requested,bool center,bool yawCentered,bool settled,bool withinEnvelope,
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
    if(!yawCentered) {
      stableTime_=outsideTime_=0; ready_=false;
    } else if(centered_ && !ready_) {
      stableTime_=settled && withinEnvelope ? std::fmin(stableTime_+dt,.5f) : 0;
      ready_=stableTime_>=.5f;
      outsideTime_=0;
    } else if(ready_) {
      // A tighter acquisition band and wider retention band prevent a usable
      // permission from flickering with each small normal-flight correction.
      outsideTime_=withinEnvelope ? 0 : std::fmin(outsideTime_+dt,.25f);
      if(outsideTime_>=.25f) {
        ready_=false; stableTime_=outsideTime_=0;
      }
    }
    if(requested) {
      if(!centered_ || !ready_ || !yawCentered) {
        reset(); state_=SpinEntryState::CenterRequired; return false;
      }
      // A brief excursion pauses confirmation; acceptance still requires a
      // fresh sample inside the envelope. A sustained excursion rejects the
      // request above and requires center, never a delayed unattended start.
      if(withinEnvelope) requestTime_=std::fmin(requestTime_+dt,.3f);
      const bool accepted=withinEnvelope && requestTime_>=.3f;
      state_=accepted ? SpinEntryState::Running : SpinEntryState::Confirming;
      return accepted;
    }
    requestTime_=0;
    state_=!centered_ ? SpinEntryState::CenterRequired :
        (ready_ ? SpinEntryState::Ready : SpinEntryState::Settling);
    return false;
  }
 private:
  float centerTime_=0,stableTime_=0,requestTime_=0,outsideTime_=0;
  bool centered_=false,ready_=false;
  SpinEntryState state_=SpinEntryState::Idle;
};
} // namespace tricopter
