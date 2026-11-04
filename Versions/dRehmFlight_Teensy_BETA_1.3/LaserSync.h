#pragma once
#include "FlightMath.h"
#include <stdint.h>

namespace tricopter {
enum class LaserMode : uint8_t { Off, Continuous, PhasePulse };

struct LaserSyncConfig {
  float alignWindowDeg=2;
  uint32_t pulseUs=10000, minIntervalUs=100000;
  bool valid() const {
    return std::isfinite(alignWindowDeg) && alignWindowDeg>0 && alignWindowDeg<45 &&
           pulseUs>0 && pulseUs<=200000 && minIntervalUs>=pulseUs && minIntervalUs<=2000000;
  }
};

// Normal flight holds the direction marker high. During spin and its exit,
// emit one high pulse when projected body +X crosses generalized +X, either way.
// Azimuth is gyro-referenced, not an absolute compass direction. No delay(),
// timer ISR or telemetry dependency. Count/time survive mode exits so a 100 Hz
// recording cannot lose the existence of a sub-sample-width laser pulse.
class LaserSync {
 public:
  bool update(LaserMode mode,float phase,float relativeRateDps,uint32_t now,
              const LaserSyncConfig &cfg) {
    if (mode!=LaserMode::PhasePulse) {
      tracking_=pulsing_=false;
      return mode==LaserMode::Continuous && cfg.valid();
    }
    phase=wrapDegrees(phase);
    if (!cfg.valid() || !std::isfinite(phase) ||
        !std::isfinite(relativeRateDps) || std::fabs(relativeRateDps)<5) {
      tracking_=pulsing_=false;
      return false;
    }
    if (!tracking_) { lastPhase_=phase; tracking_=true; return false; }
    const bool crossed=((lastPhase_<0 && phase>=0) || (lastPhase_>0 && phase<=0)) &&
                       std::fabs(phase-lastPhase_)<180 && std::fabs(phase)<=cfg.alignWindowDeg;
    lastPhase_=phase;
    if (crossed && (!havePulse_ || static_cast<uint32_t>(now-lastPulseUs_)>=cfg.minIntervalUs)) {
      lastPulseUs_=now; ++pulseCount_; havePulse_=pulsing_=true;
    }
    if (pulsing_ && (static_cast<uint32_t>(now-lastPulseUs_)>=cfg.pulseUs ||
                    std::fabs(phase)>cfg.alignWindowDeg)) pulsing_=false;
    return pulsing_;
  }
  uint32_t pulseCount() const { return pulseCount_; }
  uint32_t lastPulseUs() const { return lastPulseUs_; }
 private:
  bool tracking_=false, pulsing_=false, havePulse_=false;
  float lastPhase_=0;
  uint32_t lastPulseUs_=0, pulseCount_=0;
};
} // namespace tricopter
