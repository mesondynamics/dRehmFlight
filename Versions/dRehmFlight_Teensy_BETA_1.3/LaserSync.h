#pragma once
#include "FlightMath.h"
#include <stdint.h>

namespace tricopter {
enum class LaserMode : uint8_t { Off, Continuous, PhasePulse };

struct LaserSyncConfig {
  float alignWindowDeg=2;
  uint32_t pulseUs=200000, minIntervalUs=200000;
  float phaseLeadSeconds=0; // Explicit measured/estimated AHRS latency, zero preserves v5.
  bool valid() const {
    return std::isfinite(alignWindowDeg) && alignWindowDeg>0 && alignWindowDeg<45 &&
           pulseUs>0 && pulseUs<=200000 && minIntervalUs>=pulseUs && minIntervalUs<=2000000 &&
           std::isfinite(phaseLeadSeconds) && phaseLeadSeconds>=0 && phaseLeadSeconds<=.03f;
  }
};

// Normal flight holds the direction marker high. During spin and its exit,
// emit one bounded high interval on entering the generalized +X phase window,
// including an already aligned mode entry. Leave the window -> low this tick.
// Azimuth is gyro-referenced, not an absolute compass direction. No delay(),
// timer ISR or telemetry dependency. Count/time survive mode exits so a 100 Hz
// recording cannot lose the existence of a sub-sample-width laser pulse.
class LaserSync {
 public:
  bool update(LaserMode mode,float phase,float relativeRateDps,uint32_t now,
              const LaserSyncConfig &cfg) {
    if (mode!=LaserMode::PhasePulse) {
      tracking_=inside_=pulsing_=false;
      return mode==LaserMode::Continuous && cfg.valid();
    }
    if (!cfg.valid() || !std::isfinite(phase) ||
        !std::isfinite(relativeRateDps)) {
      tracking_=inside_=pulsing_=false;
      return false;
    }
    phase=wrapDegrees(phase+relativeRateDps*cfg.phaseLeadSeconds);
    const bool inside=std::fabs(phase)<=cfg.alignWindowDeg;
    const bool entered=inside && (!tracking_ || !inside_);
    tracking_=true; inside_=inside;
    if (entered && (!havePulse_ || static_cast<uint32_t>(now-lastPulseUs_)>=cfg.minIntervalUs)) {
      lastPulseUs_=now; ++pulseCount_; havePulse_=pulsing_=true;
    }
    if (pulsing_ && (static_cast<uint32_t>(now-lastPulseUs_)>=cfg.pulseUs ||
                    !inside)) pulsing_=false;
    return pulsing_;
  }
  uint32_t pulseCount() const { return pulseCount_; }
  uint32_t lastPulseUs() const { return lastPulseUs_; }
 private:
  bool tracking_=false, inside_=false, pulsing_=false, havePulse_=false;
  uint32_t lastPulseUs_=0, pulseCount_=0;
};
} // namespace tricopter
