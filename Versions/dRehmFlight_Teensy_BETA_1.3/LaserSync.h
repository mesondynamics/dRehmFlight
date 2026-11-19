#pragma once
#include "FlightMath.h"
#include <stdint.h>

namespace tricopter {
enum class LaserMode : uint8_t { Off, Continuous, PhasePulse, CenteredSpinExit };

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

// Normal flight holds the direction marker high. During spin and fast braking,
// emit one bounded high interval on entering the generalized +X phase window,
// including an already aligned mode entry. Leave the window -> low this tick.
// Azimuth is gyro-referenced, not an absolute compass direction. No delay(),
// timer ISR or telemetry dependency. Count/time survive mode exits so a 100 Hz
// recording cannot lose the existence of a sub-sample-width laser pulse.
class LaserSync {
 public:
  bool update(LaserMode mode,float phase,float relativeRateDps,uint32_t now,
              const LaserSyncConfig &cfg,float bodyRateDps=NAN,float targetRateDps=NAN) {
    if (mode!=LaserMode::Off && !cfg.valid()) mode=LaserMode::Off;
    if ((mode==LaserMode::PhasePulse || mode==LaserMode::CenteredSpinExit) &&
        (!std::isfinite(phase) || !std::isfinite(relativeRateDps))) mode=LaserMode::Off;
    if (mode==LaserMode::CenteredSpinExit &&
        (!std::isfinite(bodyRateDps) || !std::isfinite(targetRateDps))) mode=LaserMode::Off;
    if (mode==LaserMode::CenteredSpinExit) {
      mode=showBodyHeading(std::fabs(bodyRateDps),std::fabs(targetRateDps),now) ?
          LaserMode::Continuous : LaserMode::PhasePulse;
    } else {
      exitMarker_=ExitMarker::Waiting;
    }
    if (mode!=LaserMode::PhasePulse) {
      tracking_=inside_=pulsing_=false;
      return mode==LaserMode::Continuous;
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
  enum class ExitMarker : uint8_t { Waiting, Timing, Steady };
  // The beam now identifies the physical nose, even while the yaw controller
  // is still braking or aligning. These thresholds affect only the indicator.
  bool showBodyHeading(float bodySpeed,float targetSpeed,uint32_t now) {
    if (exitMarker_==ExitMarker::Steady && bodySpeed<=10 && targetSpeed<=5)
      return true;
    if (bodySpeed>5 || targetSpeed>2) {
      exitMarker_=ExitMarker::Waiting;
      return false;
    }
    if (exitMarker_==ExitMarker::Waiting) {
      exitMarker_=ExitMarker::Timing;
      exitSlowSince_=now;
    }
    if (static_cast<uint32_t>(now-exitSlowSince_)>=300000u)
      exitMarker_=ExitMarker::Steady;
    return exitMarker_==ExitMarker::Steady;
  }
  ExitMarker exitMarker_=ExitMarker::Waiting;
  uint32_t exitSlowSince_=0;
  bool tracking_=false, inside_=false, pulsing_=false, havePulse_=false;
  uint32_t lastPulseUs_=0, pulseCount_=0;
};
} // namespace tricopter
