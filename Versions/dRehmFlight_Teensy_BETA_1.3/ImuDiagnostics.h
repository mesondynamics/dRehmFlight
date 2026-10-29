#pragma once
#include <stdint.h>

namespace tricopter {
// Retained until reboot, so a 500 us rejection remains visible in 1 Hz STATE.
enum ImuFaultReason : uint32_t {
  imuReadFailure=1, imuZeroAcceleration=2, imuAccelerationRail=4,
  imuGyroscopeRail=8, imuFusionFailure=16
};
struct ImuDiagnostics {
  uint32_t readFailures=0, zeroFrames=0, accelerationRails=0, gyroscopeRails=0;
  uint32_t fusionFailures=0, currentReason=0, lastReason=0;
  uint32_t peakAccelerationRaw=0, peakGyroscopeRaw=0;
  void beginSample() { currentReason=0; }
  void readFailure() { ++readFailures; reject(imuReadFailure); }
  void fusionFailure() { ++fusionFailures; reject(imuFusionFailure); }
  bool checkRaw(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) {
    const int32_t a[3]={ax,ay,az}, g[3]={gx,gy,gz};
    uint32_t peakA=0,peakG=0;
    for (unsigned i=0;i<3;++i) {
      const uint32_t av=a[i]<0 ? -a[i] : a[i], gv=g[i]<0 ? -g[i] : g[i];
      if (av>peakA) peakA=av;
      if (gv>peakG) peakG=gv;
    }
    if (peakA>peakAccelerationRaw) peakAccelerationRaw=peakA;
    if (peakG>peakGyroscopeRaw) peakGyroscopeRaw=peakG;
    if (!peakA) { ++zeroFrames; currentReason|=imuZeroAcceleration; }
    if (peakA>=32760) { ++accelerationRails; currentReason|=imuAccelerationRail; }
    if (peakG>=32760) { ++gyroscopeRails; currentReason|=imuGyroscopeRail; }
    if (currentReason) lastReason=currentReason;
    return currentReason==0;
  }
 private:
  void reject(uint32_t reason) { currentReason|=reason; lastReason=currentReason; }
};
} // namespace tricopter
