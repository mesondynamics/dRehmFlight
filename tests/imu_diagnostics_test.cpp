#include "../Versions/dRehmFlight_Teensy_BETA_1.3/ImuDiagnostics.h"
#include <cassert>
#include <cstdio>
using namespace tricopter;
int main() {
  ImuDiagnostics d;
  assert(d.checkRaw(0,0,16384,0,0,0));
  assert(d.peakAccelerationRaw==16384 && !d.lastReason);
  d.beginSample(); d.readFailure();
  assert(d.currentReason==imuReadFailure && d.readFailures==1);
  d.beginSample(); assert(d.checkRaw(0,0,16384,0,0,0));
  assert(d.lastReason==imuReadFailure && !d.currentReason);
  d.beginSample(); assert(!d.checkRaw(0,0,0,0,0,0));
  assert(d.currentReason==imuZeroAcceleration && d.zeroFrames==1);
  d.beginSample(); assert(!d.checkRaw(-32768,0,1,32767,0,0));
  assert(d.currentReason==(imuAccelerationRail|imuGyroscopeRail));
  assert(d.accelerationRails==1 && d.gyroscopeRails==1);
  assert(d.peakAccelerationRaw==32768 && d.peakGyroscopeRaw==32767);
  d.beginSample(); assert(d.checkRaw(32759,0,1,32759,0,0));
  d.fusionFailure();
  assert(d.fusionFailures==1 && d.lastReason==imuFusionFailure);
  d.beginSample(); assert(!d.currentReason && d.lastReason==imuFusionFailure);
  std::puts("IMU rejection causes, raw rails and retained counters passed");
}
