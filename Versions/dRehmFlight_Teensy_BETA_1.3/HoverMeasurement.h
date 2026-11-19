#pragma once
#include "FlightMath.h"

namespace tricopter {

// Observation only. Both channels use the same three 12 Hz poles before
// decimation from the flight loop to the 100 Hz recording.
class HoverMeasurement {
 public:
  void update(bool healthy, Quaternion sensorAttitude, Vec3 sensorAccelG,
              float meanDshot, float dt) {
    // Match the accelerometer axes supplied to the raw Madgwick estimator.
    const float up = Rotation(sensorAttitude)
                         .world({-sensorAccelG.x, sensorAccelG.y, sensorAccelG.z}).z;
    if (!healthy || !std::isfinite(up) || !(dt > 0 && dt <= .01f)) {
      initialized_ = false;
      return;
    }
    Pair sample{up, meanDshot};
    const float wdt = 75.39822369f * dt;  // 2*pi*12*dt
    const float alpha = wdt / (1.0f + wdt);
    for (auto &stage : stages_) {
      if (initialized_) {
        stage.up += alpha * (sample.up - stage.up);
        stage.code += alpha * (sample.code - stage.code);
      } else {
        stage = sample;
      }
      sample = stage;
    }
    initialized_ = true;
  }
  float upG() const { return initialized_ ? stages_[2].up : 0; }
  float collectiveCode() const { return initialized_ ? stages_[2].code : 0; }

 private:
  struct Pair { float up, code; };
  Pair stages_[3]{};
  bool initialized_ = false;
};

}  // namespace tricopter
