#pragma once
#include <stdint.h>

// Used only by the isolated bench image, never by the normal flight sketch.
namespace motor_bench {
constexpr int maxPulseUs = 137;
constexpr uint32_t maxRunUs = 3000000;

class Safety {
 public:
  bool allow(bool armed, unsigned cutUs, unsigned throttleUs, uint32_t nowUs) {
    if (cutUs >= 1500) {
      active_ = timedOut_ = false;
      return false;
    }
    if (!armed || timedOut_) return false;
    if (!active_ && throttleUs > 1040) {
      active_ = true;
      started_ = nowUs;
    }
    if (active_ && uint32_t(nowUs - started_) >= maxRunUs) {
      timedOut_ = true;
      return false;
    }
    return true;
  }
  bool timedOut() const { return timedOut_; }
  int capPulse(int pulseUs) const {
    return pulseUs < 125 ? 125 : (pulseUs > maxPulseUs ? maxPulseUs : pulseUs);
  }

 private:
  uint32_t started_ = 0;
  bool active_ = false;
  bool timedOut_ = false;
};
} // namespace motor_bench
