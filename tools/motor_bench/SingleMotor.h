#pragma once
#include <stdint.h>

namespace motor_bench {
// Routing only; the existing receiver, arming and three-second timeout gates
// remain mandatory in the caller. Never used by the normal flight image.
class SingleMotor {
 public:
  static constexpr uint16_t maxDigital = 239;
  bool select(unsigned motor, bool safe) {
    if (motor == 0) { selected_ = 0; return true; }
    if (!safe || motor < 2 || motor > 4) return false;
    selected_ = motor;
    return true;
  }
  unsigned selected() const { return selected_; }
  void route(uint16_t value, bool enabled, uint16_t out[3]) const {
    out[0] = out[1] = out[2] = 0;
    if (!enabled || selected_ == 0 || value < 48) return;
    out[selected_ - 2] = value > maxDigital ? maxDigital : value;
  }
 private:
  unsigned selected_ = 0;
};
} // namespace motor_bench
