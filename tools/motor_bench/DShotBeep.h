#pragma once
#include <stdint.h>

namespace motor_bench {
// Acoustic BEEP1 only: no throttle, direction changes or persistent settings.
// State machine runs once per frame, never delays the control loop.
class DShotBeep {
 public:
  bool request(unsigned motor, bool safe, uint32_t now) {
    if (motor < 2 || motor > 4 || !safe || !idle_ || cooling_ || pending_ ||
        uint32_t(now - idleSince_) < 1000000) return false;
    pending_ = motor;
    return true;
  }
  // true means this frame must override all normal throttle values.
  bool frame(bool safe, uint32_t now, uint16_t values[3], uint8_t &telemetryMask) {
    values[0] = values[1] = values[2] = 0;
    telemetryMask = 0;
    if (!safe) { idle_ = false; pending_ = 0; }
    else if (!idle_) { idle_ = true; idleSince_ = now; }
    if (cooling_) {
      if (uint32_t(now - beepAt_) < 500000) return true;
      cooling_ = false;
    }
    if (!pending_) return false;
    const unsigned channel = pending_ - 2;
    values[channel] = 1; // DSHOT_CMD_BEEP1, one command frame.
    telemetryMask = uint8_t(1u << channel); // Command request bit, as in Betaflight.
    lastMotor_ = pending_;
    pending_ = 0;
    ++count_;
    cooling_ = true;
    beepAt_ = now;
    return true;
  }
  unsigned lastMotor() const { return lastMotor_; }
  uint32_t count() const { return count_; }

 private:
  bool idle_ = false, cooling_ = false;
  uint32_t idleSince_ = 0, beepAt_ = 0, count_ = 0;
  unsigned pending_ = 0, lastMotor_ = 0;
};
} // namespace motor_bench
