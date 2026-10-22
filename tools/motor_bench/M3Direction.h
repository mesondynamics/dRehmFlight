#pragma once
#include <stdint.h>

namespace motor_bench {
// Only M3 (array index 1). R = temporary reverse of stored direction (21),
// S = save current ESC settings (12), available only after R was transmitted.
// No automatic startup command, arbitrary ESC setting or throttle override.
class M3Direction {
 public:
  bool reverse(bool safe, uint32_t now) {
    if (!qualified(safe, now) || reversed_ || saveAttempted_) return false;
    start(21);
    return true;
  }
  bool save(bool safe, uint32_t now) {
    if (!qualified(safe, now) || !reversed_ || saveAttempted_) return false;
    saveAttempted_ = true;
    start(12);
    return true;
  }
  // Return true to suppress all normal output. sendFrame=false leaves pins low
  // between command packets, so zero packets cannot reset the ESC repeat count.
  bool frame(bool safe, uint32_t now, uint16_t values[3], uint8_t &mask, bool &sendFrame) {
    values[0] = values[1] = values[2] = 0;
    mask = 0; sendFrame = true;
    if (!safe) idle_ = false;
    else if (!idle_) { idle_ = true; idleSince_ = now; }
    if (cooling_) {
      if (uint32_t(now - coolAt_) < 100000) return true;
      cooling_ = false;
    }
    if (!command_) return false;
    if (!safe) {
      command_ = 0; ++aborts_;
      cooling_ = true; coolAt_ = now;
      return true;
    }
    if (sent_ && uint32_t(now - sentAt_) < 1000) {
      sendFrame = false;
      return true;
    }
    values[1] = command_; mask = 2;
    sentAt_ = now;
    if (++sent_ == 10) {
      if (command_ == 21) reversed_ = true;
      else saved_ = true;
      command_ = 0;
      cooling_ = true; coolAt_ = now;
    }
    return true;
  }
  unsigned lastCommand() const { return lastCommand_; }
  unsigned repeats() const { return sent_; }
  bool reversedTransmitted() const { return reversed_; }
  bool saveTransmitted() const { return saved_; }
  bool busy() const { return command_ || cooling_; }
  uint32_t aborts() const { return aborts_; }
 private:
  bool qualified(bool safe, uint32_t now) const {
    return safe && idle_ && !busy() && uint32_t(now - idleSince_) >= 1000000;
  }
  void start(unsigned command) { command_ = lastCommand_ = command; sent_ = 0; }
  bool idle_ = false, cooling_ = false, reversed_ = false, saved_ = false;
  bool saveAttempted_ = false;
  unsigned command_ = 0, lastCommand_ = 0, sent_ = 0;
  uint32_t idleSince_ = 0, sentAt_ = 0, coolAt_ = 0, aborts_ = 0;
};
} // namespace motor_bench
