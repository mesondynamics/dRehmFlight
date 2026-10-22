#pragma once
#include <Arduino.h>
#include "DShotProtocol.h"

#if !defined(__IMXRT1062__) || !defined(ARDUINO_TEENSY40)
#error This driver is restricted to Teensy 4.0 and physical digital pins 1, 2, 3.
#endif
static_assert(F_CPU >= 300000000, "DShot bit timing requires a CPU clock of at least 300 MHz");

// Pins 1/2/3 use GPIO6 bit2 and GPIO9 bits4/5 in the installed Teensy core.
// Three parallel channels occupy one 16-bit frame. Servo timers are untouched.
class DShotTeensy4 {
 public:
  void begin(unsigned bitrate = 600000) {
    pinMode(1, OUTPUT); pinMode(2, OUTPUT); pinMode(3, OUTPUT);
    low();
    if (bitrate != 300000 && bitrate != 600000) return;
    period_ = (F_CPU_ACTUAL + bitrate / 2) / bitrate;
    zeroHigh_ = period_ * 3 / 8;
    oneHigh_ = period_ * 3 / 4;
    initialized_ = F_CPU_ACTUAL >= 300000000;
  }
  bool ready() const { return initialized_ && !fault_; }
  bool fault() const { return fault_; }
  uint16_t last[3] = {};

  void send(uint16_t m2, uint16_t m3, uint16_t m4, uint8_t telemetryMask = 0) {
    if (!ready()) { low(); last[0] = last[1] = last[2] = 0; return; }
    const auto zeros = dshot::zeroChannels(m2, m3, m4, telemetryMask);
    uint32_t clear6[16], clear9[16];
    for (unsigned bit = 0; bit < 16; ++bit) {
      clear6[bit] = zeros.bits[bit] & 1 ? CORE_PIN1_BITMASK : 0;
      clear9[bit] = (zeros.bits[bit] & 2 ? CORE_PIN2_BITMASK : 0) |
                   (zeros.bits[bit] & 4 ? CORE_PIN3_BITMASK : 0);
    }
    uint32_t savedMask;
    asm volatile("mrs %0, primask\n cpsid i" : "=r"(savedMask) :: "memory");
    const uint32_t blockedAt = ARM_DWT_CYCCNT;
    const uint32_t start = ARM_DWT_CYCCNT;
    uint32_t base = 0, late = 0;
    const bool probe = (++frameCount_ & 63u) == 0;
    uint8_t padErrors = 0;
    for (unsigned bit = 0; bit < 16; ++bit) {
      wait(start, base);
      CORE_PIN1_PORTSET = CORE_PIN1_BITMASK;
      CORE_PIN2_PORTSET = CORE_PIN2_BITMASK | CORE_PIN3_BITMASK;
      asm volatile("dsb" ::: "memory");
      noteLate(start, base, late);
      if (probe) {
        wait(start, base + period_ / 8);
        padErrors |= pads() ^ 7u;
      }
      wait(start, base + zeroHigh_);
      CORE_PIN1_PORTCLEAR = clear6[bit];
      CORE_PIN2_PORTCLEAR = clear9[bit];
      asm volatile("dsb" ::: "memory");
      noteLate(start, base + zeroHigh_, late);
      if (probe) {
        wait(start, base + period_ / 2);
        padErrors |= pads() ^ (7u ^ zeros.bits[bit]);
      }
      wait(start, base + oneHigh_);
      low();
      asm volatile("dsb" ::: "memory");
      noteLate(start, base + oneHigh_, late);
      if (probe) {
        wait(start, base + period_ * 7 / 8);
        padErrors |= pads();
      }
      if (late > period_ / 8) { fault_ = true; break; }
      base += period_;
    }
    if (!fault_) wait(start, 16 * period_);
    low();
    asm volatile("dsb" ::: "memory");
    const uint32_t frameCycles = uint32_t(ARM_DWT_CYCCNT - start);
    const uint32_t blockedCycles = uint32_t(ARM_DWT_CYCCNT - blockedAt);
    asm volatile("msr primask, %0" :: "r"(savedMask) : "memory");
    if (frameCycles > maxFrameCycles_) maxFrameCycles_ = frameCycles;
    if (blockedCycles > maxBlockedCycles_) maxBlockedCycles_ = blockedCycles;
    if (late > maxLateCycles_) maxLateCycles_ = late;
    if (probe) { ++padFrames_; padErrorMask_ |= padErrors; }
    if (padErrorMask_) fault_ = true;
    last[0] = fault_ ? 0 : m2;
    last[1] = fault_ ? 0 : m3;
    last[2] = fault_ ? 0 : m4;
  }
  uint32_t maxFrameNs() const { return ns(maxFrameCycles_); }
  uint32_t maxBlockedNs() const { return ns(maxBlockedCycles_); }
  uint32_t maxLateNs() const { return ns(maxLateCycles_); }
  uint32_t padFrames() const { return padFrames_; }
  unsigned padErrorMask() const { return padErrorMask_; }

 private:
  // Actual GPIO pad status, not the output data register. Does not measure
  // voltage margins or the waveform at the far end of the ESC signal cable.
  static inline __attribute__((always_inline)) uint8_t pads() {
    const uint32_t p6 = CORE_PIN1_PINREG, p9 = CORE_PIN2_PINREG;
    return ((p6 & CORE_PIN1_BITMASK) ? 1 : 0) |
           ((p9 & CORE_PIN2_BITMASK) ? 2 : 0) |
           ((p9 & CORE_PIN3_BITMASK) ? 4 : 0);
  }
  static inline __attribute__((always_inline)) void low() {
    CORE_PIN1_PORTCLEAR = CORE_PIN1_BITMASK;
    CORE_PIN2_PORTCLEAR = CORE_PIN2_BITMASK | CORE_PIN3_BITMASK;
  }
  static inline __attribute__((always_inline)) void wait(uint32_t start, uint32_t elapsed) {
    while (uint32_t(ARM_DWT_CYCCNT - start) < elapsed) {}
  }
  static inline __attribute__((always_inline)) void noteLate(uint32_t start, uint32_t target, uint32_t &late) {
    const uint32_t delay = uint32_t(ARM_DWT_CYCCNT - start) - target;
    if (delay > late) late = delay;
  }
  static uint32_t ns(uint32_t cycles) { return uint64_t(cycles) * 1000000000 / F_CPU_ACTUAL; }
  uint32_t period_ = 0, zeroHigh_ = 0, oneHigh_ = 0;
  uint32_t maxFrameCycles_ = 0, maxBlockedCycles_ = 0, maxLateCycles_ = 0;
  uint32_t frameCount_ = 0, padFrames_ = 0;
  uint8_t padErrorMask_ = 0;
  bool initialized_ = false, fault_ = false;
};
