#include "../tools/motor_bench/DShotBeep.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/DShotProtocol.h"
#include <cassert>
#include <cstdio>

int main() {
  motor_bench::DShotBeep beep;
  uint16_t values[3]; uint8_t mask;
  assert(!beep.request(2, true, 2000000)); // Must observe safe idle first.
  assert(!beep.frame(true, 100, values, mask));
  assert(!beep.request(2, true, 1000099));
  assert(!beep.request(2, false, 1000100));
  assert(!beep.request(1, true, 1000100));
  for (unsigned motor = 2; motor <= 4; ++motor) {
    const uint32_t now = 1000100 + (motor - 2) * 2000000;
    assert(beep.request(motor, true, now));
    assert(beep.frame(true, now, values, mask));
    assert(beep.lastMotor() == motor && beep.count() == motor - 1);
    for (unsigned channel = 0; channel < 3; ++channel)
      assert(values[channel] == (channel == motor - 2 ? 1 : 0));
    assert(mask == (1u << (motor - 2)));
    const auto zeros = dshot::zeroChannels(values[0], values[1], values[2], mask);
    for (unsigned channel = 0; channel < 3; ++channel) {
      uint16_t word = 0;
      for (unsigned bit = 0; bit < 16; ++bit)
        word = (word << 1) | !(zeros.bits[bit] & (1u << channel));
      assert(word == (channel == motor - 2 ? 0x0033 : 0)); // BEEP1 + request bit + CRC.
    }
    assert(!beep.request(motor, true, now + 1));
    assert(beep.frame(false, now + 499999, values, mask)); // Even unexpected arm keeps outputs zero.
    assert(values[0] == 0 && values[1] == 0 && values[2] == 0 && mask == 0);
    assert(!beep.frame(true, now + 500000, values, mask));
    // Safe idle was interrupted; qualify again for the next command.
    assert(!beep.request(motor, true, now + 999999));
    assert(!beep.frame(true, now + 1500000, values, mask));
  }
  motor_bench::DShotBeep cancel;
  assert(!cancel.frame(true, UINT32_MAX - 500000, values, mask));
  assert(cancel.request(2, true, 500000)); // Qualification across timer wrap.
  assert(!cancel.frame(false, 500001, values, mask)); // Unsafe before transmission cancels request.
  assert(cancel.count() == 0);
  std::puts("DShot acoustic command framing, idle qualification, cancellation and stop cooldown passed");
}
