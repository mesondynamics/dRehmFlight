#include "../Versions/dRehmFlight_Teensy_BETA_1.3/DShotProtocol.h"
#include <cassert>
#include <cstdio>

int main() {
  assert(dshot::packet(0) == 0x0000);
  assert(dshot::packet(48) == 0x0606);
  assert(dshot::packet(1046) == 0x82c6); // Published Betaflight example.
  assert(dshot::packet(2047) == 0xffee);
  assert(dshot::fromPulseScale(250, false) == 0);
  assert(dshot::fromPulseScale(120, true) == 0);
  assert(dshot::fromPulseScale(125, true) == 0);
  assert(dshot::fromPulseScale(137, true) == 239);
  assert(dshot::fromPulseScale(250, true) == 2047);
  for (int pulse = 0; pulse <= 300; ++pulse) {
    const auto value = dshot::fromPulseScale(pulse, true);
    assert(value == 0 || (value >= 48 && value <= 2047));
  }
  for (unsigned value = 0; value <= 2047; ++value) {
    const uint16_t word = dshot::packet(value);
    assert((word >> 5) == value);
    assert(!(word & 16)); // No telemetry request.
    assert(((word >> 12) ^ (word >> 8) ^ (word >> 4) ^ word) % 16 == 0);
    const uint16_t targets[3] = {uint16_t(value), uint16_t(2047-value), 239};
    const auto zeros = dshot::zeroChannels(targets[0], targets[1], targets[2]);
    for (unsigned channel = 0; channel < 3; ++channel) {
      uint16_t reconstructed = 0;
      for (unsigned bit = 0; bit < 16; ++bit)
        reconstructed = (reconstructed << 1) | !(zeros.bits[bit] & (1u << channel));
      assert(reconstructed == dshot::packet(targets[channel]));
    }
  }
  std::puts("DShot published vectors, stop mapping, reserved-command exclusion and three-channel bit order passed");
}
