#pragma once
#include <stdint.h>

// Standard, unidirectional DShot. Normal throttle frames have no telemetry request.
namespace dshot {
inline uint16_t packet(uint16_t throttle, bool requestTelemetry = false) {
  if (throttle > 2047) throttle = 2047;
  const uint16_t payload = (throttle << 1) | (requestTelemetry ? 1 : 0);
  const uint16_t checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 15;
  return (payload << 4) | checksum;
}

// Retain the flight sketch's internal 125..250 scale. A stop command or its
// minimum value maps to zero; values 1..47 (special commands) are never emitted.
inline uint16_t fromPulseScale(int pulseUs, bool enabled) {
  if (!enabled || pulseUs <= 125) return 0;
  if (pulseUs >= 250) return 2047;
  return 48 + uint32_t(pulseUs - 125) * 1999 / 125;
}

struct ZeroChannels { uint8_t bits[16]; };
inline ZeroChannels zeroChannels(uint16_t m2, uint16_t m3, uint16_t m4,
                                 uint8_t telemetryMask = 0) {
  const uint16_t packets[3] = {packet(m2, telemetryMask & 1),
                             packet(m3, telemetryMask & 2),
                             packet(m4, telemetryMask & 4)};
  ZeroChannels result = {};
  for (unsigned bit = 0; bit < 16; ++bit) {
    for (unsigned channel = 0; channel < 3; ++channel) {
      if (!(packets[channel] & (uint16_t(0x8000) >> bit)))
        result.bits[bit] |= uint8_t(1u << channel);
    }
  }
  return result;
}
} // namespace dshot
