#pragma once

// Proven on the connected RuiBet 65A: 300000 = DShot300, 600000 = DShot600,
// 0 = legacy OneShot125. Digital output is restricted to Teensy 4.0 pins 1/2/3.
#define DREHM_ESC_DSHOT_RATE 300000
static_assert(DREHM_ESC_DSHOT_RATE == 0 || DREHM_ESC_DSHOT_RATE == 300000 ||
              DREHM_ESC_DSHOT_RATE == 600000, "Unsupported ESC protocol");

// Top view, +X forward: M2 front-left, M4 front-right, M3 rear.
// The central servo tilts all three arms together (Cyclone mechanism).
namespace tricopter {
constexpr float rollLever = 0.8660254f; // sin(60 deg); equal arms, 120 deg spacing

struct MotorMix {
  float m2;
  float m4;
  float m3;
};

inline MotorMix mix(float throttle, float roll, float pitch) {
  return {throttle + rollLever * roll - 0.5f * pitch,
          throttle - rollLever * roll - 0.5f * pitch,
          throttle + pitch};
}

// Standard AETR: right yaw = 2000 us. dRehmFlight's +Z is up, so its
// positive yaw/GyroZ is LEFT (CCW from above); its manual expects right = 1000 us.
inline unsigned yawFromAetrUs(unsigned rawUs) { return 3000 - rawUs; }

// PWMServo uses logical 0..180 even with a physical 270-degree servo.
// Its attach endpoints are quantized to 16 us: 512..2496 us here.
constexpr int servoMinUs = 512;
constexpr int servoMaxUs = 2500;
constexpr int servoEffectiveMaxUs = 2496;
constexpr float servoCenterDeg = 90.0f; // about 1504 us; mechanical neutral must be checked
// Measured 2026-10-15: increasing PWM tilts rear M3 left (clockwise yaw thrust).
// Positive yaw PID needs M3 right, so decrease PWM for positive/CCW yaw.
constexpr float yawServoDirection = -1.0f;
constexpr float yawServoLimitDeg = 40.0f; // PWMServo units: +/-60 deg at a linear 270-deg shaft, before gearing
// Temporary first-hover isolation: CH4 directly positions the tilt servo.
// Set true and reflash to restore the previously verified gyro yaw-rate feedback.
constexpr bool yawFeedbackEnabled = false;

// Temporary request: full S1 travel ONLY with throttle cut engaged.
// After the range test, set false to restore the limited bench range.
constexpr bool fullRangeBenchTest = false; // Use the configured +/-60-deg shaft limit for S1 and flight yaw.

inline float clamp(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

inline float benchServoDeg(float ch6Us) {
  const float input = clamp((ch6Us - 1500.0f) / 500.0f, -1.0f, 1.0f);
  return servoCenterDeg + input * (fullRangeBenchTest ? 90.0f : yawServoLimitDeg);
}

inline float yawServoDeg(float yawPid) {
  return clamp(servoCenterDeg + yawServoDirection * yawPid * 180.0f,
               servoCenterDeg - yawServoLimitDeg, servoCenterDeg + yawServoLimitDeg);
}

// yawUs is the project's internal yaw channel: LEFT=2000, RIGHT=1000.
inline float manualYawServoDeg(float yawUs) {
  const float input = clamp((yawUs - 1500.0f) / 500.0f, -1.0f, 1.0f);
  return servoCenterDeg + yawServoDirection * input * yawServoLimitDeg;
}

inline float armedYawServoDeg(float yawPid, float yawUs) {
  return yawFeedbackEnabled ? yawServoDeg(yawPid) : manualYawServoDeg(yawUs);
}

inline bool canArm(bool receiverReady, unsigned throttleUs, unsigned cutUs, unsigned ch6Us) {
  const bool knobCentered = !fullRangeBenchTest || (ch6Us >= 1460 && ch6Us <= 1540);
  return receiverReady && throttleUs < 1050 && cutUs < 1500 && knobCentered;
}

static_assert(servoCenterDeg - yawServoLimitDeg >= 0 && servoCenterDeg + yawServoLimitDeg <= 180,
              "Yaw servo limits must fit PWMServo's logical range");
} // namespace tricopter
