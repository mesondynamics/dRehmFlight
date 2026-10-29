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
// Flight always uses gyro feedback; direct servo positioning is bench-only.
constexpr bool yawFeedbackEnabled = true;

// Explicit props-off servo test: CH6 scans the full range with SA cut.
// This configuration inhibits all arming; normal flight holds neutral when cut.
constexpr bool fullRangeBenchTest = false;

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

// TX16S receiver contract (verify on the receiver monitor, reverse at TX if needed):
// CH1 right=2000; CH2 forward=2000; CH3 high=2000; CH4 right=2000;
// CH6 right/CW=2000. Internal yaw is left/CCW=2000.
constexpr float stickCenterUs = 1500.0f;
constexpr float stickDeadbandUs = 20.0f;
constexpr float rollDirection = 1.0f;
constexpr float pitchDirection = 1.0f;
constexpr float spinDirection = -1.0f; // raw CH6 left -> positive/CCW
constexpr float spinEnterUs = 50.0f;
constexpr float spinExitUs = 30.0f;
constexpr float normalTiltDeg = 30.0f;
constexpr float spinTiltDeg = 15.0f;
constexpr float tiltLimitSlewDps = 15.0f;
constexpr float pilotYawMaxDps = 160.0f;
constexpr float spinMaxDps = 360.0f;
constexpr float yawSlewDps2 = 180.0f;
constexpr float spinStoppedDps = 15.0f;
constexpr float spinStoppedSeconds = .3f;
constexpr float exitBlendSeconds = 1.0f;
constexpr float faultRecoverySeconds = .3f;
constexpr float tiltIntegralLimit = .075f; // actuator-output units, per reference axis
// Spin changes the tilt dynamics. Keep full P/D feedback and lower only the
// integral bandwidth continuously, to avoid integrating gyroscopic transients.
constexpr float spinTiltIntegralScale = .1f; // at spinMaxDps, not an actuator fade
constexpr float yawIntegralLimit = .18f;   // leaves room for torque-balancing trim
constexpr float yawOutputLimit = yawServoLimitDeg / 180.0f;

// Central linkage tilts all three motors: start below the stock quad yaw gains.
// D is disabled by default because motor vibration is amplified by differentiation.
constexpr float yawKp = .12f;
constexpr float yawKi = .03f;
constexpr float yawKd = 0.0f;
constexpr float yawRateFilterHz = 12.0f;
constexpr float yawDerivativeFilterHz = 5.0f;
constexpr float yawRateDeadbandDps = .5f;
constexpr float yawServoFilterHz = 8.0f;
constexpr float yawServoSlewDegPerSec = 60.0f; // PWMServo logical degrees, not arm tilt
constexpr float yawLowThrottleGain = .2f;
constexpr float yawFeedbackStartThrottle = .08f;
constexpr float yawFeedbackFullThrottle = .25f;
constexpr float yawIntegralStartThrottle = .12f;
constexpr float servoCommandHysteresisDeg = .15f;

// Nearest-degree conversion avoids truncating 89.999 to 89 around neutral.
// The extra hysteresis prevents adjacent PWM commands chattering at half-degrees.
inline int servoCommandDeg(float angle, int previous) {
  angle=clamp(angle,0,180);
  if (angle-previous > .5f+servoCommandHysteresisDeg ||
      previous-angle > .5f+servoCommandHysteresisDeg)
    return static_cast<int>(angle+.5f);
  return previous;
}


inline float centered(float us, float deadband = stickDeadbandUs) {
  const float v = clamp(us - stickCenterUs, -500.0f, 500.0f);
  if (v > deadband) return (v-deadband)/(500.0f-deadband);
  if (v < -deadband) return (v+deadband)/(500.0f-deadband);
  return 0;
}
inline bool spinCentered(unsigned us) {
  return us >= stickCenterUs-spinExitUs && us <= stickCenterUs+spinExitUs;
}
inline bool canArm(bool receiverReady, unsigned throttleUs, unsigned cutUs, unsigned ch6Us) {
  return receiverReady && throttleUs < 1050 && cutUs < 1500 && spinCentered(ch6Us);
}

struct LimitedMix {
  MotorMix motors;
  float scale;
};
// Preserve collective and the direction of the complete roll/pitch correction.
inline LimitedMix mixLimited(float throttle, float roll, float pitch) {
  throttle = clamp(throttle, 0, 1);
  const MotorMix d = mix(0, roll, pitch);
  float scale = 1;
  const float values[] = {d.m2, d.m4, d.m3};
  for (float v : values) {
    if (v > 0) scale = clamp((1-throttle)/v, 0, scale);
    if (v < 0) scale = clamp(-throttle/v, 0, scale);
  }
  return {mix(throttle, roll*scale, pitch*scale), scale};
}

static_assert(yawFeedbackStartThrottle < yawFeedbackFullThrottle &&
              yawIntegralStartThrottle < yawFeedbackFullThrottle, "Invalid yaw throttle ramp");
static_assert(yawRateFilterHz > 0 && yawDerivativeFilterHz > 0 && yawServoFilterHz > 0 &&
              yawServoSlewDegPerSec > 0, "Invalid yaw filtering or slew limit");
static_assert(spinExitUs < spinEnterUs && spinEnterUs < 500, "Invalid CH6 hysteresis");
static_assert(spinTiltIntegralScale > 0 && spinTiltIntegralScale <= 1, "Invalid spin integral scale");
static_assert(spinMaxDps > 0 && spinMaxDps <= 360, "High-speed winged flight is not supported yet");
static_assert(servoCenterDeg - yawServoLimitDeg >= 0 && servoCenterDeg + yawServoLimitDeg <= 180,
              "Yaw servo limits must fit PWMServo's logical range");
} // namespace tricopter
