// Offline response of the production transverse torque lead to a rotating vector.
// Build from the repository root with c++ -std=c++17 tools/cyclone/spin_phase_probe.cpp -o /tmp/spin_phase_probe
#include "../../Versions/dRehmFlight_Teensy_BETA_1.3/TricopterDynamics.h"

#include <cmath>
#include <complex>
#include <cstdio>

namespace {
using Complex = std::complex<double>;
constexpr double pi = 3.14159265358979323846;
constexpr double dt = 0.0005;
constexpr double lag = 0.040;

void report(const char *input, double bodyRpm, double demandHz) {
  const double yawRadS = 2 * pi * bodyRpm / 60;
  tricopter::Vec3 output{};
  Complex response{};
  constexpr int warmup = 4000, samples = 16000;
  for (int i = 0; i < warmup + samples; ++i) {
    const auto demand = std::polar(1.0, 2 * pi * demandHz * i * dt);
    const tricopter::Vec3 target{float(demand.real()), float(demand.imag()), 0};
    const auto command = tricopter::cyclicMotorLead(target, yawRadS, lag, 1);
    // Same 2 kHz first-order update as the firmware motor estimate.
    output = output + (command - output) * float(dt / (lag + dt));
    if (i >= warmup) {
      response += std::conj(demand) * Complex(output.x, output.y);
    }
  }
  response /= samples;
  std::printf("%-12s %+7.1f  %+8.3f  %+9.3f  %9.4f\n", input, bodyRpm,
              demandHz, std::arg(response) * 180 / pi, std::abs(response));
}
} // namespace

int main() {
  // Last 200 Spin samples per segment of 18e976ebc13caac0-3a, in body FLU.
  // RPM is mean GyroZ / 6. Fit the signed peak of GyroX+i*GyroY using a Hann
  // window after weighted mean removal and a -4..+4 Hz grid in 0.01 Hz steps.
  // The frequencies are representative inputs, not measured motor responses.
  struct Segment { double bodyRpm, demandHz; };
  constexpr Segment segments[] = {{37.8, -2.30}, {-35.1, 2.55}, {27.8, -2.66}};
  std::puts("input        bodyRPM  demandHz  phase_deg  amp_ratio");
  for (const auto &s : segments) {
    report("world-fixed", s.bodyRpm, -s.bodyRpm / 60);
    report("oscillating", s.bodyRpm, s.demandHz);
  }
}
