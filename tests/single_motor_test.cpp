#include "../tools/motor_bench/SingleMotor.h"
#include "../tools/motor_bench/MotorBenchSafety.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/DShotProtocol.h"
#include <cassert>
#include <cstdio>

int main() {
  motor_bench::SingleMotor motor;
  motor_bench::Safety safety;
  uint16_t values[3];
  assert(dshot::fromPulseScale(motor_bench::maxPulseUs, true) == motor.maxDigital);
  motor.route(2047, true, values);
  assert(values[0] == 0 && values[1] == 0 && values[2] == 0); // No default selection.
  for (unsigned target = 2; target <= 4; ++target) {
    assert(!motor.select(target, false)); // Cannot switch targets while unsafe/armed.
    assert(motor.select(target, true));
    assert(!motor.select(5, true));
    assert(motor.selected() == target);
    for (unsigned command = 0; command <= 2047; ++command) {
      motor.route(command, true, values);
      for (unsigned c = 0; c < 3; ++c) {
        if (c != target - 2 || command < 48) assert(values[c] == 0);
        else assert(values[c] == (command > 239 ? 239 : command));
      }
      motor.route(command, false, values);
      assert(values[0] == 0 && values[1] == 0 && values[2] == 0);
    }
  }
  assert(safety.allow(true, 1000, 1100, 100));
  assert(!safety.allow(true, 1000, 1100, 3000100));
  motor.route(239, safety.allow(true, 1000, 1100, 3000200), values);
  assert(values[0] == 0 && values[1] == 0 && values[2] == 0); // Timeout stops selected output too.
  assert(motor.select(0, false)); // Immediate deselection is always permitted.
  assert(motor.selected() == 0);
  std::puts("Single-motor routing, all command values, idle stop, selection gates and timeout stop passed");
}
