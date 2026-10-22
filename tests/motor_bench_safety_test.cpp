#include "../tools/motor_bench/MotorBenchSafety.h"
#include <cassert>
#include <cstdio>

int main() {
  motor_bench::Safety safety;
  for (int pulse = 0; pulse <= 300; ++pulse) {
    assert(safety.capPulse(pulse) >= 125);
    assert(safety.capPulse(pulse) <= 137);
  }
  assert(!safety.allow(false, 1000, 1100, 100));
  assert(safety.allow(true, 1000, 1000, 200));
  assert(safety.allow(true, 1000, 1100, 300));
  assert(safety.allow(true, 1000, 1100, 3000299));
  assert(!safety.allow(true, 1000, 1100, 3000300));
  assert(safety.timedOut());
  assert(!safety.allow(true, 1000, 1000, 3000400)); // Low throttle must not rearm after timeout.
  assert(!safety.allow(true, 2000, 1000, 3000500));
  assert(!safety.timedOut());
  assert(safety.allow(true, 1000, 1000, 3000600));
  assert(safety.allow(true, 1000, 1100, UINT32_MAX - 1000));
  assert(safety.allow(true, 1000, 1100, 1000)); // Timer rollover remains within the run window.
  assert(!safety.allow(true, 1000, 1100, 2998999));
  std::puts("Bench output cap, timeout latch, switch reset, and timer rollover checks passed");
}
