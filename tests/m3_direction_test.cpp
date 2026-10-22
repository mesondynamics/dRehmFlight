#include "../tools/motor_bench/M3Direction.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/DShotProtocol.h"
#include <cassert>
#include <cstdio>

int main() {
  motor_bench::M3Direction config;
  uint16_t v[3]; uint8_t mask; bool send;
  assert(!config.reverse(true, 2000000));
  assert(!config.frame(true, 100, v, mask, send));
  assert(!config.save(true, 1000100)); // Cannot save before temporary reversal.
  assert(!config.reverse(false, 1000100));
  assert(config.reverse(true, 1000100));
  auto burst = [&](uint32_t start, unsigned command) {
    unsigned packets = 0;
    for (unsigned step = 0; step < 20; ++step) {
      assert(config.frame(true, start + step * 500, v, mask, send));
      if (step % 2 == 0) {
        assert(send && v[0] == 0 && v[1] == command && v[2] == 0 && mask == 2);
        ++packets;
        assert(dshot::packet(v[1], true) == (command == 21 ? 0x02b9 : 0x0198));
      } else if (step < 19) assert(!send); // No interleaved zeros during repetition.
      else assert(send && v[0] == 0 && v[1] == 0 && v[2] == 0 && mask == 0);
    }
    assert(packets == 10 && config.repeats() == 10 && config.lastCommand() == command);
    assert(config.frame(false, start + 108999, v, mask, send));
    assert(send && !v[0] && !v[1] && !v[2] && !mask); // Cooldown even if SA changes.
    assert(!config.frame(true, start + 109000, v, mask, send));
  };
  burst(1000100, 21);
  assert(config.reversedTransmitted() && !config.saveTransmitted());
  assert(!config.reverse(true, 3000000)); // Prevent accidentally repeating after a save.
  assert(config.save(true, 3000000));
  burst(3000000, 12);
  assert(config.saveTransmitted());
  assert(!config.save(true, 6000000)); // No repeated EEPROM writes.
  assert(!config.reverse(true, 6000000));
  motor_bench::M3Direction abort;
  assert(!abort.frame(true, UINT32_MAX - 500000, v, mask, send));
  assert(abort.reverse(true, 500000));
  assert(abort.frame(true, 500001, v, mask, send));
  assert(abort.frame(false, 500501, v, mask, send));
  assert(send && !v[0] && !v[1] && !v[2] && !mask);
  assert(abort.aborts() == 1 && !abort.reversedTransmitted());
  assert(!abort.save(true, 2000000));
  std::puts("M3-only command packets, repeat timing, safe-idle gate, abort, cooldown and single save checks passed");
}
