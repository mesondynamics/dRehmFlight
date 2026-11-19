#include <cassert>
#include <cstring>
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CrsfFlightReceiver.h"

using tricopter::CrsfArmSwitch;
using tricopter::CrsfFlightReceiver;
using Event = CrsfFlightReceiver::Event;

static const uint8_t cut[] = {0xc8,0x18,0x16,0xe0,0xa3,0x28,0x2b,0x26,0x3e,0x71,0xf0,0x81,0x0f,
                                0x7c,0xe0,0x03,0x1f,0xf8,0xc0,0x07,0x3e,0xf0,0x81,0x0f,0x7c,0x0d};
static const uint8_t arm[] = {0xc8,0x18,0x16,0xe0,0xa3,0x28,0x2b,0x26,0xce,0x0a,0xf0,0x81,0x0f,
                                0x7c,0xe0,0x03,0x1f,0xf8,0xc0,0x07,0x3e,0xf0,0x81,0x0f,0x7c,0x49};
static const uint8_t stats[] = {0xc8,0x0c,0x14,0x28,0x2a,0x63,0x03,0x01,0x02,0x03,0x2c,0x64,0x02,0xe8};
static const uint8_t shortRc[] = {0xc8,0x04,0x16,0x00,0x00,0xb9};

template <size_t N>
static Event frame(CrsfFlightReceiver &receiver, const uint8_t (&bytes)[N], uint32_t now) {
  for (size_t index = 0; index < N - 1; ++index) assert(receiver.consume(bytes[index],now) == Event::None);
  return receiver.consume(bytes[N-1],now);
}

int main() {
  CrsfFlightReceiver receiver;
  CrsfArmSwitch gate;
  const uint32_t start = UINT32_MAX - 2000;
  assert(!receiver.fresh(start));
  assert(!receiver.statisticsFresh(start));
  assert(frame(receiver,cut,start) == Event::Channels);
  const uint16_t expected[] = {1500,1693,1000,2000,2000,1500};
  assert(!memcmp(receiver.channelsUs,expected,sizeof(expected)));
  uint8_t broken[sizeof(arm)];
  memcpy(broken,arm,sizeof(arm));
  broken[12] ^= 1;
  for (uint32_t delay = 10000; delay <= 400000; delay += 10000) {
    assert(frame(receiver,broken,start+delay) == Event::None);
    assert(frame(receiver,shortRc,start+delay) == Event::None);
    assert(frame(receiver,stats,start+delay) == Event::LinkStatistics);
    assert(receiver.uplinkQuality == 99 && receiver.uplinkRssi == 42);
    assert(receiver.fresh(start+delay) == (delay <= 300000));
  }
  assert(!memcmp(receiver.channelsUs,expected,sizeof(expected)));
  memcpy(broken,arm,sizeof(arm)); broken[0] = 0xea;
  assert(frame(receiver,broken,1000000) == Event::None);
  for (unsigned index = 0; index < 9; ++index) receiver.consume(arm[index],1100000);
  assert(frame(receiver,arm,1110000) == Event::Channels);
  assert(receiver.fresh(1110000) && receiver.channelsUs[4] == 1000);
  assert(receiver.consume(0xc8,1110010) == Event::None);
  assert(receiver.consume(0xff,1110010) == Event::None);
  assert(frame(receiver,cut,1110020) == Event::Channels);
  uint8_t largest[64] = {0xc8,62,0x55};
  assert(frame(receiver,largest,1110030) == Event::None);
  uint8_t extendedRc[sizeof(cut)+2], extendedStats[sizeof(stats)+2];
  memcpy(extendedRc,cut,sizeof(cut)-1); memcpy(extendedStats,stats,sizeof(stats)-1);
  extendedRc[1]+=2; extendedStats[1]+=2;
  extendedRc[25]=0xbe; extendedRc[26]=0xef; extendedRc[27]=0x18;
  extendedStats[13]=0xbe; extendedStats[14]=0xef; extendedStats[15]=0x9e;
  assert(frame(receiver,extendedRc,1200000)==Event::Channels);
  assert(!memcmp(receiver.channelsUs,expected,sizeof(expected)));
  assert(frame(receiver,extendedStats,1300000)==Event::LinkStatistics);
  extendedStats[13]^=1;
  assert(frame(receiver,extendedStats,1400000)==Event::None);
  assert(receiver.statisticsFresh(4300000) && !receiver.statisticsFresh(4300001));
  uint8_t foreign[sizeof(arm)+4]={0xea,sizeof(arm)+2,0x55};
  memcpy(foreign+3,arm,sizeof(arm));
  assert(frame(receiver,foreign,4500000)==Event::None);
  assert(!memcmp(receiver.channelsUs,expected,sizeof(expected)));

  auto observe = [&](unsigned sa, uint32_t elapsed, unsigned throttle=1000, unsigned spin=1500) {
    gate.onChannels(throttle,sa,spin,start+elapsed,true);
  };
  observe(1000,0); observe(1000,75000); observe(1000,150000);
  assert(!gate.armed());
  observe(1700,175000); observe(1300,180000);
  for (unsigned copy=0; copy<400; ++copy) observe(1300,180000);
  assert(!gate.armed());
  observe(1300,255000); assert(!gate.armed());
  observe(1300,330000,1040); assert(gate.armed());
  observe(1300,345000,1600,2000); assert(gate.armed());
  observe(1301,350000,1600); assert(!gate.armed());
  observe(1000,355000); observe(1000,430000); observe(1000,505000);
  assert(!gate.armed());
  uint32_t base = 600000;
  for (unsigned failure=0; failure<5; ++failure,base+=500000) {
    observe(2000,base); observe(1000,base+1000);
    switch(failure) {
      case 0: observe(1000,base+101001); break;
      case 1: observe(1000,base+80000,1041); break;
      case 2: observe(1000,base+80000,1000,1531); break;
      case 3: gate.reset(); break;
      default: observe(1499,base+80000); break;
    }
    for (unsigned delay=150000;delay<=400000;delay+=25000) observe(1000,base+delay);
    assert(!gate.armed());
  }
  observe(2000,base); observe(1500,base+500); observe(1000,base+1000);
  observe(1000,base+90000); observe(1000,base+151000); assert(gate.armed());
  gate.expirePreparation(); assert(gate.armed());
  gate.onChannels(1400,1000,1500,base+151500,false); assert(gate.armed());
  observe(2000,base+152000,1800); assert(!gate.armed());
  gate.onChannels(1000,2000,1500,base+160000,false);
  for (unsigned delay=170000;delay<=470000;delay+=25000)
    gate.onChannels(1000,1000,1500,base+delay,true);
  assert(!gate.armed());
  gate.onChannels(1000,2000,1500,base+490000,true);
  gate.onChannels(1000,1000,1500,base+500000,true);
  gate.onChannels(1000,1000,1500,base+575000,true);
  gate.onChannels(1000,1000,1500,base+650000,false);
  assert(!gate.armed());
}
