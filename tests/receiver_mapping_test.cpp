#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace tricopter;

static PilotInput decode(const uint16_t raw[6]) {
  uint16_t mapped[6]{};
  mapCrsfChannels(raw,mapped);
  return pilotInput(mapped[0],mapped[1],mapped[2],mapped[3],mapped[5],false);
}

int main() {
  const uint16_t raw[]={1100,1200,1300,1400,1900,1600};
  uint16_t mapped[6]{};
  mapCrsfChannels(raw,mapped);
  assert(mapped[0]==1300 && mapped[1]==1400 && mapped[2]==1200);
  assert(mapped[3]==1900 && mapped[4]==1900 && mapped[5]==1600);

  uint16_t neutral[]={1500,1500,1000,1500,2000,1500};
  auto p=decode(neutral);
  assert(p.throttle==0 && p.roll==0 && p.pitch==0 && p.yaw==0 && !p.spinRequested);
  for (unsigned channel=0;channel<4;++channel) {
    for (uint16_t value : {uint16_t(1000),uint16_t(2000)}) {
      uint16_t isolated[]={1500,1500,1000,1500,2000,1500};
      isolated[channel]=value;
      p=decode(isolated);
      if (channel==0) {
        assert(p.yaw==(value==1000 ? 1.f : -1.f));
        assert(p.roll==0 && p.pitch==0 && p.throttle==0);
      } else if (channel==1) {
        assert(p.pitch==(value==2000 ? 1.f : -1.f));
        assert(p.roll==0 && p.yaw==0 && p.throttle==0);
      } else if (channel==2) {
        assert(p.throttle==(value==2000 ? 1.f : 0.f));
        assert(p.roll==0 && p.pitch==0 && p.yaw==0);
      } else {
        assert(p.roll==(value==2000 ? 1.f : -1.f));
        assert(p.pitch==0 && p.yaw==0 && p.throttle==0);
      }
      mapCrsfChannels(isolated,mapped);
      assert(mapped[4]==2000 && mapped[5]==1500 && !p.spinRequested);
    }
  }
  std::puts("CH1 yaw / CH4 roll routing, direction and channel isolation passed");
}
