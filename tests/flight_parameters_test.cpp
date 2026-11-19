#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CrsfFlightReceiver.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <vector>

struct Value {
  uint16_t id; float value; uint8_t type;
  Value(uint16_t key,float number,uint8_t kind=RMP_PARAM_F32) : id(key),value(number),type(kind) {}
};
static uint64_t nextId=1;
static std::vector<uint8_t> batch(tricopter::FlightParameters &p,
    std::initializer_list<Value> values,bool safe=true,uint64_t requestId=0) {
  std::vector<uint8_t> q(16+values.size()*8); rmp_put_u64(q.data(),requestId ? requestId : nextId++);
  rmp_put_u32(q.data()+8,p.revision()); q[12]=values.size(); unsigned i=0;
  for (const auto &v : values) {
    uint8_t *e=q.data()+16+8*i++; rmp_put_u16(e,v.id); e[2]=v.type;
    if (v.type==RMP_PARAM_F32) rmp_put_f32(e+4,v.value); else rmp_put_u32(e+4,v.value);
  }
  std::vector<uint8_t> response(RMP_PARAM_BYTES);
  const size_t n=p.handle(RMP_PARAM_SET,q.data(),q.size(),safe,response.data());
  response.resize(n); return response;
}
static std::vector<uint8_t> readAllParameters(tricopter::FlightParameters &p) {
  std::vector<uint8_t> entries;
  for (unsigned first=0;first<p.count;first+=RMP_PARAM_BATCH) {
    uint8_t q[16]={},response[RMP_PARAM_BYTES];
    rmp_put_u64(q,nextId++); rmp_put_u32(q+8,p.revision());
    rmp_put_u16(q+12,first); q[14]=RMP_PARAM_BATCH;
    const auto length=p.handle(RMP_PARAM_READ_PAGE,q,sizeof(q),true,response);
    assert(response[8]==RMP_APPLIED && rmp_get_u16(response+20)==p.count);
    assert(rmp_get_u32(response+16)==0 && rmp_get_u16(response+22)==first);
    assert(length==24+8u*response[10]);
    entries.insert(entries.end(),response+24,response+length);
  }
  assert(entries.size()==p.count*8);
  return entries;
}
static void normalHoverStartup() {
  tricopter::FlightParameters p; p.begin();
  assert(!p.applyNormalHoverAtStartup(0x1881be) && p.revision()==1);
  assert(!p.config().flightEnabled && !p.config().laserEnabled && !p.config().ready);
  assert(p.applyNormalHoverAtStartup(0x001881bf) && p.revision()==2);
  assert(p.config().ready && p.config().flightEnabled && p.config().initialModelEnabled);
  assert(!p.config().model.calibrated && !p.config().spinEnabled && p.config().laserEnabled);
  assert(p.config().motorLag==.04f && p.config().headingKp==.8f &&
         p.config().reference.normalPitchTrimDeg==-.5f && p.config().cyclicLeadScale==1);
  const auto startup=readAllParameters(p);
  // CRC32 of the 112 wire entries (ID, type, reserved, little-endian value)
  // in the 2026-11-19 v15 preflight readback. Includes unchanged defaults.
  assert(rmp_crc32(startup.data(),startup.size())==0x0da7c008u);
  tricopter::FlightParameters remote; remote.begin();
  assert(batch(remote,{{1,1,RMP_PARAM_BOOL},{75,1,RMP_PARAM_BOOL},{82,90},{83,30},{84,1},
                       {87,1,RMP_PARAM_BOOL},{97,5},{115,1},{116,4.5f},{117,-.5f}})[8]==RMP_APPLIED);
  assert(readAllParameters(remote)==startup);
  // RAM edits survive a telemetry reconnect. A new cold boot uses the fixed
  // ordinary preset again, regardless of the last session's spin command.
  assert(batch(p,{{2,1,RMP_PARAM_BOOL},{117,.5f}})[8]==RMP_APPLIED);
  p.newSession();
  assert(!p.applyNormalHoverAtStartup(0x1881bf) && p.config().spinEnabled);
  p.begin(); assert(!p.config().flightEnabled);
  assert(p.applyNormalHoverAtStartup(0x1881bf));
  assert(readAllParameters(p)==startup && !p.config().spinEnabled);
  // An already-active SA cannot start the aircraft on power-up. Only fresh
  // physical cut, low throttle, centered S1 and a confirmed arm edge can.
  tricopter::CrsfArmSwitch arm;
  for (unsigned i=0;i<5;++i) arm.onChannels(1000,1000,1500,i*50000,true);
  assert(!arm.armed());
  arm.onChannels(1000,2000,1500,250000,true);
  for (unsigned i=0;i<4;++i) arm.onChannels(1000,1000,1500,300000+i*50000,true);
  assert(arm.armed());
}
int main() {
  normalHoverStartup();
  tricopter::FlightParameters p; p.begin();
  assert(!p.config().ready && !p.config().flightEnabled);
  assert(!p.config().spinEnabled && !p.config().model.calibrated);
  assert(!p.config().initialModelEnabled && p.config().model.valid(false));
  assert(p.config().model.tiltLimitDeg==15);
  assert(p.config().model.massKg==.75f && p.config().model.izz==.0625f);
  assert(p.config().level.valid() && tricopter::FlightParameters::count==112);
  assert(p.config().reference.normalPitchTrimDeg==0);
  assert(p.config().cyclicLeadScale==0);
  for (unsigned rotor=0;rotor<3;++rotor) {
    assert(p.config().tiltMappingReady[rotor]);
    for (unsigned j=0;j<9;++j)
      assert(std::fabs(p.config().model.rotor[rotor].tiltDeg[j]-
          tricopter::servo_calibration::defaultTilt(j))<1e-5f);
  }
  // Large bench angles remain valid calibration data, without enabling flight.
  assert(batch(p,{{48,55.9f},{56,-63.6f}})[8]==RMP_APPLIED);
  assert(batch(p,{{56,-85.1f}})[8]==RMP_OUT_OF_RANGE);
  assert(batch(p,{{1,1,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  // Initial estimates enter the full dynamics controller, without marking
  // those estimates as verified. Switching the flag off cannot bypass readiness.
  assert(batch(p,{{75,1,RMP_PARAM_BOOL},{1,1,RMP_PARAM_BOOL}})[8]==RMP_APPLIED);
  assert(p.config().ready && p.config().flightEnabled && !p.config().model.calibrated);
  assert(batch(p,{{75,0,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  assert(batch(p,{{16,0}})[8]==RMP_UNSAFE);
  assert(batch(p,{{1,0,RMP_PARAM_BOOL},{75,0,RMP_PARAM_BOOL},{16,0}})[8]==RMP_APPLIED);
  assert(batch(p,{{14,1,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  assert(batch(p,{{75,1,RMP_PARAM_BOOL},{1,1,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  assert(batch(p,{{3,2}},false)[8]==RMP_UNSAFE);
  assert(batch(p,{{3,2},{4,-1}})[8]==RMP_OUT_OF_RANGE);
  assert(p.config().headingKp==.8f); // failed batch is atomic
  auto response=batch(p,{{3,1.7f}},true,9876); assert(response[8]==RMP_APPLIED);
  response=batch(p,{{3,2.5f}},true,9876);
  assert(response[8]==RMP_REQUEST_CONFLICT && p.config().headingKp==1.7f);
  assert(batch(p,{{15,1.2f},{16,.025f},{20,.25f},{21,.25f},{22,.25f},
      {26,.015f},{27,.015f},{28,.015f}})[8]==RMP_APPLIED);
  for (unsigned rotor=0;rotor<3;++rotor) {
    const uint16_t b=30+9*rotor;
    assert(batch(p,{{b,20},{static_cast<uint16_t>(b+1),15},{static_cast<uint16_t>(b+2),10},
      {static_cast<uint16_t>(b+3),5},{static_cast<uint16_t>(b+4),0},{static_cast<uint16_t>(b+5),-5},
      {static_cast<uint16_t>(b+6),-10},{static_cast<uint16_t>(b+7),-15},{static_cast<uint16_t>(b+8),-20}})[8]==RMP_APPLIED);
    const uint16_t d=60+5*rotor;
    assert(batch(p,{{d,0},{static_cast<uint16_t>(d+1),1.2f},{static_cast<uint16_t>(d+2),3.5f},
      {static_cast<uint16_t>(d+3),6},{static_cast<uint16_t>(d+4),9}})[8]==RMP_APPLIED);
  }
  assert(!p.config().ready);
  assert(batch(p,{{14,1,RMP_PARAM_BOOL}})[8]==RMP_APPLIED && p.config().ready);
  assert(batch(p,{{1,1,RMP_PARAM_BOOL}})[8]==RMP_APPLIED && p.config().flightEnabled);
  assert(p.config().spinReady && !p.config().laserEnabled);
  assert(p.config().spin.maxRateDps==15 && p.config().spin.maxAccelDps2==10);
  assert(p.config().safety.valid() && p.config().safety.stoppedRate(15)==1.5f);
  assert(batch(p,{{100,1},{101,.7f},{102,15},{103,45}})[8]==RMP_APPLIED);
  assert(p.config().safety.stopRateDps==1 && p.config().safety.stopDwellSeconds==.7f &&
      p.config().safety.exitReferenceDps==15 && p.config().safety.tiltCommandDps==45);
  const auto safetyRevision=p.revision();
  assert(batch(p,{{100,0},{101,.8f}})[8]==RMP_OUT_OF_RANGE);
  assert(batch(p,{{110,1}})[8]==RMP_OUT_OF_RANGE);
  assert(p.revision()==safetyRevision && p.config().safety.stopDwellSeconds==.7f);
  assert(batch(p,{{103,60}},false)[8]==RMP_UNSAFE);
  // Level calibration can only change at a stopped aircraft. An invalid batch
  // cannot partly change another setting or leave an invalid rotation cached.
  assert(batch(p,{{113,.1f},{114,-.2f}})[8]==RMP_APPLIED);
  const auto levelRevision=p.revision();
  assert(batch(p,{{113,0},{114,5.01f}})[8]!=RMP_APPLIED);
  assert(batch(p,{{114,-.3f}},false)[8]==RMP_UNSAFE);
  assert(batch(p,{{114,std::numeric_limits<float>::quiet_NaN()}})[8]!=RMP_APPLIED);
  assert(p.revision()==levelRevision && p.config().level.valid());
  const auto adjusted=p.config().level.attitude({1,0,0,0});
  assert(std::fabs(tricopter::Rotation(adjusted).bx.z*tricopter::degrees-.2f)<.001f);
  assert(batch(p,{{2,1,RMP_PARAM_BOOL},{87,1,RMP_PARAM_BOOL}})[8]==RMP_APPLIED);
  assert(p.config().spinEnabled && p.config().laserEnabled);
  // Cross-field pulse timing is rejected atomically; candidate decoding must
  // also be defined for malformed floating-point milliseconds before validation.
  const auto laserRevision=p.revision();
  assert(batch(p,{{85,150},{86,100}})[8]==RMP_UNSAFE);
  assert(p.revision()==laserRevision && p.config().laser.pulseUs==200000);
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::infinity(),-1.f}) {
    assert(batch(p,{{85,invalid}})[8]!=RMP_APPLIED);
    assert(batch(p,{{86,invalid}})[8]!=RMP_APPLIED);
    assert(p.revision()==laserRevision);
  }
  assert(batch(p,{{82,360},{115,1}})[8]==RMP_APPLIED);
  assert(p.config().spin.maxRateDps==360 && p.config().cyclicLeadScale==1);
  const auto spinRevision=p.revision();
  assert(batch(p,{{82,360.1f}})[8]!=RMP_APPLIED);
  assert(batch(p,{{115,1.01f}})[8]==RMP_OUT_OF_RANGE);
  assert(batch(p,{{115,.5f}},false)[8]==RMP_UNSAFE);
  assert(p.revision()==spinRevision && p.config().spin.maxRateDps==360);
  assert(batch(p,{{116,4.5f}})[8]==RMP_APPLIED);
  assert(std::fabs(p.config().laser.phaseLeadSeconds-.0045f)<1e-6f);
  const auto phaseRevision=p.revision();
  assert(batch(p,{{116,30.1f}})[8]!=RMP_APPLIED);
  assert(batch(p,{{116,5}},false)[8]==RMP_UNSAFE);
  assert(p.revision()==phaseRevision);
  assert(batch(p,{{116,30}})[8]==RMP_APPLIED && p.config().laser.valid());
  assert(batch(p,{{117,-.5f}})[8]==RMP_APPLIED);
  const auto trimRevision=p.revision();
  assert(batch(p,{{117,-2.01f}})[8]==RMP_OUT_OF_RANGE);
  assert(batch(p,{{117,.5f}},false)[8]==RMP_UNSAFE);
  assert(p.revision()==trimRevision && p.config().reference.normalPitchTrimDeg==-.5f);
  assert(batch(p,{{88,.25f},{93,.06f},{94,2},{95,.5f}})[8]==RMP_APPLIED);
  assert(p.config().rollKp==.25f && p.config().pitchKd==.06f && p.config().spinRateKp==2);
  assert(batch(p,{{2,0,RMP_PARAM_BOOL},{87,0,RMP_PARAM_BOOL}})[8]==RMP_APPLIED);
  const auto revision=p.revision();
  assert(batch(p,{{16,0}})[8]==RMP_UNSAFE && p.revision()==revision);
  assert(batch(p,{{14,0,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  uint8_t q[16]={0},reply[RMP_PARAM_BYTES]; rmp_put_u64(q,nextId++); rmp_put_u32(q+8,p.revision());
  p.handle(RMP_PARAM_SAVE,q,sizeof(q),true,reply);
  assert(reply[8]==RMP_UNSAFE && rmp_get_u32(reply+16)==0);
  // Metadata is paged and reports the complete ordinary/spin parameter set.
  rmp_put_u64(q,nextId++); rmp_put_u32(q+8,0); q[14]=8;
  const size_t n=p.handle(RMP_PARAM_SCHEMA_PAGE,q,sizeof(q),true,reply);
  assert(n>24 && reply[8]==RMP_APPLIED &&
         rmp_get_u16(reply+20)==tricopter::FlightParameters::count && reply[10]==8);
  std::puts("Flight gate, paged parameters, atomic validation, conflict immunity and nonblocking save refusal passed");
}
