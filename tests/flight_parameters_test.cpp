#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>
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
int main() {
  tricopter::FlightParameters p; p.begin();
  assert(!p.config().ready && !p.config().flightEnabled);
  assert(batch(p,{{1,1,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  assert(batch(p,{{3,2}},false)[8]==RMP_UNSAFE);
  assert(batch(p,{{3,2},{4,-1}})[8]==RMP_OUT_OF_RANGE);
  assert(p.config().headingKp==1.5f); // failed batch is atomic
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
  const auto revision=p.revision();
  assert(batch(p,{{16,0}})[8]==RMP_UNSAFE && p.revision()==revision);
  assert(batch(p,{{14,0,RMP_PARAM_BOOL}})[8]==RMP_UNSAFE);
  uint8_t q[16]={0},reply[RMP_PARAM_BYTES]; rmp_put_u64(q,nextId++); rmp_put_u32(q+8,p.revision());
  p.handle(RMP_PARAM_SAVE,q,sizeof(q),true,reply);
  assert(reply[8]==RMP_UNSAFE && rmp_get_u32(reply+16)==0);
  // Metadata is paged, UTF-8 safe ASCII, and reports all 69 real parameters.
  rmp_put_u64(q,nextId++); rmp_put_u32(q+8,0); q[14]=8;
  const size_t n=p.handle(RMP_PARAM_SCHEMA_PAGE,q,sizeof(q),true,reply);
  assert(n>24 && reply[8]==RMP_APPLIED && rmp_get_u16(reply+20)==69 && reply[10]==8);
  std::puts("Flight gate, paged parameters, atomic validation, conflict immunity and nonblocking save refusal passed");
}
