#include "../Versions/dRehmFlight_Teensy_BETA_1.3/RemoteMcuTelemetry.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <deque>
#include <limits>
#include <string>
#include <vector>

struct Port {
  bool connected=true;
  int capacity=2048;
  unsigned reads=0;
  std::deque<uint8_t> input;
  std::vector<std::vector<uint8_t>> output;
  explicit operator bool() const { return connected; }
  int availableForWrite() { return capacity; }
  int available() { return input.size(); }
  int read() { const int b=input.front(); input.pop_front(); ++reads; return b; }
  size_t write(const uint8_t *p,size_t n) { output.emplace_back(p,p+n); return n; }
};

static std::string payload(const std::vector<uint8_t> &f) {
  return std::string(f.begin()+28,f.end()-4);
}
static void request(Port &port,uint8_t type,uint32_t session,bool corrupt=false) {
  const uint8_t body[]={0x3f,0x63,0,0xff}; // Includes legacy '?'/'c': never text commands.
  const rmp_frame_t f{type,session,123,5000,body,sizeof(body)};
  uint8_t wire[RMP_MAX_FRAME];
  size_t n=rmp_encode(&f,wire,sizeof(wire));
  if (corrupt) wire[n-1]^=1;
  port.input.insert(port.input.end(),wire,wire+n);
}
static void enqueue(Port &port,uint8_t type,uint32_t session,const std::vector<uint8_t> &body) {
  const rmp_frame_t frame{type,session,456,5000,body.data(),static_cast<uint16_t>(body.size())};
  uint8_t wire[RMP_MAX_FRAME]; const size_t n=rmp_encode(&frame,wire,sizeof(wire));
  assert(n); port.input.insert(port.input.end(),wire,wire+n);
}
static size_t parameters(const rmp_frame_t *f,uint8_t *reply,bool safe,void *context) {
  return static_cast<tricopter::FlightParameters *>(context)->handle(
      f->type,f->payload,f->payload_length,safe,reply);
}
static void controlIntegration() {
  Port port; remote_mcu::Telemetry telemetry; remote_mcu::Snapshot sample{};
  tricopter::FlightParameters params; params.begin();
  telemetry.setParameters(parameters,&params,tricopter::FlightParameters::count);
  uint32_t now=1000;
  const auto poll=[&](unsigned ticks) {
    for (unsigned i=0;i<ticks;++i) {
      const auto session=telemetry.session(); const auto reads=port.reads;
      telemetry.poll(port,sample,now+=500);
      if (session!=telemetry.session()) params.newSession();
      assert(port.reads-reads<=32);
    }
  };
  const auto last=[&](uint8_t kind)->std::vector<uint8_t> {
    for (auto it=port.output.rbegin();it!=port.output.rend();++it)
      if ((*it)[5]==kind) return *it;
    assert(false); return {};
  };
  poll(4);
  assert(payload(last(RMP_HELLO)).find("\"parameter_count\":69")!=std::string::npos);
  uint64_t id=10000;
  std::vector<uint8_t> set(24); rmp_put_u64(set.data(),id++);
  rmp_put_u32(set.data()+8,params.revision()); set[12]=1;
  rmp_put_u16(set.data()+16,3); set[18]=RMP_PARAM_F32; rmp_put_f32(set.data()+20,2);
  enqueue(port,RMP_PARAM_SET,telemetry.session(),set); poll(10);
  assert(last(RMP_PARAM_RESPONSE)[36]==RMP_UNSAFE && params.config().headingKp==1.5f);
  sample.safeToConfigure=true;
  rmp_put_u64(set.data(),id++);
  enqueue(port,RMP_PARAM_SET,telemetry.session(),set); poll(10);
  assert(last(RMP_PARAM_RESPONSE)[36]==RMP_APPLIED && params.config().headingKp==2);
  // Reusing an ID with a different mutation cannot apply it, through the full wire adapter.
  rmp_put_f32(set.data()+20,3); rmp_put_u32(set.data()+8,params.revision());
  enqueue(port,RMP_PARAM_SET,telemetry.session(),set); poll(10);
  assert(last(RMP_PARAM_RESPONSE)[36]==RMP_REQUEST_CONFLICT && params.config().headingKp==2);
  // Schema pages remain readable during flight, without changing any values.
  sample.safeToConfigure=false;
  std::vector<uint8_t> page(16); rmp_put_u64(page.data(),id++); page[14]=8;
  enqueue(port,RMP_PARAM_SCHEMA_PAGE,telemetry.session(),page); poll(10);
  const auto schema=last(RMP_PARAM_SCHEMA_RESPONSE);
  assert(schema[36]==RMP_APPLIED && schema[38]==8 && rmp_get_u16(schema.data()+48)==69);

  const auto command=[&](uint8_t op,uint32_t arg) {
    std::vector<uint8_t> q(16); rmp_put_u64(q.data(),id++); q[8]=op;
    rmp_put_u32(q.data()+12,arg); enqueue(port,68,telemetry.session(),q); poll(12);
    return last(69);
  };
  auto reply=command(2,1000); assert(reply[36]==RMP_UNSAFE);
  reply=command(0,0); assert(reply[36]==0 && rmp_get_u32(reply.data()+40)==100);
  sample.safeToConfigure=true;
  reply=command(2,123); assert(reply[36]==RMP_OUT_OF_RANGE);
  const uint32_t old=telemetry.session();
  reply=command(2,200);
  assert(reply[36]==0 && rmp_get_u32(reply.data()+12)==old);
  assert(rmp_get_u32(reply.data()+44)==telemetry.session() && telemetry.session()!=old);
  assert(payload(last(RMP_HELLO)).find("\"sample_rate_hz\":200")!=std::string::npos);
  poll(20);
  assert(rmp_get_u16(last(RMP_TELEMETRY).data()+36)==5000);
  // Request IDs can be reused after a new session; a late old-session write is ignored.
  rmp_put_u32(set.data()+8,params.revision());
  enqueue(port,RMP_PARAM_SET,old,set); poll(10);
  assert(params.config().headingKp==2);
  enqueue(port,RMP_PARAM_SET,telemetry.session(),set); poll(10);
  assert(params.config().headingKp==3);
  // The maximum legal atomic batch is 144 payload bytes and spans several
  // polls. It must remain supported by the application's receive bound.
  std::vector<uint8_t> full(16+16*8); rmp_put_u64(full.data(),id++);
  rmp_put_u32(full.data()+8,params.revision()); full[12]=16;
  const float defaults[]={3,4,1.5f,30,60,12,45,2,60,.08f,30,0,0,0,2,.01f};
  for (unsigned i=0;i<16;++i) {
    uint8_t *e=full.data()+16+i*8; rmp_put_u16(e,3+i);
    e[2]=i==11 ? RMP_PARAM_BOOL : RMP_PARAM_F32;
    if (i!=11) rmp_put_f32(e+4,defaults[i]);
  }
  enqueue(port,RMP_PARAM_SET,telemetry.session(),full); poll(12);
  assert(last(RMP_PARAM_RESPONSE)[36]==RMP_APPLIED && params.config().damping==.01f);
  reply=command(1,0); assert(reply[36]==0 && reply[38]==0);
  port.output.clear(); poll(2200);
  for (const auto &f : port.output) assert(f[5]!=RMP_TELEMETRY);
  auto state=last(17);
  assert(state[32]==19 && rmp_get_u16(state.data()+52)==3 && state[54]==3);
  assert(rmp_get_u32(state.data()+56)==0); // STATE ID 3 matches RemoteMCU's web badge.
  reply=command(1,1); assert(reply[36]==0 && reply[38]==1);
  state=last(17); assert(rmp_get_u32(state.data()+56)==1);
  const uint32_t enabledSession=telemetry.session();
  reply=command(1,1); assert(telemetry.session()==enabledSession); // no-op creates no segment
  // A staged command cannot cross a USB reconnection before its reply commits.
  port.capacity=0;
  std::vector<uint8_t> pending(16); rmp_put_u64(pending.data(),id++); pending[8]=2;
  rmp_put_u32(pending.data()+12,500);
  enqueue(port,68,telemetry.session(),pending); poll(5);
  port.connected=false; poll(1); port.connected=true; port.capacity=2048; poll(5);
  assert(payload(last(RMP_HELLO)).find("\"sample_rate_hz\":200")!=std::string::npos);
  // A maximum-sized corrupt frame followed by noise must resynchronize within
  // bounded receive chunks, without changing parameters or replying to noise.
  std::vector<uint8_t> noise(RMP_MAX_PAYLOAD,0x5a);
  enqueue(port,RMP_PARAM_SET,telemetry.session(),noise);
  port.input.back()^=1;
  for (unsigned i=0;i<3000;++i) port.input.push_back(0xaa);
  request(port,RMP_PING,telemetry.session());
  poll(160);
  assert(port.input.empty() && params.config().headingKp==3);
  assert(last(RMP_PONG).size()==40);
}
int main() {
  Port port;
  remote_mcu::Telemetry telemetry;
  telemetry.setDeviceId(0x1881bf);
  remote_mcu::Snapshot sample{};
  for (unsigned i=0;i<32;++i) sample.values[i]=i+.25f;
  sample.values[6]=90; sample.values[7]=1504; sample.values[18]=1; sample.values[19]=0;
  sample.values[20]=0; sample.values[21]=1; sample.values[27]=0;
  sample.safeToFlash=true;
  uint32_t now=1000;
  for (int i=0;i<50;++i,now+=500) telemetry.poll(port,sample,now);
  assert(port.output[0][5]==RMP_HELLO);
  assert(payload(port.output[0]).find("drehm-yaw-model-20261029")!=std::string::npos);
  assert(payload(port.output[0]).find("teensy40-1881bf")!=std::string::npos);
  assert(payload(port.output[0]).find("\"safe_to_flash\":true")!=std::string::npos);
  const uint32_t session=rmp_get_u32(port.output[0].data()+12);
  uint32_t expectedFrame=0;
  unsigned samples=0;
  for (const auto &f : port.output) {
    assert(rmp_get_u32(f.data()+16)==expectedFrame++);
    assert(rmp_crc32(f.data(),f.size()-4)==rmp_get_u32(f.data()+f.size()-4));
    if (f[5]!=RMP_TELEMETRY) continue;
    const uint8_t *p=f.data()+28;
    assert(rmp_get_u32(p)==1 && rmp_get_u16(p+8)==10000 && p[10]==1 && p[11]==32);
    float servo; const uint32_t bits=rmp_get_u32(p+12+6*4); std::memcpy(&servo,&bits,4);
    assert(servo==90); ++samples;
  }
  assert(samples>=2);
  // Safety state is advertised on the first poll after it changes.
  sample.safeToFlash=false;
  telemetry.poll(port,sample,now+=500);
  assert(port.output.back()[5]==RMP_HELLO);
  assert(payload(port.output.back()).find("\"safe_to_flash\":false")!=std::string::npos);
  request(port,RMP_PING,session,true);
  request(port,RMP_PING,session+1);
  request(port,65,session); // PARAM_SET is unsupported and cannot mutate this adapter.
  request(port,RMP_PING,session);
  for (int i=0;i<40;++i) {
    const unsigned before=port.reads;
    telemetry.poll(port,sample,now+=500);
    assert(port.reads-before<=32);
  }
  unsigned pongs=0;
  for (const auto &f : port.output) if (f[5]==RMP_PONG) {
    assert(rmp_get_u32(f.data()+28)==123 && f.size()==40); ++pongs;
  }
  assert(pongs==1);
  // Full queues drop samples without writes or waits. Missing samples keep
  // their sequence numbers and are reported in the next heartbeat.
  const size_t before=port.output.size();
  port.capacity=0;
  for (int i=0;i<100;++i) telemetry.poll(port,sample,now+=500);
  assert(port.output.size()==before);
  port.capacity=2048;
  sample.values[10]=std::numeric_limits<float>::quiet_NaN();
  for (int i=0;i<2200;++i) telemetry.poll(port,sample,now+=500);
  bool sawDrops=false;
  for (const auto &f : port.output) if (f[5]==RMP_HEARTBEAT && rmp_get_u32(f.data()+32)>0) sawDrops=true;
  assert(sawDrops);
  // USB reconnection creates a fresh session; micros rollover stays monotonic.
  port.connected=false; telemetry.poll(port,sample,now+=500);
  port.connected=true; telemetry.poll(port,sample,0xfffffff0u);
  const auto previous=port.output.back();
  assert(previous[5]==RMP_HELLO && rmp_get_u32(previous.data()+12)!=session);
  telemetry.poll(port,sample,0x20);
  assert(rmp_get_u64(port.output.back().data()+20)>rmp_get_u64(previous.data()+20));
  controlIntegration();
  std::puts("RemoteMCU framing, calibration snapshots, safety updates, bounded I/O, drops and reconnect passed");
}
