#pragma once
#include "src/RemoteMcu/rmp.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace remote_mcu {
constexpr unsigned channelCount = 32;
constexpr unsigned stateCount = 34;
constexpr const char *buildId = "drehm-yaw-fault-diag-20261029";

struct Snapshot {
  float values[channelCount];
  bool safeToFlash;
  bool safeToConfigure=false;
  uint32_t state[stateCount]{};
};

// Cooperative RMP transport. No direct actuator or EEPROM/reset commands.
// One transmit attempt and <=32 received bytes per
// call; slow/disconnected hosts cannot stall the flight loop.
class Telemetry {
 public:
  using RequestHandler=size_t (*)(const rmp_frame_t *,uint8_t *,bool,void *);
  void setParameters(RequestHandler handler,void *context,unsigned count) {
    handler_=handler; context_=context; parameterCount_=count;
  }
  uint32_t session() const { return session_; }
  void setDeviceId(uint32_t suffix) {
    std::snprintf(deviceId_, sizeof(deviceId_), "teensy40-%06lx",
                  static_cast<unsigned long>(suffix & 0xffffffu));
  }

  template<class SerialPort>
  void poll(SerialPort &port, const Snapshot &snapshot, uint32_t now) {
    clock_ += static_cast<uint32_t>(now-previousMicros_);
    previousMicros_=now;
    const bool connected=static_cast<bool>(port);
    if (!connected) { wasConnected_=false; return; }
    if (!wasConnected_) {
      const uint32_t previous=session_;
      session_=now ^ (++generation_*0x9e3779b9u);
      if (!session_ || session_==previous) session_=previous+1u;
      if (!session_) session_=1;
      frameSequence_=sampleSequence_=droppedSamples_=0;
      lastSample_=now;
      management_=1;
      replySize_=0;
      pendingSession_=0;
      resetDecoder();
    }
    wasConnected_=true;
    if (snapshot.safeToFlash!=lastSafe_) management_=1;
    lastSafe_=snapshot.safeToFlash;
    safeToConfigure_=snapshot.safeToConfigure;
    if (!management_ && static_cast<uint32_t>(now-lastManagement_)>=1000000u)
      management_=1;

    // Identity changes (especially safe_to_flash) take priority over telemetry.
    if (management_) {
      sendManagement(port,snapshot);
      if (!management_) lastManagement_=now;
    } else if (replySize_) {
      if (send(port,replyType_,reply_,replySize_)) {
        replySize_=0;
        if (pendingSession_) {
          session_=pendingSession_; pendingSession_=0;
          sampleRate_=pendingRate_; streamEnabled_=pendingEnabled_;
          frameSequence_=sampleSequence_=droppedSamples_=0;
          lastSample_=now; management_=1;
          resetDecoder();
        }
      }
    } else if (streamEnabled_ && static_cast<uint32_t>(now-lastSample_)>=1000000u/sampleRate_) {
      const uint32_t period=1000000u/sampleRate_;
      const uint32_t elapsed=static_cast<uint32_t>(now-lastSample_)/period;
      lastSample_+=elapsed*period;
      sampleSequence_+=elapsed-1u;
      droppedSamples_+=elapsed-1u;
      uint8_t payload[12+channelCount*4];
      rmp_put_u32(payload,1); // RemoteMCU v0.3's fixed float-vector wire schema
      rmp_put_u32(payload+4,sampleSequence_++);
      rmp_put_u16(payload+8,period);
      payload[10]=1; payload[11]=channelCount;
      for (unsigned i=0;i<channelCount;++i)
        rmp_put_f32(payload+12+4*i,std::isfinite(snapshot.values[i]) ? snapshot.values[i] : 0);
      if (!send(port,RMP_TELEMETRY,payload,sizeof(payload))) ++droppedSamples_;
    }

    uint8_t input[32];
    unsigned count=0;
    while (count<sizeof(input) && port.available()>0) {
      const int byte=port.read();
      if (byte<0) break;
      input[count++]=static_cast<uint8_t>(byte);
    }
    if (count) rmp_decoder_feed(&decoder_,input,count,receive,this);
  }

 private:
  void resetDecoder() {
    rmp_decoder_init(&decoder_);
    // Largest supported request: SET header + 16 value entries. Reject larger
    // lengths at the header, so corrupt traffic cannot trigger a 1 KiB CRC / scan.
    decoder_.max_payload=16+16*8;
  }
  template<class SerialPort>
  bool send(SerialPort &port, uint8_t type, const uint8_t *payload, uint16_t size) {
    const unsigned length=RMP_HEADER_SIZE+size+4;
    if (port.availableForWrite()<static_cast<int>(length)) return false;
    uint8_t wire[RMP_MAX_FRAME];
    const rmp_frame_t frame{type,session_,frameSequence_,clock_,payload,size};
    const size_t encoded=rmp_encode(&frame,wire,sizeof(wire));
    if (!encoded || port.write(wire,encoded)!=encoded) return false;
    ++frameSequence_;
    return true;
  }

  template<class SerialPort>
  void sendManagement(SerialPort &port, const Snapshot &s) {
    if (management_==1) {
      char hello[640];
      const int n=std::snprintf(hello,sizeof(hello),
        "{\"target\":\"teensy40\",\"build_id\":\"%s\",\"schema_id\":1,"
        "\"channels\":32,\"sample_rate_hz\":%lu,\"samples_per_frame\":1,"
        "\"stream_enabled\":%s,\"source\":\"drehmflight-control\","
        "\"safe_to_flash\":%s,\"device_id\":\"%s\","
        "\"capabilities\":[\"telemetry\",\"state\",\"parameters\",\"stream-control\"],"
        "\"parameter_count\":%u,\"parameter_paging\":true}",
        buildId,static_cast<unsigned long>(sampleRate_),streamEnabled_ ? "true" : "false",
        s.safeToFlash ? "true" : "false",deviceId_,parameterCount_);
      if (n>0 && n<static_cast<int>(sizeof(hello)) &&
          send(port,RMP_HELLO,reinterpret_cast<const uint8_t *>(hello),n)) ++management_;
    } else if (management_==2) {
      uint8_t heartbeat[12];
      rmp_put_u32(heartbeat,sampleSequence_);
      rmp_put_u32(heartbeat+4,droppedSamples_);
      rmp_put_u32(heartbeat+8,sampleSequence_);
      if (send(port,RMP_HEARTBEAT,heartbeat,sizeof(heartbeat))) ++management_;
    } else {
      // State IDs match this firmware's supplied application profile only.
      uint8_t state[8+stateCount*8]={0};
      rmp_put_u32(state,1); state[4]=stateCount;
      for (unsigned i=0;i<stateCount;++i) {
        uint8_t *entry=state+8+8*i;
        rmp_put_u16(entry,i+1);
        entry[2]=(i<6 || i==8 || i==9 || i==12 || i==13 || (i>=18 && i<=21)) ? 3 : 2;
        uint32_t bits=s.state[i];
        if (i==2) bits=streamEnabled_; // Current RemoteMCU web expects STATE ID 3.
        if (i==4) bits=s.safeToFlash;
        if (i==6) bits=droppedSamples_;
        if (i==7) bits=sampleRate_;
        rmp_put_u32(entry+4,bits);
      }
      if (send(port,17,state,sizeof(state))) management_=0;
    }
  }

  static void receive(const rmp_frame_t *frame,const uint8_t *,size_t,void *context) {
    Telemetry &self=*static_cast<Telemetry *>(context);
    if (frame->boot_id!=self.session_ || self.replySize_) return;
    if (frame->type==68) { self.deviceCommand(*frame); return; }
    if (self.handler_ && (frame->type==64 || frame->type==65 || frame->type==66 ||
                         frame->type==70 || frame->type==72)) {
      self.replySize_=self.handler_(frame,self.reply_,self.safeToConfigure_,self.context_);
      self.replyType_=frame->type==70 ? 71 : frame->type==72 ? 73 : 67;
      return;
    }
    if (frame->type!=RMP_PING || frame->payload_length>64) return;
    rmp_put_u32(self.reply_,frame->sequence);
    if (frame->payload_length) std::memcpy(self.reply_+4,frame->payload,frame->payload_length);
    self.replySize_=frame->payload_length+4; self.replyType_=RMP_PONG;
  }

  void deviceCommand(const rmp_frame_t &f) {
    if (f.payload_length!=16) return;
    const uint8_t *q=f.payload; const uint8_t op=q[8];
    const uint32_t arg=rmp_get_u32(q+12);
    uint8_t status=0; uint32_t rate=sampleRate_; bool enabled=streamEnabled_;
    if (q[9] || q[10] || q[11] || op>3 || ((op==0 || op==3) && arg)) status=10;
    else if ((op==1 || op==2) && !safeToConfigure_) status=6;
    else if (op==1) { if (arg>1) status=4; else enabled=arg!=0; }
    else if (op==2) { if (arg!=100 && arg!=200 && arg!=500 && arg!=1000) status=4; else rate=arg; }
    else if (op==3) status=6; // No log producer is advertised by this application.
    std::memset(reply_,0,24); std::memcpy(reply_,q,8);
    reply_[8]=status; reply_[9]=op; reply_[10]=enabled;
    if (!status && (enabled!=streamEnabled_ || rate!=sampleRate_)) {
      pendingSession_=session_+1; if (!pendingSession_) pendingSession_=1;
      pendingRate_=rate; pendingEnabled_=enabled;
    }
    rmp_put_u32(reply_+12,rate);
    rmp_put_u32(reply_+16,pendingSession_ ? pendingSession_ : session_);
    replySize_=24; replyType_=69;
  }

  rmp_decoder_t decoder_{};
  uint64_t clock_=0;
  uint32_t previousMicros_=0,session_=0,generation_=0,frameSequence_=0;
  uint32_t sampleSequence_=0,droppedSamples_=0,lastSample_=0,lastManagement_=0;
  bool wasConnected_=false,lastSafe_=false;
  unsigned management_=0;
  char deviceId_[32]="teensy40-unknown";
  uint8_t reply_[568]{};
  uint16_t replySize_=0;
  uint8_t replyType_=0;
  RequestHandler handler_=nullptr;
  void *context_=nullptr;
  unsigned parameterCount_=0;
  bool safeToConfigure_=false,streamEnabled_=true,pendingEnabled_=true;
  uint32_t sampleRate_=100,pendingRate_=100,pendingSession_=0;
};
} // namespace remote_mcu
