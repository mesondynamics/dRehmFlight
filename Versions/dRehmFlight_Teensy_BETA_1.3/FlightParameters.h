#pragma once
#include "NormalYawControl.h"
#include "src/RemoteMcu/parameters.h"
#include <cstdio>
#include <cstring>

namespace tricopter {
class FlightParameters {
 public:
  static constexpr unsigned count=69;
  void begin() {
    added_=0;
    add(1,"flight_enable",RMP_PARAM_BOOL,0,1,0);
    add(2,"spin_enable",RMP_PARAM_BOOL,0,1,0);
    add(3,"heading_kp_1_s",RMP_PARAM_F32,.05,10,1.5);
    add(4,"rate_kp_1_s",RMP_PARAM_F32,.05,30,4);
    add(5,"rate_ki_1_s2",RMP_PARAM_F32,0,30,1.5);
    add(6,"yaw_max_deg_s",RMP_PARAM_F32,1,160,30);
    add(7,"yaw_accel_deg_s2",RMP_PARAM_F32,5,360,60);
    add(8,"yaw_lpf_hz",RMP_PARAM_F32,1,50,12);
    add(9,"heading_error_max_deg",RMP_PARAM_F32,1,90,45);
    add(10,"anti_windup_1_s",RMP_PARAM_F32,0,20,2);
    add(11,"servo_slew_logical_deg_s",RMP_PARAM_F32,1,120,60);
    add(12,"servo_lag_s",RMP_PARAM_F32,.01,.5,.08);
    add(13,"physical_tilt_limit_deg",RMP_PARAM_F32,1,45,30);
    add(14,"model_verified",RMP_PARAM_BOOL,0,1,0);
    add(15,"mass_kg",RMP_PARAM_F32,0,20,0);
    add(16,"izz_kg_m2",RMP_PARAM_F32,0,2,0);
    add(17,"yaw_i_accel_max_rad_s2",RMP_PARAM_F32,.01,20,2);
    add(18,"yaw_damping_Nm_s_rad",RMP_PARAM_F32,0,1,0);
    const char *geometry[]={"arm_M2_m","arm_M4_m","arm_M3_m","cg_x_m","cg_y_m",
      "rotor_height_from_cg_m","kappa_M2_m","kappa_M4_m","kappa_M3_m"};
    for (unsigned i=0;i<9;++i)
      add(20+i,geometry[i],RMP_PARAM_F32,i>=3 && i<=5 ? -1 : 0,i<3 ? 2 : 1,0);
    const unsigned motors[]={2,4,3};
    for (unsigned r=0;r<3;++r) for (unsigned j=0;j<9;++j) {
      char name[48]; std::snprintf(name,sizeof(name),"M%u_tilt_at_servo_%u_deg",motors[r],50+10*j);
      add(30+9*r+j,name,RMP_PARAM_F32,-45,45,0);
    }
    for (unsigned r=0;r<3;++r) for (unsigned j=0;j<5;++j) {
      char name[48]; std::snprintf(name,sizeof(name),"M%u_thrust_at_%u_pct_N",motors[r],25*j);
      add(60+5*r+j,name,RMP_PARAM_F32,0,100,0);
    }
    rmp_params_init(&parameters_,definitions_,count,nullptr,nullptr);
    rebuild();
  }
  const NormalYawConfig &config() const { return config_; }
  uint32_t revision() const { return parameters_.revision; }
  void newSession() { // Request IDs are scoped to the RMP session.
    for (auto &entry : parameters_.cache) entry.valid=false;
    parameters_.next_cache=0;
  }
  size_t handle(uint8_t kind,const uint8_t *request,size_t length,bool stopped,uint8_t *response) {
    bool safe=stopped && kind!=RMP_PARAM_SAVE; // Never erase/program flash in a 500 us control loop.
    if (!stopped) safe=false;
    if (kind==RMP_PARAM_SET && safe && length>=16 && length==16+request[12]*8u) {
      uint32_t candidate[RMP_PARAM_MAX]; std::memcpy(candidate,parameters_.values,sizeof(candidate));
      for (unsigned i=0;i<request[12];++i) {
        const uint8_t *e=request+16+8*i;
        for (unsigned j=0;j<count;++j) if (definitions_[j].id==rmp_get_u16(e)) candidate[j]=rmp_get_u32(e+4);
      }
      const NormalYawConfig next=decode(candidate);
      if ((candidate[13] && !next.ready) || (candidate[0] && !next.ready)) safe=false;
    }
    const uint32_t before=parameters_.revision;
    const size_t size=rmp_params_handle(&parameters_,kind,request,length,safe,response);
    if (parameters_.revision!=before) rebuild();
    return size;
  }
 private:
  static float number(const uint32_t *values,unsigned i) {
    float f; std::memcpy(&f,values+i,4); return f;
  }
  NormalYawConfig decode(const uint32_t *v) const {
    NormalYawConfig c;
    c.flightEnabled=v[0]; c.spinEnabled=v[1]; c.headingKp=number(v,2); c.rateKp=number(v,3);
    c.rateKi=number(v,4); c.maxRateDps=number(v,5); c.maxAccelDps2=number(v,6);
    c.rateFilterHz=number(v,7); c.maxHeadingError=number(v,8); c.antiWindupGain=number(v,9);
    c.servoSlew=number(v,10); c.servoLag=number(v,11); c.model.tiltLimitDeg=number(v,12);
    c.model.calibrated=v[13]; c.model.massKg=number(v,14); c.model.izz=number(v,15);
    c.integralLimit=number(v,16); c.damping=number(v,17);
    const float cx=number(v,21),cy=number(v,22),z=number(v,23);
    const float a=number(v,18),b=number(v,19),d=number(v,20);
    c.model.rotor[0].position={.5f*a-cx,rollLever*a-cy,z};
    c.model.rotor[1].position={.5f*b-cx,-rollLever*b-cy,z};
    c.model.rotor[2].position={-d-cx,-cy,z};
    c.model.rotor[0].tangent={-rollLever,.5f,0};
    c.model.rotor[1].tangent={rollLever,.5f,0};
    c.model.rotor[2].tangent={0,-1,0};
    for (unsigned r=0;r<3;++r) {
      auto &m=c.model.rotor[r]; m.reactionSign=r==1 ? 1 : -1; m.kappa=number(v,24+r);
      for (unsigned j=0;j<9;++j) m.tiltDeg[j]=number(v,27+9*r+j);
      for (unsigned j=0;j<5;++j) m.thrustN[j]=number(v,54+5*r+j);
    }
    c.ready=a>0 && b>0 && d>0 && c.model.valid();
    return c;
  }
  void rebuild() { config_=decode(parameters_.values); }
  void add(uint16_t id,const char *name,uint8_t type,double low,double high,float initial) {
    std::snprintf(names_[added_],sizeof(names_[added_]),"%s",name);
    uint32_t bits=static_cast<uint32_t>(initial);
    if (type==RMP_PARAM_F32) std::memcpy(&bits,&initial,4);
    definitions_[added_]={id,type,low,high,bits,names_[added_]}; ++added_;
  }
  unsigned added_=0;
  char names_[count][48]{};
  rmp_param_def_t definitions_[count]{};
  rmp_params_t parameters_{};
  NormalYawConfig config_{};
};
} // namespace tricopter
