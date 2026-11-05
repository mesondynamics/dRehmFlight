#pragma once
#include "NormalYawControl.h"
#include "ServoTiltCalibration.h"
#include "InitialYawModel.h"
#include "src/RemoteMcu/parameters.h"
#include <cstdio>
#include <cstring>

namespace tricopter {
class FlightParameters {
 public:
  static constexpr unsigned count=109;
  void begin() {
    added_=0;
    add(1,"flight_enable",RMP_PARAM_BOOL,0,1,0);
    add(2,"spin_enable",RMP_PARAM_BOOL,0,1,0);
    add(3,"heading_kp_1_s",RMP_PARAM_F32,.05,10,.8);
    add(4,"rate_kp_1_s",RMP_PARAM_F32,.05,30,1.5);
    add(5,"rate_ki_1_s2",RMP_PARAM_F32,0,30,.4);
    add(6,"yaw_max_deg_s",RMP_PARAM_F32,1,160,20);
    add(7,"yaw_accel_deg_s2",RMP_PARAM_F32,5,360,30);
    add(8,"yaw_lpf_hz",RMP_PARAM_F32,1,50,12);
    add(9,"heading_error_max_deg",RMP_PARAM_F32,1,90,45);
    add(10,"anti_windup_1_s",RMP_PARAM_F32,0,20,2);
    add(11,"servo_slew_logical_deg_s",RMP_PARAM_F32,1,120,40);
    add(12,"servo_lag_s",RMP_PARAM_F32,.01,.5,.08);
    add(13,"physical_tilt_limit_deg",RMP_PARAM_F32,1,45,15);
    add(14,"model_verified",RMP_PARAM_BOOL,0,1,0);
    add(15,"mass_kg",RMP_PARAM_F32,0,20,initial_model::massKg);
    add(16,"izz_kg_m2",RMP_PARAM_F32,0,2,initial_model::izz);
    add(17,"yaw_i_accel_max_rad_s2",RMP_PARAM_F32,.01,20,2);
    add(18,"yaw_damping_Nm_s_rad",RMP_PARAM_F32,0,1,0);
    const char *geometry[]={"arm_M2_m","arm_M4_m","arm_M3_m","cg_x_m","cg_y_m",
      "rotor_height_from_cg_m","kappa_M2_m","kappa_M4_m","kappa_M3_m"};
    for (unsigned i=0;i<9;++i)
      add(20+i,geometry[i],RMP_PARAM_F32,i>=3 && i<=5 ? -1 : 0,i<3 ? 2 : 1,
          i<3 ? initial_model::armM : (i>=6 ? initial_model::kappaM : 0));
    const unsigned motors[]={2,4,3};
    for (unsigned r=0;r<3;++r) for (unsigned j=0;j<9;++j) {
      char name[48]; std::snprintf(name,sizeof(name),"M%u_tilt_at_servo_%u_deg",motors[r],50+10*j);
      add(30+9*r+j,name,RMP_PARAM_F32,-maxCalibratedTiltDeg,maxCalibratedTiltDeg,
          servo_calibration::defaultTilt(j));
    }
    for (unsigned r=0;r<3;++r) for (unsigned j=0;j<5;++j) {
      char name[48]; std::snprintf(name,sizeof(name),"M%u_thrust_at_%u_pct_N",motors[r],25*j);
      add(60+5*r+j,name,RMP_PARAM_F32,0,100,initial_model::thrustN[j]);
    }
    // Provisional parameters use the full nonlinear allocator too. Never
    // replace missing physical values with synthetic regression fixtures.
    add(75,"model_initial_estimate",RMP_PARAM_BOOL,0,1,0);
    add(76,"motor_lag_s",RMP_PARAM_F32,.005,.5,.04);
    add(77,"ixx_kg_m2",RMP_PARAM_F32,.0001,2,initial_model::izz*.5f);
    add(78,"iyy_kg_m2",RMP_PARAM_F32,.0001,2,initial_model::izz*.5f);
    add(79,"spin_damping_Nm_s_rad",RMP_PARAM_F32,0,1,0);
    add(80,"spin_quadratic_damping",RMP_PARAM_F32,0,1,0);
    add(81,"spin_feedforward_scale",RMP_PARAM_F32,0,1,1);
    add(82,"spin_max_deg_s",RMP_PARAM_F32,1,90,15);
    add(83,"spin_max_accel_deg_s2",RMP_PARAM_F32,1,180,10);
    add(84,"laser_align_window_deg",RMP_PARAM_F32,.1,30,2);
    add(85,"laser_pulse_ms",RMP_PARAM_F32,1,200,200);
    add(86,"laser_min_interval_ms",RMP_PARAM_F32,10,2000,200);
    add(87,"laser_enable",RMP_PARAM_BOOL,0,1,0);
    add(88,"roll_angle_kp",RMP_PARAM_F32,0,3,.2f);
    add(89,"roll_angle_ki",RMP_PARAM_F32,0,3,.3f);
    add(90,"roll_rate_kd",RMP_PARAM_F32,0,1,.05f);
    add(91,"pitch_angle_kp",RMP_PARAM_F32,0,3,.2f);
    add(92,"pitch_angle_ki",RMP_PARAM_F32,0,3,.3f);
    add(93,"pitch_rate_kd",RMP_PARAM_F32,0,1,.05f);
    add(94,"spin_rate_kp_1_s",RMP_PARAM_F32,.05,30,1.5f);
    add(95,"spin_rate_ki_1_s2",RMP_PARAM_F32,0,30,.4f);
    add(96,"gyro_coupling_scale",RMP_PARAM_F32,0,1,1);
    add(97,"spin_tilt_limit_deg",RMP_PARAM_F32,1,15,10);
    add(98,"normal_tilt_limit_deg",RMP_PARAM_F32,1,35,30);
    add(99,"generalized_yaw_max_deg_s",RMP_PARAM_F32,1,60,20);
    add(100,"spin_stop_rate_deg_s",RMP_PARAM_F32,.2,3,2);
    add(101,"spin_stop_dwell_s",RMP_PARAM_F32,.3,2,.5);
    add(102,"exit_reference_deg_s",RMP_PARAM_F32,1,30,20);
    add(103,"tilt_command_deg_s",RMP_PARAM_F32,5,90,60);
    add(104,"spin_overspeed_margin_deg_s",RMP_PARAM_F32,2,30,5);
    add(105,"spin_overspeed_dwell_s",RMP_PARAM_F32,.1,2,.25);
    add(106,"spin_rate_error_deg_s",RMP_PARAM_F32,2,30,8);
    add(107,"spin_rate_error_dwell_s",RMP_PARAM_F32,.3,5,1);
    add(108,"spin_tilt_error_deg",RMP_PARAM_F32,3,20,8);
    add(109,"spin_tilt_error_dwell_s",RMP_PARAM_F32,.2,2,.5);
    add(110,"spin_allocation_min_scale",RMP_PARAM_F32,.5,.99,.95);
    add(111,"spin_allocation_dwell_s",RMP_PARAM_F32,.2,2,.5);
    add(112,"spin_braking_settle_timeout_s",RMP_PARAM_F32,2,20,5);
    add(113,"imu_level_roll_deg",RMP_PARAM_F32,-5,5,measuredLevelRollDeg);
    add(114,"imu_level_pitch_deg",RMP_PARAM_F32,-5,5,measuredLevelPitchDeg);
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
      if ((candidate[13] && !next.ready) || (candidate[0] && !next.ready) ||
          (candidate[1] && !next.spinReady) || (next.laserEnabled && !next.laser.valid())) safe=false;
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
      c.tiltMappingReady[r]=tiltMappingValid(m);
      for (unsigned j=0;j<5;++j) m.thrustN[j]=number(v,54+5*r+j);
    }
    c.initialModelEnabled=v[69]; c.motorLag=number(v,70);
    c.model.ixx=number(v,71); c.model.iyy=number(v,72);
    c.spin.viscousDampingNmS=number(v,73);
    c.spin.quadraticDampingNmS2=number(v,74);
    c.spin.feedforwardScale=number(v,75);
    c.spin.maxRateDps=number(v,76);
    c.spin.maxAccelDps2=number(v,77);
    c.laser.alignWindowDeg=number(v,78);
    c.laser.pulseUs=milliseconds(v,79);
    c.laser.minIntervalUs=milliseconds(v,80);
    c.laserEnabled=v[81];
    c.rollKp=number(v,82); c.rollKi=number(v,83); c.rollKd=number(v,84);
    c.pitchKp=number(v,85); c.pitchKi=number(v,86); c.pitchKd=number(v,87);
    c.spinRateKp=number(v,88); c.spinRateKi=number(v,89);
    c.gyroFeedforwardScale=number(v,90);
    c.reference.spinTilt=number(v,91); c.reference.normalTilt=number(v,92);
    c.reference.generalizedYawMaxDps=number(v,93);
    c.safety.stopRateDps=number(v,94); c.safety.stopDwellSeconds=number(v,95);
    c.safety.exitReferenceDps=number(v,96); c.safety.tiltCommandDps=number(v,97);
    c.safety.overspeedMarginDps=number(v,98); c.safety.overspeedDwellSeconds=number(v,99);
    c.safety.rateErrorDps=number(v,100); c.safety.rateErrorDwellSeconds=number(v,101);
    c.safety.tiltErrorDeg=number(v,102); c.safety.tiltErrorDwellSeconds=number(v,103);
    c.safety.allocationMinScale=number(v,104); c.safety.allocationDwellSeconds=number(v,105);
    c.safety.brakingSettleSeconds=number(v,106);
    c.level.set(number(v,107),number(v,108));
    c.ready=a>0 && b>0 && d>0 && c.model.valid(false) && c.level.valid() &&
        (c.model.calibrated || c.initialModelEnabled);
    c.spinReady=c.ready && c.spin.valid() && c.safety.valid() && c.model.ixx>0 && c.model.iyy>0;
    return c;
  }
  static uint32_t milliseconds(const uint32_t *v,unsigned i) {
    const float ms=number(v,i);
    // Candidate batches are decoded before the parameter library validates
    // types/ranges. Never convert NaN/Inf or a negative float to uint32 here.
    return std::isfinite(ms) && ms>=0 && ms<=2000 ?
        static_cast<uint32_t>(ms*1000.0f+.5f) : 0;
  }
  void rebuild() { config_=decode(parameters_.values); }
  void add(uint16_t id,const char *name,uint8_t type,double low,double high,float initial) {
    std::snprintf(names_[added_],sizeof(names_[added_]),"%s",name);
    uint32_t bits=0;
    if (type==RMP_PARAM_F32) std::memcpy(&bits,&initial,4);
    else bits=static_cast<uint32_t>(initial);
    definitions_[added_]={id,type,low,high,bits,names_[added_]}; ++added_;
  }
  unsigned added_=0;
  char names_[count][48]{};
  rmp_param_def_t definitions_[count]{};
  rmp_params_t parameters_{};
  NormalYawConfig config_{};
};
} // namespace tricopter
