#pragma once
#include "TricopterDynamics.h"
#include "YawServoControl.h"

namespace tricopter {
struct NormalYawConfig {
  DynamicsModel model{};
  bool flightEnabled=false, spinEnabled=false, ready=false;
  bool initialModelEnabled=false; // Same dynamics path, explicitly provisional physical parameters.
  bool tiltMappingReady[3]{}; // Cached on parameter changes; independent of thrust/inertia readiness.
  float headingKp=1.5f, rateKp=4, rateKi=1.5f;
  float maxRateDps=30, maxAccelDps2=60, rateFilterHz=12, maxHeadingError=45;
  float antiWindupGain=2, servoSlew=60, servoLag=.08f, integralLimit=2, damping=0;
  float motorLag=.04f;
};
struct NormalYawOutput {
  MotorMix motors{};
  float servoDeg=90, headingTarget=0, rateTarget=0, rateFiltered=0;
  float torqueRequested=0, torqueEstimated=0, residual=0, integral=0;
  float thrust[3]{}, tilt[3]{}, lift=0, liftScale=1;
  float servoEstimated=servoCenterDeg;
  bool active=false, limited=false, fault=false;
};

class NormalYawControl {
 public:
  void reset(float heading=0) {
    active_=hadStick_=fault_=false; headingTarget_=heading;
    targetRate_=rate_=integral_=allocationTime_=0;
    servo_=estimatedServo_=servoCenterDeg; allocation_={};
    for (float &t : estimatedThrust_) t=0;
  }
  bool faulted() const { return fault_; }
  NormalYawOutput step(const NormalYawConfig &c,MotorMix nominal,float heading,
                       float headingRate,float stick,float dt,bool active) {
    NormalYawOutput out{}; out.motors=nominal; out.headingTarget=heading;
    // The bench midpoint is not the measured vertical thrust direction. Use
    // the inverse calibrated rear-rotor map; do not hardcode a logical trim.
    if (!active) {
      reset(heading);
      if (c.tiltMappingReady[2]) servo_=estimatedServo_=servoForTilt(c.model.rotor[2],0);
      out.servoDeg=servo_; out.servoEstimated=estimatedServo_;
      return out;
    }
    if (!c.ready || !std::isfinite(dt) || dt<=0 || dt>.01f ||
        !std::isfinite(heading) || !std::isfinite(headingRate) || !std::isfinite(stick)) {
      fault_=true; out.fault=true; return out;
    }
    if (!active_) {
      reset(heading); rate_=headingRate; active_=true; allocationTime_=.005f;
      servo_=estimatedServo_=servoForTilt(c.model.rotor[2],0);
      const float commands[]={nominal.m2,nominal.m4,nominal.m3};
      for (unsigned i=0;i<3;++i) estimatedThrust_[i]=motorThrust(c.model.rotor[i],commands[i]);
    }
    if (fault_) { out.fault=true; return out; }
    rate_+=lowpassAlpha(c.rateFilterHz,dt)*(headingRate-rate_);
    const bool moving=std::fabs(stick)>0;
    if (moving) {
      headingTarget_=wrapDegrees(headingTarget_+stick*c.maxRateDps*dt);
      headingTarget_=wrapDegrees(heading+clamp(wrapDegrees(headingTarget_-heading),
                                              -c.maxHeadingError,c.maxHeadingError));
    } else if (hadStick_) {
      // Capture an attainable stopping heading once at stick release.
      const float stop=clamp(rate_*std::fabs(rate_)/(2*c.maxAccelDps2),
                             -c.maxHeadingError,c.maxHeadingError);
      headingTarget_=wrapDegrees(heading+stop);
    }
    hadStick_=moving;
    const float errorHeading=clamp(wrapDegrees(headingTarget_-heading),
                                    -c.maxHeadingError,c.maxHeadingError);
    const float desiredRate=clamp(stick*c.maxRateDps+c.headingKp*errorHeading,
                                  -c.maxRateDps,c.maxRateDps);
    targetRate_=approach(targetRate_,desiredRate,c.maxAccelDps2*dt);
    const float rateError=(targetRate_-rate_)*radians;
    const float proposed=clamp(integral_+c.rateKi*rateError*dt,-c.integralLimit,c.integralLimit);
    const float torque=c.model.izz*(c.rateKp*rateError+proposed)+c.damping*rate_*radians;
    const float t[3]={motorThrust(c.model.rotor[0],nominal.m2),
      motorThrust(c.model.rotor[1],nominal.m4),motorThrust(c.model.rotor[2],nominal.m3)};
    Vec3 target{};
    for (unsigned i=0;i<3;++i) {
      target.x+=t[i]; target.y+=c.model.rotor[i].position.y*t[i];
      target.z-=c.model.rotor[i].position.x*t[i];
    }
    allocationTime_+=dt;
    if (allocationTime_>=.005f) { // allocation 200 Hz; estimation/rate feedback 2 kHz
      allocation_=allocateYaw(c.model,target,torque,servo_,c.servoSlew*allocationTime_);
      allocationTime_=0;
      servo_=allocation_.servoDeg;
    } else {
      // Preserve the 2 kHz collective/roll/pitch response between yaw solves.
      // Only the servo position is held; motor targets are never held for 5 ms.
      allocation_=allocateAt(c.model,target,torque,servo_);
      allocation_.limited=allocation_.limited || std::fabs(allocation_.yawResidual)>.0001f;
    }
    if (!allocation_.valid) { fault_=true; out.fault=true; return out; }
    const float lagAlpha=dt/(c.servoLag+dt);
    const float pwmTarget=quantizedServoLogical(servo_);
    estimatedServo_+=lagAlpha*(pwmTarget-estimatedServo_);
    // Reproduce the flight sketch's 125-step motor command quantization and
    // estimate motor response. These are predictions, never measured thrust.
    float commands[3];
    for (unsigned i=0;i<3;++i) {
      commands[i]=motorCommand(c.model.rotor[i],allocation_.thrust[i]);
      const float quantized=std::floor(commands[i]*125)/125;
      const float targetThrust=motorThrust(c.model.rotor[i],quantized);
      estimatedThrust_[i]+=dt/(c.motorLag+dt)*(targetThrust-estimatedThrust_[i]);
    }
    const Wrench estimated=wrench(c.model,estimatedThrust_,estimatedServo_);
    const float residual=torque-estimated.torque.z;
    // Conditional integration plus bounded back calculation in rad/s^2.
    integral_=proposed;
    if (allocation_.limited && rateError*residual>0) integral_-=c.rateKi*rateError*dt;
    // Back-calculate only the allocator's amplitude/rate shortfall. Feeding
    // normal servo lag/quantization back here would bias I against the plant
    // dynamics and leave a persistent heading error.
    if (allocation_.limited)
      integral_+=c.antiWindupGain*(allocation_.achieved.torque.z-torque)/c.model.izz*dt;
    integral_=clamp(integral_,-c.integralLimit,c.integralLimit);
    out.motors={commands[0],commands[1],commands[2]};
    out.servoDeg=servo_; out.headingTarget=headingTarget_; out.rateTarget=targetRate_;
    out.rateFiltered=rate_; out.torqueRequested=torque; out.torqueEstimated=estimated.torque.z;
    out.residual=residual; out.integral=integral_; out.lift=estimated.force.z;
    out.liftScale=allocation_.liftScale; out.limited=allocation_.limited;
    out.active=true; out.servoEstimated=estimatedServo_;
    for (unsigned i=0;i<3;++i) {
      out.thrust[i]=allocation_.thrust[i];
      out.tilt[i]=physicalTilt(c.model.rotor[i],estimatedServo_);
    }
    return out;
  }
 private:
  bool active_=false,hadStick_=false,fault_=false;
  float headingTarget_=0,targetRate_=0,rate_=0,integral_=0,allocationTime_=0;
  float servo_=90,estimatedServo_=90;
  float estimatedThrust_[3]{};
  Allocation allocation_{};
};
} // namespace tricopter
