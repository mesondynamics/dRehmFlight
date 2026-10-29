#pragma once
#include "TricopterConfig.h"
#include "FlightMath.h"

namespace tricopter {
constexpr unsigned tiltPoints=9, thrustPoints=5;
// Calibration may cover large bench travel; flight remains limited separately
// to at most 45 degrees (30 degrees by default) in allocateAt/allocateYaw.
constexpr float maxCalibratedTiltDeg=85;
struct RotorModel {
  Vec3 position, tangent;
  float reactionSign, kappa;
  float tiltDeg[tiltPoints]; // logical 50,60,...,130; physical, positive CCW tangent
  float thrustN[thrustPoints]; // normalized motor command 0,.25,.5,.75,1
};
inline bool tiltMappingValid(const RotorModel &r) {
  const bool increasing=r.tiltDeg[tiltPoints-1]>r.tiltDeg[0];
  for (unsigned i=0;i<tiltPoints;++i) {
    if (!std::isfinite(r.tiltDeg[i]) || std::fabs(r.tiltDeg[i])>maxCalibratedTiltDeg)
      return false;
    if (i && (increasing ? r.tiltDeg[i]-r.tiltDeg[i-1] :
                          r.tiltDeg[i-1]-r.tiltDeg[i])<.01f) return false;
  }
  return std::fmin(r.tiltDeg[0],r.tiltDeg[tiltPoints-1])<=0 &&
         std::fmax(r.tiltDeg[0],r.tiltDeg[tiltPoints-1])>=0;
}
struct DynamicsModel {
  RotorModel rotor[3]; // M2, M4, M3
  float massKg=0, izz=0, tiltLimitDeg=30;
  bool calibrated=false;
  bool valid() const {
    if (!calibrated || !std::isfinite(massKg) || massKg<=0 ||
        !std::isfinite(izz) || izz<=0 || !std::isfinite(tiltLimitDeg) ||
        tiltLimitDeg<=0 || tiltLimitDeg>45) return false;
    for (const auto &r : rotor) {
      if (!r.position.finite() || !r.tangent.finite() ||
          std::fabs(r.tangent.dot(r.tangent)-1)>.001f || std::fabs(r.tangent.z)>.001f ||
          !std::isfinite(r.kappa) || r.kappa<=0 ||
          (r.reactionSign!=1 && r.reactionSign!=-1) || r.thrustN[0]!=0) return false;
      for (unsigned i=0;i<thrustPoints;++i)
        if (!std::isfinite(r.thrustN[i]) || (i && r.thrustN[i]<=r.thrustN[i-1])) return false;
      if (!tiltMappingValid(r) || std::fabs(r.tiltDeg[4])>tiltLimitDeg) return false;
    }
    const Vec3 a=rotor[1].position-rotor[0].position;
    const Vec3 b=rotor[2].position-rotor[0].position;
    return std::fabs(a.cross(b).z)>1e-4f;
  }
};

inline float interpolate(const float *table,unsigned count,float index) {
  index=clamp(index,0,count-1);
  const unsigned i=static_cast<unsigned>(index);
  return i+1<count ? table[i]+(index-i)*(table[i+1]-table[i]) : table[i];
}
inline float physicalTilt(const RotorModel &r,float servo) {
  return interpolate(r.tiltDeg,tiltPoints,(servo-50)/10);
}
inline float motorThrust(const RotorModel &r,float command) {
  return interpolate(r.thrustN,thrustPoints,clamp(command,0,1)*4);
}
inline float motorCommand(const RotorModel &r,float thrust) {
  thrust=clamp(thrust,0,r.thrustN[4]);
  for (unsigned i=0;i<4;++i) if (thrust<=r.thrustN[i+1])
    return .25f*(i+(thrust-r.thrustN[i])/(r.thrustN[i+1]-r.thrustN[i]));
  return 1;
}
struct Wrench { Vec3 force, torque; };
inline Vec3 thrustAxis(const RotorModel &r,float servo) {
  const float angle=physicalTilt(r,servo)*radians;
  return r.tangent*std::sin(angle)+Vec3{0,0,std::cos(angle)};
}
inline Vec3 torquePerNewton(const RotorModel &r,Vec3 axis) {
  return r.position.cross(axis)+axis*(r.reactionSign*r.kappa);
}
inline Wrench wrench(const DynamicsModel &model,const float thrust[3],float servo) {
  Wrench out{};
  for (unsigned i=0;i<3;++i) {
    const auto &r=model.rotor[i]; const Vec3 axis=thrustAxis(r,servo);
    out.force=out.force+axis*thrust[i];
    out.torque=out.torque+torquePerNewton(r,axis)*thrust[i];
  }
  return out;
}
inline Vec3 angularAcceleration(Vec3 inertia,Vec3 omega,Vec3 torque) {
  const Vec3 momentum{inertia.x*omega.x,inertia.y*omega.y,inertia.z*omega.z};
  const Vec3 net=torque-omega.cross(momentum);
  return {net.x/inertia.x,net.y/inertia.y,net.z/inertia.z};
}

struct Allocation {
  float thrust[3];
  float servoDeg, liftScale, yawResidual;
  Wrench achieved;
  bool valid, limited;
};

// At a given servo angle, solve the exact 3x3 vertical force / roll / pitch
// equations, including tilted reaction torques. Positive thrust only.
inline Allocation allocateAt(const DynamicsModel &m,Vec3 liftRollPitch,float yaw,float servo) {
  Allocation out{}; out.servoDeg=servo; out.liftScale=1;
  Vec3 columns[3];
  for (unsigned i=0;i<3;++i) {
    if (std::fabs(physicalTilt(m.rotor[i],servo))>m.tiltLimitDeg+.001f) return out;
    const Vec3 axis=thrustAxis(m.rotor[i],servo);
    const Vec3 torque=torquePerNewton(m.rotor[i],axis);
    columns[i]={axis.z,torque.x,torque.y};
  }
  const float determinant=columns[0].dot(columns[1].cross(columns[2]));
  if (!std::isfinite(determinant) || std::fabs(determinant)<1e-7f) return out;
  out.thrust[0]=liftRollPitch.dot(columns[1].cross(columns[2]))/determinant;
  out.thrust[1]=columns[0].dot(liftRollPitch.cross(columns[2]))/determinant;
  out.thrust[2]=columns[0].dot(columns[1].cross(liftRollPitch))/determinant;
  for (unsigned i=0;i<3;++i) {
    if (!std::isfinite(out.thrust[i]) || out.thrust[i]<-1e-5f) return out;
    out.thrust[i]=std::fmax(0,out.thrust[i]);
    if (out.thrust[i]>m.rotor[i].thrustN[4])
      out.liftScale=std::fmin(out.liftScale,m.rotor[i].thrustN[4]/out.thrust[i]);
  }
  for (float &t : out.thrust) t*=out.liftScale;
  out.achieved=wrench(m,out.thrust,servo);
  out.yawResidual=yaw-out.achieved.torque.z;
  out.valid=out.achieved.force.finite() && out.achieved.torque.finite();
  out.limited=out.liftScale<.9999f;
  return out;
}

inline float servoForTilt(const RotorModel &r,float tilt) {
  const bool increasing=r.tiltDeg[8]>r.tiltDeg[0];
  for (unsigned i=0;i<8;++i) {
    if (increasing ? tilt<=r.tiltDeg[i+1] : tilt>=r.tiltDeg[i+1])
      return 50+10*(i+clamp((tilt-r.tiltDeg[i])/(r.tiltDeg[i+1]-r.tiltDeg[i]),0,1));
  }
  return 130;
}

// At most five fixed evaluations, no unbounded optimization. Reachable servo limits
// combine measured physical travel and the configured command slew rate.
inline Allocation allocateYaw(const DynamicsModel &m,Vec3 target,float yaw,
                              float previousServo,float maxStep) {
  Allocation failure{};
  if (!target.finite() || !std::isfinite(yaw) || !std::isfinite(previousServo) ||
      !std::isfinite(maxStep) || maxStep<0 || target.x<=.001f) return failure;
  float low=50,high=130;
  for (const auto &r : m.rotor) {
    const float a=servoForTilt(r,-m.tiltLimitDeg),b=servoForTilt(r,m.tiltLimitDeg);
    low=std::fmax(low,std::fmin(a,b)); high=std::fmin(high,std::fmax(a,b));
  }
  if (low>high || previousServo<low-.001f || previousServo>high+.001f) return failure;
  const float travelLow=low,travelHigh=high;
  low=std::fmax(low,previousServo-maxStep); high=std::fmin(high,previousServo+maxStep);
  float servo=clamp(previousServo,low,high);
  Allocation best=allocateAt(m,target,yaw,servo);
  for (unsigned iteration=0;iteration<2;++iteration) {
    const float probe=travelHigh-servo>=servo-travelLow ? servo+.25f : servo-.25f;
    // Probe outside slew range is allowed for the derivative only, but remains
    // inside the physical range. Final commands always obey low/high.
    const float bounded=clamp(probe,travelLow,travelHigh);
    const Allocation next=allocateAt(m,target,yaw,bounded);
    if (!best.valid || !next.valid || std::fabs(bounded-servo)<1e-5f) break;
    const float gain=(next.achieved.torque.z-best.achieved.torque.z)/(bounded-servo);
    if (!std::isfinite(gain) || std::fabs(gain)<1e-6f) break;
    const float candidate=clamp(servo+best.yawResidual/gain,low,high);
    const Allocation result=allocateAt(m,target,yaw,candidate);
    if (!result.valid || std::fabs(result.yawResidual)>std::fabs(best.yawResidual)+1e-6f) break;
    servo=candidate; best=result;
  }
  best.limited=best.limited || std::fabs(best.yawResidual)>.0001f;
  return best;
}
} // namespace tricopter
