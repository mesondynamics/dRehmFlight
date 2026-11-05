#include "../Versions/dRehmFlight_Teensy_BETA_1.3/FlightParameters.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/rmp.c"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/src/RemoteMcu/parameters.c"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace tricopter;

static void cyclicAndLaserChecks() {
  for(float rate : {-360.f,360.f}) for(float lag : {.04f,.08f}) {
    Vec3 compensated{0,0,0},uncompensated{0,0,0};
    double leadError=0,plainError=0;
    constexpr float dt=.0005f;
    for(unsigned i=0;i<12000;++i) {
      const float phase=rate*radians*i*dt;
      const Vec3 desired{std::cos(phase),-std::sin(phase),0};
      const Vec3 lead=cyclicMotorLead(desired,rate*radians,lag,1);
      const Vec3 disabled=cyclicMotorLead(desired,rate*radians,lag,0);
      assert(disabled.x==desired.x && disabled.y==desired.y && disabled.z==desired.z);
      compensated=compensated+(lead-compensated)*(dt/(lag+dt));
      uncompensated=uncompensated+(desired-uncompensated)*(dt/(lag+dt));
      if(i>=4000) {
        const Vec3 a=compensated-desired,b=uncompensated-desired;
        leadError+=a.dot(a);plainError+=b.dot(b);
      }
    }
    assert(leadError<plainError*.001);
  }
  const Vec3 bounded=cyclicMotorLead({1,0,3},100,.5f,1);
  assert(bounded.x==1 && bounded.y==-1 && bounded.z==3);
  LaserSyncConfig cfg;cfg.alignWindowDeg=1;cfg.phaseLeadSeconds=.0045f;
  for(float sign : {-1.f,1.f}) {
    LaserSync sync;
    assert(sync.update(LaserMode::PhasePulse,-sign*1.62f,sign*360,1000,cfg));
    assert(!sync.update(LaserMode::PhasePulse,0,sign*360,2000,cfg));
    assert(sync.pulseCount()==1);
  }
}

// Sensor-in-the-loop rigid-body test at one revolution/second. Truth and AHRS
// are separate; reproduce the sketch's gyro/accel filters, 50 Hz servo hold,
// nonlinear linkage, 125-level motors, unknown body drag and rotor momentum.
// IMU is at CG as requested. This does not simulate position, altitude, contact
// or large rotating wings and cannot establish actual flight stability.
static void run(float inertiaScale,float motorLag,float rotorMomentum) {
  FlightParameters parameters; parameters.begin();
  auto c=parameters.config(); c.ready=c.spinReady=true;
  c.spin.maxRateDps=360; c.spin.maxAccelDps2=30;
  c.reference.spinTilt=5;
  c.safety.overspeedMarginDps=20; c.safety.rateErrorDps=18;
  c.safety.brakingSettleSeconds=10;
  c.cyclicLeadScale=1;
  c.motorLag=.08f;
  c.laser.alignWindowDeg=1;c.laser.phaseLeadSeconds=.0045f;
  auto plant=c.model;
  plant.ixx*=inertiaScale; plant.iyy*=inertiaScale; plant.izz*=inertiaScale;
  const Vec3 inertia=plant.inertia();
  const ControlGains gains{{c.rollKp*.01f,c.rollKi*.01f,c.rollKd*.01f},
      {c.pitchKp*.01f,c.pitchKi*.01f,c.pitchKd*.01f},{0,0,0}};
  CycloneControl reference; NormalYawControl controller; LaserSync laser;
  Quaternion truth{1,0,0,0}, estimate=truth;
  Vec3 omega{0,0,0}, gyro{0,0,0}, accel{0,0,1};
  const float throttle=motorCommand(c.model.rotor[0],c.model.massKg*9.81f/3);
  float thrust[3]={plant.massKg*9.81f/3,plant.massKg*9.81f/3,plant.massKg*9.81f/3};
  float servo=servoForTilt(plant.rotor[2],0), heldServo=servo;
  float peakTilt=0,peakEstimateError=0,peakRate=0,steadyError=0;
  unsigned steadySamples=0,brakingEntries=0,aligningEntries=0,normalReturns=0;
  FlightMode previous=FlightMode::Normal;
  ControlOutput refs{};
  constexpr float dt=.0005f;
  for(unsigned i=0;i<270000;++i) {
    const float t=i*dt;
    PilotInput input{throttle,0,0,0,0,false};
    if(t>=3 && t<25) {input.spin=-1;input.spinRequested=true;}
    if(t>=60 && t<82) {input.spin=1;input.spinRequested=true;}
    if((t>=18 && t<21) || (t>=75 && t<78)) {input.pitch=.2f;input.roll=-.15f;}
    if(t>=125 && t<127) input.yaw=.2f;
    refs=reference.step(input,estimate,gyro,dt,true,false,true,gains,&c.spin,&c.reference,&c.safety);
    if(refs.fault || reference.supervisor().inhibited()) {
      std::fprintf(stderr,"60RPM failed J=%.2f lag=%.3f H=%.3f t=%.3f mode=%u reasons=%u measured=%.2f target=%.2f tilt=%.2f\n",
          inertiaScale,motorLag,rotorMomentum,t,unsigned(refs.mode),reference.supervisor().reasons(),
          refs.yawRate,refs.yawTarget,refs.tiltErrorDeg);
      assert(false);
    }
    if(refs.mode!=previous) {
      brakingEntries+=refs.mode==FlightMode::Braking;
      aligningEntries+=refs.mode==FlightMode::Aligning;
      normalReturns+=refs.mode==FlightMode::Normal;
      previous=refs.mode;
    }
    YawControlRequest request;
    request.rateMode=refs.mode!=FlightMode::Normal; request.rateTargetDps=refs.yawTarget;
    request.attitude=estimate;request.gyroDps=gyro;
    const auto out=controller.step(c,refs.motors,refs.heading,refs.yawRate,input.yaw,dt,true,request);
    assert(out.active && !out.fault);
    reference.applyAllocationFeedback(out.liftScale,out.attitudeScale,out.limited,out.rateTarget);
    if(i%40==0) heldServo=quantizedServoLogical(out.servoDeg);
    servo+=dt/(.16f+dt)*(heldServo-servo);
    const float command[]={out.motors.m2,out.motors.m4,out.motors.m3};
    for(unsigned k=0;k<3;++k) {
      assert(command[k]>=0 && command[k]<=1);
      const float target=motorThrust(plant.rotor[k],std::floor(command[k]*125)/125);
      thrust[k]+=dt/(motorLag+dt)*(target-thrust[k]);
    }
    Wrench w=wrench(plant,thrust,servo);
    w.torque.z-=.003f*omega.z+.0002f*omega.z*std::fabs(omega.z);
    // Unknown net small-prop momentum, intentionally absent from controller.
    w.torque=w.torque-omega.cross({0,0,rotorMomentum});
    if((t>=20 && t<20.2f) || (t>=77 && t<77.2f)) w.torque.x+=.004f;
    const Vec3 alpha=angularAcceleration(inertia,omega,w.torque);
    omega=omega+alpha*dt;
    assert(updateImu(truth,{omega.x*degrees,-omega.y*degrees,-omega.z*degrees},
        Rotation(truth).body({0,0,1}),dt,0));
    gyro=gyro+(omega*degrees-gyro)*.1f;
    accel=accel+(Rotation(truth).body({0,0,1})-accel)*.14f;
    assert(updateImu(estimate,{gyro.x,-gyro.y,-gyro.z},accel,dt,.04f));
    const float tilt=vectorAngle(Rotation(truth).bz,{0,0,1})*degrees;
    const float estimateError=vectorAngle(Rotation(truth).bz,Rotation(estimate).bz)*degrees;
    peakTilt=std::fmax(peakTilt,tilt);peakEstimateError=std::fmax(peakEstimateError,estimateError);
    peakRate=std::fmax(peakRate,std::fabs(Rotation(truth).headingRate(omega*degrees)));
    if((t>=18 && t<24) || (t>=75 && t<81)) {
      const float desired=t<30 ? -360 : 360;
      const float error=Rotation(truth).headingRate(omega*degrees)-desired;
      steadyError+=error*error;++steadySamples;
    }
    const bool high=laser.update(refs.mode==FlightMode::Normal ? LaserMode::Continuous : LaserMode::PhasePulse,
        refs.phase,refs.yawRate-refs.referenceRate,i*500,c.laser);
    if(refs.mode!=FlightMode::Normal && high) {
      const float predicted=wrapDegrees(refs.phase+(refs.yawRate-refs.referenceRate)*c.laser.phaseLeadSeconds);
      assert(std::fabs(predicted)<=c.laser.alignWindowDeg);
      const float physical=wrapDegrees(Rotation(truth).heading()-refs.reference);
      assert(std::fabs(physical)<2);
    }
    if(tilt>=12) {
      std::fprintf(stderr,"60RPM tilt %.2f t=%.2f J=%.2f\n",tilt,t,inertiaScale);assert(false);
    }
  }
  const float rmse=std::sqrt(steadyError/steadySamples);
  std::printf("60RPM Jx%.2f motor %.0fms H%.3f: peak %.2fdeg/s tilt %.2fdeg AHRS %.2fdeg RMSE %.2fdeg/s brake/align/normal %u/%u/%u laser %u\n",
      inertiaScale,motorLag*1000,rotorMomentum,peakRate,peakTilt,peakEstimateError,rmse,
      brakingEntries,aligningEntries,normalReturns,laser.pulseCount());
  assert(rmse<10 && peakEstimateError<3 && refs.mode==FlightMode::Normal);
  assert(brakingEntries>=2 && aligningEntries>=2 && normalReturns>=2 && laser.pulseCount()>12);
}
int main() {
  cyclicAndLaserChecks();
  run(1,.04f,0);
  run(.5f,.04f,.015f);
  run(2,.08f,-.015f);
}
