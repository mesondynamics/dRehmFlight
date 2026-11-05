#include "../Versions/dRehmFlight_Teensy_BETA_1.3/CycloneControl.h"
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/LevelAlignment.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace tricopter;

// Independent ZYX Euler construction: public pitch is nose-down and heading
// is CCW; fusion uses forward/right/down and therefore opposite Y/Z signs.
static Quaternion euler(float roll,float pitch,float heading) {
  const float cr=std::cos(roll*radians/2),sr=std::sin(roll*radians/2);
  const float cp=std::cos(-pitch*radians/2),sp=std::sin(-pitch*radians/2);
  const float cy=std::cos(-heading*radians/2),sy=std::sin(-heading*radians/2);
  return {cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
      cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
}
static void near(Vec3 a,Vec3 b,float tolerance=1e-5f) {
  assert(std::fabs(a.x-b.x)<tolerance && std::fabs(a.y-b.y)<tolerance &&
      std::fabs(a.z-b.z)<tolerance);
}
static Vec3 pilot(Vec3 frd) { return {frd.x,-frd.y,-frd.z}; }

int main() {
  for (float roll : {-5.f,0.f,.04f,3.f}) for (float pitch : {-4.f,-.15f,0.f,5.f}) {
    LevelAlignment alignment; assert(alignment.set(roll,pitch));
    for (float heading : {-179.f,-90.f,0.f,45.f,179.f}) {
      const Quaternion raw=euler(roll,pitch,heading);
      const Rotation corrected(alignment.attitude(raw));
      near(corrected.bz,{0,0,1});
      assert(std::fabs(wrapDegrees(corrected.heading()-heading))<.0001f);
      // A true world-vertical spin must have no apparent transverse body rate
      // after correction, at EVERY heading and in both spin directions.
      for (float rate : {-10.f,0.f,10.f}) {
        const Vec3 rawFrd=Rotation(raw).body({0,0,-rate});
        near(alignment.gyro(pilot(rawFrd)),{0,0,rate});
      }
    }
    // Tilted body + simultaneous roll/pitch/yaw: corrected quaternion and
    // corrected gyro must obey the SAME differential kinematics.
    Quaternion raw=euler(17,-12,43);
    Quaternion corrected=alignment.attitude(raw);
    const Vec3 rawRates{21,-32,13};
    const Vec3 correctedRates=alignment.gyro(rawRates);
    constexpr float dt=.0005f;
    assert(updateImu(raw,pilot(rawRates),Rotation(raw).body({0,0,1}),dt,0));
    assert(updateImu(corrected,pilot(correctedRates),Rotation(corrected).body({0,0,1}),dt,0));
    const Rotation actual(alignment.attitude(raw)),expected(corrected);
    near(actual.bx,expected.bx); near(actual.by,expected.by); near(actual.bz,expected.bz);
  }
  // Zero compensation preserves the previous flight path exactly.
  LevelAlignment zero; assert(zero.set(0,0));
  const Quaternion arbitrary=euler(10,-20,70),unchanged=zero.attitude(arbitrary);
  assert(unchanged.w==arbitrary.w && unchanged.x==arbitrary.x &&
      unchanged.y==arbitrary.y && unchanged.z==arbitrary.z);
  near(zero.gyro({10,20,30}),{10,20,30});
  for (float invalid : {-5.01f,5.01f,std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()}) {
    assert(!zero.set(invalid,0) && !zero.valid());
    assert(!zero.set(0,invalid) && !zero.valid());
  }
  // At the measured physical level, a centered-stick controller must not
  // command a continuing forward tilt as the heading passes around a circle.
  LevelAlignment measured; assert(measured.set(measuredLevelRollDeg,measuredLevelPitchDeg));
  const ControlGains gains{{.002f,.003f,.0005f},{.002f,.003f,.0005f},{0,0,0}};
  for (float heading : {-179.f,-90.f,0.f,90.f,179.f}) {
    CycloneControl control;
    const Quaternion raw=euler(measuredLevelRollDeg,measuredLevelPitchDeg,heading);
    ControlOutput out{};
    for (unsigned i=0;i<100;++i)
      out=control.step({.4f,0,0,0,0,false},measured.attitude(raw),{0,0,0},.0005f,
          true,false,true,gains);
    assert(!out.fault && out.tiltErrorDeg<.0001f);
    assert(std::fabs(out.roll)<1e-7f && std::fabs(out.pitch)<1e-7f);
  }
  // The compensated sensor and an ideally aligned sensor must drive the same
  // controller through Normal -> Spin -> Braking -> Aligning -> Normal in
  // BOTH directions, including nonzero generalized roll/pitch stick commands.
  LevelAlignment mounted; assert(mounted.set(3,-4));
  CycloneControl ideal,compensated;
  SpinDynamicsModel spin; spin.maxRateDps=10; spin.maxAccelDps2=10;
  AttitudeReferenceConfig reference;
  unsigned modes=0;
  for (unsigned i=0;i<24000;++i) {
    const float t=i*.0005f;
    const bool positive=t>=1 && t<3,negative=t>=6 && t<8;
    const float rate=positive ? 5 : negative ? -5 : 0;
    const float heading=t<1 ? 0 : t<3 ? (t-1)*5 : t<6 ? 10 : t<8 ? 10-(t-6)*5 : 0;
    const Quaternion raw=euler(3,-4,heading);
    const Vec3 rawGyro=pilot(Rotation(raw).body({0,0,-rate}));
    const PilotInput in{.4f,.05f*std::sin(t),.05f*std::cos(t),0,
        positive ? .5f : negative ? -.5f : 0,positive || negative};
    const auto expected=ideal.step(in,euler(0,0,heading),{0,0,rate},.0005f,
        true,false,true,gains,&spin,&reference);
    const auto actual=compensated.step(in,mounted.attitude(raw),mounted.gyro(rawGyro),
        .0005f,true,false,true,gains,&spin,&reference);
    assert(!actual.fault && actual.mode==expected.mode);
    assert(!compensated.supervisor().inhibited());
    assert(std::fabs(actual.yawRate-expected.yawRate)<.0001f);
    assert(std::fabs(actual.motors.m2-expected.motors.m2)<.00001f);
    assert(std::fabs(actual.motors.m4-expected.motors.m4)<.00001f);
    assert(std::fabs(actual.motors.m3-expected.motors.m3)<.00001f);
    near(actual.desiredUp,expected.desiredUp,.0001f);
    modes|=1u<<static_cast<unsigned>(actual.mode);
  }
  assert(modes==15);
  std::puts("Level reference, both spin directions, rotated gyro, tilted kinematics and neutral control passed");
}
