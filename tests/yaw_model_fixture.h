#pragma once
#include "../Versions/dRehmFlight_Teensy_BETA_1.3/NormalYawControl.h"

// Synthetic fixture, not this aircraft's measured parameters.
inline tricopter::NormalYawConfig syntheticYawConfig() {
  using namespace tricopter;
  NormalYawConfig c;
  c.model.massKg=1.2f; c.model.izz=.025f; c.model.tiltLimitDeg=20;
  c.model.calibrated=true;
  c.model.rotor[0].position={.125f,.25f*rollLever,0};
  c.model.rotor[1].position={.125f,-.25f*rollLever,0};
  c.model.rotor[2].position={-.25f,0,0};
  c.model.rotor[0].tangent={-rollLever,.5f,0};
  c.model.rotor[1].tangent={rollLever,.5f,0};
  c.model.rotor[2].tangent={0,-1,0};
  const float thrust[]={0,1.2f,3.5f,6,9};
  for (unsigned r=0;r<3;++r) {
    auto &m=c.model.rotor[r]; m.kappa=.015f; m.reactionSign=r==1 ? 1 : -1;
    for (unsigned j=0;j<9;++j) m.tiltDeg[j]=(90-(50+10*static_cast<int>(j)))*.5f;
    for (unsigned j=0;j<5;++j) m.thrustN[j]=thrust[j];
  }
  c.ready=c.model.valid(); c.flightEnabled=true;
  return c;
}
