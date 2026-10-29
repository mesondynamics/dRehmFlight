#pragma once

namespace tricopter { namespace initial_model {
// User supplied 2026-10-29: 750 g including battery, each rotor axis 0.5 m
// from the linkage center, CG at that center, 5-inch 3-blade, 2750 KV, 4S.
constexpr float massKg=.750f;
constexpr float armM=.500f;
// Priors for first-flight identification, NOT measurements or motor specs.
// Uniform radial mass distribution: Izz = m*R^2/3. The actual battery,
// motors and arm mass distribution may produce a substantially different Izz.
constexpr float izz=massKg*armM*armM/3;
constexpr float kappaM=.010f;
// Assume total maximum thrust / weight = 3 and T proportional to command^2.
// Neither this margin nor the curve follows from KV, prop size or C rating.
// Five live knots may be replaced independently after identifying each motor.
constexpr float maxThrustN=massKg*9.81f;
constexpr float thrustN[5]={0,maxThrustN*.0625f,maxThrustN*.25f,
                           maxThrustN*.5625f,maxThrustN};
} } // namespace tricopter::initial_model
