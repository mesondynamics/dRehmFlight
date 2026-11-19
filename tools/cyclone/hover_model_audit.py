#!/usr/bin/env python3
"""Check v12 hover allocation and rigid-body balances against sealed telemetry.

This offline reconstruction uses the airframe geometry from FlightParameters.h.
Body vectors are FLU. Rotor order is M2, M4, M3. Command-derived forces are
predictions. Only angular velocity and sensor Z specific force are measured.
At 100 Hz the reconstruction approximates the onboard 2 kHz actuator states.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_record import load_hover


def rms(values):
    return np.sqrt(np.mean(values ** 2, axis=0))


def lagged(values, times, tau):
    """First-order lag, treating each sampled command as held over one interval."""
    result = values.copy()
    for k, gain in enumerate(-np.expm1(-np.diff(times) / tau), start=1):
        result[k] = result[k - 1] + gain * (values[k] - result[k - 1])
    return result


def model_from_parameters(p):
    inertia = np.array([p[77] or p[16] / 2, p[78] or p[16] / 2, p[16]])
    arms = np.array([p[i] for i in (20, 21, 22)])
    lever = np.sqrt(3.) / 2
    # The compiled v12 geometry has radial arms spaced 120 degrees apart.
    radial = np.array([[.5, lever, 0], [.5, -lever, 0], [-1., 0, 0]])
    positions = arms[:, None] * radial + [-p[23], -p[24], p[25]]
    tangents = np.cross([0, 0, 1], radial)
    reaction = np.array([-p[26], p[27], -p[28]])
    thrust = np.array([[p[60 + 5 * i + j] for j in range(5)] for i in range(3)])
    tilt = np.array([[p[30 + 9 * i + j] for j in range(9)] for i in range(3)])
    if (not all(np.isfinite(x).all() for x in (inertia, arms, positions, reaction, thrust, tilt))
            or np.any(inertia <= 0) or np.any(arms <= 0) or min(p[i] for i in (26, 27, 28)) <= 0
            or np.any(thrust[:, 0] != 0) or np.any(np.diff(thrust, axis=1) <= 0)
            or not all(np.all(d > 0) or np.all(d < 0) for d in np.diff(tilt, axis=1))
            or not (p[12] > 0 and p[76] > 0 and p[15] > 0)):
        raise ValueError("Invalid geometry, inertia, actuator lag, or calibration table")
    return positions, tangents, reaction, inertia, thrust, tilt


def wrench(thrust, servo, geometry):
    positions, tangents, reaction, _, _, tilt = geometry
    logical_nodes = np.arange(50, 131, 10)
    angles = np.deg2rad(np.column_stack([np.interp(servo, logical_nodes, table) for table in tilt]))
    axes = np.sin(angles)[:, :, None] * tangents
    axes[:, :, 2] = np.cos(angles)
    forces = thrust[:, :, None] * axes
    moments = np.cross(positions, forces) + reaction[None, :, None] * forces
    return forces.sum(axis=1), moments.sum(axis=1)


def impulse_comparison(t, omega, acceleration):
    """Compare angular rate changes over measured intervals near 0.5 s."""
    steps = int(round(.5 / np.median(np.diff(t))))
    if steps < 1 or len(t) <= steps:
        raise ValueError("Window too short for the 0.5 s angular impulse check")
    integrated = np.zeros_like(omega)
    integrated[1:] = np.cumsum(.5 * (acceleration[1:] + acceleration[:-1])
                               * np.diff(t)[:, None], axis=0)
    measured = np.rad2deg(omega[steps:] - omega[:-steps])
    predicted = np.rad2deg(integrated[steps:] - integrated[:-steps])
    correlations = [float(np.corrcoef(x, y)[0, 1]) if x.std() and y.std() else None
                    for x, y in zip(measured.T, predicted.T)]
    return {"mean_interval_s": (t[steps:] - t[:-steps]).mean(),
            "predicted_mean_dps": predicted.mean(axis=0), "measured_mean_dps": measured.mean(axis=0),
            "predicted_rms_dps": rms(predicted), "measured_rms_dps": rms(measured),
            "correlation": correlations}


def audit(t, waves, mask, p, mass):
    geom = model_from_parameters(p)
    positions, _, reaction, inertia, thrust_table, tilt_table = geom
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError("Mass must be finite and positive")
    if t[mask][0] - t[0] < 6 * max(p[12], p[76]):
        raise ValueError("Insufficient recorded actuator history before the window")
    # DShotProtocol.h, normal v12 path. The logged PWM is a command, not position feedback.
    codes = waves[:, 23:26]
    if np.any((codes != codes.astype(int)) | (codes < 0) | (codes > 2047) | ((codes > 0) & (codes < 48))):
        raise ValueError("Unexpected motor code in throttle stream")
    u = np.maximum(0, (codes - 48) / 1999.)
    thrust = np.column_stack([np.interp(x, np.linspace(0, 1, 5), table)
                              for x, table in zip(u.T, thrust_table)])
    servo = (waves[:, 1] - 512.) * 180 / 1984
    _, command_torque = wrench(thrust[mask], servo[mask], geom)
    force, torque = wrench(lagged(thrust, t, p[76])[mask], lagged(servo, t, p[12])[mask], geom)
    v, tw = waves[mask], t[mask]
    omega = np.deg2rad(v[:, [20, 21, 17]])
    coupling = np.cross(omega, omega * inertia)
    required = v[:, [26, 27, 11]]
    roll, pitch = np.deg2rad(v[:, 15:17]).T
    up = np.column_stack((-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)))
    lift = np.sum(force * up, axis=1)
    expected_net = (omega[-1] - omega[0]) * inertia / (tw[-1] - tw[0]) + coupling.mean(axis=0)
    return {
        "model": {"rotor_order": ["M2", "M4", "M3"], "body_axes": "forward, left, up",
                  "configured_mass_kg": p[15], "comparison_mass_kg": mass, "inertia_kg_m2": inertia,
                  "positions_relative_cg_m": positions, "signed_kappa_m": reaction,
                  "thrust_table_N": thrust_table, "tilt_table_deg": tilt_table,
                  "model_verified": bool(p[14]), "initial_model_enabled": bool(p[75]),
                  "motor_lag_s": p[76], "servo_lag_s": p[12]},
        "samples": len(tw), "throttle_us": v[0, 18], "device_window_samples_s": [tw[0], tw[-1]],
        "mean_command_derived_body_force_N": force.mean(axis=0),
        "mean_projected_upward_thrust_N": lift.mean(), "weight_N": mass * 9.81,
        "upward_thrust_fraction_of_weight": lift.mean() / (mass * 9.81),
        "mean_sensor_z_specific_force_g": v[:, 22].mean(),
        "direct_body_horizontal_thrust_p50_p95_N": np.percentile(np.linalg.norm(force[:, :2], axis=1), [50, 95]),
        "servo_us_min_median_max": np.percentile(v[:, 1], [0, 50, 100]),
        "requested_torque_rms_Nm": rms(required),
        "requested_torque_mean_Nm": required.mean(axis=0),
        "wire_command_vs_request_torque_rms_Nm": rms(command_torque - required),
        "lagged_yaw_prediction_vs_onboard_rms_Nm": rms(torque[:, 2] - v[:, 12]),
        "mean_command_derived_torque_Nm": torque.mean(axis=0),
        "mean_gyro_inferred_net_torque_given_inertia_Nm": expected_net,
        "mean_yaw_integral_contribution_Nm": inertia[2] * v[:, 14].mean(),
        "angular_impulse_half_second": impulse_comparison(tw, omega, (torque - coupling) / inertia),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="Sealed v12 .rmplog with its adjacent .json")
    parser.add_argument("--start", type=float, default=64., help="Device seconds, inclusive")
    parser.add_argument("--end", type=float, default=124., help="Device seconds, exclusive")
    parser.add_argument("--mass-kg", type=float, help="Independent all-up mass for lift comparison")
    args = parser.parse_args()
    try:
        metadata, _, t, waves, mask = load_hover(args.log, args.start, args.end)
        params = metadata["hover_parameters"]
        mass = params[15] if args.mass_kg is None else args.mass_kg
        result = audit(t, waves, mask, params, mass)
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))
    result["recording"] = args.log.stem
    result["boot"] = metadata["boot"]
    result["sha256"] = metadata["raw_sha256"]
    result["parameter_revision_from_log"] = metadata["parameter_revision"]
    print(json.dumps(result, default=lambda array: array.tolist(), allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
