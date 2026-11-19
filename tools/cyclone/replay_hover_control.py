#!/usr/bin/env python3
"""Reconstruct v12 ordinary-hover control requests from sampled attitude and rates.

Input is a sealed RMPLOG with complete on-device parameter readback frames.
This replay is for the symmetric,
center-of-gravity-centered v12 airframe, a single linear thrust-curve segment,
centered sticks, settled trim, and unsaturated transverse allocation.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_record import load_hover


def residual(actual, predicted):
    error = actual - predicted
    return {
        "rms": np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
        "max_abs": np.max(np.abs(error), axis=0).tolist(),
    }


def replay(time, waves, params):
    # In the instantaneous heading frame, the body rotation is Ry(-pitch) Rx(roll).
    # Columns give the body axes in forward/right/down coordinates. Logged gyro
    # axes are forward/left/up. All feedback angles and rates here use degrees.
    roll, pitch = np.deg2rad(waves[:, 15:17]).T
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rotation = np.zeros((len(time), 3, 3))
    rotation[:, 0, 0] = cp
    rotation[:, 2, 0] = sp
    rotation[:, 0, 1] = -sp * sr
    rotation[:, 1, 1] = cr
    rotation[:, 2, 1] = cp * sr
    rotation[:, 0, 2] = -sp * cr
    rotation[:, 1, 2] = -sr
    rotation[:, 2, 2] = cp * cr

    up = -rotation[:, :, 2]
    trim = np.deg2rad(params[117])
    desired_up = np.array([np.sin(trim), 0, -np.cos(trim)])
    cross = np.cross(up, desired_up)
    magnitude = np.linalg.norm(cross, axis=1)
    tilt_error = np.rad2deg(np.arctan2(magnitude, up @ desired_up))
    scale = np.divide(tilt_error, magnitude, out=np.full_like(magnitude, 180 / np.pi),
                      where=magnitude > 1e-6)
    error = cross[:, :2] * scale[:, None] * [1, -1]

    gyro = waves[:, [20, 21, 17]] * [1, -1, -1]
    relative = np.einsum("nij,nj->ni", rotation, gyro)
    relative[:, 2] += waves[:, 9]
    relative -= up * (relative * up).sum(axis=1)[:, None]
    kp = 0.01 * np.array([params[88], params[91]])
    ki = 0.01 * np.array([params[89], params[92]])
    kd = 0.01 * np.array([params[90], params[93]])
    pd = kp * error + kd * relative[:, :2] * [-1, 1]

    # On an equal-arm tricopter each differential mix command has moment gain
    # 1.5 * radius * local dT/du. FlightParameters uses motor order M2,M4,M3.
    if not np.allclose([params[20], params[21]], params[22]) or any(params[k] for k in (23, 24)):
        raise ValueError("Replay requires equal rotor radii and centered horizontal CG")
    curves = np.array([[params[base + k] for k in range(5)] for base in (60, 65, 70)])
    if not np.allclose(curves, curves[0]):
        raise ValueError("Replay requires equal thrust tables")
    collective = (waves[0, 18] - 1000) / 1000
    segment = int(4 * collective)
    if not 0 <= segment < 4:
        raise ValueError("Collective is outside the interior of the thrust table")
    lower, upper = segment / 4, (segment + 1) / 4
    gain = 1.5 * params[20] * 4 * np.diff(curves[0])[segment]
    demand = waves[:, 26:28]
    roll_mix, pitch_mix = (demand / gain).T
    nominal_commands = collective + np.column_stack([
        np.sqrt(3) / 2 * roll_mix - pitch_mix / 2,
        -np.sqrt(3) / 2 * roll_mix - pitch_mix / 2,
        pitch_mix,
    ])
    # The raw throttle wave precedes an integer receiver filter. Stay clear of
    # both knots so a few microseconds of input-filter offset cannot change dT/du.
    if nominal_commands.min() <= lower + 0.005 or nominal_commands.max() >= upper - 0.005:
        raise ValueError("Nominal mix approaches a thrust-table knot")

    projection = rotation[:, :2, :2].transpose(0, 2, 1) * [[1, -1], [-1, 1]]
    # Only the first recorded torque initializes the two unlogged integral
    # states. Every later transverse request is predicted from the feedback.
    integral = np.empty_like(pd)
    integral[0] = np.linalg.solve(projection[0], demand[0] / gain) - pd[0]
    integral_scale = 1 / (1 + 9 * np.minimum(np.abs(waves[:, 9]) / 360, 1) ** 2)
    integral_rate = ki * integral_scale[:, None] * error
    delta_heading = np.deg2rad((np.diff(waves[:, 6]) + 180) % 360 - 180)
    elapsed = np.diff(time)
    for i, angle in enumerate(delta_heading, start=1):
        c, s = np.cos(angle), np.sin(angle)
        transport = np.array([[c, s], [-s, c]])
        increment = (transport @ integral_rate[i - 1] + integral_rate[i]) * elapsed[i - 1] / 2
        integral[i] = np.clip(transport @ integral[i - 1] + increment, -0.075, 0.075)
    predicted = gain * np.einsum("nij,nj->ni", projection, pd + integral)

    heading_error = np.clip((waves[:, 7] - waves[:, 6] + 180) % 360 - 180,
                            -params[9], params[9])
    raw_target = np.clip(params[3] * heading_error, -params[6], params[6])
    target = np.empty_like(raw_target)
    target[0] = waves[0, 8]
    for i, step in enumerate(elapsed * params[7], start=1):
        target[i] = target[i - 1] + np.clip(raw_target[i] - target[i - 1], -step, step)
    # The allocator's anti-windup decision is not streamed at 2 kHz. Verify
    # the yaw control equation with its recorded post-update integral/filtered rate.
    yaw_torque = (params[16] * (params[4] * np.deg2rad(waves[:, 8] - waves[:, 10]) + waves[:, 14])
                  + params[18] * np.deg2rad(waves[:, 10]))
    return {
        "tilt_error_deg": residual(waves[:, 30], tilt_error),
        "roll_pitch_requested_Nm": residual(demand[1:], predicted[1:]),
        "roll_pitch_integral_initial_normalized": integral[0].tolist(),
        "roll_pitch_integral_peak_abs_normalized": np.max(np.abs(integral), axis=0).tolist(),
        "yaw_rate_target_dps": residual(waves[1:, 8], target[1:]),
        "yaw_requested_Nm_using_recorded_filter_and_integral": residual(waves[:, 11], yaw_torque),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--start", type=float, default=64)
    parser.add_argument("--end", type=float, default=124)
    args = parser.parse_args()
    metadata, session, time, waves, mask = load_hover(args.log, args.start, args.end)
    state = [s for t, s in session["states"].items() if args.start <= t / 1e6 < args.end]
    if not state or any(min(s[42], s[46]) < 0.9999 for s in state):
        raise ValueError("Window must have sampled lift and attitude allocation scales of one")
    result = replay(time[mask], waves[mask], metadata["hover_parameters"])
    print(json.dumps({"recording": metadata["id"], "parameter_revision": metadata["parameter_revision"],
                      "samples": int(mask.sum()), "window_s": [args.start, args.end],
                      **result}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
