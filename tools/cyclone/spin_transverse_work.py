#!/usr/bin/env python3
"""Audit v19 spin transverse work from RMPLOG and a parameter-readback JSON.

Run with Python and NumPy. JSON goes to stdout. The parameters are a supplied
reference. The reconstructed actuator moments and inertia-based kinetic energy
are model estimates. Only the wire commands and IMU rates were recorded.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from hover_model_audit import lagged, model_from_parameters, wrench
from inspect_recording import read_recording, SPIN_RAMP_LASER_BUILD


def spin_windows(t, waves):
    """At 100 Hz, flag the first 300 ms with trailing 500 ms gyro RMS >=20 deg/s."""
    xy_rate_squared = np.sum(waves[:, 20:22] ** 2, axis=1)
    rms_squared = np.convolve(xy_rate_squared, np.ones(50) / 50, "full")[:len(t)]
    spin = waves[:, 2] == 1
    edges = np.flatnonzero(np.diff(np.r_[False, spin, False]))
    windows = []
    for start, stop in edges.reshape(-1, 2):
        window = {"spin_start_s": t[start], "spin_end_s": t[stop] if stop < len(t) else None}
        windows.append(window)
        if stop - start < 30 or stop == len(t):
            window["excluded"] = "short or unfinished spin"
            continue
        qualified = np.convolve((rms_squared[start:stop] >= 400).astype(int),
                                np.ones(30, dtype=int), "valid")
        hits = np.flatnonzero(qualified == 30)
        if not len(hits):
            window["excluded"] = "no sustained gyro onset"
            continue
        onset = start + hits[0]
        window.update(onset_s=t[onset], signed_onset_rpm=waves[onset, 9] / 6,
                      exit_mode=int(waves[stop, 2]), exit_bits=int(waves[stop, 31]))
        zeros = np.flatnonzero(np.any(waves[start:stop, 23:26] == 0, axis=1))
        first_zero = start + zeros[0] if len(zeros) else stop
        end = min(onset + 100, first_zero, stop)
        if end - onset < 2:
            window["excluded"] = "first zero motor code precedes usable onset interval"
        else:
            window["indices"] = (onset, end)
    return windows


def analyze(recording, parameter_file):
    sessions, frames, digest = read_recording(recording, include_parameter_pages=True)
    if len(sessions) != 1:
        raise ValueError("Expected a single recorded boot")
    boot, session = next(iter(sessions.items()))
    if session.get("build_id") != SPIN_RAMP_LASER_BUILD:
        raise ValueError("Expected a v19 identity in the recording")
    rows = sorted(session["rows"].items())
    if len(rows) < 50 or np.any(np.diff([seq for seq, _ in rows]) != 1):
        raise ValueError("Insufficient data or a gap in waveform sequence numbers")
    t = np.array([us / 1e6 for _, (us, _) in rows])
    waves = np.array([vals for _, (_, vals) in rows])
    dt = np.diff(t)
    if np.any(abs(dt - .01) > .003) or abs(np.median(dt) - .01) > .0005:
        raise ValueError("Expected continuous nominal 100 Hz source timestamps")
    params_raw = parameter_file.read_bytes()
    snapshot = json.loads(params_raw)
    if not snapshot["ok"]:
        raise ValueError("Parameter readback did not succeed")
    p = {row["id"]: row["value"] for row in snapshot["values"]}
    geometry = model_from_parameters(p)
    inertia, thrust_table = geometry[3:5]
    rates = np.deg2rad(waves[:, [20, 21, 17]])
    gyro_torque = p[96] * np.cross(rates, rates * inertia)[:, :2]
    lead = np.clip(rates[:, 2] * p[76] * p[115], -1, 1)
    feedforward = gyro_torque + lead[:, None] * gyro_torque[:, [1, 0]] * [1, -1]
    # NormalYawControl adds this term only in Spin, Braking and Aligning.
    normal = waves[:, 2] == 0
    gyro_torque[normal] = 0
    feedforward[normal] = 0
    codes = waves[:, 23:26]
    if np.any((codes != np.floor(codes)) | (codes < 0) | (codes > 2047) |
              ((codes > 0) & (codes < 48))):
        raise ValueError("Invalid wire motor code")
    commands = np.maximum(0, (codes - 48) / 1999)
    thrust = np.column_stack([np.interp(u, np.linspace(0, 1, 5), table)
                              for u, table in zip(commands.T, thrust_table)])
    servo = (waves[:, 1] - 512.) * 180 / 1984
    wire_torque = wrench(thrust, servo, geometry)[1][:, :2]
    kinetic = .5 * np.sum(rates[:, :2] ** 2 * inertia[:2], axis=1)
    windows = spin_windows(t, waves)
    xy = rates[:, :2]
    for w in windows:
        if "indices" not in w:
            continue
        start, end = w["indices"]
        if t[start] - t[0] < 6 * max(p[12], p[76], .060):
            raise ValueError("Insufficient actuator history before onset")
        omega = xy[start:end]
        duration = t[end - 1] - t[start]
        intervals = dt[start:end - 1]
        radius2 = np.sum(omega ** 2, axis=1)
        area2 = np.sum(omega[:-1, 0] * omega[1:, 1] - omega[:-1, 1] * omega[1:, 0])
        # Integral of x*dy - y*dx divided by the integrated squared magnitude.
        angular_area_time = np.sum(intervals * (radius2[:-1] + radius2[1:])) / 2
        w.update(first_last_sample_s=[t[start], t[end - 1]], samples=int(end - start),
                 mean_heading_hz=float(np.mean(waves[start:end, 9]) / 360),
                 signed_transverse_gyro_hz=area2 / (2 * np.pi * angular_area_time),
                 kinetic_change_mW=1000 * (kinetic[end - 1] - kinetic[start]) / duration,
                 predictions={})

    # Commands between the 100 Hz samples are unknown. Try both endpoints.
    for endpoint in ("right", "left"):
        def response(signal, tau):
            inputs = signal if endpoint == "right" else np.concatenate((signal[:1], signal[:-1]))
            return lagged(inputs, t, tau)

        servo_estimate = response(servo, p[12])
        for tau in dict.fromkeys((p[76], .060)):
            moment = wrench(response(thrust, tau), servo_estimate, geometry)[1][:, :2]
            gyro_estimate = response(feedforward, tau)
            gyro_without_lead = response(gyro_torque, tau)
            for w in windows:
                if "indices" not in w:
                    continue
                start, end = w["indices"]
                duration = t[end - 1] - t[start]

                def power(torque):
                    samples = np.sum(torque[start:end] * xy[start:end], axis=1)
                    return 500 * np.sum(dt[start:end - 1] * (samples[:-1] + samples[1:])) / duration

                total, gyro = power(moment), power(gyro_estimate)
                w["wire_torque_mW"] = power(wire_torque)
                w["predictions"][f"{endpoint}_{1000*tau:.1f}ms"] = {
                    "total_mW": total, "gyro_plus_lead_mW": gyro, "remaining_mW": total - gyro,
                    "gyro_without_lead_mW": power(gyro_without_lead)}
    for w in windows:
        w.pop("indices", None)
    return {"build": session["build_id"], "boot": boot, "frames": frames, "samples": len(t),
            "recording_sha256": digest, "parameter_sha256": hashlib.sha256(params_raw).hexdigest(),
            "parameter_reference_revision": snapshot["revision"],
            "in_recording_parameter_pages": len(session.get("parameter_pages", [])),
            "reference_inertia_kg_m2": inertia.tolist(), "reference_motor_lag_s": p[76],
            "reference_servo_lag_s": p[12], "gyro_feedforward_scale": p[96],
            "cyclic_lead_scale": p[115], "spins": windows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("parameters", type=Path, help="reference RemoteMCU parameter readback JSON")
    args = parser.parse_args()
    print(json.dumps(analyze(args.recording, args.parameters), indent=2, allow_nan=False))
