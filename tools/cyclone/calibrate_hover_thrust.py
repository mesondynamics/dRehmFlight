#!/usr/bin/env python3
"""Estimate one common rotor thrust operating point from weight balance in ordinary hover."""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_model_audit import lagged, model_from_parameters
from hover_record import HOVER_HOLDOUT, HOVER_TRAIN, load_hover, motor_commands_from_wire


G = 9.81  # Same gravitational acceleration as the v12 audit, m/s².


def time_mean(t, values):
    return np.trapezoid(values, t, axis=0) / (t[-1] - t[0])


def projection_bounds(a, b, half_width):
    """Bounds of a*cos(error)+b*sin(error) for a near-vertical thrust direction."""
    if np.any(a <= 0) or not 0 <= half_width < np.pi / 2:
        raise ValueError("Projection bound requires upright flight and play below 180 degrees")
    low = a * np.cos(half_width) - abs(b) * np.sin(half_width)
    peak = np.clip(np.arctan2(b, a), -half_width, half_width)
    high = a * np.cos(peak) + b * np.sin(peak)
    if np.any(low <= 0):
        raise ValueError("Ambiguous upward force direction")
    return low, high


def summarize(directory, window, mass, play_width):
    record, start, end = window
    path = directory / (record + ".rmplog")
    meta, _, time, waves, mask = load_hover(path, start, end, allow_v11=True)
    p = meta["hover_parameters"]
    _, tangents, _, _, curves, tilt = model_from_parameters(p)
    # Identical motor response and tilt maps are the assumptions of this calibration.
    if not (np.allclose(curves, curves[0]) and np.allclose(tilt, tilt[0])):
        raise ValueError("This calibration requires the common nominal actuator maps")
    if time[mask][0] - time[0] < 6 * max(p[12], p[76]):
        raise ValueError("Insufficient actuator history before hover")
    codes = waves[:, 23:26]
    build = meta["target"]["build_id"]
    firmware_cmd = motor_commands_from_wire(codes, build)  # Also validates the codebook.
    # ESC-side input, after v11's coarse quantizer or v12's native mapping.
    wire_cmd = np.maximum(0, codes - 48) / 1999.
    motor_thrust = np.interp(wire_cmd, np.linspace(0, 1, 5), curves[0])
    thrust = lagged(motor_thrust, time, p[76])[mask]
    logical = lagged((waves[:, 1] - 512) * 180 / 1984, time, p[12])[mask]
    angle = np.deg2rad(np.interp(logical, np.arange(50, 131, 10), tilt[0]))[:, None]
    t = time[mask]
    roll, pitch = np.deg2rad(waves[mask, 15:17]).T
    up = np.column_stack((-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)))
    vertical = up[:, 2, None]
    lateral = up @ tangents.T
    # The vertical projection of each rotor is a*cos(play)+b*sin(play).
    a = vertical * np.cos(angle) + lateral * np.sin(angle)
    b = lateral * np.cos(angle) - vertical * np.sin(angle)
    nominal_lift = time_mean(t, np.sum(a * thrust, axis=1))
    nominal_total = time_mean(t, np.sum(thrust, axis=1))
    weight = G * mass
    scale = weight / nominal_lift
    axial = scale * nominal_total
    common_min, common_max = projection_bounds((a * thrust).sum(axis=1), (b * thrust).sum(axis=1),
                                                np.deg2rad(play_width / 2))
    # Conservative companion bound when the three joints can rest on different flanks.
    arm_min, arm_max = projection_bounds(a, b, np.deg2rad(play_width / 2))
    duration = t[-1] - t[0]
    wire_levels = codes[mask]
    collective = wire_levels.mean(axis=1)
    result = {
        "record": record, "sha256": meta["raw_sha256"], "boot": meta["boot"],
        "revision": meta["parameter_revision"], "build": build, "window_s": [start, end],
        "samples": int(mask.sum()), "duration_between_samples_s": duration,
        "throttle_us": int(waves[mask, 18][0]),
        "averaged_DShot_by_rotor_M2_M4_M3": time_mean(t, wire_levels),
        "DShot_by_rotor_min_max_sd": np.stack((wire_levels.min(axis=0), wire_levels.max(axis=0),
                                                wire_levels.std(axis=0)), axis=1),
        "combined_mean_DShot": time_mean(t, collective),
        "instantaneous_three_rotor_mean_DShot_min_max_sd":
            [collective.min(), collective.max(), collective.std()],
        "combined_mean_wire_command": time_mean(t, wire_cmd[mask]).mean(),
        "combined_mean_firmware_quantized_command": time_mean(t, firmware_cmd[mask]).mean(),
        "assumed_weight_N": weight,
        "nominal_mean_upward_N": nominal_lift, "nominal_sum_of_axial_thrusts_N": nominal_total,
        "mean_upward_per_axial": nominal_lift / nominal_total,
        "local_scale_to_weight": scale, "equivalent_total_axial_N": axial,
        "equivalent_axial_per_rotor_N": axial / 3,
        "axial_N_using_equal_instantaneous_rotor_thrust": weight / time_mean(t, a.mean(axis=1)),
        "axial_N_bounds_only_positivity_and_projection_with_independent_play":
            [weight / np.max(arm_max), weight / np.min(arm_min)],
        "axial_N_range_common_play_given_nominal_load_shares":
            [weight * nominal_total / time_mean(t, common_max),
             weight * nominal_total / time_mean(t, common_min)],
        "axial_N_range_independent_play_given_nominal_load_shares":
            [weight * nominal_total / time_mean(t, (thrust * arm_max).sum(axis=1)),
             weight * nominal_total / time_mean(t, (thrust * arm_min).sum(axis=1))],
        "vertical_force_sensitivity": {
            "N_per_extra_gram": G / 1000,
            "N_per_m_s_end_minus_start_vertical_velocity": mass / duration,
            "N_per_N_additional_upward_external_support": -1.0,
        },
    }
    return result, curves[0]


def calibrate(directory, mass, play_width):
    if not np.isfinite(mass) or mass <= 0 or not np.isfinite(play_width) or not 0 <= play_width < 180:
        raise ValueError("Mass must be positive and play must be a finite total output angle in [0, 180)")
    train, curve = summarize(directory, HOVER_TRAIN, mass, play_width)
    comparisons = []
    for window in HOVER_HOLDOUT:
        test, other_curve = summarize(directory, window, mass, play_width)
        if not np.array_equal(curve, other_curve):
            raise ValueError("Different prior thrust curves would invalidate this reference comparison")
        prediction = train["local_scale_to_weight"] * test["nominal_mean_upward_N"]
        # Contrast against a locally flat axial-force predictor to expose the unverified prior slope.
        flat = train["equivalent_total_axial_N"] * test["mean_upward_per_axial"]
        residual = prediction - mass * G
        comparisons.append({**test, "frozen_scaled_prior_upward_N": prediction,
                            "frozen_scaled_prior_minus_weight_N": residual,
                            "frozen_scaled_prior_relative_error_pct": 100 * residual / (mass * G),
                            "flat_local_baseline_upward_N": flat})
    return {
        "scope": "Local aggregate hover calibration conditional on negligible mean external vertical force "
                 "and near-equal initial and final vertical speed. Output only; no new flight parameters.",
        "mass_kg": mass, "g_m_s2": G, "total_tilt_output_free_play_deg": play_width,
        "train": train, "same_mass_other_flights": comparisons,
        "note": "Each equivalent per-rotor value is one third of total. The flight gives one aggregate "
                "constraint, not three independent measured thrusts. Held-out weight balances check "
                "transfer near hover, without independent force, height or voltage measurements.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path, help="Directory with the three sealed RMPLOG/JSON pairs")
    parser.add_argument("--mass-kg", type=float, default=.75)
    parser.add_argument("--play-width-deg", type=float, default=2, help="Full output-side clearance")
    args = parser.parse_args()
    try:
        result = calibrate(args.recordings, args.mass_kg, args.play_width_deg)
    except (KeyError, OSError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, default=lambda value: value.tolist(), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
