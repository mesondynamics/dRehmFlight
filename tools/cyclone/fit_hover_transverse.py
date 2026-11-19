"""Fit local Roll/Pitch responses across separate, sealed v11/v12 ordinary hovers.

Every score is angular-rate RMSE in deg/s, 20 nominal 100 Hz samples ahead.
All origins are used to avoid choosing one sampling phase. Adjacent windows
overlap and are not independent replicates. These are closed-loop records.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_record import load_hover, motor_commands_from_wire


HORIZON = 20


def load_window(spec):
    log, start, end = spec
    meta, _, time, waves, mask = load_hover(Path(log), float(start), float(end), allow_v11=True)
    codes = waves[:, 23:26]
    motor_commands_from_wire(codes, meta["target"]["build_id"])  # Check the version's wire codebook.
    # Both builds log the actual DShot code sent to the ESC. The older build
    # quantizes before transmission. Do not try to undo those 16-code steps.
    m2, m4, m3 = codes.T
    commanded = np.column_stack((np.sqrt(3) / 2 * (m2 - m4), m3 - (m2 + m4) / 2))
    tau = meta["hover_parameters"][76]
    if tau <= 0 or time[mask][0] - time[0] < 6 * tau:
        raise ValueError("Need a positive motor lag and sufficient actuator history")
    effective = commanded.copy()
    for i, weight in enumerate(-np.expm1(-np.diff(time) / tau), start=1):
        effective[i] = effective[i - 1] + weight * (commanded[i - 1] - effective[i - 1])
    t = time[mask]
    if len(t) < 10 * HORIZON or np.max(abs(t[HORIZON:] - t[:-HORIZON] - .2)) > .004:
        raise ValueError("Insufficient window length or irregular forecast horizon")
    return dict(id=meta["id"], boot=meta["boot"], sha256=meta["raw_sha256"],
                start=float(start), end=float(end), motor_lag_s=tau,
                t=t, rate=waves[mask, 20:22], angle=waves[mask, 15:17], wire=effective[mask])


def forecast_matrix(window, axis, include_wire):
    cols = [window["rate"][:-HORIZON, axis], window["angle"][:-HORIZON, axis]]
    if include_wire:
        cols.append(window["wire"][:-HORIZON, axis])
    return np.column_stack([*cols, np.ones(len(cols[0]))])


def plant_terms(window, damping):
    # Conditional replay of zdot = -d*z + k*filtered_wire + bias.
    # Only this simulation uses the recorded controls *within* the interval.
    t = window["t"]
    count = len(t) - HORIZON
    weighted_wire = np.zeros((count, 2))
    for i in range(HORIZON):
        dt = t[i+1:i+count+1] - t[i:i+count]
        decay = np.exp(-damping * dt)
        gain = -np.expm1(-damping * dt) / damping if damping else dt
        weighted_wire = decay[:, None] * weighted_wire + gain[:, None] * window["wire"][i:i+count]
    delta = t[HORIZON:] - t[:-HORIZON]
    bias_gain = -np.expm1(-damping * delta) / damping if damping else delta
    initial = np.exp(-damping * delta)[:, None] * window["rate"][:-HORIZON]
    return initial, weighted_wire, bias_gain


def fit_plant(window):
    target = window["rate"][HORIZON:]
    best_cost = [np.inf, np.inf]
    best = [None, None]
    for damping in np.arange(0., 20.25, .25):
        initial, wire, bias = plant_terms(window, damping)
        for axis in range(2):
            matrix = np.column_stack((wire[:, axis], bias))
            y = target[:, axis] - initial[:, axis]
            coeff = np.linalg.lstsq(matrix, y, rcond=None)[0]
            if coeff[0] < 0:
                coeff = np.array([0., bias @ y / (bias @ bias)])
            cost = np.mean((matrix @ coeff - y) ** 2)
            if cost < best_cost[axis]:
                best_cost[axis] = cost
                best[axis] = (damping, *coeff)
    return best


def run(train, validation):
    boots = [window["boot"] for window in [train, *validation]]
    if len(set(boots)) != len(boots):
        raise ValueError("Training and validation require distinct flight boots")
    coeff = {}
    for name, has_wire in (("state", False), ("state_and_wire", True)):
        coeff[name] = []
        for axis in range(2):
            matrix = forecast_matrix(train, axis, has_wire)
            fit, _, rank, _ = np.linalg.lstsq(matrix, train["rate"][HORIZON:, axis], rcond=None)
            if rank != matrix.shape[1]:
                raise ValueError("Insufficient independent variation for the transverse fit")
            coeff[name].append(fit)
    plant = fit_plant(train)
    windows = []
    for w in [train, *validation]:
        target = w["rate"][HORIZON:]
        predictions = {"persist": w["rate"][:-HORIZON], "zero_rate": np.zeros_like(target)}
        for name, has_wire in (("state", False), ("state_and_wire", True)):
            predictions[name] = np.column_stack([
                forecast_matrix(w, j, has_wire) @ coeff[name][j] for j in range(2)])
        replay = np.empty_like(target)
        for j, (damping, gain, offset) in enumerate(plant):
            initial, wire, bias = plant_terms(w, damping)
            replay[:, j] = initial[:, j] + gain * wire[:, j] + offset * bias
        predictions["conditional_plant_replay"] = replay
        description = {key: w[key] for key in ("id", "boot", "sha256", "start", "end", "motor_lag_s")}
        windows.append(dict(**description, overlapping_origins=len(target),
                            rmse_dps={name: np.sqrt(np.mean((pred - target)**2, axis=0))
                                      for name, pred in predictions.items()}))
    return dict(horizon_s=.2, axis_order=["roll", "pitch"],
                forecast_columns=["initial_rate_dps", "initial_angle_deg", "optional_filtered_wire_code", "one"],
                forecast_coefficients=coeff,
                plant_columns=["damping_per_s", "gain_dps2_per_differential_DShot_code", "bias_dps2"],
                conditional_plant_coefficients=plant, train=windows[0], held_out=windows[1:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", nargs=3, required=True, metavar=("LOG", "START", "END"))
    parser.add_argument("--validate", nargs=3, action="append", required=True, metavar=("LOG", "START", "END"))
    args = parser.parse_args()
    try:
        result = run(load_window(args.train), [load_window(s) for s in args.validate])
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, default=lambda x: x.tolist(), allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
