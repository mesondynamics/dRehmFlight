#!/usr/bin/env python3
"""Fit a local yaw response and compare common linkage play across separate hover recordings.

The fixed-collective model is dr/dt = gain * tilt + bias - damping * r,
with measured body-Z gyro r (deg/s) and estimated rotor-axis tilt (deg).
Only combinations of torque and inertia are fitted. Each 0.5 s replay uses
the initial measured rate and the recorded subsequent command sequence.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_model_audit import lagged
from hover_record import HOVER_HOLDOUT, HOVER_TRAIN, load_hover


# Full reversal travels (the two tooth faces are width degrees apart).
PLAY_WIDTHS = (0.0, 1.0, 2.0, 4.0)
DAMPING = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 12.0)
STEPS = 50                         # v11/v12 waveform is nominally 100 Hz


def play(tilt, width):
    """One rate-independent play element, applied once after the servo lag."""
    out = tilt.copy()
    half = width / 2
    for i in range(1, len(tilt)):
        out[i] = max(tilt[i] - half, min(out[i - 1], tilt[i] + half))
    return out


def hover(path, start, end, assumed_servo_lag=None):
    meta, _, t, wave, keep = load_hover(path, start, end, allow_v11=True)
    p = meta["hover_parameters"]
    maps = np.array([[p[30 + i * 9 + j] for j in range(9)] for i in range(3)])
    if not np.allclose(maps, maps[0]):
        raise ValueError("Common-angle analysis requires the same measured linkage map")
    # Replay the preceding PWM history to initialize both lag and play at takeoff.
    # The fit uses PWM only; v11 and v12 motor command quantization is irrelevant here.
    logical = (wave[:, 1] - 512.) * 180 / 1984
    tau = p[12] if assumed_servo_lag is None else assumed_servo_lag
    if not np.isfinite(tau) or tau <= 0:
        raise ValueError("Servo lag must be finite and positive")
    tilt = np.interp(lagged(logical, t, tau), np.arange(50., 131., 10.), maps[0])
    first = np.flatnonzero(keep)[0]
    if t[first] - t[0] < 6 * tau or np.ptp(tilt[:first]) <= max(PLAY_WIDTHS):
        raise ValueError("Insufficient pre-window servo history to initialize lag and play")
    outputs = {w: play(tilt, w)[keep] for w in PLAY_WIDTHS}
    return t[keep], wave[keep, 17], outputs, dict(id=meta["id"], sha256=meta["raw_sha256"],
        boot=meta["boot"], window_s=[start, end], collective_us=float(wave[keep, 18][0]),
        servo_lag_readback_s=p[12], assumed_servo_lag_s=tau,
        tilt_command_range_deg=[min(tilt[keep]), max(tilt[keep])])


def blocks(times, rate, tilt, damping, stride=STEPS):
    """Exact damped rate propagation under piecewise constant, previously sampled inputs."""
    starts = np.arange(0, len(rate) - STEPS, stride)
    edges = starts[:, None] + np.arange(STEPS)
    dt = times[edges + 1] - times[edges]
    remaining = times[starts + STEPS, None] - times[edges + 1]
    weights = dt if damping == 0 else np.exp(-damping * remaining) * (-np.expm1(-damping * dt) / damping)
    natural = rate[starts] * np.exp(-damping * (times[starts + STEPS] - times[starts]))
    design = np.column_stack((np.sum(weights * tilt[edges], axis=1), np.sum(weights, axis=1)))
    return design, rate[starts + STEPS] - natural


def estimate(design, target, use_servo):
    # Physics fixes the sign of the tilt-to-yaw gain. A zero optimum means
    # the fit could not find a response with that sign for this model.
    if use_servo:
        params = np.linalg.lstsq(design, target, rcond=None)[0]
        if params[0] >= 0:
            return params
    exposure = design[:, 1]
    return np.array([0., exposure @ target / (exposure @ exposure)])


def origin_scores(error):
    """All origins overlap; each phase alone uses nonoverlapping half-second windows."""
    ms = error**2
    phase_rms = [np.sqrt(np.mean(ms[i::STEPS])) for i in range(STEPS)]
    return dict(windows=len(ms[::STEPS]), rms_dps=phase_rms[0],
                mean_error_dps=error[::STEPS].mean(), overlapping_windows=len(ms),
                pooled_all_origins_rms_dps=np.sqrt(ms.mean()), phase_rms_dps=phase_rms)


def fit(training, validations, width, use_servo):
    t, r, angles, _ = training
    best = None
    for damping in DAMPING:
        x, y = blocks(t, r, angles[width], damping)
        coeff = estimate(x, y, use_servo)
        error = y - x @ coeff
        loss = np.sqrt(np.mean(error**2))
        if best is None or loss < best[0]:
            best = (loss, damping, coeff)
    loss, damping, coeff = best
    scores = []
    for tv, rv, a, _ in validations:
        x, y = blocks(tv, rv, a[width], damping, stride=1)
        scores.append(origin_scores(y - x @ coeff))
    return dict(full_play_deg=width if use_servo else None, gain_per_s2=coeff[0],
                bias_dps2=coeff[1], damping_per_s=damping, training_rms_dps=loss, validation=scores)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path, help="Directory of sealed hardware RMPLOG and JSON pairs")
    parser.add_argument("--assumed-servo-lag-s", type=float,
                        help="Offline sensitivity check; default is the preflight servo lag in each log")
    args = parser.parse_args()
    train, *tests = [hover(args.recordings / (name + ".rmplog"), a, b, args.assumed_servo_lag_s)
                     for name, a, b in (HOVER_TRAIN, *HOVER_HOLDOUT)]
    cases = [fit(train, tests, 0, False)] + [fit(train, tests, w, True) for w in PLAY_WIDTHS]
    persistence = []
    for t, r, angles, _ in [train, *tests]:
        _, delta = blocks(t, r, angles[0], 0, stride=1)
        persistence.append(origin_scores(delta))
    result = dict(recordings=[h[3] for h in [train, *tests]], horizon_samples=STEPS,
                  persistence=persistence, fitted_models=cases)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
