"""Quantify inertial increments and the spectrum visible in a sealed 100 Hz v12 hover."""
import argparse
import json
from pathlib import Path

import numpy as np

from hover_record import load_hover


def band_rms(signals, low, high):
    # Welch integration on the nominal 100 Hz sequence. No inference above Nyquist.
    n = 1000
    x = np.arange(n) - (n - 1) / 2
    window = np.hanning(n)
    freq = np.fft.rfftfreq(n, .01)
    band = (freq >= low) & (freq < high)
    energies = []
    for first in range(0, len(signals) - n + 1, n // 2):
        y = signals[first:first + n]
        residual = y - y.mean(axis=0) - x[:, None] * ((x @ y) / (x @ x))
        spectrum = np.fft.rfft(residual * window[:, None], axis=0)
        energies.append(2 * (abs(spectrum[band]) ** 2).sum(axis=0) / (n * (window @ window)))
    if not energies:
        raise ValueError("The spectrum requires at least ten seconds of hover")
    return np.sqrt(np.mean(energies, axis=0))


def feedback(t, v):
    roll, pitch = np.deg2rad(v[:, 15:17]).T
    p, q, r = v[:, [20, 21, 17]].T
    heading_rate = (np.sin(roll) * q + np.cos(roll) * r) / np.cos(pitch)
    rates = np.column_stack((p + np.sin(pitch) * heading_rate,
                             np.cos(roll) * q - np.sin(roll) * r, heading_rate))
    angles = np.column_stack((v[:, 15:17], np.rad2deg(np.unwrap(np.deg2rad(v[:, 6])))))
    gyro_integral = np.zeros_like(angles)
    gyro_integral[1:] = np.cumsum((rates[1:] + rates[:-1]) * .5 * np.diff(t)[:, None], axis=0)
    obs = angles[50:] - angles[:-50]
    pred = gyro_integral[50:] - gyro_integral[:-50]
    increments = angles[20:, :2] - angles[:-20, :2]
    lengths = np.linalg.norm(increments, axis=1)
    biggest = np.argmax(lengths)
    z_window_sd = np.lib.stride_tricks.sliding_window_view(v[:, 22], 21).std(axis=1)
    spectra = {}
    for lo, hi in ((.5, 2), (5, 15)):
        raw, filtered, z = band_rms(v[:, [9, 10, 22]], lo, hi)
        spectra[f"{lo}-{hi}_Hz"] = {"yaw_rate_filtered_to_input_rms": filtered / raw if raw else None,
                                     "sensor_z_rms_g": z}
    heading_error = (v[:, 7] - v[:, 6] + 180) % 360 - 180
    return {
        "samples": len(v), "roll_pitch_sd_deg": v[:, 15:17].std(axis=0),
        "heading_error_rms_deg": np.sqrt(np.mean(heading_error ** 2)),
        "heading_error_max_abs_deg": max(abs(heading_error)),
        "gyro_vs_fusion_half_second_corr": [np.corrcoef(a, b)[0, 1] if a.std() and b.std() else None
                                            for a, b in zip(obs.T, pred.T)],
        "gyro_vs_fusion_increment_rms_error_deg": np.sqrt(np.mean((obs - pred) ** 2, axis=0)),
        "tilt_increment_200ms_median_p99_max_deg": np.percentile(lengths, [50, 99, 100]),
        "strongest_increment_start_s": t[biggest], "roll_pitch_change_deg": increments[biggest],
        "sensor_z_sd_g": v[:, 22].std(), "abs_sensor_z_above_1p2g": int((abs(v[:, 22]) > 1.2).sum()),
        "correlation_200ms_sensor_z_sd_tilt_increment":
            np.corrcoef(z_window_sd, lengths)[0, 1] if z_window_sd.std() and lengths.std() else None,
        "apparent_spectral_bands": spectra,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--start", type=float, default=64)
    parser.add_argument("--end", type=float, default=124)
    args = parser.parse_args()
    metadata, _, time, waves, mask = load_hover(args.log, args.start, args.end)
    if mask.sum() < 1000:
        parser.error("Use at least ten seconds for the spectral and increment comparisons")
    result = feedback(time[mask], waves[mask])
    result.update(sha256=metadata["raw_sha256"], window_s=[args.start, args.end])
    print(json.dumps(result, default=lambda x: x.tolist(), allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
