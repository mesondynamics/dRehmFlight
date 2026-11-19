#!/usr/bin/env python3
"""Compare pilot-confirmed airborne plateaus within a single v13 battery flight."""
import argparse
from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path
from statistics import fmean, median

from inspect_recording import HOVER_SLOPE_BUILD as BUILD, read_recording


G = 9.81
SETTLE_S = .25
MIN_WINDOW_S = .50
MAX_PAIR_GAP_S = 3.
MIN_CODE_STEP = 12.


def read_flight(path, metadata_path):
    metadata = json.loads(metadata_path.read_text())
    metadata = metadata.get("recording", metadata)
    target = metadata["target"]
    if (metadata["state"] != "complete" or metadata["dropped_frames"] or metadata["error"] or
            target["build_id"] != BUILD or target["sample_rate_hz"] != 100 or
            target["channels"] != 32 or target["schema_id"] != 1):
        raise ValueError("Expected a sealed 100 Hz v13 hardware recording with no recording drops")
    if metadata["mode"] != "hardware" or target["device_id"] != "teensy40-1881bf":
        raise ValueError("Incorrect data source")
    sessions, frames, digest = read_recording(path, include_parameter_pages=True)
    if len(sessions) != 1 or frames != metadata["frames"]:
        raise ValueError("Boot or frame-count mismatch")
    boot, session = next(iter(sessions.items()))
    if session.get("build_id") != BUILD:
        raise ValueError("On-wire firmware identity does not match the v13 measurement layout")
    return metadata, boot, session, digest


def time_mean(rows, channel):
    integral = sum((b[0] - a[0]) * (a[1][channel] + b[1][channel]) / 2
                   for a, b in zip(rows, rows[1:]))
    return integral / (rows[-1][0] - rows[0][0])


def level(rows, times, start, end):
    first = bisect_left(times, start)
    last = bisect_left(times, end)
    data = rows[first:last]
    settled = rows[bisect_left(times, start - SETTLE_S):last]
    # Demand a held level before the interval, allowing motor and observer settling.
    throttle = [v[18] for _, v in settled]
    raw_codes = [fmean(v[23:26]) for _, v in settled]
    if max(throttle) - min(throttle) > 5 or max(raw_codes) - min(raw_codes) > 12:
        raise ValueError(f"Collective input has not settled at {start:g} s")
    if any(max(abs(v[c] - 1500) for c in (4, 5, 19)) > 25 or abs(v[0] - 1500) > 30
           for _, v in settled):
        raise ValueError(f"Control sticks moved during plateau at {start:g} s")
    mean_g = time_mean(data, 11)
    code = time_mean(data, 12)
    raw_mean = fmean(fmean(v[23:26]) for _, v in data)
    if abs(raw_mean - code) > 3:
        raise ValueError(f"Logged and matched collective disagree at {start:g} s")
    if any(math.hypot(v[15], v[16]) > 12 for _, v in data):
        raise ValueError(f"Excessive roll/pitch for a near-level comparison at {start:g} s")
    # Within-plateau variation is a descriptive noise/drift scale, not an IID error bar.
    groups = [data[len(data)*i//3:len(data)*(i+1)//3] for i in range(3)]
    blocks = [time_mean(group, 11) for group in groups]
    return {"window_s": [start, end], "samples": len(data), "mean_up_g": mean_g,
            "mean_DShot": code, "mean_wire_DShot_M2_M4_M3": [time_mean(data, c) for c in (23, 24, 25)],
            "mean_throttle_us": time_mean(data, 18), "mean_servo_us": time_mean(data, 1),
            "mean_roll_deg": time_mean(data, 15), "mean_pitch_deg": time_mean(data, 16),
            "block_range_g": max(blocks) - min(blocks)}


def analyze(session, pairs, mass):
    if not pairs or not math.isfinite(mass) or mass <= 0:
        raise ValueError("Specify at least one airborne pair and a positive finite mass")
    ordered = sorted(session["rows"].items())
    sequence = [seq for seq, _ in ordered]
    rows = [(us / 1e6, v) for _, (us, v) in ordered]
    times = [t for t, _ in rows]
    if session["duplicates"] or any(a >= b for a, b in zip(times, times[1:])):
        raise ValueError("Duplicate samples or nonmonotonic device time")
    states = session["states"]
    state_times = sorted(states)
    accepted = []
    ranges = []
    for side, ref_start, ref_end, step_start, step_end in pairs:
        windows = [(ref_start, ref_end), (step_start, step_end)]
        if side not in ("up", "down") or not all(math.isfinite(x) for w in windows for x in w):
            raise ValueError("Invalid pair direction or times")
        early, late = sorted(windows)
        if (any(b - a < MIN_WINDOW_S for a, b in windows) or
                not SETTLE_S <= late[0] - early[1] <= MAX_PAIR_GAP_S):
            raise ValueError("Use separate, adjacent plateaus of at least 0.50 s after 0.25 s settling")
        start, end = early[0] - SETTLE_S, late[1]
        if any(a < end and start < b for a, b in ranges):
            raise ValueError("Repetitions must use independent, nonoverlapping time intervals")
        ranges.append((start, end))
    # Validate the transitions between repetitions too; a second arming is another flight.
    start, end = min(a for a, _ in ranges), max(b for _, b in ranges)
    first, last = bisect_left(times, start), bisect_left(times, end)
    if (last <= first + 1 or times[first] - start > .015 or end - times[last-1] > .015 or
            sequence[last-1] - sequence[first] != last - first - 1 or
            any(b - a > .015 for a, b in zip(times[first:last-1], times[first+1:last]))):
        raise ValueError("Incomplete 100 Hz waveform across selected maneuvers")
    for _, v in rows[first:last]:
        if (v[2] != 0 or v[31] != 0 or v[12] <= 0 or
                any(code != int(code) or not 63 <= code < 2047 for code in v[23:26])):
            raise ValueError("Expected ordinary powered flight with a valid observer and no abort or clipped motor")
    lo = bisect_right(state_times, start * 1e6) - 1
    hi = bisect_left(state_times, end * 1e6)
    if lo < 0 or hi == len(state_times):
        raise ValueError("State snapshots must bracket selected maneuvers")
    stamps = state_times[lo:hi+1]
    if any(b - a > 1.5e6 for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Missing state snapshots")
    snapshots = [states[us] for us in stamps]
    for s in snapshots:
        if (s[1], s[4], s[6], s[10], s[11], s[19], s[37], s[47]) != (1, 1, 0, 1, 0, 0, 0, 0):
            raise ValueError("Unexpected arming, IMU, fault, flight, link or spin state")
    for key in (17, 24, 26, 27, 28, 29, 30):
        if len({s[key] for s in snapshots}) != 1:
            raise ValueError("Loop overrun, control fault or IMU diagnostic count changed")
    revisions = {s[12] for s in snapshots}
    if len(revisions) != 1:
        raise ValueError("Parameter revision changed")
    for side, ref_start, ref_end, step_start, step_end in pairs:
        windows = [(ref_start, ref_end), (step_start, step_end)]
        ref, step = [level(rows, times, a, b) for a, b in windows]
        delta_code = step["mean_DShot"] - ref["mean_DShot"]
        if (delta_code if side == "up" else -delta_code) < MIN_CODE_STEP:
            raise ValueError("Emitted collective step is too small or opposite to selected direction")
        delta_g = step["mean_up_g"] - ref["mean_up_g"]
        delta_f = mass * G * delta_g
        accepted.append({"side": side, "reference": ref, "step": step, "delta_DShot": delta_code,
                         "delta_up_g": delta_g, "delta_force_N": delta_f,
                         "slope_N_per_code": delta_f / delta_code,
                         "above_plateau_variation": abs(delta_g) > ref["block_range_g"] + step["block_range_g"]})
    reasons = []
    if len(accepted) < 3 or len({item["side"] for item in accepted}) < 2:
        reasons.append("Need at least three distinct pairs spanning both directions")
    reference_codes = [p["reference"]["mean_DShot"] for p in accepted]
    if max(reference_codes) - min(reference_codes) > .5 * min(abs(p["delta_DShot"]) for p in accepted):
        reasons.append("Reference levels differ by more than half the smallest tested step")
    if any(abs(p["reference"]["mean_up_g"] - 1) > .10 for p in accepted):
        reasons.append("Reference contains sustained vertical acceleration above 0.10 g")
    if any(not item["above_plateau_variation"] for item in accepted):
        reasons.append("Force contrast overlaps within-plateau variation")
    slopes = [p["slope_N_per_code"] for p in accepted]
    slope = median(slopes)
    if min(slopes) <= 0:
        reasons.append("Estimated force does not increase in every pair")
    # Use other repetitions to predict each force contrast, to expose divergent local responses.
    if len(slopes) > 1:
        errors = [(median(slopes[:i] + slopes[i+1:]) * p["delta_DShot"] - p["delta_force_N"])**2
                  for i, p in enumerate(accepted)]
        energy = sum(p["delta_force_N"]**2 for p in accepted)
        normalized_error = math.sqrt(sum(errors) / energy) if energy > 0 else None
        if normalized_error is None or normalized_error > .4:
            reasons.append("Leave-one-pair-out relative RMS error exceeds 40% or is undefined")
    else:
        normalized_error = None
    return {"mass_kg": mass, "parameter_revision": revisions.pop(), "pairs": accepted,
            "reference_mean_DShot_range": [min(reference_codes), max(reference_codes)],
            "median_vertical_N_per_mean_DShot_code": slope,
            "vertical_N_per_code_pair_range": [min(slopes), max(slopes)],
            "leave_one_pair_out_relative_rms": normalized_error,
            "within_flight_repeatability": "descriptive_only" if reasons else "passes_internal_checks",
            "limitations": reasons, "scope": "Net nongravitational vertical force within this flight. "
            "Rotor attribution assumes little change in other external vertical forces. "
            "Independent flights are needed to test transfer across batteries and conditions."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--metadata", type=Path, help="Defaults to adjacent .json")
    parser.add_argument("--mass-kg", type=float, default=.75)
    parser.add_argument("--pair", nargs=5, action="append", required=True,
                        metavar=("SIDE", "REF_START", "REF_END", "STEP_START", "STEP_END"),
                        help="Mark pilot-confirmed airborne levels (up or down vs reference, in device seconds)")
    args = parser.parse_args()
    try:
        metadata, boot, session, digest = read_flight(args.recording, args.metadata or args.recording.with_suffix(".json"))
        pairs = [(side, *(float(v) for v in rest)) for side, *rest in args.pair]
        result = analyze(session, pairs, args.mass_kg)
        revision = result["parameter_revision"]
        first = min(min(pair[1], pair[3]) for pair in pairs) - SETTLE_S
        count = metadata["target"]["parameter_count"]
        entries = {}
        for us, rev, total, start, page in session.get("parameter_pages", []):
            if rev == revision and us / 1e6 <= first:
                if total != count:
                    raise ValueError("Inconsistent preflight parameter count")
                for index, entry in enumerate(page, start):
                    if entries.setdefault(index, entry) != entry:
                        raise ValueError("Conflicting preflight parameter readbacks")
        if set(entries) != set(range(count)) or len({item[0] for item in entries.values()}) != count:
            raise ValueError("Incomplete preflight parameter readback")
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    print(json.dumps({"recording_id": metadata["id"], "build": BUILD, "boot": boot,
                      "sha256": digest, **result}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
