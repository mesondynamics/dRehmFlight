#!/usr/bin/env python3
"""Inspect finalized Cyclone level-build RMPLOGs without replaying plant dynamics.

Use the recording metadata to verify the build; schema 1 alone is insufficient.
JSON goes to stdout. This tool never accesses hardware or writes parameters.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import zlib


BUILD = "drehm-cyclone-rotor-v2-level-20261105"
SUPPORTED_BUILDS = (BUILD, "drehm-cyclone-rotor-v3-entry-20261105",
                    "drehm-cyclone-rotor-v4-entry-hys-20261105",
                    "drehm-cyclone-rotor-v5-laser-20261105")
CHANNELS = (
    "S1_us", "servo_us", "mode", "phase_deg", "roll_input_us",
    "pitch_input_us", "heading_deg", "reference_deg", "target_rate_dps",
    "heading_rate_dps", "filtered_rate_dps", "requested_torque_Nm",
    "predicted_torque_Nm", "feedforward_torque_Nm", "integral_rad_s2",
    "roll_deg", "pitch_deg", "gyro_z_dps", "throttle_us", "yaw_input_us",
    "gyro_x_dps", "gyro_y_dps", "sensor_acc_z_g", "M2_dshot", "M4_dshot",
    "M3_dshot", "roll_torque_Nm", "pitch_torque_Nm", "period_us", "work_us",
    "tilt_error_deg", "abort_bits",
)


def read_recording(path):
    data = path.read_bytes()
    if data[:8] != b"RMPLOG1\n":
        raise ValueError("Not an RMPLOG1 recording")
    sessions = {}
    offset = 8
    frames = 0
    while offset < len(data):
        if len(data) - offset < 12:
            raise ValueError("Unfinished record header; stop recording first")
        _, size = struct.unpack_from("<QI", data, offset)
        offset += 12
        if size < 32 or size > len(data) - offset:
            raise ValueError("Truncated/invalid frame; stop recording first")
        frame = data[offset:offset + size]
        offset += size
        frames += 1
        if zlib.crc32(frame[:-4]) != struct.unpack_from("<I", frame, size - 4)[0]:
            raise ValueError(f"CRC mismatch at frame {frames}")
        boot = struct.unpack_from("<I", frame, 12)[0]
        timestamp = struct.unpack_from("<Q", frame, 20)[0]
        payload = frame[28:-4]
        session = sessions.setdefault(boot, {"rows": {}, "states": {}, "duplicates": 0})
        if frame[5] == 16:
            if len(payload) < 12:
                raise ValueError("Truncated telemetry header")
            schema, first, period, count, channels = struct.unpack_from("<IIHBB", payload)
            if schema != 1 or channels != len(CHANNELS) or period == 0:
                raise ValueError("Unexpected telemetry layout/rate")
            if len(payload) != 12 + count * channels * 4:
                raise ValueError("Unexpected telemetry payload size")
            for sample in range(count):
                sequence = first + sample
                values = struct.unpack_from("<32f", payload, 12 + sample * 128)
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("Nonfinite telemetry")
                row = (timestamp + sample * period, values)
                if sequence in session["rows"]:
                    if session["rows"][sequence] != row:
                        raise ValueError("Conflicting samples with the same boot/sequence")
                    session["duplicates"] += 1
                session["rows"][sequence] = row
        elif frame[5] == 17:
            if len(payload) < 8 or len(payload) != 8 + payload[4] * 8:
                raise ValueError("Unexpected state payload size")
            state = {}
            for index in range(payload[4]):
                key, kind, _, bits = struct.unpack_from("<HBBI", payload, 8 + index * 8)
                if kind not in (1, 2, 3) or key in state:
                    raise ValueError("Invalid state entry")
                state[key] = struct.unpack("<f", struct.pack("<I", bits))[0] if kind == 1 else bits
            session["states"][timestamp] = state
    return sessions, frames, hashlib.sha256(data).hexdigest()


def summarize(session, start, end):
    ordered = sorted(session["rows"].items())
    if any(b[1][0] <= a[1][0] for a, b in zip(ordered, ordered[1:])):
        raise ValueError("Source time is not monotonic within this boot")
    rows = [(seq, t, v) for seq, (t, v) in ordered if start <= t / 1e6 <= end]
    if not rows:
        raise ValueError("No telemetry in requested window")
    gaps = [(b[1] - a[1]) / 1e6 for a, b in zip(rows, rows[1:]) if b[0] != a[0] + 1]
    changes = []
    for seq, t, values in rows:
        if not changes or values[2] != changes[-1]["mode"]:
            changes.append({"time_s": t / 1e6, "mode": int(values[2])})
    state = [(t, v) for t, v in sorted(session["states"].items()) if start <= t / 1e6 <= end]
    expected = rows[-1][0] - rows[0][0] + 1
    # Wrapped heading change is an estimator output, not an independent measurement.
    delta = sum((b[2][6] - a[2][6] + 180) % 360 - 180 for a, b in zip(rows, rows[1:]))
    stats = {}
    for index, name in enumerate(CHANNELS):
        values = [row[2][index] for row in rows]
        item = {"first": values[0], "last": values[-1], "min": min(values), "max": max(values)}
        if index in (3, 6, 7):
            sine = statistics.mean(math.sin(math.radians(v)) for v in values)
            cosine = statistics.mean(math.cos(math.radians(v)) for v in values)
            item.update(circular_mean_deg=math.degrees(math.atan2(sine, cosine)),
                        circular_resultant=math.hypot(sine, cosine))
        elif index in (2, 31):
            item["counts"] = {str(int(v)): values.count(v) for v in sorted(set(values))}
        else:
            item.update(mean=statistics.mean(values), stddev=statistics.pstdev(values))
        stats[name] = item
    return {"samples": len(rows), "first_s": rows[0][1] / 1e6, "last_s": rows[-1][1] / 1e6,
            "first_sequence": rows[0][0], "last_sequence": rows[-1][0],
            "missing_samples": expected - len(rows), "gap_count": len(gaps),
            "max_gap_s": max(gaps, default=0),
            "recording_duplicates_in_boot": session["duplicates"],
            "statistics_warning": "Statistics weight received samples equally; missing intervals are not interpolated",
            "mode_changes": changes, "estimated_heading_change_deg": delta,
            "heading_change_warning": "Estimator output; gaps may hide turns; not independent ground truth",
            "last_state": state[-1][1] if state else None, "channels": stats}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--boot", type=int, required=True)
    parser.add_argument("--start", type=float, default=0)
    parser.add_argument("--end", type=float, default=math.inf)
    args = parser.parse_args()
    metadata = json.loads(args.metadata.read_text())
    metadata = metadata.get("recording", metadata)
    if metadata.get("state") != "complete":
        raise ValueError("Recording metadata must indicate a finalized recording")
    build = metadata.get("target", {}).get("build_id")
    if build not in SUPPORTED_BUILDS:
        raise ValueError("Unsupported build; inspect its channel mapping before analysis")
    if not math.isfinite(args.start) or args.end < args.start or math.isnan(args.end):
        raise ValueError("Invalid time window")
    sessions, frames, digest = read_recording(args.recording)
    if args.boot not in sessions:
        raise ValueError("Requested boot absent")
    result = {"recording_id": metadata["id"], "build_id": build, "boot": args.boot,
              "raw_sha256": digest, "validated_frames": frames, "crc_errors": 0,
              "boots": sorted(sessions), **summarize(sessions[args.boot], args.start, args.end)}
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
