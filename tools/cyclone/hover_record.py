"""Read a continuous, fixed-throttle ordinary-hover interval and its on-board parameters."""
import json
from pathlib import Path

import numpy as np

from inspect_recording import read_recording


V11 = "drehm-cyclone-rotor-v11-entry-888-20261119"
V12 = "drehm-cyclone-rotor-v12-dshot-resolution-20261119"

# Flight-disjoint windows for the 2026-11-19 ordinary-hover identification.
HOVER_TRAIN = ("18e969bdb4687f80-24", 64, 124)
HOVER_HOLDOUT = (("18e96796850df0e0-15", 240, 260), ("18e9684085984998-18", 160, 209.75))


def motor_commands_from_wire(codes, build_id):
    """Recover the flight version's thrust-table input from emitted throttle codes."""
    codes = np.asarray(codes)
    if not np.all((codes == 0) | ((codes >= 63) & (codes <= 2047) & (codes == np.floor(codes)))):
        raise ValueError("Unexpected DShot throttle code")
    u = np.maximum(0., (codes.astype(float) - 48) / 1999.)
    if build_id == V12:
        return u
    if build_id != V11:
        raise ValueError("Unknown motor command mapping")
    steps = np.rint(u * 125).astype(int)
    if np.any(np.where(steps == 0, 0, 48 + steps * 1999 // 125) != codes):
        raise ValueError("Throttle code is outside the v11 output lattice")
    return steps / 125.


def load_hover(log: Path, start: float, end: float, *, allow_v11=False):
    metadata = json.loads(log.with_suffix(".json").read_text())
    metadata = metadata.get("recording", metadata)
    target = metadata["target"]
    builds = (V11, V12) if allow_v11 else (V12,)
    if (metadata["state"] != "complete" or metadata["error"] or metadata["dropped_frames"] or
            target["build_id"] not in builds or
            target["sample_rate_hz"] != 100):
        raise ValueError("Expected a finalized 100 Hz recording of an accepted build without drops")
    sessions, frames, digest = read_recording(log, include_parameter_pages=True)
    if len(sessions) != 1 or frames != metadata["frames"]:
        raise ValueError("Expected a single boot and the complete frame count")
    boot, session = next(iter(sessions.items()))
    rows = sorted(session["rows"].items())
    if (not rows or rows[-1][0] - rows[0][0] + 1 != len(rows) or session["duplicates"]):
        raise ValueError("Missing or duplicate waveform samples")
    time = np.array([us for _, (us, _) in rows]) / 1e6
    waves = np.array([v for _, (_, v) in rows])
    mask = (time >= start) & (time < end)
    selected = waves[mask]
    if (not np.all(np.diff(time) > 0) or len(selected) < 2 or
            time[mask][0] - start > .015 or end - time[mask][-1] > .015 or
            np.max(np.diff(time[mask])) > .015):
        raise ValueError("Unsupported sample timing or incomplete analysis interval")
    if (np.any(selected[:, [2, 31]] != 0) or np.any(selected[:, 23:26] <= 48) or
            np.any(np.abs(selected[:, [4, 5, 19]] - 1500) > 20) or
            np.any(np.abs(selected[:, 0] - 1500) > 30) or np.ptp(selected[:, 18]) != 0):
        raise ValueError("Expected ordinary powered flight, neutral sticks and fixed throttle")
    # Include bracketing states so a very short window cannot escape a revision change.
    state_times = sorted(session["states"])
    lo = max(0, int(np.searchsorted(state_times, start * 1e6, side="right")) - 1)
    hi = int(np.searchsorted(state_times, end * 1e6))
    if hi >= len(state_times) or state_times[lo] > start * 1e6:
        raise ValueError("Missing bracketing state snapshots")
    revisions = {session["states"][us][12] for us in state_times[lo:hi + 1]}
    if len(revisions) != 1:
        raise ValueError("Parameter revision changed during the analysis interval")
    revision = revisions.pop()
    total = target["parameter_count"]
    entries = {}
    for us, page_revision, size, first, page in session.get("parameter_pages", []):
        if page_revision == revision and us / 1e6 <= start:
            if size != total:
                raise ValueError("Inconsistent parameter count")
            for index, entry in enumerate(page, first):
                if entries.setdefault(index, entry) != entry:
                    raise ValueError("Conflicting parameter readbacks")
    params = {key: value for key, _, value in entries.values()}
    if len(params) != total or set(entries) != set(range(total)):
        raise ValueError("Missing complete preflight parameter readback for this revision")
    metadata.update(boot=boot, raw_sha256=digest, parameter_revision=revision, hover_parameters=params)
    return metadata, session, time, waves, mask
