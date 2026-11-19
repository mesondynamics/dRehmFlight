"""Read a continuous, fixed-throttle v12 ordinary-hover interval and its on-board parameters."""
import json
from pathlib import Path

import numpy as np

from inspect_recording import read_recording


def load_hover(log: Path, start: float, end: float):
    metadata = json.loads(log.with_suffix(".json").read_text())
    metadata = metadata.get("recording", metadata)
    target = metadata["target"]
    if (metadata["state"] != "complete" or metadata["error"] or metadata["dropped_frames"] or
            target["build_id"] != "drehm-cyclone-rotor-v12-dshot-resolution-20261119" or
            target["sample_rate_hz"] != 100):
        raise ValueError("Expected a finalized v12 100 Hz recording without recorder drops")
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
