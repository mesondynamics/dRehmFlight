"""Offline checks for using one battery flight's airborne plateau contrasts."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "cyclone"))
from fit_collective_slope import G, analyze


def flight():
    session = {"rows": {}, "states": {}, "duplicates": 0}
    sides = ("up", "down", "up")
    pairs = []
    for n, side in enumerate(sides):
        base = 1 + n * 4
        pairs.append((side, base, base + .75, base + 1.5, base + 2.25))
    for n in range(1300):
        us = n * 10000
        t = us / 1e6
        code = 700
        for side, _, _, start, end in pairs:
            if start - .4 <= t < end:
                code += 30 if side == "up" else -24
        v = [0.] * 32
        for i in (0, 1, 4, 5, 19):
            v[i] = 1500
        v[11] = 1 + (code - 700) * .002
        v[12] = code
        v[18] = 1327 + (code - 700) / 2
        v[23:26] = [code + 8, code - 3, code - 5]
        session["rows"][n] = (us, v)
        if n % 20 == 0:
            state = dict.fromkeys(range(1, 60), 0)
            state.update({1: 1, 4: 1, 10: 1, 12: 3})
            session["states"][us] = state
    return session, pairs


class CollectiveSlopeTest(unittest.TestCase):
    def test_known_up_down_response(self):
        session, pairs = flight()
        fit = analyze(session, pairs, .75)
        self.assertAlmostEqual(fit["median_vertical_N_per_mean_DShot_code"], .002 * G * .75)
        self.assertEqual(fit["within_flight_repeatability"], "passes_internal_checks")
        self.assertAlmostEqual(fit["leave_one_pair_out_relative_rms"], 0)

    def test_one_pair_or_one_side_is_only_descriptive(self):
        session, pairs = flight()
        for chosen in (pairs[:1], pairs[::2]):
            self.assertEqual(analyze(session, chosen, .75)["within_flight_repeatability"], "descriptive_only")

    def test_change_in_wrong_direction_and_reused_windows(self):
        session, pairs = flight()
        with self.assertRaisesRegex(ValueError, "opposite"):
            analyze(session, [("down", *pairs[0][1:])], .75)
        with self.assertRaisesRegex(ValueError, "nonoverlapping"):
            analyze(session, [pairs[1], pairs[1]], .75)

    def test_inconsistent_force_response_is_descriptive(self):
        session, pairs = flight()
        for us, v in session["rows"].values():
            if 10.45e6 < us < 11.25e6:
                v[11] = 1.32
        fit = analyze(session, pairs, .75)
        self.assertGreater(fit["leave_one_pair_out_relative_rms"], .4)
        self.assertEqual(fit["within_flight_repeatability"], "descriptive_only")

    def test_reject_sample_gap_fault_and_unmatched_observer(self):
        session, pairs = flight()
        broken = deepcopy(session)
        del broken["rows"][120]
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            analyze(broken, pairs, .75)
        broken = deepcopy(session)
        broken["states"][1200000][4] = 0
        with self.assertRaisesRegex(ValueError, "IMU"):
            analyze(broken, pairs, .75)
        broken = deepcopy(session)
        broken["states"][4000000][1] = 0  # After the first pair, before the next.
        with self.assertRaisesRegex(ValueError, "arming"):
            analyze(broken, pairs, .75)
        for us, v in session["rows"].values():
            if 1e6 <= us < 1.75e6:
                v[12] += 12
        with self.assertRaisesRegex(ValueError, "disagree"):
            analyze(session, pairs, .75)


if __name__ == "__main__":
    unittest.main()
