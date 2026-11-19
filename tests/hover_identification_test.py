"""Numerical regressions for the offline hover models (NumPy 2)."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "cyclone"))
from calibrate_hover_thrust import projection_bounds
from fit_hover_yaw import play
from hover_record import V11, V12, motor_commands_from_wire


class HoverIdentificationTest(unittest.TestCase):
    def test_v11_codebook_and_v12_native_counts(self):
        steps = np.arange(126)
        codes = np.where(steps == 0, 0, 48 + steps * 1999 // 125)
        np.testing.assert_allclose(motor_commands_from_wire(codes, V11), steps / 125)
        np.testing.assert_allclose(motor_commands_from_wire([700, 701], V12), np.array([652, 653]) / 1999)
        with self.assertRaises(ValueError):
            motor_commands_from_wire([700], V11)

    def test_two_degrees_of_reversal_travel(self):
        command = np.array([0., 1., 3., 2., 1., 0., -3., -2., -1., 0.])
        np.testing.assert_array_equal(play(command, 2.), [0, 0, 2, 2, 2, 1, -2, -2, -2, -1])
        np.testing.assert_array_equal(play(command, 0.), command)

    def test_projection_extremum_inside_the_gap(self):
        # An upright nominal rotor can reach exactly vertical inside its clearance.
        tilt = np.deg2rad(.5)
        low, high = projection_bounds(np.cos(tilt), -np.sin(tilt), np.deg2rad(1.))
        self.assertAlmostEqual(low, np.cos(np.deg2rad(1.5)))
        self.assertAlmostEqual(high, 1.)


if __name__ == "__main__":
    unittest.main()
