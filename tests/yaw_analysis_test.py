import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('yaw_analysis', ROOT/'tools/yaw/analyze_flight.py')
yaw = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(yaw)


class AnalysisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(ROOT/'docs/yaw-first-flight-model.json') as stream:
            cls.p = yaw.parameters(json.load(stream))
        p = cls.p
        cls.samples = []
        cls.truth = (.05, .012, .003)
        inertia, kappa, damping = cls.truth
        command = 48+63*1999//125
        thrust = yaw.interpolate([p[60+j] for j in range(5)], 4*yaw.dshot_command(command))
        rate = heading = old_torque = 0.0
        dt = .005
        for i in range(7000):
            t = i*dt
            angle = math.atan(kappa/(3*p[20]))+math.radians(.2)*math.sin(1.3*t)+math.radians(.1)*math.sin(.43*t)
            torque = thrust*(3*p[20]*math.sin(angle)-kappa*math.cos(angle))
            if i:
                # Independent trapezoidal rigid-body plant, exact for the
                # integrated regression. No controller/fit output is fed in.
                rate = ((1-damping*dt/(2*inertia))*rate+dt*(old_torque+torque)/(2*inertia))/(1+damping*dt/(2*inertia))
            old_torque = torque
            heading += math.degrees(rate)*dt
            tilt = math.degrees(angle)
            table = [p[30+j] for j in range(9)]
            logical = next(50+10*(j+(tilt-table[j])/(table[j+1]-table[j]))
                           for j in range(8) if table[j+1] <= tilt <= table[j])
            v = [0.0]*32
            v[2] = 512+1984*logical/180
            v[6] = v[7] = heading
            v[9] = v[17] = math.degrees(rate)
            v[19] = 1500
            v[22] = 1
            v[23:26] = [command]*3
            v[26], v[27], v[28], v[29], v[30], v[31] = logical, 1, 500, 270, 310, 500
            cls.samples.append({'boot': 5, 'seq': i, 'time': t, 'v': v})

    def test_independent_rigid_body_identification(self):
        report = yaw.analyze(self.samples, self.p, (1, 34))
        fit = report['fit']
        self.assertEqual(fit['status'], 'candidate_only')
        self.assertAlmostEqual(fit['izz_given_current_thrust_kg_m2'], self.truth[0], places=6)
        self.assertAlmostEqual(fit['common_kappa_m'], self.truth[1], places=6)
        self.assertAlmostEqual(fit['damping_given_current_thrust_Nm_s_rad'], self.truth[2], places=6)
        self.assertLess(fit['holdout_delta_omega_rmse_rad_s'], 1e-7)
        self.assertEqual(report['max_loop_period_us'], 500)

    def test_hover_is_not_independent_excitation(self):
        with self.assertRaisesRegex(ValueError, 'excitation'):
            yaw.least_squares([[1, 2, 3]]*30, [0]*30)

    def test_gaps_are_reported_and_not_bridged(self):
        samples = [s for i, s in enumerate(self.samples) if i % 100 != 0]
        report = yaw.analyze(samples, self.p, (1, 34))
        self.assertGreater(report['sequence_or_time_discontinuities'], 50)
        self.assertEqual(report['fit']['status'], 'insufficient_or_inconsistent')

    def test_restarts_need_explicit_session(self):
        with self.assertRaisesRegex(ValueError, 'session'):
            yaw.analyze(self.samples+[dict(s, boot=6) for s in self.samples], self.p, (1, 34))


if __name__ == '__main__':
    unittest.main()
