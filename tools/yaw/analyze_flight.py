#!/usr/bin/env python3
"""Analyze RemoteMCU first-flight CSV. Offline only; never writes MCU parameters."""
import argparse
import csv
import json
import math
import statistics


def parameters(data):
    """Accept the checked-in initial model or an actual Agent GET snapshot."""
    if isinstance(data.get('parameters'), dict):
        return {int(k): float(v) for k, v in data['parameters'].items()}
    return {int(v['id']): float(v['value']) for v in data['values']}


def interpolate(table, index):
    index = max(0, min(len(table)-1, index))
    i = int(index)
    return table[i] if i+1 == len(table) else table[i]+(index-i)*(table[i+1]-table[i])


def dshot_command(value):
    # Invert the ACTUAL integer DShot value, not the pre-quantized target.
    return 0 if value < 48 else max(0, min(1, (value-48)/1999))


def least_squares(rows, targets):
    """Three-column QR with column scaling and explicit excitation rejection."""
    n = len(rows)
    if n < 12:
        raise ValueError('Need at least 12 contiguous integration windows')
    columns = [[r[j] for r in rows] for j in range(3)]
    scales = [math.sqrt(sum(x*x for x in c)) for c in columns]
    if min(scales) < 1e-10:
        raise ValueError('Insufficient excitation: zero regressor')
    q, r = [], [[0.0]*3 for _ in range(3)]
    for j in range(3):
        v = [x/scales[j] for x in columns[j]]
        # Reorthogonalization prevents near-collinear hover data becoming a fit.
        for _ in range(2):
            for k in range(j):
                projection = sum(a*b for a, b in zip(q[k], v))
                r[k][j] += projection
                v = [a-projection*b for a, b in zip(v, q[k])]
        r[j][j] = math.sqrt(sum(x*x for x in v))
        if r[j][j] < .02:
            raise ValueError('Insufficient independent excitation; hold-only data cannot identify all terms')
        q.append([x/r[j][j] for x in v])
    y = [sum(a*b for a, b in zip(col, targets)) for col in q]
    beta = [0.0]*3
    for j in (2, 1, 0):
        beta[j] = (y[j]-sum(r[j][k]*beta[k] for k in range(j+1, 3)))/r[j][j]
    return [b/s for b, s in zip(beta, scales)], min(r[j][j] for j in range(3))


def read_csv(path):
    with open(path, newline='') as stream:
        rows = []
        for row in csv.DictReader(stream):
            values = [float(row[f'ch{i:02}']) for i in range(1, 33)]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Non-finite or missing telemetry channel')
            rows.append({'boot': int(row['boot_id']), 'seq': int(row['sample_sequence']),
                         'time': int(row['target_timestamp_us'])/1e6, 'v': values})
    return rows


def analyze(samples, p, airborne, boot_id=None):
    """Use explicitly confirmed airborne times, in TARGET-clock seconds."""
    if not 0 <= airborne[0] < airborne[1]:
        raise ValueError('Invalid airborne interval')
    sessions = {s['boot'] for s in samples if airborne[0] <= s['time'] <= airborne[1]}
    if boot_id is None:
        if len(sessions) != 1:
            raise ValueError('Select exactly one session with --boot-id; never merge restarts')
        boot_id = next(iter(sessions))
    stream = [s for s in samples if s['boot'] == boot_id]
    selected = [s for s in stream if airborne[0] <= s['time'] <= airborne[1]]
    if len(selected) < 20:
        raise ValueError('Too few received airborne samples')
    values = [s['v'] for s in selected]
    errors = sorted(abs((v[7]-v[6]+180) % 360-180) for v in values)
    hold_errors = sorted(abs((v[7]-v[6]+180) % 360-180)
                         for v in values if abs(v[19]-1500) <= 20)
    gaps = duplicates = reversed_time = 0
    windows, targets, hover_scales = [], [], []
    previous = None
    thrust = None
    start = None
    accum = [0.0]*3
    prior_terms = None
    invalid_since = None
    geometry = [(0.5*p[20]-p[23], .8660254*p[20]-p[24], -.8660254, .5),
                (0.5*p[21]-p[23], -.8660254*p[21]-p[24], .8660254, .5),
                (-p[22]-p[23], -p[24], 0, -1)]
    for sample in stream:
        v, t = sample['v'], sample['time']
        dt = 0 if previous is None else t-previous['time']
        seq_delta = 0 if previous is None else (sample['seq']-previous['seq']) % 2**32
        contiguous = previous is not None and seq_delta == 1 and 0 < dt <= .03
        in_range = airborne[0] <= t <= airborne[1]
        if in_range and previous is not None:
            duplicates += seq_delta == 0
            reversed_time += dt <= 0
            gaps += not contiguous
        command_thrust = [interpolate([p[60+5*i+j] for j in range(5)],
                                     4*dshot_command(v[23+i])) for i in range(3)]
        if thrust is None or not contiguous:
            thrust = command_thrust
            invalid_since = t
        else:
            alpha = dt/(p[76]+dt)
            thrust = [a+alpha*(b-a) for a, b in zip(thrust, command_thrust)]
        # Restrict the Izz approximation to confirmed upright free flight.
        # The CSV has no armed STATE fields; airborne times must be supplied
        # after checking the raw recording's state and parameter revisions.
        valid = in_range and contiguous and t-invalid_since >= .5 and \
            max(abs(v[15]), abs(v[16])) <= 10 and \
            max(abs(v[20]), abs(v[21])) <= 20 and \
            all(48 < v[23+i] < 2047 for i in range(3)) and v[27] >= .999
        if not valid:
            start = prior_terms = None
            accum = [0.0]*3
            previous = sample
            continue
        torque_geometry = torque_reaction = lift = 0.0
        for i, (x, y, tx, ty) in enumerate(geometry):
            tilt = interpolate([p[30+9*i+j] for j in range(9)], (v[26]-50)/10)
            a = math.radians(tilt)
            torque_geometry += (x*ty-y*tx)*thrust[i]*math.sin(a)
            torque_reaction += (-1 if i != 1 else 1)*thrust[i]*math.cos(a)
            lift += thrust[i]*math.cos(a)
        omega = math.radians(v[17])
        terms = [torque_geometry, torque_reaction, -omega]
        if abs(v[22]-1) < .08 and max(abs(v[15]), abs(v[16])) < 5 and abs(v[17]) < 5 and lift > 0:
            hover_scales.append(p[15]*9.81/lift)
        if start is None:
            start = (t, omega)
        elif prior_terms is not None:
            accum = [a+.5*dt*(b+c) for a, b, c in zip(accum, prior_terms, terms)]
            if t-start[0] >= .30:
                windows.append(accum)
                targets.append(omega-start[1])
                start = (t, omega)
                accum = [0.0]*3
        prior_terms = terms
        previous = sample
    report = {
        'boot_id': boot_id, 'airborne_target_seconds': list(airborne), 'samples': len(selected),
        'sequence_or_time_discontinuities': gaps, 'duplicates': duplicates, 'reversed_time': reversed_time,
        'heading_abs_error_mean_deg': statistics.mean(errors),
        'heading_abs_error_p95_deg': errors[int(.95*(len(errors)-1))],
        'yaw_rate_peak_deg_s': max(abs(v[9]) for v in values),
        'servo_pwm_range_us': [min(v[2] for v in values), max(v[2] for v in values)],
        'max_work_us': max(v[30] for v in values), 'max_loop_period_us': max(v[31] for v in values),
        'integration_windows': len(windows),
        'limits': [
            'Uses independently confirmed airborne interval, build/profile and parameter snapshot.',
            'Common kappa only; cannot identify three independent motor curves from one hover.',
            'Izz and damping are conditional on thrust scale and assumed actuator lag.',
            'No parameter changes are applied. Check holdout residual, excitation and raw STATE counters.',
        ],
    }
    if len(hover_scales) >= 100:
        report['near_hover_common_thrust_scale_candidate'] = statistics.median(hover_scales)
        report['near_hover_scale_samples'] = len(hover_scales)
    report['centered_yaw_stick_samples'] = len(hold_errors)
    if hold_errors:
        report['centered_stick_heading_abs_error_mean_deg'] = statistics.mean(hold_errors)
        report['centered_stick_heading_abs_error_p95_deg'] = hold_errors[int(.95*(len(hold_errors)-1))]
    try:
        split = int(.7*len(windows))
        beta, excitation = least_squares(windows[:split], targets[:split])
        a, b, c = beta
        if a <= 0 or b <= 0 or c < 0:
            raise ValueError('Physically inconsistent fit; inspect excitation, lag, sign and flight interval')
        def rms(rows, yy):
            return math.sqrt(statistics.mean((sum(b*x for b, x in zip(beta, row))-y)**2
                                             for row, y in zip(rows, yy)))
        report['fit'] = {'status': 'candidate_only', 'izz_given_current_thrust_kg_m2': 1/a,
                         'common_kappa_m': b/a, 'damping_given_current_thrust_Nm_s_rad': c/a,
                         'excitation_ratio': excitation,
                         'train_delta_omega_rmse_rad_s': rms(windows[:split], targets[:split]),
                         'holdout_delta_omega_rmse_rad_s': rms(windows[split:], targets[split:])}
    except ValueError as error:
        report['fit'] = {'status': 'insufficient_or_inconsistent', 'reason': str(error)}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='RemoteMCU CSV exported from the new flight build')
    parser.add_argument('--parameters', required=True, help='Exact parameter GET snapshot or initial-model JSON')
    parser.add_argument('--airborne', nargs=2, type=float, required=True, metavar=('START_S', 'END_S'),
                        help='Confirmed airborne interval, target clock seconds; excludes ground contact')
    parser.add_argument('--boot-id', type=int)
    args = parser.parse_args()
    try:
        with open(args.parameters) as stream:
            p = parameters(json.load(stream))
        report = analyze(read_csv(args.csv), p, args.airborne, args.boot_id)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
