#!/usr/bin/env python3
"""Compare equal-duration neutral-stick hover windows (device seconds, not GPS)."""
import argparse
import json
import math
import statistics as st
from pathlib import Path
from inspect_recording import read_recording

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('log', type=Path)
p.add_argument('boot', type=int)
p.add_argument('start', type=float)
p.add_argument('--seconds', type=float, default=20)
a = p.parse_args()
meta = json.loads(a.log.with_suffix('.json').read_text())
sessions, frames, digest = read_recording(a.log)
if (meta['state'] != 'complete' or meta['error'] or meta['frames'] != frames or
        meta['target']['build_id'] != 'drehm-cyclone-rotor-v10-arm-guard-20261119'):
    p.error('not a finalized v10 recording')
rows = sorted((seq,t/1e6,v) for seq,(t,v) in sessions[a.boot]['rows'].items()
              if a.start <= t/1e6 < a.start+a.seconds)
if (not rows or rows[-1][1]-rows[0][1] < a.seconds-.025 or
        any(j[0]!=i[0]+1 or not 0 < j[1]-i[1] < .018 for i,j in zip(rows,rows[1:]))):
    p.error('incomplete or discontinuous window')
v = [r[2] for r in rows]
if any(r[2] != 0 or r[31] or min(r[23:26]) <= 48 or
       max(abs(r[k]-1500) for k in (4,5,19)) > 20 for r in v):
    p.error('not a neutral-stick, powered normal-mode window')
def col(i): return [r[i] for r in v]
if max(col(18))-min(col(18)) > 3: p.error('throttle not steady')
def rms(x): return math.sqrt(st.mean(y*y for y in x))
print(json.dumps(dict(record=meta['id'],sha256=digest,start=a.start,seconds=a.seconds,
    samples=len(v),throttle_us=st.mean(col(18)),pitch_sd_deg=st.pstdev(col(16)),
    roll_sd_deg=st.pstdev(col(15)),pitch_rate_rms_dps=rms(col(21)),
    yaw_error_rms_deg=rms([(r[6]-r[7]+180)%360-180 for r in v]),
    motor_step_rms_DShot=[rms([y-x for x,y in zip(col(k),col(k)[1:])]) for k in (23,24,25)]),indent=2))
