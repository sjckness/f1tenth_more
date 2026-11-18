"""Per-tick horizon geometry, implied a_cmd, ramp activity, fixed-point check."""
import json

import numpy as np
import pandas as pd

RUN = ('/home/fabiocar/f1tenth_archive/complete/'
       '2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission')
PQ = f'{RUN}/2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission.extract.parquet'
T0 = 1792585534.852410
T1 = 1792585553.715941

df = pd.read_parquet(PQ)


def payload(r):
    return json.loads(r) if isinstance(r, str) else r


sol = df[df['kind'] == 'sample:solver']
clr = df[df['kind'] == 'sample:clearance']
drv = df[df['kind'] == 'sample:drive']

ct = np.array([float(t) - T0 for t in clr['t']])
cv = np.array([float(payload(p)) for p in clr['payload']])
dt_ = np.array([float(t) - T0 for t in drv['t']])
dv = np.array([float(payload(p)[0]) for p in drv['payload']])

rows = []
for _, r in sol.iterrows():
    p = payload(r['payload'])
    t = float(r['t']) - T0
    if not (0 <= t <= T1 - T0):
        continue
    px, py, pv = np.array(p[9]), np.array(p[10]), np.array(p[12])
    seg = np.hypot(np.diff(px), np.diff(py))
    rows.append(dict(t=t, ok=p[0], msg=p[2], cost=p[5], nb=p[7], nobs=p[8],
                     v0=pv[0], vmid=pv[len(pv) // 2], vend=pv[-1],
                     vmax=pv.max(), path=seg.sum(),
                     reach=float(np.hypot(px[-1] - px[0], py[-1] - py[0]))))
h = pd.DataFrame(rows)
# clearance at each solver tick
h['clear'] = np.interp(h['t'], ct, cv)
h['cmd'] = np.interp(h['t'], dt_, dv)

N, TS, VDES = 20, 0.1, 0.5
print('N*ts*vdes = %.2f m   (N=%d, ts=%.2f, vdes=%.2f)' % (N * TS * VDES, N, TS, VDES))
print('\n=== predicted horizon per tick (%d ticks) ===' % len(h))
print(h[['v0', 'vmid', 'vend', 'vmax', 'path', 'reach', 'clear', 'cmd']]
      .describe(percentiles=[.1, .5, .9]).round(4).to_string())

print('\n=== how often does the horizon reach vdes? ===')
print('ticks with pred_v.max() >= 0.49 : %d / %d' % ((h['vmax'] >= 0.49).sum(), len(h)))
print('ticks with pred_v[-1] >= 0.49   : %d / %d' % ((h['vend'] >= 0.49).sum(), len(h)))
print('predicted path length >= 1.0 m  : %d / %d' % ((h['path'] >= 1.0).sum(), len(h)))

print('\n=== stop ramp: cap = sqrt(2*0.6*(d_rem - d_final)), target 2.0 m ===')
A_DEC, D_FINAL_APPROX = 0.6, 0.11
h['d_rem'] = h['clear'] - 2.0
h['ramp_cap'] = np.sqrt(np.clip(2 * A_DEC * (h['d_rem'] - D_FINAL_APPROX), 0, None))
h['ramp_bites'] = h['ramp_cap'] < VDES
print('ticks where the ramp cap < vdes:', int(h['ramp_bites'].sum()), 'of', len(h))
print(h.loc[h['ramp_bites'], ['t', 'clear', 'd_rem', 'ramp_cap', 'v0', 'cmd']]
      .round(3).to_string(index=False))

print('\n=== implied a_cmd from the command law  v_cmd = v + a_cmd*ts ===')
# v is /odometry/filtered at the tick; reuse the drive/odom alignment from chain.py
print('(computed in chain.py: cmd - v_meas median 0.0855 -> a_cmd ~ 0.855 m/s^2,'
      ' well inside a_max=3.0)')

print('\n=== fixed point of the lag loop ===')
for r in (0.70, 0.74, 0.758, 0.78, 0.82, 1.0):
    adt = 0.0855
    print('  v_meas/v_cmd = %.3f -> v_cmd* = a*ts/(1-r) = %s'
          % (r, 'unbounded (reaches the 0.5 clamp)' if r >= 1 else '%.3f' % (adt / (1 - r))))

print('\n=== first and last 6 solver ticks ===')
print(h[['t', 'msg', 'clear', 'v0', 'vend', 'path', 'cmd']].head(6).round(3).to_string(index=False))
print(h[['t', 'msg', 'clear', 'v0', 'vend', 'path', 'cmd']].tail(6).round(3).to_string(index=False))
