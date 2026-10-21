"""Time-align /drive, /ackermann_drive, /odom, /odometry/filtered, clearance, solver."""
import json
import sqlite3
import sys

import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RUN = ('/home/fabiocar/f1tenth_archive/complete/'
       '2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission')
BAG = f'{RUN}/bag/bag_0.db3'
PQ = (f'{RUN}/2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission'
      '.extract.parquet')
MISSION_START = 1792585534.852410   # manifest mission_start_time
MISSION_END = 1792585553.715941

con = sqlite3.connect(f'file:{BAG}?mode=ro', uri=True)
topics = {name: (tid, typ) for tid, name, typ in
          con.execute('select id, name, type from topics')}

WANT = ['/drive', '/ackermann_drive', '/odom', '/odometry/filtered',
        '/ekf_global/odometry/filtered', '/costmap/front_clearance',
        '/mission/status', '/mpc/solver_status', '/mpc/reverse_status',
        '/mpc/hold']
series = {}
for name in WANT:
    if name not in topics:
        print(f'MISSING from bag: {name}')
        continue
    tid, typ = topics[name]
    cls = get_message(typ)
    rows = []
    for ts, raw in con.execute(
            'select timestamp, data from messages where topic_id=? order by timestamp',
            (tid,)):
        rows.append((ts / 1e9, deserialize_message(bytes(raw), cls)))
    series[name] = rows
    print(f'{name}: {len(rows)} msgs')
con.close()


def win(rows, t0=MISSION_START, t1=MISSION_END):
    return [(t, m) for t, m in rows if t0 <= t <= t1]


def arr(rows, f):
    return np.array([(t - MISSION_START, f(m)) for t, m in rows], dtype=float)


drive = arr(win(series['/drive']), lambda m: m.drive.speed)
ack = arr(win(series['/ackermann_drive']), lambda m: m.drive.speed)
odom = arr(win(series['/odom']), lambda m: m.twist.twist.linear.x)
filt = arr(win(series['/odometry/filtered']), lambda m: m.twist.twist.linear.x)
glob = arr(win(series['/ekf_global/odometry/filtered']),
           lambda m: m.twist.twist.linear.x)
clear = arr(win(series['/costmap/front_clearance']), lambda m: m.data)

print('\n=== mission window', round(MISSION_END - MISSION_START, 2), 's ===')
for name, a in [('/drive cmd', drive), ('/ackermann_drive', ack),
                ('/odom vx', odom), ('/odometry/filtered vx', filt),
                ('/ekf_global vx', glob)]:
    v = a[:, 1]
    print(f'{name:24s} n={len(v):4d} mean={v.mean():.4f} median={np.median(v):.4f} '
          f'max={v.max():.4f} p90={np.percentile(v, 90):.4f} zeros={(np.abs(v) < 1e-6).sum()}')

print('\nclearance: min %.3f max %.3f first %.3f last %.3f'
      % (clear[:, 1].min(), clear[:, 1].max(), clear[0, 1], clear[-1, 1]))

# steady window: drive command above 0.05 and not the last 2 s (the stop)
mask = (drive[:, 1] > 0.05) & (drive[:, 0] < drive[-1, 0] - 2.0)
steady = drive[mask]
print('\nsteady cmd window: t %.2f..%.2f  n=%d mean=%.4f median=%.4f'
      % (steady[0, 0], steady[-1, 0], len(steady), steady[:, 1].mean(),
         np.median(steady[:, 1])))
t0s, t1s = steady[0, 0], steady[-1, 0]


def in_steady(a):
    v = a[(a[:, 0] >= t0s) & (a[:, 0] <= t1s), 1]
    return v


print('\n=== steady-segment medians and ratios ===')
hops = [('/drive (MPC out)', in_steady(drive)), ('/ackermann_drive (mux out)', in_steady(ack)),
        ('/odom vx (VESC erpm)', in_steady(odom)),
        ('/odometry/filtered vx', in_steady(filt)), ('/ekf_global vx', in_steady(glob))]
prev = None
for name, v in hops:
    med = float(np.median(v))
    ratio = '' if prev is None else f'  ratio to prev = {med / prev:.3f}'
    print(f'{name:28s} median={med:.4f} mean={float(v.mean()):.4f} n={len(v)}{ratio}')
    prev = med

# cmd - v_meas coupling test (nearest-neighbour align odom to each cmd)
oi = np.searchsorted(odom[:, 0], drive[mask, 0])
oi = np.clip(oi, 1, len(odom) - 1)
vmeas = odom[oi, 1]
diff = drive[mask, 1] - vmeas
print('\ncmd - v_meas(/odom): mean %.4f median %.4f std %.4f  (a_max*ts at 0.1 s?)'
      % (diff.mean(), np.median(diff), diff.std()))
corr = np.corrcoef(drive[mask, 1], vmeas)[0, 1]
print('corr(cmd, v_meas) = %.3f' % corr)

# solver pred_v
df = pd.read_parquet(PQ)
sol = df[df['kind'] == 'sample:solver']
rows = []
for _, r in sol.iterrows():
    p = json.loads(r['payload']) if isinstance(r['payload'], str) else r['payload']
    rows.append((float(r['t']) - MISSION_START, p[0], p[1], p[2], p[5], p[12]))
print('\n=== solver (%d ticks in file) ===' % len(rows))
inwin = [r for r in rows if 0 <= r[0] <= MISSION_END - MISSION_START]
print('in mission window:', len(inwin))
import collections
print('status_message:', collections.Counter(r[3] for r in inwin).most_common())
print('success rate: %.2f' % np.mean([r[1] for r in inwin]))
pv = np.array([r[5] for r in inwin], dtype=float)
print('pred_v shape', pv.shape)
print('pred_v[0]  median %.4f max %.4f' % (np.median(pv[:, 0]), pv[:, 0].max()))
print('pred_v[-1] median %.4f max %.4f' % (np.median(pv[:, -1]), pv[:, -1].max()))
print('pred_v row medians:', np.round(np.median(pv, axis=0), 3).tolist())
