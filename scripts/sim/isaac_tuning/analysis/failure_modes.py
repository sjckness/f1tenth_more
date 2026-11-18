#!/usr/bin/env python3
"""How the dodge runs that are not good fail: where the cylinder is at the closest approach.

    failure_modes.py <variant dir> [<variant dir> ...]

For every D run that is not good (metrics.good_enough) the instant of minimum body clearance
(footprint rectangle to the cylinder's circle, ground truth) and, at that instant: where the
cylinder is in the car frame (front: ahead of the nose's probes; side; rear: behind the axle
midpoint), the car's speed, whether the cylinder moves towards the car's side, whether the
LiDAR hard stop had engaged. Prints the counts per variant and one line per run.
"""
import bisect
import collections
import csv
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import metrics  # noqa: E402


def analyse(run_dir):
    r = json.load(open(os.path.join(run_dir, 'run.json')))
    if not r.get('valid') or metrics.good_enough(r):
        return None
    meta = json.load(open(os.path.join(run_dir, 'meta.json')))
    t0 = meta['t_running']
    gt = [x for x in csv.DictReader(open(os.path.join(run_dir, 'gt.csv')))
          if float(x['t']) >= t0 and x['x'] != 'nan' and abs(float(x['v'])) < 20]
    mv = collections.defaultdict(list)
    for x in csv.DictReader(open(os.path.join(run_dir, 'movers.csv'))):
        mv[x['id']].append((float(x['t']), float(x['x']), float(x['y'])))
    info = {o['id']: o for o in json.load(open(os.path.join(run_dir, 'instance.json')))['obstacle_info']}
    best = (1e9, None)
    for g in gt:
        t, x, y, yaw = float(g['t']), float(g['x']), float(g['y']), float(g['yaw'])
        for oid, tr in mv.items():
            if oid not in info:
                continue
            ts = [p[0] for p in tr]
            i = max(0, min(bisect.bisect_right(ts, t), len(tr)) - 1)
            ox, oy = tr[i][1], tr[i][2]
            c = metrics.footprint_circle_clearance(x, y, yaw, ox, oy, info[oid]['radius_m'])
            if c < best[0]:
                j, k = min(i + 4, len(tr) - 1), max(i - 4, 0)
                dt = (tr[j][0] - tr[k][0]) or 1.0
                vx, vy = (tr[j][1] - tr[k][1]) / dt, (tr[j][2] - tr[k][2]) / dt
                dx, dy = ox - x, oy - y
                bx = math.cos(yaw) * dx + math.sin(yaw) * dy
                by = -math.sin(yaw) * dx + math.cos(yaw) * dy
                vby = -math.sin(yaw) * vx + math.cos(yaw) * vy
                best = (c, {'t': t - t0, 'v_car': float(g['v']), 'bx': bx, 'by': by, 'vby': vby})
    b = best[1]
    if b is None:
        return None
    where = 'front' if b['bx'] > 0.35 else ('rear' if b['bx'] < -0.1 else 'side')
    return {'run': os.path.basename(run_dir), 'outcome': r['outcome'], 'body_m': best[0],
            't_s': b['t'], 'v_car': b['v_car'], 'where': where,
            'closing_on_side': b['by'] * b['vby'] < 0,
            'hard_stop': (r.get('safety') or {}).get('hard_stop_engaged')}


def main(argv):
    for vdir in argv:
        rows = [a for a in (analyse(d) for d in sorted(glob.glob(os.path.join(vdir, 'D*_r*')))
                            if os.path.exists(os.path.join(d, 'run.json'))) if a]
        cnt = collections.Counter((x['where'], x['outcome']) for x in rows)
        speed = collections.Counter('stopped' if x['v_car'] < 0.1 else ('slow' if x['v_car'] < 0.4
                                                                         else 'full') for x in rows)
        print(f'== {vdir}: {len(rows)} runs not good; where x outcome {dict(cnt)}; '
              f'car speed {dict(speed)}; closing on the side '
              f'{sum(1 for x in rows if x["closing_on_side"])}; hard stop '
              f'{sum(1 for x in rows if x["hard_stop"])}')
        for x in rows:
            print(f'   {x["run"]:<12} {x["outcome"]:<9} body {x["body_m"]:+.2f} m  t {x["t_s"]:5.1f} s  '
                  f'v {x["v_car"]:+.2f}  {x["where"]:<5} closing {x["closing_on_side"]} '
                  f'hard_stop {x["hard_stop"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
