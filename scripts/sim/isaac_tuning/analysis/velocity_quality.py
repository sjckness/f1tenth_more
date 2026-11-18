#!/usr/bin/env python3
"""How good the semantic layer's velocity is on moving cylinders (dodge / M runs of one variant).

    velocity_quality.py <variant dir> [...]

For every /costmap/semantic_obstacles disk (obstacles.csv, recorder) whose centre, taken to
map_sim through the ground-truth car pose, lies within 0.45 m of a cylinder moving faster than
0.15 m/s: the share flagged v_valid, the velocity error of the valid ones against the ground
truth (movers.csv, central difference over +-3 samples), and the disk-to-centre distance.
"""
import bisect
import collections
import csv
import glob
import json
import math
import os
import statistics
import sys


def main(argv):
    for vdir in argv:
        valid, err, rel, pos = [], [], [], []
        for d in sorted(glob.glob(os.path.join(vdir, '[DM]*_r*'))):
            if not os.path.exists(os.path.join(d, 'meta.json')):
                continue
            t0 = json.load(open(os.path.join(d, 'meta.json'))).get('t_running')
            if t0 is None:
                continue
            gt = [(float(x['t']), float(x['x']), float(x['y']), float(x['yaw']))
                  for x in csv.DictReader(open(os.path.join(d, 'gt.csv'))) if x['x'] != 'nan']
            gts = [g[0] for g in gt]
            mv = collections.defaultdict(list)
            for x in csv.DictReader(open(os.path.join(d, 'movers.csv'))):
                mv[x['id']].append((float(x['t']), float(x['x']), float(x['y'])))
            for x in csv.DictReader(open(os.path.join(d, 'obstacles.csv'))):
                if not x['x'] or x.get('id') in (None, '', '-1') or float(x['t']) < t0:
                    continue
                t = float(x['t'])
                _, cx, cy, cyaw = gt[min(len(gt) - 1, bisect.bisect_left(gts, t))]
                ox, oy = float(x['x']), float(x['y'])
                gx = cx + math.cos(cyaw) * ox - math.sin(cyaw) * oy
                gy = cy + math.sin(cyaw) * ox + math.cos(cyaw) * oy
                best = None
                for tr in mv.values():
                    ts = [p[0] for p in tr]
                    j = max(0, min(bisect.bisect_right(ts, t), len(tr)) - 1)
                    dd = math.hypot(tr[j][1] - gx, tr[j][2] - gy)
                    if dd < 0.45 and (best is None or dd < best[0]):
                        k1, k0 = min(j + 3, len(tr) - 1), max(j - 3, 0)
                        dt = (tr[k1][0] - tr[k0][0]) or 1.0
                        best = (dd, (tr[k1][1] - tr[k0][1]) / dt, (tr[k1][2] - tr[k0][2]) / dt)
                if best is None or math.hypot(best[1], best[2]) < 0.15:
                    continue
                valid.append(int(x['v_valid']))
                pos.append(best[0])
                if x['v_valid'] == '1':
                    e = math.hypot(float(x['vx']) - best[1], float(x['vy']) - best[2])
                    err.append(e)
                    rel.append(e / math.hypot(best[1], best[2]))
        if not valid:
            print(f'== {vdir}: no disk on a moving cylinder')
            continue
        err.sort()
        print(f'== {vdir}: {len(valid)} disks on a moving cylinder, v_valid {sum(valid) / len(valid):.0%}, '
              f'velocity error median {statistics.median(err) if err else float("nan"):.2f} m/s '
              f'(p90 {err[int(0.9 * len(err))] if err else float("nan"):.2f}, relative median '
              f'{statistics.median(rel) if rel else float("nan"):.0%}), disk to centre median '
              f'{statistics.median(pos):.2f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
