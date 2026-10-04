#!/usr/bin/env python3
"""Summarise isolation batches: per batch, runs with an isolated node, which
nodes and by which signal, CPU busy % over each run, discovery errors.

  isolation_summary.py OUT_JSON BATCH_DIR...
"""
import glob
import json
import os
import re
import sys


def cpu_busy(run):
    def read(p):
        v = [int(x) for x in open(p).read().split()[1:]]
        idle = v[3] + v[4]
        return sum(v), idle
    try:
        t0, i0 = read(os.path.join(run, 'cpu_t0.txt'))
        t1, i1 = read(os.path.join(run, 'cpu_t1.txt'))
        return round(100.0 * (1 - (i1 - i0) / max(t1 - t0, 1)), 1)
    except (OSError, ValueError):
        return None


def main():
    out = {}
    for b in sys.argv[2:]:
        runs = sorted(glob.glob(os.path.join(b, 'run_*')))
        rows = []
        for r in runs:
            try:
                iso = json.load(open(os.path.join(r, 'isolation.json')))
            except (OSError, ValueError):
                rows.append({'run': os.path.basename(r), 'error': 'no isolation.json'})
                continue
            de = open(os.path.join(r, 'discovery_errors.txt')).read() if os.path.exists(
                os.path.join(r, 'discovery_errors.txt')) else ''
            m = re.search(r'participant: (\d+)', de)
            slam = ''.join(open(f, errors='replace').read() for f in glob.glob(
                os.path.join(r, 'supervisor', '*slam.launch.py.log')))
            rows.append({
                'run': os.path.basename(r),
                'isolated': iso['isolated'],
                # slam.launch.py never got slam_toolbox to ACTIVE (configure or
                # activate lost under discovery): it stays unconfigured or inactive and every
                # /slam/* consumer starves.
                'slam_lifecycle_hang': bool(slam) and 'Activating' not in slam,
                'cpu_busy_pct': cpu_busy(r),
                'load1': float(open(os.path.join(r, 'loadavg.txt')).read().split()[0])
                if os.path.exists(os.path.join(r, 'loadavg.txt')) else None,
                'matching_unexisting': int(m.group(1)) if m else None,
                'nodes_settled': len(json.load(open(os.path.join(r, 'nodes_startup.json')))['final']),
            })
        hit = [x for x in rows if x.get('isolated')]
        hang = [x for x in hit if x.get('slam_lifecycle_hang')]
        other = [x for x in hit if not x.get('slam_lifecycle_hang')]
        busy = [x['cpu_busy_pct'] for x in rows if x.get('cpu_busy_pct') is not None]
        out[os.path.basename(b.rstrip('/'))] = {
            'runs': len(rows), 'runs_with_isolation': len(hit),
            'runs_slam_lifecycle_hang': len(hang),
            'runs_other_isolation': [x['run'] for x in other],
            'cpu_busy_pct_mean': round(sum(busy) / len(busy), 1) if busy else None,
            'cpu_busy_pct_with_isolation': [x['cpu_busy_pct'] for x in hit],
            'rows': rows}
    json.dump(out, open(sys.argv[1], 'w'), indent=2)
    for k, v in out.items():
        print(f"{k}: {v['runs_with_isolation']}/{v['runs']} runs with isolation "
              f"({v['runs_slam_lifecycle_hang']} slam lifecycle hang; other "
              f"{v['runs_other_isolation']}); cpu busy mean {v['cpu_busy_pct_mean']}% "
              f"(isolation runs: {v['cpu_busy_pct_with_isolation']})")
        for x in v['rows']:
            if x.get('isolated') or x.get('error'):
                print('   ', x['run'], 'SLAM-HANG' if x.get('slam_lifecycle_hang') else '',
                      x.get('error') or sorted(x['isolated']))


if __name__ == '__main__':
    main()
