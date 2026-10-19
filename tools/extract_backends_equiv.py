#!/usr/bin/env python3
"""H1 equivalence: the C backend against the Python, over every scan of a bag.

Usage (after `source install/setup.bash`):
    python3 tools/extract_backends_equiv.py BAG [--json OUT]

Runs mwt.extract_lines with backend 'python' and backend 'c' over EVERY /scan
of the recording and compares, per scan: the segment count, then segment by
segment in the order each backend returned them -- rho, alpha, both endpoints,
the covariance, the point count, and the dropped-per-gate statistics. Prints
the maximum difference of each.

THE PYTHON IS THE REFERENCE. If anything differs, the C is wrong.

BOTH BACKENDS ARE FED THE SAME NUMBERS. The C takes float32 ranges (its API
does), so the Python is given those same values widened back to float64 --
otherwise the comparison would be measuring float32 rounding of the input,
~1e-7 m on a range, rather than the algorithm. In production the C sees the
float32 of a float64 range and that rounding is the one difference between the
backends; it is four orders of magnitude under the 1e-4 tolerance here.
"""

import argparse
import json
import math
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from corridor_perception.scan import clean_ranges
from f1tenth_perception import multi_wall_tracker as mwt


def scans(bag):
    bag = Path(bag)
    db = sorted(bag.glob('*.db3')) or sorted((bag / 'bag').glob('*.db3'))
    if not db:
        raise SystemExit(f'no .db3 under {bag}')
    con = sqlite3.connect(f'file:{db[0]}?mode=ro', uri=True)
    cls = get_message('sensor_msgs/msg/LaserScan')
    for _ts, data in con.execute(
            'select m.timestamp, m.data from messages m join topics t '
            "on m.topic_id=t.id where t.name='/scan' order by m.timestamp"):
        yield deserialize_message(bytes(data), cls)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--json', default=None)
    ap.add_argument('--laser-x', type=float, default=0.305)
    cfg = ap.parse_args()

    from corridor_perception import extraction_c
    if not extraction_c.available():
        raise SystemExit(f'C backend unavailable: {extraction_c.load_error()}')
    print(f'C library: {extraction_c.library_path()} (version {extraction_c.version()})')

    laser = (cfg.laser_x, 0.0, 0.0)
    worst = {k: 0.0 for k in ('rho', 'alpha', 'p0', 'p1', 'cov', 'length')}
    worst_at = {k: None for k in worst}
    n_scans = n_segs = 0
    count_mismatch = []
    stats_mismatch = []
    ms = {'python': [], 'c': []}

    for idx, msg in enumerate(scans(cfg.bag)):
        # One float32 view of the cleaned ranges, fed to BOTH backends.
        clean = clean_ranges(msg.ranges, msg.range_min, msg.range_max)
        as32 = np.asarray(clean, dtype=np.float32)
        same = clean_ranges(as32.astype(np.float64), -math.inf, math.inf)
        out = {}
        for backend in ('python', 'c'):
            p = mwt.extraction_params(1.5, quality_gates=True)
            p.backend = backend
            st = {}
            t0 = time.perf_counter()
            lines = mwt.extract_lines(same, msg.angle_min, msg.angle_increment,
                                      msg.range_min, msg.range_max, laser,
                                      (), params=p, stats=st)
            ms[backend].append((time.perf_counter() - t0) * 1e3)
            out[backend] = (lines, st)
        n_scans += 1
        py, c = out['python'][0], out['c'][0]
        if len(py) != len(c):
            count_mismatch.append((idx, len(py), len(c)))
            continue
        if out['python'][1] != out['c'][1]:
            stats_mismatch.append((idx, out['python'][1], out['c'][1]))
        for k, (a, b) in enumerate(zip(py, c)):
            n_segs += 1
            d = {'rho': abs(a.rho - b.rho),
                 'alpha': abs(a.alpha - b.alpha),
                 'p0': float(np.max(np.abs(np.asarray(a.p_start) - b.p_start))),
                 'p1': float(np.max(np.abs(np.asarray(a.p_end) - b.p_end))),
                 'cov': float(np.max(np.abs(np.asarray(a.cov) - b.cov))),
                 'length': abs(a.length - b.length)}
            for key, value in d.items():
                if value > worst[key]:
                    worst[key] = value
                    worst_at[key] = (idx, k)

    print(f'\nscans compared: {n_scans}, segments compared: {n_segs}')
    print(f'segment-count mismatches: {len(count_mismatch)}'
          + (f' -> {count_mismatch[:5]}' if count_mismatch else ''))
    print(f'gate-stats mismatches:    {len(stats_mismatch)}'
          + (f' -> {stats_mismatch[:3]}' if stats_mismatch else ''))
    print('\nmax |python - c| per quantity (tolerance 1e-4):')
    for key in ('rho', 'alpha', 'p0', 'p1', 'length', 'cov'):
        flag = 'OK ' if worst[key] <= 1e-4 else 'FAIL'
        print(f'  {flag} {key:7s} {worst[key]:.3e}  at scan/segment {worst_at[key]}')
    ok = (not count_mismatch and not stats_mismatch
          and all(v <= 1e-4 for v in worst.values()))
    print(f'\nEQUIVALENT: {ok}')
    for backend in ('python', 'c'):
        v = np.array(ms[backend])
        print(f'  extract_lines {backend:7s} p50 {np.median(v):7.3f} ms  '
              f'p95 {np.percentile(v, 95):7.3f} ms  max {v.max():7.3f} ms')
    speedup = np.median(ms['python']) / max(np.median(ms['c']), 1e-9)
    print(f'  speedup (p50): {speedup:.1f}x')
    if cfg.json:
        Path(cfg.json).write_text(json.dumps({
            'scans': n_scans, 'segments': n_segs,
            'count_mismatches': count_mismatch, 'stats_mismatches': stats_mismatch[:20],
            'max_abs_diff': {k: worst[k] for k in worst},
            'worst_at': {k: worst_at[k] for k in worst_at},
            'equivalent': bool(ok),
            'ms': {b: {'p50': float(np.median(ms[b])), 'p95': float(np.percentile(ms[b], 95)),
                       'max': float(np.max(ms[b]))} for b in ms},
            'speedup_p50': float(speedup)}, indent=1))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
