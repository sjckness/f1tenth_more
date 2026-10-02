#!/usr/bin/env python3
"""Phase 3 timing summary -- INDICATIVE only; the deployment target is the Orin.

Sources, each labelled with what it actually measures:
  capture:LABEL=capture.npz   solve_mpc_step wall time inside the live node
                              (mpc_capture_node.py, perf_counter around the
                              call -- the same span MPC_corr.py's own solve_dt
                              brackets, which on the live Humble stack ran on
                              the system clock).
  frozen:LABEL=out.npz        solve_mpc_step wall time in run_mpc_frozen.py
                              (no ROS, one tick after another).
  log:LABEL=node.log          control-loop period in WALL time, from the
                              system-clock timestamp rclpy puts on the per-tick
                              "SOLVE/in" log line (the replay's own tick stamps
                              are sim time and say nothing about jitter).
  recorded:LABEL=BAG          the live Humble/Orin run: solve_dt_sec and header
                              stamp periods from /mpc/solver_status (system
                              clock, live, loaded).

Why not /mpc/solver_status solve_dt_sec of a replay: under use_sim_time the
node clock only advances in the /clock callback, which cannot run while
control_loop holds the single-threaded executor, so it reads 0.0 for every
tick (see replay_localization.sh, CLOCK_HZ).

Usage: mpc_timing.py --out FILE.json SPEC [SPEC ...]
"""
import argparse
import json
import re

import numpy as np

import mpc_frozen_io

LOG_TS = re.compile(r'\[(\d+\.\d+)\] \[mpc_corr\]: SOLVE/in')


def pct(v):
    v = np.asarray(v, dtype=float)
    return {'n': int(v.size), 'mean': float(v.mean()), 'p50': float(np.percentile(v, 50)),
            'p95': float(np.percentile(v, 95)), 'p99': float(np.percentile(v, 99)),
            'max': float(v.max()), 'std': float(v.std())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('specs', nargs='+')
    args = ap.parse_args()
    res = {}
    for spec in args.specs:
        kind, rest = spec.split(':', 1)
        label, path = rest.split('=', 1)
        if kind in ('capture', 'frozen'):
            ticks, _, _ = mpc_frozen_io.load_ticks(path, ('wall_dt',))
            res[label] = {'kind': kind, 'solve_ms': pct([t['wall_dt'] * 1e3 for t in ticks])}
        elif kind == 'log':
            ts = [float(m.group(1)) for m in LOG_TS.finditer(open(path).read())]
            per = np.diff(ts) * 1e3
            res[label] = {'kind': kind, 'period_ms': pct(per),
                          'periods_over_110ms': int(np.sum(per > 110)),
                          'periods_over_150ms': int(np.sum(per > 150))}
        elif kind == 'recorded':
            import mpc_metrics as mm
            run = mm.load_mpc_run(path)
            st = run['status']
            per = np.diff([s['t'] for s in st]) * 1e3
            res[label] = {'kind': kind, 'solve_ms': pct([s['solve_dt'] * 1e3 for s in st]),
                          'period_ms': pct(per), 'periods_over_110ms': int(np.sum(per > 110)),
                          'periods_over_150ms': int(np.sum(per > 150))}
        else:
            raise SystemExit('unknown spec kind: ' + kind)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2)
    for label, r in res.items():
        for key in ('solve_ms', 'period_ms'):
            if key in r:
                s = r[key]
                extra = ''
                if key == 'period_ms':
                    extra = '  >110ms:%d >150ms:%d' % (r['periods_over_110ms'], r['periods_over_150ms'])
                print('%-34s %-9s n=%4d mean %6.2f p50 %6.2f p95 %6.2f p99 %6.2f max %6.2f std %5.2f%s' % (
                    label, key, s['n'], s['mean'], s['p50'], s['p95'], s['p99'], s['max'], s['std'], extra))
    print('wrote', args.out)


if __name__ == '__main__':
    main()
