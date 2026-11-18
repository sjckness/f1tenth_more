#!/usr/bin/env python3
"""
MPC timing summary: solve time and control-loop period, per source.

Runs on: anywhere with the side's ROS Python (rosbag2_py for bags; numpy).
Plan P9.4: "MPC timing on a car bag replay: container p95 period <= 1.05 x
native". Run replay_localization.sh mpc (and mpc_capture) on both sides with
the same input bag, summarise each side here, then
`parity_gates.py mpc-timing NATIVE.json CONTAINER.json`.

Sources, each labelled with what it measures:
  capture:LABEL=capture.npz  solve_mpc_step wall time inside the node
                             (mpc_capture_node.py, perf_counter around the
                             call): solve_ms
  wallrx:LABEL=stamps.json   receive wall time of every /mpc/solver_status
                             message (stack_probe.py stamps, run beside the
                             replay): period_ms -- the control loop's real
                             period, whatever clock the node runs on
  recorded:LABEL=BAG         a bag with /mpc/solver_status: solve_dt_sec and
                             the header-stamp periods. On wall time (a live car
                             mission bag) both are real; in a use_sim_time
                             replay solve_dt is 0.0 (the node clock does not
                             advance while control_loop holds the executor)
                             and the periods are sim time: use wallrx/capture
                             there.

Every series: n, mean, p50, p95, p99, max, std; periods also the count over
110 ms and over 150 ms (the loop's budget is 100 ms at 10 Hz).

Usage: mpc_timing.py --out FILE.json SPEC [SPEC ...]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def pct(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return {'n': 0}
    return {'n': int(v.size), 'mean': float(v.mean()), 'p50': float(np.percentile(v, 50)),
            'p95': float(np.percentile(v, 95)), 'p99': float(np.percentile(v, 99)),
            'max': float(v.max()), 'std': float(v.std())}


def period_stats(times_s):
    per = np.diff(np.asarray(times_s, dtype=float)) * 1e3
    return {'period_ms': pct(per), 'periods_over_110ms': int(np.sum(per > 110)),
            'periods_over_150ms': int(np.sum(per > 150))}


def from_capture(path):
    import mpc_frozen_io
    ticks, meta, _ = mpc_frozen_io.load_ticks(path, ('wall_dt',))
    return {'kind': 'capture', 'solve_ms': pct([t['wall_dt'] * 1e3 for t in ticks if t]),
            'meta': meta}


def from_wallrx(path):
    with open(path) as f:
        res = json.load(f)
    out = {'kind': 'wallrx', 'topic': res.get('topic')}
    out.update(period_stats([r[0] for r in res.get('rx', [])]))
    return out


def from_recorded(path):
    from bag_read import header_stamp_sec, read_topic
    st = sorted((header_stamp_sec(m), float(m.solve_dt_sec))
                for _, m in read_topic(path, '/mpc/solver_status'))
    out = {'kind': 'recorded', 'solve_ms': pct([s * 1e3 for _, s in st])}
    out.update(period_stats([t for t, _ in st]))
    return out


KINDS = {'capture': from_capture, 'wallrx': from_wallrx, 'recorded': from_recorded}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('Sources'):])
    ap.add_argument('--out', required=True)
    ap.add_argument('specs', nargs='+', metavar='KIND:LABEL=PATH')
    args = ap.parse_args(argv)
    res = {}
    for spec in args.specs:
        kind, _, rest = spec.partition(':')
        label, _, path = rest.partition('=')
        if kind not in KINDS or not label or not path:
            raise SystemExit('bad spec %r (KIND:LABEL=PATH, KIND in %s)' % (spec, sorted(KINDS)))
        res[label] = KINDS[kind](path)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2)
    for label, r in res.items():
        for key in ('solve_ms', 'period_ms'):
            s = r.get(key)
            if not s or not s.get('n'):
                continue
            extra = ''
            if key == 'period_ms':
                extra = '  >110ms:%d >150ms:%d' % (r['periods_over_110ms'],
                                                   r['periods_over_150ms'])
            print('%-30s %-9s n=%4d mean %6.2f p50 %6.2f p95 %6.2f p99 %6.2f max %6.2f%s' % (
                label, key, s['n'], s['mean'], s['p50'], s['p95'], s['p99'], s['max'], extra))
    print('wrote', args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
