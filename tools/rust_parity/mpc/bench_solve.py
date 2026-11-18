#!/usr/bin/env python3
"""Time one MPC RTI tick (solve_mpc_step) per implementation on the R2.0 fixture.

    tools/rust_parity/mpc/bench_solve.py --impl py|rust [--repeats 7] [--out FILE.json]
        [--fixture F] [--cpu N] [--lib PATH]
    tools/rust_parity/mpc/bench_solve.py --compare PY.json RUST.json   # exit 0 = gate

WHAT IS TIMED. mpc_solver.solve_mpc_step(**kwargs, impl=IMPL) as MPC_corr calls
it: weight scaling, the dispatch, the whole tick (QP build, OSQP, rollout, true
cost) and, for rust, the ctypes marshalling both ways. The kwargs are decoded
from the fixture and deep-copied BEFORE the clock starts. Each repeat runs every
fixture tick once, in fixture order, after one untimed warm-up pass; the garbage
collector is off while a repeat runs (collected between repeats), the same for
both implementations.

REPORTED. Over every timed call: mean, p50, p95, p99, max (ms); per tick the
minimum of the repeats (robust against other load), summarised the same way;
the same per group (fixture group + speed model) and for the live configuration
(offset_free, 8 obstacles, 4 live wall rows). Plus the machine (CPU, governor,
load average) and the library.

GATE (design R2): p95 of every call with rust <= p95 with py (--compare), on the
whole fixture and on the live configuration.
"""

import argparse
import copy
import gc
import json
import math
import os
import platform
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rti_fixture  # noqa: E402

DEFAULT_FIXTURE = os.path.join(HERE, 'data', 'rti_ticks.jsonl.gz')
LIVE_CONFIG = 'matrix/offset_free/obs8/walls4'


def pct(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def summary(values_s):
    v = [x * 1e3 for x in values_s]
    return {'n': len(v), 'mean': statistics.fmean(v) if v else math.nan,
            'p50': pct(v, 50), 'p95': pct(v, 95), 'p99': pct(v, 99),
            'max': max(v) if v else math.nan}


def group_of(rec):
    cfg = rec.get('config', {})
    if rec['group'] == 'matrix':
        return f"matrix/{cfg.get('speed_model')}"
    if rec['group'] == 'edge':
        return f"edge/{cfg.get('speed_model')}"
    return rec['group']


def config_of(rec):
    return rec['id'].rsplit('/', 1)[0]


def machine():
    info = {'platform': platform.platform(), 'python': platform.python_version(),
            'numpy': np.__version__, 'loadavg_start': os.getloadavg(),
            'affinity': sorted(os.sched_getaffinity(0))}
    try:
        with open('/proc/cpuinfo', encoding='utf-8') as fh:
            info['cpu'] = next(line.split(':', 1)[1].strip() for line in fh
                               if line.startswith('model name'))
    except (OSError, StopIteration):
        pass
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor',
                  encoding='utf-8') as fh:
            info['governor'] = fh.read().strip()
    except OSError:
        pass
    return info


def run(args):
    if args.lib:
        os.environ['F1TENTH_MPC_CORE_LIB'] = args.lib
    os.environ.pop('F1TENTH_MPC_SOLVER_IMPL', None)
    os.environ.pop('F1TENTH_MPC_CAPTURE', None)
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    from mpc_controller import mpc_native, mpc_solver  # noqa: E402
    if args.impl == 'rust' and not mpc_native.available():
        raise SystemExit(f'Rust library not loadable: {mpc_native.load_error()}')
    header, records = rti_fixture.load(args.fixture)
    base = [rti_fixture.solve_kwargs(r) for r in records]
    solve = mpc_solver.solve_mpc_step
    impl = args.impl

    def one_pass(timed):
        kws = [copy.deepcopy(k) for k in base]
        out = []
        gc.collect()
        gc.disable()
        try:
            for kw in kws:
                t0 = time.perf_counter()
                solve(**kw, impl=impl)
                out.append(time.perf_counter() - t0)
        finally:
            gc.enable()
        return out if timed else None

    info0 = machine()
    one_pass(False)
    samples = [one_pass(True) for _ in range(args.repeats)]
    per_tick = list(zip(*samples))
    all_calls = [s for rep in samples for s in rep]
    mins = [min(t) for t in per_tick]
    groups = {}
    for rec, calls in zip(records, per_tick):
        for key in (group_of(rec), config_of(rec) if rec['group'] == 'matrix' else None):
            if key is None:
                continue
            g = groups.setdefault(key, {'calls': [], 'mins': []})
            g['calls'].extend(calls)
            g['mins'].append(min(calls))
    result = {
        'impl': impl, 'repeats': args.repeats, 'fixture': os.path.relpath(args.fixture),
        'fixture_ticks': len(records), 'fixture_git_head': header.get('git_head'),
        'machine': dict(info0, loadavg_end=os.getloadavg()),
        'library': mpc_native.library_path() if impl == 'rust' else None,
        'build': mpc_native.build_info() if impl == 'rust' else None,
        'all_calls': summary(all_calls), 'per_tick_min': summary(mins),
        'groups': {k: {'all_calls': summary(v['calls']), 'per_tick_min': summary(v['mins'])}
                   for k, v in sorted(groups.items())},
    }
    s = result['all_calls']
    print(f"{impl}: {len(records)} ticks x {args.repeats}: mean {s['mean']:.3f} ms, "
          f"p50 {s['p50']:.3f}, p95 {s['p95']:.3f}, p99 {s['p99']:.3f}, max {s['max']:.3f}")
    live = result['groups'].get(LIVE_CONFIG)
    if live:
        s = live['all_calls']
        print(f"  live config {LIVE_CONFIG}: p50 {s['p50']:.3f} ms, p95 {s['p95']:.3f}, "
              f"max {s['max']:.3f}")
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, indent=1)
            fh.write('\n')
    return 0


def compare(path_py, path_rust):
    with open(path_py, encoding='utf-8') as fh:
        py = json.load(fh)
    with open(path_rust, encoding='utf-8') as fh:
        rs = json.load(fh)
    if (py['impl'], rs['impl']) != ('py', 'rust'):
        raise SystemExit(f'expected a py and a rust run, got {py["impl"]} / {rs["impl"]}')
    rows = [('all ticks', py['all_calls'], rs['all_calls'])]
    for key in sorted(set(py['groups']) & set(rs['groups'])):
        rows.append((key, py['groups'][key]['all_calls'], rs['groups'][key]['all_calls']))
    ok = True
    print(f"{'group':<36} {'py p50':>8} {'py p95':>8} {'rust p50':>9} {'rust p95':>9} "
          f"{'speed-up p95':>13}")
    for name, a, b in rows:
        gated = name in ('all ticks', LIVE_CONFIG)
        good = b['p95'] <= a['p95']
        if gated:
            ok &= good
        mark = ('  GATE ' + ('ok' if good else 'FAIL')) if gated else ''
        print(f"{name:<36} {a['p50']:8.3f} {a['p95']:8.3f} {b['p50']:9.3f} {b['p95']:9.3f} "
              f"{a['p95'] / b['p95']:12.1f}x{mark}")
    print('GATE PASS (p95 rust <= p95 py)' if ok else 'GATE FAIL')
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--impl', choices=('py', 'rust'))
    ap.add_argument('--repeats', type=int, default=7)
    ap.add_argument('--fixture', default=DEFAULT_FIXTURE)
    ap.add_argument('--out')
    ap.add_argument('--cpu', type=int)
    ap.add_argument('--lib')
    ap.add_argument('--compare', nargs=2, metavar=('PY_JSON', 'RUST_JSON'))
    args = ap.parse_args(argv)
    if args.compare:
        return compare(*args.compare)
    if not args.impl:
        ap.error('--impl or --compare is required')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
