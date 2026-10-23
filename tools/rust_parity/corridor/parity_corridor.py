#!/usr/bin/env python3
r"""Corridor parity, Python against Rust (main-jazzy-rust R1).

Python: f1tenth_params.corridor_geometry and mpc_solver's queries. Rust:
f1tenth_mpc_core through mpc_native.

    python3 tools/rust_parity/corridor/parity_corridor.py \\
        --sweep 10000 --replay <capture dir>/corpus.pkl --json out.json

Parts (each optional except the rebuilds):
  rebuilds  the 24 archived go_to_object rebuilds
            (src/f1tenth_params/test/data/object_rebuilds.json), each as the
            planner's shapes -- the straight corridor on the target line, the
            ramp arc from the car (unclamped, r_min, r_min + length_ref), the
            pose arc to the goal (unclamped, r_min, length_ref) -- x both wall
            constructions x n in (2, 30, 120, 500); plus the four corridor
            queries at the car, the goal, the lead-in origin and 12 points
            around each corridor;
  sweep     N seeded edge cases: n = 2..5, L ~ 0 (0, -0, subnormal, 1e-300,
            1e-12), dpsi +-pi / +-2pi / beyond, ramp u_end == u_start and
            inverted, tiny / zero chords, clamps at their limits, huge
            coordinates, non-finite inputs (the exception must match), and
            the queries on each corridor built;
  replay    a corpus.pkl written by corridor_capture_plugin.py (every geometry
            call of a test run, a sample of the queries), replayed through both;
  timing    per-call time of the planner's shapes, Python vs Rust (ctypes and
            the dict included).

The gate (R1): every array value and float within 1e-12, flags / indices /
keys / types / exceptions identical. The target is bit identity; both counts
are reported.
"""

import argparse
import json
import math
import os
from pathlib import Path
import pickle
import platform
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import corridor_compare as cc  # noqa: E402, I100

REPO = Path(__file__).resolve().parents[3]
REBUILDS = REPO / 'src' / 'f1tenth_params' / 'test' / 'data' / 'object_rebuilds.json'

#: The shipped steering calibration (test_corridor_geometry.py).
RMIN_LEFT, RMIN_RIGHT = 0.923, 0.955
W0, W1, L_BASE = 0.4333, 0.7667, 3.0


def _fns():
    from f1tenth_params import corridor_geometry as cg
    from mpc_controller import mpc_native
    if not mpc_native.available():
        raise SystemExit(f'Rust library not loadable: {mpc_native.load_error()}')
    return cg, mpc_native


def _geometry(tally, name, args, kwargs, label, queries=None, qtally=None, points=()):
    cg, nat = _fns()
    py, py_exc, rs, rs_exc = cc.run_pair(getattr(cg, name), getattr(nat, name), args, kwargs)
    if py_exc is not None or rs_exc is not None:
        if cc.exc_key(py_exc) == cc.exc_key(rs_exc):
            tally.exceptions_matched += 1
        else:
            tally.exception_mismatch.append((label, cc.exc_key(py_exc), cc.exc_key(rs_exc)))
        return None
    cc.add_to(tally, label, cc.compare_dicts(py, rs))
    if qtally is not None:
        for p in points:
            _query(qtally, np.asarray(p, dtype=float), py, label)
    return py


def _query(qtally, p, corridor, label):
    py, py_exc, rs, rs_exc = cc.run_pair(cc.py_queries, cc.rs_queries, (p, corridor), {})
    if py_exc is not None or rs_exc is not None:
        if (py_exc is None) == (rs_exc is None) and type(py_exc) is type(rs_exc):
            qtally.exceptions_matched += 1
        else:
            qtally.exception_mismatch.append((label, repr(p), cc.exc_key(py_exc),
                                              cc.exc_key(rs_exc)))
        return
    cc.add_to(qtally, f'{label} p={p.tolist()}', cc.compare_dicts(py, rs))


def _points_around(corridor, rng, k=12):
    xc, yc = corridor['xc'], corridor['yc']
    pts = []
    for _ in range(k):
        i = int(rng.integers(0, len(xc)))
        pts.append((xc[i] + rng.normal(0, 0.5), yc[i] + rng.normal(0, 0.5)))
    pts.append((xc[0], yc[0]))            # exactly on a sample
    pts.append((xc[-1] + 5.0, yc[-1]))    # beyond the end
    return pts


# ---------------------------------------------------------------------------
# rebuilds
# ---------------------------------------------------------------------------

def rebuild_cases():
    recs = json.loads(REBUILDS.read_text())['rebuilds']
    out = []
    for rec in recs:
        car, goal, origin = rec['car'], rec['goal'], rec['lead_in_origin']
        dpsi = math.atan2(math.sin(rec['psi_c'] - rec['yaw']), math.cos(rec['psi_c'] - rec['yaw']))
        r_min = RMIN_LEFT if dpsi >= 0.0 else RMIN_RIGHT
        length = float(np.hypot(goal[0] - car[0], goal[1] - car[1]))
        line = float(np.hypot(goal[0] - origin[0], goal[1] - origin[1]))
        tag = f'{rec["run"][:9]}#{rec["id"]}'
        for walls in ('bezier', 'trapezoid'):
            for n in (2, 30, 120, 500):
                base = {'w0': W0, 'w1': W1, 'walls': walls}
                ramp = dict(base, u_start=0.0, u_end=0.40)
                out.append((f'{tag} straight {walls} n{n}', 'corridor_curves',
                            (origin[0], origin[1], rec['psi_c'], rec['psi_c'], line, n),
                            dict(ramp, dpsi=0.0), (car, goal, origin)))
                arc = (car[0], car[1], rec['yaw'], rec['psi_c'], max(length, 1e-3), n)
                out.append((f'{tag} arc {walls} n{n}', 'corridor_curves', arc,
                            dict(ramp, dpsi=dpsi), (car, goal, origin)))
                out.append((f'{tag} arc+rmin {walls} n{n}', 'corridor_curves', arc,
                            dict(ramp, dpsi=dpsi, r_min=r_min), (car, goal, origin)))
                out.append((f'{tag} arc+rmin+rate {walls} n{n}', 'corridor_curves', arc,
                            dict(ramp, dpsi=dpsi, r_min=r_min, length_ref=L_BASE),
                            (car, goal, origin)))
                pose = (car[0], car[1], rec['yaw'], goal[0], goal[1], rec['psi_c'], n)
                out.append((f'{tag} pose {walls} n{n}', 'corridor_curves_to_pose', pose,
                            dict(base), (car, goal, origin)))
                out.append((f'{tag} pose+rmin {walls} n{n}', 'corridor_curves_to_pose', pose,
                            dict(base, r_min=r_min), (car, goal, origin)))
                out.append((f'{tag} pose+rate {walls} n{n}', 'corridor_curves_to_pose', pose,
                            dict(base, length_ref=L_BASE), (car, goal, origin)))
    return out


def run_rebuilds(seed):
    rng = np.random.default_rng(seed)
    tally, qtally = cc.Tally(), cc.Tally()
    flags = {'feasible': 0, 'ramp_clamped': 0}
    cases = rebuild_cases()
    for label, name, args, kwargs, anchors in cases:
        py = _geometry(tally, name, args, kwargs, label)
        if py is None:
            continue
        flags['feasible'] += int(py['feasible'])
        flags['ramp_clamped'] += int(py['ramp_clamped'])
        for p in list(anchors) + _points_around(py, rng):
            _query(qtally, np.asarray(p, dtype=float), py, label)
    return {'geometry': tally.as_dict(), 'queries': qtally.as_dict(),
            'feasible_true': flags['feasible'], 'ramp_clamped_true': flags['ramp_clamped'],
            'rebuilds': 24, 'cases': len(cases)}


# ---------------------------------------------------------------------------
# the edge-case sweep
# ---------------------------------------------------------------------------

_TINY = (0.0, -0.0, 5e-324, 1e-300, 1e-15, 1e-12, 1e-9, 1e-6, 1e-3)
_ANGLES = (0.0, -0.0, math.pi, -math.pi, 2 * math.pi, -2 * math.pi,
           math.pi - 1e-15, -math.pi + 1e-15, 3 * math.pi, -2.5 * math.pi, 1e3, -7.0)
_NONFINITE = (math.inf, -math.inf, math.nan)


def _pick(rng, seq):
    return seq[int(rng.integers(0, len(seq)))]


def _angle(rng):
    r = rng.random()
    if r < 0.4:
        return _pick(rng, _ANGLES)
    if r < 0.8:
        return float(rng.uniform(-math.pi, math.pi))
    return float(rng.uniform(-4 * math.pi, 4 * math.pi))


def _length(rng):
    r = rng.random()
    if r < 0.35:
        return _pick(rng, _TINY)
    if r < 0.85:
        return float(rng.uniform(0.0, 3.5))
    if r < 0.95:
        return float(rng.uniform(3.5, 50.0))
    return float(-rng.uniform(0.0, 1.0))       # negative: accepted by the Python


def _n(rng):
    r = rng.random()
    if r < 0.3:
        return int(rng.integers(2, 6))
    if r < 0.85:
        return 120
    if r < 0.95:
        return int(rng.integers(6, 400))
    return _pick(rng, (1000, 4096, 1, 0, -3))   # 1, 0, -3: the n < 2 ValueError


def _coord(rng):
    r = rng.random()
    if r < 0.8:
        return float(rng.normal(0, 3))
    if r < 0.95:
        return float(rng.normal(0, 1e6))
    return _pick(rng, (1e15, -1e15, 0.0, -0.0))


def _maybe_nonfinite(rng, value, p=0.02):
    return _pick(rng, _NONFINITE) if rng.random() < p else value


def sweep_case(rng):
    """One random edge case: (name, args, kwargs, family)."""
    walls = 'trapezoid' if rng.random() < 0.3 else 'bezier'
    if rng.random() < 0.01:
        walls = 'spline'                          # the walls ValueError
    w0 = float(_pick(rng, (W0, 0.0, 1e-9, 0.65, 2.0)))
    w1 = float(_pick(rng, (W1, w0, 0.0, 0.65 * 2.3 / 1.3, 5.0)))
    r_min = _pick(rng, (None, None, RMIN_LEFT, RMIN_RIGHT, 1e-9, 100.0, -1.0, 0.0))
    length_ref = _pick(rng, (None, None, L_BASE, 0.0, -1.0, 1e-9, 1e9))
    if rng.random() < 0.55:
        family = 'ramp'
        psi_start, psi_end = _angle(rng), _angle(rng)
        dpsi = None if rng.random() < 0.5 else _angle(rng)
        r = rng.random()
        if r < 0.2:
            u_start = u_end = float(_pick(rng, (0.0, 0.4, 1.0, 0.5)))
            family = 'ramp u_end==u_start'
        elif r < 0.3:
            u_start, u_end = float(rng.uniform(0.3, 1.0)), float(rng.uniform(0.0, 0.3))
            family = 'ramp inverted'
        elif r < 0.4:
            u_start, u_end = float(rng.uniform(-0.5, 0.0)), float(rng.uniform(1.0, 1.5))
            family = 'ramp outside [0,1]'
        else:
            u_start = float(_pick(rng, (0.0, 0.0, 0.1, 0.3)))
            u_end = float(_pick(rng, (0.40, 0.40, 0.6, 1.0, u_start + 1e-9)))
        length = _length(rng)
        if length in _TINY or length < 1e-3:
            family += ' L~0'
        if dpsi is not None and abs(abs(dpsi) - math.pi) < 1e-12:
            family += ' dpsi=+-pi'
        args = [_coord(rng), _coord(rng), psi_start, psi_end, length, _n(rng)]
        kwargs = {'dpsi': dpsi, 'u_start': u_start, 'u_end': u_end, 'w0': w0, 'w1': w1,
                  'r_min': r_min, 'length_ref': length_ref, 'walls': walls}
        if rng.random() < 0.05:
            i = int(rng.integers(0, 5))
            args[i] = _maybe_nonfinite(rng, args[i], 1.0)
            family += ' non-finite'
        return 'corridor_curves', tuple(args), kwargs, family
    family = 'pose'
    x0, y0 = _coord(rng), _coord(rng)
    r = rng.random()
    if r < 0.25:
        d = _pick(rng, (0.0, 1e-300, 1e-15, 1e-9, 1e-6, 1e-3))
        a = float(rng.uniform(-math.pi, math.pi))
        x1, y1 = x0 + d * math.cos(a), y0 + d * math.sin(a)
        family = 'pose tiny chord'
    else:
        x1, y1 = x0 + float(rng.normal(0, 2)), y0 + float(rng.normal(0, 2))
    psi_start, psi_end = _angle(rng), _angle(rng)
    if rng.random() < 0.15:
        psi_end = math.atan2(y0 - y1, x0 - x1)       # arriving back along the chord
        family += ' backwards'
    handle_a = float(_pick(rng, (0.40, 0.40, 0.0, 1e-9, 2.0)))
    handle_b = float(_pick(rng, (0.40, 0.40, 0.0, 0.35, 5.0)))
    args = [x0, y0, psi_start, x1, y1, psi_end, _n(rng)]
    kwargs = {'handle_a': handle_a, 'handle_b': handle_b, 'w0': w0, 'w1': w1,
              'r_min': r_min, 'length_ref': length_ref, 'walls': walls}
    if rng.random() < 0.05:
        i = int(rng.integers(0, 6))
        args[i] = _maybe_nonfinite(rng, args[i], 1.0)
        family += ' non-finite'
    return 'corridor_curves_to_pose', tuple(args), kwargs, family


def run_sweep(count, seed):
    rng = np.random.default_rng(seed)
    tally, qtally = cc.Tally(), cc.Tally()
    families = {}
    for i in range(count):
        name, args, kwargs, family = sweep_case(rng)
        fam = families.setdefault(family, {'cases': 0, 'built': 0, 'raised_both': 0,
                                           'failures': 0})
        label = f'sweep#{i} {family}: {name}{args!r}{kwargs!r}'
        failures_before = len(tally.failures) + len(tally.exception_mismatch)
        raised_before = tally.exceptions_matched
        py = _geometry(tally, name, args, kwargs, label)
        fam['cases'] += 1
        fam['built'] += int(py is not None)
        fam['raised_both'] += tally.exceptions_matched - raised_before
        fam['failures'] += len(tally.failures) + len(tally.exception_mismatch) - failures_before
        if py is None:
            continue
        pts = _points_around(py, rng, k=3) if np.all(np.isfinite(py['xc'])) else [(0.0, 0.0)]
        if rng.random() < 0.05:
            pts.append((math.nan, 0.0))
        for p in pts:
            _query(qtally, np.asarray(p, dtype=float), py, label)
    return {'count': count, 'seed': seed, 'geometry': tally.as_dict(),
            'queries': qtally.as_dict(), 'families': dict(sorted(families.items()))}


# ---------------------------------------------------------------------------
# replay of a captured corpus
# ---------------------------------------------------------------------------

def run_replay(path):
    with open(path, 'rb') as f:
        corpus = pickle.load(f)
    tally, qtally = cc.Tally(), cc.Tally()
    sites = {}
    calls = 0
    for name, args, kwargs, site, count in corpus['geometry']:
        calls += count
        sites[site] = sites.get(site, 0) + 1
        _geometry(tally, name, args, kwargs, f'{name} @ {site}')
    for kind, p, corridor in corpus['query']:
        _query(qtally, p, corridor, kind)
    return {'corpus': str(path), 'geometry_distinct': len(corpus['geometry']),
            'geometry_calls': calls, 'sites': len(sites), 'geometry': tally.as_dict(),
            'queries_sampled': len(corpus['query']), 'queries': qtally.as_dict()}


# ---------------------------------------------------------------------------
# timing
# ---------------------------------------------------------------------------

def run_timing(repeat):
    cg, nat = _fns()
    from mpc_controller import mpc_solver
    out = {}
    cases = [c for c in rebuild_cases() if c[0].endswith('bezier n120')]
    for shape in ('straight', 'arc+rmin', 'pose+rmin'):
        sel = [c for c in cases if f' {shape} ' in c[0]]
        res = {}
        for impl, mod in (('py', cg), ('rust', nat)):
            times = []
            for _label, name, args, kwargs, _a in sel:
                fn = getattr(mod, name)
                for _ in range(repeat):
                    t0 = time.perf_counter_ns()
                    fn(*args, **kwargs)
                    times.append(time.perf_counter_ns() - t0)
            res[impl] = {'median_us': float(np.median(times)) / 1e3,
                         'p95_us': float(np.percentile(times, 95)) / 1e3, 'calls': len(times)}
        res['speedup_median'] = res['py']['median_us'] / res['rust']['median_us']
        out[shape] = res
    corridor = cg.corridor_curves(0.0, 0.0, 0.0, 0.6, 3.0, 120)
    p = np.array([1.2, 0.3])
    q = {}
    for impl, fn in (('py', lambda: mpc_solver.corridor_lateral_coordinates(p, corridor)),
                     ('rust', lambda: nat.corridor_frame(p, corridor))):
        times = []
        for _ in range(repeat * 20):
            t0 = time.perf_counter_ns()
            fn()
            times.append(time.perf_counter_ns() - t0)
        q[impl] = {'median_us': float(np.median(times)) / 1e3, 'calls': len(times)}
    out['query (lateral coordinates / frame)'] = q
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--sweep', type=int, default=0, help='edge cases (R1: 10000)')
    ap.add_argument('--seed', type=int, default=20261023)
    ap.add_argument('--replay', action='append', default=[], help='corpus.pkl (repeatable)')
    ap.add_argument('--timing', type=int, default=0, help='repeats per timed case')
    ap.add_argument('--json', help='write the results here')
    args = ap.parse_args(argv)
    import numpy
    from mpc_controller import mpc_native
    _fns()
    res = {'host': platform.node(), 'machine': platform.machine(),
           'python': platform.python_version(), 'numpy': numpy.__version__,
           'library': mpc_native.library_path(), 'library_version': mpc_native.version(),
           'library_build': mpc_native.build_info(), 'tolerance': cc.TOL,
           'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    t0 = time.time()
    res['rebuilds'] = run_rebuilds(args.seed)
    if args.sweep:
        res['sweep'] = run_sweep(args.sweep, args.seed)
    res['replay'] = [run_replay(p) for p in args.replay]
    if args.timing:
        res['timing'] = run_timing(args.timing)
    res['seconds'] = time.time() - t0
    gate = []
    for part in ([res['rebuilds']] + ([res['sweep']] if 'sweep' in res else [])
                 + res['replay']):
        for key in ('geometry', 'queries'):
            t = part[key]
            gate.append(t['failures'] == 0 and t['exception_mismatch'] == 0)
    res['gate_pass'] = all(gate)
    text = json.dumps(res, indent=1, default=str)
    if args.json:
        Path(args.json).write_text(text + '\n')
    for name in ('rebuilds', 'sweep'):
        if name in res:
            g, q = res[name]['geometry'], res[name]['queries']
            print(f'{name}: geometry {g["cases"]} cases, {g["identical"]} bit-identical, '
                  f'{g["failures"]} failures, {g["exceptions_matched"]} matched exceptions, '
                  f'{g["exception_mismatch"]} exception mismatches, max |d| {g["max_abs_diff"]}; '
                  f'queries {q["cases"]} / {q["identical"]} identical / {q["failures"]} failures')
    for r in res['replay']:
        g, q = r['geometry'], r['queries']
        print(f'replay {r["corpus"]}: {g["cases"]} geometry ({g["identical"]} identical, '
              f'{g["failures"]} failures, {g["exceptions_matched"]} exceptions matched), '
              f'{q["cases"]} queries ({q["failures"]} failures)')
    if 'timing' in res:
        for k, v in res['timing'].items():
            print('timing', k, json.dumps(v))
    print('GATE', 'PASS' if res['gate_pass'] else 'FAIL', f'({res["seconds"]:.1f} s)')
    return 0 if res['gate_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
