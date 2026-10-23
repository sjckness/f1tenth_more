#!/usr/bin/env python3
"""R2 parity: the Rust RTI solver core against mpc_solver._solve_rti, in layers.

    tools/rust_parity/mpc/parity_solver.py [--fixture F] [--sweep 3000] [--seed 20261023]
        [--capture-dir DIR ...] [--json OUT] [--lib PATH]

Every tick is solved twice in this process, by the Python (impl='py') and by
the Rust core (impl='rust'), through mpc_solver.solve_mpc_step. osqp-python is
spied on (osqp.OSQP.setup / warm_start / solve wrapped, nothing changed) so the
Python's QP and OSQP result are available as OSQP C received / returned them.

  L1  QP assembly: mpc_native.build_qp against the QP the Python handed OSQP C
      (prob._derivative_cache after setup: P upper triangle, q, A, l, u clamped)
      and the warm start: identical sparsity, every value compared bit for bit
      (target 0 ulp; the gate is 1e-14). Fixture ticks also against the
      recorded qp_sha256.
  L2  OSQP alone: mpc_native.osqp_solve_csc (the Rust core's OSQP, settings and
      fresh workspace) on the PYTHON's QP and warm start: x bit for bit,
      iterations, status, polish status, rho updates, obj_val / residuals; the
      effective settings field by field against prob.settings.
  L3  the whole tick: solve_mpc_step py vs rust, every output (u0, zopt, x_pred,
      v_cmd_pred, cost, slacks, status, message, iterations, success); fixture
      ticks also against the recorded outputs. Gate: >= 99 % of ticks within
      1e-9 on u0 / zopt / x_pred with identical status, every outlier <= 1e-3.
      Inputs: the 723-tick fixture, a seeded sweep of perturbed fixture ticks
      (states, weights, obstacles, walls, warm starts, steps, turn branch,
      target line, extreme values; exceptions must match in type), and live
      captures (F1TENTH_MPC_CAPTURE files, solver_backend.py).

Exit 0 when every layer passes its gate.
"""

import argparse
import copy
import glob
import gzip
import hashlib
import json
import math
import os
import platform
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rti_fixture  # noqa: E402

DEFAULT_FIXTURE = os.path.join(HERE, 'data', 'rti_ticks.jsonl.gz')
L1_TOL = 1e-14
L3_TOL = 1e-9
L3_OUTLIER_MAX = 1e-3
L3_FRACTION = 0.99
TRAJ_KEYS = ('u0', 'zopt', 'x_pred', 'v_cmd_pred')
INFO_FIELDS = ('iter', 'status_val', 'status', 'status_polish', 'rho_updates',
               'rho_estimate', 'obj_val', 'prim_res', 'dual_res')


def log(msg):
    print(msg, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ the spy
class OsqpSpy:
    """Wraps osqp.OSQP.setup / warm_start / solve to record what OSQP C saw."""

    def __init__(self):
        import osqp
        self.osqp = osqp
        self.calls = []
        self.current = None
        cls = osqp.OSQP
        self._orig = (cls.setup, cls.warm_start, cls.solve)
        spy = self

        def setup(prob, *a, **k):
            out = spy._orig[0](prob, *a, **k)
            c = prob._derivative_cache
            spy.current = {
                'P': c['P'].copy(), 'A': c['A'].copy(),
                'q': np.array(c['q'], dtype=float), 'l': np.array(c['l'], dtype=float),
                'u': np.array(c['u'], dtype=float), 'warm_x': None,
                'settings': {s: getattr(prob.settings, s) for s in dir(prob.settings)
                             if not s.startswith('_')},
            }
            return out

        def warm_start(prob, x=None, y=None):
            if spy.current is not None:
                spy.current['warm_x'] = None if x is None else np.array(x, dtype=float)
                spy.current['warm_y'] = y
            return spy._orig[1](prob, x=x, y=y)

        def solve(prob, *a, **k):
            res = spy._orig[2](prob, *a, **k)
            info = res.info
            if spy.current is not None:
                spy.current['result'] = {
                    'x': None if res.x is None else np.array(res.x, dtype=float),
                    'iter': int(info.iter), 'status_val': int(info.status_val),
                    'status': str(info.status), 'status_polish': int(info.status_polish),
                    'rho_updates': int(info.rho_updates),
                    'rho_estimate': float(info.rho_estimate), 'obj_val': float(info.obj_val),
                    'prim_res': float(info.prim_res), 'dual_res': float(info.dual_res)}
                spy.calls.append(spy.current)
                spy.current = None
            return res

        cls.setup, cls.warm_start, cls.solve = setup, warm_start, solve

    def take(self):
        calls, self.calls = self.calls, []
        return calls


# ------------------------------------------------------------------ helpers
def bits_equal(a, b):
    a = np.ascontiguousarray(np.asarray(a, dtype=np.float64))
    b = np.ascontiguousarray(np.asarray(b, dtype=np.float64))
    if a.shape != b.shape:
        return False
    # NaN == NaN; otherwise the bit patterns
    same = a.view(np.uint64) == b.view(np.uint64)
    same |= np.isnan(a) & np.isnan(b)
    return bool(np.all(same))


def ulp_stats(a, b):
    """Return (n_differ, max_abs, max_ulp) of two float64 arrays of one shape."""
    a = np.asarray(a, dtype=np.float64).ravel()
    b = np.asarray(b, dtype=np.float64).ravel()
    both_nan = np.isnan(a) & np.isnan(b)
    differ = (a.view(np.int64) != b.view(np.int64)) & ~both_nan
    if not np.any(differ):
        return 0, 0.0, 0
    with np.errstate(invalid='ignore'):
        d = np.abs(a[differ] - b[differ])
    ia = a[differ].view(np.int64).astype(object)
    ib = b[differ].view(np.int64).astype(object)
    ulp = max(abs(int(x) - int(y)) for x, y in zip(ia, ib))
    finite = d[np.isfinite(d)]
    return int(np.sum(differ)), float(finite.max()) if finite.size else math.inf, ulp


def qp_sha(P_indptr, P_indices, P_data, q, A_indptr, A_indices, A_data, l, u):  # noqa: E741
    h = hashlib.sha256()
    for arr, dt in ((P_indptr, '<i8'), (P_indices, '<i8'), (P_data, '<f8'), (q, '<f8'),
                    (A_indptr, '<i8'), (A_indices, '<i8'), (A_data, '<f8'), (l, '<f8'),
                    (u, '<f8')):
        h.update(np.ascontiguousarray(np.asarray(arr, dtype=dt)).tobytes())
    return h.hexdigest()


def outputs_of(u0, info):
    return rti_fixture.outputs_of(u0, info)


def traj_diff(a, b):
    """Largest absolute difference over u0, zopt, x_pred, v_cmd_pred (inf on shape)."""
    worst = 0.0
    for key in TRAJ_KEYS:
        va, vb = a.get(key), b.get(key)
        if va is None and vb is None:
            continue
        if va is None or vb is None:
            return math.inf
        fa = np.asarray(va, dtype=float).ravel()
        fb = np.asarray(vb, dtype=float).ravel()
        if fa.shape != fb.shape:
            return math.inf
        with np.errstate(invalid='ignore'):
            d = np.abs(fa - fb)
        nan_mismatch = np.isnan(fa) != np.isnan(fb)
        if np.any(nan_mismatch):
            return math.inf
        d = d[np.isfinite(d)]
        if d.size:
            worst = max(worst, float(d.max()))
    return worst


def solve_both(mpc_solver, kw):
    """Return {'py': (u0, info) | exc, 'rust': ...} for one tick, plus the spy's calls."""
    out = {}
    for impl in ('py', 'rust'):
        try:
            out[impl] = mpc_solver.solve_mpc_step(**copy.deepcopy(kw), impl=impl)
        except Exception as exc:  # noqa: BLE001 - compared by type
            out[impl] = exc
    return out


# ------------------------------------------------------------------ layers
class Layers:
    """Accumulates L1 / L2 / L3 results over ticks."""

    def __init__(self, mpc_native, mpc_solver, spy):
        self.native, self.solver, self.spy = mpc_native, mpc_solver, spy
        self.l1 = {'ticks': 0, 'pattern_identical': 0, 'bit_identical': 0,
                   'sha_checked': 0, 'sha_equal': 0, 'max_abs': 0.0, 'max_ulp': 0,
                   'z_guess_identical': 0, 'fail': []}
        self.l2 = {'ticks': 0, 'x_bit_identical': 0, 'info_identical': 0,
                   'settings_identical': 0, 'fail': []}
        self.l3 = {}

    def group(self, name):
        return self.l3.setdefault(name, {
            'ticks': 0, 'bit_identical': 0, 'within_tol': 0, 'status_identical': 0,
            'recorded_bit_identical': 0, 'recorded_checked': 0, 'exceptions_matched': 0,
            'exceptions_mismatched': 0, 'max_traj_diff': 0.0, 'outliers': [],
            'statuses': {}})

    def check_tick(self, tick_id, kw, group, recorded=None, sha=None, qp_layers=True):
        g = self.group(group)
        self.spy.take()
        res = solve_both(self.solver, kw)
        calls = self.spy.take()
        py, rs = res['py'], res['rust']
        if isinstance(py, Exception) or isinstance(rs, Exception):
            same = (type(py) is type(rs)) and str(py) == str(rs)
            g['exceptions_matched' if same else 'exceptions_mismatched'] += 1
            if not same:
                g['outliers'].append({'id': tick_id, 'py': repr(py), 'rust': repr(rs)})
            return
        g['ticks'] += 1
        out_py, out_rs = outputs_of(*py), outputs_of(*rs)
        cmp = rti_fixture.compare_outputs(out_py, out_rs)
        status_same = out_py['status'] == out_rs['status'] and \
            out_py['success'] == out_rs['success']
        diff = traj_diff(out_py, out_rs)
        g['statuses'][str(out_py['status'])] = g['statuses'].get(str(out_py['status']), 0) + 1
        g['bit_identical'] += int(cmp['identical'])
        g['status_identical'] += int(status_same)
        ok = status_same and diff <= L3_TOL
        g['within_tol'] += int(ok)
        g['max_traj_diff'] = max(g['max_traj_diff'], diff)
        if not cmp['identical']:
            g['outliers'].append({'id': tick_id, 'traj_max_abs': diff,
                                  'status': [out_py['status'], out_rs['status']],
                                  'fields': {k: v for k, v in cmp['fields'].items() if v}})
        if recorded is not None:
            g['recorded_checked'] += 1
            g['recorded_bit_identical'] += int(
                rti_fixture.compare_outputs(recorded, out_rs)['identical'])
        if qp_layers:
            self._qp_layers(tick_id, kw, calls, sha)

    def _qp_layers(self, tick_id, kw, calls, sha):
        if len(calls) != 1:
            self.l1['fail'].append({'id': tick_id, 'why': f'{len(calls)} OSQP calls spied'})
            return
        c = calls[0]
        w = self.solver.scale_stage_weights(kw['weights'], kw['horizon'])
        args = {k: v for k, v in kw.items() if k not in ('solver', 'stage_spec', 'weights')}
        qp = self.native.build_qp(weights=w, **copy.deepcopy(args))
        P, A = c['P'], c['A']
        l1 = self.l1
        l1['ticks'] += 1
        pattern = (np.array_equal(qp['P_indptr'], P.indptr)
                   and np.array_equal(qp['P_indices'], P.indices)
                   and np.array_equal(qp['A_indptr'], A.indptr)
                   and np.array_equal(qp['A_indices'], A.indices)
                   and qp['n'] == P.shape[0] and qp['m'] == A.shape[0])
        l1['pattern_identical'] += int(pattern)
        if pattern:
            pairs = ((qp['P_data'], P.data), (qp['q'], c['q']), (qp['A_data'], A.data),
                     (qp['l'], c['l']), (qp['u'], c['u']))
            stats = [ulp_stats(a, b) for a, b in pairs]
            n_diff = sum(s[0] for s in stats)
            l1['bit_identical'] += int(n_diff == 0)
            l1['max_abs'] = max([l1['max_abs']] + [s[1] for s in stats])
            l1['max_ulp'] = max([l1['max_ulp']] + [s[2] for s in stats])
            if n_diff:
                l1['fail'].append({'id': tick_id, 'differing_values': n_diff,
                                   'max_abs': max(s[1] for s in stats)})
        else:
            l1['fail'].append({'id': tick_id, 'why': 'sparsity pattern differs'})
        zg_ok = c['warm_x'] is not None and bits_equal(qp['z_guess'], c['warm_x'])
        l1['z_guess_identical'] += int(zg_ok)
        if sha is not None:
            l1['sha_checked'] += 1
            l1['sha_equal'] += int(sha == qp_sha(
                qp['P_indptr'], qp['P_indices'], qp['P_data'], qp['q'], qp['A_indptr'],
                qp['A_indices'], qp['A_data'], qp['l'], qp['u']))
        # L2: the Rust core's OSQP on the Python's QP and warm start
        l2 = self.l2
        l2['ticks'] += 1
        r = self.native.osqp_solve_csc(P, c['q'], A, c['l'], c['u'], warm_x=c['warm_x'])
        p = c['result']
        x_ok = (p['x'] is None and r['x'] is None) or (
            p['x'] is not None and r['x'] is not None and bits_equal(p['x'], r['x']))
        info_ok = all((r[k] == p[k]) or (isinstance(p[k], float) and math.isnan(p[k])
                                         and math.isnan(r[k])) for k in INFO_FIELDS)
        settings = {k: (int(v) if not isinstance(v, (int, float, bool)) else v)
                    for k, v in c['settings'].items()}
        settings_ok = all(r['settings'].get(k) == v for k, v in settings.items())
        l2['x_bit_identical'] += int(x_ok)
        l2['info_identical'] += int(info_ok)
        l2['settings_identical'] += int(settings_ok)
        if not (x_ok and info_ok and settings_ok):
            l2['fail'].append({'id': tick_id, 'x': x_ok, 'info': info_ok,
                               'settings': settings_ok,
                               'py': {k: p[k] for k in INFO_FIELDS},
                               'rust': {k: r[k] for k in INFO_FIELDS}})


# ------------------------------------------------------------------ inputs
def fixture_ticks(path):
    header, records = rti_fixture.load(path)
    return header, records


def capture_records(dirs):
    out = []
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(d, 'rti_capture_*.jsonl.gz'))):
            try:
                with gzip.open(path, 'rt', encoding='utf-8') as fh:
                    for line in fh:
                        if line.strip():
                            out.append(json.loads(line, object_hook=rti_fixture._hook))
            except (EOFError, OSError, json.JSONDecodeError) as exc:
                log(f'  {path}: stopped reading at {exc!r} (truncated capture)')
    return out


def perturb(rng, kw):
    """One random variant of a fixture tick (the sweep)."""
    kw = copy.deepcopy(kw)
    x0 = np.asarray(kw['x0'], dtype=float).copy()
    kind = rng.randrange(14)
    x0[0] += rng.uniform(-0.3, 0.3)
    x0[1] += rng.uniform(-0.3, 0.3)
    x0[2] += rng.uniform(-0.6, 0.6)
    x0[3] = max(-0.2, x0[3] + rng.uniform(-0.3, 0.3))
    kw['x0'] = x0
    kw['last_u'] = np.asarray(kw['last_u'], dtype=float) + np.array(
        [rng.uniform(-0.1, 0.1), rng.uniform(-0.5, 0.5)])
    w = dict(kw['weights'])
    for k in list(w):
        if rng.random() < 0.5:
            w[k] = float(w[k]) * rng.uniform(0.3, 3.0)
    kw['weights'] = w
    cor = kw['corridor']
    if kind == 0:
        cor['obstacles_world'] = [(float(x0[0]) + rng.uniform(0.3, 3.0),
                                   float(x0[1]) + rng.uniform(-1.0, 1.0),
                                   rng.uniform(0.05, 0.5)) for _ in range(rng.randrange(1, 9))]
    elif kind == 1:
        kw['boundaries'] = [(math.cos(a), math.sin(a), rng.uniform(-0.5, 3.0))
                            for a in (rng.uniform(-math.pi, math.pi)
                                      for _ in range(rng.randrange(1, 6)))]
        kw['boundary_hard'] = rng.random() < 0.4
        kw['boundary_max_sources'] = rng.choice([1, 2, 4, 8])
    elif kind == 2 and kw.get('warm_start_z') is not None:
        z = np.asarray(kw['warm_start_z'], dtype=float).copy()
        kw['warm_start_z'] = z + np.array([rng.uniform(-0.05, 0.05) for _ in z])
    elif kind == 3:
        kw['ts'] = rng.choice([0.05, 0.1, 0.15, 0.2])
        kw['control_period'] = rng.choice([None, 0.05, 0.1])
    elif kind == 4:
        cor['psiRefTurn'] = rng.uniform(-4.0, 4.0)
        cor['psiStart'] = float(x0[2]) + rng.uniform(-0.5, 0.5)
    elif kind == 5:
        cor['targetLine'] = {'p': [float(x0[0]) + rng.uniform(-1, 3),
                                   float(x0[1]) + rng.uniform(-1, 1)],
                             'psi': rng.uniform(-math.pi, math.pi)}
        w['w_line'] = rng.uniform(0.1, 5.0)
    elif kind == 6:
        kw['vdes'] = rng.uniform(-0.5, 1.5)
    elif kind == 7:
        cor['car_radius'] = rng.uniform(0.0, 0.5)
        cor['avoidance_margin'] = rng.uniform(0.0, 0.4)
    elif kind == 8:
        kw['horizon'] = rng.choice([1, 2, 5, 10, 30])
        kw['warm_start_z'] = None
        if kw.get('speed_model'):
            sm = kw['speed_model']
            n = kw['horizon']
            for key in ('c_lo', 'c_hi', 'v_hi'):
                if sm.get(key) is not None:
                    arr = np.asarray(sm[key], dtype=float)
                    sm[key] = np.resize(arr, n)
    elif kind == 9:
        # extreme magnitudes: far away, fast, huge weights
        x0[0] += rng.choice([1e3, -1e3, 1e6])
        kw['x0'] = x0
        w['w_obs'] = rng.choice([1e-9, 1e6])
    elif kind == 10:
        # failures the Python raises on: the Rust must raise the same
        choice = rng.randrange(5)
        if choice == 0:
            x0[2] = math.inf
            kw['x0'] = x0
        elif choice == 1:
            kw['params'] = {'L': 0.0, 'lr': 0.17}
        elif choice == 2:
            cor.pop('obstacles_world', None)
        elif choice == 3:
            kw['x0'] = x0[:3]
        else:
            for key in ('xc', 'yc', 'nx', 'ny', 'halfWidth'):
                cor[key] = np.array([])
    elif kind == 11:
        x0[3] = math.nan
        kw['x0'] = x0
    elif kind == 12 and kw.get('speed_model'):
        sm = kw['speed_model']
        sm['alpha'] = rng.uniform(0.05, 0.9)
        sm['d'] = rng.uniform(-0.2, 0.2)
        sm['delay'] = rng.choice([0, 1])
        if sm.get('v_hi') is not None and rng.random() < 0.3:
            sm['v_hi'] = None
    elif kind == 13:
        kw['boundary_slack_weight'] = rng.choice([1.0, 1e4, 1e8])
        kw['boundary_slack_max'] = rng.choice([0.0, 0.1, 2.0])
        kw['boundaries'] = list(kw['boundaries'] or []) + [(0.0, 1.0, float(x0[1]) - 0.2)]
    return kw


# ------------------------------------------------------------------ main
def gate(layers, args):
    l1, l2 = layers.l1, layers.l2
    checks = {
        'L1 sparsity identical on every tick': l1['pattern_identical'] == l1['ticks'] > 0,
        f'L1 values within {L1_TOL:g} (target 0 ulp)': l1['max_abs'] <= L1_TOL
        and l1['pattern_identical'] == l1['ticks'],
        'L1 warm start identical': l1['z_guess_identical'] == l1['ticks'],
        'L1 fixture qp_sha256 equal': l1['sha_equal'] == l1['sha_checked'],
        'L2 x bit-identical': l2['x_bit_identical'] == l2['ticks'] > 0,
        'L2 iterations/status/info identical': l2['info_identical'] == l2['ticks'],
        'L2 settings identical': l2['settings_identical'] == l2['ticks'],
    }
    for name, g in layers.l3.items():
        if g['ticks']:
            frac = g['within_tol'] / g['ticks']
            checks[f'L3 {name}: >= {L3_FRACTION:.0%} within {L3_TOL:g}, status identical'] = \
                frac >= L3_FRACTION
            checks[f'L3 {name}: every outlier <= {L3_OUTLIER_MAX:g}'] = \
                g['max_traj_diff'] <= L3_OUTLIER_MAX
        checks[f'L3 {name}: exceptions identical'] = g['exceptions_mismatched'] == 0
        if g['recorded_checked']:
            checks[f'L3 {name}: Rust equals the recorded outputs'] = \
                g['recorded_bit_identical'] == g['recorded_checked']
    return checks


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--fixture', default=DEFAULT_FIXTURE)
    ap.add_argument('--sweep', type=int, default=3000)
    ap.add_argument('--seed', type=int, default=20261023)
    ap.add_argument('--capture-dir', action='append', default=[])
    ap.add_argument('--capture-limit', type=int, default=0, help='0: all')
    ap.add_argument('--json')
    ap.add_argument('--lib', help='F1TENTH_MPC_CORE_LIB for this run')
    args = ap.parse_args(argv)
    if args.lib:
        os.environ['F1TENTH_MPC_CORE_LIB'] = args.lib
    os.environ.pop('F1TENTH_MPC_SOLVER_IMPL', None)
    os.environ.pop('F1TENTH_MPC_CAPTURE', None)

    from mpc_controller import mpc_native, mpc_solver  # noqa: E402
    if not mpc_native.available():
        raise SystemExit(f'Rust library not loadable: {mpc_native.load_error()}')
    spy = OsqpSpy()
    layers = Layers(mpc_native, mpc_solver, spy)
    t_start = time.time()

    header, records = fixture_ticks(args.fixture)
    log(f'fixture: {len(records)} ticks')
    for rec in records:
        layers.check_tick(rec['id'], rti_fixture.solve_kwargs(rec), 'fixture/' + rec['group'],
                          recorded=rec['outputs'], sha=rec['osqp']['qp_sha256'])

    rng = random.Random(args.seed)
    log(f'sweep: {args.sweep} perturbed ticks (seed {args.seed})')
    for i in range(args.sweep):
        src = records[rng.randrange(len(records))]
        kw = perturb(rng, rti_fixture.solve_kwargs(src))
        layers.check_tick(f'sweep/{i:05d}/{src["id"]}', kw, 'sweep')

    caps = capture_records(args.capture_dir)
    if args.capture_limit:
        caps = caps[:args.capture_limit]
    log(f'live captures: {len(caps)} ticks from {args.capture_dir}')
    for rec in caps:
        recorded = rec['outputs'] if rec.get('impl') == 'py' else None
        layers.check_tick(rec['id'], rti_fixture.solve_kwargs(rec), 'capture',
                          recorded=recorded)

    checks = gate(layers, args)
    for g in layers.l3.values():
        g['outliers'] = sorted(g['outliers'], key=lambda o: -float(
            o.get('traj_max_abs', math.inf) if isinstance(o.get('traj_max_abs'), float)
            else math.inf))[:25]
    layers.l1['fail'] = layers.l1['fail'][:25]
    layers.l2['fail'] = layers.l2['fail'][:25]
    out = {
        'machine': {'platform': platform.platform(), 'python': platform.python_version(),
                    'numpy': np.__version__, 'osqp_python': spy.osqp.__version__,
                    'osqp_c_rust': mpc_native.osqp_version(),
                    'library': mpc_native.library_path(), 'build': mpc_native.build_info()},
        'fixture': {'path': os.path.relpath(args.fixture), 'ticks': len(records),
                    'git_head': header.get('git_head')},
        'sweep': {'n': args.sweep, 'seed': args.seed},
        'captures': {'dirs': args.capture_dir, 'ticks': len(caps)},
        'L1': layers.l1, 'L2': layers.l2, 'L3': layers.l3,
        'gate': checks, 'pass': all(checks.values()),
        'seconds': time.time() - t_start,
    }
    for name, ok in checks.items():
        print(f'{"PASS" if ok else "FAIL"}  {name}')
    l1, l2 = layers.l1, layers.l2
    print(f'L1: {l1["ticks"]} QPs, pattern {l1["pattern_identical"]}, bit-identical '
          f'{l1["bit_identical"]}, sha {l1["sha_equal"]}/{l1["sha_checked"]}, max abs '
          f'{l1["max_abs"]:.3g}, max ulp {l1["max_ulp"]}')
    print(f'L2: {l2["ticks"]} solves, x bit-identical {l2["x_bit_identical"]}, info '
          f'{l2["info_identical"]}, settings {l2["settings_identical"]}')
    for name, g in layers.l3.items():
        print(f'L3 {name}: {g["ticks"]} ticks, bit-identical {g["bit_identical"]}, within '
              f'{L3_TOL:g} {g["within_tol"]}, status {g["status_identical"]}, max '
              f'{g["max_traj_diff"]:.3g}, exceptions {g["exceptions_matched"]} matched / '
              f'{g["exceptions_mismatched"]} not, recorded {g["recorded_bit_identical"]}/'
              f'{g["recorded_checked"]}, statuses {g["statuses"]}')
    print('GATE PASS' if out['pass'] else 'GATE FAIL')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1, default=str)
            fh.write('\n')
    return 0 if out['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
