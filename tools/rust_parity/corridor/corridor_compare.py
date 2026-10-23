"""Shared pieces of the corridor parity harness (tools/rust_parity/corridor).

Python reference: f1tenth_params.corridor_geometry (corridor_curves,
corridor_curves_to_pose) and mpc_controller.mpc_solver's corridor queries.
Rust: f1tenth_mpc_core through mpc_controller.mpc_native (the installed
library, or F1TENTH_MPC_CORE_LIB).

compare_dicts() is the gate's comparator: every array element and every float
within TOL (1e-12) with NaN == NaN, every flag / index / None / key / type
identical, and -- the target -- the number of values that are bit-identical.
"""

import math
import struct

import numpy as np

#: The R1 gate: arrays and floats within this, absolute.
TOL = 1e-12


def _fbits(x):
    return struct.pack('<d', float(x))


class Tally:
    """Counts over many comparisons, merged into the report's numbers."""

    def __init__(self):
        self.cases = 0
        self.identical = 0          # every value bit-identical, types and keys too
        self.within_tol = 0         # passes the gate
        self.failures = []          # (label, reasons)
        self.values = 0             # float values compared (array elements + scalars)
        self.values_bit_identical = 0
        self.values_nan_bits_differ = 0   # NaN on both sides, sign/payload bits differ
        self.nan_bits_cases = []          # labels of the cases that have any
        self.max_abs_diff = 0.0
        self.exceptions_matched = 0
        self.exception_mismatch = []

    def as_dict(self):
        return {
            'cases': self.cases, 'identical': self.identical,
            'within_tol': self.within_tol, 'failures': len(self.failures),
            'failure_examples': self.failures[:20],
            'values': self.values, 'values_bit_identical': self.values_bit_identical,
            'values_nan_bits_differ': self.values_nan_bits_differ,
            'nan_bits_cases': len(self.nan_bits_cases),
            # every such case had a non-finite input (its label shows it)
            'nan_bits_all_nonfinite_inputs': all(
                any(tok in label for tok in ('non-finite', 'nan', 'inf'))
                for label in self.nan_bits_cases),
            'nan_bits_case_examples': self.nan_bits_cases[:10],
            'max_abs_diff': self.max_abs_diff,
            'exceptions_matched': self.exceptions_matched,
            'exception_mismatch': len(self.exception_mismatch),
            'exception_mismatch_examples': self.exception_mismatch[:20],
        }


def _cmp_float(a, b, state):
    state['values'] += 1
    if _fbits(a) == _fbits(b):
        state['bit'] += 1
        return True
    if math.isnan(a) and math.isnan(b):
        state['bit_nan_payload'] += 1
        return True
    if math.isinf(a) or math.isinf(b):
        state['reasons'].append(f'{state["path"]}: {a!r} != {b!r}')
        return False
    d = abs(a - b)
    state['max'] = max(state['max'], d)
    if d <= TOL:
        return True
    state['reasons'].append(f'{state["path"]}: {a!r} != {b!r} (|d| {d:.3g})')
    return False


def _walk(a, b, state, path):
    state['path'] = path
    if type(a) is not type(b):
        state['reasons'].append(f'{path}: type {type(a).__name__} != {type(b).__name__}')
        state['structural'] = True
        return
    if isinstance(a, dict):
        if list(a) != list(b):
            state['reasons'].append(f'{path}: keys {list(a)} != {list(b)}')
            state['structural'] = True
            return
        for key in a:
            _walk(a[key], b[key], state, f'{path}[{key!r}]')
    elif isinstance(a, np.ndarray):
        if a.dtype != b.dtype or a.shape != b.shape:
            state['reasons'].append(f'{path}: {a.dtype}{a.shape} != {b.dtype}{b.shape}')
            state['structural'] = True
            return
        fa = np.asarray(a, dtype=np.float64).ravel()
        fb = np.asarray(b, dtype=np.float64).ravel()
        same = fa.view(np.uint64) == fb.view(np.uint64)
        state['values'] += fa.size
        state['bit'] += int(same.sum())
        if not same.all():
            ia, ib = fa[~same], fb[~same]
            both_nan = np.isnan(ia) & np.isnan(ib)
            state['bit_nan_payload'] += int(both_nan.sum())
            ia, ib = ia[~both_nan], ib[~both_nan]
            if ia.size:
                with np.errstate(invalid='ignore'):
                    d = np.abs(ia - ib)
                bad_inf = ~np.isfinite(d)
                if bad_inf.any():
                    state['reasons'].append(f'{path}: non-finite difference')
                finite = d[~bad_inf]
                if finite.size:
                    state['max'] = max(state['max'], float(finite.max()))
                    if float(finite.max()) > TOL:
                        state['reasons'].append(
                            f'{path}: {int((finite > TOL).sum())} elements beyond {TOL}, '
                            f'max {float(finite.max()):.3g}')
    elif isinstance(a, (list, tuple)):
        if len(a) != len(b):
            state['reasons'].append(f'{path}: length {len(a)} != {len(b)}')
            state['structural'] = True
            return
        for i, (x, y) in enumerate(zip(a, b)):
            _walk(x, y, state, f'{path}[{i}]')
    elif isinstance(a, float):
        _cmp_float(a, b, state)
    else:
        if a != b:
            state['reasons'].append(f'{path}: {a!r} != {b!r}')
            state['structural'] = True


def compare_dicts(a, b):
    """Compare two corridor results (or any nest of dict/list/ndarray/float/flags).

    Returns a dict: ok (passes the gate), identical (every value
    bit-identical), values, bit, max, reasons.
    """
    state = {'values': 0, 'bit': 0, 'bit_nan_payload': 0, 'max': 0.0, 'reasons': [],
             'structural': False, 'path': ''}
    _walk(a, b, state, 'r')
    if isinstance(a, dict) and isinstance(b, dict) and 'tx' in a and 'ny' in a:
        if (a['ny'] is a['tx']) != (b['ny'] is b['tx']):
            state['reasons'].append("'ny' is 'tx' differs")
            state['structural'] = True
    ok = not state['reasons']
    return {'ok': ok, 'identical': ok and state['bit'] == state['values'],
            'values': state['values'], 'bit': state['bit'], 'max': state['max'],
            'nan_bits': state['bit_nan_payload'], 'reasons': state['reasons'][:5]}


def add_to(tally, label, result):
    tally.cases += 1
    tally.values += result['values']
    tally.values_bit_identical += result['bit']
    tally.values_nan_bits_differ += result['nan_bits']
    if result['nan_bits']:
        tally.nan_bits_cases.append(label[:300])
    tally.max_abs_diff = max(tally.max_abs_diff, result['max'])
    if result['ok']:
        tally.within_tol += 1
        if result['identical']:
            tally.identical += 1
    else:
        tally.failures.append((label, result['reasons']))


def run_pair(py_fn, rs_fn, args, kwargs):
    """Call both; returns (py_result, py_exc, rs_result, rs_exc)."""
    py = rs = None
    py_exc = rs_exc = None
    try:
        py = py_fn(*args, **kwargs)
    except Exception as exc:          # noqa: BLE001 - compared, not handled
        py_exc = exc
    try:
        rs = rs_fn(*args, **kwargs)
    except Exception as exc:          # noqa: BLE001
        rs_exc = exc
    return py, py_exc, rs, rs_exc


def exc_key(exc):
    return None if exc is None else (type(exc).__name__, str(exc))


# -- the corridor queries, Python side and Rust side as comparable tuples ----

def py_queries(p, corridor):
    """Return mpc_solver's four corridor queries on (p, corridor), as one dict."""
    from mpc_controller import mpc_solver
    d2, idx = mpc_solver.nearest_centerline_index(p, corridor)
    d_lat, half_w = mpc_solver.corridor_lateral_coordinates(p, corridor)
    heading = mpc_solver.corridor_heading_at(p, corridor)
    pc, n, hw = mpc_solver._nearest_corridor_frame(p, corridor)
    return {'nearest': (d2, idx), 'lateral': (d_lat, half_w), 'heading': heading,
            'frame': (pc, n, hw)}


def rs_queries(p, corridor):
    """Return the same four from the Rust library's two query entry points."""
    from mpc_controller import mpc_native
    d2, idx = mpc_native.nearest_centerline_index(p, corridor)
    f = mpc_native.corridor_frame(p, corridor)
    return {'nearest': (d2, idx), 'lateral': (f['d_lat'], f['half_w']),
            'heading': f['heading'], 'frame': (f['pc'], f['n'], f['half_w'])}
