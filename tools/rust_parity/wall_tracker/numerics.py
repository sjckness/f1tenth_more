#!/usr/bin/env python3
"""The numerics the Rust wall tracker reproduces, against numpy / scipy / CPython, bit for bit.

Usage (after `source install/setup.bash`; F1TENTH_WALL_TRACKER_CORE_LIB may name
a fresh cargo build):
    python3 tools/rust_parity/wall_tracker/numerics.py [--n 200000] [--json OUT]

Each primitive the port reimplements is called through the library's debug
exports on random inputs built to hit its branches, and compared with the
Python it replaces:

  solve2 / inv2   np.linalg.solve / inv on 2x2 (dgesv; numpy on linus links the
                  system LAPACK 3.12): SPD
                  covariance sums like the tracker's, general matrices,
                  pivoting cases, tiny pivots, exact zeros
  lsa             scipy.optimize.linear_sum_assignment: square, wide and tall
                  matrices of small integers (ties everywhere), with the
                  tracker's "big" filler, and random floats
  hypot           math.hypot (CPython's own algorithm, not libm's)
  pow2            Python float x ** 2 (libm pow, not x * x); an overflow is
                  OverflowError in CPython and inf in the port (expected)
  wrap_pi         corridor_perception.geometry.wrap_pi
  sum             builtin sum() (CPython 3.12 Neumaier)
  arange          np.arange(start, stop, step) on the tracker's 5 cm grid
  searchsorted    np.searchsorted on sorted and unsorted arrays
  svd_vt          np.linalg.svd(M x 2)[2] (dgesdd: dgeqr2 with the Blue dnrm2,
                  dbdsqr on the 2 x 2 bidiagonal), the robust line fit

A mismatch is a bit difference (NaN == NaN counts as equal; -0.0 != +0.0).
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from corridor_perception.geometry import wrap_pi
from f1tenth_perception import wall_tracker_native as wtn


def same(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if a.shape != b.shape:
        return False
    both_nan = np.isnan(a) & np.isnan(b)
    return bool(np.all(both_nan | (a.view(np.int64) == b.view(np.int64))))


def check(name, cases, fn_rust, fn_py, report, expected=None):
    """Compare on every case; `expected(want, got)` marks a documented difference."""
    bad, first, known = 0, None, 0
    for c in cases:
        try:
            want = ('ok', fn_py(*c))
        except Exception as exc:   # noqa: BLE001
            want = ('err', type(exc).__name__)
        try:
            got = ('ok', fn_rust(*c))
        except Exception as exc:   # noqa: BLE001
            got = ('err', type(exc).__name__)
        if want[0] != got[0]:
            ok = False
        elif want[0] == 'err':
            ok = want[1] == got[1]
        elif isinstance(want[1], tuple):
            ok = all(same(x, y) for x, y in zip(want[1], got[1]))
        else:
            ok = same(want[1], got[1])
        if not ok and expected is not None and expected(want, got):
            known += 1
            continue
        if not ok:
            bad += 1
            if first is None:
                first = {'case': repr(c)[:400], 'py': repr(want)[:400], 'rust': repr(got)[:400]}
    report[name] = {'cases': len(cases), 'mismatches': bad, 'expected_differences': known,
                    'first': first}
    print(f'{name:14s} {len(cases):8d} cases  {bad} mismatches'
          + (f'  ({known} expected)' if known else '') + (f'  first {first}' if first else ''))


def spd_cases(rng, n):
    out = []
    for _ in range(n):
        a = rng.normal(size=(2, 2)) * 10.0 ** rng.uniform(-4, 1)
        s = a @ a.T + np.diag(rng.uniform(1e-8, 1e-2, 2))
        out.append((s, rng.normal(size=2)))
    return out


def general_cases(rng, n):
    out = []
    for k in range(n):
        a = rng.normal(size=(2, 2))
        if k % 7 == 0:
            a[0, 0] = 0.0
        if k % 11 == 0:
            a[1, 0] = a[0, 0]          # an exact pivot tie
        if k % 13 == 0:
            a[1, :] = a[0, :] * 2.0    # singular
        if k % 17 == 0:
            a[0, 0] = 1e-310           # below DBL_MIN: the divide branch
        b = rng.normal(size=2)
        if k % 5 == 0:
            b[0] = 0.0
        out.append((a, b))
    return out


def lsa_cases(rng, n):
    out = []
    for k in range(n):
        nr, nc = int(rng.integers(1, 7)), int(rng.integers(1, 7))
        if k % 3 == 0:
            c = rng.integers(0, 4, size=(nr, nc)).astype(float)
        elif k % 3 == 1:
            c = rng.uniform(0, 9.21, size=(nr, nc))
            gated = rng.uniform(size=(nr, nc)) < 0.6
            if (~gated).any():
                big = c[~gated].max() * 10.0 + 1e6
                c = np.where(gated, big, c)
        else:
            c = rng.normal(size=(nr, nc))
        out.append((c,))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=200000)
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    if not wtn.available():
        sys.exit(f'library not loadable: {wtn.load_error()}')
    print(f'library {wtn.library_path()} ({wtn.build_info()})')
    rng = np.random.default_rng(20261023)
    n = cfg.n
    report = {'library': wtn.library_path(), 'build': wtn.build_info(), 'n': n}

    cases = spd_cases(rng, n // 2) + general_cases(rng, n // 2)
    check('solve2', cases, lambda a, b: wtn.debug_solve2(a, b),
          lambda a, b: np.linalg.solve(a, b), report)
    check('inv2', [(a,) for a, _b in cases], lambda a: wtn.debug_inv2(a),
          lambda a: np.linalg.inv(a), report)

    check('lsa', lsa_cases(rng, max(2000, n // 10)),
          lambda c: tuple(wtn.debug_lsa(c)), lambda c: tuple(linear_sum_assignment(c)), report)

    xs = np.concatenate([rng.normal(size=n) * 10.0 ** rng.uniform(-8, 3, n),
                         [0.0, -0.0, 1e-310, -1e-310, 5e-324, 1e300, math.inf, -math.inf,
                          math.nan]])
    ys = np.concatenate([rng.normal(size=n) * 10.0 ** rng.uniform(-8, 3, n),
                         [0.0, 1.0, 1e-310, 2e-310, 0.0, 1e300, 1.0, math.nan, 1.0]])
    check('hypot', list(zip(xs.tolist(), ys.tolist())), wtn.debug_hypot, math.hypot, report)
    # Expected: CPython raises OverflowError for an overflowing float `**`; the
    # port returns inf (wall_tracker_backend.py, DIFFERENCES FROM THE PYTHON).
    check('pow2', [(x,) for x in xs.tolist()], wtn.debug_pow2, lambda x: x ** 2, report,
          expected=lambda want, got: want == ('err', 'OverflowError')
          and got[0] == 'ok' and math.isinf(got[1]))
    angles = np.concatenate([rng.uniform(-20, 20, n), [math.pi, -math.pi, 0.0, -0.0,
                                                       3 * math.pi, math.inf, math.nan]])
    check('wrap_pi', [(a,) for a in angles.tolist()], wtn.debug_wrap_pi, wrap_pi, report)

    sums = [(rng.normal(size=int(rng.integers(1, 9))) * 10.0 ** rng.uniform(-3, 3),)
            for _ in range(n // 4)]
    sums += [([1e100, 1.0, -1e100],), ([-0.0],), ([0.5, 0.25],)]
    check('sum', sums, lambda v: wtn.debug_sum(v), lambda v: sum(v.tolist() if hasattr(v, 'tolist')
                                                                 else v), report)

    ar = []
    for _ in range(n // 10):
        lo = rng.uniform(-10, 10)
        ar.append((lo + 0.025, lo + rng.uniform(-0.1, 8.0), 0.05))
    check('arange', ar, wtn.debug_arange, np.arange, report)

    ss = []
    for k in range(n // 20):
        arr = np.sort(rng.uniform(-3, 3, int(rng.integers(2, 60))))
        if k % 4 == 0:
            arr = rng.uniform(-3, 3, arr.size)      # unsorted: the path is the answer
        keys = rng.uniform(-3.5, 3.5, int(rng.integers(1, 30)))
        if k % 9 == 0:
            keys[0] = math.nan
        ss.append((arr, keys))
    check('searchsorted', ss, lambda a, k: wtn.debug_searchsorted(a, k),
          lambda a, k: np.searchsorted(a, k), report)

    svd = []
    for k in range(n // 10):
        m = int(rng.integers(3, 401))
        if k % 4 == 0:
            p = rng.normal(size=(m, 2))                       # blobs
        else:                                                 # noisy wall lines, centred
            ang = rng.uniform(-math.pi, math.pi)
            t = rng.uniform(-2, 2, m)
            p = np.column_stack([t * math.cos(ang), t * math.sin(ang)])
            p += rng.normal(scale=10.0 ** rng.uniform(-4, -1), size=(m, 2))
            p += rng.uniform(-5, 5, 2)
        if k % 50 == 0:
            p[:, 1] = p[0, 1]                                 # exactly flat: e1 = 0
        svd.append((p - p.mean(axis=0),))
    check('svd_vt', svd, lambda x: wtn.debug_svd_vt(x),
          lambda x: np.linalg.svd(x, full_matrices=False)[2], report)

    total = sum(v['mismatches'] for v in report.values() if isinstance(v, dict) and 'mismatches' in v)
    report['total_mismatches'] = total
    print(f'TOTAL mismatches: {total}')
    if cfg.json:
        cfg.json.parent.mkdir(parents=True, exist_ok=True)
        cfg.json.write_text(json.dumps(report, indent=1))
    return 0 if total == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
