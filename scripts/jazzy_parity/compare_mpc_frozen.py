#!/usr/bin/env python3
"""Phase 3: compare two sets of per-tick MPC solve results on the same frozen
inputs -- run_mpc_frozen.py outputs (Thor vs Orin, osqp version A vs B) or a
re-solve vs the live capture (mpc_live_outputs.npz).

Reported:
  - whether the two used the same inputs file and the same solver source
    (sha256s recorded by run_mpc_frozen.py);
  - per field, bit-equal tick counts and max abs difference: u0 (steering
    rad, accel m/s^2), the full control sequence zopt, the predicted
    trajectory x_pred, the true cost, OSQP status / iterations;
  - when both carry the QP (run_mpc_frozen.py outputs): whether the QP
    handed to OSQP is bit-identical (P, q, A, l, u) -- this separates a
    NumPy/SciPy difference in building the QP from an osqp difference in
    solving it;
  - the OSQP-tolerance judgement for every tick whose solutions differ:
    each side's raw primal/dual solution is checked against the FIRST
    side's QP with OSQP's own termination test (unscaled, infinity norms):
        primal  ||A x - proj_[l,u](A x)|| <= eps_abs + eps_rel * max(||A x||, ||proj||)
        dual    ||P x + q + A' y||          <= eps_abs + eps_rel * max(||P x||, ||A' y||, ||q||)
    with eps_abs/eps_rel = the values passed to setup(), else osqp's
    defaults (1e-3, 1e-3). Two solutions that both pass are both valid
    answers to the same QP at the tolerance the controller runs with; their
    difference is solver tolerance, not a behaviour change.
  - every tick where success/status differ, listed with its inputs.

Usage: compare_mpc_frozen.py A.npz B.npz [--inputs mpc_frozen_inputs.npz] [--max-list N]
Plain Python 3 + NumPy + SciPy.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mpc_frozen_io  # noqa: E402

NAMES = ('u0', 'info', 'qp', 'wall_dt', 'sim_ns')
OSQP_DEFAULT_EPS = 1e-3


def bits_equal(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return a.shape == b.shape and a.tobytes() == b.tobytes()


def maxabs(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if a.shape != b.shape:
        return float('inf')
    if a.size == 0:
        return 0.0
    return float(np.nanmax(np.abs(a - b)))


def csc(parts):
    import scipy.sparse as sp
    shape = tuple(int(v) for v in parts['shape'])
    return sp.csc_matrix((parts['data'], parts['indices'], parts['indptr']), shape=shape)


def osqp_residual_check(qp, x, y):
    """(r_prim, eps_prim, r_dual, eps_dual) of (x, y) on qp, OSQP's test."""
    import scipy.sparse as sp
    P = csc(qp['P'])
    A = csc(qp['A'])
    # OSQP reads only the upper triangle of P; rebuild the symmetric matrix.
    Pu = sp.triu(P, format='csc')
    Pf = Pu + sp.triu(Pu, k=1, format='csc').T
    s = qp.get('settings', {})
    eps_abs = float(s.get('eps_abs', OSQP_DEFAULT_EPS))
    eps_rel = float(s.get('eps_rel', OSQP_DEFAULT_EPS))
    Ax = A.dot(x)
    z = np.minimum(np.maximum(Ax, qp['l']), qp['u'])
    r_prim = float(np.max(np.abs(Ax - z))) if Ax.size else 0.0
    eps_prim = eps_abs + eps_rel * max(float(np.max(np.abs(Ax))) if Ax.size else 0.0,
                                       float(np.max(np.abs(z))) if z.size else 0.0)
    Px = Pf.dot(x)
    Aty = A.T.dot(y)
    r_dual = float(np.max(np.abs(Px + qp['q'] + Aty)))
    eps_dual = eps_abs + eps_rel * max(float(np.max(np.abs(Px))), float(np.max(np.abs(Aty))),
                                       float(np.max(np.abs(qp['q']))))
    return r_prim, eps_prim, r_dual, eps_dual


def describe_inputs(tick):
    inp = tick['inputs']
    x0 = np.asarray(inp['x0'], dtype=float)
    return ('x0=[%.4f %.4f %.4f %.4f] last_u=[%.4f %.4f] vdes=%.3f n_obs=%d n_bnd=%d' % (
        x0[0], x0[1], x0[2], x0[3], float(inp['last_u'][0]), float(inp['last_u'][1]),
        float(inp['vdes']), len(inp['obstacles']), len(inp.get('boundaries') or [])))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--inputs')
    ap.add_argument('--max-list', type=int, default=40)
    args = ap.parse_args()

    A, ma, _ = mpc_frozen_io.load_ticks(args.a, NAMES)
    B, mb, _ = mpc_frozen_io.load_ticks(args.b, NAMES)
    la, lb = ma.get('label', 'A'), mb.get('label', 'B')
    inputs = mpc_frozen_io.load_ticks(args.inputs, ('inputs',))[0] if args.inputs else None

    print('=== metadata ===')
    for k in sorted(set(ma) | set(mb)):
        va, vb = ma.get(k, '-'), mb.get(k, '-')
        print('%-24s %s: %s' % (k, la, va) + ('' if va == vb else '\n%-24s %s: %s   <- differs' % ('', lb, vb)))
    for k in ('sha256_mpc_solver.py', 'sha256_vehicle_model.py', 'sha256_object_geometry.py',
              'inputs_sha256'):
        if k in ma and k in mb and ma[k] != mb[k]:
            sys.exit('NOT COMPARABLE: %s differs' % k)
    if len(A) != len(B):
        sys.exit('NOT COMPARABLE: %d vs %d ticks' % (len(A), len(B)))
    n = len(A)
    print('\nticks: %d\n' % n)

    eq = {k: 0 for k in ('u0', 'zopt', 'x_pred', 'cost', 'status', 'success', 'iterations',
                         'boundary_slack')}
    diff = {k: 0.0 for k in ('steer', 'accel', 'zopt', 'x_pred', 'cost_rel')}
    has_qp = all('qp' in t for t in A) and all('qp' in t for t in B)
    qp_eq = {k: 0 for k in ('P', 'A', 'q', 'l', 'u', 'settings')} if has_qp else None
    differing, status_mismatch = [], []
    tol_fail = []
    worst_ratio = 0.0
    for i in range(n):
        a, b = A[i], B[i]
        ia, ib = a['info'], b['info']
        same_u0 = bits_equal(a['u0'], b['u0'])
        eq['u0'] += same_u0
        eq['zopt'] += bits_equal(ia['zopt'], ib['zopt'])
        eq['x_pred'] += bits_equal(ia['x_pred'], ib['x_pred'])
        eq['cost'] += bits_equal(ia['cost'], ib['cost'])
        eq['status'] += ia['status'] == ib['status']
        eq['success'] += ia['success'] == ib['success']
        eq['iterations'] += ia.get('iterations') == ib.get('iterations')
        eq['boundary_slack'] += bits_equal(ia.get('boundary_slack', []), ib.get('boundary_slack', []))
        diff['steer'] = max(diff['steer'], abs(float(a['u0'][0]) - float(b['u0'][0])))
        diff['accel'] = max(diff['accel'], abs(float(a['u0'][1]) - float(b['u0'][1])))
        diff['zopt'] = max(diff['zopt'], maxabs(ia['zopt'], ib['zopt']))
        diff['x_pred'] = max(diff['x_pred'], maxabs(ia['x_pred'], ib['x_pred']))
        diff['cost_rel'] = max(diff['cost_rel'], abs(ia['cost'] - ib['cost']) / max(abs(ia['cost']), 1e-12))
        if ia['status'] != ib['status'] or ia['success'] != ib['success']:
            status_mismatch.append(i)
        if not (same_u0 and bits_equal(ia['zopt'], ib['zopt'])):
            differing.append(i)
        if has_qp:
            qa, qb = a['qp'], b['qp']
            for k in ('P', 'A'):
                qp_eq[k] += all(bits_equal(qa[k][f], qb[k][f]) for f in ('shape', 'data', 'indices', 'indptr'))
            for k in ('q', 'l', 'u'):
                qp_eq[k] += bits_equal(qa[k], qb[k])
            qp_eq['settings'] += qa.get('settings') == qb.get('settings')
            if not (same_u0 and bits_equal(ia['zopt'], ib['zopt'])):
                for side, q in ((la, a['qp']), (lb, b['qp'])):
                    if q.get('x') is None or q.get('y') is None:
                        tol_fail.append((i, side, 'no solution'))
                        continue
                    rp, ep, rd, ed = osqp_residual_check(qa, q['x'], q['y'])
                    worst_ratio = max(worst_ratio, rp / ep, rd / ed)
                    if rp > ep or rd > ed:
                        tol_fail.append((i, side, 'r_prim %.2e/%.2e r_dual %.2e/%.2e' % (rp, ep, rd, ed)))

    print('=== per-field agreement (%s vs %s) ===' % (la, lb))
    for k, v in eq.items():
        print('%-16s %4d/%d bit-equal' % (k, v, n))
    print('max |d steer| = %.3e rad   max |d accel| = %.3e m/s^2' % (diff['steer'], diff['accel']))
    print('max |d zopt|  = %.3e       max |d x_pred| = %.3e     max rel d cost = %.3e' % (
        diff['zopt'], diff['x_pred'], diff['cost_rel']))
    if has_qp:
        print('\n=== QP handed to OSQP ===')
        for k, v in qp_eq.items():
            print('%-10s %4d/%d bit-identical' % (k, v, n))

    print('\nticks whose solution differs: %d' % len(differing))
    print('ticks with a status/success mismatch: %d' % len(status_mismatch))
    for i in status_mismatch[:args.max_list]:
        line = 'tick %d: %s status=%s success=%s | %s status=%s success=%s' % (
            i, la, A[i]['info']['status'], A[i]['info']['success'],
            lb, B[i]['info']['status'], B[i]['info']['success'])
        if inputs:
            line += ' | ' + describe_inputs(inputs[i])
        print('  ' + line)

    if has_qp:
        print('\n=== OSQP-tolerance check on differing ticks (vs %s QP) ===' % la)
        print('worst residual / tolerance ratio: %.3e' % worst_ratio)
        print('solutions failing the termination test: %d' % len(tol_fail))
        for t in tol_fail[:args.max_list]:
            print('  tick %d (%s): %s' % t)

    print('\n=== verdict ===')
    if not differing and not status_mismatch:
        print('IDENTICAL: all %d ticks bit-identical (u0, zopt).' % n)
    elif not status_mismatch and has_qp and not tol_fail:
        print('WITHIN OSQP TOLERANCE: %d ticks differ, statuses agree on all %d, and every '
              'differing solution passes OSQP\'s own termination test on the same QP.' % (len(differing), n))
    else:
        print('DIFFERENT: %d differing ticks, %d status mismatches, %d tolerance failures -- '
              'root-cause before accepting.' % (len(differing), len(status_mismatch), len(tol_fail)))


if __name__ == '__main__':
    main()
