#!/usr/bin/env python3
"""Phase 2 follow-up: compare two run_boundary_frozen.py outputs tick by tick.

Both runs must come from the same boundary_frozen_inputs.npz and the same
costmap_boundary.py source (checked via the sha256s each run records), so
any difference is a numerics difference between the two platforms.

Reports, per output field: how many ticks are bit-identical, the max abs
difference and the max ULP distance. Then every tick where anything
differs, split into DECISION flips (a different nearest cell chosen, or a
direction present on one side only) and float-only differences, each
listed with its frozen inputs.

With --humble, also scores each run against what Humble actually published
(same index-matched method as verify_costmap_boundary_logic.py) and checks
whether the two runs agree on the large-error tail ticks.

Plain Python 3 + NumPy, no ROS.
"""
import argparse
import math
import sys

import numpy as np

DIRECTIONS = ('front', 'left', 'right')
FLOAT_FIELDS = ['yaw', 'front_clearance'] + [
    d + '_' + k for d in DIRECTIONS for k in ('nx', 'ny', 'off', 'car_dx', 'car_dy', 'dist')]
DECISION_FIELDS = ['n_constraints'] + [
    d + '_' + k for d in DIRECTIONS for k in ('present', 'cell_col', 'cell_row')]


def bits_equal(a, b):
    """Elementwise bit-for-bit equality (NaN == NaN when bit-identical)."""
    if a.dtype.kind == 'f':
        return a.view(np.int64) == b.view(np.int64)
    return a == b


def ulp_dist(a, b):
    """ULP distance between float64 arrays (0 where bit-identical)."""
    ia = a.view(np.int64).astype(object)
    ib = b.view(np.int64).astype(object)
    # map sign-magnitude to a monotonic integer line
    ia = np.array([x if x >= 0 else -(x & 0x7FFFFFFFFFFFFFFF) for x in ia], dtype=object)
    ib = np.array([x if x >= 0 else -(x & 0x7FFFFFFFFFFFFFFF) for x in ib], dtype=object)
    return np.array([abs(x - y) for x, y in zip(ia, ib)], dtype=object)


def humble_errors(run, hum):
    """Per tick: (max normal-angle err deg, max offset err m, clearance err m)
    vs the recorded Humble output; NaN where counts differ / no clearance."""
    n = len(run['tick_bag_index'])
    ang = np.full(n, np.nan)
    off = np.full(n, np.nan)
    clr = np.full(n, np.nan)
    all_ang, all_off = [], []
    for i in range(n):
        mine = [(run[d + '_nx'][i], run[d + '_ny'][i], run[d + '_off'][i])
                for d in DIRECTIONS if run[d + '_present'][i]]
        k = int(hum['rec_n'][i])
        if k == len(mine):
            a_s, o_s = [], []
            for (cnx, cny, coff), (rnx, rny, roff) in zip(mine, hum['rec_nx_ny_off'][i][:k]):
                dot = max(-1.0, min(1.0, rnx * cnx + rny * cny))
                a_s.append(math.degrees(math.acos(dot)))
                o_s.append(abs(roff - coff))
            all_ang.extend(a_s)
            all_off.extend(o_s)
            if a_s:
                ang[i], off[i] = max(a_s), max(o_s)
        rc = hum['rec_front_clearance'][i]
        if not np.isnan(rc):
            clr[i] = abs(rc - run['front_clearance'][i])
    return ang, off, clr, np.array(all_ang), np.array(all_off)


def describe_tick(i, inp):
    m = int(inp['tick_map_idx'][i])
    q = inp['tick_pose_quat_xyzw'][i]
    return ('bag_index=%d out_recv_ns=%d map=%d (bag map #%d, %dx%d res=%r origin=(%r, %r)) '
            'pose_bag_index=%d pose=(%r, %r) quat_xyzw=(%r, %r, %r, %r)' % (
                inp['tick_bag_index'][i], inp['tick_out_recv_ns'][i], m,
                inp['map_bag_index'][m], inp['map_width'][m], inp['map_height'][m],
                float(inp['map_resolution'][m]), float(inp['map_origin_x'][m]),
                float(inp['map_origin_y'][m]), inp['tick_pose_bag_index'][i],
                float(inp['tick_pose_x'][i]), float(inp['tick_pose_y'][i]),
                float(q[0]), float(q[1]), float(q[2]), float(q[3])))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('a', help='first run_boundary_frozen.py output (e.g. Thor)')
    ap.add_argument('b', help='second run_boundary_frozen.py output (e.g. Orin)')
    ap.add_argument('--inputs', required=True, help='boundary_frozen_inputs.npz')
    ap.add_argument('--humble', help='boundary_humble_recorded.npz (optional)')
    args = ap.parse_args()

    A = dict(np.load(args.a, allow_pickle=False))
    B = dict(np.load(args.b, allow_pickle=False))
    inp = dict(np.load(args.inputs, allow_pickle=False))
    la, lb = str(A['meta_label']), str(B['meta_label'])

    print('=== run metadata ===')
    for k in sorted(x for x in A if x.startswith('meta_')):
        va, vb = str(A[k]).replace('\n', ' '), str(B.get(k, '')).replace('\n', ' ')
        flag = '' if va == vb else '   <- differs'
        print('%-20s %s: %s%s' % (k[5:], la, va, flag))
        if va != vb:
            print('%-20s %s: %s' % ('', lb, vb))
    problems = []
    if str(A['meta_inputs_sha256']) != str(B['meta_inputs_sha256']):
        problems.append('runs used DIFFERENT input files')
    if str(A['meta_module_sha256']) != str(B['meta_module_sha256']):
        problems.append('runs used DIFFERENT costmap_boundary.py sources')
    if not np.array_equal(A['tick_bag_index'], B['tick_bag_index']):
        problems.append('tick sets differ')
    if problems:
        sys.exit('NOT COMPARABLE: ' + '; '.join(problems))
    n = len(A['tick_bag_index'])
    print('\nticks: %d\n' % n)

    print('=== per-field agreement (%s vs %s) ===' % (la, lb))
    print('%-16s %10s %14s %10s' % ('field', 'bit-equal', 'max |diff|', 'max ULP'))
    tick_diff = np.zeros(n, dtype=bool)
    tick_decision = np.zeros(n, dtype=bool)
    for f in DECISION_FIELDS + FLOAT_FIELDS:
        eq = bits_equal(A[f], B[f])
        tick_diff |= ~eq
        if f in DECISION_FIELDS:
            tick_decision |= ~eq
        if A[f].dtype.kind == 'f':
            both = ~np.isnan(A[f]) & ~np.isnan(B[f])
            mad = float(np.max(np.abs(A[f][both] - B[f][both]))) if both.any() else 0.0
            mu = max(ulp_dist(A[f][both], B[f][both]), default=0)
            print('%-16s %6d/%-4d %14.3e %10d' % (f, eq.sum(), n, mad, mu))
        else:
            mad = int(np.max(np.abs(A[f].astype(np.int64) - B[f].astype(np.int64))))
            print('%-16s %6d/%-4d %14d %10s' % (f, eq.sum(), n, mad, '-'))

    n_diff = int(tick_diff.sum())
    n_dec = int(tick_decision.sum())
    print('\nticks with ANY difference:              %d' % n_diff)
    print('ticks with a DECISION flip (cell/count): %d' % n_dec)
    print('ticks differing only in float bits:      %d' % (n_diff - n_dec))

    for title, sel in (('DECISION flips', tick_decision),
                       ('float-only differences', tick_diff & ~tick_decision)):
        idx = np.nonzero(sel)[0]
        if not len(idx):
            continue
        print('\n=== %s (%d ticks) ===' % (title, len(idx)))
        for i in idx:
            print('tick %d: %s' % (i, describe_tick(i, inp)))
            for f in DECISION_FIELDS + FLOAT_FIELDS:
                if not bits_equal(A[f][i:i + 1], B[f][i:i + 1])[0]:
                    print('    %-16s %s=%r  %s=%r' % (f, la, A[f][i].item(), lb, B[f][i].item()))

    if args.humble:
        hum = dict(np.load(args.humble, allow_pickle=False))
        assert np.array_equal(hum['tick_bag_index'], A['tick_bag_index'])
        print('\n=== each run vs what Humble actually published ===')
        tails = {}
        for lab, R in ((la, A), (lb, B)):
            ang, off, clr, all_a, all_o = humble_errors(R, hum)
            c = clr[~np.isnan(clr)]
            tails[lab] = ang > 5.0
            print('%s: count mismatches=%d' % (lab, int(np.sum(np.isnan(ang)))))
            print('  normal angle (deg): median=%.4f p95=%.4f max=%.4f frac>5deg=%.4f'
                  % (np.median(all_a), np.percentile(all_a, 95), all_a.max(), np.mean(all_a > 5)))
            print('  offset (m):         median=%.5f p95=%.5f max=%.5f frac>0.1m=%.4f'
                  % (np.median(all_o), np.percentile(all_o, 95), all_o.max(), np.mean(all_o > 0.1)))
            print('  front_clearance(m): median=%.5f p95=%.5f max=%.5f frac>0.1m=%.4f n=%d'
                  % (np.median(c), np.percentile(c, 95), c.max(), np.mean(c > 0.1), len(c)))
            print('  ticks with >5deg error vs Humble: %d' % int(tails[lab].sum()))
        tail = tails[la] | tails[lb]
        same_on_tail = int(np.sum(tail & ~tick_diff))
        print('\ntail ticks (>5deg vs Humble on either run): %d, of which %s and %s are '
              'bit-identical on %d' % (int(tail.sum()), la, lb, same_on_tail))

    print('\n=== verdict ===')
    if n_diff == 0:
        print('IDENTICAL: %s and %s produce bit-identical outputs on all %d frozen ticks.'
              % (la, lb, n))
    elif n_dec == 0:
        print('FLOAT-ONLY: %d ticks differ in float bits only; the chosen cell, presence and '
              'constraint count agree on all %d ticks. Root-cause the float differences.'
              % (n_diff, n))
    else:
        print('DIFFERENT: %d ticks differ (%d decision flips) -- root-cause each before any live test.'
              % (n_diff, n_dec))


if __name__ == '__main__':
    main()
