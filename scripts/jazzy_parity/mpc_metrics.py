"""Phase 3 MPC replay metrics, used by compare_runs.py's `mpc` layer.

Every replay of the same input bag runs mpc_corr's 10 Hz control timer on the
sim clock, but the timer's phase relative to the input messages is set by
when the node started, so tick k of one run is not tick k of another and
can sit up to half a period (50 ms) away. Commands are therefore compared
two ways:
  - tick-matched: each tick of run A paired with the nearest tick of run B
    within 50 ms (header stamps, sim time);
  - zero-order hold on a common 50 Hz grid: "the command in force at time t",
    what the mux and the VESC actually act on.
Both are restricted to [first tick after the injected goal, last tick before
/mpc/hold], the window in which the MPC is executing the mission's move.

Reads bags only (rosbag2_py reading is distro-agnostic).
"""
import json
import math

import numpy as np

from bag_read import read_topic, header_stamp_sec

OSQP_LABELS = {1: 'solved', 2: 'solved_inaccurate', -2: 'max_iter',
               -3: 'primal_infeasible', 3: 'primal_infeasible_inaccurate',
               -4: 'dual_infeasible', 4: 'dual_infeasible_inaccurate',
               -6: 'time_limit', -10: 'non_convex', -7: 'sigint', -1: 'no_solve'}


def load_mpc_run(bag):
    run = {'bag': str(bag)}
    d = [(header_stamp_sec(m), m.drive.steering_angle, m.drive.speed)
         for _, m in read_topic(bag, '/drive')]
    d.sort()
    run['drive'] = np.array(d, dtype=float).reshape(-1, 3)
    st = []
    for _, m in read_topic(bag, '/mpc/solver_status'):
        st.append({'t': header_stamp_sec(m), 'success': bool(m.success), 'status': int(m.status),
                   'message': m.status_message, 'solve_dt': float(m.solve_dt_sec),
                   'cost': float(m.cost), 'solver': m.solver,
                   'n_bnd': int(m.n_boundary_constraints), 'n_obs': int(m.n_obstacles),
                   'horizon': len(m.pred_x),
                   'pred': np.array([m.pred_x, m.pred_y, m.pred_yaw, m.pred_v], dtype=float)})
    st.sort(key=lambda r: r['t'])
    run['status'] = st
    ms = []
    for t, m in read_topic(bag, '/mpc/status'):
        try:
            p = json.loads(m.data)
        except ValueError:
            continue
        ms.append({'t_recv': t * 1e-9, 'iterations': p.get('iterations'), 'label': p.get('status'),
                   'solve_time_ms': p.get('solve_time_ms')})
    run['mpc_status'] = ms
    run['clamp'] = np.array(sorted((header_stamp_sec(m), m.requested_speed, m.applied_speed)
                                   for _, m in read_topic(bag, '/mpc/drive_clamp')),
                            dtype=float).reshape(-1, 3)
    cor = []
    for _, m in read_topic(bag, '/mpc/corridor_markers'):
        entry = {}
        for mk in m.markers:
            if mk.points:
                entry[mk.ns] = np.array([[p.x, p.y] for p in mk.points])
                entry['t'] = mk.header.stamp.sec + mk.header.stamp.nanosec * 1e-9
        if 'corridor_centerline' in entry:
            cor.append(entry)
    cor.sort(key=lambda e: e['t'])
    run['corridor'] = cor
    goal = [t * 1e-9 for t, _ in read_topic(bag, '/mpc/goal_drive')]
    run['goal_t'] = goal[0] if goal else None
    return run


def window(run, goal_t, hold_t):
    """[first tick after the goal, hold_t)."""
    t = run['drive'][:, 0]
    lo = t[t > goal_t][0] if goal_t is not None and np.any(t > goal_t) else t[0]
    hi = hold_t if hold_t is not None else t[-1] + 1.0
    return lo, hi


def nearest(ta, tb, max_dt=0.05):
    """For each ta[i], index into tb of the nearest sample within max_dt, or -1."""
    tb = np.asarray(tb)
    j = np.searchsorted(tb, ta)
    out = np.full(len(ta), -1, dtype=int)
    for i, (t, k) in enumerate(zip(ta, j)):
        best, bd = -1, max_dt
        for c in (k - 1, k):
            if 0 <= c < len(tb) and abs(tb[c] - t) <= bd:
                best, bd = c, abs(tb[c] - t)
        out[i] = best
    return out


def _err_stats(e):
    e = np.abs(np.asarray(e, dtype=float))
    if e.size == 0:
        return {'n': 0}
    return {'n': int(e.size), 'rms': float(np.sqrt(np.mean(e ** 2))),
            'p95': float(np.percentile(e, 95)), 'max': float(np.max(e)),
            'mean': float(np.mean(e))}


def zoh(t, v, grid):
    idx = np.searchsorted(t, grid, side='right') - 1
    out = np.full(len(grid), np.nan)
    ok = idx >= 0
    out[ok] = v[idx[ok]]
    return out


def command_stats(a, b, lo, hi, label):
    da, db = a['drive'], b['drive']
    sel = (da[:, 0] >= lo) & (da[:, 0] < hi)
    ta = da[sel, 0]
    m = nearest(ta, db[:, 0])
    ok = m >= 0
    res = {'label': label, 'ticks_a': int(sel.sum()), 'matched': int(ok.sum()),
           'steer_tick': _err_stats(da[sel, 1][ok] - db[m[ok], 1]),
           'speed_tick': _err_stats(da[sel, 2][ok] - db[m[ok], 2]),
           'match_dt_ms_max': float(np.max(np.abs(ta[ok] - db[m[ok], 0])) * 1e3) if ok.any() else None}
    grid = np.arange(lo + 0.1, hi, 0.02)
    sa, sb = zoh(da[:, 0], da[:, 1], grid), zoh(db[:, 0], db[:, 1], grid)
    va, vb = zoh(da[:, 0], da[:, 2], grid), zoh(db[:, 0], db[:, 2], grid)
    good = ~np.isnan(sa) & ~np.isnan(sb)
    res['steer_zoh50hz'] = _err_stats(sa[good] - sb[good])
    res['speed_zoh50hz'] = _err_stats(va[good] - vb[good])
    return res


def status_stats(a, b, lo, hi, label, max_list=60):
    sa = [s for s in a['status'] if lo <= s['t'] < hi]
    sb = b['status']
    m = nearest(np.array([s['t'] for s in sa]), np.array([s['t'] for s in sb]))
    disagree = []
    n_match = 0
    cost_rel, bnd_eq, hor_eq = [], 0, 0
    for s, k in zip(sa, m):
        if k < 0:
            continue
        n_match += 1
        o = sb[k]
        if (s['status'], s['success']) != (o['status'], o['success']):
            disagree.append({'t': s['t'], 'a': [s['status'], s['success'], s['message']],
                             'b': [o['status'], o['success'], o['message']],
                             'n_bnd': [s['n_bnd'], o['n_bnd']], 'n_obs': [s['n_obs'], o['n_obs']]})
        cost_rel.append(abs(s['cost'] - o['cost']) / max(abs(o['cost']), 1e-9))
        bnd_eq += s['n_bnd'] == o['n_bnd']
        hor_eq += s['horizon'] == o['horizon']
    return {'label': label, 'ticks_a': len(sa), 'matched': n_match,
            'status_disagreements': len(disagree), 'disagreement_list': disagree[:max_list],
            'cost_rel_err': _err_stats(cost_rel), 'n_boundary_equal': bnd_eq,
            'horizon_equal': hor_eq}


def status_summary(run, lo, hi):
    s = [x for x in run['status'] if lo <= x['t'] < hi]
    counts = {}
    for x in s:
        key = OSQP_LABELS.get(x['status'], str(x['status'])) + ('' if x['success'] else '/rejected')
        counts[key] = counts.get(key, 0) + 1
    its = [x['iterations'] for x in run['mpc_status'] if x['iterations'] is not None]
    dt = np.array([x['solve_dt'] for x in s]) * 1e3
    per = np.diff([x['t'] for x in s]) * 1e3
    return {'n_ticks': len(s), 'status_counts': counts,
            'solver': sorted({x['solver'] for x in s}),
            'horizon_lengths': sorted({x['horizon'] for x in s}),
            'n_boundary_counts': {str(k): int(v) for k, v in zip(*np.unique([x['n_bnd'] for x in s], return_counts=True))} if s else {},
            'iterations': (_pct(np.array(its, dtype=float)) if its else None),
            'solve_dt_ms': _pct(dt), 'tick_period_ms': _pct(per)}


def _pct(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return None
    return {'n': int(v.size), 'mean': float(v.mean()), 'p50': float(np.percentile(v, 50)),
            'p95': float(np.percentile(v, 95)), 'p99': float(np.percentile(v, 99)),
            'max': float(v.max()), 'min': float(v.min()), 'std': float(v.std())}


def clamp_stats(a, b, lo, hi, label):
    ca = a['clamp'][(a['clamp'][:, 0] >= lo) & (a['clamp'][:, 0] < hi)] if len(a['clamp']) else a['clamp']
    cb = b['clamp'][(b['clamp'][:, 0] >= lo) & (b['clamp'][:, 0] < hi)] if len(b['clamp']) else b['clamp']
    m = nearest(ca[:, 0], cb[:, 0]) if len(ca) and len(cb) else np.array([], dtype=int)
    return {'label': label, 'events_a': int(len(ca)), 'events_b': int(len(cb)),
            'a_events_with_b_event_within_50ms': int(np.sum(m >= 0)),
            'applied_speed_err': _err_stats(ca[m >= 0, 2] - cb[m[m >= 0], 2]) if len(m) else {'n': 0}}


def _poly_dist(p, q):
    """For each point of p, distance to the nearest point of q."""
    d = np.sqrt(((p[:, None, :] - q[None, :, :]) ** 2).sum(axis=2))
    return d.min(axis=1)


def corridor_stats(a, b, lo, hi, label):
    ca = [c for c in a['corridor'] if lo <= c['t'] < hi]
    tb = np.array([c['t'] for c in b['corridor']])
    m = nearest(np.array([c['t'] for c in ca]), tb, max_dt=0.15)
    cl, lr, head = [], [], []
    for c, k in zip(ca, m):
        if k < 0:
            continue
        o = b['corridor'][k]
        cl.append(float(_poly_dist(c['corridor_centerline'], o['corridor_centerline']).max()))
        for side in ('corridor_left', 'corridor_right'):
            if side in c and side in o:
                lr.append(float(_poly_dist(c[side], o[side]).max()))
        pa, pb = c['corridor_centerline'], o['corridor_centerline']
        ha = math.atan2(pa[-1, 1] - pa[0, 1], pa[-1, 0] - pa[0, 0])
        hb = math.atan2(pb[-1, 1] - pb[0, 1], pb[-1, 0] - pb[0, 0])
        head.append(abs((ha - hb + math.pi) % (2 * math.pi) - math.pi))
    return {'label': label, 'corridors_a': len(ca), 'matched': int(np.sum(m >= 0)),
            'centerline_max_dev_m': _err_stats(cl), 'edges_max_dev_m': _err_stats(lr),
            'heading_err_rad': _err_stats(head)}


STEADY_WINDOW_SEC = 24.5


def steady_mean_steer(run, hi):
    """Mean commanded steering over the last STEADY_WINDOW_SEC before /mpc/hold
    (bag t ~17-41.4 s, after the obstacle manoeuvre). Phase-robust: the
    tick-level floor is dominated by WHICH tick the 1 s corridor rebuild lands
    on (a sawtooth shifted by one tick), which a mean over ~24 rebuild cycles
    does not see."""
    d = run['drive']
    sel = (d[:, 0] >= hi - STEADY_WINDOW_SEC) & (d[:, 0] < hi)
    return float(d[sel, 1].mean()), float(d[sel, 2].mean())


def compare_pair(a, b, lo, hi, label):
    ma, va = steady_mean_steer(a, hi)
    mb, vb = steady_mean_steer(b, hi)
    return {'steady_window': {'mean_steer_a': ma, 'mean_steer_b': mb, 'd_mean_steer': ma - mb,
                              'mean_speed_a': va, 'mean_speed_b': vb, 'd_mean_speed': va - vb},
            'commands': command_stats(a, b, lo, hi, label),
            'solver': status_stats(a, b, lo, hi, label),
            'clamp': clamp_stats(a, b, lo, hi, label),
            'corridor': corridor_stats(a, b, lo, hi, label)}
