#!/usr/bin/env python3
"""Phase 1 localization-parity metrics: noise floor (Jazzy vs Jazzy) and
Jazzy vs Humble, for the relay and ekf_global layers. Reads bags only
(rosbag2_py reading is distro-agnostic; the Jazzy TopicMetadata id=
requirement in bag_compat.py only matters for writing), so this script
itself runs the same on Jazzy or Humble.

Usage:
  compare_runs.py relay    HUMBLE_BAG JAZZY_BAG_1 [JAZZY_BAG_2 ...] --out DIR
  compare_runs.py ekf_global HUMBLE_BAG JAZZY_BAG_1 [JAZZY_BAG_2 ...] --out DIR

HUMBLE_BAG here is the *source* bag (the original recording) -- its own
/slam/pose_calibrated / /ekf_global/odometry/filtered / /tf ARE the Humble
reference for this layer, per the Phase 1 report's Step 1 finding (the
commit that produced it has zero relevant diff vs the Humble baseline).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from bag_read import read_topic, header_stamp_sec


def quat_to_yaw(q):
    # standard yaw-from-quaternion (z-axis rotation only; this stack is 2D)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def load_pose_series(bag_path, topic):
    """-> sorted list of (t_sec, x, y, yaw, cov_diag[6])."""
    out = []
    for _, msg in read_topic(bag_path, topic):
        t = header_stamp_sec(msg)
        p = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
        x, y = p.position.x, p.position.y
        yaw = quat_to_yaw(p.orientation)
        cov = msg.pose.covariance
        diag = [cov[0], cov[7], cov[14], cov[21], cov[28], cov[35]]
        out.append((t, x, y, yaw, diag))
    out.sort(key=lambda r: r[0])
    return out


def load_tf_series(bag_path, parent, child):
    """-> sorted list of (t_sec, x, y, yaw) for a specific TF edge."""
    out = []
    for _, msg in read_topic(bag_path, '/tf'):
        for tr in msg.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                out.append((t, tr.transform.translation.x, tr.transform.translation.y,
                            quat_to_yaw(tr.transform.rotation)))
    out.sort(key=lambda r: r[0])
    return out


def nearest_match(ref, other, max_dt=0.05):
    """For each ref sample, find the nearest `other` sample within max_dt.
    Returns list of (ref_row, other_row_or_None, dt)."""
    if not other:
        return [(r, None, None) for r in ref]
    other_t = np.array([o[0] for o in other])
    out = []
    for r in ref:
        idx = np.searchsorted(other_t, r[0])
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(other)]
        if not candidates:
            out.append((r, None, None))
            continue
        best = min(candidates, key=lambda i: abs(other_t[i] - r[0]))
        dt = other_t[best] - r[0]
        if abs(dt) <= max_dt:
            out.append((r, other[best], dt))
        else:
            out.append((r, None, dt))
    return out


def pose_series_stats(ref, other, label):
    matched = nearest_match(ref, other, max_dt=0.05)
    pos_err, yaw_err, dts = [], [], []
    for r, o, dt in matched:
        if o is None:
            continue
        pos_err.append(math.hypot(r[1] - o[1], r[2] - o[2]))
        yaw_err.append(abs(wrap(r[3] - o[3])))
        dts.append(dt)
    n_matched = len(pos_err)
    n_total = len(ref)
    result = {
        'label': label,
        'n_ref': n_total,
        'n_matched': n_matched,
        'match_rate': n_matched / n_total if n_total else 0.0,
    }
    if pos_err:
        pos_err = np.array(pos_err)
        yaw_err = np.array(yaw_err)
        result.update({
            'pos_err_rms': float(np.sqrt(np.mean(pos_err ** 2))),
            'pos_err_p95': float(np.percentile(pos_err, 95)),
            'pos_err_max': float(np.max(pos_err)),
            'yaw_err_rms_deg': float(math.degrees(np.sqrt(np.mean(yaw_err ** 2)))),
            'yaw_err_p95_deg': float(math.degrees(np.percentile(yaw_err, 95))),
            'yaw_err_max_deg': float(math.degrees(np.max(yaw_err))),
        })
    return result


def output_rate_stats(series):
    if len(series) < 2:
        return {'n': len(series)}
    ts = np.array([r[0] for r in series])
    dts = np.diff(ts)
    return {
        'n': len(series),
        'rate_hz_mean': float(1.0 / np.mean(dts)) if np.mean(dts) > 0 else None,
        'period_ms_p90': float(np.percentile(dts, 90) * 1000),
        'period_ms_max': float(np.max(dts) * 1000),
    }


def correction_steps(pose0_series, map_odom_series, window=0.15):
    """Largest map->odom yaw/translation step within `window` seconds of
    each pose0 (/slam/pose_calibrated) correction arriving."""
    steps = []
    for t_corr, *_ in pose0_series:
        before = [r for r in map_odom_series if t_corr - window <= r[0] < t_corr]
        after = [r for r in map_odom_series if t_corr <= r[0] <= t_corr + window]
        if not before or not after:
            continue
        b, a = before[-1], after[0]
        d_xy = math.hypot(a[1] - b[1], a[2] - b[2])
        d_yaw = abs(wrap(a[3] - b[3]))
        steps.append({'t': t_corr, 'd_xy_m': d_xy, 'd_yaw_deg': math.degrees(d_yaw)})
    return steps


def rejection_rate(pose0_series, map_odom_series, step_floor_deg):
    """A pose0 correction counts as REJECTED if the map->odom yaw step
    across it is no larger than the no-correction noise floor (i.e.
    indistinguishable from odom0-only dead-reckoning drift over the same
    window) -- see the Phase 1 report's Step 5 section for why the
    original 689-interval/37-archived-run methodology can't be
    reproduced from the one bag available here, and this substitute
    method's derivation from the outputs instead."""
    steps = correction_steps(pose0_series, map_odom_series)
    if not steps:
        return {'n_corrections': 0, 'n_rejected': 0, 'rate': None, 'steps': []}
    n_rejected = sum(1 for s in steps if s['d_yaw_deg'] <= step_floor_deg)
    return {
        'n_corrections': len(steps),
        'n_rejected': n_rejected,
        'rate': n_rejected / len(steps),
        'steps': steps,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('layer', choices=['relay', 'ekf_global'])
    ap.add_argument('humble_bag')
    ap.add_argument('jazzy_bags', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--skip-transient-sec', type=float, default=0.0,
                     help='Drop samples before t0+N seconds. ekf_global '
                          'only: this bag is a mid-mission segment whose '
                          'Humble instance had already converged before '
                          'recording started, while this replay starts '
                          'ekf_global cold (identity seed) -- see the '
                          'Phase 1 report Step 5 for the measured '
                          'convergence curve this value is chosen from.')
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    result = {'layer': args.layer, 'humble_bag': args.humble_bag,
              'jazzy_bags': args.jazzy_bags}

    if args.layer == 'relay':
        topic = '/slam/pose_calibrated'
        humble = load_pose_series(args.humble_bag, topic)
        jazzy_runs = [load_pose_series(b, topic) for b in args.jazzy_bags]

        result['humble_n'] = len(humble)
        result['jazzy_n'] = [len(j) for j in jazzy_runs]

        # noise floor: pairwise among the Jazzy runs
        floor = []
        for i in range(len(jazzy_runs)):
            for j in range(i + 1, len(jazzy_runs)):
                floor.append(pose_series_stats(
                    jazzy_runs[i], jazzy_runs[j], f'jazzy_{i+1}_vs_jazzy_{j+1}'))
        result['noise_floor'] = floor

        # Jazzy vs Humble
        vs_humble = [pose_series_stats(j, humble, f'jazzy_{i+1}_vs_humble')
                     for i, j in enumerate(jazzy_runs)]
        result['jazzy_vs_humble'] = vs_humble

    else:  # ekf_global
        odom_topic = '/ekf_global/odometry/filtered'
        humble_odom = load_pose_series(args.humble_bag, odom_topic)
        humble_map_odom = load_tf_series(args.humble_bag, 'map', 'odom')
        humble_pose0 = load_pose_series(args.humble_bag, '/slam/pose_calibrated')

        jazzy_odoms, jazzy_map_odoms = [], []
        for b in args.jazzy_bags:
            jazzy_odoms.append(load_pose_series(b, odom_topic))
            jazzy_map_odoms.append(load_tf_series(b, 'map', 'odom'))

        if args.skip_transient_sec > 0:
            t0 = humble_odom[0][0]
            cutoff = t0 + args.skip_transient_sec
            result['transient_cutoff_sec'] = args.skip_transient_sec
            humble_odom = [r for r in humble_odom if r[0] >= cutoff]
            jazzy_odoms = [[r for r in j if r[0] >= cutoff] for j in jazzy_odoms]

        result['humble_n'] = len(humble_odom)
        result['jazzy_n'] = [len(j) for j in jazzy_odoms]
        result['humble_rate'] = output_rate_stats(humble_odom)
        result['jazzy_rate'] = [output_rate_stats(j) for j in jazzy_odoms]

        floor = []
        for i in range(len(jazzy_odoms)):
            for j in range(i + 1, len(jazzy_odoms)):
                floor.append(pose_series_stats(
                    jazzy_odoms[i], jazzy_odoms[j], f'jazzy_{i+1}_vs_jazzy_{j+1}'))
        result['noise_floor'] = floor

        vs_humble = [pose_series_stats(j, humble_odom, f'jazzy_{i+1}_vs_humble')
                     for i, j in enumerate(jazzy_odoms)]
        result['jazzy_vs_humble'] = vs_humble

        # map->odom correction steps, Humble and each Jazzy run
        humble_steps = correction_steps(humble_pose0, humble_map_odom)
        result['humble_correction_steps'] = humble_steps
        result['humble_max_yaw_step_deg'] = (
            max((s['d_yaw_deg'] for s in humble_steps), default=None))

        jazzy_steps_all = []
        for i, mo in enumerate(jazzy_map_odoms):
            s = correction_steps(humble_pose0, mo)  # same pose0 times, same input
            jazzy_steps_all.append(s)
        result['jazzy_correction_steps'] = jazzy_steps_all
        result['jazzy_max_yaw_step_deg'] = [
            max((s['d_yaw_deg'] for s in js), default=None) for js in jazzy_steps_all]

        # pose0 rejection rate: use the noise floor's own 95th-percentile
        # yaw step as the "indistinguishable from drift" threshold --
        # derived from THIS data, not invented (see rejection_rate()'s own
        # docstring and the Phase 1 report's Step 5 section).
        all_floor_yaw_steps = []
        for i in range(len(jazzy_map_odoms)):
            for j in range(i + 1, len(jazzy_map_odoms)):
                # floor steps: difference between two Jazzy runs' own step sizes
                si = correction_steps(humble_pose0, jazzy_map_odoms[i])
                sj = correction_steps(humble_pose0, jazzy_map_odoms[j])
                for a, b in zip(si, sj):
                    all_floor_yaw_steps.append(abs(a['d_yaw_deg'] - b['d_yaw_deg']))
        step_floor_deg = float(np.percentile(all_floor_yaw_steps, 95)) if all_floor_yaw_steps else 0.5
        result['rejection_step_floor_deg'] = step_floor_deg
        result['humble_rejection'] = rejection_rate(humble_pose0, humble_map_odom, step_floor_deg)
        result['jazzy_rejection'] = [
            rejection_rate(humble_pose0, mo, step_floor_deg) for mo in jazzy_map_odoms]

    with open(out_dir / f'{args.layer}_metrics.json', 'w') as f:
        json.dump(result, f, indent=2, default=str)
    print(f'wrote {out_dir / (args.layer + "_metrics.json")}')
    print(json.dumps({k: v for k, v in result.items()
                       if k in ('noise_floor', 'jazzy_vs_humble', 'humble_max_yaw_step_deg',
                                'jazzy_max_yaw_step_deg', 'rejection_step_floor_deg')},
                      indent=2, default=str))


if __name__ == '__main__':
    main()
