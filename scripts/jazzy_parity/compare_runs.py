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


def resample_zoh(series, sample_times):
    """Zero-order-hold: value of `series` as of (at or before) each time in
    `sample_times`. For slam_toolbox's /slam/pose specifically: event
    TIMES are themselves non-deterministic run-to-run (same inputs, same
    code -- confirmed live: the interior sequence of which scans cross
    the minimum_travel_distance/heading gates differs by up to ~1.6s
    between runs, not just a few ms of jitter), so nearest-timestamp
    matching (pose_series_stats' own method, fine for the deterministic
    relay/ekf_global layers in Phase 1) isn't meaningful here -- it would
    be comparing two different real corrections that happen to be
    temporally close, not the same event. Resampling onto a common grid
    at the rate a downstream consumer would actually see ("the latest
    pose as of time t") is the metric that matches what matters: is the
    overall estimated trajectory the same, independent of exactly which
    scan produced which correction."""
    out = []
    idx = 0
    cur = None
    for t in sample_times:
        while idx < len(series) and series[idx][0] <= t:
            cur = series[idx]
            idx += 1
        out.append(cur)  # None until the first real sample arrives
    return out


def resampled_stats(humble, jazzy_runs, period_sec=1.0):
    t0 = max(humble[0][0], min(j[0][0] for j in jazzy_runs))
    t1 = min(humble[-1][0], min(j[-1][0] for j in jazzy_runs))
    sample_times = np.arange(t0, t1, period_sec)

    h_rs = resample_zoh(humble, sample_times)
    j_rs_all = [resample_zoh(j, sample_times) for j in jazzy_runs]

    floor = []
    for i in range(len(j_rs_all)):
        for j in range(i + 1, len(j_rs_all)):
            pos_e, yaw_e = [], []
            for a, b in zip(j_rs_all[i], j_rs_all[j]):
                if a is None or b is None:
                    continue
                pos_e.append(math.hypot(a[1] - b[1], a[2] - b[2]))
                yaw_e.append(abs(math.degrees(wrap(a[3] - b[3]))))
            if pos_e:
                floor.append({
                    'label': f'jazzy_{i+1}_vs_jazzy_{j+1}',
                    'n': len(pos_e),
                    'pos_err_rms': float(np.sqrt(np.mean(np.square(pos_e)))),
                    'pos_err_max': float(np.max(pos_e)),
                    'yaw_err_rms_deg': float(np.sqrt(np.mean(np.square(yaw_e)))),
                    'yaw_err_max_deg': float(np.max(yaw_e)),
                })

    vs_humble = []
    for i, j_rs in enumerate(j_rs_all):
        pos_e, yaw_e = [], []
        for a, b in zip(j_rs, h_rs):
            if a is None or b is None:
                continue
            pos_e.append(math.hypot(a[1] - b[1], a[2] - b[2]))
            yaw_e.append(abs(math.degrees(wrap(a[3] - b[3]))))
        if pos_e:
            vs_humble.append({
                'label': f'jazzy_{i+1}_vs_humble',
                'n': len(pos_e),
                'pos_err_rms': float(np.sqrt(np.mean(np.square(pos_e)))),
                'pos_err_max': float(np.max(pos_e)),
                'yaw_err_rms_deg': float(np.sqrt(np.mean(np.square(yaw_e)))),
                'yaw_err_max_deg': float(np.max(yaw_e)),
            })
    return {'period_sec': period_sec, 'n_samples': len(sample_times),
            'noise_floor': floor, 'jazzy_vs_humble': vs_humble}


def load_boundaries_series(bag_path, topic='/costmap/boundaries'):
    """-> sorted list of (t_sec, [(normal_x, normal_y, offset), ...])."""
    out = []
    for _, msg in read_topic(bag_path, topic):
        t = header_stamp_sec(msg)
        constraints = [(float(c.normal[0]), float(c.normal[1]), float(c.offset))
                       for c in msg.constraints]
        out.append((t, constraints))
    out.sort(key=lambda r: r[0])
    return out


def load_scalar_series(bag_path, topic):
    """-> sorted list of (t_sec, value) for a std_msgs/Float32-like topic
    with no header (uses bag recv time, not a message header stamp --
    Float32 has none)."""
    out = []
    for t, msg in read_topic(bag_path, topic):
        out.append((t / 1e9, float(msg.data)))
    out.sort(key=lambda r: r[0])
    return out


def load_tracks_series(bag_path, topic='/costmap/semantic_tracks'):
    """-> sorted list of (t_sec, [(class_id, score, x, y), ...])."""
    out = []
    for _, msg in read_topic(bag_path, topic):
        t = header_stamp_sec(msg)
        dets = []
        for d in msg.detections:
            hyp = d.results[0].hypothesis if d.results else None
            dets.append((hyp.class_id if hyp else None,
                         float(hyp.score) if hyp else None,
                         float(d.bbox.center.position.x), float(d.bbox.center.position.y)))
        out.append((t, dets))
    out.sort(key=lambda r: r[0])
    return out


def boundaries_stats(ref, other, label, max_dt=0.1):
    """Per matched-stamp message pair: constraint count agreement, and
    (when counts match) direct index-wise normal/offset comparison --
    costmap_boundary_node's own module docstring: "Order constraints are
    appended to the published array in -- arbitrary but fixed", so index
    i in one message corresponds to index i in another from the same
    extraction method on the same inputs, not something requiring its own
    matching step."""
    matched = nearest_match(ref, other, max_dt=max_dt)
    n_total = len(ref)
    n_matched = 0
    n_count_match = 0
    normal_angle_err_deg, offset_err = [], []
    for r, o, dt in matched:
        if o is None:
            continue
        n_matched += 1
        rc, oc = r[1], o[1]
        if len(rc) == len(oc):
            n_count_match += 1
            for (rnx, rny, roff), (onx, ony, ooff) in zip(rc, oc):
                dot = max(-1.0, min(1.0, rnx * onx + rny * ony))
                normal_angle_err_deg.append(math.degrees(math.acos(dot)))
                offset_err.append(abs(roff - ooff))
    result = {
        'label': label, 'n_ref': n_total, 'n_matched': n_matched,
        'match_rate': n_matched / n_total if n_total else 0.0,
        'count_match_rate': n_count_match / n_matched if n_matched else None,
    }
    if normal_angle_err_deg:
        a, o_ = np.array(normal_angle_err_deg), np.array(offset_err)
        result.update({
            'normal_angle_err_deg_median': float(np.median(a)),
            'normal_angle_err_deg_p90': float(np.percentile(a, 90)),
            'normal_angle_err_deg_rms': float(np.sqrt(np.mean(np.square(a)))),
            'normal_angle_err_deg_max': float(np.max(a)),
            'normal_angle_frac_over_5deg': float(np.mean(a > 5)),
            'offset_err_m_median': float(np.median(o_)),
            'offset_err_m_p90': float(np.percentile(o_, 90)),
            'offset_err_m_rms': float(np.sqrt(np.mean(np.square(o_)))),
            'offset_err_m_max': float(np.max(o_)),
        })
    return result


def scalar_stats(ref, other, label, max_dt=0.1):
    matched = nearest_match(ref, other, max_dt=max_dt)
    errs = [abs(r[1] - o[1]) for r, o, dt in matched if o is not None]
    n_matched = len(errs)
    result = {'label': label, 'n_ref': len(ref), 'n_matched': n_matched,
              'match_rate': n_matched / len(ref) if ref else 0.0}
    if errs:
        e = np.array(errs)
        result.update({
            'err_median': float(np.median(e)),
            'err_rms': float(np.sqrt(np.mean(np.square(e)))),
            'err_p90': float(np.percentile(e, 90)),
            'err_p95': float(np.percentile(e, 95)),
            'err_max': float(np.max(e)),
        })
    return result


def tracks_stats(ref, other, label, max_dt=0.2, match_dist_m=1.0):
    """Per matched-stamp message pair: track COUNT agreement, plus
    nearest-neighbour position matching within each pair (track id/order
    is not guaranteed to correspond 1:1 across runs -- the tracker's own
    spawn order can differ under the same gate-timing nondeterminism the
    SLAM layer already showed), reporting position error only for pairs
    matched within `match_dist_m`."""
    matched = nearest_match(ref, other, max_dt=max_dt)
    n_total = len(ref)
    n_matched = 0
    count_diffs = []
    pos_errs = []
    for r, o, dt in matched:
        if o is None:
            continue
        n_matched += 1
        rd, od = r[1], o[1]
        count_diffs.append(abs(len(rd) - len(od)))
        od_remaining = list(od)
        for (rc, rs, rx, ry) in rd:
            if not od_remaining:
                break
            dists = [math.hypot(rx - ox, ry - oy) for (oc, os_, ox, oy) in od_remaining]
            j = int(np.argmin(dists))
            if dists[j] <= match_dist_m:
                pos_errs.append(dists[j])
                od_remaining.pop(j)
    result = {
        'label': label, 'n_ref': n_total, 'n_matched': n_matched,
        'match_rate': n_matched / n_total if n_total else 0.0,
        'mean_count_diff': float(np.mean(count_diffs)) if count_diffs else None,
    }
    if pos_errs:
        result.update({
            'pos_err_rms': float(np.sqrt(np.mean(np.square(pos_errs)))),
            'pos_err_max': float(np.max(pos_errs)),
            'n_track_pairs': len(pos_errs),
        })
    return result


def load_map(bag_path, topic='/slam/map'):
    """-> last OccupancyGrid message on `topic`, or None."""
    last = None
    for _, msg in read_topic(bag_path, topic):
        last = msg
    return last


def compare_maps(ref_map, other_map):
    """Agreement over the WORLD-COORDINATE overlap of two OccupancyGrids
    (different runs can have different origin/width/height -- slam_toolbox
    grows the grid as it explores, so comparing raw array indices without
    going through world coordinates would silently compare unrelated
    cells). -1 (unknown) is tracked separately from the 0-100 occupancy
    scale; agreement is computed only over cells BOTH maps have actually
    observed (both != -1), since a cell unknown in one and free in the
    other is a coverage difference, not a disagreement."""
    def world_to_idx(mp, wx, wy):
        ox, oy = mp.info.origin.position.x, mp.info.origin.position.y
        res = mp.info.resolution
        return int((wx - ox) / res), int((wy - oy) / res)

    def cell(mp, arr, wx, wy):
        cx, cy = world_to_idx(mp, wx, wy)
        if 0 <= cx < mp.info.width and 0 <= cy < mp.info.height:
            return arr[cy * mp.info.width + cx]
        return -1

    ref_arr = np.array(ref_map.data, dtype=np.int16)
    other_arr = np.array(other_map.data, dtype=np.int16)

    ox0 = ref_map.info.origin.position.x
    oy0 = ref_map.info.origin.position.y
    res = ref_map.info.resolution
    both_known = 0
    agree_occ_vs_free = 0  # binary occupied(>=65)-vs-free(<65) agreement
    abs_diff_sum = 0
    ref_known = 0
    other_known = 0

    for cy in range(ref_map.info.height):
        wy = oy0 + cy * res
        for cx in range(ref_map.info.width):
            wx = ox0 + cx * res
            rv = ref_arr[cy * ref_map.info.width + cx]
            ov = cell(other_map, other_arr, wx, wy)
            if rv != -1:
                ref_known += 1
            if ov != -1:
                other_known += 1
            if rv != -1 and ov != -1:
                both_known += 1
                abs_diff_sum += abs(int(rv) - int(ov))
                r_occ = rv >= 65
                o_occ = ov >= 65
                if r_occ == o_occ:
                    agree_occ_vs_free += 1

    return {
        'ref_width': ref_map.info.width, 'ref_height': ref_map.info.height,
        'other_width': other_map.info.width, 'other_height': other_map.info.height,
        'ref_known_cells': ref_known, 'other_known_cells': other_known,
        'both_known_cells': both_known,
        'occ_vs_free_agreement': agree_occ_vs_free / both_known if both_known else None,
        'mean_abs_occupancy_diff': abs_diff_sum / both_known if both_known else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('layer', choices=['relay', 'ekf_global', 'slam',
                                       'semantic_layer', 'costmap_boundary', 'mpc'])
    ap.add_argument('humble_bag', help="Humble reference bag; for the mpc layer a "
                                       "Humble REPLAY output bag (Orin), or '-' while "
                                       "none exists (noise floor only)")
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
    ap.add_argument('--recorded-humble', default=None,
                     help='mpc only: the ORIGINAL bag, for the sanity comparison of '
                          'a Jazzy replay against the live-recorded Humble commands '
                          '(not a parity verdict -- different reference heading, '
                          'see the Phase 3 report).')
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

    elif args.layer == 'ekf_global':
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

    elif args.layer == 'slam':
        pose_topic = '/slam/pose'
        humble_pose = load_pose_series(args.humble_bag, pose_topic)
        jazzy_poses = [load_pose_series(b, pose_topic) for b in args.jazzy_bags]

        result['humble_n'] = len(humble_pose)
        result['jazzy_n'] = [len(j) for j in jazzy_poses]

        # Raw nearest-timestamp match, reported for transparency only --
        # see resample_zoh()'s own docstring for why this is NOT the
        # metric that matters for this layer (event times themselves are
        # non-deterministic, so a low/inconsistent match rate here is
        # expected, not a bug).
        floor = []
        for i in range(len(jazzy_poses)):
            for j in range(i + 1, len(jazzy_poses)):
                floor.append(pose_series_stats(
                    jazzy_poses[i], jazzy_poses[j], f'jazzy_{i+1}_vs_jazzy_{j+1}'))
        result['noise_floor_raw_event_match'] = floor

        vs_humble = [pose_series_stats(j, humble_pose, f'jazzy_{i+1}_vs_humble')
                     for i, j in enumerate(jazzy_poses)]
        result['jazzy_vs_humble_raw_event_match'] = vs_humble

        # THE metric that matters: zero-order-hold resampled trajectory
        # comparison at 1 Hz -- see resampled_stats()/resample_zoh()'s own
        # docstrings.
        result['resampled_1hz'] = resampled_stats(humble_pose, jazzy_poses, period_sec=1.0)

        # map comparison: each Jazzy run's FINAL map vs Humble's FINAL map
        humble_map = load_map(args.humble_bag)
        result['humble_map_n'] = sum(1 for _ in read_topic(args.humble_bag, '/slam/map'))
        jazzy_map_results = []
        for i, b in enumerate(args.jazzy_bags):
            jmap = load_map(b)
            n_maps = sum(1 for _ in read_topic(b, '/slam/map'))
            cmp = compare_maps(humble_map, jmap) if jmap is not None else None
            jazzy_map_results.append({'run': i + 1, 'n_maps': n_maps, 'comparison': cmp})
        result['jazzy_map_n'] = [r['n_maps'] for r in jazzy_map_results]
        result['map_comparison'] = jazzy_map_results

    elif args.layer == 'semantic_layer':
        topic = '/costmap/semantic_tracks'
        humble = load_tracks_series(args.humble_bag, topic)
        jazzy_runs = [load_tracks_series(b, topic) for b in args.jazzy_bags]
        result['humble_n'] = len(humble)
        result['jazzy_n'] = [len(j) for j in jazzy_runs]
        result['noise_floor'] = [
            tracks_stats(jazzy_runs[i], jazzy_runs[j], f'jazzy_{i+1}_vs_jazzy_{j+1}')
            for i in range(len(jazzy_runs)) for j in range(i + 1, len(jazzy_runs))]
        result['jazzy_vs_humble'] = [
            tracks_stats(j, humble, f'jazzy_{i+1}_vs_humble')
            for i, j in enumerate(jazzy_runs)]

    elif args.layer == 'costmap_boundary':
        humble_b = load_boundaries_series(args.humble_bag)
        jazzy_b = [load_boundaries_series(b) for b in args.jazzy_bags]
        humble_c = load_scalar_series(args.humble_bag, '/costmap/front_clearance')
        jazzy_c = [load_scalar_series(b, '/costmap/front_clearance') for b in args.jazzy_bags]

        result['humble_n_boundaries'] = len(humble_b)
        result['jazzy_n_boundaries'] = [len(j) for j in jazzy_b]
        result['humble_n_clearance'] = len(humble_c)
        result['jazzy_n_clearance'] = [len(j) for j in jazzy_c]

        result['boundaries_noise_floor'] = [
            boundaries_stats(jazzy_b[i], jazzy_b[j], f'jazzy_{i+1}_vs_jazzy_{j+1}')
            for i in range(len(jazzy_b)) for j in range(i + 1, len(jazzy_b))]
        result['boundaries_jazzy_vs_humble'] = [
            boundaries_stats(j, humble_b, f'jazzy_{i+1}_vs_humble')
            for i, j in enumerate(jazzy_b)]

        result['clearance_noise_floor'] = [
            scalar_stats(jazzy_c[i], jazzy_c[j], f'jazzy_{i+1}_vs_jazzy_{j+1}')
            for i in range(len(jazzy_c)) for j in range(i + 1, len(jazzy_c))]
        result['clearance_jazzy_vs_humble'] = [
            scalar_stats(j, humble_c, f'jazzy_{i+1}_vs_humble')
            for i, j in enumerate(jazzy_c)]

    elif args.layer == 'mpc':
        import mpc_metrics as mm
        jazzy = [mm.load_mpc_run(b) for b in args.jazzy_bags]
        humble = mm.load_mpc_run(args.humble_bag) if args.humble_bag != '-' else None
        # The window: from the injected goal to /mpc/hold. The hold time is
        # read from the input bag's own /mpc/hold (identical on every run);
        # each replay recorded /mpc/goal_drive, so its arrival is per run.
        hold_t = None
        src = args.recorded_humble
        if src:
            holds = [t * 1e-9 for t, m in read_topic(src, '/mpc/hold') if m.data]
            hold_t = holds[0] if holds else None
        goal_t = max(r['goal_t'] for r in jazzy if r['goal_t'] is not None)
        lo, hi = mm.window(jazzy[0], goal_t, hold_t)
        result['window'] = {'goal_t': goal_t, 'hold_t': hold_t, 'lo': lo, 'hi': hi}
        result['jazzy_summary'] = [mm.status_summary(r, lo, hi) for r in jazzy]
        result['noise_floor'] = [
            mm.compare_pair(jazzy[i], jazzy[j], lo, hi, f'jazzy_{i+1}_vs_jazzy_{j+1}')
            for i in range(len(jazzy)) for j in range(i + 1, len(jazzy))]
        if humble is not None:
            result['humble_summary'] = mm.status_summary(humble, lo, hi)
            result['jazzy_vs_humble'] = [
                mm.compare_pair(j, humble, lo, hi, f'jazzy_{i+1}_vs_humble')
                for i, j in enumerate(jazzy)]
        if src:
            rec = mm.load_mpc_run(src)
            result['recorded_humble_summary'] = mm.status_summary(rec, lo, hi)
            result['sanity_vs_recorded_humble'] = [
                mm.compare_pair(j, rec, lo, hi, f'jazzy_{i+1}_vs_recorded_humble')
                for i, j in enumerate(jazzy)]

    with open(out_dir / f'{args.layer}_metrics.json', 'w') as f:
        json.dump(result, f, indent=2, default=str)
    print(f'wrote {out_dir / (args.layer + "_metrics.json")}')
    print(json.dumps({k: v for k, v in result.items()
                       if k in ('noise_floor', 'jazzy_vs_humble', 'humble_max_yaw_step_deg',
                                'jazzy_max_yaw_step_deg', 'rejection_step_floor_deg')},
                      indent=2, default=str))


if __name__ == '__main__':
    main()
