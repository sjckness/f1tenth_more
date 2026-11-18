#!/usr/bin/env python3
"""
obstacles_v2_replay.py: obstacles v2 (B7), replayed offline on recorded bags.

Usage (no ROS environment needed; numpy, scipy, matplotlib, rosbags):
    PYTHONPATH=src/f1tenth_costmap:src/f1tenth_control/mpc_controller:\
src/f1tenth_params:src/corridor_perception \
    python3 tools/obstacles_v2_replay.py --out output/obstacles_v2 BAG [BAG ...]

BAG is a rosbag2 directory (sqlite3). The .db3 is read with sqlite3 directly
and decoded with rosbags' CDR deserialisers, the repo's own .msg files
registered for f1tenth_messages: rosbags' Reader rejects the
offered_qos_profiles format some of these bags carry in metadata.yaml. Never
rclpy.

WHAT IS REPLAYED. semantic_layer_node's v2 path, with the node's own pure
functions and its parameter values (stack_params.yaml): the global-EKF pose
history; every /camera/detections_3d batch through update_tracks_batch (camera
-> base_link from /tf_static, -> map at the nearest pose, footprint width)
and dedupe_tracks; and at 10 Hz (scan header stamps) the newest /scan through
cell_view -> cell_ownership (CONFIRMED /perception/walls segments, confirmed
recognised tracks) -> island_candidates -> update_unknown_batch ->
dedupe_tracks, then obstacles_for_mpc and speed_cap in mode 'active'.

The ANALYSIS WINDOW is the first movement's start - 2 s to the second
movement's end + 2 s (|v| on /odometry/filtered above 0.05 m/s, gaps under
1 s merged); a bag with one movement uses its end, a bag with none the whole
bag, and the report says which. The tracker runs from the bag start, so it is
warm at the window start.

OUTPUTS in --out, per bag: <name>.frames.csv (one row per LiDAR cycle in the
window), <name>.summary.json, <name>_topdown_*.png.
"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sqlite3
import statistics
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
for sub in ('src/f1tenth_costmap', 'src/f1tenth_control/mpc_controller', 'src/f1tenth_params',
            'src/corridor_perception'):
    sys.path.insert(0, str(REPO / sub))

from f1tenth_costmap import unknown_objects as uo  # noqa: E402, I100
from f1tenth_costmap.semantic_layer import (  # noqa: E402
    compose_base_link_to_map, dedupe_tracks, find_nearest_pose_by_stamp,
    update_tracks_batch, update_unknown_batch)

from rosbags.typesys import get_types_from_msg, get_typestore, Stores  # noqa: E402

import yaml  # noqa: E402

TOPICS = ('/scan', '/perception/walls', '/perception/wall_observations',
          '/camera/detections_3d', '/tf_static',
          '/ekf_global/odometry/filtered', '/odometry/filtered', '/mpc/solver_status',
          '/perception/obstacles_2d', '/costmap/boundaries')
CYCLE_S = 0.1
MOVING_MPS = 0.05


def stack_params():
    with open(REPO / 'src/f1tenth_params/config/stack_params.yaml') as f:
        raw = yaml.safe_load(f)
    return {k: v['default'] for k, v in raw.items() if isinstance(v, dict) and 'default' in v}


def bag_distro(bag):
    """ROS distro that recorded `bag` (dir or .db3): its metadata.yaml's ros_distro.

    rosbag2 writes ros_distro from Iron on; the car's Humble bags have none.
    """
    p = Path(bag)
    for meta in (p / 'metadata.yaml', p.parent / 'metadata.yaml'):
        if meta.is_file():
            with open(meta) as f:
                info = (yaml.safe_load(f) or {}).get('rosbag2_bagfile_information') or {}
            return str(info.get('ros_distro') or 'humble')
    return 'humble'


def typestore(bag=None):
    store = get_typestore(Stores.ROS2_JAZZY if bag is not None and bag_distro(bag) == 'jazzy'
                          else Stores.ROS2_HUMBLE)
    types = {}
    for path in sorted((REPO / 'src/f1tenth_messages/msg').glob('*.msg')):
        types.update(get_types_from_msg(path.read_text(), f'f1tenth_messages/msg/{path.stem}'))
    if 'vision_msgs/msg/Detection3DArray' not in store.types:
        vdir = Path('/opt/ros/jazzy/share/vision_msgs/msg')
        for path in sorted(vdir.glob('*.msg')):
            types.update(get_types_from_msg(path.read_text(), f'vision_msgs/msg/{path.stem}'))
    store.register(types)
    return store


def read_bag(bag, store):
    """{topic: [(stamp_sec or receipt, msg)]} for TOPICS, receipt order."""
    db = sorted(Path(bag).glob('*.db3'))
    out = {t: [] for t in TOPICS}
    for path in db:
        con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        topics = {tid: (name, typ) for tid, name, typ in
                  con.execute('select id, name, type from topics')}
        want = {tid: v for tid, v in topics.items() if v[0] in TOPICS}
        for tid, ts, data in con.execute(
                'select topic_id, timestamp, data from messages order by timestamp'):
            if tid not in want:
                continue
            name, typ = want[tid]
            out[name].append((ts * 1e-9, store.deserialize_cdr(data, typ)))
        con.close()
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def planar(odom):
    """Return (x, y, yaw) of an Odometry message's pose."""
    p = odom.pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation))


def _mat(t):
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


def tf_to_base(tf_static_msgs, base='base_link'):
    edges = {}
    for _t, msg in tf_static_msgs:
        for tr in msg.transforms:
            edges[tr.child_frame_id] = (tr.header.frame_id, _mat(tr.transform))

    def resolve(frame):
        m = np.eye(4)
        seen = 0
        while frame != base:
            if frame not in edges or seen > 20:
                return None
            parent, e = edges[frame]
            m = e @ m
            frame, seen = parent, seen + 1
        return m
    return resolve


def movement_window(odom):
    """(start, end, n_movements, description) of the analysis window."""
    t = np.array([s for s, _ in odom])
    v = np.array([abs(m.twist.twist.linear.x) for _, m in odom])
    moving = v > MOVING_MPS
    segs = []
    for k, grp in itertools.groupby(range(len(t)), key=lambda i: moving[i]):
        idx = list(grp)
        if k:
            segs.append([t[idx[0]], t[idx[-1]]])
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] < 1.0:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    merged = [s for s in merged if s[1] - s[0] > 0.3]
    if not merged:
        return t[0], t[-1], 0, 'no movement: whole bag'
    end = merged[1][1] if len(merged) > 1 else merged[0][1]
    desc = ('1st movement start -2 s .. 2nd movement end +2 s' if len(merged) > 1
            else '1st movement start -2 s .. its end +2 s (one movement only)')
    return merged[0][0] - 2.0, end + 2.0, len(merged), desc


def replay(bag, out_dir, params, store, wall_obs_min_span=None, behind_filter=True):
    name = Path(bag).name
    data = read_bag(bag, store)
    resolve = tf_to_base(data['/tf_static'])
    p = uo.UnknownParams(
        range_m=params['unknown_range_m'], wall_tol_m=params['unknown_wall_tol_m'],
        object_margin_m=params['unknown_object_margin_m'],
        cluster_gap_m=params['unknown_cluster_gap_m'], min_cells=params['unknown_min_cells'],
        persist_s=params['unknown_persist_s'], decay_s=params['unknown_decay_s'],
        unobserved_decay_s=params['unknown_unobserved_decay_s'],
        half_width_m=params['swept_clearance_body_half_width_m'])
    merge_d, alpha = 0.5, 0.3          # semantic_layer_node merge_distance_m, ema_alpha
    if wall_obs_min_span is None:
        wall_obs_min_span = params['unknown_wall_obs_min_span_m']
    strip_half = params['swept_clearance_body_half_width_m'] + params[
        'lidar_front_wall_front_strip_margin_m']
    margins = params.get('obstacle_class_margin_m') or {}

    gpose = [(stamp(m), planar(m)) for _r, m in data['/ekf_global/odometry/filtered']]
    lpose = [(stamp(m), planar(m)) for _r, m in data['/odometry/filtered']]
    w0, w1, n_moves, wdesc = movement_window(
        [(stamp(m), m) for _r, m in data['/odometry/filtered']])

    events = ([(stamp(m), 'pose', i) for i, (_r, m) in
               enumerate(data['/ekf_global/odometry/filtered'])]
              + [(stamp(m), 'walls', m) for _r, m in data['/perception/walls']]
              + [(stamp(m), 'obs', m) for _r, m in data['/perception/wall_observations']]
              + [(stamp(m), 'det', m) for _r, m in data['/camera/detections_3d']]
              + [(stamp(m), 'scan', m) for _r, m in data['/scan']])
    events.sort(key=lambda e: e[0])

    tracks, ids = [], itertools.count()
    history, latest_walls, latest_obs, last_cycle = [], None, None, -math.inf
    laser = None
    rows, life = [], {}
    n_upgrades, upgrade_delays, n_dedupe = 0, [], 0
    snapshots = []

    def confirmed_walls():
        if latest_walls is None:
            return []
        pose = find_nearest_pose_by_stamp(history, stamp(latest_walls))
        if pose is None:
            return []
        segs = []
        for t in latest_walls.tracks:
            if t.state != 1:
                continue
            a = compose_base_link_to_map(t.p0_bl.x, t.p0_bl.y, pose)
            b = compose_base_link_to_map(t.p1_bl.x, t.p1_bl.y, pose)
            segs.append((a, b))
        return segs + observed_walls()

    def observed_walls():
        if latest_obs is None or wall_obs_min_span <= 0.0:
            return []
        pose = find_nearest_pose_by_stamp(history, stamp(latest_obs))
        if pose is None:
            return []
        return [(compose_base_link_to_map(o.p0.x, o.p0.y, pose),
                 compose_base_link_to_map(o.p1.x, o.p1.y, pose))
                for o in latest_obs.observations
                if o.source != 1 and math.hypot(o.p1.x - o.p0.x, o.p1.y - o.p0.y)
                >= wall_obs_min_span]

    for t, kind, payload in events:
        if kind == 'pose':
            history.append(gpose[payload])
            if len(history) > 200:
                history.pop(0)
            continue
        if kind == 'walls':
            latest_walls = payload
            continue
        if kind == 'obs':
            latest_obs = payload
            continue
        if kind == 'det':
            if not history:
                continue
            msg = payload
            m = resolve(msg.header.frame_id) if msg.detections else None
            pose = find_nearest_pose_by_stamp(history, t)
            batch = []
            for det in msg.detections:
                if m is None or not det.results:
                    continue
                hyp = det.results[0].hypothesis
                if float(hyp.score) < 0.5:
                    continue
                c = det.bbox.center.position
                xb, yb, _zb = (m @ np.array([c.x, c.y, c.z, 1.0]))[:3]
                xm, ym = compose_base_link_to_map(xb, yb, pose)
                cov = det.results[0].pose.covariance
                var = max(float(cov[0]), float(cov[7]))
                batch.append((str(hyp.class_id), xm, ym, float(hyp.score),
                              math.sqrt(var) if var > 0 else None, float(det.bbox.size.x)))
            unknown_before = {tr.track_id: tr for tr in tracks if tr.is_unknown}
            tracks = update_tracks_batch(tracks, batch, t, merge_d, alpha, 3, 5,
                                         lambda: next(ids), confirm_grace_misses=2,
                                         alpha_sigma_ref_m=0.05, min_alpha_scale=0.25,
                                         gate_sigma_scale=2.0)
            for tr in tracks:
                if tr.track_id in unknown_before and not tr.is_unknown:
                    n_upgrades += 1
                    upgrade_delays.append(t - life.get(tr.track_id, {}).get('born', t))
                    life.setdefault(tr.track_id, {})['upgraded_to'] = tr.class_id
            tracks, folded = dedupe_tracks(tracks, merge_d)
            n_dedupe += folded
            continue
        # scan: one LiDAR cycle at most every CYCLE_S
        if t - last_cycle < CYCLE_S - 1e-6 or not history:
            continue
        last_cycle = t
        scan = payload
        if laser is None:
            lm = resolve(scan.header.frame_id)
            if lm is None:
                continue
            laser = (lm[0, 3], lm[1, 3], math.atan2(lm[1, 0], lm[0, 0]))
        pose = find_nearest_pose_by_stamp(history, t)
        t0 = time.perf_counter()
        pts, bearings, ranges = uo.scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                               scan.range_min, scan.range_max, laser)
        objects = [(tr.x_map, tr.y_map, tr.radius()) for tr in uo.fresh_recognised(
            tracks, t, params['semantic_recognised_max_age_s'])]
        view = uo.cell_view(pts, pose, p)
        walls = confirmed_walls()
        owner = uo.cell_ownership(view, walls, objects, p)
        cands = uo.island_candidates(view, owner, pose, p)
        if behind_filter:
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            origin = (pose[0] + c * laser[0] - s_ * laser[1],
                      pose[1] + s_ * laser[0] + c * laser[1])
            cands = uo.drop_behind_walls(cands, walls, origin)

        island_xy = view.centres(owner == uo.OWNER_ISLAND)[2]

        def observe(trk, pose=pose, bearings=bearings, ranges=ranges, island_xy=island_xy):
            bl = uo.to_base_link([(trk.x_map, trk.y_map)], pose)[0]
            seen = uo.rays_pass_through(bl, trk.radius(), bearings, ranges, laser[:2], p)
            return seen + (uo.cells_hit((trk.x_map, trk.y_map), trk.radius(), island_xy,
                                        view.resolution),)
        tracks, st = update_unknown_batch(tracks, cands, t, p, merge_d, alpha,
                                          lambda: next(ids), observe)
        tracks, folded = dedupe_tracks(tracks, merge_d)
        n_dedupe += folded
        obstacles = uo.obstacles_for_mpc(tracks, pose, 'active', margins, t,
                                         params['semantic_recognised_max_age_s'])
        cap = uo.speed_cap(tracks, pose, 'active', True, 0.4, 1.0, p.front_x_m, strip_half)
        ms = (time.perf_counter() - t0) * 1e3
        live = {tr.track_id for tr in tracks}
        for tr in tracks:
            rec = life.setdefault(tr.track_id, {'born': t, 'class0': tr.class_id})
            rec.setdefault('born', t)
            rec.setdefault('class0', tr.class_id)
            rec['last'] = t
            if tr.is_unknown and tr.confirmed and 'confirmed' not in rec:
                rec['confirmed'] = t
                rec['where_map'] = (round(tr.x_map, 2), round(tr.y_map, 2))
                bl = uo.to_base_link([(tr.x_map, tr.y_map)], pose)[0]
                rec['where_bl'] = (round(float(bl[0]), 2), round(float(bl[1]), 2))
                rec['near_wall_m'] = round(min(
                    [float(uo._segment_distance(np.array([[tr.x_map, tr.y_map]]), a, b)[0])
                     for a, b in walls] or [math.inf]), 2)
            if tr.is_unknown:
                rec['max_size'] = max(rec.get('max_size', 0.0), 2.0 * tr.radius())
        for tid, rec in life.items():
            if tid not in live and 'died' not in rec and 'last' in rec:
                rec['died'] = t
        if not (w0 <= t <= w1):
            continue
        counts = np.bincount(owner.ravel(), minlength=4)
        unknown = [tr for tr in tracks if tr.is_unknown]
        rows.append({
            't': round(t - w0, 3), 'wall_cells': int(counts[1]), 'object_cells': int(counts[2]),
            'island_cells': int(counts[3]), 'candidates': len(cands),
            'unknown_tentative': sum(1 for tr in unknown if not tr.confirmed),
            'unknown_confirmed': sum(1 for tr in unknown if tr.confirmed),
            'recognised_confirmed': sum(1 for tr in tracks if tr.confirmed and not tr.is_unknown),
            'spawned': st['spawned'], 'deleted': st['deleted'], 'absorbed': st['absorbed'],
            'mpc_obstacles_v2': len(obstacles), 'cell_rows_v2': 0, 'speed_cap': cap,
            'upgrades_total': n_upgrades, 'duplicates_total': n_dedupe, 'lidar_ms': round(ms, 3)})
        snapshots.append((t, pose, view, owner, walls,
                          [(tr.track_id, tr.class_id, tr.x_map, tr.y_map, tr.radius(),
                            tr.confirmed) for tr in tracks], cap,
                          [d for tr in tracks if tr.confirmed for d in uo.track_disks(tr)]))

    # ---- live, for comparison ------------------------------------------------
    solves = [m for _r, m in data['/mpc/solver_status'] if w0 <= stamp(m) <= w1]
    # How close each LIVE predicted trajectory came to the disks v2 would have
    # handed the MPC at that instant (edge distance, base_link positions of
    # the prediction): below car_radius + avoidance_margin (0.32 m) the w_obs
    # term would have been active, i.e. v2 would have changed that plan.
    plan_clearance = []
    snap_t = np.array([sn[0] for sn in snapshots]) if snapshots else np.empty(0)
    for sol in solves:
        if not len(sol.pred_x) or snap_t.size == 0:
            continue
        ts = stamp(sol)
        k = int(np.argmin(np.abs(snap_t - ts)))
        if abs(snap_t[k] - ts) > 0.2:
            continue
        tf = _odom_to_map(lpose, gpose, ts)
        if tf is None:
            continue
        c, s_ = math.cos(tf[2]), math.sin(tf[2])
        px = np.asarray(sol.pred_x, dtype=float)
        py = np.asarray(sol.pred_y, dtype=float)
        mx, my = tf[0] + c * px - s_ * py, tf[1] + s_ * px + c * py
        disks = snapshots[k][7]        # every confirmed track's disks (track_disks)
        if not disks:
            plan_clearance.append(math.inf)
            continue
        dk = np.asarray(disks, dtype=float)
        dist = (np.hypot(mx[:, None] - dk[None, :, 0], my[:, None] - dk[None, :, 1])
                - dk[None, :, 2])
        plan_clearance.append(float(dist.min()))
    live_obs = [len(m.obstacles) for _r, m in data['/perception/obstacles_2d']
                if w0 <= stamp(m) <= w1]
    confirmed_unknown = {tid: rec for tid, rec in life.items() if 'confirmed' in rec
                         and w0 <= rec['confirmed'] <= w1}
    lifetimes = [rec.get('died', rec.get('last', rec['confirmed'])) - rec['born']
                 for rec in confirmed_unknown.values()]
    ms = [r['lidar_ms'] for r in rows]

    def pct(v, q):
        return round(float(np.percentile(v, q)), 3) if v else None
    summary = {
        'bag': name, 'window': [round(w0, 3), round(w1, 3)], 'window_rule': wdesc,
        'movements': n_moves, 'lidar_cycles': len(rows),
        'cells_mean': {k: round(statistics.mean(r[k] for r in rows), 1) if rows else None
                       for k in ('wall_cells', 'object_cells', 'island_cells')},
        'unknown_confirmed_objects': len(confirmed_unknown),
        'unknown_confirmed_per_frame_max': max((r['unknown_confirmed'] for r in rows), default=0),
        'unknown_confirmed_per_frame_mean': round(statistics.mean(
            r['unknown_confirmed'] for r in rows), 3) if rows else None,
        'unknown_lifetime_s': {'p50': pct(lifetimes, 50), 'max': pct(lifetimes, 100)},
        'unknown_objects': {str(k): v for k, v in sorted(confirmed_unknown.items())},
        'upgrades': n_upgrades, 'upgrade_delays_s': [round(d, 3) for d in upgrade_delays],
        'duplicates': n_dedupe, 'cell_rows_v2': 0,
        'mpc_obstacles_v2_per_cycle': {'mean': round(statistics.mean(
            r['mpc_obstacles_v2'] for r in rows), 3) if rows else None,
            'max': max((r['mpc_obstacles_v2'] for r in rows), default=0)},
        'speed_cap_cycles': sum(1 for r in rows if r['speed_cap'] > 0),
        'live_plan_to_v2_disks_m': {
            'solves': len(plan_clearance),
            'min': pct([c for c in plan_clearance if math.isfinite(c)], 0),
            'p5': pct([c for c in plan_clearance if math.isfinite(c)], 5),
            'below_0.32': sum(1 for c in plan_clearance if c < 0.32),
            'below_0.20': sum(1 for c in plan_clearance if c < 0.20)},
        'lidar_ms_linus': {'p50': pct(ms, 50), 'p95': pct(ms, 95), 'max': pct(ms, 100)},
        'live': {
            'solves': len(solves),
            'solve_ms': {'p50': pct([1e3 * s.solve_dt_sec for s in solves], 50),
                         'p95': pct([1e3 * s.solve_dt_sec for s in solves], 95)},
            'n_obstacles_per_solve': {'mean': round(statistics.mean(
                s.n_obstacles for s in solves), 3) if solves else None,
                'max': max((s.n_obstacles for s in solves), default=None)},
            'n_boundary_constraints_per_solve': {'mean': round(statistics.mean(
                s.n_boundary_constraints for s in solves), 3) if solves else None},
            'obstacles_2d_msgs': len(live_obs),
            'detections_3d_msgs': len(data['/camera/detections_3d']),
            'last_detection_s_after_start': round(
                stamp(data['/camera/detections_3d'][-1][1]) - stamp(data['/scan'][0][1]), 2)
            if data['/camera/detections_3d'] else None,
        },
    }
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if rows:
        with open(out_dir / f'{name}.frames.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=1, default=float))
    pngs = draw(name, snapshots, solves, lpose, gpose, out_dir, w0)
    summary['pngs'] = pngs
    (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=1, default=float))
    return summary


def _odom_to_map(lpose, gpose, t):
    """(x, y, yaw) of the map <- odom transform at t, from the two EKFs' poses."""
    lo = find_nearest_pose_by_stamp(lpose, t)
    gl = find_nearest_pose_by_stamp(gpose, t)
    if lo is None or gl is None:
        return None
    yaw = gl[2] - lo[2]
    c, s = math.cos(yaw), math.sin(yaw)
    return (gl[0] - (c * lo[0] - s * lo[1]), gl[1] - (s * lo[0] + c * lo[1]), yaw)


def draw(name, snapshots, solves, lpose, gpose, out_dir, w0, n=3):
    """Top-down PNGs: the most unknown objects, the middle, the first speed cap (or the last)."""
    if not snapshots:
        return []
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    capped = [i for i, snap in enumerate(snapshots) if snap[6] > 0]
    pick = [max(range(len(snapshots)),
                key=lambda i: sum(1 for o in snapshots[i][5] if o[1] == 'unknown' and o[5])),
            len(snapshots) // 2, capped[len(capped) // 2] if capped else len(snapshots) - 1]
    out = []
    colours = {uo.OWNER_WALL: '#3060ff', uo.OWNER_OBJECT: '#20b040', uo.OWNER_ISLAND: '#ff3030'}
    for k, i in enumerate(dict.fromkeys(pick)):
        t, pose, view, owner, walls, objs, cap, _disks = snapshots[i]
        fig, ax = plt.subplots(figsize=(7, 7))
        for own, col in colours.items():
            rows, cols, xy = view.centres(owner == own)
            if xy.size:
                ax.scatter(xy[:, 0], xy[:, 1], s=4, c=col, marker='s',
                           label=f'{uo.OWNER_NAMES[own]} cells ({xy.shape[0]})')
        for a, b in walls:
            ax.plot([a[0], b[0]], [a[1], b[1]], color='#3060ff', lw=2)
        for x, y, r in _disks:     # what the MPC receives (track_disks), filled
            ax.add_patch(plt.Circle((x, y), r, color='#a0a0a0', alpha=0.35, lw=0))
        for tid, cls, x, y, r, conf in objs:
            col = '#606060' if cls == 'unknown' else '#e08000'
            ax.add_patch(plt.Circle((x, y), r, fill=False, color=col, lw=1.2 if conf else 0.6,
                                    ls='-' if conf else '--'))
            ax.text(x + r, y + r, f'{tid} {cls}{"" if conf else "?"}', fontsize=7, color=col)
        ax.plot(pose[0], pose[1], 'k^', ms=10)
        ax.plot([pose[0], pose[0] + 0.5 * math.cos(pose[2])],
                [pose[1], pose[1] + 0.5 * math.sin(pose[2])], 'k-')
        sol = [s for s in solves if abs((s.header.stamp.sec + s.header.stamp.nanosec * 1e-9)
                                        - t) < 0.15]
        if sol and len(sol[0].pred_x):
            tf = _odom_to_map(lpose, gpose, t)
            if tf is not None:
                c, s_ = math.cos(tf[2]), math.sin(tf[2])
                px = np.asarray(sol[0].pred_x, dtype=float)
                py = np.asarray(sol[0].pred_y, dtype=float)
                ax.plot(tf[0] + c * px - s_ * py, tf[1] + s_ * px + c * py, 'm.-', lw=1,
                        label='MPC predicted trajectory (live)')
        ax.set_xlim(pose[0] - 5, pose[0] + 5)
        ax.set_ylim(pose[1] - 5, pose[1] + 5)
        ax.set_aspect('equal')
        ax.set_title(f'{name}  t={t - w0:.1f} s  map; grey = MPC disks'
                     + (f'  SPEED CAP {cap:.1f} m/s' if cap > 0 else ''))
        ax.legend(loc='lower left', fontsize=7)
        path = Path(out_dir) / f'{name}_topdown_{k}.png'
        fig.savefig(path, dpi=90)
        plt.close(fig)
        out.append(str(path))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='output/obstacles_v2')
    ap.add_argument('--wall-obs-min-span', type=float, default=None,
                    help='override unknown_wall_obs_min_span_m (0: confirmed walls only)')
    ap.add_argument('--no-behind-walls', action='store_true',
                    help='keep candidates seen through a gap in a wall')
    ap.add_argument('bags', nargs='+')
    args = ap.parse_args()
    params = stack_params()
    for bag in args.bags:
        store = typestore(bag)
        s = replay(bag, args.out, params, store, args.wall_obs_min_span,
                   not args.no_behind_walls)
        print(json.dumps({k: v for k, v in s.items() if k != 'unknown_objects'}, indent=1,
                         default=float))


if __name__ == '__main__':
    main()
