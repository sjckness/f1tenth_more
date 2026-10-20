#!/usr/bin/env python3
"""
semantic_merge_replay.py: the merged semantic layer, replayed offline on recorded bags.

Usage (no ROS environment needed; numpy, scipy, pyyaml, rosbags):
    python3 tools/semantic_merge_replay.py --out output/semantic_merge_replay BAG [BAG ...]

What is replayed: semantic_layer_node after the obstacles-v2 merge
(2026-10-20), with its own pure functions and stack_params.yaml's values.
Reading and decoding come from tools/obstacles_v2_replay.py.
- Every /camera/detections_3d batch: update_tracks_batch (memory on:
  prune_confirmed=False; z_base from the camera transform), then
  dedupe_tracks.
- At 10 Hz (scan stamps), the LiDAR cycle, then the memory cycle:
  - the LiDAR cycle runs cell ownership by the CURRENT recognised tracks,
    island candidates, update_unknown_batch (plausibility gate) and
    dedupe_tracks;
  - the memory cycle (object_memory.memory_cycle) uses the node's own
    visibility wiring: LiDAR sector and occlusion, the camera frustum from
    /tf_static, and "camera live" when a detection batch came within
    semantic_layer_sensor_stale_s. A track the memory lets go is offered to
    hand_over_to_lidar.

What is reported, per bag, for every recognised track that was ever
confirmed: its TRACKED -> MEMORY -> (handed over | removed) timeline, and
COVERAGE. A track is covered on a cycle when the MPC would have it:
- live (current recognised);
- remembered (MEMORY and plausible);
- after a hand-over, the confirmed unknown carrying the same id.
A "gap" is a cycle between the first and the last covered cycle on which
the object is not covered.

Outputs in --out, per bag: <name>.timeline.csv and <name>.summary.json.
"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'tools'))
for sub in ('src/f1tenth_costmap', 'src/f1tenth_params'):
    sys.path.insert(0, str(REPO / sub))

import obstacles_v2_replay as v2  # noqa: E402
from f1tenth_costmap import object_memory as om  # noqa: E402
from f1tenth_costmap import unknown_objects as uo  # noqa: E402
from f1tenth_costmap.semantic_layer import (  # noqa: E402
    compose_base_link_to_map, dedupe_tracks, find_nearest_pose_by_stamp, hand_over_to_lidar,
    update_tracks_batch, update_unknown_batch)
from f1tenth_params.object_geometry import ObstacleFilter  # noqa: E402

CAMERA_FRAME = 'zed2_left_camera_frame'
# Only what this replay reads: other topics of old bags may not decode with
# today's .msg files (MpcSolverStatus grew since September).
v2.TOPICS = ('/scan', '/perception/walls', '/camera/detections_3d', '/tf_static',
             '/ekf_global/odometry/filtered')


def quat_of(r):
    """Unit quaternion (x, y, z, w) of a 3x3 rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(x, r[2, 1] - r[1, 2])
    y = math.copysign(y, r[0, 2] - r[2, 0])
    z = math.copysign(z, r[1, 0] - r[0, 1])
    return x, y, z, w


def base_to_frame(resolve, frame):
    """The node's lookup_transform(frame, base_link) as ((t), (q)), or None."""
    m = resolve(frame)
    if m is None:
        return None
    inv = np.linalg.inv(m)
    return tuple(float(v) for v in inv[:3, 3]), quat_of(inv[:3, :3])


def params_from(stack):
    mem = om.MemoryParams(
        fov_miss_remove_s=float(stack['semantic_layer_fov_miss_remove_s']),
        memory_decay_s=float(stack['semantic_layer_memory_decay_s']),
        person_growth_mps=float(stack['semantic_layer_person_memory_growth_mps']),
        person_growth_max_m=float(stack['semantic_layer_person_memory_growth_max_m']))
    vis = om.VisibilityParams(
        stereo_hfov_rad=math.radians(float(stack['semantic_layer_stereo_hfov_deg'])),
        stereo_depth_min_m=float(stack['semantic_layer_stereo_depth_min_m']),
        stereo_depth_max_m=float(stack['semantic_layer_stereo_depth_max_m']),
        occlusion_margin_m=float(stack['semantic_layer_occlusion_margin_m']),
        stereo_occlusion_use_lidar=bool(stack['semantic_layer_stereo_occlusion_use_lidar']))
    unk = uo.UnknownParams(
        range_m=stack['unknown_range_m'], wall_tol_m=stack['unknown_wall_tol_m'],
        object_margin_m=stack['unknown_object_margin_m'],
        cluster_gap_m=stack['unknown_cluster_gap_m'], min_cells=stack['unknown_min_cells'],
        persist_s=stack['unknown_persist_s'], decay_s=stack['unknown_decay_s'],
        unobserved_decay_s=stack['unknown_unobserved_decay_s'],
        half_width_m=stack['swept_clearance_body_half_width_m'])
    limits = ObstacleFilter(float(stack['obstacle_z_min']), float(stack['obstacle_z_max']),
                            float(stack['min_obstacle_radius']),
                            float(stack['max_obstacle_radius']))
    return mem, vis, unk, limits, float(stack['semantic_layer_sensor_stale_s'])


def replay(bag, out_dir, stack, store):
    name = Path(bag).name if Path(bag).name != 'bag' else Path(bag).parent.name
    data = v2.read_bag(bag, store)
    resolve = v2.tf_to_base(data['/tf_static'])
    mem_p, vis_p, unk_p, limits, stale_s = params_from(stack)
    merge_d, alpha = 0.5, 0.3
    gpose = [(v2.stamp(m), v2.planar(m)) for _r, m in data['/ekf_global/odometry/filtered']]
    if not gpose:
        return {'bag': name, 'skipped': 'no /ekf_global/odometry/filtered (no map pose)'}
    cam_tf = base_to_frame(resolve, CAMERA_FRAME)
    events = ([(v2.stamp(m), 'pose', i) for i, (_r, m) in
               enumerate(data['/ekf_global/odometry/filtered'])]
              + [(v2.stamp(m), 'walls', m) for _r, m in data['/perception/walls']]
              + [(v2.stamp(m), 'det', m) for _r, m in data['/camera/detections_3d']]
              + [(v2.stamp(m), 'scan', m) for _r, m in data['/scan']])
    events.sort(key=lambda e: e[0])

    tracks, memory, ids = [], {}, itertools.count()
    history, latest_walls, last_cycle, last_det, laser, laser_tf = [], None, -math.inf, None, \
        None, None
    rows, life = [], {}
    handed = {}
    verdicts = {}

    def plausible(trk):
        return uo.plausible(trk, limits)

    def fold(survivor, folded):
        m = memory.pop(folded.track_id, None)
        if m is not None:
            m.last_hit_count = survivor.hit_count
            memory[survivor.track_id] = m

    def confirmed_walls():
        if latest_walls is None:
            return []
        pose = find_nearest_pose_by_stamp(history, v2.stamp(latest_walls))
        if pose is None:
            return []
        return [(compose_base_link_to_map(t.p0_bl.x, t.p0_bl.y, pose),
                 compose_base_link_to_map(t.p1_bl.x, t.p1_bl.y, pose))
                for t in latest_walls.tracks if t.state == 1]

    for t, kind, payload in events:
        if kind == 'pose':
            history.append(gpose[payload])
            if len(history) > 200:
                history.pop(0)
            continue
        if kind == 'walls':
            latest_walls = payload
            continue
        if kind == 'det':
            last_det = t
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
                xb, yb, zb = (m @ np.array([c.x, c.y, c.z, 1.0]))[:3]
                xm, ym = compose_base_link_to_map(xb, yb, pose)
                cov = det.results[0].pose.covariance
                var = max(float(cov[0]), float(cov[7]))
                batch.append((str(hyp.class_id), xm, ym, float(hyp.score),
                              math.sqrt(var) if var > 0 else None, float(det.bbox.size.x),
                              float(zb)))
            tracks = update_tracks_batch(tracks, batch, t, merge_d, alpha, 3, 5,
                                         lambda: next(ids), confirm_grace_misses=2,
                                         alpha_sigma_ref_m=0.05, min_alpha_scale=0.25,
                                         gate_sigma_scale=2.0, prune_confirmed=False)
            tracks, _ = dedupe_tracks(tracks, merge_d, fold, plausible)
            continue
        if t - last_cycle < 0.1 - 1e-6 or not history:
            continue
        dt = 0.0 if last_cycle == -math.inf else t - last_cycle
        last_cycle = t
        scan = payload
        if laser is None:
            lm = resolve(scan.header.frame_id)
            if lm is None:
                continue
            laser = (lm[0, 3], lm[1, 3], math.atan2(lm[1, 0], lm[0, 0]))
            laser_tf = base_to_frame(resolve, scan.header.frame_id)
        pose = find_nearest_pose_by_stamp(history, t)
        # ---- the LiDAR cycle -------------------------------------------------
        current = uo.current_recognised(tracks, t, 2.0, memory, limits)
        pts, bearings, ranges = uo.scan_points(scan.ranges, scan.angle_min,
                                               scan.angle_increment, scan.range_min,
                                               scan.range_max, laser)
        view = uo.cell_view(pts, pose, unk_p)
        walls = confirmed_walls()
        owner = uo.cell_ownership(view, walls, [(c.x_map, c.y_map, c.radius())
                                                for c in current], unk_p)
        cands = uo.island_candidates(view, owner, pose, unk_p)
        c_, s_ = math.cos(pose[2]), math.sin(pose[2])
        cands = uo.drop_behind_walls(cands, walls, (pose[0] + c_ * laser[0] - s_ * laser[1],
                                                    pose[1] + s_ * laser[0] + c_ * laser[1]))
        island_xy = view.centres(owner == uo.OWNER_ISLAND)[2]

        def observe(trk):
            bl = uo.to_base_link([(trk.x_map, trk.y_map)], pose)[0]
            seen = uo.rays_pass_through(bl, trk.radius(), bearings, ranges, laser[:2], unk_p)
            return seen + (uo.cells_hit((trk.x_map, trk.y_map), trk.radius(), island_xy,
                                        view.resolution),)
        tracks, _stats = update_unknown_batch(tracks, cands, t, unk_p, merge_d, alpha,
                                              lambda: next(ids), observe, can_absorb=plausible)
        tracks, _ = dedupe_tracks(tracks, merge_d, fold, plausible)
        # ---- the memory cycle --------------------------------------------------
        camera_live = last_det is not None and t - last_det <= stale_s
        views = {'pose': pose, 'scan': scan, 'laser_tf': laser_tf,
                 'camera_live': camera_live and cam_tf is not None,
                 'cam_tf': cam_tf if camera_live else None, 'columns': None, 'opt_tf': None}

        def visibility(trk):
            x_b, y_b = om.map_to_base(trk.x_map, trk.y_map, views['pose'])

            def in_frame(tf):
                if tf is None:
                    return None
                return om.apply_transform(tf, (x_b, y_b, om.source_origin_height(tf)))
            radius = om.track_base_radius(trk)
            vis = om.object_visibility(
                radius, in_frame(views['laser_tf']), views['scan'], in_frame(views['cam_tf']),
                views['camera_live'], None, None, vis_p,
                view_radius=radius + trk.extra_gate_m)
            verdicts[(trk.class_id, vis.lidar, vis.stereo)] = verdicts.get(
                (trk.class_id, vis.lidar, vis.stereo), 0) + 1
            return vis

        def expire(trk):
            cls, tid = trk.class_id, trk.track_id
            if plausible(trk) and hand_over_to_lidar(trk, t, unk_p.decay_s):
                handed[tid] = (t, cls)
                return True
            return False
        tracks = om.memory_cycle(tracks, memory, dt, visibility, mem_p, expire=expire)
        # ---- coverage, per recognised identity ----------------------------------
        cur_ids = {c.track_id for c in uo.current_recognised(tracks, t, 2.0, memory, limits)}
        alive = {trk.track_id: trk for trk in tracks}
        for trk in tracks:
            if trk.confirmed and (not trk.is_unknown or trk.track_id in handed):
                life.setdefault(trk.track_id, {
                    'class': handed[trk.track_id][1] if trk.track_id in handed
                    else trk.class_id, 'cycles': []})
        for tid, info in life.items():
            trk = alive.get(tid)
            mem = memory.get(tid)
            if trk is None:
                state = 'gone'
            elif trk.is_unknown:
                state = 'unknown(hand-over)'
            elif tid in cur_ids:
                state = 'live'
            elif mem is not None and mem.state == om.MEMORY and plausible(trk):
                state = 'memory'
            else:
                state = 'not covered'
            info['cycles'].append(state)
            if trk is not None:
                rows.append((round(t, 2), tid, info['class'], state, round(trk.x_map, 2),
                             round(trk.y_map, 2), round(trk.radius(), 3),
                             None if trk.z_base is None else round(trk.z_base, 3)))

    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / f'{name}.timeline.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(('t', 'track_id', 'class', 'state', 'x_map', 'y_map', 'radius_m', 'z_base_m'))
        w.writerows(rows)
    per = []
    for tid, info in sorted(life.items()):
        cyc = info['cycles']
        covered = [c in ('live', 'memory', 'unknown(hand-over)') for c in cyc]
        first = covered.index(True) if True in covered else None
        last = len(covered) - 1 - covered[::-1].index(True) if True in covered else None
        gaps = (sum(1 for c in covered[first:last + 1] if not c) if first is not None else 0)
        per.append({'track_id': tid, 'class': info['class'],
                    'cycles': {s: cyc.count(s) for s in sorted(set(cyc))},
                    'went_to_memory': 'memory' in cyc,
                    'handed_over_at': handed.get(tid, (None,))[0],
                    'ended': cyc[-1] if cyc else None, 'gap_cycles': gaps})
    summary = {'bag': name, 'tracks': per,
               'persons': [p for p in per if p['class'] == 'person'],
               'hand_overs': len(handed),
               'visibility_verdicts': {f'{c} lidar={li} stereo={st}': n
                                       for (c, li, st), n in sorted(verdicts.items())},
               'gap_cycles_total': sum(p['gap_cycles'] for p in per)}
    with open(out_dir / f'{name}.summary.json', 'w') as f:
        json.dump(summary, f, indent=2)
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default=str(REPO / 'output/semantic_merge_replay'))
    args = ap.parse_args()
    stack = v2.stack_params()
    store = v2.typestore()
    for bag in args.bags:
        try:
            s = replay(bag, Path(args.out), stack, store)
        except Exception as exc:  # noqa: BLE001 - one bad bag does not stop the batch
            print(json.dumps({'bag': str(bag), 'error': f'{type(exc).__name__}: {exc}'}))
            continue
        persons = s.get('persons', [])
        print(json.dumps({'bag': s['bag'], 'skipped': s.get('skipped'),
                          'persons': len(persons),
                          'persons_to_memory': sum(p['went_to_memory'] for p in persons),
                          'hand_overs': s.get('hand_overs'),
                          'gap_cycles_total': s.get('gap_cycles_total')}))


if __name__ == '__main__':
    main()
