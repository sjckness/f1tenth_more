#!/usr/bin/env python3
"""Replay archived runs through lidar_front_wall_node, scene labels OFF vs ON.

Usage (after `source install/setup.bash`):
    python3 tools/scene_labels_replay.py --out DIR [--stop-distance M] RUN [RUN ...]

RUN is an archive run folder (the one holding bag/). Each bag's /tf_static,
/odometry/filtered, /costmap/front_clearance, /camera/detections_3d and /scan
are fed, in recorded order and with the node clock set to the recorded time,
into TWO in-process LidarFrontWallNode instances: enable_scene_labels false
(the deployed code path) and true. Nothing is published to a graph: the node
publishers are replaced by capture lists, and rclpy runs on an isolated
domain with no discovery server, so a live stack on this machine never sees
the replay.

WHAT THE BAGS CAN AND CANNOT VALIDATE. The archive records no depth image,
point cloud or camera_info. So the LiDAR half of scene_labels (detection
sectors, span check, wall planes, VirtualWall with odometry) is replayed for
real; the stereo half (ground, ceiling, IGNORE, WALL pixels, OTHER-only object
depth, the stereo wall distance) is NOT, and every ON number below is the
virtual wall with no stereo input. The camera-only wall the stack actually
used (/perception/front_distance) is read back from the recorded
/perception/debug/front_wall payload as the OFF wall truth.

Per run it writes <run>.csv (every scan), <run>.md (0.5 s table + summary) and
<run>.scene.mp4: the recorded debug image with the LiDAR wall planes projected
into it and the virtual wall (perception_debug.draw_scene_overlay), next to a
top-down LiDAR view, encoded by f1tenth_logger.mission_debug_video.write_mp4.
Runs with no recorded debug image get the top-down view alone.

--model-labels CEILING_M also tints that image with ground / ceiling / wall
labels from front_clearance_node's labelling tick run on a MODEL depth image
(flat floor, flat ceiling at CEILING_M, the frame's LiDAR wall segments). It
shows where the labeller would draw them in that idealised room; with no depth
in the bags it is not a measurement and objects are absent from the model.

--label-timing also times front_clearance_node's labelling tick on SYNTHETIC
depth frames (floor, ceiling, the run's LiDAR walls, a person box) at the real
640 x 360 size: a CPU measurement, not a validation of the labels.
"""

import argparse
import csv
import glob
import json
import math
import os
import sqlite3
import statistics
import sys
import time
from pathlib import Path

# Isolated before rclpy loads: no discovery server, localhost only, own domain.
os.environ.pop('ROS_DISCOVERY_SERVER', None)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = os.environ.get('SCENE_REPLAY_DOMAIN_ID', '87')
# One BLAS thread, as the nodes run: lidar_front_wall_node is pinned to one
# core, front_clearance_node gets OPENBLAS_NUM_THREADS=1 from detection.launch.py.
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from f1tenth_description.vehicle_frames import base_front_x  # noqa: E402
from f1tenth_logger.mission_debug_video import decode_compressed_image, write_mp4  # noqa: E402
from f1tenth_params.param_defaults import _load, get_value  # noqa: E402
from f1tenth_perception import perception_debug, scene_labels  # noqa: E402
from f1tenth_perception.ground_plane import quat_to_matrix  # noqa: E402
from f1tenth_perception.lidar_front_wall import (  # noqa: E402
    PROVENANCE_NAMES, REASON_NAMES)
from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode  # noqa: E402
from mpc_controller.wall_turn import wall_turn_trigger_distance  # noqa: E402

TOPICS = ['/tf_static', '/odometry/filtered', '/costmap/front_clearance',
          '/camera/detections_3d', '/scan', '/perception/debug/front_wall',
          '/perception/obstacles_2d', '/camera/debug_image/compressed', '/mission/status']
FEED = {'/tf_static': '_tf_static_cb', '/odometry/filtered': '_odom_cb',
        '/costmap/front_clearance': '_seed_cb', '/scan': '_scan_cb'}
BT_TICK_S = 0.1          # condition sampling period for the stop / wall_turn verdicts
DEBOUNCE_TICKS = 3       # M02's front_clearance stop_condition debounce_ticks
CORRIDOR_HALF_W = 0.35   # front_clearance_corridor_half_width_m


def read_bag(run):
    db = sorted(glob.glob(f'{run}/bag/*.db3'))[0]
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    tmap = {i: (n, t) for i, n, t in con.execute('SELECT id, name, type FROM topics')}
    out = []
    for tid, (name, typ) in tmap.items():
        if name not in TOPICS:
            continue
        for ts, data in con.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=?', (tid,)):
            if name == '/camera/debug_image/compressed':
                try:
                    stamp, _fmt, jpeg = decode_compressed_image(bytes(data))
                except ValueError:
                    continue
                out.append((ts * 1e-9, name, (stamp, jpeg)))
            else:
                out.append((ts * 1e-9, name, deserialize_message(bytes(data), get_message(typ))))
    con.close()
    out.sort(key=lambda r: r[0])
    return out


#: Groups a launch file passes to the node under their OWN stack_params.yaml
#: names, not prefix-stripped: THE CONFIRMATION LEVEL (wall_confirm_*,
#: lidar_front_wall.launch.py WALL_CONFIRM_ARGS).
UNPREFIXED = {'lidar_front_wall_': ('wall_confirm_',)}


def prefixed_params(prefix, enable):
    """Return the node's stack_params.yaml block as its launch file passes it.

    Prefix stripped, plus its UNPREFIXED groups under their own names, plus
    enable_scene_labels.
    """
    params = [Parameter('enable_scene_labels', value=bool(enable))]
    for key, entry in _load().items():
        if key.startswith(prefix):
            params.append(Parameter(key[len(prefix):], value=entry['default']))
        elif key.startswith(UNPREFIXED.get(prefix, ())):
            params.append(Parameter(key, value=entry['default']))
    return params


class Capture:
    def __init__(self):
        self.items = []

    def publish(self, msg):
        self.items.append(msg)

    def pop(self):
        items, self.items = self.items, []
        return items


def coerce_like_default(name, value):
    """`value` as the type stack_params.yaml gives `lidar_front_wall_<name>`
    (or `<name>` itself, for an UNPREFIXED group).

    An override read from a YAML file arrives as whatever YAML made of it, and
    `1` for a float parameter is an InvalidParameterTypeException at node
    construction -- the same trap as a launch file's untyped string parameter.
    The stack default is the only authority on the type, so it is asked.
    """
    entry = _load().get('lidar_front_wall_' + name)
    if entry is None and name.startswith(UNPREFIXED['lidar_front_wall_']):
        entry = _load().get(name)
    if entry is None:
        return value
    want = type(entry['default'])
    if want is bool:
        return value if isinstance(value, bool) else str(value).lower() in ('1', 'true', 'yes')
    if want is float:
        return float(value)
    if want is int and not isinstance(value, bool):
        return int(value)
    if want is str:
        return str(value)
    return value


def make_node(enable, clock, overrides=None):
    # wall_debug_tracks is OFF on the car (it costs a transform and two
    # distances per live track per scan) and ON here: the per-track table and
    # the top-down drawing are exactly what it exists for.
    # WALL_DEBUG_TRACKS=0 measures the cycle as the CAR runs it, with the
    # payload off; anything else keeps it on, which is what the per-track
    # table and the top-down drawing need.
    want = os.environ.get('WALL_DEBUG_TRACKS', '1') not in ('0', 'false', 'False')
    params = prefixed_params('lidar_front_wall_', enable)
    params = [p for p in params if p.name not in
              ('wall_debug_tracks', 'wall_tracker_scan_decimation')]
    params.append(Parameter('wall_debug_tracks', value=want))
    # DECIMATION 1 in every replay. The bag holds one recorded tracker CYCLE
    # per /perception/virtual_wall/debug message -- about 3.2 a second, not the
    # 40 scans a second the car sees -- so decimating that again would compare
    # a different set of scans against the recording and would also leave
    # three cycles in four reporting the previous cycle's tracker_ms. The
    # decimation's effect is arithmetic on the RATE, not on the cost of a
    # cycle: WALL_TRACKER_DECIMATION overrides it if a run wants to see it.
    params.append(Parameter('wall_tracker_scan_decimation',
                            value=int(os.environ.get('WALL_TRACKER_DECIMATION', '1'))))
    # `overrides` is the ONE way a caller changes the node's behaviour, and it
    # never touches stack_params.yaml: a variant run is a parameter override on
    # top of the installed defaults, exactly as a launch argument would be, so
    # the defaults the car ships keep speaking for the car.
    if overrides:
        params = [p for p in params if p.name not in set(overrides)]
        params += [Parameter(k, value=coerce_like_default(k, v))
                   for k, v in overrides.items()]
    node = LidarFrontWallNode(parameter_overrides=params)
    node._now = lambda: clock[0]
    caps = {}
    for attr in ('fit_pub', 'estimate_pub', 'virtual_pub', 'virtual_debug_pub', 'planes_pub'):
        if hasattr(node, attr):
            caps[attr] = Capture()
            setattr(node, attr, caps[attr])
    return node, caps


def static_tf(tf_msgs):
    """{child: (parent, R, t)} from /tf_static, and a resolver to base_link."""
    edges = {}
    for msg in tf_msgs:
        for tr in msg.transforms:
            q, p = tr.transform.rotation, tr.transform.translation
            edges[tr.child_frame_id] = (tr.header.frame_id, quat_to_matrix(q.x, q.y, q.z, q.w),
                                        np.array([p.x, p.y, p.z]))

    def to_base(frame):
        rot, trans = np.eye(3), np.zeros(3)
        while frame != 'base_link':
            if frame not in edges:
                return None
            parent, r, t = edges[frame]
            rot, trans = r @ rot, r @ trans + t
            frame = parent
        return rot, trans
    return to_base


def zed_intrinsics(w=640, h=360):
    """fx, fy, cx, cy of the left ZED at w x h, from the factory calibration (HD720 / 2)."""
    conf = glob.glob('/usr/local/zed/settings/SN*.conf')
    k = {}
    if conf:
        section = None
        for line in open(conf[0]):
            line = line.strip()
            if line.startswith('['):
                section = line
            elif section == '[LEFT_CAM_HD]' and '=' in line:
                name, val = line.split('=')
                k[name] = float(val)
    if not k:
        return None
    s = w / 1280.0
    return k['fx'] * s, k['fy'] * s, k['cx'] * s, k['cy'] * s


def reference_wall(scan, laser_pose, front_x, max_gap_m=1.5):
    """Independent LiDAR reference for the wall ahead, from the bumper, or NaN.

    No detection cut, every beam. A line counts when it faces the car and its
    inliers bracket the heading with at most max_gap_m between the nearest
    inlier on each side (a person's shadow on a wall 5 m away is ~1.1 m; the
    doorway of 2026-09-11 is 2.4 m). The longest such line wins: a person or a
    laptop never does.
    """
    pts, _, _ = scene_labels.scan_points(
        scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
        laser_pose, math.radians(60.0))
    best = None
    for w in scene_labels.sequential_lines(pts, 0.03, 0.8, 15, 4, 800):
        if w.nx <= 1e-6 or abs(w.normal_angle) > math.radians(45.0):
            continue
        on = np.abs(w.offset(pts[:, 0], pts[:, 1])) <= 0.03
        t = w.along(pts[on, 0], pts[on, 1])
        t_hit = w.along(w.d / w.nx, 0.0)
        left, right = t[t <= t_hit], t[t > t_hit]
        if left.size == 0 or right.size == 0 or right.min() - left.max() > max_gap_m:
            continue
        if best is None or w.span > best.span:
            best = w
    return math.nan if best is None else best.d - front_x * best.nx


def nearest_person(det_msg, to_base, front_x):
    if det_msg is None:
        return math.nan
    tf = to_base(det_msg.header.frame_id)
    if tf is None:
        return math.nan
    rot, trans = tf
    best = math.nan
    for det in det_msg.detections:
        if not det.results or det.results[0].hypothesis.class_id != 'person':
            continue
        p = det.bbox.center.position
        x, y, _ = rot @ np.array([p.x, p.y, p.z]) + trans
        if x > 0 and abs(y) < 1.0:
            d = x - front_x
            best = d if not math.isfinite(best) else min(best, d)
    return best


def nearest_obstacle(obs_msg, front_x):
    if obs_msg is None:
        return math.nan
    best = math.nan
    for o in obs_msg.obstacles:
        x, y, r = float(o.x), float(o.y), float(o.r)
        if x <= 0 or abs(y) - r >= CORRIDOR_HALF_W:
            continue
        d = max(x - r - front_x, 0.01)
        best = d if not math.isfinite(best) else min(best, d)
    return best


def first_trigger(times, values, threshold, debounce):
    """First BT-tick time a 0 < value < threshold streak reaches debounce, or None."""
    if not times:
        return None
    streak = 0
    k = 0
    t = times[0]
    last = None
    while t <= times[-1]:
        while k < len(times) and times[k] <= t:
            last = values[k]
            k += 1
        if last is not None and math.isfinite(last) and 0.0 < last < threshold:
            streak += 1
            if streak >= debounce:
                return t
        else:
            streak = 0
        t += BT_TICK_S
    return None


def topdown(scan, laser_pose, sectors, walls, virtual_m, person_m, front_x, size=360):
    """Top-down panel: 6 m ahead x +-3 m, car at the bottom."""
    img = np.full((size, size, 3), 24, np.uint8)
    scale = size / 6.0

    def px(x, y):
        return int(size / 2 - y * scale), int(size - 8 - x * scale)
    for m in range(1, 6):
        cv2.line(img, (0, px(m, 0)[1]), (size, px(m, 0)[1]), (50, 50, 50), 1)
        cv2.putText(img, f'{m}m', (2, px(m, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (110, 110, 110), 1)
    r = np.asarray(scan.ranges)
    lx, ly, lyaw = laser_pose
    ang = scan.angle_min + np.arange(r.size) * scan.angle_increment + lyaw
    bear = np.arctan2(np.sin(ang), np.cos(ang))
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    cut = scene_labels.in_sectors(bear, sectors) if sectors else np.zeros_like(ok)
    for sel, color in ((ok & ~cut, (170, 170, 170)), (ok & cut, (255, 0, 255))):
        xs, ys = lx + r[sel] * np.cos(bear[sel]), ly + r[sel] * np.sin(bear[sel])
        for x, y in zip(xs, ys):
            if 0 <= x <= 6 and abs(y) <= 3:
                cv2.circle(img, px(x, y), 1, color, -1)
    for w in walls:
        (ax, ay), (bx, by) = w.endpoints()
        cv2.line(img, px(ax, ay), px(bx, by), perception_debug.COLOR_WALL, 2)
    cv2.rectangle(img, px(front_x, 0.15), px(front_x - 0.5, -0.15), (200, 200, 200), 1)
    if math.isfinite(virtual_m):
        y = px(front_x + virtual_m, 0)[1]
        cv2.line(img, (size // 2 - 40, y), (size // 2 + 40, y), (0, 255, 0), 2)
    if math.isfinite(person_m):
        cv2.circle(img, px(front_x + person_m, 0), 5, (255, 0, 255), 1)
    return img


def project_walls(walls, cam_tf, k, z_top=1.0):
    rot, trans = cam_tf
    fx, fy, cx, cy = k
    out = []
    for w in walls:
        (ax, ay), (bx, by) = w.endpoints()
        corners = np.array([[ax, ay, 0.0], [bx, by, 0.0], [bx, by, z_top], [ax, ay, z_top]])
        opt = (corners - trans) @ rot
        if np.all(opt[:, 2] > 0.1):
            out.append(dict(w.to_dict(), outline_px=[
                [float(fx * x / z + cx), float(fy * y / z + cy)] for x, y, z in opt]))
    return out


class ModelLabeller:
    """Ground / ceiling / wall labels for the video WITHOUT recorded depth.

    The bags carry no depth, so this ray-casts a MODEL depth image (flat floor
    z = 0, flat ceiling at ceiling_m, the frame's LiDAR wall segments within
    their span, nothing else) and runs front_clearance_node's real labelling
    tick on it. The tint shows where the labeller WOULD put ground, ceiling
    and wall if the room were that model: objects, other walls and a real
    ceiling height are not in it.
    """

    def __init__(self, rows, to_base, ceiling_m, w=640, h=360):
        from std_msgs.msg import Header
        from f1tenth_perception.front_clearance_node import FrontClearanceNode
        self.node = FrontClearanceNode(parameter_overrides=prefixed_params('front_clearance_', True))
        for attr in ('scene_labels_pub', 'scene_debug_pub', 'stereo_wall_pub', 'unclassified_pub'):
            setattr(self.node, attr, Capture())
        for _, topic, msg in rows:
            if topic == '/tf_static':
                self.node._tf_static_callback(msg)
        self.frame = 'zed2_left_camera_optical_frame'
        fx, fy, cx, cy = zed_intrinsics(w, h)
        self.node._camera_k = (fx, fy, cx, cy, w, h)
        rot, self.trans = to_base(self.frame)
        vv, uu = np.mgrid[0:h, 0:w]
        self.rays = np.stack(((uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, float)), -1) @ rot.T
        self.ceiling_m = ceiling_m
        self.header = Header(frame_id=self.frame)

    def labels(self, walls):
        rays, trans = self.rays, self.trans
        z = np.full(rays.shape[:2], np.inf)
        with np.errstate(divide='ignore', invalid='ignore'):
            for plane_z in (0.0, self.ceiling_m):
                zp = (plane_z - trans[2]) / rays[..., 2]
                z = np.where((zp > 0) & (zp < z), zp, z)
            for wl in walls:
                zw = (wl.d - (trans[0] * wl.nx + trans[1] * wl.ny)) / (
                    rays[..., 0] * wl.nx + rays[..., 1] * wl.ny)
                t = wl.along(trans[0] + zw * rays[..., 0], trans[1] + zw * rays[..., 1])
                hit = (zw > 0) & (zw < z) & (t >= wl.t0) & (t <= wl.t1)
                z = np.where(hit, zw, z)
        depth = z.astype(np.float32)
        depth[~np.isfinite(depth)] = np.nan
        self.node._wall_planes = (0.0, list(walls))
        self.node._label_scene(depth, self.header, None, (75, 215, 140, 500))
        for attr in ('scene_labels_pub', 'scene_debug_pub', 'stereo_wall_pub', 'unclassified_pub'):
            getattr(self.node, attr).pop()
        return self.node._scene['labels']


def replay(run, out_dir, stop_distance, label_timing, model_ceiling_m=None):
    run = Path(run)
    name = run.name
    rows = read_bag(run)
    if not any(t == '/scan' for _, t, _ in rows):
        print(f'{name}: no /scan, skipped')
        return None
    front_x = base_front_x()
    clock = [0.0]
    off, off_cap = make_node(False, clock)
    on, on_cap = make_node(True, clock)
    to_base = static_tf([m for _, t, m in rows if t == '/tf_static'])
    t0 = rows[0][0]
    turn_trigger = wall_turn_trigger_distance(
        math.pi / 2, get_value('mpc_wheelbase_m'), get_value('mpc_steering_angle_max_rad'),
        get_value('corr_wall_turn_k_safety'), get_value('corr_wall_turn_safety_margin_m'))

    recorded = None   # latest debug/front_wall payload
    det_msg = obs_msg = None
    planes = []
    trace = []
    frames = []
    ms_off, ms_on = [], []
    status = []
    k = zed_intrinsics()
    cam_tf = to_base('zed2_left_camera_optical_frame')
    for t, topic, msg in rows:
        clock[0] = t
        if topic == '/perception/debug/front_wall':
            recorded = json.loads(msg.data)
            continue
        if topic == '/perception/obstacles_2d':
            obs_msg = msg
            continue
        if topic == '/mission/status':
            status.append((t - t0, msg.state))
            continue
        if topic == '/camera/debug_image/compressed':
            frames.append((t, msg))
            continue
        if topic == '/camera/detections_3d':
            det_msg = msg
            on._detections_cb(msg)
            continue
        if topic != '/scan':
            for node in (off, on):
                getattr(node, FEED[topic])(msg)
            continue
        a = time.perf_counter()
        off._scan_cb(msg)
        b = time.perf_counter()
        on._scan_cb(msg)
        c = time.perf_counter()
        ms_off.append((b - a) * 1e3)
        ms_on.append((c - b) * 1e3)
        est_off = off_cap['estimate_pub'].pop()[-1]
        fit_off = off_cap['fit_pub'].pop()[-1]
        fit_on = on_cap['fit_pub'].pop()[-1]
        on_cap['estimate_pub'].pop()
        vdbg = json.loads(on_cap['virtual_debug_pub'].pop()[-1].data)
        on_cap['virtual_pub'].pop()
        for p in on_cap['planes_pub'].pop():
            d = json.loads(p.data)
            planes = [scene_labels.WallLine.from_dict(w) for w in d['walls']]
        laser_pose = on._laser_pose[1] if on._laser_pose else None
        if laser_pose is None:
            continue
        lidar_off = (est_off.distance - front_x * math.cos(est_off.normal_angle)
                     if est_off.valid and math.isfinite(est_off.normal_angle)
                     else (est_off.distance - front_x if est_off.valid else math.nan))
        # The reference: the deployed plain fit when it measured, else the
        # gap-rule line (reference_wall); both blind to the new code path.
        if est_off.valid and est_off.provenance == 1:
            ref, ref_src = lidar_off, 'plain_fit'
        else:
            ref, ref_src = reference_wall(msg, laser_pose, front_x), 'gap_rule'
        person = nearest_person(det_msg, to_base, front_x) if det_msg is not None else math.nan
        obstacle = nearest_obstacle(obs_msg, front_x)
        v_on = vdbg['distance_m'] if vdbg['valid'] else math.nan
        clear_on = min([x for x in (v_on, obstacle) if math.isfinite(x)], default=math.nan)
        rec_fd = recorded.get('front_distance_m') if recorded else None
        rec_cl = recorded.get('front_clearance_m') if recorded else None
        trace.append({
            't': round(t - t0, 3),
            'off_wall_m': rec_fd if rec_fd is not None and rec_fd > 0 else math.nan,
            'off_wall_src': 'camera' if recorded else 'n/a',
            'off_lidar_m': lidar_off,
            'off_lidar_src': PROVENANCE_NAMES[est_off.provenance],
            'off_fit': REASON_NAMES.get(fit_off.reason, fit_off.reason),
            'on_wall_m': v_on, 'on_src': vdbg['source'], 'on_disagree': vdbg['disagree'],
            'on_fit': REASON_NAMES.get(fit_on.reason, fit_on.reason),
            'on_sectors': len(on._sectors(t, laser_pose)),
            'lidar_ref_m': ref, 'ref_src': ref_src,
            'person_m': person,
            'off_clear_m': rec_cl if rec_cl is not None and rec_cl > 0 else math.nan,
            'on_clear_proxy_m': clear_on,
        })

    for node in (off, on):
        node.destroy_node()
    if not trace:
        print(f'{name}: no usable scans')
        return None

    times = [r['t'] for r in trace]
    stop_off = first_trigger(times, [r['off_wall_m'] for r in trace], stop_distance,
                             DEBOUNCE_TICKS)
    stop_on = first_trigger(times, [r['on_wall_m'] for r in trace], stop_distance,
                            DEBOUNCE_TICKS)
    turn_off = first_trigger(times, [r['off_wall_m'] for r in trace], turn_trigger, 1)
    turn_on = first_trigger(times, [r['on_wall_m'] for r in trace], turn_trigger, 1)

    # Acceptance numbers.
    dev = [abs(r['on_wall_m'] - r['lidar_ref_m']) for r in trace
           if math.isfinite(r['on_wall_m']) and math.isfinite(r['lidar_ref_m'])]
    person_as_wall = [r for r in trace if math.isfinite(r['on_wall_m'])
                      and math.isfinite(r['person_m'])
                      and math.isfinite(r['lidar_ref_m'])
                      and r['lidar_ref_m'] - r['person_m'] > 0.5
                      and abs(r['on_wall_m'] - r['person_m']) < 0.3]
    person_as_wall_off = [r for r in trace if math.isfinite(r['off_wall_m'])
                          and math.isfinite(r['person_m'])
                          and math.isfinite(r['lidar_ref_m'])
                          and r['lidar_ref_m'] - r['person_m'] > 0.5
                          and r['off_wall_m'] < r['lidar_ref_m'] - 0.5]
    on_valid = sum(1 for r in trace if math.isfinite(r['on_wall_m']))
    srcs = {}
    for r in trace:
        srcs[r['on_src']] = srcs.get(r['on_src'], 0) + 1
    summary = {
        'run': name, 'scans': len(trace), 'span_s': round(times[-1] - times[0], 2),
        'mission_status': status,
        'on_valid_pct': round(100.0 * on_valid / len(trace), 1),
        'on_sources': srcs,
        'on_vs_lidar_ref_abs_dev_m': {
            'n': len(dev),
            'median': round(statistics.median(dev), 4) if dev else None,
            'p95': round(float(np.percentile(dev, 95)), 4) if dev else None,
            'max': round(max(dev), 4) if dev else None},
        'person_as_wall_ticks_on': len(person_as_wall),
        'wall_short_by_0p5_with_person_ticks_off': len(person_as_wall_off),
        'stop_distance_m': stop_distance, 'wall_turn_trigger_m': round(turn_trigger, 3),
        'stop_trigger_t_off': stop_off, 'stop_trigger_t_on': stop_on,
        'wall_turn_commit_t_off': turn_off, 'wall_turn_commit_t_on': turn_on,
        'scan_cb_ms_off': {'mean': round(statistics.mean(ms_off), 3),
                           'p95': round(float(np.percentile(ms_off, 95)), 3),
                           'max': round(max(ms_off), 3)},
        'scan_cb_ms_on': {'mean': round(statistics.mean(ms_on), 3),
                          'p95': round(float(np.percentile(ms_on, 95)), 3),
                          'max': round(max(ms_on), 3)},
        'scan_rate_hz': round(len(trace) / max(times[-1] - times[0], 1e-6), 1),
    }
    summary['cpu_pct_off'] = round(summary['scan_cb_ms_off']['mean'] * summary['scan_rate_hz']
                                   / 10.0, 2)
    summary['cpu_pct_on'] = round(summary['scan_cb_ms_on']['mean'] * summary['scan_rate_hz']
                                  / 10.0, 2)

    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / f'{name}.csv', 'w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(trace[0]))
        writer.writeheader()
        writer.writerows(trace)

    def f(x):
        return '--' if x is None or (isinstance(x, float) and not math.isfinite(x)) \
            else f'{x:.2f}'
    lines = [f'# {name}', '', '```json', json.dumps(summary, indent=1), '```', '',
             '| t [s] | OFF wall (camera) | OFF LiDAR est. (no cut) | ON virtual wall | ON source '
             '| ON fit | cut sectors | LiDAR ref | person | OFF clearance '
             '| ON clearance (proxy) |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    next_t = times[0]
    for r in trace:
        if r['t'] < next_t:
            continue
        next_t = r['t'] + 0.5
        lines.append('| {} | {} | {} {} | {} | {}{} | {} | {} | {} | {} | {} | {} |'.format(
            f(r['t']), f(r['off_wall_m']), f(r['off_lidar_m']), r['off_lidar_src'],
            f(r['on_wall_m']), r['on_src'], ' DISAGREE' if r['on_disagree'] else '',
            r['on_fit'], r['on_sectors'],
            f(r['lidar_ref_m']) + ('' if r['ref_src'] == 'plain_fit' else ' g'), f(r['person_m']),
            f(r['off_clear_m']), f(r['on_clear_proxy_m'])))
    (out_dir / f'{name}.md').write_text('\n'.join(lines) + '\n')

    # Video: recorded debug JPEG + projected planes + virtual wall, next to top-down.
    video = []
    scans = [(tt, m) for tt, tp, m in rows if tp == '/scan']
    tr_times = np.array([r['t'] + t0 for r in trace])
    if frames:
        sources = [(tt, m) for tt, m in frames]
    else:
        sources = [(tt, None) for tt, _ in scans[::8]]
    # Per video frame, the nearest scan's detection cut and wall lines, redone
    # here with the same library calls the node makes.
    det_by_t = [(tt, m) for tt, tp, m in rows if tp == '/camera/detections_3d']
    det_t = np.array([d[0] for d in det_by_t]) if det_by_t else np.array([])
    laser = on._laser_pose[1] if on._laser_pose else (0.0, 0.0, 0.0)
    model = (ModelLabeller(rows, to_base, model_ceiling_m)
             if model_ceiling_m and frames and cam_tf and k else None)
    for tt, payload in sources:
        i = int(np.clip(np.searchsorted(tr_times, tt), 0, len(trace) - 1))
        r = trace[i]
        scan = scans[min(i, len(scans) - 1)][1]
        sec = []
        if det_by_t:
            j = int(np.searchsorted(det_t, tt)) - 1
            if j >= 0 and tt - det_t[j] <= 0.5:
                tf = to_base(det_by_t[j][1].header.frame_id)
                if tf is not None:
                    objs = []
                    for det in det_by_t[j][1].detections:
                        p = det.bbox.center.position
                        x, y, _ = tf[0] @ np.array([p.x, p.y, p.z]) + tf[1]
                        objs.append((x, y, 0.5 * det.bbox.size.x))
                    sec = scene_labels.detection_sectors(objs, laser[:2], math.radians(
                        get_value('lidar_front_wall_detection_margin_deg')))
        pts, _, _ = scene_labels.scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, laser, math.radians(60.0), sec, stride=2)
        walls = scene_labels.sequential_lines(pts, 0.03, 0.8, 15, 3)
        panel = topdown(scan, laser, sec, walls, r['on_wall_m'], r['person_m'], front_x)
        if payload is not None:
            img = cv2.imdecode(np.frombuffer(payload[1], np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            ih = int(round(img.shape[1] * 360 / 640))
            top = img[:ih]
            scene = {'img_wh': (640, 360),
                     'walls': (project_walls(walls, cam_tf, k) if cam_tf and k else []),
                     'virtual': {'distance_m': r['on_wall_m'] if math.isfinite(r['on_wall_m'])
                                 else None, 'source': r['on_src'],
                                 'disagree': r['on_disagree']}}
            for wl in scene['walls']:
                wl['label'] = '{:.2f}m'.format(wl['d'] - front_x * wl['nx'])
            if model is not None:
                scene['labels'] = model.labels(walls)
            perception_debug.draw_scene_overlay(top, scene)
            perception_debug._text(top, 'OFF camera {} | NO DEPTH IN BAG: {}'.format(
                f(r['off_wall_m']),
                'MODEL ground/ceiling {:.1f}m + LiDAR walls'.format(model_ceiling_m)
                if model is not None else 'LiDAR half only'), (3, ih - 4), (0, 255, 255))
            panel = cv2.resize(panel, (img.shape[0], img.shape[0]))
            frame = np.hstack((img, panel))
        else:
            frame = panel
            perception_debug._text(frame, 'ON {} [{}]  OFF {}'.format(
                f(r['on_wall_m']), r['on_src'], f(r['off_wall_m'])), (3, 12), (0, 255, 0))
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            video.append((tt, buf.tobytes()))
    mp4 = None
    if video:
        mp4 = write_mp4(video, out_dir / f'{name}.scene.mp4')
    summary['mp4'] = str(mp4) if mp4 else None
    summary['video_frames'] = len(video)
    if model is not None:
        model.node.destroy_node()
    summary['model_labels_ceiling_m'] = model_ceiling_m if model is not None else None
    if label_timing:
        summary['label_timing'] = synthetic_label_timing(rows, to_base, planes, det_msg)
    (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    return summary


def synthetic_label_timing(rows, to_base, planes, det_msg, n=60):
    """ms per labelling tick of front_clearance_node on synthetic 640 x 360 depth."""
    from f1tenth_perception.front_clearance_node import FrontClearanceNode
    node = FrontClearanceNode(parameter_overrides=prefixed_params('front_clearance_', True))
    for attr in ('scene_labels_pub', 'scene_debug_pub', 'stereo_wall_pub', 'unclassified_pub'):
        setattr(node, attr, Capture())
    for _, topic, msg in rows:
        if topic == '/tf_static':
            node._tf_static_callback(msg)
    k = zed_intrinsics()
    w, h = 640, 360
    node._camera_k = (*k, w, h)
    frame = 'zed2_left_camera_optical_frame'
    rot, trans = to_base(frame)
    fx, fy, cx, cy = k
    vv, uu = np.mgrid[0:h, 0:w]
    rays = np.stack(((uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, float)), -1) @ rot.T
    # depth Z along the optical axis: base point = trans + Z * ray_base
    z = np.full((h, w), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        zf = (0.0 - trans[2]) / rays[..., 2]           # floor z = 0
        z = np.where((zf > 0) & (zf < z), zf, z)
        zc = (2.6 - trans[2]) / rays[..., 2]           # ceiling 2.6 m
        z = np.where((zc > 0) & (zc < z), zc, z)
        for wl in planes or [scene_labels.WallLine(1.0, 0.0, 4.0, -3.0, 3.0, 50, 0.0)]:
            denom = rays[..., 0] * wl.nx + rays[..., 1] * wl.ny
            zw = (wl.d - (trans[0] * wl.nx + trans[1] * wl.ny)) / denom
            z = np.where((zw > 0) & (zw < z), zw, z)
        # a person: 0.5 m wide vertical patch 1.7 m tall, 1.5 m ahead of the camera
        zp = (1.5 - 0.0) / rays[..., 0]
        py = trans[1] + zp * rays[..., 1]
        pz = trans[2] + zp * rays[..., 2]
        person = (zp > 0) & (np.abs(py) < 0.25) & (pz > 0) & (pz < 1.7) & (zp < z)
        z = np.where(person, zp, z)
    rng = np.random.default_rng(0)
    depth = (z * (1.0 + 0.004 * rng.standard_normal(z.shape))).astype(np.float32)
    depth[~np.isfinite(depth)] = np.nan
    conf = person.astype(np.float32)
    from std_msgs.msg import Header
    header = Header(frame_id=frame)
    if planes:
        node._wall_planes = (0.0, list(planes))
    ms = []
    for i in range(n):
        header.stamp.sec, header.stamp.nanosec = 0, 0
        a = time.perf_counter()
        node._label_scene(depth, header, conf, (75, 215, 140, 500))
        ms.append((time.perf_counter() - a) * 1e3)
    dbg = json.loads(node.scene_debug_pub.items[-1].data)
    labels = node._scene['labels']
    gh, gw = labels.shape
    pm = cv2.resize(person.astype(np.uint8), (gw, gh), interpolation=cv2.INTER_NEAREST) > 0
    node.destroy_node()
    return {'synthetic': True, 'n': n, 'ms_mean': round(statistics.mean(ms[5:]), 2),
            'ms_p95': round(float(np.percentile(ms[5:], 95)), 2),
            'ms_max': round(max(ms[5:]), 2),
            'cpu_pct_at_5hz': round(statistics.mean(ms[5:]) * 5.0 / 10.0, 2),
            'counts': dbg['counts'], 'ground': dbg['ground'], 'ceiling_m': dbg['ceiling_m'],
            'stereo_wall_m': dbg['stereo_wall_m'],
            'person_px_labelled_wall': int(np.count_nonzero(
                labels[pm] == scene_labels.LABEL_WALL)),
            'person_px': int(np.count_nonzero(pm))}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--stop-distance', type=float, default=2.0)
    ap.add_argument('--label-timing', action='store_true')
    ap.add_argument('--model-labels', type=float, metavar='CEILING_M', default=None,
                    help='tint the video with ground / ceiling / wall labels on a MODEL '
                         'depth image (flat floor, ceiling at CEILING_M, LiDAR walls)')
    cfg = ap.parse_args()
    rclpy.init()
    try:
        for run in cfg.runs:
            replay(run, cfg.out, cfg.stop_distance, cfg.label_timing, cfg.model_labels)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
