#!/usr/bin/env python3
"""Offline reproduction of the front-distance sources on a recorded bag, with video.

Usage (after `source install/setup.bash`):
    python3 tools/front_phantom_replay.py [--out DIR] [--map-bag BAG] [--no-video] BAG [BAG ...]

BAG is a bag directory (metadata.yaml + *.db3) or an archive run folder (the
one holding bag/). Bags are opened read-only (sqlite `mode=ro`). Nothing is
published to a graph: the nodes are constructed in-process with their
publishers replaced by capture lists (scene_labels_replay.make_node), and
rclpy runs on the isolated domain scene_labels_replay sets up, so a live
stack on this machine never sees the replay.

WHAT IS REPLAYED, AND FROM WHAT
  lidar_front_wall_node  (/perception/virtual_wall, /perception/walls, d_ray_front)
      Fed the bag's /tf_static, /odometry/filtered, /camera/detections_3d,
      /scan, and its two other inputs: /costmap/front_clearance (the seed)
      and /perception/stereo_wall (the stereo half of the virtual wall, which
      is front_clearance_node's output from ZED depth and cannot be rebuilt
      from LiDAR). Parameters: stack_params.yaml through the launch file's
      own argument list, as scene_labels_replay.prefixed_params builds them.
  costmap_boundary_node  (/costmap/front_clearance)
      Fed /slam/map and the map-frame pose. The pose is
      /ekf_global/odometry/filtered when recorded, else map->odom->base_link
      from /tf. A bag without /slam/map can borrow another bag's maps
      (--map-bag, same wall clock): only ticks after the borrowed map's first
      publish are a reproduction; earlier ones run on a map from the future
      and are reported apart.
  front_clearance_node   (/perception/front_distance) is NOT replayed. It is
      ZED depth plus YOLO masks; a bag without depth cannot rebuild it. Its
      ROI is drawn from the node's parameters and the camera intrinsics, and
      its recorded value is drawn as the depth locus it implies.

THE LIVE NODE DID NOT PROCESS EVERY SCAN. lidar_front_wall_node runs one
scan callback at a time on a best-effort depth-5 /scan queue, and on the car
a callback takes ~100-250 ms, so it publishes for roughly one scan in eight.
Which scans: every /perception/walls header carries its scan's stamp (and
/perception/virtual_wall/debug does, where recorded). The replay feeds
exactly those scans, in that order, with this executor model:
  * callback k starts at C_k = max(receipt of its scan, end of callback k-1)
    and ends at P_k, the receipt time of its /perception/virtual_wall;
  * messages arriving while a callback runs wait in their KEEP_LAST queue
    (/odometry/filtered depth 5, the rest 10) and are taken one per topic
    per executor cycle, in the node's subscription order, before the next
    scan; with no scan waiting the node drains them as they arrive;
  * every callback reads the node clock as C_k.
The per-scan comparison against the recorded values is the check that this
model is close enough.

THE TRUE STRIP RANGE is the nearest /scan return ahead of the bumper inside
|y| < body_half_width + 0.10 m (swept_clearance_body_half_width_m), from the
bumper. It sees only the laser plane (z = 0.19 m).

OUTPUTS per bag in --out: <name>.info.txt (topic counts, rates, gaps,
flags), <name>.scans.csv (one row per replayed scan: recorded vs replay),
<name>.costmap.csv, <name>.summary.json, <name>.summary.md (diffs, phantom
episodes) and <name>.mp4: a 10 Hz real-time top-down view in base_link (car
fixed, pointing up) with each source's ROI, the returns or wall producing
each value, a text panel and a strip chart.
"""

import argparse
import bisect
import collections
import csv
import importlib.util
import json
import math
import statistics
import sqlite3
import sys
from pathlib import Path

import yaml

# Isolation (domain, discovery server, BLAS threads) happens on this import,
# before rclpy loads.
import scene_labels_replay as slr  # noqa: E402  (tools/, sibling module)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from corridor_perception.geometry import Pose2D  # noqa: E402
from f1tenth_costmap.costmap_boundary import nearest_occupied_in_window  # noqa: E402
from f1tenth_costmap.costmap_boundary_node import CostmapBoundaryNode  # noqa: E402
from f1tenth_description.vehicle_frames import base_front_x  # noqa: E402
from f1tenth_logger.mission_debug_video import write_mp4  # noqa: E402
from f1tenth_params.param_defaults import get_value  # noqa: E402
from f1tenth_perception import scene_labels  # noqa: E402
from f1tenth_perception.ground_plane import back_project  # noqa: E402
from f1tenth_perception.lidar_front_wall import REASON_NAMES  # noqa: E402

WS = Path(__file__).resolve().parents[1]
STRIP_MARGIN_M = 0.10
VIDEO_HZ = 10.0
PHANTOM_MARGIN_M = 0.5      # a front value this much short of the true strip range
PHANTOM_MIN_S = 0.3         # ... for at least this long is a phantom episode
STEREO_MATCH_M = 0.10       # a stereo distance this close to a plane's own is that plane's

TOPICS = [
    '/tf_static', '/tf', '/odometry/filtered', '/odom', '/scan', '/camera/detections_3d',
    '/costmap/front_clearance', '/costmap/boundaries', '/perception/stereo_wall',
    '/perception/virtual_wall', '/perception/virtual_wall/debug', '/perception/walls',
    '/perception/wall_planes', '/perception/front_distance', '/perception/debug/front_wall',
    '/drive', '/slam/map', '/ekf_global/odometry/filtered', '/zed2/zed_node/depth/camera_info',
    '/camera/camera_info', '/mission/status',
]
# What this replay needs, and what it compares against.
EXPECTED_INPUTS = ['/scan', '/tf', '/tf_static', '/odometry/filtered', '/camera/detections_3d',
                   '/costmap/front_clearance', '/perception/stereo_wall', '/slam/map',
                   '/ekf_global/odometry/filtered']
EXPECTED_OUTPUTS = ['/perception/virtual_wall', '/perception/virtual_wall/debug',
                    '/perception/walls', '/perception/front_distance',
                    '/perception/debug/front_wall', '/costmap/front_clearance',
                    '/costmap/boundaries', '/perception/wall_planes', '/drive']
# lidar_front_wall_node subscription order (its __init__), and KEEP_LAST depths.
LFW_SUBS = [('/tf_static', 100), ('/odometry/filtered', 5), ('/costmap/front_clearance', 10),
            ('/camera/detections_3d', 10), ('/perception/stereo_wall', 10)]
LFW_FEED = {'/tf_static': '_tf_static_cb', '/odometry/filtered': '_odom_cb',
            '/costmap/front_clearance': '_seed_cb', '/camera/detections_3d': '_detections_cb',
            '/perception/stereo_wall': '_stereo_wall_cb'}


# ============================================================================
# Bag access
# ============================================================================

def bag_db(path):
    path = Path(path).expanduser()
    for pattern in ('bag/*.db3', '*.db3'):
        found = sorted(path.glob(pattern))
        if found:
            return found[0]
    raise FileNotFoundError(f'no .db3 under {path}')


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def stamp_ns(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


class Bag:
    """Read-only view of one sqlite3 bag: per-topic [(receipt s, msg)]."""

    def __init__(self, path, topics=TOPICS):
        self.path = Path(path).expanduser()
        self.db = bag_db(self.path)
        self.name = self.path.name
        con = sqlite3.connect(f'file:{self.db}?mode=ro', uri=True)
        self.types = {n: t for _i, n, t in con.execute('SELECT id, name, type FROM topics')}
        ids = {n: i for i, n, _t in con.execute('SELECT id, name, type FROM topics')}
        self.t0, self.t1 = (x * 1e-9 for x in con.execute(
            'SELECT min(timestamp), max(timestamp) FROM messages').fetchone())
        self.receipts = {}
        for name, tid in ids.items():
            self.receipts[name] = np.array([r[0] for r in con.execute(
                'SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))],
                dtype=np.int64) * 1e-9
        self.msgs = {}
        for name in topics:
            if name not in ids:
                continue
            cls = get_message(self.types[name])
            self.msgs[name] = [(ts * 1e-9, deserialize_message(bytes(d), cls)) for ts, d in
                               con.execute('SELECT timestamp, data FROM messages WHERE topic_id=?'
                                           ' ORDER BY timestamp', (ids[name],))]
        con.close()

    def get(self, topic):
        return self.msgs.get(topic, [])


def topic_report(bag):
    """Counts, rates and gaps per topic, and the flags for what is missing."""
    lines = [f'bag {bag.path}', f'  db3 {bag.db}',
             f'  span {bag.t1 - bag.t0:.2f} s  ({bag.t0:.3f} .. {bag.t1:.3f})', '',
             f'  {"topic":40s} {"count":>6s} {"rate Hz":>8s} {"med dt":>8s} {"max dt":>8s}'
             f' {"at t":>7s} {"gaps":>5s}  (gap = dt > max(3 x median, 0.25 s))']
    stats = {}
    for name in sorted(bag.receipts):
        ts = bag.receipts[name]
        row = {'type': bag.types[name], 'count': int(ts.size)}
        if ts.size >= 2:
            dt = np.diff(ts)
            med = float(np.median(dt))
            gaps = np.nonzero(dt > max(3.0 * med, 0.25))[0]
            row.update(rate_hz=round((ts.size - 1) / (ts[-1] - ts[0]), 2),
                       median_dt_ms=round(med * 1e3, 1), max_dt_ms=round(float(dt.max()) * 1e3, 1),
                       max_dt_at_s=round(float(ts[int(dt.argmax())] - bag.t0), 2),
                       gaps=[[round(float(ts[g] - bag.t0), 2), round(float(dt[g]), 3)]
                             for g in gaps])
            lines.append(f'  {name:40s} {ts.size:6d} {row["rate_hz"]:8.1f} '
                         f'{row["median_dt_ms"]:7.1f}ms {row["max_dt_ms"]:7.1f}ms '
                         f'{row["max_dt_at_s"]:7.2f} {len(gaps):5d}')
        else:
            lines.append(f'  {name:40s} {ts.size:6d}')
        stats[name] = row
    flags = []
    for t in EXPECTED_INPUTS + EXPECTED_OUTPUTS:
        if t not in stats or stats[t]['count'] == 0:
            note = ''
            if t == '/perception/front_distance' and stats.get(
                    '/perception/debug/front_wall', {}).get('count'):
                note = ' (its value is in /perception/debug/front_wall: front_distance_m)'
            flags.append(f'MISSING {t}{note}')
    if '/odom' not in stats:
        flags.append('MISSING /odom (raw wheel odometry; the replay uses /odometry/filtered, '
                     'which every replayed node subscribes to)')
    scan, walls = stats.get('/scan'), stats.get('/perception/walls')
    if scan and walls and scan.get('rate_hz') and walls.get('rate_hz'):
        flags.append(f'lidar_front_wall_node published for {walls["count"]} of '
                     f'{scan["count"]} scans ({walls["rate_hz"]:.1f} Hz vs /scan '
                     f'{scan["rate_hz"]:.1f} Hz): the live node skipped '
                     f'{100.0 * (1 - walls["count"] / scan["count"]):.0f}% of scans')
    for name, row in stats.items():
        if row.get('gaps') and name in EXPECTED_INPUTS + EXPECTED_OUTPUTS:
            worst = max(row['gaps'], key=lambda g: g[1])
            flags.append(f'GAPS {name}: {len(row["gaps"])}, worst {worst[1]:.2f} s at '
                         f't={worst[0]:.2f} s')
    lines += ['', 'flags:'] + [f'  - {f}' for f in flags]
    return '\n'.join(lines), stats, flags


def latency_report(bag):
    """Receipt minus header stamp for the stamped inputs and outputs."""
    out = {}
    for name in ('/scan', '/odometry/filtered', '/camera/detections_3d', '/perception/walls'):
        rows = bag.get(name)
        if rows:
            lag = np.array([t - stamp_s(m.header) for t, m in rows])
            out[name] = {'median_ms': round(float(np.median(lag)) * 1e3, 1),
                         'p95_ms': round(float(np.percentile(lag, 95)) * 1e3, 1),
                         'max_ms': round(float(lag.max()) * 1e3, 1)}
    return out


# ============================================================================
# Poses
# ============================================================================

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomTrack:
    """/odometry/filtered by header stamp and by receipt, with path length."""

    def __init__(self, rows):
        self.stamp = np.array([stamp_s(m.header) for _t, m in rows])
        self.recv = np.array([t for t, _m in rows])
        self.xyt = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                              yaw_of(m.pose.pose.orientation)] for _t, m in rows])
        self.path = np.concatenate([[0.0], np.cumsum(np.hypot(
            np.diff(self.xyt[:, 0]), np.diff(self.xyt[:, 1])))]) if rows else np.zeros(0)

    def at_stamp(self, s):
        """Pose2D interpolated at header stamp s (clamped at the ends)."""
        i = int(np.searchsorted(self.stamp, s))
        if i <= 0:
            return Pose2D(*self.xyt[0])
        if i >= len(self.stamp):
            return Pose2D(*self.xyt[-1])
        t0, t1 = self.stamp[i - 1], self.stamp[i]
        f = (s - t0) / (t1 - t0) if t1 > t0 else 0.0
        a, b = self.xyt[i - 1], self.xyt[i]
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return Pose2D(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dyaw)

    def latest(self, t):
        """(Pose2D, path length) of the last message received at or before t."""
        i = max(int(np.searchsorted(self.recv, t, side='right')) - 1, 0)
        return Pose2D(*self.xyt[i]), float(self.path[i])


def relative(pose_then, pose_now):
    """base_link@then -> base_link@now."""
    return pose_now.inverse().compose(pose_then)


class TfMapPose:
    """map -> base_link from /tf (map->odom composed with odom->base_link)."""

    def __init__(self, tf_rows):
        self.map_odom, self.odom_base = [], []
        for t, msg in tf_rows:
            for tr in msg.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                p = tr.transform.translation
                rec = (t, Pose2D(p.x, p.y, yaw_of(tr.transform.rotation)))
                if key == ('map', 'odom'):
                    self.map_odom.append(rec)
                elif key == ('odom', 'base_link'):
                    self.odom_base.append(rec)
        self._mo_t = [r[0] for r in self.map_odom]
        self._ob_t = [r[0] for r in self.odom_base]

    def latest(self, t):
        i = bisect.bisect_right(self._mo_t, t) - 1
        j = bisect.bisect_right(self._ob_t, t) - 1
        if i < 0 or j < 0:
            return None
        return self.map_odom[i][1].compose(self.odom_base[j][1])


# ============================================================================
# lidar_front_wall_node replay
# ============================================================================

def processed_scans(bag):
    """[(scan msg, scan receipt, end of callback, recorded vwall, debug, walls msg)].

    One per recorded /perception/virtual_wall. Its scan is the stamp in the
    paired /perception/virtual_wall/debug, else the stamp of the
    /perception/walls published in the same callback (by order when the
    counts match, else the last one received after the previous virtual
    wall); None when neither identifies it (the tracker skipped that scan,
    or the scan was received before the bag started).
    """
    scans = {stamp_ns(m.header): (t, m) for t, m in bag.get('/scan')}
    by_10us = {int(round(k * 1e-4)): v for k, v in scans.items()}
    vw = bag.get('/perception/virtual_wall')
    dbg = bag.get('/perception/virtual_wall/debug')
    walls = bag.get('/perception/walls')
    wall_t = [t for t, _m in walls]
    dbg_t = [t for t, _m in dbg]
    out = []
    prev = -math.inf
    # virtual_wall and its debug go out back to back in one callback: with
    # equal counts they pair by order, else by the nearest receipt.
    by_order = len(dbg) == len(vw)
    for n, (t, m) in enumerate(vw):
        d = None
        if by_order:
            d = json.loads(dbg[n][1].data)
        elif dbg:
            i = min(range(max(bisect.bisect_left(dbg_t, t) - 1, 0),
                          min(bisect.bisect_left(dbg_t, t) + 1, len(dbg))),
                    key=lambda i: abs(dbg_t[i] - t))
            if abs(dbg_t[i] - t) < 0.05:
                d = json.loads(dbg[i][1].data)
        if len(walls) == len(vw):
            # One walls message per callback (the recorder can receive a
            # topic in a late burst, so receipt order is not the evidence).
            w = walls[n][1]
        else:
            j = bisect.bisect_right(wall_t, t) - 1
            w = walls[j][1] if j >= 0 and wall_t[j] > prev else None
        scan = None
        if d is not None and d.get('stamp') is not None:
            us = int(round(d['stamp'] * 1e5))
            scan = next((by_10us[k] for k in (us, us - 1, us + 1) if k in by_10us), None)
        elif w is not None:
            scan = scans.get(stamp_ns(w.header))
        out.append({'scan': scan[1] if scan else None, 'scan_recv': scan[0] if scan else None,
                    'end': t, 'rec_vw': float(m.data), 'rec_dbg': d, 'rec_walls': w})
        prev = t
    return out


class LidarReplay:
    """In-process lidar_front_wall_node, driven by the executor model above."""

    def __init__(self, bag):
        self.bag = bag
        self.clock = [0.0]
        self.node, self.caps = slr.make_node(True, self.clock)
        for attr in ('walls_pub', 'walls_markers_pub'):
            setattr(self.node, attr, slr.Capture())
        self.order = [t for t, _ in LFW_SUBS]
        self.depth = dict(LFW_SUBS)
        # /tf_static is transient-local and the live node had it from its
        # start, long before any bag began: give it all up front.
        for t, m in bag.get('/tf_static'):
            self.clock[0] = t
            self.node._tf_static_cb(m)
        arrivals = []
        for topic in self.order[1:]:
            for k, (t, m) in enumerate(bag.get(topic)):
                arrivals.append((t, self.order.index(topic), k, topic, m))
        arrivals.sort(key=lambda a: (a[0], a[1], a[2]))
        self.arrivals = arrivals
        self.ai = 0
        self.queues = {t: collections.deque() for t in self.order}

    def _arrive_until(self, t):
        while self.ai < len(self.arrivals) and self.arrivals[self.ai][0] <= t:
            _r, _o, _k, topic, msg = self.arrivals[self.ai]
            q = self.queues[topic]
            q.append(msg)
            while len(q) > self.depth[topic]:
                q.popleft()
            self.ai += 1

    def _cycle(self, now):
        """One executor cycle: one message per ready subscription, in order."""
        took = False
        for topic in self.order:
            if self.queues[topic]:
                self.clock[0] = now
                getattr(self.node, LFW_FEED[topic])(self.queues[topic].popleft())
                took = True
        return took

    def _idle_until(self, t_from, t_to):
        """Node idle from t_from: drain the backlog, then take arrivals as they come."""
        self._arrive_until(t_from)
        while self._cycle(t_from):
            pass
        while self.ai < len(self.arrivals) and self.arrivals[self.ai][0] <= t_to:
            t = self.arrivals[self.ai][0]
            self._arrive_until(t)
            while self._cycle(t):
                pass

    def run(self, callbacks):
        node = self.node
        rows = []
        free = -math.inf
        for k, cb in enumerate(callbacks):
            scan = cb['scan']
            if scan is None:
                rows.append(None)
                free = cb['end']
                continue
            start = max(cb['scan_recv'], free) if math.isfinite(free) else cb['scan_recv']
            if math.isfinite(free) and cb['scan_recv'] <= free:
                self._arrive_until(free)
                self._cycle(free)             # one cycle, then the waiting scan
            else:
                self._idle_until(free if math.isfinite(free) else start, start)
            self.clock[0] = start
            laser = node._laser_pose_for(scan.header.frame_id)
            sectors = node._sectors(start, laser, stamp_s(scan.header)) if laser else []
            prev_anchor = node.estimator._anchor
            node._scan_cb(scan)
            rows.append(self._collect(cb, start, scan, laser, sectors, prev_anchor))
            free = cb['end']
        return rows

    def _collect(self, cb, start, scan, laser, sectors, prev_anchor):
        node, caps = self.node, self.caps
        vw = caps['virtual_pub'].pop()
        dbg = caps['virtual_debug_pub'].pop()
        est = caps['estimate_pub'].pop()
        fit = caps['fit_pub'].pop()
        walls = node.walls_pub.pop()
        node.walls_markers_pub.pop()
        planes = caps['planes_pub'].pop()
        d = json.loads(dbg[-1].data)
        pts, _u, _e, bearings = scene_labels.scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            laser, node.planes_half_angle_rad, sectors, with_bearings=True)
        anchor = node.estimator._anchor
        epose = node.estimator._pose
        return {
            'stereo_used': node.virtual._stereo,
            'est_pose': Pose2D(epose.x, epose.y, epose.yaw) if epose is not None else None,
            'start': start, 'end': cb['end'], 'scan_stamp': stamp_s(scan.header), 'scan': scan,
            'scan_recv': cb['scan_recv'], 'laser': laser, 'sectors': sectors,
            'points': pts, 'bearings': bearings,
            'vw': float(vw[-1].data), 'dbg': d,
            'estimate': est[-1], 'fit': fit[-1],
            'walls': walls[-1] if walls else None,
            'planes': json.loads(planes[-1].data) if planes else None,
            'tracked_wall': node._tracked_wall, 'seq_wall': node._seq_wall,
            'wall_source': node._wall_source, 'anchor': anchor,
            'anchor_new': anchor is not prev_anchor,
            'rec_vw': cb['rec_vw'], 'rec_dbg': cb['rec_dbg'], 'rec_walls': cb['rec_walls'],
        }

    def close(self):
        self.node.destroy_node()


def front_ray(walls_msg):
    """Smallest finite d_ray_front among front tracks of a MultiWallTrackArray, or NaN."""
    if walls_msg is None:
        return math.nan
    vals = [t.d_ray_front for t in walls_msg.tracks
            if t.label == 'front' and math.isfinite(t.d_ray_front)]
    return min(vals) if vals else math.nan


def match_tracks(rec, rep):
    """Pair recorded and replayed tracks by label and nearest (rho_bl, theta_bl).

    Returns ([(rec track, rep track or None)], unmatched replay count).
    Track ids differ (the replay starts from an empty tracker).
    """
    pairs, used = [], set()
    for a in (rec.tracks if rec else []):
        best, cost = None, math.inf
        for j, b in enumerate(rep.tracks if rep else []):
            if j in used or b.label != a.label:
                continue
            dth = math.atan2(math.sin(a.theta_bl - b.theta_bl), math.cos(a.theta_bl - b.theta_bl))
            c = abs(a.rho_bl - b.rho_bl) + abs(dth)
            if c < cost:
                best, cost = j, c
        if best is not None and cost < 0.5:
            used.add(best)
            pairs.append((a, rep.tracks[best]))
        else:
            pairs.append((a, None))
    return pairs, (len(rep.tracks) if rep else 0) - len(used)


# ============================================================================
# costmap_boundary_node replay
# ============================================================================

def launch_defaults(path):
    """{argument name: default value} of a launch file's DeclareLaunchArguments."""
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('replay_launch_' + Path(path).stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    out = {}
    for ent in mod.generate_launch_description().entities:
        if isinstance(ent, DeclareLaunchArgument) and ent.default_value:
            text = ''.join(getattr(s, 'text', '') for s in ent.default_value)
            out[ent.name] = yaml.safe_load(text)
    return out, mod


def costmap_params():
    """costmap_boundary_node's parameters as costmap.launch.py passes them."""
    src = WS / 'src/f1tenth_costmap/launch/costmap.launch.py'
    d, _ = launch_defaults(src)
    keys = {'front_facing_max_deg': float, 'side_window_min_deg': float,
            'side_window_max_deg': float, 'occupied_threshold': int, 'max_range_m': float,
            'extraction_rate_hz': float, 'use_convex_polytope': bool,
            'polytope_r_local_m': float, 'polytope_max_faces': int}
    return {k: typ(d['costmap_boundary_' + k]) for k, typ in keys.items()}


class CostmapReplay:
    """In-process costmap_boundary_node; one _extraction_tick per recorded publish."""

    def __init__(self):
        self.params = costmap_params()
        self.node = CostmapBoundaryNode(parameter_overrides=[
            Parameter(k, value=v) for k, v in self.params.items()])
        self.node.clearance_pub = slr.Capture()
        self.node.boundary_pub = slr.Capture()

    def tick(self, grid, pose_xyyaw):
        self.node._latest_grid = grid
        self.node._latest_pose_xy_yaw = pose_xyyaw
        self.node._extraction_tick()
        c = self.node.clearance_pub.pop()
        b = self.node.boundary_pub.pop()
        return (float(c[-1].data) if c else math.nan), (b[-1] if b else None)

    def close(self):
        self.node.destroy_node()


def occupied_cells(grid, threshold):
    """(N, 2) map-frame centres of the grid's occupied cells."""
    info = grid.info
    arr = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    r, c = np.nonzero(arr >= threshold)
    return np.column_stack([info.origin.position.x + (c + 0.5) * info.resolution,
                            info.origin.position.y + (r + 0.5) * info.resolution])


def replay_costmap(bag, map_bag, tfpose):
    """Per recorded /costmap/front_clearance: (t, recorded, replay, cell, pose, map tag)."""
    recorded = bag.get('/costmap/front_clearance')
    if not recorded:
        return [], None
    maps = bag.get('/slam/map') or (map_bag.get('/slam/map') if map_bag else [])
    map_src = 'own' if bag.get('/slam/map') else ('borrowed' if maps else None)
    if not maps:
        return [], None
    ekf = bag.get('/ekf_global/odometry/filtered')
    ekf_t = [t for t, _m in ekf]
    map_t = [t for t, _m in maps]
    # A latched map arrives at record start; slam_toolbox republishes every
    # map_update_interval. The first recorded map was in force from one
    # interval before the second one, at the latest.
    first_valid = maps[0][0]
    if len(maps) >= 2:
        period = (statistics.median(np.diff(map_t[1:])) if len(maps) >= 3
                  else maps[1][0] - maps[0][0])
        first_valid = min(maps[0][0], maps[1][0] - period)
    rep = CostmapReplay()
    cells_cache = {}
    out = []
    for t, msg in recorded:
        i = bisect.bisect_right(map_t, t) - 1
        exact = i >= 0 or t >= first_valid
        grid = maps[max(i, 0)][1]
        if ekf:
            j = bisect.bisect_right(ekf_t, t) - 1
            if j < 0:
                continue
            p = ekf[j][1].pose.pose
            pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        else:
            pp = tfpose.latest(t)
            if pp is None:
                continue
            pose = (pp.x, pp.y, pp.theta)
        val, boundaries = rep.tick(grid, pose)
        # The cell behind the published value: the node's own front_clearance
        # search (_extraction_tick), same function, same arguments.
        n = rep.node
        info = grid.info
        hit = nearest_occupied_in_window(
            grid.data, info.width, info.height, info.resolution, info.origin.position.x,
            info.origin.position.y, pose[0], pose[1], pose[2], -n.front_facing_max_rad,
            n.front_facing_max_rad, n.occupied_threshold, n.max_range_m,
            exclude_beside_body=(base_front_x(), n.front_side_limit_m))
        key = id(grid)
        if key not in cells_cache:
            cells_cache[key] = occupied_cells(grid, rep.params['occupied_threshold'])
        out.append({'t': t, 'rec': float(msg.data), 'rep': val, 'boundaries': boundaries,
                    'cell': (hit[0], hit[1]) if hit else None,
                    'pose': pose, 'grid_key': key, 'exact_map': bool(exact),
                    'map_src': map_src})
    rep.close()
    return out, cells_cache


# ============================================================================
# front_clearance_node replay (bags with depth only)
# ============================================================================

DEPTH_TOPICS = ['/zed2/zed_node/depth/depth_registered',
                '/zed2/zed_node/depth/depth_registered_5hz']
CAMERA_TOPICS = DEPTH_TOPICS + ['/camera/detections', '/camera/detection_masks',
                                '/perception/obstacles_2d', '/perception/scene_labels']


def front_clearance_params():
    """front_clearance_node's parameters as detection.launch.py passes them.

    The name/type list is the launch file's own _front_clearance_args, read
    with ast (executing that launch file would start resolving the camera
    stack); values from stack_params.yaml, plus the LiDAR housing edges and
    the scene-labels switch the launch adds.
    """
    import ast
    src = (WS / 'src/f1tenth_perception/launch/detection.launch.py').read_text()
    types = {'int': int, 'float': float, 'bool': bool, 'str': str}
    names = None
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and getattr(node.targets[0], 'id', None) == '_front_clearance_args'):
            names = [(e.elts[0].value, types[e.elts[1].id]) for e in node.value.elts]
    if names is None:
        raise RuntimeError('_front_clearance_args not found in detection.launch.py')
    params = [Parameter(n, value=typ(get_value('front_clearance_' + n))) for n, typ in names]
    for k in ('x_min', 'x_max', 'y_min', 'y_max'):
        params.append(Parameter('lidar_exclusion_' + k,
                                value=float(get_value('lidar_exclusion_' + k))))
    params.append(Parameter('enable_scene_labels', value=bool(get_value('enable_scene_labels'))))
    return params


class CameraReplay:
    """In-process front_clearance_node on the bag's depth frames.

    The scene labels are not recomputed: the recorded /perception/scene_labels
    image the live node had for each frame (the latest by stamp at or before
    it) is put in place and the node's own labelling is switched off, so the
    WALL / OTHER pixel selection is the live one. Object masks, detections
    and obstacles are fed as recorded. A subsampled depth topic (the
    archive's *_5hz) makes the EMA'd front_distance differ from the live one;
    the per-frame raw distance is what is compared.
    """

    def __init__(self, bag, depth_topic):
        from cv_bridge import CvBridge
        from f1tenth_perception.front_clearance_node import FrontClearanceNode
        self.bag = bag
        self.topic = depth_topic
        self.bridge = CvBridge()
        self.clock = [0.0]
        node = FrontClearanceNode(parameter_overrides=front_clearance_params())
        node._now_seconds = lambda: self.clock[0]
        for attr in ('front_distance_pub', 'front_wall_pub', 'front_clearance_pub',
                     'front_blocked_pub', 'wall_debug_pub', 'scene_labels_pub', 'scene_debug_pub',
                     'stereo_wall_pub', 'unclassified_pub', 'front_distance_raw_pub',
                     'bg_pixel_count_pub'):
            if getattr(node, attr, None) is not None:
                setattr(node, attr, slr.Capture())
        node._next_scene = math.inf
        self.node = node

    def run(self):
        bag, node = self.bag, self.node
        for t, m in bag.get('/tf_static'):
            self.clock[0] = t
            node._tf_static_callback(m)
        for topic in ('/zed2/zed_node/depth/camera_info', '/camera/camera_info'):
            if bag.get(topic):
                node._camera_info_callback(bag.get(topic)[0][1])
                break
        labels = [(stamp_s(m.header), self.bridge.imgmsg_to_cv2(m, desired_encoding='mono8'), t)
                  for t, m in bag.get('/perception/scene_labels')]
        labels.sort(key=lambda x: x[0])
        label_stamps = [lb[0] for lb in labels]
        feed = {'/camera/detections': node._detections_callback,
                '/camera/detection_masks': node._masks_callback,
                '/perception/obstacles_2d': node._obstacles_callback}
        events = [(t, 0, topic, m) for topic in feed for t, m in bag.get(topic)]
        events += [(t, 1, self.topic, m) for t, m in bag.get(self.topic)]
        events.sort(key=lambda e: (e[0], e[1]))
        rec_dbg = {round(json.loads(m.data)['stamp'], 6): json.loads(m.data)
                   for _t, m in bag.get('/perception/debug/front_wall')}
        planes, _src = plane_sets({'bag': bag, 'rows': []})
        out = []
        for t, _k, topic, m in events:
            self.clock[0] = t
            if topic != self.topic:
                feed[topic](m)
                continue
            s = stamp_s(m.header)
            i = bisect.bisect_right(label_stamps, s + 1e-6) - 1
            if i >= 0:
                node._scene = {'stamp': labels[i][0], 'labels': labels[i][1]}
            node.depth_callback(m)
            dbg = node.wall_debug_pub.pop()
            for attr in ('front_distance_pub', 'front_wall_pub', 'front_clearance_pub',
                         'front_blocked_pub'):
                getattr(node, attr).pop()
            d = json.loads(dbg[-1].data) if dbg else {}
            band = self._band(m, s, planes, labels[i] if i >= 0 else None)
            rec = rec_dbg.get(round(s, 6))
            out.append({'t': t, 'stamp': s, 'raw_rep': d.get('stereo_raw_m'),
                        'n_bg_rep': d.get('px', {}).get('bg'),
                        'fd_rep': d.get('front_distance_m'),
                        'raw_rec': rec.get('stereo_raw_m') if rec else None,
                        'n_bg_rec': rec.get('px', {}).get('bg') if rec else None,
                        'fd_rec': rec.get('front_distance_m') if rec else None, **band})
        return out

    def _band(self, msg, stamp, planes, label):
        """The ROI pixels inside the percentile band, back-projected to base_link.

        Steps 5-6b of depth_callback with the node's own helpers and state
        (object confidence, housing, scene validity or the floor cut), then
        the band of _robust_background_distance.
        """
        node = self.node
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        h, w = depth.shape[:2]
        cy, cx = h // 2 - node.roi_shift_up, w // 2
        y0, y1 = max(0, cy - node.roi_half_h), min(h, cy + node.roi_half_h)
        x0, x1 = max(0, cx - node.roi_half_w), min(w, cx + node.roi_half_w)
        bounds = (y0, y1, x0, x1)
        roi = depth[y0:y1, x0:x1]
        housing = node._lidar_exclusion_roi_mask((h, w), bounds)
        valid = np.isfinite(roi) & (roi > 0)
        conf = node.object_conf_mask
        if conf is not None:
            valid &= (1.0 - conf[y0:y1, x0:x1]) >= node.bg_weight_threshold
        valid &= ~housing
        scene_valid = node._scene_roi_valid(stamp, bounds, (h, w)) if node.scene_enabled else None
        if scene_valid is not None:
            valid &= scene_valid
        else:
            ground = node._ground_roi_mask(depth, msg.header.frame_id, bounds)
            if ground is not None:
                valid &= ~ground
        rr, cc = np.nonzero(valid)
        z = roi[rr, cc]
        empty = {'band_xy': np.zeros((0, 2)), 'band_kind': np.zeros(0, dtype='<U6'),
                 'n_valid_check': int(z.size)}
        if z.size < node.min_bg_pixels:
            return empty
        lo, hi = np.percentile(z, node.bg_pct_low), np.percentile(z, node.bg_pct_high)
        sel = (z >= lo) & (z <= hi)
        rr, cc = rr[sel] + y0, cc[sel] + x0
        k = node._intrinsics(w, h)
        tf = node._frame_transform(msg.header.frame_id)
        if k is None or tf is None:
            return empty
        pts = back_project(depth, rr, cc, *k, *tf)
        kind = np.full(len(pts), 'other', dtype='<U6')
        if label is not None:
            lab = label[1]
            gh, gw = lab.shape
            is_wall = lab[rr * gh // h, cc * gw // w] == scene_labels.LABEL_WALL
            walls = []
            for pt, pstamp, ws in reversed(planes):
                if pt <= label[2] and abs(label[0] - pstamp) <= float(
                        get_value('front_clearance_scene_wall_max_age_s')):
                    walls = ws
                    break
            best = np.full(len(pts), np.inf)
            for wl in walls:
                off = np.abs(wl.offset(pts[:, 0], pts[:, 1]))
                hit = is_wall & (off < best) & (off <= 0.10)
                best[hit] = off[hit]
                kind[hit] = wall_kind(wl)
            kind[is_wall & ~np.isfinite(best)] = 'wall?'
        return {'band_xy': pts[:, :2], 'band_kind': kind, 'n_valid_check': int(z.size),
                'band_z_mean': float(np.mean(z[sel]))}

    def close(self):
        self.node.destroy_node()


# ============================================================================
# Geometry for the view and the summary
# ============================================================================

def scan_xy(scan, laser):
    r = np.asarray(scan.ranges, dtype=np.float64)
    ang = scan.angle_min + np.arange(r.size) * scan.angle_increment + laser[2]
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    return np.column_stack([laser[0] + r[ok] * np.cos(ang[ok]),
                            laser[1] + r[ok] * np.sin(ang[ok])])


def strip_nearest(pts, front_x, half):
    """(range from the bumper, point) of the nearest return in the strip, or (NaN, None)."""
    if pts is None or len(pts) == 0:
        return math.nan, None
    sel = (pts[:, 0] > front_x) & (np.abs(pts[:, 1]) < half)
    if not sel.any():
        return math.nan, None
    i = int(np.argmin(np.where(sel, pts[:, 0], np.inf)))
    return float(pts[i, 0] - front_x), pts[i]


def stereo_plane(planes, stereo_m, front_x):
    """Index and residual of the wall plane whose bumper offset matches the stereo value.

    front_clearance_node measures the stereo wall as the median of the WALL
    pixels' offsets along ONE plane's normal minus front_x * nx
    (scene_labels.stereo_wall_distance); WALL pixels lie within 6 cm of it.
    So that plane's own d - front_x * nx is within a few cm of the value.
    """
    best, res = None, math.inf
    for k, w in enumerate(planes or []):
        r = abs((w.d - front_x * w.nx) - stereo_m)
        if r < res:
            best, res = k, r
    return best, res


def camera_roi(bag, to_base):
    """Camera ROI wedge of front_clearance_node from its parameters and the intrinsics.

    Returns dict(origin (x, y), left/right bearings [rad], floor_x: x where
    the ROI's bottom row meets the floor, source of K) or None.
    """
    k = None
    src = None
    for topic in ('/zed2/zed_node/depth/camera_info', '/camera/camera_info'):
        rows = bag.get(topic)
        if rows:
            ci = rows[0][1]
            k = (ci.k[0], ci.k[4], ci.k[2], ci.k[5], ci.width, ci.height)
            src = topic
            break
    if k is None:
        z = slr.zed_intrinsics()
        if z is None:
            return None
        k = (*z, 640, 360)
        src = 'ZED factory calibration (/usr/local/zed/settings), HD720/2'
    fx, fy, cx, cy, w, h = k
    half_w = int(get_value('front_clearance_roi_half_width_px'))
    half_h = int(get_value('front_clearance_roi_half_height_px'))
    shift = int(get_value('front_clearance_roi_shift_up_px'))
    rcy, rcx = h // 2 - shift, w // 2
    y0, y1 = max(0, rcy - half_h), min(h, rcy + half_h)
    x0, x1 = max(0, rcx - half_w), min(w, rcx + half_w)
    tf = to_base('zed2_left_camera_optical_frame')
    if tf is None:
        return None
    rot, trans = tf

    def ray(u, v):
        d = rot @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        return d
    left, right = ray(x0, 0.5 * (y0 + y1)), ray(x1, 0.5 * (y0 + y1))
    bottom = ray(rcx, y1)
    floor_x = (trans[0] + bottom[0] * (-trans[2] / bottom[2])) if bottom[2] < 0 else math.inf
    return {'origin': (float(trans[0]), float(trans[1])), 'z': float(trans[2]),
            'left': math.atan2(left[1], left[0]), 'right': math.atan2(right[1], right[0]),
            'floor_x': float(floor_x), 'roi_px': [x0, y0, x1, y1], 'k_src': src,
            'axis': (float(rot[0, 2]), float(rot[1, 2]))}


# ============================================================================
# Per-bag run
# ============================================================================

def num(x, nd=3):
    return None if x is None or not math.isfinite(x) else round(float(x), nd)


def diff_stats(pairs):
    d = [abs(a - b) for a, b in pairs if math.isfinite(a) and math.isfinite(b)]
    if not d:
        return {'n': 0}
    return {'n': len(d), 'median': round(statistics.median(d), 4),
            'p95': round(float(np.percentile(d, 95)), 4), 'max': round(max(d), 4)}


def episodes(times, flags, min_len):
    """[(t_start, t_end)] of runs of True lasting at least min_len."""
    out, start = [], None
    for t, f in zip(times, flags):
        if f and start is None:
            start = t
        elif not f and start is not None:
            if t - start >= min_len:
                out.append((start, t))
            start = None
    if start is not None and times and times[-1] - start >= min_len:
        out.append((start, times[-1]))
    return out


def camera_summary(cam, depth_topic):
    if not cam:
        return ('not replayed: no depth image in this bag, and front_clearance_node is ZED '
                'depth plus YOLO masks. Recorded values only; the ROI is drawn from the '
                "node's parameters and the depth locus of the recorded value.")
    both = [c for c in cam if c['raw_rec'] is not None and c['raw_rep'] is not None]
    with_raw = [c for c in cam if c['raw_rep'] is not None]
    band_ok = sum(1 for c in with_raw if c.get('band_z_mean') is not None
                  and abs(c['band_z_mean'] - c['raw_rep']) < 1e-3)
    return {
        'replayed_on': depth_topic, 'frames': len(cam),
        'frames_with_recorded_debug_same_stamp': len(both),
        'raw_distance_abs_diff_m': diff_stats([(c['raw_rec'], c['raw_rep']) for c in both]),
        'bg_pixel_count_equal': f"{sum(1 for c in both if c['n_bg_rec'] == c['n_bg_rep'])}"
                                f'/{len(both)}',
        'band_recomputation_matches_node': f'{band_ok}/{len(with_raw)} (a miss is a frame '
                                           'with stale labels, where the floor cut is a '
                                           'random RANSAC draw)',
        'note': 'front_distance itself is EMA-smoothed per frame; on a subsampled depth topic '
                'only the per-frame raw distance is comparable',
    }


def run_bag(path, out_dir, map_bag, video):
    bag = Bag(path, topics=TOPICS + CAMERA_TOPICS)
    name = bag.name if bag.name != 'bag' else bag.path.parent.name
    if bag.path.name != 'bag' and (bag.path / 'bag').is_dir():
        name = bag.path.name
    out_dir.mkdir(parents=True, exist_ok=True)
    info, stats, flags = topic_report(bag)
    lat = latency_report(bag)
    info += '\n\nreceipt - header stamp:\n' + '\n'.join(
        f'  {k:28s} median {v["median_ms"]:7.1f} ms  p95 {v["p95_ms"]:7.1f}  max {v["max_ms"]:7.1f}'
        for k, v in lat.items())
    (out_dir / f'{name}.info.txt').write_text(info + '\n')
    print(info)

    front_x = base_front_x()
    half = float(get_value('swept_clearance_body_half_width_m')) + STRIP_MARGIN_M
    to_base = slr.static_tf([m for _t, m in bag.get('/tf_static')])
    odom = OdomTrack(bag.get('/odometry/filtered'))
    tfpose = TfMapPose(bag.get('/tf'))

    # ---- lidar_front_wall_node ---------------------------------------------
    cbs = processed_scans(bag)
    lid = LidarReplay(bag)
    rows = lid.run(cbs)
    lid.close()
    t0 = bag.t0

    # ---- costmap_boundary_node ---------------------------------------------
    cm, cells = replay_costmap(bag, map_bag, tfpose)

    # ---- front_clearance_node (only with depth in the bag) -----------------
    depth_topic = next((t for t in DEPTH_TOPICS if bag.get(t)), None)
    cam = []
    if depth_topic:
        camr = CameraReplay(bag, depth_topic)
        cam = camr.run()
        camr.close()

    # ---- per-scan comparison -----------------------------------------------
    stereo = [(t, json.loads(m.data)) for t, m in bag.get('/perception/stereo_wall')]
    scan_rows = []
    vw_pairs, vw_pairs_warm, ray_pairs, perp_pairs = [], [], [], []
    track_counts = collections.Counter()
    gaps_model = []
    for r in rows:
        if r is None:
            track_counts['callback_without_scan'] += 1
            continue
        gaps_model.append(r['start'] - r['scan_recv'])
        rec_ray = front_ray(r['rec_walls'])
        rep_ray = front_ray(r['walls'])
        pairs, extra = match_tracks(r['rec_walls'], r['walls'])
        for a, b in pairs:
            if b is None:
                track_counts['recorded_unmatched'] += 1
            else:
                track_counts['matched'] += 1
                perp_pairs.append((a.d_perp_front, b.d_perp_front))
        track_counts['replay_unmatched'] += extra
        rec_vw = r['rec_vw'] if r['rec_vw'] > 0 else math.nan
        rep_vw = r['vw'] if r['vw'] > 0 else math.nan
        both_invalid = not math.isfinite(rec_vw) and not math.isfinite(rep_vw)
        vw_pairs.append((rec_vw, rep_vw))
        if r['end'] - t0 > 2.0:
            vw_pairs_warm.append((rec_vw, rep_vw))
        ray_pairs.append((rec_ray, rep_ray))
        pts_now = r['points']
        strip, _p = strip_nearest(scan_xy(r['scan'], r['laser']), front_x, half)
        rec_src = r['rec_dbg']['source'] if r['rec_dbg'] else None
        scan_rows.append({
            't': round(r['end'] - t0, 3), 'scan_t': round(r['scan_stamp'] - t0, 3),
            'cb_start_t': round(r['start'] - t0, 3),
            'scan_age_at_publish_ms': round((r['end'] - r['scan_stamp']) * 1e3, 1),
            'rec_virtual_wall': r['rec_vw'], 'rep_virtual_wall': round(r['vw'], 4),
            'vw_abs_diff': (round(abs(rec_vw - rep_vw), 4) if math.isfinite(rec_vw)
                            and math.isfinite(rep_vw) else ('both_invalid' if both_invalid
                                                            else 'validity_differs')),
            'rec_source': rec_src, 'rep_source': r['dbg']['source'],
            'rep_lidar_m': r['dbg']['lidar_m'], 'rep_stereo_m': r['dbg']['stereo_m'],
            'rep_provenance': r['dbg']['lidar_provenance'],
            'rep_lidar_reason': REASON_NAMES.get(r['dbg']['lidar_reason']),
            'rep_wall_source': r['dbg']['wall_source'],
            'rec_d_ray_front': num(rec_ray, 4), 'rep_d_ray_front': num(rep_ray, 4),
            'rec_tracks': ' '.join(f'{x.id}:{x.label}:{x.d_perp_front:.2f}'
                                   for x in (r['rec_walls'].tracks if r['rec_walls'] else [])),
            'rep_tracks': ' '.join(f'{x.id}:{x.label}:{x.d_perp_front:.2f}'
                                   for x in (r['walls'].tracks if r['walls'] else [])),
            'true_strip_m': num(strip, 4), 'points_in_sector': int(len(pts_now)),
            'cut_sectors': len(r['sectors']),
        })
    if scan_rows:
        with open(out_dir / f'{name}.scans.csv', 'w', newline='') as fh:
            wr = csv.DictWriter(fh, fieldnames=list(scan_rows[0]))
            wr.writeheader()
            wr.writerows(scan_rows)
    validity = collections.Counter()
    for a, b in vw_pairs:
        validity[(math.isfinite(a), math.isfinite(b))] += 1
    src_agree = sum(1 for s in scan_rows if s['rec_source'] is not None
                    and s['rec_source'] == s['rep_source'])
    src_known = sum(1 for s in scan_rows if s['rec_source'] is not None)

    # The winning map cell against the live LiDAR: how far it is from the
    # nearest return of the latest scan (map / map-pose misalignment).
    laser_tf = to_base('laser')
    laser = (float(laser_tf[1][0]), float(laser_tf[1][1]),
             math.atan2(laser_tf[0][1, 0], laser_tf[0][0, 0]))
    scans = bag.get('/scan')
    scan_t = [t for t, _ in scans]
    cm_rows = []
    for c in cm:
        cell = c['cell']
        gap = math.nan
        i = bisect.bisect_right(scan_t, c['t']) - 1
        if cell is not None and i >= 0:
            pts = scan_xy(scans[i][1], laser)
            if len(pts):
                gap = float(np.min(np.hypot(pts[:, 0] - cell[0], pts[:, 1] - cell[1])))
        c['cell_to_scan'] = gap
        cm_rows.append({'t': round(c['t'] - t0, 3), 'rec': round(c['rec'], 4),
                        'rep': round(c['rep'], 4), 'abs_diff': round(abs(c['rec'] - c['rep']), 4),
                        'map': c['map_src'] + ('' if c['exact_map'] else '_from_later'),
                        'cell_x': num(cell[0]) if cell else None,
                        'cell_y': num(cell[1]) if cell else None,
                        'cell_to_nearest_scan_return_m': num(gap)})
    if cm_rows:
        with open(out_dir / f'{name}.costmap.csv', 'w', newline='') as fh:
            wr = csv.DictWriter(fh, fieldnames=list(cm_rows[0]))
            wr.writeheader()
            wr.writerows(cm_rows)

    summary = {
        'bag': str(bag.path), 'name': name, 'span_s': round(bag.t1 - bag.t0, 2),
        'flags': flags, 'latency': lat,
        'lidar_front_wall': {
            'callbacks_recorded': len(cbs),
            'callbacks_replayed': len(scan_rows),
            'callbacks_without_identifiable_scan': track_counts['callback_without_scan'],
            'callback_start_minus_scan_receipt_ms': {
                'median': round(statistics.median(gaps_model) * 1e3, 1) if gaps_model else None,
                'p95': round(float(np.percentile(gaps_model, 95)) * 1e3, 1)
                if gaps_model else None},
            'virtual_wall_abs_diff_m': diff_stats(vw_pairs),
            'virtual_wall_abs_diff_m_after_2s': diff_stats(vw_pairs_warm),
            'virtual_wall_validity': {f'rec_{"valid" if a else "-1"}/rep_{"valid" if b else "-1"}':
                                      n for (a, b), n in sorted(validity.items())},
            'virtual_wall_source_agreement': (f'{src_agree}/{src_known}' if src_known
                                              else 'no recorded debug'),
            'tracks_d_perp_front_abs_diff_m': diff_stats(perp_pairs),
            'track_matching': dict(track_counts),
            'd_ray_front_finite': {'recorded': sum(1 for a, _ in ray_pairs if math.isfinite(a)),
                                   'replay': sum(1 for _, b in ray_pairs if math.isfinite(b)),
                                   'of': len(ray_pairs)},
            'd_ray_front_abs_diff_m': diff_stats(ray_pairs),
        },
        'costmap_front_clearance': {
            'ticks': len(cm_rows),
            'map': (cm[0]['map_src'] if cm else 'no /slam/map in this bag and no --map-bag'),
            'pose': ('/ekf_global/odometry/filtered' if bag.get('/ekf_global/odometry/filtered')
                     else 'map->odom->base_link from /tf'),
            'abs_diff_m_exact_map': diff_stats([(c['rec'], c['rep']) for c in cm
                                                if c['exact_map']]),
            'abs_diff_m_map_from_later': diff_stats([(c['rec'], c['rep']) for c in cm
                                                     if not c['exact_map']]),
            'params': costmap_params(),
        },
        'front_distance': camera_summary(cam, depth_topic),
    }
    ctx = {'bag': bag, 'name': name, 'rows': rows, 'cm': cm, 'cells': cells, 'odom': odom,
           'cam': cam,
           'stereo': stereo, 'front_x': front_x, 'half': half,
           'to_base': to_base, 'tfpose': tfpose, 'summary': summary,
           'cam_roi': camera_roi(bag, to_base)}
    ctx['frames'] = build_timeline(ctx)
    summary['camera_roi'] = ({k: (round(v, 4) if isinstance(v, float) else v)
                              for k, v in ctx['cam_roi'].items()} if ctx['cam_roi'] else None)
    if summary['camera_roi']:
        for k in ('left', 'right'):
            summary['camera_roi'][k + '_deg'] = round(math.degrees(ctx['cam_roi'][k]), 2)
    summary['front_distance_from'] = ctx['fd_src']
    att = [a for a in ctx['stereo_att'] if a['distance'] is not None]
    matched = [a for a in att if a['plane'] is not None]
    kinds = collections.Counter(wall_kind(a['plane']) for a in matched)
    summary['stereo_wall_attribution'] = {
        'planes_from': ctx['stereo_att_src'],
        'messages_with_distance': len(att), 'matched_to_a_plane': len(matched),
        'residual_m': ({'median': round(statistics.median([a['residual'] for a in matched]), 4),
                        'p95': round(float(np.percentile([a['residual'] for a in matched], 95)),
                                     4)} if matched else None),
        'matched_plane_kind': dict(kinds),
        'source_field': dict(collections.Counter(a['source'] for a in att)),
    }
    summary['phantoms'] = phantom_summary(ctx)
    (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=1, default=str))
    write_md(out_dir / f'{name}.summary.md', summary)
    if video:
        mp4 = render_video(ctx, out_dir / f'{name}.mp4')
        summary['mp4'] = str(mp4) if mp4 else None
        (out_dir / f'{name}.summary.json').write_text(json.dumps(summary, indent=1,
                                                                 default=str))
    print(json.dumps({k: summary[k] for k in ('lidar_front_wall', 'costmap_front_clearance')},
                     indent=1, default=str))
    return summary


# ============================================================================
# Timeline: every source at VIDEO_HZ, and what produced it
# ============================================================================

def plane_sets(ctx):
    """[(receipt, stamp, [WallLine])] of /perception/wall_planes: recorded if in
    the bag, else the replay's. The node publishes them at the START of a scan
    callback (_scene_fit, before the tracker), so a replayed set counts as
    received at its callback's start."""
    rec = ctx['bag'].get('/perception/wall_planes')
    if rec:
        out = []
        for t, m in rec:
            d = json.loads(m.data)
            out.append((t, d['stamp'], [scene_labels.WallLine.from_dict(w) for w in d['walls']]))
        return out, 'recorded /perception/wall_planes'
    out = [(r['start'], r['planes']['stamp'],
            [scene_labels.WallLine.from_dict(w) for w in r['planes']['walls']])
           for r in ctx['rows'] if r and r['planes']]
    return out, 'replayed /perception/wall_planes'


def attribute_stereo(ctx):
    """Per /perception/stereo_wall message: the wall plane its distance was measured on.

    front_clearance_node labels its depth points WALL against the latest
    /perception/wall_planes no older than front_clearance_scene_wall_max_age_s
    and takes the distance on ONE plane (stereo_plane). Candidates are the
    last three plane sets received before the stereo message; a match needs
    the plane's own bumper offset within STEREO_MATCH_M of the value.
    """
    sets, src = plane_sets(ctx)
    max_age = float(get_value('front_clearance_scene_wall_max_age_s'))
    times = [s[0] for s in sets]
    out = []
    for t, d in ctx['stereo']:
        res = {'t': t, 'stamp': d.get('stamp'), 'distance': d.get('distance_m'),
               'source': d.get('source'), 'n_px': d.get('n_px'), 'plane': None,
               'residual': None}
        # 'stereo' = a stereo-only fallback plane, which is in no LiDAR set.
        if (res['distance'] is not None and res['stamp'] is not None
                and res['source'] != 'stereo'):
            i = bisect.bisect_right(times, t) - 1
            best = None
            for k in range(i, max(i - 3, -1), -1):
                _pt, pstamp, walls = sets[k]
                if abs(res['stamp'] - pstamp) > max_age:
                    continue
                j, r = stereo_plane(walls, res['distance'], ctx['front_x'])
                if j is not None and (best is None or r < best[1]):
                    best = (walls[j], r)
            if best and best[1] <= STEREO_MATCH_M:
                res['plane'], res['residual'] = best
        out.append(res)
    return out, src


def latest(times, values, t, max_age=math.inf, default=None):
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or t - times[i] > max_age:
        return default
    return values[i]


def wall_kind(w):
    """'side' or 'facing' by the normal's angle off the heading (oblique_max_deg)."""
    oblique = math.radians(float(get_value('lidar_front_wall_oblique_max_deg')))
    return 'side' if abs(w.normal_angle) > oblique else 'facing'


def build_timeline(ctx):
    bag, front_x, half = ctx['bag'], ctx['front_x'], ctx['half']
    laser_tf = ctx['to_base']('laser')
    rot, trans = laser_tf
    laser = (float(trans[0]), float(trans[1]), math.atan2(rot[1, 0], rot[0, 0]))
    ctx['laser'] = laser
    scans = bag.get('/scan')
    scan_t = [t for t, _ in scans]
    rows = [r for r in ctx['rows'] if r]
    row_t = [r['end'] for r in rows]
    vw = bag.get('/perception/virtual_wall')
    fd = bag.get('/perception/front_distance')
    if fd:
        fd_tv = ([t for t, _ in fd], [float(m.data) for _t, m in fd])
        fd_src = '/perception/front_distance'
    else:
        dbg = [(t, json.loads(m.data)) for t, m in bag.get('/perception/debug/front_wall')]
        fd_tv = ([t for t, _ in dbg],
                 [-1.0 if d.get('front_distance_m') is None else d['front_distance_m']
                  for _t, d in dbg])
        fd_src = '/perception/debug/front_wall front_distance_m'
    ctx['fd_src'] = fd_src
    cm = ctx['cm']
    cm_rec = bag.get('/costmap/front_clearance')
    drive = bag.get('/drive')
    att, att_src = attribute_stereo(ctx)
    ctx['stereo_att'], ctx['stereo_att_src'] = att, att_src
    att_t = [a['t'] for a in att]
    dets = bag.get('/camera/detections_3d')
    det_t = [t for t, _ in dets]
    cam_t = [c['t'] for c in ctx['cam']]
    wr_t = [r['end'] for r in rows if r['rec_walls'] is not None]
    wr = [r['rec_walls'] for r in rows if r['rec_walls'] is not None]
    ser = {
        'vw': ([t for t, _ in vw], [float(m.data) for _t, m in vw]),
        'fd': fd_tv,
        'cm': ([t for t, _ in cm_rec], [float(m.data) for _t, m in cm_rec]),
        'cm_rep': ([c['t'] for c in cm], cm),
        'st': (att_t, att),
        'spd': ([t for t, _ in drive], [m.drive.speed for _t, m in drive]),
        'row': (row_t, rows),
    }
    ctx['series'] = ser
    frames = []
    if not scans:
        return frames
    for t in np.arange(scan_t[0], bag.t1, 1.0 / VIDEO_HZ):
        t = float(t)
        pose, path = ctx['odom'].latest(t)
        scan = latest(scan_t, [m for _t, m in scans], t)
        pts = scan_xy(scan, laser)
        strip, strip_pt = strip_nearest(pts, front_x, half)
        row = latest(*ser['row'], t)
        cmr = latest(*ser['cm_rep'], t, max_age=0.5)
        f = {
            't': t, 'pose': pose, 'path': path, 'scan': scan, 'pts': pts,
            'strip': strip, 'strip_pt': strip_pt, 'row': row,
            'vw': latest(*ser['vw'], t, max_age=1.0),
            'fd': latest(*ser['fd'], t, max_age=1.0),
            'cm': latest(*ser['cm'], t, max_age=0.5),
            'cm_rep': cmr,
            'st': latest(*ser['st'], t, max_age=1.0),
            'spd': latest(*ser['spd'], t, max_age=0.5),
            'tr_rec': front_ray(row['rec_walls']) if row else math.nan,
            'tr_rep': front_ray(row['walls']) if row else math.nan,
            'det': latest(det_t, [m for _t, m in dets], t, max_age=1.0),
            'cam': latest(cam_t, ctx['cam'], t, max_age=0.6),
            'walls_rec': latest(wr_t, wr, t),
        }
        f['vw_src'] = (row['rec_dbg'] or row['dbg'])['source'] if row else None
        f['vw_stereo'] = stereo_for_row(row, att) if row else None
        frames.append(f)
    return frames


def stereo_for_row(row, att):
    """The stereo attribution entry the replayed callback used, or None."""
    used = row.get('stereo_used')
    if used is None:
        return None
    best = None
    for a in att:
        if a['t'] > row['start'] + 1e-6:
            break
        if a['distance'] is not None and abs(a['distance'] - used[1]) < 1e-6:
            best = a
    return best


SOURCES = [
    # key, label, BGR colour
    ('vw', 'virtual_wall', (0, 140, 255)),
    ('fd', 'front_distance (camera)', (255, 80, 255)),
    ('cm', 'costmap front_clearance', (255, 255, 0)),
    ('tr', 'tracker d_ray_front', (0, 220, 0)),
]
C_STRIP = (255, 255, 255)
C_STEREO = (0, 230, 230)
C_SCAN = (175, 175, 175)
C_CUT = (110, 60, 130)
C_MAP = (110, 70, 40)
C_TRACK = (255, 160, 60)
C_FRONT_TRACK = (40, 40, 255)


def frame_value(f, key):
    v = {'vw': f['vw'], 'fd': f['fd'], 'cm': f['cm'], 'tr': f['tr_rec']}[key]
    return v if v is not None and math.isfinite(v) and v > 0.0 else math.nan


def phantom_summary(ctx):
    """Episodes where a source reads PHANTOM_MARGIN_M or more short of the true
    strip range for at least PHANTOM_MIN_S, with what produced the value."""
    frames = ctx['frames']
    times = [f['t'] for f in frames]
    t0 = ctx['bag'].t0
    out = []
    for key, label, _c in SOURCES:
        flags = []
        for f in frames:
            v = frame_value(f, key)
            strip = f['strip'] if math.isfinite(f['strip']) else math.inf
            flags.append(math.isfinite(v) and v < strip - PHANTOM_MARGIN_M)
        for a, b in episodes(times, flags, PHANTOM_MIN_S):
            sel = [f for f in frames if a <= f['t'] < b]
            vals = [frame_value(f, key) for f in sel]
            strips = [f['strip'] for f in sel]
            ep = {'source': label, 't_start': round(a - t0, 2), 't_end': round(b - t0, 2),
                  'duration_s': round(b - a, 2),
                  'travelled_m': [round(sel[0]['path'], 2), round(sel[-1]['path'], 2)],
                  'value_m': {'median': round(statistics.median(vals), 3),
                              'min': round(min(vals), 3), 'max': round(max(vals), 3)},
                  'true_strip_m': {
                      'median': num(statistics.median([s for s in strips if math.isfinite(s)])
                                    if any(math.isfinite(s) for s in strips) else math.nan),
                      'no_return_frames': sum(1 for s in strips if not math.isfinite(s)),
                      'frames': len(strips)},
                  'moving_frames': sum(1 for f in sel if f['spd'] and abs(f['spd']) > 0.05)}
            ep['producer'] = producer_summary(ctx, key, sel)
            out.append(ep)
    return out


def track_of_plane(w, walls_msg):
    """'id:label' of the recorded track lying on a base_link WallLine, or None."""
    if walls_msg is None:
        return None
    best, cost = None, math.inf
    for trk in walls_msg.tracks:
        for rho, th in ((trk.rho_bl, trk.theta_bl), (-trk.rho_bl, trk.theta_bl + math.pi)):
            dth = abs(math.atan2(math.sin(th - w.normal_angle), math.cos(th - w.normal_angle)))
            c = abs(rho - w.d) + dth
            if dth < math.radians(10.0) and abs(rho - w.d) < 0.2 and c < cost:
                best, cost = f'{trk.id}:{trk.label}', c
    return best


def producer_summary(ctx, key, sel):
    """What produced a source's value over an episode, from the code paths."""
    if key == 'vw':
        srcs = collections.Counter(f['vw_src'] for f in sel)
        lidar = collections.Counter(
            f"{f['row']['dbg']['lidar_provenance']}/"
            f"{REASON_NAMES.get(f['row']['dbg']['lidar_reason'])}" for f in sel if f['row'])
        planes = collections.Counter()
        tracks = collections.Counter()
        resid = []
        for f in sel:
            st = f['vw_stereo']
            if st and st['plane'] is not None and 'stereo' in (f['vw_src'] or ''):
                w = st['plane']
                planes[(wall_kind(w), round(math.degrees(w.normal_angle)),
                        round(w.d - ctx['front_x'] * w.nx, 2))] += 1
                resid.append(st['residual'])
                tracks[track_of_plane(w, f['walls_rec'])] += 1
        return {'virtual_wall_source': dict(srcs),
                'lidar_estimate (provenance/reason)': dict(lidar.most_common(4)),
                'stereo_plane_is_track (id:label)': dict(tracks.most_common(4)),
                'stereo_planes (kind, normal deg, bumper offset m): frames':
                    {f'{k[0]} wall, normal {k[1]:+d} deg, {k[2]:.2f} m': n
                     for k, n in planes.most_common(5)},
                'stereo_plane_match_residual_m': (
                    {'median': round(statistics.median(resid), 4), 'max': round(max(resid), 4)}
                    if resid else None),
                'stereo_planes_from': ctx['stereo_att_src']}
    if key == 'cm':
        cells = [f['cm_rep']['cell'] for f in sel if f['cm_rep'] and f['cm_rep']['cell']]
        if not cells:
            return {'cell': None}
        xs, ys = [c[0] for c in cells], [c[1] for c in cells]
        bearings = [math.degrees(math.atan2(y, x)) for x, y in cells]
        gaps = [f['cm_rep']['cell_to_scan'] for f in sel
                if f['cm_rep'] and math.isfinite(f['cm_rep'].get('cell_to_scan', math.nan))]
        return {'cell_to_nearest_scan_return_m': (
                    {'median': round(statistics.median(gaps), 3),
                     'p95': round(float(np.percentile(gaps, 95)), 3)} if gaps else None),
                'nearest_map_cell_base_link': {
                    'x_median': round(statistics.median(xs), 2),
                    'y_median': round(statistics.median(ys), 2),
                    'bearing_deg_median': round(statistics.median(bearings), 1),
                    'bearing_deg_range': [round(min(bearings), 1), round(max(bearings), 1)]},
                'cone_half_angle_deg': ctx['summary']['costmap_front_clearance']['params'][
                    'front_facing_max_deg'],
                'map': ctx['cm'][0]['map_src'] if ctx['cm'] else None}
    if key == 'fd':
        cams = {id(f['cam']): f['cam'] for f in sel if f['cam'] is not None}
        if cams:
            kinds = collections.Counter()
            inside = total = 0
            for c in cams.values():
                kinds.update(c['band_kind'].tolist())
                xy = c['band_xy']
                inside += int(np.count_nonzero(np.abs(xy[:, 1]) < ctx['half']))
                total += len(xy)
            return {'depth_frames': len(cams),
                    'band_pixels_by_kind': {k: round(n / max(total, 1), 3)
                                            for k, n in kinds.most_common()},
                    'band_pixels_inside_strip': round(inside / max(total, 1), 3),
                    'kinds': 'side/facing = WALL-labelled pixel on a side/facing wall plane, '
                             'wall? = WALL with no plane within 0.10 m, other = OTHER label'}
        roi = ctx.get('cam_roi')
        if not roi:
            return {}
        vals = [frame_value(f, 'fd') for f in sel]
        x = ctx['front_x'] + statistics.median(vals)
        ox, oy = roi['origin']
        y_left = oy + (x - ox) * math.tan(roi['left'])
        y_right = oy + (x - ox) * math.tan(roi['right'])
        return {'depth_locus_x_base_link': round(x, 2),
                'roi_width_at_locus_y': [round(y_right, 2), round(y_left, 2)],
                'note': 'front_distance is a band mean of ROI depths (Z, along the optical '
                        'axis); without depth in the bag only its locus can be drawn'}
    return {}


def write_md(path, summary):
    lid = summary['lidar_front_wall']
    cmf = summary['costmap_front_clearance']
    lines = [f'# {summary["name"]}', '', f'bag `{summary["bag"]}`, {summary["span_s"]} s', '',
             '## Flags', ''] + [f'- {f}' for f in summary['flags']] + [
        '', '## Reproduction', '',
        '| quantity | n | median abs diff | p95 | max |', '|---|---|---|---|---|']

    def row(label, d):
        if not d.get('n'):
            return f'| {label} | 0 | | | |'
        return f'| {label} | {d["n"]} | {d["median"]} | {d["p95"]} | {d["max"]} |'
    lines += [row('virtual_wall [m], per replayed scan', lid['virtual_wall_abs_diff_m']),
              row('track d_perp_front [m], matched tracks', lid['tracks_d_perp_front_abs_diff_m']),
              row('tracker d_ray_front [m]', lid['d_ray_front_abs_diff_m']),
              row('costmap front_clearance [m], own-time map', cmf['abs_diff_m_exact_map']),
              row('costmap front_clearance [m], map from later', cmf['abs_diff_m_map_from_later']),
              '',
              f'- virtual_wall validity (recorded / replay): `{lid["virtual_wall_validity"]}`',
              f'- virtual_wall source agreement: {lid["virtual_wall_source_agreement"]}',
              f'- d_ray_front finite: `{lid["d_ray_front_finite"]}`',
              f'- tracks: `{lid["track_matching"]}`',
              f'- callbacks: {lid["callbacks_replayed"]} replayed of {lid["callbacks_recorded"]}'
              f' recorded; callback start - scan receipt '
              f'{lid["callback_start_minus_scan_receipt_ms"]} ms',
              f'- front_distance: {summary["front_distance"]}', '',
              '## Phantom episodes', '',
              f'A source reading at least {PHANTOM_MARGIN_M} m short of the true strip range '
              f'for at least {PHANTOM_MIN_S} s.', '']
    for ep in summary['phantoms']:
        lines.append(f'- **{ep["source"]}** {ep["t_start"]}-{ep["t_end"]} s '
                     f'({ep["duration_s"]} s, travelled {ep["travelled_m"][0]}-'
                     f'{ep["travelled_m"][1]} m, moving {ep["moving_frames"]} frames): value '
                     f'{ep["value_m"]["median"]} m (min {ep["value_m"]["min"]}) vs true strip '
                     f'{ep["true_strip_m"]["median"]} m; producer `{json.dumps(ep["producer"])}`')
    path.write_text('\n'.join(lines) + '\n')


# ============================================================================
# Video
# ============================================================================

class View:
    """base_link top-down, car fixed and pointing up."""

    def __init__(self, w=760, h=760, x_min=-1.0, x_max=8.5):
        self.w, self.h = w, h
        self.s = h / (x_max - x_min)
        self.cu = w / 2.0
        self.v0 = h + x_min * self.s

    def px(self, x, y):
        return int(round(self.cu - y * self.s)), int(round(self.v0 - x * self.s))

    def pxs(self, pts):
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        return np.column_stack([self.cu - pts[:, 1] * self.s,
                                self.v0 - pts[:, 0] * self.s]).round().astype(np.int32)

    def line(self, img, a, b, color, thick=1, dash=0):
        ok, p, q = cv2.clipLine((0, 0, self.w, self.h), self.px(*a), self.px(*b))
        if not ok:
            return
        if not dash:
            cv2.line(img, p, q, color, thick, cv2.LINE_AA)
            return
        p, q = np.array(p, float), np.array(q, float)
        n = np.linalg.norm(q - p)
        if n < 1:
            return
        u = (q - p) / n
        for s0 in np.arange(0.0, n, 2 * dash):
            a1, b1 = p + u * s0, p + u * min(s0 + dash, n)
            cv2.line(img, tuple(a1.astype(int)), tuple(b1.astype(int)), color, thick, cv2.LINE_AA)

    def ray(self, img, origin, angle, color, thick=1, dash=0, length=30.0):
        self.line(img, origin, (origin[0] + length * math.cos(angle),
                                origin[1] + length * math.sin(angle)), color, thick, dash)

    def arc(self, img, origin, r, a0, a1, color, thick=1):
        cv2.ellipse(img, self.px(*origin), (int(r * self.s), int(r * self.s)), 0,
                    -90 - math.degrees(a1), -90 - math.degrees(a0), color, thick, cv2.LINE_AA)

    def text(self, img, s, xy, color, scale=0.42, thick=1):
        u, v = self.px(*xy)
        u = int(np.clip(u, 2, self.w - 8 * len(s) * scale / 0.42))
        v = int(np.clip(v, 12, self.h - 4))
        put(img, s, (u, v), color, scale, thick)


def put(img, s, uv, color, scale=0.45, thick=1):
    cv2.putText(img, s, uv, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, uv, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def line_through(w, rel=None):
    """Two far points of a WallLine (base_link), optionally moved by rel (Pose2D)."""
    px, py = w.nx * w.d, w.ny * w.d
    tx, ty = -w.ny, w.nx
    pts = np.array([[px + tx * -40, py + ty * -40], [px + tx * 40, py + ty * 40]])
    seg = np.array(w.endpoints())
    if rel is not None:
        pts, seg = rel.transform_points(pts), rel.transform_points(seg)
    return pts, seg


def fmt(x, unit=' m'):
    return '--' if x is None or not math.isfinite(x) or x <= 0.0 else f'{x:.2f}{unit}'


def draw_topdown(ctx, f, view):
    front_x, half = ctx['front_x'], ctx['half']
    img = np.full((view.h, view.w, 3), 18, np.uint8)
    pose = f['pose']
    # grid
    for m in range(-1, 9):
        view.line(img, (m, -6), (m, 6), (45, 45, 45))
        view.text(img, f'{m} m', (m + 0.05, 4.6), (100, 100, 100), 0.38)
    for yy in range(-4, 5):
        view.line(img, (-1, yy), (9, yy), (35, 35, 35))
    # map cells (costmap's input)
    cmr = f['cm_rep']
    if cmr is not None and ctx['cells']:
        cells = ctx['cells'][cmr['grid_key']]
        mp = Pose2D(*cmr['pose'])
        rel = mp.inverse()
        loc = rel.transform_points(cells)
        keep = (loc[:, 0] > -1.2) & (loc[:, 0] < 9) & (np.abs(loc[:, 1]) < 5)
        for u, v in view.pxs(loc[keep]):
            cv2.rectangle(img, (u - 1, v - 1), (u + 1, v + 1), C_MAP, -1)
    # ---- ROIs --------------------------------------------------------------
    lx, ly, _ = ctx['laser']
    row = f['row']
    rel_row = (relative(ctx['odom'].at_stamp(row['scan_stamp']), pose) if row else None)
    planes_half = math.radians(float(get_value('lidar_front_wall_wall_planes_half_angle_deg')))
    c_vw = SOURCES[0][2]
    for a in (-planes_half, planes_half):
        view.ray(img, (lx, ly), a, c_vw, 1, dash=6)
    probe = math.radians(2.0)
    for a in (-probe, probe):
        view.ray(img, (lx, ly), a, c_vw, 1, dash=2, length=12)
    view.text(img, 'VW LiDAR +-60 deg', (lx + 4.6 * math.cos(planes_half),
                                         4.6 * math.sin(planes_half) - 0.1), c_vw, 0.38)
    if row:
        for lo, hi in row['sectors']:
            poly = [(lx, ly)] + [(lx + 9 * math.cos(a), ly + 9 * math.sin(a))
                                 for a in np.linspace(lo, hi, 8)]
            poly = rel_row.transform_points(poly)
            over = img.copy()
            cv2.fillPoly(over, [view.pxs(poly)], C_CUT)
            img = cv2.addWeighted(over, 0.35, img, 0.65, 0)
    roi = ctx.get('cam_roi')
    c_fd = SOURCES[1][2]
    if roi:
        for a in (roi['left'], roi['right']):
            view.ray(img, roi['origin'], a, c_fd, 1, dash=4)
        if math.isfinite(roi['floor_x']):
            y_l = roi['origin'][1] + (roi['floor_x'] - roi['origin'][0]) * math.tan(roi['left'])
            y_r = roi['origin'][1] + (roi['floor_x'] - roi['origin'][0]) * math.tan(roi['right'])
            view.line(img, (roi['floor_x'], y_r), (roi['floor_x'], y_l), c_fd, 1, dash=2)
        view.text(img, 'camera ROI', (roi['origin'][0] + 7.6, (7.4) * math.tan(roi['left'])),
                  c_fd, 0.38)
    c_cm = SOURCES[2][2]
    cmp = ctx['summary']['costmap_front_clearance']['params']
    cone = math.radians(cmp['front_facing_max_deg'])
    rmax = cmp['max_range_m']
    for a in (-cone, cone):
        view.ray(img, (0, 0), a, c_cm, 1, dash=8, length=rmax)
    view.arc(img, (0, 0), rmax, -cone, cone, c_cm, 1)
    view.text(img, 'costmap +-35 deg, 5 m', (rmax * math.cos(cone) + 0.1, rmax * math.sin(cone)),
              c_cm, 0.38)
    c_tr = SOURCES[3][2]
    view.line(img, (front_x, 0), (30, 0), c_tr, 1, dash=3)
    view.line(img, (front_x, half), (30, half), C_STRIP, 1)
    view.line(img, (front_x, -half), (30, -half), C_STRIP, 1)
    # ---- scan, detections ------------------------------------------------------
    for u, v in view.pxs(f['pts']):
        if 0 <= u < view.w and 0 <= v < view.h:
            img[max(v - 1, 0):v + 2, max(u - 1, 0):u + 2] = C_SCAN
    det = f['det']
    if det is not None:
        tf = ctx['to_base'](det.header.frame_id)
        if tf is not None:
            for d in det.detections:
                p = d.bbox.center.position
                x, y, _z = tf[0] @ np.array([p.x, p.y, p.z]) + tf[1]
                u, v = view.px(x, y)
                r = max(int(0.5 * d.bbox.size.x * view.s), 3)
                cv2.circle(img, (u, v), r, C_CUT, 1, cv2.LINE_AA)
                cls = d.results[0].hypothesis.class_id if d.results else '?'
                put(img, cls, (u + r + 2, v), (200, 140, 220), 0.38)
    # ---- recorded tracks: segment, label, faint infinite extension ---------------
    rec_walls = f['walls_rec']
    if rec_walls is not None:
        inv = pose.inverse()
        for trk in rec_walls.tracks:
            p0, p1 = inv.transform_points([[trk.p0.x, trk.p0.y], [trk.p1.x, trk.p1.y]])
            d = p1 - p0
            n = np.linalg.norm(d)
            if n < 1e-6:
                continue
            u = d / n
            view.line(img, p0 - 40 * u, p1 + 40 * u, (90, 90, 90), 1, dash=3)
            col = C_FRONT_TRACK if trk.label == 'front' else C_TRACK
            view.line(img, p0, p1, col, 2)
            mid = 0.5 * (p0 + p1)
            mid = np.array([np.clip(mid[0], -0.8, 8.2), np.clip(mid[1], -4.4, 4.4)])
            view.text(img, f'{trk.id}:{trk.label} d={trk.d_perp_front:.2f}',
                      (mid[0] + 0.15, mid[1]), col, 0.4)
    # ---- producers ---------------------------------------------------------------
    if row is not None:
        src = f['vw_src'] or ''
        tw = row['tracked_wall']
        if src.startswith('lidar') and not src.startswith('lidar_dr') and tw is not None:
            pts = row['points']
            res = np.abs(tw.d - (pts[:, 0] * tw.nx + pts[:, 1] * tw.ny))
            on = res <= 0.03
            far, seg = line_through(tw, rel_row)
            view.line(img, seg[0], seg[1], c_vw, 4)
            for u, v in view.pxs(rel_row.transform_points(pts[on])):
                cv2.circle(img, (u, v), 2, c_vw, -1)
            view.text(img, f'VW LiDAR wall {fmt(f["vw"])}', (seg[0] + seg[1]) / 2 + [0.3, 0],
                      c_vw, 0.45)
        elif src.startswith('lidar_dr') and row['anchor'] is not None:
            est = row['estimate']
            ep = row['est_pose']
            if ep is not None and math.isfinite(est.distance):
                na = est.normal_angle
                w = scene_labels.WallLine(math.cos(na), math.sin(na), est.distance, -1.5, 1.5,
                                          0, 0.0, 'dr')
                far, seg = line_through(w, relative(ep, pose))
                view.line(img, seg[0], seg[1], c_vw, 3, dash=6)
                view.text(img, f'VW dead-reckoned {fmt(f["vw"])} ({est.source_age:.1f} s)',
                          (seg[0] + seg[1]) / 2 + [0.3, 0], c_vw, 0.45)
        if 'stereo' in src and f['vw_stereo'] and f['vw_stereo']['plane'] is not None:
            w = f['vw_stereo']['plane']
            stamp_pose = ctx['odom'].at_stamp(f['vw_stereo']['stamp'])
            rel = relative(stamp_pose, pose)
            far, seg = line_through(w, rel)
            view.line(img, seg[0], seg[1], C_STEREO, 4)
            bump = rel.transform_points([[front_x, 0.0]])[0]
            n_now = rel.R @ np.array([w.nx, w.ny])
            foot = bump + n_now * f['vw_stereo']['distance']
            cv2.arrowedLine(img, view.px(*bump), view.px(*foot), C_STEREO, 2, cv2.LINE_AA,
                            tipLength=0.08)
            view.text(img, f'stereo {f["vw_stereo"]["distance"]:.2f} = perpendicular to '
                           f'{wall_kind(w)} plane', (foot + bump) / 2 + [0.25, 0.1],
                      C_STEREO, 0.45)
        if f['vw'] is not None and f['vw'] > 0:
            view.line(img, (front_x + f['vw'], -0.6), (front_x + f['vw'], 0.6), c_vw, 3)
            view.text(img, f'virtual_wall {f["vw"]:.2f}', (front_x + f['vw'] + 0.12, 0.62),
                      c_vw, 0.45)
    cam = f['cam']
    if cam is not None and len(cam['band_xy']):
        cpose = ctx['odom'].at_stamp(cam['stamp'])
        rel = relative(cpose, pose)
        colors = {'side': (255, 80, 255), 'facing': (255, 255, 255), 'wall?': (200, 150, 255),
                  'other': (150, 60, 150)}
        xy = rel.transform_points(cam['band_xy'])
        step = max(len(xy) // 4000, 1)
        for kind, color in colors.items():
            sel = cam['band_kind'][::step] == kind
            for u, v in view.pxs(xy[::step][sel]):
                if 0 <= u < view.w and 0 <= v < view.h:
                    img[v, u] = color
    if f['fd'] is not None and f['fd'] > 0 and roi:
        x = front_x + f['fd']
        y_l = roi['origin'][1] + (x - roi['origin'][0]) * math.tan(roi['left'])
        y_r = roi['origin'][1] + (x - roi['origin'][0]) * math.tan(roi['right'])
        view.line(img, (x, y_r), (x, y_l), c_fd, 2, dash=5)
        view.text(img, f'front_distance {f["fd"]:.2f} (depth locus)', (x + 0.12, y_r - 0.05),
                  c_fd, 0.42)
    if cmr is not None and cmr['cell'] is not None:
        cx, cy = cmr['cell']
        # The cell is base_link at the tick's map pose; that pose and the
        # current odometry pose are the same instant up to the tick period.
        u, v = view.px(cx, cy)
        cv2.rectangle(img, (u - 5, v - 5), (u + 5, v + 5), c_cm, 2)
        view.line(img, (front_x, 0), (cx, cy), c_cm, 1)
        view.text(img, f'costmap cell {fmt(cmr["rep"])}', (cx + 0.15, cy + 0.1), c_cm, 0.42)
    if math.isfinite(f['tr_rec']):
        view.line(img, (front_x + f['tr_rec'], -0.3), (front_x + f['tr_rec'], 0.3), c_tr, 3)
    if f['strip_pt'] is not None:
        u, v = view.px(*f['strip_pt'])
        cv2.circle(img, (u, v), 6, C_STRIP, 2, cv2.LINE_AA)
        view.text(img, f'true strip {f["strip"]:.2f}', (f['strip_pt'][0] + 0.1, -half - 0.05),
                  C_STRIP, 0.45)
    # ---- car ----------------------------------------------------------------------
    body = float(get_value('swept_clearance_body_half_width_m'))
    cv2.rectangle(img, view.px(front_x, body), view.px(-0.15, -body), (230, 230, 230), 2)
    view.line(img, (front_x, -body - 0.05), (front_x, body + 0.05), (0, 0, 255), 2)
    return img


def draw_panel(ctx, f, w=520, h=760):
    img = np.full((h, w, 3), 28, np.uint8)
    t0 = ctx['bag'].t0
    row = f['row']
    y = 26

    def line(s, color=(220, 220, 220), scale=0.5, gap=24):
        nonlocal y
        put(img, s, (12, y), color, scale)
        y += gap
    line(f'{ctx["name"]}', (255, 255, 255), 0.5)
    line(f't = {f["t"] - t0:6.2f} s    travelled {f["path"]:.2f} m (odometry/filtered)')
    line(f'/drive speed {fmt(f["spd"], " m/s") if f["spd"] else "0.00 m/s"}')
    y += 6
    c_vw, c_fd, c_cm, c_tr = (s[2] for s in SOURCES)
    rep_vw = row['vw'] if row else None
    line(f'virtual_wall      {fmt(f["vw"])}   replay {fmt(rep_vw)}', c_vw, 0.55)
    if row and abs(max(row['rec_vw'], 0.0) - max(row['vw'], 0.0)) > 0.05:
        line('   REPLAY DIFFERS here; the producer drawn is the replay\'s',
             (60, 60, 255), 0.45, 20)
    if row:
        d = row['dbg']
        line(f'   source {f["vw_src"]}  lidar {fmt(d["lidar_m"] or math.nan)}  '
             f'stereo {fmt(d["stereo_m"] or math.nan)}', c_vw, 0.45, 20)
        line(f'   LiDAR {d["lidar_provenance"]} / {REASON_NAMES.get(d["lidar_reason"])}, '
             f'wall from {d["wall_source"]}', c_vw, 0.45, 20)
        line(f'   from the scan of t-{f["t"] - row["scan_stamp"]:.2f} s', c_vw, 0.45, 20)
        st = f['vw_stereo']
        if st and st['plane'] is not None:
            w = st['plane']
            line(f'   stereo plane: {wall_kind(w)} wall, normal '
                 f'{math.degrees(w.normal_angle):+.0f} deg, d-front_x*nx '
                 f'{w.d - ctx["front_x"] * w.nx:.2f}', C_STEREO, 0.45, 20)
    y += 6
    cam = f['cam']
    if ctx['cam']:
        line(f'front_distance    {fmt(f["fd"])}   replay raw (lens) '
             f'{fmt(cam["raw_rep"]) if cam and cam["raw_rep"] else "--"}', c_fd, 0.55)
        if cam is not None and len(cam['band_kind']):
            kinds = collections.Counter(cam['band_kind'].tolist())
            n = sum(kinds.values())
            inside = np.count_nonzero(np.abs(cam['band_xy'][:, 1]) < ctx['half'])
            line('   band px: ' + ', '.join(f'{k} {100 * v / n:.0f}%'
                                            for k, v in kinds.most_common())
                 + f'; in strip {100 * inside / n:.0f}%', c_fd, 0.45, 20)
    else:
        line(f'front_distance    {fmt(f["fd"])}   (camera depth: not in bag)', c_fd, 0.55)
    cmr = f['cm_rep']
    line(f'costmap clearance {fmt(f["cm"])}   replay {fmt(cmr["rep"]) if cmr else "--"}', c_cm,
         0.55)
    if cmr and cmr['cell']:
        cx, cy = cmr['cell']
        line(f'   nearest map cell ({cx:.2f}, {cy:.2f}) bearing '
             f'{math.degrees(math.atan2(cy, cx)):+.0f} deg'
             + ('' if cmr['exact_map'] else ' [later map]'), c_cm, 0.45, 20)
        line(f'   that cell is {fmt(cmr.get("cell_to_scan", math.nan))} from the nearest '
             'LiDAR return', c_cm, 0.45, 20)
    line(f'tracker d_ray     {fmt(f["tr_rec"])}   replay {fmt(f["tr_rep"])}', c_tr, 0.55)
    if row and row['rec_walls'] is not None:
        fronts = [t for t in row['rec_walls'].tracks if t.label == 'front']
        line('   front tracks: ' + (', '.join(f'{t.id} d_perp {t.d_perp_front:.2f}'
                                              for t in fronts) or 'none'), c_tr, 0.45, 20)
    line(f'TRUE strip range  {fmt(f["strip"])}' + (
        f'  at ({f["strip_pt"][0]:.2f}, {f["strip_pt"][1]:+.2f})' if f['strip_pt'] is not None
        else '  (no return in strip)'), C_STRIP, 0.55)
    line(f'   |y| < {ctx["half"]:.3f} m, from the bumper (x {ctx["front_x"]:.3f})', C_STRIP, 0.45,
         20)
    y += 8
    strip = f['strip'] if math.isfinite(f['strip']) else math.inf
    for key, label, color in SOURCES:
        v = frame_value(f, key)
        if math.isfinite(v) and v < strip - PHANTOM_MARGIN_M:
            line(f'PHANTOM {label}: {v:.2f} vs strip {fmt(f["strip"])}', (60, 60, 255), 0.5)
    y = h - 146
    if ctx['cam']:
        line('camera band px: pink side wall, white facing wall, dim other', (255, 80, 255),
             0.42, 18)
    else:
        y += 18
    for key, label, color in SOURCES:
        line(f'-- {label}', color, 0.42, 18)
    line('-- stereo plane (VW stereo input), arrow = its distance', C_STEREO, 0.42, 18)
    line('-- strip |y| < half width + 0.10, o = nearest return', C_STRIP, 0.42, 18)
    line('blue: tracks (red: labelled front); grey dashed: extensions', C_TRACK, 0.42, 18)
    return img


class Chart:
    """Strip chart of every front value against time, drawn once."""

    def __init__(self, ctx, w=1280, h=240, y_max=10.0):
        self.w, self.h, self.y_max = w, h, y_max
        self.t0, self.t1 = ctx['frames'][0]['t'], ctx['frames'][-1]['t']
        self.l, self.r, self.top, self.bot = 50, w - 130, 12, h - 26
        img = np.full((h, w, 3), 22, np.uint8)
        for m in range(0, int(y_max) + 1, 2):
            v = self.v(m)
            cv2.line(img, (self.l, v), (self.r, v), (50, 50, 50), 1)
            put(img, f'{m}', (self.l - 22, v + 4), (130, 130, 130), 0.38)
        for s in range(0, int(self.t1 - self.t0) + 1, 10):
            u = self.u(self.t0 + s)
            cv2.line(img, (u, self.top), (u, self.bot), (40, 40, 40), 1)
            put(img, f'{s}s', (u - 8, h - 8), (130, 130, 130), 0.38)
        frames = ctx['frames']
        # speed, right axis 0..1 m/s, filled
        sp = [(self.u(f['t']),
               self.bot - int(min(abs(f['spd'] or 0.0), 1.0) * (self.bot - self.top) * 0.3))
              for f in frames]
        poly = np.array([(self.l, self.bot)] + sp + [(sp[-1][0], self.bot)], np.int32)
        cv2.fillPoly(img, [poly], (60, 60, 60))
        put(img, 'speed', (self.r + 6, self.bot - 4), (150, 150, 150), 0.38)
        # phantom episodes of the virtual wall
        for ep in ctx['summary']['phantoms']:
            if ep['source'] != 'virtual_wall':
                continue
            a = self.u(ctx['bag'].t0 + ep['t_start'])
            b = self.u(ctx['bag'].t0 + ep['t_end'])
            over = img.copy()
            cv2.rectangle(over, (a, self.top), (b, self.bot), (40, 40, 140), -1)
            img = cv2.addWeighted(over, 0.4, img, 0.6, 0)
        self.series(img, [(f['t'], f['strip']) for f in frames], C_STRIP, 2)
        for key, _label, color in SOURCES:
            self.series(img, [(f['t'], frame_value(f, key)) for f in frames], color, 2)
        rep = [(r['end'], r['vw'] if r['vw'] > 0 else math.nan) for r in ctx['rows'] if r]
        self.series(img, rep, (0, 70, 140), 1)
        st = [(a['t'], a['distance'] if a['distance'] is not None else math.nan)
              for a in ctx['stereo_att']]
        self.series(img, st, C_STEREO, 1, dots=True)
        x = self.r + 6
        for i, (label, color) in enumerate([('strip', C_STRIP)]
                                           + [(s[1].replace(' (camera)', ''), s[2])
                                              for s in SOURCES]
                                           + [('stereo', C_STEREO), ('VW replay', (0, 70, 140))]):
            put(img, label[:16], (x, self.top + 12 + 16 * i), color, 0.38)
        put(img, f'm (clipped at {y_max:.0f})', (4, self.top + 4), (130, 130, 130), 0.36)
        self.base = img

    def u(self, t):
        return int(self.l + (t - self.t0) / max(self.t1 - self.t0, 1e-6) * (self.r - self.l))

    def v(self, m):
        m = min(max(m, 0.0), self.y_max)
        return int(self.bot - m / self.y_max * (self.bot - self.top))

    def series(self, img, tv, color, thick, dots=False):
        prev = None
        for t, m in tv:
            if m is None or not math.isfinite(m) or m <= 0:
                prev = None
                continue
            p = (self.u(t), self.v(m))
            if dots:
                cv2.circle(img, p, 1, color, -1)
            elif prev is not None and p[0] - prev[0] < 40:
                cv2.line(img, prev, p, color, thick, cv2.LINE_AA)
            prev = p

    def draw(self, t):
        img = self.base.copy()
        u = self.u(t)
        cv2.line(img, (u, self.top), (u, self.bot), (255, 255, 255), 1)
        return img


def render_video(ctx, path):
    frames = ctx['frames']
    if not frames:
        return None
    view = View()
    chart = Chart(ctx)
    out = []
    for f in frames:
        top = np.hstack([draw_topdown(ctx, f, view), draw_panel(ctx, f)])
        img = np.vstack([top, chart.draw(f['t'])])
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            out.append((f['t'], buf.tobytes()))
    return write_mp4(out, path, fps=int(VIDEO_HZ), crf=23)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', type=Path)
    ap.add_argument('--out', type=Path, default=WS / 'mission_videos' / 'front_phantom')
    ap.add_argument('--map-bag', type=Path, default=None,
                    help='bag whose /slam/map messages stand in for a bag without them '
                         '(same wall clock)')
    ap.add_argument('--no-video', action='store_true')
    cfg = ap.parse_args()
    rclpy.init()
    try:
        map_bag = Bag(cfg.map_bag, topics=['/slam/map']) if cfg.map_bag else None
        for b in cfg.bags:
            run_bag(b, cfg.out, map_bag, not cfg.no_video)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
