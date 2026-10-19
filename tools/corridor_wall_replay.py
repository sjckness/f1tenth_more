#!/usr/bin/env python3
"""Offline re-analysis of a corridor recording through the CURRENT wall pipeline.

Usage (after `source install/setup.bash`):
    python3 tools/corridor_wall_replay.py [--out DIR] [--no-video]
        [--mask auto|recorded|replay] [--t0 S] [--t1 S] [--end-wall-range M]
        [--tracker-steps car|debug] [--compare-summary OLD.summary.json|DIR]
        BAG [BAG ...]

BAG is a bag directory (metadata.yaml + *.db3) or an archive run folder (the
one holding bag/). A split bag's files (<name>_0.db3, _1.db3, ...) are all
read, in split order. Bags are opened read-only (sqlite `mode=ro`). Nothing is
published to a graph: the two nodes are constructed in-process with their
publishers replaced by capture lists, and rclpy runs on the isolated domain
scene_labels_replay sets up, so a live stack never sees the replay.

WHAT IS REPLAYED

  front_clearance_node  masking -> /perception/scene_labels/stats,
      /perception/stereo_columns, /perception/wall_points, /perception/
      stereo_wall. Fed the bag's /tf_static, depth camera_info, depth frames,
      /camera/detection_masks, /camera/detections, /perception/obstacles_2d,
      /camera/image_raw and /scan. The scene mask is RECOMPUTED here (unlike
      front_phantom_replay.CameraReplay, which injects the recorded labels):
      the per-pixel classes are what steps 2 and 4 are about.
  lidar_front_wall_node  wall_observations -> tracker -> the WALL / CLEAR /
      NO_DATA contract. Fed /tf_static, /odometry/filtered, /costmap/
      front_clearance, /camera/detections_3d, /perception/stereo_wall and the
      two stereo inputs, then one /scan per recorded cycle.

TWO LIDAR PASSES, AND THE DIFFERENCE BETWEEN THEM IS THE POINT

  pass "rec"   fed the RECORDED /perception/stereo_columns and
               /perception/wall_points. Its diff against the recorded output
               isolates the LiDAR half: tracker, roles, contract.
  pass "full"  fed the REPLAYED ones. Its diff against "rec" is what the
               recomputed camera half costs, end to end.

TIME BASE: HEADER STAMPS, NOT BAG RECEIPTS. On this recording the writer ran
0.4 s (/scan) to 4.5 s (/perception/stereo_columns) behind the sensor stamps
and wrote in bursts -- `info.txt` prints the per-topic figures. Receipt order
is therefore not arrival order and front_phantom_replay's executor model, which
keys on receipts, cannot be used. Every input is instead fed in HEADER-STAMP
order up to the stamp of the cycle being replayed, and each replayed cycle is
compared against the recorded message carrying the SAME stamp. What this gives
up is the live queue behaviour; what it buys is that every comparison is
like-for-like. The one input with no header, /costmap/front_clearance, is fed
in recorded order.

THE CAMERA HALF CANNOT BE REPRODUCED FRAME FOR FRAME. The live node consumed
the full-rate /zed2/zed_node/depth/depth_registered; a mission bag records the
throttled *_5hz copy (record_zed_depth), and the throttle's choice of frames is
independent of the node's 5 Hz mask tick. Only the frames whose stamp appears
in BOTH the bag and a recorded /perception/stereo_columns message are a
like-for-like comparison; `replay.md` reports that subset separately and it is
the only camera number to believe.

A COLD TRACKER CANNOT REPRODUCE A WARM ONE. The tracker's publish ranking is
length * hits / distance and only the top max_published (4) tracks can become
the FRONT wall, so whether a far wall ahead is reported at all depends on a hit
count that grows with node UPTIME. A bag records the node's output, not its
tracker state, so a replay always starts cold. Every cycle therefore also asks
the same question with the cap lifted (WallReplay._shadow_front) and the
reports say how many WALL -> CLEAR differences that one cap accounts for. Read
those before reading the contract match rate as a code difference.

THE TRACKER RUNS ON THE CAR'S OWN STEPS, FROM THE BAG START. On the car the
tracker processes one scan in lidar_front_wall_wall_tracker_scan_decimation
(4, i.e. ~10 Hz of 40 Hz scans). Which scans those were is read off the
recording, never re-derived from a counter:
  - bags from before cpu-cleanup R2 carry a /perception/virtual_wall/debug
    message on EVERY scan the node processed, with `tracker_skipped` = scans
    skipped since the last tracker step, 0 on a step: the replayed cycles are
    the recorded messages with tracker_skipped == 0;
  - since R2 the debug topic (and /perception/virtual_wall) is published on
    tracker steps only, so every recorded message is a step;
  - a payload without the key (older still, no decimation) is a step too.
The replay node's own gate is then forced open (node._tracker_due -> True) and
only the step scans are fed, so the replayed steps are exactly the car's,
whatever phase the node's counter would have had. The scans between steps are
not fed: the tracker and the contract change only on a step (the per-scan
estimator fields of the payload, lidar_m / stereo_m, are therefore not
like-for-like). virtual_wall_rate_hz is lifted in the replay (its 75 % rule
runs on the live receipt clock; the recorded steps already are the published
ones). A step whose debug message the recorder dropped cannot be identified,
so the replay skips it: run_bag prints how many the tracker_skipped sequence
says are missing, and how many sequence breaks it has. `--tracker-steps debug`
is the pre-v2 behaviour, one tracker step per recorded debug message, which on
a pre-R2 bag runs the tracker four times as often as the car did.
Every step from the bag start is replayed, always. --t0/--t1 (seconds, t = 0
the first /scan stamp) crop only the summary and the videos, so the tracker is
already warm at t0; run_bag prints the warm-up (t0 - first step) and the age
of the car's oldest track at the bag start, the head start the live tracker
had that no replay can have.

THE CLASS MASK IN THE CAMERA VIDEO. --mask auto (default) draws the RECORDED
/perception/scene_labels/classes (mono8, scene_mask.CLASS_* ids; the mission
logger's campaign profile records it) -- what the car computed, independent of
the installed code. --mask replay draws MaskReplay's recomputed mask instead,
labelled REPLAYED. When the chosen mask is not in the bag the panel says MASK
NOT RECORDED (or DEPTH NOT RECORDED: no depth, no replayed mask) and nothing
is tinted. floor_inferred is drawn YELLOW here (scene_mask.CLASS_COLOURS is
not changed; its light green is too close to floor's). Under the image a
stacked bar gives the per-class share of the frame; the top-down chart has the
same as a thin strip over time. The base photo is the raw /camera/image_raw_5hz
or /camera/image_raw when one lies within RAW_PREFER_S of the frame, else the
debug JPEG, desaturated and labelled as such (it is never a mask).
The rectangle verification of both LiDAR passes is fed ONE mask + depth pair
source, never two: the replayed pair when the camera half ran, else the
recorded classes + depth_mm pair if the bag has both (equal stamps only).

THE RECORDER DROPS SCANS. A cycle whose /scan is not in the bag cannot be
replayed at all; `run_bag` prints how many and the reports exclude them.

THE TRUE STRIP RANGE is the nearest /scan return ahead of the bumper inside the
car-width strip, from the bumper -- vehicle_frames.front_side_limit_m(), the one
strip the whole stack shares. It sees only the laser plane (z = 0.19 m).

MOVING WINDOW: |v| from /odometry/filtered above --speed-threshold, padded
--speed-pad each side and merged. The camera video covers the moving window;
the top-down video covers the whole run and marks it.

OUTPUTS per bag in --out:
    <name>.info.txt      topic inventory, rates, latency, the moving window
    <name>.replay.csv    one row per replayed cycle, recorded vs both passes
    <name>.columns.csv   one row per replayed depth frame, column statuses
    <name>.replay.md     the per-topic diffs of step 1
    <name>.summary.md    the headline metrics per pass, per-state time, FRONT
                         episodes, all-WALL error, ID switches, DISAGREE vs
                         range, stage timing, and the parameter evidence
                         (distributions, no changes)
    <name>.summary.json  the same numbers, machine-readable
    <name>.compare.md    with --compare-summary: before/after of the headline
    <name>.camera.mp4    moving window, 5 Hz real time, left image + depth
    <name>.topdown.mp4   whole run, 10 Hz real time, base_link top-down

THE HEADLINE METRICS (summary.json `summary.headline`), each for the passes
  recorded     the car's own /perception/virtual_wall/debug on its tracker
               steps (and /perception/walls for the tracks),
  replay_rec   pass "rec" (recorded stereo inputs, the installed code),
  replay_full  pass "full" (recomputed stereo inputs), when it ran,
over the cycles inside --t0/--t1:
  phantom_front_episodes   WALL episodes (runs of consecutive WALL cycles)
      whose per-cycle error, value - true strip range, has a median below
      -0.5 m, an EMPTY strip counting as -inf: a WALL with nothing in the
      strip is the plainest phantom. Also split by kind (short / empty) and
      counted inside the moving window. A glass wall the laser cannot see
      counts as a phantom too: read the episode list.
  end_wall_error           cycles in the moving window whose true strip range
      is finite and under --end-wall-range (3.0 m): over the WALL ones,
      value - strip (median, p95 and max of |err|); the others are counted as
      misses (CLEAR / NO_DATA while a return sat that close in the strip).
  all_wall_error           every WALL cycle with a finite strip, as the pre-v2
      summary's "end-wall error" was (signed p05/p95, max |err|).
  no_data_pct              NO_DATA share of contract time, all and moving.
  tracks_rho_lt_0_5        CONFIRMED tracks with rho_bl < 0.5 m: recorded from
      /perception/walls (only the published <= 4 tracks), replay both from its
      own published /perception/walls (like-for-like with recorded) and from
      every live track in its debug payload (NOT like-for-like).
"""

import argparse
import bisect
import collections
import csv
import json
import math
import re
import sqlite3
import sys
import time
from pathlib import Path

# Isolation (domain, discovery server, BLAS threads) happens on this import,
# before rclpy loads. front_phantom_replay imports it the same way.
import scene_labels_replay as slr  # noqa: E402
import front_phantom_replay as fpr  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from f1tenth_description.vehicle_frames import base_front_x, front_side_limit_m  # noqa: E402
from f1tenth_logger.mission_debug_video import find_db3, write_mp4  # noqa: E402
from f1tenth_params.param_defaults import get_value  # noqa: E402
from f1tenth_perception import multi_wall_tracker as mwt  # noqa: E402
from f1tenth_perception import scene_mask, wall_fusion  # noqa: E402

WS = Path(__file__).resolve().parents[1]
CAM_HZ = 5.0
TOP_HZ = 10.0
SPEED_THRESHOLD = 0.05
#: One rectangle sample in four is drawn (Debug, the work order).
RECT_SAMPLE_STRIDE = 4
#: What one sample said, in the camera view: at, beyond, against, nothing.
KIND_COLOUR = {wall_fusion.KIND_AT: (90, 220, 90),
               wall_fusion.KIND_BEYOND: (240, 170, 60),
               wall_fusion.KIND_AGAINST: (60, 60, 255),
               wall_fusion.KIND_UNINFORMATIVE: (120, 120, 120)}
#: Line style of a track by its rectangle verdict: solid, dashed, dotted.
VERIFY_DASH = {'verified': 0.0, 'unverified': 0.22, 'rejected': 0.07}
SPEED_PAD = 1.0
LASER_Z = 0.19          # set per bag from /tf_static; the scan plane's height in base_link

DEPTH_TOPICS = ['/zed2/zed_node/depth/depth_registered_5hz',
                '/zed2/zed_node/depth/depth_registered']
#: Raw camera frames first (the campaign profile's 5 Hz copy, then the full
#: topic), the detector's debug JPEG last. See choose_rgb for the rule.
RGB_TOPICS = ['/camera/image_raw_5hz', '/camera/image_raw', '/camera/debug_image/compressed']
#: A raw frame within this many seconds of the video frame beats a nearer JPEG.
RAW_PREFER_S = 0.1
#: The car's own per-pixel class mask (campaign profile) and its depth pair.
REC_CLASSES_TOPIC = '/perception/scene_labels/classes'
REC_DEPTH_MM_TOPIC = '/perception/scene_labels/depth_mm'
#: Big image topics only the videos (and the rectangle fallback) need: kept as
#: references into the bag and decoded on use, never all in memory at once.
LAZY_TOPICS = ('/camera/image_raw_5hz', '/camera/image_raw', REC_CLASSES_TOPIC,
               REC_DEPTH_MM_TOPIC)
#: --end-wall-range default: a strip return closer than this is the end wall.
END_WALL_RANGE_M = 3.0
#: A WALL more than this short of the true strip range is a phantom.
PHANTOM_MARGIN_M = 0.5
#: A confirmed track whose line passes closer than this to base_link.
RHO_BL_CLOSE_M = 0.5

# Everything deserialized. Anything absent is simply reported absent.
TOPICS = [
    '/tf_static', '/tf', '/scan', '/odometry/filtered', '/ekf_global/odometry/filtered',
    '/costmap/front_clearance', '/camera/detections_3d', '/camera/detections',
    '/camera/detection_masks', '/perception/obstacles_2d', '/camera/camera_info',
    '/zed2/zed_node/depth/camera_info', '/perception/stereo_wall',
    '/perception/stereo_columns', '/perception/wall_points', '/perception/scene_labels',
    '/perception/scene_labels/stats', '/perception/wall_observations', '/perception/walls',
    '/perception/virtual_wall', '/perception/virtual_wall/debug', '/perception/wall_planes',
    '/perception/lidar_front_wall', '/mission/status', '/drive',
    REC_CLASSES_TOPIC, REC_DEPTH_MM_TOPIC,
] + DEPTH_TOPICS + RGB_TOPICS

# What step 0 checks for, and why each one matters.
REQUIRED = {
    '/scan': 'the LiDAR half of every observation, and the true strip range',
    '/odometry/filtered': 'places the tracks and the moving window',
    '/tf_static': 'the extrinsics every projection needs',
    '/camera/detection_masks': 'the OBJECT class of the scene mask',
    '/perception/wall_observations': 'the recorded step-3 output to diff against',
    '/perception/walls': 'the recorded tracks to diff against',
    '/perception/virtual_wall/debug': 'the recorded contract to diff against',
    '/perception/stereo_columns': 'the recorded step-2 output to diff against',
}
PREFERRED = {
    '/zed2/zed_node/depth/depth_registered_5hz': 'the only depth a mission bag carries',
    REC_CLASSES_TOPIC: 'the class mask the CAR computed (campaign profile); drawn by --mask auto',
    '/camera/image_raw_5hz': 'the frame YOLO and the mask used (campaign profile): the video base',
    '/costmap/front_clearance': 'the estimator seed',
    '/camera/detections_3d': 'the detection sectors cut out of the scan',
}

#: The producer's own staleness limit (front_clearance_node's watchdog), so
#: the video and the node agree on what "stale" means.
MASK_WATCHDOG_AGE_S = float(get_value('front_clearance_scene_mask_watchdog_age_s'))

SRC_NAMES = {0: 'lidar', 1: 'stereo', 2: 'both'}
STATE_NAMES = {0: 'tentative', 1: 'confirmed', 2: 'coasting'}
STATUS_ORDER = [wall_fusion.STATUS_AGREE, wall_fusion.STATUS_LIDAR_ONLY,
                wall_fusion.STATUS_STEREO_ONLY, wall_fusion.STATUS_DISAGREE,
                wall_fusion.STATUS_EMPTY]
STATUS_COLOUR = {
    wall_fusion.STATUS_AGREE: (90, 220, 90),
    wall_fusion.STATUS_LIDAR_ONLY: (230, 190, 70),
    wall_fusion.STATUS_STEREO_ONLY: (220, 120, 220),
    wall_fusion.STATUS_DISAGREE: (60, 60, 255),
    wall_fusion.STATUS_EMPTY: (110, 110, 110),
}
ROLE_COLOUR = {'front': (60, 60, 255), 'left': (230, 170, 60),
               'right': (60, 210, 230), 'back': (170, 170, 170),
               # Tracked, drawn, a reference wall for nothing.
               'other': (120, 120, 130), '': (120, 120, 130)}
CONTRACT_COLOUR = {'wall': (60, 60, 255), 'clear': (90, 220, 90), 'no_data': (0, 200, 255)}
#: The class colours THIS TOOL draws with: scene_mask.CLASS_COLOURS, except
#: floor_inferred in yellow, which its light green (90, 230, 120) is too close
#: to floor's green to tell apart under a tint. scene_mask itself is unchanged.
CLASS_FLOOR_INFERRED = getattr(scene_mask, 'CLASS_FLOOR_INFERRED', 7)
DRAW_CLASS_COLOURS = dict(scene_mask.CLASS_COLOURS)
DRAW_CLASS_COLOURS[CLASS_FLOOR_INFERRED] = (0, 235, 255)
#: Amber, for "this is missing" notes drawn on a panel.
C_MISSING = (0, 200, 255)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def num(x, nd=3):
    if x is None:
        return None
    x = float(x)
    return None if not math.isfinite(x) else round(x, nd)


def pct(values, q):
    v = [x for x in values if x is not None and math.isfinite(x)]
    return float(np.percentile(v, q)) if v else math.nan


class Capture(slr.Capture):
    """slr.Capture, plus the publisher API the scene-mask tick asks about."""

    def get_subscription_count(self):
        return 1


# ============================================================================
# Bag access -- header stamps first
# ============================================================================

def split_order_key(path):
    """Sort key putting a split bag's files in split order: _2 before _10."""
    name = Path(path).name
    m = re.search(r'_(\d+)\.db3$', name)
    return (int(m.group(1)) if m else -1, name)


def bag_files(path):
    """Every .db3 of a bag (split files in order), as mission_debug_video finds them."""
    found = sorted(find_db3(path), key=split_order_key)
    if not found:
        found = [fpr.bag_db(path)]     # its own FileNotFoundError names the path
    return found


_RO_CONNECTIONS = {}


def ro_connection(db):
    """One read-only sqlite connection per .db3, shared by every LazyMsg."""
    con = _RO_CONNECTIONS.get(str(db))
    if con is None:
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        _RO_CONNECTIONS[str(db)] = con
    return con


def close_ro_connections():
    for con in _RO_CONNECTIONS.values():
        con.close()
    _RO_CONNECTIONS.clear()


class LazyMsg:
    """A big message left in the bag: (db3, row id, class), decoded by load()."""

    __slots__ = ('db', 'rowid', 'cls')

    def __init__(self, db, rowid, cls):
        self.db, self.rowid, self.cls = db, rowid, cls

    def load(self):
        (data,) = ro_connection(self.db).execute(
            'SELECT data FROM messages WHERE id=?', (self.rowid,)).fetchone()
        return deserialize_message(bytes(data), self.cls)


def materialize(msg):
    return msg.load() if isinstance(msg, LazyMsg) else msg


class Src:
    """One bag, read once: per topic a stamp-sorted [(stamp, receipt, msg)].

    `stamp` is the message header's own stamp where it has one, the String
    payload's "stamp" field for the debug topics that carry one, and the
    receipt otherwise (noted in `headerless`). A split bag is read file by
    file in split order. LAZY_TOPICS hold a LazyMsg instead of the message:
    `materialize(r[2])` decodes it.
    """

    def __init__(self, path, topics=TOPICS):
        self.path = Path(path).expanduser()
        self.dbs = bag_files(self.path)
        self.db = self.dbs[0]
        self.name = self.path.name
        self.types, self.counts, self.msgs = {}, collections.Counter(), {}
        self.headerless = set()
        bounds = []
        for db in self.dbs:
            con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            ids = {}
            for i, n, t in con.execute('SELECT id, name, type FROM topics'):
                self.types[n], ids[n] = t, i
            for n, c in con.execute(
                    'SELECT t.name, count(m.id) FROM topics t LEFT JOIN messages m'
                    ' ON m.topic_id = t.id GROUP BY t.id'):
                self.counts[n] += c
            lo, hi = con.execute('SELECT min(timestamp), max(timestamp) FROM messages').fetchone()
            if lo is not None:
                bounds.append((lo * 1e-9, hi * 1e-9))
            for name in topics:
                if name not in ids:
                    continue
                cls = get_message(self.types[name])
                rows = self.msgs.setdefault(name, [])
                for rowid, ts, data in con.execute(
                        'SELECT id, timestamp, data FROM messages WHERE topic_id=?'
                        ' ORDER BY timestamp', (ids[name],)):
                    msg = deserialize_message(bytes(data), cls)
                    stamp = self._stamp(name, msg, ts * 1e-9)
                    if name in LAZY_TOPICS:
                        msg = LazyMsg(db, rowid, cls)
                    rows.append((stamp, ts * 1e-9, msg))
            con.close()
        for rows in self.msgs.values():
            rows.sort(key=lambda r: r[0])
        self.counts = dict(self.counts)
        self.t0 = min((b[0] for b in bounds), default=0.0)
        self.t1 = max((b[1] for b in bounds), default=0.0)
        scans = self.get('/scan')
        self.T0 = scans[0][0] if scans else self.t0

    def _stamp(self, name, msg, receipt):
        header = getattr(msg, 'header', None)
        if header is None and getattr(msg, 'transforms', None):
            header = msg.transforms[0].header
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            return stamp_s(header)
        if type(msg).__name__ == 'String':
            try:
                s = json.loads(msg.data).get('stamp')
                if isinstance(s, (int, float)):
                    return float(s)
            except (ValueError, AttributeError):
                pass
        self.headerless.add(name)
        return receipt

    def get(self, topic):
        return self.msgs.get(topic, [])

    def stamps(self, topic):
        return [r[0] for r in self.get(topic)]

    def rel(self, t):
        return t - self.T0

    def pick(self, topic):
        """The first of a preference list that has messages, or None."""
        for name in topic:
            if self.get(name):
                return name
        return None


# ============================================================================
# Step 0: what is in the bag, and when the car was moving
# ============================================================================

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Poses:
    """Stamped (x, y, yaw, speed) from an odometry topic, looked up by stamp."""

    def __init__(self, rows):
        self.t = np.array([r[0] for r in rows])
        self.x = np.array([r[2].pose.pose.position.x for r in rows])
        self.y = np.array([r[2].pose.pose.position.y for r in rows])
        self.yaw = np.unwrap([yaw_of(r[2].pose.pose.orientation) for r in rows]) \
            if rows else np.zeros(0)
        self.v = np.array([math.hypot(r[2].twist.twist.linear.x, r[2].twist.twist.linear.y)
                           for r in rows])

    def at(self, t):
        if self.t.size == 0:
            return None
        i = int(np.clip(np.searchsorted(self.t, t), 1, self.t.size - 1))
        t0, t1 = self.t[i - 1], self.t[i]
        f = 0.0 if t1 <= t0 else float(np.clip((t - t0) / (t1 - t0), 0.0, 1.0))
        return (float(self.x[i - 1] + f * (self.x[i] - self.x[i - 1])),
                float(self.y[i - 1] + f * (self.y[i] - self.y[i - 1])),
                float(self.yaw[i - 1] + f * (self.yaw[i] - self.yaw[i - 1])))

    def speed(self, t):
        if self.t.size == 0:
            return math.nan
        return float(self.v[int(np.clip(np.searchsorted(self.t, t), 0, self.t.size - 1))])


def moving_window(poses, threshold=SPEED_THRESHOLD, pad=SPEED_PAD):
    """[(t_start, t_end)] where |v| > threshold, padded each side and merged."""
    if poses.t.size == 0:
        return []
    moving = poses.v > threshold
    edges = np.flatnonzero(np.diff(np.r_[0, moving.astype(int), 0]))
    padded = [(poses.t[a] - pad, poses.t[b - 1] + pad)
              for a, b in zip(edges[0::2], edges[1::2])]
    merged = []
    for a, b in padded:
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    return merged


def in_window(window, t):
    return any(a <= t <= b for a, b in window)


def inventory(src, poses, window):
    """The step-0 report: counts, stamp rates, writer latency, the window."""
    lines = [f'bag  {src.path}',
             'db3  ' + '\n     '.join(str(d) for d in src.dbs),
             f'span {src.t1 - src.t0:.2f} s of receipts; t = 0 is the first /scan stamp '
             f'({src.T0:.3f})', '']
    lines.append('WRITER LATENCY. receipt - header stamp. A mission bag written by a loaded')
    lines.append('recorder lags the sensors; where it does, receipt ORDER is not arrival order')
    lines.append('and only the header stamps mean anything.')
    lines.append('')
    head = (f'  {"topic":46s} {"n":>6s} {"stamp Hz":>9s} {"med dt":>8s} {"max dt":>8s}'
            f' {"lat med":>8s} {"lat p95":>8s} {"first":>7s} {"last":>7s} {"in win":>7s}')
    lines.append(head)
    rows = {}
    for name in sorted(src.msgs):
        rec = src.get(name)
        if not rec:
            rows[name] = {'count': 0}
            continue
        st = np.array([r[0] for r in rec])
        lat = np.array([r[1] - r[0] for r in rec])
        dt = np.diff(st)
        win = sum(1 for s in st if in_window(window, s))
        row = {'count': len(rec), 'headerless': name in src.headerless,
               'stamp_hz': round((len(st) - 1) / max(st[-1] - st[0], 1e-9), 2),
               'median_dt_ms': round(float(np.median(dt)) * 1e3, 1) if dt.size else None,
               'max_dt_ms': round(float(dt.max()) * 1e3, 1) if dt.size else None,
               'latency_med_ms': round(float(np.median(lat)) * 1e3, 1),
               'latency_p95_ms': round(float(np.percentile(lat, 95)) * 1e3, 1),
               'first_s': round(src.rel(st[0]), 2), 'last_s': round(src.rel(st[-1]), 2),
               'in_window': win}
        rows[name] = row
        flag = ' (no header: receipt)' if row['headerless'] else ''
        lines.append(
            f'  {name:46s} {row["count"]:6d} {row["stamp_hz"]:9.2f}'
            f' {row["median_dt_ms"] or 0:8.1f} {row["max_dt_ms"] or 0:8.1f}'
            f' {row["latency_med_ms"]:8.1f} {row["latency_p95_ms"]:8.1f}'
            f' {row["first_s"]:7.2f} {row["last_s"]:7.2f} {win:7d}{flag}')
    lines += ['', 'dt and Hz are of the HEADER stamps; lat is receipt - stamp, in ms.', '']

    missing = [(n, why) for n, why in REQUIRED.items() if not src.get(n)]
    thin = [(n, why) for n, why in PREFERRED.items() if not src.get(n)]
    lines.append('REQUIRED TOPICS')
    for name, why in REQUIRED.items():
        ok = 'OK  ' if src.get(name) else 'MISS'
        lines.append(f'  [{ok}] {name:46s} {len(src.get(name)):5d}  {why}')
    lines.append('PREFERRED TOPICS')
    for name, why in PREFERRED.items():
        ok = 'OK  ' if src.get(name) else 'MISS'
        lines.append(f'  [{ok}] {name:46s} {len(src.get(name)):5d}  {why}')
    lines.append('')

    total = sum(b - a for a, b in window)
    lines.append(f'MOVING WINDOW  |v| > {SPEED_THRESHOLD} m/s on /odometry/filtered, '
                 f'padded {SPEED_PAD:.1f} s each side, merged:')
    for a, b in window:
        lines.append(f'    {src.rel(a):7.2f} .. {src.rel(b):7.2f} s   ({b - a:5.2f} s)')
    if poses.t.size:
        mov = poses.v > SPEED_THRESHOLD
        dist = float(np.hypot(np.diff(poses.x), np.diff(poses.y)).sum())
        mean = float(poses.v[mov].mean()) if mov.any() else float('nan')
        lines.append(f'    total {total:.2f} s of {poses.t[-1] - poses.t[0]:.2f} s; '
                     f'path {dist:.2f} m; speed max {poses.v.max():.3f}, '
                     f'mean while moving {mean:.3f} m/s')
    lines.append('')
    win = [[round(src.rel(a), 3), round(src.rel(b), 3)] for a, b in window]
    return '\n'.join(lines), {'topics': rows, 'window': win,
                              'window_total_s': round(total, 2),
                              'missing_required': [m[0] for m in missing],
                              'missing_preferred': [m[0] for m in thin]}


# ============================================================================
# Step 1a: the camera half -- masking -> stereo columns
# ============================================================================

class MaskReplay:
    """front_clearance_node over the bag's depth frames, scene mask recomputed.

    Every recorded depth frame gets a mask tick: the bag's depth rate (~4 Hz
    after the throttle) is already below the node's scene_mask_rate_hz, so
    gating the tick would only drop frames the analysis needs. The old
    scene_labels labeller runs at its own rate, as on the car, because its
    /perception/stereo_wall is an input to the LiDAR node.
    """

    CAPTURES = ('front_distance_pub', 'front_wall_pub', 'front_clearance_pub',
                'front_blocked_pub', 'wall_debug_pub', 'scene_labels_pub', 'scene_debug_pub',
                'stereo_wall_pub', 'unclassified_pub', 'front_distance_raw_pub',
                'bg_pixel_count_pub', 'scene_mask_image_pub', 'wall_points_pub',
                'scene_mask_stats_pub', 'stereo_columns_pub',
                'scene_mask_classes_pub', 'scene_mask_depth_pub')

    def __init__(self, src, depth_topic):
        from cv_bridge import CvBridge
        from f1tenth_perception.front_clearance_node import FrontClearanceNode
        self.src, self.topic = src, depth_topic
        self.bridge = CvBridge()
        self.clock = [0.0]
        node = FrontClearanceNode(parameter_overrides=fpr.front_clearance_params())
        node._now_seconds = lambda: self.clock[0]
        self.caps = {}
        for attr in self.CAPTURES:
            if getattr(node, attr, None) is not None:
                self.caps[attr] = Capture()
                setattr(node, attr, self.caps[attr])
        self.node = node

    def run(self):
        src, node = self.src, self.node
        for t, _r, m in src.get('/tf_static'):
            self.clock[0] = t
            node._tf_static_callback(m)
        for name in ('/zed2/zed_node/depth/camera_info', '/camera/camera_info'):
            if src.get(name):
                node._camera_info_callback(src.get(name)[0][2])
                self.info_topic = name
                break
        else:
            self.info_topic = None

        # A recorder (mask) sink: the per-pixel classes are not published, and
        # steps 2 and 4 are about them. scene_mask is looked up on the module
        # at call time, so replacing the attribute reaches the node's call.
        sink = {}
        original = scene_mask.classify

        def classify(*a, **kw):
            t_start = time.perf_counter()
            labels, z = original(*a, **kw)
            sink['labels'] = labels
            sink['z'] = z
            sink['classify_ms'] = (time.perf_counter() - t_start) * 1e3
            return labels, z

        scene_mask.classify = classify
        feed = {'/camera/detections': node._detections_callback,
                '/camera/detection_masks': node._masks_callback,
                '/perception/obstacles_2d': node._obstacles_callback,
                '/camera/image_raw': node._image_callback,
                '/scan': node._scan_callback}
        events = [(t, 0, name, m) for name in feed for t, _r, m in src.get(name)
                  if src.get(name)]
        events += [(t, 1, self.topic, m) for t, _r, m in src.get(self.topic)]
        events.sort(key=lambda e: (e[0], e[1]))
        out = []
        try:
            for t, _k, name, m in events:
                self.clock[0] = t
                if name != self.topic:
                    feed[name](materialize(m))
                    continue
                node._next_scene_mask = -math.inf
                t_start = time.perf_counter()
                node.depth_callback(m)
                ms = (time.perf_counter() - t_start) * 1e3
                out.append(self._collect(m, t, ms, sink))
        finally:
            scene_mask.classify = original
        self.rows = out
        return out

    def _collect(self, depth_msg, t, ms, sink):
        caps = self.caps
        cols = caps['stereo_columns_pub'].pop()
        stats = caps['scene_mask_stats_pub'].pop()
        cloud = caps['wall_points_pub'].pop()
        classes = caps['scene_mask_classes_pub'].pop() \
            if 'scene_mask_classes_pub' in caps else []
        depth_mm = caps['scene_mask_depth_pub'].pop() \
            if 'scene_mask_depth_pub' in caps else []
        caps['scene_mask_image_pub'].pop()
        wall = caps['stereo_wall_pub'].pop()
        labels = caps['scene_labels_pub'].pop()
        dbg = caps['wall_debug_pub'].pop()
        for attr in ('front_distance_pub', 'front_wall_pub', 'front_clearance_pub',
                     'front_blocked_pub', 'scene_debug_pub', 'unclassified_pub',
                     'front_distance_raw_pub', 'bg_pixel_count_pub'):
            if attr in caps:
                caps[attr].pop()
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        row = {
            'stamp': t, 'depth': depth, 'depth_msg': depth_msg,
            'mask': sink.get('labels'), 'z': sink.get('z'),
            'columns': cols[-1] if cols else None,
            'cloud': cloud[-1] if cloud else None,
            # E2's two inputs. Whenever the camera half ran, the rectangle
            # half of EVERY pass is fed this recomputed pair (and only it:
            # WallReplay never feeds a recorded pair alongside), so the
            # rec/full distinction stays about the stereo COLUMNS.
            'classes': classes[-1] if classes else None,
            'depth_mm': depth_mm[-1] if depth_mm else None,
            'scene_labels': labels[-1] if labels else None,
            'stats': json.loads(stats[-1].data) if stats else None,
            'stereo_wall': json.loads(wall[-1].data) if wall else None,
            'front_wall': json.loads(dbg[-1].data) if dbg else None,
            'depth_callback_ms': ms, 'classify_ms': sink.get('classify_ms'),
        }
        if row['mask'] is not None:
            # Enumerate the CLASSES, never a hardcoded count: the mask grew a
            # seventh (floor_inferred) and a range() would have dropped it
            # silently from every table below.
            row['frac'] = {scene_mask.CLASS_NAMES[i]: float((row['mask'] == i).mean())
                           for i in scene_mask.CLASS_NAMES}
        return row

    def close(self):
        self.node.destroy_node()


# ============================================================================
# Step 1b: the LiDAR half -- observations -> tracker -> contract
# ============================================================================

LFW_FEED = {'/odometry/filtered': '_odom_cb', '/costmap/front_clearance': '_seed_cb',
            '/camera/detections_3d': '_detections_cb', '/perception/stereo_wall':
            '_stereo_wall_cb', '/perception/stereo_columns': '_stereo_columns_cb',
            '/perception/wall_points': '_wall_points_cb',
            '/perception/scene_labels/classes': '_mask_classes_cb',
            '/perception/scene_labels/depth_mm': '_mask_depth_cb'}


def tracker_steps(debug):
    """Which recorded debug payloads are tracker steps, and what the record says.

    `debug` is the stamp-ordered list of /perception/virtual_wall/debug dicts.
    A step is a payload with tracker_skipped == 0 (a missing key counts as 0):
    pre-R2 bags carry one payload per scan, skipped > 0 on the held ones; R2
    bags carry steps only. Returns (flags, info). info['format'] is
    'per_scan' when any payload has skipped > 0, else 'per_step'. For a
    per_scan record, 'missing_steps' counts payloads with skipped = k > 0 whose
    predecessor had skipped >= k (a step went by unrecorded) and 'breaks' every
    place where the sequence is not 0, 1, 2, ... (a dropped payload).
    """
    skipped = []
    for d in debug:
        try:
            skipped.append(int(d.get('tracker_skipped') or 0))
        except (TypeError, ValueError):
            skipped.append(0)
    flags = [k == 0 for k in skipped]
    decim = collections.Counter(
        d.get('tracker_decimation') for d in debug if d.get('tracker_decimation') is not None)
    info = {'payloads': len(debug), 'steps': sum(flags),
            'format': 'per_scan' if any(k > 0 for k in skipped) else 'per_step',
            'decimation': decim.most_common(1)[0][0] if decim else None,
            'missing_steps': 0, 'breaks': 0}
    if info['format'] == 'per_scan':
        for prev, k in zip(skipped, skipped[1:]):
            if k > 0 and prev >= k:
                info['missing_steps'] += 1
            if k != prev + 1 and k != 0:
                info['breaks'] += 1
            elif k == 0 and info['decimation'] and prev != info['decimation'] - 1:
                info['breaks'] += 1
    return flags, info


def step_gaps(stamps, factor=1.5):
    """How many step-to-step gaps exceed factor x the median gap, and that median."""
    dt = np.diff(np.asarray(sorted(stamps), float))
    if dt.size == 0:
        return 0, None
    med = float(np.median(dt))
    return int(np.count_nonzero(dt > factor * med)), med


def recorded_cycles(src, steps='car'):
    """([(scan_stamp, scan, recorded debug dict)] for every replayed cycle, info).

    /perception/virtual_wall/debug carries its scan's own header stamp, so the
    cycle set and its order come from the stamps alone -- no receipt-order
    pairing, which this bag's writer latency would make guesswork. steps='car'
    keeps the car's tracker steps only (tracker_steps); 'debug' keeps every
    recorded payload (the pre-v2 behaviour).
    """
    scans = {}
    for t, _r, m in src.get('/scan'):
        scans[round(t, 6)] = m
    out = []
    for t, _r, m in src.get('/perception/virtual_wall/debug'):
        d = json.loads(m.data)
        stamp = float(d.get('stamp', t))
        scan = scans.get(round(stamp, 6))
        out.append({'stamp': stamp, 'scan': scan, 'rec_dbg': d})
    out.sort(key=lambda c: c['stamp'])
    flags, info = tracker_steps([c['rec_dbg'] for c in out])
    info['mode'] = steps
    if steps == 'car':
        out = [c for c, f in zip(out, flags) if f]
    info['replayed_cycles'] = len(out)
    info['long_gaps'], info['median_step_s'] = step_gaps([c['stamp'] for c in out])
    return out, info


def index_by_stamp(rows):
    return {round(r[0], 6): r[2] for r in rows}


class WallReplay:
    """lidar_front_wall_node over the recorded cycles, fed in stamp order.

    `stereo` picks which /perception/stereo_columns and /perception/wall_points
    the node is fed: 'rec' the recorded ones, 'full' the ones MaskReplay just
    recomputed. The clock is the CURRENT CYCLE's scan stamp for every callback
    in that cycle, which is how the live node saw its inputs -- everything it
    had, it had just received. The two stamp-gated ages (detection sectors,
    stereo column pairing) read header stamps, not the clock, so they are
    reproduced exactly either way.
    """

    def __init__(self, src, stereo='rec', replayed=None):
        self.src, self.mode = src, stereo
        self.clock = [0.0]
        node, caps = slr.make_node(True, self.clock)
        for attr in ('walls_pub', 'walls_markers_pub', 'observations_pub', 'doors_pub'):
            if getattr(node, attr, None) is not None:
                caps[attr] = Capture()
                setattr(node, attr, caps[attr])
        # Only the car's tracker-step scans are fed (recorded_cycles), so the
        # gate is forced open: every fed scan IS a step, whatever phase the
        # node's own counter (or WALL_TRACKER_DECIMATION) would give it.
        node._tracker_due = lambda: True
        # The live publish cap's 75 % rule ran on the receipt clock; the
        # recorded steps already are the published ones, so lift it here or
        # a step a little early on the STAMP clock would publish no payload.
        if hasattr(node, 'virtual_wall_rate_hz'):
            node.virtual_wall_rate_hz = 0.0
        self.node, self.caps = node, caps
        # /tf_static is transient-local and the live node had it from its start,
        # long before any bag began: give it all up front.
        for t, _r, m in src.get('/tf_static'):
            self.clock[0] = t
            node._tf_static_cb(m)
        # Odometry is fed on its own cursor: see _feed_odom.
        self.odom = [(t, m) for t, _r, m in src.get('/odometry/filtered')]
        self.oi = 0
        feeds = []
        for name in LFW_FEED:
            if name in ('/odometry/filtered', REC_CLASSES_TOPIC, REC_DEPTH_MM_TOPIC):
                # The mask pair has ONE source, chosen below; feeding the
                # recorded pair too would interleave two masks into the
                # rectangle check, the last one fed winning.
                continue
            if name in ('/perception/stereo_columns', '/perception/wall_points') \
                    and stereo == 'full':
                continue
            for t, _r, m in src.get(name):
                feeds.append((t, name, m))
        if stereo == 'full':
            for row in (replayed or []):
                if row['columns'] is not None:
                    feeds.append((row['stamp'], '/perception/stereo_columns', row['columns']))
                if row['cloud'] is not None:
                    feeds.append((row['stamp'], '/perception/wall_points', row['cloud']))
        # The class mask and its depth: the replay's pair when the camera
        # half ran (see MaskReplay._collect), else the recorded pair when the
        # bag has both halves of it on equal stamps. Both passes get the same
        # source, so the rectangle verification runs in both.
        self.rect_source = None
        for row in (replayed or []):
            if row.get('classes') is not None and row.get('depth_mm') is not None:
                self.rect_source = 'replayed'
                feeds.append((row['stamp'], REC_CLASSES_TOPIC, row['classes']))
                feeds.append((row['stamp'], REC_DEPTH_MM_TOPIC, row['depth_mm']))
        if self.rect_source is None and src.get(REC_CLASSES_TOPIC) \
                and src.get(REC_DEPTH_MM_TOPIC):
            depth_by = {round(t, 6): m for t, _r, m in src.get(REC_DEPTH_MM_TOPIC)}
            for t, _r, m in src.get(REC_CLASSES_TOPIC):
                if round(t, 6) in depth_by:
                    self.rect_source = 'recorded'
                    feeds.append((t, REC_CLASSES_TOPIC, m))
                    feeds.append((t, REC_DEPTH_MM_TOPIC, depth_by[round(t, 6)]))
        feeds.sort(key=lambda f: f[0])
        self.feeds = feeds
        self.fi = 0

    def _feed_until(self, t):
        while self.fi < len(self.feeds) and self.feeds[self.fi][0] <= t:
            _s, name, msg = self.feeds[self.fi]
            getattr(self.node, LFW_FEED[name])(materialize(msg))
            self.fi += 1

    def _feed_odom(self, t):
        """Odometry up to AND INCLUDING the first sample after the scan stamp.

        The tracker places every wall with PoseHistory.at(scan_stamp), which
        interpolates between two samples and extrapolates by at most 0.1 s. The
        live callback ran after its scan had arrived, so a sample past the scan
        stamp was already in its queue; cutting the feed at the scan stamp
        instead makes every cycle after a >0.1 s odometry gap return no pose
        and be skipped, which the live node did only 4 times in this bag.
        """
        while self.oi < len(self.odom):
            stamp, msg = self.odom[self.oi]
            self.node._odom_cb(msg)
            self.oi += 1
            if stamp > t:
                break

    def run(self, cycles):
        node = self.node
        sink = {}
        original = wall_fusion.cross_check_profile

        def cross_check_profile(*a, **kw):
            check = original(*a, **kw)
            sink['check'] = check
            sink['scan_cols'] = np.asarray(a[0])
            sink['scan_depths'] = np.asarray(a[1])
            sink['stereo_m'] = np.asarray(a[2])
            return check

        wall_fusion.cross_check_profile = cross_check_profile
        original_rect = wall_fusion.verify_rectangles

        def verify_rectangles(*a, **kw):
            # The debug views draw one sample in four (the work order's own
            # stride), so only that quarter is kept per cycle.
            detail = {}
            kw['detail'] = detail
            kw['detail_stride'] = RECT_SAMPLE_STRIDE
            verdicts = original_rect(*a, **kw)
            sink['rect_detail'] = detail
            sink['rect_segments'] = list(a[0])
            return verdicts

        wall_fusion.verify_rectangles = verify_rectangles
        rows = []
        try:
            for cycle in cycles:
                if cycle['scan'] is None:
                    rows.append(None)
                    continue
                self.clock[0] = cycle['stamp']
                self._feed_until(cycle['stamp'])
                self._feed_odom(cycle['stamp'])
                sink.clear()
                t_start = time.perf_counter()
                node._scan_cb(cycle['scan'])
                ms = (time.perf_counter() - t_start) * 1e3
                rows.append(self._collect(cycle, ms, sink))
        finally:
            wall_fusion.cross_check_profile = original
            wall_fusion.verify_rectangles = original_rect
        self.rows = rows
        return rows

    def _shadow_front(self, cycle):
        """The FRONT track the publish cap hides, if any.

        front_track() only looks at tracker.published(), which is at most
        max_published (4) tracks ranked by length * hits / distance. A far wall
        dead ahead is SHORT and FAR, so that ranking demotes it below the long
        near side walls unless its hit count is large -- and hits grow with node
        UPTIME, not with evidence quality. This re-asks the same question with
        the cap lifted and reports what the cap cost, using the node's own
        is_front and front_distances so the answer is the pipeline's, not ours.
        """
        tracker = self.node.wall_tracker
        if tracker is None:
            return None
        car = self.node._poses.at(cycle['stamp'])
        if car is None:
            return None
        front_x = self.node.front_x
        cap = tracker.p.max_published
        published = [t.id for t in tracker.published(car)]
        visible = tracker.front_track(car, front_x)
        try:
            tracker.p.max_published = 10 ** 6
            uncapped = tracker.front_track(car, front_x)
            order = [t.id for t in tracker.published(car)]
        finally:
            tracker.p.max_published = cap
        if uncapped is None or (visible is not None and uncapped.id == visible.id):
            return None
        _perp, d_ray = mwt.front_distances(uncapped, car, front_x, tracker.p.ray_margin_m)
        return {'id': int(uncapped.id), 'd_ray': float(d_ray),
                'rank': order.index(uncapped.id) if uncapped.id in order else None,
                'cap': int(cap), 'n_tracks': len(order), 'published': published,
                'hits': int(uncapped.hits), 'length': float(uncapped.length)}

    def _collect(self, cycle, ms, sink):
        caps = self.caps
        vw = caps['virtual_pub'].pop()
        dbg = caps['virtual_debug_pub'].pop()
        caps['estimate_pub'].pop()
        caps['fit_pub'].pop()
        caps['planes_pub'].pop()
        walls = caps['walls_pub'].pop() if 'walls_pub' in caps else []
        if 'walls_markers_pub' in caps:
            caps['walls_markers_pub'].pop()
        obs = caps['observations_pub'].pop() if 'observations_pub' in caps else []
        doors = caps['doors_pub'].pop() if 'doors_pub' in caps else []
        d = json.loads(dbg[-1].data) if dbg else {}
        profile = self.node._stereo_columns
        return {
            'stamp': cycle['stamp'], 'scan': cycle['scan'],
            'vw': float(vw[-1].data) if vw else math.nan, 'dbg': d,
            'state': d.get('state'), 'value': d.get('distance_m'),
            'walls': walls[-1] if walls else None,
            'obs': obs[-1] if obs else None,
            # /perception/doors of this cycle (DoorArray), None with door
            # detection off or on a node that predates it.
            'doors': doors[-1] if doors else None,
            'check': sink.get('check'), 'scan_cols': sink.get('scan_cols'),
            'scan_depths': sink.get('scan_depths'), 'stereo_m': sink.get('stereo_m'),
            'profile_stamp': stamp_s(profile[1].header) if profile else None,
            # The node's own verdict on the profile, not the tool's: the age it
            # measured and the weight it gave it (wall_fusion.age_weight).
            'stereo_age': d.get('stereo_age_s'),
            'stereo_weight': d.get('stereo_weight'),
            'scan_ms': ms, 'tracker_ms': d.get('tracker_ms'),
            'rect_ms': d.get('rect_ms'), 'rect_age': d.get('rect_age_s'),
            'rect_weight': d.get('rect_weight'),
            'rect_informative': d.get('rect_informative'),
            'roles': d.get('roles'),
            'negative_killed': d.get('negative_killed'),
            'objects_behind_wall': d.get('objects_behind_wall'),
            'rect_verdicts': dict(self.node._rect_verdicts),
            'rect_detail': sink.get('rect_detail'),
            'rect_track_ids': list(self.node._rect_verdicts),
            'extract_dropped': d.get('extract_dropped'),
            'shadow_front': self._shadow_front(cycle),
            'rec_dbg': cycle['rec_dbg'],
        }

    def close(self):
        self.node.destroy_node()


# ============================================================================
# Step 1c: the diffs
# ============================================================================

def diff(pairs):
    """median / p95 / max |a - b| over the finite pairs, and how many there were."""
    d = [abs(a - b) for a, b in pairs
         if a is not None and b is not None and math.isfinite(a) and math.isfinite(b)]
    if not d:
        return {'n': 0}
    return {'n': len(d), 'median': round(float(np.median(d)), 4),
            'p95': round(float(np.percentile(d, 95)), 4), 'max': round(float(max(d)), 4)}


def obs_signature(msg):
    """A stamp-free summary of a WallObservationArray, for comparing two runs."""
    if msg is None:
        return None
    return {'n': len(msg.observations), 'rejected': int(msg.n_rejected),
            'disagree': int(msg.n_disagree_columns),
            'by_source': collections.Counter(SRC_NAMES[o.source] for o in msg.observations),
            'd': sorted(round(float(o.d), 3) for o in msg.observations)}


def match_observations(rec, rep, tol=0.05):
    """Pair two observation arrays by (normal, d); returns matched, rec-only, rep-only."""
    if rec is None or rep is None:
        return [], list(getattr(rec, 'observations', [])), list(getattr(rep, 'observations', []))
    free = list(rep.observations)
    matched, only_rec = [], []
    for a in rec.observations:
        best, bi = None, None
        for i, b in enumerate(free):
            cost = abs(a.d - b.d) + abs(a.nx - b.nx) + abs(a.ny - b.ny)
            if cost < tol * 3 and (best is None or cost < best):
                best, bi = cost, i
        if bi is None:
            only_rec.append(a)
        else:
            matched.append((a, free.pop(bi)))
    return matched, only_rec, free


def compare(src, cycles, rows_rec, rows_full, mask_rows):
    """Everything step 1 asks: per-topic diff, median/max, and why they differ."""
    rec_obs = index_by_stamp(src.get('/perception/wall_observations'))
    rec_walls = index_by_stamp(src.get('/perception/walls'))
    rec_cols = index_by_stamp(src.get('/perception/stereo_columns'))
    out = {'passes': {}}

    # ---- stereo columns: only the shared stamps are like-for-like -----------
    shared, rep_only = [], 0
    for row in mask_rows:
        cols = row['columns']
        if cols is None:
            continue
        key = round(stamp_s(cols.header), 6)
        if key in rec_cols:
            shared.append((rec_cols[key], cols))
        else:
            rep_only += 1
    col_diff = {'recorded': len(rec_cols), 'replayed': len(mask_rows),
                'shared_stamps': len(shared), 'replay_only': rep_only}
    if shared:
        dist, npx, zlo, zhi, nfin = [], [], [], [], []
        for a, b in shared:
            da = np.asarray(a.distance_m, float)
            db = np.asarray(b.distance_m, float)
            both = np.isfinite(da) & np.isfinite(db)
            dist += list(np.abs(da[both] - db[both]))
            nfin.append((int(np.isfinite(da).sum()), int(np.isfinite(db).sum())))
            npx += list(np.abs(np.asarray(a.n_px, float) - np.asarray(b.n_px, float)))
            za, zb = np.asarray(a.z_min, float), np.asarray(b.z_min, float)
            ok = np.isfinite(za) & np.isfinite(zb)
            zlo += list(np.abs(za[ok] - zb[ok]))
            za, zb = np.asarray(a.z_max, float), np.asarray(b.z_max, float)
            ok = np.isfinite(za) & np.isfinite(zb)
            zhi += list(np.abs(za[ok] - zb[ok]))
        col_diff.update({
            'distance_m': {'n': len(dist), 'median': round(float(np.median(dist)), 5),
                           'p95': round(float(np.percentile(dist, 95)), 5),
                           'max': round(float(max(dist)), 5)} if dist else {'n': 0},
            'n_px': {'median': round(float(np.median(npx)), 2),
                     'max': round(float(max(npx)), 2)} if npx else {},
            'z_min': {'max': round(float(max(zlo)), 5)} if zlo else {},
            'z_max': {'max': round(float(max(zhi)), 5)} if zhi else {},
            'finite_columns': {'recorded_median': float(np.median([a for a, _b in nfin])),
                               'replay_median': float(np.median([b for _a, b in nfin]))},
        })
    out['stereo_columns'] = col_diff

    for name, rows in (('rec', rows_rec), ('full', rows_full)):
        if rows is None:
            continue
        vw, obs_n, obs_rej, obs_dis, track_n, front_d = [], [], [], [], [], []
        state_match = state_total = 0
        state_conf = collections.Counter()
        src_match = collections.Counter()
        obs_pairs = []
        shadow_explains = shadow_any = 0
        shadow_err = []
        for cycle, row in zip(cycles, rows):
            if row is None:
                continue
            rec = cycle['rec_dbg']
            state_total += 1
            key = (rec.get('state'), row['state'])
            state_conf[key] += 1
            if rec.get('state') == row['state']:
                state_match += 1
            vw.append((rec.get('distance_m'), row['value']))
            shadow = row.get('shadow_front')
            if shadow:
                shadow_any += 1
            if rec.get('state') == 'wall' and row['state'] == 'clear' and shadow:
                ref = rec.get('distance_m')
                if ref is not None and math.isfinite(shadow['d_ray']):
                    shadow_err.append(abs(shadow['d_ray'] - ref))
                    if abs(shadow['d_ray'] - ref) < 0.25:
                        shadow_explains += 1
            front_d.append((rec.get('front_d_ray_m'), row['dbg'].get('front_d_ray_m')))
            k = round(row['stamp'], 6)
            if k in rec_obs and row['obs'] is not None:
                a, b = rec_obs[k], row['obs']
                obs_n.append((len(a.observations), len(b.observations)))
                obs_rej.append((a.n_rejected, b.n_rejected))
                obs_dis.append((a.n_disagree_columns, b.n_disagree_columns))
                matched, only_a, only_b = match_observations(a, b)
                obs_pairs.append((len(matched), len(only_a), len(only_b)))
                for oa, ob in matched:
                    src_match[(SRC_NAMES[oa.source], SRC_NAMES[ob.source])] += 1
            if k in rec_walls and row['walls'] is not None:
                track_n.append((len(rec_walls[k].tracks), len(row['walls'].tracks)))
        out['passes'][name] = {
            'cycles': state_total,
            'contract_state_match': state_match,
            'contract_state_match_frac': round(state_match / max(state_total, 1), 4),
            'contract_confusion': {f'{a}->{b}': n for (a, b), n in sorted(state_conf.items())},
            'virtual_wall_m': diff(vw),
            'front_d_ray_m': diff(front_d),
            'n_observations': diff([(float(a), float(b)) for a, b in obs_n]),
            'n_rejected': diff([(float(a), float(b)) for a, b in obs_rej]),
            'n_disagree_columns': diff([(float(a), float(b)) for a, b in obs_dis]),
            'n_tracks': diff([(float(a), float(b)) for a, b in track_n]),
            'observation_pairing': {
                'matched': sum(p[0] for p in obs_pairs),
                'recorded_only': sum(p[1] for p in obs_pairs),
                'replay_only': sum(p[2] for p in obs_pairs)},
            'source_confusion': {f'{a}->{b}': n for (a, b), n in sorted(src_match.items())},
            'publish_cap': {
                'cycles_with_hidden_front': shadow_any,
                'wall_to_clear': state_conf.get(('wall', 'clear'), 0),
                'wall_to_clear_explained_by_cap': shadow_explains,
                'hidden_front_error_median': num(float(np.median(shadow_err)))
                if shadow_err else None},
        }
    return out


# ============================================================================
# The timeline both videos and the summary are built on
# ============================================================================

def latest(stamps, t):
    """Index of the last entry at or before t, or None."""
    i = bisect.bisect_right(stamps, t) - 1
    return i if i >= 0 else None


def nearest_index(times, t):
    """Index of the entry of a sorted, non-empty list nearest to t."""
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return 0
    if i >= len(times):
        return len(times) - 1
    return i if times[i] - t < t - times[i - 1] else i - 1


def choose_rgb(t, sources, prefer_raw_s=RAW_PREFER_S):
    """(source index, message index, signed age t - stamp) of the base image, or None.

    `sources` is [(sorted stamps, is_raw)]. NEAREST in time, not latest-before
    (at under 1.5 Hz a latest-before rule shows a frame seconds stale while a
    newer one sits half a period away), but not simply the nearest of all: a
    raw frame within prefer_raw_s wins over a nearer debug JPEG. Otherwise the
    nearest non-raw frame, and with none of those the nearest raw one anyway.
    """
    best = {True: None, False: None}
    for k, (times, raw) in enumerate(sources):
        if not len(times):
            continue
        j = nearest_index(times, t)
        age = t - times[j]
        cur = best[bool(raw)]
        if cur is None or abs(age) < abs(cur[2]):
            best[bool(raw)] = (k, j, age)
    raw, other = best[True], best[False]
    if raw is not None and abs(raw[2]) <= prefer_raw_s:
        return raw
    return other if other is not None else raw


def choose_mask_source(mode, has_recorded, has_replay, has_depth=True):
    """(which mask the camera video tints with, or None; the note when None).

    'auto' and 'recorded' draw the car's recorded mask and nothing else;
    'replay' draws the recomputed one, which needs depth. Never a silent
    fallback from one to the other: a frame without its mask says so.
    """
    if mode == 'replay':
        if has_replay:
            return 'replay', None
        return None, ('DEPTH NOT RECORDED: no replayed mask' if not has_depth
                      else 'NO REPLAYED MASK')
    if has_recorded:
        return 'recorded', None
    return None, 'MASK NOT RECORDED'


#: Bytes per pixel of the 8-bit encodings this tool decodes.
_ENC_CHANNELS = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3, '8UC3': 3,
                 'bgra8': 4, 'rgba8': 4, '8UC4': 4}


def image_array(data, height, width, step, encoding):
    """An 8-bit Image's pixels as an (h, w, c) uint8 array, or None.

    Honours `step`: a row may be padded past width * channels, which a plain
    reshape to (h, w, -1) would smear into the next row.
    """
    ch = _ENC_CHANNELS.get(encoding)
    if ch is None or height <= 0 or width <= 0:
        return None
    step = int(step) if step else width * ch
    if step < width * ch:
        return None
    buf = np.frombuffer(bytes(data), np.uint8)
    need = (height - 1) * step + width * ch
    if buf.size < need:
        return None
    if buf.size < height * step:
        buf = np.concatenate([buf, np.zeros(height * step - buf.size, np.uint8)])
    return buf[:height * step].reshape(height, step)[:, :width * ch].reshape(height, width, ch)


def class_fractions(labels, n_classes):
    """{class id: share of the pixels} for ids 0..n_classes-1 (and any above, if present)."""
    labels = np.asarray(labels)
    if labels.size == 0:
        return {}
    counts = np.bincount(labels.ravel().astype(np.int64), minlength=n_classes)
    return {i: float(c) / labels.size for i, c in enumerate(counts)}


def fraction_segments(fracs, order, width):
    """[(class id, x0, x1)] of a stacked bar `width` px wide, in `order`, no gaps."""
    out, acc, x_prev = [], 0.0, 0
    for cls in order:
        f = float(fracs.get(cls, 0.0))
        if f <= 0.0:
            continue
        acc += f
        x = int(round(min(acc, 1.0) * width))
        if x > x_prev:
            out.append((cls, x_prev, x))
            x_prev = x
    return out


class MaskSource:
    """The class mask the camera video tints with, per --mask (module docstring)."""

    def __init__(self, src, mask_rows, mode='auto', has_depth=True):
        self.rec = src.get(REC_CLASSES_TOPIC)
        self.rec_t = [r[0] for r in self.rec]
        self.rep = [r for r in mask_rows if r.get('mask') is not None]
        self.rep_t = [r['stamp'] for r in self.rep]
        self.mode = mode
        self.source, self.note = choose_mask_source(
            mode, bool(self.rec), bool(self.rep), has_depth)
        self._last = (None, None)
        self._fracs = {}

    def labels(self, i):
        """The (h, w) uint8 class image of mask i of the chosen source, or None."""
        if self.source is None:
            return None
        if self.source == 'replay':
            return self.rep[i]['mask']
        if self._last[0] == i:
            return self._last[1]
        msg = materialize(self.rec[i][2])
        arr = image_array(msg.data, msg.height, msg.width, msg.step, msg.encoding)
        labels = None if arr is None else arr[:, :, 0]
        self._last = (i, labels)
        return labels

    def lookup(self, t):
        """{'i', 'stamp', 'age', 'frac'} of the mask held at t (latest at or before)."""
        if self.source is None:
            return None
        times = self.rec_t if self.source == 'recorded' else self.rep_t
        i = latest(times, t)
        if i is None:
            return None
        if i not in self._fracs:
            labels = self.labels(i)
            self._fracs[i] = (None if labels is None else
                              class_fractions(labels, max(scene_mask.CLASS_NAMES) + 1))
        return {'i': i, 'stamp': times[i], 'age': t - times[i], 'frac': self._fracs[i]}


def laser_pose_of(src):
    """(x, y, yaw) of the scan frame in base_link, from /tf_static."""
    to_base = slr.static_tf([m for _s, _r, m in src.get('/tf_static')])
    pose = to_base('laser')
    if pose is None:
        return (0.305, 0.0, 0.0)
    rot, trans = pose
    return (float(trans[0]), float(trans[1]), float(math.atan2(rot[1, 0], rot[0, 0])))


def build_timeline(src, rows, mask_rows, poses, window, front_x, half, hz, span=None,
                   masks=None):
    """One frame per 1/hz over `span` (default: the whole run), zero-order hold.

    Nothing in this bag runs at the video rate -- the wall cycles are ~4 Hz,
    the depth ~4 Hz, the RGB under 1 Hz -- so each frame holds the newest of
    everything at or before its time and carries that input's AGE, which the
    overlays print. A frame is not evidence that anything was recomputed at
    that instant.
    """
    laser = laser_pose_of(src)
    scans = src.get('/scan')
    scan_t = [r[0] for r in scans]
    rows = [r for r in rows if r]
    row_t = [r['stamp'] for r in rows]
    mask_t = [r['stamp'] for r in mask_rows]
    rec_dbg = [(t, json.loads(m.data)) for t, _r, m in src.get('/perception/virtual_wall/debug')]
    rec_t = [t for t, _d in rec_dbg]
    clear = src.get('/costmap/front_clearance')
    clear_t = [r[0] for r in clear]
    rgb_names = [n for n in RGB_TOPICS if src.get(n)]
    rgb_sources = [([r[0] for r in src.get(n)], not n.endswith('/compressed'))
                   for n in rgb_names]
    masks2d = src.get('/camera/detection_masks')
    masks2d_t = [r[0] for r in masks2d]
    t0 = span[0] if span else (scan_t[0] if scan_t else src.T0)
    t1 = span[1] if span else (scan_t[-1] if scan_t else src.t1)
    frames = []
    for k in range(int((t1 - t0) * hz) + 1):
        t = t0 + k / hz
        f = {'t': t, 'rel': src.rel(t), 'moving': in_window(window, t),
             'speed': poses.speed(t), 'pose': poses.at(t), 'laser': laser}
        i = latest(scan_t, t)
        if i is not None:
            scan = scans[i][2]
            pts = fpr.scan_xy(scan, laser)
            rng, pt = fpr.strip_nearest(pts, front_x, half)
            f.update(scan=scan, scan_age=t - scan_t[i], points=pts,
                     strip=rng, strip_point=pt)
        i = latest(row_t, t)
        if i is not None:
            f['row'] = rows[i]
            f['row_age'] = t - row_t[i]
        i = latest(rec_t, t)
        if i is not None:
            f['rec'] = rec_dbg[i][1]
            f['rec_age'] = t - rec_t[i]
        i = latest(mask_t, t)
        if i is not None:
            f['mask_row'] = mask_rows[i]
            f['mask_age'] = t - mask_t[i]
        i = latest(clear_t, t)
        if i is not None:
            f['clearance'] = float(clear[i][2].data)
            f['clearance_age'] = t - clear_t[i]
        # The base image: see choose_rgb. The signed age is printed on the frame.
        pick = choose_rgb(t, rgb_sources)
        if pick is not None:
            k, j, age = pick
            f['rgb_age'], f['rgb'], f['rgb_topic'] = age, src.get(rgb_names[k])[j][2], \
                rgb_names[k]
        if masks is not None:
            f['cls'] = masks.lookup(t)
        i = latest(masks2d_t, t)
        if i is not None:
            f['masks2d'] = masks2d[i][2]
            f['masks2d_age'] = t - masks2d_t[i]
        frames.append(f)
    return frames


# ============================================================================
# Step 4: the summary
# ============================================================================

def state_times(entries):
    """{state: seconds} from [(stamp, state)], the last entry held for the median dt."""
    if not entries:
        return {}
    entries = sorted(entries)
    st = np.array([e[0] for e in entries])
    dt = np.diff(st)
    tail = float(np.median(dt)) if dt.size else 0.0
    out = collections.Counter()
    for (t, state), d in zip(entries, list(dt) + [tail]):
        out[state] += float(d)
    return {k: round(v, 2) for k, v in out.items()}


def episodes(entries, predicate, min_len=0.0):
    """[(t_start, t_end)] of runs where predicate(entry) holds."""
    out, start, prev = [], None, None
    for t, payload in sorted(entries):
        if predicate(payload):
            if start is None:
                start = t
        elif start is not None:
            if prev is not None and prev - start >= min_len:
                out.append((start, prev))
            start = None
        prev = t
    if start is not None and prev is not None and prev - start >= min_len:
        out.append((start, prev))
    return out


def in_span(span, t):
    return span is None or span[0] <= t <= span[1]


def summarize(src, cycles, rows, mask_rows, poses, window, front_x, half, span=None):
    """Per-state time, FRONT episodes, all-WALL error, ID switches, DISAGREE, timing.

    Over `span` (absolute stamps; None = the whole bag): the replay ran from
    the bag start regardless, only what is counted here is cropped.
    """
    def get(topic):
        return [r for r in src.get(topic) if in_span(span, r[0])]

    rows = [r for r in rows if r and in_span(span, r['stamp'])]
    mask_rows = [r for r in mask_rows if in_span(span, r['stamp'])]
    duration = (span[1] - span[0]) if span else (src.t1 - src.t0)
    scans = get('/scan')
    laser = laser_pose_of(src)
    strip = {}
    for t, _r, m in scans:
        pts = fpr.scan_xy(m, laser)
        strip[round(t, 6)] = fpr.strip_nearest(pts, front_x, half)[0]
    strip_t = sorted(strip)

    def strip_at(t):
        i = latest(strip_t, t)
        return strip[strip_t[i]] if i is not None else math.nan

    out = {}
    # ---- per-state time, recorded and replayed -----------------------------
    rec = [(json.loads(m.data)['stamp'], json.loads(m.data)['state'])
           for _t, _r, m in get('/perception/virtual_wall/debug')]
    rep = [(r['stamp'], r['state']) for r in rows if r]
    out['state_time_s'] = {'recorded': state_times(rec), 'replay': state_times(rep)}
    out['state_time_s']['recorded_moving'] = state_times(
        [e for e in rec if in_window(window, e[0])])

    # ---- FRONT in the open corridor ----------------------------------------
    rec_rows = [json.loads(m.data) for _t, _r, m in get('/perception/virtual_wall/debug')]
    rec_rows.sort(key=lambda d: d['stamp'])
    front_eps = []
    for t_a, t_b in episodes([(d['stamp'], d) for d in rec_rows],
                             lambda d: d['state'] == 'wall'):
        sel = [d for d in rec_rows if t_a <= d['stamp'] <= t_b]
        truth = [strip_at(d['stamp']) for d in sel]
        err = [d['distance_m'] - s for d, s in zip(sel, truth) if math.isfinite(s)]
        front_eps.append({
            'start_s': round(src.rel(t_a), 2), 'end_s': round(src.rel(t_b), 2),
            'duration_s': round(t_b - t_a, 2), 'n': len(sel),
            'value_min': num(min(d['distance_m'] for d in sel)),
            'value_max': num(max(d['distance_m'] for d in sel)),
            'true_strip_min': num(np.nanmin(truth)) if truth else None,
            'error_median': num(float(np.median(err))) if err else None,
            'error_p95': num(pct(err, 95)) if err else None,
            'moving': any(in_window(window, d['stamp']) for d in sel)})
    out['front_episodes'] = front_eps
    # A phantom is a WALL the strip does not support: the laser sees nothing
    # in the strip out to `margin` beyond the reported wall.
    phantom = [e for e in front_eps
               if e['true_strip_min'] is not None and e['error_median'] is not None
               and e['error_median'] < -0.5]
    out['phantom_front_episodes'] = phantom
    clear_rows = [d for d in rec_rows if d['state'] == 'clear']
    clear_truth = [strip_at(d['stamp']) for d in clear_rows]
    clear_truth = [s for s in clear_truth if math.isfinite(s)]
    out['clear_state'] = {
        'n': len(clear_rows), 'true_strip_min': num(min(clear_truth)) if clear_truth else None,
        'true_strip_median': num(float(np.median(clear_truth))) if clear_truth else None,
        'n_with_strip_under_5m': sum(1 for s in clear_truth if s < 5.0),
        'n_with_strip_under_2m': sum(1 for s in clear_truth if s < 2.0)}

    # ---- end-wall error vs the true strip range ----------------------------
    for name, source in (('recorded', [(d['stamp'], d['state'], d['distance_m'])
                                       for d in rec_rows]),
                         ('replay', [(r['stamp'], r['state'], r['value'])
                                     for r in rows if r])):
        err = [v - strip_at(t) for t, s, v in source
               if s == 'wall' and v is not None and math.isfinite(strip_at(t))]
        out.setdefault('end_wall_error_m', {})[name] = {
            'n': len(err), 'median': num(float(np.median(err))) if err else None,
            'p05': num(pct(err, 5)), 'p95': num(pct(err, 95)),
            'max_abs': num(max((abs(e) for e in err), default=math.nan))}

    # ---- track ID switches and stereo-only tracks --------------------------
    for name, msgs in (('recorded', [m for _t, _r, m in get('/perception/walls')]),
                       ('replay', [r['walls'] for r in rows if r and r['walls']])):
        ids, born, roles = set(), [], collections.defaultdict(collections.Counter)
        stereo_only, both, lidar = set(), set(), set()
        seen = set()
        role_switches = 0
        confirmed_lifetime = {}
        for msg in msgs:
            now = set()
            for tr in msg.tracks:
                ids.add(tr.id)
                now.add(tr.id)
                roles[tr.id][tr.role] += 1
                if tr.source == 1:
                    stereo_only.add(tr.id)
                elif tr.source == 2:
                    both.add(tr.id)
                else:
                    lidar.add(tr.id)
                if tr.state == 1:
                    confirmed_lifetime[tr.id] = max(confirmed_lifetime.get(tr.id, 0.0), tr.age)
            born += [i for i in now - seen]
            seen |= now
        for i, counter in roles.items():
            role_switches += max(len(counter) - 1, 0)
        out.setdefault('tracks', {})[name] = {
            'distinct_ids': len(ids), 'births': len(born),
            'births_per_minute': round(len(born) / max(
                duration / 60.0, 1e-9), 2),
            'role_switches': role_switches,
            'source_lidar': len(lidar - both - stereo_only),
            'source_both': len(both), 'source_stereo_only': len(stereo_only),
            'stereo_only_ids': sorted(stereo_only),
            'max_confirmed_age_s': num(max(confirmed_lifetime.values(), default=math.nan))}

    # ---- DISAGREE rate vs range --------------------------------------------
    edges = [0, 1, 2, 3, 4, 6, 8, 12, 16, 25]
    buckets = {f'{a}-{b}': collections.Counter() for a, b in zip(edges, edges[1:])}
    resid = {k: [] for k in buckets}
    cycles_with_profile = 0
    for r in rows:
        if not r or r['check'] is None:
            continue
        cycles_with_profile += 1
        check = r['check']
        lidar_m, stereo_m = check['lidar_m'], check['stereo_m']
        status = check['status']
        both = np.isfinite(lidar_m) & np.isfinite(stereo_m)
        for a, b in zip(edges, edges[1:]):
            key = f'{a}-{b}'
            sel = both & (lidar_m >= a) & (lidar_m < b)
            if not sel.any():
                continue
            for value, nm in wall_fusion.STATUS_NAMES.items():
                buckets[key][nm] += int(np.count_nonzero(status[sel] == value))
            resid[key] += list(np.abs(stereo_m[sel] - lidar_m[sel]))
    out['disagree_vs_range'] = {}
    for key, counter in buckets.items():
        n_both = counter['agree'] + counter['disagree']
        out['disagree_vs_range'][key] = {
            'n_both': n_both,
            'disagree': counter['disagree'],
            'rate': round(counter['disagree'] / n_both, 4) if n_both else None,
            'abs_residual_median': num(float(np.median(resid[key]))) if resid[key] else None,
            'abs_residual_p95': num(pct(resid[key], 95)),
            'abs_residual_p99': num(pct(resid[key], 99))}
    out['cycles_with_stereo_profile'] = {
        'replay': cycles_with_profile, 'cycles': sum(1 for r in rows if r)}

    # ---- stereo pairing, the reason the profile is usually absent ----------
    col_stamps = sorted(r[0] for r in get('/perception/stereo_columns'))
    gaps = []
    for r in rows:
        if not r:
            continue
        i = latest(col_stamps, r['stamp'])
        best = min((abs(r['stamp'] - s) for s in col_stamps), default=math.nan)
        gaps.append({'held': r['profile_stamp'],
                     'age_held': (r['stamp'] - r['profile_stamp'])
                     if r['profile_stamp'] else None,
                     'age_best': best,
                     'age_latest': (r['stamp'] - col_stamps[i]) if i is not None else None})
    # What the LIVE node managed, read straight off the recording: a cycle used
    # a profile iff some observation carries a column span, or the message
    # counts disagreeing columns or rejections. The replay holds the latest
    # profile BY STAMP and the live node held the latest ARRIVED, which is
    # older, so the replay's figure is an upper bound on the live one.
    rec_obs_msgs = [m for _t, _r, m in get('/perception/wall_observations')]
    rec_used = sum(1 for m in rec_obs_msgs
                   if m.n_disagree_columns or m.n_rejected
                   or any(o.col_end > o.col_begin for o in m.observations))
    max_age = float(get_value('lidar_front_wall_stereo_profile_max_age_s'))
    held = [g['age_held'] for g in gaps if g['age_held'] is not None]
    best = [g['age_best'] for g in gaps
            if g['age_best'] is not None and math.isfinite(g['age_best'])]
    out['stereo_pairing'] = {
        'max_age_s': max_age, 'cycles': len(gaps),
        'held_age_median': num(float(np.median(held))) if held else None,
        'held_age_p95': num(pct(held, 95)),
        'within_limit_held': sum(1 for a in held if abs(a) <= max_age),
        'best_possible_within_limit': sum(1 for a in best if a <= max_age),
        'best_age_median': num(float(np.median(best))) if best else None,
        'recorded_cycles': len(rec_obs_msgs),
        'recorded_cycles_using_a_profile': rec_used,
        'recorded_observations_rejected': int(sum(m.n_rejected for m in rec_obs_msgs))}

    # ---- E2/E3: the per-track table, and where the two checks disagree ----
    seen = {}
    for r in rows:
        if not r or not r['dbg'].get('tracks'):
            continue
        for t in r['dbg']['tracks']:
            row = seen.setdefault(int(t['id']), {
                'id': int(t['id']), 'first': r['stamp'], 'last': r['stamp'],
                'cycles': 0, 'roles': collections.Counter(),
                'states': collections.Counter(),
                'verify': collections.Counter(), 'glass': 0, 'occluded': 0,
                'max_len': 0.0, 'max_hits': 0, 'max_miss': 0,
                'cov': [], 'sup': [], 'beyond': [], 'is_front': 0,
                'source': collections.Counter(), 'rho': []})
            row['last'] = r['stamp']
            row['cycles'] += 1
            row['roles'][t['role'] or '?'] += 1
            row['states'][t['state'] or '?'] += 1
            row['verify'][t['verify']] += 1
            row['source'][t['source']] += 1
            row['glass'] += bool(t['glass'])
            row['occluded'] += bool(t['occluded'])
            row['is_front'] += bool(t['is_front'])
            row['max_len'] = max(row['max_len'], t['length_m'] or 0.0)
            row['max_hits'] = max(row['max_hits'], t['hits'])
            row['max_miss'] = max(row['max_miss'], t['miss'])
            if t['rho_bl'] is not None:
                row['rho'].append(t['rho_bl'])
            for key, field in (('cov', 'rect_coverage'), ('sup', 'rect_support'),
                               ('beyond', 'rect_beyond_share')):
                if t[field] is not None:
                    row[key].append(t[field])
    out['per_track'] = []
    for row in sorted(seen.values(), key=lambda d: d['id']):
        out['per_track'].append({
            'id': row['id'], 'role': row['roles'].most_common(1)[0][0],
            'roles': dict(row['roles']), 'lifetime_s': num(row['last'] - row['first']),
            'cycles': row['cycles'], 'state': row['states'].most_common(1)[0][0],
            'verify': row['verify'].most_common(1)[0][0],
            'verify_counts': dict(row['verify']),
            'glass_cycles': row['glass'], 'occluded_cycles': row['occluded'],
            'front_cycles': row['is_front'],
            'source': row['source'].most_common(1)[0][0],
            'length_m': num(row['max_len']), 'hits': row['max_hits'],
            'max_miss': row['max_miss'],
            'rho_bl_median': num(float(np.median(row['rho']))) if row['rho'] else None,
            # The MAX as well as the median: the payload repeats the last
            # verdict on every cycle, and the rectangle runs at the mask's
            # ~3.4 Hz against the tracker's 30, so a median over cycles is
            # mostly the zeros of a track born before its first verdict.
            'coverage_median': num(float(np.median(row['cov']))) if row['cov'] else None,
            'coverage_max': num(max(row['cov'])) if row['cov'] else None,
            'support_median': num(float(np.median(row['sup']))) if row['sup'] else None,
            'support_max': num(max(row['sup'])) if row['sup'] else None,
            'beyond_max': num(max(row['beyond'])) if row['beyond'] else None,
            'beyond_median': num(float(np.median(row['beyond']))) if row['beyond'] else None})

    # How often the two checks disagree, at the CYCLE level -- the only level
    # at which both are recorded: the columns verdict belongs to an
    # OBSERVATION and the rectangle's to a TRACK, and nothing records which
    # observation matched which track.
    cells = collections.Counter()
    rect_cycles = 0
    for r in rows:
        if not r or not r['obs']:
            continue
        tracks = r['dbg'].get('tracks') or []
        informative = [t for t in tracks if (t['rect_coverage'] or 0.0) >= 0.25]
        if not informative:
            continue
        rect_cycles += 1
        cols_reject = int(r['obs'].n_rejected) > 0
        cols_agree = any(o.col_end > o.col_begin and o.source == wall_fusion.SOURCE_BOTH
                         for o in r['obs'].observations)
        rect_reject = any(t['verify'] == 'rejected' for t in informative)
        rect_verify = any(t['verify'] == 'verified' for t in informative)
        cells[(('reject' if cols_reject else 'agree' if cols_agree else 'quiet'),
               ('reject' if rect_reject else 'verify' if rect_verify else 'quiet'))] += 1
    out['columns_vs_rectangle'] = {
        'cycles_with_an_informative_rectangle': rect_cycles,
        'cells': {f'columns_{a}/rect_{b}': n for (a, b), n in sorted(cells.items())},
        'disagreements': sum(n for (a, b), n in cells.items()
                             if (a, b) in (('reject', 'verify'), ('agree', 'reject')))}

    # ---- what the publish cap hides ---------------------------------------
    hidden = [r['shadow_front'] for r in rows if r and r.get('shadow_front')]
    cap_rows = [r for r in rows if r]
    out['publish_cap'] = {
        'max_published': hidden[0]['cap'] if hidden else None,
        'cycles': len(cap_rows),
        'cycles_with_a_hidden_front': len(hidden),
        'hidden_d_ray_median': num(float(np.median([h['d_ray'] for h in hidden])))
        if hidden else None,
        'hidden_rank_median': num(float(np.median(
            [h['rank'] for h in hidden if h['rank'] is not None])), 1) if hidden else None,
        'hidden_hits_median': num(float(np.median([h['hits'] for h in hidden])), 1)
        if hidden else None,
        'hidden_length_median': num(float(np.median([h['length'] for h in hidden])))
        if hidden else None,
        'tracks_held_median': num(float(np.median([h['n_tracks'] for h in hidden])), 1)
        if hidden else None,
    }
    rec_front_ages = []
    for _t, _r, m in get('/perception/walls'):
        rec_front_ages += [tr.age for tr in m.tracks if tr.is_front]
    out['publish_cap']['recorded_front_track_age_s'] = {
        'n': len(rec_front_ages),
        'median': num(float(np.median(rec_front_ages))) if rec_front_ages else None,
        'max': num(max(rec_front_ages)) if rec_front_ages else None}

    # ---- timing per stage ---------------------------------------------------
    def stage(values):
        v = [x for x in values if x is not None and math.isfinite(x)]
        return {'n': len(v), 'median_ms': num(float(np.median(v)), 2) if v else None,
                'p95_ms': num(pct(v, 95), 2)} if v else {'n': 0}

    out['timing_ms'] = {
        'replay_scene_mask_classify': stage(r.get('classify_ms') for r in mask_rows),
        'replay_depth_callback': stage(r.get('depth_callback_ms') for r in mask_rows),
        'replay_mask_tick_node': stage((r['stats'] or {}).get('ms') for r in mask_rows),
        'replay_inferred_floor': stage(
            ((r['stats'] or {}).get('inferred_floor') or {}).get('ms') for r in mask_rows),
        'replay_scan_callback': stage(r['scan_ms'] for r in rows if r),
        'replay_tracker': stage(r['tracker_ms'] for r in rows if r),
        'replay_rect_verify': stage(r.get('rect_ms') for r in rows if r),
        'recorded_mask_tick_node': stage(
            json.loads(m.data).get('ms')
            for _t, _r, m in get('/perception/scene_labels/stats')),
        'recorded_tracker': stage(
            json.loads(m.data).get('tracker_ms')
            for _t, _r, m in get('/perception/virtual_wall/debug')),
    }
    out['true_strip_range_m'] = {
        'scans': len(strip), 'finite': sum(1 for v in strip.values() if math.isfinite(v)),
        'min': num(np.nanmin(list(strip.values()))),
        'median': num(float(np.nanmedian(list(strip.values())))),
        'max': num(np.nanmax(list(strip.values())))}
    return out


# ============================================================================
# Step 4: the headline metrics, per pass (module docstring, THE HEADLINE METRICS)
# ============================================================================

def wall_runs(states):
    """[(i0, i1)] inclusive index runs of consecutive 'wall' states."""
    out, start = [], None
    for i, st in enumerate(states):
        if st == 'wall':
            if start is None:
                start = i
        elif start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(states) - 1))
    return out


def classify_wall_episode(values, truths, margin=PHANTOM_MARGIN_M):
    """(phantom?, kind or None, median finite error or None) of one WALL episode.

    Per cycle the error is value - true strip range, and -inf when the strip
    is EMPTY (truth NaN / None: nothing out to the scan's range, the plainest
    phantom). Phantom = median error < -margin, so an episode is a phantom
    when at least half its cycles had an empty strip, or the median WALL sat
    more than `margin` short of the laser. kind: 'empty' when at least half
    the cycles had an empty strip, else 'short'.
    """
    err, empty = [], 0
    for v, s in zip(values, truths):
        if v is None or not math.isfinite(float(v)):
            continue
        if s is None or not math.isfinite(float(s)):
            err.append(-math.inf)
            empty += 1
        else:
            err.append(float(v) - float(s))
    finite = [e for e in err if math.isfinite(e)]
    med_finite = float(np.median(finite)) if finite else None
    if not err:
        return False, None, med_finite
    phantom = float(np.median(err)) < -margin
    kind = ('empty' if 2 * empty >= len(err) else 'short') if phantom else None
    return phantom, kind, med_finite


def state_shares(times, states, max_dt=0.5):
    """{state: share of the time} from per-cycle states, each held to the next.

    A hold is capped at max_dt so a gap (a dropped record, the space between
    two moving windows) is not credited to whichever state preceded it; the
    last cycle is held for the median step. None states are not counted.
    """
    pairs = sorted((t, st) for t, st in zip(times, states) if st is not None)
    if not pairs:
        return {}
    ts = np.array([p[0] for p in pairs], float)
    dt = np.diff(ts)
    tail = float(np.median(dt)) if dt.size else 1.0
    hold = np.minimum(np.r_[dt, tail], max_dt)
    total = collections.Counter()
    for (_t, st), h in zip(pairs, hold):
        total[st] += float(h)
    norm = sum(total.values())
    return {k: v / norm for k, v in total.items()} if norm > 0 else {}


def error_stats(err):
    """n, signed median, p95 and max of |err| over the finite errors."""
    v = [float(e) for e in err if e is not None and math.isfinite(float(e))]
    if not v:
        return {'n': 0, 'median_m': None, 'abs_p95_m': None, 'abs_max_m': None}
    a = np.abs(np.array(v))
    return {'n': len(v), 'median_m': num(float(np.median(v))),
            'abs_p95_m': num(float(np.percentile(a, 95))), 'abs_max_m': num(float(a.max()))}


def close_tracks(per_cycle, limit=RHO_BL_CLOSE_M):
    """Confirmed tracks whose line passes within `limit` of base_link.

    per_cycle: one list per cycle of (track id, confirmed?, rho_bl). Returns
    the distinct ids, their count and the number of cycles holding one.
    """
    ids, cycles = set(), 0
    for tracks in per_cycle:
        hit = False
        for tid, confirmed, rho in tracks:
            if confirmed and rho is not None and math.isfinite(float(rho)) \
                    and abs(float(rho)) < limit:
                ids.add(int(tid))
                hit = True
        cycles += hit
    return {'n_ids': len(ids), 'ids': sorted(ids), 'cycles': cycles}


def pass_headline(times, states, values, truths, moving, end_range=END_WALL_RANGE_M,
                  margin=PHANTOM_MARGIN_M, t_ref=0.0):
    """The headline block of one pass, from its per-cycle (t, state, value, strip, moving)."""
    eps = []
    for i0, i1 in wall_runs(states):
        phantom, kind, med = classify_wall_episode(values[i0:i1 + 1], truths[i0:i1 + 1],
                                                   margin)
        eps.append({'start_s': round(times[i0] - t_ref, 2), 'end_s': round(times[i1] - t_ref, 2),
                    'cycles': i1 - i0 + 1, 'phantom': phantom, 'kind': kind,
                    'error_median_m': num(med), 'moving': any(moving[i0:i1 + 1])})
    phantoms = [e for e in eps if e['phantom']]
    end_err, end_cycles, miss = [], 0, collections.Counter()
    all_err = []
    for st, v, s, mv in zip(states, values, truths, moving):
        finite = s is not None and math.isfinite(float(s))
        if st == 'wall' and finite and v is not None and math.isfinite(float(v)):
            all_err.append(float(v) - float(s))
        if not (mv and finite and float(s) < end_range):
            continue
        end_cycles += 1
        if st == 'wall' and v is not None and math.isfinite(float(v)):
            end_err.append(float(v) - float(s))
        else:
            miss[str(st)] += 1
    shares = state_shares(times, states)
    shares_mv = state_shares([t for t, m in zip(times, moving) if m],
                             [st for st, m in zip(states, moving) if m])
    return {
        'cycles': len(states), 'cycles_without_state': sum(1 for st in states if st is None),
        'wall_episodes': len(eps),
        'phantom_front_episodes': len(phantoms),
        'phantom_front_episodes_moving': sum(1 for e in phantoms if e['moving']),
        'phantom_front_by_kind': dict(collections.Counter(e['kind'] for e in phantoms)),
        'phantom_episode_list': phantoms,
        'end_wall_error': dict(error_stats(end_err), cycles=end_cycles,
                               miss_cycles=sum(miss.values()), miss_by_state=dict(miss)),
        'all_wall_error': {'n': len(all_err), 'p05_m': num(pct(all_err, 5)),
                           'p95_m': num(pct(all_err, 95)),
                           'max_abs_m': num(max((abs(e) for e in all_err), default=math.nan))},
        'no_data_pct': num(100.0 * shares.get('no_data', 0.0), 2) if shares else None,
        'no_data_pct_moving': num(100.0 * shares_mv.get('no_data', 0.0), 2)
        if shares_mv else None,
    }


def strip_truth(src, front_x, half):
    """strip_at(t): the true strip range of the last /scan at or before t (NaN: empty)."""
    laser = laser_pose_of(src)
    stamps, ranges = [], []
    for t, _r, m in src.get('/scan'):
        stamps.append(t)
        ranges.append(fpr.strip_nearest(fpr.scan_xy(m, laser), front_x, half)[0])

    def strip_at(t):
        i = latest(stamps, t)
        return ranges[i] if i is not None else math.nan
    return strip_at


def headline(src, cycles, rows_by_pass, window, front_x, half, span=None,
             end_range=END_WALL_RANGE_M):
    """summary['headline']: the metrics an A/B is judged on, for every pass."""
    strip_at = strip_truth(src, front_x, half)
    passes = {}
    keep = [i for i, c in enumerate(cycles) if in_span(span, c['stamp'])]

    def block(entries, walls, live=None):
        entries = sorted(entries, key=lambda e: e[0])
        times = [e[0] for e in entries]
        out = pass_headline(times, [e[1] for e in entries], [e[2] for e in entries],
                            [strip_at(t) for t in times],
                            [in_window(window, t) for t in times], end_range,
                            t_ref=src.T0)
        out['tracks_rho_lt_0_5'] = close_tracks(walls)
        if live is not None:
            out['tracks_rho_lt_0_5_all_live'] = close_tracks(live)
        return out

    rec_walls = [[(tr.id, tr.state == 1, tr.rho_bl) for tr in m.tracks]
                 for t, _r, m in src.get('/perception/walls') if in_span(span, t)]
    passes['recorded'] = block(
        [(cycles[i]['stamp'], cycles[i]['rec_dbg'].get('state'),
          cycles[i]['rec_dbg'].get('distance_m')) for i in keep], rec_walls)
    passes['recorded']['source'] = ('/perception/virtual_wall/debug on the car\'s tracker '
                                    'steps; tracks from /perception/walls (published <= 4)')
    for name, rows in rows_by_pass.items():
        if rows is None:
            continue
        sel = [rows[i] for i in keep if rows[i] is not None]
        if not sel:
            continue
        key = f'replay_{name}'
        passes[key] = block(
            [(r['stamp'], r['state'], r['value']) for r in sel],
            [[(tr.id, tr.state == 1, tr.rho_bl) for tr in r['walls'].tracks]
             for r in sel if r['walls'] is not None],
            [[(t.get('id'), t.get('state') == 'confirmed', t.get('rho_bl'))
              for t in (r['dbg'].get('tracks') or [])] for r in sel])
        passes[key]['source'] = (f'replay pass "{name}" ('
                                 + ('recorded' if name == 'rec' else 'recomputed')
                                 + ' stereo inputs); tracks_rho_lt_0_5 from its own '
                                 '/perception/walls (like-for-like with recorded), '
                                 '_all_live from every live track of its payload')
    return {
        'span_s': [round(src.rel(span[0]), 3), round(src.rel(span[1]), 3)] if span else None,
        'definitions': {
            'phantom_front_episodes': f'WALL episodes whose median (value - true strip) < '
                                      f'-{PHANTOM_MARGIN_M} m, an empty strip counting as '
                                      '-inf; kind empty/short',
            'end_wall_error': f'moving-window cycles with a true strip range < {end_range} m: '
                              'value - strip over the WALL ones (|err| p95 / max); the rest '
                              'are miss_cycles',
            'all_wall_error': 'every WALL cycle with a finite strip (the pre-v2 '
                              '"end-wall error"): signed p05/p95, max |err|',
            'no_data_pct': 'NO_DATA share of contract time (holds capped at 0.5 s), all and '
                           'moving window',
            'tracks_rho_lt_0_5': f'distinct CONFIRMED track ids with |rho_bl| < '
                                 f'{RHO_BL_CLOSE_M} m; recorded and replay published '
                                 'tracks are like-for-like, _all_live is not'},
        'passes': passes}


# ---- --compare-summary ------------------------------------------------------

#: (metric, how to read it) of the before/after table, in print order.
COMPARE_METRICS = [
    ('phantom_front_episodes', 'count, lower is better'),
    ('phantom_front_episodes_moving', 'count in the moving window'),
    ('end_wall_err_abs_p95_m', 'end-wall abs err p95 [m]'),
    ('end_wall_err_abs_max_m', 'end-wall abs err max [m]'),
    ('end_wall_miss_cycles', 'end-wall cycles not WALL'),
    ('all_wall_err_p95_m', 'all-WALL signed err p95 [m]'),
    ('all_wall_err_max_abs_m', 'all-WALL abs err max [m]'),
    ('no_data_pct', 'NO_DATA % of time'),
    ('no_data_pct_moving', 'NO_DATA % while moving'),
    ('tracks_rho_lt_0_5', 'confirmed published tracks, rho_bl < 0.5 m'),
    ('tracks_rho_lt_0_5_all_live', 'confirmed live tracks (replay only)'),
]


def flat_headline(doc):
    """({pass: {metric: value}}, {(pass, metric): note}) from a summary.json document.

    A v2 summary carries summary.headline. A pre-v2 one does not, and what it
    has is mapped with a note: its phantom count is the RECORDED pass only and
    never counted an empty strip, its "end-wall error" is the all-WALL error,
    and its replay ran the tracker on every recorded debug message.
    """
    summary = (doc or {}).get('summary') or {}
    head = summary.get('headline')
    out, notes = {}, {}
    if head:
        for name, p in (head.get('passes') or {}).items():
            ew, aw = p.get('end_wall_error') or {}, p.get('all_wall_error') or {}
            row = {'phantom_front_episodes': p.get('phantom_front_episodes'),
                   'phantom_front_episodes_moving': p.get('phantom_front_episodes_moving'),
                   'end_wall_err_abs_p95_m': ew.get('abs_p95_m'),
                   'end_wall_err_abs_max_m': ew.get('abs_max_m'),
                   'end_wall_miss_cycles': ew.get('miss_cycles'),
                   'all_wall_err_p95_m': aw.get('p95_m'),
                   'all_wall_err_max_abs_m': aw.get('max_abs_m'),
                   'no_data_pct': p.get('no_data_pct'),
                   'no_data_pct_moving': p.get('no_data_pct_moving'),
                   'tracks_rho_lt_0_5': (p.get('tracks_rho_lt_0_5') or {}).get('n_ids')}
            if p.get('tracks_rho_lt_0_5_all_live') is not None:
                row['tracks_rho_lt_0_5_all_live'] = p['tracks_rho_lt_0_5_all_live'].get('n_ids')
            out[name] = row
        return out, notes
    legacy = {'recorded': 'recorded', 'replay': 'replay_rec'}
    st = summary.get('state_time_s') or {}
    for old, name in legacy.items():
        row = {}
        ew = (summary.get('end_wall_error_m') or {}).get(old) or {}
        row['all_wall_err_p95_m'] = ew.get('p95')
        row['all_wall_err_max_abs_m'] = ew.get('max_abs')
        times = st.get(old) or {}
        if times and sum(times.values()) > 0:
            row['no_data_pct'] = round(100.0 * times.get('no_data', 0.0)
                                       / sum(times.values()), 2)
            notes[(name, 'no_data_pct')] = 'before: pre-v2 time base'
        out[name] = row
    if 'phantom_front_episodes' in summary:
        out['recorded']['phantom_front_episodes'] = len(summary['phantom_front_episodes'])
        notes[('recorded', 'phantom_front_episodes')] = \
            'before: pre-v2 definition (empty strip NOT counted)'
    mv = st.get('recorded_moving') or {}
    if mv and sum(mv.values()) > 0:
        out['recorded']['no_data_pct_moving'] = round(
            100.0 * mv.get('no_data', 0.0) / sum(mv.values()), 2)
    notes[('replay_rec', '*')] = 'before: pre-v2 replay (tracker on every recorded debug msg)'
    return out, notes


def compare_rows(before, after, notes=None):
    """[[metric, pass, before, after, after - before, note]] for COMPARE_METRICS."""
    notes = notes or {}
    rows = []
    for metric, _how in COMPARE_METRICS:
        for name in sorted(set(before) | set(after)):
            a = (before.get(name) or {}).get(metric)
            b = (after.get(name) or {}).get(metric)
            if a is None and b is None:
                continue
            delta = (round(b - a, 4) if isinstance(a, (int, float))
                     and isinstance(b, (int, float)) else None)
            note = '; '.join(n for n in (notes.get((name, metric)), notes.get((name, '*')))
                             if n)
            rows.append([metric, name, a, b, delta, note])
    return rows


# ============================================================================
# Step 4: parameter evidence. Distributions and proposals. NOTHING is changed.
# ============================================================================

ZED_BASELINE_M = 0.12   # ZED 2i, the figure wall_fusion's own docstring uses


def param_evidence(src, rows, mask_rows, self_mask, self_polygons, window):
    """What the recording says about the tuning constants. No parameter is written."""
    out = {'current': {k: get_value(k) for k in (
        'lidar_front_wall_wall_obs_tol_base_m', 'lidar_front_wall_wall_obs_tol_quad_per_m2',
        'lidar_front_wall_stereo_profile_max_age_s',
        'lidar_front_wall_stereo_profile_min_weight',
        'lidar_front_wall_wall_obs_disagree_reject_frac',
        'lidar_front_wall_wall_obs_stereo_min_columns',
        'front_clearance_scene_mask_floor_tol_m',
        'front_clearance_scene_mask_ceiling_min_z_m',
        'front_clearance_scene_mask_column_percentile',
        'front_clearance_scene_mask_column_min_px',
        'front_clearance_scene_mask_self_polygons',
        'front_clearance_scene_mask_occluded_polygons',
        'front_clearance_inferred_floor_enabled',
        'front_clearance_inferred_floor_r_max_m',
        'front_clearance_inferred_floor_margin_m',
        'front_clearance_inferred_floor_window_k',
        'front_clearance_inferred_floor_sigma_beams',
        'front_clearance_inferred_floor_max_scan_age_s',
        'lidar_exclusion_x_min', 'lidar_exclusion_y_min',
        'lidar_exclusion_x_max', 'lidar_exclusion_y_max')}}

    # ---- the agreement residual, which is what both tolerances are for -----
    r_all, e_all, signed = [], [], []
    for row in rows:
        if not row or row['check'] is None:
            continue
        lidar_m, stereo_m = row['check']['lidar_m'], row['check']['stereo_m']
        both = np.isfinite(lidar_m) & np.isfinite(stereo_m)
        r_all += list(lidar_m[both])
        e_all += list(np.abs(stereo_m[both] - lidar_m[both]))
        signed += list(stereo_m[both] - lidar_m[both])
    r_all, e_all, signed = np.array(r_all), np.array(e_all), np.array(signed)
    res = {'n_columns': int(r_all.size)}
    if r_all.size:
        res.update({
            'range_median': num(float(np.median(r_all))),
            'abs_residual_median': num(float(np.median(e_all))),
            'abs_residual_p95': num(float(np.percentile(e_all, 95))),
            'signed_residual_median': num(float(np.median(signed))),
            'signed_residual_p05': num(float(np.percentile(signed, 5))),
            'signed_residual_p95': num(float(np.percentile(signed, 95)))})
        # Least squares of the p95 envelope against base + quad * r^2, on the
        # same buckets the summary prints -- the tolerance's own functional form.
        edges = [0, 1, 2, 3, 4, 6, 8, 12, 16, 25]
        xs, ys, ns = [], [], []
        for a, b in zip(edges, edges[1:]):
            sel = (r_all >= a) & (r_all < b)
            if np.count_nonzero(sel) < 30:
                continue
            mid = float(np.median(r_all[sel]))
            xs.append(mid * mid)
            ys.append(float(np.percentile(e_all[sel], 95)))
            ns.append(int(np.count_nonzero(sel)))
        res['envelope'] = [{'r2': round(x, 2), 'p95': round(y, 4), 'n': n}
                           for x, y, n in zip(xs, ys, ns)]
        if len(xs) >= 2:
            A = np.column_stack([np.ones(len(xs)), np.array(xs)])
            coef, *_ = np.linalg.lstsq(A, np.array(ys), rcond=None)
            res['fit_p95'] = {'base_m': num(float(coef[0]), 4),
                              'quad_per_m2': num(float(coef[1]), 5)}
            # The disparity sigma that quadratic implies: sigma_Z = Z^2 s / (fx b).
            info = src.get('/zed2/zed_node/depth/camera_info') or src.get('/camera/camera_info')
            fx = float(info[0][2].k[0]) if info else math.nan
            res['implied_sigma_disparity_px'] = num(
                float(coef[1]) * fx * ZED_BASELINE_M, 4)
            res['fx_used'] = num(fx, 2)
            res['baseline_m'] = ZED_BASELINE_M
    # What a different tolerance would actually buy. A proposal is only worth
    # making if it moves the AGREE rate without making the check vacuous.
    sweep = []
    if r_all.size:
        for base, quad in ((0.05, 0.012), (0.10, 0.012), (0.05, 0.03), (0.15, 0.03),
                           (0.25, 0.05), (0.50, 0.05), (1.20, 0.023)):
            tol = wall_fusion.tolerance(r_all, base, quad)
            agree = int(np.count_nonzero(e_all <= tol))
            sweep.append({'base_m': base, 'quad_per_m2': quad,
                          'agree': agree, 'disagree': int(r_all.size - agree),
                          'agree_frac': round(agree / r_all.size, 4),
                          'tol_at_2m': round(float(base + quad * 4), 3),
                          'tol_at_8m': round(float(base + quad * 64), 3)})
    res['tolerance_sweep'] = sweep
    # The LiDAR and the stereo correlate, so they are looking at the same
    # scene; the offset is a bias, not noise. Both numbers belong in the report.
    if r_all.size > 2:
        stereo = np.array(signed) + r_all
        res['correlation'] = num(float(np.corrcoef(r_all, stereo)[0, 1]), 4)
    out['agreement_residual'] = res

    # ---- the scene-mask class split, which is what floor_tol decides -------
    frac = collections.defaultdict(list)
    for row in mask_rows:
        if row.get('frac'):
            for k, v in row['frac'].items():
                frac[k].append(v)
    out['class_fractions'] = {k: {'median': num(float(np.median(v)), 4),
                                  'p05': num(pct(v, 5), 4), 'p95': num(pct(v, 95), 4)}
                              for k, v in sorted(frac.items())}
    # Where the floor actually is. FLOOR is |z| < floor_tol and REFLECTION is
    # z < -floor_tol, so the two class' own z distributions, and the gap
    # between them, are what that one threshold is deciding.
    per_class = {name: [] for name in (scene_mask.CLASS_NAMES[scene_mask.CLASS_FLOOR],
                                       scene_mask.CLASS_NAMES[scene_mask.CLASS_REFLECTION])}
    lower_invalid, lower_classes = [], collections.Counter()
    for row in mask_rows[::2]:
        z, labels = row.get('z'), row.get('mask')
        if z is None or labels is None:
            continue
        for cls in (scene_mask.CLASS_FLOOR, scene_mask.CLASS_REFLECTION):
            v = z[(labels == cls) & np.isfinite(z)]
            if v.size:
                per_class[scene_mask.CLASS_NAMES[cls]].append(v[::29])
        h, w = labels.shape
        low = labels[int(h * 0.6):, int(w * 0.25):int(w * 0.75)]
        lower_invalid.append(float((low == scene_mask.CLASS_INVALID).mean()))
        for cls in scene_mask.CLASS_NAMES:
            lower_classes[scene_mask.CLASS_NAMES[cls]] += int((low == cls).sum())
    out['floor_z'] = {'n': 0}
    stats = {}
    for name, chunks in per_class.items():
        if not chunks:
            continue
        v = np.concatenate(chunks)
        stats[name] = {'n': int(v.size), 'median': num(float(np.median(v)), 4),
                       'p05': num(pct(v, 5), 4), 'p95': num(pct(v, 95), 4)}
    total = sum(lower_classes.values()) or 1
    out['floor_z'] = {
        'n': sum(s['n'] for s in stats.values()),
        'by_class': stats,
        'floor_tol_m': float(get_value('front_clearance_scene_mask_floor_tol_m')),
        'lower_centre_invalid_frac_median': num(float(np.median(lower_invalid)), 4)
        if lower_invalid else None,
        'lower_centre_class_frac': {k: round(v / total, 4)
                                    for k, v in sorted(lower_classes.items())},
    }

    # ---- the LiDAR-inferred floor (inferred_floor.py) ----------------------
    # The whole point of the class is that it is a SEPARATE number, so the
    # report says how much floor was seen, how much was inferred, and how many
    # cycles could not infer at all and why.
    inf_ms, applied, lut_px, skips = [], [], [], collections.Counter()
    for row in mask_rows:
        block = (row.get('stats') or {}).get('inferred_floor')
        if not block:
            skips['off_or_absent'] += 1
            continue
        if block.get('ms') is not None:
            inf_ms.append(block['ms'])
        skips[block.get('skipped') or 'applied'] += 1
        applied.append(block.get('applied_px', 0))
        lut_px.append(block.get('lut_px', 0))
    class_frac = out['class_fractions']
    out['inferred_floor'] = {
        'frames': len(mask_rows),
        'cycles': dict(sorted(skips.items())),
        'applied_px_median': num(float(np.median(applied)), 1) if applied else None,
        'lut_px_median': num(float(np.median(lut_px)), 1) if lut_px else None,
        'ms_median': num(float(np.median(inf_ms)), 3) if inf_ms else None,
        'ms_p95': num(pct(inf_ms, 95), 3) if inf_ms else None,
        'floor_frac_median': (class_frac.get('floor') or {}).get('median'),
        'floor_inferred_frac_median': (
            class_frac.get('floor_inferred') or {}).get('median'),
        'params': {k: get_value('front_clearance_' + k) for k in (
            'inferred_floor_enabled', 'inferred_floor_r_max_m',
            'inferred_floor_margin_m', 'inferred_floor_window_k',
            'inferred_floor_sigma_beams', 'inferred_floor_max_scan_age_s')},
    }

    # ---- the self / occlusion polygons -------------------------------------
    # The occluder the polygons stand for shows up as depth that is MISSING or
    # absurdly near in nearly every frame, not as a near reading: the ZED is
    # below its minimum range on something this close and returns -Inf or
    # nothing. So the footprint is measured as persistent non-depth, and every
    # polygon is scored on how much of it is that and how much is live scene.
    invalid, valid, depth_sum, total = None, None, None, 0
    for row in mask_rows:
        depth = row.get('depth')
        if depth is None:
            continue
        if invalid is None:
            invalid = np.zeros(depth.shape, np.int32)
            valid = np.zeros(depth.shape, np.int32)
            depth_sum = np.zeros(depth.shape, np.float64)
        with np.errstate(invalid='ignore'):
            good = np.isfinite(depth) & (depth > 0)
            near = good & (depth < 0.6)
            invalid += (~good | near).astype(np.int32)
            valid += (good & ~near).astype(np.int32)
            depth_sum += np.where(good & ~near, depth, 0.0)
        total += 1
    poly = {'frames': total, 'polygons': []}
    if invalid is not None and total:
        h, w = invalid.shape
        occluder = invalid / total >= 0.9
        live = valid / total >= 0.5
        poly['image_wh'] = [int(w), int(h)]
        poly['occluder_frac'] = round(float(occluder.mean()), 4)
        if self_mask is not None:
            inside = self_mask.astype(bool)
            poly['self_mask_frac'] = round(float(inside.mean()), 4)
            poly['occluder_outside_self_frac'] = round(
                float((occluder & ~inside).mean()), 4)
        for i, vertices in enumerate(self_polygons or ()):
            mask = scene_mask.polygon_mask((h, w), [vertices]).astype(bool)
            if not mask.any():
                continue
            sub = occluder & mask
            entry = {'index': i,
                     'vertices': [[round(float(a), 3), round(float(b), 3)]
                                  for a, b in vertices],
                     'frac_of_frame': round(float(mask.mean()), 4),
                     'px': int(mask.sum()),
                     'occluder_px': int(sub.sum()),
                     'live_scene_px': int((live & mask).sum()),
                     'live_scene_frac': round(float((live & mask).sum() / mask.sum()), 4)}
            if (live & mask).any():
                d = depth_sum[live & mask] / np.maximum(valid[live & mask], 1)
                entry['live_scene_depth_median_m'] = num(float(np.median(d)))
            if sub.any():
                rr, cc = np.nonzero(sub)
                entry['occluder_bbox_frac'] = [round(float(cc.min()) / w, 3),
                                               round(float(rr.min()) / h, 3),
                                               round(float(cc.max() + 1) / w, 3),
                                               round(float(rr.max() + 1) / h, 3)]
            poly['polygons'].append(entry)
    out['self_polygons'] = poly

    # ---- the per-column profile's own gates --------------------------------
    finite, npx = [], []
    for row in mask_rows:
        cols = row.get('columns')
        if cols is None:
            continue
        d = np.asarray(cols.distance_m, float)
        n = np.asarray(cols.n_px, float)
        finite.append(float(np.isfinite(d).mean()))
        npx.append(n[n > 0])
    out['stereo_columns'] = {
        'frames': len(finite),
        'finite_fraction_median': num(float(np.median(finite)), 4) if finite else None,
        'n_px_median': num(float(np.median(np.concatenate(npx))), 1) if npx else None,
        'n_px_p05': num(pct(np.concatenate(npx), 5), 1) if npx else None}
    return out


# ============================================================================
# Step 2: the camera video
# ============================================================================

def camera_pose(src, frame_id):
    """(R, t) of a camera optical frame in base_link, from /tf_static."""
    to_base = slr.static_tf([m for _s, _r, m in src.get('/tf_static')])
    return to_base(frame_id)


def base_image(frame, size=(640, 360)):
    """The left image for this frame, and what it came from.

    /camera/image_raw_5hz (campaign profile) and /camera/image_raw are the
    clean left image (a loaded recorder keeps few full-rate frames);
    /camera/debug_image/compressed is the same camera at a usable rate with the
    detector's own annotation and text bar drawn into it -- a PHOTO, never the
    class mask. Whichever is used is named on the frame, with its age.
    """
    msg = materialize(frame.get('rgb'))
    if msg is None:
        return np.full((size[1], size[0], 3), 30, np.uint8), 'none', math.nan
    name = frame.get('rgb_topic', '')
    if name.endswith('/compressed'):
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return np.full((size[1], size[0], 3), 30, np.uint8), 'none', math.nan
        # The detector's debug image is the frame above a text bar; keep the
        # 16:9 part, which is the camera.
        keep = int(round(img.shape[1] * 9 / 16))
        img = img[:min(keep, img.shape[0])]
        # It already carries the detector's boxes and text. Pull it most of the
        # way to grey so those do not compete with this tool's own overlay.
        grey = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        img = cv2.addWeighted(img, 0.35, grey, 0.65, 0.0)
    else:
        arr = image_array(msg.data, msg.height, msg.width, msg.step, msg.encoding)
        if arr is None:
            return np.full((size[1], size[0], 3), 30, np.uint8), 'none', math.nan
        img = (np.ascontiguousarray(arr[:, :, :3]) if arr.shape[2] >= 3
               else cv2.cvtColor(np.ascontiguousarray(arr[:, :, 0]), cv2.COLOR_GRAY2BGR))
        if msg.encoding in ('rgb8', 'rgba8'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if img.shape[1] != size[0] or img.shape[0] != size[1]:
        img = cv2.resize(img, size, interpolation=cv2.INTER_LINEAR)
    return img, name, frame.get('rgb_age', math.nan)


def depth_panel(frame, size=(640, 360), max_m=12.0, has_depth=True):
    """The depth image, TURBO over 0..max_m, INVALID black."""
    row = frame.get('mask_row')
    img = np.zeros((size[1], size[0], 3), np.uint8)
    if not has_depth:
        put(img, 'DEPTH NOT RECORDED', (12, 40), C_MISSING, 0.9, 2)
        return img
    if row is None or row.get('depth') is None:
        put(img, 'no depth frame yet', (12, 24), (120, 120, 120), 0.5)
        return img
    depth = row['depth']
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(depth) & (depth > 0)
    v = np.clip(np.where(ok, depth, 0.0) / max_m, 0.0, 1.0)
    out = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    out[~ok] = 0
    if out.shape[1] != size[0] or out.shape[0] != size[1]:
        out = cv2.resize(out, size, interpolation=cv2.INTER_NEAREST)
    put(out, f'depth 0..{max_m:.0f} m, INVALID black', (10, 20), (235, 235, 235), 0.45)
    return out


def put(img, s, uv, color, scale=0.45, thick=1):
    fpr.put(img, s, uv, color, scale, thick)


def short_class(name):
    """A 4-character class label for the video's bottom bar.

    Not name[:4]: 'floor' and 'floor_inferred' both truncate to 'floo', and
    the whole point of the second class is telling it from the first.
    """
    return 'finf' if name == 'floor_inferred' else name[:4]


def confirmed_tracks(frame):
    walls = (frame.get('row') or {}).get('walls')
    return [] if walls is None else [t for t in walls.tracks if t.state in (1, 2)]


#: Door colours (BGR): green open, orange with a person in it; dimmed coasting.
DOOR_COLOUR = (90, 220, 90)
DOOR_OCCUPIED_COLOUR = (0, 140, 255)
DOOR_STATE_NAMES = {0: 'tentative', 1: 'confirmed', 2: 'coasting'}


def door_segments(doors):
    """
    Return [(jamb0_bl, jamb1_bl, BGR, thickness, label)] of one DoorArray, or [].

    The top-down view is base_link, so the doors' *_bl copies are drawn:
    they are base_link at the same scan stamp as the rest of the frame.
    """
    out = []
    for door in ([] if doors is None else doors.doors):
        state = DOOR_STATE_NAMES.get(int(door.state), '?')
        colour = DOOR_OCCUPIED_COLOUR if door.occupied else DOOR_COLOUR
        if state == 'coasting':
            colour = tuple(int(c * 0.55) for c in colour)
        thick = 1 if state == 'tentative' else 3
        label = (f'door {door.id} {door.width:.2f}m {state}'
                 + (' OCCUPIED' if door.occupied else ''))
        out.append(((door.jamb0_bl.x, door.jamb0_bl.y), (door.jamb1_bl.x, door.jamb1_bl.y),
                    colour, thick, label))
    return out


def payload_tracks(frame):
    """Every LIVE track of this cycle, from /perception/virtual_wall/debug.

    The `tracks` list, not /perception/walls: the message carries only the
    published four and none of the rectangle's verdict, and the tracks worth
    drawing are exactly the ones the cap hides.
    """
    row = frame.get('row') or {}
    return (row.get('dbg') or {}).get('tracks') or []


def class_strip(fracs, width, height=30):
    """A stacked bar of the per-class share of this frame, in CLASS_NAMES order."""
    strip = np.full((height, width, 3), 16, np.uint8)
    if not fracs:
        put(strip, 'no class mask for this frame', (8, height - 10), (150, 150, 150), 0.46)
        return strip
    for cls, x0, x1 in fraction_segments(fracs, list(scene_mask.CLASS_NAMES), width):
        cv2.rectangle(strip, (x0, 0), (x1 - 1, height - 1),
                      DRAW_CLASS_COLOURS.get(cls, (128, 128, 128)), -1)
        label = f'{short_class(scene_mask.CLASS_NAMES[cls])} {100 * fracs[cls]:.0f}%'
        if x1 - x0 >= 9 * len(label):
            put(strip, label, (x0 + 4, height - 10), (255, 255, 255), 0.44)
        elif x1 - x0 >= 40:
            put(strip, f'{100 * fracs[cls]:.0f}%', (x0 + 3, height - 10), (255, 255, 255), 0.4)
    return strip


def mask_label(masks, cls):
    """What the class tint is, for the text bar."""
    if masks is None or masks.source is None:
        return (masks.note if masks is not None else 'no mask'), False
    if cls is None:
        return f'{masks.source.upper()} mask: none yet', False
    stale = cls['age'] > MASK_WATCHDOG_AGE_S
    what = (f'RECORDED {REC_CLASSES_TOPIC}' if masks.source == 'recorded'
            else 'REPLAYED (recomputed by the installed code, --mask replay)')
    return (f'{"MASK STALE  " if stale else ""}mask {what}  age {cls["age"]:.2f}s', stale)


def draw_camera(src, frame, cam, scale=2, mask_src=None):
    """The left image with the classes, the scan, the walls and the masks on it."""
    img, source, age = base_image(frame)
    row = frame.get('mask_row')
    h, w = img.shape[:2]

    # ---- classes: ONE source, MaskSource's; nothing if it is missing ------
    counts = {}
    cls_ref = frame.get('cls')
    labels = mask_src.labels(cls_ref['i']) if mask_src is not None and cls_ref else None
    if labels is not None:
        if labels.shape != (h, w):
            labels = cv2.resize(labels, (w, h), interpolation=cv2.INTER_NEAREST)
        tint = np.zeros_like(img)
        for cls, colour in DRAW_CLASS_COLOURS.items():
            tint[labels == cls] = colour
        img = cv2.addWeighted(img, 0.62, tint, 0.38, 0.0)
        counts = {scene_mask.CLASS_NAMES[i]: v for i, v in (cls_ref['frac'] or {}).items()
                  if i in scene_mask.CLASS_NAMES}
    img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_LINEAR)
    if labels is None:
        put(img, 'MASK UNDECODABLE' if cls_ref else mask_label(mask_src, cls_ref)[0],
            (24, 56), C_MISSING, 1.1, 2)

    # ---- YOLO instance masks, outlined -------------------------------------
    masks = frame.get('masks2d')
    n_inst = 0
    if masks is not None and masks.height and masks.width and len(masks.data):
        m = np.frombuffer(bytes(masks.data), np.uint8)[:masks.height * masks.width]
        m = m.reshape(masks.height, masks.width).copy()
        if m.shape != (h, w):
            m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        for inst in np.unique(m):
            if inst == 0:
                continue
            n_inst += 1
            cnts, _ = cv2.findContours((m == inst).astype(np.uint8) * scale,
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, [c * scale for c in cnts], -1, (0, 165, 255), 2, cv2.LINE_AA)

    # ---- the scan, coloured by this cycle's column status ------------------
    rrow = frame.get('row')
    n_disagree = None
    if cam is not None and frame.get('scan') is not None and row is not None \
            and row.get('columns') is not None:
        rot, trans = cam
        cols = row['columns']
        k = (float(cols.fx), float(cols.fy), float(cols.cx), float(cols.cy))
        pts = frame['points']
        xyz = np.column_stack([pts, np.full(len(pts), LASER_Z)])
        uv = scene_mask.project_to_image(xyz, *k, rot, trans)
        status = None
        if rrow and rrow.get('check') is not None:
            status = rrow['check']['status']
            n_disagree = int(np.count_nonzero(status == wall_fusion.STATUS_DISAGREE))
        sw = cols.width
        for (u, v) in uv:
            if not (math.isfinite(u) and math.isfinite(v)):
                continue
            if not (0 <= u < sw and 0 <= v < cols.height):
                continue
            c = STATUS_COLOUR[wall_fusion.STATUS_EMPTY]
            if status is not None:
                c = STATUS_COLOUR.get(int(status[int(u)]), c)
            cv2.circle(img, (int(u * scale * w / sw), int(v * scale * h / cols.height)),
                       2, c, -1, cv2.LINE_AA)

    # ---- the rectangle's own samples, one in four --------------------------
    # Green AT, blue BEYOND, red AGAINST, grey uninformative, and per track
    # "ID role sup cov" at the samples' centre of mass. This is E2 as the node
    # saw it: the detail comes straight out of verify_rectangles.
    detail = (rrow or {}).get('rect_detail') or {}
    if detail.get('u') is not None and len(detail['u']) and row is not None \
            and row.get('columns') is not None:
        cols = row['columns']
        sx, sy = scale * w / cols.width, scale * h / cols.height
        ids = (rrow or {}).get('rect_track_ids') or []
        by_id = {t['id']: t for t in payload_tracks(frame)}
        for u, v, kind in zip(detail['u'], detail['v'], detail['kind']):
            cv2.circle(img, (int(u * sx), int(v * sy)), 1,
                       KIND_COLOUR[int(kind)], -1, cv2.LINE_AA)
        for k_idx, tid in enumerate(ids):
            sel = detail['idx'] == k_idx
            if not np.any(sel):
                continue
            tr = by_id.get(tid)
            if tr is None:
                continue
            cu = float(np.mean(detail['u'][sel])) * sx
            cv_ = float(np.mean(detail['v'][sel])) * sy
            colour = ROLE_COLOUR.get(tr['role'], ROLE_COLOUR['other'])
            put(img, f"{tid} {tr['role']} sup {tr['rect_support']:.2f} "
                     f"cov {tr['rect_coverage']:.2f}"
                     f"{' G' if tr['glass'] else ''}",
                (int(cu) - 40, int(cv_)), colour, 0.42)

    # ---- confirmed walls as vertical outlines ------------------------------
    if cam is not None and row is not None and row.get('columns') is not None:
        rot, trans = cam
        cols = row['columns']
        k = (float(cols.fx), float(cols.fy), float(cols.cx), float(cols.cy))
        sx, sy = scale * w / cols.width, scale * h / cols.height
        for tr in confirmed_tracks(frame):
            z0 = tr.z_min if tr.height_measured else 0.0
            z1 = tr.z_max if tr.height_measured else float(
                get_value('lidar_front_wall_wall_default_height_m'))
            corners = np.array([
                [tr.p0_bl.x, tr.p0_bl.y, z0], [tr.p1_bl.x, tr.p1_bl.y, z0],
                [tr.p1_bl.x, tr.p1_bl.y, z1], [tr.p0_bl.x, tr.p0_bl.y, z1]])
            uv = scene_mask.project_to_image(corners, *k, rot, trans)
            if not np.isfinite(uv).all():
                continue
            poly = np.column_stack([uv[:, 0] * sx, uv[:, 1] * sy]).round().astype(np.int32)
            if np.abs(poly).max() > 20000:
                continue
            colour = ROLE_COLOUR.get(tr.role, (200, 200, 200))
            thick = 3 if tr.is_front else 1
            cv2.polylines(img, [poly], True, colour, thick, cv2.LINE_AA)
            if not tr.height_measured:
                cv2.polylines(img, [poly], True, (60, 60, 60), 1, cv2.LINE_AA)
            label = (f'{tr.id} {tr.role} {tr.d_ray_front:.2f}m'
                     if math.isfinite(tr.d_ray_front) else f'{tr.id} {tr.role}')
            label += f' {SRC_NAMES[tr.source]} {STATE_NAMES[tr.state]}'
            label += '' if tr.height_measured else ' h?'
            u0 = int(np.clip(poly[:, 0].min(), 4, img.shape[1] - 240))
            v0 = int(np.clip(poly[:, 1].min() - 6, 14, img.shape[0] - 6))
            put(img, label, (u0, v0), colour, 0.46)

    # ---- text --------------------------------------------------------------
    rec = frame.get('rec') or {}
    state = (rrow or {}).get('state') or rec.get('state') or '--'
    value = (rrow or {}).get('value')
    if value is None:
        value = rec.get('distance_m')
    # 132 px: five text lines; the MASK STALE warning is on the fifth, inside
    # the bar (it used to be drawn at y = 116 of a 108 px bar, i.e. nowhere).
    bar = np.zeros((132, img.shape[1], 3), np.uint8)
    put(bar, f't {frame["rel"]:7.2f}s   speed {frame["speed"]:.3f} m/s'
             f'   {"MOVING" if frame["moving"] else "still"}', (10, 20),
        (255, 255, 255) if frame['moving'] else (150, 150, 150), 0.52)
    put(bar, f'contract {state.upper():8s} {fmtm(value)}'
             f'   recorded {str(rec.get("state", "--")).upper():8s} {fmtm(rec.get("distance_m"))}'
             f'   DISAGREE cols {n_disagree if n_disagree is not None else "--"}',
        (10, 44), CONTRACT_COLOUR.get(state, (200, 200, 200)), 0.52)
    put(bar, '  '.join(f'{short_class(k)} {v * 100:4.1f}%' for k, v in counts.items())
        or 'no scene mask', (10, 68), (200, 200, 200), 0.46)
    note = (' (debug JPEG PHOTO: detector-annotated, desaturated; not a mask)'
            if source.endswith('/compressed') else '')
    put(bar, f'base {source or "none"}{note} age {age:+.2f}s   cycle age '
             f'{frame.get("row_age", float("nan")):.2f}s   yolo inst {n_inst}',
        (10, 92), (160, 160, 160), 0.44)
    # The watchdog's own verdict, where the eye is already looking: on the
    # 10-15 run the mask froze at t = 30.5 s and the only trace was its age
    # growing 1:1 with the video clock.
    text, stale = mask_label(mask_src, cls_ref)
    put(bar, text, (10, 118), C_MISSING if stale or labels is None else (160, 160, 160),
        0.56 if stale else 0.46)
    img = np.vstack([img, class_strip(cls_ref['frac'] if cls_ref else None, img.shape[1]),
                     bar])

    # ---- legends -----------------------------------------------------------
    for i, (cls, name) in enumerate(scene_mask.CLASS_NAMES.items()):
        y = 22 + 18 * i
        cv2.rectangle(img, (img.shape[1] - 150, y - 11), (img.shape[1] - 134, y + 2),
                      DRAW_CLASS_COLOURS.get(cls, (128, 128, 128)), -1)
        put(img, name, (img.shape[1] - 128, y), (230, 230, 230), 0.42)
    for i, status in enumerate(STATUS_ORDER):
        y = 22 + 18 * (len(scene_mask.CLASS_NAMES) + 1 + i)
        cv2.circle(img, (img.shape[1] - 142, y - 4), 4, STATUS_COLOUR[status], -1)
        put(img, wall_fusion.STATUS_NAMES[status], (img.shape[1] - 128, y),
            (230, 230, 230), 0.42)
    return img


def fmtm(x):
    return '--' if x is None or not math.isfinite(float(x)) else f'{float(x):7.2f} m'


def draw_side_notes(img, frame, y0):
    """Under the depth panel: the column statuses and what produced this frame."""
    row = (frame.get('row') or {})
    check = row.get('check')
    y = [y0]

    def line(text, colour=(210, 210, 210), scale=0.5, gap=26):
        put(img, text, (14, y[0]), colour, scale)
        y[0] += gap

    line('COLUMN CROSS-CHECK (this cycle)', (150, 150, 150), 0.52)
    # The age AND the weight, always. "No fresh stereo profile" was never the
    # whole truth: the difference between 0.16 s (worth 0.65) and 2.5 s (worth
    # nothing) is the difference between a tuning question and a dead camera.
    age, weight = row.get('stereo_age'), row.get('stereo_weight')
    if age is not None and math.isfinite(age):
        colour = (90, 220, 90) if (weight or 0.0) > 0.0 else (0, 200, 255)
        line(f'stereo age {age:+.2f} s  w {weight if weight is not None else 0.0:.2f}',
             colour, 0.5)
    if check is None:
        line('  LiDAR only this cycle', (0, 200, 255), 0.46)
    else:
        for status in STATUS_ORDER:
            n = int(np.count_nonzero(check['status'] == status))
            line(f'  {wall_fusion.STATUS_NAMES[status]:12s} {n:4d}', STATUS_COLOUR[status])
    y[0] += 10
    line('TRACKS', (150, 150, 150), 0.52)
    for tr in sorted(confirmed_tracks(frame), key=lambda t: t.rho_bl)[:8]:
        d = tr.d_ray_front if math.isfinite(tr.d_ray_front) else float('nan')
        line(f'  {tr.id:3d} {tr.role:5s} {SRC_NAMES[tr.source]:6s} rho {tr.rho_bl:6.2f}'
             f' d_ray {d:6.2f}{"" if tr.height_measured else " h?"}'
             f'{" FRONT" if tr.is_front else ""}',
             ROLE_COLOUR.get(tr.role, (200, 200, 200)), 0.46, 22)


def render_camera(src, frames, cam, path, masks=None, has_depth=True):
    out = []
    for f in frames:
        main = draw_camera(src, f, cam, mask_src=masks)
        w = main.shape[1] // 2
        depth = cv2.resize(depth_panel(f, has_depth=has_depth), (w, int(round(w * 9 / 16))),
                           interpolation=cv2.INTER_NEAREST)
        side = np.zeros((main.shape[0], w, 3), np.uint8)
        side[:depth.shape[0]] = depth
        draw_side_notes(side, f, depth.shape[0] + 26)
        img = np.hstack([main, side])
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            out.append((f['t'], buf.tobytes()))
    return write_mp4(out, path, fps=int(CAM_HZ), crf=24) if out else None


# ============================================================================
# Step 3: the top-down video
# ============================================================================

C_SCAN = (170, 170, 170)
C_STRIP = (255, 255, 255)
C_OBS = (120, 200, 255)
C_CLEAR = (120, 90, 200)
C_TRUE = (255, 255, 255)


def draw_topdown(src, frame, view, front_x, half):
    img = np.full((view.h, view.w, 3), 18, np.uint8)
    rrow = frame.get('row')
    rec = frame.get('rec') or {}

    # grid
    for m in range(0, 25, 1):
        v = view.px(m, 0)[1]
        if 0 <= v < view.h:
            cv2.line(img, (0, v), (view.w, v), (34, 34, 34) if m % 5 else (54, 54, 54), 1)
            if m % 5 == 0:
                put(img, f'{m} m', (4, v - 3), (90, 90, 90), 0.36)

    # the shared strip from the bumper
    for sign in (-1, 1):
        view.line(img, (front_x, sign * half), (front_x + 30.0, sign * half),
                  (70, 70, 70), 1, dash=0.25)

    # the car
    cv2.rectangle(img, view.px(front_x, half), view.px(-0.1, -half), (60, 60, 60), 1)
    put(img, 'base_front', (view.px(front_x, half)[0] + 6, view.px(front_x, 0)[1] - 4),
        (90, 90, 90), 0.36)

    # scan
    if frame.get('points') is not None and len(frame['points']):
        for u, v in view.pxs(frame['points']):
            if 0 <= u < view.w and 0 <= v < view.h:
                cv2.circle(img, (int(u), int(v)), 1, C_SCAN, -1)

    # observations, thin
    if rrow and rrow.get('obs') is not None:
        for o in rrow['obs'].observations:
            colour = {0: (150, 150, 150), 1: (220, 120, 220), 2: (90, 220, 90)}[o.source]
            view.line(img, (o.p0.x, o.p0.y), (o.p1.x, o.p1.y), colour, 1)

    # Every LIVE track as a SEGMENT between its endpoints -- not only the
    # published four, because 'other' and REJECTED are the interesting ones and
    # the publish cap keeps them off /perception/walls. Colour by role, solid
    # VERIFIED / dashed UNVERIFIED / dotted REJECTED, and a G for glass.
    front_track = None
    for tr in payload_tracks(frame):
        if tr['p0_bl'][0] is None or tr['p1_bl'][0] is None:
            continue
        a, b = tuple(tr['p0_bl']), tuple(tr['p1_bl'])
        colour = ROLE_COLOUR.get(tr['role'], ROLE_COLOUR['other'])
        thick = 4 if tr['is_front'] else 2 if tr['state'] != 'tentative' else 1
        if tr['state'] == 'coasting':
            colour = tuple(int(c * 0.55) for c in colour)
        view.line(img, a, b, colour, thick, dash=VERIFY_DASH.get(tr['verify'], 0.0))
        for end in (a, b):
            cv2.circle(img, view.px(*end), 3, colour, -1, cv2.LINE_AA)
        mid = (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
        flags = ('G' if tr['glass'] else '') + ('O' if tr['occluded'] else '')
        miss = f" m{tr['miss']}" if tr['miss'] else ''
        view.text(img, f"{tr['id']} {tr['role']} {tr['source']} {tr['state']}"
                       f"{(' ' + flags) if flags else ''}{miss}", mid, colour, 0.40)
        if tr['is_front']:
            front_track = tr

    # Doors (/perception/doors, perception only): a dot per jamb and the
    # threshold between them, in base_link at the scan stamp.
    for a, b, colour, thick, label in door_segments((rrow or {}).get('doors')):
        view.line(img, a, b, colour, thick)
        for end in (a, b):
            cv2.circle(img, view.px(*end), 5, colour, -1, cv2.LINE_AA)
        view.text(img, label, (0.5 * (a[0] + b[0]) + 0.15, 0.5 * (a[1] + b[1])), colour, 0.40)

    # the FRONT wall and its d_ray_front
    if front_track is not None and front_track['d_ray_front'] is not None:
        d = front_track['d_ray_front']
        view.line(img, (front_x, 0.0), (front_x + d, 0.0), (60, 60, 255), 2, dash=0.12)
        view.text(img, f'd_ray_front {d:.2f} m', (front_x + d * 0.5, half + 0.12),
                  (60, 60, 255), 0.46)

    # the true nearest return in the strip
    if frame.get('strip_point') is not None:
        p = frame['strip_point']
        cv2.circle(img, view.px(p[0], p[1]), 6, C_TRUE, 2, cv2.LINE_AA)
        view.text(img, f'true strip {frame["strip"]:.2f} m', (p[0], p[1] - 0.25),
                  C_TRUE, 0.42)

    # the costmap front clearance, as an arc from the bumper
    if frame.get('clearance') is not None and frame['clearance'] > 0:
        view.arc(img, (front_x, 0.0), frame['clearance'], -0.6, 0.6, C_CLEAR, 2)
        view.text(img, f'costmap {frame["clearance"]:.2f} m',
                  (front_x + frame['clearance'], -0.9), C_CLEAR, 0.42)

    # the contract itself
    state = (rrow or {}).get('state') or rec.get('state') or '--'
    value = (rrow or {}).get('value', rec.get('distance_m'))
    if state == 'wall' and value is not None and math.isfinite(value):
        view.line(img, (front_x + value, -1.4), (front_x + value, 1.4),
                  CONTRACT_COLOUR['wall'], 2)
    put(img, f'{"MOVING WINDOW" if frame["moving"] else "stationary"}',
        (view.w - 170, view.h - 10),
        (90, 220, 90) if frame['moving'] else (110, 110, 110), 0.46)
    return img


def draw_panel(src, frame, w=520, h=760):
    img = np.full((h, w, 3), 24, np.uint8)
    rrow = frame.get('row') or {}
    rec = frame.get('rec') or {}
    dbg = rrow.get('dbg') or {}
    y = [26]

    def line(text, colour=(220, 220, 220), scale=0.46, gap=21):
        put(img, text, (12, y[0]), colour, scale)
        y[0] += gap

    line(f't {frame["rel"]:.2f} s    speed {frame["speed"]:.3f} m/s', (255, 255, 255), 0.56, 28)
    line('MOVING' if frame['moving'] else 'stationary',
         (90, 220, 90) if frame['moving'] else (120, 120, 120), 0.5)
    y[0] += 6
    line('FRONT VALUES', (150, 150, 150), 0.48)
    state = rrow.get('state') or '--'
    line(f'  contract (replay)  {state.upper():8s} {fmtm(rrow.get("value"))}',
         CONTRACT_COLOUR.get(state, (200, 200, 200)))
    line(f'  contract (recorded){str(rec.get("state", "--")).upper():8s}'
         f' {fmtm(rec.get("distance_m"))}',
         CONTRACT_COLOUR.get(rec.get('state'), (200, 200, 200)))
    line(f'  true strip range   {fmtm(frame.get("strip"))}', C_TRUE)
    line(f'  costmap clearance  {fmtm(frame.get("clearance"))}', C_CLEAR)
    line(f'  front_d_ray        {fmtm(dbg.get("front_d_ray_m"))}')
    line(f'  lidar estimator    {fmtm(dbg.get("lidar_m"))}')
    line(f'  stereo wall        {fmtm(dbg.get("stereo_m"))}')
    line(f'  clear_distance     {fmtm(dbg.get("clear_distance_m"))}', (120, 120, 120))
    line(f'  wall_source        {dbg.get("wall_source")}', (160, 160, 160))
    y[0] += 6
    obs = rrow.get('obs')
    if obs is not None:
        by = collections.Counter(SRC_NAMES[o.source] for o in obs.observations)
        line('OBSERVATIONS (this cycle)', (150, 150, 150), 0.48)
        line(f'  n {len(obs.observations)}   rejected {obs.n_rejected}'
             f'   disagree cols {obs.n_disagree_columns}',
             (60, 60, 255) if obs.n_rejected else (200, 200, 200))
        line('  ' + '  '.join(f'{k} {v}' for k, v in sorted(by.items())) or '  none')
    check = rrow.get('check')
    if check is not None:
        counts = wall_fusion.status_counts(check['status'])
        line('  ' + '  '.join(f'{k[:5]} {v}' for k, v in counts.items()), (170, 170, 170), 0.42)
        line(f'  profile stamp age {frame["t"] - (rrow.get("profile_stamp") or frame["t"]):+.3f} s',
             (170, 170, 170), 0.42)
    else:
        line('  no stereo profile this cycle: LiDAR only', (0, 200, 255), 0.44)
    y[0] += 6
    line('TRACKS', (150, 150, 150), 0.48)
    for tr in sorted(confirmed_tracks(frame), key=lambda t: t.rho_bl)[:9]:
        colour = ROLE_COLOUR.get(tr.role, (200, 200, 200))
        line(f'  {tr.id:3d} {tr.role:5s} {STATE_NAMES[tr.state][:4]:4s}'
             f' {SRC_NAMES[tr.source]:6s} rho {tr.rho_bl:6.2f}'
             f' d_ray {tr.d_ray_front if math.isfinite(tr.d_ray_front) else float("nan"):6.2f}'
             f'{"" if tr.height_measured else " h?"}{" FRONT" if tr.is_front else ""}',
             colour, 0.42, 18)
    y[0] = h - 58
    line(f'cycle age {frame.get("row_age", float("nan")):.2f} s   '
         f'scan age {frame.get("scan_age", float("nan")):.2f} s', (130, 130, 130), 0.42)
    # The age of the mask the camera video draws (recorded or replayed), else
    # of the replay's own mask.
    cls = frame.get('cls')
    mask_age = cls['age'] if cls else frame.get('mask_age', float('nan'))
    stale = math.isfinite(mask_age) and mask_age > MASK_WATCHDOG_AGE_S
    line(f'{"drawn" if cls else "replay"} mask age {mask_age:.2f} s'
         f'{"  STALE" if stale else ""}   '
         f'costmap age {frame.get("clearance_age", float("nan")):.2f} s',
         (0, 200, 255) if stale else (130, 130, 130), 0.42)
    return img


def wall_value(payload):
    """The contract value only where the state is WALL; see TopChart."""
    if not payload:
        return None
    state = payload.get('state')
    value = payload.get('value', payload.get('distance_m'))
    return value if state == 'wall' else None


class TopChart:
    """Contract value and state, true strip range, costmap clearance, speed,
    and a thin strip of the per-class shares of the drawn mask over time."""

    def __init__(self, frames, window, src, w=1280, h=274, y_max=25.0):
        self.w, self.h, self.y_max = w, h, y_max
        self.t0, self.t1 = frames[0]['t'], frames[-1]['t']
        self.l, self.r, self.top, self.bot = 52, w - 168, 14, h - 42
        img = np.full((h, w, 3), 22, np.uint8)
        for a, b in window:
            over = img.copy()
            cv2.rectangle(over, (self.u(a), self.top), (self.u(b), self.bot), (40, 70, 40), -1)
            img = cv2.addWeighted(over, 0.5, img, 0.5, 0)
        for m in range(0, int(y_max) + 1, 5):
            v = self.v(m)
            cv2.line(img, (self.l, v), (self.r, v), (48, 48, 48), 1)
            put(img, f'{m}', (self.l - 26, v + 4), (125, 125, 125), 0.38)
        for s in range(0, int(self.t1 - self.t0) + 1, 10):
            u = self.u(self.t0 + s)
            cv2.line(img, (u, self.top), (u, self.bot), (38, 38, 38), 1)
            put(img, f'{src.rel(self.t0) + s:.0f}s', (u - 12, h - 8), (125, 125, 125), 0.38)
        # contract state as a band under the axis
        for f in frames:
            state = (f.get('row') or {}).get('state')
            if state is None:
                continue
            u = self.u(f['t'])
            cv2.line(img, (u, self.bot + 2), (u, self.bot + 10),
                     CONTRACT_COLOUR.get(state, (90, 90, 90)), 2)
        # the class shares of the drawn mask, stacked bottom-up in CLASS_NAMES order
        y0, sh = self.bot + 13, 10
        for f, nxt in zip(frames, frames[1:] + frames[-1:]):
            frac = (f.get('cls') or {}).get('frac')
            if not frac:
                continue
            u0, u1 = self.u(f['t']), max(self.u(nxt['t']), self.u(f['t']) + 1)
            for cls, a, b in fraction_segments(frac, list(scene_mask.CLASS_NAMES), sh):
                cv2.rectangle(img, (u0, y0 + sh - b), (u1 - 1, y0 + sh - a - 1),
                              DRAW_CLASS_COLOURS.get(cls, (128, 128, 128)), -1)
        sp = [(self.u(f['t']),
               self.bot - int(min(abs(f['speed'] or 0.0), 1.0) * (self.bot - self.top) * 0.35))
              for f in frames]
        cv2.fillPoly(img, [np.array([(self.l, self.bot)] + sp + [(sp[-1][0], self.bot)],
                                    np.int32)], (58, 58, 58))
        self.series(img, [(f['t'], f.get('strip')) for f in frames], C_TRUE, 2)
        self.series(img, [(f['t'], f.get('clearance')) for f in frames], C_CLEAR, 2)
        # CLEAR carries wall_clear_distance_m (50 m), a sentinel and not a
        # distance, so only the WALL state is drawn as a value; CLEAR and
        # NO_DATA are the band under the axis.
        self.series(img, [(f['t'], wall_value(f.get('row'))) for f in frames],
                    (60, 60, 255), 2)
        self.series(img, [(f['t'], wall_value(f.get('rec'))) for f in frames],
                    (0, 200, 255), 1)
        for i, (label, colour) in enumerate([
                ('true strip', C_TRUE), ('costmap clear', C_CLEAR),
                ('WALL m, replay', (60, 60, 255)),
                ('WALL m, recorded', (0, 200, 255)),
                ('speed (0..1)', (90, 90, 90)), ('WALL', CONTRACT_COLOUR['wall']),
                ('CLEAR', CONTRACT_COLOUR['clear']), ('NO_DATA', CONTRACT_COLOUR['no_data']),
                ('mask classes (strip)', (150, 150, 150))]):
            put(img, label, (self.r + 8, self.top + 14 + 17 * i), colour, 0.38)
        put(img, f'm (clipped at {y_max:.0f}); green band = moving window',
            (4, self.top + 2), (125, 125, 125), 0.36)
        self.base = img

    def u(self, t):
        return int(self.l + (t - self.t0) / max(self.t1 - self.t0, 1e-6) * (self.r - self.l))

    def v(self, m):
        return int(self.bot - min(max(m, 0.0), self.y_max) / self.y_max * (self.bot - self.top))

    def series(self, img, tv, colour, thick):
        prev = None
        for t, m in tv:
            if m is None or not math.isfinite(float(m)) or m <= 0:
                prev = None
                continue
            p = (self.u(t), self.v(float(m)))
            if prev is not None:
                cv2.line(img, prev, p, colour, thick, cv2.LINE_AA)
            prev = p

    def draw(self, t):
        img = self.base.copy()
        u = self.u(t)
        cv2.line(img, (u, self.top), (u, self.bot + 10), (255, 255, 255), 1)
        return img


def render_topdown(src, frames, window, front_x, half, path):
    view = fpr.View(w=760, h=760, x_min=-1.0, x_max=24.0)
    chart = TopChart(frames, window, src)
    out = []
    for f in frames:
        top = np.hstack([draw_topdown(src, f, view, front_x, half), draw_panel(src, f)])
        img = np.vstack([top, chart.draw(f['t'])])
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            out.append((f['t'], buf.tobytes()))
    return write_mp4(out, path, fps=int(TOP_HZ), crf=23) if out else None


# ============================================================================
# Reports
# ============================================================================

def table(rows, headers):
    out = ['| ' + ' | '.join(headers) + ' |',
           '|' + '|'.join('---' for _ in headers) + '|']
    for r in rows:
        out.append('| ' + ' | '.join('--' if c is None else str(c) for c in r) + ' |')
    return out


def write_replay_md(path, src, cmp_out, cycles, rows_rec, rows_full, mask_rows, depth_topic):
    L = [f'# Step 1 - offline replay of the current code over `{src.name}`', '',
         'Header stamps throughout; every comparison is against the recorded message',
         'carrying the SAME stamp. See the module docstring for why receipts are not used.',
         '']
    col = cmp_out['stereo_columns']
    L += ['## /perception/stereo_columns (the camera half)', '',
          f'Depth replayed from `{depth_topic}`.', '',
          f'- recorded messages: {col["recorded"]}',
          f'- replayed frames: {col["replayed"]}',
          f'- **stamps present in both: {col["shared_stamps"]}**'
          f' ({col["replay_only"]} replayed frames the live node never processed)', '']
    if col.get('distance_m', {}).get('n'):
        d = col['distance_m']
        L += [f'On the shared stamps, per column where both are finite '
              f'(n = {d["n"]}): median |diff| {d["median"]} m, p95 {d["p95"]} m, '
              f'max {d["max"]} m.',
              f'Finite columns per frame: recorded median '
              f'{col["finite_columns"]["recorded_median"]:.0f}, replay '
              f'{col["finite_columns"]["replay_median"]:.0f} of {640}.',
              f'n_px per column: median |diff| {col["n_px"].get("median")}, '
              f'max {col["n_px"].get("max")}.', '']
    for name, title in (('rec', 'fed the RECORDED stereo columns'),
                        ('full', 'fed the REPLAYED stereo columns')):
        p = cmp_out['passes'].get(name)
        if not p:
            continue
        L += [f'## Pass `{name}` - {title}', '',
              f'- cycles replayed: {p["cycles"]}',
              f'- contract state identical on {p["contract_state_match"]}'
              f'/{p["cycles"]} ({100 * p["contract_state_match_frac"]:.1f} %)', '']
        L += table([[k, v] for k, v in p['contract_confusion'].items()],
                   ['recorded -> replay', 'cycles']) + ['']
        L += table([[k, v.get('n'), v.get('median'), v.get('p95'), v.get('max')]
                    for k, v in (('virtual_wall value [m]', p['virtual_wall_m']),
                                 ('front_d_ray [m]', p['front_d_ray_m']),
                                 ('n_observations', p['n_observations']),
                                 ('n_rejected', p['n_rejected']),
                                 ('n_disagree_columns', p['n_disagree_columns']),
                                 ('n_tracks', p['n_tracks']))],
                   ['quantity', 'n', 'median |diff|', 'p95', 'max']) + ['']
        cap = p['publish_cap']
        if cap['wall_to_clear']:
            L += [f'Of the {cap["wall_to_clear"]} cycles that went recorded WALL -> replay '
                  f'CLEAR, **{cap["wall_to_clear_explained_by_cap"]}** have a confirmed '
                  f'track in the replay tracker that `front_track()` would have returned if '
                  f'`max_published` were not {4}, at a distance within 0.25 m of the recorded '
                  f'value (median error {cap["hidden_front_error_median"]} m). That is the '
                  'divergence: not the code, the cap plus the tracker history this bag does '
                  'not contain.', '']
        op = p['observation_pairing']
        L += [f'Observations paired by (normal, d): {op["matched"]} matched, '
              f'{op["recorded_only"]} recorded-only, {op["replay_only"]} replay-only.', '']
        if p['source_confusion']:
            L += table([[k, v] for k, v in p['source_confusion'].items()],
                       ['source recorded -> replay', 'observations']) + ['']
    return Path(path).write_text('\n'.join(L) + '\n')


def headline_md(head, steps_info=None, cmp_summary=None):
    """The headline section of summary.md (and the whole of compare.md)."""
    L = ['## Headline metrics, per pass', '']
    span = head.get('span_s')
    L += [f'Span: {"t = %.2f .. %.2f s" % tuple(span) if span else "the whole bag"} '
          '(the tracker replay always starts at the bag start).', '']
    if steps_info:
        L += [f'Tracker steps: {steps_info.get("replayed_cycles")} replayed '
              f'(--tracker-steps {steps_info.get("mode")}; record format '
              f'{steps_info.get("format")}, car decimation {steps_info.get("decimation")}, '
              f'{steps_info.get("payloads")} recorded payloads, missing steps '
              f'{steps_info.get("missing_steps")}, sequence breaks '
              f'{steps_info.get("breaks")}, gaps > 1.5x median {steps_info.get("long_gaps")}).',
              '']
    passes = head.get('passes') or {}
    names = list(passes)
    flat, _notes = flat_headline({'summary': {'headline': head}})
    L += table([[metric, how] + [flat.get(n, {}).get(metric) for n in names]
                for metric, how in COMPARE_METRICS],
               ['metric', 'meaning'] + names) + ['']
    for n in names:
        L += [f'- `{n}`: {passes[n].get("source")}']
    L += ['']
    for k, v in (head.get('definitions') or {}).items():
        L += [f'- **{k}**: {v}']
    L += ['']
    for n in names:
        eps = passes[n].get('phantom_episode_list') or []
        if eps:
            L += [f'Phantom FRONT episodes, `{n}`:', '']
            L += table([[e['start_s'], e['end_s'], e['cycles'], e['kind'], e['error_median_m'],
                         'yes' if e['moving'] else 'no'] for e in eps],
                       ['start [s]', 'end [s]', 'cycles', 'kind', 'finite err med [m]',
                        'moving']) + ['']
        ids = (passes[n].get('tracks_rho_lt_0_5') or {}).get('ids')
        if ids:
            L += [f'`{n}` confirmed tracks with |rho_bl| < {RHO_BL_CLOSE_M} m: {ids}', '']
    if cmp_summary:
        L += compare_md(cmp_summary)
    return L


def compare_md(cmp_summary):
    L = [f'## Before / after: `{cmp_summary["before"]}`', '',
         'before = the older summary.json, after = this run; same metric names, same '
         'pass names. Read the note column: a pre-v2 summary has no headline block and '
         'only some metrics map.', '']
    L += table(cmp_summary['rows'], ['metric', 'pass', 'before', 'after', 'delta', 'note'])
    return L + ['']


def write_summary_md(path, src, summary, evidence, window, videos, steps_info=None,
                     cmp_summary=None):
    L = [f'# Step 4 - summary for `{src.name}`', '']
    if summary.get('headline'):
        L += headline_md(summary['headline'], steps_info, cmp_summary)
    st = summary['state_time_s']
    L += ['## Per-state time (the WALL / CLEAR / NO_DATA contract)', '']
    keys = sorted(set(st['recorded']) | set(st['replay']))
    L += table([[k, st['recorded'].get(k), st['replay'].get(k),
                 st['recorded_moving'].get(k)] for k in keys],
               ['state', 'recorded [s]', 'replay [s]', 'recorded, moving only [s]']) + ['']
    L += ['## FRONT in the corridor', '']
    cs = summary['clear_state']
    L += [f'While the contract said CLEAR ({cs["n"]} cycles) the true strip range never fell '
          f'below **{cs["true_strip_min"]} m** (median {cs["true_strip_median"]} m); '
          f'{cs["n_with_strip_under_5m"]} cycles under 5 m, '
          f'{cs["n_with_strip_under_2m"]} under 2 m.', '']
    if summary['front_episodes']:
        L += table([[e['start_s'], e['end_s'], e['duration_s'], e['n'], e['value_min'],
                     e['value_max'], e['true_strip_min'], e['error_median'], e['error_p95'],
                     'yes' if e['moving'] else 'no']
                    for e in summary['front_episodes']],
                   ['start [s]', 'end [s]', 'dur [s]', 'cycles', 'value min', 'value max',
                    'true strip min', 'err med', 'err p95', 'moving']) + ['']
    else:
        L += ['No WALL episode at all.', '']
    L += [f'Phantom episodes, RECORDED, pre-v2 definition (WALL more than 0.5 m short of '
          f'a NON-EMPTY true strip; an empty strip is not counted here -- the headline '
          f'table counts it): **{len(summary["phantom_front_episodes"])}**.', '']
    L += ['## All-WALL error vs the true strip range', '',
          'Every WALL cycle of the span with a return in the strip -- side walls, '
          'phantoms and the stationary time included. This is what earlier summaries '
          'called the "end-wall error"; the headline end-wall error is the moving-window '
          'cycles with the strip under --end-wall-range.', '']
    L += table([[k, v['n'], v['median'], v['p05'], v['p95'], v['max_abs']]
                for k, v in summary['end_wall_error_m'].items()],
               ['pass', 'n', 'median [m]', 'p05', 'p95', 'max |err|']) + ['']
    t = summary['true_strip_range_m']
    L += [f'True strip range over the run: {t["finite"]}/{t["scans"]} scans have a return '
          f'in the strip; min {t["min"]} m, median {t["median"]} m, max {t["max"]} m.', '']
    L += ['## Tracks', '']
    L += table([[k, v['distinct_ids'], v['births'], v['births_per_minute'],
                 v['role_switches'], v['source_lidar'], v['source_both'],
                 v['source_stereo_only'], v['max_confirmed_age_s']]
                for k, v in summary['tracks'].items()],
               ['pass', 'ids', 'births', 'births/min', 'role switches', 'lidar only',
                'both', 'stereo only', 'max confirmed age [s]']) + ['']
    per = summary.get('per_track') or []
    if per:
        L += ['### Every track of the replay', '',
              'One row per track the replay ever held, not only the published four: '
              'a track can be real, tracked and a reference wall for nothing, and '
              'those are the rows that say so. `role`, `state` and `verify` are each '
              'the value the track spent most of its cycles in.', '']
        L += table([[t['id'], t['role'], t['lifetime_s'], t['cycles'], t['state'],
                     t['verify'], t['glass_cycles'], t['occluded_cycles'],
                     t['front_cycles'], t['length_m'], t['rho_bl_median'],
                     t['hits'], t['max_miss'], t['source'], t['coverage_max'],
                     t['support_max'], t['beyond_max']] for t in per],
                   ['id', 'role', 'lifetime [s]', 'cycles', 'state', 'verify',
                    'glass cyc', 'occl cyc', 'front cyc', 'length [m]',
                    'rho_bl med', 'hits', 'max miss', 'source', 'cov max',
                    'sup max', 'beyond max']) + ['']
    cvr = summary.get('columns_vs_rectangle') or {}
    if cvr.get('cycles_with_an_informative_rectangle'):
        L += ['### Where the columns and the rectangle disagree', '',
              f"Cycles with an informative rectangle (coverage >= 0.25 on at least "
              f"one track): **{cvr['cycles_with_an_informative_rectangle']}**. The "
              'comparison is at the CYCLE level because that is the only level at '
              'which both are recorded -- a columns verdict belongs to an '
              'OBSERVATION and the rectangle\'s to a TRACK, and nothing records '
              'which observation matched which track. A disagreement is the columns '
              'rejecting while the rectangle verifies, or the columns agreeing while '
              f"the rectangle rejects: **{cvr['disagreements']}**.", '']
        L += table([[k, v] for k, v in cvr['cells'].items()],
                   ['columns / rectangle', 'cycles']) + ['']
    L += ['## DISAGREE rate vs range', '',
          'Only the columns where BOTH sensors reported. `abs residual` is '
          '|stereo - lidar| in that bucket.', '']
    L += table([[k, v['n_both'], v['disagree'], v['rate'], v['abs_residual_median'],
                 v['abs_residual_p95'], v['abs_residual_p99']]
                for k, v in summary['disagree_vs_range'].items() if v['n_both']],
               ['range [m]', 'columns with both', 'disagree', 'rate',
                'resid med [m]', 'p95', 'p99']) + ['']
    cap = summary['publish_cap']
    L += ['## The publish cap, and what it hides', '',
          'front_track() only considers tracker.published(), at most '
          f'`max_published` = {cap["max_published"]} tracks ranked by '
          '`length * hits / distance`. A far wall dead ahead is short and far, so that '
          'ranking puts it below the long near side walls unless its hit count is large -- '
          'and hits grow with node UPTIME.', '',
          f'- replay cycles: {cap["cycles"]}',
          f'- cycles where a confirmed track WOULD be the FRONT wall but the cap hides it: '
          f'**{cap["cycles_with_a_hidden_front"]}**',
          f'- its distance, median {cap["hidden_d_ray_median"]} m; its rank in the ranking, '
          f'median {cap["hidden_rank_median"]}; its hits, median {cap["hidden_hits_median"]}; '
          f'its length, median {cap["hidden_length_median"]} m; tracks held, median '
          f'{cap["tracks_held_median"]}',
          f'- the RECORDED run\'s FRONT track was {cap["recorded_front_track_age_s"]["median"]} s '
          f'old (max {cap["recorded_front_track_age_s"]["max"]} s) -- older than this bag, so it '
          'had been accumulating hits long before the recording began.', '']
    sp = summary['stereo_pairing']
    L += ['## Stereo pairing (why the cross-check usually does not run)', '',
          f'`stereo_profile_max_age_s` = {sp["max_age_s"]} s.',
          f'Cycles: {sp["cycles"]}. The profile the node was holding was within the limit on '
          f'**{sp["within_limit_held"]}**; the best any /perception/stereo_columns message in '
          f'the bag could have done is **{sp["best_possible_within_limit"]}**.',
          f'Held-profile age: median {sp["held_age_median"]} s, p95 {sp["held_age_p95"]} s; '
          f'best achievable |age| median {sp["best_age_median"]} s.',
          f'Replay cycles that actually ran the cross-check: '
          f'{summary["cycles_with_stereo_profile"]["replay"]} of '
          f'{summary["cycles_with_stereo_profile"]["cycles"]}.',
          f'The LIVE node managed **{sp["recorded_cycles_using_a_profile"]} of '
          f'{sp["recorded_cycles"]}** recorded cycles -- it held the latest ARRIVED '
          'profile, which is older than the latest by stamp, so it did worse than this '
          'replay.',
          f'Every one of those cycles rejected at least one wall observation: '
          f'{sp["recorded_observations_rejected"]} rejections in all.', '']
    L += ['## Timing per stage [ms]', '']
    L += table([[k, v.get('n'), v.get('median_ms'), v.get('p95_ms')]
                for k, v in summary['timing_ms'].items()],
               ['stage', 'n', 'median', 'p95']) + ['']
    L += ['## Parameter evidence - NOTHING WAS CHANGED', '']
    L += table([[k, v] for k, v in evidence['current'].items()],
               ['parameter', 'current value']) + ['']
    r = evidence['agreement_residual']
    if r.get('n_columns'):
        L += ['### wall_obs_tol_base_m and wall_obs_tol_quad_per_m2', '',
              f'{r["n_columns"]} columns had both sensors. |stereo - lidar|: median '
              f'{r["abs_residual_median"]} m, p95 {r["abs_residual_p95"]} m. '
              f'Signed: median {r["signed_residual_median"]} m, '
              f'p05 {r["signed_residual_p05"]}, p95 {r["signed_residual_p95"]}.', '']
        L += [f'LiDAR and stereo column distances correlate at r = {r.get("correlation")}: '
              'the two are looking at the same scene, so this is a BIAS, not noise.', '']
        L += table([[e['r2'], e['n'], e['p95']] for e in r['envelope']],
                   ['range^2 [m^2]', 'columns', 'p95 |resid| [m]']) + ['']
        if 'fit_p95' in r:
            L += [f'Least squares of that p95 envelope on base + quad * r^2 gives '
                  f'**base {r["fit_p95"]["base_m"]} m, quad '
                  f'{r["fit_p95"]["quad_per_m2"]} m/m^2**, against the current '
                  f'{evidence["current"]["lidar_front_wall_wall_obs_tol_base_m"]} and '
                  f'{evidence["current"]["lidar_front_wall_wall_obs_tol_quad_per_m2"]}.',
                  f'At fx {r["fx_used"]} px and baseline {r["baseline_m"]} m that quadratic '
                  f'is a disparity sigma of {r["implied_sigma_disparity_px"]} px '
                  '(sigma_Z = Z^2 sigma_d / (fx b)); wall_fusion assumes 0.5 px.', '']
        if r.get('tolerance_sweep'):
            L += ['What each tolerance would actually buy, over the same columns:', '']
            L += table([[e['base_m'], e['quad_per_m2'], e['tol_at_2m'], e['tol_at_8m'],
                         e['agree'], e['disagree'], e['agree_frac']]
                        for e in r['tolerance_sweep']],
                       ['base [m]', 'quad [m/m^2]', 'tol at 2 m', 'tol at 8 m',
                        'agree', 'disagree', 'agree frac']) + ['']
    L += ['### scene mask classes', '']
    L += table([[k, v['median'], v['p05'], v['p95']]
                for k, v in evidence['class_fractions'].items()],
               ['class', 'median frac', 'p05', 'p95']) + ['']
    fz = evidence['floor_z']
    if fz.get('n'):
        L += ['### scene_mask_floor_tol_m', '',
              f'FLOOR is |z| < {fz["floor_tol_m"]} m and REFLECTION is z below it, so these '
              'two distributions are what that threshold separates.', '']
        L += table([[k, v['n'], v['median'], v['p05'], v['p95']]
                    for k, v in fz['by_class'].items()],
                   ['class', 'pixels sampled', 'median z [m]', 'p05', 'p95']) + ['']
        L += [f'In the lower-centre quarter of the frame -- the floor straight ahead -- '
              f'{100 * (fz["lower_centre_invalid_frac_median"] or 0):.0f} % of pixels are '
              'INVALID (median over frames). The class split there is: ' +
              ', '.join(f'{k} {100 * v:.1f} %'
                        for k, v in fz['lower_centre_class_frac'].items() if v > 0.005) + '.', '']
    inf = evidence.get('inferred_floor') or {}
    if inf.get('frames'):
        L += ['### inferred floor (floor_inferred)', '',
              'The LiDAR-inferred floor is its own class, never FLOOR: it is an '
              'inference from the other sensor through two extrinsics '
              '(inferred_floor.py). These are the two numbers that have to be read '
              'together -- what the stereo SAW and what the LiDAR INFERRED.', '']
        L += table([[k, v] for k, v in inf['params'].items()],
                   ['parameter', 'value']) + ['']
        L += table([[inf['floor_frac_median'], inf['floor_inferred_frac_median'],
                     inf['applied_px_median'], inf['lut_px_median'],
                     inf['ms_median'], inf['ms_p95']]],
                   ['floor frac (median)', 'floor_inferred frac (median)',
                    'applied px (median)', 'inferable px (median)',
                    'ms median', 'ms p95']) + ['']
        L += ['Per cycle, what the inference did: ' +
              ', '.join(f'{k} {v}' for k, v in inf['cycles'].items()) +
              f' (of {inf["frames"]} depth frames).', '']
    sp = evidence['self_polygons']
    if sp.get('image_wh'):
        L += ['### scene_mask_self_polygons / scene_mask_occluded_polygons', '',
              f'Over {sp["frames"]} replayed depth frames at {sp["image_wh"][0]}x'
              f'{sp["image_wh"][1]}. "occluder" = no usable depth, or depth under 0.6 m, '
              'in at least 90 % of frames, which is what something this close to the lens '
              'actually looks like; "live scene" = a usable depth over 0.6 m in at least '
              'half of them.', '',
              f'- the self mask covers {100 * sp.get("self_mask_frac", 0):.1f} % of the frame',
              f'- the occluder footprint covers {100 * sp["occluder_frac"]:.1f} %, of which '
              f'{100 * sp.get("occluder_outside_self_frac", 0):.1f} % of the frame lies '
              'outside the mask', '']
        L += table([[e['index'], e['frac_of_frame'], e['px'], e['occluder_px'],
                     e['live_scene_px'], e['live_scene_frac'],
                     e.get('live_scene_depth_median_m'), e.get('occluder_bbox_frac'),
                     e['vertices']]
                    for e in sp['polygons']],
                   ['#', 'frac of frame', 'px', 'occluder px', 'live scene px',
                    'live frac', 'live median depth [m]', 'occluder bbox (frac)',
                    'vertices']) + ['']
    sc = evidence['stereo_columns']
    L += ['### scene_mask_column_percentile / column_min_px', '',
          f'Replayed frames {sc["frames"]}: {100 * (sc["finite_fraction_median"] or 0):.1f} % '
          f'of columns produce a distance; WALL pixels per non-empty column median '
          f'{sc["n_px_median"]}, p05 {sc["n_px_p05"]} against min_px '
          f'{evidence["current"]["front_clearance_scene_mask_column_min_px"]}.', '']
    L += ['## Videos', '']
    for k, v in videos.items():
        L += [f'- {k}: `{v}`']
    L += ['']
    return Path(path).write_text('\n'.join(L) + '\n')


def write_csv(path, cycles, rows_rec, rows_full, src, poses, window):
    cols = ['t_rel', 'stamp', 'moving', 'speed', 'rec_state', 'rec_value', 'rec_front_d_ray',
            'rep_state', 'rep_value', 'rep_front_d_ray', 'full_state', 'full_value',
            'rec_n_obs', 'rep_n_obs', 'rec_rejected', 'rep_rejected', 'rec_disagree',
            'rep_disagree', 'rep_n_tracks', 'rep_profile_age', 'rep_stereo_age',
            'rep_stereo_weight', 'scan_ms', 'tracker_ms']
    rec_obs = index_by_stamp(src.get('/perception/wall_observations'))
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for cycle, a, b in zip(cycles, rows_rec, rows_full or [None] * len(cycles)):
            rec = cycle['rec_dbg']
            k = round(cycle['stamp'], 6)
            ro = rec_obs.get(k)
            w.writerow([
                round(src.rel(cycle['stamp']), 3), round(cycle['stamp'], 6),
                int(in_window(window, cycle['stamp'])), num(poses.speed(cycle['stamp'])),
                rec.get('state'), num(rec.get('distance_m')), num(rec.get('front_d_ray_m')),
                a and a['state'], a and num(a['value']),
                a and num((a['dbg'] or {}).get('front_d_ray_m')),
                b and b['state'], b and num(b['value']),
                ro and len(ro.observations), a and a['obs'] and len(a['obs'].observations),
                ro and int(ro.n_rejected), a and a['obs'] and int(a['obs'].n_rejected),
                ro and int(ro.n_disagree_columns),
                a and a['obs'] and int(a['obs'].n_disagree_columns),
                a and a['walls'] and len(a['walls'].tracks),
                a and num(cycle['stamp'] - (a['profile_stamp'] or math.nan)),
                a and num(a.get('stereo_age')),
                a and num(a.get('stereo_weight'), 3),
                a and num(a['scan_ms'], 2), a and num(a['tracker_ms'], 2)])


def write_columns_csv(path, mask_rows, rows):
    by_stamp = {round(r['stamp'], 6): r for r in rows if r and r['check'] is not None}
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['stamp', 'finite_columns', 'wall_frac', 'floor_frac', 'reflection_frac',
                    'ceiling_frac', 'self_frac', 'object_frac', 'invalid_frac',
                    'classify_ms', 'agree', 'lidar_only', 'stereo_only', 'disagree', 'empty'])
        for row in mask_rows:
            cols = row.get('columns')
            frac = row.get('frac') or {}
            counts = {}
            cyc = by_stamp.get(round(row['stamp'], 6))
            if cyc is not None:
                counts = wall_fusion.status_counts(cyc['check']['status'])
            w.writerow([
                round(row['stamp'], 6),
                int(np.isfinite(np.asarray(cols.distance_m, float)).sum()) if cols else None,
                num(frac.get('wall'), 4), num(frac.get('floor'), 4),
                num(frac.get('reflection'), 4), num(frac.get('ceiling'), 4),
                num(frac.get('self'), 4), num(frac.get('object'), 4),
                num(frac.get('invalid'), 4), num(row.get('classify_ms'), 2),
                counts.get('agree'), counts.get('lidar_only'), counts.get('stereo_only'),
                counts.get('disagree'), counts.get('empty')])


# ============================================================================
# Driver
# ============================================================================

def load_compare(path, name):
    """The old summary.json for bag `name`: PATH itself, or <PATH>/<name>.summary.json."""
    path = Path(path).expanduser()
    if path.is_dir():
        hits = [path / f'{name}.summary.json'] + sorted(path.glob('*.summary.json'))
        path = next((h for h in hits if h.is_file()), path / f'{name}.summary.json')
    return path, json.loads(path.read_text())


def warmup_report(src, cycles, span):
    """Lines on how warm the replayed tracker is at the span start, and the car's head start."""
    lines = []
    if cycles:
        first = cycles[0]['stamp']
        start = span[0] if span else first
        n = sum(1 for c in cycles if c['stamp'] < start)
        lines.append(f'warm-up: replay cold from the first step at t = {src.rel(first):.2f} s; '
                     f'{max(start - first, 0.0):.2f} s ({n} steps) before the span starts '
                     f'at t = {src.rel(start):.2f} s')
    walls = src.get('/perception/walls')
    if walls:
        ages = [tr.age for tr in walls[0][2].tracks]
        if ages:
            lines.append(f'the CAR\'s oldest track at its first /perception/walls '
                         f'(t = {src.rel(walls[0][0]):.2f} s) was {max(ages):.1f} s old: '
                         'a head start no replay has')
    return lines


def run_bag(path, out_dir, video=True, stereo_passes=('rec', 'full'), mask_mode='auto',
            t0=None, t1=None, end_range=END_WALL_RANGE_M, tracker_steps_mode='car',
            compare_path=None):
    global LASER_Z
    out_dir = Path(out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    src = Src(path)
    name = src.name
    print(f'== {name}: {sum(src.counts.values())} messages on {len(src.counts)} topics'
          f' in {len(src.dbs)} file(s)')
    # Read the old summary first: a bad path should fail before the replay, not after.
    before = load_compare(compare_path, name) if compare_path is not None else None

    poses = Poses(src.get('/odometry/filtered'))
    window = moving_window(poses)
    info, info_json = inventory(src, poses, window)
    (out_dir / f'{name}.info.txt').write_text(info)
    for line in info.splitlines():
        if line.startswith('    ') and '..' in line:
            print('   moving window:' + line)
    missing = info_json['missing_required']
    if missing:
        print(f'   MISSING REQUIRED: {", ".join(missing)}')
    # --t0 / --t1 are relative to the first /scan stamp, like every printed t.
    span = None
    if t0 is not None or t1 is not None:
        span = (src.T0 + (t0 if t0 is not None else -1e9),
                src.T0 + (t1 if t1 is not None else 1e9))
        print(f'   summary/video span t = {src.rel(span[0]):.2f} .. {src.rel(span[1]):.2f} s '
              '(the replay itself still runs from the bag start)')

    to_base = slr.static_tf([m for _s, _r, m in src.get('/tf_static')])
    lp = to_base('laser')
    LASER_Z = float(lp[1][2]) if lp is not None else 0.19
    front_x, half = base_front_x(), front_side_limit_m()

    depth_topic = src.pick(DEPTH_TOPICS)
    # _publish_stereo_columns does `msg.header = header` and then rewrites that
    # header's frame_id to base_link. On the car each depth message is a fresh
    # deserialization, so nothing is lost; in a replay the SAME object is held
    # in the bag's message list, so its frame_id is rewritten in place. Read the
    # camera's own frame before the replay runs.
    cam_frame = (src.get(depth_topic)[0][2].header.frame_id
                 if depth_topic and src.get(depth_topic) else None)
    mask_rows, self_mask, self_polygons = [], None, []
    if depth_topic:
        print(f'   camera replay on {depth_topic} ({len(src.get(depth_topic))} frames)')
        cam_replay = MaskReplay(src, depth_topic)
        mask_rows = cam_replay.run()
        self_mask = cam_replay.node._self_mask
        self_polygons = list(cam_replay.node._self_polygons)
        cam_replay.close()
    else:
        print('   no depth in this bag: the camera half is not replayed')

    cycles, steps_info = recorded_cycles(src, tracker_steps_mode)
    print(f'   tracker steps: {len(cycles)} of {steps_info["payloads"]} recorded debug '
          f'payloads (--tracker-steps {tracker_steps_mode}; record format '
          f'{steps_info["format"]}, car decimation {steps_info["decimation"]}), '
          f'{sum(1 for c in cycles if c["scan"] is None)} without their scan')
    if steps_info['missing_steps'] or steps_info['breaks']:
        print(f'   PHASE: {steps_info["missing_steps"]} car tracker steps missing from the '
              f'record (their payload was not recorded: the replay cannot make them), '
              f'{steps_info["breaks"]} breaks in the tracker_skipped sequence')
    if steps_info['long_gaps']:
        print(f'   {steps_info["long_gaps"]} step-to-step gaps over 1.5x the median '
              f'{steps_info["median_step_s"]:.3f} s (dropped or unpublished steps)')
    for line in warmup_report(src, cycles, span):
        print('   ' + line)
    rows = {}
    rect_sources = set()
    for mode in stereo_passes:
        if mode == 'full' and not mask_rows:
            continue
        rp = WallReplay(src, stereo=mode, replayed=mask_rows)
        rect_sources.add(rp.rect_source)
        rows[mode] = rp.run(cycles)
        rp.close()
        print(f'   lidar replay "{mode}": {sum(1 for r in rows[mode] if r)} cycles')
    print(f'   rectangle check fed the {"/".join(sorted(str(x) for x in rect_sources))} '
          'class mask + depth pair (one source only)')
    rows_rec = rows.get('rec') or [None] * len(cycles)
    rows_full = rows.get('full')

    cmp_out = compare(src, cycles, rows_rec, rows_full, mask_rows)
    write_replay_md(out_dir / f'{name}.replay.md', src, cmp_out, cycles,
                    rows_rec, rows_full, mask_rows, depth_topic)
    write_csv(out_dir / f'{name}.replay.csv', cycles, rows_rec, rows_full, src, poses, window)
    write_columns_csv(out_dir / f'{name}.columns.csv', mask_rows, rows_rec)

    summary = summarize(src, cycles, rows_rec, mask_rows, poses, window, front_x, half, span)
    summary['headline'] = headline(src, cycles, {'rec': rows.get('rec'), 'full': rows_full},
                                   window, front_x, half, span, end_range)
    summary['tracker_steps'] = steps_info
    span_rows = [r for r in rows_rec if r and in_span(span, r['stamp'])]
    span_masks = [r for r in mask_rows if in_span(span, r['stamp'])]
    evidence = param_evidence(src, span_rows, span_masks, self_mask, self_polygons, window)

    compare_out = None
    if before is not None:
        before_path, before_doc = before
        old, notes = flat_headline(before_doc)
        new, _n = flat_headline({'summary': summary})
        compare_out = {'before': str(before_path), 'rows': compare_rows(old, new, notes)}
        if Path(str(before_doc.get('bag', ''))).name not in ('', name):
            print(f'   NOTE: {before_path} is for bag {before_doc.get("bag")}, not {name}')
        text = '\n'.join(compare_md(compare_out)) + '\n'
        (out_dir / f'{name}.compare.md').write_text(text)
        print(text)

    videos = {}
    if video:
        cam = camera_pose(src, cam_frame) if cam_frame else None
        masks = MaskSource(src, mask_rows, mask_mode, has_depth=bool(depth_topic))
        print(f'   camera video mask: {masks.source or masks.note} (--mask {mask_mode})')
        cam_span = clip_window(window, span)
        if cam_span:
            frames = build_timeline(src, rows_rec, mask_rows, poses, window,
                                    front_x, half, CAM_HZ, cam_span, masks)
            p = render_camera(src, frames, cam, out_dir / f'{name}.camera.mp4', masks,
                              bool(depth_topic))
            videos['camera (moving window, 5 Hz)'] = p
            print(f'   camera video: {p}')
        top_span = None
        if span is not None:
            scan_t = src.stamps('/scan') or [src.t0, src.t1]
            top_span = (max(span[0], scan_t[0]), min(span[1], scan_t[-1]))
        frames = build_timeline(src, rows_rec, mask_rows, poses, window,
                                front_x, half, TOP_HZ, top_span, masks)
        if frames:
            p = render_topdown(src, frames, window, front_x, half,
                               out_dir / f'{name}.topdown.mp4')
            videos['top-down (whole run, 10 Hz)' if span is None
                   else 'top-down (span, 10 Hz)'] = p
            print(f'   top-down video: {p}')

    write_summary_md(out_dir / f'{name}.summary.md', src, summary, evidence, window,
                     {k: str(v) for k, v in videos.items()}, steps_info, compare_out)
    (out_dir / f'{name}.summary.json').write_text(json.dumps(
        {'bag': str(src.path), 'info': info_json, 'compare': cmp_out,
         'summary': summary, 'evidence': evidence, 'compare_summary': compare_out,
         'options': {'mask': mask_mode, 't0': t0, 't1': t1, 'end_wall_range_m': end_range,
                     'tracker_steps': tracker_steps_mode, 'stereo': list(stereo_passes)},
         'videos': {k: str(v) for k, v in videos.items()}},
        indent=2, default=str))
    close_ro_connections()
    print(f'   wrote {out_dir}/{name}.*')
    return summary


def clip_window(window, span):
    """(first start, last end) of the moving window, cut to span; None if empty."""
    if not window:
        return None
    a, b = window[0][0], window[-1][1]
    if span is not None:
        a, b = max(a, span[0]), min(b, span[1])
    return (a, b) if b > a else None


def main():
    global SPEED_THRESHOLD, SPEED_PAD
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', type=Path)
    ap.add_argument('--out', type=Path, default=WS / 'mission_videos' / 'corridor_wall')
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--speed-threshold', type=float, default=SPEED_THRESHOLD)
    ap.add_argument('--speed-pad', type=float, default=SPEED_PAD)
    ap.add_argument('--stereo', default='rec,full',
                    help='which LiDAR passes to run: rec, full, or both (default)')
    ap.add_argument('--mask', choices=('auto', 'recorded', 'replay'), default='auto',
                    help='class mask drawn in the camera video: the recorded '
                         '/perception/scene_labels/classes (auto: when present, else none), '
                         'or the replay\'s recomputed one (default auto)')
    ap.add_argument('--t0', type=float, default=None,
                    help='summary/video span start [s from the first /scan]; the tracker '
                         'replay still starts at the bag start')
    ap.add_argument('--t1', type=float, default=None, help='summary/video span end [s]')
    ap.add_argument('--end-wall-range', type=float, default=END_WALL_RANGE_M,
                    help='end-wall error: moving cycles with the true strip range under '
                         'this [m] (default %(default)s)')
    ap.add_argument('--tracker-steps', choices=('car', 'debug'), default='car',
                    help='car: the tracker steps the car made (tracker_skipped == 0); '
                         'debug: one per recorded debug payload (pre-v2 behaviour)')
    ap.add_argument('--compare-summary', type=Path, default=None,
                    help='an older <bag>.summary.json (or the folder holding it): print '
                         'and write a before/after table of the headline metrics')
    cfg = ap.parse_args()
    SPEED_THRESHOLD, SPEED_PAD = cfg.speed_threshold, cfg.speed_pad
    passes = tuple(s.strip() for s in cfg.stereo.split(',') if s.strip())
    rclpy.init()
    try:
        for bag in cfg.bags:
            run_bag(bag, cfg.out, not cfg.no_video, passes, cfg.mask, cfg.t0, cfg.t1,
                    cfg.end_wall_range, cfg.tracker_steps, cfg.compare_summary)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
