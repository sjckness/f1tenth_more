#!/usr/bin/env python3
"""
mission_replay_video.py

Renders one top-down MP4 per mission bag showing, frame by frame and in the
same time base: what the MPC was allowed to drive through (its hard boundary
constraints), what it was being pushed away from (the camera-derived soft-cost
obstacles), what the car actually did, and -- in a HUD -- why. It is a READER:
it opens already-recorded bags and already-computed fields, and changes no
node's runtime behaviour.

WHAT "CORRIDOR" MEANS HERE (verified against the bags, not assumed)
------------------------------------------------------------------
The two obstacle paths in this stack are architecturally different and are
therefore drawn differently, so the video never conflates them:

  * HARD CONSTRAINT -- /costmap/boundaries (f1tenth_messages/
    BoundaryConstraintArray, published by f1tenth_costmap's
    costmap_boundary_node from lidar + /slam/map). Each entry is a halfspace
    `normal . p <= offset` in BASE_LINK (confirmed from the recorded
    header.frame_id; 2-3 of them per message in the sampled bags). These enter
    mpc_solver.py as actual inequality constraints, tightened at the point of
    use to `normal . p <= offset - car_radius - avoidance_margin`. Drawn as a
    SOLID line (the raw geometric wall) plus a DASHED line (the tightened line
    the car's centre must respect) with the excluded halfspace shaded.

  * SOFT COST -- /perception/obstacles_2d (Obstacle2DArray, base_link, from
    obstacle_projector_node). These are NOT constraints: mpc_solver.py adds a
    softplus penalty that starts biting at `r + car_radius + avoidance_margin`.
    Drawn as a filled disk with a soft GLOW ring at that activation radius --
    a gradient, not a wall, because that is what it is.

  * TRACKS + UNCERTAINTY -- /costmap/semantic_markers (map frame, CONFIRMED
    tracks only, from semantic_layer_node) for identity/position, and
    /camera/detections_3d for the per-detection 1-sigma position covariance
    (detection_3d_node writes var_xy into pose.covariance[0]/[7]; the tracker
    consumes it for alpha weighting and gate widening but does NOT republish
    it on the markers -- checked). So the uncertainty ellipse is drawn from
    the detections, the confirmed identity from the markers.

  * MPC REFERENCE CORRIDOR -- /mpc/corridor_markers (MarkerArray of three
    LINE_STRIPs: 'corridor_left', 'corridor_right', 'corridor_centerline', in
    the ODOM frame). This is build_straight_corridor()'s Bezier wall pair,
    half-width corr_wmin at the car widening to corr_wmax at the far end --
    the funnel Foxglove's 3D panel shows. It is NOT the three halfspaces above
    and must not be confused with them: the halfspaces are hard QP rows, this
    is the reference the corridor cost is shaped by. Drawn as a filled,
    widening band so the widening is visible frame to frame.

  * PREDICTED HORIZON -- MpcSolverStatus.pred_x/pred_y/pred_yaw/pred_v, one
    entry per horizon step, ODOM frame. These are mpc_solver's info["x_pred"],
    rolled forward through the true nonlinear model. Drawn fresh every frame as
    a ghost path that tapers and fades toward the far end of the horizon.

FRAMES
------
The corridor markers and the predicted horizon are published in ODOM (they come
from x0/build_straight_corridor's X0/Y0, never the map-frame EKF), while this
video draws in MAP. Both are transformed with the live map->odom TF from /tf,
per frame -- not with a fixed offset, which would be wrong by however far odom
has drifted at that instant.

OLDER BAGS
----------
The horizon fields were appended to MpcSolverStatus after the first three
mission videos were rendered, and /mpc/corridor_markers was added to
mission_logger_node's topic list at the same time. A bag recorded before that
therefore (a) has no corridor markers at all, and (b) carries a SHORTER
MpcSolverStatus than the installed one, which the normal deserializer rejects
outright. _decode_legacy_solver_status() below reads that older layout by hand
so those bags keep their solver HUD instead of silently losing it.

TIME BASE
---------
Every stream is resampled onto a uniform grid at --dt (default 0.1s, the MPC's
own control period `ts`) by zero-order hold with a per-stream max age, so the
picture at frame k is "what was current at that instant", not an interpolation
across topics that run at wildly different rates (camera ~12.6Hz vs odometry
~40Hz). Bag RECEIVE timestamps are the clock -- one clock for every topic, so
nothing can desync because one publisher stamped its header from a different
source.

USAGE
-----
    ros2 run f1tenth_logger mission_replay_video             # 3 most recent missions
    ros2 run f1tenth_logger mission_replay_video --count 5
    ros2 run f1tenth_logger mission_replay_video <bag_dir> [<bag_dir> ...]
    ros2 run f1tenth_logger mission_replay_video --speed 0.5      # slow-motion
    ros2 run f1tenth_logger mission_replay_video --follow 6.0     # camera follows the car

(Was `python3 scripts/mission_replay_video.py` before this module moved into
f1tenth_logger; `python3 -m f1tenth_logger.mission_replay_video` also works
from a sourced workspace.)

Videos are written to <workspace root>/mission_videos/<run_id>.mp4 (gitignored)
unless --out-dir says otherwise.

Requires a sourced ROS 2 workspace (rosbag2_py + f1tenth_messages), matplotlib
and ffmpeg.
"""

import argparse
import json
import math
import sys
import time
from bisect import bisect_right
from pathlib import Path

import numpy as np

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle, Ellipse, Polygon as MplPolygon  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402


DEFAULT_BAG_ROOT = Path.home() / '.ros' / 'mission_bags'
# Videos land in the workspace root (gitignored), NOT next to the bags: the
# bag root lives on the scratch volume and is what gets rsync'd/pruned, while
# this is the directory anyone working in the repo already has open. Resolved
# from this file's own location so it does not depend on the cwd the script
# happens to be run from.
#
# The walk up to the workspace root replaces a plain parents[1], which was
# correct only while this file lived at <workspace>/scripts/. It now lives at
# <workspace>/src/f1tenth_logger/f1tenth_logger/, so a fixed index would point
# at the package directory and silently write videos somewhere new. Walking to
# the parent of the enclosing 'src' keeps the SAME output directory the 24
# already-rendered videos are in, and keeps working if the package is ever
# nested differently. resolve() first: --symlink-install makes the installed
# copy a symlink back into src/, and src/ is the tree with a workspace root
# above it.


def _workspace_root(start: Path) -> Path:
    for parent in start.parents:
        if parent.name == 'src':
            return parent.parent
    # Installed outside a source workspace (a real install/ tree, no src/
    # above it): fall back to the cwd rather than guessing at an index.
    return Path.cwd()


DEFAULT_OUT_DIR = _workspace_root(Path(__file__).resolve()) / 'mission_videos'

# Palette: dark surface + the reference categorical slots 1-3, which are the
# ones validated for ALL-pairs separation (obstacle classes appear side by side
# on the map, so adjacent-pair validation would be the wrong gate). A 4th class
# folds into OTHER grey rather than inventing a hue; every track is also
# direct-labelled with its class name, so identity is never colour-alone.
SURFACE = '#1a1a19'
PANEL = '#121211'
INK = '#ffffff'
INK_SECONDARY = '#c3c2b7'
INK_MUTED = '#898781'
GRID = '#2c2c2a'
CLASS_COLORS = ['#3987e5', '#d95926', '#199e70']
CLASS_OTHER = '#898781'
STATUS = {'good': '#0ca30c', 'warning': '#fab219', 'serious': '#ec835a',
          'critical': '#d03b3b'}
HARD_COLOR = '#c3c2b7'          # hard constraint: neutral ink, reads as structure
CORRIDOR_COLOR = '#3987e5'      # MPC reference corridor: the funnel band
# Predicted horizon: magenta, deliberately NOT white and NOT the corridor's
# blue -- the executed trail is white and the corridor band is blue, and a
# prediction that looks like either of those is a prediction being read as
# fact.
HORIZON_COLOR = '#d55181'
CAR_COLOR = '#ffffff'
MAP_CMAP = LinearSegmentedColormap.from_list(
    'slam_occ', ['#242423', '#3a3a37', '#7e7c74'])

# Per-stream max age for the zero-order hold, seconds. 0.5 for boundaries is
# not a guess: it is the same staleness gate MPC_corr.py applies to them
# (odom_stale_timeout_sec, reused by _get_live_boundaries) -- past it the
# solver itself would have dropped them, so the video must too.
MAX_AGE = {
    'pose': 0.5,
    'boundaries': 0.5,
    'clearance': 0.5,
    'obstacles': 0.5,
    'detections': 0.5,
    'markers': 1.0,
    # The corridor is rebuilt at 0.5 * ts (see MPC_corr's corridor_update_
    # period) and the horizon comes with every solve, so both are as live as
    # the control loop itself; 0.5s only bridges a dropped message.
    'corridor': 0.5,
    'map_to_odom': 0.5,
    'tree': 0.5,
    'solver': 0.5,
    'drive': 0.5,
    'stop': 0.25,
}

TOPICS = {
    'pose_global': '/ekf_global/odometry/filtered',
    'pose_local': '/odometry/filtered',
    'boundaries': '/costmap/boundaries',
    'clearance': '/costmap/front_clearance',
    'obstacles': '/perception/obstacles_2d',
    'detections': '/camera/detections_3d',
    'markers': '/costmap/semantic_markers',
    'corridor': '/mpc/corridor_markers',
    'tree': '/behavior/tree_status',
    'solver': '/mpc/solver_status',
    'stop': '/safety_stop',
    'drive': '/drive',
    'map': '/slam/map',
    'tf': '/tf',
    'tf_static': '/tf_static',
}


# --------------------------------------------------------------------------
# bag discovery
# --------------------------------------------------------------------------
def discover_bags(bag_root: Path, count: int):
    """Newest `count` mission bags by MANIFEST start_time (not directory
    mtime): the manifest is the mission's own record of when the run began,
    and a bag directory's mtime moves whenever anything touches it."""
    entries = []
    for manifest_path in sorted(bag_root.glob('*.manifest.json')):
        try:
            manifest = json.loads(manifest_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            print(f'  skipping {manifest_path.name}: {exc}', file=sys.stderr)
            continue
        bag_dir = bag_root / manifest.get('run_id', manifest_path.stem)
        if not (bag_dir / 'metadata.yaml').exists():
            # bag_path in the manifest can point at the recording machine's own
            # path (~/.ros/...) even when the bags now live somewhere else, so
            # resolve against bag_root and skip if there is genuinely no bag.
            continue
        entries.append((manifest.get('start_time', ''), bag_dir, manifest))
    entries.sort(key=lambda e: e[0], reverse=True)
    return entries[:count]


def load_manifest_for(bag_dir: Path):
    manifest_path = bag_dir.parent / f'{bag_dir.name}.manifest.json'
    if manifest_path.exists():
        try:
            return json.loads(manifest_path.read_text())
        except (OSError, json.JSONDecodeError):
            pass
    return {}


# --------------------------------------------------------------------------
# small geometry helpers
# --------------------------------------------------------------------------
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def body_to_map(px, py, pose):
    """(x, y) in base_link -> map, using the frame's own vehicle pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def odom_points_to_map(points, map_to_odom):
    """Transform a polyline from the MPC's odom frame into map with the live
    map->odom TF (the pose of odom expressed in map). Identity when the caller
    is already drawing in odom (--pose-source local) or when no TF is fresh --
    the latter is reported once per bag rather than silently drawn wrong."""
    if map_to_odom is None:
        return None
    ox, oy, oyaw = map_to_odom
    c, s_ = math.cos(oyaw), math.sin(oyaw)
    return [(ox + c * px - s_ * py, oy + s_ * px + c * py) for px, py in points]


def boundary_to_map(nx, ny, offset, pose):
    """Halfspace `n . p <= offset` from base_link into map. Same derivation as
    MPC_corr.py's own _boundary_to_world (which transforms into the MPC's
    odom-frame world instead) -- kept identical so the drawn line is the line
    the solver constrained against, not a lookalike."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    nwx = c * nx - s * ny
    nwy = s * nx + c * ny
    return nwx, nwy, offset + nwx * x + nwy * y


def clip_polygon_halfspace(poly, nx, ny, c):
    """Sutherland-Hodgman clip of `poly` down to {p : n . p <= c}. Used to
    shade the EXCLUDED side by clipping the view rectangle with the flipped
    halfspace, which stays correct for any line/rectangle intersection
    (including a line that misses the view entirely -> empty polygon)."""
    out = []
    n = len(poly)
    for i in range(n):
        cur = poly[i]
        nxt = poly[(i + 1) % n]
        d_cur = nx * cur[0] + ny * cur[1] - c
        d_nxt = nx * nxt[0] + ny * nxt[1] - c
        if d_cur <= 0:
            out.append(cur)
        if (d_cur <= 0) != (d_nxt <= 0):
            t = d_cur / (d_cur - d_nxt)
            out.append((cur[0] + t * (nxt[0] - cur[0]),
                        cur[1] + t * (nxt[1] - cur[1])))
    return out


def segment_near(nx, ny, c, cx, cy, span):
    """The piece of `n . p = c` within `span` metres (along the line) of the
    foot of the perpendicular from (cx, cy) -- i.e. the stretch of wall beside
    the car, not the infinite line."""
    norm = math.hypot(nx, ny)
    if norm < 1e-9:
        return None
    ux, uy = nx / norm, ny / norm
    cc = c / norm
    d = ux * cx + uy * cy - cc               # signed distance car -> line
    fx, fy = cx - d * ux, cy - d * uy        # foot of the perpendicular
    dx, dy = -uy, ux
    return ((fx - dx * span, fy - dy * span), (fx + dx * span, fy + dy * span))


def _decode_legacy_solver_status(raw: bytes):
    """Hand-decode a PRE-horizon MpcSolverStatus (the 9-scalar layout, before
    prediction_frame_id/pred_* were appended) straight from its CDR bytes.

    Appending fields to a .msg is wire-incompatible in the reading direction:
    the installed deserializer expects the longer layout and raises
    "Fast CDR exception" on every older message, which would silently strip the
    solver panel -- the single most useful part of the HUD -- from every bag
    recorded before the change. Rather than lose that, decode the old layout
    directly: plain little-endian CDR with the standard 4-byte-encapsulation
    prefix and natural alignment relative to the body start.

    Returns the same tuple shape read_bag builds for a current message (with an
    empty horizon), or None if the bytes are not that layout either.
    """
    import struct

    if len(raw) < 4:
        return None
    body = raw[4:]                       # skip the encapsulation header
    off = 0

    def align(n):
        nonlocal off
        off += (-off) % n

    def take(fmt, size, alignment):
        nonlocal off
        align(alignment)
        if off + size > len(body):
            raise ValueError('truncated')
        val = struct.unpack_from(fmt, body, off)[0]
        off += size
        return val

    def take_string():
        nonlocal off
        length = take('<I', 4, 4)
        if off + length > len(body):
            raise ValueError('truncated string')
        val = body[off:off + length - 1].decode('utf-8', 'replace') if length else ''
        off += length
        return val

    try:
        take('<i', 4, 4)                 # header.stamp.sec
        take('<I', 4, 4)                 # header.stamp.nanosec
        take_string()                    # header.frame_id
        success = bool(take('<?', 1, 1))
        status = take('<i', 4, 4)
        status_message = take_string()
        solve_dt = take('<f', 4, 4)
        period = take('<f', 4, 4)
        cost = take('<f', 4, 4)
        backend = take_string()
        n_boundary = take('<i', 4, 4)
        n_obstacles = take('<i', 4, 4)
    except (ValueError, struct.error):
        return None
    return (success, status, status_message, solve_dt, period, cost, backend,
            n_boundary, n_obstacles, [], [], [], [], '')


# --------------------------------------------------------------------------
# bag reading
# --------------------------------------------------------------------------
class Stream:
    """Timestamped samples plus a zero-order-hold lookup with a max age."""

    def __init__(self, max_age):
        self.t = []
        self.v = []
        self.max_age = max_age

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def at(self, t):
        """Value in force at time t, or None if nothing is fresh enough."""
        i = bisect_right(self.t, t) - 1
        if i < 0:
            return None
        if self.max_age is not None and (t - self.t[i]) > self.max_age:
            return None
        return self.v[i]

    def __len__(self):
        return len(self.t)


def read_bag(bag_dir: Path, pose_source: str):
    """One pass over the bag, decoding only the topics this video needs.

    Deliberately topic-filtered: these bags carry /camera/image_annotated and
    /camera/detection_masks, and deserializing every image would dominate the
    runtime for pixels this video never draws.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in TOPICS.values() if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    pose_topic = TOPICS['pose_global'] if pose_source == 'global' else TOPICS['pose_local']
    data = {
        'pose': Stream(MAX_AGE['pose']),
        'boundaries': Stream(MAX_AGE['boundaries']),
        'clearance': Stream(MAX_AGE['clearance']),
        'obstacles': Stream(MAX_AGE['obstacles']),
        'detections': Stream(MAX_AGE['detections']),
        'markers': Stream(MAX_AGE['markers']),
        'tree': Stream(MAX_AGE['tree']),
        'solver': Stream(MAX_AGE['solver']),
        'stop': Stream(MAX_AGE['stop']),
        'drive': Stream(MAX_AGE['drive']),
        'map': Stream(None),        # occupancy grid: latched-ish, never expires
        'corridor': Stream(MAX_AGE['corridor']),
        'map_to_odom': Stream(MAX_AGE['map_to_odom']),
    }
    static_tf = {}
    pose_frame = None
    t0 = None
    tend = None
    # Semantic markers are a DIFF stream (ADD/DELETE per (ns, id)), so the set
    # of live tracks has to be accumulated as we read and snapshotted per
    # message -- taking a single MarkerArray in isolation would show only the
    # tracks that happened to change that tick.
    live_tracks = {}
    legacy_solver_msgs = [0]
    dropped = {}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        tend = t
        try:
            msg = deserialize_message(raw, get_message(types[topic]))
        except Exception:                       # noqa: BLE001
            # Only one topic in this set has ever changed layout; anything else
            # failing here is a genuinely corrupt/foreign message and is
            # dropped rather than guessed at.
            if topic == TOPICS['solver']:
                legacy = _decode_legacy_solver_status(raw)
                if legacy is not None:
                    data['solver'].add(t, legacy)
                    legacy_solver_msgs[0] += 1
                    continue
            dropped[topic] = dropped.get(topic, 0) + 1
            continue

        if topic == pose_topic:
            pose_frame = msg.header.frame_id
            p = msg.pose.pose
            tw = msg.twist.twist
            data['pose'].add(t, (p.position.x, p.position.y,
                                 yaw_from_quat(p.orientation),
                                 tw.linear.x))
        elif topic == TOPICS['boundaries']:
            data['boundaries'].add(t, [(c.normal[0], c.normal[1], c.offset)
                                       for c in msg.constraints
                                       if math.isfinite(c.offset)])
        elif topic == TOPICS['clearance']:
            data['clearance'].add(t, float(msg.data))
        elif topic == TOPICS['obstacles']:
            data['obstacles'].add(t, [(o.x, o.y, o.r) for o in msg.obstacles])
        elif topic == TOPICS['detections']:
            dets = []
            for det in msg.detections:
                if not det.results:
                    continue
                h = det.results[0]
                var = max(float(h.pose.covariance[0]), float(h.pose.covariance[7]))
                dets.append((h.hypothesis.class_id, float(h.hypothesis.score),
                             h.pose.pose.position.x, h.pose.pose.position.y,
                             h.pose.pose.position.z,
                             math.sqrt(var) if var > 0.0 else None))
            data['detections'].add(t, (msg.header.frame_id, dets))
        elif topic == TOPICS['markers']:
            for mk in msg.markers:
                key = (mk.ns, mk.id)
                if mk.action == 2:                       # DELETE
                    live_tracks.pop(key, None)
                elif mk.type == 3:                       # CYLINDER = track disk
                    live_tracks[key] = ('disk', mk.ns, mk.pose.position.x,
                                        mk.pose.position.y, mk.scale.x * 0.5, '')
                elif mk.type == 9:                       # TEXT = label
                    live_tracks[key] = ('label', mk.ns, mk.pose.position.x,
                                        mk.pose.position.y, 0.0, mk.text)
            data['markers'].add(t, list(live_tracks.values()))
        elif topic == TOPICS['tree']:
            data['tree'].add(t, (msg.active_lane, list(msg.lane_names),
                                 list(msg.lane_statuses), msg.emergency_trip,
                                 bool(msg.safety_stop_active), msg.stop_source))
        elif topic == TOPICS['solver']:
            data['solver'].add(t, (bool(msg.success), int(msg.status),
                                   msg.status_message, float(msg.solve_dt_sec),
                                   float(msg.control_period_sec), float(msg.cost),
                                   msg.solver, int(msg.n_boundary_constraints),
                                   int(msg.n_obstacles),
                                   list(msg.pred_x), list(msg.pred_y),
                                   list(msg.pred_yaw), list(msg.pred_v),
                                   msg.prediction_frame_id))
        elif topic == TOPICS['stop']:
            # frame_id is 'base_link/emergency' or 'base_link/obstacle' -- the
            # only thing distinguishing the two Stop publishers on this topic.
            src = msg.header.frame_id.split('/')[-1] if msg.header.frame_id else ''
            data['stop'].add(t, (src, float(msg.drive.speed)))
        elif topic == TOPICS['drive']:
            data['drive'].add(t, (float(msg.drive.speed),
                                  float(msg.drive.steering_angle)))
        elif topic == TOPICS['map']:
            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            res = msg.info.resolution
            data['map'].add(t, (grid, (ox, ox + msg.info.width * res,
                                       oy, oy + msg.info.height * res)))
        elif topic == TOPICS['corridor']:
            # Three LINE_STRIPs keyed by ns; kept as separate polylines rather
            # than merged, because the fill between left and right is built per
            # frame from the pair (and the centerline is drawn on top).
            walls = {}
            for mk in msg.markers:
                if mk.action != 0 or not mk.points:
                    continue
                walls[mk.ns] = [(pt.x, pt.y) for pt in mk.points]
            if walls:
                data['corridor'].add(t, walls)
        elif topic == TOPICS['tf']:
            for tr in msg.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    data['map_to_odom'].add(t, (
                        tr.transform.translation.x, tr.transform.translation.y,
                        yaw_from_quat(tr.transform.rotation)))
        elif topic == TOPICS['tf_static']:
            for tr in msg.transforms:
                static_tf[tr.child_frame_id] = (
                    tr.header.frame_id,
                    np.array([tr.transform.translation.x,
                              tr.transform.translation.y,
                              tr.transform.translation.z]),
                    quat_to_matrix(tr.transform.rotation.x, tr.transform.rotation.y,
                                   tr.transform.rotation.z, tr.transform.rotation.w))

    return {'streams': data, 'static_tf': static_tf, 't0': t0, 't1': tend,
            'pose_frame': pose_frame, 'pose_topic': pose_topic,
            'legacy_solver_msgs': legacy_solver_msgs[0], 'dropped': dropped}


def static_chain_to_base(static_tf, frame, base='base_link'):
    """Compose the static transform `frame` -> `base` (rotation, translation),
    or None if the chain does not reach base. Camera detections arrive in
    zed2_left_camera_frame; the chain up to base_link is entirely static
    (/tf_static), so no time-varying lookup is needed."""
    rot = np.eye(3)
    trans = np.zeros(3)
    cur = frame
    for _ in range(16):
        if cur == base:
            return rot, trans
        entry = static_tf.get(cur)
        if entry is None:
            return None
        parent, t_pc, r_pc = entry
        rot = r_pc @ rot
        trans = r_pc @ trans + t_pc
        cur = parent
    return None


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------
def class_color(class_id, order):
    if class_id not in order:
        if len(order) < len(CLASS_COLORS):
            order[class_id] = len(order)
        else:
            order[class_id] = None
    slot = order[class_id]
    return CLASS_COLORS[slot] if slot is not None else CLASS_OTHER


def compute_view(bag, dt):
    """Fixed view box covering the whole run: the car never leaves frame and
    the eye has a stable reference, which a per-frame autoscale destroys."""
    xs, ys = [], []
    for (x, y, _yaw, _v) in bag['streams']['pose'].v:
        xs.append(x)
        ys.append(y)
    for tracks in bag['streams']['markers'].v:
        for (kind, _ns, x, y, _r, _txt) in tracks:
            if kind == 'disk':
                xs.append(x)
                ys.append(y)
    if not xs:
        return (-5, 5), (-5, 5)
    pad = 2.5
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    # Square the box so equal-aspect axes do not letterbox unpredictably.
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    half = 0.5 * max(x1 - x0, y1 - y0)
    return (cx - half, cx + half), (cy - half, cy + half)


def draw_frame(axes, bag, t, rel_t, cfg, state):
    ax_map, ax_hud, ax_time = axes
    streams = bag['streams']
    ax_map.clear()
    ax_hud.clear()
    ax_time.clear()

    pose = streams['pose'].at(t)
    xlim, ylim = state['view']
    if cfg.follow and pose is not None:
        h = cfg.follow
        xlim = (pose[0] - h, pose[0] + h)
        ylim = (pose[1] - h, pose[1] + h)

    ax_map.set_facecolor(SURFACE)
    ax_map.set_xlim(*xlim)
    ax_map.set_ylim(*ylim)
    ax_map.set_aspect('equal')
    ax_map.grid(True, color=GRID, linewidth=0.5, alpha=0.6)
    ax_map.tick_params(colors=INK_MUTED, labelsize=7)
    for spine in ax_map.spines.values():
        spine.set_color(GRID)
    ax_map.set_xlabel('x [m]', color=INK_MUTED, fontsize=8)
    ax_map.set_ylabel('y [m]', color=INK_MUTED, fontsize=8)

    # --- SLAM occupancy grid, recessive: context, never a subject ---------
    grid_entry = streams['map'].at(t)
    if grid_entry is not None and not cfg.no_map:
        grid, extent = grid_entry
        # Unknown (-1) -> NaN -> fully transparent, so "never mapped" reads as
        # the page's own background rather than as free space. Free -> barely
        # above the surface, occupied -> light: on a dark surface the WALLS
        # must be the bright thing (matplotlib's own 'Greys' would do the
        # opposite and turn free space into a glowing slab).
        occ = np.where(grid < 0, np.nan, grid.astype(float))
        ax_map.imshow(occ, origin='lower', extent=extent, cmap=MAP_CMAP,
                      vmin=0, vmax=100, alpha=0.9, zorder=0,
                      interpolation='nearest')

    # --- hard-constraint corridor ----------------------------------------
    bounds = streams['boundaries'].at(t)
    if bounds and pose is not None:
        for (nx, ny, offset) in bounds:
            nwx, nwy, c_raw = boundary_to_map(nx, ny, offset, pose[:3])
            norm = math.hypot(nwx, nwy)
            if norm < 1e-9:
                continue
            c_pad = c_raw - (cfg.car_radius + cfg.avoidance_margin) * norm
            # Shade the whole excluded halfspace (it IS infinite -- the solver
            # applies it everywhere), but draw the lines only within
            # --corridor-span of the car: an unbounded line pair per constraint
            # crossing the whole map reads as three random diagonals, and the
            # part that decides anything is the part beside the vehicle.
            excluded = clip_polygon_halfspace(
                [(xlim[0], ylim[0]), (xlim[1], ylim[0]),
                 (xlim[1], ylim[1]), (xlim[0], ylim[1])],
                -nwx, -nwy, -c_raw)
            if len(excluded) >= 3:
                ax_map.add_patch(MplPolygon(excluded, closed=True, facecolor=HARD_COLOR,
                                            alpha=0.13, edgecolor='none', zorder=1))
            for c_line, width, style, alpha in ((c_raw, 2.2, 'solid', 0.95),
                                                (c_pad, 1.2, (0, (5, 4)), 0.7)):
                seg = segment_near(nwx, nwy, c_line, pose[0], pose[1], cfg.corridor_span)
                if seg:
                    ax_map.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
                                color=HARD_COLOR, linewidth=width, linestyle=style,
                                alpha=alpha, zorder=4, solid_capstyle='round')

    # --- MPC reference corridor: the widening funnel ----------------------
    # Drawn as a filled band between the left and right Bezier walls, NOT as
    # two outlines: the whole point of this layer is that the corridor starts
    # narrow at the car and opens out ahead, and a filled region makes that
    # legible in a single frame where two diverging lines do not.
    walls = streams['corridor'].at(t)
    if walls:
        m2o = state['map_to_odom_fn'](t)
        left = odom_points_to_map(walls.get('corridor_left', []), m2o)
        right = odom_points_to_map(walls.get('corridor_right', []), m2o)
        center = odom_points_to_map(walls.get('corridor_centerline', []), m2o)
        if left and right:
            band = left + right[::-1]
            ax_map.add_patch(MplPolygon(band, closed=True,
                                        facecolor=CORRIDOR_COLOR, alpha=0.16,
                                        edgecolor='none', zorder=1.5))
            for wall in (left, right):
                ax_map.plot([px for px, _ in wall], [py for _, py in wall],
                            color=CORRIDOR_COLOR, linewidth=1.4, alpha=0.85,
                            zorder=3.5, solid_capstyle='round')
        if center:
            ax_map.plot([px for px, _ in center], [py for _, py in center],
                        color=CORRIDOR_COLOR, linewidth=0.9, alpha=0.5,
                        linestyle=(0, (4, 4)), zorder=3.5)

    # --- soft-cost obstacles (camera path) --------------------------------
    obstacles = streams['obstacles'].at(t)
    if obstacles and pose is not None:
        activation = cfg.car_radius + cfg.avoidance_margin
        for (ox, oy, r) in obstacles:
            mx, my = body_to_map(ox, oy, pose[:3])
            # Glow: concentric rings out to the radius at which mpc_solver's
            # softplus penalty starts biting -- a gradient, because the
            # mechanism is a gradient, unlike the hard lines above.
            for k in range(6):
                frac = 1.0 - k / 6.0
                ax_map.add_patch(Circle(
                    (mx, my), r + activation * frac, facecolor=STATUS['serious'],
                    alpha=0.055, edgecolor='none', zorder=2))
            ax_map.add_patch(Circle((mx, my), r + activation, facecolor='none',
                                    edgecolor=STATUS['serious'], linewidth=0.9,
                                    linestyle=(0, (2, 3)), alpha=0.8, zorder=3))
            ax_map.add_patch(Circle((mx, my), max(r, 0.05),
                                    facecolor=STATUS['serious'], alpha=0.85,
                                    edgecolor='none', zorder=5))

    # --- raw detections: uncertainty ellipses ------------------------------
    det_entry = streams['detections'].at(t)
    if det_entry is not None and pose is not None and state['cam_to_base'] is not None:
        rot, trans = state['cam_to_base']
        _frame, dets = det_entry
        for (class_id, score, dx, dy, dz, sigma) in dets:
            p_base = rot @ np.array([dx, dy, dz]) + trans
            mx, my = body_to_map(p_base[0], p_base[1], pose[:3])
            color = class_color(class_id, state['class_order'])
            if sigma:
                for k in (1.0, 2.0):
                    ax_map.add_patch(Ellipse(
                        (mx, my), 2 * k * sigma, 2 * k * sigma, facecolor=color,
                        alpha=0.10 if k == 2.0 else 0.16, edgecolor=color,
                        linewidth=0.8, linestyle=(0, (1, 2)), zorder=3))
            ax_map.plot([mx], [my], marker='x', markersize=7, color=color,
                        alpha=0.85, markeredgewidth=1.6, zorder=6)

    # --- confirmed semantic tracks ----------------------------------------
    tracks = streams['markers'].at(t)
    if tracks:
        for (kind, ns, tx, ty, tr, txt) in tracks:
            if kind != 'disk':
                continue
            color = class_color(ns, state['class_order'])
            ax_map.add_patch(Circle((tx, ty), max(tr, 0.12) * 2.2, facecolor=color,
                                    alpha=0.13, edgecolor='none', zorder=6))
            ax_map.add_patch(Circle((tx, ty), max(tr, 0.12), facecolor=color,
                                    alpha=0.95, edgecolor=SURFACE, linewidth=1.5,
                                    zorder=7))
        for (kind, ns, tx, ty, _tr, txt) in tracks:
            if kind != 'label':
                continue
            ax_map.text(tx + 0.18, ty + 0.18, txt or ns, color=INK_SECONDARY,
                        fontsize=7.5, zorder=8,
                        path_effects=None)

    # --- car + fading trail ------------------------------------------------
    trail = [(tt, v) for tt, v in zip(streams['pose'].t, streams['pose'].v)
             if t - cfg.trail_seconds <= tt <= t]
    if len(trail) >= 2:
        pts = np.array([[v[0], v[1]] for _tt, v in trail])
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        alphas = np.linspace(0.04, 0.65, len(segs))
        lc = LineCollection(segs, colors=[(1, 1, 1, a) for a in alphas],
                            linewidths=1.4, zorder=8)
        ax_map.add_collection(lc)
    if pose is not None:
        x, y, yaw, _v = pose
        c, s = math.cos(yaw), math.sin(yaw)
        body = [(0.26, 0.0), (-0.12, 0.13), (-0.05, 0.0), (-0.12, -0.13)]
        pts = [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in body]
        ax_map.add_patch(MplPolygon(pts, closed=True, facecolor=CAR_COLOR,
                                    edgecolor=SURFACE, linewidth=1.0, zorder=10))
        ax_map.add_patch(Circle((x, y), cfg.car_radius, facecolor='none',
                                edgecolor=CAR_COLOR, alpha=0.35, linewidth=0.9,
                                linestyle=(0, (2, 3)), zorder=9))

    # --- MPC predicted horizon --------------------------------------------
    # Fresh every frame (it is re-solved every tick), starting at the car's
    # own position so the ghost reads as "from here, this is where I think I
    # am going". Tapered width + fading alpha toward the far end: the far end
    # of an MPC horizon is the least trustworthy part of it, and the styling
    # should say so rather than drawing it as confidently as the executed
    # trail (which is solid white) or the corridor walls (which are blue).
    solver_frame = streams['solver'].at(t)
    if solver_frame is not None and solver_frame[9] and pose is not None:
        m2o = state['map_to_odom_fn'](t)
        horizon = odom_points_to_map(list(zip(solver_frame[9], solver_frame[10])), m2o)
        if horizon:
            pts = np.array([[pose[0], pose[1]]] + horizon)
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            n_seg = len(segs)
            fade = np.linspace(0.95, 0.15, n_seg)
            widths = np.linspace(2.4, 0.7, n_seg)
            rgb = matplotlib.colors.to_rgb(HORIZON_COLOR)
            ax_map.add_collection(LineCollection(
                segs, colors=[(*rgb, a) for a in fade], linewidths=widths,
                zorder=11, capstyle='round'))
            # One dot per predicted STATE (not a smooth curve): the horizon is
            # a discrete sequence of states at ts intervals, and their spacing
            # is itself the predicted speed profile.
            ax_map.plot(pts[1:, 0], pts[1:, 1], linestyle='none', marker='o',
                        markersize=2.6, color=HORIZON_COLOR, alpha=0.75,
                        zorder=11)
            ax_map.plot([pts[-1, 0]], [pts[-1, 1]], marker='o', markersize=5.0,
                        markerfacecolor='none', markeredgecolor=HORIZON_COLOR,
                        markeredgewidth=1.2, alpha=0.9, zorder=11)

    # --- HUD ---------------------------------------------------------------
    stop = streams['stop'].at(t)
    tree = streams['tree'].at(t)
    solver = streams['solver'].at(t)
    drive = streams['drive'].at(t)
    clearance = streams['clearance'].at(t)

    ax_hud.set_facecolor(PANEL)
    ax_hud.set_xlim(0, 1)
    ax_hud.set_ylim(0, 1)
    ax_hud.set_xticks([])
    ax_hud.set_yticks([])
    for spine in ax_hud.spines.values():
        spine.set_color(GRID)

    lines = []
    lines.append((f"{state['mission_id']}", INK, 10.5, 'bold'))
    lines.append((f"outcome {state['outcome']}   {state['run_id'][:19]}",
                  INK_MUTED, 7.5, 'normal'))
    lines.append((f"t = {rel_t:6.2f} s / {state['duration']:.2f} s", INK_SECONDARY, 9, 'normal'))
    lines.append(('', INK, 4, 'normal'))

    if stop is not None:
        src = stop[0] or 'unknown'
        lines.append((f"SAFETY STOP  ({src})", STATUS['critical'], 11, 'bold'))
    elif tree is not None and tree[4]:
        lines.append((f"SAFETY STOP  ({tree[5] or 'unknown'})", STATUS['critical'], 11, 'bold'))
    else:
        lines.append(('driving', STATUS['good'], 10, 'bold'))

    if tree is not None:
        active, names, statuses, trip, _sa, _src = tree
        lines.append((f"BT lane: {active or '(root failed)'}", INK_SECONDARY, 8.5, 'normal'))
        lines.append(('  ' + '  '.join(f'{n}:{s[:4]}' for n, s in zip(names, statuses)),
                      INK_MUTED, 7, 'normal'))
        if trip:
            lines.append((f"  emergency trip: {trip}", STATUS['critical'], 8.5, 'normal'))
    else:
        lines.append(('BT lane: (no tick)', INK_MUTED, 8.5, 'normal'))

    lines.append(('', INK, 4, 'normal'))
    if solver is not None:
        (ok, status, status_msg, dt_s, period, cost, backend, nb, nobs,
         pred_x, _pred_y, _pred_yaw, pred_v, _pred_frame) = solver
        color = STATUS['good'] if ok else STATUS['critical']
        lines.append((f"MPC: {'solved' if ok else 'FAILED'}  [{status_msg}]", color, 9, 'bold'))
        late = dt_s > period
        lines.append((f"  solve {dt_s * 1000:5.1f} ms / {period * 1000:.0f} ms budget"
                      + ('  LATE' if late else ''),
                      STATUS['warning'] if late else INK_SECONDARY, 8, 'normal'))
        lines.append((f"  cost {cost:8.3f}   backend {backend}", INK_SECONDARY, 8, 'normal'))
        lines.append((f"  hard constraints {nb}   obstacle disks {nobs}",
                      INK_SECONDARY, 8, 'normal'))
        if pred_x:
            v_end = f"   v_end {pred_v[-1]:.2f} m/s" if pred_v else ''
            lines.append((f"  horizon {len(pred_x)} steps"
                          f" ({len(pred_x) * period:.1f} s){v_end}",
                          INK_SECONDARY, 8, 'normal'))
        else:
            lines.append(('  horizon: not published in this bag',
                          INK_MUTED, 7.5, 'normal'))
    else:
        lines.append(('MPC: (no solve this frame)', INK_MUTED, 9, 'normal'))

    lines.append(('', INK, 4, 'normal'))
    if drive is not None:
        lines.append((f"cmd  v {drive[0]:5.2f} m/s   steer {math.degrees(drive[1]):6.1f} deg",
                      INK_SECONDARY, 8, 'normal'))
    if pose is not None:
        lines.append((f"pose ({pose[0]:6.2f}, {pose[1]:6.2f}) m  "
                      f"yaw {math.degrees(pose[2]):6.1f} deg",
                      INK_SECONDARY, 8, 'normal'))
        lines.append((f"     v_meas {pose[3]:5.2f} m/s   [{state['pose_frame']}]",
                      INK_MUTED, 7.5, 'normal'))
    if clearance is not None:
        lines.append((f"front clearance {clearance:5.2f} m", INK_SECONDARY, 8, 'normal'))

    lines.append(('', INK, 6, 'normal'))
    lines.append(('LEGEND', INK_MUTED, 7.5, 'bold'))
    lines.append(('  filled band = MPC reference corridor (funnel)', CORRIDOR_COLOR, 7, 'normal'))
    lines.append(('  tapering ghost = predicted horizon (this tick)', HORIZON_COLOR, 7, 'normal'))
    lines.append(('  solid/dashed line = hard boundary halfspace (raw / padded)',
                  HARD_COLOR, 7, 'normal'))
    lines.append(('  glow disk = soft-cost obstacle (camera)', STATUS['serious'], 7, 'normal'))
    lines.append(('  dotted ellipse = detection 1/2-sigma', INK_MUTED, 7, 'normal'))
    for cls, slot in sorted(state['class_order'].items(), key=lambda kv: (kv[1] is None, kv[1])):
        col = CLASS_COLORS[slot] if slot is not None else CLASS_OTHER
        lines.append((f'  {cls}', col, 7, 'normal'))

    y = 0.975
    for text, color, size, weight in lines:
        if text:
            ax_hud.text(0.04, y, text, color=color, fontsize=size, weight=weight,
                        va='top', ha='left', family='monospace',
                        transform=ax_hud.transAxes)
        y -= (size + 4.5) / 460.0

    # --- timeline strip ----------------------------------------------------
    ax_time.set_facecolor(PANEL)
    ax_time.set_xlim(0, state['duration'])
    ax_time.set_ylim(0, 1)
    ax_time.set_yticks([])
    ax_time.tick_params(colors=INK_MUTED, labelsize=7)
    for spine in ax_time.spines.values():
        spine.set_color(GRID)
    for (t_a, t_b) in state['stop_spans']:
        ax_time.axvspan(t_a, t_b, color=STATUS['critical'], alpha=0.55, lw=0)
    for tf in state['solver_fail_times']:
        ax_time.plot([tf, tf], [0.0, 0.35], color=STATUS['warning'], linewidth=1.0)
    ax_time.axvline(rel_t, color=INK, linewidth=1.4)
    ax_time.set_xlabel('mission time [s]   (red = safety stop, amber = solver failure)',
                       color=INK_MUTED, fontsize=7.5)


def build_stop_spans(bag, dt):
    """Contiguous [start, end) windows in mission time where a safety stop was
    active, from /safety_stop itself (the recorded fact) rather than from the
    BT's own flag -- both are recorded, and the topic is what actually reached
    the mux."""
    t0 = bag['t0']
    stops = bag['streams']['stop']
    spans = []
    gap = 0.5
    for tt in stops.t:
        rel = tt - t0
        if spans and rel - spans[-1][1] <= gap:
            spans[-1][1] = rel
        else:
            spans.append([rel, rel])
    return [(a, max(b, a + dt)) for a, b in spans]


def render_bag(bag_dir: Path, cfg):
    manifest = load_manifest_for(bag_dir)
    print(f'[{bag_dir.name}] reading...')
    read_start = time.time()
    bag = read_bag(bag_dir, cfg.pose_source)
    if len(bag['streams']['pose']) == 0:
        print(f'[{bag_dir.name}] no pose samples on {bag["pose_topic"]}; skipped',
              file=sys.stderr)
        return None
    duration = bag['t1'] - bag['t0']
    print(f'[{bag_dir.name}] {duration:.1f}s, '
          f'{len(bag["streams"]["pose"])} poses, '
          f'{len(bag["streams"]["boundaries"])} boundary msgs, '
          f'{len(bag["streams"]["obstacles"])} obstacle msgs, '
          f'{len(bag["streams"]["stop"])} safety_stop msgs, '
          f'{len(bag["streams"]["corridor"])} corridor msgs '
          f'(read {time.time() - read_start:.1f}s)')
    n_horizon = sum(1 for v in bag['streams']['solver'].v if v[9])
    if bag['legacy_solver_msgs']:
        print(f'[{bag_dir.name}] {bag["legacy_solver_msgs"]} solver_status msgs '
              'read with the pre-horizon layout (bag predates the horizon '
              'fields) -- no predicted trajectory to draw')
    else:
        print(f'[{bag_dir.name}] {n_horizon}/{len(bag["streams"]["solver"])} '
              'solves carry a predicted horizon')
    if len(bag['streams']['corridor']) == 0:
        print(f'[{bag_dir.name}] no /mpc/corridor_markers in this bag -- the '
              'reference corridor funnel cannot be drawn (bag predates it '
              'being recorded)', file=sys.stderr)
    elif cfg.pose_source != 'local' and len(bag['streams']['map_to_odom']) == 0:
        print(f'[{bag_dir.name}] no map->odom TF -- odom-frame layers '
              '(corridor, horizon) will be skipped', file=sys.stderr)
    for topic, count in bag['dropped'].items():
        print(f'[{bag_dir.name}] dropped {count} undecodable msgs on {topic}',
              file=sys.stderr)

    cam_to_base = None
    det_entry = bag['streams']['detections'].v[0] if len(bag['streams']['detections']) else None
    if det_entry is not None:
        cam_to_base = static_chain_to_base(bag['static_tf'], det_entry[0])
        if cam_to_base is None:
            print(f'[{bag_dir.name}] no static tf chain {det_entry[0]} -> base_link; '
                  'uncertainty ellipses disabled', file=sys.stderr)

    solver_fail_times = [tt - bag['t0']
                         for tt, v in zip(bag['streams']['solver'].t,
                                          bag['streams']['solver'].v)
                         if not v[0]]

    # In map-frame mode every odom-frame layer (corridor, horizon) needs the
    # live map->odom TF; in local mode the render IS in odom, so the transform
    # is the identity and no TF lookup is needed at all.
    if cfg.pose_source == 'local':
        def map_to_odom_fn(_t):
            return (0.0, 0.0, 0.0)
    else:
        def map_to_odom_fn(t):
            return bag['streams']['map_to_odom'].at(t)

    state = {
        'view': compute_view(bag, cfg.dt),
        'map_to_odom_fn': map_to_odom_fn,
        'class_order': {},
        'cam_to_base': cam_to_base,
        'mission_id': manifest.get('mission_id', bag_dir.name),
        'outcome': manifest.get('outcome', '?'),
        'run_id': manifest.get('run_id', bag_dir.name),
        'duration': duration,
        'pose_frame': bag['pose_frame'] or '?',
        'stop_spans': build_stop_spans(bag, cfg.dt),
        'solver_fail_times': solver_fail_times,
    }

    fig = plt.figure(figsize=(12.8, 7.2), dpi=cfg.dpi)
    fig.patch.set_facecolor(SURFACE)
    ax_map = fig.add_axes([0.045, 0.16, 0.60, 0.79])
    ax_hud = fig.add_axes([0.665, 0.16, 0.315, 0.79])
    ax_time = fig.add_axes([0.045, 0.055, 0.935, 0.055])

    n_frames = max(1, int(duration / cfg.dt) + 1)
    fps = max(1, int(round(cfg.speed / cfg.dt)))
    out_path = cfg.out_dir / f'{state["run_id"]}.mp4'
    cfg.out_dir.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, bitrate=cfg.bitrate,
                          metadata={'title': state['run_id'],
                                    'comment': 'mission_replay_video.py'})
    print(f'[{bag_dir.name}] rendering {n_frames} frames @ {fps} fps '
          f'({cfg.speed}x) -> {out_path}')
    render_start = time.time()
    with writer.saving(fig, str(out_path), cfg.dpi):
        for k in range(n_frames):
            rel_t = k * cfg.dt
            draw_frame((ax_map, ax_hud, ax_time), bag, bag['t0'] + rel_t, rel_t,
                       cfg, state)
            writer.grab_frame(facecolor=SURFACE)
            if cfg.progress and (k % 25 == 0 or k == n_frames - 1):
                print(f'  frame {k + 1}/{n_frames}', end='\r', flush=True)
    plt.close(fig)
    print(f'\n[{bag_dir.name}] done in {time.time() - render_start:.1f}s -> {out_path}')
    return out_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*', type=Path,
                    help='explicit bag directories; default is the newest --count '
                         'missions under --bag-root')
    ap.add_argument('--bag-root', type=Path, default=DEFAULT_BAG_ROOT)
    ap.add_argument('--count', type=int, default=3)
    ap.add_argument('--out-dir', type=Path, default=None,
                    help=f'default: {DEFAULT_OUT_DIR}')
    ap.add_argument('--dt', type=float, default=0.1,
                    help="resample period [s]; default 0.1 = the MPC's control period")
    ap.add_argument('--speed', type=float, default=1.0,
                    help='playback speed (1.0 = real time, 0.5 = slow motion)')
    ap.add_argument('--follow', type=float, default=None, metavar='HALF_WIDTH_M',
                    help='follow the car with a window of this half-width instead '
                         'of a fixed whole-run view')
    ap.add_argument('--trail-seconds', type=float, default=3.0)
    ap.add_argument('--corridor-span', type=float, default=5.0, metavar='M',
                    help='half-length of the drawn hard-constraint lines around '
                         'the car; the shaded excluded side is always full-view')
    ap.add_argument('--pose-source', choices=('global', 'local'), default='global',
                    help='global = /ekf_global/odometry/filtered (map frame, the '
                         'frame the semantic tracks and SLAM map live in); '
                         'local = /odometry/filtered (odom frame)')
    ap.add_argument('--car-radius', type=float, default=0.20,
                    help='must match stack_params.yaml car_radius')
    ap.add_argument('--avoidance-margin', type=float, default=0.12,
                    help='must match stack_params.yaml obstacle_safety_margin_m')
    ap.add_argument('--dpi', type=int, default=100)
    ap.add_argument('--bitrate', type=int, default=4000)
    ap.add_argument('--no-map', action='store_true', help='skip the SLAM grid layer')
    ap.add_argument('--no-progress', dest='progress', action='store_false', default=True)
    cfg = ap.parse_args(argv)

    if cfg.bags:
        bag_dirs = [b.expanduser().resolve() for b in cfg.bags]
    else:
        found = discover_bags(cfg.bag_root.expanduser(), cfg.count)
        if not found:
            print(f'no mission bags found under {cfg.bag_root}', file=sys.stderr)
            return 1
        bag_dirs = [d for _t, d, _m in found]
        print('selected (newest first):')
        for st, d, m in found:
            print(f'  {st}  {m.get("outcome", "?"):9s}  {d.name}')

    cfg.out_dir = (cfg.out_dir or DEFAULT_OUT_DIR).expanduser()
    outputs = []
    for bag_dir in bag_dirs:
        if not (bag_dir / 'metadata.yaml').exists():
            print(f'{bag_dir}: not a rosbag2 directory; skipped', file=sys.stderr)
            continue
        try:
            out = render_bag(bag_dir, cfg)
        except Exception as exc:                       # noqa: BLE001 - one bad bag
            print(f'{bag_dir.name}: FAILED: {exc}', file=sys.stderr)  # must not
            continue                                   # abort the others
        if out:
            outputs.append(out)
    print('\nwrote:')
    for o in outputs:
        print(f'  {o}')
    return 0 if outputs else 1


if __name__ == '__main__':
    sys.exit(main())
