#!/usr/bin/env python3
"""
door_replay.py: door_detection replayed offline on a recorded bag.

Usage (no ROS environment needed; numpy, scipy, matplotlib, PyYAML, rosbags):
    python3 tools/door_replay.py [--walls recorded|retrack-old|retrack-new]
        [--cadence steps|every] [--every N] [--params stack|code]
        [--set [tracker.|extraction.]FIELD=VALUE ...] [--no-frames]
        [--src CHECKOUT] [--door-images all|confirmed|none] [--out DIR] BAG

BAG is a rosbag2 directory of .db3 files (sqlite3; mcap is not read). It is
read with sqlite3 directly, read-only, and decoded with rosbags' CDR
deserialisers, the replayed checkout's own .msg files registered for
f1tenth_messages. Never rclpy, and nothing is launched.

WHAT IS REPLAYED. lidar_front_wall_node's door cycle (_detect_doors), with the
replayed checkout's own pure functions: the laser mount from /tf_static; the
car pose in odom interpolated at the scan stamp from /odometry/filtered (the
node's PoseHistory); the scan in base_link (scan_in_base_link); the wall
tracks (WALLS); then [scan_pieces ->] door_candidates -> DoorTracker.step(...,
scan=), as on the car. Nothing from the camera or YOLO reaches it: no stereo
support and no glass veto, no object sectors cut (so 'object' rejects cannot
occur), no persons (so no door is ever 'occupied').

WALLS (--walls).
  recorded (default): the tracks the car itself published on
    /perception/walls at that step, rebuilt as multi_wall_tracker.Track. They
    come from the tracker that ran ON THE CAR when the bag was recorded -- for
    corridor10m_20261016T144537 a tracker older than block 3 and than THE
    COLLINEAR GAP RULE (extraction merge 1.5 m, association gap 0.5 m) -- and
    they are only the PUBLISHED ones: at most wall_tracker_max_tracks (4),
    ranked by role, never 'other', where the node hands door_candidates ALL its
    tracks. A change to the extraction or the tracker -- change 1, the
    collinear merge rule -- CANNOT show with recorded walls; only the
    detector's changes do.
  retrack-new / retrack-old: the wall tracker run HERE, LiDAR-only and pure, on
    the same cycles: extract_lines (the checkout's extraction_params, Python
    backend) -> LiDAR wall_fusion.Observations -> MultiWallTracker.step(scan=,
    defer_roles) -> update_roles; door_candidates gets ALL its tracks. 'new' is
    the checkout's own collinear rule, its number DoorParams.width_min_m (the
    node reads that ONE parameter for the extraction's merge, the tracker and
    the detector). 'old' is the pre-step-2 rule: extraction merge_gap 1.5 m and
    a 0.5 m association / merge / extent gap, inclusive
    (TrackerParams(collinear_gap_m=0.5 + 1e-9)). A checkout older than the rule
    (TrackerParams.assoc_max_gap_m) runs 'old' natively and 'new' emulated
    (assoc_max_gap_m = width_min_m - 1e-9, merge_gap width_min_m). This is what
    makes change 1 visible offline. It is NOT the car's tracker either: no
    stereo observations and no rectangle verification (no stereo-only walls,
    no REJECTED ones), no detection sectors cut, and a cold start at the first
    cycle -- it is one tracker under two rules, which is what isolates change 1.
    Nor does it rebuild what an older car tracker did: on
    corridor10m_20261016T144537 the recorded tracks run up to 15 m through
    glass stretches (no gap check in its merge then), so the in-track doors
    the recorded walls carry there do not appear with retrack.

THE CADENCE (--cadence). steps (default): one cycle per recorded
/perception/walls message, on the /scan with the identical header stamp: the
car's own tracker steps (the node runs doors on those only; 4.3 Hz on
corridor10m_20261016T144537, where it processed 2830 of 6523 scans), with the
tracks that step published. every: every Nth recorded /scan (--every, 4) with
the nearest walls message -- on a bag whose node dropped scans that is NOT the
car's rate. Without /perception/walls, steps falls back to every.

PARAMETERS (--params). stack (default): DoorParams, and for retrack the
TrackerParams, the extraction's quality gates and the observation model, as
lidar_front_wall_node builds them from the replayed checkout's
stack_params.yaml -- STACK_TO_DOOR / STACK_TO_TRACKER / STACK_TO_NODE mirror
its constructor (keep them in step), a field the checkout's dataclass lacks is
skipped. code: the dataclass defaults. --set FIELD=VALUE overrides a
DoorParams field, tracker.FIELD=VALUE a TrackerParams one and
extraction.FIELD=VALUE an ExtractionParams one, after everything else (radians
where the field is *_rad; e.g. --set square_looks=0).

--src CHECKOUT replays another checkout's detector and tracker with this tool
(git worktree add --detach DIR COMMIT, or git archive COMMIT src | tar -x -C
DIR). Detectors older than step 3 (no frame jambs, no scan argument, no panes)
and trackers older than step 2 (no collinear_gap_m) are driven through feature
tests (dataclass fields, inspect.signature, getattr), never version numbers.

OUTPUTS in --out (default ~/f1tenth_data/replays/doors_<bag>/<src>-<walls>-
<cadence>; <tag> = <bag>.doors, <bag>.doors_noframes with --no-frames):
  <tag>.json            the run (src, walls and rule, cadence, params), reject
                        codes (the live trail's 16-entry cap lifted), EVERY
                        door track's life and kind (frame / in-track /
                        between), the pane kills, timing
  <tag>.candidates.csv  every gap the detector judged and every pair of tracks
                        _groups refused, every cycle (CSV_FIELDS): code (or
                        accepted), width, beams, parents, jambs in odom (a
                        pair's facing ends), the door track it fed
  <tag>.cycles.csv      one row per cycle: walls (eligible), gaps, doors, cost;
                        retrack: the extracted lines and the tracker's own
                        collinear-gap refusals (observations, track merges)
  <tag>.png             the accumulated scan in odom, the eligible walls, frame
                        pieces, doors (green confirmed, orange tentative),
                        panes
  door_<id>.png         per door track, small (palette PNG): the
                        /camera/debug_image/compressed frame nearest its best
                        sighting by HEADER stamp (the capture stamp; the
                        recorder received them ~0.26 s late), plus the nearest
                        frame of its life +-5 s with both jambs in view when
                        that one does not show it, jamb posts drawn; and a
                        top-down of that scan
"""

import argparse
import ast
import bisect
import collections
import contextlib
import csv
import dataclasses
import inspect
import io
import json
import math
from pathlib import Path
import re
import sqlite3
import struct
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np

REPO = Path(__file__).resolve().parents[1]
OUT_ROOT = Path('~/f1tenth_data/replays')
TOPICS = ('/scan', '/tf_static', '/odometry/filtered', '/perception/walls')
IMAGE_TOPICS = ('/camera/debug_image/compressed', '/zed2/zed_node/depth/camera_info')
NODE_PREFIX = 'lidar_front_wall_'

#: (node parameter, DoorParams field, conversion): lidar_front_wall_node's
#: DoorParams(...) constructor, mirrored. The yaml key is NODE_PREFIX + name.
STACK_TO_DOOR = (
    ('door_collinear_angle_deg', 'collinear_angle_rad', math.radians),
    ('door_collinear_offset_m', 'collinear_offset_m', float),
    ('door_width_min_m', 'width_min_m', float),
    ('door_width_max_m', 'width_max_m', float),
    ('door_behind_margin_m', 'behind_margin_m', float),
    ('wall_negative_tol_m', 'wall_tol_m', float),
    ('door_min_beams', 'min_beams', int),
    ('door_min_beam_fraction', 'min_beam_fraction', float),
    ('door_confirm_hits', 'confirm_hits', int),
    ('door_confirm_window', 'confirm_window', int),
    ('door_delete_after_sec', 'delete_after_s', float),
    ('door_core_fraction', 'core_fraction', float),
    ('door_leaf_max_len_m', 'leaf_max_len_m', float),
    ('door_leaf_hinge_tol_m', 'leaf_hinge_tol_m', float),
    ('door_stereo_glass_veto', 'stereo_glass_veto', bool),
    ('door_stereo_plane_tol_m', 'stereo_plane_tol_m', float),
    ('door_stereo_wall_veto_frac', 'stereo_wall_veto_frac', float),
    ('door_frame_jambs', 'frame_jambs', bool),
    ('door_frame_min_pieces', 'frame_min_pieces', int),
    ('door_frame_jamb_min_length_m', 'frame_jamb_min_length_m', float),
    ('door_piece_max_rms_m', 'piece_max_rms_m', float),
    ('door_piece_max_range_m', 'piece_max_range_m', float),
    ('wall_min_span_birth_m', 'piece_max_length_m', float),
    ('door_square_looks', 'square_looks', int),
    ('door_square_incidence_deg', 'square_incidence_rad', math.radians),
    ('door_square_min_beams', 'square_min_beams', int),
    ('door_pane_min_on_beams', 'pane_min_on_beams', int),
    ('door_pane_min_cycles', 'pane_min_cycles', int),
    ('door_pane_memory_sec', 'pane_memory_s', float),
)
#: The same for its TrackerParams(...) (collinear_gap_m is set by the rule,
#: --walls; the two pre-step-2 gaps exist only in older checkouts).
STACK_TO_TRACKER = (
    ('wall_tracker_process_noise_rho', 'process_noise_rho', float),
    ('wall_tracker_process_noise_theta', 'process_noise_theta', float),
    ('wall_tracker_k_yaw', 'k_yaw', float),
    ('wall_assoc_max_gap_m', 'assoc_max_gap_m', float),
    ('wall_merge_max_gap_m', 'merge_max_gap_m', float),
    ('wall_extent_decay_sec', 'extent_decay_s', float),
    ('wall_tracker_delete_after_sec', 'delete_after_s', float),
    ('camera_seg_min_ray_angle_deg', 'camera_seg_min_ray_angle_rad', math.radians),
    ('camera_seg_min_bearing_span_deg', 'camera_seg_min_bearing_span_rad', math.radians),
    ('stereo_max_rms_m', 'stereo_max_rms_m', float),
    ('stereo_min_vertical_extent_m', 'stereo_min_vertical_extent_m', float),
    ('stereo_min_length_m', 'stereo_min_length_m', float),
    ('camera_confirm_fresh_only', 'camera_confirm_fresh_only', bool),
    ('camera_side_x_max_m', 'camera_side_x_max_m', float),
    ('camera_side_max_normal_deg', 'camera_side_max_normal_rad', math.radians),
    ('camera_side_offset_min_m', 'camera_side_offset_min_m', float),
    ('camera_side_offset_max_m', 'camera_side_offset_max_m', float),
    ('wall_coast_max_odom_m', 'coast_max_odom_m', float),
    ('wall_tracker_max_sigma_rho_m', 'max_sigma_rho_m', float),
    ('wall_tracker_max_tracks', 'max_published', int),
    ('wall_tracker_front_max_coast_sec', 'front_max_coast_s', float),
    ('wall_tracker_front_max_coast_m', 'front_max_coast_m', float),
    ('lidar_sigma_base_m', 'lidar_sigma_base_m', float),
    ('lidar_sigma_per_m', 'lidar_sigma_per_m', float),
    ('stereo_sigma_base_m', 'stereo_sigma_base_m', float),
    ('stereo_sigma_quad_per_m2', 'stereo_sigma_quad_per_m2', float),
    ('min_span_for_angle_m', 'min_span_for_angle_m', float),
    ('stereo_confirm_hits', 'stereo_confirm_hits', int),
    ('stereo_confirm_window', 'stereo_confirm_window', int),
    ('role_hysteresis_rad', 'role_hysteresis_rad', float),
    ('stereo_front_max_normal_rad', 'stereo_front_max_normal_rad', float),
    ('front_strip_margin_m', 'front_strip_margin_m', float),
    ('front_min_confidence', 'front_min_confidence', float),
    ('front_lidar_recent_cycles', 'front_lidar_recent_cycles', int),
    ('front_contradict_margin_m', 'front_contradict_margin_m', float),
    ('front_contradict_cycles', 'front_contradict_cycles', int),
    ('wall_obs_stereo_min_rho_m', 'stereo_min_rho_m', float),
    ('wall_min_span_birth_m', 'min_span_birth_m', float),
    ('wall_min_span_update_m', 'min_span_update_m', float),
    ('wall_role_back_x_m', 'back_x_m', float),
    ('wall_role_side_x_min_m', 'side_x_min_m', float),
    ('wall_role_side_x_max_m', 'side_x_max_m', float),
    ('wall_role_side_max_normal_deg', 'side_max_normal_rad', math.radians),
    ('wall_role_hysteresis_m', 'role_hysteresis_m', float),
    ('wall_occlusion_margin_m', 'occlusion_margin_m', float),
    ('wall_negative_tol_m', 'negative_tol_m', float),
    ('wall_negative_min_beams', 'negative_min_beams', int),
    ('wall_negative_pass_frac', 'negative_pass_frac', float),
    ('wall_negative_miss_to_kill', 'negative_miss_to_kill', int),
)
#: ...and what else the node takes from the yaml for the walls: the
#: extraction's gates, the LiDAR observation's height and confidence model.
STACK_TO_NODE = (
    ('wall_extract_quality_gates', 'quality_gates', bool),
    ('wall_default_height_m', 'height_m', float),
    ('wall_conf_range_ref_m', 'range_ref_m', float),
    ('wall_conf_min_inlier_ref', 'min_inlier_ref', float),
    ('wall_conf_max_rms_scale', 'max_rms_scale', float),
)
#: The pre-step-2 rule (--walls retrack-old): corridor_perception's merge gap
#: (wall_merge_max_gap_m, 193fe1d) and wall_assoc_max_gap_m (d578963), <=.
OLD_MERGE_GAP_M = 1.5
OLD_ASSOC_GAP_M = 0.5
#: How far a scan stamp may lie outside /odometry/filtered (the node's
#: pose_max_extrapolation_sec).
POSE_MAX_EXTRAPOLATION_S = 0.15

BEAM_NAMES = ('on', 'beyond', 'none', 'short', 'shallow', 'cut')    # BEAM_ON .. BEAM_CUT
CSV_FIELDS = ('cycle', 't', 'stamp', 'stage', 'code', 'width', 'n_open', 'n_req', 'n_gap') \
    + BEAM_NAMES + ('leaf', 'parent0', 'parent1', 'j0x', 'j0y', 'j1x', 'j1y', 'cx', 'cy',
                    'range_m', 'view_deg', 'door', 'door_state')
CYCLE_FIELDS = ('cycle', 't', 'stamp', 'walls', 'eligible', 'pieces', 'gaps', 'accepted',
                'doors', 'confirmed', 'lines', 'gap_rejected', 'merge_gap_refused',
                'walls_ms', 'door_ms')
RETRACK_COUNTS = ('lines', 'gap_rejected', 'merge_gap_refused')

# The replayed checkout's modules (import_detector).
dd = mwt = wall_fusion = Pose2D = None


# ============================================================================
# The replayed checkout
# ============================================================================

def import_detector(src):
    """Import the detector, the tracker, wall_fusion and Pose2D from checkout `src`."""
    global dd, mwt, wall_fusion, Pose2D
    src = Path(src).resolve()
    sys.dont_write_bytecode = True      # never litter the replayed checkout
    for sub in ('src/f1tenth_params', 'src/corridor_perception', 'src/f1tenth_perception'):
        sys.path.insert(0, str(src / sub))
    from corridor_perception import geometry
    from f1tenth_perception import door_detection, multi_wall_tracker
    from f1tenth_perception import wall_fusion as fusion
    if src not in Path(door_detection.__file__).resolve().parents:
        raise SystemExit(f'door_detection was already imported from {door_detection.__file__},'
                         f' not from {src}: one checkout per process')
    dd, mwt, wall_fusion, Pose2D = door_detection, multi_wall_tracker, fusion, geometry.Pose2D


def checkout_commit(src):
    """Return the short commit of checkout `src` (+ '-dirty'), or None outside git."""
    def git(*args):
        return subprocess.run(['git', '-C', str(src), *args], capture_output=True,
                              text=True, timeout=10).stdout.strip()
    try:
        if Path(git('rev-parse', '--show-toplevel') or '/nonexistent').resolve() != src:
            return None
        commit = git('rev-parse', '--short', 'HEAD')
        dirty = git('status', '--porcelain', '--untracked-files=no', '--', 'src', 'tools')
    except (OSError, subprocess.SubprocessError):
        return None
    return (commit + ('-dirty' if dirty else '')) or None


# ============================================================================
# The bag
# ============================================================================

def typestore(src):
    """Return a ROS2_HUMBLE typestore with checkout `src`'s f1tenth_messages registered."""
    from rosbags.typesys import get_types_from_msg, get_typestore, Stores
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for path in sorted((Path(src) / 'src/f1tenth_messages/msg').glob('*.msg')):
        types.update(get_types_from_msg(path.read_text(), f'f1tenth_messages/msg/{path.stem}'))
    store.register(types)
    return store


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def sort_by_stamp(msgs):
    """Sort messages by HEADER stamp, in place (receipt order is not capture order)."""
    msgs.sort(key=stamp)
    return msgs


def header_only(data):
    """Return a stand-in holding just the std_msgs/Header a CDR message starts with."""
    endian = '<' if data[1] == 1 else '>'
    sec, nanosec = struct.unpack_from(endian + 'iI', data, 4)
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=''), header_only=True)


def db3_files(bag):
    """Return the bag's .db3 files in split order (_2 before _10)."""
    bag = Path(bag)
    files = [bag] if bag.suffix == '.db3' else list(bag.glob('*.db3'))
    return sorted(files, key=lambda p: [int(s) if s.isdigit() else s
                                        for s in re.split(r'(\d+)', p.name)])


def read_bag(bag, store, topics, first_only=()):
    """
    Return ({topic: [messages, header-stamp order]}, {topic: decode error}).

    A topic whose messages fail to decode (a bag older than the checkout's
    .msg files) keeps their headers only (header_only), so its stamps still
    drive the cadence.
    """
    out = {t: [] for t in topics}
    failed = {}
    files = db3_files(bag)
    if not files:
        raise SystemExit(f'{bag}: no .db3 files (mcap is not read: convert the bag first)')
    for path in files:
        con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        want = {tid: (name, typ) for tid, name, typ in
                con.execute('select id, name, type from topics') if name in topics}
        for tid, data in con.execute('select topic_id, data from messages order by timestamp'):
            if tid not in want:
                continue
            name, typ = want[tid]
            if name in first_only and out[name]:
                continue
            if name not in failed:
                try:
                    out[name].append(store.deserialize_cdr(data, typ))
                    continue
                except Exception as exc:  # noqa: BLE001 - any decode error, reported
                    failed[name] = f'{type(exc).__name__}: {exc}'
            out[name].append(header_only(data))
        con.close()
    for msgs in out.values():
        if msgs and hasattr(msgs[0], 'header'):
            sort_by_stamp(msgs)
    return out, failed


# ============================================================================
# Geometry
# ============================================================================

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def laser_pose(tf_static, frame, base='base_link'):
    """(x, y, yaw) of `frame` in base_link from /tf_static (planar chain)."""
    edges = {}
    for msg in tf_static:
        for tr in msg.transforms:
            edges[tr.child_frame_id] = (tr.header.frame_id, tr.transform)
    x, y, yaw = 0.0, 0.0, 0.0
    while frame != base:
        parent, tf = edges[frame]
        c, s = math.cos(yaw_of(tf.rotation)), math.sin(yaw_of(tf.rotation))
        x, y = tf.translation.x + c * x - s * y, tf.translation.y + s * x + c * y
        yaw += yaw_of(tf.rotation)
        frame = parent
    return x, y, yaw


def nearest_index(stamps, t):
    """Index of the stamp nearest t in the ascending list `stamps`."""
    i = bisect.bisect_left(stamps, t)
    if i <= 0:
        return 0
    if i >= len(stamps):
        return len(stamps) - 1
    return i if stamps[i] - t < t - stamps[i - 1] else i - 1


def nearest(msgs, t):
    """Return the message whose header stamp is nearest t (msgs sorted by header stamp)."""
    return msgs[nearest_index([stamp(m) for m in msgs], t)]


class Poses:
    """Odometry poses, linear between neighbours (lidar_front_wall_node's PoseHistory)."""

    def __init__(self, odom):
        self.t = [stamp(m) for m in odom]
        self.xyyaw = [(m.pose.pose.position.x, m.pose.pose.position.y,
                       yaw_of(m.pose.pose.orientation)) for m in odom]

    def at(self, t, max_extrapolation_s=POSE_MAX_EXTRAPOLATION_S):
        """Pose2D (odom from base_link) at t, or None outside the poses."""
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i == len(self.t):
            k = 0 if i == 0 else len(self.t) - 1
            return Pose2D(*self.xyyaw[k]) if abs(self.t[k] - t) <= max_extrapolation_s else None
        (x0, y0, a0), (x1, y1, a1) = self.xyyaw[i - 1], self.xyyaw[i]
        f = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9)
        return Pose2D(x0 + f * (x1 - x0), y0 + f * (y1 - y0), _wrap(a0 + f * _wrap(a1 - a0)))


def segment_odom(trk):
    """Return a Track's measured extent as its two odom endpoints."""
    n = np.array([math.cos(trk.theta), math.sin(trk.theta)])
    tan = np.array([-n[1], n[0]])
    return np.vstack([trk.rho * n + trk.t0 * tan, trk.rho * n + trk.t1 * tan])


# ============================================================================
# Parameters
# ============================================================================

def stack_values(src, table, prefix=NODE_PREFIX):
    """Return {field: value} for the entries of `table` checkout `src`'s stack_params.yaml has."""
    import yaml
    doc = yaml.safe_load((Path(src) / 'src/f1tenth_params/config/stack_params.yaml')
                         .read_text()) or {}
    out = {}
    for key, name, conv in table:
        entry = doc.get(prefix + key)
        if entry is not None:
            out[name] = conv(entry['default'] if isinstance(entry, dict) else entry)
    return out


def build(cls, values):
    """cls(**values), the values for fields cls does not have dropped (older checkouts)."""
    names = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in values.items() if k in names})


def parse_value(text, current):
    """`text` as the type of `current` (true/false/1/0 for a bool)."""
    if isinstance(current, bool):
        low = text.strip().lower()
        if low not in ('true', 'false', '1', '0'):
            raise ValueError(f'not a bool: {text!r}')
        return low in ('true', '1')
    value = ast.literal_eval(text)
    return value if current is None else type(current)(value)


def apply_overrides(obj, overrides, prefix=''):
    """Apply the --set items addressed to `obj` (prefix 'tracker.', 'extraction.' or '')."""
    for item in overrides:
        name, value = item.split('=', 1)
        if '.' in name:
            if not name.startswith(prefix) or not prefix:
                continue
            name = name[len(prefix):]
        elif prefix:
            continue
        if not hasattr(obj, name):
            raise SystemExit(f'--set {item}: {type(obj).__name__} has no field {name!r}')
        try:
            setattr(obj, name, parse_value(value, getattr(obj, name)))
        except (ValueError, TypeError, SyntaxError) as exc:
            raise SystemExit(f'--set {item}: {exc}') from exc


def door_params(src, mode, frames, overrides):
    """Return the DoorParams the node builds (mode 'stack') or the code defaults ('code')."""
    values = stack_values(src, STACK_TO_DOOR) if mode == 'stack' else {}
    if not frames:
        values['frame_jambs'] = False
    p = build(dd.DoorParams, values)
    apply_overrides(p, overrides)
    return p


def wall_rule(tp, rule, width_min_m):
    """
    Set THE COLLINEAR GAP RULE (--walls) on `tp`; return (extraction merge gap, how).

    'new' is the checkout's own rule with the door's width_min_m as its number;
    'old' the pre-step-2 rule. Native on the tracker that has it, emulated on
    the other (module docstring, WALLS).
    """
    if hasattr(tp, 'collinear_gap_m'):
        if rule == 'new':
            tp.collinear_gap_m = float(width_min_m)
            return float(width_min_m), 'native'
        tp.collinear_gap_m = OLD_ASSOC_GAP_M + 1e-9
        return OLD_MERGE_GAP_M, 'emulated'
    if rule == 'old':
        return float(tp.merge_max_gap_m), 'native'
    tp.assoc_max_gap_m = float(width_min_m) - 1e-9
    return float(width_min_m), 'emulated'


def wall_setup(src, mode, rule, p, overrides):
    """Return (TrackerParams, ExtractionParams, node settings, rule record): retrack-*."""
    tracker_kw = stack_values(src, STACK_TO_TRACKER) if mode == 'stack' else {}
    node = stack_values(src, STACK_TO_NODE) if mode == 'stack' else {}
    if mode == 'stack':
        tracker_kw.update(stack_values(src, (('swept_clearance_body_half_width_m',
                                              'side_edge_half_width_m', float),), prefix=''))
    tp = build(mwt.TrackerParams, tracker_kw)
    merge_gap, how = wall_rule(tp, rule, p.width_min_m)
    apply_overrides(tp, overrides, 'tracker.')
    ep = mwt.extraction_params(merge_gap, quality_gates=node.get('quality_gates'),
                               backend='python')
    apply_overrides(ep, overrides, 'extraction.')
    record = {'rule': rule, 'how': how, 'merge_gap_m': round(float(ep.merge_gap), 9),
              'quality_gates': bool(ep.quality_gates), 'backend': ep.backend}
    for name in ('collinear_gap_m', 'assoc_max_gap_m'):
        if hasattr(tp, name):
            record[name] = round(float(getattr(tp, name)), 9)
    return tp, ep, node, record


def _rounded(params):
    return {k: (round(v, 6) if isinstance(v, float) else v)
            for k, v in dataclasses.asdict(params).items()}


# ============================================================================
# The walls
# ============================================================================

def tracks_from(msg):
    """Return the published MultiWallTracks of one message as multi_wall_tracker.Track."""
    if getattr(msg, 'header_only', False):
        raise SystemExit('--walls recorded: /perception/walls did not decode with this '
                         "checkout's .msg files; use --walls retrack-new or retrack-old")
    names = {f.name for f in dataclasses.fields(mwt.Track)}
    out = []
    for t in msg.tracks:
        tan = np.array([-math.sin(t.theta), math.cos(t.theta)])
        a = tan @ np.array([t.p0.x, t.p0.y])
        b = tan @ np.array([t.p1.x, t.p1.y])
        extra = {k: v for k, v in (('role', str(t.role)), ('hits', int(t.hits)))
                 if k in names}
        out.append(mwt.Track(
            id=int(t.id), x=np.array([t.rho, t.theta], dtype=float), P=np.eye(2) * 1e-3,
            t0=float(min(a, b)), t1=float(max(a, b)), born=0.0, last_seen=0.0,
            ever_confirmed=t.state in (1, 2), state=int(t.state), **extra))
    return out


class Retracker:
    """The wall tracker run here, LiDAR-only, as _track_walls runs it (WALLS)."""

    def __init__(self, tp, ep, node, lpose):
        self.trk = mwt.MultiWallTracker(tp)
        self.ep, self.lpose = ep, lpose
        self.height_m = float(node.get('height_m', 2.5))
        self.conf_kw = {k: float(node[k])
                        for k in ('range_ref_m', 'min_inlier_ref', 'max_rms_scale') if k in node}
        self.defer = 'defer_roles' in inspect.signature(self.trk.step).parameters
        #: The last step's extracted lines and the tracker's own collinear-gap
        #: refusals (observations, track merges): THE RULE at work.
        self.counts = dict.fromkeys(RETRACK_COUNTS, 0)

    def observation(self, line):
        """Return one extracted line as the node's LiDAR-only wall_fusion.Observation."""
        wl = mwt.line_to_wall_line(line)
        conf = wall_fusion.confidence(wall_fusion.SOURCE_LIDAR, wl.inliers, wl.rms, 0, 0,
                                      range_m=abs(float(wl.d)), **self.conf_kw)
        return wall_fusion.Observation(wl, 0.0, self.height_m, False,
                                       wall_fusion.SOURCE_LIDAR, conf)

    def step(self, msg, t, car, scan):
        """One tracker step on LaserScan `msg` at stamp t; return ALL tracks."""
        lines = mwt.extract_lines(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max, self.lpose, (),
                                  params=self.ep)
        self.trk.laser_xy = self.lpose[:2]
        self.trk.step([self.observation(ln) for ln in lines], t, car, scan=scan,
                      **({'defer_roles': True} if self.defer else {}))
        if hasattr(self.trk, 'apply_rect_verdicts'):
            self.trk.apply_rect_verdicts({})
        if self.defer:
            self.trk.update_roles(car)
        self.counts = {'lines': len(lines),
                       'gap_rejected': int(getattr(self.trk, 'n_gap_rejected', 0)),
                       'merge_gap_refused': int(getattr(self.trk, 'n_merge_gap_refused', 0))}
        return list(self.trk.tracks)


def plan_cycles(scans, walls_msgs, cadence, every):
    """
    Return ([(scan, walls message or None)], cadence run, scans missing).

    steps: one cycle per walls message, on the scan with the identical header
    stamp (a walls message whose scan the bag lacks is counted, not run).
    every: every `every`-th scan with the nearest walls message.
    """
    if cadence == 'steps' and walls_msgs:
        by_stamp = {(s.header.stamp.sec, s.header.stamp.nanosec): s for s in scans}
        plan = [(by_stamp.get((w.header.stamp.sec, w.header.stamp.nanosec)), w)
                for w in walls_msgs]
        return ([(s, w) for s, w in plan if s is not None], 'steps',
                sum(1 for s, _w in plan if s is None))
    wt = [stamp(w) for w in walls_msgs]
    return ([(s, walls_msgs[nearest_index(wt, stamp(s))] if walls_msgs else None)
             for k, s in enumerate(scans) if k % max(1, int(every)) == 0], 'every', 0)


# ============================================================================
# The detector
# ============================================================================

@contextlib.contextmanager
def capture(sink):
    """
    Record into `sink` every gap door_candidates judges and every pair _groups refuses.

    Wraps the module globals door_detection._judge_gap and _groups (looked up
    at call time) and lifts the 16-entry reject cap, for the duration only.
    A gap record: stage 'gap', code ('accepted' when none), base_link jambs,
    n_open / n_req, the beam classes of the gap, parents, the leaf jamb. A
    pair record: stage 'pair', code, width, the two walls' facing ends.
    """
    judge, groups, cap = dd._judge_gap, dd._groups, dd._MAX_REJECTS

    def judged(gap, jambs, spans, u, walls, *rest, **kw):
        res = judge(gap, jambs, spans, u, walls, *rest, **kw)
        cls = np.asarray(gap)
        sink.append({'stage': 'gap', 'code': res[0] or 'accepted',
                     'jambs': np.array([jambs[0], jambs[1]], dtype=float),
                     'n_open': int(res[1]), 'n_req': int(res[2]), 'n_gap': int(cls.size),
                     'parents': (int(walls[0].track_id), int(walls[1].track_id)),
                     'leaf': res[3] if len(res) > 3 else False,
                     **{k: int(np.count_nonzero(cls == v)) for v, k in enumerate(BEAM_NAMES)}})
        return res

    def grouped(walls, p, rejects):
        n0 = len(rejects) if rejects is not None else 0
        out = groups(walls, p, rejects)
        by_id = {w.track_id: w for w in walls}
        for code, ids, width in (rejects or [])[n0:]:
            wa, wb = by_id.get(ids[0]), by_id.get(ids[1])
            sink.append({'stage': 'pair', 'code': code, 'width': float(width),
                         'parents': tuple(int(x) for x in ids),
                         'jambs': (np.array(dd._facing_ends(wa, wb), dtype=float)
                                   if wa is not None and wb is not None else None)})
        return out

    dd._judge_gap, dd._groups, dd._MAX_REJECTS = judged, grouped, 1 << 30
    try:
        yield sink
    finally:
        dd._judge_gap, dd._groups, dd._MAX_REJECTS = judge, groups, cap


class Detector:
    """
    One door cycle of the replayed checkout (_detect_doors without stereo).

    Feature tests, not versions: frame pieces only where DoorParams has
    frame_jambs and the module scan_pieces; door_candidates(pieces=) and
    DoorTracker.step(scan=) only where their signatures take them.
    """

    def __init__(self, p):
        self.p = p
        self.tracker = dd.DoorTracker(p)
        self.frames = bool(getattr(p, 'frame_jambs', False)) and hasattr(dd, 'scan_pieces')
        self.takes_pieces = 'pieces' in inspect.signature(dd.door_candidates).parameters
        self.takes_scan = 'scan' in inspect.signature(dd.DoorTracker.step).parameters

    def cycle(self, walls, car, scan, t):
        """Return (pieces, candidates, live doors, rejects) of one cycle."""
        rejects = []
        pieces = dd.scan_pieces(scan, self.p) if self.frames else []
        cands = dd.door_candidates(walls, car, scan, self.p, rejects,
                                   **({'pieces': pieces} if self.takes_pieces else {}))
        doors = self.tracker.step(cands, car, t, **({'scan': scan} if self.takes_scan else {}))
        return pieces, cands, doors, rejects


def _normal(j):
    tang = j[1] - j[0]
    return np.array([-tang[1], tang[0]]) / max(float(np.hypot(*tang)), 1e-9)


def door_of(tracker, doors, j_odom):
    """Return (door id, state name) a candidate (odom jambs) fed, by the tracker's own cost."""
    normal = _normal(j_odom)
    costs = sorted((tracker._cost(d, j_odom[0], j_odom[1], normal), d.id, d) for d in doors)
    if costs and math.isfinite(costs[0][0]):
        return str(costs[0][1]), dd.STATE_NAMES[costs[0][2].state]
    for pane in getattr(tracker, 'panes', []):
        if math.isfinite(tracker._cost(pane, j_odom[0], j_odom[1], normal)):
            return f'pane:{pane.id}', 'pane'
    return '', ''


def candidate_rows(records, cycle, t_rel, t, car, lxy, tracker, doors):
    """CSV rows (CSV_FIELDS) of one cycle's capture records."""
    rows = []
    for rec in records:
        row = dict.fromkeys(CSV_FIELDS, '')
        row.update(cycle=cycle, t=round(t_rel, 3), stamp=round(t, 6), stage=rec['stage'],
                   code=rec['code'], parent0=rec['parents'][0], parent1=rec['parents'][1])
        jb = rec['jambs']
        if rec['stage'] == 'gap':
            row.update({k: rec[k] for k in ('n_open', 'n_req', 'n_gap') + BEAM_NAMES})
            row['width'] = round(float(np.hypot(*(jb[1] - jb[0]))), 3)
        else:
            row['width'] = round(rec['width'], 3)
        if jb is not None:
            centre = 0.5 * (jb[0] + jb[1])
            ray = centre - np.asarray(lxy, dtype=float)
            normal = _normal(jb)
            cos_view = abs(float(normal @ ray)) / max(float(np.hypot(*ray)), 1e-9)
            row['range_m'] = round(float(np.hypot(*ray)), 2)
            row['view_deg'] = round(math.degrees(math.acos(min(1.0, cos_view))), 1)
            if rec['code'] == 'accepted' and rec['leaf'] is not False:
                if rec['leaf'] is None:
                    row['leaf'] = 'none'
                else:
                    outward = normal if float(normal @ ray) >= 0.0 else -normal
                    row['leaf'] = dd.LEAF_NAMES[dd.side_of(rec['leaf'], centre, outward)]
            j = car.transform_points(jb)
            c = 0.5 * (j[0] + j[1])
            row.update(j0x=round(float(j[0][0]), 3), j0y=round(float(j[0][1]), 3),
                       j1x=round(float(j[1][0]), 3), j1y=round(float(j[1][1]), 3),
                       cx=round(float(c[0]), 3), cy=round(float(c[1]), 3))
            if rec['code'] == 'accepted':
                row['door'], row['door_state'] = door_of(tracker, doors, j)
        rows.append(row)
    return rows


def write_csv(path, fields, rows):
    with open(path, 'w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


# ============================================================================
# The replay
# ============================================================================

def default_out(bag, src, walls, cadence, frames):
    label = checkout_commit(src) or Path(src).name
    return (OUT_ROOT.expanduser() / f'doors_{Path(bag).resolve().name}'
            / f'{label}-{walls}-{cadence}{"" if frames else "-noframes"}')


def _life_update(life, door, t_rel, t, car, seen=True):
    """Fold one cycle's state of a DoorTrack into its life record (a sighting if seen at t)."""
    rec = life.setdefault(door.id, {'born': round(t_rel, 2), 'states': set(), 'hits': 0,
                                    'window_hits': 0, 'square_looks': 0, 'sightings': [],
                                    'pane_killed': False, 'first_confirmed': None})
    rec['states'].add(dd.STATE_NAMES[door.state])
    rec['last'] = round(t_rel, 2)
    rec['hits'] = max(rec['hits'], int(door.hits))
    rec['window_hits'] = max(rec['window_hits'], int(sum(door.history)))
    rec['square_looks'] = max(rec['square_looks'], int(getattr(door, 'square_looks', 0)))
    if door.state == dd.STATE_CONFIRMED and rec['first_confirmed'] is None:
        rec['first_confirmed'] = round(t_rel, 2)
    if seen and door.last_seen == t:
        rec['sightings'].append({
            't': t, 'width': round(door.width, 3), 'n_beams': int(door.n_beams),
            'parents': [int(x) for x in door.parents],
            'leaf': dd.LEAF_NAMES[door.leaf_side] if hasattr(door, 'leaf_side') else 'n/a',
            'jambs': [[round(float(v), 3) for v in door.jamb0],
                      [round(float(v), 3) for v in door.jamb1]],
            'normal': [round(float(v), 3) for v in door.normal],
            'car': [round(float(car.x), 3), round(float(car.y), 3), round(float(car.theta), 4)]})
    return rec


def door_kind(parents, frame_id):
    """Return 'frame' (a jamb on a frame piece), 'in-track' or 'between' two tracks."""
    if frame_id is not None and frame_id in parents:
        return 'frame'
    return 'in-track' if parents[0] == parents[1] else 'between'


def _track_summary(rec, t0, frame_id):
    sights = rec['sightings']
    best = max(sights, key=lambda s: s['n_beams']) if sights else None
    return {
        'kind': door_kind(best['parents'], frame_id) if best else None,
        'born': rec['born'], 'last': rec['last'], 'states': sorted(rec['states']),
        'first_confirmed': rec['first_confirmed'], 'hits': rec['hits'],
        'window_hits': rec['window_hits'], 'square_looks': rec['square_looks'],
        'sightings': len(sights), 'pane_killed': rec['pane_killed'],
        'width': round(float(np.median([s['width'] for s in sights])), 3) if sights else None,
        'n_beams': int(np.median([s['n_beams'] for s in sights])) if sights else None,
        'parents': sorted({tuple(s['parents']) for s in sights}),
        'leaf': dict(collections.Counter(s['leaf'] for s in sights)),
        'best': best and {**best, 't': round(best['t'] - t0, 2), 'stamp': best['t']}}


def _pct(values, q):
    return round(float(np.percentile(values, q)), 2) if len(values) else None


def replay(bag, out_dir=None, walls='recorded', cadence='steps', every=4, frames=True,
           params='stack', overrides=(), src=REPO, images='all'):
    """Run the replay and write its outputs; return the summary (the JSON)."""
    clock = time.perf_counter()
    src = Path(src).resolve()
    p = door_params(src, params, frames, overrides)
    store = typestore(src)
    data, undecoded = read_bag(bag, store, TOPICS + (IMAGE_TOPICS if images != 'none' else ()),
                               first_only=(IMAGE_TOPICS[1],))
    if not data['/scan']:
        raise SystemExit(f'{bag}: no /scan')
    read_s = time.perf_counter() - clock
    name = Path(bag).resolve().name
    lpose = laser_pose(data['/tf_static'], data['/scan'][0].header.frame_id)
    poses = Poses(data['/odometry/filtered'])
    plan, cadence_run, missing = plan_cycles(data['/scan'], data['/perception/walls'],
                                             cadence, every)
    retrack, wall_record = None, {'rule': 'recorded'}
    if walls != 'recorded':
        tp, ep, node, wall_record = wall_setup(src, params, walls.split('-')[1], p, overrides)
        retrack = Retracker(tp, ep, node, lpose)
    detector = Detector(p)
    tracker = detector.tracker
    out = Path(out_dir) if out_dir else default_out(bag, src, walls, cadence_run, frames)
    out.mkdir(parents=True, exist_ok=True)

    life, kills, rows, cycle_rows = {}, [], [], []
    codes = collections.Counter()
    acc, piece_xy, wall_xy, door_ms, walls_ms = [], [], [], [], []
    no_pose = 0
    t0 = stamp(data['/scan'][0])
    sink = []
    loop = time.perf_counter()
    with capture(sink):
        for k, (msg, wmsg) in enumerate(plan):
            t = stamp(msg)
            car = poses.at(t)
            if car is None:
                no_pose += 1        # the node: no pose at the scan stamp, no step
                continue
            r, b, _angles, cut = mwt.scan_in_base_link(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                lpose, (), with_cut=True)
            scan = (r, b, lpose[:2], cut)
            tic = time.perf_counter()
            tracks = (retrack.step(msg, t, car, scan) if retrack is not None
                      else tracks_from(wmsg) if wmsg is not None else [])
            walls_ms.append((time.perf_counter() - tic) * 1e3)
            del sink[:]
            kill0 = getattr(tracker, 'n_pane_kills', 0)
            tic = time.perf_counter()
            pieces, _cands, doors, rejects = detector.cycle(tracks, car, scan, t)
            door_ms.append((time.perf_counter() - tic) * 1e3)
            codes.update(code for code, _ids, _w in rejects)
            new_kills = getattr(tracker, 'n_pane_kills', 0) - kill0
            for pane in (tracker.panes[-new_kills:] if new_kills else []):
                kills.append({'id': int(pane.id), 't': round(t - t0, 2)})
                _life_update(life, pane, t - t0, t, car, seen=False)['pane_killed'] = True
            for door in doors:
                _life_update(life, door, t - t0, t, car)
            new_rows = candidate_rows(sink, k, t - t0, t, car, lpose[:2], tracker, doors)
            rows += new_rows
            eligible = [trk for trk in tracks if dd.eligible(trk)]
            cycle_rows.append({
                'cycle': k, 't': round(t - t0, 3), 'stamp': round(t, 6), 'walls': len(tracks),
                'eligible': len(eligible), 'pieces': len(pieces), 'gaps': len(new_rows),
                'accepted': sum(1 for row in new_rows if row['code'] == 'accepted'),
                'doors': len(doors),
                'confirmed': sum(1 for d in doors if d.state == dd.STATE_CONFIRMED),
                **(retrack.counts if retrack is not None else dict.fromkeys(RETRACK_COUNTS, '')),
                'walls_ms': round(walls_ms[-1], 2), 'door_ms': round(door_ms[-1], 2)})
            ok = np.isfinite(r)
            pts = np.column_stack([lpose[0] + r[ok] * np.cos(b[ok]),
                                   lpose[1] + r[ok] * np.sin(b[ok])])
            acc.append(car.transform_points(pts[np.hypot(pts[:, 0], pts[:, 1]) < 6.0][::3]))
            piece_xy += [car.transform_points(np.vstack([pc.a, pc.b])) for pc in pieces]
            wall_xy += [segment_odom(trk) for trk in eligible]
    loop_s = time.perf_counter() - loop
    frame_id = getattr(dd, 'FRAME_ID', None)
    tracks = {str(k): _track_summary(rec, t0, frame_id) for k, rec in sorted(life.items())}
    confirmed = {k: v for k, v in tracks.items() if 'confirmed' in v['states']}
    n_cycles = len(cycle_rows)
    summary = {
        'bag': name, 'src': str(src), 'src_commit': checkout_commit(src),
        'walls': walls, 'wall_rule': wall_record, 'cadence': cadence_run, 'every': every,
        'cycles': n_cycles, 'cycles_no_pose': no_pose, 'scans_missing_for_steps': missing,
        'undecoded_topics': undecoded, 'params': params, 'overrides': list(overrides),
        'door_params': _rounded(p), 'frame_jambs': detector.frames,
        'tracker_params': _rounded(retrack.trk.p) if retrack is not None else None,
        'door_tracks': len(tracks), 'confirmed_doors': len(confirmed),
        'confirmed_ids': [int(k) for k in confirmed],
        'frame_doors_confirmed': sum(1 for v in confirmed.values() if frame_id is not None
                                     and any(frame_id in pr for pr in v['parents'])),
        'door_kinds': dict(collections.Counter(v['kind'] for v in tracks.values())),
        'pane_kills': kills, 'panes_remembered_at_end': len(getattr(tracker, 'panes', [])),
        'pane_suppressed': int(getattr(tracker, 'n_pane_suppressed', 0)),
        'reject_codes': dict(codes.most_common()),
        'gap_codes': dict(collections.Counter(row['code'] for row in rows).most_common()),
        'walls_per_cycle': {
            'mean': round(float(np.mean([c['walls'] for c in cycle_rows])), 2) if n_cycles
            else None,
            'eligible_mean': round(float(np.mean([c['eligible'] for c in cycle_rows])), 2)
            if n_cycles else None,
            'eligible_max': max((c['eligible'] for c in cycle_rows), default=None)},
        'tracker_refusals': ({k: int(sum(c[k] for c in cycle_rows)) for k in RETRACK_COUNTS[1:]}
                             if retrack is not None else None),
        'ms_linus': {'walls_p50': _pct(walls_ms, 50), 'walls_p95': _pct(walls_ms, 95),
                     'door_p50': _pct(door_ms, 50), 'door_p95': _pct(door_ms, 95)},
        'out': str(out),
        'tracks': tracks,
    }
    tag = f'{name}.doors{"" if frames else "_noframes"}'
    write_csv(out / f'{tag}.candidates.csv', CSV_FIELDS, rows)
    write_csv(out / f'{tag}.cycles.csv', CYCLE_FIELDS, cycle_rows)
    draw(out / f'{tag}.png', acc, piece_xy, wall_xy, tracks, getattr(tracker, 'panes', []),
         data['/odometry/filtered'], f'{name}: {summary["src_commit"] or src.name}, '
         f'walls {walls}, cadence {cadence_run}')
    n_images = 0
    if images != 'none' and data.get(IMAGE_TOPICS[0]):
        for k, v in tracks.items():
            if v['best'] and (images == 'all' or k in confirmed):
                door_image(out / f'door_{k}.png', k, v, data, lpose, poses)
                n_images += 1
    summary['door_images'] = n_images
    summary['seconds'] = {'read': round(read_s, 2), 'loop': round(loop_s, 2),
                          'total': round(time.perf_counter() - clock, 2)}
    (out / f'{tag}.json').write_text(json.dumps(summary, indent=1))
    return summary


# ============================================================================
# Pictures
# ============================================================================

def _pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def draw(path, acc, piece_xy, wall_xy, tracks, panes, odom, title):
    """Draw the overview: accumulated scan, eligible walls, frame pieces, doors, panes."""
    plt = _pyplot()
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(26, 12))
    if acc:
        pts = np.vstack(acc)
        ax.scatter(pts[:, 0], pts[:, 1], s=0.3, c='#b0b0b0', rasterized=True)
    if wall_xy:
        ax.add_collection(LineCollection(wall_xy, colors='#202020', linewidths=0.8,
                                         alpha=0.08))
    if piece_xy:
        ax.add_collection(LineCollection(piece_xy[::2], colors='#3060ff', linewidths=0.6,
                                         alpha=0.15))
    if odom:
        path_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in odom])
        ax.plot(path_xy[:, 0], path_xy[:, 1], 'k-', lw=1)
    for pn in panes:
        ax.plot([pn.jamb0[0], pn.jamb1[0]], [pn.jamb0[1], pn.jamb1[1]], color='#00b0d0', lw=3)
    for k, v in tracks.items():
        if not v['best']:
            continue
        j = np.array(v['best']['jambs'])
        colour = '#10c040' if 'confirmed' in v['states'] else '#ff9020'
        ax.plot(j[:, 0], j[:, 1], '-', color=colour, lw=3)
        ax.text(j[:, 0].mean() + 0.05, j[:, 1].mean() + 0.08, f'{k} {v["width"]:.2f}',
                fontsize=8, color=colour)
    ax.set_aspect('equal')
    ax.autoscale_view()
    ax.grid(True, alpha=0.3)
    ax.set_title(f'{title} -- grey scan, dark eligible walls (every cycle), blue frame pieces, '
                 'green confirmed / orange tentative doors (best sighting), cyan panes')
    fig.tight_layout()
    fig.savefig(path, dpi=60)
    plt.close(fig)


def _quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def camera_model(tf_static, info, frame):
    """
    Return (R, T, (fx, fy, cx, cy), width) of camera `frame`, or None.

    R and T take its optical-frame points into base_link (p_bl = R p + T,
    the /tf_static chain), so a base_link point p is (p - T) @ R there, as
    perception_debug.project_floor_segment has it; K is `info`'s.
    """
    edges = {}
    for msg in tf_static:
        for tr in msg.transforms:
            t = tr.transform.translation
            edges[tr.child_frame_id] = (tr.header.frame_id, _quat(tr.transform.rotation),
                                        np.array([t.x, t.y, t.z]))
    rot, trans = np.eye(3), np.zeros(3)
    while frame != 'base_link':
        if frame not in edges:
            return None
        parent, r, t = edges[frame]
        rot, trans = r @ rot, r @ trans + t
        frame = parent
    return rot, trans, (info.k[0], info.k[4], info.k[2], info.k[5]), info.width


def project_posts(jambs_odom, car, cam, img_width, height_m=2.0, min_depth_m=0.1):
    """
    Return each jamb's post, (u, v) at 0 and height_m, in an image img_width px wide.

    None unless both posts lie in front of the camera and inside the image's
    width. `car` is the pose at the image's stamp; K is scaled by img_width
    over camera_info's width (the debug image is the camera image resized, a
    text strip added BELOW it).
    """
    rot, trans, (fx, fy, cx, cy), info_width = cam
    scale = img_width / float(info_width)
    out = []
    for jx, jy in car.inverse().transform_points(np.asarray(jambs_odom, dtype=float)):
        o = (np.array([[jx, jy, 0.0], [jx, jy, height_m]]) - trans) @ rot
        if not np.all(o[:, 2] > min_depth_m):
            return None
        u = scale * (fx * o[:, 0] / o[:, 2] + cx)
        if not np.all((u >= 0.0) & (u < img_width)):
            return None
        out.append((u, scale * (fy * o[:, 1] / o[:, 2] + cy)))
    return out


def select_frames(frames, t, window, in_view=None):
    """
    Return [(frame, label)] for a door seen best at stamp t (door_<id>.png).

    `frames` are sorted by HEADER stamp, the camera's capture stamp -- never
    by receipt time, which lags it by a frame or two. First the frame nearest
    t; when `in_view(frame)` says the door is not in it, also the frame
    nearest t inside `window` (lo, hi) in which it is.
    """
    stamps = [stamp(f) for f in frames]
    first = frames[nearest_index(stamps, t)]
    shown = [(first, 'nearest by header stamp')]
    if in_view is None or in_view(first):
        return shown
    lo, hi = bisect.bisect_left(stamps, window[0]), bisect.bisect_right(stamps, window[1])
    for i in sorted(range(lo, hi), key=lambda i: abs(stamps[i] - t)):
        if frames[i] is not first and in_view(frames[i]):
            shown.append((frames[i], 'nearest with the door in view'))
            break
    return shown


def save_small_png(fig, path, dpi):
    """Save `fig` as a 256-colour palette PNG (debug frames make big truecolour ones)."""
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=dpi)
    try:
        from PIL import Image
    except ImportError:
        Path(path).write_bytes(buf.getvalue())
        return
    buf.seek(0)
    Image.open(buf).convert('RGB').quantize(colors=256).save(path, optimize=True)


def door_image(path, door_id, track, data, lpose, poses):
    """Write door_<id>.png: the debug frame(s) and a top-down of the door's best sighting."""
    plt = _pyplot()
    frames, best = data[IMAGE_TOPICS[0]], track['best']
    t = best['stamp']
    infos = data.get(IMAGE_TOPICS[1]) or []
    cam = (camera_model(data['/tf_static'], infos[0], frames[0].header.frame_id)
           if infos else None)

    def decode(msg):
        return plt.imread(io.BytesIO(bytes(msg.data)), format='jpeg')

    width = decode(frames[0]).shape[1]

    def posts(frame):
        car = poses.at(stamp(frame))
        return project_posts(best['jambs'], car, cam, width) if car is not None else None

    window = (t - (best['t'] - track['born']) - 5.0, t + (track['last'] - best['t']) + 5.0)
    shown = select_frames(frames, t, window, posts if cam is not None else None)
    fig, axes = plt.subplots(1, len(shown) + 1, figsize=(4.2 * (len(shown) + 1), 3.4))
    for ax, (frame, label) in zip(axes, shown):
        img = decode(frame)
        ax.imshow(img)
        uv = posts(frame) if cam is not None else None
        if uv:
            (u0, v0), (u1, v1) = uv
            ax.fill([u0[0], u0[1], u1[1], u1[0]], [v0[0], v0[1], v1[1], v1[0]],
                    color='lime', alpha=0.25, lw=0)
            for u, v in uv:
                ax.plot(u, v, '-', color='lime', lw=2)
        ax.set_xlim(0, img.shape[1])
        ax.set_ylim(img.shape[0], 0)
        note = ', jambs 0-2 m' if uv else (', door NOT in view' if cam else ', no camera_info')
        ax.set_title(f'{label}: dt {stamp(frame) - t:+.2f} s{note}', fontsize=7)
        ax.axis('off')
    ax = axes[-1]
    j, n = np.array(best['jambs']), np.array(best['normal'])
    centre = j.mean(axis=0)
    scan = data['/scan'][nearest_index([stamp(m) for m in data['/scan']], t)]
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment + lpose[2]
    ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
    car = Pose2D(*best['car'])
    pts = car.transform_points(np.column_stack([lpose[0] + r[ok] * np.cos(a[ok]),
                                                lpose[1] + r[ok] * np.sin(a[ok])]))
    ax.scatter(pts[:, 0], pts[:, 1], s=1.5, c='#888888')
    ax.plot(j[:, 0], j[:, 1], 'g-', lw=3)
    ax.arrow(centre[0], centre[1], 0.4 * n[0], 0.4 * n[1], head_width=0.08, color='g')
    ax.plot(car.x, car.y, 'k^')
    ax.plot([car.x, car.x + 0.5 * math.cos(car.theta)],
            [car.y, car.y + 0.5 * math.sin(car.theta)], 'k-', lw=1)
    ax.set_xlim(centre[0] - 3.0, centre[0] + 3.0)
    ax.set_ylim(centre[1] - 3.0, centre[1] + 3.0)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.tick_params(labelsize=6)
    ax.set_title(f'door {door_id} {"/".join(track["states"])}: w {best["width"]:.2f} m, '
                 f'{best["n_beams"]} beams, parents {best["parents"]}\nleaf {best["leaf"]}, '
                 f'square looks {track["square_looks"]}, t {best["t"]:.1f} s (top-down, odom)',
                 fontsize=7)
    fig.tight_layout()
    save_small_png(fig, path, dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--walls', choices=('recorded', 'retrack-old', 'retrack-new'),
                    default='recorded')
    ap.add_argument('--cadence', choices=('steps', 'every'), default='steps')
    ap.add_argument('--every', type=int, default=4)
    ap.add_argument('--no-frames', action='store_true')
    ap.add_argument('--params', choices=('stack', 'code'), default='stack')
    ap.add_argument('--set', action='append', default=[],
                    metavar='[tracker.|extraction.]FIELD=VALUE')
    ap.add_argument('--src', type=Path, default=REPO)
    ap.add_argument('--door-images', choices=('all', 'confirmed', 'none'), default='all')
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('bag')
    args = ap.parse_args()
    src = args.src.expanduser().resolve()
    if not (src / 'src/f1tenth_perception/f1tenth_perception/door_detection.py').exists():
        ap.error(f'--src {src}: no src/f1tenth_perception/f1tenth_perception/door_detection.py')
    for item in args.set:
        name = item.split('=', 1)[0]
        if '=' not in item or name.count('.') > 1 or (
                '.' in name and name.split('.')[0] not in ('tracker', 'extraction')):
            ap.error(f'--set {item}: FIELD=VALUE, tracker.FIELD=VALUE or extraction.FIELD=VALUE')
        if '.' in name and args.walls == 'recorded':
            ap.error(f'--set {item}: the tracker and the extraction run only with --walls '
                     'retrack-old / retrack-new')
    import_detector(src)
    s = replay(args.bag, args.out, args.walls, args.cadence, args.every, not args.no_frames,
               args.params, args.set, src, args.door_images)
    print(json.dumps({k: v for k, v in s.items()
                      if k not in ('tracks', 'door_params', 'tracker_params')}, indent=1))


if __name__ == '__main__':
    main()
