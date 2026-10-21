"""
Run-report overlay: the camera frame with the scene mask and the LiDAR on it.

Drawn OFFLINE, from a run's bag:

    python3 -m f1tenth_logger.mission_overlay <run_dir> --out-dir DIR [--count N]
    ros2 run f1tenth_logger mission_overlay <run_dir> --out-dir DIR

writes N evenly spaced overlay stills, <out_dir>/<run_id>.NNN.jpg: the same
stills mission_logger_node writes to <run_dir>/snapshots/ at finalize, which
the app's Run view shows. The per-run video is mission_debug_video's, which
draws every frame through here when the run has the data.

THE PICTURE. The camera image; the scene mask's WALL, FLOOR, FLOOR_INFERRED
and OBJECT classes as semi-transparent fills with thin outlines; the /scan
returns projected into the image as small dots coloured by range (red near,
yellow, white far); a small legend in the top-left corner. No other text: it
has to stay readable as a camera image. Drawn by
f1tenth_perception.scene_mask.overlay, the renderer of the live
/perception/scene_labels/image debug overlay -- one renderer, two callers;
the report only asks it for fewer classes, outlines, range colours and the
legend. Fill alpha, dot size and legend on/off are stack_params.yaml's
mission_report_overlay_* (style_from_params).

WHY OFFLINE. Nothing is drawn on the car during a run: the inputs are already
in the bag, and the drawing happens after the run, here or at finalize (ten
stills). The one thing the default recording lacked was a camera frame with
nothing drawn on it; yolo_detector_node now publishes its downscaled 5 Hz
debug-tick frame clean, and only while the logger records it.

THE DATA, best source first:

  camera    /camera/image_raw_5hz       campaign profile: the frame YOLO and
                                        the mask use, full resolution
            /camera/debug_image/clean/compressed
                                        yolo_detector_node's debug-tick frame
                                        before anything is drawn on it (480 px
                                        JPEG at 5 Hz); the default recording
            /camera/image_annotated     record_annotated_image: YOLO's boxes on
  mask      /perception/scene_labels/classes
                                        campaign profile: the car's own mask
            REBUILT                     from the 5 Hz depth (record_depth, on by
                                        default): scene_mask.classify and
                                        inferred_floor, the YOLO instance masks
                                        of /camera/detection_masks, and the
                                        run's OWN params snapshot -- the pure
                                        functions front_clearance_node's mask
                                        tick calls, on the parameters the run
                                        flew with (a key the snapshot lacks:
                                        today's stack_params.yaml)
  LiDAR     /scan
  geometry  /camera/camera_info, else the depth's; /tf_static for base_link
            to the camera's optical frame and to the laser (/tf only for a
            link /tf_static lacks)

A run without a camera frame, a mask source or the camera geometry gets no
overlay: open_run says why, and the callers keep what they did before (the
recorded debug JPEGs). A run that only lacks data is never an error.

PAIRING. By header stamp: the mask nearest a camera frame within MASK_TOL_S,
the scan within SCAN_TOL_S. A frame with no mask that near is drawn with the
LiDAR alone in the video and is not used as a still. The bag is indexed by
receive time without reading a blob, and a message is found by bisecting on
the stamps of the rows the search touches (Topic), so a still costs a few
depth frames read, not the whole run, however late the recorder was.

A REBUILT MASK IS THE CAR'S RULES ON THE NEAREST INPUTS, not the car's mask:
the recorder's 5 Hz depth is not the frame the car's own 5 Hz mask tick used,
and its objects are the YOLO masks of the nearest detection frame (the node
takes the newest whose mask and detections share a stamp, and falls back to
boxes without a mask). The campaign profile records the car's own.

NO ROS NEEDED, like mission_debug_video: the bag is read with sqlite3 and each
message is decoded by hand from its CDR bytes. numpy, OpenCV, PyYAML and
f1tenth_perception's pure modules. Only sqlite3 bags (the car records
sqlite3, see mission_logger_node's STORAGE FORMAT note).
"""

import argparse
import bisect
import collections
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys

import cv2
from f1tenth_logger.mission_debug_video import find_db3, run_id_of
from f1tenth_perception import inferred_floor, scene_mask
from f1tenth_perception.ground_plane import quat_to_matrix
import numpy as np
import yaml

#: Camera sources, best first (module docstring).
CAMERA_TOPICS = ('/camera/image_raw_5hz', '/camera/debug_image/clean/compressed',
                 '/camera/image_annotated')
CLASSES_TOPIC = '/perception/scene_labels/classes'
DEPTH_TOPIC = '/zed2/zed_node/depth/depth_registered_5hz'
DEPTH_INFO_TOPIC = '/zed2/zed_node/depth/camera_info'
CAMERA_INFO_TOPICS = ('/camera/camera_info', DEPTH_INFO_TOPIC)
DETECTION_MASKS_TOPIC = '/camera/detection_masks'
SCAN_TOPIC = '/scan'
BASE_FRAME = 'base_link'

#: Pairing windows [s] by header stamp. The camera frame and the 5 Hz depth
#: are picked from the same 30 Hz grabs by two unrelated 5 Hz throttles, so
#: the nearest depth is 0 to 3 grabs (0-100 ms) away: 88 of the 89 debug
#: frames of 2026-10-21T12-25-22 paired within 0.101 s, the other was the
#: first frame (0.333 s). 0.12 takes every 5 Hz neighbour and nothing older.
#: The scan runs at ~22 Hz, the detection masks at the detector's ~13 Hz.
MASK_TOL_S = 0.12
SCAN_TOL_S = 0.1
OBJECT_TOL_S = 0.1
#: The stamp search gives up after this many rows in a row without a stamp.
UNSTAMPED_RUN = 8

#: The stack_params.yaml keys a rebuilt mask reads: front_clearance_node's
#: mask and inferred-floor parameters, and the lidar_exclusion_* rectangle
#: that is its self mask when no polygon is configured.
MASK_PARAMS = tuple('front_clearance_' + name for name in (
    'scene_mask_floor_tol_m', 'scene_mask_ceiling_min_z_m', 'scene_mask_min_depth_m',
    'scene_mask_max_depth_m', 'scene_mask_self_polygons', 'scene_mask_occluded_polygons',
    'inferred_floor_enabled', 'inferred_floor_r_max_m', 'inferred_floor_margin_m',
    'inferred_floor_window_k', 'inferred_floor_sigma_beams',
    'inferred_floor_max_scan_age_s')) + (
    'lidar_exclusion_x_min', 'lidar_exclusion_y_min', 'lidar_exclusion_x_max',
    'lidar_exclusion_y_max')


@dataclass
class Style:
    """How a report frame is drawn. The first three are stack_params.yaml's."""

    alpha: float = 0.3          # mission_report_overlay_alpha
    point_px: int = 2           # mission_report_overlay_point_px
    legend: bool = True         # mission_report_overlay_legend
    outline_px: int = 1
    range_max_m: float = 8.0    # the colour scale's far end
    jpeg_quality: int = 90


_STYLE_PARAMS = (('alpha', 'mission_report_overlay_alpha', float),
                 ('point_px', 'mission_report_overlay_point_px', int),
                 ('legend', 'mission_report_overlay_legend', bool))


def style_from_params(**overrides):
    """
    Return the Style in stack_params.yaml (the installed copy), then `overrides`.

    A report host without f1tenth_params installed gets the Style defaults,
    which are the yaml's. An override of None is no override. param_defaults
    reads the yaml once per process: the commands see an edit on their next
    run, mission_logger_node's finalize stills at its next start.
    """
    style = Style()
    try:
        from f1tenth_params.param_defaults import get_value
        for attr, key, cast in _STYLE_PARAMS:
            setattr(style, attr, cast(get_value(key)))
    except Exception:  # noqa: BLE001 - no f1tenth_params (or no key): the defaults
        pass
    for attr, value in overrides.items():
        if value is not None:
            setattr(style, attr, value)
    return style


# ---- CDR ----------------------------------------------------------------------


class _Cdr:
    """Little-endian CDR reader, alignment relative to the body after the 4-byte header."""

    def __init__(self, raw):
        if len(raw) < 4 or raw[1] != 0x01:
            raise ValueError('not a little-endian CDR message')
        self.buf = memoryview(raw)[4:]
        self.off = 0

    def _take(self, size, align):
        self.off += (-self.off) % align
        if self.off + size > len(self.buf):
            raise ValueError('truncated')
        start = self.off
        self.off += size
        return start

    def scalar(self, fmt):
        size = struct.calcsize(fmt)
        return struct.unpack_from(fmt, self.buf, self._take(size, size))[0]

    def string(self):
        n = self.scalar('<I')
        start = self._take(n, 1)
        return bytes(self.buf[start:start + n]).rstrip(b'\0').decode('utf-8', 'replace')

    def array(self, dtype, n):
        dt = np.dtype(dtype)
        if n == 0:
            return np.empty(0, dtype=dt)
        return np.frombuffer(self.buf, dt, n, self._take(dt.itemsize * n, dt.itemsize))

    def sequence(self, dtype):
        return self.array(dtype, self.scalar('<I'))

    def header(self):
        sec = self.scalar('<i')
        nanosec = self.scalar('<I')
        return sec + nanosec * 1e-9, self.string().lstrip('/')


#: sensor_msgs/Image encodings read here: (dtype, channels).
_ENCODINGS = {
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1), 'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3), 'bgra8': (np.uint8, 4), 'rgba8': (np.uint8, 4),
    '32FC1': (np.float32, 1), 'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
}


def decode_image(raw):
    """
    Decode a CDR sensor_msgs/Image into (stamp_sec, frame_id, encoding, array).

    The array is (H, W) or (H, W, C) in the encoding's own dtype and channel
    order, or None for a zero-size image (yolo_detector_node's empty mask:
    "no detections this frame"). Raises ValueError on a truncated message or
    an encoding not in _ENCODINGS.
    """
    c = _Cdr(raw)
    stamp, frame = c.header()
    h, w = c.scalar('<I'), c.scalar('<I')
    encoding = c.string()
    bigendian = c.scalar('<B')
    step = c.scalar('<I')
    data = c.sequence(np.uint8)
    if h == 0 or w == 0:
        return stamp, frame, encoding, None
    if encoding not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding {encoding!r}')
    dtype, channels = _ENCODINGS[encoding]
    dt = np.dtype(dtype).newbyteorder('>' if bigendian else '<')
    row_bytes = w * channels * dt.itemsize
    if step < row_bytes or len(data) < h * step:
        raise ValueError('image data shorter than its geometry')
    rows = data[:h * step].reshape(h, step)[:, :row_bytes]
    px = np.ascontiguousarray(rows).view(dt).reshape((h, w, channels) if channels > 1 else (h, w))
    return stamp, frame, encoding, px.astype(dtype, copy=False)


def decode_compressed_image(raw):
    """Decode a CDR sensor_msgs/CompressedImage into (stamp_sec, frame_id, format, bytes)."""
    c = _Cdr(raw)
    stamp, frame = c.header()
    fmt = c.string()
    return stamp, frame, fmt, c.sequence(np.uint8).tobytes()


def decode_laser_scan(raw):
    """Decode a CDR sensor_msgs/LaserScan into a dict (stamp, frame_id, geometry, ranges)."""
    c = _Cdr(raw)
    stamp, frame = c.header()
    angle_min, _angle_max, angle_increment, _t_inc, _scan_t, range_min, range_max = (
        c.scalar('<f') for _ in range(7))
    return {'stamp': stamp, 'frame_id': frame, 'angle_min': angle_min,
            'angle_increment': angle_increment, 'range_min': range_min,
            'range_max': range_max, 'ranges': c.sequence(np.float32)}


def decode_camera_info(raw):
    """Decode a CDR sensor_msgs/CameraInfo into (stamp_sec, frame_id, (w, h), (fx, fy, cx, cy))."""
    c = _Cdr(raw)
    stamp, frame = c.header()
    h, w = c.scalar('<I'), c.scalar('<I')
    c.string()                                    # distortion_model
    c.sequence(np.float64)                        # d
    k = c.array(np.float64, 9)
    return stamp, frame, (int(w), int(h)), (float(k[0]), float(k[4]), float(k[2]), float(k[5]))


def decode_tf_message(raw):
    """Decode a CDR tf2_msgs/TFMessage into [(parent, child, (x, y, z), (qx, qy, qz, qw))]."""
    c = _Cdr(raw)
    out = []
    for _ in range(c.scalar('<I')):
        _stamp, parent = c.header()
        child = c.string().lstrip('/')
        xyz = tuple(float(v) for v in c.array(np.float64, 3))
        quat = tuple(float(v) for v in c.array(np.float64, 4))
        out.append((parent, child, xyz, quat))
    return out


# ---- the bag ------------------------------------------------------------------


class Bag:
    """A run's sqlite3 files, read-only."""

    def __init__(self, db3_files):
        self.cons = [sqlite3.connect(f'file:{p}?mode=ro', uri=True) for p in db3_files]

    def close(self):
        for con in self.cons:
            con.close()

    def rows(self, topic):
        """[(receive_ns, file index, row id)] of `topic`, sorted; no blob is read."""
        out = []
        for ci, con in enumerate(self.cons):
            row = con.execute('SELECT id FROM topics WHERE name = ?', (topic,)).fetchone()
            if row is not None:
                out += [(ts, ci, rid) for rid, ts in con.execute(
                    'SELECT id, timestamp FROM messages WHERE topic_id = ?', (row[0],))]
        out.sort()
        return out

    def blob(self, ci, rid):
        row = self.cons[ci].execute('SELECT data FROM messages WHERE id = ?', (rid,)).fetchone()
        return bytes(row[0]) if row is not None else None

    def messages(self, topic):
        """Yield every raw message of `topic` in receive order."""
        for _ts, ci, rid in self.rows(topic):
            raw = self.blob(ci, rid)
            if raw is not None:
                yield raw


class Topic:
    """
    One topic, found by HEADER stamp without reading every blob.

    `decode` returns a tuple or dict whose stamp `stamp_of` extracts. The rows
    are indexed by receive time, and the stamps of one publisher's topic rise
    in that order, so a stamp is found by bisecting on the stamps themselves,
    decoding only the rows the search touches. It starts at t plus the
    topic's median receive-minus-header delay, measured on up to five
    messages spread over the run, so a lookup usually costs two or three
    blobs. The delay only places the start: a recorder that falls behind
    (/scan of corridor10m_20261019T131815 arrived -0.002 to 0.679 s after its
    stamp) or a clock that runs at another rate (a sim below real time) costs
    a few more blobs, never a miss. Decoded messages are kept in a small LRU,
    stamps for good.
    """

    def __init__(self, bag, name, decode, stamp_of=lambda m: m[0], cache=8):
        self.bag, self.name, self.decode, self.stamp_of = bag, name, decode, stamp_of
        self.rows = bag.rows(name)
        self.recv = [ts * 1e-9 for ts, _ci, _rid in self.rows]
        self._stamps = {}
        self._cache = collections.OrderedDict()
        self._cache_size = cache
        self._lag = None

    def __len__(self):
        return len(self.rows)

    def message(self, i):
        """Return the decoded message at row i, or None if it does not decode."""
        if i in self._cache:
            self._cache.move_to_end(i)
            return self._cache[i]
        _ts, ci, rid = self.rows[i]
        raw = self.bag.blob(ci, rid)
        try:
            msg = self.decode(raw) if raw is not None else None
        except ValueError:
            msg = None
        stamp = self.stamp_of(msg) if msg is not None else None
        # A zero stamp is an unstamped message (the last detection mask of
        # corridor_obstacles_20261019T135307): no time to be found by.
        self._stamps[i] = stamp if stamp is not None and stamp > 0.0 else None
        self._cache[i] = msg
        if len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)
        return msg

    def stamp(self, i):
        """Return row i's header stamp, or None (it does not decode, or is zero)."""
        if i not in self._stamps:
            self.message(i)
        return self._stamps[i]

    def lag(self):
        if self._lag is None:
            n = len(self.rows)
            picks = sorted({int(round(k * (n - 1) / 4.0)) for k in range(5)}) if n else []
            lags = [self.recv[i] - self.stamp(i) for i in picks if self.stamp(i) is not None]
            self._lag = float(np.median(lags)) if lags else 0.0
        return self._lag

    def _key(self, i):
        """Row i's stamp for the bisection; a row without one takes the next row's."""
        for j in range(i, min(i + UNSTAMPED_RUN, len(self.rows))):
            s = self.stamp(j)
            if s is not None:
                return s
        return math.inf

    def first_at_or_after(self, t):
        """Return the first row stamped at or after t (len(self) if none)."""
        n, key = len(self.rows), self._key
        g = min(bisect.bisect_left(self.recv, t + self.lag()), n - 1)
        # Gallop from the guess to a bracket lo < answer <= hi (lo -1 and hi
        # n stand for the ends), then bisect it.
        if key(g) < t:
            lo, hi, step = g, g + 1, 1
            while hi < n and key(hi) < t:
                lo, step = hi, step * 2
                hi = lo + step
            hi = min(hi, n)
        else:
            lo, hi, step = g - 1, g, 1
            while lo >= 0 and key(lo) >= t:
                hi, step = lo, step * 2
                lo = hi - step
            lo = max(lo, -1)
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if key(mid) < t:
                lo = mid
            else:
                hi = mid
        return hi

    def nearest(self, t, tol):
        """Return the row whose header stamp is nearest t, within tol, or None."""
        n = len(self.rows)
        if not n:
            return None
        first = self.first_at_or_after(t)
        best = None
        # Out from the bracket both ways until a stamp is past tol; a run of
        # rows without a stamp ends the walk too.
        for start, step in ((first, 1), (first - 1, -1)):
            i, unstamped = start, 0
            while 0 <= i < n and unstamped < UNSTAMPED_RUN:
                s = self.stamp(i)
                if s is None:
                    unstamped += 1
                else:
                    unstamped = 0
                    if (s - t) * step > tol:
                        break
                    if abs(s - t) <= tol and (best is None or abs(s - t) < best[0]):
                        best = (abs(s - t), i)
                i += step
        return None if best is None else best[1]


class StaticTf:
    """base_link <- frame transforms from /tf_static, /tf only for a link it lacks."""

    def __init__(self, bag, base=BASE_FRAME):
        self.bag, self.base = bag, base
        self.links = {}
        self._dynamic_read = False
        for raw in bag.messages('/tf_static'):
            self._add(raw, overwrite=True)

    def _add(self, raw, overwrite):
        try:
            transforms = decode_tf_message(raw)
        except ValueError:
            return
        for parent, child, xyz, quat in transforms:
            if overwrite or child not in self.links:
                self.links[child] = (parent, quat_to_matrix(*quat), np.asarray(xyz, dtype=float))

    def to_base(self, frame):
        """(R, t) taking `frame` points into base_link, or None while a link is unknown."""
        rot, trans = np.eye(3), np.zeros(3)
        f, seen = (frame or '').lstrip('/'), set()
        while f != self.base:
            if f not in self.links and not self._dynamic_read:
                # Sensor mounts are static; the first /tf sample of a link
                # /tf_static lacks is as good as any.
                self._dynamic_read = True
                for raw in self.bag.messages('/tf'):
                    self._add(raw, overwrite=False)
            if f not in self.links or f in seen:
                return None
            seen.add(f)
            parent, r_link, t_link = self.links[f]
            rot, trans = r_link @ rot, r_link @ trans + t_link
            f = parent
        return rot, trans


def run_dir_of(run):
    """Return the run folder (the parent of bag/) of a run dir, bag dir or .db3, or None."""
    path = Path(run).expanduser().resolve()
    if path.suffix == '.db3':
        path = path.parent
    if path.name == 'bag':
        return path.parent
    return path if (path / 'bag').is_dir() else None


def run_params(run):
    """{key: default} of the run's own params snapshot (<run_dir>/*.params.yaml), or {}."""
    run_dir = run_dir_of(run)
    # libyaml's loader when PyYAML has it (Ubuntu's does): the pure-Python one
    # spends 0.7 s on the 340 KB snapshot, most of a finalize's ten stills.
    loader = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)
    for path in sorted(run_dir.glob('*.params.yaml')) if run_dir is not None else ():
        try:
            data = yaml.load(path.read_text(), Loader=loader) or {}
        except (OSError, yaml.YAMLError):
            continue
        return {k: v['default'] for k, v in data.items()
                if isinstance(v, dict) and 'default' in v}
    return {}


def _polygons(text):
    """front_clearance_node's polygon JSON ([[[x, y], ...], ...] fractions); [] if bad."""
    try:
        return [[(float(x), float(y)) for x, y in poly] for poly in json.loads(text or '[]')]
    except (ValueError, TypeError):
        return []


def _scaled_k(info, w, h):
    """Return CameraInfo (wh, k) intrinsics rescaled to a w x h image."""
    (kw, kh), (fx, fy, cx, cy) = info
    sx, sy = w / float(kw), h / float(kh)
    return fx * sx, fy * sy, cx * sx, cy * sy


# ---- the mask ----------------------------------------------------------------


class RecordedMask:
    """The car's own mask (/perception/scene_labels/classes, campaign profile)."""

    def __init__(self, topic):
        self.topic = topic

    def offset_at(self, t):
        """|stamp difference| [s] to the mask a frame at t would get, or None."""
        i = self.topic.nearest(t, MASK_TOL_S)
        return None if i is None else abs(self.topic.stamp(i) - t)

    def labels_at(self, t):
        i = self.topic.nearest(t, MASK_TOL_S)
        msg = self.topic.message(i) if i is not None else None
        return None if msg is None or msg[3] is None else msg[3]


class RebuiltMask:
    """The scene mask rebuilt from the recorded 5 Hz depth (module docstring)."""

    def __init__(self, depth, depth_info, masks, scans, tf, params):
        """
        `params` is the run's params snapshot (run_params).

        A key the snapshot lacks (a run older than the key, a bag with no
        snapshot) comes from the installed stack_params.yaml: that is what
        the launch files hand the node, never the node's code default.
        Raises KeyError for a key in neither (MASK_PARAMS).
        """
        def p(name):
            key = name if name.startswith('lidar_') else 'front_clearance_' + name
            if key in params:
                return params[key]
            from f1tenth_params.param_defaults import get_value
            return get_value(key)

        self.depth, self.depth_info, self.masks, self.scans, self.tf = (
            depth, depth_info, masks, scans, tf)
        self.floor_tol = float(p('scene_mask_floor_tol_m'))
        self.ceiling_min_z = float(p('scene_mask_ceiling_min_z_m'))
        self.min_depth = float(p('scene_mask_min_depth_m'))
        self.max_depth = float(p('scene_mask_max_depth_m'))
        self.polygons = _polygons(p('scene_mask_self_polygons')) or [scene_mask.rect_polygon(
            *(float(p(k)) for k in ('lidar_exclusion_x_min', 'lidar_exclusion_y_min',
                                    'lidar_exclusion_x_max', 'lidar_exclusion_y_max')))]
        self.polygons += _polygons(p('scene_mask_occluded_polygons'))
        self.floor_on = bool(p('inferred_floor_enabled'))
        self.floor_r_max = float(p('inferred_floor_r_max_m'))
        self.floor_margin = float(p('inferred_floor_margin_m'))
        self.floor_window_k = int(p('inferred_floor_window_k'))
        self.floor_sigma = float(p('inferred_floor_sigma_beams'))
        self.floor_scan_age = float(p('inferred_floor_max_scan_age_s'))
        self._self_mask = None
        self._lut = None
        self._last = (None, None)

    def offset_at(self, t):
        """|stamp difference| [s] to the depth a frame at t would be masked from, or None."""
        i = self.depth.nearest(t, MASK_TOL_S)
        return None if i is None else abs(self.depth.stamp(i) - t)

    def labels_at(self, t):
        i = self.depth.nearest(t, MASK_TOL_S)
        if i is None:
            return None
        if self._last[0] != i:
            self._last = (i, self._build(self.depth.message(i)))
        return self._last[1]

    def _build(self, msg):
        if msg is None or msg[3] is None or msg[3].ndim != 2:
            return None
        stamp, frame, _enc, depth = msg
        if depth.dtype == np.uint16:
            # 16UC1 millimetres (the ZED's openni depth mode); 0 is no depth.
            depth = np.where(depth > 0, depth.astype(np.float32) / 1000.0, np.nan)
        elif depth.dtype != np.float32:
            return None
        h, w = depth.shape[:2]
        pose = self.tf.to_base(frame)
        if pose is None:
            return None
        k = _scaled_k(self.depth_info, w, h)
        if self._self_mask is None or self._self_mask.shape != (h, w):
            self._self_mask = scene_mask.polygon_mask((h, w), self.polygons)
        labels, _z = scene_mask.classify(
            depth, *k, *pose, self_mask=self._self_mask,
            object_mask=self._object_mask(stamp, (h, w)), floor_tol=self.floor_tol,
            ceiling_min_z=self.ceiling_min_z, min_depth=self.min_depth,
            max_depth=self.max_depth)
        if self.floor_on:
            self._infer_floor(labels, stamp, k, pose)
        return labels

    def _object_mask(self, stamp, shape_hw):
        """Return the YOLO masks nearest the depth frame as one bool image, or None."""
        i = self.masks.nearest(stamp, OBJECT_TOL_S)
        msg = self.masks.message(i) if i is not None else None
        if msg is None or msg[3] is None:
            return None
        raw = msg[3] if msg[3].ndim == 2 else msg[3][..., 0]
        mask = (raw > 0).astype(np.uint8)
        if mask.shape != tuple(shape_hw):
            mask = cv2.resize(mask, (shape_hw[1], shape_hw[0]), interpolation=cv2.INTER_NEAREST)
        return mask.astype(bool)

    def _infer_floor(self, labels, stamp, k, pose):
        """front_clearance_node._infer_floor_step on the scan nearest the depth frame."""
        i = self.scans.nearest(stamp, self.floor_scan_age)
        scan = self.scans.message(i) if i is not None else None
        laser = self.tf.to_base(scan['frame_id']) if scan is not None else None
        if laser is None:
            return
        r_l, t_l = laser
        laser_pose = (float(t_l[0]), float(t_l[1]), math.atan2(r_l[1, 0], r_l[0, 0]))
        h, w = labels.shape[:2]
        n = len(scan['ranges'])
        key = inferred_floor.lut_key((h, w), k, pose[0], pose[1], laser_pose,
                                     (scan['angle_min'], scan['angle_increment'], n),
                                     self.floor_r_max)
        if self._lut is None or not self._lut.matches(key):
            self._lut = inferred_floor.build_lut(
                (h, w), *k, pose[0], pose[1], laser_pose, scan['angle_min'],
                scan['angle_increment'], n, r_max=self.floor_r_max, self_mask=self._self_mask)
        if self._lut.n_valid == 0:
            return
        r_eff = inferred_floor.effective_ranges(
            scan['ranges'], scan['range_min'], scan['range_max'],
            far_value=self.floor_r_max + self.floor_margin + 1.0,
            window_k=self.floor_window_k, sigma_beams=self.floor_sigma)
        inferred_floor.apply(labels, self._lut, r_eff, margin=self.floor_margin)


# ---- the run -----------------------------------------------------------------


def _camera_image(msg, compressed):
    """(stamp, frame_id, BGR uint8 image or None) of a camera message."""
    if msg is None:
        return None, None, None
    if compressed:
        stamp, frame, _fmt, data = msg
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        return stamp, frame, img
    stamp, frame, encoding, px = msg
    if px is None or px.dtype != np.uint8:
        return stamp, frame, None
    convert = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
               'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
               '8UC1': cv2.COLOR_GRAY2BGR}.get(encoding)
    return stamp, frame, cv2.cvtColor(px, convert) if convert is not None else px


def draw(img, labels, scan, laser, camera, k, style):
    """
    Draw one report frame: scene_mask.overlay with the report's options.

    img: the BGR camera frame. labels: the scene-mask classes at any
    resolution, or None. scan: a decode_laser_scan dict, or None; laser and
    camera: (R, t) of the laser and the camera's optical frame in base_link;
    k: (fx, fy, cx, cy) at img's resolution. The LiDAR is drawn far first,
    so the near returns end on top.
    """
    uv = colours = None
    if scan is not None:
        pts, ranges = scene_mask.scan_to_base(
            scan['ranges'], scan['angle_min'], scan['angle_increment'],
            scan['range_min'], scan['range_max'], *laser)
        far_first = np.argsort(-ranges, kind='stable')
        uv = scene_mask.project_to_image(pts[far_first], *k, *camera)
        colours = scene_mask.range_colours(ranges[far_first], style.range_max_m)
    return scene_mask.overlay(
        img, labels, scan_px=uv, alpha=style.alpha, classes=scene_mask.REPORT_CLASSES,
        outline_px=style.outline_px, scan_colours=colours, point_px=style.point_px,
        legend_range_m=style.range_max_m if style.legend else None)


class RunOverlay:
    """Everything one run's overlay frames are drawn from; made by open_run."""

    def __init__(self, bag, camera, camera_info, mask, scans, tf, style):
        self.bag, self.camera, self.camera_info = bag, camera, camera_info
        self.mask, self.scans, self.tf, self.style = mask, scans, tf, style
        self.compressed = camera.name.endswith('/compressed')
        self.mask_source = 'recorded' if isinstance(mask, RecordedMask) else 'rebuilt'

    def __len__(self):
        return len(self.camera)

    def close(self):
        self.bag.close()

    def stamp(self, i):
        """Return camera frame i's stamp, or None if it has none (a zero header stamp)."""
        stamp = self.camera.stamp(i)
        return stamp if stamp is not None and stamp > 0.0 else None

    def render(self, i):
        """Return (camera frame i drawn, whether it has a mask), or (None, False)."""
        stamp, frame, img = _camera_image(self.camera.message(i), self.compressed)
        cam = self.tf.to_base(frame) if img is not None else None
        if cam is None or self.stamp(i) is None:
            return None, False
        labels = self.mask.labels_at(stamp)
        j = self.scans.nearest(stamp, SCAN_TOL_S)
        scan = self.scans.message(j) if j is not None else None
        laser = self.tf.to_base(scan['frame_id']) if scan is not None else None
        k = _scaled_k(self.camera_info, img.shape[1], img.shape[0])
        return draw(img, labels, scan if laser is not None else None, laser, cam, k,
                    self.style), labels is not None

    def jpegs(self):
        """Yield (stamp, JPEG bytes) of every camera frame that draws."""
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.style.jpeg_quality)]
        for i in range(len(self)):
            img, _masked = self.render(i)
            if img is None:
                continue
            ok, buf = cv2.imencode('.jpg', img, params)
            if ok:
                yield self.stamp(i), buf.tobytes()


def open_run(run, style=None):
    """
    Return (RunOverlay, None), or (None, reason) when the run cannot have one.

    `run` is a run folder, a bag directory or a .db3. The reason is a short
    sentence for a log line; a run that only lacks data is never an error.
    """
    db3 = find_db3(run)
    if not db3:
        return None, 'no .db3 bag (only sqlite3 bags are read)'
    bag = Bag(db3)
    try:
        overlay, reason = _open(bag, run, style or style_from_params())
    except Exception:
        bag.close()
        raise
    if overlay is None:
        bag.close()
    return overlay, reason


def _open(bag, run, style):
    camera = next((t for t in (Topic(bag, name, _camera_decoder(name)) for name in CAMERA_TOPICS)
                   if len(t)), None)
    if camera is None:
        return None, 'no camera frame (none of {})'.format(', '.join(CAMERA_TOPICS))
    info = None
    for name in CAMERA_INFO_TOPICS:
        for raw in bag.messages(name):
            try:
                _s, _f, wh, k = decode_camera_info(raw)
            except ValueError:
                continue
            if wh[0] > 0 and wh[1] > 0 and k[0] > 0.0:
                info = (wh, k)
                break
        if info is not None:
            break
    if info is None:
        return None, 'no camera_info (none of {})'.format(', '.join(CAMERA_INFO_TOPICS))
    tf = StaticTf(bag)
    scans = Topic(bag, SCAN_TOPIC, decode_laser_scan, stamp_of=lambda m: m['stamp'], cache=64)
    classes = Topic(bag, CLASSES_TOPIC, decode_image)
    depth = Topic(bag, DEPTH_TOPIC, decode_image)
    if len(classes):
        mask = RecordedMask(classes)
    elif len(depth):
        depth_info = None
        for raw in bag.messages(DEPTH_INFO_TOPIC):
            try:
                _s, _f, wh, k = decode_camera_info(raw)
            except ValueError:
                continue
            depth_info = (wh, k)
            break
        try:
            mask = RebuiltMask(depth, depth_info or info,
                               Topic(bag, DETECTION_MASKS_TOPIC, decode_image), scans, tf,
                               run_params(run))
        except (ImportError, KeyError) as exc:
            return None, f'no mask parameters: {exc!r} (not in the run params, no stack_params)'
    else:
        return None, f'no scene mask (neither {CLASSES_TOPIC} nor {DEPTH_TOPIC})'
    first = next((camera.message(i) for i in range(min(len(camera), 5))
                  if camera.message(i) is not None), None)
    if first is None or tf.to_base(first[1]) is None:
        return None, 'camera pose unknown ({!r} -> {} not in /tf_static or /tf)'.format(
            first[1] if first is not None else '?', BASE_FRAME)
    return RunOverlay(bag, camera, info, mask, scans, tf, style), None


def _camera_decoder(name):
    return decode_compressed_image if name.endswith('/compressed') else decode_image


def spread(count, limit):
    """Up to `limit` indices evenly spread over range(count), first and last included."""
    if count <= 0 or limit <= 0:
        return []
    if count <= limit:
        return list(range(count))
    if limit == 1:
        return [0]
    step = (count - 1) / (limit - 1)
    return sorted({int(round(i * step)) for i in range(limit)})


def pick_stills(count, offset_at, limit=10, search=3):
    """
    Return the camera frames (of `count`) the stills are drawn from, in time order.

    One per place spread evenly over the run: of the frames within `search`
    of the place and not picked yet, the one whose mask is nearest in time
    (`offset_at(i)`: |stamp difference| [s], None for no mask), then the one
    nearest the place. On a run shorter than about 2 * search * limit frames
    the windows overlap, and a place can pick a frame before its
    neighbour's, hence the sort: the gateway shows the stills by name.
    """
    # The windows overlap below (2 * search + 1) * (limit - 1) + 1 frames.
    picked = set()
    for target in spread(count, limit):
        best = None
        for i in range(max(0, target - search), min(count, target + search + 1)):
            off = offset_at(i) if i not in picked else None
            if off is not None and (best is None or (off, abs(i - target)) < best[0]):
                best = ((off, abs(i - target)), i)
        if best is not None:
            picked.add(best[1])
    return sorted(picked)


def write_snapshots(run, out_dir, run_id=None, limit=10, style=None, search=3):
    """
    Write up to `limit` overlay stills as <out_dir>/<run_id>.NNN.jpg.

    Returns (paths, None), or ([], reason) with nothing written: the caller
    then writes the stills it always wrote. The frames are pick_stills', so
    a still never shows the LiDAR alone and its mask is as close to the
    image as the run allows. All are drawn before any is written, so a
    failure leaves no partial set behind.
    """
    overlay, reason = open_run(run, style)
    if overlay is None:
        return [], reason
    try:
        def offset_at(i):
            stamp = overlay.stamp(i)
            return overlay.mask.offset_at(stamp) if stamp is not None else None

        drawn = []
        for i in pick_stills(len(overlay), offset_at, limit, search):
            img, masked = overlay.render(i)
            if img is not None and masked:
                drawn.append(img)
        quality = overlay.style.jpeg_quality
    finally:
        overlay.close()
    if not drawn:
        return [], f'no camera frame has a mask within {MASK_TOL_S} s'
    out_dir = Path(out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    run_id = run_id or run_id_of(run)
    paths = []
    for n, img in enumerate(drawn):
        path = out_dir / f'{run_id}.{n:03d}.jpg'
        if cv2.imwrite(str(path), img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]):
            paths.append(str(path))
    return paths, None


def main(argv=None):
    """Write a run's overlay stills. No ROS required."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', type=Path,
                    help='run folders, bag directories or .db3 files')
    ap.add_argument('--out-dir', type=Path, required=True)
    ap.add_argument('--count', type=int, default=10)
    ap.add_argument('--alpha', type=float, default=None,
                    help='mask fill opacity (default: mission_report_overlay_alpha)')
    ap.add_argument('--point-px', type=int, default=None,
                    help='LiDAR dot size (default: mission_report_overlay_point_px)')
    ap.add_argument('--no-legend', dest='legend', action='store_false', default=None)
    cfg = ap.parse_args(argv)
    style = style_from_params(alpha=cfg.alpha, point_px=cfg.point_px, legend=cfg.legend)
    wrote = False
    for run in cfg.runs:
        paths, reason = write_snapshots(run, cfg.out_dir, limit=cfg.count, style=style)
        if paths:
            wrote = True
            print(f'{run}: {len(paths)} overlay still(s) -> {cfg.out_dir}')
        else:
            print(f'{run}: no overlay ({reason})', file=sys.stderr)
    return 0 if wrote else 1


if __name__ == '__main__':
    sys.exit(main())
