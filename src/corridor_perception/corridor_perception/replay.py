"""Bag replay: recorded /scan and /odom from a rosbag2 sqlite3 recording.

This module replaced a simulator. Nothing here generates a scan, a pose or a
noise sample; every number comes off the vehicle. If a test seems to need a
scan that no recording contains, the answer is a new recording, not a
synthetic one.

Three frames, kept apart on purpose:

  * odom   the odometry frame the recording was made in. Poses come straight
           from /odom and are NOT re-zeroed: a bag is a fragment of a longer
           session and pretending it starts at the identity would invent a
           datum. Consumers that want a run-relative origin compose with
           frames[0].odom.inverse() themselves, and can see that they did.
  * base   the vehicle, base_link.
  * laser  the sensor. sensor_in_base is read FROM THE BAG's /tf_static, not
           from the URDF -- see SENSOR_IN_BASE_NOTE below, which is a live
           contradiction in this workspace, not a stylistic choice.

Invalid returns are handled in exactly one place, scan.clean_ranges(), which
every frame's ranges pass through on the way out of here. See that module for
why `isfinite` is not a no-return test on this sensor. Frames therefore carry
CleanRanges, which is the only thing extract_segments() will accept.
"""

from __future__ import annotations

import math
import pathlib
import sqlite3
import time
from dataclasses import dataclass
from typing import Iterator

import numpy as np

from .geometry import Pose2D, wrap_pi
from .scan import NO_RETURN_MIN_M, clean_ranges

# The URDF (f1tenth_description/urdf/sensors.xacro) mounts the laser at
# x = -0.12, yaw = pi -- rear-facing. EVERY recorded bag from 2026-09-11 to
# 2026-09-17 publishes base_link -> laser as x = +0.12, z = 0.20, yaw = 0 --
# forward-facing. They cannot both be right, and the recordings settle it: on
# 2026-09-11T16-11-08_mission-drive_stop_2m_from_wall the beam at laser angle 0
# closes from 20.01 m to 1.67 m while the vehicle drives forward. The 0 beam
# looks FORWARD, so the tf is right and the URDF is stale. Taking the URDF
# value would rotate every scan by pi and displace it by 0.24 m.
#
# Hence: read the transform from the bag, fall back to this constant, and never
# read the URDF here. Reported by BagInfo so a consumer can check what it got.
SENSOR_IN_BASE_NOTE = 'from bag /tf_static; URDF sensors.xacro disagrees (see module docstring)'
DEFAULT_SENSOR_IN_BASE = Pose2D(0.12, 0.0, 0.0)



@dataclass(frozen=True)
class BagFrame:
    """One scan, with the odometry interpolated to its stamp."""

    index: int
    stamp: float                 # seconds, from the scan header
    ranges: np.ndarray           # invalid returns are inf
    angles: np.ndarray           # sensor frame
    odom: Pose2D                 # base in odom, interpolated to `stamp`
    odom_step: Pose2D            # motion since the previous frame, in the PREVIOUS base frame
    dt: float                    # seconds since the previous frame
    sensor_in_base: Pose2D

    @property
    def odom_sensor(self) -> Pose2D:
        """Laser in odom: the transform to apply to sensor-frame lines."""
        return self.odom.compose(self.sensor_in_base)

    @property
    def ds(self) -> float:
        """Arc length of odom_step, metres."""
        return float(math.hypot(self.odom_step.x, self.odom_step.y))

    @property
    def dtheta(self) -> float:
        """Heading change over odom_step, radians."""
        return float(self.odom_step.theta)


@dataclass(frozen=True)
class BagInfo:
    """What a recording turned out to contain. Evidence, not configuration."""

    path: str
    n_scan: int
    n_odom: int
    n_frames: int                # scans that fell inside the odometry coverage
    n_dropped: int               # scans outside it, hence unusable
    duration_s: float
    sensor_in_base: Pose2D
    sensor_in_base_source: str
    header_range_max: float
    observed_max_range: float    # largest real return, sentinel excluded
    no_return_fraction: float
    path_length_m: float
    yaw_range_deg: float


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _header_stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def resolve_bag(path) -> pathlib.Path:
    """Accept an archive run dir, a bag dir, or the .db3 itself."""
    p = pathlib.Path(path)
    if p.is_file():
        return p
    for candidate in (p / 'bag', p):
        if candidate.is_dir():
            dbs = sorted(candidate.glob('*.db3'))
            if dbs:
                return dbs[0]
    raise FileNotFoundError(f'no rosbag2 .db3 under {path}')


def _connect(db: pathlib.Path) -> sqlite3.Connection:
    """Read-only, by URI. A plain connect() to a mistyped name CREATES an empty
    database inside the archive, which is how you lose a recording."""
    return sqlite3.connect(f'file:{db}?mode=ro', uri=True)


def _interpolate(odom: list[tuple[float, float, float, float]], stamp: float):
    """Pose at `stamp` from (t, x, y, yaw) samples, or None if out of coverage.

    /scan and /odom run at different rates (in these bags 40 Hz and ~10 Hz), so
    a scan almost never lands on an odometry sample. Linear in position, and on
    the wrapped difference in yaw so a sample pair straddling +-pi does not
    interpolate the long way round.
    """
    ts = [o[0] for o in odom]
    i = np.searchsorted(ts, stamp)
    if i == 0 or i >= len(odom):
        return None
    t0, x0, y0, th0 = odom[i - 1]
    t1, x1, y1, th1 = odom[i]
    if t1 <= t0:
        return None
    u = (stamp - t0) / (t1 - t0)
    return Pose2D(x0 + u * (x1 - x0), y0 + u * (y1 - y0),
                  wrap_pi(th0 + u * wrap_pi(th1 - th0)))


class BagReplay:
    """Iterate BagFrames from a recording.

    rate=None plays as fast as the consumer can take them; rate=1.0 plays in
    real time, rate=2.0 at double speed.
    """

    def __init__(self, path, *, scan_topic: str = '/scan', odom_topic: str = '/odom',
                 sensor_in_base: Pose2D | None = None, max_range: float | None = None,
                 rate: float | None = None):
        self.db = resolve_bag(path)
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.rate = rate
        self._override_sensor = sensor_in_base
        self._max_range = max_range
        self._load()

    # -- loading ----------------------------------------------------------
    def _load(self) -> None:
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        con = _connect(self.db)
        try:
            topics = {n: (i, t) for i, n, t in
                      con.execute('select id, name, type from topics').fetchall()}
            for required in (self.scan_topic, self.odom_topic):
                if required not in topics:
                    raise KeyError(
                        f'{self.db} has no {required}; it carries '
                        f'{sorted(topics)[:8]}...')

            self._scans = []
            for _, blob in con.execute(
                    'select timestamp, data from messages where topic_id=? order by timestamp',
                    (topics[self.scan_topic][0],)):
                self._scans.append(deserialize_message(bytes(blob), LaserScan))

            self._odom = []
            for _, blob in con.execute(
                    'select timestamp, data from messages where topic_id=? order by timestamp',
                    (topics[self.odom_topic][0],)):
                m = deserialize_message(bytes(blob), Odometry)
                p, q = m.pose.pose.position, m.pose.pose.orientation
                self._odom.append((_header_stamp(m), p.x, p.y,
                                   _yaw_from_quat(q.x, q.y, q.z, q.w)))

            self._sensor_in_base = self._override_sensor
            self._sensor_source = 'caller override'
            if self._sensor_in_base is None and '/tf_static' in topics:
                for (blob,) in con.execute(
                        'select data from messages where topic_id=?',
                        (topics['/tf_static'][0],)):
                    for tf in deserialize_message(bytes(blob), TFMessage).transforms:
                        if tf.child_frame_id == 'laser' and tf.header.frame_id == 'base_link':
                            tr, q = tf.transform.translation, tf.transform.rotation
                            self._sensor_in_base = Pose2D(
                                tr.x, tr.y, _yaw_from_quat(q.x, q.y, q.z, q.w))
                            self._sensor_source = SENSOR_IN_BASE_NOTE
            if self._sensor_in_base is None:
                self._sensor_in_base = DEFAULT_SENSOR_IN_BASE
                self._sensor_source = 'DEFAULT_SENSOR_IN_BASE (no /tf_static in bag)'
        finally:
            con.close()

        if not self._scans or not self._odom:
            raise ValueError(f'{self.db}: {len(self._scans)} scans, {len(self._odom)} odom')
        self._odom.sort(key=lambda o: o[0])

    # -- iteration --------------------------------------------------------
    def _clean(self, scan) -> tuple[np.ndarray, np.ndarray]:
        """Ranges with invalid returns as inf, and the matching bearings."""
        r = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + scan.angle_increment * np.arange(r.size)
        return clean_ranges(r, float(scan.range_min),
                            self._max_range if self._max_range is not None
                            else float(scan.range_max)), angles

    def __iter__(self) -> Iterator[BagFrame]:
        self._dropped = 0
        prev_pose, prev_stamp, index = None, None, 0
        wall_t0, bag_t0 = time.monotonic(), None
        for scan in self._scans:
            stamp = _header_stamp(scan)
            pose = _interpolate(self._odom, stamp)
            if pose is None:
                self._dropped += 1
                continue
            if prev_pose is None:
                step, dt = Pose2D(), 0.0
            else:
                step = prev_pose.inverse().compose(pose)
                dt = stamp - prev_stamp
            if self.rate:
                if bag_t0 is None:
                    bag_t0 = stamp
                behind = (stamp - bag_t0) / self.rate - (time.monotonic() - wall_t0)
                if behind > 0:
                    time.sleep(behind)
            ranges, angles = self._clean(scan)
            yield BagFrame(index=index, stamp=stamp, ranges=ranges, angles=angles,
                           odom=pose, odom_step=step, dt=dt,
                           sensor_in_base=self._sensor_in_base)
            prev_pose, prev_stamp, index = pose, stamp, index + 1

    # -- description ------------------------------------------------------
    def info(self) -> BagInfo:
        """Play the bag once and report what it actually contains."""
        frames = list(self)
        observed, no_return, total = 0.0, 0, 0
        for f in frames:
            finite = f.ranges[np.isfinite(f.ranges)]
            no_return += int(f.ranges.size - finite.size)
            total += int(f.ranges.size)
            if finite.size:
                observed = max(observed, float(finite.max()))
        path_len = sum(f.ds for f in frames)
        yaws = np.unwrap([f.odom.theta for f in frames]) if frames else np.zeros(1)
        return BagInfo(
            path=str(self.db), n_scan=len(self._scans), n_odom=len(self._odom),
            n_frames=len(frames), n_dropped=self._dropped,
            duration_s=(frames[-1].stamp - frames[0].stamp) if frames else 0.0,
            sensor_in_base=self._sensor_in_base,
            sensor_in_base_source=self._sensor_source,
            header_range_max=float(self._scans[0].range_max),
            observed_max_range=observed,
            no_return_fraction=(no_return / total) if total else 0.0,
            path_length_m=float(path_len),
            yaw_range_deg=float(math.degrees(yaws.max() - yaws.min())),
        )
