"""Bag replay helpers shared by the wall-tracker tools (R6): profiler and parity.

lidar_front_wall_node is driven in-process through tools/corridor_wall_replay.py
(its Src, recorded_cycles, WallReplay and MaskReplay), on the isolated domain
scene_labels_replay sets up on import (no discovery server, localhost only). A
bag is opened read-only. Nothing is published to a graph.

CYCLES. 'car' replays the car's own tracker steps (the recorded
/perception/virtual_wall/debug payloads, corridor_wall_replay.recorded_cycles).
'every' makes EVERY recorded /scan a tracker step (the 2292-scan replay of
corridor10m_20261015T171753: four times the steps the car ran, so four times
the tracker updates per second of bag).

PASSES. 'rec' feeds the recorded /perception/stereo_columns and
/perception/wall_points (the camera half as the car computed it). 'full'
recomputes the camera half from the bag's depth frames (MaskReplay), which is
the only way a bag without a recorded class mask + depth pair exercises the
rectangle verification.
"""

import os
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[2]
WS = TOOLS.parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
os.environ.setdefault('SCENE_REPLAY_DOMAIN_ID', '96')

import corridor_wall_replay as cwr  # noqa: E402  (isolation happens on this import)

#: What the LiDAR node is fed (corridor_wall_replay.LFW_FEED), the scans, and
#: the recorded debug payloads that say which scans were tracker steps.
LIDAR_TOPICS = ['/tf_static', '/scan', '/odometry/filtered', '/costmap/front_clearance',
                '/camera/detections_3d', '/perception/stereo_wall',
                '/perception/stereo_columns', '/perception/wall_points',
                cwr.REC_CLASSES_TOPIC, cwr.REC_DEPTH_MM_TOPIC,
                '/perception/virtual_wall/debug']

#: The campaign bags (2026-10-21/22, real car runs with stereo + depth).
CAMPAIGN = Path.home() / 'f1tenth_campaigns/first_test_campaign/raw/f1tenth_archive/complete'

#: The named bags of the R6 runs.
BAGS = {
    'c1015': Path.home() / 'bags/yolo_gpu/corridor10m_20261015T171753',
    'c1016': Path.home() / 'bags/corridor10m_20261016T144537',
    'c1019': Path.home() / 'bags/corridor10m_20261019T131815',
    'obst1019': Path.home() / 'bags/corridor_obstacles_20261019T135307',
    'potted1435': CAMPAIGN / '2026-10-22T14-35-30_mission-straight_until_potted_plant',
    'person1455': CAMPAIGN / '2026-10-22T14-55-23_mission-person_then_potted_plant',
    'goto1550': CAMPAIGN / '2026-10-22T15-50-16_mission-go_to_person_1m',
    'mission1021': CAMPAIGN / '2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission',
}


def rclpy_init():
    """rclpy.init() once (the nodes are constructed in-process)."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()


def bag_path(name_or_path):
    """A BAGS key or a path."""
    return Path(BAGS.get(str(name_or_path), name_or_path)).expanduser()


def load(path, full=False):
    """corridor_wall_replay.Src of a bag, LiDAR topics only unless `full`."""
    topics = cwr.TOPICS if full else LIDAR_TOPICS
    return cwr.Src(bag_path(path), topics=topics)


def cycles_of(src, mode='car'):
    """[(cycle dict)] for `mode` 'car' (recorded tracker steps) or 'every' (every scan)."""
    if mode == 'car':
        cycles, _info = cwr.recorded_cycles(src, 'car')
        return [c for c in cycles if c['scan'] is not None]
    if mode == 'every':
        return [{'stamp': t, 'scan': m, 'rec_dbg': None} for t, _r, m in src.get('/scan')]
    raise ValueError(f'cycle mode must be car or every, got {mode!r}')


def mask_rows(src):
    """MaskReplay's rows (the recomputed camera half), [] without depth in the bag."""
    depth_topic = src.pick(cwr.DEPTH_TOPICS)
    if not depth_topic:
        return []
    mr = cwr.MaskReplay(src, depth_topic)
    rows = mr.run()
    mr.close()
    return rows


class Replay(cwr.WallReplay):
    """WallReplay driven one cycle at a time (two of them can run in lockstep).

    `exact_odom` replaces WallReplay._feed_odom, which feeds at least one
    odometry sample per cycle: right for the car's sparse tracker steps
    (~10 Hz against 20 Hz odometry), but at every scan (30-40 Hz) it runs the
    odometry ahead of the scans until no pose is left at the scan stamp (1560
    of the 2292 cycles of corridor10m_20261015T171753). The exact feed stops
    at the first sample after the scan stamp and never feeds past it twice.
    """

    def __init__(self, src, stereo='rec', replayed=None, overrides=None, exact_odom=False):
        super().__init__(src, stereo=stereo, replayed=replayed, overrides=overrides)
        self.exact_odom = bool(exact_odom)
        self._last_odom_stamp = None

    def _feed_odom(self, t):
        if not self.exact_odom:
            return super()._feed_odom(t)
        while self.oi < len(self.odom) and (self._last_odom_stamp is None
                                            or self._last_odom_stamp <= t):
            stamp, msg = self.odom[self.oi]
            self.node._odom_cb(msg)
            self._last_odom_stamp = stamp
            self.oi += 1
        return None

    def step(self, cycle):
        """Feed everything up to the cycle's stamp, run its scan; (ms, captures)."""
        self.clock[0] = cycle['stamp']
        self._feed_until(cycle['stamp'])
        self._feed_odom(cycle['stamp'])
        t0 = time.perf_counter()
        self.node._scan_cb(cycle['scan'])
        ms = (time.perf_counter() - t0) * 1e3
        caps = {name: cap.pop() for name, cap in self.caps.items()}
        return ms, caps


def percentile(values, q):
    """numpy's default (linear) percentile of the finite values, NaN when none."""
    import math

    import numpy as np
    v = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    return float(np.percentile(v, q)) if v else math.nan
