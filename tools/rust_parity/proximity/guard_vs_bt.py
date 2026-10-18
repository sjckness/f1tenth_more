#!/usr/bin/env python3
"""Live: the Rust guard's decisions next to the BT's, on a running stack.

Run as a Discovery Server super client while the stack runs with guard:=shadow
(or active), e.g.  scripts/sim/sim_isaac.sh run -- python3 <this> --out X.json
Records until SIGINT/SIGTERM or --seconds:
  /scan                     -> the BT's FRONT decision on that scan, computed
                               with the real IsProximityTooClose and the real
                               /tf_static (the same object the emergency lane ticks)
  /proximity_guard/decision -> the guard's decision for the same scan (matched
                               by scan stamp), its reason, latency, compute time
  /behavior/tree_status     -> emergency-lane episodes, and which condition
  /safety_stop              -> stop commands by source (frame_id)
Writes one JSON summary: decision agreement (front, scan by scan), guard
fail-safe reasons, BT emergency episodes, stop sources, latency.
"""

import argparse
import json
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus
from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
from f1tenth_messages.msg import BehaviorTreeStatus
from f1tenth_params.param_defaults import get_value
from sensor_msgs.msg import LaserScan


def _stamp_ns(s):
    return s.sec * 1_000_000_000 + s.nanosec


class Recorder(Node):

    def __init__(self):
        super().__init__('guard_vs_bt')
        self.py = IsProximityTooClose(
            front_distance_threshold=float(get_value('proximity_front_threshold_m')),
            lidar_distance_threshold=float(get_value('proximity_side_threshold_m')))
        self.py.node = self
        self.py.tf_static_sub = self.py._frames.subscribe(self)
        self.py_front = {}            # scan stamp ns -> (tripped, min)
        self.guard = {}               # scan stamp ns -> decision dict
        self.guard_nodata = []        # (wall s, reason) for stale/no_scan_yet
        self.lanes = []               # (wall s, active_lane, emergency_trip, stop_source)
        self.stops = {}               # frame_id -> count
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, '/proximity_guard/decision', self._dec, 50)
        self.create_subscription(BehaviorTreeStatus, '/behavior/tree_status', self._tree, 50)
        self.create_subscription(AckermannDriveStamped, '/safety_stop', self._stop, 50)

    def _scan(self, msg):
        self.py.latest_lidar_front_min = None
        self.py._scan_callback(msg)
        m = self.py.latest_lidar_front_min
        self.py_front[_stamp_ns(msg.header.stamp)] = (
            m is not None and m < self.py.front_distance_threshold, m)

    def _dec(self, msg):
        v = {kv.key: kv.value for kv in msg.values}
        if v.get('scan_stamp_ns', 'none') == 'none':
            self.guard_nodata.append((time.time(), v.get('reason')))
            return
        self.guard[int(v['scan_stamp_ns'])] = v

    def _tree(self, msg):
        self.lanes.append((time.time(), msg.active_lane, msg.emergency_trip, msg.stop_source))

    def _stop(self, msg):
        self.stops[msg.header.frame_id] = self.stops.get(msg.header.frame_id, 0) + 1

    def summary(self):
        both = sorted(set(self.py_front) & set(self.guard))
        agree = sum(1 for s in both if self.py_front[s][0] == (self.guard[s]['tripped'] == 'true'))
        disagree = [dict(stamp=s, py=self.py_front[s], rs=self.guard[s]) for s in both
                    if self.py_front[s][0] != (self.guard[s]['tripped'] == 'true')][:20]
        reasons = {}
        for v in self.guard.values():
            reasons[v['reason']] = reasons.get(v['reason'], 0) + 1
        nodata = {}
        for _, r in self.guard_nodata:
            nodata[r] = nodata.get(r, 0) + 1
        lat = np.array([float(v['latency_ms']) for v in self.guard.values()
                        if v.get('latency_ms', 'none') != 'none'])
        comp = np.array([float(v['compute_us']) for v in self.guard.values()
                         if v.get('compute_us', 'none') != 'none'])
        tf_fb = sum(1 for v in self.guard.values() if v.get('tf_fallback') == 'true')
        episodes, prev = [], None
        for t, lane, trip, src in self.lanes:
            on = lane == 'emergency'
            if on and not prev:
                episodes.append(dict(t=t, trip=trip, source=src))
            prev = on
        pct = (lambda a, q: float(np.percentile(a, q)) if len(a) else None)
        return dict(
            scans_seen=len(self.py_front), guard_scan_decisions=len(self.guard),
            matched=len(both), front_agree=agree, front_disagree=len(both) - agree,
            disagree_examples=disagree,
            py_front_trips=sum(1 for s in both if self.py_front[s][0]),
            rs_trips=sum(1 for s in both if self.guard[s]['tripped'] == 'true'),
            rs_stop_decisions=sum(1 for v in self.guard.values() if v['stop'] == 'true'),
            guard_reasons=reasons, guard_no_data_decisions=nodata, guard_tf_fallback=tf_fb,
            guard_latency_ms=dict(p50=pct(lat, 50), p95=pct(lat, 95),
                                  max=float(lat.max()) if len(lat) else None),
            guard_compute_us=dict(p50=pct(comp, 50), p95=pct(comp, 95),
                                  max=float(comp.max()) if len(comp) else None),
            bt_ticks=len(self.lanes), bt_emergency_episodes=episodes[:50],
            bt_emergency_episode_count=len(episodes), safety_stop_by_frame_id=self.stops)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=0.0, help='0: until SIGINT/SIGTERM')
    ap.add_argument('--stop-file', default=None, help='stop when this file exists')
    a = ap.parse_args()
    rclpy.init()
    node = Recorder()
    stop = {'v': False}

    def handler(*_):
        stop['v'] = True
    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)
    end = time.monotonic() + a.seconds if a.seconds > 0 else None
    import os
    while not stop['v'] and (end is None or time.monotonic() < end):
        rclpy.spin_once(node, timeout_sec=0.1)
        if a.stop_file and os.path.exists(a.stop_file):
            break
    s = node.summary()
    with open(a.out, 'w') as f:
        json.dump(s, f, indent=1, default=str)
    print(json.dumps({k: v for k, v in s.items() if k not in ('disagree_examples',
                                                              'bt_emergency_episodes')}))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
