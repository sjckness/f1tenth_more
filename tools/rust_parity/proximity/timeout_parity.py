#!/usr/bin/env python3
"""Timeout parity: what the guard and the BT's front check do when /scan stops.

Runs on an isolated localhost ROS domain:
  rs_car  proximity_guard_node mode:=active, use_sim_time false (the car)
  rs_sim  proximity_guard_node mode:=active, use_sim_time true; /clock comes from this driver
  py      the real IsProximityTooClose in a host node, with BOTH of the BT's stop paths:
          - event: the cpu-cleanup stop from its /scan callback (immediate_stop_topic)
          - tick:  a 10 Hz timer that calls update() and publishes the emergency lane's Stop
                   when it returns SUCCESS (behavior_executor_node: emergency Sequence ->
                   Stop(safety_stop) every tick while the condition succeeds)
Each case: 1 s of scans (the last ones CLEAR or TRIPPING), a gap of G ms without scans, 1 s of
clear scans. /clock either keeps running through the gap (scans lost) or freezes (a simulator
pause: ROS time does not advance). Counted per case and stop source: stops during the gap and
in the 300 ms after scans resume, and the delay of the first gap stop after the last scan.

Usage (workspace sourced): timeout_parity.py [--recorded-ms 260] [--out FILE]
"""

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time

import numpy as np

FRONT_X = 0.405
N = 1081
ANGLE_MIN = -2.356194496154785
INC = 0.004363323096185923


def py_host():
    import py_trees
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
    from f1tenth_behavior.behaviours.stop import make_stop_msg
    rclpy.init()
    node = rclpy.create_node('py_timeout_host')
    b = IsProximityTooClose(scan_topic='/parity/scan', base_front_x_m=FRONT_X,
                            immediate_stop_topic='/parity/py_event_stop',
                            immediate_stop_frame_id='base_link/emergency')
    b.setup(node=node)
    tick_pub = node.create_publisher(AckermannDriveStamped, '/parity/py_tick_stop', 10)

    def tick():
        if b.update() == py_trees.common.Status.SUCCESS:
            tick_pub.publish(make_stop_msg(node, 'base_link/emergency'))
    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--py-host', action='store_true')
    ap.add_argument('--recorded-ms', type=float, default=260.0,
                    help='the Isaac pause to replay (run A impl:=py: 232-282 ms)')
    ap.add_argument('--out', default='output/rust/timeout_parity.json')
    a = ap.parse_args()
    if a.py_host:
        py_host()
        return 0

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(150, 220))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for k in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT'):
        env.pop(k, None)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    os.environ.pop('ROS_DISCOVERY_SERVER', None)

    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('f1tenth_proximity_guard'), 'lib',
                       'f1tenth_proximity_guard', 'proximity_guard_node')

    def guard(tag, sim):
        return subprocess.Popen(
            [exe, '--ros-args', '-r', f'__node:=guard_{tag}', '-p', 'mode:=active',
             '-p', f'base_front_x_m:={FRONT_X}', '-p', 'scan_topic:=/parity/scan',
             '-p', f'stop_topic:=/parity/{tag}_stop',
             '-p', f'decision_topic:=/parity/{tag}_decision',
             '-p', f'use_sim_time:={"true" if sim else "false"}'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    procs = {'rs_car': guard('rs_car', False), 'rs_sim': guard('rs_sim', True),
             'py': subprocess.Popen([sys.executable, __file__, '--py-host'], env=env,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)}

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('timeout_driver')
    sources = ('rs_car_stop', 'rs_sim_stop', 'py_event_stop', 'py_tick_stop')
    rx = {k: [] for k in sources}
    lock = threading.Lock()

    def rec(k):
        def cb(_):
            with lock:
                rx[k].append(time.monotonic())
        return cb
    for k in sources:
        node.create_subscription(AckermannDriveStamped, f'/parity/{k}', rec(k), 100)
    scan_pub = node.create_publisher(LaserScan, '/parity/scan', 10)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    tf_pub = node.create_publisher(TFMessage, '/tf_static', QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # /clock at 200 Hz, advancing with wall time unless frozen.
    sim = {'ns': 1_000_000_000_000, 'frozen': False, 'last': time.monotonic(), 'run': True}

    def clock_loop():
        while sim['run']:
            now = time.monotonic()
            if not sim['frozen']:
                sim['ns'] += int((now - sim['last']) * 1e9)
            sim['last'] = now
            c = Clock()
            c.clock.sec, c.clock.nanosec = divmod(sim['ns'], 1_000_000_000)
            clock_pub.publish(c)
            time.sleep(0.005)
    threading.Thread(target=clock_loop, daemon=True).start()

    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
    t.transform.translation.x, t.transform.rotation.w = 0.305, 1.0
    tf_pub.publish(TFMessage(transforms=[t]))

    ang = ANGLE_MIN + np.arange(N) * INC
    clear = [30.0] * N
    trip = list(clear)
    for i in np.flatnonzero(np.abs(ang) < 0.05):
        trip[i] = 0.20

    def publish(ranges):
        m = LaserScan()
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(sim['ns'], 1_000_000_000)
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max, m.angle_increment = ANGLE_MIN, -ANGLE_MIN, INC
        m.range_min, m.range_max, m.ranges = 0.02, 30.0, ranges
        scan_pub.publish(m)
        return time.monotonic()

    def stream(ranges, seconds):
        last = None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            last = publish(ranges)
            time.sleep(0.025)
        return last

    deadline = time.time() + 20
    while time.time() < deadline and scan_pub.get_subscription_count() < 3:
        time.sleep(0.1)
    stream(clear, 3.0)   # discovery, /tf_static, /clock

    cases = []
    gaps = [150.0, 250.0, 500.0, 1000.0, float(a.recorded_ms)]
    for pre in ('clear', 'trip'):
        for clock in ('running', 'frozen'):
            for g in gaps:
                cases.append((pre, clock, g))
    results = []
    for pre, clock, g in cases:
        stream(clear, 1.0)
        last = stream(trip if pre == 'trip' else clear, 1.0)
        with lock:
            for v in rx.values():
                v.clear()
        sim['frozen'] = clock == 'frozen'
        time.sleep(g / 1000.0)
        sim['frozen'] = False
        resume = stream(clear, 1.0)
        first_post = resume - 1.0
        with lock:
            snap = {k: list(v) for k, v in rx.items()}
        row = dict(pre=pre, clock=clock, gap_ms=g)
        for k in sources:
            ts = snap[k]
            in_gap = [x for x in ts if last < x < first_post + 0.005]
            after = [x for x in ts if first_post + 0.005 <= x < first_post + 0.3]
            row[k] = dict(gap_stops=len(in_gap), after_resume_stops=len(after),
                          first_gap_stop_ms=round((in_gap[0] - last) * 1e3, 1) if in_gap else None)
        results.append(row)
        print(json.dumps(row), flush=True)

    sim['run'] = False
    for p in procs.values():
        p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    node.destroy_node()
    rclpy.try_shutdown()
    with open(a.out, 'w') as f:
        json.dump(dict(recorded_ms=a.recorded_ms, results=results), f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
