#!/usr/bin/env python3
"""Live test: the Python and the Rust costmap_boundary_node side by side.

Both real nodes run at once on an isolated localhost ROS domain, on the SAME
inputs (/parity/map transient-local, /parity/pose best-effort), each on its
own output topics. Phases:
  0  no input            -> both publish empty arrays, no front_clearance
  1  map A, static pose  -> identical constraints/clearance; frame_id
                            base_link; stamps = node clock now; ~10 Hz
  2  map B mid-run       -> both switch to the new map, identical again
  3  pose outside map B  -> identical (permissive: nothing found, 6.0 m)
  4  CPU: a recorded car map and its recorded pose stream at 50 Hz for
     --cpu-s seconds -> process CPU %, tick p50/p95/max (each node's own
     timing log: timing_report_period_s / timed_py_node.py)

Usage (workspace sourced): live_costmap_test.py [--bag DIR] [--out FILE]
"""

import argparse
import json
import math
import os
import random
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BAG = os.path.expanduser(
    '~/f1tenth_archive/mission_row/2026-10-08T12-29-44_mission-llm_2b356aac445f/bag')


def cpu_seconds(pid):
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def recorded_map_and_poses(bag):
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import deserialize_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/slam/map', '/ekf_global/odometry/filtered']))
    grid, poses = None, []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/slam/map':
            grid = deserialize_message(data, OccupancyGrid)
        else:
            poses.append(deserialize_message(data, Odometry))
    return grid, poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--cpu-s', type=float, default=30.0)
    ap.add_argument('--out', default='output/rust/live_costmap.json')
    a = ap.parse_args()

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(150, 220))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for k in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT'):
        env.pop(k, None)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    os.environ.pop('ROS_DISCOVERY_SERVER', None)

    from ament_index_python.packages import get_package_prefix
    from f1tenth_description.vehicle_frames import (
        base_front_x, front_side_limit_m, MIN_FRONT_DISTANCE_M)
    rs_exe = os.path.join(get_package_prefix('f1tenth_costmap_boundary_rs'), 'lib',
                          'f1tenth_costmap_boundary_rs', 'costmap_boundary_node')
    logs = tempfile.mkdtemp(prefix='live_costmap_')

    def topics(tag):
        return ['-p', 'map_topic:=/parity/map', '-p', 'pose_topic:=/parity/pose',
                '-p', f'output_topic:=/parity/{tag}/boundaries',
                '-p', f'front_clearance_topic:=/parity/{tag}/front_clearance']

    py_log = open(os.path.join(logs, 'py.log'), 'w')
    rs_log = open(os.path.join(logs, 'rs.log'), 'w')
    procs = {
        'py': subprocess.Popen(
            [sys.executable, os.path.join(HERE, 'timed_py_node.py'), '--report-s', '5',
             '--ros-args', '-r', '__node:=costmap_boundary_py'] + topics('py'),
            env=env, stdout=py_log, stderr=subprocess.STDOUT),
        'rs': subprocess.Popen(
            [rs_exe, '--ros-args', '-r', '__node:=costmap_boundary_rs'] + topics('rs')
            + ['-p', f'base_front_x_m:={base_front_x()}',
               '-p', f'front_side_limit_m:={front_side_limit_m()}',
               '-p', f'min_front_distance_m:={MIN_FRONT_DISTANCE_M}',
               '-p', 'timing_report_period_s:=5.0'],
            env=env, stdout=rs_log, stderr=subprocess.STDOUT),
    }

    import rclpy
    from f1tenth_messages.msg import BoundaryConstraintArray
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32

    rclpy.init()
    node = rclpy.create_node('costmap_driver')
    rx = {f'{t}_{k}': [] for t in ('py', 'rs') for k in ('b', 'c')}
    lock = threading.Lock()

    def rec(key):
        def cb(msg):
            with lock:
                rx[key].append((time.time_ns(), msg))
        return cb

    for tag in ('py', 'rs'):
        node.create_subscription(BoundaryConstraintArray, f'/parity/{tag}/boundaries',
                                 rec(f'{tag}_b'), 50)
        node.create_subscription(Float32, f'/parity/{tag}/front_clearance', rec(f'{tag}_c'), 50)
    map_pub = node.create_publisher(OccupancyGrid, '/parity/map', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    pose_pub = node.create_publisher(Odometry, '/parity/pose', QoSProfile(
        depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + 20
    while time.time() < deadline and (map_pub.get_subscription_count() < 2
                                      or pose_pub.get_subscription_count() < 2):
        time.sleep(0.1)

    def window(seconds):
        with lock:
            for v in rx.values():
                v.clear()
        t0 = time.time_ns()
        time.sleep(seconds)
        with lock:
            snap = {k: list(v) for k, v in rx.items()}
        return t0, snap

    def content(snap, tag, skip_s=0.3, t0=0):
        keep = [(t, m) for t, m in snap[f'{tag}_b'] if t > t0 + skip_s * 1e9]
        cons = {tuple((round(c.normal[0], 15), round(c.normal[1], 15), round(c.offset, 15))
                      for c in m.constraints) for _, m in keep}
        clr = {round(float(m.data), 7) for t, m in snap[f'{tag}_c'] if t > t0 + skip_s * 1e9}
        frames = {m.header.frame_id for _, m in keep}
        lag = [abs(t - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e6
               for t, m in keep]
        return dict(constraints=sorted(cons), clearance=sorted(clr), frames=sorted(frames),
                    n=len(keep), max_stamp_lag_ms=max(lag) if lag else None)

    def grid(width, height, res, ox, oy, walls):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.width, m.info.height, m.info.resolution = width, height, res
        m.info.origin.position.x, m.info.origin.position.y = ox, oy
        m.info.origin.orientation.w = 1.0
        d = np.zeros((height, width), dtype=np.int8)
        for (r0, r1, c0, c1) in walls:
            d[r0:r1, c0:c1] = 100
        m.data = d.ravel().tolist()
        return m

    def odom(x, y, yaw):
        o = Odometry()
        o.header.frame_id = 'map'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return o

    stop_pose = threading.Event()
    current_pose = [None]

    def pose_loop():
        while not stop_pose.is_set():
            if current_pose[0] is not None:
                pose_pub.publish(current_pose[0])
            time.sleep(0.02)
    threading.Thread(target=pose_loop, daemon=True).start()

    res = dict(domain=env['ROS_DOMAIN_ID'])
    t0, snap = window(1.5)
    res['0_no_input'] = {t: dict(arrays=len(snap[f'{t}_b']),
                                 nonempty=sum(1 for _, m in snap[f'{t}_b'] if m.constraints),
                                 clearances=len(snap[f'{t}_c'])) for t in ('py', 'rs')}

    map_a = grid(160, 160, 0.05, -4.0, -4.0, [(0, 3, 0, 160), (157, 160, 0, 160),
                                              (0, 160, 0, 3), (0, 160, 157, 160),
                                              (100, 104, 110, 114)])
    map_pub.publish(map_a)
    current_pose[0] = odom(0.3, 0.2, 0.4)
    t0, snap = window(3.0)
    res['1_map_a'] = {t: content(snap, t, t0=t0) for t in ('py', 'rs')}
    res['1_map_a']['rate_hz'] = {t: len(snap[f'{t}_b']) / 3.0 for t in ('py', 'rs')}
    res['1_map_a']['identical'] = (res['1_map_a']['py']['constraints']
                                   == res['1_map_a']['rs']['constraints']
                                   and res['1_map_a']['py']['clearance']
                                   == res['1_map_a']['rs']['clearance'])

    map_b = grid(220, 120, 0.1, -10.0, -6.0, [(0, 2, 0, 220), (118, 120, 0, 220),
                                              (40, 80, 120, 122)])
    map_pub.publish(map_b)
    t0, snap = window(3.0)
    res['2_map_b'] = {t: content(snap, t, t0=t0) for t in ('py', 'rs')}
    res['2_map_b']['identical'] = (res['2_map_b']['py']['constraints']
                                   == res['2_map_b']['rs']['constraints']
                                   and res['2_map_b']['py']['clearance']
                                   == res['2_map_b']['rs']['clearance'])
    res['2_map_b']['changed_from_a'] = (res['2_map_b']['rs']['constraints']
                                        != res['1_map_a']['rs']['constraints'])

    current_pose[0] = odom(40.0, 30.0, 1.0)
    t0, snap = window(2.0)
    res['3_outside'] = {t: content(snap, t, t0=t0) for t in ('py', 'rs')}
    res['3_outside']['identical'] = (res['3_outside']['py']['constraints']
                                     == res['3_outside']['rs']['constraints']
                                     and res['3_outside']['py']['clearance']
                                     == res['3_outside']['rs']['clearance'])

    # ---- 4: CPU on a recorded map + pose stream
    grid_rec, poses = recorded_map_and_poses(a.bag)
    current_pose[0] = None
    map_pub.publish(grid_rec)
    time.sleep(1.0)
    stop_cpu = threading.Event()

    def replay_poses():
        i = 0
        while not stop_cpu.is_set():
            o = poses[i % len(poses)]
            pose_pub.publish(o)
            i += 1
            time.sleep(0.02)
    th = threading.Thread(target=replay_poses, daemon=True)
    th.start()
    time.sleep(1.0)
    cpu0 = {k: cpu_seconds(p.pid) for k, p in procs.items()}
    w0 = time.monotonic()
    e0 = time.time()
    with lock:
        for v in rx.values():
            v.clear()
    time.sleep(a.cpu_s)
    cpu1 = {k: cpu_seconds(p.pid) for k, p in procs.items()}
    wall = time.monotonic() - w0
    e1 = time.time()
    with lock:
        n_ticks = {t: len(rx[f'{t}_b']) for t in ('py', 'rs')}
    stop_cpu.set()
    stop_pose.set()
    res['4_cpu'] = dict(
        map=dict(width=grid_rec.info.width, height=grid_rec.info.height,
                 resolution=grid_rec.info.resolution), poses=len(poses), seconds=wall,
        cpu_percent={k: 100.0 * (cpu1[k] - cpu0[k]) / wall for k in procs},
        ticks=n_ticks)

    for p in procs.values():
        p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    py_log.close()
    rs_log.close()
    # Tick statistics: each node's own 5 s reports that fall inside phase 4.
    pat = re.compile(r'\[(\d+\.\d+)\].*tick timing over (\d+) ticks: p50 ([\d.]+) us, '
                     r'p95 ([\d.]+) us, max ([\d.]+) us')
    for tag in ('py', 'rs'):
        reps = []
        for line in open(os.path.join(logs, f'{tag}.log')):
            m = pat.search(line)
            if m and e0 + 4.0 < float(m.group(1)) <= e1 + 0.5:
                reps.append(dict(n=int(m.group(2)), p50_us=float(m.group(3)),
                                 p95_us=float(m.group(4)), max_us=float(m.group(5))))
        res['4_cpu'].setdefault('tick_reports', {})[tag] = reps
    node.destroy_node()
    rclpy.try_shutdown()
    res['logs'] = logs
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
