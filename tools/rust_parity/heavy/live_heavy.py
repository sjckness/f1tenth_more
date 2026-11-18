#!/usr/bin/env python3
"""Census of the heavy topics on a running stack: who subscribes, rate, size, CPU.

Run as a Discovery Server super client next to the running stack
(scripts/sim/sim_isaac.sh run -- python3 <this> --out X.json). For every topic
whose type is Image, CompressedImage, CameraInfo, PointCloud2, LaserScan,
StereoColumns or OccupancyGrid:
  - publishers and subscribers (node names) from the graph,
  - rate and serialized size over --seconds, from RAW subscriptions (the
    serialized bytes: nothing is deserialized, so the census costs little),
and, over the same window, the CPU % (100 % = one core) and RSS of every ROS
node process (psutil; node name from __node:= or the executable), with its
language (python / native).
"""

import argparse
import json
import os
import sys
import time

import psutil
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

HEAVY = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CameraInfo',
         'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/LaserScan',
         'f1tenth_messages/msg/StereoColumns', 'nav_msgs/msg/OccupancyGrid')


def ros_processes():
    out = {}
    for p in psutil.process_iter(['pid', 'cmdline', 'name']):
        cmd = p.info['cmdline'] or []
        line = ' '.join(cmd)
        if '--ros-args' not in line and 'isaac' not in line:
            continue
        name = None
        for i, a in enumerate(cmd):
            if a.startswith('__node:='):
                name = a.split(':=', 1)[1]
            elif a == '-r' and i + 1 < len(cmd) and cmd[i + 1].startswith('__node:='):
                name = cmd[i + 1].split(':=', 1)[1]
        if name is None:
            exe = next((c for c in cmd if '/lib/' in c), cmd[0] if cmd else p.info['name'])
            name = os.path.basename(exe)
        lang = 'python' if any(os.path.basename(c).startswith('python') for c in cmd[:1]) else 'native'
        out[p.info['pid']] = dict(name=name, lang=lang, proc=p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('heavy_census')
    time.sleep(3.0)   # discovery as a super client
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = {}
    for name, types in node.get_topic_names_and_types():
        t = next((x for x in types if x in HEAVY), None)
        if t is None:
            continue
        subs = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                for i in node.get_subscriptions_info_by_topic(name)
                if i.node_name not in ('heavy_census', '_NODE_NAME_UNKNOWN_')]
        pubs = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                for i in node.get_publishers_info_by_topic(name)]
        topics[name] = dict(type=t, publishers=sorted(set(pubs)), subscribers=sorted(set(subs)),
                            msgs=0, bytes=0)

    def cb(topic):
        def f(raw):
            topics[topic]['msgs'] += 1
            topics[topic]['bytes'] += len(raw)
        return f
    for name, info in topics.items():
        node.create_subscription(get_message(info['type']), name, cb(name),
                                 qos_profile_sensor_data, raw=True)
    procs = ros_processes()
    for v in procs.values():
        try:
            v['cpu0'] = v['proc'].cpu_times()
        except psutil.Error:
            v['cpu0'] = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.monotonic() - t0
    cpu = {}
    for pid, v in procs.items():
        try:
            c1 = v['proc'].cpu_times()
            rss = v['proc'].memory_info().rss
        except psutil.Error:
            continue
        if v['cpu0'] is None:
            continue
        used = (c1.user + c1.system) - (v['cpu0'].user + v['cpu0'].system)
        key = v['name']
        while key in cpu:
            key += '+'
        cpu[key] = dict(pid=pid, lang=v['lang'], cpu_percent=round(100.0 * used / wall, 2),
                        rss_mb=round(rss / 2**20, 1))
    for info in topics.values():
        info['hz'] = round(info['msgs'] / wall, 2)
        info['avg_bytes'] = int(info['bytes'] / info['msgs']) if info['msgs'] else 0
        info['MB_per_s'] = round(info['bytes'] / wall / 2**20, 3)
        del info['bytes']
    res = dict(seconds=round(wall, 2), topics=topics,
               processes=dict(sorted(cpu.items(), key=lambda kv: -kv[1]['cpu_percent'])))
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: (v['hz'], v['avg_bytes'], len(v['subscribers']))
                      for k, v in topics.items()}))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
