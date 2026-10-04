#!/usr/bin/env python3
"""Play a bag in real time with every header.stamp set to the wall clock at
publish time -- the way the drivers it stands in for would publish.

Phase 5 runs the full stack the way the car runs it, on wall time (no
use_sim_time). `ros2 bag play` keeps the recorded stamps, which are ~11 days
old, and with those the stack does not behave like it does on the car: in the
first Step 4 run the local EKF put odom->base_link on /tf only 28 times in
12 s, stamped "now", while it fuses /odom at 50 Hz on the car. This player
keeps the bag's message timing and payloads and changes only header.stamp
(and each TransformStamped's header.stamp on /tf_static), repeated --loops
times back to back.

QoS is each topic's recorded offered QoS (reliability, durability), depth 10.
Topics without a header are refused rather than published with stale stamps.

Usage: restamp_play.py BAG [--loops N]
"""
import argparse
import os
import time

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from bag_compat import open_reader


def _offered_qos(bag):
    """{topic: offered QoS profile list} from metadata.yaml (rosbag2_py's QoS
    objects expose no getters on Jazzy). Humble writes the list as a YAML
    string, Jazzy as YAML."""
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    return {t['topic_metadata']['name']: t['topic_metadata'].get('offered_qos_profiles')
            for t in info['topics_with_message_count']}


def _qos(offered):
    profiles = yaml.safe_load(offered) if isinstance(offered, str) else offered
    p = profiles[0] if profiles else {}
    rel = str(p.get('reliability', 'reliable')).lower()
    dur = str(p.get('durability', 'volatile')).lower()
    return QoSProfile(
        depth=10,
        reliability=(ReliabilityPolicy.BEST_EFFORT if rel in ('2', 'best_effort')
                     else ReliabilityPolicy.RELIABLE),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if dur in ('1', 'transient_local')
                    else DurabilityPolicy.VOLATILE))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--loops', type=int, default=1)
    a = ap.parse_args()

    reader = open_reader(a.bag)
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    msgs = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        msgs.append((topic, deserialize_message(data, get_message(topics[topic].type)), t))
    del reader
    for name, meta in topics.items():
        sample = get_message(meta.type)()
        if name != '/tf_static' and not hasattr(sample, 'header'):
            raise SystemExit(f'{name} has no header to re-stamp')

    rclpy.init()
    node = rclpy.create_node('restamp_player')
    offered = _offered_qos(a.bag)
    pubs = {n: node.create_publisher(get_message(m.type), n, _qos(offered.get(n)))
            for n, m in topics.items()}
    time.sleep(1.0)  # let subscribers match before the first message
    t_first = msgs[0][2]
    sent = 0
    for loop in range(a.loops):
        start = time.monotonic()
        for topic, msg, t in msgs:
            if topic == '/tf_static' and loop > 0:
                continue  # latched; once is what a static publisher does
            due = start + (t - t_first) * 1e-9
            while True:
                left = due - time.monotonic()
                if left <= 0:
                    break
                time.sleep(min(left, 0.05))
            stamp = node.get_clock().now().to_msg()
            if topic == '/tf_static':
                for tr in msg.transforms:
                    tr.header.stamp = stamp
            else:
                msg.header.stamp = stamp
            pubs[topic].publish(msg)
            sent += 1
        print(f'loop {loop + 1}/{a.loops} done, {sent} messages so far', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
