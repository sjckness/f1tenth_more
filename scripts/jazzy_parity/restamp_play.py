#!/usr/bin/env python3
"""
Play a bag in real time with every header.stamp set to the wall clock.

The stamp is the time of publishing, the way the drivers it stands in for
would publish.

Runs on: the Orin host (native Humble), the harness container, linus.
Feeds phase5_bringup.sh's FEED=bag runs; the same bag and the same player on
both sides, so the stacks under comparison get identical input.

The stack runs on wall time (no use_sim_time), as on the car. `ros2 bag
play` keeps the recorded stamps, which are days old, and with those the
stack does not behave as on the car (Phase 5: the local EKF put
odom->base_link on /tf 28 times in 12 s instead of fusing /odom at 50 Hz).
This player keeps the bag's message timing and payloads and changes only
header.stamp (and each TransformStamped's header.stamp on /tf and
/tf_static), repeated --loops times back to back. /tf_static is published
once (it is latched).

QoS is each topic's recorded offered QoS (reliability, durability), depth
10. A topic without a header is refused unless --pass-headerless names it
(then published unchanged: std_msgs/Bool /mpc/hold has no stamp to fix).

Usage: restamp_play.py BAG [--loops N] [--pass-headerless TOPIC ...]
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_compat import open_reader  # noqa: E402, I100


def offered_qos(bag):
    """
    Return {topic: offered QoS profile list} from metadata.yaml.

    rosbag2_py's QoS objects expose no getters on Jazzy. Humble writes the list
    as a YAML string, Jazzy as YAML.
    """
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    return {t['topic_metadata']['name']: t['topic_metadata'].get('offered_qos_profiles')
            for t in info['topics_with_message_count']}


def qos_for(offered):
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--pass-headerless', nargs='*', default=[],
                    help='topics without a header to publish unchanged')
    a = ap.parse_args(argv)

    reader = open_reader(a.bag)
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    headerless = set()
    for name, meta in topics.items():
        sample = get_message(meta.type)()
        if name not in ('/tf', '/tf_static') and not hasattr(sample, 'header'):
            if name not in a.pass_headerless:
                raise SystemExit('%s has no header to re-stamp (see --pass-headerless)' % name)
            headerless.add(name)
    msgs = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        msgs.append((topic, deserialize_message(data, get_message(topics[topic].type)), t))
    del reader
    if not msgs:
        raise SystemExit('empty bag')

    rclpy.init()
    node = rclpy.create_node('restamp_player')
    offered = offered_qos(a.bag)
    pubs = {n: node.create_publisher(get_message(m.type), n, qos_for(offered.get(n)))
            for n, m in topics.items()}
    time.sleep(1.0)  # let subscribers match before the first message
    t_first = msgs[0][2]
    sent = 0
    try:
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
                if topic in ('/tf', '/tf_static'):
                    for tr in msg.transforms:
                        tr.header.stamp = stamp
                elif topic not in headerless:
                    msg.header.stamp = stamp
                pubs[topic].publish(msg)
                sent += 1
            print('loop %d/%d done, %d messages so far' % (loop + 1, a.loops, sent), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
