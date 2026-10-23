#!/usr/bin/env python3
r"""Replay a bag's /tf and /tf_static through MPC_corr's old and new TF paths, per tick.

R3 (2026-10-23) replaced MPC_corr's TransformListener with f1tenth_comms'
PolledTf (map -> odom, polled once per control tick) plus a /tf_static-only
buffer (base_link <- laser). This tool checks, offline, that every lookup
MPC_corr makes per control tick returns the same thing as before:

- the real MPCController (stack_params defaults, use_sim_time, its control
  timer cancelled: the tool plays the tick) receives the bag's /tf and
  /tf_static over DDS on a private localhost-only domain;
- a reference node runs the OLD path: a TransformListener into a plain Buffer
  (what MPC_corr.__init__ built before R3), counting every delivery;
- the bag is cut into 0.1 s ticks of receive time (the control period). Each
  tick: publish the tick's messages and /clock = the tick's end, wait until
  both sides received all of them (the node polls through its own
  _poll_tf(), as control_loop does), then evaluate on the node:
    map_odom        _map_odom_tf.lookup(map, odom) vs the reference Buffer's
                    lookup_transform(map, odom, Time()) (TransformStamped ==)
    anchor          _lookup_map_odom()                       (capture path)
    anchor_guarded  _lookup_map_odom(max_age_sec=map_odom_max_age_sec)
    laser           _laser_pose_for(<laser frame>)
  the OLD value of the last three by running the same node method with the
  reference Buffer swapped in (_map_odom_tf / static_tf_buffer), i.e. the
  pre-R3 code on the pre-R3 buffer. Equality is exact (==), no tolerance.

Usage (the workspace's install sourced, isolated DDS):
    python3 tools/r3_tf_replay.py ~/bags/corridor10m_20261016T144537 \\
        --json output/rust_main/r3_tf_replay_20261016.json
Exit 1 when any tick differs.
"""

import argparse
import json
import os
import sys
import time


def read_tf(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    dynamic, static = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        (static if topic == '/tf_static' else dynamic).append((t_ns, msg))
    return dynamic, static


class _Shim:
    """Stands in for _map_odom_tf with the reference buffer: the pre-R3 lookup."""

    def __init__(self, buffer):
        self.buffer = buffer

    def lookup(self, target, source):
        import rclpy.time
        return self.buffer.lookup_transform(target, source, rclpy.time.Time())

    def poll(self):
        pass


def _ts(t):
    if t is None:
        return None
    return [t.header.stamp.sec, t.header.stamp.nanosec, t.header.frame_id, t.child_frame_id,
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
            t.transform.rotation.w]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--tick', type=float, default=0.1, help='control period [s]')
    ap.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '96')))
    ap.add_argument('--laser-frame', default='laser')
    ap.add_argument('--max-ticks', type=int, default=0, help='0: the whole bag')
    ap.add_argument('--json')
    a = ap.parse_args()

    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
                'FASTRTPS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    import rclpy.time
    from builtin_interfaces.msg import Time as TimeMsg
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    dynamic, static = read_tf(a.bag)
    if not dynamic:
        print('no /tf in the bag', file=sys.stderr)
        return 2
    t0 = dynamic[0][0]
    tick_ns = int(round(a.tick * 1e9))
    n_ticks = (dynamic[-1][0] - t0) // tick_ns + 1
    if a.max_ticks:
        n_ticks = min(n_ticks, a.max_ticks)

    rclpy.init(args=['--ros-args', '--log-level', 'fatal', '-p', 'use_sim_time:=true'])
    from mpc_controller.MPC_corr import MPCController
    node = MPCController()
    node.timer.cancel()
    player = Node('r3_tf_replay_player', use_global_arguments=False)
    tf_pub = player.create_publisher(TFMessage, '/tf', QoSProfile(
        depth=1000, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
    static_pub = player.create_publisher(TFMessage, '/tf_static', QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST))
    clock_pub = player.create_publisher(Clock, '/clock', 10)

    ref_node = Node('r3_tf_replay_reference', use_global_arguments=False)
    ref_buffer = Buffer()
    counts = {'n': 0}

    class _Counting(TransformListener):
        def callback(self, data):
            super().callback(data)
            counts['n'] += 1

        def static_callback(self, data):
            super().static_callback(data)
            counts['n'] += 1

    _Counting(ref_buffer, ref_node)
    ex = SingleThreadedExecutor()
    for n in (node, player, ref_node):
        ex.add_node(n)

    def spin_until(cond, what, timeout=20.0):
        t_start = time.monotonic()
        while not cond():
            if time.monotonic() - t_start > timeout:
                raise RuntimeError(f'timeout waiting for {what}')
            ex.spin_once(timeout_sec=0.002)
            node._poll_tf()

    poller = node._map_odom_tf
    spin_until(lambda: tf_pub.get_subscription_count() >= 2
               and static_pub.get_subscription_count() >= 3, 'discovery')
    for _, msg in static:
        static_pub.publish(msg)
    sent = len(static)
    spin_until(lambda: counts['n'] >= sent and poller.taken >= sent, 'static delivery')

    def stamp_msg(ns):
        return TimeMsg(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))

    keys = ('map_odom', 'anchor', 'anchor_guarded', 'laser')
    stats = {k: {'equal': 0, 'differ': 0, 'both_none': 0} for k in keys}
    first_diff = []
    guarded_none = {'old': 0, 'new': 0}
    i = 0
    wall0 = time.monotonic()
    for k in range(n_ticks):
        end_ns = t0 + (k + 1) * tick_ns
        while i < len(dynamic) and dynamic[i][0] < end_ns:
            tf_pub.publish(dynamic[i][1])
            i += 1
            sent += 1
        clock_pub.publish(Clock(clock=stamp_msg(end_ns)))
        spin_until(lambda: counts['n'] >= sent and poller.taken >= sent
                   and node.get_clock().now().nanoseconds >= end_ns, f'tick {k}')

        results = {}
        # NEW: the node as it is now.
        try:
            new_mo = poller.lookup('map', 'odom')
        except TransformException:
            new_mo = None
        results['anchor'] = [None, node._lookup_map_odom()]
        results['anchor_guarded'] = [
            None, node._lookup_map_odom(max_age_sec=node.map_odom_max_age_sec)]
        node._laser_pose = None
        results['laser'] = [None, node._laser_pose_for(a.laser_frame)]
        # OLD: the same methods on the pre-R3 buffer (the listener's).
        try:
            old_mo = ref_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except TransformException:
            old_mo = None
        node._map_odom_tf = _Shim(ref_buffer)
        static_buffer, node.static_tf_buffer = node.static_tf_buffer, ref_buffer
        try:
            results['anchor'][0] = node._lookup_map_odom()
            results['anchor_guarded'][0] = node._lookup_map_odom(
                max_age_sec=node.map_odom_max_age_sec)
            node._laser_pose = None
            results['laser'][0] = node._laser_pose_for(a.laser_frame)
        finally:
            node._map_odom_tf = poller
            node.static_tf_buffer = static_buffer
        results['map_odom'] = [_ts(old_mo), _ts(new_mo)]

        for key in keys:
            old, new = results[key]
            if old is None and new is None:
                stats[key]['both_none'] += 1
            elif old == new:
                stats[key]['equal'] += 1
            else:
                stats[key]['differ'] += 1
                if len(first_diff) < 20:
                    first_diff.append({'tick': k, 'key': key, 'old': old, 'new': new})
        guarded_none['old'] += results['anchor_guarded'][0] is None
        guarded_none['new'] += results['anchor_guarded'][1] is None

    report = {
        'bag': os.path.abspath(a.bag),
        'ticks': int(n_ticks), 'tick_s': a.tick,
        'tf_messages': i, 'tf_static_messages': len(static),
        'received_reference': counts['n'], 'taken_polled': poller.taken,
        'stats': stats, 'anchor_guarded_none': guarded_none,
        'first_differences': first_diff,
        'wall_s': round(time.monotonic() - wall0, 1),
    }
    text = json.dumps(report, indent=1)
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as f:
            f.write(text + '\n')
    print(text)
    ex.shutdown()
    ref_node.destroy_node()
    player.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
    differ = sum(s['differ'] for s in stats.values())
    return 1 if differ else 0


if __name__ == '__main__':
    sys.exit(main())
