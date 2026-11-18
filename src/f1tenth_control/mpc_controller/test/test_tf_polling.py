"""
MPC_corr's TF without a TransformListener: map -> odom polled, the laser from /tf_static.

R3 (2026-10-23). MPC_corr used to own a TransformListener, so its executor took
every /tf message (20-105/s on the car) for one map -> odom lookup per tick and
one base_link <- laser lookup per run. Now map -> odom comes from
f1tenth_comms' PolledTf (side node mpc_corr_tf_poll, polled once per control
tick) and base_link <- laser from a buffer fed by /tf_static alone.

What is pinned here, on the real MPCController (stack_params defaults, its
control timer cancelled) on a private localhost-only DDS domain:

- the node itself no longer subscribes /tf; tf_polling:=false is the listener;
- THE LOOKUPS ARE IDENTICAL: the same /tf and /tf_static stream, published over
  DDS, is received by the node and by a reference node that runs the old code
  path (a TransformListener into a plain Buffer). After each tick the node's
  map -> odom TransformStamped, its _lookup_map_odom() result (with and without
  the age guard) and its _laser_pose_for() result equal the reference's;
- control_loop polls every tick, before any early return.

The offline replay of a recorded car bag through the same comparison is
tools/r3_tf_replay.py (output/rust_main/r3.md).

Run standalone: python3 -m pytest test/test_tf_polling.py -v
"""

import math
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def make_node(monkeypatch):
    """Build MPCController(s) with `overrides`, control timer cancelled, torn down after."""
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
                'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '149'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    made = []

    def _make(**overrides):
        from mpc_controller.MPC_corr import MPCController
        args = ['--ros-args', '--log-level', 'fatal']
        for key, value in overrides.items():
            args += ['-p', f'{key}:={value}']
        if not rclpy.ok():
            rclpy.init(args=args)
        node = MPCController()
        node.timer.cancel()
        made.append(node)
        return node

    yield _make
    for node in made:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _topics(node):
    return {sub.topic_name for sub in node.subscriptions}


def test_the_node_has_no_tf_subscription_of_its_own(make_node):
    from f1tenth_comms.tf_poll import PolledTf
    node = make_node()
    assert node.tf_polling is True
    assert isinstance(node._map_odom_tf, PolledTf)
    assert node._map_odom_tf.side.get_name() == 'mpc_corr_tf_poll'
    assert {s.topic_name for s, _ in node._map_odom_tf.subs} == {'/tf', '/tf_static'}
    assert '/tf' not in _topics(node)
    assert '/tf_static' in _topics(node)        # the laser's static buffer
    assert node.tf_buffer is node._map_odom_tf.buffer


def test_tf_polling_false_is_the_transform_listener(make_node):
    from f1tenth_comms.tf_poll import ListenerTf
    node = make_node(tf_polling='false')
    assert isinstance(node._map_odom_tf, ListenerTf)
    assert '/tf' in _topics(node)


class _Reference:
    """The old code path: a TransformListener into a plain Buffer, counting deliveries."""

    def __init__(self, context):
        from rclpy.node import Node
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener

        self.node = Node('r3_reference_listener', context=context)
        self.buffer = Buffer()
        self.received = 0
        ref = self

        class _Counting(TransformListener):
            def callback(self, data):
                super().callback(data)
                ref.received += 1

            def static_callback(self, data):
                super().static_callback(data)
                ref.received += 1

        self.listener = _Counting(self.buffer, self.node)


def _tf(parent, child, stamp, x=0.0, y=0.0, yaw=0.0):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def _wait(executor, node, ref, expected, timeout=10.0):
    """Spin until the reference took `expected` messages and the poller took its /tf+static."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        executor.spin_once(timeout_sec=0.01)
        node._poll_tf()
        if ref.received >= expected and node._map_odom_tf.taken >= expected:
            return
    raise AssertionError(
        f'delivery stalled: reference {ref.received}, poller {node._map_odom_tf.taken}, '
        f'want {expected}')


def test_every_tick_looks_up_what_the_listener_would_have(make_node):
    import rclpy.time
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster

    node = make_node()
    assert node.use_map_frame_goal_anchor is True   # localization_source ekf
    ctx = node.context
    peer = Node('r3_tf_player', context=ctx)
    tf_pub = peer.create_publisher(TFMessage, '/tf', QoSProfile(
        depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
    ref = _Reference(ctx)
    ex = SingleThreadedExecutor(context=ctx)
    for n in (node, peer, ref.node):
        ex.add_node(n)
    try:
        # Nothing yet: both sides fail alike.
        assert node._lookup_map_odom() is None
        now = node.get_clock().now().to_msg()
        StaticTransformBroadcaster(peer).sendTransform(
            [_tf('base_link', 'laser', now, x=0.305, yaw=math.pi)])
        sent = 1
        # Discovery first, so no tick's message can be lost to it.
        t0 = time.monotonic()
        while (tf_pub.get_subscription_count() < 2 and time.monotonic() - t0 < 10.0):
            ex.spin_once(timeout_sec=0.05)
        assert tf_pub.get_subscription_count() >= 2
        _wait(ex, node, ref, sent)
        t0 = time.monotonic()
        while (not node.static_tf_buffer.can_transform('base_link', 'laser', rclpy.time.Time())
               and time.monotonic() - t0 < 10.0):
            ex.spin_once(timeout_sec=0.01)
        compared = 0
        for k in range(40):
            stamp = node.get_clock().now().to_msg()
            # The global EKF's map -> odom with the local odom -> base_link, then a
            # second odom -> base_link alone (the local EKF runs faster).
            tf_pub.publish(TFMessage(transforms=[
                _tf('map', 'odom', stamp, 0.25 + 0.01 * k, 0.07 - 0.002 * k, 0.16 + 0.004 * k),
                _tf('odom', 'base_link', stamp, 0.05 * k, 0.0, 0.01 * k)]))
            tf_pub.publish(TFMessage(transforms=[
                _tf('odom', 'base_link', stamp, 0.05 * k + 0.02, 0.0, 0.01 * k)]))
            sent += 2
            _wait(ex, node, ref, sent)

            new = node._map_odom_tf.lookup('map', 'odom')
            old = ref.buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            assert new == old
            # What the old _lookup_map_odom returned from that TransformStamped.
            want = (float(old.transform.translation.x), float(old.transform.translation.y),
                    float(node.quaternion_to_yaw(old.transform.rotation)))
            assert want[2] == pytest.approx(0.16 + 0.004 * k)
            assert node._lookup_map_odom() == want
            assert node._lookup_map_odom(max_age_sec=node.map_odom_max_age_sec) == want
            old_laser = ref.buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
            node._laser_pose = None                   # look it up again every tick
            assert node._laser_pose_for('laser') == (
                old_laser.transform.translation.x, old_laser.transform.translation.y,
                node.quaternion_to_yaw(old_laser.transform.rotation))
            compared += 1
        assert compared == 40
        assert node._map_odom_tf.taken == ref.received == sent
    finally:
        ex.shutdown()
        ref.node.destroy_node()
        peer.destroy_node()


def test_control_loop_polls_before_any_early_return(make_node):
    import rclpy.time
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    node = make_node()
    peer = Node('r3_tf_player_2', context=node.context)
    pub = peer.create_publisher(TFMessage, '/tf', QoSProfile(
        depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
    try:
        t0 = time.monotonic()
        while pub.get_subscription_count() < 1 and time.monotonic() - t0 < 10.0:
            time.sleep(0.02)
        stamp = node.get_clock().now().to_msg()
        for k in range(5):
            pub.publish(TFMessage(transforms=[_tf('map', 'odom', stamp, 0.1 * k)]))
        assert node.x is None                         # control_loop returns early
        t0 = time.monotonic()
        while node._map_odom_tf.taken < 5 and time.monotonic() - t0 < 10.0:
            node.control_loop()
            time.sleep(0.01)
        assert node._map_odom_tf.taken == 5
        assert node.tf_buffer.can_transform('map', 'odom', rclpy.time.Time())
    finally:
        peer.destroy_node()
