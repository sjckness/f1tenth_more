"""tf_poll.py: a pose from /tf and /tf_static without spinning the subscriptions."""

import time

import pytest


def _publish(ros_context, seconds=1.0):
    """Spin a peer that publishes map -> odom (static) and odom -> base_link (dynamic)."""
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    peer = Node('tf_poll_peer', context=ros_context)
    st = TransformStamped()
    st.header.stamp = peer.get_clock().now().to_msg()
    st.header.frame_id, st.child_frame_id = 'map', 'odom'
    st.transform.translation.x = 10.0
    st.transform.rotation.w = 1.0
    StaticTransformBroadcaster(peer).sendTransform(st)
    dyn = TransformBroadcaster(peer)

    def tick():
        m = TransformStamped()
        m.header.stamp = peer.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.transform.translation.y = 2.0
        m.transform.rotation.w = 1.0
        dyn.sendTransform(m)

    peer.create_timer(0.01, tick)
    ex = SingleThreadedExecutor(context=ros_context)
    ex.add_node(peer)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ex.spin_once(timeout_sec=0.01)
    return peer, ex


def test_polled_lookup_composes_static_and_dynamic(ros_context):
    from rclpy.node import Node
    from tf2_ros import TransformException
    from f1tenth_comms.tf_poll import make_tf, PolledTf

    node = Node('poll_owner', context=ros_context)
    tf = make_tf(node)
    assert isinstance(tf, PolledTf)
    assert tf.side.get_name() == 'poll_owner_tf_poll'
    with pytest.raises(TransformException):
        tf.lookup('map', 'base_link')
    peer, ex = _publish(ros_context)
    try:
        t = None
        for _ in range(50):             # discovery can take a moment
            try:
                t = tf.lookup('map', 'base_link')
                break
            except TransformException:
                ex.spin_once(timeout_sec=0.05)
        assert t is not None, 'no transform'
        assert (t.transform.translation.x, t.transform.translation.y) == (10.0, 2.0)
        assert tf.taken > 10            # the messages were queued, then taken at once
    finally:
        tf.close()
        ex.shutdown()
        peer.destroy_node()
        node.destroy_node()


def test_listener_fallback(ros_context):
    from rclpy.node import Node
    from f1tenth_comms.tf_poll import ListenerTf, make_tf

    node = Node('listen_owner', context=ros_context)
    tf = make_tf(node, polled=False)
    try:
        assert isinstance(tf, ListenerTf)
    finally:
        tf.close()
        node.destroy_node()


def test_side_node_ignores_the_launch_rename(ros_context):
    """Keep the side node's own name when launch renames the node (global `-r __node:=`)."""
    import rclpy
    from rclpy.node import Node
    from f1tenth_comms.tf_poll import PolledTf

    ctx = rclpy.Context()
    rclpy.init(context=ctx, args=['--ros-args', '-r', '__node:=wifi_coverage_node'])
    try:
        node = Node('whatever', context=ctx)
        tf = PolledTf(node)
        assert node.get_name() == 'wifi_coverage_node'
        assert tf.side.get_name() == 'wifi_coverage_node_tf_poll'
        tf.close()
        node.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)
