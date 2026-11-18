"""detection_3d_node and obstacle_projector_node read TF from /tf_static only.

R3 (2026-10-23): both nodes used a full TransformListener for one chain of
fixed joints, which made rclpy deserialise every /tf message on their
executors. By default they now fill their tf2 Buffer from the latched
/tf_static alone (f1tenth_perception/static_tf.py). What is pinned here:

- the default: no /tf subscription, a /tf_static one; tf_static_only:=false
  restores the listener (the old path);
- a moving output_frame (map, odom) is refused at startup with
  tf_static_only true, accepted with false as before;
- BEHAVIOUR IDENTITY: the same /tf_static messages, fed once through the new
  static subscription and once through the old TransformListener's own
  callback, give byte-identical published Detection3DArray / MarkerArray /
  Obstacle2DArray for the real car/Isaac camera chain
  (output/jazzy_port/e2e/tf.json), with /tf traffic present for the listener;
- the real subscription receives a latched StaticTransformBroadcaster over
  DDS (isolated domain, localhost only).

Same no-spin convention as test_detection_3d_node.py for everything but the
DDS test.

Run standalone: python3 -m pytest test/test_static_tf.py -v
"""

import os
import time
from unittest.mock import MagicMock

from cv_bridge import CvBridge
from f1tenth_perception.detection_3d_node import Detection3DNode
from f1tenth_perception.obstacle_projector_node import ObstacleProjectorNode
from f1tenth_perception.static_tf import DYNAMIC_FRAMES, refuse_dynamic_frame
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _tf(parent, child, xyz=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0), sec=0):
    t = TransformStamped()
    t.header.stamp.sec = sec
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = q
    return t


def _camera_chain():
    """Return the camera edges as the Isaac e2e run measured them (tf.json), all static."""
    return TFMessage(transforms=[
        _tf('base_link', 'zed2_camera_link', (0.2399, 0.0, 0.205),
            (0.0, -0.01817399939, 0.0, 0.99983483923)),
        _tf('zed2_camera_link', 'zed2_camera_center', (0.0, 0.0, 0.015)),
        _tf('zed2_camera_center', 'zed2_left_camera_frame', (-0.01, 0.06, 0.0)),
        _tf('zed2_left_camera_frame', 'zed2_left_camera_optical_frame', (0.0, 0.0, 0.0),
            (0.5, -0.4999999999999999, 0.5, -0.5000000000000001)),
        _tf('base_link', 'laser', (0.305, 0.0, 0.2)),
    ])


def _dynamic_tf(sec):
    """/tf traffic the old listener also took in: odom -> base_link, map -> odom."""
    return TFMessage(transforms=[
        _tf('odom', 'base_link', (0.1 * sec, 0.02 * sec, 0.0),
            (0.0, 0.0, 0.0998334, 0.9950042), sec=sec),
        _tf('map', 'odom', (0.25, 0.07, 0.0), (0.0, 0.0, 0.16, 0.987), sec=sec),
    ])


def _topics(node):
    return sorted(s.topic_name for s in node.subscriptions)


def _params(d):
    return [Parameter(k, value=v) for k, v in (d or {}).items()]


def _detection_node(params=None):
    node = Detection3DNode(parameter_overrides=_params(params))
    node.det3d_pub.publish = MagicMock()
    node.marker_pub.publish = MagicMock()
    return node


def _projector(params=None):
    node = ObstacleProjectorNode(parameter_overrides=_params(params))
    node.obstacles_pub.publish = MagicMock()
    return node


def _feed(node, static_msgs, dynamic_msgs=()):
    """Deliver TF the way each mode's own subscription callbacks would."""
    if node.tf_listener is None:
        for m in static_msgs:
            node._tf.on_tf_static(m)
        return
    for m in static_msgs:
        node.tf_listener.static_callback(m)
    for m in dynamic_msgs:
        node.tf_listener.callback(m)


class TestTheDefaultIsTfStaticOnly:

    @pytest.mark.parametrize('make', [_detection_node, _projector])
    def test_no_tf_subscription_and_no_listener(self, make):
        node = make()
        try:
            assert node.tf_static_only is True
            assert node.tf_listener is None
            assert node._tf.static_sub is not None
            assert '/tf_static' in _topics(node)
            assert '/tf' not in _topics(node)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('make', [_detection_node, _projector])
    def test_false_restores_the_transform_listener(self, make):
        node = make({'tf_static_only': False})
        try:
            assert isinstance(node.tf_listener, TransformListener)
            assert node._tf.static_sub is None
            assert node.tf_listener.buffer is node.tf_buffer
            assert '/tf' in _topics(node) and '/tf_static' in _topics(node)
        finally:
            node.destroy_node()

    def test_the_stack_params_defaults_are_true(self):
        from f1tenth_params.param_defaults import get_value
        assert get_value('detection_3d_tf_static_only') is True
        assert get_value('obstacle_projector_tf_static_only') is True


class TestAMovingOutputFrameIsRefused:

    @pytest.mark.parametrize('frame', ['map', 'odom', '/odom'])
    @pytest.mark.parametrize('make', [_detection_node, _projector])
    def test_refused_at_startup(self, make, frame):
        with pytest.raises(ValueError, match='moving frame'):
            make({'output_frame': frame})

    @pytest.mark.parametrize('make', [_detection_node, _projector])
    def test_accepted_with_the_full_listener(self, make):
        node = make({'output_frame': 'odom', 'tf_static_only': False})
        try:
            assert node.output_frame == 'odom'
        finally:
            node.destroy_node()

    def test_the_fixed_frames_pass(self):
        for frame in ('base_link', 'zed2_left_camera_frame', 'laser', 'base_front'):
            refuse_dynamic_frame(frame)
        assert DYNAMIC_FRAMES == {'map', 'odom'}

    def test_a_dynamic_edge_never_resolves_from_tf_static(self):
        """A chain through /tf drops the frame and says why."""
        node = _projector()
        try:
            node.get_logger().warn = MagicMock()
            _feed(node, [_camera_chain()])
            # Even if odom -> base_link were offered, the static buffer has no
            # /tf subscription to take it from: the lookup fails.
            with pytest.raises(TransformException):
                node.tf_buffer.lookup_transform('odom', 'zed2_left_camera_frame',
                                                Time())
            msg = Detection3DArray()
            msg.header.frame_id = 'zed2_left_camera_frame'
            msg.detections.append(_det3d(2.0, 0.0, 0.0))
            node.output_frame = 'odom'   # bypasses the startup check on purpose
            node._detections_callback(msg)
            node.obstacles_pub.publish.assert_not_called()
            text = node.get_logger().warn.call_args[0][0]
            assert 'tf_static_only' in text
        finally:
            node.destroy_node()


def _camera_info(fx=500.0, fy=500.0, cx=320.0, cy=240.0):
    msg = CameraInfo()
    msg.k[0], msg.k[4], msg.k[2], msg.k[5] = fx, fy, cx, cy
    return msg


def _det2d(cx, cy, w, h, score=0.9, class_id='person'):
    det = Detection2D()
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = float(score)
    det.results.append(hyp)
    det.bbox.center.position.x = float(cx)
    det.bbox.center.position.y = float(cy)
    det.bbox.size_x = float(w)
    det.bbox.size_y = float(h)
    return det


def _det3d(x, y, z, w=0.5, h=1.7, class_id='person'):
    det = Detection3D()
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = 0.9
    det.results.append(hyp)
    det.bbox.center.position.x = float(x)
    det.bbox.center.position.y = float(y)
    det.bbox.center.position.z = float(z)
    det.bbox.center.orientation.w = 1.0
    det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = float(w), float(h), 0.3
    return det


class TestTheOutputIsIdenticalToTheListenerPath:
    """Old path (TransformListener callbacks) vs new (static subscription), same input."""

    def test_detection_3d_publishes_the_same_arrays(self):
        new, old = _detection_node(), _detection_node({'tf_static_only': False})
        try:
            _feed(new, [_camera_chain()])
            _feed(old, [_camera_chain()], [_dynamic_tf(s) for s in range(1, 6)])
            depth = np.full((480, 640), 3.0, dtype=np.float32)
            depth[200:300, 100:200] = 1.7
            depth[100:180, 400:520] = 2.4
            bridge = CvBridge()
            for node in (new, old):
                node._camera_info_callback(_camera_info())
                for k in range(3):
                    det = Detection2DArray()
                    det.header.stamp.sec = 10 + k
                    det.detections.append(_det2d(150, 250, 80, 90))
                    det.detections.append(_det2d(460, 140, 100, 60, 0.6, 'chair'))
                    det.detections.append(_det2d(320, 240, 20, 20, 0.1))  # below threshold
                    img = bridge.cv2_to_imgmsg(depth + 0.1 * k, encoding='32FC1')
                    img.header.stamp.sec = 10 + k
                    img.header.frame_id = 'zed2_left_camera_optical_frame'
                    node._synced_callback(det, img)
            assert new.det3d_pub.publish.call_count == 3
            assert old.det3d_pub.publish.call_count == 3
            for got, want in zip(new.det3d_pub.publish.call_args_list,
                                 old.det3d_pub.publish.call_args_list):
                assert len(got[0][0].detections) == 2
                assert got[0][0] == want[0][0]
            for got, want in zip(new.marker_pub.publish.call_args_list,
                                 old.marker_pub.publish.call_args_list):
                assert got[0][0] == want[0][0]
            assert new._stats['drop_tf2'] == old._stats['drop_tf2'] == 0
            # The optical -> camera rotation really was applied (not identity).
            pose = new.det3d_pub.publish.call_args[0][0].detections[0].bbox.center
            assert pose.position.x == pytest.approx(1.9, abs=1e-5)  # depth 1.7 + 0.2
        finally:
            new.destroy_node()
            old.destroy_node()

    def test_obstacle_projector_publishes_the_same_obstacles(self):
        new, old = _projector(), _projector({'tf_static_only': False})
        try:
            _feed(new, [_camera_chain()])
            _feed(old, [_camera_chain()], [_dynamic_tf(s) for s in range(1, 6)])
            for node in (new, old):
                for k in range(3):
                    msg = Detection3DArray()
                    msg.header.stamp.sec = 20 + k
                    msg.header.frame_id = 'zed2_left_camera_frame'
                    msg.detections.append(_det3d(2.0 + 0.1 * k, 0.3, 0.1))
                    msg.detections.append(_det3d(3.5, -0.8, 0.0, w=0.45, h=0.85,
                                                 class_id='chair'))
                    node._detections_callback(msg)
            calls_new = new.obstacles_pub.publish.call_args_list
            calls_old = old.obstacles_pub.publish.call_args_list
            assert len(calls_new) == len(calls_old) == 3
            for got, want in zip(calls_new, calls_old):
                assert len(got[0][0].obstacles) == 2
                assert got[0][0] == want[0][0]
            first = calls_new[0][0][0].obstacles[0]
            # The camera sits ~0.23 m ahead of base_link, pitched down 2.1 deg:
            # the chain was applied, not an identity.
            assert 2.2 < first.x < 2.25
        finally:
            new.destroy_node()
            old.destroy_node()


@pytest.fixture
def isolated_context(monkeypatch):
    """Yield an rclpy context on its own domain that can only see this host."""
    for k in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
              'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=int(os.environ.get('ROS_DOMAIN_ID', '148')))
    try:
        yield ctx
    finally:
        rclpy.shutdown(context=ctx)


def test_a_latched_static_broadcaster_reaches_the_buffer_over_dds(isolated_context):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_ros import StaticTransformBroadcaster

    ctx = isolated_context
    peer = Node('static_tf_test_peer', context=ctx)
    StaticTransformBroadcaster(peer).sendTransform(list(_camera_chain().transforms))
    node = ObstacleProjectorNode(context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    ex.add_node(peer)
    try:
        got = None
        t0 = time.monotonic()
        while got is None and time.monotonic() - t0 < 10.0:
            ex.spin_once(timeout_sec=0.05)
            try:
                got = node.tf_buffer.lookup_transform(
                    'base_link', 'zed2_left_camera_optical_frame', Time())
            except TransformException:
                pass
        assert got is not None, 'no /tf_static over DDS within 10 s'
        pitch = 2.0 * np.arcsin(-0.01817399939)
        want_x = 0.2399 + 0.015 * np.sin(pitch) - 0.01 * np.cos(pitch)
        assert got.transform.translation.x == pytest.approx(want_x, abs=1e-5)
    finally:
        ex.shutdown()
        node.destroy_node()
        peer.destroy_node()
