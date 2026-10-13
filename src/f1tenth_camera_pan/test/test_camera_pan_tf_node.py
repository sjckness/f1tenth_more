"""
camera_pan_tf_node with the pan disabled (stack_params camera_pan_enabled).

Disabled, the camera joint is static at pan 0: base_link -> camera_pan_base ->
zed2_camera_link is latched on /tf_static once and /camera_pan/joint_state is
not subscribed. The node is a real (never spun) rclpy Node; what it latches is
captured from its StaticTransformBroadcaster.
"""
from f1tenth_camera_pan import camera_pan_tf_node as mod
from f1tenth_camera_pan.frames import CAMERA_FRAME, PAN_BASE_FRAME
import pytest
import rclpy
from rclpy.parameter import Parameter


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def sent(monkeypatch):
    out = []

    def capture(self, transforms):
        out.extend(transforms if isinstance(transforms, list) else [transforms])
    monkeypatch.setattr(mod.StaticTransformBroadcaster, 'sendTransform', capture)
    return out


def _subscribed(node):
    return [s.topic_name for s in node.subscriptions]


def test_disabled_latches_the_camera_at_pan_zero(sent):
    node = mod.CameraPanTf()
    try:
        assert node.enabled is False
        edges = {(t.header.frame_id, t.child_frame_id): t for t in sent}
        assert set(edges) == {('base_link', PAN_BASE_FRAME), (PAN_BASE_FRAME, CAMERA_FRAME)}
        pan = edges[(PAN_BASE_FRAME, CAMERA_FRAME)].transform
        assert (pan.translation.x, pan.translation.y, pan.translation.z) == (0.0, 0.0, 0.0)
        assert (pan.rotation.x, pan.rotation.y, pan.rotation.z, pan.rotation.w) == (
            0.0, 0.0, 0.0, 1.0)
        assert '/camera_pan/joint_state' not in _subscribed(node)
    finally:
        node.destroy_node()


def test_enabled_follows_the_measured_angle(sent):
    node = mod.CameraPanTf(parameter_overrides=[Parameter('enabled', value=True)])
    try:
        assert [(t.header.frame_id, t.child_frame_id) for t in sent] == [
            ('base_link', PAN_BASE_FRAME)]
        assert '/camera_pan/joint_state' in _subscribed(node)
    finally:
        node.destroy_node()
