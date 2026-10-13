"""
front_depth_monitor_node: /perception/front_distance from the front bumper.

The node is not launched (front_clearance_node publishes the topic), but it
publishes the same topic by hand, so it must mean the same thing: the lens
percentile minus the lens-to-bumper offset (base_front x minus the depth
frame's x in base_link, from /tf_static), never below MIN_FRONT_DISTANCE_M,
and nothing at all before the camera's transform is known.

Run standalone: python3 -m pytest test/test_front_depth_monitor_node.py -v
"""

from unittest.mock import MagicMock

from cv_bridge import CvBridge
from f1tenth_description.vehicle_frames import base_front_x, MIN_FRONT_DISTANCE_M
from f1tenth_perception.front_depth_monitor_node import FrontDepthMonitorNode
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from tf2_msgs.msg import TFMessage

FRONT = base_front_x()
CAMERA_FRAME = 'test_zed2_left_camera_optical_frame'
#: A lens 0.323 m behind the bumper (camera at base_link x 0.12).
LENS_BEHIND_M = 0.323


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node(camera_x):
    node = FrontDepthMonitorNode()
    if camera_x is not None:
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = CAMERA_FRAME
        t.transform.translation.x = float(camera_x)
        t.transform.rotation.w = 1.0
        node._tf_static_callback(TFMessage(transforms=[t]))
    node.front_distance_pub.publish = MagicMock()
    return node


def _depth(value):
    msg = CvBridge().cv2_to_imgmsg(np.full((100, 100), value, dtype=np.float32),
                                   encoding='32FC1')
    msg.header.frame_id = CAMERA_FRAME
    return msg


def test_front_distance_is_the_lens_reading_minus_the_lens_to_bumper_offset():
    node = _node(FRONT - LENS_BEHIND_M)
    node._depth_callback(_depth(2.0))
    assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(
        2.0 - LENS_BEHIND_M)
    node.destroy_node()


def test_a_wall_at_or_behind_the_bumper_publishes_the_positive_floor():
    node = _node(FRONT - LENS_BEHIND_M)
    node._depth_callback(_depth(0.30))
    assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(
        MIN_FRONT_DISTANCE_M)
    node.destroy_node()


def test_without_the_camera_transform_nothing_is_published():
    node = _node(None)
    node._depth_callback(_depth(2.0))
    node.front_distance_pub.publish.assert_not_called()
    node.destroy_node()
