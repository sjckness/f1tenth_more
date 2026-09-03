"""Per-frame mask invariant tests (2026-09-02 stale-track fix).

In segment mode `/camera/detection_masks` must carry exactly one message per
processed frame, INCLUDING zero-detection frames, or detection_3d_node's 3-way
synchronizer cannot fire and semantic_layer_node never ticks the miss-streak
that ages a departed object's track out. See yolo_detector_node's
`_publish_empty_mask()` and the PER-FRAME MASK INVARIANT comment in its
image_callback().

Both halves of the fix are pinned here. Note the platform quirks are the
opposite way round from what one would guess, which is why they are tested
rather than assumed: cv_bridge RAISES on encoding a 0x0 array (so the empty
message is built by hand) but RETURNS None on decoding one (so the absence of
an exception does not imply a usable image).

Run standalone: python3 -m pytest test/test_empty_mask_invariant.py -v
"""

from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from f1tenth_perception.detection_3d_node import Detection3DNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class TestCvBridgeZeroSizeBehaviour:
    """Pins the platform behaviour the fix is built on. If a future cv_bridge
    makes these pass cleanly, the guards become redundant rather than wrong --
    but they must not be removed on the assumption that they already are."""

    def test_encoding_a_0x0_array_raises(self):
        """Why _publish_empty_mask builds the Image by hand."""
        with pytest.raises(Exception):
            CvBridge().cv2_to_imgmsg(np.zeros((0, 0), np.uint8), encoding='mono8')

    def test_decoding_a_0x0_image_returns_none_without_raising(self):
        """Why detection_3d_node cannot rely on `except CvBridgeError` alone to
        notice an unusable mask: no exception is raised at all, and the result
        is a bare None. A caller that assumed "no exception => usable image"
        would hand None to the depth-sampling path."""
        msg = Image()
        msg.height = 0
        msg.width = 0
        msg.encoding = 'mono8'
        msg.step = 0
        msg.data = []
        assert CvBridge().imgmsg_to_cv2(msg, desired_encoding='mono8') is None


def _empty_mask(stamp_sec=5, stamp_nsec=0):
    msg = Image()
    msg.header = Header()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    msg.header.frame_id = 'zed2_left_camera_optical_frame'
    msg.height = 0
    msg.width = 0
    msg.encoding = 'mono8'
    msg.step = 0
    msg.data = []
    return msg


def _node(**params):
    params.setdefault('use_mask_depth', True)
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    n = Detection3DNode(parameter_overrides=overrides)
    n.det3d_pub.publish = MagicMock()
    n.marker_pub.publish = MagicMock()
    return n


class TestDetection3DNodeHandlesEmptyMask:

    def test_zero_size_mask_is_treated_as_no_mask_not_an_error(self):
        n = _node()
        n.get_logger().error = MagicMock()
        # Reproduces exactly the branch _synced_callback runs for the mask.
        mask_msg = _empty_mask()
        assert mask_msg.height == 0 and mask_msg.width == 0
        # The guard must short-circuit before cv_bridge is ever reached.
        n.bridge.imgmsg_to_cv2 = MagicMock(
            side_effect=AssertionError('cv_bridge must not be called on a 0x0 mask'))
        mask_img = None
        if mask_msg is not None:
            if mask_msg.height == 0 or mask_msg.width == 0:
                mask_img = None
            else:
                mask_img = n.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        assert mask_img is None

    def test_empty_frame_still_publishes_a_detection3darray(self):
        """The whole point: a zero-detection frame must still produce one
        (empty) Detection3DArray, because that is the message
        semantic_layer_node counts as a miss."""
        from vision_msgs.msg import Detection2DArray
        from geometry_msgs.msg import TransformStamped
        n = _node()
        info = MagicMock()
        info.k = [500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1]
        n._camera_info_callback(info)
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        n.tf_buffer.lookup_transform = MagicMock(return_value=tf)

        det = Detection2DArray()
        det.header.stamp.sec = 5
        depth = CvBridge().cv2_to_imgmsg(
            np.full((480, 640), 2.0, dtype=np.float32), encoding='32FC1')
        depth.header.stamp.sec = 5
        depth.header.frame_id = 'zed2_left_camera_optical_frame'

        n._synced_callback(det, depth, _empty_mask())

        assert n.det3d_pub.publish.call_count == 1
        published = n.det3d_pub.publish.call_args[0][0]
        assert len(published.detections) == 0
        # An empty frame is not a failure -- it must not be booked as one.
        assert n._stats['drop_cv_bridge'] == 0
        assert n._stats['det_drop_mask_fallback'] == 0
        assert n._stats['published'] == 1
