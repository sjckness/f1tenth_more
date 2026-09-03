"""detection_3d_node.py tests -- constructs a real (but never spun) rclpy Node
and calls its callbacks/methods directly, same "no live topics/hardware
needed, pure in-process method calls" convention established by
f1tenth_diagnostics/test/test_slam_pose_covariance_calibration_node.py and
followed by f1tenth_costmap/test/test_costmap_boundary_node.py (see either
file's own module docstring). Publishers are never actually sent over DDS
here (no spin() anywhere in this file) -- det3d_pub.publish/marker_pub.publish
are directly mocked so each test can assert exactly what would have been
published without a live subscriber.

Scope: the mask-based depth-sampling addition (_mask_median_depth, and its
wiring into _synced_callback via use_mask_depth) -- see detection_3d_node.py's
own module docstring's "Mask-based depth sampling" paragraph. The pre-existing
box-region method (_median_depth) is exercised here only as the known-good
baseline the fallback path is checked against; it is NOT itself being changed
by this pass.

tf2 in TestSyncedCallbackMaskWiring: `_synced_callback` looks up a real tf2
transform (optical frame -> output_frame) via self.tf_buffer -- mocked here to
return a fixed IDENTITY TransformStamped rather than standing up a real
TransformListener/broadcaster, since none of these tests care about that
rotation (see _identity_transform()'s own docstring for why the resulting
pose.position.z is unaffected by it either way).

Run standalone: python3 -m pytest test/test_detection_3d_node.py -v
"""

from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from f1tenth_perception.detection_3d_node import Detection3DNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = Detection3DNode(parameter_overrides=overrides)
    # Never actually sent over DDS in this test file (no spin()) -- mocked so
    # tests can assert on exactly what WOULD have been published.
    node.det3d_pub.publish = MagicMock()
    node.marker_pub.publish = MagicMock()
    return node


def _fake_camera_info(fx=500.0, fy=500.0, cx=320.0, cy=240.0):
    msg = CameraInfo()
    msg.k[0] = fx
    msg.k[4] = fy
    msg.k[2] = cx
    msg.k[5] = cy
    return msg


def _fake_detection2d(cx, cy, w, h, score=0.9, class_id='person'):
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


def _fake_depth_image(depth_arr, frame_id='zed2_left_camera_optical_frame'):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(depth_arr.astype(np.float32), encoding='32FC1')
    msg.header.frame_id = frame_id
    return msg


def _fake_mask_image(label_arr):
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(label_arr.astype(np.uint8), encoding='mono8')


def _identity_transform(
        child_frame='zed2_left_camera_optical_frame',
        parent_frame='zed2_left_camera_frame'):
    """A no-rotation, no-translation TransformStamped. optical_pose.position.z
    is copied straight from the sampled depth (see detection_3d_node.py's
    _synced_callback: `optical_pose.position.z = float(z)`, no fx/fy/cx/cy
    involved) and an identity rotation leaves z unchanged too -- so asserting
    on out_pose.position.z after this transform is exactly asserting on the
    depth-sampling method's own return value, decoupled from tf2 correctness
    (which is out of scope here -- unchanged by this pass)."""
    t = TransformStamped()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.rotation.w = 1.0
    return t


class TestMaskMedianDepthCore:
    """_mask_median_depth in isolation -- no sync/tf2/publish involved."""

    def test_median_over_masked_valid_pixels_only(self):
        node = _construct_with_params()
        try:
            depth = np.full((10, 10), 5.0, dtype=np.float32)
            # Label 1 covers a 2x3 block with one outlier value -- median of
            # {1.0, 1.0, 1.0, 1.0, 1.0, 9.0} is 1.0, not the mean (~2.33) and
            # not the background 5.0 outside the mask.
            mask = np.zeros((10, 10), dtype=np.uint8)
            mask[2:4, 2:5] = 1
            depth[2:4, 2:5] = 1.0
            depth[3, 4] = 9.0

            z = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=3, cy_px=3, box_w=4, box_h=4,
                img_w=10, img_h=10)

            assert z == pytest.approx(1.0)
        finally:
            node.destroy_node()

    def test_invalid_depth_values_filtered_out_within_mask(self):
        node = _construct_with_params()
        try:
            depth = np.full((10, 10), 2.0, dtype=np.float32)
            mask = np.zeros((10, 10), dtype=np.uint8)
            mask[1:6, 1:6] = 1  # 25 pixels labeled
            # Salt the labeled region with every "invalid" convention this
            # codebase already uses elsewhere for ZED depth (see
            # _median_depth's own np.isfinite(...) & (roi > 0.0) filter):
            # NaN, +inf, exactly zero, and a negative value.
            depth[1, 1] = np.nan
            depth[1, 2] = np.inf
            depth[1, 3] = 0.0
            depth[1, 4] = -1.0
            # Every remaining labeled pixel stays at 2.0 -- if any of the
            # four invalid values leaked into the median this would not be
            # exactly 2.0.

            z = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=3, cy_px=3, box_w=5, box_h=5,
                img_w=10, img_h=10)

            assert z == pytest.approx(2.0)
        finally:
            node.destroy_node()

    def test_zero_valid_pixels_falls_back_to_box_region_method(self):
        node = _construct_with_params()
        try:
            depth = np.full((20, 20), 3.0, dtype=np.float32)
            mask = np.zeros((20, 20), dtype=np.uint8)
            mask[2:5, 2:5] = 1
            # The entire labeled footprint is unreadable depth (occluded/out
            # of stereo range) -- zero valid pixels after filtering.
            depth[2:5, 2:5] = np.nan

            expected = node._median_depth(
                depth, cx_px=10, cy_px=10, box_w=6, box_h=6, img_w=20, img_h=20)
            z = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=10, cy_px=10, box_w=6, box_h=6,
                img_w=20, img_h=20)

            assert expected is not None  # sanity: the box region IS readable
            assert z == pytest.approx(expected)
        finally:
            node.destroy_node()

    def test_no_mask_data_at_all_falls_back_like_zero_valid_pixels(self):
        """Same fallback path, reached the way it actually happens live: a
        mask image with no pixels at all carrying this detection's label
        (e.g. a detect-task model upstream that never publishes any
        masks_topic data -- see module docstring's 'safe to leave on' note)."""
        node = _construct_with_params()
        try:
            depth = np.full((20, 20), 4.0, dtype=np.float32)
            mask = np.zeros((20, 20), dtype=np.uint8)  # label 1 appears nowhere

            expected = node._median_depth(
                depth, cx_px=10, cy_px=10, box_w=6, box_h=6, img_w=20, img_h=20)
            z = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=10, cy_px=10, box_w=6, box_h=6,
                img_w=20, img_h=20)

            assert z == pytest.approx(expected)
        finally:
            node.destroy_node()

    def test_box_straddling_image_edge_still_falls_back_cleanly(self):
        """Combines the zero-valid-mask-pixel fallback with a box that hangs
        off the image edge (a detection near the frame boundary) -- exercises
        _median_depth's own clipping (img_w/img_h clamp) reached THROUGH the
        fallback path, not just in isolation."""
        node = _construct_with_params()
        try:
            depth = np.full((20, 20), 6.0, dtype=np.float32)
            mask = np.zeros((20, 20), dtype=np.uint8)  # no valid mask pixels

            # Box center right at the corner, half-hanging off both edges.
            z = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=1, cy_px=1, box_w=8, box_h=8,
                img_w=20, img_h=20)

            assert z == pytest.approx(6.0)  # clipped ROI still reads real depth
        finally:
            node.destroy_node()

    def test_mask_resolution_mismatch_is_nearest_neighbor_resized(self):
        """masks_topic at half the depth image's resolution (e.g. a config
        drift between the two ZED-published streams -- see module docstring's
        resize-on-mismatch paragraph) -- must still select the right region
        after an explicit nearest-neighbor resize, not silently misalign or
        blend label values across the upscale."""
        node = _construct_with_params()
        try:
            depth = np.full((20, 20), 7.0, dtype=np.float32)
            depth[8:12, 8:12] = 1.0  # a distinct patch in the depth image's OWN resolution

            # Half-resolution mask (10x10): label 1 covers what upscales to
            # roughly the same [8:12, 8:12] region at 20x20.
            small_mask = np.zeros((10, 10), dtype=np.uint8)
            small_mask[4:6, 4:6] = 1

            z = node._mask_median_depth(
                depth, small_mask, det_index=0, cx_px=10, cy_px=10, box_w=6,
                box_h=6, img_w=20, img_h=20)

            assert z == pytest.approx(1.0)
        finally:
            node.destroy_node()

    def test_second_detections_label_does_not_pick_up_first_detections_pixels(self):
        """Two adjacent instances in one mask image -- label 2 must sample
        only its own pixels, never label 1's, even though both sit in the
        same array."""
        node = _construct_with_params()
        try:
            depth = np.full((10, 10), 5.0, dtype=np.float32)
            mask = np.zeros((10, 10), dtype=np.uint8)
            mask[0:5, 0:10] = 1
            mask[5:10, 0:10] = 2
            depth[0:5, :] = 2.0
            depth[5:10, :] = 8.0

            z0 = node._mask_median_depth(
                depth, mask, det_index=0, cx_px=5, cy_px=2, box_w=8, box_h=4,
                img_w=10, img_h=10)
            z1 = node._mask_median_depth(
                depth, mask, det_index=1, cx_px=5, cy_px=7, box_w=8, box_h=4,
                img_w=10, img_h=10)

            assert z0 == pytest.approx(2.0)
            assert z1 == pytest.approx(8.0)
        finally:
            node.destroy_node()


class TestUseMaskDepthSyncWiring:
    """use_mask_depth's effect on __init__ (whether mask_sub/a 3-way sync
    exists at all) and on _synced_callback's per-detection method choice."""

    def test_use_mask_depth_false_never_creates_mask_subscriber(self):
        node = _construct_with_params({'use_mask_depth': False})
        try:
            assert node.mask_sub is None
        finally:
            node.destroy_node()

    def test_use_mask_depth_true_creates_mask_subscriber(self):
        node = _construct_with_params({'use_mask_depth': True})
        try:
            assert node.mask_sub is not None
        finally:
            node.destroy_node()

    def test_synced_callback_use_mask_depth_false_ignores_extra_arg_path(self):
        """2-way sync (use_mask_depth false): _synced_callback is only ever
        called with (det_msg, depth_msg) in production -- confirms that call
        shape alone reproduces box-region depth end to end."""
        node = _construct_with_params({'use_mask_depth': False})
        try:
            node._camera_info_callback(_fake_camera_info())
            node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform())

            depth = np.full((20, 20), 4.0, dtype=np.float32)
            det_arr = Detection2DArray()
            det_arr.detections.append(_fake_detection2d(10, 10, 6, 6))

            node._synced_callback(det_arr, _fake_depth_image(depth))

            node.det3d_pub.publish.assert_called_once()
            published = node.det3d_pub.publish.call_args[0][0]
            assert len(published.detections) == 1
            assert published.detections[0].bbox.center.position.z == pytest.approx(4.0)
        finally:
            node.destroy_node()

    def test_synced_callback_use_mask_depth_true_prefers_mask_over_box(self):
        """3-way sync (use_mask_depth true): crafts a scene where the mask
        region and the box-region method would disagree, and confirms the
        published z comes from the mask, not the box -- i.e. this is actually
        wired end to end, not a dead code path only reachable via direct
        _mask_median_depth() calls."""
        node = _construct_with_params({'use_mask_depth': True})
        try:
            node._camera_info_callback(_fake_camera_info())
            node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform())

            depth = np.full((20, 20), 9.0, dtype=np.float32)  # background/box-region value
            depth[8:12, 8:12] = 1.5  # the object's TRUE (mask) depth
            mask = np.zeros((20, 20), dtype=np.uint8)
            mask[8:12, 8:12] = 1

            det_arr = Detection2DArray()
            # A box centered on the same region, but wide enough that its
            # center_fraction-shrunk ROI still includes plenty of the 9.0
            # background -- the two methods must disagree for this test to
            # mean anything.
            det_arr.detections.append(_fake_detection2d(10, 10, 16, 16))

            node._synced_callback(det_arr, _fake_depth_image(depth), _fake_mask_image(mask))

            published = node.det3d_pub.publish.call_args[0][0]
            assert len(published.detections) == 1
            z = published.detections[0].bbox.center.position.z
            assert z == pytest.approx(1.5)
            assert z != pytest.approx(9.0)
        finally:
            node.destroy_node()

    def test_synced_callback_use_mask_depth_true_but_no_mask_msg_falls_back(self):
        """use_mask_depth true against an upstream detect-task model (never
        publishes masks_topic at all, so the synchronizer would never fire on
        3 topics together in production) -- exercised here directly by
        calling with mask_msg=None, confirming the safe-default claim in the
        module docstring: identical output to use_mask_depth false."""
        node = _construct_with_params({'use_mask_depth': True})
        try:
            node._camera_info_callback(_fake_camera_info())
            node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform())

            depth = np.full((20, 20), 4.0, dtype=np.float32)
            det_arr = Detection2DArray()
            det_arr.detections.append(_fake_detection2d(10, 10, 6, 6))

            node._synced_callback(det_arr, _fake_depth_image(depth), None)

            published = node.det3d_pub.publish.call_args[0][0]
            assert published.detections[0].bbox.center.position.z == pytest.approx(4.0)
        finally:
            node.destroy_node()

    def test_mask_index_matches_detection_array_order_not_confidence_filtered_index(self):
        """A low-confidence detection at index 0 (dropped before publish) must
        not shift index 1's mask label lookup -- `i` in _synced_callback is
        det_msg.detections' own enumerate index, unaffected by the `continue`
        for below-threshold detections (see detection_3d_node.py's own
        comment at the call site)."""
        node = _construct_with_params(
            {'use_mask_depth': True, 'confidence_threshold': 0.5})
        try:
            node._camera_info_callback(_fake_camera_info())
            node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform())

            depth = np.full((20, 20), 9.0, dtype=np.float32)
            depth[8:12, 8:12] = 2.5
            mask = np.zeros((20, 20), dtype=np.uint8)
            mask[8:12, 8:12] = 2  # label 2 -> det_msg.detections[1]

            det_arr = Detection2DArray()
            # index 0: below confidence_threshold, dropped -- but still
            # consumes index 0's mask label slot (which has no pixels here).
            det_arr.detections.append(_fake_detection2d(5, 5, 4, 4, score=0.1))
            # index 1: kept, and must read label 2's pixels (2.5), not label
            # 1's (nonexistent -> would wrongly fall back to box/background).
            det_arr.detections.append(_fake_detection2d(10, 10, 16, 16, score=0.9))

            node._synced_callback(det_arr, _fake_depth_image(depth), _fake_mask_image(mask))

            published = node.det3d_pub.publish.call_args[0][0]
            assert len(published.detections) == 1
            assert published.detections[0].bbox.center.position.z == pytest.approx(2.5)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
