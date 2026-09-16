"""obstacle_projector_node ground-disk radius: obstacle_radius_source and
obstacle_class_margin_m.

The bug: detection_3d_node puts the back-projected image HEIGHT in
bbox.size.y, and the projector sized the ground disk as max(size.x, size.y)/2,
so a standing person got a disk of half their height. "footprint" uses the
width; "legacy" keeps the old rule for comparison with pre-fix runs.

Same no-spin convention as test_detection_3d_node.py: real rclpy Nodes, callbacks
called directly, publishers and tf lookups mocked.

Run standalone: python3 -m pytest test/test_obstacle_projector_radius.py -v
"""

import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from f1tenth_perception.detection_3d_node import Detection3DNode
from f1tenth_perception.obstacle_projector_node import (
    DETECTION_3D_DEPTH_EXTENT_IS_MEASURED,
    ObstacleProjectorNode,
    obstacle_radius,
    parse_class_margins,
)

# Real-world sizes [m]. A standing adult and a chair whose backrest makes it
# taller than wide: both are the shape the legacy rule over-sizes.
PERSON_W, PERSON_H = 0.50, 1.75
CHAIR_W, CHAIR_H = 0.45, 0.85
DEPTH_PLACEHOLDER = 0.3  # detection_3d_node's default_depth_extent


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _identity_transform(parent='base_link', child='zed2_left_camera_frame'):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.rotation.w = 1.0
    return t


def _projector(params=None):
    overrides = [Parameter(k, value=v) for k, v in (params or {}).items()]
    node = ObstacleProjectorNode(parameter_overrides=overrides)
    node.obstacles_pub.publish = MagicMock()
    node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform())
    return node


def _det3d(width, height, class_id, x=2.0, y=0.0, z=0.5, depth=DEPTH_PLACEHOLDER):
    det = Detection3D()
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = 0.9
    det.results.append(hyp)
    det.bbox.center.position.x = float(x)
    det.bbox.center.position.y = float(y)
    det.bbox.center.position.z = float(z)
    det.bbox.center.orientation.w = 1.0
    det.bbox.size.x = float(width)
    det.bbox.size.y = float(height)
    det.bbox.size.z = float(depth)
    return det


def _project(node, detections):
    msg = Detection3DArray()
    msg.header.frame_id = 'zed2_left_camera_frame'
    msg.detections = list(detections)
    node._detections_callback(msg)
    return node.obstacles_pub.publish.call_args[0][0].obstacles


class TestObstacleRadiusRule:

    def test_person_footprint_is_half_width_legacy_is_half_height(self):
        size = (PERSON_W, PERSON_H, DEPTH_PLACEHOLDER)
        assert obstacle_radius(*size, 'footprint') == pytest.approx(0.25)
        assert obstacle_radius(*size, 'legacy') == pytest.approx(0.875)

    def test_chair_footprint_is_half_width_legacy_is_half_height(self):
        size = (CHAIR_W, CHAIR_H, DEPTH_PLACEHOLDER)
        assert obstacle_radius(*size, 'footprint') == pytest.approx(0.225)
        assert obstacle_radius(*size, 'legacy') == pytest.approx(0.425)

    def test_wider_than_tall_object_is_the_same_in_both_modes(self):
        # A low bench: height never won the max, so legacy was already right.
        size = (1.2, 0.45, DEPTH_PLACEHOLDER)
        assert obstacle_radius(*size, 'footprint') == pytest.approx(0.6)
        assert obstacle_radius(*size, 'legacy') == pytest.approx(0.6)

    def test_footprint_ignores_the_depth_placeholder_when_it_exceeds_the_width(self):
        # A pole 0.2 m wide: the 0.3 m placeholder would win a max() it has
        # no business entering.
        assert obstacle_radius(0.2, 1.8, DEPTH_PLACEHOLDER, 'footprint') == pytest.approx(0.1)

    def test_footprint_uses_depth_extent_only_when_it_is_measured(self):
        assert obstacle_radius(
            0.5, 1.75, 0.6, 'footprint', depth_extent_is_measured=True) == pytest.approx(0.3)
        assert obstacle_radius(
            0.5, 1.75, 0.6, 'footprint', depth_extent_is_measured=False) == pytest.approx(0.25)

    def test_depth_extent_is_declared_unmeasured(self):
        assert DETECTION_3D_DEPTH_EXTENT_IS_MEASURED is False

    def test_unknown_source_raises(self):
        with pytest.raises(ValueError, match='obstacle_radius_source'):
            obstacle_radius(0.5, 1.75, 0.3, 'height')


class TestDetection3DSizeFieldsFeedTheRule:
    """The coupling the fix rests on: detection_3d_node really does put width
    in size.x, height in size.y and a depth-independent placeholder in
    size.z. If that node changes, the rule above is wrong and this fails."""

    FX = FY = 500.0

    def _detect(self, z, w_m, h_m, depth_extent):
        node = Detection3DNode(parameter_overrides=[
            Parameter('use_mask_depth', value=False),
            Parameter('default_depth_extent', value=depth_extent)])
        try:
            node.det3d_pub.publish = MagicMock()
            node.marker_pub.publish = MagicMock()
            info = CameraInfo()
            info.k[0], info.k[4], info.k[2], info.k[5] = self.FX, self.FY, 320.0, 240.0
            node._camera_info_callback(info)
            node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform(
                parent='zed2_left_camera_frame', child='zed2_left_camera_optical_frame'))

            det = Detection2D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = 0.9
            det.results.append(hyp)
            det.bbox.center.position.x = 320.0
            det.bbox.center.position.y = 240.0
            det.bbox.size_x = w_m * self.FX / z
            det.bbox.size_y = h_m * self.FY / z
            dets = Detection2DArray()
            dets.detections.append(det)

            depth = CvBridge().cv2_to_imgmsg(
                np.full((480, 640), z, dtype=np.float32), encoding='32FC1')
            depth.header.frame_id = 'zed2_left_camera_optical_frame'
            node._synced_callback(dets, depth)
            return node.det3d_pub.publish.call_args[0][0].detections[0]
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('z', [2.0, 4.0])
    def test_size_x_is_width_size_y_is_height_size_z_is_the_placeholder(self, z):
        det = self._detect(z, PERSON_W, PERSON_H, depth_extent=DEPTH_PLACEHOLDER)
        assert det.bbox.size.x == pytest.approx(PERSON_W)
        assert det.bbox.size.y == pytest.approx(PERSON_H)
        assert det.bbox.size.z == pytest.approx(DEPTH_PLACEHOLDER)

    def test_size_z_follows_the_parameter_not_the_scene(self):
        det = self._detect(3.0, PERSON_W, PERSON_H, depth_extent=0.7)
        assert det.bbox.size.z == pytest.approx(0.7)

    def test_person_from_detection_3d_gets_width_radius_in_footprint_mode(self):
        det = self._detect(4.0, PERSON_W, PERSON_H, depth_extent=DEPTH_PLACEHOLDER)
        det.bbox.center.position.x, det.bbox.center.position.z = 4.0, 0.5
        node = _projector()
        try:
            (obstacle,) = _project(node, [det])
            assert obstacle.r == pytest.approx(0.25)
        finally:
            node.destroy_node()


class TestProjectorNodeRadius:

    @pytest.mark.parametrize('source, expected', [('footprint', 0.25), ('legacy', 0.875)])
    def test_person_disk_per_mode(self, source, expected):
        node = _projector({'obstacle_radius_source': source})
        try:
            (obstacle,) = _project(node, [_det3d(PERSON_W, PERSON_H, 'person')])
            assert obstacle.r == pytest.approx(expected)
        finally:
            node.destroy_node()

    @pytest.mark.parametrize('source, expected', [('footprint', 0.225), ('legacy', 0.425)])
    def test_chair_disk_per_mode(self, source, expected):
        node = _projector({'obstacle_radius_source': source})
        try:
            (obstacle,) = _project(node, [_det3d(CHAIR_W, CHAIR_H, 'chair')])
            assert obstacle.r == pytest.approx(expected)
        finally:
            node.destroy_node()

    def test_default_mode_is_footprint(self):
        node = _projector()
        try:
            assert node.obstacle_radius_source == 'footprint'
        finally:
            node.destroy_node()

    def test_unknown_mode_fails_at_startup(self):
        with pytest.raises(ValueError, match='obstacle_radius_source'):
            ObstacleProjectorNode(
                parameter_overrides=[Parameter('obstacle_radius_source', value='height')])

    def test_tall_person_rejected_by_max_radius_only_in_legacy(self):
        # 3.2 m tall -> legacy r 1.6 > max_obstacle_radius 1.5: the whole
        # obstacle vanished under the old rule. The reject filter is unchanged.
        det = _det3d(0.5, 3.2, 'person')
        legacy = _projector({'obstacle_radius_source': 'legacy'})
        footprint = _projector({'obstacle_radius_source': 'footprint'})
        try:
            assert _project(legacy, [det]) == []
            assert [o.r for o in _project(footprint, [det])] == [pytest.approx(0.25)]
        finally:
            legacy.destroy_node()
            footprint.destroy_node()

    def test_narrow_object_below_min_radius_rejected_in_footprint_kept_in_legacy(self):
        # A 5 cm wide, 25 cm tall bottle: footprint r 0.025 < min 0.03.
        det = _det3d(0.05, 0.25, 'bottle')
        legacy = _projector({'obstacle_radius_source': 'legacy'})
        footprint = _projector({'obstacle_radius_source': 'footprint'})
        try:
            assert [o.r for o in _project(legacy, [det])] == [pytest.approx(0.125)]
            assert _project(footprint, [det]) == []
        finally:
            legacy.destroy_node()
            footprint.destroy_node()


class TestClassMarginParsing:

    @pytest.mark.parametrize('text', ['{}', '', '   '])
    def test_empty_means_no_margins(self, text):
        assert parse_class_margins(text) == {}

    def test_valid_map(self):
        assert parse_class_margins('{"person": 0.3, "chair": 0}') == {
            'person': 0.3, 'chair': 0.0}

    @pytest.mark.parametrize('text, match', [
        ('{person: 0.3}', 'not valid JSON'),
        ('[0.3]', 'JSON object'),
        ('{"person": "0.3"}', 'must be a number'),
        ('{"person": true}', 'must be a number'),
        ('{"person": -0.1}', '>= 0'),
        ('{"person": NaN}', 'finite'),
        ('{"person": Infinity}', 'finite'),
    ])
    def test_invalid_values_raise(self, text, match):
        with pytest.raises(ValueError, match=match):
            parse_class_margins(text)


class TestClassMarginInProjector:

    @pytest.mark.parametrize('source, base', [('footprint', 0.25), ('legacy', 0.875)])
    def test_margin_is_added_once_in_both_modes(self, source, base):
        node = _projector({'obstacle_radius_source': source,
                           'obstacle_class_margin_m': '{"person": 0.3}'})
        try:
            (obstacle,) = _project(node, [_det3d(PERSON_W, PERSON_H, 'person')])
            assert obstacle.r == pytest.approx(base + 0.3)
        finally:
            node.destroy_node()

    def test_unlisted_class_gets_no_margin(self):
        node = _projector({'obstacle_class_margin_m': '{"person": 0.3}'})
        try:
            (obstacle,) = _project(node, [_det3d(CHAIR_W, CHAIR_H, 'chair')])
            assert obstacle.r == pytest.approx(0.225)
        finally:
            node.destroy_node()

    def test_default_is_no_margin(self):
        node = _projector()
        try:
            assert node.obstacle_class_margin_m == {}
        finally:
            node.destroy_node()

    def test_margin_does_not_rescue_a_box_below_min_radius(self):
        node = _projector({'obstacle_class_margin_m': '{"bottle": 0.3}'})
        try:
            assert _project(node, [_det3d(0.04, 0.25, 'bottle')]) == []
        finally:
            node.destroy_node()

    def test_margin_can_carry_a_plausible_object_past_max_radius(self):
        # The filter judges the measured size (1.4 <= 1.5); the clearance
        # asked for around it is not a plausibility question.
        node = _projector({'obstacle_class_margin_m': '{"person": 0.3}'})
        try:
            (obstacle,) = _project(node, [_det3d(2.8, 1.75, 'person')])
            assert obstacle.r == pytest.approx(1.7)
        finally:
            node.destroy_node()

    def test_merge_keeps_the_margined_radius(self):
        # Two person boxes 10 cm apart merge; the merged disk is the larger
        # margined one, so the margin is neither dropped nor added twice.
        node = _projector({'obstacle_class_margin_m': '{"person": 0.3}'})
        try:
            (obstacle,) = _project(node, [
                _det3d(PERSON_W, PERSON_H, 'person', y=0.0),
                _det3d(0.4, PERSON_H, 'person', y=0.1)])
            assert obstacle.r == pytest.approx(0.55)
        finally:
            node.destroy_node()

    def test_invalid_margin_fails_at_startup(self):
        with pytest.raises(ValueError, match='obstacle_class_margin_m'):
            ObstacleProjectorNode(parameter_overrides=[
                Parameter('obstacle_class_margin_m', value='{"person": -1}')])


def test_math_person_r_safe_example():
    """Worked example used in the stack_params.yaml note: footprint person
    plus the default car_radius 0.20 and avoidance_margin 0.12."""
    r = obstacle_radius(PERSON_W, PERSON_H, DEPTH_PLACEHOLDER, 'footprint')
    assert math.isclose(r + 0.20 + 0.12, 0.57)
