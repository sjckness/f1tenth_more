"""detection_3d_node position-uncertainty tests -- the covariance added by the
2026-09-01 mission-analysis follow-up (see mission_analysis_2026-09-01.md).

Same "construct a real but never-spun rclpy Node and call its methods
directly" convention as test_detection_3d_node.py in this directory (see that
file's own module docstring).

WHY THIS MATTERS: before this pass every detection on /camera/detections_3d
arrived equally precise, so semantic_layer_node's tracker blended a
low-confidence detection measured at 0.143-0.249 m frame-to-frame noise with
exactly the same weight as a high-confidence one at 0.033-0.121 m. The
covariance published here is what lets the tracker tell them apart.

Run standalone: python3 -m pytest test/test_detection_3d_uncertainty.py -v
"""

import math

import pytest
import rclpy
from rclpy.parameter import Parameter

from f1tenth_perception.detection_3d_node import Detection3DNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node(**params):
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return Detection3DNode(parameter_overrides=overrides)


class TestPositionSigma:

    def test_high_confidence_close_range_gets_the_base_sigma(self):
        n = _node(position_sigma_base_m=0.05, position_sigma_range_coeff=0.0,
                  low_conf_sigma_scale=4.0)
        assert n._position_sigma(1.0, 0.0) == pytest.approx(0.05)

    def test_low_confidence_is_scaled_up(self):
        """The measured 2-7x confidence-dependent spread is what this encodes."""
        n = _node(confidence_threshold=0.3, position_sigma_base_m=0.05,
                  position_sigma_range_coeff=0.0, low_conf_sigma_scale=4.0)
        assert n._position_sigma(0.3, 0.0) == pytest.approx(0.20)

    def test_sigma_is_monotonic_in_confidence(self):
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.0)
        sigmas = [n._position_sigma(s, 1.0) for s in (0.3, 0.5, 0.7, 0.9, 1.0)]
        assert sigmas == sorted(sigmas, reverse=True)

    def test_sigma_grows_with_range(self):
        """Standard stereo behaviour: depth noise grows with distance."""
        n = _node(position_sigma_base_m=0.05, position_sigma_range_coeff=0.02)
        assert n._position_sigma(1.0, 5.0) > n._position_sigma(1.0, 1.0)
        assert n._position_sigma(1.0, 5.0) == pytest.approx(0.05 + 0.02 * 5.0)

    def test_score_below_threshold_is_clamped_not_extrapolated(self):
        """Sub-threshold detections are dropped by the caller; if one ever
        reaches here it must not produce a wilder sigma than the worst
        legitimate one."""
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.0)
        assert n._position_sigma(0.0, 0.0) == pytest.approx(n._position_sigma(0.3, 0.0))

    def test_score_above_one_is_clamped(self):
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.0)
        assert n._position_sigma(1.5, 0.0) == pytest.approx(n._position_sigma(1.0, 0.0))

    def test_negative_range_does_not_reduce_sigma_below_base(self):
        n = _node(position_sigma_base_m=0.05, position_sigma_range_coeff=0.02)
        assert n._position_sigma(1.0, -3.0) == pytest.approx(0.05)

    def test_degenerate_threshold_does_not_divide_by_zero(self):
        """confidence_threshold >= 1.0 leaves no range to interpolate over."""
        n = _node(confidence_threshold=1.0, position_sigma_base_m=0.05,
                  position_sigma_range_coeff=0.0)
        assert n._position_sigma(1.0, 0.0) == pytest.approx(0.05)


class TestCovarianceOnPublishedDetection:

    @staticmethod
    def _cov(node, score, z):
        from geometry_msgs.msg import Pose
        from std_msgs.msg import Header
        det = node._build_detection3d(
            header=Header(frame_id='zed2_left_camera_frame'), pose=Pose(),
            width=0.4, height=0.4, class_id='person', score=score, z=z)
        return det.results[0].pose.covariance

    def test_position_diagonal_is_populated(self):
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.0,
                  position_sigma_base_m=0.05, low_conf_sigma_scale=4.0)
        cov = self._cov(n, 1.0, 2.0)
        assert cov[0] == pytest.approx(0.05 ** 2)
        assert cov[7] == pytest.approx(0.05 ** 2)
        assert cov[14] > cov[0]  # z extent is a placeholder, not a measurement

    def test_low_confidence_detection_reports_larger_variance(self):
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.0)
        assert self._cov(n, 0.35, 2.0)[0] > self._cov(n, 0.95, 2.0)[0]

    def test_rotation_block_left_unestimated(self):
        """This node estimates no orientation at all -- the pose's rotation is
        a fixed frame transform, not a measurement. Inventing a rotation
        variance would be worse than leaving it at 0.0."""
        n = _node()
        cov = self._cov(n, 0.9, 2.0)
        assert cov[21] == 0.0 and cov[28] == 0.0 and cov[35] == 0.0

    def test_variance_round_trips_to_the_sigma_the_tracker_reads(self):
        """semantic_layer_node reads sqrt(max(cov[0], cov[7])) -- this pins
        that contract end to end."""
        n = _node(confidence_threshold=0.3, position_sigma_range_coeff=0.02,
                  position_sigma_base_m=0.05, low_conf_sigma_scale=4.0)
        cov = self._cov(n, 0.6, 3.0)
        assert math.sqrt(max(cov[0], cov[7])) == pytest.approx(
            n._position_sigma(0.6, 3.0))
