"""/costmap/semantic_tracks carries the detection batch's CAPTURE stamp.

Same convention as test_semantic_layer_node.py: a real, never-spun rclpy Node
built with parameter_overrides, callbacks called directly, publishers mocked.
No live topics, no TF, no hardware.

WHY THIS IS WORTH PINNING. The tracks topic was added so something could
drive at a tracked object. Its header.stamp used to be the clock at publish,
which reported every track as zero-age no matter how far behind the detection
pipeline had fallen -- the detector's inference, the two-hop transform and the
association pass are all latency this node knows about and a consumer cannot
reconstruct. A downstream staleness check against a publish-time stamp can
only measure its own receive gap, so a track a second old and a fresh one look
identical, and the car keeps steering at where a person was.

The markers keep the current clock, deliberately, and that asymmetry is
asserted here too so a later "make the stamps consistent" tidy-up has to
argue with a test rather than quietly undo this.
"""

from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from f1tenth_costmap.semantic_layer_node import SemanticLayerNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _identity_transform_stamped():
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    return ts


def _construct(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = SemanticLayerNode(parameter_overrides=overrides)
    node.marker_pub.publish = MagicMock()
    node.tracks_pub.publish = MagicMock()
    node.tf_buffer.lookup_transform = MagicMock(
        return_value=_identity_transform_stamped())
    return node


def _fake_pose_msg(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def _fake_detections(stamp_sec, entries):
    """entries: list of (class_id, x, y, z, score) in camera frame."""
    msg = Detection3DArray()
    msg.header.frame_id = 'zed2_left_camera_frame'
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
    for class_id, x, y, z, score in entries:
        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis(class_id=class_id, score=score)
        det.results.append(hyp)
        det.bbox.center.position.x = x
        det.bbox.center.position.y = y
        det.bbox.center.position.z = z
        det.bbox.center.orientation.w = 1.0
        msg.detections.append(det)
    return msg


def _last_tracks(node):
    return node.tracks_pub.publish.call_args[0][0]


class TestTracksCarryTheCaptureStamp:

    def test_published_stamp_equals_the_input_detections_stamp(self):
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            detections = _fake_detections(
                1234.5, [('person', 2.0, 0.0, 0.0, 0.9)])
            node._detections_cb(detections)

            published = _last_tracks(node)
            assert published.header.stamp.sec == detections.header.stamp.sec
            assert published.header.stamp.nanosec == detections.header.stamp.nanosec
        finally:
            node.destroy_node()

    def test_an_empty_batch_also_carries_its_own_stamp(self):
        """The empty-frame path is a separate _publish() call site, and it is
        the most common one -- a frame with nothing in it still ticks the miss
        lifecycle, so it still publishes."""
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(7.25, [('person', 2.0, 0.0, 0.0, 0.9)]))
            empty = _fake_detections(8.5, [])
            node._detections_cb(empty)

            published = _last_tracks(node)
            assert published.header.stamp.sec == empty.header.stamp.sec
            assert published.header.stamp.nanosec == empty.header.stamp.nanosec
        finally:
            node.destroy_node()

    def test_the_stamp_advances_with_the_batches_not_with_wall_clock(self):
        """Three batches, three distinct capture stamps, in order."""
        node = _construct({
            'confirm_hit_count': 1, 'score_threshold': 0.0, 'merge_distance_m': 5.0,
        })
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            wanted = [10.0, 10.08, 10.16]
            seen = []
            for stamp in wanted:
                node._detections_cb(
                    _fake_detections(stamp, [('person', 2.0, 0.0, 0.0, 0.9)]))
                header = _last_tracks(node).header.stamp
                seen.append(header.sec + header.nanosec * 1e-9)
            for got, want in zip(seen, wanted):
                assert got == pytest.approx(want, abs=1e-6)
        finally:
            node.destroy_node()

    def test_a_stale_batch_is_reported_stale_not_fresh(self):
        """The failure mode in one assertion: a capture stamp well behind the
        node's own clock must survive to the wire. Under the old publish-time
        stamp this difference was zero by construction."""
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('person', 2.0, 0.0, 0.0, 0.9)]))

            published = _last_tracks(node)
            stamped = published.header.stamp.sec + published.header.stamp.nanosec * 1e-9
            node_now = node.get_clock().now().nanoseconds * 1e-9
            assert node_now - stamped > 1.0, (
                'the published stamp tracks the node clock, not the capture '
                'time -- a stale track would report as fresh')
        finally:
            node.destroy_node()


class TestMarkersKeepTheCurrentClock:

    def test_marker_stamp_is_not_the_capture_stamp(self):
        """Deliberate asymmetry: a marker's stamp is what RViz resolves its TF
        against, so back-dating it would make every marker look stale to a
        viewer that has since moved."""
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('person', 2.0, 0.0, 0.0, 0.9)]))

            markers = node.marker_pub.publish.call_args[0][0].markers
            assert markers, 'expected a confirmed track to produce markers'
            for marker in markers:
                stamped = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9
                assert stamped > 1.0 + 1e-6
        finally:
            node.destroy_node()


class TestFrameIsUnchanged:

    def test_tracks_are_still_published_in_the_map_frame(self):
        """Guard: this pass touched the header, and the frame lives there too."""
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('person', 2.0, 0.0, 0.0, 0.9)]))
            assert _last_tracks(node).header.frame_id == node.map_frame
        finally:
            node.destroy_node()

    def test_each_detection_header_matches_the_message_header(self):
        """Detection3D carries its own header; it is set from the array's."""
        node = _construct({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(42.0, [('person', 2.0, 0.0, 0.0, 0.9)]))
            published = _last_tracks(node)
            assert published.detections, 'expected one confirmed track'
            for det in published.detections:
                assert det.header.stamp == published.header.stamp
                assert det.header.frame_id == published.header.frame_id
        finally:
            node.destroy_node()
