"""semantic_layer_node.py tests -- same convention test_costmap_boundary_
node.py already established: construct a real (never spun) rclpy Node via
parameter_overrides and call its callbacks directly, publishers mocked so
each test can assert exactly what would have been published, no live
topics/TF/hardware needed.

Covers what's actually new at the ROS-glue layer for the real per-frame
tracking pass (semantic_layer.py's own test file covers update_tracks_batch/
SemanticObject in isolation, pure-function level):
  - _publish() only emits markers for CONFIRMED tracks.
  - DELETE-diffing when a previously-confirmed track drops out (pruned).
  - marker id is the track's own track_id (position-independent), not the
    old hash(class_id)+list-index scheme.
  - an empty Detection3DArray still ticks the miss lifecycle (does not just
    return early and do nothing), without needing a live tf2/pose lookup.

tf2/pose plumbing itself (the two-hop transform) is exercised indirectly via
_detections_cb's real code path, using a stub tf2 buffer -- not re-testing
the transform MATH here (compose_base_link_to_map/pose_to_xytheta already
have their own dedicated tests in test_semantic_layer.py).

Run standalone: python3 -m pytest test/test_semantic_layer_node.py -v
"""

from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
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


def _construct_with_params(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = SemanticLayerNode(parameter_overrides=overrides)
    node.marker_pub.publish = MagicMock()
    # Identity camera_frame -> base_frame transform, always available --
    # isolates these tests from real TF (see module docstring: the
    # transform MATH itself is covered elsewhere).
    node.tf_buffer.lookup_transform = MagicMock(return_value=_identity_transform_stamped())
    return node


def _identity_transform_stamped():
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    return ts


def _fake_pose_msg(x, y):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def _fake_detections(stamp_sec, entries):
    """entries: list of (class_id, x, y, z, score) in camera frame."""
    msg = Detection3DArray()
    msg.header.frame_id = 'zed2_left_camera_frame'
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
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


class TestConfirmedOnlyPublish:

    def test_unconfirmed_track_publishes_no_markers(self):
        node = _construct_with_params({'confirm_hit_count': 3, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('cone', 1.0, 0.0, 0.0, 0.9)]))

            node.marker_pub.publish.assert_called_once()
            published = node.marker_pub.publish.call_args[0][0]
            assert list(published.markers) == []
        finally:
            node.destroy_node()

    def test_confirmed_after_n_hits_publishes_disk_and_label(self):
        node = _construct_with_params({
            'confirm_hit_count': 2, 'merge_distance_m': 5.0, 'score_threshold': 0.0,
        })
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('cone', 1.0, 0.0, 0.0, 0.9)]))
            node._detections_cb(_fake_detections(2.0, [('cone', 1.1, 0.0, 0.0, 0.9)]))

            published = node.marker_pub.publish.call_args[0][0]
            types = sorted(m.type for m in published.markers)
            assert len(published.markers) == 2  # disk + label, same track
            from visualization_msgs.msg import Marker
            assert types == sorted([Marker.CYLINDER, Marker.TEXT_VIEW_FACING])
        finally:
            node.destroy_node()


class TestMarkerIdIsTrackId:

    def test_marker_id_matches_track_id_not_list_position(self):
        node = _construct_with_params({'confirm_hit_count': 1, 'score_threshold': 0.0})
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(
                1.0, [('cone', 1.0, 0.0, 0.0, 0.9), ('cone', 20.0, 0.0, 0.0, 0.9)]))

            published = node.marker_pub.publish.call_args[0][0]
            from visualization_msgs.msg import Marker
            disk_ids = {m.id for m in published.markers if m.type == Marker.CYLINDER}
            track_ids = {t.track_id for t in node._objects}
            assert disk_ids == track_ids
        finally:
            node.destroy_node()


class TestDeleteDiffingOnPrune:

    def test_pruned_confirmed_track_emits_delete_markers(self):
        node = _construct_with_params({
            'confirm_hit_count': 1, 'lost_miss_count': 1, 'score_threshold': 0.0,
        })
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('cone', 1.0, 0.0, 0.0, 0.9)]))
            first_published = node.marker_pub.publish.call_args[0][0]
            assert len(first_published.markers) == 2  # disk + label, confirmed on first hit
            track_id = node._objects[0].track_id

            # Next frame: no detections at all -> the one track misses once
            # and (lost_miss_count=1) is pruned immediately.
            node._detections_cb(_fake_detections(2.0, []))

            assert node._objects == []
            second_published = node.marker_pub.publish.call_args[0][0]
            from visualization_msgs.msg import Marker
            assert len(second_published.markers) == 2  # DELETE for disk + label
            assert all(m.action == Marker.DELETE for m in second_published.markers)
            assert {m.id for m in second_published.markers} == {track_id}
            assert {m.ns for m in second_published.markers} == {'cone', 'cone_label'}
        finally:
            node.destroy_node()


class TestEmptyBatchStillTicksLifecycle:

    def test_empty_detections_message_still_publishes_and_ticks_misses(self):
        """Regression guard: the old code's `if not msg.detections: return`
        skipped empty messages ENTIRELY -- under the new lifecycle, an empty
        frame is itself a real "nothing matched" signal that must still
        tick miss_streak (see module docstring), not be silently dropped."""
        node = _construct_with_params({
            'confirm_hit_count': 1, 'lost_miss_count': 5, 'score_threshold': 0.0,
        })
        try:
            node._pose_cb(_fake_pose_msg(0.0, 0.0))
            node._detections_cb(_fake_detections(1.0, [('cone', 1.0, 0.0, 0.0, 0.9)]))
            assert node._objects[0].miss_streak == 0

            node.marker_pub.publish.reset_mock()
            node._detections_cb(_fake_detections(2.0, []))  # empty this frame

            node.marker_pub.publish.assert_called_once()  # did NOT return early / skip
            assert len(node._objects) == 1  # not yet pruned (1 miss < lost_miss_count=5)
            assert node._objects[0].miss_streak == 1
        finally:
            node.destroy_node()

    def test_empty_detections_before_any_pose_does_not_crash(self):
        """No tf2/pose lookup needed for the miss-tick path -- an empty
        message must not require /slam/pose to have arrived yet, unlike a
        real (non-empty) batch which does (see module docstring)."""
        node = _construct_with_params({'score_threshold': 0.0})
        try:
            node._detections_cb(_fake_detections(1.0, []))  # no _pose_cb call at all
            node.marker_pub.publish.assert_called_once()
        finally:
            node.destroy_node()


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
