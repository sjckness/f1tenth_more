"""front_clearance_node.py tests -- constructs a real (but never spun) rclpy
Node and calls its methods directly, same "no live topics/hardware needed,
pure in-process method calls" convention as this package's own
test/test_yolo_detector_node.py and test/test_detection_3d_node.py (see those
files' module docstrings). Publishers are never actually sent over DDS here
(no spin() anywhere in this file) -- publish() is mocked so tests can assert
exactly what would have been published.

WHAT THIS FILE IS ACTUALLY DEFENDING. The node exists because a previous
front-distance signal published raw per-frame values and flapped. Almost
every test below therefore asserts a NON-flip or a HOLD -- that some input
which would have moved a naive implementation does NOT move this one:

  * a value sitting inside the dead band does not flip the latch
  * a single outlier frame does not flip the latch even when it clears the
    threshold outright
  * a frame with no usable data holds the EMA rather than seeding it, and
    resets the dwell counter rather than counting toward a flip
  * a dropped detection decays the object exclusion instead of deleting it

Those are the regressions worth catching, and each of them is a case where
the WRONG implementation still produces plausible-looking output on a live
robot -- which is exactly how the original version shipped.

The two filter classes are pure (no rclpy dependency at all) and tested
directly rather than through the node, same discipline as
yolo_detector_node.py's own _bbox_overlap_fraction/_mask_overlap_fraction.

Run standalone: python3 -m pytest test/test_front_clearance_node.py -v
"""

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from unittest.mock import MagicMock
from vision_msgs.msg import Detection2D, Detection2DArray

from f1tenth_messages.msg import Obstacle2D, Obstacle2DArray

from f1tenth_perception.front_clearance_node import (
    EmaFilter,
    FrontClearanceNode,
    HysteresisLatch,
    _stamps_equal,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = FrontClearanceNode(parameter_overrides=overrides)
    node.front_distance_pub.publish = MagicMock()
    node.front_wall_pub.publish = MagicMock()
    node.front_clearance_pub.publish = MagicMock()
    node.front_blocked_pub.publish = MagicMock()
    if node.front_distance_raw_pub is not None:
        node.front_distance_raw_pub.publish = MagicMock()
    if node.bg_pixel_count_pub is not None:
        node.bg_pixel_count_pub.publish = MagicMock()
    return node


def _depth_msg(array, stamp_sec=1, stamp_nsec=0):
    msg = CvBridge().cv2_to_imgmsg(array.astype(np.float32), encoding='32FC1')
    msg.header = Header()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    return msg


def _detections(boxes, stamp_sec=1, stamp_nsec=0):
    """boxes: iterable of (cx, cy, w, h) in pixels."""
    msg = Detection2DArray()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    for cx, cy, w, h in boxes:
        det = Detection2D()
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        msg.detections.append(det)
    return msg


def _mask_msg(label_img, stamp_sec=1, stamp_nsec=0):
    msg = CvBridge().cv2_to_imgmsg(label_img.astype(np.uint8), encoding='mono8')
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    return msg


def _obstacles(items, frame='base_link'):
    """items: iterable of (x, y, r) in metres."""
    msg = Obstacle2DArray()
    msg.header.frame_id = frame
    for x, y, r in items:
        obs = Obstacle2D()
        obs.x, obs.y, obs.r = float(x), float(y), float(r)
        msg.obstacles.append(obs)
    return msg


# ==============================================================================
# EmaFilter
# ==============================================================================

class TestEmaFilter:
    def test_starts_empty_so_callers_can_tell_no_reading_from_a_reading(self):
        assert EmaFilter(0.25).value is None

    def test_first_sample_seeds_rather_than_blending_against_zero(self):
        # Blending the first sample against an implicit 0.0 start would make
        # the topic's first published value a fraction of the true distance --
        # a very-close reading, which is the dangerous direction to be wrong in.
        f = EmaFilter(0.25)
        assert f.update(4.0) == pytest.approx(4.0)
        assert f.value == pytest.approx(4.0)

    def test_blends_at_alpha_after_seeding(self):
        f = EmaFilter(0.25)
        f.update(4.0)
        assert f.update(8.0) == pytest.approx(0.25 * 8.0 + 0.75 * 4.0)

    def test_none_holds_the_last_value_and_does_not_decay_it(self):
        # THE point of this class: "no measurement" must not move the estimate.
        f = EmaFilter(0.25)
        f.update(4.0)
        for _ in range(50):
            assert f.update(None) == pytest.approx(4.0)
        assert f.value == pytest.approx(4.0)

    def test_none_before_any_sample_stays_none_rather_than_seeding_zero(self):
        f = EmaFilter(0.25)
        assert f.update(None) is None
        assert f.value is None

    def test_reset_clears_back_to_no_reading(self):
        f = EmaFilter(0.5)
        f.update(2.0)
        f.reset()
        assert f.value is None


# ==============================================================================
# HysteresisLatch
# ==============================================================================

class TestHysteresisLatchDeadBand:
    def test_value_inside_the_band_holds_the_current_state_both_ways(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=1, initial=False)
        assert latch.update(100) is False       # band, held at False
        latch.update(200)
        assert latch.state is True
        assert latch.update(100) is True        # same band value, now holds True

    def test_enter_threshold_is_inclusive(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=1, initial=False)
        assert latch.update(150) is True

    def test_exit_threshold_is_inclusive(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=1, initial=True)
        assert latch.update(50) is False


class TestHysteresisLatchDwell:
    def test_single_outlier_frame_does_not_flip_even_far_past_the_threshold(self):
        # The dead band alone would flip here -- 10000 is nowhere near the
        # band. Only the dwell counter stops it, which is why both exist.
        latch = HysteresisLatch(150, 50, min_dwell_frames=3, initial=False)
        assert latch.update(10000) is False
        assert latch.update(0) is False
        assert latch.state is False

    def test_flip_commits_exactly_on_the_nth_agreeing_frame(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=3, initial=False)
        assert latch.update(200) is False   # 1
        assert latch.update(200) is False   # 2
        assert latch.update(200) is True    # 3 -- commits

    def test_a_disagreeing_frame_restarts_the_count_from_one(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=3, initial=False)
        latch.update(200)
        latch.update(200)
        latch.update(0)      # proposes False == current state, resets counter
        latch.update(200)    # counting starts over
        latch.update(200)
        assert latch.state is False
        assert latch.update(200) is True

    def test_min_dwell_frames_below_one_is_clamped_not_disabled(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=0, initial=False)
        assert latch.min_dwell_frames == 1
        assert latch.update(200) is True

    def test_counter_resets_after_committing_so_the_next_flip_pays_full_dwell(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=2, initial=False)
        latch.update(200)
        assert latch.update(200) is True
        assert latch.update(0) is True      # one frame of False is not enough
        assert latch.update(0) is False


class TestHysteresisLatchNone:
    def test_none_holds_state_and_does_not_count_toward_a_flip(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=2, initial=False)
        latch.update(200)                  # 1 toward True
        assert latch.update(None) is False  # gap: resets the counter
        assert latch.update(200) is False   # so this is 1 again, not 2
        assert latch.update(200) is True

    def test_a_long_gap_never_flips_the_state_by_itself(self):
        latch = HysteresisLatch(150, 50, min_dwell_frames=1, initial=True)
        for _ in range(100):
            assert latch.update(None) is True


class TestHysteresisLatchInverted:
    """higher_enters=False -- the distance case, where SMALLER is triggered."""

    def test_small_distance_enters_and_large_exits(self):
        latch = HysteresisLatch(0.5, 0.7, min_dwell_frames=1,
                                initial=False, higher_enters=False)
        assert latch.update(0.4) is True
        assert latch.update(0.8) is False

    def test_band_between_enter_and_exit_holds(self):
        latch = HysteresisLatch(0.5, 0.7, min_dwell_frames=1,
                                initial=False, higher_enters=False)
        assert latch.update(0.6) is False
        latch.update(0.4)
        assert latch.state is True
        assert latch.update(0.6) is True

    def test_a_single_close_frame_does_not_trip_blocked_with_dwell_two(self):
        latch = HysteresisLatch(0.5, 0.7, min_dwell_frames=2,
                                initial=False, higher_enters=False)
        assert latch.update(0.1) is False
        assert latch.update(5.0) is False


def test_stamps_equal_is_exact_not_tolerant():
    a = Header()
    a.stamp.sec, a.stamp.nanosec = 5, 100
    b = Header()
    b.stamp.sec, b.stamp.nanosec = 5, 101
    assert not _stamps_equal(a.stamp, b.stamp)
    b.stamp.nanosec = 100
    assert _stamps_equal(a.stamp, b.stamp)


# ==============================================================================
# Background distance
# ==============================================================================

class TestRobustBackgroundDistance:
    def test_empty_sample_is_none_not_zero(self):
        node = _construct()
        assert node._robust_background_distance(np.array([], dtype=np.float32)) is None
        node.destroy_node()

    def test_trims_the_far_tail_so_a_gap_does_not_pull_the_wall_backwards(self):
        # 90 pixels of wall at 2.0 m, 10 pixels of open doorway at 40 m. A
        # bare mean lands ~5.8 m; the 5-50 percentile band keeps the wall.
        node = _construct()
        depths = np.array([2.0] * 90 + [40.0] * 10, dtype=np.float32)
        assert node._robust_background_distance(depths) == pytest.approx(2.0)
        node.destroy_node()

    def test_trims_near_field_speckle_a_bare_minimum_would_latch_onto(self):
        node = _construct()
        depths = np.array([0.05] + [3.0] * 99, dtype=np.float32)
        got = node._robust_background_distance(depths)
        assert got == pytest.approx(3.0, abs=0.05)
        assert got > 1.0
        node.destroy_node()

    def test_degenerate_all_equal_sample_falls_back_instead_of_returning_none(self):
        node = _construct()
        depths = np.full(20, 1.5, dtype=np.float32)
        assert node._robust_background_distance(depths) == pytest.approx(1.5)
        node.destroy_node()


# ==============================================================================
# Car's-own-LiDAR exclusion cut-out
# ==============================================================================

class TestLidarExclusionRoiMask:
    def test_marks_only_the_configured_corner_of_the_roi(self):
        node = _construct({
            'lidar_exclusion_x_min': 0.5, 'lidar_exclusion_x_max': 1.0,
            'lidar_exclusion_y_min': 0.5, 'lidar_exclusion_y_max': 1.0,
        })
        # Whole image as the ROI: bottom-right quadrant must be True.
        mask = node._lidar_exclusion_roi_mask((100, 100), (0, 100, 0, 100))
        assert mask.shape == (100, 100)
        assert mask[75, 75]
        assert not mask[25, 25]
        assert not mask[25, 75]
        assert not mask[75, 25]
        node.destroy_node()

    def test_a_rectangle_outside_the_roi_marks_nothing(self):
        node = _construct({
            'lidar_exclusion_x_min': 0.9, 'lidar_exclusion_x_max': 1.0,
            'lidar_exclusion_y_min': 0.9, 'lidar_exclusion_y_max': 1.0,
        })
        # ROI is the top-left corner only; the rectangle is bottom-right.
        mask = node._lidar_exclusion_roi_mask((100, 100), (0, 20, 0, 20))
        assert mask.shape == (20, 20)
        assert not mask.any()
        node.destroy_node()

    def test_housing_pixels_are_excluded_from_the_distance_estimate(self):
        # THE reason this cut exists: the housing returns real, valid, very
        # small depths, so a low-percentile estimate would report it as the
        # wall. Whole image is 5 m; the housing corner reads 0.1 m.
        node = _construct({
            'lidar_exclusion_x_min': 0.5, 'lidar_exclusion_x_max': 1.0,
            'lidar_exclusion_y_min': 0.5, 'lidar_exclusion_y_max': 1.0,
            'roi_half_width_px': 50, 'roi_half_height_px': 50,
            'min_bg_pixels_for_reading': 1,
        })
        depth = np.full((100, 100), 5.0, dtype=np.float32)
        depth[50:, 50:] = 0.1
        node.depth_callback(_depth_msg(depth))
        published = node.front_distance_pub.publish.call_args[0][0].data
        assert published == pytest.approx(5.0)
        node.destroy_node()


# ==============================================================================
# Object exclusion mask
# ==============================================================================

class TestRawObjectMask:
    def test_no_detections_message_at_all_is_none_not_an_empty_mask(self):
        # None means "no information" (hold the confidence map); an all-zero
        # mask means "nothing detected" (decay it). Conflating them would
        # slowly re-admit a still-present object into the background.
        node = _construct()
        assert node._raw_object_mask((10, 10)) is None
        node.destroy_node()

    def test_boxes_are_rasterized_when_no_mask_matches_the_stamp(self):
        node = _construct()
        node._latest_detections = _detections([(50, 50, 20, 20)])
        raw = node._raw_object_mask((100, 100))
        assert raw[50, 50] == 1.0
        assert raw[10, 10] == 0.0
        assert raw.sum() == pytest.approx(400.0)
        node.destroy_node()

    def test_boxes_are_clamped_to_the_depth_image_instead_of_raising(self):
        node = _construct()
        node._latest_detections = _detections([(98, 98, 40, 40)])
        raw = node._raw_object_mask((100, 100))
        assert raw.shape == (100, 100)
        assert raw[99, 99] == 1.0
        node.destroy_node()

    def test_mask_wins_over_boxes_when_the_stamps_match(self):
        # The mask is pixel-accurate; the box is its bounding rectangle. A
        # matching stamp must select the mask, or a segment model silently
        # degrades to box accuracy.
        node = _construct()
        node._latest_detections = _detections([(50, 50, 100, 100)], stamp_nsec=7)
        label = np.zeros((100, 100), dtype=np.uint8)
        label[40:60, 40:60] = 1
        node._latest_mask_msg = _mask_msg(label, stamp_nsec=7)
        raw = node._raw_object_mask((100, 100))
        assert raw.sum() == pytest.approx(400.0)   # the mask, not the 100x100 box
        node.destroy_node()

    def test_a_stale_mask_is_ignored_and_the_boxes_are_used(self):
        node = _construct()
        node._latest_detections = _detections([(50, 50, 20, 20)], stamp_nsec=9)
        label = np.zeros((100, 100), dtype=np.uint8)
        label[0:10, 0:10] = 1
        node._latest_mask_msg = _mask_msg(label, stamp_nsec=8)   # different frame
        raw = node._raw_object_mask((100, 100))
        assert raw[50, 50] == 1.0
        assert raw[5, 5] == 0.0
        node.destroy_node()

    def test_zero_size_mask_means_no_detections_not_a_decode_failure(self):
        # yolo_detector_node's own per-frame empty-mask invariant: a
        # zero-detection frame publishes a 0x0 mask. It must read as an
        # all-zero exclusion, and must NOT fall back to boxes (there are none).
        node = _construct()
        node._latest_detections = _detections([], stamp_nsec=3)
        empty = _mask_msg(np.zeros((1, 1), dtype=np.uint8), stamp_nsec=3)
        empty.height = 0
        empty.width = 0
        node._latest_mask_msg = empty
        raw = node._raw_object_mask((100, 100))
        assert raw is not None
        assert raw.shape == (100, 100)
        assert not raw.any()
        node.destroy_node()

    def test_mask_at_a_different_resolution_is_resized_to_the_depth_image(self):
        node = _construct()
        node._latest_detections = _detections([(25, 25, 10, 10)], stamp_nsec=4)
        label = np.zeros((50, 50), dtype=np.uint8)
        label[20:30, 20:30] = 1
        node._latest_mask_msg = _mask_msg(label, stamp_nsec=4)
        raw = node._raw_object_mask((100, 100))
        assert raw.shape == (100, 100)
        assert raw[50, 50] == 1.0
        node.destroy_node()

    def test_any_nonzero_instance_index_counts_as_object(self):
        # mono8 encoding is an instance INDEX, not a boolean -- instance 3
        # excludes exactly as much as instance 1.
        node = _construct()
        node._latest_detections = _detections([(0, 0, 1, 1)], stamp_nsec=2)
        label = np.zeros((20, 20), dtype=np.uint8)
        label[5:10, 5:10] = 3
        node._latest_mask_msg = _mask_msg(label, stamp_nsec=2)
        raw = node._raw_object_mask((20, 20))
        assert raw[7, 7] == 1.0
        node.destroy_node()


class TestObjectConfidenceEma:
    def test_first_frame_seeds_the_confidence_map(self):
        node = _construct()
        raw = np.ones((4, 4), dtype=np.float32)
        conf = node._update_object_confidence(raw, now=100.0)
        assert conf == pytest.approx(raw)
        node.destroy_node()

    def test_a_dropped_detection_decays_the_exclusion_instead_of_deleting_it(self):
        # The whole reason this is an EMA and not a freshness switch: one
        # missed frame must not hand the object's pixels straight back to the
        # background estimate.
        node = _construct({'mask_time_constant': 0.35})
        ones = np.ones((4, 4), dtype=np.float32)
        zeros = np.zeros((4, 4), dtype=np.float32)
        node._update_object_confidence(ones, now=100.0)
        conf = node._update_object_confidence(zeros, now=100.05)
        assert 0.0 < float(conf[0, 0]) < 1.0
        assert float(conf[0, 0]) > 0.5   # still excluded on the next frame
        node.destroy_node()

    def test_none_holds_the_map_completely(self):
        node = _construct()
        ones = np.ones((4, 4), dtype=np.float32)
        node._update_object_confidence(ones, now=100.0)
        conf = node._update_object_confidence(None, now=100.5)
        assert conf == pytest.approx(ones)
        node.destroy_node()

    def test_alpha_is_dt_normalized_so_the_time_constant_is_wall_clock(self):
        # A longer gap must decay further. If alpha were a fixed per-frame
        # constant, these two would be identical and the time constant would
        # silently mean "frames" at whatever rate the depth stream happens to
        # run.
        zeros = np.zeros((4, 4), dtype=np.float32)
        ones = np.ones((4, 4), dtype=np.float32)

        fast = _construct({'mask_time_constant': 0.35})
        fast._update_object_confidence(ones, now=0.0)
        short_gap = float(fast._update_object_confidence(zeros, now=0.05)[0, 0])
        fast.destroy_node()

        slow = _construct({'mask_time_constant': 0.35})
        slow._update_object_confidence(ones, now=0.0)
        long_gap = float(slow._update_object_confidence(zeros, now=1.0)[0, 0])
        slow.destroy_node()

        assert long_gap < short_gap

    def test_a_depth_resolution_change_reseeds_rather_than_raising(self):
        node = _construct()
        node._update_object_confidence(np.ones((4, 4), dtype=np.float32), now=0.0)
        conf = node._update_object_confidence(np.ones((8, 8), dtype=np.float32), now=0.1)
        assert conf.shape == (8, 8)
        node.destroy_node()

    def test_continuous_detection_with_wobbling_edges_is_damped(self):
        # The larger jitter source, per the module docstring: an object that
        # is never missed, whose silhouette breathes by a pixel every frame.
        # The boundary pixel must not alternate between fully-excluded and
        # fully-included.
        node = _construct({'mask_time_constant': 0.35})
        wide = np.zeros((4, 4), dtype=np.float32)
        wide[:, 0:3] = 1.0
        narrow = np.zeros((4, 4), dtype=np.float32)
        narrow[:, 0:2] = 1.0
        t = 0.0
        for i in range(10):
            node._update_object_confidence(wide if i % 2 else narrow, now=t)
            t += 0.05
        edge = float(node.object_conf_mask[0, 2])
        assert 0.0 < edge < 1.0
        node.destroy_node()


# ==============================================================================
# Corridor obstacles
# ==============================================================================

class TestNearestCorridorObstacle:
    def test_none_without_any_obstacles_message(self):
        node = _construct()
        assert node._nearest_corridor_obstacle(now=10.0) is None
        node.destroy_node()

    def test_stale_message_is_dropped_rather_than_trusted(self):
        node = _construct({'obstacle_max_age_s': 0.4})
        node._latest_obstacles = _obstacles([(1.0, 0.0, 0.1)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.2) is not None
        assert node._nearest_corridor_obstacle(now=10.5) is None
        node.destroy_node()

    def test_distance_is_to_the_near_face_not_the_centre(self):
        node = _construct()
        node._latest_obstacles = _obstacles([(2.0, 0.0, 0.5)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) == pytest.approx(1.5)
        node.destroy_node()

    def test_obstacle_outside_the_corridor_is_ignored(self):
        node = _construct({'corridor_half_width_m': 0.35})
        node._latest_obstacles = _obstacles([(1.0, 5.0, 0.1)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) is None
        node.destroy_node()

    def test_a_wide_object_straddling_the_edge_still_counts(self):
        # Its centre is outside the corridor but its body is in it -- it
        # blocks the car exactly as much as a narrow one dead ahead.
        node = _construct({'corridor_half_width_m': 0.35})
        node._latest_obstacles = _obstacles([(1.0, 0.8, 0.6)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) == pytest.approx(0.4)
        node.destroy_node()

    def test_obstacles_behind_the_car_are_skipped(self):
        node = _construct()
        node._latest_obstacles = _obstacles([(-2.0, 0.0, 0.1)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) is None
        node.destroy_node()

    def test_returns_the_nearest_of_several(self):
        node = _construct()
        node._latest_obstacles = _obstacles(
            [(3.0, 0.0, 0.1), (1.0, 0.0, 0.1), (2.0, 0.0, 0.1)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) == pytest.approx(0.9)
        node.destroy_node()

    def test_an_overlapping_obstacle_floors_at_zero_never_negative(self):
        node = _construct()
        node._latest_obstacles = _obstacles([(0.1, 0.0, 0.5)])
        node._latest_obstacles_time = 10.0
        assert node._nearest_corridor_obstacle(now=10.0) == pytest.approx(0.0)
        node.destroy_node()


# ==============================================================================
# depth_callback: the publish contract
# ==============================================================================

class TestDepthCallbackPublishContract:
    def test_all_four_topics_publish_on_every_frame(self):
        node = _construct()
        node.depth_callback(_depth_msg(np.full((100, 100), 3.0, dtype=np.float32)))
        assert node.front_distance_pub.publish.call_count == 1
        assert node.front_wall_pub.publish.call_count == 1
        assert node.front_clearance_pub.publish.call_count == 1
        assert node.front_blocked_pub.publish.call_count == 1
        node.destroy_node()

    def test_a_frame_with_no_usable_depth_still_publishes_all_four(self):
        # A consumer must never have to distinguish "nothing in front" from
        # "node wedged" -- that is why publishing is not gated on a reading.
        node = _construct()
        node.depth_callback(_depth_msg(np.zeros((100, 100), dtype=np.float32)))
        assert node.front_distance_pub.publish.call_count == 1
        assert node.front_wall_pub.publish.call_count == 1
        assert node.front_clearance_pub.publish.call_count == 1
        assert node.front_blocked_pub.publish.call_count == 1
        node.destroy_node()

    def test_no_reading_yet_publishes_minus_one_not_zero(self):
        node = _construct()
        node.depth_callback(_depth_msg(np.zeros((100, 100), dtype=np.float32)))
        assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(-1.0)
        assert node.front_clearance_pub.publish.call_args[0][0].data == pytest.approx(-1.0)
        node.destroy_node()

    def test_a_bad_frame_after_a_good_one_holds_the_last_value(self):
        # The EMA holds; -1.0 must NOT reappear once a real value exists.
        node = _construct({'min_bg_pixels_for_reading': 1})
        node.depth_callback(_depth_msg(np.full((100, 100), 2.0, dtype=np.float32)))
        node.depth_callback(_depth_msg(np.zeros((100, 100), dtype=np.float32)))
        assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(2.0)
        node.destroy_node()

    def test_non_finite_depths_are_excluded_from_the_estimate(self):
        node = _construct({'min_bg_pixels_for_reading': 1})
        depth = np.full((100, 100), 2.0, dtype=np.float32)
        depth[0:50, :] = np.nan
        depth[50:60, :] = np.inf
        node.depth_callback(_depth_msg(depth))
        assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(2.0)
        node.destroy_node()

    def test_debug_topics_absent_by_default_and_present_when_asked(self):
        plain = _construct()
        assert plain.front_distance_raw_pub is None
        assert plain.bg_pixel_count_pub is None
        plain.destroy_node()

        debug = _construct({'publish_debug_raw': True,
                            'min_bg_pixels_for_reading': 1})
        debug.depth_callback(_depth_msg(np.full((100, 100), 2.0, dtype=np.float32)))
        assert debug.front_distance_raw_pub.publish.call_count == 1
        assert debug.bg_pixel_count_pub.publish.call_count == 1
        assert debug.bg_pixel_count_pub.publish.call_args[0][0].data > 0.0
        debug.destroy_node()


class TestWallLatchIsFedPixels:
    def test_a_far_wall_filling_the_roi_still_reads_as_a_wall(self):
        # The latch input is COVERAGE, not range. A distance-fed latch would
        # call this open space.
        node = _construct({'wall_enter_px': 10, 'wall_min_dwell_frames': 1,
                           'min_bg_pixels_for_reading': 1})
        node.depth_callback(_depth_msg(np.full((100, 100), 40.0, dtype=np.float32)))
        assert node.front_wall_pub.publish.call_args[0][0].data is True
        node.destroy_node()

    def test_an_empty_roi_reads_as_no_wall(self):
        node = _construct({'wall_enter_px': 10, 'wall_exit_px': 5,
                           'wall_min_dwell_frames': 1})
        node.depth_callback(_depth_msg(np.zeros((100, 100), dtype=np.float32)))
        assert node.front_wall_pub.publish.call_args[0][0].data is False
        node.destroy_node()

    def test_one_empty_frame_does_not_drop_the_wall_with_dwell_two(self):
        node = _construct({'wall_enter_px': 10, 'wall_exit_px': 5,
                           'wall_min_dwell_frames': 2,
                           'min_bg_pixels_for_reading': 1})
        wall = _depth_msg(np.full((100, 100), 2.0, dtype=np.float32))
        node.depth_callback(wall)
        node.depth_callback(wall)
        assert node.front_wall_pub.publish.call_args[0][0].data is True
        node.depth_callback(_depth_msg(np.zeros((100, 100), dtype=np.float32)))
        assert node.front_wall_pub.publish.call_args[0][0].data is True
        node.destroy_node()


class TestClearanceFoldsInObstacles:
    def test_clearance_tracks_the_background_when_there_are_no_obstacles(self):
        node = _construct({'min_bg_pixels_for_reading': 1})
        node.depth_callback(_depth_msg(np.full((100, 100), 3.0, dtype=np.float32)))
        assert node.front_clearance_pub.publish.call_args[0][0].data == pytest.approx(3.0)
        node.destroy_node()

    def test_a_near_obstacle_pulls_clearance_below_the_wall_distance(self):
        # front_distance must NOT move (it is wall-only); front_clearance must.
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 1.0})
        node._latest_obstacles = _obstacles([(0.4, 0.0, 0.1)])
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(_depth_msg(np.full((100, 100), 6.0, dtype=np.float32)))
        distance = node.front_distance_pub.publish.call_args[0][0].data
        clearance = node.front_clearance_pub.publish.call_args[0][0].data
        assert distance == pytest.approx(6.0)
        assert clearance == pytest.approx(0.3)
        node.destroy_node()

    def test_blocked_trips_on_a_sustained_near_obstacle(self):
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 1.0,
                           'clearance_enter_m': 0.5, 'clearance_exit_m': 0.7,
                           'clearance_min_dwell_frames': 2})
        node._latest_obstacles = _obstacles([(0.3, 0.0, 0.0)])
        depth = _depth_msg(np.full((100, 100), 6.0, dtype=np.float32))
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(depth)
        assert node.front_blocked_pub.publish.call_args[0][0].data is False
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(depth)
        assert node.front_blocked_pub.publish.call_args[0][0].data is True
        node.destroy_node()

    def test_a_single_near_frame_does_not_trip_blocked(self):
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 1.0,
                           'clearance_min_dwell_frames': 2})
        depth = _depth_msg(np.full((100, 100), 6.0, dtype=np.float32))
        node._latest_obstacles = _obstacles([(0.2, 0.0, 0.0)])
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(depth)
        node._latest_obstacles = _obstacles([])
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(depth)
        assert node.front_blocked_pub.publish.call_args[0][0].data is False
        node.destroy_node()

    def test_blocked_latch_sees_the_smoothed_clearance_not_the_raw_one(self):
        # With a heavy EMA (alpha 0.1) a single 0.1 m spike against an
        # established 6 m background smooths to ~5.4 m, nowhere near the 0.5 m
        # enter threshold. A latch fed the RAW value would see 0.1 and, at
        # dwell 1, trip immediately -- this asserts it does not.
        #
        # The clear frames first are load-bearing, not padding: an EMA SEEDS
        # on its first sample (there is nothing to blend against), so on frame
        # one the smoothed and raw clearance are necessarily the same value
        # and this distinction cannot be observed. Smoothing only exists once
        # a prior value does.
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 0.1,
                           'clearance_enter_m': 0.5, 'clearance_exit_m': 0.7,
                           'clearance_min_dwell_frames': 1})
        depth = _depth_msg(np.full((100, 100), 6.0, dtype=np.float32))
        for _ in range(20):
            node.depth_callback(depth)
        assert node.front_clearance_pub.publish.call_args[0][0].data > 5.0
        assert node.front_blocked_pub.publish.call_args[0][0].data is False

        node._latest_obstacles = _obstacles([(0.1, 0.0, 0.0)])
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(depth)
        assert node.front_blocked_pub.publish.call_args[0][0].data is False
        assert node.front_clearance_pub.publish.call_args[0][0].data > 0.5
        node.destroy_node()

    def test_the_very_first_clearance_reading_is_the_raw_one_by_construction(self):
        # Documents the seed above rather than leaving it as a surprise: an
        # EMA has nothing to blend its first sample against, so frame one is
        # unsmoothed no matter what alpha is. It is the reason the test above
        # needs a warm-up, and it means the latches -- not the filters -- are
        # what protect the first frame after startup.
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 0.1})
        node._latest_obstacles = _obstacles([(0.1, 0.0, 0.0)])
        node._latest_obstacles_time = node._now_seconds()
        node.depth_callback(_depth_msg(np.full((100, 100), 6.0, dtype=np.float32)))
        assert node.front_clearance_pub.publish.call_args[0][0].data == pytest.approx(0.1)
        node.destroy_node()


class TestObjectsAreExcludedFromFrontDistance:
    def test_a_detected_object_does_not_lower_front_distance(self):
        # THE semantic guarantee of front_distance: a person at 0.5 m in front
        # of a 6 m wall must leave it reading the wall.
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 1.0,
                           'roi_half_width_px': 50, 'roi_half_height_px': 50,
                           'background_weight_threshold': 0.5})
        depth = np.full((100, 100), 6.0, dtype=np.float32)
        depth[30:70, 30:70] = 0.5           # the object's own depth
        node._latest_detections = _detections([(50, 50, 40, 40)])
        # Two frames: the first seeds the confidence map at full strength.
        node.depth_callback(_depth_msg(depth))
        node.depth_callback(_depth_msg(depth))
        assert node.front_distance_pub.publish.call_args[0][0].data == pytest.approx(6.0)
        node.destroy_node()

    def test_without_detections_the_same_object_does_lower_it(self):
        # The control for the test above -- proves the exclusion, not the
        # percentile band, is what produced that result.
        node = _construct({'min_bg_pixels_for_reading': 1,
                           'distance_ema_alpha': 1.0,
                           'roi_half_width_px': 50, 'roi_half_height_px': 50})
        depth = np.full((100, 100), 6.0, dtype=np.float32)
        depth[30:70, 30:70] = 0.5
        node.depth_callback(_depth_msg(depth))
        assert node.front_distance_pub.publish.call_args[0][0].data < 6.0
        node.destroy_node()


class TestTransitionLogging:
    def test_info_logs_only_on_a_state_change(self, monkeypatch):
        node = _construct({'wall_enter_px': 10, 'wall_exit_px': 5,
                           'wall_min_dwell_frames': 1,
                           'min_bg_pixels_for_reading': 1})
        calls = []
        monkeypatch.setattr(node.get_logger(), 'info', lambda msg: calls.append(msg))
        wall = _depth_msg(np.full((100, 100), 2.0, dtype=np.float32))
        node.depth_callback(wall)
        after_first = len(calls)
        for _ in range(5):
            node.depth_callback(wall)
        assert len(calls) == after_first     # no per-frame logging
        assert after_first >= 1              # but the transition did log
        node.destroy_node()
