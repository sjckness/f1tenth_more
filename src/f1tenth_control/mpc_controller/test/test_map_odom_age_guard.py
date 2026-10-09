"""
The map -> odom age guard, pinned at its CURRENT behaviour.

Covers _lookup_map_odom, _refresh_goal_anchor and the anchor capture in
goal_distance_callback. Nothing tested these before: the guard was described
in prose (see _lookup_map_odom's "AGE GUARD" note) and never executed by the
suite. These tests are part of the refactor's baseline, not of the refactor:
they record what the code does today, including the parts that read as
surprising, so a later move of this logic can be checked against them.

Real rclpy Time and geometry_msgs TransformStamped objects are used for the
clock and the transform, because the guard's behaviour lives in how those
types subtract (negative ages for post-dated transforms, TypeError across
clock types, ValueError on a negative stamp). Only the node plumbing (tf2
buffer, clock, logger) is a stand-in.

Run standalone: python3 -m pytest test/test_map_odom_age_guard.py -v
"""

import math

from geometry_msgs.msg import TransformStamped
from mpc_controller.MPC_corr import (
    MPCController, _pose_map_to_odom, _pose_odom_to_map)
import numpy as np
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import (
    ConnectivityException, ExtrapolationException, LookupException)

# Clock value every test runs at unless it says otherwise: an arbitrary
# epoch-like instant, in integer nanoseconds so ages are exact.
_NOW_NS = 1_789_394_870_000_000_000


class _Logger:
    """Records every call as (level, text, kwargs)."""

    def __init__(self):
        self.records = []

    def _record(self, level, text, **kwargs):
        self.records.append((level, text, kwargs))

    def info(self, text, **kwargs):
        self._record('info', text, **kwargs)

    def warn(self, text, **kwargs):
        self._record('warn', text, **kwargs)

    def texts(self, level):
        return [text for lvl, text, _ in self.records if lvl == level]


class _Clock:
    def __init__(self, now):
        self._now = now

    def now(self):
        return self._now


class _Buffer:
    """A tf2 buffer stand-in: one transform, or one exception, every call."""

    def __init__(self, transform=None, exc=None):
        self.transform = transform
        self.exc = exc
        self.calls = []

    def lookup_transform(self, target, source, time):
        self.calls.append((target, source, time))
        if self.exc is not None:
            raise self.exc
        return self.transform


def _transform(x, y, yaw, stamp_ns):
    """Build the map -> odom edge as tf2 hands it back, stamped at stamp_ns."""
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.rotation.z = math.sin(yaw / 2.0)
    tf.transform.rotation.w = math.cos(yaw / 2.0)
    return tf


def _now(ns=_NOW_NS, clock_type=ClockType.ROS_TIME):
    return Time(nanoseconds=ns, clock_type=clock_type)


class _Node:
    """
    What _lookup_map_odom and _refresh_goal_anchor read, nothing else.

    Defaults are the shipped ones: map/odom frame names, the feature on (what
    localization_source 'ekf' derives), map_odom_max_age_sec 0.2.
    """

    def __init__(self, buffer, now=None, use_anchor=True):
        self.tf_buffer = buffer
        self.map_frame = 'map'
        self.odom_frame = 'odom'
        self.use_map_frame_goal_anchor = use_anchor
        self.map_odom_max_age_sec = 0.2
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self._logger = _Logger()
        self._clock = _Clock(now if now is not None else _now())

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    quaternion_to_yaw = staticmethod(MPCController.quaternion_to_yaw)
    _lookup_map_odom = MPCController._lookup_map_odom
    _refresh_goal_anchor = MPCController._refresh_goal_anchor


_TF_XYYAW = (1.25, -0.5, 0.3)


def _aged(age_ns, **kwargs):
    """Make a node whose only transform is age_ns old (negative: post-dated)."""
    return _Node(_Buffer(_transform(*_TF_XYYAW, _NOW_NS - age_ns)), **kwargs)


def _assert_is_the_transform(result):
    assert result is not None
    x, y, yaw = result
    assert x == _TF_XYYAW[0]
    assert y == _TF_XYYAW[1]
    assert yaw == pytest.approx(_TF_XYYAW[2], abs=1e-12)


class TestLookupBasics:
    """The lookup itself, before any age is considered."""

    def test_feature_off_returns_none_without_looking_anything_up(self):
        node = _aged(0, use_anchor=False)
        assert node._lookup_map_odom() is None
        assert node._lookup_map_odom(max_age_sec=0.2) is None
        assert node.tf_buffer.calls == []

    def test_asks_for_map_from_odom_at_latest_available(self):
        node = _aged(0)
        node._lookup_map_odom()
        ((target, source, when),) = node.tf_buffer.calls
        assert (target, source) == ('map', 'odom')
        assert when.nanoseconds == 0

    def test_returns_translation_and_yaw_of_the_edge(self):
        _assert_is_the_transform(_aged(0)._lookup_map_odom())

    @pytest.mark.parametrize('exc_type', [
        LookupException, ConnectivityException, ExtrapolationException])
    def test_each_tf2_failure_is_a_none_and_a_throttled_warn(self, exc_type):
        node = _Node(_Buffer(exc=exc_type('no edge')))
        assert node._lookup_map_odom(max_age_sec=0.2) is None
        ((level, text, kwargs),) = node.get_logger().records
        assert level == 'warn'
        assert text == 'tf2 lookup "map" -> "odom" failed: no edge'
        assert kwargs == {'throttle_duration_sec': 5.0}


class TestAgeGuard:
    """max_age_sec set: the refresh path's guard."""

    def test_no_max_age_means_no_guard_at_all(self):
        # The capture path calls with no max_age: a ten-second-old edge is
        # still returned, and nothing is logged.
        node = _aged(10_000_000_000)
        _assert_is_the_transform(node._lookup_map_odom())
        assert node.get_logger().records == []

    def test_fresh_transform_is_returned_silently(self):
        node = _aged(50_000_000)
        _assert_is_the_transform(node._lookup_map_odom(max_age_sec=0.2))
        assert node.get_logger().records == []

    def test_stale_beyond_max_age_is_none_and_warns_with_the_age(self):
        node = _aged(250_000_000)
        assert node._lookup_map_odom(max_age_sec=0.2) is None
        ((level, text, kwargs),) = node.get_logger().records
        assert level == 'warn'
        assert text == ('map -> odom STALE: age=250ms > 200ms -- not '
                        'reprojecting through it')
        assert kwargs == {'throttle_duration_sec': 5.0}

    def test_just_inside_and_just_outside_the_threshold(self):
        assert _aged(199_000_000)._lookup_map_odom(max_age_sec=0.2) is not None
        assert _aged(201_000_000)._lookup_map_odom(max_age_sec=0.2) is None

    def test_age_exactly_at_the_threshold_is_not_stale(self):
        # One-sided: stale means age > max_age. 200 ms in integer nanoseconds
        # times 1e-9 is exactly 0.2 in floating point, so this is the true
        # boundary and it is kept.
        assert 200_000_000 * 1e-9 == 0.2
        _assert_is_the_transform(_aged(200_000_000)._lookup_map_odom(max_age_sec=0.2))

    def test_post_dated_by_transform_time_offset_never_trips(self):
        # ekf_global.yaml's transform_time_offset 0.05 post-dates every
        # broadcast: a fresh edge has age -50 ms. Even a zero ceiling keeps it.
        node = _aged(-50_000_000)
        _assert_is_the_transform(node._lookup_map_odom(max_age_sec=0.0))
        assert node.get_logger().records == []

    def test_clock_type_mismatch_proceeds_unguarded_and_says_so(self):
        # Node clock on SYSTEM_TIME, the stamp read back as ROS_TIME: rclpy
        # refuses the subtraction. The guard degrades to no guard -- even a
        # transform ten seconds old is returned.
        node = _aged(10_000_000_000, now=_now(clock_type=ClockType.SYSTEM_TIME))
        _assert_is_the_transform(node._lookup_map_odom(max_age_sec=0.2))
        ((level, text, kwargs),) = node.get_logger().records
        assert level == 'warn'
        assert text == ('map -> odom age unavailable (Can\'t subtract times with '
                        'different clock types): proceeding UNGUARDED')
        assert kwargs == {'throttle_duration_sec': 5.0}

    def test_negative_stamp_value_error_also_proceeds_unguarded(self):
        tf = _transform(*_TF_XYYAW, _NOW_NS)
        tf.header.stamp.sec = -1
        node = _Node(_Buffer(tf))
        _assert_is_the_transform(node._lookup_map_odom(max_age_sec=0.2))
        (text,) = node.get_logger().texts('warn')
        assert text.startswith('map -> odom age unavailable (')
        assert text.endswith('): proceeding UNGUARDED')


_ANCHOR_MAP = (3.0, -1.0, 0.4)
_HELD = (9.0, 9.0, 9.0)   # a previous reprojection, deliberately implausible


def _anchored(age_ns, **kwargs):
    node = _aged(age_ns, **kwargs)
    node.goal_anchor_map = _ANCHOR_MAP
    node.goal_anchor_odom = _HELD
    return node


class TestRefreshGoalAnchor:
    """_refresh_goal_anchor: reproject when it can, hold when it cannot."""

    def test_no_map_anchor_is_a_no_op_and_looks_nothing_up(self):
        node = _aged(0)
        node.goal_anchor_odom = _HELD
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom is _HELD
        assert node.tf_buffer.calls == []
        assert node.get_logger().records == []

    def test_it_applies_map_odom_max_age_sec(self):
        seen = []

        class _Recording(_Node):
            def _lookup_map_odom(self, max_age_sec=None):
                seen.append(max_age_sec)
                return None

        node = _Recording(_Buffer())
        node.goal_anchor_map = _ANCHOR_MAP
        node.map_odom_max_age_sec = 0.37
        node._refresh_goal_anchor()
        assert seen == [0.37]

    def test_fresh_transform_reprojects_exactly(self):
        node = _anchored(50_000_000)
        node._refresh_goal_anchor()
        tf = node._lookup_map_odom()
        assert node.goal_anchor_odom == _pose_map_to_odom(*_ANCHOR_MAP, *tf)
        assert node.get_logger().texts('warn') == []

    def test_post_dated_transform_reprojects(self):
        node = _anchored(-50_000_000)
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom != _HELD

    def test_stale_transform_holds_the_last_reprojection(self):
        node = _anchored(250_000_000)
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom is _HELD
        warns = node.get_logger().texts('warn')
        assert warns == [
            'map -> odom STALE: age=250ms > 200ms -- not reprojecting through it',
            'ANCHOR/hold | map -> odom unavailable OR STALE this tick: holding '
            'last reprojected anchor (NOT reverting to the raw odom anchor)',
        ]
        assert node.get_logger().records[-1][2] == {'throttle_duration_sec': 5.0}

    def test_lookup_failure_holds_the_last_reprojection(self):
        node = _Node(_Buffer(exc=LookupException('gone')))
        node.goal_anchor_map = _ANCHOR_MAP
        node.goal_anchor_odom = _HELD
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom is _HELD

    def test_clock_type_mismatch_reprojects_through_the_unguarded_edge(self):
        node = _anchored(10_000_000_000, now=_now(clock_type=ClockType.SYSTEM_TIME))
        node._refresh_goal_anchor()
        tf = node._lookup_map_odom()
        assert node.goal_anchor_odom == _pose_map_to_odom(*_ANCHOR_MAP, *tf)

    def test_hold_survives_consecutive_stale_ticks_then_recovers(self):
        node = _anchored(250_000_000)
        node._refresh_goal_anchor()
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom is _HELD
        node.tf_buffer.transform = _transform(*_TF_XYYAW, _NOW_NS)
        node._refresh_goal_anchor()
        assert node.goal_anchor_odom != _HELD


class _CaptureNode(_Node):
    """Adds what goal_distance_callback touches on its way through."""

    def __init__(self, buffer, **kwargs):
        super().__init__(buffer, **kwargs)
        self.x, self.y, self.yaw = 1.0, 2.0, 0.25
        self.drive_cmd = None
        self.goal_pose_xy = (5.0, 5.0)
        self.goal_pose_yaw = 0.0
        self.pose_goal_reached = True

    _clear_drive_state = MPCController._clear_drive_state
    _invalidate_move_state = MPCController._invalidate_move_state
    goal_distance_callback = MPCController.goal_distance_callback


class _Float32:
    def __init__(self, data):
        self.data = data


class TestCaptureAtMoveStart:
    """goal_distance_callback's capture: no age guard, never blocking."""

    def test_capture_converts_the_live_pose_through_the_edge(self):
        node = _CaptureNode(_Buffer(_transform(*_TF_XYYAW, _NOW_NS)))
        node.goal_distance_callback(_Float32(4.0))
        assert node.goal_anchor_odom == (1.0, 2.0, 0.25)
        tf = node._lookup_map_odom()
        assert node.goal_anchor_map == _pose_odom_to_map(1.0, 2.0, 0.25, *tf)

    def test_capture_applies_no_age_guard(self):
        # Ten seconds stale: the refresh path would refuse this edge, the
        # capture path takes it.
        node = _CaptureNode(_Buffer(_transform(*_TF_XYYAW, _NOW_NS - 10_000_000_000)))
        node.goal_distance_callback(_Float32(4.0))
        assert node.goal_anchor_map is not None
        assert not any('STALE' in t for t in node.get_logger().texts('warn'))

    def test_no_edge_at_move_start_keeps_the_raw_odom_anchor_and_warns(self):
        node = _CaptureNode(_Buffer(exc=LookupException('none yet')))
        node.goal_distance_callback(_Float32(4.0))
        assert node.goal_anchor_map is None
        assert node.goal_anchor_odom == (1.0, 2.0, 0.25)
        assert ('ANCHOR/capture | no "map" -> "odom" transform at move start: '
                'this move keeps the odom-frame-only frozen anchor (heading drift '
                'absorbed by map -> odom will NOT be corrected for it).'
                in node.get_logger().texts('warn'))

    def test_feature_off_captures_nothing_and_does_not_warn(self):
        node = _CaptureNode(_Buffer(_transform(*_TF_XYYAW, _NOW_NS)), use_anchor=False)
        node.goal_distance_callback(_Float32(4.0))
        assert node.goal_anchor_map is None
        assert node.goal_anchor_odom == (1.0, 2.0, 0.25)
        assert not any(t.startswith('ANCHOR/') for t in node.get_logger().texts('warn'))

    def test_capture_then_refresh_through_the_same_edge_is_identity(self):
        node = _CaptureNode(_Buffer(_transform(*_TF_XYYAW, _NOW_NS)))
        node.goal_distance_callback(_Float32(4.0))
        node._refresh_goal_anchor()
        np.testing.assert_allclose(node.goal_anchor_odom, (1.0, 2.0, 0.25),
                                   rtol=0.0, atol=1e-12)
