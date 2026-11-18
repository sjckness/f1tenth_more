"""/mpc/goal_pose: the map -> odom drift correction it never had.

WHY THIS EXISTS. test_map_frame_anchor.py documents and pins the fix for
goal_distance mode: this stack's dual-EKF split routes the car's real heading
drift into the map -> odom edge, not into odom-frame yaw (measured: 12.8 deg
of map -> odom yaw drift over one mission, odom-frame yaw under 0.5 deg
throughout -- see that file's own docstring). goal_distance_callback captures
its move's anchor in MAP frame and _refresh_goal_anchor() reprojects it into
the live odom frame every tick, so the drift shows up as an ordinary tracking
error the solver can oppose.

goal_pose_callback -- the single-shot target behind EnterDoor and
PublishMoveGoal's move.goal_pose branch, both of which publish in MAP frame
(enter_door.py's _publish_goal, publish_move_goal.py line ~206) -- got none of
this: it took msg.pose.position straight into self.goal_pose_xy, which
control_loop's pose-mode branch then differences directly against self.x/
self.y, this node's ODOM state. Any map -> odom drift between the goal's
capture and now reaches the car as positional error of exactly that size --
for a 0.9 m doorway with a 0.57 m car (about 16.5 cm of jamb clearance on each
side), the drift this stack ALREADY measures in goal_distance mode is enough
on its own to put the goal on a jamb instead of the opening.

The fix mirrors goal_distance's: goal_pose_callback captures the map-frame
point as goal_pose_anchor_map when the message says frame_id == map_frame,
and _refresh_goal_anchor() reprojects it into goal_pose_xy/goal_pose_yaw every
tick through the same map -> odom lookup. An odom-frame goal (goal_turn_
callback's synthetic turn target, built from self.x/self.y directly) carries
no such anchor and is driven at raw, unchanged from before.

Same "testable without constructing a real node" shape as
test_map_frame_anchor.py and test_object_goal_callback.py: a duck-typed
stand-in, the REAL unbound methods called against it. No rclpy, no node, no
topics, no hardware.

Run standalone: python3 -m pytest test/test_goal_pose_map_frame_anchor.py -v
"""

import inspect
import math
from types import SimpleNamespace

import pytest

from mpc_controller.MPC_corr import MPCController, _pose_map_to_odom, _pose_odom_to_map


class _FakeLogger:

    def __init__(self):
        self.infos = []
        self.warns = []

    def info(self, msg, *_a, **_k):
        self.infos.append(str(msg))

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))


class _FakeMPC:
    """Just enough of MPCController for goal_pose_callback + _refresh_goal_anchor."""

    def __init__(self, x=0.0, y=0.0, map_frame='map', odom_frame='odom'):
        self.x, self.y = x, y
        self._logger = _FakeLogger()
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.map_odom_max_age_sec = 0.2

        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_reached = False
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.goal_pose_anchor_map = None
        self.pose_goal_reached = False
        self._no_goal_warned = False

        self._tf = None               # what _lookup_map_odom() returns
        self._lookup_calls = 0

    def get_logger(self):
        return self._logger

    def _clear_drive_state(self):
        pass

    def _invalidate_move_state(self):
        pass

    def _lookup_map_odom(self, max_age_sec=None):
        self._lookup_calls += 1
        return self._tf

    quaternion_to_yaw = staticmethod(MPCController.quaternion_to_yaw)


def _pose_msg(x, y, yaw, frame_id='map'):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))))


def _send(fake, msg):
    MPCController.goal_pose_callback(fake, msg)


# The map -> odom edge exactly as tf2 returns it, same convention as
# test_map_frame_anchor.py.
_IDENTITY_TF = (0.0, 0.0, 0.0)

# The drift test_map_frame_anchor.py's own docstring measured on one mission.
_MEASURED_DRIFT_RAD = math.radians(12.8)


class TestCaptureOnReceipt:
    """goal_pose_callback: when does a map anchor get captured at all."""

    def test_a_map_frame_goal_captures_an_anchor(self):
        fake = _FakeMPC()
        _send(fake, _pose_msg(5.0, 1.0, 0.3, frame_id='map'))
        assert fake.goal_pose_anchor_map == pytest.approx((5.0, 1.0, 0.3))

    def test_an_odom_frame_goal_captures_no_anchor(self):
        """goal_turn_callback's synthetic target needs none; nor does a
        caller that is already in this node's own driving frame."""
        fake = _FakeMPC()
        _send(fake, _pose_msg(5.0, 1.0, 0.3, frame_id='odom'))
        assert fake.goal_pose_anchor_map is None
        assert fake.get_logger().warns == []

    def test_an_unexpected_frame_warns_and_drives_raw(self):
        fake = _FakeMPC()
        _send(fake, _pose_msg(5.0, 1.0, 0.3, frame_id='base_link'))
        assert fake.goal_pose_anchor_map is None
        assert fake.goal_pose_xy == pytest.approx((5.0, 1.0))
        assert any('base_link' in w for w in fake.get_logger().warns)

    def test_a_second_map_goal_replaces_the_first_anchor(self):
        fake = _FakeMPC()
        _send(fake, _pose_msg(5.0, 1.0, 0.3, frame_id='map'))
        _send(fake, _pose_msg(2.0, -1.0, 0.0, frame_id='map'))
        assert fake.goal_pose_anchor_map == pytest.approx((2.0, -1.0, 0.0))

    def test_it_clears_distance_mode(self):
        fake = _FakeMPC()
        fake.goal_distance = 4.0
        fake.goal_start_xy = (0.0, 0.0)
        fake.goal_anchor_map = (1.0, 1.0, 0.0)
        fake.goal_anchor_odom = (1.0, 1.0, 0.0)
        _send(fake, _pose_msg(5.0, 1.0, 0.3))
        assert fake.goal_distance is None
        assert fake.goal_start_xy is None
        assert fake.goal_anchor_map is None
        assert fake.goal_anchor_odom is None


class TestRefreshReprojectsThePoseGoal:
    """_refresh_goal_anchor: the actual bug fix, at the level the car drives on."""

    def test_raw_capture_is_unreprojected_for_one_tick(self):
        """Pins that the callback itself does not reproject -- only the
        per-tick refresh does, same division of labour as goal_distance."""
        fake = _FakeMPC()
        fake._tf = (0.0, 0.0, _MEASURED_DRIFT_RAD)
        _send(fake, _pose_msg(5.0, 0.0, 0.0))
        assert fake.goal_pose_xy == pytest.approx((5.0, 0.0))

    def test_a_tick_with_no_drift_leaves_the_goal_unchanged(self):
        fake = _FakeMPC()
        fake._tf = _IDENTITY_TF
        _send(fake, _pose_msg(5.0, 0.0, 0.0))
        MPCController._refresh_goal_anchor(fake)
        assert fake.goal_pose_xy == pytest.approx((5.0, 0.0))
        assert fake.goal_pose_yaw == pytest.approx(0.0)

    def test_a_tick_with_drift_moves_the_goal_by_exactly_that_drift(self):
        """THE HEADLINE CASE. A door captured in map before the car had
        turned is reprojected into odom by the live map -> odom edge -- the
        same correction goal_distance mode already gets."""
        fake = _FakeMPC()
        _send(fake, _pose_msg(5.0, 0.0, 0.0))
        anchor_map = fake.goal_pose_anchor_map

        fake._tf = (0.0, 0.0, _MEASURED_DRIFT_RAD)
        MPCController._refresh_goal_anchor(fake)

        expected = _pose_map_to_odom(*anchor_map, *fake._tf)
        assert fake.goal_pose_xy == pytest.approx(expected[:2])
        assert fake.goal_pose_yaw == pytest.approx(expected[2])
        # And it actually moved: an uncorrected goal would still read (5, 0).
        assert fake.goal_pose_xy != pytest.approx((5.0, 0.0), abs=1e-6)

    def test_capture_then_reproject_through_an_unmoved_transform_is_identity(self):
        """No drift since capture -> no correction, exactly as
        test_map_frame_anchor.py pins for goal_distance mode."""
        tf = (4.2, -1.1, -0.63)
        fake = _FakeMPC()
        fake._tf = tf
        _send(fake, _pose_msg(1.0, 2.0, 0.25))
        # Re-express what was JUST captured as if tf were already live at
        # capture time (the capture itself used the raw numbers, not tf, so
        # seed the anchor as goal_distance's own capture does: map coords
        # that ARE this odom pose under tf).
        fake.goal_pose_anchor_map = _pose_odom_to_map(1.0, 2.0, 0.25, *tf)
        MPCController._refresh_goal_anchor(fake)
        assert fake.goal_pose_xy == pytest.approx((1.0, 2.0), abs=1e-9)
        assert fake.goal_pose_yaw == pytest.approx(0.25, abs=1e-9)

    def test_no_anchor_leaves_an_odom_frame_goal_untouched(self):
        """goal_turn_callback's synthetic target: no map anchor, no lookup,
        no change -- the pre-existing behaviour for that case."""
        fake = _FakeMPC()
        fake.goal_pose_xy = (3.0, -2.0)
        fake.goal_pose_yaw = 1.1
        fake.goal_pose_anchor_map = None
        MPCController._refresh_goal_anchor(fake)
        assert fake.goal_pose_xy == (3.0, -2.0)
        assert fake.goal_pose_yaw == 1.1
        assert fake._lookup_calls == 0

    def test_lookup_failure_holds_the_last_reprojected_goal(self):
        """Same hold-last-good rule goal_distance mode already has: reverting
        to the raw map numbers would jump the target by the whole
        accumulated correction."""
        fake = _FakeMPC()
        fake._tf = (0.1, 0.2, 0.05)
        _send(fake, _pose_msg(5.0, 0.0, 0.0))
        MPCController._refresh_goal_anchor(fake)
        held = fake.goal_pose_xy

        fake._tf = None
        MPCController._refresh_goal_anchor(fake)
        assert fake.goal_pose_xy == held
        assert any('POSEGOAL' in w for w in fake.get_logger().warns)

    def test_one_lookup_per_tick_serves_both_anchors(self):
        """Distance mode and pose mode are mutually exclusive in practice,
        but the refresh must not query twice when both happen to be set."""
        fake = _FakeMPC()
        fake.goal_anchor_map = (0.0, 0.0, 0.0)
        fake.goal_pose_anchor_map = (1.0, 1.0, 0.0)
        fake._tf = _IDENTITY_TF
        MPCController._refresh_goal_anchor(fake)
        assert fake._lookup_calls == 1


class TestOtherGoalShapesClearTheAnchor:
    """A stale pose-mode anchor must not survive a switch to another mode --
    see goal_pose_anchor_map's own comment in __init__ for the failure this
    guards: goal_turn_callback's synthetic (already odom-frame) target would
    otherwise be silently overwritten next tick by a left-over door anchor.
    """

    def _src(self, name):
        return inspect.getsource(getattr(MPCController, name))

    def test_goal_distance_callback_clears_it(self):
        src = self._src('goal_distance_callback')
        assert 'self.goal_pose_anchor_map = None' in src

    def test_goal_turn_callback_clears_it(self):
        src = self._src('goal_turn_callback')
        assert 'self.goal_pose_anchor_map = None' in src

    def test_goal_object_callback_clears_it(self):
        src = self._src('goal_object_callback')
        assert 'self.goal_pose_anchor_map = None' in src

    def test_goal_drive_callback_clears_it(self):
        src = self._src('goal_drive_callback')
        assert 'self.goal_pose_anchor_map = None' in src

    def test_goal_pose_callback_itself_resets_it_before_recapturing(self):
        src = self._src('goal_pose_callback')
        assert 'self.goal_pose_anchor_map = None' in src


if __name__ == '__main__':
    import unittest
    unittest.main()
