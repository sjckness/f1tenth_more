"""/mpc/goal_object: a target refresh must not re-anchor the corridor.

This is the whole reason ObjectGoal carries a move_id, and the reason the
callback branches on it before touching any state. The failure being
prevented is concrete and was live until the go_to_object move replaced it:
object_goal_bridge drove /mpc/goal_pose at 20 Hz, and goal_pose_callback
treats every message as a new move -- it calls _invalidate_move_state(),
which nulls cached_corridor and last_corridor_time and drops warm_start_z.
With a message per 50 ms into a 10 Hz control loop, the corridor is rebuilt
and re-anchored on EVERY control tick, corridor_update_period stops meaning
anything, and the RTI warm start never survives long enough to be used.

Same "testable without constructing a real node" shape as
test_corridor_direction_recovery.py: a duck-typed stand-in carrying just the
instance state the methods under test touch, with the REAL unbound methods
called against it. Using the real _invalidate_move_state matters here --
counting calls to a mock would only prove the test's own wiring, whereas
watching cached_corridor actually survive proves the thing that broke.

No rclpy, no node, no topics, no hardware.
"""

import collections
import inspect
import math
from types import SimpleNamespace

import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.object_approach import TargetBehindPersistence, heading_margin_for
from mpc_controller.object_guard import RefreshWatchdog, SteeringRamp


class _FakeLogger:

    def __init__(self):
        self.warns = []
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def debug(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.warns.append(str(msg))

    def error(self, msg, *_a, **_k):
        self.errors.append(str(msg))


class _FakeClock:
    """A settable clock with rclpy's .now().nanoseconds shape."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(round(self.t * 1e9)))


class _FakeMPC:
    """Just enough of MPCController for goal_object_callback + friends."""

    _log_object_corridor_mode = MPCController._log_object_corridor_mode

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self._logger = _FakeLogger()
        self.clock = _FakeClock()
        self.drive = []                 # (speed, steering) per _publish_drive

        # Leaving object mode, mirroring MPCController.__init__.
        self.object_ended_move_ids = collections.deque(maxlen=64)
        self.object_exit_ramp = SteeringRamp(0.6)
        self.object_exit_ramping = False
        self._object_exit_last_sec = None
        self.object_goal_watchdog = RefreshWatchdog(0.5)
        self.object_goal_watchdog_tripped = False
        self._last_published_steer = 0.0
        self.delta_real = None
        self.hold = False

        # A hold RELEASE now also says which object-corridor geometry is
        # active (MPC_corr._log_object_corridor_mode, added so a run's log
        # records the mode beside the mission it governs). Bound rather than
        # stubbed: a stand-in that skipped it would let the callback break on
        # the car while these tests stayed green.
        self.object_corridor_mode = 'off'
        self._object_mode_default = 'off'

        # Object-mode state, mirroring MPCController.__init__.
        self.goal_object_move_id = None
        self.goal_object_target_class = ''
        self.goal_object_map_xy = None
        self.goal_object_odom_xy = None
        self.goal_object_stamp = None
        self.goal_object_standoff = 0.0
        self.goal_object_speed = 0.0
        self.object_psi_c = None
        self.object_target_at_build = None
        self.object_target_held = False
        self.object_last_step = None
        self.object_retarget_distance = 0.2
        self.object_r_freeze = 0.4
        self.object_heading_margin = heading_margin_for(0.4)
        self.object_behind = TargetBehindPersistence(0.5)
        self.object_last_flags = None
        self.object_behind_terminal = False
        self.object_behind_for_s = 0.0
        self.params = {'L': 0.305}
        self.limits = {'delta_min': -0.283, 'delta_max': 0.278}

        # The other four goal shapes' state, which a new object move clears.
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_reached = False
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self._no_goal_warned = False
        self.drive_cmd = None

        # What _invalidate_move_state touches.
        self.wall_turn_committed = False
        self.wall_turn_commanded_rot = None
        self.smoothed_target = None
        self.last_deflection_vec = None
        self.deflection_decay_remaining = 0
        self.cached_corridor = None
        self.last_corridor_time = None
        self.last_corridor_stamp = None
        self.cached_pref_nom = None
        self.warm_start_z = None

        self.invalidations = 0

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self.clock

    def _clear_drive_state(self):
        pass

    def _publish_drive(self, speed, steering):
        self._last_published_steer = float(steering)
        self.drive.append((float(speed), float(steering)))

    def _mark_object_move_ended(self, move_id):
        MPCController._mark_object_move_ended(self, move_id)

    def _end_object_move(self, reason):
        MPCController._end_object_move(self, reason)

    def _publish_stop(self):
        MPCController._publish_stop(self)

    def _invalidate_move_state(self):
        """The real one, plus a counter so a test can assert how often it ran."""
        self.invalidations += 1
        MPCController._invalidate_move_state(self)

    # -- state a "warm" mid-move node would be carrying -------------------

    def warm_up(self):
        """Pretend a corridor has been built and a solve has completed."""
        self.cached_corridor = {'sentinel': 'corridor from the last rebuild'}
        self.last_corridor_time = 100.0
        self.last_corridor_stamp = 'stamp'
        self.cached_pref_nom = 'pref'
        self.warm_start_z = 'warm start from the last solve'
        self.smoothed_target = 'smoothed'
        return self


def _goal(move_id, x, y, standoff=1.0, speed=0.5, stamp=1.0,
          target_class='person'):
    """An ObjectGoal-shaped stand-in; the callback only reads these fields."""
    sec = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=int((stamp - sec) * 1e9))),
        move_id=move_id,
        target_class=target_class,
        point=SimpleNamespace(x=x, y=y, z=0.0),
        standoff=standoff,
        speed=speed,
    )


def _send(fake, goal):
    MPCController.goal_object_callback(fake, goal)


# ------------------------------------------------- the refresh fast path

class TestRepeatedMessagesDoNotReAnchor:

    def test_two_hundred_refreshes_invalidate_exactly_once(self):
        """The headline. 200 messages at 20 Hz is 10 s of a real approach."""
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0))
        fake.warm_up()

        for i in range(200):
            # The target drifts slightly, as a live tracker's would.
            _send(fake, _goal('run7:move_0', 4.0 + 0.001 * i, 0.002 * i,
                              stamp=1.0 + 0.05 * i))

        assert fake.invalidations == 1, (
            f'{fake.invalidations} invalidations for one move -- a target '
            'refresh is re-anchoring the corridor')

    def test_the_warm_start_survives_all_of_them(self):
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0))
        fake.warm_up()

        for i in range(200):
            _send(fake, _goal('run7:move_0', 4.0, 0.01 * i))

        assert fake.warm_start_z == 'warm start from the last solve'

    def test_the_corridor_cache_survives_all_of_them(self):
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0))
        fake.warm_up()

        for _ in range(200):
            _send(fake, _goal('run7:move_0', 4.0, 0.0))

        assert fake.cached_corridor == {'sentinel': 'corridor from the last rebuild'}
        assert fake.last_corridor_time == 100.0, (
            'last_corridor_time was nulled -- the next tick would rebuild '
            'regardless of corridor_update_period')

    def test_the_held_heading_survives_all_of_them(self):
        """psi_c moves at REBUILDS only, never on a message."""
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0))
        seeded = fake.object_psi_c

        for i in range(200):
            _send(fake, _goal('run7:move_0', 4.0, 3.0 + i))   # wild swings

        assert fake.object_psi_c == seeded

    def test_a_refresh_does_update_the_target_and_the_stamp(self):
        """The fast path is small, not inert."""
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0, stamp=1.0))
        _send(fake, _goal('run7:move_0', 4.5, 0.25, stamp=9.5))

        assert fake.goal_object_map_xy == pytest.approx((4.5, 0.25))
        assert fake.goal_object_stamp == pytest.approx(9.5, abs=1e-6)

    def test_a_refresh_can_change_the_standoff_and_speed(self):
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0, standoff=1.0, speed=0.5))
        _send(fake, _goal('run7:move_0', 4.0, 0.0, standoff=1.2, speed=0.4))

        assert fake.goal_object_standoff == pytest.approx(1.2)
        assert fake.goal_object_speed == pytest.approx(0.4)
        assert fake.invalidations == 1


# ---------------------------------------------------------- the new-move path

class TestANewMoveDoesReAnchor:

    def test_a_different_move_id_invalidates(self):
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0', 4.0, 0.0))
        fake.warm_up()
        _send(fake, _goal('run7:move_1', 2.0, 2.0))

        assert fake.invalidations == 2
        assert fake.cached_corridor is None
        assert fake.warm_start_z is None

    def test_the_same_mission_re_run_is_a_new_move(self):
        """Move ids repeat across runs; the id must carry the run with it.

        This test does not enforce the composition -- the sender owns that --
        but it pins that two DIFFERENT ids re-anchor, which is what makes a
        correctly composed id work.
        """
        fake = _FakeMPC()
        _send(fake, _goal('run7:move_0_go_to_person', 4.0, 0.0))
        _send(fake, _goal('run8:move_0_go_to_person', 4.0, 0.0))
        assert fake.invalidations == 2

    def test_psi_c_is_seeded_from_the_bearing_not_from_yaw(self):
        fake = _FakeMPC(x=0.0, y=0.0, yaw=1.2)
        _send(fake, _goal('m', 3.0, 3.0))
        assert fake.object_psi_c == pytest.approx(math.pi / 4)

    def test_psi_c_falls_back_to_yaw_for_a_coincident_target(self):
        fake = _FakeMPC(x=2.0, y=2.0, yaw=0.7)
        _send(fake, _goal('m', 2.0, 2.0))
        assert fake.object_psi_c == pytest.approx(0.7)

    def test_it_clears_the_other_four_goal_shapes(self):
        fake = _FakeMPC()
        fake.goal_distance = 6.0
        fake.goal_start_xy = (0.0, 0.0)
        fake.goal_pose_xy = (9.0, 9.0)
        fake.goal_pose_yaw = 0.3
        fake.goal_reached = True
        fake.pose_goal_reached = True

        _send(fake, _goal('m', 4.0, 0.0))

        assert fake.goal_distance is None
        assert fake.goal_start_xy is None
        assert fake.goal_pose_xy is None
        assert fake.goal_pose_yaw is None
        assert fake.goal_reached is False
        assert fake.pose_goal_reached is False

    def test_the_seeded_state_survives_its_own_invalidation(self):
        """_invalidate_move_state clears object state, so ordering matters."""
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0))
        assert fake.object_psi_c is not None
        assert fake.goal_object_map_xy is not None


# ------------------------------------------------------------------ guards

class TestGuards:

    def test_a_message_before_odom_is_ignored(self):
        fake = _FakeMPC()
        fake.x = fake.y = None
        _send(fake, _goal('m', 4.0, 0.0))
        assert fake.goal_object_move_id is None
        assert fake.invalidations == 0

    def test_a_zero_standoff_is_refused_not_clamped(self):
        """It would aim the corridor at the target, and the target is a person."""
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0, standoff=0.0))
        assert fake.goal_object_move_id is None
        assert fake.invalidations == 0
        assert any('standoff' in e for e in fake.get_logger().errors)

    def test_a_negative_standoff_is_refused(self):
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0, standoff=-0.5))
        assert fake.goal_object_move_id is None

    def test_an_active_move_is_not_disturbed_by_a_refused_message(self):
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0, standoff=1.0))
        seeded = fake.object_psi_c
        _send(fake, _goal('m2', 1.0, 1.0, standoff=0.0))
        assert fake.goal_object_move_id == 'm'
        assert fake.object_psi_c == seeded
        assert fake.invalidations == 1


# ---------------------------------------------------- supersede / exclusivity

class TestSupersededByAnotherGoal:

    def test_another_goal_shape_switches_object_mode_off(self):
        """_invalidate_move_state clears psi_c, which is the active-mode test."""
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0))
        fake.goal_object_odom_xy = (3.0, 0.0)

        MPCController._invalidate_move_state(fake)      # any other goal does this

        assert fake.object_psi_c is None
        assert fake.goal_object_odom_xy is None

    def test_a_refresh_after_that_does_not_restart_the_approach(self):
        """The gate in _refresh_object_target; without it the mode resumes."""
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0))
        MPCController._invalidate_move_state(fake)
        _send(fake, _goal('m', 4.2, 0.0))               # a late refresh

        assert fake.object_psi_c is None, (
            'a refresh re-armed object mode after another goal superseded it')
        assert fake.goal_object_odom_xy is None


# --------------------------------------------------------- retarget trigger

class TestRetargetTrigger:

    def test_no_build_yet_forces_a_build(self):
        fake = _FakeMPC()
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.object_target_at_build = None
        assert MPCController._object_target_moved_since_build(fake) is True

    def test_a_small_drift_does_not_trigger(self):
        """Estimate jitter must not pre-empt corridor_update_period."""
        fake = _FakeMPC()
        fake.object_target_at_build = (4.0, 0.0)
        fake.goal_object_odom_xy = (4.05, 0.03)
        assert MPCController._object_target_moved_since_build(fake) is False

    def test_motion_past_the_threshold_triggers(self):
        fake = _FakeMPC()
        fake.object_target_at_build = (4.0, 0.0)
        fake.goal_object_odom_xy = (4.0, 0.25)
        assert MPCController._object_target_moved_since_build(fake) is True

    def test_the_threshold_is_the_parameter(self):
        fake = _FakeMPC()
        fake.object_retarget_distance = 0.2
        fake.object_target_at_build = (0.0, 0.0)
        for dist, expected in ((0.19, False), (0.20, True), (0.21, True)):
            fake.goal_object_odom_xy = (dist, 0.0)
            assert MPCController._object_target_moved_since_build(fake) is expected


class TestObjectRange:

    def test_it_is_distance_minus_standoff(self):
        fake = _FakeMPC(x=0.0, y=0.0)
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.goal_object_standoff = 1.0
        assert MPCController._object_range(fake) == pytest.approx(3.0)

    def test_it_goes_negative_inside_the_standoff(self):
        fake = _FakeMPC(x=0.0, y=0.0)
        fake.goal_object_odom_xy = (0.6, 0.0)
        fake.goal_object_standoff = 1.0
        assert MPCController._object_range(fake) == pytest.approx(-0.4)

    def test_it_is_infinite_with_no_target(self):
        fake = _FakeMPC()
        assert MPCController._object_range(fake) == math.inf


# ------------------------------------------------------ per-tick flags

class TestPerTickFlags:
    """_assess_object_tick: the flags come from THIS tick's pose, every tick."""

    def _active(self, target=(4.0, 0.0), yaw=0.0):
        fake = _FakeMPC(yaw=yaw)
        _send(fake, _goal('m', *target))
        fake.goal_object_odom_xy = target
        return fake

    def test_nothing_outside_object_mode(self):
        fake = _FakeMPC()
        assert MPCController._assess_object_tick(fake, (0.0, 0.0, 0.0), 0.0) is None

    def test_r_is_live_not_the_last_rebuild(self):
        fake = self._active()
        fake.object_last_step = SimpleNamespace(r=3.0, k=1.0, dpsi_max=0.1)
        flags = MPCController._assess_object_tick(fake, (1.5, 0.0, 0.0), 0.0)
        assert flags.r == pytest.approx(1.5)

    def test_terminal_only_after_the_persistence_time(self):
        fake = self._active(target=(-3.0, 0.0), yaw=0.0)   # astern
        states = []
        for tick in range(7):
            MPCController._assess_object_tick(fake, (0.0, 0.0, 0.0), tick * 0.1)
            states.append(fake.object_behind_terminal)
        assert states == [False] * 5 + [True, True]
        assert fake.object_behind_for_s == pytest.approx(0.6)

    def test_a_new_move_resets_the_persistence(self):
        fake = self._active(target=(-3.0, 0.0))
        for tick in range(6):
            MPCController._assess_object_tick(fake, (0.0, 0.0, 0.0), tick * 0.1)
        assert fake.object_behind_terminal is True
        _send(fake, _goal('m2', -3.0, 0.0))
        fake.goal_object_odom_xy = (-3.0, 0.0)
        assert fake.object_behind_terminal is False
        MPCController._assess_object_tick(fake, (0.0, 0.0, 0.0), 0.6)
        assert fake.object_behind_terminal is False


class TestStatusFields:
    """_publish_object_status: live fields from the flags, rebuild fields from the step."""

    class _Clock:
        def now(self):
            from builtin_interfaces.msg import Time
            return SimpleNamespace(nanoseconds=int(12.5e9),
                                   to_msg=lambda: Time(sec=12, nanosec=500000000))

    def _fake(self):
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0, stamp=12.0))
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.odom_frame = 'odom'
        fake.published = []
        fake.object_status_pub = SimpleNamespace(publish=fake.published.append)
        fake.get_clock = lambda: self._Clock()
        return fake

    def test_live_and_rebuild_fields_come_from_their_own_sources(self):
        fake = self._fake()
        step = SimpleNamespace(r=9.9, bearing=9.9, e=9.9, k=0.25, dpsi_max=0.05,
                               psi_c_new=9.9)
        flags = MPCController._assess_object_tick(fake, (2.0, 0.0, 0.0), 12.5)
        MPCController._publish_object_status(fake, step, flags, 0.3)
        (msg,) = fake.published
        assert msg.r == pytest.approx(1.0)
        assert msg.e == pytest.approx(0.0)
        assert msg.k == pytest.approx(0.25)
        assert msg.dpsi_max == pytest.approx(0.05)
        assert msg.psi_c == pytest.approx(fake.object_psi_c)
        assert msg.inside_turn_radius is False
        assert msg.target_behind is False
        assert msg.target_behind_terminal is False
        assert msg.target_age_s == pytest.approx(0.5)
        assert msg.move_id == 'm'

    def test_the_debug_echoes_track_id_and_the_live_gap(self):
        """gap = live r + the goal's gap_m: centre distance - nose_reach - radius."""
        fake = _FakeMPC()
        goal = _goal('m', 4.0, 0.0, standoff=1.3, stamp=12.0)
        goal.track_id, goal.gap_m = '12', 0.5
        _send(fake, goal)
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.odom_frame = 'odom'
        fake.published = []
        fake.object_status_pub = SimpleNamespace(publish=fake.published.append)
        fake.get_clock = lambda: self._Clock()
        flags = MPCController._assess_object_tick(fake, (2.0, 0.0, 0.0), 12.5)
        MPCController._publish_object_status(fake, None, flags, 0.4)
        (msg,) = fake.published
        assert msg.track_id == '12'
        assert msg.r == pytest.approx(2.0 - 1.3)
        assert msg.gap == pytest.approx(0.7 + 0.5)

    def test_a_goal_without_the_debug_fields_reports_an_unknown_gap(self):
        fake = self._fake()
        flags = MPCController._assess_object_tick(fake, (2.0, 0.0, 0.0), 12.5)
        MPCController._publish_object_status(fake, None, flags, 0.4)
        (msg,) = fake.published
        assert msg.track_id == ''
        assert math.isnan(msg.gap)


# ------------------------------------------------------- leaving object mode

def _end(fake, move_id):
    MPCController.goal_object_end_callback(fake, SimpleNamespace(data=move_id))


class TestLeavingObjectMode:
    """How an object move ends, and that it stays ended.

    BEFORE this change the only exit was _invalidate_move_state, called when
    another goal shape arrived. /mpc/hold zeroed the drive but kept object
    mode, so releasing it resumed the approach, and nothing remembered which
    move ids had finished.
    """

    def _active(self, move_id='run7:m'):
        fake = _FakeMPC()
        _send(fake, _goal(move_id, 4.0, 0.0))
        fake.goal_object_odom_xy = (4.0, 0.0)
        return fake

    def test_the_end_message_leaves_object_mode(self):
        fake = self._active()
        _end(fake, 'run7:m')
        assert fake.object_psi_c is None
        assert fake.goal_object_odom_xy is None
        assert 'run7:m' in fake.object_ended_move_ids

    def test_a_late_refresh_of_an_ended_move_does_not_restart_it(self):
        """The queued-message case: same id, arriving after the end."""
        fake = self._active()
        _end(fake, 'run7:m')
        invalidations = fake.invalidations
        _send(fake, _goal('run7:m', 4.1, 0.0))
        assert fake.object_psi_c is None, 'a late ObjectGoal restarted an ended move'
        assert fake.goal_object_map_xy == (4.0, 0.0), 'nor may it update the target'
        assert fake.invalidations == invalidations

    def test_an_end_for_a_move_never_started_still_blocks_it(self):
        fake = _FakeMPC()
        _end(fake, 'run7:m')
        _send(fake, _goal('run7:m', 4.0, 0.0))
        assert fake.object_psi_c is None

    def test_an_end_for_another_id_does_not_touch_the_active_move(self):
        fake = self._active('run7:m')
        _end(fake, 'run6:m')
        assert fake.object_psi_c is not None
        assert 'run6:m' in fake.object_ended_move_ids

    def test_hold_ends_an_active_object_move(self):
        fake = self._active()
        MPCController.hold_callback(fake, SimpleNamespace(data=True))
        assert fake.object_psi_c is None
        assert 'run7:m' in fake.object_ended_move_ids

    def test_releasing_the_hold_does_not_resume_it(self):
        fake = self._active()
        MPCController.hold_callback(fake, SimpleNamespace(data=True))
        MPCController.hold_callback(fake, SimpleNamespace(data=False))
        _send(fake, _goal('run7:m', 4.0, 0.0))
        assert fake.object_psi_c is None

    def test_hold_outside_object_mode_marks_nothing(self):
        fake = _FakeMPC()
        MPCController.hold_callback(fake, SimpleNamespace(data=True))
        assert list(fake.object_ended_move_ids) == []

    def test_a_superseding_goal_of_another_shape_ends_the_move(self):
        fake = self._active()
        MPCController._invalidate_move_state(fake)      # what every other goal does
        assert 'run7:m' in fake.object_ended_move_ids
        _send(fake, _goal('run7:m', 4.0, 0.0))
        assert fake.object_psi_c is None

    def test_a_new_object_move_ends_the_previous_one_not_itself(self):
        fake = self._active('run7:a')
        _send(fake, _goal('run7:b', 5.0, 1.0))
        assert 'run7:a' in fake.object_ended_move_ids
        assert 'run7:b' not in fake.object_ended_move_ids
        assert fake.goal_object_move_id == 'run7:b'
        assert fake.object_psi_c is not None
        _send(fake, _goal('run7:a', 4.0, 0.0))            # late refresh of a
        assert fake.goal_object_move_id == 'run7:b'

    def test_the_ended_memory_is_bounded(self):
        fake = _FakeMPC()
        for i in range(200):
            _end(fake, f'run{i}:m')
        assert len(fake.object_ended_move_ids) == 64
        assert 'run199:m' in fake.object_ended_move_ids


class TestExitRamp:
    """Ported from the prototype's LOST ramp: speed zero at once, wheels ramp out."""

    def _ended_at(self, steer, measured=None):
        fake = _FakeMPC()
        _send(fake, _goal('m', 4.0, 0.0))
        fake._publish_drive(0.4, steer)                   # the last solve's command
        fake.delta_real = measured
        _end(fake, 'm')
        return fake

    def _stop_ticks(self, fake, n, dt=0.1):
        for _ in range(n):
            MPCController._publish_stop(fake)
            fake.clock.t += dt
        return fake.drive[-n:]

    def test_speed_is_zero_on_every_tick_and_the_steering_is_a_ramp(self):
        fake = self._ended_at(0.25)
        out = self._stop_ticks(fake, 8)
        assert all(speed == 0.0 for speed, _ in out)
        steers = [0.25] + [steer for _, steer in out]
        steps = [abs(b - a) for a, b in zip(steers, steers[1:])]
        assert max(steps) <= 0.6 * 0.1 + 1e-12, 'a ramp, not a step'
        assert steers[-1] == 0.0, 'and it does reach centre'
        assert fake.object_exit_ramping is False

    def test_the_first_tick_holds_because_no_time_has_elapsed(self):
        fake = self._ended_at(0.25)
        (first,) = self._stop_ticks(fake, 1)
        assert first == (0.0, 0.25)

    def test_it_starts_from_the_measured_angle_when_there_is_one(self):
        fake = self._ended_at(0.25, measured=-0.20)
        out = self._stop_ticks(fake, 2)
        assert out[0][1] == pytest.approx(-0.20)
        assert out[1][1] == pytest.approx(-0.20 + 0.06)

    def test_without_a_measurement_it_starts_from_the_last_published_angle(self):
        fake = self._ended_at(-0.18)
        out = self._stop_ticks(fake, 2)
        assert out[1][1] == pytest.approx(-0.18 + 0.06)

    def test_a_sign_change_on_the_next_move_is_not_inherited(self):
        """The ramp is cancelled by a new goal, which owns the steering again."""
        fake = self._ended_at(0.25)
        self._stop_ticks(fake, 2)
        MPCController._invalidate_move_state(fake)
        assert fake.object_exit_ramping is False
        (out,) = self._stop_ticks(fake, 1)
        assert out == (0.0, 0.0)

    def test_no_ramp_when_no_object_move_ended(self):
        fake = _FakeMPC()
        fake._publish_drive(0.3, 0.2)
        (out,) = self._stop_ticks(fake, 1)
        assert out == (0.0, 0.0)

    def test_hold_and_no_goal_both_stop_through_the_ramp(self):
        """Source check: the two stop paths an ended move lands on use it."""
        src = inspect.getsource(MPCController.control_loop)
        hold_branch = src[src.index('if self.hold:'):]
        assert 'self._publish_stop()' in hold_branch.split('return', 1)[0]
        no_goal = src[src.index('elif self.goal_distance is None or self.goal_start_xy is None:'):]
        assert 'self._publish_stop()' in no_goal.split('return', 1)[0]

    def test_lost_odometry_is_a_hard_zero_and_cancels_the_ramp(self):
        src = inspect.getsource(MPCController.control_loop)
        odom_branch = src[src.index("'ODOM non disponibile"):].split('return', 1)[0]
        assert 'self.object_exit_ramping = False' in odom_branch
        assert 'self._publish_drive(0.0, 0.0)' in odom_branch


class TestGoalRefreshWatchdog:
    """Ported from the prototype's watchdog: a silent sender is a hard stop."""

    def test_refreshes_feed_the_watchdog(self):
        fake = _FakeMPC()
        fake.clock.t = 10.0
        _send(fake, _goal('m', 4.0, 0.0))
        fake.clock.t = 10.4
        _send(fake, _goal('m', 4.1, 0.0))
        assert fake.object_goal_watchdog.tripped(10.8) is None
        assert fake.object_goal_watchdog.tripped(11.0) is not None

    def test_a_new_move_starts_fresh(self):
        fake = _FakeMPC()
        fake.clock.t = 1.0
        _send(fake, _goal('a', 4.0, 0.0))
        fake.clock.t = 9.0
        _send(fake, _goal('b', 4.0, 0.0))
        assert fake.object_goal_watchdog.tripped(9.1) is None

    def test_the_trip_branch_is_a_hard_zero_not_the_ramp(self):
        src = inspect.getsource(MPCController.control_loop)
        branch = src[src.index('watchdog_reason = self.object_goal_watchdog.tripped'):]
        branch = branch.split('return', 1)[0]
        assert 'self.object_exit_ramp.seed(0.0)' in branch
        assert 'self._publish_drive(0.0, 0.0)' in branch
        assert '_publish_stop' not in branch

    def test_the_trip_status_carries_live_range_not_a_zero(self):
        src = inspect.getsource(MPCController.control_loop)
        branch = src[src.index('watchdog_reason = self.object_goal_watchdog.tripped'):]
        branch = branch.split('return', 1)[0]
        assert 'self._assess_object_tick(' in branch
