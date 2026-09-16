"""/mpc/goal_object: a target refresh must not re-anchor the corridor.

This is the whole reason ObjectGoal carries a move_id, and the reason the
callback branches on it before touching any state. The failure being
prevented is concrete and is happening today: object_goal_bridge drives
/mpc/goal_pose at 20 Hz, and goal_pose_callback treats every message as a new
move -- it calls _invalidate_move_state(), which nulls cached_corridor and
last_corridor_time and drops warm_start_z. With a message per 50 ms into a
10 Hz control loop, the corridor is rebuilt and re-anchored on EVERY control
tick, corridor_update_period stops meaning anything, and the RTI warm start
never survives long enough to be used.

Same "testable without constructing a real node" shape as
test_corridor_direction_recovery.py: a duck-typed stand-in carrying just the
instance state the methods under test touch, with the REAL unbound methods
called against it. Using the real _invalidate_move_state matters here --
counting calls to a mock would only prove the test's own wiring, whereas
watching cached_corridor actually survive proves the thing that broke.

No rclpy, no node, no topics, no hardware.
"""

import math
from types import SimpleNamespace

import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.object_approach import TargetBehindPersistence, heading_margin_for


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


class _FakeMPC:
    """Just enough of MPCController for goal_object_callback + friends."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self._logger = _FakeLogger()

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

    def _clear_drive_state(self):
        pass

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
