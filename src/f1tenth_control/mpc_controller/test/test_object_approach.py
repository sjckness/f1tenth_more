"""object_approach.py -- the held corridor heading, its schedule and its caps.

Pure-function tests, same convention as test_wall_turn_increment.py: no
rclpy, no node, no hardware. Every number here is either derived in the
assertion itself or taken from the shipped parameter set, so a retune shows
up as a failing test rather than as a silently different vehicle.

The shipped geometry these use:
    wheelbase  0.305   delta_min -0.283   delta_max +0.278
    N 20        ts 0.1  v_ref 0.5   ->  horizon reach 1.0 m
    R_min      1.0685 m turning left, 1.0483 m turning right
    r_full 1.5  r_freeze 0.4  c_safety 1.5
"""

import math

import pytest

from mpc_controller.object_approach import (
    DEFAULT_HEADING_MARGIN_RAD,
    TargetBehindPersistence,
    approach_lookahead,
    assess_object_approach,
    build_object_centreline,
    goal_point,
    heading_margin_for,
    SPEED_BELOW_FLOOR,
    SPEED_DRIVE,
    SPEED_STOP_LATCHED,
    SPEED_WAIT,
    ObjectStopLatch,
    floor_moving_speed,
    min_standoff_clear_of,
    object_speed_decision,
    plan_object_heading,
)
from mpc_controller.wall_turn import horizon_heading_reach, min_turn_radius

WHEELBASE = 0.305
DELTA_MIN = -0.283
DELTA_MAX = 0.278
N_STEPS = 20
TS = 0.1
V_REF = 0.5

R_FULL = 1.5
R_FREEZE = 0.4
C_SAFETY = 1.5

KINEMATICS = dict(
    wheelbase=WHEELBASE, delta_min=DELTA_MIN, delta_max=DELTA_MAX,
    n_steps=N_STEPS, ts=TS, v_ref=V_REF,
)
SCHEDULE = dict(r_full=R_FULL, r_freeze=R_FREEZE, c_safety=C_SAFETY)

# A schedule that is frozen (k == 0) at every range these tests use, so
# psi_c_new == psi_c exactly.
FROZEN = dict(r_full=101.0, r_freeze=100.0)

FLAG_GEOMETRY = dict(wheelbase=WHEELBASE, delta_min=DELTA_MIN, delta_max=DELTA_MAX)


def _plan(psi_c, target, car=(0.0, 0.0), yaw=0.0, standoff=1.0, **overrides):
    kwargs = dict(KINEMATICS)
    kwargs.update(SCHEDULE)
    kwargs.update(overrides)
    return plan_object_heading(psi_c, target, car, yaw, standoff, **kwargs)


def _assess(psi_c, target, car=(0.0, 0.0), yaw=0.0, standoff=1.0,
            r_freeze=R_FREEZE, heading_margin=DEFAULT_HEADING_MARGIN_RAD):
    return assess_object_approach(
        psi_c, target, car, yaw, standoff, r_freeze=r_freeze,
        heading_margin=heading_margin, **FLAG_GEOMETRY)


def _target_at(range_m, bearing_rad, standoff=1.0):
    """A target whose STANDOFF range is `range_m` at the given bearing."""
    d = range_m + standoff
    return (d * math.cos(bearing_rad), d * math.sin(bearing_rad))


# ---------------------------------------------------------------- k schedule

class TestDistanceSchedule:

    def test_k_is_one_at_and_beyond_r_full(self):
        for r in (R_FULL, 2.0, 5.0):
            step = _plan(0.0, _target_at(r, 0.3))
            assert step.k == pytest.approx(1.0)

    def test_k_is_zero_at_and_inside_r_freeze(self):
        for r in (R_FREEZE, 0.2, 0.0, -0.3):
            step = _plan(0.0, _target_at(r, 0.3))
            assert step.k == pytest.approx(0.0)

    def test_k_is_linear_between_the_two_radii(self):
        midpoint = 0.5 * (R_FULL + R_FREEZE)
        step = _plan(0.0, _target_at(midpoint, 0.1))
        assert step.k == pytest.approx(0.5, abs=1e-9)

    def test_k_matches_the_formula_across_the_ramp(self):
        for r in (0.5, 0.7, 0.9, 1.1, 1.3):
            step = _plan(0.0, _target_at(r, 0.05))
            expected = (r - R_FREEZE) / (R_FULL - R_FREEZE)
            assert step.k == pytest.approx(expected, abs=1e-9)

    def test_a_frozen_schedule_leaves_the_heading_exactly_unchanged(self):
        """Inside r_freeze the corridor is rigid -- bit-for-bit, not nearly."""
        psi_c = 0.21
        step = _plan(psi_c, _target_at(0.2, 0.9))
        assert step.psi_c_new == psi_c

    def test_r_full_not_above_r_freeze_is_rejected(self):
        with pytest.raises(ValueError, match='r_full'):
            _plan(0.0, _target_at(2.0, 0.0), r_full=0.4, r_freeze=0.4)


# ------------------------------------------------------------- kinematic cap

class TestKinematicCap:

    def test_the_cap_binds_at_small_r(self):
        """Close in, the distance term is the binding one."""
        r = 0.6
        step = _plan(0.0, _target_at(r, 1.2))
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        assert step.dpsi_max == pytest.approx(r / (C_SAFETY * r_min), abs=1e-9)
        assert step.dpsi_max < horizon_heading_reach(N_STEPS, TS, V_REF, r_min)

    def test_the_horizon_binds_far_away(self):
        """Far out, the distance term is huge and the horizon is the cap."""
        step = _plan(0.0, _target_at(8.0, 1.2))
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        assert step.dpsi_max == pytest.approx(
            horizon_heading_reach(N_STEPS, TS, V_REF, r_min), abs=1e-9)

    def test_a_large_error_is_clamped_to_the_cap(self):
        step = _plan(0.0, _target_at(8.0, 1.4))
        assert abs(step.psi_c_new - 0.0) == pytest.approx(step.dpsi_max, abs=1e-9)

    def test_a_small_error_is_not_clamped(self):
        step = _plan(0.0, _target_at(8.0, 0.01))
        assert abs(step.psi_c_new) < step.dpsi_max
        assert step.psi_c_new == pytest.approx(step.k * step.e, abs=1e-9)

    def test_the_cap_never_goes_negative_inside_the_standoff(self):
        """r < 0 would invert the clamp into a minimum rotation."""
        step = _plan(0.0, _target_at(-0.5, 1.0))
        assert step.dpsi_max >= 0.0
        assert step.psi_c_new == 0.0      # k is 0 there anyway

    def test_each_direction_uses_its_own_steering_bound(self):
        """delta_min and delta_max differ, so R_min does too."""
        left = _plan(0.0, _target_at(8.0, 1.0))
        right = _plan(0.0, _target_at(8.0, -1.0))
        r_left = min_turn_radius(WHEELBASE, DELTA_MAX)
        r_right = min_turn_radius(WHEELBASE, DELTA_MIN)
        assert r_left != r_right
        assert left.dpsi_max == pytest.approx(
            horizon_heading_reach(N_STEPS, TS, V_REF, r_left), abs=1e-12)
        assert right.dpsi_max == pytest.approx(
            horizon_heading_reach(N_STEPS, TS, V_REF, r_right), abs=1e-12)

    def test_c_safety_below_one_is_rejected(self):
        with pytest.raises(ValueError, match='c_safety'):
            _plan(0.0, _target_at(2.0, 0.0), c_safety=0.9)

    def test_c_safety_between_one_and_the_wall_turn_floor_is_allowed(self):
        """1.5 is wall_turn's smoothstep floor; this corridor is straight."""
        step = _plan(0.0, _target_at(2.0, 0.2), c_safety=1.0)
        assert step.dpsi_max > 0.0


# ------------------------------------------------- psi_c is an ABSOLUTE state

class TestAbsoluteHeldHeading:

    def test_a_constant_bearing_converges_and_then_stops_moving(self):
        """The property the whole design exists for: psi_c does not drift."""
        target = _target_at(3.0, 0.5)
        psi_c = 0.0
        for _ in range(200):
            psi_c = _plan(psi_c, target).psi_c_new
        settled = psi_c
        for _ in range(200):
            psi_c = _plan(psi_c, target).psi_c_new
        assert psi_c == settled, 'psi_c kept moving after convergence'
        assert psi_c == pytest.approx(0.5, abs=1e-6)

    def test_yaw_does_not_enter_the_error_term(self):
        """e is measured against psi_c, never against the car's heading."""
        target = _target_at(3.0, 0.5)
        errors = {
            yaw: _plan(0.2, target, yaw=yaw).e
            for yaw in (-1.0, -0.3, 0.0, 0.3, 1.0)
        }
        assert len(set(errors.values())) == 1

    def test_a_car_that_has_not_turned_yet_still_owes_the_same_rotation(self):
        """Under a live-yaw error term this would shrink for free."""
        target = _target_at(3.0, 0.6)
        first = _plan(0.0, target, yaw=0.0)
        # Same psi_c, car now pointed at the target: e must be unchanged.
        second = _plan(0.0, target, yaw=0.6)
        assert second.e == pytest.approx(first.e, abs=1e-12)

    def test_overshooting_psi_c_does_not_make_the_corridor_fight_itself(self):
        target = _target_at(3.0, 0.0)
        step = _plan(0.0, target, yaw=0.8)
        assert step.e == pytest.approx(0.0, abs=1e-12)
        assert step.psi_c_new == 0.0

    def test_the_step_never_exceeds_the_error_it_was_given(self):
        """k <= 1 and the clamp only shrinks, so no rebuild overshoots."""
        for bearing in (0.02, 0.2, 0.9, 1.4, -0.5):
            for r in (0.5, 1.0, 2.0, 6.0):
                step = _plan(0.0, _target_at(r, bearing))
                assert abs(step.psi_c_new) <= abs(step.e) + 1e-12


# --------------------------------------------------------------- wraparound

class TestWraparound:

    def test_a_target_just_across_pi_turns_the_short_way(self):
        """psi_c near +pi, bearing near -pi: the error is small, not 2pi."""
        psi_c = math.pi - 0.05
        target = _target_at(6.0, -math.pi + 0.05)
        step = _plan(psi_c, target, yaw=math.pi)
        assert step.e == pytest.approx(0.1, abs=1e-9)
        assert step.psi_c_new > 0.0 or step.psi_c_new < -math.pi + 0.2

    def test_psi_c_new_is_always_wrapped(self):
        psi_c = math.pi - 1e-3
        for _ in range(50):
            psi_c = _plan(psi_c, _target_at(6.0, -math.pi + 0.3),
                          yaw=math.pi).psi_c_new
            assert -math.pi < psi_c <= math.pi

    def test_crossing_the_seam_does_not_reverse_direction(self):
        """Stepping across +pi must keep going the same way round."""
        target = _target_at(6.0, -math.pi + 0.2)
        psi_c = math.pi - 0.02
        first = _plan(psi_c, target, yaw=math.pi)
        second = _plan(first.psi_c_new, target, yaw=math.pi)
        # Both steps shrink the same wrapped error.
        assert abs(second.e) < abs(first.e)

    def test_the_error_is_never_larger_than_pi(self):
        for bearing in (-3.0, -1.5, 0.0, 1.5, 3.0):
            for psi_c in (-3.0, -1.0, 0.0, 1.0, 3.0):
                step = _plan(psi_c, _target_at(5.0, bearing), yaw=bearing)
                assert -math.pi < step.e <= math.pi


# ------------------------------------------------------------- infeasibility

class TestTargetBehind:
    """The raw per-tick flag. Terminal only through TargetBehindPersistence."""

    def test_a_target_directly_astern_is_behind(self):
        assert _assess(0.0, _target_at(3.0, math.pi), yaw=0.0).target_behind is True

    def test_just_past_abeam_is_behind(self):
        assert _assess(0.0, _target_at(3.0, math.pi / 2 + 0.02)).target_behind is True

    def test_just_inside_abeam_is_not_behind(self):
        assert _assess(0.0, _target_at(6.0, math.pi / 2 - 0.02)).target_behind is False

    def test_behind_is_relative_to_yaw_not_to_psi_c(self):
        """The car cannot reverse; where the corridor points is irrelevant."""
        target = _target_at(3.0, math.pi)
        assert _assess(math.pi, target, yaw=0.0).target_behind is True
        assert _assess(0.0, target, yaw=math.pi).target_behind is False

    def test_alpha_is_bearing_minus_yaw_wrapped(self):
        flags = _assess(0.0, _target_at(3.0, 3.0), yaw=-3.0)
        assert flags.alpha == pytest.approx(math.atan2(math.sin(6.0), math.cos(6.0)))

    def test_plan_object_heading_still_returns_a_usable_heading_astern(self):
        step = _plan(0.0, _target_at(3.0, math.pi), yaw=0.0)
        assert math.isfinite(step.psi_c_new)


class TestTargetBehindPersistence:

    def test_not_terminal_before_the_persistence_time(self):
        p = TargetBehindPersistence(0.5)
        assert p.update(True, 10.0) == (False, 0.0)
        terminal, held = p.update(True, 10.4)
        assert terminal is False and held == pytest.approx(0.4)

    def test_terminal_at_the_persistence_time(self):
        p = TargetBehindPersistence(0.5)
        p.update(True, 10.0)
        assert p.update(True, 10.5)[0] is True

    def test_tick_times_built_from_a_float_period_still_reach_it(self):
        """5 * 0.1 is not exactly 0.5 after subtraction; 5 ticks must do."""
        p = TargetBehindPersistence(0.5)
        results = [p.update(True, tick * 0.1)[0] for tick in range(123, 129)]
        assert results == [False] * 5 + [True]

    def test_a_shorter_excursion_is_never_terminal_and_resets(self):
        p = TargetBehindPersistence(0.5)
        for t in (0.0, 0.1, 0.2, 0.3, 0.4):
            assert p.update(True, t)[0] is False
        assert p.update(False, 0.5) == (False, 0.0)
        assert p.update(True, 0.6) == (False, 0.0)
        assert p.update(True, 1.0)[0] is False

    def test_it_is_not_a_latch(self):
        p = TargetBehindPersistence(0.5)
        p.update(True, 0.0)
        assert p.update(True, 0.6)[0] is True
        assert p.update(False, 0.7)[0] is False

    def test_reset_starts_the_clock_again(self):
        p = TargetBehindPersistence(0.5)
        p.update(True, 0.0)
        p.reset()
        assert p.update(True, 0.6) == (False, 0.0)

    def test_zero_persistence_is_immediately_terminal(self):
        assert TargetBehindPersistence(0.0).update(True, 3.0)[0] is True

    def test_negative_persistence_is_rejected(self):
        with pytest.raises(ValueError):
            TargetBehindPersistence(-0.1)


class TestHeadingMargin:

    def test_it_is_the_two_sigma_goal_bearing_error_at_r_freeze(self):
        assert heading_margin_for(0.4) == pytest.approx(math.atan(2 * 0.05 / 0.4))
        assert math.degrees(DEFAULT_HEADING_MARGIN_RAD) == pytest.approx(14.036, abs=1e-3)

    def test_it_moves_with_r_freeze(self):
        assert heading_margin_for(0.8) < heading_margin_for(0.4) < heading_margin_for(0.2)

    def test_a_non_positive_r_freeze_has_no_margin(self):
        with pytest.raises(ValueError):
            heading_margin_for(0.0)


class TestInsideTurnRadius:
    """ADVISORY. Judged on the goal implied by the HELD psi_c, per tick."""

    def _goal_target(self, goal_x, goal_y, psi_c=0.0, standoff=1.0):
        """The target whose goal (standoff back along psi_c) is (goal_x, goal_y)."""
        return (goal_x + standoff * math.cos(psi_c), goal_y + standoff * math.sin(psi_c))

    def test_a_goal_abeam_and_close_is_unreachable(self):
        """|sin a| = 1, threshold 2 * R_min; the margin barely matters abeam."""
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        target = self._goal_target(0.0, 0.9 * 2.0 * r_min)
        assert _assess(0.0, target).inside_turn_radius is True

    def test_a_goal_dead_ahead_is_reachable_at_any_range(self):
        """r < R_min would wrongly reject this; the circle test does not."""
        for goal_range in (0.05, 0.2, 0.5, 1.0):
            flags = _assess(0.0, self._goal_target(goal_range, 0.0), r_freeze=0.0)
            assert flags.inside_turn_radius is False, goal_range

    @pytest.mark.parametrize('margin', [0.0, DEFAULT_HEADING_MARGIN_RAD])
    def test_the_threshold_is_two_r_min_sin_of_alpha_less_the_margin(self, margin):
        """Straddle r_goal = 2 * R_min * sin(|alpha| - margin) from both sides."""
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        alpha = 0.8
        threshold = 2.0 * r_min * math.sin(alpha - margin)
        for factor, want_flag in ((0.95, True), (1.05, False)):
            gx = factor * threshold * math.cos(alpha)
            gy = factor * threshold * math.sin(alpha)
            flags = _assess(0.0, self._goal_target(gx, gy), r_freeze=0.0,
                            heading_margin=margin)
            assert flags.inside_turn_radius is want_flag, (factor, margin)

    def test_the_margin_clears_a_goal_the_bare_test_would_flag(self):
        """The rig's one spurious tick: alpha_goal 16.3 deg at r_goal 0.44 m."""
        alpha = math.radians(16.3)
        r_goal = 0.441
        target = self._goal_target(r_goal * math.cos(alpha), r_goal * math.sin(alpha))
        assert _assess(0.0, target, r_freeze=0.0, heading_margin=0.0).inside_turn_radius
        assert not _assess(0.0, target, r_freeze=0.0).inside_turn_radius

    def test_it_is_never_evaluated_at_or_inside_r_freeze(self):
        """Abeam and 0.3 m away would be unreachable; at r <= r_freeze it is not asked."""
        target = self._goal_target(0.0, 0.3)
        r = math.hypot(*target) - 1.0
        assert _assess(0.0, target, r_freeze=r + 0.01).inside_turn_radius is False
        assert _assess(0.0, target, r_freeze=r - 0.01).inside_turn_radius is True

    def test_arrival_never_raises_it(self):
        """The old r <= 0 special case, now subsumed by r_freeze."""
        for r in (0.0, 0.02, -0.05):
            flags = _assess(0.3, _target_at(r, 1.2), yaw=0.0)
            assert flags.inside_turn_radius is False, r

    def test_a_negative_r_freeze_is_rejected(self):
        with pytest.raises(ValueError):
            _assess(0.0, _target_at(1.0, 0.0), r_freeze=-0.1)

    def test_the_circle_is_chosen_by_the_goal_side(self):
        """Right-hand goal: R_min from delta_min (the tighter bound here)."""
        r_left = min_turn_radius(WHEELBASE, DELTA_MAX)
        r_right = min_turn_radius(WHEELBASE, DELTA_MIN)
        assert r_right < r_left
        alpha = -0.9
        # Between the two thresholds: unreachable on the left circle's radius,
        # reachable on the right's.
        r_goal = 2.0 * math.sin(abs(alpha)) * (r_left + r_right) / 2.0
        target = self._goal_target(r_goal * math.cos(alpha), r_goal * math.sin(alpha))
        assert _assess(0.0, target, r_freeze=0.0, heading_margin=0.0).inside_turn_radius is False

    def test_behind_takes_precedence(self):
        """A target astern also fails the circle test; only behind is reported."""
        flags = _assess(0.0, _target_at(1.5, math.pi), yaw=0.0)
        assert flags.target_behind is True
        assert flags.inside_turn_radius is False

    def test_a_normal_approach_raises_nothing(self):
        for r in (0.5, 1.0, 2.0, 4.0):
            for bearing in (-0.4, 0.0, 0.4):
                flags = _assess(bearing, _target_at(r, bearing), yaw=bearing)
                assert not flags.inside_turn_radius and not flags.target_behind, (r, bearing)


# ------------------------------------------------------------------- speed

class TestStopLatch:
    """Trips once live r reaches reach_tol + stop_distance, then holds for the move."""

    def test_the_trigger_is_reach_tol_plus_stop_distance(self):
        """The trip range is the sum of the two stack_params keys."""
        assert ObjectStopLatch(0.10, 0.14).trigger_r == pytest.approx(0.24)

    def test_it_does_not_trip_above_the_trigger(self):
        """Anything short of the trigger leaves the approach driving."""
        latch = ObjectStopLatch(0.10, 0.14)
        for r in (3.0, 1.0, 0.5, 0.2401):
            assert latch.update(r) is False
        assert latch.latched_r is None

    def test_it_trips_at_the_trigger_and_records_where(self):
        """At the trigger it trips, and the r it tripped at is kept for the status."""
        latch = ObjectStopLatch(0.10, 0.14)
        latch.update(0.30)
        assert latch.update(0.24) is True
        assert latch.latched_r == pytest.approx(0.24)

    def test_it_holds_when_r_grows_again(self):
        """Jitter, a person stepping back, the last centimetres: none re-arm it."""
        latch = ObjectStopLatch(0.10, 0.14)
        latch.update(0.2)
        for r in (0.5, 2.0, 10.0):
            assert latch.update(r) is True
        assert latch.latched_r == pytest.approx(0.2)

    def test_reset_starts_a_new_move_driving(self):
        """A new move clears the latch and its recorded range."""
        latch = ObjectStopLatch(0.10, 0.14)
        latch.update(0.0)
        latch.reset()
        assert latch.latched is False and latch.latched_r is None
        assert latch.update(1.0) is False

    def test_a_non_finite_r_never_trips_it(self):
        """No target yet reads as infinite range."""
        latch = ObjectStopLatch(0.10, 0.14)
        assert latch.update(math.inf) is False
        assert latch.update(math.nan) is False

    def test_a_zero_stop_distance_is_rejected(self):
        """Zero would trip at the reach tolerance itself and coast past it."""
        with pytest.raises(ValueError, match='stop_distance_m'):
            ObjectStopLatch(0.10, 0.0)

    def test_a_negative_reach_tolerance_is_rejected(self):
        """A negative tolerance is a configuration error, named."""
        with pytest.raises(ValueError, match='reach_tol_m'):
            ObjectStopLatch(-0.01, 0.14)


class TestSpeedDecision:
    """The move's speed or zero -- never anything between zero and the floor."""

    FLOOR = 0.4

    def test_driving_uses_the_move_speed_as_the_reference(self):
        """Not a ramp: the reference is the move speed itself."""
        d = object_speed_decision(0.4, False, self.FLOOR)
        assert (d.mode, d.speed_ref) == (SPEED_DRIVE, 0.4)

    def test_a_latched_stop_is_zero_whatever_the_goal_says(self):
        """The latch wins over a goal that still asks to drive."""
        d = object_speed_decision(0.5, True, self.FLOOR)
        assert (d.mode, d.speed_ref) == (SPEED_STOP_LATCHED, 0.0)

    def test_a_zero_goal_speed_is_stop_and_wait(self):
        """The handler acquiring, or holding the last point of a lost track."""
        d = object_speed_decision(0.0, False, self.FLOOR)
        assert (d.mode, d.speed_ref) == (SPEED_WAIT, 0.0)

    def test_a_speed_below_the_floor_stops_rather_than_crawls(self):
        """0.2 was the old grace speed: it must stop the car, not drive it at 0.2 or 0.4."""
        d = object_speed_decision(0.2, False, self.FLOOR)
        assert (d.mode, d.speed_ref) == (SPEED_BELOW_FLOOR, 0.0)

    def test_exactly_the_floor_drives(self):
        """The floor is inclusive."""
        assert object_speed_decision(0.4, False, self.FLOOR).mode == SPEED_DRIVE

    def test_no_decision_ever_asks_for_a_speed_between_zero_and_the_floor(self):
        """The operating constraint itself, over goals on both sides of the floor."""
        for goal in (0.0, 0.05, 0.2, 0.39, 0.4, 0.45, 0.5):
            for latched in (False, True):
                ref = object_speed_decision(goal, latched, self.FLOOR).speed_ref
                assert ref == 0.0 or ref >= self.FLOOR, (goal, latched, ref)


class TestFloorMovingSpeed:
    """The published command while driving: never below the floor."""

    def test_a_command_below_the_floor_is_raised_to_it(self):
        """The solver's first tick from rest is 0.3 (a_max 3.0 over 0.1 s)."""
        assert floor_moving_speed(0.3, 0.4) == 0.4
        assert floor_moving_speed(-0.2, 0.4) == 0.4

    def test_a_command_above_the_floor_passes_through(self):
        """The top of the range is the /drive clamp's business, not this one's."""
        assert floor_moving_speed(0.45, 0.4) == 0.45
        assert floor_moving_speed(0.65, 0.4) == 0.65


# ------------------------------------------------------------- goal geometry

class TestGoalPoint:

    def test_it_is_standoff_short_along_psi_c(self):
        g = goal_point((3.0, 0.0), 0.0, 1.0)
        assert g == pytest.approx((2.0, 0.0))

    def test_it_follows_psi_c_not_the_line_from_the_car(self):
        g = goal_point((0.0, 3.0), math.pi / 2, 1.0)
        assert g[0] == pytest.approx(0.0, abs=1e-9)
        assert g[1] == pytest.approx(2.0)

    def test_it_never_lands_past_the_target(self):
        for psi in (-2.0, -0.5, 0.0, 0.5, 2.0):
            target = (4.0, 1.0)
            g = goal_point(target, psi, 1.0)
            assert math.hypot(g[0] - target[0], g[1] - target[1]) == pytest.approx(1.0)


class TestCentreline:

    def test_it_ends_at_the_goal(self):
        (x0, y0), psi, length = build_object_centreline(
            (0.0, 0.0), (4.0, 0.0), 0.0, 1.0, 40)
        end = (x0 + length * math.cos(psi), y0 + length * math.sin(psi))
        assert end == pytest.approx(goal_point((4.0, 0.0), 0.0, 1.0))

    def test_it_starts_behind_the_car(self):
        (x0, _y0), psi, _length = build_object_centreline(
            (0.0, 0.0), (4.0, 0.0), 0.0, 1.0, 40, behind=0.5)
        assert x0 == pytest.approx(-0.5)

    def test_there_is_no_length_floor(self):
        """The goal_pose branch clips to [1.0, corr_L_base]; this does not."""
        _origin, _psi, length = build_object_centreline(
            (1.8, 0.0), (2.0, 0.0), 0.0, 1.0, 40, behind=0.5)
        assert length < 1.0

    def test_the_line_passes_through_the_target(self):
        origin, psi, _length = build_object_centreline(
            (0.0, 0.0), (3.0, 2.0), 0.4, 1.0, 40)
        # Cross-track distance of the target from the line must be zero.
        dx, dy = 3.0 - origin[0], 2.0 - origin[1]
        cross = -dx * math.sin(psi) + dy * math.cos(psi)
        assert cross == pytest.approx(0.0, abs=1e-9)

    def test_a_car_off_to_the_side_still_gets_an_interior_projection(self):
        """The whole point of the lead-in: index 0 must not be the nearest."""
        origin, psi, length = build_object_centreline(
            (0.0, 1.5), (4.0, 0.0), 0.0, 1.0, 40, behind=0.5)
        s_car = ((0.0 - origin[0]) * math.cos(psi)
                 + (1.5 - origin[1]) * math.sin(psi))
        assert 0.0 < s_car < length


class TestApproachLookahead:

    def test_it_reports_at_end_instead_of_warning(self):
        _la, at_end = approach_lookahead(
            r=0.2, corridor_length=0.8, lookahead_frac=0.5,
            reach_floor=1.25, arrival_band=0.3)
        assert at_end is True

    def test_a_long_corridor_is_not_at_end(self):
        la, at_end = approach_lookahead(
            r=3.0, corridor_length=4.0, lookahead_frac=0.5,
            reach_floor=1.25, arrival_band=0.3)
        assert at_end is False
        assert la == pytest.approx(2.0)

    def test_the_reach_floor_still_applies(self):
        la, _at_end = approach_lookahead(
            r=3.0, corridor_length=2.0, lookahead_frac=0.5,
            reach_floor=1.25, arrival_band=0.3)
        assert la == pytest.approx(1.25)


class TestMinStandoff:

    def test_it_is_r_safe_as_compute_local_target_forms_it(self):
        assert min_standoff_clear_of(0.85, 0.20, 0.12) == pytest.approx(1.17)

    def test_legacy_height_radius_person_needs_more_than_one_metre(self):
        """obstacle_radius_source legacy: r = max(width, HEIGHT) / 2."""
        radius = max(0.50, 1.70) / 2.0
        assert min_standoff_clear_of(radius, 0.20, 0.12) > 1.0

    def test_footprint_radius_person_clears_at_one_metre(self):
        """obstacle_radius_source footprint (the default): r = width / 2."""
        radius = 0.50 / 2.0
        assert min_standoff_clear_of(radius, 0.20, 0.12) == pytest.approx(0.57)

    @pytest.mark.parametrize('class_margin, r_safe', [(0.2, 0.77), (0.3, 0.87), (0.4, 0.97)])
    def test_footprint_person_with_class_margin_still_clears_one_metre(self, class_margin,
                                                                       r_safe):
        """obstacle_class_margin_m enters through r, once."""
        radius = 0.50 / 2.0 + class_margin
        assert min_standoff_clear_of(radius, 0.20, 0.12) == pytest.approx(r_safe)
        assert r_safe < 1.0
