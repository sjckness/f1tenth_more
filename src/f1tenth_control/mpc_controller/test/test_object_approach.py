"""object_approach.py -- the held corridor heading, its schedule and its caps.

Pure-function tests, same convention as test_wall_turn_increment.py: no
rclpy, no node, no hardware. Every number here is either derived in the
assertion itself or taken from the shipped parameter set, so a retune shows
up as a failing test rather than as a silently different vehicle.

The shipped geometry these use:
    wheelbase  0.305   delta_min -0.283   delta_max +0.278
    N 20        ts 0.1  v_ref 0.5   ->  horizon reach 1.0 m
    R_min      1.0685 m turning left, 1.0483 m turning right
    r_full 1.5  r_freeze 0.4  c_safety 1.5  a_dec 0.3
"""

import math

import pytest

from mpc_controller.object_approach import (
    REASON_INSIDE_TURN_RADIUS,
    REASON_TARGET_BEHIND,
    approach_lookahead,
    build_object_centreline,
    goal_point,
    min_standoff_clear_of,
    object_speed_ref,
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
# psi_c_new == psi_c exactly and a test can place the goal where it means to.
# Needed because feasibility is judged on the goal implied by psi_c_NEW.
FROZEN = dict(r_full=101.0, r_freeze=100.0)


def _plan(psi_c, target, car=(0.0, 0.0), yaw=0.0, standoff=1.0, **overrides):
    kwargs = dict(KINEMATICS)
    kwargs.update(SCHEDULE)
    kwargs.update(overrides)
    return plan_object_heading(psi_c, target, car, yaw, standoff, **kwargs)


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

    def test_a_target_directly_astern_is_infeasible(self):
        step = _plan(0.0, _target_at(3.0, math.pi), yaw=0.0)
        assert step.feasible is False
        assert step.reason == REASON_TARGET_BEHIND

    def test_just_past_abeam_is_behind(self):
        step = _plan(0.0, _target_at(3.0, math.pi / 2 + 0.02), yaw=0.0)
        assert step.feasible is False
        assert step.reason == REASON_TARGET_BEHIND

    def test_just_inside_abeam_is_not_behind_for_that_reason(self):
        step = _plan(0.0, _target_at(6.0, math.pi / 2 - 0.02), yaw=0.0)
        assert step.reason != REASON_TARGET_BEHIND

    def test_behind_is_relative_to_yaw_not_to_psi_c(self):
        """The car cannot reverse; where the corridor points is irrelevant."""
        target = _target_at(3.0, math.pi)
        assert _plan(math.pi, target, yaw=0.0).reason == REASON_TARGET_BEHIND
        assert _plan(0.0, target, yaw=math.pi).feasible is True

    def test_a_still_usable_heading_is_returned_anyway(self):
        step = _plan(0.0, _target_at(3.0, math.pi), yaw=0.0)
        assert math.isfinite(step.psi_c_new)


class TestInsideTurnRadius:

    def test_a_goal_abeam_and_close_is_unreachable(self):
        """The worst case: |sin a| = 1, threshold 2 * R_min.

        FROZEN is not decoration. Feasibility is judged on the goal implied by
        psi_c_NEW -- the heading the corridor will actually be built along this
        rebuild -- so a live schedule would move the goal out from under the
        geometry this test is placing. Holding k at 0 makes psi_c_new == psi_c
        exactly, and the goal is then where it was put.
        """
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        goal_range = 0.9 * 2.0 * r_min
        standoff = 1.0
        target = (0.0 + standoff, goal_range)      # psi_c = 0 -> goal = target - x
        step = _plan(0.0, target, yaw=0.0, standoff=standoff, **FROZEN)
        assert step.k == 0.0
        assert step.feasible is False
        assert step.reason == REASON_INSIDE_TURN_RADIUS

    def test_a_goal_dead_ahead_is_reachable_at_any_range(self):
        """r < R_min would wrongly reject this; the circle test does not."""
        for goal_range in (0.05, 0.2, 0.5, 1.0):
            standoff = 1.0
            target = (goal_range + standoff, 0.0)
            step = _plan(0.0, target, yaw=0.0, standoff=standoff)
            assert step.feasible is True, f'rejected a goal {goal_range} m dead ahead'

    def test_the_threshold_is_exactly_two_r_min_sin_alpha(self):
        """Straddle the derived threshold from both sides. FROZEN -- see above."""
        r_min = min_turn_radius(WHEELBASE, DELTA_MAX)
        alpha = 0.6
        threshold = 2.0 * r_min * abs(math.sin(alpha))
        standoff = 1.0
        for factor, want_feasible in ((0.95, False), (1.05, True)):
            gx = factor * threshold * math.cos(alpha)
            gy = factor * threshold * math.sin(alpha)
            target = (gx + standoff, gy)      # psi_c = 0 -> goal = target - x
            step = _plan(0.0, target, yaw=0.0, standoff=standoff, **FROZEN)
            assert (step.reason != REASON_INSIDE_TURN_RADIUS) is want_feasible, (
                f'factor {factor}: goal at range '
                f'{math.hypot(gx, gy):.4f}, threshold {threshold:.4f}')

    def test_behind_is_reported_before_inside_turn_radius(self):
        """A target astern also fails the circle test; the name must not lie."""
        step = _plan(0.0, _target_at(0.1, math.pi), yaw=0.0)
        assert step.reason == REASON_TARGET_BEHIND

    def test_a_normal_approach_is_feasible(self):
        for r in (0.5, 1.0, 2.0, 4.0):
            for bearing in (-0.4, 0.0, 0.4):
                step = _plan(bearing, _target_at(r, bearing), yaw=bearing)
                assert step.feasible is True, f'r={r} bearing={bearing}'


# ------------------------------------------------------------------- speed

class TestSpeedRef:

    def test_it_is_zero_at_the_standoff(self):
        assert object_speed_ref(0.0, 0.5, 0.3) == 0.0

    def test_it_is_zero_inside_the_standoff(self):
        assert object_speed_ref(-0.4, 0.5, 0.3) == 0.0

    def test_it_saturates_at_the_move_speed_far_out(self):
        assert object_speed_ref(10.0, 0.5, 0.3) == pytest.approx(0.5)

    def test_it_follows_the_braking_parabola_in_between(self):
        for r in (0.05, 0.1, 0.2, 0.4):
            expected = min(0.5, math.sqrt(2.0 * 0.3 * r))
            assert object_speed_ref(r, 0.5, 0.3) == pytest.approx(expected)

    def test_it_is_monotone_in_range(self):
        speeds = [object_speed_ref(r / 100.0, 0.5, 0.3) for r in range(0, 200)]
        assert speeds == sorted(speeds)

    def test_a_non_positive_a_dec_disables_the_ramp_visibly(self):
        assert object_speed_ref(0.1, 0.5, 0.0) == 0.5
        assert object_speed_ref(0.1, 0.5, -1.0) == 0.5

    def test_the_ramp_can_actually_stop_in_the_distance_it_leaves(self):
        """v^2 / (2 a) <= r at every range -- the defining property."""
        for r in (0.05, 0.3, 0.8, 1.5):
            v = object_speed_ref(r, 0.5, 0.3)
            assert v * v / (2.0 * 0.3) <= r + 1e-9


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

    def test_a_standing_person_needs_more_than_one_metre(self):
        """bbox.size.y is the object's HEIGHT, and the radius is max(w,h)/2."""
        radius = 1.70 / 2.0
        assert min_standoff_clear_of(radius, 0.20, 0.12) > 1.0
