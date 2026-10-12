"""Speed floor, keep-outs and the BLOCKED detector (M02 campaign fix A3) -- units."""

import math

import numpy as np
import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.blocked import (
    BlockedDetector, BodyBox, first_floor_stage, footprint_circles, gate_speed,
    plan_body_gap, plan_keepouts)
from mpc_controller.pass_side import PassSideMemory
from mpc_controller.mpc_solver import solve_mpc_step

import test_object_approach_closed_loop as rig
from test_prediction_dt_split import _straight_corridor

BODY = BodyBox()
FLOOR = 0.4


class TestGateSpeed:

    @pytest.mark.parametrize('v_cmd', [0.0, 0.01, 0.09, 0.13, 0.399])
    def test_anything_below_the_floor_is_raised_to_it(self, v_cmd):
        assert gate_speed(v_cmd, FLOOR) == pytest.approx(FLOOR)

    @pytest.mark.parametrize('v_cmd', [0.4, 0.45, 0.8])
    def test_at_or_above_the_floor_is_unchanged(self, v_cmd):
        assert gate_speed(v_cmd, FLOOR) == pytest.approx(v_cmd)

    @pytest.mark.parametrize('v_cmd', [math.nan, math.inf, -math.inf])
    def test_a_non_finite_command_is_a_stop(self, v_cmd):
        assert gate_speed(v_cmd, FLOOR) == 0.0

    def test_the_floor_is_the_shipped_operating_floor(self):
        assert get_value('min_moving_speed_mps') == pytest.approx(0.4)
        assert get_value('mpc_min_speed_all_moves') is True


class TestFirstFloorStage:

    def test_already_at_the_floor(self):
        assert first_floor_stage(0.45, 0.0, FLOOR, 3.0, 2.0, 0.2, 0.1, 20) == 1

    def test_from_rest_the_accel_rate_decides(self):
        # a: 0.2, 0.6, 1.0, 1.4 -> v: 0.04, 0.16, 0.36, 0.64
        assert first_floor_stage(0.0, 0.0, FLOOR, 3.0, 2.0, 0.2, 0.1, 20) == 4

    def test_out_of_reach_is_none(self):
        assert first_floor_stage(0.0, -2.0, FLOOR, 0.01, 0.01, 0.2, 0.1, 5) is None


def _corridor(half_width=0.6):
    return _straight_corridor(length=4.0, half_width=half_width)


def _keepouts(obstacles, car=(0.0, 0.0), memory=None, half_width=0.6, now=0.0,
              circles=None):
    return plan_keepouts(obstacles, _corridor(half_width), car, 0.20, 0.12, BODY,
                         reach_m=1.6, memory=memory, now=now, circles=circles)


class TestPlanKeepouts:

    def test_a_centred_plant_ahead_of_a_centred_car_passes_on_the_left(self):
        plan = _keepouts([(1.4, 0.0, 0.15)])
        assert len(plan.keepouts) == 1 and not plan.blocked
        ko = plan.keepouts[0]
        assert ko.side == 1
        assert ko.d == pytest.approx(0.15 + 0.20 + 0.12)

    def test_the_car_already_right_of_it_keeps_right(self):
        plan = _keepouts([(1.4, 0.0, 0.15)], car=(0.0, -0.1))
        assert plan.keepouts[0].side == -1

    def test_a_committed_side_is_kept_when_the_costs_flip(self):
        """Review item 2: c jitters with the detection; the side must not."""
        memory = PassSideMemory()
        first = _keepouts([(1.4, 0.0, 0.15)], car=(0.0, -0.05), memory=memory)
        assert first.keepouts[0].side == -1
        # The detection jitters 0.1 m to the right: on costs alone the left
        # would now be nearer. The commitment holds.
        again = _keepouts([(1.4, -0.10, 0.15)], car=(0.0, -0.05), memory=memory, now=0.1)
        assert again.keepouts[0].side == -1
        assert [r['reason'] for r in again.report] == ['committed']

    def test_a_committed_side_that_stops_fitting_switches(self):
        memory = PassSideMemory()
        _keepouts([(1.4, 0.0, 0.15)], memory=memory)          # left (tie -> left)
        # It turns out to be wider and sits left: left no longer fits.
        plan = _keepouts([(1.4, 0.25, 0.15)], memory=memory, now=0.1)
        assert plan.keepouts[0].side == -1
        assert plan.report[0]['reason'] == 'switched'

    def test_a_forgotten_obstacle_is_decided_afresh(self):
        memory = PassSideMemory(forget_after_s=2.0)
        _keepouts([(1.4, 0.0, 0.15)], car=(0.0, -0.05), memory=memory)
        plan = _keepouts([(1.4, 0.0, 0.15)], car=(0.0, 0.05), memory=memory, now=3.0)
        assert plan.keepouts[0].side == 1
        assert plan.report[0]['reason'] == 'new'

    def test_an_offset_obstacle_is_passed_on_the_side_that_fits(self):
        # c = +0.3, d = 0.47: left needs +0.77 > 0.6, right needs -0.17.
        plan = _keepouts([(1.4, 0.3, 0.15)])
        assert plan.keepouts[0].side == -1

    def test_a_metre_wide_object_centred_fits_on_no_side(self):
        # d = 0.5 + 0.32 = 0.82 > 0.6 both ways.
        plan = _keepouts([(1.4, 0.0, 0.5)])
        assert plan.keepouts == []
        assert len(plan.blocked) == 1

    def test_a_centred_person_fits_in_the_wide_corridor_but_not_the_old_one(self):
        assert _keepouts([(2.0, 0.0, 0.25)]).keepouts
        assert _keepouts([(2.0, 0.0, 0.25)], half_width=0.4333).blocked

    def test_behind_beyond_and_outside_are_ignored(self):
        plan = _keepouts([(-1.0, 0.0, 0.15), (3.5, 0.0, 0.15), (1.4, 1.2, 0.15)])
        assert plan.keepouts == [] and plan.blocked == []
        assert [r['skip'] for r in plan.report] == ['behind', 'beyond_reach',
                                                    'outside_corridor']


class TestFootprintCircles:

    def test_three_circles_cover_the_body_box(self):
        circles = footprint_circles(BODY, 3)
        assert [o for o, _ in circles] == pytest.approx([0.0055, 0.1805, 0.3555], abs=1e-4)
        assert circles[0][1] == pytest.approx(0.1617, abs=1e-4)
        # Every point of the rectangle (incl. the front bumper corners) is
        # inside some circle.
        for lon in np.linspace(BODY.rear, BODY.front, 41):
            for lat in np.linspace(-BODY.half_width, BODY.half_width, 11):
                assert any(math.hypot(lon - o, lat) <= rc + 1e-9 for o, rc in circles)

    def test_the_circle_rows_keep_the_whole_body_clear(self):
        circles = footprint_circles(BODY, 3)
        ko = _keepouts([(2.0, 0.0, 0.15)], circles=circles).keepouts[0]
        assert ko.circles == circles
        corridor = _corridor()
        corridor['obstacles_world'] = [(2.0, 0.0, 0.15)]
        _, info = _solve((0.0, 0.0, 0.0, 0.4), keepouts=[ko.as_dict()], v_floor=0.4,
                         from_stage=1, obstacles=[(2.0, 0.0, 0.15)])
        assert info['success']
        assert np.asarray(info['x_pred'])[:, 1].max() > 0.3

    def test_the_window_covers_the_body_overlap(self):
        ko = _keepouts([(1.4, 0.0, 0.15)]).keepouts[0]
        assert ko.s_lo == pytest.approx(-(0.15 + BODY.front + 0.12))
        assert ko.s_hi == pytest.approx(0.15 - BODY.rear + 0.12)


class TestPlanBodyGap:

    def test_a_plan_through_the_obstacle_is_negative(self):
        plan = [(x, 0.0, 0.0, 0.4) for x in np.linspace(0.0, 2.0, 11)]
        assert plan_body_gap(plan, [(1.4, 0.0, 0.15)], BODY) < 0.0

    def test_a_plan_beside_it_is_the_lateral_gap(self):
        plan = [(x, 0.6, 0.0, 0.4) for x in np.linspace(0.0, 2.0, 41)]
        gap = plan_body_gap(plan, [(1.4, 0.0, 0.15)], BODY)
        assert gap == pytest.approx(0.6 - BODY.half_width - 0.15, abs=1e-6)

    def test_nothing_to_measure_is_inf(self):
        assert plan_body_gap(None, [(1.0, 0.0, 0.1)], BODY) == math.inf
        assert plan_body_gap([(0, 0, 0, 0)], [], BODY) == math.inf


def _detector():
    return BlockedDetector(FLOOR, progress_window_s=2.5, min_progress_m=0.15,
                           lock_window_s=2.0, lock_fraction=0.95, infeasible_s=1.0)


def _tick(det, t, x, v_cmd=0.4, steer=0.0, turning=False, feasible=True):
    return det.update(t, x, 0.0, v_cmd, steer, -0.309, 0.319, turning, 0.0, feasible,
                      {'why': 'test'})


class TestBlockedDetector:

    def test_a_car_that_does_not_move_under_the_floor_is_blocked_at_the_window(self):
        det = _detector()
        verdicts = [_tick(det, 0.1 * k, 0.001 * k) for k in range(30)]
        first = next(i for i, v in enumerate(verdicts) if v is not None)
        assert first == 25
        assert verdicts[first].reason == 'no_progress'

    def test_a_moving_car_is_never_blocked(self):
        det = _detector()
        assert all(_tick(det, 0.1 * k, 0.04 * k) is None for k in range(200))

    def test_standing_still_while_commanding_zero_is_not_no_progress(self):
        det = _detector()
        assert all(_tick(det, 0.1 * k, 0.0, v_cmd=0.0) is None for k in range(100))

    def test_infeasible_for_a_second_is_blocked(self):
        det = _detector()
        verdicts = [_tick(det, 0.1 * k, 0.04 * k, v_cmd=0.0, feasible=False)
                    for k in range(15)]
        first = next(i for i, v in enumerate(verdicts) if v is not None)
        assert first == 10
        assert verdicts[first].reason == 'no_feasible_plan'
        assert verdicts[first].detail['why'] == 'test'

    def test_a_feasible_tick_restarts_the_infeasible_clock(self):
        """A flickering detection: 0.9 s infeasible, one good tick, repeat."""
        det = _detector()
        for k in range(60):
            feasible = (k % 10 == 9)
            v_cmd = 0.4 if feasible else 0.0
            assert _tick(det, 0.1 * k, 0.04 * k, v_cmd=v_cmd, feasible=feasible) is None

    def test_steering_lock_without_progress_along_the_heading(self):
        det = _detector()
        # Circling at lock: moving (path grows) but not along the heading.
        verdict = None
        for k in range(40):
            ang = 0.4 * 0.1 * k / 0.92
            x = 0.92 * math.sin(ang) * 0.1  # stays near x = 0
            verdict = verdict or det.update(
                0.1 * k, x, 0.92 * (1 - math.cos(ang)), 0.4, 0.319,
                -0.309, 0.319, False, 0.0, True)
        assert verdict is not None and verdict.reason == 'steering_lock_no_progress'

    def test_a_turn_is_exempt_from_the_lock_rule(self):
        det = _detector()
        for k in range(40):
            ang = 0.4 * 0.1 * k / 0.92
            v = det.update(0.1 * k, 0.92 * math.sin(ang), 0.92 * (1 - math.cos(ang)),
                           0.4, 0.319, -0.309, 0.319, True, 0.0, True)
            assert v is None

    def test_the_verdict_latches_until_reset(self):
        det = _detector()
        for k in range(26):
            _tick(det, 0.1 * k, 0.0)
        assert det.verdict is not None
        assert _tick(det, 10.0, 5.0) is det.verdict
        det.reset()
        assert _tick(det, 11.0, 5.0) is None

    def test_the_shipped_windows(self):
        assert get_value('mpc_blocked_progress_window_sec') == pytest.approx(2.5)
        assert get_value('mpc_blocked_min_progress_m') == pytest.approx(0.15)
        assert get_value('mpc_blocked_infeasible_sec') == pytest.approx(1.0)


def _solve(x0, keepouts=None, v_floor=None, from_stage=None, obstacles=()):
    corridor = _corridor()
    corridor['obstacles_world'] = list(obstacles)
    return solve_mpc_step(
        x0=np.asarray(x0, dtype=float), last_u=np.zeros(2),
        pref_nom=np.array([2.55, 0.0]), corridor=corridor, horizon=rig.HORIZON,
        ts=0.2, params=rig.PARAMS, limits=rig.LIMITS, weights=dict(rig.WEIGHTS),
        obstacles=list(obstacles), dmin=rig.DMIN, vdes=0.4, solver='rti',
        control_period=0.1, v_floor=v_floor, v_floor_from_stage=from_stage,
        keepouts=keepouts)


class TestTheSolverRows:

    def test_the_floor_row_holds_the_predicted_speed(self):
        # A target right next to the car makes slowing down attractive.
        _, info = _solve((0.0, 0.0, 0.0, 0.4), v_floor=0.4, from_stage=1)
        assert info['success']
        v = np.asarray(info['x_pred'])[1:, 3]
        assert v.min() >= 0.4 - 1e-3

    def test_a_keepout_moves_the_plan_to_its_side(self):
        ko = _keepouts([(2.0, 0.0, 0.15)]).keepouts[0]
        assert ko.side == 1
        _, info = _solve((0.0, 0.0, 0.0, 0.4), keepouts=[ko.as_dict()],
                         v_floor=0.4, from_stage=1, obstacles=[(2.0, 0.0, 0.15)])
        assert info['success']
        # One RTI step: the rows sit on the stages whose LINEARISATION point
        # overlaps the plant, and x_pred is the nonlinear rollout, so the
        # first solve lands near the row rather than on it; the closed loop
        # (test_blocked_closed_loop.py) converges. Without the row the plan
        # stays on the centreline.
        xp = np.asarray(info['x_pred'])
        assert xp[:, 1].max() >= ko.d - 0.1
        _, free = _solve((0.0, 0.0, 0.0, 0.4), v_floor=0.4, from_stage=1)
        assert np.abs(np.asarray(free['x_pred'])[:, 1]).max() < 0.05

    def test_a_keepout_no_plan_can_meet_is_not_a_success(self):
        # Already beside the plant, 0.1 m off its centre: the row wants 0.47.
        ko = _keepouts([(0.2, 0.0, 0.15)], car=(0.0, 0.1)).keepouts[0]
        _, info = _solve((0.0, 0.1, 0.0, 0.4), keepouts=[ko.as_dict()],
                         v_floor=0.4, from_stage=1)
        assert not info['success']
