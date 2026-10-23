"""
Coverage for the wall_turn increment and its build_straight_corridor wiring.

The rule lives in mpc_controller/wall_turn.py.

THE BUG. A wall_turn corridor asked for the WHOLE remaining turn on every
rebuild. At 0.5 m/s the MPC horizon covers 1.0 m and a 74.5 degree turn at
full lock needs 1.36 m of arc, so the terminal heading target was out of
reach and the QP sat on the steering bound.

THE FIX. Once the wall distance forces the turn to commit, each rebuild asks
for sign * min(|still owed|, d_avail / (k_safety * R_min), horizon / R_min),
and the corridor's end heading is ratcheted against dFront jitter.

WHICH QUANTITY IS MONOTONIC. dpsi_this is measured from the LIVE heading, so
it has to shrink as the car turns and end on exactly the remainder -- it
cannot also be non-decreasing. What grows across rebuilds and never retreats
is the END HEADING the corridor commands, commanded_rot. The monotonicity and
chatter tests pin commanded_rot; the exactness and feasibility tests pin
dpsi_this.

THE SIMULATED APPROACH. _approach() feeds a scripted dFront sequence and turns
the car by `carried` of each ask before the next rebuild. carried=1.0 flies
every increment in full, so the sum of dpsi_this is the car's total rotation.
carried=0.35 lags the corridor the way one 1 s rebuild period at 0.5 m/s does
(0.5 m of a ramp that runs 0.3 -> 2.1 m). Neither is a vehicle model.

Run standalone: python3 -m pytest test/test_wall_turn_increment.py -v
"""

import math
import unittest

from f1tenth_messages.msg import WallEstimate
from f1tenth_params.param_defaults import get_value
from mpc_controller import wall_boundaries as wb
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import terminal_heading_target
from mpc_controller.wall_turn import (
    min_turn_radius,
    plan_wall_turn_step,
    SMOOTHSTEP_PEAK_SLOPE,
    wall_turn_trigger_distance,
    wrap_to_pi,
)
from mpc_controller.wall_turn_distance import (
    SOURCE_LIDAR, SOURCE_NONE, SOURCE_SEMANTIC, WallTurnDistance)
import numpy as np

# READ, NOT COPIED: the deployed values from stack_params.yaml.
WHEELBASE = float(get_value('mpc_wheelbase_m'))
DELTA_MIN = float(get_value('mpc_steering_angle_min_rad'))
DELTA_MAX = float(get_value('mpc_steering_angle_max_rad'))
K_SAFETY = float(get_value('corr_wall_turn_k_safety'))
MARGIN = float(get_value('corr_wall_turn_safety_margin_m'))
# MPC_corr's own horizon literals (self.N, self.ts).
N_STEPS = 20
TS = 0.1

R_LEFT = min_turn_radius(WHEELBASE, DELTA_MAX)
R_RIGHT = min_turn_radius(WHEELBASE, DELTA_MIN)

# The LLM plan's own number (llm_plan_1789136064, move_1).
TURN_74 = math.radians(74.48451336700703)
TURN_90 = math.radians(90.0)

# One rebuild per 0.5 m (1 s at 0.5 m/s), and one per 0.1 m.
COARSE = [5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5]
FINE = [round(5.0 - 0.1 * i, 2) for i in range(51)]


def _step(turn, progress, d_front, v_ref=0.5, n_steps=N_STEPS, ts=TS,
          k_safety=K_SAFETY, committed=False, prev_commanded_rot=None,
          commit_on_unknown_first_rebuild=True, commit_lead=0.0):
    return plan_wall_turn_step(
        turn, progress, d_front,
        wheelbase=WHEELBASE, delta_min=DELTA_MIN, delta_max=DELTA_MAX,
        k_safety=k_safety, safety_margin=MARGIN,
        n_steps=n_steps, ts=ts, v_ref=v_ref,
        committed=committed, prev_commanded_rot=prev_commanded_rot,
        commit_on_unknown_first_rebuild=commit_on_unknown_first_rebuild,
        commit_lead=commit_lead)


def _approach(turn, d_fronts, carried=1.0, v_ref=0.5, commit_on_unknown_first_rebuild=True,
              commit_lead=0.0):
    """Rebuild once per dFront sample; return [(progress_before, step)]."""
    progress, committed, prev, out = 0.0, False, None, []
    for d_front in d_fronts:
        step = _step(turn, progress, d_front, v_ref=v_ref,
                     committed=committed, prev_commanded_rot=prev,
                     commit_on_unknown_first_rebuild=commit_on_unknown_first_rebuild,
                     commit_lead=commit_lead)
        out.append((progress, step))
        committed, prev = step.committed, step.commanded_rot
        progress += carried * step.dpsi_this
    return out


def _in_turn_sense(turn, value):
    return value if turn >= 0.0 else -value


def _feasible(step):
    """1.5 * |dpsi_this| / d_avail <= 1 / R_min, with d_avail == 0 meaning the
    distance cap is lifted (2026-10-23 fix) and the horizon cap is what
    bounds dpsi_this instead -- never 0, see TestInsideTheSafetyMargin."""
    if step.d_avail == 0.0:
        return abs(step.dpsi_this) <= step.dpsi_by_horizon + 1e-12
    return (SMOOTHSTEP_PEAK_SLOPE * abs(step.dpsi_this) / step.d_avail
            <= 1.0 / step.r_min + 1e-12)


class TestFarFromTheWall(unittest.TestCase):
    """Required test 1."""

    def test_beyond_the_trigger_distance_nothing_is_asked(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            bound = DELTA_MAX if turn > 0 else DELTA_MIN
            trigger = wall_turn_trigger_distance(turn, WHEELBASE, bound, K_SAFETY, MARGIN)
            for d_front in (trigger + 0.01, trigger + 1.0, 5.0, 10.0):
                if d_front <= trigger:
                    continue
                step = _step(turn, 0.0, d_front)
                self.assertEqual(step.dpsi_this, 0.0, (turn, d_front))
                self.assertFalse(step.committed)

    def test_the_first_sample_at_the_trigger_commits(self):
        trigger = wall_turn_trigger_distance(TURN_74, WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN)
        self.assertTrue(_step(TURN_74, 0.0, trigger).committed)
        self.assertGreater(_step(TURN_74, 0.0, trigger).dpsi_this, 0.0)


class TestCommitLead(unittest.TestCase):
    """corr_wall_turn_commit_lead_m moves the gate and nothing else."""

    LEAD = 0.4

    def test_the_lead_moves_the_trigger_out_by_exactly_the_lead(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            bound = DELTA_MAX if turn > 0 else DELTA_MIN
            base = wall_turn_trigger_distance(turn, WHEELBASE, bound, K_SAFETY, MARGIN)
            led = wall_turn_trigger_distance(turn, WHEELBASE, bound, K_SAFETY, MARGIN,
                                             commit_lead=self.LEAD)
            self.assertAlmostEqual(led - base, self.LEAD, delta=1e-12)
            self.assertFalse(_step(turn, 0.0, led + 0.01, commit_lead=self.LEAD).committed)
            self.assertTrue(_step(turn, 0.0, led, commit_lead=self.LEAD).committed)
            # Between the two triggers only the led gate commits.
            self.assertFalse(_step(turn, 0.0, base + 0.2).committed)
            self.assertTrue(_step(turn, 0.0, base + 0.2, commit_lead=self.LEAD).committed)

    def test_the_lead_leaves_the_increment_of_a_committed_turn_alone(self):
        for d_front in (3.0, 2.0, 1.5, 1.0, 0.7, 0.5):
            plain = _step(TURN_90, 0.4, d_front, committed=True, prev_commanded_rot=0.9)
            led = _step(TURN_90, 0.4, d_front, committed=True, prev_commanded_rot=0.9,
                        commit_lead=self.LEAD)
            self.assertEqual(led, plain)

    def test_a_negative_lead_is_no_lead(self):
        trigger = wall_turn_trigger_distance(TURN_90, WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN)
        self.assertEqual(
            wall_turn_trigger_distance(TURN_90, WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN,
                                       commit_lead=-1.0), trigger)
        self.assertTrue(_step(TURN_90, 0.0, trigger, commit_lead=-1.0).committed)

    def test_an_approach_with_the_lead_finishes_with_more_distance_in_hand(self):
        # 1 Hz at 0.45 m/s, from R012's 3.24 m start.
        seq = [round(3.24 - 0.45 * i, 2) for i in range(8)]
        plain = _approach(TURN_90, seq)
        led = [s for _p, s in _approach(TURN_90, seq, commit_lead=self.LEAD)]
        first = next(i for i, (_p, s) in enumerate(plain) if s.committed)
        first_led = next(i for i, s in enumerate(led) if s.committed)
        self.assertEqual(first_led, first - 1)
        self.assertAlmostEqual(sum(s.dpsi_this for s in led), TURN_90, delta=1e-12)


class TestTheApproach(unittest.TestCase):
    """Required tests 2, 3 and 4."""

    CASES = [(turn, seq, carried)
             for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90)
             for seq in (COARSE, FINE)
             for carried in (1.0, 0.35)]

    def test_the_commanded_end_heading_never_retreats_as_dfront_shrinks(self):
        for turn, seq, carried in self.CASES:
            steps = _approach(turn, seq, carried)
            cmds = [_in_turn_sense(turn, s.commanded_rot) for _p, s in steps]
            self.assertTrue(all(b >= a for a, b in zip(cmds, cmds[1:])),
                            (math.degrees(turn), len(seq), carried, cmds))
            self.assertGreater(cmds[-1], 0.0)

    def test_nothing_is_asked_before_the_turn_commits(self):
        for turn, seq, carried in self.CASES:
            for _p, step in _approach(turn, seq, carried):
                if not step.committed:
                    self.assertEqual(step.dpsi_this, 0.0)

    def test_fully_carried_increments_sum_to_the_whole_turn(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            for seq in (COARSE, FINE):
                steps = _approach(turn, seq, carried=1.0)
                total = sum(s.dpsi_this for _p, s in steps)
                self.assertAlmostEqual(total, turn, delta=1e-12)
                # More than one rebuild did the turning at 0.5 m/s: no single
                # corridor was allowed the whole angle.
                turning = [s for _p, s in steps if s.dpsi_this != 0.0]
                self.assertGreater(len(turning), 1)

    def test_the_last_turning_rebuild_asks_for_exactly_the_remainder(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            for seq in (COARSE, FINE):
                steps = _approach(turn, seq, carried=1.0)
                last_progress, last = [(p, s) for p, s in steps if s.dpsi_this != 0.0][-1]
                self.assertLessEqual(abs(last.dpsi_rem), last.dpsi_by_horizon)
                self.assertLessEqual(abs(last.dpsi_rem), last.dpsi_by_dist)
                # Bit-for-bit, not approximately.
                self.assertEqual(last.dpsi_this, last.dpsi_rem)
                self.assertAlmostEqual(last_progress + last.dpsi_this, turn, delta=1e-15)

    def test_a_lagging_car_is_still_commanded_the_whole_turn(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            steps = _approach(turn, FINE, carried=0.35)
            self.assertAlmostEqual(steps[-1][1].commanded_rot, turn, delta=1e-12)

    def test_every_rebuild_of_a_fully_carried_approach_is_trackable(self):
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            r_expected = R_LEFT if turn > 0 else R_RIGHT
            for seq in (COARSE, FINE):
                for _p, step in _approach(turn, seq, carried=1.0):
                    self.assertFalse(step.held)
                    if step.dpsi_this != 0.0:
                        self.assertAlmostEqual(step.r_min, r_expected, delta=1e-12)
                    self.assertTrue(_feasible(step), (math.degrees(turn), step))

    def test_a_lagging_car_breaks_feasibility_only_on_held_rebuilds(self):
        """The ratchet's price, pinned so it cannot grow unnoticed."""
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            for seq in (COARSE, FINE):
                for _p, step in _approach(turn, seq, carried=0.35):
                    if not step.held:
                        self.assertTrue(_feasible(step), (math.degrees(turn), step))


class TestTheHorizonCap(unittest.TestCase):
    """Required test 5."""

    def test_at_half_a_metre_per_second_a_90_degree_ask_stops_at_1_083(self):
        first = next(s for _p, s in _approach(TURN_90, FINE) if s.dpsi_this != 0.0)
        self.assertEqual(first.dpsi_this, first.dpsi_by_horizon)
        self.assertAlmostEqual(first.dpsi_this, N_STEPS * TS * 0.5 / R_LEFT, delta=1e-12)
        # 0.935 before the 2026-09-17 steering gains (R_min 1.069 m left).
        self.assertAlmostEqual(first.dpsi_this, 1.083, delta=1e-3)
        self.assertLess(first.dpsi_this, TURN_90)

    def test_at_one_and_a_half_metres_per_second_the_horizon_cap_does_not_bind(self):
        first = next(s for _p, s in _approach(TURN_90, FINE, v_ref=1.5) if s.dpsi_this != 0.0)
        self.assertAlmostEqual(first.dpsi_by_horizon, 3.0 / R_LEFT, delta=1e-12)
        self.assertLess(abs(first.dpsi_this), first.dpsi_by_horizon)
        self.assertGreater(first.dpsi_this, 1.083)

    def test_the_cap_is_recomputed_from_the_horizon_it_is_given(self):
        base = _step(TURN_90, 0.0, 1.5).dpsi_by_horizon
        self.assertAlmostEqual(_step(TURN_90, 0.0, 1.5, n_steps=10).dpsi_by_horizon,
                               base / 2.0, delta=1e-12)
        self.assertAlmostEqual(_step(TURN_90, 0.0, 1.5, ts=0.05).dpsi_by_horizon,
                               base / 2.0, delta=1e-12)
        self.assertAlmostEqual(_step(TURN_90, 0.0, 1.5, v_ref=0.3).dpsi_by_horizon,
                               base * 0.6, delta=1e-12)


class TestEachDirectionUsesItsOwnSteeringBound(unittest.TestCase):
    """Required test 6."""

    def test_the_radii_are_0_923_left_and_0_955_right(self):
        # L / tan(bound) at +0.319 / -0.309. The work order's 1.069 / 1.049
        # were the same formula at the placeholder gains' +0.278 / -0.283.
        self.assertAlmostEqual(R_LEFT, 0.923, delta=5e-4)
        self.assertAlmostEqual(R_RIGHT, 0.955, delta=5e-4)

    def test_a_left_turn_uses_delta_max_and_a_right_turn_delta_min(self):
        self.assertEqual(_step(TURN_74, 0.0, 3.0).r_min, R_LEFT)
        self.assertEqual(_step(-TURN_74, 0.0, 3.0).r_min, R_RIGHT)

    def test_coming_back_from_an_overshoot_uses_the_other_side(self):
        self.assertEqual(_step(0.5, 0.7, 3.0).r_min, R_RIGHT)
        self.assertEqual(_step(-0.5, -0.7, 3.0).r_min, R_LEFT)

    def test_the_trigger_distances_are_the_work_orders_table(self):
        self.assertAlmostEqual(
            wall_turn_trigger_distance(math.radians(74.5), WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN),
            3.71, delta=5e-3)
        self.assertAlmostEqual(
            wall_turn_trigger_distance(TURN_90, WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN),
            4.27, delta=5e-3)

    def test_the_asymmetry_changes_when_the_turn_commits(self):
        # 3.68 m sits between the right trigger (3.66) and the left one (3.71).
        self.assertTrue(_step(TURN_74, 0.0, 3.68).committed)
        self.assertFalse(_step(-TURN_74, 0.0, 3.68).committed)


class TestInsideTheSafetyMargin(unittest.TestCase):
    """Required test 7.

    FIXED 2026-10-23: d_avail still clamps at 0 inside the margin, but once
    committed that no longer freezes dpsi_by_dist (and so dpsi_this) at 0 --
    see wall_turn.py's 'A WALL INSIDE THE MARGIN, COMMITTED' section. The old
    freeze pinned cmd_rot forever while the car sat at the margin, confirmed
    live on R017/M05_turn_left_wall (held at 46.6 deg of a commanded 90,
    dFront flat at 0.14 m for 20+ s, then contact)."""

    def test_d_avail_clamps_at_zero_but_the_cap_lifts_once_committed(self):
        for turn in (TURN_74, -TURN_74):
            sense = 1.0 if turn > 0 else -1.0
            for d_front in (MARGIN, MARGIN - 1e-9, 0.5, 0.0):
                step = _step(turn, 0.0, d_front)
                self.assertEqual(step.d_avail, 0.0)
                self.assertTrue(step.committed)
                self.assertEqual(step.dpsi_by_dist, math.inf)
                # Horizon-capped, same as an unknown dFront post-commit --
                # not frozen, and not inverted.
                self.assertEqual(step.dpsi_this, sense * step.dpsi_by_horizon)

    def test_a_wall_already_inside_the_margin_does_not_freeze_a_committed_turn(self):
        for turn in (TURN_74, -TURN_74):
            sense = 1.0 if turn > 0 else -1.0
            step = _step(turn, sense * 0.4, 0.3, committed=True,
                         prev_commanded_rot=sense * 0.6)
            # The distance cap is lifted, so the ratchet has nothing to hold
            # against: the ask is the horizon cap (or the remainder, if
            # smaller), not the stale end heading from prev_commanded_rot.
            self.assertFalse(step.held)
            expected = sense * min(abs(turn) - 0.4, step.dpsi_by_horizon)
            self.assertAlmostEqual(step.dpsi_this, expected, delta=1e-12)
            self.assertGreater(abs(step.dpsi_this), 0.2)

    def test_pinned_at_the_margin_with_no_real_progress_the_ask_does_not_decay(self):
        """The regression test for the stall. R017 (2026-10-23) held dFront
        at ~0.14 m -- inside the 0.5 m margin -- for 20+ s while the car's
        real yaw barely moved, and dpsi_this decayed from 0.1672 toward
        0.147: the ratchet re-floored the ask at the SAME frozen cmd_rot
        every rebuild because dpsi_by_dist was 0 (the old bug). carried=0.0
        below is the worst case this scripts directly -- a car making zero
        real progress on the ask -- and the ask must stay at the horizon
        cap, not decay toward 0, every one of the 20 rebuilds."""
        seq = [0.46] + [0.14] * 20
        for turn in (TURN_90, -TURN_90):
            steps = _approach(turn, seq, carried=0.0)
            for _p, s in steps:
                self.assertTrue(s.committed)
                self.assertFalse(s.held)
                self.assertAlmostEqual(abs(s.dpsi_this), s.dpsi_by_horizon, delta=1e-12)


class TestChatter(unittest.TestCase):
    """Required test 8: a jittering dFront, including dropouts and spikes."""

    @staticmethod
    def _jittered(seed):
        rng = np.random.default_rng(seed)
        out = []
        for i, d_front in enumerate(FINE):
            if i % 9 == 4:
                out.append(None)                      # unknown / stale
            elif i % 13 == 6:
                out.append(5.0)                       # spurious far reading
            elif i % 11 == 8:
                out.append(-1.0)                      # front_clearance_node's sentinel
            else:
                out.append(max(d_front + float(np.clip(rng.normal(0.0, 0.15), -0.3, 0.3)),
                               0.0))
        return out

    CASES = [(turn, seed, carried)
             for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90)
             for seed in (1, 7, 42)
             for carried in (1.0, 0.35)]

    def test_the_end_heading_never_retreats_and_no_ask_exceeds_the_horizon(self):
        for turn, seed, carried in self.CASES:
            steps = _approach(turn, self._jittered(seed), carried)
            cmds = [_in_turn_sense(turn, s.commanded_rot) for _p, s in steps]
            self.assertTrue(all(b >= a for a, b in zip(cmds, cmds[1:])),
                            (math.degrees(turn), seed, carried))
            for (p0, s0), (p1, s1) in zip(steps, steps[1:]):
                self.assertLessEqual(abs(s1.dpsi_this), s1.dpsi_by_horizon + 1e-12)
                self.assertLessEqual(abs(s1.commanded_rot - s0.commanded_rot),
                                     s1.dpsi_by_horizon + abs(p1 - p0) + 1e-12)
            self.assertAlmostEqual(steps[-1][1].commanded_rot, turn, delta=1e-12)

    def test_without_the_ratchet_the_same_jitter_would_retreat(self):
        """The raw distance-capped end heading, for the same rebuilds, does
        go backwards. dFront stays well above the margin throughout (0.7 to
        2.8 m) so this is jitter alone, not the margin relief above: the
        2026-10-23 fix lifts the distance cap only once d_avail hits 0, and
        closes off the one dip _jittered() happened to exercise (its tail
        runs down to 0.0 m), so this uses its own mid-range sequence."""
        seq = [2.6, 1.1, 0.7, 1.3, 0.9]
        retreated = 0
        for turn in (TURN_74, -TURN_74, TURN_90, -TURN_90):
            raw = []
            for progress, step in _approach(turn, seq, carried=0.35):
                if not step.committed:
                    raw.append(0.0)
                    continue
                self.assertGreater(step.d_avail, 0.0)
                ask = min(abs(step.dpsi_rem), step.dpsi_by_dist, step.dpsi_by_horizon)
                raw.append(_in_turn_sense(turn, progress) + ask * (
                    1.0 if _in_turn_sense(turn, step.dpsi_rem) >= 0.0 else -1.0))
            retreated += sum(1 for a, b in zip(raw, raw[1:]) if b < a - 1e-9)
        self.assertGreater(retreated, 0)


class TestUnknownFrontDistance(unittest.TestCase):

    def test_legacy_on_the_first_rebuild_it_commits_and_turns_within_the_horizon(self):
        for unknown in (None, float('nan'), -1.0):
            step = _step(TURN_90, 0.0, unknown)
            self.assertTrue(step.committed)
            self.assertIsNone(step.d_avail)
            self.assertEqual(step.dpsi_this, step.dpsi_by_horizon)

    def test_semantic_on_the_first_rebuild_it_does_not_commit(self):
        # corr_wall_turn_distance_source 'semantic' passes False.
        for unknown in (None, float('nan'), -1.0):
            step = _step(TURN_90, 0.0, unknown, commit_on_unknown_first_rebuild=False)
            self.assertFalse(step.committed)
            self.assertEqual(step.dpsi_this, 0.0)

    def test_semantic_stays_straight_through_unknowns_then_commits_at_the_gate(self):
        trigger = wall_turn_trigger_distance(TURN_90, WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN)
        samples = [None, None, None, trigger + 1.0, None, trigger + 0.5, trigger]
        steps = [s for _, s in _approach(TURN_90, samples,
                                         commit_on_unknown_first_rebuild=False)]
        self.assertEqual([s.committed for s in steps], [False] * 6 + [True])
        self.assertEqual([s.dpsi_this for s in steps[:6]], [0.0] * 6)
        self.assertGreater(steps[-1].dpsi_this, 0.0)

    def test_the_flag_changes_nothing_once_committed(self):
        for flag in (True, False):
            step = _step(TURN_90, 0.3, None, committed=True, prev_commanded_rot=0.9,
                         commit_on_unknown_first_rebuild=flag)
            self.assertEqual(step.dpsi_this, min(TURN_90 - 0.3, step.dpsi_by_horizon))

    def test_later_and_before_commit_it_changes_nothing(self):
        for unknown in (None, float('nan'), -1.0):
            step = _step(TURN_90, 0.02, unknown, committed=False, prev_commanded_rot=0.02)
            self.assertFalse(step.committed)
            self.assertEqual(step.dpsi_this, 0.0)

    def test_after_commit_it_drops_only_the_distance_cap(self):
        step = _step(TURN_90, 0.3, None, committed=True, prev_commanded_rot=0.9)
        self.assertEqual(step.dpsi_this, min(TURN_90 - 0.3, step.dpsi_by_horizon))


class TestTheRemainderIsUnwrapped(unittest.TestCase):

    def test_a_180_degree_turn_keeps_its_commanded_direction(self):
        for sign in (1.0, -1.0):
            step = _step(sign * math.pi, 0.0, 2.0)
            self.assertEqual(step.dpsi_rem, sign * math.pi)
            self.assertEqual(math.copysign(1.0, step.dpsi_this), sign)

    def test_across_the_pi_seam_it_equals_the_short_way_wrap(self):
        psi_base, turn = 3.0, 0.6
        target = psi_base + turn
        progress, last_yaw = 0.0, psi_base
        for yaw in (3.05, 3.12, -3.13, -3.05, -2.95):
            progress += wrap_to_pi(yaw - last_yaw)
            last_yaw = yaw
            step = _step(turn, progress, 2.0)
            self.assertAlmostEqual(step.dpsi_rem, wrap_to_pi(target - yaw), delta=1e-12)
            self.assertGreater(step.dpsi_rem, 0.0)


class TestKSafetyFloor(unittest.TestCase):

    def test_below_the_smoothstep_peak_slope_it_is_refused(self):
        with self.assertRaises(ValueError):
            _step(TURN_74, 0.0, 3.0, k_safety=1.49)
        _step(TURN_74, 0.0, 3.0, k_safety=1.5)


# ---------------------------------------------------------------------------
# build_straight_corridor wiring
# ---------------------------------------------------------------------------

class _FakeLogger:
    def info(self, *args, **kwargs):
        pass


class _FakeMPC:
    """Just enough of MPCController for build_straight_corridor's drive branch."""

    def __init__(self, mode='wall_turn', turn_deg=74.5, sign=1.0, psi_base=0.2,
                 d_front=5.0, vdes=0.5, source='legacy'):
        self.corr_wall_turn_distance_source = source
        self.semantic_calls = []
        self.front_distance = d_front
        self._d_front = d_front
        self.goal_pose_xy = None
        self.goal_anchor_odom = None
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 0.4333
        self.corr_wmax = 0.7667
        self.corr_turn_u_start = float(get_value('corr_turn_u_start'))
        self.corr_turn_u_end = float(get_value('corr_turn_u_end'))
        self.corridor_heading_return = False
        self.psi_init_corridor = psi_base
        self.drive_cmd = {'mode': mode, 'turn_sign': sign, 'turn_mag_deg': turn_deg}
        self.turn_progress_rad = 0.0
        self.params = {'L': WHEELBASE, 'lr': 0.17}
        self.limits = {'delta_min': DELTA_MIN, 'delta_max': DELTA_MAX}
        self.corr_wall_turn_k_safety = K_SAFETY
        self.corr_wall_turn_safety_margin_m = MARGIN
        self.N = N_STEPS
        self.ts = TS
        self.vdes = vdes
        self.wall_turn_committed = False
        self.wall_turn_commanded_rot = None

    def _fresh_front_distance(self):
        if self.corr_wall_turn_distance_source == 'semantic':
            raise AssertionError('semantic mode must not read the wall topic')
        return self._d_front

    def _wall_turn_semantic_distance(self, x, y, heading):
        self.semantic_calls.append((x, y, heading))
        if self._d_front is None:
            return WallTurnDistance(None, SOURCE_NONE, None)
        return WallTurnDistance(self._d_front, SOURCE_SEMANTIC, 7)

    def get_logger(self):
        return _FakeLogger()


def _headings(corridor):
    return np.arctan2(np.diff(corridor['yc']), np.diff(corridor['xc']))


def _build(fake, psi0):
    return MPCController.build_straight_corridor(fake, [1.0, -0.5, psi0, 0.5])


class TestCorridorWiring(unittest.TestCase):

    def test_far_from_the_wall_the_corridor_is_straight_along_the_live_heading(self):
        fake = _FakeMPC(d_front=6.0)
        corridor = _build(fake, 0.23)
        self.assertEqual(corridor['psiRef'], 0.23)
        self.assertEqual(corridor['dpsi'], 0.0)
        self.assertEqual(corridor['psiRefTurn'], 0.0)
        np.testing.assert_allclose(_headings(corridor), 0.23, atol=1e-9)

    def test_psi_end_psi_ref_turn_and_the_centreline_share_one_increment(self):
        for sign in (1.0, -1.0):
            fake = _FakeMPC(sign=sign, d_front=2.5)
            psi0 = 0.2 + sign * 0.1
            fake.turn_progress_rad = sign * 0.1
            corridor = _build(fake, psi0)
            dpsi_this = corridor['psiRefTurn']
            self.assertNotEqual(dpsi_this, 0.0)
            self.assertLess(abs(dpsi_this), math.radians(74.5) - 0.1)
            self.assertEqual(corridor['dpsi'], dpsi_this)
            self.assertAlmostEqual(corridor['psiRef'] - corridor['psiStart'], dpsi_this,
                                   delta=1e-12)
            # The lead-out past corr_turn_u_end runs along psiEnd.
            lead_out = _headings(corridor)[int(0.8 * (fake.corr_N - 1)):]
            np.testing.assert_allclose(lead_out, corridor['psiRef'], atol=1e-9)
            # The solver's terminal target is the same heading.
            self.assertAlmostEqual(
                terminal_heading_target(corridor, corridor['psiRef'], psi0),
                psi0 + dpsi_this, delta=1e-12)
            self.assertTrue(fake.wall_turn_committed)
            self.assertAlmostEqual(fake.wall_turn_commanded_rot,
                                   fake.turn_progress_rad + dpsi_this, delta=1e-15)

    def test_the_last_rebuild_points_the_corridor_at_the_final_heading(self):
        fake = _FakeMPC(d_front=2.5, psi_base=0.2)
        total = math.radians(74.5)
        # Mid-move: the commit latched on an earlier rebuild.
        fake.wall_turn_committed = True
        fake.turn_progress_rad = total - 0.2
        psi0 = 0.2 + fake.turn_progress_rad
        corridor = _build(fake, psi0)
        self.assertAlmostEqual(corridor['psiRef'], 0.2 + total, delta=1e-12)
        self.assertAlmostEqual(corridor['psiRefTurn'], 0.2, delta=1e-12)

    def test_an_unknown_wall_distance_builds_a_corridor_without_raising(self):
        # /perception/virtual_wall's -1.0 (or a stale reading) reaches the
        # wall_turn branch as None; the corridor debug used to float(None).
        for committed in (False, True):
            fake = _FakeMPC(d_front=-1.0)
            fake._d_front = None
            fake.wall_turn_committed = committed
            fake.wall_turn_commanded_rot = 0.3 if committed else None
            corridor = _build(fake, 0.2)
            self.assertEqual(corridor['dFront'], -1.0)
            self.assertTrue(np.isfinite(corridor['psiRef']))

    def test_semantic_an_unknown_first_rebuild_builds_a_straight_corridor(self):
        fake = _FakeMPC(d_front=-1.0, source='semantic')
        fake._d_front = None
        corridor = _build(fake, 0.2)
        self.assertFalse(fake.wall_turn_committed)
        self.assertEqual(corridor['psiRefTurn'], 0.0)
        self.assertEqual(corridor['dFront'], -1.0)
        # The legacy fake would have committed on the same rebuild.
        legacy = _FakeMPC(d_front=-1.0)
        legacy._d_front = None
        _build(legacy, 0.2)
        self.assertTrue(legacy.wall_turn_committed)

    def test_semantic_commits_once_its_distance_reaches_the_trigger(self):
        fake = _FakeMPC(d_front=-1.0, source='semantic')
        fake._d_front = None
        _build(fake, 0.2)
        trigger = wall_turn_trigger_distance(
            math.radians(74.5), WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN)
        fake._d_front = trigger + 0.1
        _build(fake, 0.2)
        self.assertFalse(fake.wall_turn_committed)
        fake._d_front = trigger - 0.1
        corridor = _build(fake, 0.2)
        self.assertTrue(fake.wall_turn_committed)
        self.assertNotEqual(corridor['psiRefTurn'], 0.0)
        self.assertEqual(corridor['dFront'], trigger - 0.1)

    def test_the_node_passes_its_commit_lead_to_the_gate(self):
        trigger = wall_turn_trigger_distance(
            math.radians(74.5), WHEELBASE, DELTA_MAX, K_SAFETY, MARGIN)
        plain = _FakeMPC(d_front=trigger + 0.2)
        _build(plain, 0.2)
        self.assertFalse(plain.wall_turn_committed)
        led = _FakeMPC(d_front=trigger + 0.2)
        led.corr_wall_turn_commit_lead_m = 0.4
        _build(led, 0.2)
        self.assertTrue(led.wall_turn_committed)

    def test_semantic_casts_along_the_start_heading_plus_the_turn_so_far(self):
        fake = _FakeMPC(d_front=6.0, source='semantic', psi_base=0.2)
        fake.turn_progress_rad = 0.5
        _build(fake, 0.7)
        self.assertEqual(fake.semantic_calls, [(1.0, -0.5, 0.2 + 0.5)])

    def test_the_straight_drive_branch_is_unchanged(self):
        fake = _FakeMPC(mode='straight', d_front=2.0, psi_base=0.2)
        corridor = _build(fake, 0.25)
        self.assertIsNone(corridor['psiRefTurn'])
        self.assertEqual(corridor['psiRef'], 0.2)
        self.assertFalse(fake.wall_turn_committed)
        self.assertIsNone(fake.wall_turn_commanded_rot)


class _ClockFake:
    def __init__(self, now_sec):
        self._ns = int(now_sec * 1e9)

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self._ns


class TestFreshFrontDistanceAndReset(unittest.TestCase):

    class _Fake:
        def __init__(self, value, stamp, now_sec=100.0, clear_m=None):
            self.front_distance = value
            self.front_distance_stamp_sec = stamp
            self.corr_wall_turn_front_distance_max_age_sec = 0.5
            self.wall_clear_distance_m = (
                float(get_value('wall_clear_distance_m')) if clear_m is None else clear_m)
            self._clock = _ClockFake(now_sec)

        def get_clock(self):
            return self._clock

    def _fresh(self, value, stamp):
        return MPCController._fresh_front_distance(self._Fake(value, stamp))

    def test_only_a_recent_positive_finite_reading_is_a_distance(self):
        self.assertEqual(self._fresh(2.5, 99.8), 2.5)
        self.assertIsNone(self._fresh(10.0, None))      # the bootstrap, never received
        self.assertIsNone(self._fresh(2.5, 99.0))       # stale
        self.assertIsNone(self._fresh(-1.0, 99.9))      # "no reading" on both wall topics
        self.assertIsNone(self._fresh(0.0, 99.9))       # the contract is <= 0
        self.assertIsNone(self._fresh(float('nan'), 99.9))

    def test_the_clear_value_never_reaches_the_geometry(self):
        """wall_clear_distance_m means "nothing ahead", not "a wall at 50 m".
        It must arrive as UNKNOWN, so no distance cap and no corridor length
        is ever built from it."""
        clear = float(get_value('wall_clear_distance_m'))
        self.assertIsNone(self._fresh(clear, 99.9))
        self.assertIsNone(self._fresh(clear + 10.0, 99.9))
        # Just under it is still a real wall and still gets through.
        self.assertEqual(self._fresh(clear - 0.1, 99.9), clear - 0.1)

    def test_a_new_turn_starts_uncommitted_with_no_end_heading_to_hold(self):
        fake = _FakeMPC()
        fake.yaw = 0.4
        fake.wall_turn_committed = True
        fake.wall_turn_commanded_rot = 0.9
        MPCController._reset_turn_progress(fake)
        self.assertFalse(fake.wall_turn_committed)
        self.assertIsNone(fake.wall_turn_commanded_rot)


class TestSemanticDistanceGlue(unittest.TestCase):
    """MPCController._wall_turn_semantic_distance: cached messages in, the chain out."""

    class _Fake:
        _odom_frame_name = MPCController._odom_frame_name

        def __init__(self, tracks, walls_time=99.9, provenance=WallEstimate.PROVENANCE_MEASURED,
                     lidar_distance=2.0):
            self.wall_tracks = tracks
            self.wall_tracks_frame = 'odom'
            self.wall_tracks_last_time = walls_time
            self.wall_tracks_stamp = 99.9
            self.wall_coast_clock = wb.CoastClock()
            self.wall_coast_clock.update(tracks, 99.9)
            est = WallEstimate()
            est.valid = provenance != WallEstimate.PROVENANCE_NONE
            est.provenance = provenance
            est.distance = lidar_distance
            est.normal_angle = 0.0
            self.lidar_wall_estimate = est
            self.lidar_wall_estimate_stamp_sec = 99.9
            self.wall_turn_bumper_x_m = 0.4
            self.wall_row_max_tf_age_s = 0.5
            self.wall_row_max_coast_s = 1.0
            self.corr_wall_turn_front_distance_max_age_sec = 0.5
            self._clock = _ClockFake(100.0)

        def get_clock(self):
            return self._clock

    @staticmethod
    def _wall(source=wb.SOURCE_LIDAR):
        return wb.WallTrackView(5, 'front', wb.STATE_CONFIRMED, source, 3.0, 0.0,
                                (3.0, -1.0), (3.0, 1.0))

    def _pick(self, fake):
        return MPCController._wall_turn_semantic_distance(fake, 0.0, 0.0, 0.0)

    def test_a_fresh_lidar_backed_wall_ahead_is_the_distance(self):
        pick = self._pick(self._Fake([self._wall()]))
        self.assertEqual((pick.source, pick.track_id), (SOURCE_SEMANTIC, 5))
        self.assertAlmostEqual(pick.distance, 2.6)

    def test_a_stereo_only_wall_falls_through_to_the_lidar_fit(self):
        pick = self._pick(self._Fake([self._wall(wb.SOURCE_STEREO)]))
        self.assertEqual(pick.source, SOURCE_LIDAR)
        self.assertAlmostEqual(pick.distance, 1.6)

    def test_a_wall_message_past_the_wall_row_age_falls_through_to_the_lidar_fit(self):
        self.assertEqual(self._pick(self._Fake([self._wall()], walls_time=99.4)).source,
                         SOURCE_LIDAR)
        self.assertEqual(self._pick(self._Fake([self._wall()], walls_time=None)).source,
                         SOURCE_LIDAR)

    def test_a_seeded_or_invalid_lidar_estimate_is_not_a_distance(self):
        for provenance in (WallEstimate.PROVENANCE_SEEDED, WallEstimate.PROVENANCE_NONE):
            self.assertEqual(self._pick(self._Fake([], provenance=provenance)).source,
                             SOURCE_NONE)
        self.assertEqual(
            self._pick(self._Fake([], provenance=WallEstimate.PROVENANCE_DEAD_RECKONED)).source,
            SOURCE_LIDAR)
