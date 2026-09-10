"""
Coverage for the SIGNED turn branch (corridor["psiRefTurn"]).

THE BUG. corridor["psiRef"] is a heading, and a heading cannot say which way
round to get there: "turn 180 right" and "turn 180 left" name the same
psiRef. mpc_solver's terminal yaw cost therefore had to guess, and it
guessed shortest-way -- which at exactly 180 degrees is decided by the sign
of sin(pi) in floating point. It does not stay a coin flip either: let the
RTI reference rollout drift a hair the wrong way, the error crosses pi, the
unwrap flips to the other side, the solver steers further that way, and the
next tick starts deeper in. Measured on the real geometry before the fix, a
commanded +181 degrees came out as a target of -179 and the first control
inverted.

missions/drive_turn_180.json commands turn_sign -1.0, turn_mag_deg 180.0 --
exactly on the boundary -- so this is a live mission's behaviour, not a
hypothetical.

THE FIX, and the layer. build_straight_corridor knows turn_sign; by the time
the solver has a bare float psiRef the information is gone. So the corridor
carries the signed rotation STILL OWED as psiRefTurn, and the terminal cost
applies it without wrapping. Absent/None -- every corridor that is not an
active wall_turn -- restores the original shortest-branch unwrap exactly.

Run standalone: python3 -m pytest test/test_turn_branch.py -v
"""

import math
import unittest

import numpy as np

import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import (
    OSQP_AVAILABLE,
    solve_mpc_step,
    terminal_heading_target,
    unwrapped_heading_target,
)

from test_warm_start import (
    HORIZON,
    LIMITS,
    PARAMS,
    TS,
    WEIGHTS,
    _corridor,
)


def _turn_corridor_geometry(psi_start, signed_turn):
    """A wall_turn corridor: geometry AND psiRefTurn on the signed branch.

    _corridor() takes the shortest branch for its own S-curve, exactly as
    build_straight_corridor used to; passing the signed rotation through
    dpsi is what MPC_corr now does for a wall_turn, and the two have to
    match or the position terms fight the heading term.
    """
    c = _corridor(psi_start, psi_start + signed_turn, dpsi=signed_turn)
    c["psiStart"] = float(psi_start)
    c["psiRefTurn"] = float(signed_turn)
    return c


def _lookahead_target(corridor, length=3.0, frac=0.5):
    """The point compute_local_target would pick, near enough.

    NOT corridor["Pend"]. On a corridor that has curled 180 degrees the far
    end sits BEHIND the car, and w_term (9.0, the largest weight in the set)
    aimed at a point behind the car will happily pick either direction to
    get there -- which swamps the heading term and tests nothing about the
    branch. MPC_corr aims at corr_lookahead_frac * L (0.5 * 3.0 = 1.5 m)
    along the centreline instead, for exactly that reason: see
    _corridor_lookahead's own docstring on keeping the terminal cost a
    "steer toward" and not an "arrive at".
    """
    idx = min(int(round(frac * (len(corridor["xc"]) - 1))),
              len(corridor["xc"]) - 1)
    return np.array([corridor["xc"][idx], corridor["yc"][idx]], dtype=float)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class TestTerminalHeadingTarget(unittest.TestCase):
    """The branch choice itself -- pure, no solver, no node."""

    @staticmethod
    def _turn_corridor(psi_start, signed_turn):
        """Only the keys terminal_heading_target reads."""
        return {"psiStart": float(psi_start),
                "psiRefTurn": float(signed_turn),
                "psiRef": float(psi_start + signed_turn)}

    def test_without_the_key_it_is_the_original_shortest_branch_unwrap(self):
        """
        The no-op guarantee for every corridor that is not a wall_turn.

        goal_distance, goal_pose, the bootstrap fallback and every duck-typed
        test stand-in reach the terminal cost with no psiRefTurn at all, and
        must be bit-for-bit unaffected.
        """
        for psi_ref in (0.0, 0.7, -2.5, math.pi - 0.01, -math.pi + 0.01):
            for psi_n in (0.0, 1.2, -1.2, 3.0, -3.0):
                expected = unwrapped_heading_target(psi_ref, psi_n)
                for corridor in ({"psiRef": psi_ref},
                                 {"psiRef": psi_ref, "psiRefTurn": None}):
                    self.assertAlmostEqual(
                        terminal_heading_target(corridor, psi_ref, psi_n),
                        expected, places=12)

    def test_turns_under_180_are_unchanged(self):
        """
        Where a regression would hide.

        Wherever the rotation still owed fits inside (-pi, pi] the shortest
        branch was already the right branch, so the signed form must agree
        with it to floating point -- otherwise this change silently re-tunes
        every 90 degree turn the car makes.

        THE BOUND IS ON THE ASK, NOT ON THE COMMANDED MAGNITUDE, and the
        difference is the bug in miniature: a commanded 179 degrees with the
        reference rollout drifting 9 degrees the wrong way is an ask of 188,
        which the shortest branch renders as -172 -- a full reversal on a
        turn nobody would call a boundary case. So the equivalence is
        asserted exactly where it holds and no further; the cases beyond it
        are the point of the next test, not a regression.
        """
        for deg in (10, 45, 90, 135, 170, 179):
            for sign in (+1.0, -1.0):
                turn = math.radians(sign * deg)
                for psi_start in (0.0, 1.1, -2.2, 3.0):
                    for drift in (0.0, 0.15, -0.15):
                        psi_n = psi_start + drift
                        corridor = self._turn_corridor(psi_start, turn)
                        if abs(turn - drift) >= math.pi:
                            continue
                        signed = terminal_heading_target(
                            corridor, corridor["psiRef"], psi_n)
                        shortest = unwrapped_heading_target(
                            corridor["psiRef"], psi_n)
                        self.assertAlmostEqual(signed, shortest, places=9)

    def test_at_and_beyond_180_the_commanded_direction_is_what_survives(self):
        """
        170 / 180 / 181 / 270, both ways round.

        The asserted property is the one the mission actually cares about:
        the rotation the cost asks for has the sign the mission commanded,
        and the magnitude it commanded.
        """
        for deg in (170, 180, 181, 270):
            for sign in (+1.0, -1.0):
                turn = math.radians(sign * deg)
                corridor = self._turn_corridor(0.0, turn)
                target = terminal_heading_target(corridor, corridor["psiRef"], 0.0)
                asked = target - 0.0
                self.assertAlmostEqual(asked, turn, places=9,
                                       msg=f"{sign:+.0f} * {deg} deg")
                self.assertEqual(asked > 0, sign > 0)

    def test_the_shortest_branch_really_would_have_flipped(self):
        """
        Pin the defect itself, so this file fails if the fix is reverted.

        Not a test of the new code -- a test that the old code was wrong,
        which is what stops someone deleting the signed branch as redundant.
        """
        for deg in (181, 200, 270):
            turn = math.radians(deg)
            shortest = unwrapped_heading_target(turn, 0.0)
            self.assertLess(shortest, 0.0)          # asked LEFT, got RIGHT
            corridor = self._turn_corridor(0.0, turn)
            self.assertGreater(
                terminal_heading_target(corridor, corridor["psiRef"], 0.0), 0.0)

    def test_a_rollout_drifting_the_wrong_way_does_not_flip_the_branch(self):
        """
        The positive-feedback case.

        The RTI reference trajectory is a rollout, so psi_ref_N can sit on
        the wrong side of psi_start -- that is exactly how the old ratchet
        started. Drifting the wrong way must make the ask BIGGER, never make
        it change sign.
        """
        for deg in (170, 180, 181, 270):
            for sign in (+1.0, -1.0):
                turn = math.radians(sign * deg)
                corridor = self._turn_corridor(0.0, turn)
                asks = []
                for drift_deg in (0, 2, 5, 10, 20, 40):
                    psi_n = math.radians(-sign * drift_deg)   # the WRONG way
                    target = terminal_heading_target(
                        corridor, corridor["psiRef"], psi_n)
                    ask = target - psi_n
                    self.assertEqual(ask > 0, sign > 0,
                                     msg=f"{sign:+.0f}*{deg} drift {drift_deg}")
                    asks.append(abs(ask))
                # strictly growing: the further the wrong way, the more owed
                self.assertTrue(all(b > a for a, b in zip(asks, asks[1:])))

    def test_progress_the_right_way_shrinks_the_ask_monotonically(self):
        """The mirror of the case above: real progress must count."""
        for sign in (+1.0, -1.0):
            turn = math.radians(sign * 180.0)
            corridor = self._turn_corridor(0.0, turn)
            asks = [abs(terminal_heading_target(corridor, corridor["psiRef"],
                                                math.radians(sign * d))
                        - math.radians(sign * d))
                    for d in (0, 10, 30, 60, 90)]
            self.assertTrue(all(b < a for a, b in zip(asks, asks[1:])))


class TestTurnProgressAccumulator(unittest.TestCase):
    """
    MPC_corr.turn_progress_rad -- the unwrapped total the corridor subtracts.

    Duck-typed stand-ins calling the unbound methods, the same shape
    test_corridor_direction_recovery.py uses.
    """

    class _Fake:
        def __init__(self, mode='wall_turn', yaw=0.0):
            self.yaw = yaw
            self.drive_cmd = None if mode is None else {'mode': mode}
            self.turn_progress_rad = 0.0
            self._turn_progress_last_yaw = None

    def _drive(self, fake, yaws):
        for y in yaws:
            fake.yaw = y
            MPCController._accumulate_turn_progress(fake)
        return fake.turn_progress_rad

    def test_the_first_tick_only_anchors_and_adds_nothing(self):
        fake = self._Fake(yaw=0.4)
        MPCController._accumulate_turn_progress(fake)
        self.assertEqual(fake.turn_progress_rad, 0.0)
        self.assertAlmostEqual(fake._turn_progress_last_yaw, 0.4)

    def test_it_accumulates_past_180_degrees_where_a_wrapped_difference_cannot(self):
        """
        The whole reason the counter exists.

        Walk the car right round through 270 degrees in small steps. The
        wrapped difference between start and end reads -90; the accumulated
        total reads +270, which is the number the corridor needs.
        """
        yaws = [_wrap(math.radians(d)) for d in range(0, 271, 5)]
        fake = self._Fake(yaw=yaws[0])
        total = self._drive(fake, yaws)
        self.assertAlmostEqual(math.degrees(total), 270.0, places=6)
        self.assertAlmostEqual(
            math.degrees(_wrap(yaws[-1] - yaws[0])), -90.0, places=6)

    def test_it_accumulates_negative_rotation_the_same_way(self):
        yaws = [_wrap(math.radians(-d)) for d in range(0, 271, 5)]
        fake = self._Fake(yaw=yaws[0])
        self.assertAlmostEqual(
            math.degrees(self._drive(fake, yaws)), -270.0, places=6)

    def test_it_is_inert_outside_wall_turn(self):
        """A straight move must not build up a total a later turn inherits."""
        for mode in ('straight', None):
            fake = self._Fake(mode=mode)
            self._drive(fake, [0.0, 0.3, 0.6, 0.9])
            self.assertEqual(fake.turn_progress_rad, 0.0)

    def test_it_tolerates_a_yaw_that_is_not_known_yet(self):
        fake = self._Fake()
        fake.yaw = None
        MPCController._accumulate_turn_progress(fake)
        self.assertEqual(fake.turn_progress_rad, 0.0)

    def test_reset_starts_a_fresh_turn_from_the_current_heading(self):
        fake = self._Fake()
        self._drive(fake, [0.0, 0.5, 1.0])
        self.assertGreater(fake.turn_progress_rad, 0.0)
        fake.yaw = 1.0
        MPCController._reset_turn_progress(fake)
        self.assertEqual(fake.turn_progress_rad, 0.0)
        self.assertAlmostEqual(fake._turn_progress_last_yaw, 1.0)


@pytest.mark.skipif(not OSQP_AVAILABLE, reason="osqp not importable")
class TestTheSolverSteersTheCommandedWay(unittest.TestCase):
    """End to end through the QP the deployed backend actually solves."""

    @staticmethod
    def _steer(deg, sign, drift_deg=0.0, progress_deg=0.0, v=0.4):
        turn = math.radians(sign * deg)
        psi_start = math.radians(sign * progress_deg)
        remaining = turn - math.radians(sign * progress_deg)
        corridor = _turn_corridor_geometry(psi_start, remaining)
        u0, _ = solve_mpc_step(
            x0=np.array([0.0, 0.0, psi_start + math.radians(drift_deg), v]),
            last_u=np.zeros(2), pref_nom=_lookahead_target(corridor),
            corridor=corridor,
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS,
            weights=dict(WEIGHTS), obstacles=[], dmin=0.32, vdes=v,
            solver='rti', warm_start_z=None)
        return float(u0[0])

    def test_170_180_181_270_steer_the_commanded_way(self):
        for deg in (170, 180, 181, 270):
            for sign in (+1.0, -1.0):
                u0 = self._steer(deg, sign)
                self.assertEqual(u0 > 0, sign > 0,
                                 msg=f"{sign:+.0f} * {deg} deg -> u0={u0:+.4f}")
                self.assertGreater(abs(u0), 1e-4)

    def test_the_two_directions_are_near_mirror_images(self):
        """
        Near, not exact: the deployed steering envelope is ASYMMETRIC
        (mpc_steering_angle_min_rad -0.283 vs max +0.278, both derived by
        inverting the servo calibration), so a left turn and its mirrored
        right turn genuinely do not produce equal-magnitude commands. What
        must hold is that they are opposite in sign and the same size to
        within that asymmetry.
        """
        for deg in (170, 180, 181, 270):
            left, right = self._steer(deg, +1.0), self._steer(deg, -1.0)
            self.assertGreater(left, 0.0)
            self.assertLess(right, 0.0)
            self.assertAlmostEqual(abs(left), abs(right), delta=0.05 * abs(left))

    def test_a_wrong_way_drift_does_not_invert_the_command(self):
        """
        The ratchet, at the level that matters.

        Start the solve already pointing the wrong way -- which is what the
        old shortest-branch unwrap turned into a permanent reversal -- and
        the first control must still go the commanded way.
        """
        for deg in (180, 181, 270):
            for sign in (+1.0, -1.0):
                for drift in (2.0, 10.0, 30.0):
                    u0 = self._steer(deg, sign, drift_deg=-sign * drift)
                    self.assertEqual(
                        u0 > 0, sign > 0,
                        msg=f"{sign:+.0f}*{deg} drifted {drift} deg the wrong way")

    def test_a_turn_nearly_complete_stops_asking_for_more(self):
        """
        Progress must actually count, or the car would spin forever.

        At 175 of a commanded 180 the remaining ask is 5 degrees, so the
        steering demand must be far below what it was at 0 progress.
        """
        for sign in (+1.0, -1.0):
            fresh = abs(self._steer(180, sign, progress_deg=0.0))
            nearly = abs(self._steer(180, sign, progress_deg=175.0))
            self.assertLess(nearly, fresh)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
