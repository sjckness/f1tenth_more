"""
Coverage for the S-curve heading-blend shape.

Ported into build_straight_corridor from f110_autonomy
(build_returning_corridor_explicit_t) at Andreas's explicit request -- see
that method's own comment for the full rationale, including why this only
governs the near-term reference within one replan window at this pass's
corridor-rebuild rate, not a standing scripted turn.

Same "testable without constructing a real MPCController" shape as
test_corridor_heading_reference.py/test_corridor_direction_recovery.py: a
duck-typed stand-in for the method.

Run standalone: python3 -m pytest test/test_corridor_turn_shape.py -v
"""

import math
import unittest

from mpc_controller.MPC_corr import MPCController


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass


class _FakeMPC:
    """
    Just enough of MPCController's instance state for the method under test.

    Defaults mirror MPCController.__init__'s real ones, same as the other
    corridor test files' own _FakeMPC.
    """

    def __init__(self, psi_init_corridor=0.0, goal_pose_xy=None,
                 corr_turn_u_start=0.10, corr_turn_u_end=0.70):
        self.front_distance = 10.0
        self.goal_pose_xy = goal_pose_xy
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.psi_init_corridor = psi_init_corridor
        self.corr_turn_u_start = corr_turn_u_start
        self.corr_turn_u_end = corr_turn_u_end

    def get_logger(self):
        return _FakeLogger()


def _heading_at(xc, yc, i):
    """
    Return the local tangent direction (rad) at sample i.

    Via the segment into it (i>0) or out of it (i==0) -- same technique
    test_corridor_direction_recovery.py's own head_start/head_end already
    use.
    """
    if i == 0:
        return math.atan2(yc[1] - yc[0], xc[1] - xc[0])
    return math.atan2(yc[i] - yc[i - 1], xc[i] - xc[i - 1])


class TestSCurveHeadingBlend(unittest.TestCase):
    """build_straight_corridor's S-curve heading-blend shape."""

    def test_lead_in_is_flat_at_psi_start(self):
        """
        Confirm the lead-in segment holds psiStart exactly.

        Every sample inside [0, corr_turn_u_start] must hold psiStart
        exactly (shape(tau<=0) == 0) -- the whole point of a straight
        lead-in, as opposed to the old linear taper which started curving
        immediately from u=0.
        """
        fake = _FakeMPC(psi_init_corridor=math.radians(90.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        u_start_idx = int(0.10 * (fake.corr_N - 1))
        for i in range(0, max(u_start_idx - 1, 1)):
            heading = _heading_at(corridor['xc'], corridor['yc'], i)
            self.assertAlmostEqual(heading, 0.0, delta=1e-3)

    def test_lead_out_is_flat_at_psi_end(self):
        """Confirm every sample inside [corr_turn_u_end, 1] holds psiEnd exactly."""
        psi_end = math.radians(90.0)
        fake = _FakeMPC(psi_init_corridor=psi_end)
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        n = fake.corr_N
        u_end_idx = int(0.70 * (n - 1))
        for i in range(u_end_idx + 2, n):
            heading = _heading_at(corridor['xc'], corridor['yc'], i)
            self.assertAlmostEqual(heading, psi_end, delta=1e-3)

    def test_endpoints_still_hit_psi_start_and_psi_end_exactly(self):
        """
        Regression guard on the S-curve's boundary conditions.

        Matches test_corridor_direction_recovery.py's own
        test_centerline_heading_blends_from_live_yaw_to_the_reference -- the
        S-curve must still hit the same boundary conditions the linear
        taper did, just with a different shape in between.
        """
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.5, 0.4, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        head_start = math.atan2(yc[1] - yc[0], xc[1] - xc[0])
        head_end = math.atan2(yc[-1] - yc[-2], xc[-1] - xc[-2])
        self.assertAlmostEqual(head_start, 0.4, delta=0.02)
        self.assertAlmostEqual(head_end, 0.0, delta=0.02)

    def test_heading_progression_is_monotonic_across_the_bend(self):
        """
        Confirm the bend is monotonic, with no overshoot or oscillation.

        Within [corr_turn_u_start, corr_turn_u_end] the heading must move
        monotonically from psiStart to psiEnd, confirming
        3*tau**2 - 2*tau**3 (a monotonic sigmoid on [0,1]) was wired up
        correctly, not some other non-monotonic shape.
        """
        fake = _FakeMPC(psi_init_corridor=math.radians(60.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        headings = [_heading_at(xc, yc, i) for i in range(len(xc))]
        # Monotonic non-decreasing (turning toward +60deg): each step's
        # heading should never exceed the next by more than fp noise.
        for a, b in zip(headings, headings[1:]):
            self.assertLessEqual(a - 1e-6, b)

    def test_dpsi_zero_is_unaffected_by_the_shape_change(self):
        """
        Confirm the already-aligned degenerate case is unaffected.

        When psiStart == psiEnd (already aligned), the blend shape is
        irrelevant -- theta is constant regardless -- matching
        test_corridor_direction_recovery.py's own
        test_centerline_stays_parallel_offset_when_already_aligned. Pinned
        here too since it's the shape function's own degenerate case, not
        just the corridor's.
        """
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.6, 0.0, 0.5])
        for y in corridor['yc']:
            self.assertAlmostEqual(float(y), 0.6, places=6)

    def test_goal_pose_mode_also_uses_the_s_curve(self):
        """
        Confirm the shape change applies to both psiEnd branches.

        Scope check: the shape change applies to both branches of
        build_straight_corridor's psiEnd computation (goal_pose_xy set or
        not) -- it only changes the shape of the blend toward whatever
        psiEnd already is, not which branch computes psiEnd.
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_pose_xy=(3.0, 3.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        head_start = _heading_at(xc, yc, 0)
        self.assertAlmostEqual(head_start, 0.0, delta=1e-3)
        self.assertAlmostEqual(corridor['psiRef'], math.atan2(3.0, 3.0), places=6)


if __name__ == '__main__':
    unittest.main()
