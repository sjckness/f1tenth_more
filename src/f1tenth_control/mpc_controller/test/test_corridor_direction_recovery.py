"""
Coverage for goal_distance reference recovery.

The along-line progress projection used by the termination check, and the
corridor-geometry contract that recovery relies on.

HISTORY, because this file's contract was deliberately inverted and the old
version of it is still worth knowing: the corridor used to be a
DIRECTION-ONLY reference -- built from the car's LIVE position and LIVE yaw
every rebuild, blending toward the heading captured at move start, so the car
recovered only PARALLEL travel and kept whatever lateral offset it had picked
up. That left a straight move with no lateral restoring force at all, which
turned out to be a drift-following loop: mechanical bias pushed the car
sideways, the next rebuild translated the whole reference sideways with it,
and nothing ever opposed the drift.

The behaviour under test NOW: a straight (goal_distance) move references a
line FROZEN at move start -- goal_start_xy for position, psi_init_corridor
for heading -- and only the corridor's forward extent advances with the car.
The old direction-only shape survives exactly as the bootstrap fallback for
when no move start has been recorded yet, and is pinned as such below.

Same "testable without constructing a real MPCController" shape as
test_corridor_heading_reference.py and test_boundary_constraints.py: a
duck-typed stand-in for the method, and a plain module-level pure function
(_project_onto_line) for the projection itself.

Run standalone: python3 -m pytest test/test_corridor_direction_recovery.py -v
"""

import math
import unittest

from mpc_controller.MPC_corr import MPCController, _project_onto_line


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass


class _FakeMPC:
    """
    Just enough of MPCController's instance state for build_straight_corridor().

    Defaults mirror MPCController.__init__'s real ones.
    """

    def __init__(self, psi_init_corridor=0.0, goal_pose_xy=None, goal_start_xy=None):
        self.front_distance = 10.0
        self.goal_pose_xy = goal_pose_xy
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.psi_init_corridor = psi_init_corridor
        # Move-start anchor for the frozen straight reference. None selects
        # build_straight_corridor's bootstrap fallback (live position, blend
        # from live yaw); a tuple selects the frozen-line path a real
        # goal_distance move takes. Default mirrors MPCController.__init__'s.
        self.goal_start_xy = goal_start_xy
        # S-curve heading-blend shape params (f110_autonomy port) --
        # defaults mirror MPCController.__init__'s real ones, same as
        # every other attribute on this fake.
        self.corr_turn_u_start = 0.10
        self.corr_turn_u_end = 0.70

    def get_logger(self):
        return _FakeLogger()


class TestProjectOntoLine(unittest.TestCase):
    """The pure projection behind the goal_distance termination check."""

    def test_robot_exactly_on_the_line_gives_its_travelled_distance(self):
        self.assertAlmostEqual(
            _project_onto_line((2.0, 0.0), (0.0, 0.0), 0.0), 2.0)

    def test_robot_behind_the_origin_projects_negative(self):
        self.assertAlmostEqual(
            _project_onto_line((-0.75, 0.0), (0.0, 0.0), 0.0), -0.75)

    def test_lateral_offset_does_not_count_as_progress(self):
        # 3m along, 4m sideways: straight-line displacement is 5m (the old
        # termination metric), along-direction progress is 3m. This is the
        # whole point -- the car keeps that lateral offset by design, so the
        # two metrics stay apart for the rest of the move.
        self.assertAlmostEqual(
            _project_onto_line((3.0, 4.0), (0.0, 0.0), 0.0), 3.0)
        self.assertAlmostEqual(math.hypot(3.0, 4.0), 5.0)

    def test_pure_lateral_motion_is_zero_progress(self):
        self.assertAlmostEqual(
            _project_onto_line((0.0, 1.5), (0.0, 0.0), 0.0), 0.0)

    def test_works_on_a_rotated_line(self):
        yaw = math.pi / 4.0
        point = (2.0 * math.cos(yaw) - 0.5 * math.sin(yaw),
                 2.0 * math.sin(yaw) + 0.5 * math.cos(yaw))
        self.assertAlmostEqual(_project_onto_line(point, (0.0, 0.0), yaw), 2.0)

    def test_origin_offset_is_subtracted(self):
        self.assertAlmostEqual(
            _project_onto_line((5.0, 1.0), (1.0, 1.0), 0.0), 4.0)


class TestBootstrapFallbackIsStillDirectionOnly(unittest.TestCase):
    """
    The pre-fix shape, kept as the no-move-start-anchor fallback.

    Every test here leaves goal_start_xy None, which is what the node looks
    like before any goal_distance has arrived (only the odom bootstrap has
    captured psi_init_corridor). With no frozen line to reference, the
    corridor is still built off the live pose and still recovers direction
    only. Pinned so the fallback cannot silently change shape either.
    """

    def test_far_end_keeps_the_cars_lateral_offset(self):
        """
        Confirm the far end keeps the car's lateral offset on this path.

        Car 0.8m off the line, no anchor captured: the corridor must NOT pull
        its far end back onto that line -- on THIS path recovering lateral
        position is not the design, only direction is.
        """
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [1.0, 0.8, 0.0, 0.5])
        self.assertAlmostEqual(float(corridor['Pend'][1]), 0.8, places=6)

    def test_near_end_starts_at_the_robot(self):
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [1.0, 0.8, 0.0, 0.5])
        self.assertAlmostEqual(float(corridor['xc'][0]), 1.0, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.8, places=6)

    def test_reference_heading_is_the_captured_one(self):
        """
        Confirm psiRef is the captured heading, not the live yaw.

        psiRef is what mpc_solver's terminal-yaw cost (w_psi) pulls toward.
        """
        fake = _FakeMPC(psi_init_corridor=0.25)
        corridor = MPCController.build_straight_corridor(
            fake, [1.0, 0.8, -0.4, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], 0.25, places=9)

    def test_centerline_heading_blends_from_live_yaw_to_the_reference(self):
        """
        Confirm the centerline blends from the live yaw to the reference.

        The corridor leaves the car along the car's own heading and turns to
        parallel with the reference by its far end -- gradually, so the pull
        cannot fight an in-progress avoidance manoeuvre.
        """
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.5, 0.4, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        head_start = math.atan2(yc[1] - yc[0], xc[1] - xc[0])
        head_end = math.atan2(yc[-1] - yc[-2], xc[-1] - xc[-2])
        self.assertAlmostEqual(head_start, 0.4, delta=0.02)
        self.assertAlmostEqual(head_end, 0.0, delta=0.02)

    def test_centerline_stays_parallel_offset_when_already_aligned(self):
        """
        Confirm an already-aligned car gets a parallel, offset centerline.

        Aligned but 0.6m off to the side: the centerline is a straight line
        parallel to the reference, holding that offset all the way -- not a
        curve converging onto y=0.
        """
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.6, 0.0, 0.5])
        for y in corridor['yc']:
            self.assertAlmostEqual(float(y), 0.6, places=6)

    def test_reference_direction_is_independent_of_position(self):
        """
        Confirm the reference direction does not depend on position.

        Two cars at different lateral offsets, same heading: identical
        corridor DIRECTION, corridors merely translated. Only true on the
        fallback path -- see TestFrozenStraightReference for the anchored
        path, where position is exactly what the corridor is pinned to.
        """
        a = MPCController.build_straight_corridor(_FakeMPC(0.0), [0.0, 0.0, 0.0, 0.5])
        b = MPCController.build_straight_corridor(_FakeMPC(0.0), [0.0, 1.2, 0.0, 0.5])
        self.assertAlmostEqual(a['psiRef'], b['psiRef'], places=9)
        for ya, yb in zip(a['yc'], b['yc']):
            self.assertAlmostEqual(float(yb) - float(ya), 1.2, places=6)

    def test_rotated_reference_direction_is_handled(self):
        yaw = math.radians(35.0)
        fake = _FakeMPC(psi_init_corridor=yaw)
        corridor = MPCController.build_straight_corridor(fake, [1.0, -2.0, yaw, 0.5])
        head_end = math.atan2(corridor['yc'][-1] - corridor['yc'][-2],
                              corridor['xc'][-1] - corridor['xc'][-2])
        self.assertAlmostEqual(head_end, yaw, delta=0.02)

    def test_bootstrap_fallback_when_no_reference_heading_captured(self):
        fake = _FakeMPC(psi_init_corridor=None)
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.3, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], 0.3, places=9)


class TestFrozenStraightReference(unittest.TestCase):
    """
    THE regression tests for the drift-following bug.

    A straight move's corridor is anchored to the line frozen at move start
    (goal_start_xy + psi_init_corridor). Rebuilding it mid-move from a pose
    that has drifted must not move that line -- only slide the corridor's
    origin forward along it.
    """

    def test_drifted_rebuild_returns_the_same_reference_heading(self):
        """
        Confirm a rebuild from a drifted pose keeps the move-start heading.

        The bug itself: the car has drifted 0.35m left and its yaw has rotated
        0.20 rad with it. A corridor rebuilt from that pose must still
        reference the heading the move started in, not the drifted one.
        """
        at_start = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.0, 0.0, 0.5])
        mid_move = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(mid_move['psiRef'], at_start['psiRef'], places=9)
        self.assertAlmostEqual(mid_move['psiRef'], 0.0, places=9)

    def test_drifted_rebuild_puts_the_centerline_back_on_the_frozen_line(self):
        """
        Confirm a rebuild puts the centerline back on the frozen line.

        Position half of the same bug: the centerline must sit on y=0 (the
        frozen line), NOT on y=0.35 (where the car has drifted to).
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        for y in corridor['yc']:
            self.assertAlmostEqual(float(y), 0.0, places=6)

    def test_centerline_has_no_bend_at_all(self):
        """
        Confirm the anchored centerline has no bend at all.

        psiStart == psiEnd on the anchored path, so the S-curve blend is inert
        and every sample shares one heading -- a straight move's reference is
        a straight line, whatever the car is doing.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(float(corridor['dpsi']), 0.0, places=12)
        xc, yc = corridor['xc'], corridor['yc']
        for i in range(1, len(xc)):
            self.assertAlmostEqual(
                math.atan2(yc[i] - yc[i - 1], xc[i] - xc[i - 1]), 0.0, delta=1e-9)

    def test_only_the_forward_extent_advances_with_the_car(self):
        """
        Confirm only the forward extent advances with the car.

        The origin slides along the frozen line by along-line progress and by
        nothing else: 1.4m made good puts the corridor start at x=1.4, y=0
        even though the car is at y=0.35.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(float(corridor['xc'][0]), 1.4, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.0, places=6)
        self.assertAlmostEqual(float(corridor['Pend'][0]), 1.4 + 3.0, places=6)
        self.assertAlmostEqual(float(corridor['Pend'][1]), 0.0, places=6)

    def test_pure_lateral_drift_does_not_advance_the_corridor(self):
        """
        Confirm pure lateral drift does not advance the corridor.

        No along-line progress, 0.5m of pure sideways drift: the corridor is
        identical to the one built at move start. That is the closed loop the
        bug had -- before the fix this corridor translated 0.5m sideways.
        """
        at_start = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.0, 0.0, 0.5])
        drifted = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.5, 0.0, 0.5])
        for a, b in zip(at_start['yc'], drifted['yc']):
            self.assertAlmostEqual(float(a), float(b), places=9)
        for a, b in zip(at_start['xc'], drifted['xc']):
            self.assertAlmostEqual(float(a), float(b), places=9)

    def test_lateral_offset_is_now_a_real_nonzero_quantity(self):
        """
        Confirm lateral offset is now a real, non-zero quantity.

        The whole point of freezing the line: the car's distance from the
        centerline is no longer zero by construction, so the corridor bound
        and the lookahead target finally have an error to act on.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        d_lat = min(abs(0.35 - float(y)) for y in corridor['yc'])
        self.assertAlmostEqual(d_lat, 0.35, places=6)

    def test_works_on_a_rotated_frozen_line(self):
        """Confirm nothing above depends on the frozen line being the x axis."""
        psi = math.radians(35.0)
        anchor = (1.0, -2.0)
        made_good = 0.9
        lateral = 0.4
        px = anchor[0] + made_good * math.cos(psi) - lateral * math.sin(psi)
        py = anchor[1] + made_good * math.sin(psi) + lateral * math.cos(psi)
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=psi, goal_start_xy=anchor),
            [px, py, psi + 0.25, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], psi, places=9)
        self.assertAlmostEqual(
            float(corridor['xc'][0]), anchor[0] + made_good * math.cos(psi), places=6)
        self.assertAlmostEqual(
            float(corridor['yc'][0]), anchor[1] + made_good * math.sin(psi), places=6)

    def test_anchor_without_a_captured_heading_falls_back(self):
        """
        Confirm both halves of the anchor are required.

        psi_init_corridor None (the pre-bootstrap state) still takes the
        live-pose fallback even when goal_start_xy is set.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=None, goal_start_xy=(0.0, 0.0)),
            [1.0, 0.6, 0.3, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], 0.3, places=9)
        self.assertAlmostEqual(float(corridor['xc'][0]), 1.0, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.6, places=6)


class TestGoalPoseModeUnchanged(unittest.TestCase):
    """
    Scope guard: goal_pose/goal_turn mode must behave exactly as before.

    It is a different, point-based (beeline) mechanism, and turns dispatch
    through it (goal_turn_callback -> goal_pose_xy), so the frozen-straight
    anchoring must not reach them -- including when a stale goal_start_xy is
    still set from a previous straight move.
    """

    def test_goal_pose_reference_heading_points_at_the_goal(self):
        fake = _FakeMPC(psi_init_corridor=0.0, goal_pose_xy=(3.0, 3.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], math.atan2(3.0, 3.0), places=6)

    def test_goal_pose_ignores_a_stale_move_start_anchor(self):
        """
        Confirm goal_pose mode ignores a stale move-start anchor.

        A goal_distance move leaves goal_start_xy/psi_init_corridor set; a
        turn published straight after must still build off the LIVE pose.
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_pose_xy=(3.0, 0.0),
                        goal_start_xy=(0.0, 0.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.8, 0.0, 0.5])
        self.assertAlmostEqual(float(corridor['xc'][0]), 0.0, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.8, places=6)
        self.assertAlmostEqual(corridor['psiRef'], math.atan2(-0.8, 3.0), places=6)

    def test_goal_pose_ignores_psi_init_corridor(self):
        goal = (2.0, 1.0)
        a = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_pose_xy=goal), [0.5, 0.4, 0.1, 0.5])
        b = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=-1.2, goal_pose_xy=goal), [0.5, 0.4, 0.1, 0.5])
        self.assertAlmostEqual(float(a['Pend'][0]), float(b['Pend'][0]), places=9)
        self.assertAlmostEqual(float(a['Pend'][1]), float(b['Pend'][1]), places=9)


if __name__ == '__main__':
    unittest.main()
