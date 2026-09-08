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

Then the fix landed in the wrong frame, and a second pass moved the frozen
heading into map coordinates (see test_map_frame_anchor.py) -- which killed
the drift live. What that pass did NOT fix was the corridor's own jitter: the
origin sat at the perpendicular foot of the live pose on the frozen line, so
the geometry combined a live position with a line that itself steps whenever
a SLAM correction lands, and the corridor visibly jumped.

The behaviour under test NOW (deliberate change, agreed with Andreas, not a
regression): a straight move's corridor ALWAYS PASSES THROUGH THE CAR'S
CURRENT POSITION, pointed along the frozen, map-corrected heading. Position
tracks the car; direction does not. The explicit price is that lateral offset
is no longer restored after a deflection -- the car carries on parallel to
the intended line rather than homing back onto it -- traded for a corridor
that only ever pivots instead of translating and pivoting. The goal_distance
progress check keeps its OWN separate anchor at the original start point, so
"go 6 m" still means 6 m along the intended line; that separation is pinned
below.

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

from f1tenth_params.param_defaults import get_value
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
        # The anchor build_straight_corridor actually reads: the move's
        # start pose expressed in the CURRENT odom frame, refreshed each
        # tick by MPCController._refresh_goal_anchor() from the map-frame
        # capture (drift-correction pass -- see _pose_odom_to_map). Derived
        # here exactly the way goal_distance_callback seeds it, so passing
        # goal_start_xy/psi_init_corridor to this fake keeps selecting the
        # frozen-line path (and a None psi_init_corridor keeps selecting the
        # bootstrap fallback) as it did before that pass. Tests wanting a
        # reference line that has moved relative to odom -- the whole point
        # of the reprojection -- set this attribute directly.
        self.goal_anchor_odom = (
            None if (goal_start_xy is None or psi_init_corridor is None)
            else (goal_start_xy[0], goal_start_xy[1], psi_init_corridor))
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


class TestCarTrackingCorridorOnAFrozenHeading(unittest.TestCase):
    """
    The straight-move corridor's position/direction split.

    POSITION follows the car: the corridor always starts at the live pose.
    The TARGET heading is frozen at move start (and map-corrected since -- see
    test_map_frame_anchor.py), and the corridor blends from the car's LIVE
    yaw back to it across its own length. Rebuilding mid-move from a drifted
    pose must not move the reference DIRECTION the corridor ends on, must put
    the corridor through wherever the car actually is, and must bend between
    the two -- that bend is the heading return.
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

    def test_with_the_blend_on_it_starts_live_and_ends_frozen(self):
        """
        Confirm WHICH END is frozen when corridor_heading_return is TRUE.

        NOT the shipping default -- see
        test_the_default_geometry_freezes_both_ends below. This pins the
        reference-implementation geometry, which stays reachable as a launch
        arg for the f110_autonomy/MATLAB comparison, so the flag is set
        explicitly here rather than left to the getattr fallback.

        The car has drifted: it sits at (1.4, 0.35) pointing 0.20 rad while
        the move was started pointing 0.0. With the blend on, the corridor
        LEAVES the car tangent to where it is actually pointing (psiStart =
        live yaw) and ARRIVES pointing where the move was supposed to go
        (psiEnd = frozen anchor heading).
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0))
        fake.corridor_heading_return = True
        corridor = MPCController.build_straight_corridor(
            fake, [1.4, 0.35, 0.20, 0.5])

        # START heading == the LIVE yaw.
        self.assertAlmostEqual(float(corridor['psiStart']), 0.20, places=9)
        # END heading == the FROZEN move-start heading.
        self.assertAlmostEqual(float(corridor['psiRef']), 0.0, places=9)
        # ...and the blend spans exactly the accumulated heading error.
        self.assertAlmostEqual(float(corridor['dpsi']), -0.20, places=9)

        # Measured off the geometry itself, not just the reported scalars:
        # the first samples still run along the live yaw (the S-curve's flat
        # lead-in) and the last along the frozen one (its flat lead-out).
        xc, yc = corridor['xc'], corridor['yc']
        self.assertAlmostEqual(
            math.atan2(yc[2] - yc[1], xc[2] - xc[1]), 0.20, delta=1e-6)
        self.assertAlmostEqual(
            math.atan2(yc[-1] - yc[-2], xc[-1] - xc[-2]), 0.0, delta=1e-6)

    def test_with_the_blend_on_the_drifted_centerline_actually_bends(self):
        """
        Confirm the centerline curves when the car is off-heading AND the
        blend is enabled.

        Complement to the test above, stated as the shape rather than the
        endpoints: a deviated car gets a CURVED reference, and the curvature
        is what the lookahead target rides back along. Again NOT the default
        geometry -- the flag is set explicitly.
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0))
        fake.corridor_heading_return = True
        corridor = MPCController.build_straight_corridor(
            fake, [1.4, 0.35, 0.20, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        headings = [
            math.atan2(yc[i] - yc[i - 1], xc[i] - xc[i - 1])
            for i in range(1, len(xc))
        ]
        self.assertGreater(max(headings) - min(headings), 0.19)
        # Monotone return: the blend never overshoots past the frozen heading
        # and never turns away from it.
        for a, b in zip(headings, headings[1:]):
            self.assertLessEqual(b, a + 1e-9)

    def test_with_the_blend_off_both_ends_are_frozen(self):
        """
        Confirm corridor_heading_return=False gives the both-ends-frozen shape.

        THIS IS THE SHIPPING GEOMETRY as of 2026-09-08 (see
        test_the_default_geometry_freezes_both_ends for the default itself).
        The two shapes do not rank the way the port expected: both-ends-frozen
        demands 100% of the heading correction on every rebuild -- through
        w_psi's terminal pull toward psiRef, not through the centerline arc --
        and is back at zero heading error inside 8 s, while the reference
        blend concedes ~75% of the error per rebuild, takes ~30 s, and gives
        away ~0.9 m of lateral offset that nothing restores on the way.
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0))
        fake.corridor_heading_return = False
        corridor = MPCController.build_straight_corridor(
            fake, [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(float(corridor['dpsi']), 0.0, places=12)
        self.assertAlmostEqual(float(corridor['psiStart']), 0.0, places=9)
        self.assertAlmostEqual(float(corridor['psiRef']), 0.0, places=9)
        for y in corridor['yc']:
            self.assertAlmostEqual(float(y), 0.35, places=6)

    def test_the_default_geometry_freezes_both_ends(self):
        """Confirm the default is BOTH-ENDS-FROZEN, sourced from the yaml.

        THIS TEST ENCODES A CONFIG DEFAULT, so it is a coupling: flipping
        corridor_heading_return in stack_params.yaml means changing this test
        in the same commit. It deliberately asserts against get_value() rather
        than a hardcoded False -- the point of the single-sourcing pass is that
        there is exactly one place the value lives, and a test with its own
        copy of it would be a fifth spelling of the same constant.
        """
        fake = _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0))
        # No attribute -> build_straight_corridor's getattr fallback decides,
        # and that fallback must be the shipped default.
        self.assertFalse(hasattr(fake, 'corridor_heading_return'))
        self.assertFalse(get_value('corridor_heading_return'))
        corridor = MPCController.build_straight_corridor(
            fake, [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(float(corridor['dpsi']), 0.0, places=12)

    def test_the_corridor_origin_is_the_cars_own_position(self):
        """
        Confirm the origin is the live pose, not a projection onto a line.

        The car is at (1.4, 0.35); the corridor starts exactly there. No
        perpendicular foot is computed any more -- that was the jittery part.
        Unchanged by the heading-return fix, which only moved which heading
        each END of the blend uses: the ORIGIN is still the live position.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        self.assertAlmostEqual(float(corridor['xc'][0]), 1.4, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.35, places=6)
        # The corridor is corr_L_base long measured ALONG ITS OWN ARC (it is
        # no longer a straight line, so Pend is not origin + L*e_frozen).
        arc = sum(
            math.hypot(corridor['xc'][i] - corridor['xc'][i - 1],
                       corridor['yc'][i] - corridor['yc'][i - 1])
            for i in range(1, len(corridor['xc'])))
        self.assertAlmostEqual(arc, 3.0, delta=3.0 / 120.0 + 1e-6)

    def test_pure_lateral_drift_translates_the_corridor_with_the_car(self):
        """
        Confirm sideways drift carries the corridor sideways with it.

        No along-line progress, 0.5m of pure sideways drift: the corridor is
        the move-start one shifted 0.5m across, unrotated. This is the
        no-lateral-homing trade stated explicitly -- the corridor follows,
        it does not pull back.
        """
        at_start = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.0, 0.0, 0.5])
        drifted = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.5, 0.0, 0.5])
        for a, b in zip(at_start['yc'], drifted['yc']):
            self.assertAlmostEqual(float(a) + 0.5, float(b), places=9)
        for a, b in zip(at_start['xc'], drifted['xc']):
            self.assertAlmostEqual(float(a), float(b), places=9)

    def test_distance_from_the_centerline_is_zero_by_construction_again(self):
        """
        Pin the accepted cost of tracking the car.

        The car sits ON its own corridor's centerline, so the half-width bound
        and w_corr have no lateral error to act on -- lateral recovery is
        given up here on purpose. The frozen HEADING is what still corrects
        the drift this whole line of work started with.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [1.4, 0.35, 0.20, 0.5])
        d_lat = min(abs(0.35 - float(y)) for y in corridor['yc'])
        self.assertAlmostEqual(d_lat, 0.0, places=9)

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
        # Origin is the car itself -- the 0.4m of lateral offset is kept, not
        # projected away, and the live yaw (psi + 0.25) is still ignored.
        self.assertAlmostEqual(float(corridor['xc'][0]), px, places=6)
        self.assertAlmostEqual(float(corridor['yc'][0]), py, places=6)

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
