"""Coverage for goal_distance direction recovery: the along-line progress
projection used by the termination check, and the corridor-geometry contract
the recovery relies on.

The behaviour under test: after an obstacle deflection the car recovers the
DIRECTION of the corridor captured at move start (psi_init_corridor) and then
runs parallel to it -- it does NOT merge back onto that line's exact lateral
position. build_straight_corridor already produced that shape; what was
missing was any cost pulling the solver onto it (w_psi was zeroed AND its
term was commented out in mpc_solver, so it was inert either way). These
tests pin the geometry contract so a future change cannot quietly turn the
direction reference into a lateral one -- and cannot quietly drop it either.

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
    """Just enough of MPCController's instance state for
    build_straight_corridor(); defaults mirror MPCController.__init__'s."""

    def __init__(self, psi_init_corridor=0.0, goal_pose_xy=None):
        self.front_distance = 10.0
        self.goal_pose_xy = goal_pose_xy
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.psi_init_corridor = psi_init_corridor

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


class TestCorridorIsADirectionReferenceNotALateralOne(unittest.TestCase):

    def test_far_end_keeps_the_cars_lateral_offset(self):
        """Car 0.8m off the line it started on: the corridor must NOT pull its
        far end back onto that line. Recovering lateral position is explicitly
        not the design -- only direction is."""
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
        """psiRef -- what mpc_solver's terminal-yaw cost (w_psi) now pulls
        toward -- is the heading captured at move start, not the live yaw."""
        fake = _FakeMPC(psi_init_corridor=0.25)
        corridor = MPCController.build_straight_corridor(
            fake, [1.0, 0.8, -0.4, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], 0.25, places=9)

    def test_centerline_heading_blends_from_live_yaw_to_the_reference(self):
        """The corridor leaves the car along the car's own heading and turns
        to parallel with the reference by its far end -- gradually, so the
        pull cannot fight an in-progress avoidance manoeuvre."""
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.5, 0.4, 0.5])
        xc, yc = corridor['xc'], corridor['yc']
        head_start = math.atan2(yc[1] - yc[0], xc[1] - xc[0])
        head_end = math.atan2(yc[-1] - yc[-2], xc[-1] - xc[-2])
        self.assertAlmostEqual(head_start, 0.4, delta=0.02)
        self.assertAlmostEqual(head_end, 0.0, delta=0.02)

    def test_centerline_stays_parallel_offset_when_already_aligned(self):
        """Aligned but 0.6m off to the side: the centerline is a straight line
        parallel to the reference, holding that offset all the way -- not a
        curve converging onto y=0."""
        fake = _FakeMPC(psi_init_corridor=0.0)
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.6, 0.0, 0.5])
        for y in corridor['yc']:
            self.assertAlmostEqual(float(y), 0.6, places=6)

    def test_reference_direction_is_independent_of_position(self):
        """Two cars at different lateral offsets, same heading: identical
        corridor DIRECTION, corridors merely translated. A position-dependent
        far end (the lateral-recentering design that was considered and
        dropped) would break this."""
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


class TestGoalPoseModeUnchanged(unittest.TestCase):
    """Scope guard: the fix is goal_distance-only. goal_pose/goal_turn mode is
    a different, point-based (beeline) mechanism and must behave as before."""

    def test_goal_pose_reference_heading_points_at_the_goal(self):
        fake = _FakeMPC(psi_init_corridor=0.0, goal_pose_xy=(3.0, 3.0))
        corridor = MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], math.atan2(3.0, 3.0), places=6)

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
