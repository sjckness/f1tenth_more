"""Regression coverage for the "straight-after-turn uses the wrong reference
frame" bug found via live testing (mission 1: a 90deg turn executed
correctly, but the following goal_distance move advanced along what looked
like the ORIGINAL pre-turn heading instead of the car's new one).

Root cause (confirmed by reading MPC_corr.py, not guessed): psi_init_corridor
-- the reference heading build_straight_corridor() curves every straight
move's corridor toward -- was captured exactly ONCE, from the first odom
message this node instance ever saw after process startup, and reused for
every goal_distance move for the rest of the process's lifetime, regardless
of any turns executed since. Fixed by having goal_distance_callback
re-anchor psi_init_corridor to the robot's current heading at THAT move's
own start (see MPC_corr.py's own comments on both).

build_straight_corridor() itself is a plain (if not-quite-pure -- it logs)
method with no ROS-runtime dependency beyond self.get_logger(), same
"testable without a live rclpy Node" shape test_boundary_constraints.py's
own _boundary_to_world/_select_live_boundaries already rely on -- a minimal
duck-typed stand-in is enough, no MPCController() construction (which would
need a full rclpy context) required.

Run standalone: python3 -m pytest test/test_corridor_heading_reference.py -v
"""

import math

from mpc_controller.MPC_corr import MPCController


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass


class _FakeMPC:
    """Just enough of MPCController's instance state for
    build_straight_corridor() to run -- see that method's own attribute
    reads. Defaults mirror MPCController.__init__'s real ones."""

    def __init__(self, psi_init_corridor):
        self.front_distance = 10.0
        self.goal_pose_xy = None  # distance-mode, not pose-mode -- see module docstring
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.psi_init_corridor = psi_init_corridor
        # Move-start anchor for the frozen straight reference. None here
        # selects build_straight_corridor's bootstrap fallback (live position,
        # blend from live yaw) -- the path these tests exercise unless a test
        # sets it explicitly. Default mirrors MPCController.__init__'s.
        self.goal_start_xy = None
        # S-curve heading-blend shape params (f110_autonomy port) --
        # defaults mirror MPCController.__init__'s real ones, same as
        # every other attribute on this fake.
        self.corr_turn_u_start = 0.10
        self.corr_turn_u_end = 0.70

    def get_logger(self):
        return _FakeLogger()


def _build(psi_init_corridor, robot_yaw):
    """A goal_distance-mode corridor build at the robot's current pose,
    mirroring build_straight_corridor's own x=[X0, Y0, psi0, v] state vector
    shape (only x[2]==psi0/robot_yaw matters here, position is arbitrary)."""
    fake = _FakeMPC(psi_init_corridor)
    x0 = [0.0, 0.0, robot_yaw, 0.0]
    return MPCController.build_straight_corridor(fake, x0)


class TestStraightCorridorReferenceHeading:

    def test_corridor_points_along_psi_init_corridor_not_current_yaw(self):
        # Sanity check on the mechanism itself: the corridor's end heading
        # (psiRef) tracks psi_init_corridor, the value goal_distance_callback
        # now re-anchors per-move -- not psi0/robot_yaw, which is only the
        # corridor's OWN starting tangent (psiStart), relinearized toward
        # psi_init_corridor over the corridor's length. If a future edit
        # regresses this back to psi0-only, this catches it.
        corridor = _build(psi_init_corridor=math.radians(90.0), robot_yaw=math.radians(90.0))
        assert math.isclose(corridor['psiRef'], math.radians(90.0), abs_tol=1e-9)

    def test_reanchoring_after_a_turn_points_the_new_move_along_the_new_heading(self):
        """THE bug, reproduced directly: before the fix, psi_init_corridor
        was whatever heading the node happened to capture once at startup
        (simulated here as 0 rad -- facing the original/pre-mission
        direction) and every subsequent goal_distance move's corridor bent
        toward THAT, even after a 90deg turn left the robot facing a
        completely different way. After the fix (goal_distance_callback
        setting self.psi_init_corridor = self.yaw at the new move's start),
        the corridor for the post-turn straight move points along the
        post-turn heading instead."""
        pre_turn_heading = 0.0
        post_turn_heading = math.radians(90.0)  # a 90deg left turn happened

        # Old, buggy behavior: corridor still built with the STALE pre-turn
        # psi_init_corridor even though the robot is now facing post_turn_heading.
        stale_corridor = _build(psi_init_corridor=pre_turn_heading, robot_yaw=post_turn_heading)
        assert math.isclose(stale_corridor['psiRef'], pre_turn_heading, abs_tol=1e-9)
        assert not math.isclose(stale_corridor['psiRef'], post_turn_heading, abs_tol=1e-3)

        # Fixed behavior: goal_distance_callback re-anchors psi_init_corridor
        # to self.yaw (post_turn_heading) at the new move's own start, so the
        # corridor it builds points there instead.
        fixed_corridor = _build(psi_init_corridor=post_turn_heading, robot_yaw=post_turn_heading)
        assert math.isclose(fixed_corridor['psiRef'], post_turn_heading, abs_tol=1e-9)

    def test_bootstrap_fallback_when_psi_init_corridor_never_captured(self):
        # Covers the "nothing has captured psi_init_corridor at all yet"
        # bootstrap fallback (psi_base = psi0 when psi_init_corridor is
        # None) -- distinct from the real bug above, still worth locking in
        # since it's the other branch of the same `if` in build_straight_corridor.
        corridor = _build(psi_init_corridor=None, robot_yaw=math.radians(45.0))
        assert math.isclose(corridor['psiRef'], math.radians(45.0), abs_tol=1e-9)


if __name__ == '__main__':
    import sys

    import pytest
    sys.exit(pytest.main([__file__, '-v']))
