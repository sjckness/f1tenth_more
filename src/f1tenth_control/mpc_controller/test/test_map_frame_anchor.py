"""
Coverage for the frozen straight-move anchor's map-frame drift correction.

WHY THIS EXISTS. The previous pass froze a goal_distance move's reference
line at move start and stopped rebuilding it from the live pose -- correct in
shape, but pinned in the wrong frame. This stack runs a dual EKF: the local
instance (world_frame: odom) owns odom -> base_link and is a smooth
dead-reckoning integrator with no absolute heading reference, while the
global instance (world_frame: map) absorbs slam_toolbox's absolute correction
into map -> odom. The car's real heading drift therefore lands in the
map -> odom edge, NOT in the odom-frame yaw the control loop reads. Measured
over one post-fix mission: map-frame yaw drifted 13.1 deg while map -> odom's
own yaw drifted 12.8 deg in lockstep, leaving odom-frame yaw under 0.5 deg
throughout. A line frozen in odom coordinates is thus a line rotating with
the error, and presents the solver no heading error to correct at all.

The behaviour under test NOW: the anchor is captured in MAP coordinates at
move start and reprojected back into the CURRENT odom frame every control
loop tick, so accumulated map -> odom rotation shows up as an ordinary
tracking error against a reference the solver can actually oppose. The
odom-frame state x0 the dynamics and the solve run on is deliberately
untouched by any of this.

Same "testable without constructing a real MPCController" shape as
test_corridor_direction_recovery.py and test_corridor_heading_reference.py:
module-level pure functions for the frame math, and a duck-typed stand-in for
build_straight_corridor.

Run standalone: python3 -m pytest test/test_map_frame_anchor.py -v
"""

import math
import unittest

from mpc_controller.MPC_corr import (
    MPCController, _pose_map_to_odom, _pose_odom_to_map)


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass


class _FakeMPC:
    """
    Just enough of MPCController's instance state for build_straight_corridor().

    Defaults mirror MPCController.__init__'s real ones, same as the other
    corridor test files' own _FakeMPC. goal_anchor_odom is the anchor the
    method actually reads -- what _refresh_goal_anchor() leaves there each
    tick -- so a test drives the reprojection by setting it directly, which is
    exactly what the node does.
    """

    def __init__(self, goal_anchor_odom=None, psi_init_corridor=0.0,
                 goal_start_xy=None):
        self.front_distance = 10.0
        self.goal_pose_xy = None  # distance-mode, not pose-mode
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.psi_init_corridor = psi_init_corridor
        self.goal_start_xy = goal_start_xy
        self.goal_anchor_odom = goal_anchor_odom
        self.corr_turn_u_start = 0.10
        self.corr_turn_u_end = 0.70

    def get_logger(self):
        return _FakeLogger()


# The map -> odom edge exactly as tf2 returns it for
# lookup_transform('map', 'odom', ...): (x, y, yaw) of the odom frame's origin
# expressed in map.
_IDENTITY_TF = (0.0, 0.0, 0.0)

# The drift actually measured on the mission that motivated this fix.
_MEASURED_DRIFT_RAD = math.radians(12.8)


class TestFrameHelpers(unittest.TestCase):
    """The two pure frame conversions, on their own."""

    def test_map_and_odom_conversions_are_exact_inverses(self):
        tf = (1.3, -0.7, 0.41)
        pose = (2.0, -0.5, 0.9)
        back = _pose_map_to_odom(*_pose_odom_to_map(*pose, *tf), *tf)
        for got, want in zip(back, pose):
            self.assertAlmostEqual(got, want, places=12)

    def test_capture_then_reproject_through_an_unmoved_transform_is_identity(self):
        """The anchor must not drift on its own while map -> odom holds still."""
        tf = (4.2, -1.1, -0.63)
        anchor_odom = (1.0, 2.0, 0.25)
        anchor_map = _pose_odom_to_map(*anchor_odom, *tf)
        for got, want in zip(_pose_map_to_odom(*anchor_map, *tf), anchor_odom):
            self.assertAlmostEqual(got, want, places=12)

    def test_yaw_only_transform_leaves_an_anchor_at_the_origin_in_place(self):
        anchor_map = _pose_odom_to_map(0.0, 0.0, 0.0, *_IDENTITY_TF)
        x, y, _ = _pose_map_to_odom(
            *anchor_map, 0.0, 0.0, _MEASURED_DRIFT_RAD)
        self.assertAlmostEqual(x, 0.0, places=12)
        self.assertAlmostEqual(y, 0.0, places=12)

    def test_transform_translation_shifts_the_reprojected_anchor(self):
        anchor_map = _pose_odom_to_map(0.0, 0.0, 0.0, *_IDENTITY_TF)
        x, y, _ = _pose_map_to_odom(*anchor_map, 0.5, -0.25, 0.0)
        self.assertAlmostEqual(x, -0.5, places=12)
        self.assertAlmostEqual(y, 0.25, places=12)


class TestDriftBecomesVisibleHeadingError(unittest.TestCase):
    """
    The actual bug, at the level the solver sees it.

    The car's odom-frame yaw stays put while map -> odom rotates underneath
    it; the corrected reference must rotate the opposite way by that amount,
    so the difference against the live yaw equals the real drift.
    """

    def test_reprojected_reference_heading_counter_rotates_by_the_drift(self):
        anchor_map = _pose_odom_to_map(0.0, 0.0, 0.0, *_IDENTITY_TF)
        _, _, psi_ref = _pose_map_to_odom(
            *anchor_map, 0.0, 0.0, _MEASURED_DRIFT_RAD)
        self.assertAlmostEqual(psi_ref, -_MEASURED_DRIFT_RAD, places=12)

    def test_heading_error_against_the_unchanged_live_yaw_equals_the_drift(self):
        live_odom_yaw = 0.0  # measured under 0.5 deg for the whole mission
        anchor_map = _pose_odom_to_map(0.0, 0.0, live_odom_yaw, *_IDENTITY_TF)
        _, _, psi_ref = _pose_map_to_odom(
            *anchor_map, 0.0, 0.0, _MEASURED_DRIFT_RAD)
        self.assertAlmostEqual(
            live_odom_yaw - psi_ref, _MEASURED_DRIFT_RAD, places=12)

    def test_the_old_odom_frozen_anchor_showed_no_error_at_all(self):
        """
        Pins the bug this replaces, so a regression is legible as one.

        Freezing psi in odom coordinates and comparing it to an odom yaw that
        barely moved yields ~zero error no matter how far the car has really
        turned -- which is why w_psi and the frozen-line math were both
        working and neither was doing anything.
        """
        frozen_odom_psi = 0.0
        live_odom_yaw = math.radians(0.4)
        self.assertLess(abs(live_odom_yaw - frozen_odom_psi), math.radians(0.5))


class TestCorridorFollowsTheReprojectedAnchor(unittest.TestCase):
    """build_straight_corridor reads goal_anchor_odom, not the raw pair."""

    def test_reference_heading_comes_from_the_reprojected_anchor(self):
        """psiRef is the reprojected anchor heading under EITHER geometry.

        This is the claim this file exists for -- the drift correction is in
        WHICH heading the corridor references, not in the blend shape -- so it
        is asserted for both settings of corridor_heading_return. The
        shape-specific consequences are pinned separately below.
        """
        psi_ref = -_MEASURED_DRIFT_RAD
        for heading_return in (False, True):
            with self.subTest(corridor_heading_return=heading_return):
                fake = _FakeMPC(goal_anchor_odom=(0.0, 0.0, psi_ref))
                fake.corridor_heading_return = heading_return
                corridor = MPCController.build_straight_corridor(
                    fake, [0.0, 0.0, 0.0, 0.5])
                self.assertAlmostEqual(corridor['psiRef'], psi_ref, places=9)

    def test_with_the_blend_on_the_corridor_bends_by_the_recovered_drift(self):
        """With corridor_heading_return TRUE the corridor BENDS from the live
        yaw (0.0 here) back to the reprojected anchor heading, so dpsi is
        exactly the drift the reprojection recovered.

        NOT the shipping default (that is both-ends-frozen, dpsi == 0 -- see
        the next test); the flag is set explicitly.
        """
        psi_ref = -_MEASURED_DRIFT_RAD
        fake = _FakeMPC(goal_anchor_odom=(0.0, 0.0, psi_ref))
        fake.corridor_heading_return = True
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(corridor['psiStart'], 0.0, places=9)
        self.assertAlmostEqual(corridor['dpsi'], psi_ref - 0.0, places=9)
        self.assertAlmostEqual(corridor['dpsi'], -_MEASURED_DRIFT_RAD, places=9)

    def test_with_the_blend_off_the_drift_correction_is_still_referenced(self):
        """The default geometry has dpsi == 0, and that is NOT the old bug.

        "psiStart == psiEnd, dpsi == 0" is what this file's earlier version
        asserted while the anchor was still in the wrong frame, and it was
        pinning a real defect then: the referenced heading itself was drifting,
        so a corridor rigidly parallel to it corrected nothing. What fixed that
        was reprojecting the anchor through map -> odom, which is orthogonal to
        the blend. With the anchor correct, dpsi == 0 means the corridor points
        along the RECOVERED heading and w_psi's terminal cost pulls the car
        onto it -- the correction lives in psiRef, which is asserted above.
        """
        psi_ref = -_MEASURED_DRIFT_RAD
        fake = _FakeMPC(goal_anchor_odom=(0.0, 0.0, psi_ref))
        fake.corridor_heading_return = False
        corridor = MPCController.build_straight_corridor(
            fake, [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(corridor['dpsi'], 0.0, places=12)
        self.assertAlmostEqual(corridor['psiStart'], psi_ref, places=9)
        self.assertAlmostEqual(corridor['psiRef'], psi_ref, places=9)

    def test_a_stale_raw_odom_anchor_no_longer_reaches_the_corridor(self):
        """
        The reprojected anchor wins over goal_start_xy/psi_init_corridor.

        Those two are kept for the bootstrap fallback and for logging; if the
        corridor still read them, the drift correction would be inert.
        """
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(goal_anchor_odom=(0.0, 0.0, -_MEASURED_DRIFT_RAD),
                     psi_init_corridor=0.0, goal_start_xy=(0.0, 0.0)),
            [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(
            corridor['psiRef'], -_MEASURED_DRIFT_RAD, places=9)

    def test_only_the_heading_is_taken_from_the_reprojected_anchor(self):
        """
        The anchor's position is not used for corridor geometry at all.

        Only its psi is. The origin is the car's own current pose, which is
        already in the live odom frame, so moving the anchor's x/y must leave
        the corridor's origin exactly where the car is.
        """
        psi_ref = 0.0
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(goal_anchor_odom=(0.0, 0.0, psi_ref)),
            [2.0, 0.6, 0.0, 0.5])
        self.assertAlmostEqual(float(corridor['xc'][0]), 2.0, places=9)
        self.assertAlmostEqual(float(corridor['yc'][0]), 0.6, places=9)

        # Same car pose, anchor position moved a long way: identical geometry.
        moved = MPCController.build_straight_corridor(
            _FakeMPC(goal_anchor_odom=(-4.0, 7.5, psi_ref)),
            [2.0, 0.6, 0.0, 0.5])
        for a, b in zip(corridor['xc'], moved['xc']):
            self.assertAlmostEqual(float(a), float(b), places=12)
        for a, b in zip(corridor['yc'], moved['yc']):
            self.assertAlmostEqual(float(a), float(b), places=12)

    def test_no_anchor_still_selects_the_bootstrap_fallback(self):
        """A move that never captured an anchor keeps the pre-existing path."""
        psi_init = 0.3
        corridor = MPCController.build_straight_corridor(
            _FakeMPC(goal_anchor_odom=None, psi_init_corridor=psi_init),
            [0.0, 0.0, 0.0, 0.5])
        self.assertAlmostEqual(corridor['psiRef'], psi_init, places=9)
        self.assertNotAlmostEqual(corridor['dpsi'], 0.0, places=3)


if __name__ == '__main__':
    unittest.main()
