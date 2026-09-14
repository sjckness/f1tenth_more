"""Coverage for wall_distance.py -- the tracked-wall distance and the
phase-aware corridor correction.

TWO TEST CLASSES CARRY THE SAFETY ARGUMENT AND THE REST ARE HOUSEKEEPING.

TestSignOnBothSides is the one that catches a car steering into glass. A
correction that is internally consistent with a convention is worthless if
the convention itself is inverted, and an inverted convention is invisible to
any test that only ever puts the wall on one side. So every sign assertion
here is made twice, once per side, against the convention WallDistance.msg
documents. It still cannot catch the convention being backwards relative to
PHYSICAL left and right -- only Stage 2 of docs/bringup_checklist.md can, and
nothing downstream runs until it passes.

TestSilenceNeverYieldsCorridor is the second. Silence on /mpc/wall_track is
overloaded five ways (no wall_turn, /mpc/hold, no odom, wall_track_enable
false, dead node) and a terminal wall_turn has no exit edge at all -- it just
goes quiet. A consumer that reads silence as "turn finished" fires on a
safety hold mid-turn. Entering corridor-following must require a positive
signal, and every other route to silence must land in UNKNOWN.

The real-glass fixtures are test/data/glass_corridor_ust10lx.npz: five scans
captured 2026-09-14 with the car 1.10 m from a clear pane. They are real at
1.1 m and at no other range. The odometry paired with them here is SYNTHETIC
-- which is exactly the thing that is wrong about testing coast behaviour
offline, and is why the coast caps are marked UNVALIDATED in
f1tenth_perception/README.md.

Run standalone: python3 -m pytest test/test_wall_distance.py -v
"""

import math
import pathlib
import unittest

import numpy as np

from f1tenth_perception.glass_detect import DetectorConfig, GlassTracker, detect
from f1tenth_perception.wall_distance import (
    PHASE_COMMITTED,
    PHASE_CORRIDOR,
    PHASE_NAMES,
    PHASE_UNKNOWN,
    PHASE_WALL_TURN,
    PROVENANCE_GEOMETRIC,
    PROVENANCE_GLASS_CONFIRMED,
    PROVENANCE_NAMES,
    PROVENANCE_NONE,
    PROVENANCE_PREDICTED,
    CorrectionConfig,
    Odometer,
    PhaseMachine,
    WallDistanceTracker,
    deadband_for,
    gate_margin,
    psi_correction,
    signed_wall_offset,
    soft_fade,
)

# This rig's geometry, matching test_glass_detect.py's -- measured off live
# /scan, not invented.
N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(0.25)
RANGE_MIN = 0.02
RANGE_MAX = 30.0
MISS = 65.533
LASER = (0.12, 0.0, 0.0)

DATA = pathlib.Path(__file__).parent / 'data' / 'glass_corridor_ust10lx.npz'


def tracker(**over):
    """A WallDistanceTracker on the shipping defaults, overridable per test."""
    kwargs = dict(
        glass_tracker=GlassTracker(
            persistence_window=8, persistence_hits=1,
            match_endpoint_tol=0.20, match_angle_tol_deg=10.0,
            expect_return_tol_deg=60.0, fov_half_angle_rad=math.pi / 2,
            max_range_m=5.0, geometry_only_multiplier=1),
        max_coast_distance=0.5,
        max_coast_yaw=0.35,
    )
    kwargs.update(over)
    return WallDistanceTracker(**kwargs)


def correction_cfg(**over):
    kwargs = dict(
        d_ref=0.60, convergence_length_m=3.0, max_psi_correction=0.20,
        max_psi_rate=0.5, deadband_floor=0.02, deadband_k=2.0,
        fade_start_age=0.3, fade_zero_age=1.0, stale_inflate_per_s=0.15)
    kwargs.update(over)
    return CorrectionConfig(**kwargs)


def wall_line(side, distance, yaw=0.0):
    """A straight wall parallel to the car's heading, `distance` metres to
    `side` ('left' or 'right'), as (nx, ny, c) in odom with the car at the
    origin pointing along `yaw`.

    Built from the geometry rather than from a fit so the expected sign is
    arithmetic, not something the fitter happened to produce.
    """
    # Left normal of the heading.
    lx, ly = -math.sin(yaw), math.cos(yaw)
    sign = 1.0 if side == 'left' else -1.0
    # A point on the wall, and the wall's own normal (which we deliberately
    # store pointing AWAY from the car on the left case and TOWARD it on the
    # right, to prove the result is invariant to the fit's arbitrary flip).
    px, py = sign * distance * lx, sign * distance * ly
    nx, ny = (lx, ly) if side == 'left' else (lx, ly)
    return nx, ny, nx * px + ny * py


class TestSignOnBothSides(unittest.TestCase):
    """The steer-into-the-wall test. Every assertion made once per side."""

    def test_a_wall_on_the_left_gives_positive_d_wall(self):
        nx, ny, c = wall_line('left', 0.8)
        self.assertAlmostEqual(signed_wall_offset(nx, ny, c, (0.0, 0.0, 0.0)), +0.8, places=9)

    def test_a_wall_on_the_right_gives_negative_d_wall(self):
        nx, ny, c = wall_line('right', 0.8)
        self.assertAlmostEqual(signed_wall_offset(nx, ny, c, (0.0, 0.0, 0.0)), -0.8, places=9)

    def test_the_sign_does_not_depend_on_the_fits_arbitrary_normal_flip(self):
        """A line fit gives the normal only up to a flip. Both spellings of
        the same wall must produce the same signed offset, or the sign is a
        property of the fitter rather than of the geometry."""
        for side, expect in (('left', +0.8), ('right', -0.8)):
            nx, ny, c = wall_line(side, 0.8)
            self.assertAlmostEqual(
                signed_wall_offset(nx, ny, c, (0.0, 0.0, 0.0)), expect, places=9, msg=side)
            self.assertAlmostEqual(
                signed_wall_offset(-nx, -ny, -c, (0.0, 0.0, 0.0)), expect, places=9, msg=side)

    def test_the_sign_holds_when_the_car_is_not_at_the_origin_or_axis_aligned(self):
        for side, expect in (('left', +0.8), ('right', -0.8)):
            for yaw in (0.3, 1.9, -2.7, math.pi):
                nx, ny, c = wall_line(side, 0.8, yaw=yaw)
                # Shift the wall and the car together: the offset is relative.
                got = signed_wall_offset(nx, ny, c + nx * 3.0 + ny * -4.0,
                                         (3.0, -4.0, yaw))
                self.assertAlmostEqual(got, expect, places=9, msg=f'{side} yaw={yaw}')

    def test_an_oblique_wall_reports_its_perpendicular_distance_not_a_projection(self):
        """The magnitude is the perpendicular distance to the LINE; the
        obliquity is reported separately as heading_rel. Projecting it onto the
        car's lateral axis instead would fold two quantities together and make
        d_wall shrink as the car turned, which is the defect d_wall exists to
        avoid in the first place (see wall_tracker.py on why dFront could not
        be used)."""
        from f1tenth_perception.wall_distance import wall_heading_rel
        # A wall 0.8 m to the left of a car yawed 0.4 rad off parallel to it.
        nx, ny, c = wall_line('left', 0.8, yaw=0.0)
        d = signed_wall_offset(nx, ny, c, (0.0, 0.0, 0.4))
        self.assertAlmostEqual(d, +0.8, places=9)
        self.assertAlmostEqual(wall_heading_rel(nx, ny, 0.4), 0.4, places=9)
        # Parallel to the wall: no obliquity at all.
        self.assertAlmostEqual(wall_heading_rel(nx, ny, 0.0), 0.0, places=9)

    def test_too_close_to_a_left_wall_steers_right_and_vice_versa(self):
        """The load-bearing assertion of the whole node. d_ref is 0.60."""
        cfg = correction_cfg()
        # Left wall, too close (0.40 < 0.60): steer AWAY, i.e. right, dpsi < 0.
        left_close = psi_correction(+0.40, cfg=cfg, fit_rms=0.0, age=0.0,
                                    prev_psi=0.0, dt=0.1, valid=True)
        self.assertLess(left_close, 0.0, 'too close on the left must steer right')
        # Right wall, too close (-0.40): steer AWAY, i.e. left, dpsi > 0.
        right_close = psi_correction(-0.40, cfg=cfg, fit_rms=0.0, age=0.0,
                                     prev_psi=0.0, dt=0.1, valid=True)
        self.assertGreater(right_close, 0.0, 'too close on the right must steer left')
        # And the two are mirror images, or one side is being handled specially.
        self.assertAlmostEqual(left_close, -right_close, places=12)

    def test_too_far_from_a_left_wall_steers_left_and_vice_versa(self):
        cfg = correction_cfg()
        left_far = psi_correction(+0.80, cfg=cfg, fit_rms=0.0, age=0.0,
                                  prev_psi=0.0, dt=0.1, valid=True)
        self.assertGreater(left_far, 0.0, 'too far from the left wall must steer left')
        right_far = psi_correction(-0.80, cfg=cfg, fit_rms=0.0, age=0.0,
                                   prev_psi=0.0, dt=0.1, valid=True)
        self.assertLess(right_far, 0.0, 'too far from the right wall must steer right')
        self.assertAlmostEqual(left_far, -right_far, places=12)

    def test_at_the_reference_distance_there_is_no_correction_on_either_side(self):
        cfg = correction_cfg()
        for d in (+0.60, -0.60):
            self.assertAlmostEqual(
                psi_correction(d, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                               dt=0.1, valid=True), 0.0, places=12, msg=str(d))

    def test_the_gain_is_one_over_the_convergence_length_on_both_sides(self):
        """k is DERIVED, never tuned: 3.0 m of convergence length is
        k = 0.333 rad/m. A test that let the gain drift would let the one
        parameter with physical meaning stop meaning it."""
        cfg = correction_cfg(convergence_length_m=3.0, deadband_floor=0.0, deadband_k=0.0,
                             max_psi_correction=10.0, max_psi_rate=1e9)
        for sign in (+1.0, -1.0):
            # 0.30 m of error at k = 1/3 is 0.10 rad.
            d = sign * (0.60 + sign * 0.0) + sign * 0.30
            got = psi_correction(d, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                                 dt=1.0, valid=True)
            self.assertAlmostEqual(abs(got), 0.30 / 3.0, places=9, msg=str(sign))
            self.assertEqual(math.copysign(1.0, got), sign, 'too far => toward the wall')

    def test_saturation_clamps_both_directions(self):
        cfg = correction_cfg(max_psi_correction=0.20, max_psi_rate=1e9)
        self.assertAlmostEqual(
            psi_correction(+9.0, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                           dt=1.0, valid=True), +0.20, places=12)
        self.assertAlmostEqual(
            psi_correction(-9.0, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                           dt=1.0, valid=True), -0.20, places=12)


class TestSilenceNeverYieldsCorridor(unittest.TestCase):
    """The four routes the prompt names, plus the ones that bite in practice."""

    def machine(self, silence_ticks=3):
        return PhaseMachine(silence_ticks=silence_ticks)

    def test_startup_is_unknown_before_anything_arrives(self):
        """Volatile QoS means a late subscriber sees nothing until the next
        turn, so UNKNOWN is the startup state as well as the failure state."""
        m = self.machine()
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_silence_from_startup_stays_unknown_forever(self):
        m = self.machine()
        for _ in range(50):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_wall_turn_then_abrupt_silence_lands_in_unknown(self):
        """Route 2. The turn never committed, so there is no corridor to
        follow -- whatever the track looks like."""
        m = self.machine()
        m.tick(math.nan, track_valid=True)
        self.assertEqual(m.phase, PHASE_WALL_TURN)
        for _ in range(3):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_committed_then_silence_with_a_valid_track_is_the_only_path_to_corridor(self):
        """Route 3 -- and the ONLY route."""
        m = self.machine()
        m.tick(math.nan, track_valid=True)
        for _ in range(5):
            m.tick(0.75, track_valid=True)
        self.assertEqual(m.phase, PHASE_COMMITTED)
        for _ in range(3):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_CORRIDOR)

    def test_committed_then_silence_with_an_invalid_track_lands_in_unknown(self):
        """Route 4. No wall means no reference, and no reference means the
        correction has nothing to be proportional to."""
        m = self.machine()
        m.tick(0.75, track_valid=True)
        for _ in range(3):
            m.tick(None, track_valid=False)
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_a_track_lost_mid_commit_and_regained_still_refuses_corridor(self):
        """'Held throughout' means throughout, not 'valid at the last tick'.
        A track that dropped out during the commit is a track whose geometry
        may have re-associated to a different surface."""
        m = self.machine()
        m.tick(0.75, track_valid=True)
        m.tick(0.75, track_valid=False)
        m.tick(0.75, track_valid=True)
        for _ in range(3):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_silence_shorter_than_the_tick_budget_does_not_transition(self):
        """One dropped message on a reliable topic is not a turn ending."""
        m = self.machine(silence_ticks=3)
        m.tick(0.75, track_valid=True)
        m.tick(None, track_valid=True)
        m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_COMMITTED)
        m.tick(0.75, track_valid=True)
        self.assertEqual(m.phase, PHASE_COMMITTED)
        for _ in range(3):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_CORRIDOR)

    def test_a_hold_mid_turn_does_not_become_corridor_following(self):
        """/mpc/hold makes control_loop return BEFORE _wall_track_tick, so a
        safety hold mid-turn is indistinguishable from a turn ending -- except
        that a held car has not finished rotating. With the track still valid
        this is the one genuinely ambiguous case, and the design accepts
        entering CORRIDOR here: a held car is stationary, so a corridor
        correction on a stopped car is inert. What must NOT happen is entering
        CORRIDOR on a hold that ALSO lost the track, which is the realistic
        shape of a hold (the e-stop fires on something in the way)."""
        m = self.machine()
        m.tick(0.75, track_valid=True)
        for _ in range(4):
            m.tick(None, track_valid=False)
        self.assertEqual(m.phase, PHASE_UNKNOWN)

    def test_a_new_turn_after_corridor_re_enters_the_turn_phases(self):
        m = self.machine()
        m.tick(0.75, track_valid=True)
        for _ in range(3):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_CORRIDOR)
        m.tick(math.nan, track_valid=True)
        self.assertEqual(m.phase, PHASE_WALL_TURN)

    def test_corridor_survives_ongoing_silence(self):
        """Silence is CORRIDOR's normal state -- it is what the phase means."""
        m = self.machine()
        m.tick(0.75, track_valid=True)
        for _ in range(3):
            m.tick(None, track_valid=True)
        for _ in range(100):
            m.tick(None, track_valid=True)
        self.assertEqual(m.phase, PHASE_CORRIDOR)

    def test_every_transition_is_reported_with_a_reason(self):
        """These log lines are the first thing anyone debugs."""
        m = self.machine()
        transitions = []
        for psi, valid in [(math.nan, True), (0.75, True), (None, True),
                           (None, True), (None, True)]:
            t = m.tick(psi, track_valid=valid)
            if t is not None:
                transitions.append(t)
        self.assertEqual([(t.old, t.new) for t in transitions],
                         [(PHASE_UNKNOWN, PHASE_WALL_TURN),
                          (PHASE_WALL_TURN, PHASE_COMMITTED),
                          (PHASE_COMMITTED, PHASE_CORRIDOR)])
        for t in transitions:
            self.assertTrue(t.reason, 'a transition with no reason is a dead log line')

    def test_unknown_is_the_phase_that_produces_nothing(self):
        cfg = correction_cfg()
        self.assertAlmostEqual(
            psi_correction(0.40, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.15,
                           dt=0.1, valid=False), 0.0, places=12)


class TestStaleAsymmetry(unittest.TestCase):
    """Same input, opposite direction. The next reader will assume the
    conservative direction is the same for both; it is not."""

    def test_the_soft_correction_fades_to_zero_with_age(self):
        self.assertAlmostEqual(soft_fade(0.0, 0.3, 1.0), 1.0, places=12)
        self.assertAlmostEqual(soft_fade(0.3, 0.3, 1.0), 1.0, places=12)
        self.assertAlmostEqual(soft_fade(0.65, 0.3, 1.0), 0.5, places=12)
        self.assertAlmostEqual(soft_fade(1.0, 0.3, 1.0), 0.0, places=12)
        self.assertAlmostEqual(soft_fade(9.0, 0.3, 1.0), 0.0, places=12)

    def test_the_gate_margin_inflates_with_age(self):
        self.assertAlmostEqual(gate_margin(0.0, 0.15), 0.0, places=12)
        self.assertAlmostEqual(gate_margin(1.0, 0.15), 0.15, places=12)
        self.assertAlmostEqual(gate_margin(4.0, 0.15), 0.60, places=12)

    def test_they_move_in_opposite_directions_over_the_same_ages(self):
        ages = [0.0, 0.25, 0.5, 0.75, 1.0]
        fades = [soft_fade(a, 0.3, 1.0) for a in ages]
        margins = [gate_margin(a, 0.15) for a in ages]
        self.assertEqual(fades, sorted(fades, reverse=True), 'soft must not grow')
        self.assertEqual(margins, sorted(margins), 'gate must not shrink')
        self.assertGreater(fades[0], fades[-1])
        self.assertLess(margins[0], margins[-1])

    def test_the_correction_actually_fades_with_age_on_both_sides(self):
        cfg = correction_cfg(max_psi_rate=1e9)
        for d in (+0.30, -0.30):
            fresh = psi_correction(d, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                                   dt=1.0, valid=True)
            half = psi_correction(d, cfg=cfg, fit_rms=0.0, age=0.65, prev_psi=0.0,
                                  dt=1.0, valid=True)
            gone = psi_correction(d, cfg=cfg, fit_rms=0.0, age=1.0, prev_psi=0.0,
                                  dt=1.0, valid=True)
            self.assertAlmostEqual(half, fresh * 0.5, places=9, msg=str(d))
            self.assertAlmostEqual(gone, 0.0, places=12, msg=str(d))


class TestDeadbandScalesWithFitQuality(unittest.TestCase):
    """Same geometric error, larger rms, smaller correction. A fixed deadband
    would steer on noise exactly when the estimate is worst."""

    def test_the_deadband_is_the_larger_of_the_floor_and_the_rms_multiple(self):
        self.assertAlmostEqual(deadband_for(0.000, 0.02, 2.0), 0.02, places=12)
        self.assertAlmostEqual(deadband_for(0.009, 0.02, 2.0), 0.02, places=12)
        self.assertAlmostEqual(deadband_for(0.050, 0.02, 2.0), 0.10, places=12)

    def test_a_worse_fit_shrinks_the_correction_for_the_same_error(self):
        # 0.90 rather than 0.30 so the correction is positive and "shrinks"
        # means the same thing numerically as it does in magnitude.
        cfg = correction_cfg(max_psi_rate=1e9)
        good = psi_correction(0.90, cfg=cfg, fit_rms=0.018, age=0.0, prev_psi=0.0,
                              dt=1.0, valid=True)
        poor = psi_correction(0.90, cfg=cfg, fit_rms=0.100, age=0.0, prev_psi=0.0,
                              dt=1.0, valid=True)
        self.assertGreater(good, poor)
        self.assertGreater(poor, 0.0, 'a poor fit still corrects a large error')

    def test_a_bad_enough_fit_swallows_the_error_entirely(self):
        cfg = correction_cfg(max_psi_rate=1e9)
        self.assertAlmostEqual(
            psi_correction(0.70, cfg=cfg, fit_rms=0.400, age=0.0, prev_psi=0.0,
                           dt=1.0, valid=True), 0.0, places=12)

    def test_the_deadband_applies_to_both_sides_identically(self):
        cfg = correction_cfg(max_psi_rate=1e9)
        for rms in (0.0, 0.02, 0.05):
            a = psi_correction(+0.75, cfg=cfg, fit_rms=rms, age=0.0, prev_psi=0.0,
                               dt=1.0, valid=True)
            b = psi_correction(-0.75, cfg=cfg, fit_rms=rms, age=0.0, prev_psi=0.0,
                               dt=1.0, valid=True)
            self.assertAlmostEqual(a, -b, places=12, msg=str(rms))


class TestNoWallAtExitIsZeroNotLastKnown(unittest.TestCase):
    """The natural implementation holds the last value. That is the bug."""

    def test_losing_the_wall_snaps_the_correction_to_zero(self):
        cfg = correction_cfg(max_psi_rate=0.5)
        held = 0.18
        got = psi_correction(math.nan, cfg=cfg, fit_rms=0.0, age=0.0,
                             prev_psi=held, dt=0.1, valid=False)
        self.assertEqual(got, 0.0)

    def test_the_rate_limit_does_not_delay_removing_a_correction(self):
        """The rate limit exists to smooth corrections, not to ration their
        withdrawal. At 0.5 rad/s and dt 0.1 a rate-limited decay from 0.18
        would take four ticks to reach zero, and every one of those ticks is
        a confident heading toward a wall nobody can see."""
        cfg = correction_cfg(max_psi_rate=0.5)
        self.assertEqual(
            psi_correction(math.nan, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.18,
                           dt=0.1, valid=False), 0.0)

    def test_a_nan_d_wall_with_valid_true_is_still_zero(self):
        """Defence in depth: valid and d_wall must agree, and if they do not,
        the answer is zero rather than a NaN propagated into the corridor."""
        cfg = correction_cfg()
        self.assertEqual(
            psi_correction(math.nan, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.1,
                           dt=0.1, valid=True), 0.0)

    def test_an_infinite_d_wall_is_rejected_the_same_way(self):
        cfg = correction_cfg()
        self.assertEqual(
            psi_correction(math.inf, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.1,
                           dt=0.1, valid=True), 0.0)


class TestRateLimit(unittest.TestCase):

    def test_a_step_is_spread_over_ticks_at_the_rate_limit(self):
        cfg = correction_cfg(max_psi_rate=0.5, deadband_floor=0.0, deadband_k=0.0)
        # Already at the reference distance: nothing to spread.
        got = psi_correction(0.60, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                             dt=0.1, valid=True)
        self.assertAlmostEqual(got, 0.0, places=12)
        # From zero toward the +0.20 saturation, one tick buys 0.05 rad.
        got = psi_correction(9.0, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                             dt=0.1, valid=True)
        self.assertAlmostEqual(got, 0.05, places=12)

    def test_the_limit_is_symmetric(self):
        cfg = correction_cfg(max_psi_rate=0.5, deadband_floor=0.0, deadband_k=0.0)
        self.assertAlmostEqual(
            psi_correction(-9.0, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                           dt=0.1, valid=True), -0.05, places=12)

    def test_it_absorbs_a_track_id_change(self):
        """A re-association steps d_wall. The consumer is told (track_id
        changes) but the rate limit is what stops the step reaching the
        corridor as a step."""
        cfg = correction_cfg(max_psi_rate=0.5, deadband_floor=0.0, deadband_k=0.0)
        before = psi_correction(0.60, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=0.0,
                                dt=0.1, valid=True)
        after = psi_correction(2.60, cfg=cfg, fit_rms=0.0, age=0.0, prev_psi=before,
                               dt=0.1, valid=True)
        self.assertLessEqual(abs(after - before), 0.5 * 0.1 + 1e-12)


class TestOdometer(unittest.TestCase):
    """Coast caps are measured against PATH LENGTH, not displacement. During
    a 90 deg turn at R_min 1.07 m the arc is 1.68 m and the chord 1.51 m --
    an 11% under-read, in the optimistic direction, from a quantity already
    biased 17-20% optimistic."""

    def test_a_straight_run_accumulates_its_length(self):
        o = Odometer()
        for i in range(11):
            o.update((0.1 * i, 0.0, 0.0))
        self.assertAlmostEqual(o.s, 1.0, places=9)
        self.assertAlmostEqual(o.yaw_travel, 0.0, places=9)

    def test_an_arc_accumulates_more_than_its_chord(self):
        o = Odometer()
        r = 1.07
        for k in range(91):
            th = math.radians(k)
            o.update((r * math.sin(th), r * (1 - math.cos(th)), th))
        chord = math.hypot(r * math.sin(math.pi / 2), r * (1 - math.cos(math.pi / 2)))
        self.assertGreater(o.s, chord)
        self.assertAlmostEqual(o.s, r * math.pi / 2, places=2)
        self.assertAlmostEqual(o.yaw_travel, math.pi / 2, places=6)

    def test_yaw_travel_is_absolute_so_a_there_and_back_wobble_still_counts(self):
        o = Odometer()
        for yaw in (0.0, 0.2, 0.0, 0.2, 0.0):
            o.update((0.0, 0.0, yaw))
        self.assertAlmostEqual(o.yaw_travel, 0.8, places=9)

    def test_yaw_travel_does_not_jump_at_the_pi_seam(self):
        o = Odometer()
        o.update((0.0, 0.0, math.pi - 0.05))
        o.update((0.0, 0.0, -math.pi + 0.05))
        self.assertAlmostEqual(o.yaw_travel, 0.1, places=9)


class TestConstantsMirrorTheMessage(unittest.TestCase):
    """wall_distance.py's constants and WallDistance.msg's must not drift --
    the node casts one straight onto the other."""

    def _msg_constants(self, prefix):
        path = (pathlib.Path(__file__).parents[2] / 'f1tenth_messages' / 'msg'
                / 'WallDistance.msg')
        out = {}
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line.startswith('uint8 ') or '=' not in line:
                continue
            name, value = line.split(None, 1)[1].split('=')
            if name.strip().startswith(prefix):
                out[name.strip()] = int(value.split('#')[0])
        return out

    def test_provenance_constants_match(self):
        self.assertEqual(self._msg_constants('PROVENANCE_'), {
            'PROVENANCE_NONE': PROVENANCE_NONE,
            'PROVENANCE_GEOMETRIC': PROVENANCE_GEOMETRIC,
            'PROVENANCE_GLASS_CONFIRMED': PROVENANCE_GLASS_CONFIRMED,
            'PROVENANCE_PREDICTED': PROVENANCE_PREDICTED,
        })

    def test_phase_constants_match(self):
        self.assertEqual(self._msg_constants('PHASE_'), {
            'PHASE_UNKNOWN': PHASE_UNKNOWN,
            'PHASE_WALL_TURN': PHASE_WALL_TURN,
            'PHASE_COMMITTED': PHASE_COMMITTED,
            'PHASE_CORRIDOR': PHASE_CORRIDOR,
        })

    def test_every_constant_has_a_name_for_the_logs(self):
        self.assertEqual(set(PHASE_NAMES), {PHASE_UNKNOWN, PHASE_WALL_TURN,
                                            PHASE_COMMITTED, PHASE_CORRIDOR})
        self.assertEqual(set(PROVENANCE_NAMES),
                         {PROVENANCE_NONE, PROVENANCE_GEOMETRIC,
                          PROVENANCE_GLASS_CONFIRMED, PROVENANCE_PREDICTED})


@unittest.skipUnless(DATA.exists(), f'fixture {DATA} missing')
class TestRealGlassThroughATurn(unittest.TestCase):
    """The five real scans, with SYNTHETIC odometry describing a 90 degree
    turn and the beams falling off the wall midway.

    The odometry is the weak part and it is the part being tested, which is
    the honest limitation of doing this offline. See
    f1tenth_perception/README.md's UNVALIDATED section.
    """

    @classmethod
    def setUpClass(cls):
        d = np.load(DATA)
        cls.ranges = d['ranges']
        cls.angle_min = float(d['angle_min'])
        cls.angle_inc = float(d['angle_increment'])
        cls.range_min = float(d['range_min'])
        cls.range_max = float(d['range_max'])
        cls.intensities = d['intensities']

    def _cfg(self):
        return DetectorConfig()

    def _detect(self, idx, car_pose):
        return detect(self.ranges[idx], self.angle_min, self.angle_inc,
                      self.range_min, self.range_max, LASER, car_pose,
                      self._cfg(), intensities=self.intensities[idx])

    def _turn_poses(self, n, total_yaw=math.pi / 2, radius=1.07):
        """n poses along an arc of `total_yaw` at `radius`, starting at the
        origin pointing along +x."""
        out = []
        for k in range(n):
            th = total_yaw * k / max(n - 1, 1)
            out.append((radius * math.sin(th), radius * (1 - math.cos(th)), th))
        return out

    def test_the_fixtures_produce_a_track_at_all(self):
        """If this fails the rest of the class is meaningless."""
        cands, _ = self._detect(0, (0.0, 0.0, 0.0))
        self.assertTrue([c for c in cands if c.accepted],
                        'the real-glass fixture must yield an accepted candidate')

    def test_track_id_is_stable_while_the_same_pane_is_re_observed(self):
        t = tracker()
        car = (0.0, 0.0, 0.0)
        ids = []
        for i in range(len(self.ranges)):
            cands, used = self._detect(i, car)
            t.odometer.update(car)
            t.update(cands, car, now=0.1 * i, use_intensity=used)
            obs = t.observation(car, now=0.1 * i)
            if obs.valid:
                ids.append(obs.track_id)
        self.assertTrue(ids, 'no valid observation across five real scans')
        self.assertEqual(len(set(ids)), 1, f'track_id changed across scans: {ids}')

    def test_provenance_degrades_to_predicted_when_the_beams_fall_off(self):
        """Through the turn the pane sweeps into incidence angles where glass
        returns nothing. Absence is the normal state, not evidence the wall
        moved -- so the track must survive with provenance PREDICTED."""
        t = tracker()
        poses = self._turn_poses(12, total_yaw=0.30, radius=1.07)
        first_id = None
        provenances = []
        for k, car in enumerate(poses):
            t.odometer.update(car)
            if k < 5:
                cands, used = self._detect(k, car)
                t.update(cands, car, now=0.1 * k, use_intensity=used)
            else:
                # Beams off the wall: no candidates at all, which is exactly
                # what a pane at 60+ deg of incidence produces.
                t.update([], car, now=0.1 * k, use_intensity=True)
            obs = t.observation(car, now=0.1 * k)
            if obs.valid and first_id is None:
                first_id = obs.track_id
            provenances.append(obs.provenance)
        self.assertIsNotNone(first_id, 'never acquired a track')
        self.assertIn(PROVENANCE_PREDICTED, provenances,
                      f'never coasted: {[PROVENANCE_NAMES[p] for p in provenances]}')
        # And the identity held across the whole thing.
        t2_ids = {p for p in provenances if p != PROVENANCE_NONE}
        self.assertTrue(t2_ids)

    def test_the_track_keeps_its_id_across_the_coast(self):
        t = tracker(max_coast_distance=5.0, max_coast_yaw=5.0)
        poses = self._turn_poses(12, total_yaw=0.30, radius=1.07)
        ids = []
        for k, car in enumerate(poses):
            t.odometer.update(car)
            if k < 5:
                cands, used = self._detect(k, car)
                t.update(cands, car, now=0.1 * k, use_intensity=used)
            else:
                t.update([], car, now=0.1 * k, use_intensity=True)
            obs = t.observation(car, now=0.1 * k)
            if obs.valid:
                ids.append(obs.track_id)
        self.assertTrue(ids)
        self.assertEqual(len(set(ids)), 1, f'identity broke across the coast: {ids}')


@unittest.skipUnless(DATA.exists(), f'fixture {DATA} missing')
class TestCoastCap(unittest.TestCase):
    """Past the cap, valid goes false rather than the track continuing on
    stale prediction. The caps are set by the MEASURED odometry bias
    (distance 17-20% short, gyro yaw gain 0.93-1.09), not by a fit."""

    @classmethod
    def setUpClass(cls):
        d = np.load(DATA)
        cls.ranges = d['ranges']
        cls.args = (float(d['angle_min']), float(d['angle_increment']),
                    float(d['range_min']), float(d['range_max']))
        cls.intensities = d['intensities']

    def _seed(self, t, car):
        cands, used = detect(self.ranges[0], *self.args, LASER, car,
                             DetectorConfig(), intensities=self.intensities[0])
        t.odometer.update(car)
        t.update(cands, car, now=0.0, use_intensity=used)
        return t.observation(car, now=0.0)

    def test_a_long_straight_coast_invalidates_on_distance(self):
        t = tracker(max_coast_distance=0.5, max_coast_yaw=10.0)
        start = (0.0, 0.0, 0.0)
        self.assertTrue(self._seed(t, start).valid, 'fixture must seed a track')
        last = None
        for k in range(1, 41):
            car = (0.05 * k, 0.0, 0.0)
            t.odometer.update(car)
            t.update([], car, now=0.1 * k, use_intensity=True)
            last = t.observation(car, now=0.1 * k)
            if not last.valid:
                self.assertGreater(last.coast_distance, 0.5 - 1e-9)
                self.assertEqual(last.provenance, PROVENANCE_NONE)
                return
        self.fail(f'coasted 2.0 m without invalidating: {last}')

    def test_a_long_rotation_invalidates_on_yaw(self):
        t = tracker(max_coast_distance=10.0, max_coast_yaw=0.35)
        start = (0.0, 0.0, 0.0)
        self.assertTrue(self._seed(t, start).valid, 'fixture must seed a track')
        last = None
        for k in range(1, 41):
            car = (0.0, 0.0, 0.02 * k)
            t.odometer.update(car)
            t.update([], car, now=0.1 * k, use_intensity=True)
            last = t.observation(car, now=0.1 * k)
            if not last.valid:
                self.assertGreater(last.coast_yaw, 0.35 - 1e-9)
                self.assertEqual(last.provenance, PROVENANCE_NONE)
                return
        self.fail(f'rotated 0.8 rad without invalidating: {last}')

    def test_a_ninety_degree_turn_cannot_be_coasted_end_to_end(self):
        """Not a limitation to work around -- the design intent. At the
        measured gyro gain a 90 deg coast misorients the wall by ~6 deg, which
        is larger than any margin here."""
        t = tracker()
        start = (0.0, 0.0, 0.0)
        self.assertTrue(self._seed(t, start).valid)
        r = 1.07
        valid_at_end = True
        for k in range(1, 91):
            th = math.radians(k)
            car = (r * math.sin(th), r * (1 - math.cos(th)), th)
            t.odometer.update(car)
            t.update([], car, now=0.1 * k, use_intensity=True)
            valid_at_end = t.observation(car, now=0.1 * k).valid
        self.assertFalse(valid_at_end,
                         'a 90 deg coast on stale prediction must not stay valid')

    def test_a_fresh_observation_resets_the_coast(self):
        t = tracker(max_coast_distance=0.5, max_coast_yaw=10.0)
        car = (0.0, 0.0, 0.0)
        self.assertTrue(self._seed(t, car).valid)
        for k in range(1, 6):
            car = (0.05 * k, 0.0, 0.0)
            t.odometer.update(car)
            t.update([], car, now=0.1 * k, use_intensity=True)
        mid = t.observation(car, now=0.5)
        self.assertGreater(mid.coast_distance, 0.0)
        # Re-observe from the same place the fixture was captured at.
        cands, used = detect(self.ranges[0], *self.args, LASER, (0.0, 0.0, 0.0),
                             DetectorConfig(), intensities=self.intensities[0])
        t.update(cands, (0.0, 0.0, 0.0), now=0.6, use_intensity=used)
        after = t.observation((0.0, 0.0, 0.0), now=0.6)
        self.assertAlmostEqual(after.coast_distance, 0.0, places=9)
        self.assertAlmostEqual(after.age, 0.0, places=9)

    def test_the_cap_warns_once_per_coast_not_once_per_tick(self):
        t = tracker(max_coast_distance=0.2, max_coast_yaw=10.0)
        self.assertTrue(self._seed(t, (0.0, 0.0, 0.0)).valid)
        warns = 0
        for k in range(1, 31):
            car = (0.05 * k, 0.0, 0.0)
            t.odometer.update(car)
            t.update([], car, now=0.1 * k, use_intensity=True)
            obs = t.observation(car, now=0.1 * k)
            warns += 1 if obs.coast_cap_first_hit else 0
        self.assertEqual(warns, 1, 'the cap warning must not repeat every tick')


class TestSweptArcUnits(unittest.TestCase):
    """swept_corridor.clearance() returns rear-axle ARC LENGTH to body
    contact, not a perpendicular gap. Anything reading it as a
    distance-to-wall is wrong, and this asserts the two genuinely differ so
    nobody 'simplifies' one into the other."""

    def test_arc_length_to_a_wall_ahead_is_the_gap_to_the_bumper_not_to_the_axle(self):
        from f1tenth_perception.swept_corridor import BODY_FRONT_X_M, clearance
        points = np.array([[3.0, 0.0]])
        got = clearance(points, 0.0, wheelbase=0.305, half_width=0.136, margin=0.1,
                        max_range=5.0, absolute_min_clearance=0.15)
        self.assertAlmostEqual(got, 3.0 - BODY_FRONT_X_M, places=6)
        self.assertNotAlmostEqual(got, 3.0, places=2)

    def test_a_wall_abeam_is_not_an_arc_length_at_all(self):
        """A wall 0.6 m to the left is 0.6 m away and infinitely far along a
        straight arc. Reading swept clearance as d_wall would report ~5 m of
        room beside a wall the car is 0.6 m from."""
        from f1tenth_perception.swept_corridor import clearance
        points = np.array([[0.0, 0.6]])
        got = clearance(points, 0.0, wheelbase=0.305, half_width=0.136, margin=0.1,
                        max_range=5.0, absolute_min_clearance=0.15)
        self.assertAlmostEqual(got, 5.0, places=6)


if __name__ == '__main__':
    unittest.main()
