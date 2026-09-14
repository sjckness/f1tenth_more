"""Coverage for glass_detect.py -- the glass wall/door detector.

THE FIRST TEST CLASS IS THE ONE THAT MATTERS. TestSurvivesRaysPassingThrough
is the clearing regression: a confirmed pane must stay confirmed through scan
after scan whose rays pass straight through it. That is the entire point of
the design -- a standard obstacle layer marks a pane on the rare beam that
sees it and then clears it on every beam that does not, so the pane can never
persist. If that class fails, nothing else here is worth anything.

The synthetic fixtures reproduce the four cases the work order calls for
(plain wall, open doorway, pane with a spike, retroreflective strip) on this
rig's real scan geometry: 1081 beams over 270 deg at 0.25 deg, range_max
30.0, and A MISS REPORTED AS 65.533 m RATHER THAN inf -- which is what this
sensor actually does, and what a detector keyed on isinf() would miss
entirely.

test_real_glass.py's fixture is not synthetic: test/data/
glass_corridor_ust10lx.npz holds five scans captured 2026-09-14 with the car
1.10 m (perpendicular, laser frame) from a clear glass pane, a 4 m corridor
behind it and a second pane beyond that. See glass_detect.py's module
docstring for the full measurement.

Run standalone: python3 -m pytest test/test_glass_detect.py -v
"""

import math
import pathlib
import unittest

import numpy as np

from f1tenth_perception.glass_detect import (
    DetectorConfig,
    EVIDENCE_SEE_THROUGH,
    EVIDENCE_SPIKE,
    EVIDENCE_WIDE,
    GlassTracker,
    REASON_NAMES,
    VOID_FAR,
    VOID_NO_RETURN,
    contiguous_extent,
    detect,
    find_voids,
    intensity_usable,
    no_return_mask,
    normalize_intensity,
    sample_segment,
)

# This rig's geometry. NOT invented: measured off live /scan.
N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(0.25)
RANGE_MIN = 0.02
RANGE_MAX = 30.0
# What a miss actually reports on a UST-10LX through urg_node: 65535 mm.
MISS = 65.533
LASER = (0.12, 0.0, 0.0)
CAR = (0.0, 0.0, 0.0)

DATA = pathlib.Path(__file__).parent / 'data' / 'glass_corridor_ust10lx.npz'


def beam_at(deg):
    """Index of the beam pointing `deg` from straight ahead."""
    return int(round((math.radians(deg) - ANGLE_MIN) / ANGLE_INC))


def flat_wall(distance, half_deg=70.0, miss_outside=True):
    """A flat wall perpendicular to straight ahead at `distance` from the
    laser: range = distance / cos(bearing) over +-half_deg."""
    ranges = np.full(N_BEAMS, MISS if miss_outside else RANGE_MAX - 1.0)
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC
    inside = np.abs(angles) <= math.radians(half_deg)
    ranges[inside] = distance / np.cos(angles[inside])
    return ranges


def punch(ranges, lo_deg, hi_deg, value):
    """Replace a bearing band with `value` (MISS, or a farther surface)."""
    out = ranges.copy()
    out[beam_at(lo_deg):beam_at(hi_deg) + 1] = value
    return out


def far_surface(ranges, lo_deg, hi_deg, distance):
    """Make a band see through to a second wall `distance` from the laser."""
    out = ranges.copy()
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC
    sl = slice(beam_at(lo_deg), beam_at(hi_deg) + 1)
    out[sl] = distance / np.cos(angles[sl])
    return out


def run(ranges, cfg=None, intensities=None):
    cfg = cfg or DetectorConfig()
    return detect(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MIN, RANGE_MAX,
                  LASER, CAR, cfg, intensities=intensities)


def accepted(ranges, cfg=None, intensities=None):
    cands, _used = run(ranges, cfg, intensities)
    return [c for c in cands if c.accepted]


# ===========================================================================
# THE TEST THAT PROVES THE DESIGN. Read this first.
# ===========================================================================

class TestSurvivesRaysPassingThrough(unittest.TestCase):
    """A confirmed pane must not be forgotten by scans that see through it.

    This is the clearing regression of the work order's Section 8.3, at the
    level where this stack can actually assert it: the tracker decides what
    is published, and a costmap marking-only source cannot clear what is
    still being published. If the tracker drops the pane, the costmap never
    hears about it again and the hole opens regardless of clearing policy.
    """

    def setUp(self):
        self.tracker = GlassTracker(
            persistence_window=20, persistence_hits=4, match_endpoint_tol=0.2,
            match_angle_tol_deg=10.0, expect_return_tol_deg=2.0,
            fov_half_angle_rad=math.radians(135.0), max_range_m=10.0)
        # A pane 1.2 m ahead, seen with a see-through band: the real geometry.
        self.ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        self.cands = [c for c in run(self.ranges)[0] if c.accepted]
        self.assertTrue(self.cands, 'fixture must produce a detection to track')

    def _confirm(self):
        for i in range(8):
            self.tracker.update(self.cands, CAR, float(i), use_intensity=False)
        confirmed = self.tracker.update(self.cands, CAR, 9.0, use_intensity=False)
        self.assertTrue(confirmed, 'pane should be confirmed after repeat sightings')
        return confirmed

    def test_a_pane_stays_confirmed_through_hundreds_of_transparent_scans(self):
        self._confirm()
        # Now every subsequent scan sees nothing: the rays pass through. This
        # is the normal state for glass, and it must not retract the pane.
        for i in range(300):
            confirmed = self.tracker.update([], CAR, 10.0 + i, use_intensity=False)
            self.assertTrue(
                confirmed,
                f'pane was dropped after {i} transparent scans -- this is the '
                f'clearing bug the whole design exists to prevent')

    def test_absence_alone_never_adds_a_miss(self):
        """Absence is the normal state for glass, so it must not even be
        recorded as evidence against a track."""
        self._confirm()
        track = self.tracker.tracks[0]
        before = track.misses
        # Car rotated so the pane is far off normal incidence: a miss here
        # carries no information at all.
        for i in range(50):
            self.tracker.update([], (0.0, 0.0, math.radians(80.0)), 100.0 + i,
                                use_intensity=False)
        self.assertEqual(track.misses, before,
                         'a miss was counted while the pane was not even at an '
                         'incidence where a return could be expected')

    def test_a_miss_counts_only_where_a_return_was_expected(self):
        """The one case a miss IS informative: square on, in range, in view.

        Needs its own fixture -- setUp's band is off to one side, so its
        segment midpoint sits ~23 deg off normal and a miss there carries no
        information. This one centres the band so the pane really is square
        on, which is the only geometry where silence is evidence."""
        centred = far_surface(flat_wall(1.2), -1.1, 1.1, 4.6)
        cands = [c for c in run(centred)[0] if c.accepted]
        self.assertTrue(cands, 'centred fixture must detect')
        tracker = GlassTracker(
            persistence_window=20, persistence_hits=4, match_endpoint_tol=0.2,
            match_angle_tol_deg=10.0, expect_return_tol_deg=2.0,
            fov_half_angle_rad=math.radians(135.0), max_range_m=10.0)
        for i in range(9):
            tracker.update(cands, CAR, float(i), use_intensity=False)
        track = tracker.tracks[0]
        before = track.misses
        for i in range(5):
            tracker.update([], CAR, 200.0 + i, use_intensity=False)
        self.assertGreater(track.misses, before)

    def test_a_square_on_pane_that_stops_returning_does_eventually_decay(self):
        """The deliberate counterpart to the test above it: silence square on
        is informative, so a pane that vanishes while the car stares straight
        at it must NOT be kept for ever. That is not the clearing bug -- the
        clearing bug is losing a pane to rays passing through it obliquely,
        which is the previous test and which must never happen."""
        centred = far_surface(flat_wall(1.2), -1.1, 1.1, 4.6)
        cands = [c for c in run(centred)[0] if c.accepted]
        tracker = GlassTracker(
            persistence_window=20, persistence_hits=4, match_endpoint_tol=0.2,
            match_angle_tol_deg=10.0, expect_return_tol_deg=2.0,
            fov_half_angle_rad=math.radians(135.0), max_range_m=10.0)
        for i in range(9):
            tracker.update(cands, CAR, float(i), use_intensity=False)
        self.assertTrue(tracker.tracks[0].confirmed(20, 8))
        for i in range(25):
            tracker.update([], CAR, 100.0 + i, use_intensity=False)
        self.assertFalse(tracker.tracks[0].confirmed(20, 8))

    def test_geometry_only_mode_demands_more_confirmations(self):
        self.assertEqual(self.tracker.required_hits(use_intensity=True), 4)
        self.assertEqual(self.tracker.required_hits(use_intensity=False), 8)


# ===========================================================================
# The four cases the work order names
# ===========================================================================

class TestPlainWall(unittest.TestCase):
    """Required case 1: a plain wall, no openings -- expect nothing."""

    def test_a_flat_wall_produces_no_voids_and_no_detections(self):
        ranges = flat_wall(1.5)
        voids = find_voids(ranges, no_return_mask(ranges, RANGE_MIN, RANGE_MAX),
                           DetectorConfig().min_void_beams,
                           DetectorConfig().void_range_jump)
        self.assertEqual(voids, [])
        self.assertEqual(accepted(ranges), [])

    def test_a_wall_at_several_distances_stays_silent(self):
        for d in (0.9, 1.5, 3.0, 6.0, 9.0):
            self.assertEqual(accepted(flat_wall(d)), [], f'wall at {d} m')


class TestOpenDoorway(unittest.TestCase):
    """Required case 2, THE CRITICAL FALSE POSITIVE. An open doorway has the
    same shape as a see-through band and must never be reported as glass:
    marking every doorway lethal strands the planner."""

    def test_a_doorway_onto_empty_space_is_not_glass(self):
        # 0.9 m opening at 1.2 m is ~43 deg, beams simply miss.
        ranges = punch(flat_wall(1.2), -21.0, 21.0, MISS)
        self.assertEqual(accepted(ranges), [])

    def test_a_doorway_onto_a_far_wall_is_not_glass(self):
        """The dangerous variant: the opening shows a real surface behind it,
        exactly like a pane transmitting. Width is what refuses it."""
        ranges = far_surface(flat_wall(1.2), -21.0, 21.0, 4.6)
        cands, _ = run(ranges)
        self.assertEqual([c for c in cands if c.accepted], [])
        for c in cands:
            self.assertFalse(c.evidence & EVIDENCE_SEE_THROUGH,
                             'a 0.9 m opening was credited as transparency')

    def test_doorways_of_every_ordinary_width_are_refused(self):
        for half in (15.0, 21.0, 25.0, 30.0):
            ranges = far_surface(flat_wall(1.2), -half, half, 4.6)
            self.assertEqual(accepted(ranges), [],
                             f'opening +-{half} deg was accepted as glass')

    def test_a_wide_wall_with_an_opaque_stripe_is_wide_and_still_refused(self):
        """EVIDENCE_WIDE describes every wall that has any hole in it, so it
        must never be sufficient on its own.

        The stripe here absorbs rather than transmits (a dark or black band --
        common, and the reason min_inlier_count_ratio exists elsewhere in this
        package): it creates an opening with no returns behind it, so there is
        no transparency evidence of any kind. The wall is 2 m away and the
        support window is widened so the fitted extent genuinely exceeds
        geom_min_length_m -- see that key's note on why extent is bounded by
        support_window_beams at close range."""
        cfg = DetectorConfig()
        cfg.support_window_beams = 200
        ranges = punch(flat_wall(2.0), -1.1, 1.1, MISS)
        cands, _ = run(ranges, cfg)
        wide = [c for c in cands if c.evidence & EVIDENCE_WIDE]
        self.assertTrue(wide, 'fixture should produce a partition-scale line')
        for c in wide:
            self.assertFalse(c.accepted)
            self.assertEqual(REASON_NAMES[c.reason], 'no_evidence')


class TestGlassWithSeeThroughBand(unittest.TestCase):
    """Required case 3, in the form this sensor actually delivers it: a
    narrow band transmitting to a second surface. Measured on real glass."""

    def test_a_narrow_see_through_band_is_detected(self):
        ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        got = accepted(ranges)
        self.assertEqual(len(got), 1)
        self.assertTrue(got[0].evidence & EVIDENCE_SEE_THROUGH)

    def test_the_fitted_line_sits_on_the_pane_not_the_far_surface(self):
        ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        got = accepted(ranges)[0]
        # The pane is 1.2 m from the laser, which is 0.12 m ahead of the
        # origin the points are expressed about.
        stand_off = abs(got.line.signed(*CAR[:2]))
        self.assertAlmostEqual(stand_off, 1.32, delta=0.05)
        self.assertLess(got.rms_m, 0.02)

    def test_several_bands_share_one_line_and_read_as_a_partition(self):
        """The real captured geometry: four transmitting bands at -26, -22,
        -16 and -9.5 deg. They must fold into ONE segment carrying several
        openings, not four separate two-point lines."""
        ranges = flat_wall(1.2)
        for lo, hi in [(-26.0, -25.0), (-22.3, -20.5), (-16.3, -15.7), (-9.8, -9.0)]:
            ranges = far_surface(ranges, lo, hi, 4.6)
        got = accepted(ranges)
        self.assertEqual(len(got), 1)
        self.assertGreaterEqual(got[0].n_voids, 2)
        self.assertTrue(got[0].evidence & EVIDENCE_SEE_THROUGH)
        # Wider than any single band's own support could reach.
        self.assertGreater(got[0].length_m, 0.9)

    def test_a_band_wider_than_the_gap_limit_stops_counting(self):
        cfg = DetectorConfig()
        narrow = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        self.assertTrue(accepted(narrow, cfg))
        # Same geometry, gap limit dropped below the measured 4 cm.
        cfg.see_through_max_gap_m = 0.01
        self.assertEqual(accepted(narrow, cfg), [])


class TestRetroreflectiveStrip(unittest.TestCase):
    """Required case 5: a bright contiguous patch is tape, not glass. The
    spike test must refuse it on width and gradient, not amplitude."""

    def test_a_bright_wide_patch_yields_no_spike_evidence(self):
        ranges = flat_wall(1.5)
        inten = np.full(N_BEAMS, 400.0)
        # 40 beams of tape: bright, contiguous, gently edged.
        inten[beam_at(-5.0):beam_at(5.0)] = 3000.0
        cands, used = run(ranges, intensities=inten)
        self.assertTrue(used, 'varying intensity should select the intensity path')
        for c in cands:
            self.assertFalse(c.evidence & EVIDENCE_SPIKE)
        self.assertEqual([c for c in cands if c.accepted], [])

    def test_a_narrow_spike_elsewhere_does_not_veto_a_real_pane(self):
        """Regression for a real bug, caught by the real-glass fixtures.

        An earlier version set the rejection reason whenever the scan held a
        spike that did not land on the candidate's own line. One narrow bright
        spot anywhere in the room therefore rejected EVERY genuine pane in the
        scan, and all four real-glass assertions failed the moment the
        intensity pipeline began finding spikes at all. A spike may only ADD
        evidence."""
        ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        # A small object 0.9 m away at 50 deg, well off the pane's plane, with
        # a spike narrow enough to survive max_spike_width.
        b = beam_at(50.0)
        ranges[b:b + 3] = 0.9
        inten = np.full(N_BEAMS, 300.0)
        inten[b:b + 3] = 5000.0
        got = accepted(ranges, intensities=inten)
        self.assertEqual(len(got), 1, 'a spike 50 deg away vetoed the pane')
        self.assertTrue(got[0].evidence & EVIDENCE_SEE_THROUGH)
        self.assertFalse(got[0].evidence & EVIDENCE_SPIKE)

    def test_tape_beside_a_real_pane_does_not_become_its_evidence(self):
        ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        inten = np.full(N_BEAMS, 400.0)
        inten[beam_at(30.0):beam_at(40.0)] = 3000.0
        got = accepted(ranges, intensities=inten)
        self.assertEqual(len(got), 1)
        self.assertFalse(got[0].evidence & EVIDENCE_SPIKE,
                         'a retroreflector 30 deg away was credited to the pane')
        self.assertTrue(got[0].evidence & EVIDENCE_SEE_THROUGH)


# ===========================================================================
# Sensor reality: the sentinel, and the intensity law
# ===========================================================================

class TestThisSensorsQuirks(unittest.TestCase):

    def test_a_miss_is_the_65535_mm_sentinel_not_inf(self):
        ranges = flat_wall(1.5)
        self.assertTrue(np.any(ranges > RANGE_MAX))
        self.assertFalse(np.any(np.isinf(ranges)))
        miss = no_return_mask(ranges, RANGE_MIN, RANGE_MAX)
        self.assertTrue(miss.any(), 'the sentinel must read as a miss')
        # A detector keyed on finiteness alone would see nothing.
        self.assertFalse((~np.isfinite(ranges)).any())

    def test_inf_and_nan_are_still_treated_as_misses(self):
        ranges = flat_wall(1.5)
        ranges[10], ranges[11] = np.inf, np.nan
        miss = no_return_mask(ranges, RANGE_MIN, RANGE_MAX)
        self.assertTrue(miss[10] and miss[11])

    def test_intensity_guard_rejects_empty_and_constant_arrays(self):
        self.assertFalse(intensity_usable(None, N_BEAMS))
        self.assertFalse(intensity_usable([], N_BEAMS))
        self.assertFalse(intensity_usable(np.full(N_BEAMS, 7.0), N_BEAMS))
        self.assertFalse(intensity_usable(np.zeros(5), N_BEAMS))
        varying = np.arange(N_BEAMS, dtype=float)
        self.assertTrue(intensity_usable(varying, N_BEAMS))

    def test_an_empty_intensity_array_falls_back_without_raising(self):
        ranges = far_surface(flat_wall(1.2), -22.3, -20.5, 4.6)
        cands, used = run(ranges, intensities=[])
        self.assertFalse(used)
        self.assertTrue([c for c in cands if c.accepted],
                        'geometry-only must still find a see-through band')

    def test_the_range_exponent_defaults_to_zero_because_it_was_measured(self):
        """This sensor's intensity is AGC-normalized (log-log slope -0.24 over
        195k returns), so the textbook r^2 correction would inject a spurious
        range dependence rather than remove one."""
        self.assertEqual(DetectorConfig().intensity_range_exponent, 0.0)
        inten = np.full(4, 100.0)
        rng = np.array([1.0, 2.0, 4.0, 8.0])
        cos = np.ones(4)
        flat = normalize_intensity(inten, rng, cos)
        np.testing.assert_allclose(flat, 100.0)
        squared = normalize_intensity(inten, rng, cos, range_exponent=2.0)
        self.assertAlmostEqual(squared[-1] / squared[0], 64.0)


class TestVoidClassification(unittest.TestCase):

    def test_a_missing_run_is_bounded_on_both_sides(self):
        ranges = punch(flat_wall(1.2), -10.0, -8.0, MISS)
        voids = find_voids(ranges, no_return_mask(ranges, RANGE_MIN, RANGE_MAX), 3, 0.5)
        near = [v for v in voids if v.kind == VOID_NO_RETURN
                and beam_at(-11.0) < v.start < beam_at(-7.0)]
        self.assertEqual(len(near), 1)
        v = near[0]
        self.assertLess(v.left, v.start)
        self.assertGreater(v.right, v.end)

    def test_a_far_run_is_found_and_classified_separately(self):
        ranges = far_surface(flat_wall(1.2), -10.0, -8.0, 4.6)
        voids = find_voids(ranges, no_return_mask(ranges, RANGE_MIN, RANGE_MAX), 3, 0.5)
        self.assertTrue(any(v.kind == VOID_FAR for v in voids))

    def test_the_edge_of_the_field_of_view_is_not_an_opening(self):
        """Beyond +-70 deg this fixture reports the sentinel forever. An
        unbounded run is the end of the sweep, not a hole in something."""
        ranges = flat_wall(1.2, half_deg=70.0)
        voids = find_voids(ranges, no_return_mask(ranges, RANGE_MIN, RANGE_MAX), 3, 0.5)
        for v in voids:
            self.assertGreater(v.start, 0)
            self.assertLess(v.end, N_BEAMS - 1)

    def test_a_run_shorter_than_the_floor_is_ignored(self):
        ranges = punch(flat_wall(1.2), -10.0, -9.75, MISS)   # 2 beams
        voids = find_voids(ranges, no_return_mask(ranges, RANGE_MIN, RANGE_MAX), 3, 0.5)
        self.assertFalse(any(v.width_beams < 3 for v in voids))


class TestGeometryHelpers(unittest.TestCase):

    def test_contiguous_extent_ignores_a_distant_collinear_cluster(self):
        """Two clusters 5 m apart on one line are not one 5 m wall."""
        values = np.array([0.0, 0.1, 0.2, 5.0, 5.1, 5.2])
        lo, hi = contiguous_extent(values, 0.6)
        self.assertAlmostEqual(hi - lo, 0.2, places=6)

    def test_contiguous_extent_spans_a_mullion_sized_gap(self):
        values = np.array([0.0, 0.1, 0.5, 0.6, 1.0])
        lo, hi = contiguous_extent(values, 0.6)
        self.assertAlmostEqual(hi - lo, 1.0, places=6)

    def test_sample_segment_never_leaves_a_costmap_cell_gap(self):
        p0, p1 = np.array([0.0, 0.0]), np.array([1.0, 0.0])
        pts = sample_segment(p0, p1, 0.025)
        steps = np.hypot(*np.diff(pts, axis=0).T)
        self.assertLessEqual(steps.max(), 0.025 + 1e-9)
        np.testing.assert_allclose(pts[0], p0)
        np.testing.assert_allclose(pts[-1], p1)

    def test_sample_segment_handles_a_degenerate_segment(self):
        p = np.array([1.0, 2.0])
        np.testing.assert_allclose(sample_segment(p, p.copy(), 0.025), np.atleast_2d(p))


# ===========================================================================
# Real glass, not synthetic
# ===========================================================================

@unittest.skipUnless(DATA.exists(), f'fixture {DATA} missing')
class TestRealGlassCapture(unittest.TestCase):
    """Five scans taken 2026-09-14 with the car 1.10 m (perpendicular, laser
    frame) from a clear pane, a 4 m corridor behind it, a second pane beyond.
    The pane returns on ~97% of forward beams out to 60 deg of incidence and
    transmits in narrow bands -- see glass_detect.py's module docstring."""

    @classmethod
    def setUpClass(cls):
        d = np.load(DATA)
        cls.R = d['ranges'].astype(float)
        cls.I = d['intensities'].astype(float)
        cls.am = float(d['angle_min'])
        cls.ai = float(d['angle_increment'])
        cls.rmin = float(d['range_min'])
        cls.rmax = float(d['range_max'])

    def _detect(self, k, cfg=None, with_intensity=True):
        return detect(self.R[k], self.am, self.ai, self.rmin, self.rmax,
                      LASER, CAR, cfg or DetectorConfig(),
                      intensities=self.I[k] if with_intensity else None)

    def test_the_pane_is_detected_in_every_captured_scan(self):
        for k in range(self.R.shape[0]):
            got = [c for c in self._detect(k)[0] if c.accepted]
            self.assertTrue(got, f'scan {k}: real glass went undetected')

    def test_it_is_found_by_see_through_which_is_the_validated_evidence(self):
        for k in range(self.R.shape[0]):
            got = [c for c in self._detect(k)[0] if c.accepted]
            self.assertTrue(any(c.evidence & EVIDENCE_SEE_THROUGH for c in got),
                            f'scan {k}: accepted without see-through evidence')

    def test_the_pane_is_located_where_it_was_measured(self):
        """1.10 m perpendicular from the laser, which sits 0.12 m ahead of
        base_link, so 1.22 m from the origin. Measured, not chosen."""
        for k in range(self.R.shape[0]):
            got = [c for c in self._detect(k)[0] if c.accepted]
            best = max(got, key=lambda c: c.inlier_count)
            self.assertAlmostEqual(abs(best.line.signed(*CAR[:2])), 1.22, delta=0.06)
            self.assertLess(best.rms_m, 0.03)
            self.assertGreater(best.length_m, 1.5)

    def test_no_intensity_spike_fires_on_this_pane(self):
        """Documented reality, pinned so a future tuning pass cannot quietly
        claim the spike path is what finds this glass. Peak/median normalized
        intensity within +-20 deg was 1.84 against a spike_factor of 6.0."""
        for k in range(self.R.shape[0]):
            for c in self._detect(k)[0]:
                self.assertFalse(c.evidence & EVIDENCE_SPIKE,
                                 f'scan {k}: a spike was credited on diffuse glass')

    def test_geometry_only_finds_the_same_pane(self):
        """publish_intensity may be off; the validated path must not need it."""
        for k in range(self.R.shape[0]):
            got = [c for c in self._detect(k, with_intensity=False)[0] if c.accepted]
            self.assertTrue(got, f'scan {k}: lost the pane without intensity')

    def test_the_detection_is_stable_enough_for_the_tracker_to_match(self):
        tracker = GlassTracker(
            persistence_window=20, persistence_hits=4, match_endpoint_tol=0.2,
            match_angle_tol_deg=10.0, expect_return_tol_deg=2.0,
            fov_half_angle_rad=math.radians(135.0), max_range_m=10.0)
        for k in range(self.R.shape[0]):
            cands = [c for c in self._detect(k)[0] if c.accepted]
            tracker.update(cands, CAR, float(k), use_intensity=True)
        # Five scans of one pane must be five hits on ONE track, not five
        # tracks: if matching drifted, persistence could never confirm.
        self.assertEqual(len(tracker.tracks), 1, 'one pane became several tracks')
        self.assertEqual(tracker.tracks[0].hits, self.R.shape[0])


if __name__ == '__main__':
    unittest.main()
