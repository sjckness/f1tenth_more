"""The confidence model's range scaling, and what it does to FRONT.

cpu-cleanup R2 block 3d made FRONT require front_confidence >= 0.5.
wall_fusion.confidence scores an observation from an ABSOLUTE inlier count
(inlier_ref 40) and an ABSOLUTE residual (rms_ref 0.03 m), and a planar
LiDAR gives neither at range: measured on corridor10m_20261016T144537, the
same real corridor end wall scores 0.65 at 1 m and 0.23 at 21 m, so the gate
refused every wall past ~12 m and the contract went CLEAR over a wall the
LiDAR had to 6 mm. The references are now stretched with range.

What must NOT change is the phantom defence. The phantom block 3 was written
for -- a FRONT built at 1.9-3.3 m from the faces either side of a corridor
mouth -- lives INSIDE the reference range, where this scaling is the identity,
and is refused on geometry (the gap, the strip's own beams) in any case.
"""

import math

import pytest

from corridor_perception.geometry import Pose2D
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion
from f1tenth_perception.lidar_front_wall import virtual_wall_due
from f1tenth_perception.scene_labels import WallLine

ORIGIN = Pose2D(0.0, 0.0, 0.0)
FRONT_X = 0.405
STRIP_HALF = 0.186

LIDAR = wall_fusion.SOURCE_LIDAR

# The three bands of the measured table (output/wall_v2/report.md), as
# (range [m], inliers, rms [m]): the far wall, the mid wall whose fit was the
# tightest in the bag, and the near band the nominal references were cut for.
FAR = (21.0, 14, 0.107)
MID = (15.0, 19, 0.0156)
NEAR = (1.0, 32, 0.033)
PHANTOM_BAND = (3.0, 22, 0.0848)


def conf(range_m, inliers, rms, **kw):
    return wall_fusion.confidence(LIDAR, inliers, rms, 0, 0, range_m=range_m, **kw)


def blind(inliers, rms):
    """The pre-2026-10-18 score: the same call with no range at all."""
    return wall_fusion.confidence(LIDAR, inliers, rms, 0, 0)


# --- the model -------------------------------------------------------------

def test_the_far_wall_was_refused_for_being_far_and_now_is_not():
    gate = mwt.TrackerParams().front_min_confidence
    assert blind(*FAR[1:]) < gate, 'premise: the far wall used to fail the FRONT gate'
    assert conf(*FAR) >= gate


def test_the_tightest_fit_in_the_bag_also_passes_now():
    gate = mwt.TrackerParams().front_min_confidence
    assert blind(*MID[1:]) < gate
    assert conf(*MID) >= gate


def test_nothing_changes_inside_the_reference_range():
    for band in (NEAR, PHANTOM_BAND):
        assert conf(*band) == pytest.approx(blind(*band[1:]))


def test_the_phantom_band_still_fails_the_gate():
    """The scaling must not hand a near phantom the confidence block 3d
    denied it: at 3 m the stretch is the identity, so the score is the old
    one and still under the gate."""
    assert conf(*PHANTOM_BAND) < mwt.TrackerParams().front_min_confidence


def test_distance_alone_buys_nothing():
    """min_inlier_ref floors the scaled reference: a surface fitted from
    three returns stays weak however far away it is."""
    gate = mwt.TrackerParams().front_min_confidence
    assert conf(60.0, 3, 0.02) < gate
    assert conf(200.0, 3, 0.02) < gate
    # and a line under the floor is scored exactly as it was before: the
    # stretch is not applied at all, so its tight rms buys it nothing.
    assert conf(60.0, 3, 0.02) == pytest.approx(blind(3, 0.02))


def test_the_rms_stretch_saturates():
    """Past max_rms_scale the tolerated residual stops growing, so a wildly
    mis-fitted line at 100 m cannot be scored as a tight one."""
    ref = wall_fusion.CONF_MAX_RMS_SCALE * 0.03
    loose = conf(100.0, 40, ref * 4.0)
    assert loose < conf(100.0, 40, ref)
    assert loose < 0.5


def test_more_inliers_and_a_tighter_fit_never_score_lower():
    for r in (1.0, 5.0, 12.0, 21.0):
        assert conf(r, 30, 0.04) >= conf(r, 10, 0.04)
        assert conf(r, 30, 0.02) >= conf(r, 30, 0.08)


def test_a_stereo_only_wall_is_still_capped_wherever_it_is():
    for r in (1.0, 21.0, 60.0):
        assert wall_fusion.confidence(
            wall_fusion.SOURCE_STEREO, 200, 0.01, 0, 0, range_m=r) == pytest.approx(0.35)


# --- the tracker: a door in the end wall, and a gap straight ahead ---------

def far_piece(d, t0, t1, inliers=14, rms=0.107):
    """One coplanar piece of a wall at x = d, spanning y in [t0, t1]."""
    line = WallLine(1.0, 0.0, d, t0, t1, inliers, rms, 'lidar')
    return wall_fusion.Observation(
        line, 0.0, 2.5, False, LIDAR,
        wall_fusion.confidence(LIDAR, inliers, rms, 0, 0, range_m=abs(d)))


def feed(tracker, observations, n=6, t0=1.0, dt=0.1):
    for i in range(n):
        tracker.step(list(observations), t0 + i * dt, ORIGIN)


def test_an_end_wall_with_a_door_is_front_because_a_piece_crosses_the_strip():
    """The corridor's end wall at 18 m with a doorway off to one side. The
    left piece still covers part of the car-width strip, so the car IS about
    to drive into a wall and the contract must say so. The doorway is 0.1 m
    wider than the collinear gap (collinear_gap_m), well off the rule's
    boundary, so the two pieces stay two tracks: the FRONT is the piece that
    is really there, not a line drawn across the opening."""
    trk = mwt.MultiWallTracker()
    doorway = trk.p.collinear_gap_m + 0.1
    feed(trk, [far_piece(18.0, -3.0, -0.1), far_piece(18.0, -0.1 + doorway, 3.0)])
    front = trk.front_track(ORIGIN, FRONT_X)
    assert front is not None, 'a piece crosses the strip: this is a wall'
    # The contract's FRONT distance is where the strip meets the MEASURED
    # segment, not where the heading ray meets the infinite line -- here the
    # ray goes through the doorway and misses, and that is not CLEAR.
    d = mwt.strip_distance(
        mwt.segment_in_base_link(front, ORIGIN), FRONT_X, STRIP_HALF)
    assert d == pytest.approx(18.0 - FRONT_X, abs=0.05)
    assert math.isnan(mwt.front_distances(front, ORIGIN, FRONT_X)[1])


def test_a_gap_straight_ahead_with_pieces_only_at_the_sides_is_not_front():
    """The block 3 failure itself, at range: two coplanar faces either side of
    an opening the car can drive through. Neither reaches the strip, the gap
    is far wider than the collinear one, and no FRONT may be invented across
    it -- the range scaling must not bring this back."""
    trk = mwt.MultiWallTracker()
    feed(trk, [far_piece(18.0, -3.0, -0.5), far_piece(18.0, 0.5, 3.0)])
    assert trk.tracks, 'the two faces are still tracked'
    for t in trk.tracks:
        assert not mwt.crosses_strip(t, ORIGIN, FRONT_X, STRIP_HALF)
    assert trk.front_track(ORIGIN, FRONT_X) is None


def test_the_far_wall_is_front_only_because_of_the_scaling():
    """The same 18 m wall scored the old way is refused, which is what the
    bag showed; scored with its range it is FRONT."""
    line = WallLine(1.0, 0.0, 18.0, -3.0, 3.0, 14, 0.107, 'lidar')
    weak = wall_fusion.Observation(line, 0.0, 2.5, False, LIDAR, blind(14, 0.107))
    strong = wall_fusion.Observation(
        line, 0.0, 2.5, False, LIDAR, conf(18.0, 14, 0.107))

    cold = mwt.MultiWallTracker()
    feed(cold, [weak])
    assert cold.front_track(cold and ORIGIN, FRONT_X) is None

    warm = mwt.MultiWallTracker()
    feed(warm, [strong])
    assert warm.front_track(ORIGIN, FRONT_X) is not None


# --- the rate gate under decimation ---------------------------------------

def test_the_rate_gate_passes_every_tracker_step_at_decimation_four():
    """virtual_wall_due must not swallow a tracker step: with the tracker on
    it publishes only on a step, and the cap is 0.75 / rate_hz, which at
    10 Hz is 0.075 s -- shorter than the 0.1 s step period 40 Hz scans
    decimated by 4 give. Every step therefore reaches the contract."""
    rate, period = 10.0, 4 / 40.0
    last, published = None, 0
    for k in range(50):
        now = k * period
        if virtual_wall_due(True, True, now, last, rate):
            last = now
            published += 1
    assert published == 50


def test_the_rate_gate_still_caps_a_tracker_that_steps_too_often():
    rate = 10.0
    last, published = None, 0
    for k in range(100):
        now = k * 0.01                      # 100 Hz of steps
        if virtual_wall_due(True, True, now, last, rate):
            last = now
            published += 1
    assert published < 20, 'the 10 Hz cap must still hold'


def test_a_scan_the_tracker_skipped_publishes_nothing():
    assert virtual_wall_due(True, False, 1.0, None, 10.0) is False
