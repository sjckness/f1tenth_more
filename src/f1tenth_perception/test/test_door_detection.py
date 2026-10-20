"""
door_detection on synthetic scans: 270 deg / 0.25 deg, range noise sigma 0.01 m.

The world frame is odom and the car sits at its origin unless a test moves it,
so base_link and odom coincide; the laser is at LASER = (0.305, 0, 0) in
base_link, as on the car. The wall tracks are built directly -- confirmed,
with the role the tracker would give them, by default 'front' -- (the
tracker has its own suite) except in TestThroughTheRealTracker and
TestTheCollinearGapRule, which run the real extraction and MultiWallTracker
on the same scans and drive the car past a side wall.

The scene most tests use: a wall square to the car at x = 3 m, a doorway in
it centred on y = 0, and the next room's wall at x = 6 m. door_candidates
returns a record for EVERY gap it evaluated; a door is a record with code ''.
"""

import copy
import math

from corridor_perception.geometry import Pose2D
from f1tenth_perception import door_detection as dd
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_mask
from f1tenth_perception import wall_fusion
from f1tenth_perception.scene_labels import detection_sectors

import numpy as np

import pytest

LASER = (0.305, 0.0, 0.0)
N_BEAMS = 1081
ANGLE_MIN = -math.radians(135.0)
INC = math.radians(0.25)
NO_RETURN = 65.533
ORIGIN = Pose2D(0.0, 0.0, 0.0)
WALL_X = 3.0
BACK_X = 6.0


#: Glass returns the beam only this close to square-on (corridor10m_20261016T144537:
#: a pane returned 100 / 94 / 51 / 19 / 0 % of its beams at 0-1 / ... / 4-5 deg).
PANE_MAX_DEG = 3.0


def raycast(car, segments, circles=(), noise=0.01, seed=0, panes=()):
    """
    Ranges of one scan from the laser on car (Pose2D) against segments and circles.

    `panes` are segments of GLASS: invisible to a beam more than PANE_MAX_DEG
    off their normal, which passes through to whatever is behind.
    """
    rng = np.random.default_rng(seed)
    laser = car.compose(Pose2D(*LASER))
    origin = laser.t
    ang = laser.theta + ANGLE_MIN + np.arange(N_BEAMS) * INC
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(N_BEAMS, np.inf)
    glass = [False] * len(segments) + [True] * len(panes)
    for ((ax, ay), (bx, by)), is_glass in zip(list(segments) + list(panes), glass):
        e = np.array([bx - ax, by - ay], dtype=float)
        w = np.array([ax, ay], dtype=float) - origin
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        if is_glass:
            normal = np.array([-e[1], e[0]]) / np.hypot(*e)
            hit &= np.abs(d @ normal) >= math.cos(math.radians(PANE_MAX_DEG))
        best = np.where(hit & (t < best), t, best)
    for (cx, cy), r in circles:
        oc = origin - np.array([cx, cy])
        b = d @ oc
        disc = b * b - (oc @ oc - r * r)
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc >= 0) & (t > 0)
        best = np.where(hit & (t < best), t, best)
    ranges = best + rng.normal(0.0, noise, N_BEAMS)
    return np.where(np.isfinite(best), ranges, NO_RETURN)


def scan_of(ranges, objects=()):
    """Build the scan tuple the node hands the tracker: (ranges, bearings, laser xy, cut)."""
    sectors = detection_sectors(objects, LASER[:2], math.radians(3.0)) if objects else ()
    r, b, _angles, cut = mwt.scan_in_base_link(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER,
                                               sectors, with_cut=True)
    return r, b, LASER[:2], cut


def wall_with_gap(gap_lo, gap_hi, x=WALL_X, y_min=-3.0, y_max=3.0):
    """Return the wall at x as two segments either side of the opening [gap_lo, gap_hi]."""
    return [((x, y_min), (x, gap_lo)), ((x, gap_hi), (x, y_max))]


def back_wall(x=BACK_X, half=6.0):
    return [((x, -half), (x, half))]


def track(track_id, t0, t1, rho=WALL_X, theta=0.0, state=mwt.STATE_CONFIRMED, role='front'):
    """
    Build a wall track on rho = x cos(theta) + y sin(theta), extent t0..t1 along it.

    At theta 0 the tangent is +y, so t is simply y on the wall x = rho.
    """
    trk = mwt.Track(
        id=track_id, x=np.array([rho, theta], dtype=float), P=np.eye(2) * 1e-3,
        t0=t0, t1=t1, born=0.0, last_seen=0.0,
        ever_confirmed=state != mwt.STATE_TENTATIVE, state=state)
    trk.role = role
    return trk


def track_through(track_id, p0, p1, err_deg=0.0, role='front'):
    """Build a track over p0..p1 whose line is fitted err_deg off, about its midpoint."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    tang = (p1 - p0) / np.hypot(*(p1 - p0))
    theta = math.atan2(-tang[0], tang[1]) + math.radians(err_deg)
    n = np.array([math.cos(theta), math.sin(theta)])
    t_axis = np.array([-n[1], n[0]])
    t0, t1 = sorted((float(t_axis @ p0), float(t_axis @ p1)))
    return track(track_id, t0, t1, rho=float(n @ (0.5 * (p0 + p1))), theta=theta, role=role)


def one_wall(track_id=1):
    """Return the x = 3 wall as ONE track across the doorway (the in-track fallback)."""
    return [track(track_id, -3.0, 3.0)]


def two_walls(ids=(1, 2), half_gap=0.425):
    """Return the x = 3 wall as two tracks either side of the doorway."""
    return [track(ids[0], -3.0, -half_gap), track(ids[1], half_gap, 3.0)]


def records(tracks, segments, circles=(), objects=(), car=ORIGIN, p=None, seed=0, panes=(),
            stereo=None, manhattan=None):
    """Return every record door_candidates makes of one synthetic scan."""
    scan = scan_of(raycast(car, segments, circles, seed=seed, panes=panes), objects)
    return dd.door_candidates(tracks, car, scan, p, stereo=stereo, manhattan=manhattan)


def doors_in(recs):
    return [r for r in recs if r.code == '']


def codes(recs):
    return {r.code for r in recs if r.code}


def only(recs):
    assert len(recs) == 1, [(r.code, r.parents, round(r.width, 3)) for r in recs]
    return recs[0]


def car_at_laser(lx, ly, heading=0.0):
    """Return the car pose whose LASER sits at (lx, ly) with the given heading."""
    return Pose2D(lx - LASER[0] * math.cos(heading), ly - LASER[0] * math.sin(heading),
                  heading)


#: The laser 1.73 m to the right of the door's centre and 1 m from its wall:
#: the ray to the centre is 60 deg off the wall's normal.
AT_60_DEG = car_at_laser(WALL_X - 1.0, -math.tan(math.radians(60.0)))
DOOR_SCENE = wall_with_gap(-0.425, 0.425) + back_wall()


# ==============================================================================
# The codes and the parameters
# ==============================================================================

class TestCodesAndParams:

    def test_every_code_has_exactly_one_class(self):
        strong, weak, geo = set(dd.STRONG_CODES), set(dd.WEAK_CODES), set(dd.GEOMETRY_CODES)
        assert not (strong & weak or strong & geo or weak & geo)
        assert strong | weak | geo == set(dd.REJECT_CODES)
        assert len(dd.REJECT_CODES) == len(set(dd.REJECT_CODES))
        assert dd.code_class('') == 'ok'
        assert {dd.code_class(c) for c in dd.STRONG_CODES} == {'strong'}
        assert {dd.code_class(c) for c in dd.WEAK_CODES} == {'weak'}
        assert {dd.code_class(c) for c in dd.GEOMETRY_CODES} == {'geometry'}

    def test_the_addendum_assigns_these_codes_to_these_classes(self):
        assert set(dd.STRONG_CODES) == {'on_line', 'behind', 'not_boundary', 'stereo_wall',
                                        'furniture', 'short_line', 'short_side',
                                        'off_manhattan', 'not_a_leaf'}
        assert set(dd.WEAK_CODES) == {'oblique_view', 'few_beams', 'jamb_occluded',
                                      'unknown_in_gap'}
        assert set(dd.GEOMETRY_CODES) == {'not_collinear', 'untracked_jamb', 'width'}

    def test_an_unknown_code_is_an_error(self):
        with pytest.raises(ValueError):
            dd.code_class('shallow')

    def test_door_params_defaults_are_the_addendum_table(self):
        """Coupled to the addendum's Params section and stack_params.yaml (CLAUDE.md)."""
        p = dd.DoorParams()
        expected = {
            'manhattan_prior': True, 'manhattan_tol_deg': 10.0,
            'manhattan_min_wall_len_m': 1.5, 'manhattan_tau_s': 2.0,
            'min_line_extent_m': 2.0, 'min_side_len_m': 0.5, 'max_view_angle_deg': 45.0,
            'behind_frac': 0.7, 'boundary_behind_min_m': 0.30, 'boundary_max_hits': 2,
            'boundary_end_margin_m': 0.10, 'on_line_min_beams': 2,
            'jamb_height_check': True, 'jamb_min_height_m': 1.8,
            'furniture_max_height_m': 1.3, 'min_viewpoints': 2, 'viewpoint_sep_m': 0.3,
            'width_std_max_m': 0.05, 'confirm_window': 20, 'candidate_max_age_s': 10.0,
            'confirmed_max_age_s': 60.0, 'behind_margin_m': 1.0, 'square_looks': 0,
            'reveal_max_depth_m': 0.20, 'core_fraction': 0.6, 'leaf_max_len_m': 1.0,
            'leaf_hinge_tol_m': 0.10, 'stereo_glass_veto': False,
            'width_min_m': mwt.DOOR_WIDTH_MIN_M, 'width_max_m': 1.0}
        for name, value in expected.items():
            assert getattr(p, name) == pytest.approx(value), name
        assert p.square_incidence_rad == pytest.approx(math.radians(3.0))

    def test_the_removed_parameters_are_gone(self):
        p = dd.DoorParams()
        for name in ('confirm_hits', 'delete_after_s', 'coast_confidence_decay',
                     'frame_jambs', 'frame_min_pieces', 'frame_jamb_min_length_m',
                     'piece_max_rms_m', 'piece_max_range_m', 'piece_max_length_m',
                     'pane_min_on_beams', 'pane_min_cycles', 'pane_memory_s'):
            assert not hasattr(p, name), name

    def test_tentative_is_the_old_name_of_candidate(self):
        assert dd.STATE_TENTATIVE == dd.STATE_CANDIDATE == 0
        assert dd.STATE_CONFIRMED == 1
        assert dd.STATE_NAMES[dd.STATE_CANDIDATE] == 'candidate'


# ==============================================================================
# A doorway, and gaps of the wrong width
# ==============================================================================

class TestTheDoorway:

    @pytest.mark.parametrize('tracks', [one_wall(), two_walls()],
                             ids=['inside_one_track', 'between_two_tracks'])
    def test_a_0_85_m_gap_seen_through_to_3_m_behind_is_a_door(self, tracks):
        door = only(records(tracks, DOOR_SCENE))
        assert door.code == '' and door.cls == 'ok' and door.visible
        # The jambs are the last returns ON the wall: within a beam spacing
        # (1.3 cm at 3 m) of the true edges, never inside the opening.
        assert door.width == pytest.approx(0.85, abs=0.04)
        assert door.width >= 0.85 - 0.02
        assert door.centre == pytest.approx([WALL_X, 0.0], abs=0.03)
        # The crossing direction points away from the car, through the door.
        assert door.normal == pytest.approx([1.0, 0.0], abs=1e-6)
        assert door.n_beams >= door.n_required >= 8
        assert set(door.parents) <= {t.id for t in tracks}
        assert 0.5 <= door.confidence <= 0.8
        assert door.view_deg == pytest.approx(0.0, abs=1.0)
        assert door.behind_frac == pytest.approx(1.0)
        assert door.boundary_hits == 0
        # No stereo: the jamb height is unknown, the glass not evaluated.
        assert door.jamb_height_unknown is True and door.glass_suspect is None
        assert door.host_offset_deg is None

    def test_no_return_at_all_through_the_gap_also_counts_as_open(self):
        assert only(records(one_wall(), wall_with_gap(-0.425, 0.425))).code == ''

    @pytest.mark.parametrize('half_gap', [0.25, 0.70], ids=['0.5m', '1.4m'])
    def test_a_gap_of_the_wrong_width_is_a_geometry_record(self, half_gap):
        rec = only(records(two_walls(half_gap=half_gap),
                           wall_with_gap(-half_gap, half_gap) + back_wall()))
        assert rec.code == 'width' and rec.cls == 'geometry' and not rec.visible

    def test_a_wall_with_no_gap_has_no_record_at_all(self):
        assert records(one_wall(), [((WALL_X, -3.0), (WALL_X, 3.0))]) == []

    def test_door_candidates_never_modifies_the_tracks_or_the_scan(self):
        """THE OVERLAY RULE: the snap and the search work on copies."""
        tracks = [track_through(1, (3.0, -3.0), (3.0, -0.425), 3.0),
                  track_through(2, (3.0, 0.425), (3.0, 3.0), 3.0)] + SIDE_TRACKS
        prior = dd.ManhattanPrior()
        prior.update(tracks, 0.0)
        scan = scan_of(raycast(ORIGIN, DOOR_SCENE + SIDE_SEGMENTS))
        before = (copy.deepcopy(tracks), copy.deepcopy(scan))
        assert doors_in(dd.door_candidates(tracks, ORIGIN, scan, manhattan=prior))
        for trk, old in zip(tracks, before[0]):
            assert np.array_equal(trk.x, old.x) and (trk.t0, trk.t1) == (old.t0, old.t1)
        for now, old in zip(scan, before[1]):
            assert np.array_equal(np.asarray(now), np.asarray(old))


# ==============================================================================
# What the beams through the gap say
# ==============================================================================

def sparse_pane_scan():
    """Glass the LiDAR half-sees: single returns ON the line every 6 beams in the gap."""
    ranges = raycast(ORIGIN, DOOR_SCENE)
    pane = raycast(ORIGIN, [((WALL_X, -0.425), (WALL_X, 0.425))])
    hit = np.flatnonzero(pane < NO_RETURN)
    ranges[hit[::6]] = pane[hit[::6]]
    return scan_of(ranges)


class TestTheBeamsThroughTheGap:

    def test_a_pane_the_lidar_sees_across_the_gap_is_wall(self):
        """A pane (or a dark panel) returning ON the line closes the gap: no record."""
        assert records(one_wall(), [((WALL_X, -3.0), (WALL_X, 3.0))] + back_wall()) == []

    def test_on_line_returns_inside_one_track_are_on_line(self):
        """In-track: the tracker holds wall across the gap, so the returns are its wall."""
        rec = only(dd.door_candidates(one_wall(), ORIGIN, sparse_pane_scan()))
        assert rec.code == 'on_line' and rec.cls == 'strong' and rec.visible

    def test_on_line_returns_between_two_tracks_are_unknown_in_gap(self):
        """No host claims that stretch of line: whatever returned there is unknown."""
        rec = only(dd.door_candidates(two_walls(), ORIGIN, sparse_pane_scan()))
        assert rec.code == 'unknown_in_gap' and rec.detail == 'unknown_on_line'
        assert rec.cls == 'weak' and not rec.visible

    def test_one_on_line_return_inside_a_track_is_only_not_counted_as_through(self):
        ranges = raycast(ORIGIN, DOOR_SCENE)
        pane = raycast(ORIGIN, [((WALL_X, -0.425), (WALL_X, 0.425))])
        k = int(np.flatnonzero(pane < NO_RETURN)[30])
        ranges[k] = pane[k]
        rec = only(dd.door_candidates(one_wall(), ORIGIN, scan_of(ranges)))
        assert rec.code == '' and rec.behind_frac < 1.0

    def test_a_surface_0_2_m_behind_across_the_gap_is_a_strong_reject(self):
        rec = only(records(one_wall(), wall_with_gap(-0.425, 0.425)
                           + [((WALL_X + 0.2, -0.6), (WALL_X + 0.2, 0.6))]))
        assert rec.cls == 'strong' and rec.visible
        assert rec.behind_frac == pytest.approx(0.0)

    def test_a_box_in_front_of_a_continuous_wall_casts_a_gap_that_is_jamb_occluded(self):
        """
        A box in front of a CONTINUOUS wall casts a door-wide gap in its returns.

        The shadow of a 0.56 m box 1.1 m in front of the wall is 0.86 m wide,
        with closer returns at its ends: a shadow edge, not a jamb -- undecidable.
        """
        box = [((1.9, -0.28), (1.9, 0.28))]
        rec = only(records(two_walls(half_gap=0.43), [((WALL_X, -3.0), (WALL_X, 3.0))] + box))
        assert rec.code == 'jamb_occluded' and rec.cls == 'weak' and not rec.visible

    def test_a_real_opening_with_one_jamb_hidden_by_a_box_is_jamb_occluded(self):
        box = [((2.0, 0.196), (2.0, 0.359))]
        rec = only(records(one_wall(), DOOR_SCENE + box))
        assert rec.code == 'jamb_occluded' and not rec.visible

    def test_too_few_beams_through_a_far_gap_is_few_beams_and_not_visible(self):
        """0.8 m at 25 m square-on is ~7 beams at 0.25 deg, under the floor of 8."""
        far = 25.0
        rec = only(records([track(1, -6.0, 6.0, rho=far)],
                           wall_with_gap(-0.4, 0.4, x=far, y_min=-6.0, y_max=6.0)))
        assert rec.code == 'few_beams' and not rec.visible

    def test_the_same_gap_near_by_is_decided(self):
        assert only(records(one_wall(), wall_with_gap(-0.4, 0.4))).code == ''

    def test_the_beam_requirement_grows_as_the_door_gets_closer(self):
        """n_required = max(floor, 0.8 x what the geometry promises)."""
        near = only(records([track(1, -3.0, 3.0, rho=1.5)], wall_with_gap(-0.425, 0.425, x=1.5)))
        far = only(records(one_wall(), wall_with_gap(-0.425, 0.425)))
        assert near.n_required > far.n_required > 8

    def test_depth_is_perpendicular_to_the_line_never_along_the_ray(self):
        """A return 0.5 m behind the line seen 60 deg off its normal is 1.0 m down the ray."""
        p = dd.DoorParams()
        cos60 = math.cos(math.radians(60.0))
        assert dd.beam_depth(1.0 + 1.0, 1.0, cos60) == pytest.approx(0.5)
        assert dd.classify_beams([2.0], [False], [1.0], p, [cos60])[0] == dd.BEAM_SHALLOW
        # Along the ray it would have read 1.0 m: through.
        assert dd.classify_beams([2.0], [False], [1.0], p)[0] == dd.BEAM_BEYOND
        assert dd.classify_beams([np.inf], [False], [1.0], p, [cos60])[0] == dd.BEAM_NONE
        assert dd.classify_beams([2.0], [True], [1.0], p, [cos60])[0] == dd.BEAM_CUT


# ==============================================================================
# The walls that may host a door
# ==============================================================================

class TestTheHosts:

    @pytest.mark.parametrize('role', ['front', 'left', 'right'])
    def test_a_confirmed_front_left_or_right_track_hosts_a_door(self, role):
        tracks = [track(1, -3.0, -0.425, role=role), track(2, 0.425, 3.0, role=role)]
        assert only(records(tracks, DOOR_SCENE)).code == ''

    @pytest.mark.parametrize('state,role', [
        (mwt.STATE_TENTATIVE, 'front'), (mwt.STATE_COASTING, 'front'),
        (mwt.STATE_CONFIRMED, ''), (mwt.STATE_CONFIRMED, 'unknown'),
        (mwt.STATE_CONFIRMED, 'other'), (mwt.STATE_CONFIRMED, 'back')],
        ids=['tentative', 'coasting', 'no_role', 'unknown', 'other', 'back'])
    def test_any_other_track_never_hosts_and_makes_no_record(self, state, role):
        tracks = [track(1, -3.0, -0.425), track(2, 0.425, 3.0, state=state, role=role)]
        assert records(tracks, DOOR_SCENE) == []

    def test_a_corner_is_a_not_collinear_record(self):
        """Two walls meeting at 90 deg with a door-wide gap at the corner."""
        a = track(1, -3.0, 0.0)                                  # x = 3, y in [-3, 0]
        # At theta pi/2 the tangent is -x, so t in [-6, -3.6] is x in [3.6, 6].
        b = track(2, -6.0, -3.6, rho=0.6, theta=math.pi / 2, role='left')
        segs = [((WALL_X, -3.0), (WALL_X, 0.0)), ((3.6, 0.6), (6.0, 0.6))]
        rec = only(records([a, b], segs + back_wall(x=8.0)))
        assert rec.code == 'not_collinear' and rec.cls == 'geometry'
        assert set(rec.parents) == {1, 2}

    def test_two_parallel_walls_offset_by_more_than_the_tolerance_are_not_collinear(self):
        a, b = track(1, -3.0, -0.425), track(2, 0.425, 3.0, rho=WALL_X + 0.3)
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X + 0.3, 0.425),
                                                     (WALL_X + 0.3, 3.0))]
        assert only(records([a, b], segs + back_wall())).code == 'not_collinear'

    def test_a_jamb_more_than_0_3_m_off_its_track_is_untracked_jamb(self):
        """The wall runs on to y = -0.425, its track only to y = -0.8."""
        rec = only(records([track(1, -3.0, -0.8), track(2, 0.425, 3.0)], DOOR_SCENE))
        assert rec.code == 'untracked_jamb' and rec.cls == 'geometry'

    def test_the_door_is_found_from_a_moved_and_turned_car(self):
        car = Pose2D(0.5, -0.4, math.radians(20.0))
        door = only(records(one_wall(), DOOR_SCENE, car=car))
        assert door.code == ''
        assert car.transform_points(door.centre)[0] == pytest.approx([WALL_X, 0.0], abs=0.04)

    def test_a_doorway_straight_ahead_has_no_host_because_its_two_pieces_are_role_other(self):
        """
        The real tracker's roles: beside the travel strip, a wall ahead is 'other'.

        A consequence of the host rule (front/left/right only) with the
        geometric roles (multi_wall_tracker, WHAT A ROLE IS): the pieces
        either side of a doorway straight ahead do not cross the 0.57 m travel
        strip, and their normal is not lateral -- so neither is a host, and
        such a doorway is found only on a track spanning it (the in-track
        fallback) or once it is beside the car (left/right).
        """
        tracker = mwt.MultiWallTracker()
        tracker.laser_xy = LASER[:2]
        ep = mwt.extraction_params(backend='python')
        for i in range(6):
            ranges = raycast(ORIGIN, DOOR_SCENE, seed=i)
            lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=ep)
            obs = [wall_fusion.Observation(mwt.line_to_wall_line(ln), 0.0, 2.5, False,
                                           wall_fusion.SOURCE_LIDAR, 0.8) for ln in lines]
            tracker.step(obs, 0.1 * i, ORIGIN, scan=scan_of(ranges))
        pieces = [t for t in tracker.tracks if abs(t.rho - WALL_X) < 0.1]
        assert len(pieces) == 2 and {t.role for t in pieces} == {'other'}
        assert dd.door_candidates(tracker.tracks, ORIGIN, scan_of(ranges)) == []


# ==============================================================================
# THE ADDENDUM's tests F1-F12, as written
# ==============================================================================

#: The room's side walls at y = +-3 (normal along y): what tells the Manhattan
#: prior the room's axes when the door's own wall is fitted off them.
SIDE_SEGMENTS = [((-3.0, 3.0), (3.0, 3.0)), ((-3.0, -3.0), (3.0, -3.0))]
SIDE_TRACKS = [track(3, -3.0, 3.0, rho=3.0, theta=math.pi / 2, role='left'),
               track(4, -3.0, 3.0, rho=3.0, theta=-math.pi / 2, role='right')]

#: Two desks 1.2 m wide, 0.8 m apart, the room's wall 0.5 m behind them.
DESKS = [((WALL_X, -1.6), (WALL_X, -0.4)), ((WALL_X, 0.4), (WALL_X, 1.6))]
DESK_TRACKS = [track(1, -1.6, -0.4), track(2, 0.4, 1.6)]


def box_behind_the_core():
    """Return a box face 0.3 m behind the doorway across its core: a STRONG reject."""
    return [((WALL_X + 0.3, -0.3), (WALL_X + 0.3, 0.3))]


class TestTheAddendumF:

    def test_f1_two_oblique_unknown_fragments_with_a_0_8_m_gap_are_no_door(self):
        """Fragments the tracker holds only tentatively, or as unknown, never host."""
        frags = [((2.6, -1.0), (3.0, -0.4)), ((3.0, 0.4), (3.4, 1.0))]
        tracks = [track_through(1, *frags[0]), track_through(2, *frags[1])]
        for trk in tracks:
            trk.state, trk.ever_confirmed, trk.role = mwt.STATE_TENTATIVE, False, 'unknown'
        assert records(tracks, frags + back_wall()) == []
        for trk in tracks:
            trk.state, trk.ever_confirmed = mwt.STATE_CONFIRMED, True
        assert records(tracks, frags + back_wall()) == []

    def test_f2_a_real_door_on_a_wall_fitted_3_deg_off_is_found_after_snapping(self):
        hosts = [track_through(1, (3.0, -3.0), (3.0, -0.425), 3.0),
                 track_through(2, (3.0, 0.425), (3.0, 3.0), 3.0)]
        tracks = hosts + SIDE_TRACKS
        prior = dd.ManhattanPrior()
        prior.update(tracks, 0.0)
        assert abs(prior.offset(hosts[0].theta)) > math.radians(1.5)
        # As fitted, the two pieces are parallel but 0.18 m apart at the door:
        # no line, no door.
        recs = records(tracks, DOOR_SCENE + SIDE_SEGMENTS)
        assert doors_in(recs) == [] and 'not_collinear' in codes(recs)
        for seed in range(5):
            door = only(doors_in(records(tracks, DOOR_SCENE + SIDE_SEGMENTS, seed=seed,
                                         manhattan=prior)))
            assert set(door.parents) == {1, 2}
            assert door.width == pytest.approx(0.85, abs=0.04)
            assert door.host_offset_deg == pytest.approx(
                math.degrees(prior.offset(hosts[0].theta)), abs=0.01)

    def test_f3_a_0_3_m_stub_next_to_a_gap_is_short_side(self):
        tracks = [track(1, -3.0, -0.425), track(2, 0.425, 0.725)]
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 0.725))]
        rec = only(records(tracks, segs + back_wall()))
        assert rec.code == 'short_side' and rec.cls == 'strong'

    def test_f4_two_desks_0_8_m_apart_a_wall_0_5_m_behind_are_rejected_head_on_and_at_60(self):
        wall = [((WALL_X + 0.5, -6.0), (WALL_X + 0.5, 6.0))]
        head_on = only(records(DESK_TRACKS, DESKS + wall))
        assert head_on.cls == 'strong' and head_on.visible
        assert head_on.behind_frac == pytest.approx(0.0)
        # At 60 deg the wall is 1.0 m down every ray through the gap, the
        # margin: along the ray it would read as through. Perpendicular, the
        # gap's core is still all shallow -- and the view is too oblique to
        # decide anyway.
        oblique = only(records(DESK_TRACKS, DESKS + wall, car=AT_60_DEG))
        assert oblique.code == 'oblique_view' and not oblique.visible
        assert oblique.behind_frac == pytest.approx(0.0)
        lifted = only(records(DESK_TRACKS, DESKS + wall, car=AT_60_DEG,
                              p=dd.DoorParams(max_view_angle_deg=89.0)))
        assert lifted.cls == 'strong'

    def test_f5_a_door_3_m_deep_is_kept_and_at_60_deg_is_oblique_view_and_stays_alive(self):
        tracker = dd.DoorTracker()
        for i, y in enumerate((0.0, 0.5)):
            door_cycle(tracker, 0.1 * i, car=Pose2D(0.0, y, 0.0))
        assert [d.state for d in tracker.doors] == [dd.STATE_CONFIRMED]
        door = tracker.doors[0]
        for i in range(2, 30):
            recs = door_cycle(tracker, 0.1 * i, car=AT_60_DEG)
            assert only(recs).code == 'oblique_view'
        assert tracker.doors == [door] and door.state == dd.STATE_CONFIRMED
        assert door.strikes == 0 and door.hits == 2

    def test_f6_two_0_6_m_cabinets_with_a_gap_are_short_line(self):
        cabinets = [((WALL_X, -1.0), (WALL_X, -0.4)), ((WALL_X, 0.4), (WALL_X, 1.0))]
        rec = only(records([track(1, -1.0, -0.4), track(2, 0.4, 1.0)],
                           cabinets + back_wall()))
        assert rec.code == 'short_line' and rec.cls == 'strong'

    @pytest.mark.parametrize('laser_y', [0.0, -1.0, 1.0, -1.4])
    def test_f7_a_door_next_to_a_radiator_0_1_m_off_the_wall_is_kept(self, laser_y):
        """A 1 m radiator 0.1 m deep, flush with the right jamb, its face 0.1 m off the wall."""
        rad = [((WALL_X - 0.1, -1.425), (WALL_X - 0.1, -0.425)),
               ((WALL_X - 0.1, -0.425), (WALL_X, -0.425))]
        door = only(records(two_walls(), DOOR_SCENE + rad, car=car_at_laser(1.5, laser_y)))
        assert door.code == '', door.code
        assert door.width == pytest.approx(0.85, abs=0.08)

    def test_f8_a_door_without_depth_is_kept_and_desks_with_depth_are_furniture(self):
        labels, depth = render(gap=True)
        blind = dd.StereoFrame(labels, np.zeros_like(depth), FX, FX, CX, CY, ROT, CAM_T,
                               depth_scale=0.001)
        door = only(records(two_walls(), DOOR_SCENE, stereo=blind))
        assert door.code == '' and door.jamb_height_unknown is True
        labels, depth = render_planes(DESK_PLANES)
        desks = dd.StereoFrame(labels, depth, FX, FX, CX, CY, ROT, CAM_T, depth_scale=0.001)
        rec = only(records(DESK_TRACKS, DESKS + back_wall(), stereo=desks))
        assert rec.code == 'furniture' and rec.cls == 'strong'
        assert rec.jamb_tops[0] == pytest.approx(0.7, abs=0.05)
        assert rec.jamb_height_unknown is False

    def test_f9_a_door_seen_from_one_position_stays_candidate_from_two_0_5_m_apart_confirms(
            self):
        tracker = dd.DoorTracker()
        for i in range(10):
            door_cycle(tracker, 0.1 * i)
        (door,) = tracker.doors
        assert door.state == dd.STATE_CANDIDATE and door.n_viewpoints == 1
        assert door.hits == 10
        door_cycle(tracker, 1.0, car=Pose2D(0.0, 0.5, 0.0))
        assert tracker.doors == [door] and door.state == dd.STATE_CONFIRMED
        assert door.n_viewpoints == 2 and tracker.n_confirmed == 1

    def test_f10_a_width_alternating_0_75_and_0_95_never_confirms(self):
        tracker = dd.DoorTracker()
        for i in range(20):
            car = Pose2D(0.0, 0.1 * i, 0.0)
            width = 0.75 if i % 2 == 0 else 0.95
            tracker.step([record_at(car, width)], car, 0.1 * i)
        (door,) = tracker.doors
        assert door.state == dd.STATE_CANDIDATE and tracker.n_confirmed == 0
        assert door.n_viewpoints >= 2 and door.width_std == pytest.approx(0.1, abs=0.01)

    def test_f11_one_strong_reject_demotes_a_second_deletes_oblique_frames_between_do_nothing(
            self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        recs = door_cycle(tracker, 1.0, segments=DOOR_SCENE + box_behind_the_core())
        assert only(recs).cls == 'strong' and recs[0].visible
        assert tracker.doors == [door] and door.state == dd.STATE_CANDIDATE
        assert door.strikes == 1 and len(door.evidence) == 0
        for i in range(10):
            recs = door_cycle(tracker, 1.1 + 0.1 * i, car=AT_60_DEG)
            assert only(recs).code == 'oblique_view'
        assert tracker.doors == [door] and door.strikes == 1
        assert door.state == dd.STATE_CANDIDATE
        door_cycle(tracker, 2.2, segments=DOOR_SCENE + box_behind_the_core())
        assert tracker.doors == [] and tracker.n_deleted_by_strikes == 1
        assert tracker.n_strikes == 2

    def test_f12_an_unknown_segment_inside_the_gap_makes_the_frame_undecidable(self):
        """A 0.2 m fragment ON the line in a gap between two hosts: unknown, not a jamb."""
        frag = [((WALL_X, -0.1), (WALL_X, 0.1))]
        rec = only(records(two_walls(), DOOR_SCENE + frag))
        assert rec.code == 'unknown_in_gap' and rec.detail == 'unknown_on_line'
        assert rec.cls == 'weak' and not rec.visible
        assert rec.width == pytest.approx(0.85, abs=0.04)        # still the doorway's jambs


# ==============================================================================
# The host rules (B): short line, short side, the Manhattan prior
# ==============================================================================

def rotate_about(pt, centre, deg):
    th = math.radians(deg)
    rot = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return tuple(np.asarray(centre) + rot @ (np.asarray(pt, dtype=float) - np.asarray(centre)))


class TestTheHostRules:

    def test_a_host_20_deg_off_the_rooms_axes_is_off_manhattan_and_visible(self):
        """A real wall 20 deg off the side walls' axis, with a real door in it."""
        c = (WALL_X, 0.0)
        wall = [(rotate_about(a, c, 20.0), rotate_about(b, c, 20.0))
                for a, b in wall_with_gap(-0.425, 0.425)]
        back = [(rotate_about(a, c, 20.0), rotate_about(b, c, 20.0)) for a, b in back_wall()]
        hosts = [track_through(1, *wall[0]), track_through(2, *wall[1])]
        prior = dd.ManhattanPrior()
        prior.update(hosts + SIDE_TRACKS, 0.0)
        assert prior.dominant_rad == pytest.approx(0.0, abs=1e-6)
        segs = wall + back + SIDE_SEGMENTS
        rec = [r for r in records(hosts + SIDE_TRACKS, segs, manhattan=prior)
               if set(r.parents) == {1, 2}]
        rec = only(rec)
        assert rec.code == 'off_manhattan' and rec.visible
        assert rec.host_offset_deg == pytest.approx(20.0, abs=1.0)
        # Without a prior there is no check: the same gap is a door.
        assert [r.code for r in records(hosts + SIDE_TRACKS, segs)
                if set(r.parents) == {1, 2}] == ['']

    def test_the_prior_off_never_checks_or_snaps(self):
        prior = dd.ManhattanPrior(dd.DoorParams(manhattan_prior=False))
        prior.update(SIDE_TRACKS, 0.0)
        assert prior.dominant_rad is None and prior.offset(0.3) is None

    def test_short_side_is_measured_from_the_jamb_to_its_own_tracks_far_end(self):
        """The in-track fallback: the far ends of the one track, 0.4 m left of the jamb."""
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 0.825))]
        rec = only(records([track(1, -3.0, 0.825)], segs + back_wall()))
        assert rec.code == 'short_side'
        rec = only(records([track(1, -3.0, 0.975)],
                           [((WALL_X, -3.0), (WALL_X, -0.425)),
                            ((WALL_X, 0.425), (WALL_X, 0.975))] + back_wall()))
        assert rec.code == ''

    def test_short_line_is_the_union_of_the_members_less_an_in_track_gap(self):
        """One track over 2.4 m with a 0.85 m doorway in it: 1.55 m of wall, short."""
        segs = [((WALL_X, -1.2), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 1.2))]
        assert only(records([track(1, -1.2, 1.2)], segs + back_wall())).code == 'short_line'
        segs = [((WALL_X, -1.6), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 1.6))]
        assert only(records([track(1, -1.6, 1.6)], segs + back_wall())).code == ''


# ==============================================================================
# not_boundary: the host segments themselves must not be see-through
# ==============================================================================

#: Opaque posts 0.15 m wide at the jambs; the rest of the wall line is glass.
POSTS = [((WALL_X, -0.575), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 0.575))]
GLASS = [((WALL_X, -3.0), (WALL_X, -0.575)), ((WALL_X, 0.575), (WALL_X, 3.0))]


class TestBoundary:

    def test_a_doorway_between_glass_hosts_is_not_boundary(self):
        rec = only(records(two_walls(), POSTS + back_wall(), panes=GLASS))
        assert rec.code == 'not_boundary' and rec.cls == 'strong' and rec.visible
        assert rec.boundary_hits > 100
        # The same line opaque: a door.
        assert only(records(two_walls(), POSTS + GLASS + back_wall())).code == ''

    def test_returns_on_a_confirmed_perpendicular_wall_behind_a_corner_do_not_count(self):
        """
        The right host's extent overshoots its wall by 0.3 m into a side corridor.

        The beams crossing the overshoot return from that corridor's far wall,
        0.6-1.0 m behind the line: perpendicular to the host, and confirmed,
        that wall is the room's shape, not a see-through host.
        """
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 3.0)),
                ((WALL_X, -4.0), (7.0, -4.0)), ((7.0, -4.0), (7.0, 3.0))]
        hosts = [track(1, -3.3, -0.425), track(2, 0.425, 3.0)]
        branch = track(3, 3.0, 7.0, rho=4.0, theta=-math.pi / 2, role='other')
        rec = only(records(hosts, segs))
        assert rec.code == 'not_boundary' and rec.boundary_hits > 2
        rec = only(records(hosts + [branch], segs))
        assert rec.code == '' and rec.boundary_hits == 0
        branch.state = mwt.STATE_TENTATIVE                      # only a confirmed one counts
        assert only(records(hosts + [branch], segs)).code == 'not_boundary'

    def test_returns_behind_the_line_inside_the_end_margins_do_not_count(self):
        """The same corner with the overshoot only 0.08 m: inside the 0.10 m end margin."""
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X, 0.425), (WALL_X, 3.0)),
                ((WALL_X, -4.0), (7.0, -4.0)), ((7.0, -4.0), (7.0, 3.0))]
        rec = only(records([track(1, -3.08, -0.425), track(2, 0.425, 3.0)], segs))
        assert rec.code == '' and rec.boundary_hits == 0


# ==============================================================================
# THE LEAF: an open door leaf hung on a jamb
# ==============================================================================

def leaf(deg, hinge=(WALL_X, -0.425), length=0.85, towards=1.0):
    """Return an open leaf hinged at `hinge`, `deg` from the wall line, as a segment."""
    th = math.radians(deg)
    end = (hinge[0] + length * math.sin(th), hinge[1] + towards * length * math.cos(th))
    return [(hinge, end)]


def thick_wall(thickness, half_gap=0.425):
    """Return the x = 3 wall `thickness` deep, its doorway's two reveals included."""
    t, h = thickness, half_gap
    return (wall_with_gap(-h, h)
            + [((WALL_X, -h), (WALL_X + t, -h)), ((WALL_X, h), (WALL_X + t, h)),
               ((WALL_X + t, -3.0), (WALL_X + t, -h)),
               ((WALL_X + t, h), (WALL_X + t, 3.0))])


class TestTheLeaf:

    def test_a_leaf_open_at_90_deg_on_the_right_jamb_is_a_door_with_its_side(self):
        door = only(records(one_wall(), DOOR_SCENE + leaf(90.0)))
        assert door.code == ''
        assert door.width == pytest.approx(0.85, abs=0.04)
        # Looking along the crossing normal (+x), y < 0 is the RIGHT jamb.
        assert door.leaf_side == dd.LEAF_RIGHT

    def test_a_leaf_on_the_left_jamb_says_left(self):
        left_leaf = leaf(90.0, hinge=(WALL_X, 0.425), towards=-1.0)
        assert [r.leaf_side for r in records(one_wall(), DOOR_SCENE + left_leaf)] == \
            [dd.LEAF_LEFT]

    def test_no_leaf_is_none(self):
        assert only(records(one_wall(), DOOR_SCENE)).leaf_side == dd.LEAF_NONE

    @pytest.mark.parametrize('laser', [(0.3, 0.0), (1.5, 0.0), (2.0, 0.0), (1.75, -0.6)],
                             ids=['2.7m', '1.5m', '1.0m', 'hinge_side'])
    def test_a_leaf_open_at_45_deg_is_a_door_on_every_seed(self, laser):
        """Its returns in the core are the leaf's own, left out of the core's count."""
        for seed in range(10):
            door = only(records(two_walls(), DOOR_SCENE + leaf(45.0), seed=seed,
                                car=car_at_laser(*laser)))
            assert (door.code, door.leaf_side) == ('', dd.LEAF_RIGHT), seed

    def test_a_leaf_open_at_90_deg_1_m_away_is_a_door_on_every_seed(self):
        """Its returns next to the hinge straddle the 0.05 m ON band: leaf, not on_line."""
        for seed in range(20):
            door = only(records(two_walls(), DOOR_SCENE + leaf(90.0), seed=seed,
                                car=car_at_laser(2.0, 0.0)))
            assert (door.code, door.leaf_side) == ('', dd.LEAF_RIGHT), seed

    @pytest.mark.parametrize('laser', [(1.75, 0.6), (1.75, 1.0), (2.0, 0.9)])
    def test_a_90_deg_leaf_seen_from_the_far_side_within_45_deg_is_a_door(self, laser):
        car = car_at_laser(*laser)
        for seed in range(5):
            door = only(records(two_walls(), DOOR_SCENE + leaf(90.0), seed=seed, car=car))
            assert door.view_deg <= 45.0
            assert (door.code, door.leaf_side) == ('', dd.LEAF_RIGHT), seed

    def test_a_45_deg_leaf_hiding_the_room_from_the_far_side_is_undecidable_not_a_reject(self):
        """Seen from the far side the leaf covers the whole core: few_beams, never STRONG."""
        for seed in range(5):
            rec = only(records(two_walls(), DOOR_SCENE + leaf(45.0), seed=seed,
                               car=car_at_laser(1.75, 0.6)))
            assert rec.code == 'few_beams' and rec.cls == 'weak'

    def test_a_box_face_in_the_outer_part_is_not_a_leaf(self):
        """A box 0.2 m past the line beside the jamb: its face misses the hinge."""
        box = [((WALL_X + 0.2, -0.40), (WALL_X + 0.2, -0.30)),
               ((WALL_X + 0.2, -0.30), (WALL_X + 0.35, -0.30))]
        rec = only(records(one_wall(), DOOR_SCENE + box))
        assert rec.code == 'not_a_leaf' and rec.cls == 'strong'

    def test_a_box_corner_in_the_outer_part_is_not_a_leaf(self):
        """A box turned corner-on, in the outer part: two faces, not one leaf."""
        box = [((WALL_X + 0.28, -0.41), (WALL_X + 0.12, -0.34)),
               ((WALL_X + 0.12, -0.34), (WALL_X + 0.28, -0.27))]
        assert only(records(one_wall(), DOOR_SCENE + box)).code == 'not_a_leaf'

    @pytest.mark.parametrize('laser', [(0.3, 0.0), (1.75, 0.0), (1.75, 0.6)])
    def test_a_segment_detached_from_the_jamb_is_never_a_leaf(self, laser):
        """
        On the 45 deg leaf's own line, but starting 0.2 m out: open beams before it.

        Seen head-on from 2.7 m its returns reach into the outer part (not a
        leaf); from 1.25 m they all lie in the core (behind). A STRONG reject
        either way, never a door with a leaf.
        """
        th = math.radians(45.0)
        start = (WALL_X + 0.2 * math.sin(th), -0.425 + 0.2 * math.cos(th))
        end = (WALL_X + 0.6 * math.sin(th), -0.425 + 0.6 * math.cos(th))
        rec = only(records(two_walls(), DOOR_SCENE + [(start, end)], car=car_at_laser(*laser)))
        assert rec.code in ('not_a_leaf', 'behind') and rec.visible
        assert rec.leaf_side == dd.LEAF_NONE
        if laser == (0.3, 0.0):
            assert rec.code == 'not_a_leaf'

    def test_a_pole_in_the_outer_part_is_not_a_leaf(self):
        """One pole 0.15 m behind the line, 0.1 m in from the jamb: never a leaf."""
        rec = only(records(two_walls(), DOOR_SCENE, circles=[((WALL_X + 0.15, -0.325), 0.02)]))
        assert rec.code == 'not_a_leaf'

    def test_is_leaf_wants_two_returns_on_a_straight_segment_starting_at_the_jamb(self):
        p = dd.DoorParams()
        jamb = np.array([0.0, 0.0])
        along = np.array([[0.0, 0.05], [0.0, 0.2], [0.0, 0.4], [0.0, 0.6]])
        assert dd.is_leaf(along, jamb, p)
        assert not dd.is_leaf(along[1:] + [0.0, 0.1], jamb, p)    # starts 0.3 m out
        assert not dd.is_leaf(along + [0.2, 0.0], jamb, p)          # misses the hinge
        assert not dd.is_leaf(np.array([[0.0, -0.3], [0.0, 0.3]]), jamb, p)   # through it
        assert not dd.is_leaf(np.array([[0.0, 0.05], [0.0, 1.2]]), jamb, p)   # too long
        assert not dd.is_leaf(np.array([[0.0, 0.05], [0.1, 0.3], [0.0, 0.6]]), jamb, p)
        assert not dd.is_leaf(np.array([[0.0, 0.05]]), jamb, p)     # a single return

    @pytest.mark.parametrize('laser', [(1.75, 0.0), (0.3, 0.0)], ids=['1.25m', '2.7m'])
    def test_a_doorway_in_a_0_25_m_thick_wall_is_a_door_with_no_leaf(self, laser):
        """Its two reveals, deeper than reveal_max_depth_m, are not two leaves."""
        tracks = [track(1, -3.0, -0.425), track(2, 0.425, 3.0)]
        for seed in range(5):
            door = only(records(tracks, thick_wall(0.25) + back_wall(), seed=seed,
                                car=car_at_laser(*laser)))
            assert door.code == '', seed
        door = only(records(tracks, thick_wall(0.25) + back_wall(), car=car_at_laser(1.75, 0.0)))
        assert door.leaf_side == dd.LEAF_NONE

    def test_a_doorway_in_a_0_15_m_wall_shows_two_reveals_and_no_leaf(self):
        door = only(records(two_walls(), thick_wall(0.15) + back_wall(),
                            car=car_at_laser(1.75, 0.0)))
        assert (door.code, door.leaf_side) == ('', dd.LEAF_NONE)

    def test_the_leaf_side_is_mirrored_for_a_sighting_from_the_other_side(self):
        tracker = dd.DoorTracker()
        cand = door_candidate()
        cand.leaf_side = dd.LEAF_RIGHT
        tracker.step([cand], ORIGIN, 0.0)
        other = door_candidate()
        other.normal = -other.normal
        other.leaf_side = dd.LEAF_LEFT
        doors = tracker.step([other], ORIGIN, 0.1)
        assert doors[0].leaf_side == dd.LEAF_RIGHT


# ==============================================================================
# The lifecycle (DoorTracker)
# ==============================================================================

PERSON = (WALL_X, 0.0, 0.2)     # (x, y, half width): standing in the doorway
#: Two doorways in the x = 3 wall, 0.575 m of wall between them, as the
#: collinear gap rule tracks them: three pieces.
TWO_DOORWAYS = [((WALL_X, -3.0), (WALL_X, -1.825)), ((WALL_X, -1.0), (WALL_X, -0.425)),
                ((WALL_X, 0.425), (WALL_X, 3.0))] + back_wall()
THREE_PIECES = [track(1, -3.0, -1.825), track(2, -1.0, -0.425), track(3, 0.425, 3.0)]


def door_cycle(tracker, stamp, person=False, car=ORIGIN, segments=DOOR_SCENE, tracks=None):
    """One door cycle as the node runs it: the records, then the tracker on the same scan."""
    objects = (PERSON,) if person else ()
    circles = (((PERSON[0], PERSON[1]), PERSON[2]),) if person else ()
    scan = scan_of(raycast(car, segments, circles, seed=int(round(stamp * 10))), objects)
    recs = dd.door_candidates(tracks or two_walls(), car, scan)
    tracker.step(recs, car, stamp, persons=list(objects), scan=scan)
    return recs


def confirmed_tracker(tracks=None):
    """Return a tracker holding the doorway CONFIRMED: seen from y = 0 and y = 0.5."""
    tracker = dd.DoorTracker()
    for i, y in enumerate((0.0, 0.0, 0.5)):
        door_cycle(tracker, 0.1 * i, car=Pose2D(0.0, y, 0.0), tracks=tracks)
    assert [d.state for d in tracker.doors] == [dd.STATE_CONFIRMED]
    return tracker


def record_at(car, width, code='', visible=True, centre=(WALL_X, 0.0)):
    """Return a record of a door in the x = 3 wall at `centre` (odom), in the car's base_link."""
    j0, j1 = (np.array([centre[0], centre[1] - width / 2]),
              np.array([centre[0], centre[1] + width / 2]))
    bl = car.inverse()
    j0, j1 = bl.transform_points(np.vstack([j0, j1]))
    normal = bl.R @ np.array([1.0, 0.0])
    return dd.DoorCandidate(j0, j1, normal, (1, 2), 40, 30, code=code,
                            cls=dd.code_class(code), visible=visible)


def door_candidate():
    return only(records(one_wall(), DOOR_SCENE))


class TestLifecycle:

    def test_a_door_is_born_a_candidate(self):
        tracker = dd.DoorTracker()
        door_cycle(tracker, 0.0)
        (door,) = tracker.doors
        assert door.state == dd.STATE_CANDIDATE and door.hits == 1 and door.strikes == 0
        assert door.born == door.last_positive == 0.0
        assert len(door.evidence) == 1 and door.evidence[0][:2] == (0.0, 0.0)

    def test_two_viewpoints_closer_than_0_3_m_are_one(self):
        tracker = dd.DoorTracker()
        for i, y in enumerate((0.0, 0.1, 0.2, 0.25)):
            door_cycle(tracker, 0.1 * i, car=Pose2D(0.0, y, 0.0))
        (door,) = tracker.doors
        assert door.n_viewpoints == 1 and door.state == dd.STATE_CANDIDATE
        door_cycle(tracker, 0.5, car=Pose2D(0.0, 0.35, 0.0))
        assert door.n_viewpoints == 2 and door.state == dd.STATE_CONFIRMED

    def test_viewpoints_are_greedy_clusters_of_car_positions(self):
        assert dd.viewpoints([], 0.3) == 0
        assert dd.viewpoints([(0.0, 0.0), (0.2, 0.0), (0.4, 0.0)], 0.3) == 2
        assert dd.viewpoints([(0.0, 0.0), (0.0, 0.31), (0.0, 0.62)], 0.3) == 3

    def test_weak_and_geometry_records_never_touch_a_door(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        before = (door.hits, door.last_positive, len(door.evidence), door.strikes)
        for i, code in enumerate(('oblique_view', 'few_beams', 'jamb_occluded',
                                  'unknown_in_gap', 'width', 'not_collinear',
                                  'untracked_jamb')):
            tracker.step([record_at(ORIGIN, 0.85, code=code)], ORIGIN, 1.0 + 0.1 * i)
        assert tracker.doors == [door] and door.state == dd.STATE_CONFIRMED
        assert (door.hits, door.last_positive, len(door.evidence), door.strikes) == before

    def test_a_strong_record_that_is_not_visible_never_strikes(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        for i in range(3):
            tracker.step([record_at(ORIGIN, 0.85, code='behind', visible=False)], ORIGIN,
                         1.0 + 0.1 * i)
        assert door.strikes == 0 and door.state == dd.STATE_CONFIRMED

    def test_a_door_seen_positive_this_cycle_is_not_struck(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        tracker.step([record_at(ORIGIN, 0.85), record_at(ORIGIN, 0.86, code='behind')],
                     ORIGIN, 1.0)
        assert door.strikes == 0 and door.state == dd.STATE_CONFIRMED

    def test_a_strike_needs_the_record_to_associate_with_the_door(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        tracker.step([record_at(ORIGIN, 0.85, code='behind', centre=(WALL_X, 1.5))],
                     ORIGIN, 1.0)
        assert door.strikes == 0 and tracker.n_strikes == 0

    def test_a_struck_door_needs_two_new_viewpoints_to_confirm_again(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        tracker.step([record_at(ORIGIN, 0.85, code='behind')], ORIGIN, 1.0)
        assert door.state == dd.STATE_CANDIDATE and door.n_viewpoints == 0
        tracker.step([record_at(ORIGIN, 0.85)], ORIGIN, 1.1)
        assert door.state == dd.STATE_CANDIDATE
        car = Pose2D(0.0, 0.5, 0.0)
        tracker.step([record_at(car, 0.85)], car, 1.2)
        assert door.state == dd.STATE_CONFIRMED and door.strikes == 0
        assert tracker.n_confirmed == 2

    def test_a_candidate_unseen_for_10_s_is_dropped(self):
        tracker = dd.DoorTracker()
        door_cycle(tracker, 0.0)
        assert tracker.step([], ORIGIN, 10.0) != []
        assert tracker.step([], ORIGIN, 10.05) == []

    def test_a_confirmed_door_unseen_for_60_s_is_dropped(self):
        tracker = confirmed_tracker()
        last = tracker.doors[0].last_positive
        assert tracker.step([], ORIGIN, last + 59.9) != []
        assert tracker.step([], ORIGIN, last + 60.1) == []

    def test_a_person_in_a_confirmed_door_keeps_it_alive_and_occupied(self):
        tracker = confirmed_tracker()
        (door,) = tracker.doors
        hits = door.hits
        # 100 s of a person standing in it, seen every 25 s: past the 60 s age.
        for stamp in (25.0, 50.0, 75.0, 100.0):
            recs = door_cycle(tracker, stamp, person=True)
            assert tracker.doors == [door] and door.occupied is True
            assert door.state == dd.STATE_CONFIRMED
            # The person is an object in the gap: undecidable, never a sighting.
            assert only(recs).code == 'unknown_in_gap'
        assert door.hits == hits
        # ...and once they step out the beams see through it again.
        door_cycle(tracker, 100.1)
        assert tracker.doors == [door] and door.occupied is False and door.hits == hits + 1
        # Unseen and unheld for 60 s after that: gone.
        assert tracker.step([], ORIGIN, 160.2) == []

    def test_a_person_in_a_gap_that_is_not_yet_a_door_never_makes_one(self):
        tracker = dd.DoorTracker()
        for i in range(30):
            recs = door_cycle(tracker, 0.1 * i, person=True)
            assert tracker.doors == []
            assert only(recs).detail in ('object', 'blocked')

    def test_a_person_does_not_hold_a_candidate(self):
        tracker = dd.DoorTracker()
        door_cycle(tracker, 0.0)
        for stamp in (5.0, 10.0):
            door_cycle(tracker, stamp, person=True)
        assert tracker.doors[0].occupied is False
        door_cycle(tracker, 10.2, person=True)
        assert tracker.doors == []

    def test_a_person_beside_the_door_does_not_occupy_it(self):
        tracker = confirmed_tracker()
        door = tracker.doors[0]
        assert not tracker.person_inside(door, [(WALL_X, 1.5, 0.2)])
        assert not tracker.person_inside(door, [(1.5, 0.0, 0.2)])
        assert tracker.person_inside(door, [(WALL_X - 0.3, 0.1, 0.2)])

    def test_the_door_is_kept_in_odom_while_the_car_moves(self):
        tracker = confirmed_tracker()
        door_id = tracker.doors[0].id
        car = Pose2D(0.8, 0.3, math.radians(-10.0))
        door_cycle(tracker, 0.3, car=car)
        assert [d.id for d in tracker.doors] == [door_id]
        assert tracker.doors[0].centre == pytest.approx([WALL_X, 0.0], abs=0.04)
        assert tracker.doors[0].state == dd.STATE_CONFIRMED

    def test_last_candidates_is_this_cycles_every_record(self):
        tracker = dd.DoorTracker()
        recs = door_cycle(tracker, 0.0, segments=DOOR_SCENE + box_behind_the_core())
        assert tracker.last_candidates == recs and tracker.doors == []

    def test_square_looks_is_an_optional_extra_requirement(self):
        """With door_square_looks 1, a door seen only obliquely stays a candidate."""
        p = dd.DoorParams(square_looks=1)
        tracker = dd.DoorTracker(p)
        cars = [car_at_laser(1.5, -1.0), car_at_laser(1.5, -0.5)]
        for i, car in enumerate(cars):
            scan = scan_of(raycast(car, DOOR_SCENE, seed=i))
            tracker.step(dd.door_candidates(two_walls(), car, scan, p), car, 0.1 * i, scan=scan)
        (door,) = tracker.doors
        assert door.n_viewpoints == 2 and door.square_looks == 0
        assert door.state == dd.STATE_CANDIDATE
        scan = scan_of(raycast(ORIGIN, DOOR_SCENE, seed=5))
        tracker.step(dd.door_candidates(two_walls(), ORIGIN, scan, p), ORIGIN, 0.5, scan=scan)
        assert door.square_looks == 1 and door.state == dd.STATE_CONFIRMED

    def test_the_gap_evidence_counts_the_square_on_beams(self):
        ev = dd.gap_evidence((WALL_X, -0.425), (WALL_X, 0.425),
                             scan_of(raycast(ORIGIN, DOOR_SCENE)))
        assert ev['on'] == 0 and ev['n'] == ev['open'] > 0
        # 6 deg of beams around the normal at 0.25 deg, all inside the core.
        assert ev['square'] == ev['square_open'] >= 16
        assert dd.is_square_look(ev, dd.DoorParams())


class TestIdentity:

    def test_a_reborn_parent_wall_keeps_the_same_door_id(self):
        tracker = confirmed_tracker(two_walls(ids=(1, 2)))
        door_id = tracker.doors[0].id
        assert set(tracker.doors[0].parents) == {1, 2}
        # The tracker lost both walls and re-spawned them: same geometry, new ids.
        door_cycle(tracker, 0.3, tracks=two_walls(ids=(17, 18)))
        assert [d.id for d in tracker.doors] == [door_id]
        assert set(tracker.doors[0].parents) == {17, 18}
        # ...and the same when the doorway is now inside one merged track.
        door_cycle(tracker, 0.4, tracks=one_wall(track_id=40))
        assert [d.id for d in tracker.doors] == [door_id]
        assert tracker.doors[0].parents == (40, 40)

    def test_a_second_doorway_along_the_wall_is_a_second_door(self):
        tracker = dd.DoorTracker()
        for i in range(3):
            door_cycle(tracker, 0.1 * i, segments=TWO_DOORWAYS, tracks=THREE_PIECES)
        assert len(tracker.doors) == 2
        assert len({d.id for d in tracker.doors}) == 2

    def test_two_doorways_inside_one_track_see_each_other_as_a_see_through_host(self):
        """
        The in-track fallback over two doorways: each is the other's open host stretch.

        THE COLLINEAR GAP RULE splits such a wall into three tracks (above);
        one track over both doorways is no boundary on either side.
        """
        recs = records(one_wall(), TWO_DOORWAYS)
        assert len(recs) == 2 and codes(recs) == {'not_boundary'}

    def test_door_ids_are_never_reused(self):
        tracker = confirmed_tracker()
        first = tracker.doors[0].id
        assert tracker.step([], ORIGIN, 100.0) == []
        door_cycle(tracker, 100.1)
        assert tracker.doors[0].id > first


# ==============================================================================
# Stereo: support, the jamb height, the glass veto
# ==============================================================================

FX, CX, CY, W, H = 262.0, 320.0, 180.0, 640, 360
# base_link -> optical: optical z = base x, optical x = -base y, optical y = -base z.
ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CAM_T = np.array([0.12, 0.0, 0.3])


def render_planes(planes):
    """
    Return (labels, depth_mm) of the floor and planes x = const seen by a forward camera.

    Each plane is (x, y_lo, y_hi, z_lo, z_hi, class, ranged); the nearest wins.
    An unranged one keeps its class with depth 0 (invalid).
    """
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d_opt = np.stack([(u - CX) / FX, (v - CY) / FX, np.ones_like(u, dtype=float)], -1)
    d = d_opt @ ROT.T                              # base_link ray per unit optical z
    best = np.full((H, W), np.inf)
    out = np.full((H, W), np.inf)
    cls = np.full((H, W), scene_mask.CLASS_INVALID, dtype=np.uint8)
    with np.errstate(divide='ignore', invalid='ignore'):
        s_floor = -CAM_T[2] / d[..., 2]
        hit = (d[..., 2] < 0) & (s_floor > 0)
        best = np.where(hit, s_floor, best)
        out = np.where(hit, s_floor, out)
        cls[hit] = scene_mask.CLASS_FLOOR
        for x, y_lo, y_hi, z_lo, z_hi, klass, ranged in planes:
            s = (x - CAM_T[0]) / d[..., 0]
            y = CAM_T[1] + s * d[..., 1]
            z = CAM_T[2] + s * d[..., 2]
            ok = (s > 0) & (y >= y_lo) & (y <= y_hi) & (z >= z_lo) & (z <= z_hi) & (s < best)
            best = np.where(ok, s, best)
            out = np.where(ok, s if ranged else 0.0, out)
            cls[ok] = klass
    depth = np.where(np.isfinite(out), np.round(out * 1000.0), 0).astype(np.uint16)
    return cls, depth


def render(gap=True, pane_depth=True):
    """
    Return (labels, depth_mm) of the door scene seen by a forward camera.

    gap False closes the doorway with a surface ON the wall line (a pane the
    stereo ranged). pane_depth False leaves that surface unranged: depth 0,
    and so INVALID in the depth-derived class mask (WALL implies a depth).
    """
    wall = scene_mask.CLASS_WALL
    planes = [(BACK_X, -50.0, 50.0, 0.0, 2.5, wall, True),
              (WALL_X, -50.0, -0.425, 0.0, 2.5, wall, True),
              (WALL_X, 0.425, 50.0, 0.0, 2.5, wall, True)]
    if not gap:
        planes.append((WALL_X, -0.425, 0.425, 0.0, 2.5,
                       wall if pane_depth else scene_mask.CLASS_INVALID, pane_depth))
    return render_planes(planes)


#: The desks of F8 (DESKS, 0.75 m high) and the room's wall 3 m behind them.
DESK_PLANES = [(BACK_X, -50.0, 50.0, 0.0, 2.5, scene_mask.CLASS_WALL, True),
               (WALL_X, -1.6, -0.4, 0.0, 0.75, scene_mask.CLASS_WALL, True),
               (WALL_X, 0.4, 1.6, 0.0, 0.75, scene_mask.CLASS_WALL, True)]


def frame(labels_depth):
    labels, depth = labels_depth
    return dd.StereoFrame(labels, depth, FX, FX, CX, CY, ROT, CAM_T, depth_scale=0.001)


class TestStereoSupport:

    def test_an_open_doorway_in_the_mask_supports_the_door(self):
        labels, depth = render(gap=True)
        support, detail = dd.stereo_gap_support(door_candidate(), labels, depth, FX, FX, CX,
                                                CY, ROT, CAM_T, depth_scale=0.001)
        assert support is True
        assert detail['wall_share'] <= 0.2 and detail['floor_share'] >= 0.5

    def test_a_wall_in_the_mask_where_the_lidar_saw_a_gap_does_not(self):
        labels, depth = render(gap=False)
        support, detail = dd.stereo_gap_support(door_candidate(), labels, depth, FX, FX, CX,
                                                CY, ROT, CAM_T, depth_scale=0.001)
        assert support is False
        assert detail['wall_share'] > 0.5

    def test_no_image_is_no_opinion(self):
        labels = np.zeros((H, W), dtype=np.uint8)
        depth = np.zeros((H, W), dtype=np.uint16)
        support, _detail = dd.stereo_gap_support(door_candidate(), labels, depth, FX, FX, CX,
                                                 CY, ROT, CAM_T, depth_scale=0.001)
        assert support is None

    def test_a_door_with_a_stereo_frame_carries_its_support(self):
        door = only(records(two_walls(), DOOR_SCENE, stereo=frame(render(gap=True))))
        assert door.code == '' and door.stereo_support is True

    def test_stereo_support_raises_the_confidence_and_is_never_required(self):
        lidar_only = dd.DoorTracker()
        both = dd.DoorTracker()
        for i, y in enumerate((0.0, 0.5)):
            car = Pose2D(0.0, y, 0.0)
            lidar_only.step([record_at(car, 0.85)], car, 0.1 * i)
            cand = record_at(car, 0.85)
            cand.stereo_support = True
            both.step([cand], car, 0.1 * i)
        assert lidar_only.doors[0].state == dd.STATE_CONFIRMED
        assert lidar_only.doors[0].stereo_support is False
        assert both.doors[0].stereo_support is True
        assert both.doors[0].confidence > lidar_only.doors[0].confidence


class TestStereoJambHeight:

    def test_the_jambs_of_a_real_door_reach_full_height(self):
        heights = dd.stereo_jamb_heights(door_candidate(), frame(render(gap=True)))
        assert heights['decision'] == 'tall' and heights['height_unknown'] is False
        for key in ('jamb0', 'jamb1'):
            assert heights[key]['valid'] and heights[key]['wall_top'] >= 1.8

    def test_a_door_with_tall_jambs_is_kept_with_its_height_known(self):
        door = only(records(two_walls(), DOOR_SCENE, stereo=frame(render(gap=True))))
        assert door.code == '' and door.jamb_height_unknown is False
        assert min(door.jamb_tops) >= 1.8

    def test_desk_tops_with_the_room_seen_above_them_are_furniture(self):
        cand = only(records(DESK_TRACKS, DESKS + back_wall()))
        heights = dd.stereo_jamb_heights(cand, frame(render_planes(DESK_PLANES)))
        assert heights['decision'] == 'furniture'
        for key in ('jamb0', 'jamb1'):
            side = heights[key]
            assert side['wall_top'] < 1.3 and side['seen_top'] >= side['wall_top'] + 0.3

    def test_a_jamb_whose_top_is_out_of_view_is_no_decision_and_unknown(self):
        """1.5 m from the wall the camera sees it only up to ~1.45 m: clipped, no decision."""
        car = car_at_laser(WALL_X - 1.5, 0.0)
        planes = [(BACK_X - car.x, -50.0, 50.0, 0.0, 2.5, scene_mask.CLASS_WALL, True),
                  (WALL_X - car.x, -50.0, -0.425, 0.0, 2.5, scene_mask.CLASS_WALL, True),
                  (WALL_X - car.x, 0.425, 50.0, 0.0, 2.5, scene_mask.CLASS_WALL, True)]
        door = only(records(two_walls(), DOOR_SCENE, car=car, stereo=frame(render_planes(planes))))
        assert door.code == '' and door.jamb_height_unknown is True
        assert all(top is not None and 1.3 <= top < 1.8 for top in door.jamb_tops)

    def test_without_depth_there_is_no_decision(self):
        labels, depth = render(gap=True)
        heights = dd.stereo_jamb_heights(door_candidate(), frame((labels, depth * 0)))
        assert heights['decision'] == 'unknown' and heights['height_unknown'] is True
        assert not heights['jamb0']['valid'] and heights['jamb0']['wall_top'] is None

    def test_the_check_off_leaves_desks_a_door(self):
        p = dd.DoorParams(jamb_height_check=False)
        rec = only(records(DESK_TRACKS, DESKS + back_wall(), p=p,
                           stereo=frame(render_planes(DESK_PLANES))))
        assert rec.code == '' and rec.jamb_height_unknown is True

    def test_the_stereo_frame_is_fetched_once_and_only_when_a_gap_reaches_it(self):
        calls = []

        def fetch():
            calls.append(1)
            return frame(render(gap=True))

        recs = records(THREE_PIECES, TWO_DOORWAYS, stereo=fetch)
        assert len(doors_in(recs)) == 2 and len(calls) == 1
        calls.clear()
        recs = records(one_wall(), DOOR_SCENE + box_behind_the_core(), stereo=fetch)
        assert doors_in(recs) == [] and calls == []
        assert only(records(two_walls(), DOOR_SCENE, stereo=lambda: None)).code == ''


def glass_check(**render_kw):
    labels, depth = render(**render_kw)
    return dd.stereo_glass_check(door_candidate(), labels, depth, FX, FX, CX, CY, ROT,
                                 CAM_T, depth_scale=0.001)


VETO = dd.DoorParams(stereo_glass_veto=True)


class TestGlassVeto:

    def test_it_is_off_by_default(self):
        assert dd.DoorParams().stereo_glass_veto is False

    def test_a_real_door_with_a_far_wall_behind_is_kept(self):
        """The room behind is WALL in the mask, but beyond the line: it never counts."""
        veto, suspect, detail = glass_check(gap=True)
        assert veto is False and suspect is False
        assert detail['on_line_frac'] < 0.1
        door = only(records(two_walls(), DOOR_SCENE, p=VETO, stereo=frame(render(gap=True))))
        assert door.code == '' and door.glass_suspect is False

    def test_glass_on_the_line_with_valid_depth_is_stereo_wall_only_with_the_veto_on(self):
        pane = frame(render(gap=False))
        rec = only(records(two_walls(), DOOR_SCENE, p=VETO, stereo=pane))
        assert rec.code == 'stereo_wall' and rec.cls == 'strong' and rec.visible
        door = only(records(two_walls(), DOOR_SCENE, stereo=pane))
        assert door.code == '' and door.glass_suspect is None
        veto, suspect, detail = glass_check(gap=False)
        assert veto is True and suspect is False and detail['on_line_frac'] >= 0.5

    def test_glass_with_invalid_depth_is_kept_and_flagged_glass_suspect(self):
        veto, suspect, detail = glass_check(gap=False, pane_depth=False)
        assert veto is False and suspect is True
        assert detail['valid_frac'] < 0.5
        door = only(records(two_walls(), DOOR_SCENE, p=VETO,
                            stereo=frame(render(gap=False, pane_depth=False))))
        assert door.code == '' and door.glass_suspect is True

    def test_no_camera_view_is_no_opinion_either_way(self):
        labels = np.zeros((H, W), dtype=np.uint8)
        depth = np.zeros((H, W), dtype=np.uint16)
        cand = door_candidate()
        cand.jamb0, cand.jamb1 = -cand.jamb0, -cand.jamb1       # behind the camera
        veto, suspect, _ = dd.stereo_glass_check(cand, labels, depth, FX, FX, CX, CY, ROT,
                                                 CAM_T, depth_scale=0.001)
        assert veto is None and suspect is None

    def test_glass_suspect_rides_on_the_door(self):
        tracker = dd.DoorTracker()
        cand = door_candidate()
        cand.glass_suspect = True
        doors = tracker.step([cand], ORIGIN, 0.0)
        assert doors[0].glass_suspect is True

    def test_a_cycle_out_of_the_cameras_view_keeps_the_glass_flag(self):
        tracker = dd.DoorTracker()
        cand = door_candidate()
        cand.glass_suspect = True
        tracker.step([cand], ORIGIN, 0.0)
        cand = door_candidate()
        cand.glass_suspect = None
        (door,) = tracker.step([cand], ORIGIN, 0.1)
        assert door.glass_suspect is True
        cand = door_candidate()
        cand.glass_suspect = False
        (door,) = tracker.step([cand], ORIGIN, 0.2)
        assert door.glass_suspect is False


# ==============================================================================
# The Manhattan prior
# ==============================================================================

def wall_track(track_id, theta_deg, length, state=mwt.STATE_CONFIRMED):
    return track(track_id, 0.0, length, rho=2.0, theta=math.radians(theta_deg), state=state)


class TestManhattanPrior:

    def test_the_dominant_direction_is_the_length_weighted_peak_mod_90_deg(self):
        prior = dd.ManhattanPrior()
        assert prior.dominant_rad is None and prior.offset(0.0) is None
        prior.update([wall_track(1, 10.0, 3.0), wall_track(2, 100.0, 2.0),
                      wall_track(3, -80.0, 4.0), wall_track(4, 55.0, 2.5)], 0.0)
        assert math.degrees(prior.dominant_rad) == pytest.approx(10.0, abs=0.01)
        assert prior.n_walls == 4 and prior.weight == pytest.approx(9.0)
        assert prior.strength == pytest.approx(9.0 / 11.5, abs=1e-6)

    def test_one_off_axis_wall_does_not_move_the_dominant_direction(self):
        prior = dd.ManhattanPrior()
        prior.update([wall_track(1, 0.0, 3.0), wall_track(2, 90.0, 4.0),
                      wall_track(3, 30.0, 5.0)], 0.0)
        assert math.degrees(prior.dominant_rad) == pytest.approx(0.0, abs=0.01)
        assert prior.weight == pytest.approx(7.0)

    def test_a_direction_across_the_0_90_deg_seam_is_one_direction(self):
        prior = dd.ManhattanPrior()
        prior.update([wall_track(1, 89.0, 3.0), wall_track(2, 1.0, 3.0),
                      wall_track(3, 30.0, 4.0)], 0.0)
        assert prior.offset(0.0) == pytest.approx(0.0, abs=1e-6)

    def test_the_prior_follows_a_new_direction_with_time_constant_tau(self):
        prior = dd.ManhattanPrior()
        prior.update([wall_track(1, 0.0, 3.0)], 0.0)
        prior.update([wall_track(1, 8.0, 3.0)], 2.0)                    # dt = tau
        alpha = 1.0 - math.exp(-1.0)
        z = (1.0 - alpha) * 1.0 + alpha * complex(math.cos(math.radians(32.0)),
                                                  math.sin(math.radians(32.0)))
        expected = math.degrees(math.atan2(z.imag, z.real) / 4.0)
        assert math.degrees(prior.dominant_rad) == pytest.approx(expected, abs=1e-6)
        assert 4.0 < expected < 8.0
        prior.update([wall_track(1, 8.0, 3.0)], 2.0)                    # dt = 0: no change
        assert math.degrees(prior.dominant_rad) == pytest.approx(expected, abs=1e-6)

    def test_short_and_unconfirmed_walls_do_not_vote(self):
        prior = dd.ManhattanPrior()
        prior.update([wall_track(1, 20.0, 1.4),
                      wall_track(2, 30.0, 4.0, state=mwt.STATE_TENTATIVE),
                      wall_track(3, 40.0, 4.0, state=mwt.STATE_COASTING)], 0.0)
        assert prior.dominant_rad is None and prior.n_walls == 0
        prior.update([wall_track(1, 5.0, 3.0)], 0.1)
        prior.update([wall_track(1, 20.0, 1.4)], 0.2)                   # nothing eligible
        assert math.degrees(prior.dominant_rad) == pytest.approx(5.0, abs=0.01)

    def test_the_offset_is_signed_and_wrapped_into_plus_minus_45_deg(self):
        prior = dd.ManhattanPrior()
        prior.update([wall_track(1, 10.0, 3.0)], 0.0)
        for theta, offset in ((13.0, 3.0), (7.0, -3.0), (190.0, 0.0), (-75.0, 5.0),
                              (54.0, 44.0), (56.0, -44.0)):
            assert math.degrees(prior.offset(math.radians(theta))) == pytest.approx(
                offset, abs=1e-6), theta

    def test_to_debug_reports_direction_strength_weight_and_count(self):
        prior = dd.ManhattanPrior()
        assert prior.to_debug() == {'dominant_deg': None, 'strength': 0.0, 'weight': 0.0,
                                    'n_walls': 0}
        prior.update([wall_track(1, -90.0, 3.0), wall_track(2, 0.0, 2.0)], 0.0)
        debug = prior.to_debug()
        assert debug['dominant_deg'] == pytest.approx(0.0, abs=0.01)
        assert debug['strength'] == pytest.approx(1.0) and debug['n_walls'] == 2
        assert debug['weight'] == pytest.approx(5.0)


# ==============================================================================
# End to end: the real extraction and tracker, driving past a side wall
# ==============================================================================

#: A corridor along x: its left wall at y = 1.2 with the doorway centred on
#: x = 0, its right wall at y = -1.2, the office behind the doorway 3 m deep.
SIDE_Y = 1.2


def corridor(half_gap):
    return [((-6.0, SIDE_Y), (-half_gap, SIDE_Y)), ((half_gap, SIDE_Y), (6.0, SIDE_Y)),
            ((-6.0, -SIDE_Y), (6.0, -SIDE_Y)), ((-9.0, SIDE_Y + 3.0), (9.0, SIDE_Y + 3.0))]


def drive_past(segments, xs=np.arange(-2.5, -0.45, 0.1)):
    """
    Drive along the corridor, the real extraction, tracker and prior every 0.1 s.

    The car stops before it is level with the doorway: from there on the
    tracker can no longer see the gap open and may join the two pieces.
    """
    tracker = mwt.MultiWallTracker()
    tracker.laser_xy = LASER[:2]
    ep = mwt.extraction_params(backend='python')
    doors = dd.DoorTracker()
    prior = dd.ManhattanPrior()
    for i, x in enumerate(xs):
        car = Pose2D(float(x), 0.0, 0.0)
        ranges = raycast(car, segments, seed=i)
        lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=ep)
        obs = [wall_fusion.Observation(mwt.line_to_wall_line(ln), 0.0, 2.5, False,
                                       wall_fusion.SOURCE_LIDAR, 0.8) for ln in lines]
        scan = scan_of(ranges)
        tracker.step(obs, 0.1 * i, car, scan=scan)
        prior.update(tracker.tracks, 0.1 * i)
        doors.step(dd.door_candidates(tracker.tracks, car, scan, manhattan=prior), car,
                   0.1 * i, scan=scan)
    return tracker, doors, prior


def walls_on_the_side(tracker):
    """Return the confirmed tracks on the y = 1.2 wall line, ordered along it."""
    on = [t for t in tracker.tracks if t.ever_confirmed and abs(math.sin(t.theta)) > 0.99
          and abs(t.rho / math.sin(t.theta) - SIDE_Y) < 0.1]
    return sorted(on, key=lambda t: min(t.endpoints()[0][0], t.endpoints()[1][0]))


class TestThroughTheRealTracker:

    def test_a_doorway_in_a_side_wall_is_confirmed_driving_past(self):
        tracker, doors, prior = drive_past(corridor(0.425))
        (door,) = doors.doors
        assert door.state == dd.STATE_CONFIRMED and door.n_viewpoints >= 2
        assert door.width == pytest.approx(0.85, abs=0.04)
        assert door.centre == pytest.approx([0.0, SIDE_Y], abs=0.04)
        assert set(door.parents) <= {t.id for t in tracker.tracks}
        assert {t.role for t in walls_on_the_side(tracker)} == {'left'}
        assert math.degrees(prior.offset(0.0)) == pytest.approx(0.0, abs=0.5)


class TestTheCollinearGapRule:

    def test_a_0_5_m_gap_is_merged_into_one_wall_and_is_no_door(self):
        tracker, doors, _prior = drive_past(corridor(0.25))
        assert len(walls_on_the_side(tracker)) == 1
        assert doors.doors == [] and doors.n_confirmed == 0

    def test_a_0_85_m_gap_stays_two_walls_with_one_confirmed_door_between_them(self):
        tracker, doors, _prior = drive_past(corridor(0.425))
        walls = walls_on_the_side(tracker)
        assert len(walls) == 2
        (door,) = doors.doors
        assert door.state == dd.STATE_CONFIRMED
        assert set(door.parents) == {walls[0].id, walls[1].id}

    def test_a_1_4_m_gap_stays_two_walls_and_is_no_door(self):
        tracker, doors, _prior = drive_past(corridor(0.70))
        assert len(walls_on_the_side(tracker)) == 2
        assert doors.doors == [] and doors.n_confirmed == 0
        assert codes(doors.last_candidates) == {'width'}

    def test_the_extractions_merge_reads_the_same_number(self):
        """The extraction alone: 0.5 m merged, 0.85 m not (one source of truth)."""
        p = mwt.extraction_params(backend='python')
        for half, n in ((0.25, 1), (0.425, 2)):
            ranges = raycast(ORIGIN, wall_with_gap(-half, half) + back_wall())
            lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=p)
            assert len([ln for ln in lines if abs(ln.rho - WALL_X) < 0.15]) == n

    def test_the_door_search_inside_one_track_is_kept_as_a_fallback(self):
        """A track that spans the doorway anyway (e.g. joined before the rule)."""
        door = only(records(one_wall(), DOOR_SCENE))
        assert door.code == '' and door.parents == (1, 1)
