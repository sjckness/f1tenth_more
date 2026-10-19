"""
door_detection on synthetic scans: 270 deg / 0.25 deg, range noise sigma 0.01 m.

The world frame is odom and the car sits at its origin unless a test moves it,
so base_link and odom coincide; the laser is at LASER = (0.305, 0, 0) in
base_link, as on the car. The wall tracks are built directly (the tracker has
its own suite) except in TestThroughTheRealTracker, which runs the real
extraction and MultiWallTracker on the same scans.

The scene most tests use: a wall square to the car at x = 3 m, a doorway in
it centred on y = 0, and the next room's wall at x = 6 m.
"""

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


def raycast(car, segments, circles=(), noise=0.01, seed=0):
    """Ranges of one scan from the laser on car (Pose2D) against segments and circles."""
    rng = np.random.default_rng(seed)
    laser = car.compose(Pose2D(*LASER))
    origin = laser.t
    ang = laser.theta + ANGLE_MIN + np.arange(N_BEAMS) * INC
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(N_BEAMS, np.inf)
    for (ax, ay), (bx, by) in segments:
        e = np.array([bx - ax, by - ay], dtype=float)
        w = np.array([ax, ay], dtype=float) - origin
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
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


def track(track_id, t0, t1, rho=WALL_X, theta=0.0, state=mwt.STATE_CONFIRMED):
    """
    Build a wall track on rho = x cos(theta) + y sin(theta), extent t0..t1 along it.

    At theta 0 the tangent is +y, so t is simply y on the wall x = rho.
    """
    return mwt.Track(
        id=track_id, x=np.array([rho, theta], dtype=float), P=np.eye(2) * 1e-3,
        t0=t0, t1=t1, born=0.0, last_seen=0.0,
        ever_confirmed=state != mwt.STATE_TENTATIVE, state=state)


def one_wall(track_id=1):
    """Return the x = 3 wall as ONE track across the doorway (the extraction's merge)."""
    return [track(track_id, -3.0, 3.0)]


def two_walls(ids=(1, 2), half_gap=0.425):
    """Return the x = 3 wall as two tracks either side of the doorway."""
    return [track(ids[0], -3.0, -half_gap), track(ids[1], half_gap, 3.0)]


def candidates(tracks, segments, circles=(), objects=(), car=ORIGIN, p=None, seed=0):
    rejects = []
    scan = scan_of(raycast(car, segments, circles, seed=seed), objects)
    return dd.door_candidates(tracks, car, scan, p, rejects), rejects


def codes(rejects):
    return {code for code, _ids, _width in rejects}


# ==============================================================================
# 1-2: a doorway, and gaps of the wrong width
# ==============================================================================

class TestTheDoorway:

    @pytest.mark.parametrize('tracks', [one_wall(), two_walls()],
                             ids=['inside_one_track', 'between_two_tracks'])
    def test_a_0_85_m_gap_seen_through_to_3_m_behind_is_a_door(self, tracks):
        cands, _rejects = candidates(tracks, wall_with_gap(-0.425, 0.425) + back_wall())
        assert len(cands) == 1
        door = cands[0]
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

    def test_no_return_at_all_through_the_gap_also_counts_as_open(self):
        cands, _rejects = candidates(one_wall(), wall_with_gap(-0.425, 0.425))
        assert len(cands) == 1

    @pytest.mark.parametrize('half_gap', [0.25, 0.70], ids=['0.5m', '1.4m'])
    def test_a_gap_of_the_wrong_width_is_not_a_door(self, half_gap):
        cands, rejects = candidates(two_walls(half_gap=half_gap),
                                    wall_with_gap(-half_gap, half_gap) + back_wall())
        assert cands == []
        assert codes(rejects) == {'width'}

    def test_a_wall_with_no_gap_has_no_door_and_nothing_to_reject(self):
        cands, rejects = candidates(one_wall(), [((WALL_X, -3.0), (WALL_X, 3.0))])
        assert cands == [] and rejects == []


# ==============================================================================
# 3-5: what the beams through the gap must say
# ==============================================================================

class TestTheBeamsThroughTheGap:

    def test_a_pane_the_lidar_sees_across_the_gap_is_wall(self):
        """A pane (or a dark panel) returning ON the line closes the gap."""
        cands, rejects = candidates(one_wall(),
                                    [((WALL_X, -3.0), (WALL_X, 3.0))] + back_wall())
        assert cands == [] and rejects == []

    def test_sparse_returns_on_the_wall_line_inside_the_gap_are_not_a_door(self):
        """
        Glass the LiDAR half-sees: single returns ON the line, every few beams.

        Each is too short to be a piece of wall, so the gap stays one gap --
        with ON returns inside it.
        """
        ranges = raycast(ORIGIN, wall_with_gap(-0.425, 0.425) + back_wall())
        pane = raycast(ORIGIN, [((WALL_X, -0.425), (WALL_X, 0.425))])
        hit = np.flatnonzero(pane < NO_RETURN)
        ranges[hit[::6]] = pane[hit[::6]]
        rejects = []
        cands = dd.door_candidates(one_wall(), ORIGIN, scan_of(ranges), None, rejects)
        assert cands == []
        assert codes(rejects) == {'on_line'}

    def test_a_return_just_past_the_line_is_too_shallow_to_call(self):
        """0.2 m beyond: more than the wall tolerance, less than the margin."""
        cands, rejects = candidates(
            one_wall(), wall_with_gap(-0.425, 0.425) + [((WALL_X + 0.2, -0.6),
                                                         (WALL_X + 0.2, 0.6))])
        assert cands == []
        assert codes(rejects) == {'shallow'}

    def test_a_box_in_front_of_the_wall_casts_a_gap_that_is_not_a_door(self):
        """
        A box in front of a CONTINUOUS wall casts a door-wide gap in its returns.

        The shadow of a 0.56 m box 1.1 m in front of the wall is 0.86 m wide,
        with closer returns at its ends.
        """
        box = [((1.9, -0.28), (1.9, 0.28))]
        cands, rejects = candidates(two_walls(half_gap=0.43),
                                    [((WALL_X, -3.0), (WALL_X, 3.0))] + box)
        assert cands == []
        assert codes(rejects) == {'jamb_occluded'}

    def test_a_real_opening_with_one_jamb_hidden_by_a_box_is_not_a_door(self):
        """The opening is real, but its right end is a shadow edge, not a jamb."""
        box = [((2.0, 0.196), (2.0, 0.359))]
        cands, rejects = candidates(one_wall(),
                                    wall_with_gap(-0.425, 0.425) + back_wall() + box)
        assert cands == []
        assert codes(rejects) == {'jamb_occluded'}

    def test_too_few_beams_through_a_far_gap_is_undecidable(self):
        """0.8 m at 25 m square-on is ~7 beams at 0.25 deg, under the floor of 8."""
        far = 25.0
        tracks = [track(1, -6.0, 6.0, rho=far)]
        cands, rejects = candidates(tracks, wall_with_gap(-0.4, 0.4, x=far, y_min=-6.0,
                                                          y_max=6.0))
        assert cands == []
        assert codes(rejects) == {'few_beams'}

    def test_the_same_gap_near_by_is_decided(self):
        tracks = [track(1, -3.0, 3.0)]
        cands, _rejects = candidates(tracks, wall_with_gap(-0.4, 0.4))
        assert len(cands) == 1

    def test_the_beam_requirement_grows_as_the_door_gets_closer(self):
        """n_required = max(floor, 0.8 x what the geometry promises)."""
        near, _ = candidates([track(1, -3.0, 3.0, rho=1.5)],
                             wall_with_gap(-0.425, 0.425, x=1.5))
        far, _ = candidates(one_wall(), wall_with_gap(-0.425, 0.425))
        assert near[0].n_required > far[0].n_required > 8


# ==============================================================================
# 1, 8: the walls the door is built on
# ==============================================================================

class TestTheWalls:

    def test_a_corner_is_not_a_door(self):
        """Two walls meeting at 90 deg with a door-wide gap at the corner."""
        a = track(1, -3.0, 0.0)                               # x = 3, y in [-3, 0]
        # At theta pi/2 the tangent is -x, so t in [-6, -3.6] is x in [3.6, 6].
        b = track(2, -6.0, -3.6, rho=0.6, theta=math.pi / 2)  # y = 0.6, x in [3.6, 6]
        segs = [((WALL_X, -3.0), (WALL_X, 0.0)), ((3.6, 0.6), (6.0, 0.6))]
        cands, rejects = candidates([a, b], segs + back_wall(x=8.0))
        assert cands == []
        assert 'not_collinear' in codes(rejects)

    def test_two_parallel_walls_offset_by_more_than_the_tolerance_are_not_one_line(self):
        a, b = track(1, -3.0, -0.425), track(2, 0.425, 3.0, rho=WALL_X + 0.3)
        segs = [((WALL_X, -3.0), (WALL_X, -0.425)), ((WALL_X + 0.3, 0.425),
                                                     (WALL_X + 0.3, 3.0))]
        cands, rejects = candidates([a, b], segs + back_wall())
        assert cands == []
        assert 'not_collinear' in codes(rejects)

    @pytest.mark.parametrize('state', [mwt.STATE_TENTATIVE])
    def test_a_tentative_wall_never_carries_a_door(self, state):
        tracks = [track(1, -3.0, -0.425), track(2, 0.425, 3.0, state=state)]
        cands, _rejects = candidates(tracks, wall_with_gap(-0.425, 0.425) + back_wall())
        assert cands == []

    def test_a_coasting_wall_still_carries_one(self):
        tracks = [track(1, -3.0, -0.425), track(2, 0.425, 3.0, state=mwt.STATE_COASTING)]
        cands, _rejects = candidates(tracks, wall_with_gap(-0.425, 0.425) + back_wall())
        assert len(cands) == 1
        assert set(cands[0].parents) == {1, 2}

    def test_a_jamb_must_be_on_a_tracked_wall(self):
        """A gap in returns no track covers is not built on the tracker."""
        tracks = [track(1, -3.0, -1.5)]
        cands, _rejects = candidates(tracks, wall_with_gap(-0.425, 0.425) + back_wall())
        assert cands == []

    def test_the_door_is_found_from_a_moved_and_turned_car(self):
        car = Pose2D(0.5, -0.4, math.radians(20.0))
        cands, _rejects = candidates(one_wall(), wall_with_gap(-0.425, 0.425) + back_wall(),
                                     car=car)
        assert len(cands) == 1
        centre_odom = car.transform_points(cands[0].centre)[0]
        assert centre_odom == pytest.approx([WALL_X, 0.0], abs=0.04)


# ==============================================================================
# 5-7: tracking, objects and identity
# ==============================================================================

PERSON = (WALL_X, 0.0, 0.2)     # (x, y, half width): standing in the doorway
DOOR_SCENE = wall_with_gap(-0.425, 0.425) + back_wall()


def cycle(tracker, tracks, stamp, person=False, car=ORIGIN):
    objects = (PERSON,) if person else ()
    circles = (((PERSON[0], PERSON[1]), PERSON[2]),) if person else ()
    cands, rejects = candidates(tracks, DOOR_SCENE, circles, objects, car=car,
                                seed=int(stamp * 10))
    tracker.rejects = rejects
    return tracker.step(cands, car, stamp, persons=list(objects))


def confirmed_tracker(tracks=None):
    tracker = dd.DoorTracker()
    for i in range(3):
        doors = cycle(tracker, tracks or one_wall(), 0.1 * i)
    assert len(doors) == 1 and doors[0].state == dd.STATE_CONFIRMED
    return tracker


class TestTracking:

    def test_tentative_then_confirmed_then_coasting_then_deleted(self):
        tracker = dd.DoorTracker()
        states = [cycle(tracker, one_wall(), 0.1 * i)[0].state for i in range(3)]
        assert states == [dd.STATE_TENTATIVE, dd.STATE_TENTATIVE, dd.STATE_CONFIRMED]
        # The doorway closes (a wall with no gap): it coasts, then goes.
        closed = [((WALL_X, -3.0), (WALL_X, 3.0))]
        scan = scan_of(raycast(ORIGIN, closed))
        doors = tracker.step(dd.door_candidates(one_wall(), ORIGIN, scan), ORIGIN, 0.5)
        assert doors[0].state == dd.STATE_COASTING
        assert tracker.step([], ORIGIN, 2.3) == []

    def test_a_door_seen_once_never_confirms_and_is_dropped(self):
        tracker = dd.DoorTracker()
        cycle(tracker, one_wall(), 0.0)
        for i in range(1, 6):
            doors = tracker.step([], ORIGIN, 0.1 * i)
        assert doors == []

    def test_the_door_is_kept_in_odom_while_the_car_moves(self):
        tracker = confirmed_tracker()
        door_id = tracker.doors[0].id
        car = Pose2D(0.8, 0.3, math.radians(-10.0))
        doors = cycle(tracker, one_wall(), 0.3, car=car)
        assert [d.id for d in doors] == [door_id]
        assert doors[0].centre == pytest.approx([WALL_X, 0.0], abs=0.04)
        assert doors[0].state == dd.STATE_CONFIRMED


class TestObjects:

    def test_a_person_in_a_confirmed_door_keeps_it_alive_and_occupied(self):
        tracker = confirmed_tracker()
        door_id = tracker.doors[0].id
        hits = tracker.doors[0].hits
        # 4 s of a person standing in it: twice delete_after_s.
        for i in range(40):
            doors = cycle(tracker, one_wall(), 0.3 + 0.1 * i, person=True)
            assert [d.id for d in doors] == [door_id]
            assert doors[0].occupied is True
            assert doors[0].state == dd.STATE_CONFIRMED
        # The person never counted as a sighting...
        assert doors[0].hits == hits
        assert 'object' in codes(tracker.rejects) or 'blocked' in codes(tracker.rejects)
        # ...and once they step out the beams see through it again.
        doors = cycle(tracker, one_wall(), 4.4)
        assert [d.id for d in doors] == [door_id]
        assert doors[0].occupied is False
        assert doors[0].hits == hits + 1

    def test_a_person_in_a_gap_that_is_not_yet_a_door_never_makes_one(self):
        tracker = dd.DoorTracker()
        for i in range(30):
            doors = cycle(tracker, one_wall(), 0.1 * i, person=True)
            assert doors == []
        assert 'object' in codes(tracker.rejects) or 'blocked' in codes(tracker.rejects)

    def test_a_person_does_not_hold_a_tentative_door(self):
        tracker = dd.DoorTracker()
        cycle(tracker, one_wall(), 0.0)
        for i in range(1, 30):
            doors = cycle(tracker, one_wall(), 0.1 * i, person=True)
        assert doors == []

    def test_a_person_beside_the_door_does_not_occupy_it(self):
        tracker = confirmed_tracker()
        door = tracker.doors[0]
        assert not tracker.person_inside(door, [(WALL_X, 1.5, 0.2)])
        assert not tracker.person_inside(door, [(1.5, 0.0, 0.2)])
        assert tracker.person_inside(door, [(WALL_X - 0.3, 0.1, 0.2)])


class TestIdentity:

    def test_a_reborn_parent_wall_keeps_the_same_door_id(self):
        tracker = confirmed_tracker(two_walls(ids=(1, 2)))
        door_id = tracker.doors[0].id
        assert set(tracker.doors[0].parents) == {1, 2}
        # The tracker lost both walls and re-spawned them: same geometry, new ids.
        doors = cycle(tracker, two_walls(ids=(17, 18)), 0.3)
        assert [d.id for d in doors] == [door_id]
        assert set(doors[0].parents) == {17, 18}
        # ...and the same when the doorway is now inside one merged track.
        doors = cycle(tracker, one_wall(track_id=40), 0.4)
        assert [d.id for d in doors] == [door_id]
        assert doors[0].parents == (40, 40)

    def test_a_second_doorway_along_the_wall_is_a_second_door(self):
        tracker = dd.DoorTracker()
        segs = [((WALL_X, -3.0), (WALL_X, -1.825)), ((WALL_X, -1.0), (WALL_X, -0.425)),
                ((WALL_X, 0.425), (WALL_X, 3.0))] + back_wall()
        for i in range(3):
            scan = scan_of(raycast(ORIGIN, segs, seed=i))
            doors = tracker.step(dd.door_candidates(one_wall(), ORIGIN, scan), ORIGIN,
                                 0.1 * i)
        assert len(doors) == 2
        assert len({d.id for d in doors}) == 2

    def test_door_ids_are_never_reused(self):
        tracker = confirmed_tracker()
        first = tracker.doors[0].id
        assert tracker.step([], ORIGIN, 5.0) == []
        doors = cycle(tracker, one_wall(), 5.1)
        assert doors[0].id > first


# ==============================================================================
# 6: the optional stereo support
# ==============================================================================

FX, CX, CY, W, H = 262.0, 320.0, 180.0, 640, 360
# base_link -> optical: optical z = base x, optical x = -base y, optical y = -base z.
ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CAM_T = np.array([0.12, 0.0, 0.3])


def render(gap=True, pane_depth=True):
    """
    Return (labels, depth_mm) of the door scene seen by a forward camera.

    gap False closes the doorway with a surface ON the wall line (a pane the
    stereo ranged). pane_depth False leaves that surface's depth invalid (0):
    a pane the stereo saw as WALL-coloured but could not range.
    """
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d_opt = np.stack([(u - CX) / FX, (v - CY) / FX, np.ones_like(u, dtype=float)], -1)
    d = d_opt @ ROT.T                              # base_link ray per unit optical z
    best = np.full((H, W), np.inf)
    cls = np.full((H, W), scene_mask.CLASS_INVALID, dtype=np.uint8)
    with np.errstate(divide='ignore', invalid='ignore'):
        s_floor = -CAM_T[2] / d[..., 2]
        hit = (d[..., 2] < 0) & (s_floor > 0)
        best = np.where(hit, s_floor, best)
        cls[hit] = scene_mask.CLASS_FLOOR
        for x_wall, is_front in ((BACK_X, False), (WALL_X, True)):
            s = (x_wall - CAM_T[0]) / d[..., 0]
            y = CAM_T[1] + s * d[..., 1]
            z = CAM_T[2] + s * d[..., 2]
            ok = (s > 0) & (z >= 0.0) & (z <= 2.5) & (s < best)
            if is_front and gap:
                ok &= np.abs(y) > 0.425
            best = np.where(ok, s, best)
            cls[ok] = scene_mask.CLASS_WALL
            if is_front and not gap and not pane_depth:
                pane = ok & (np.abs(y) <= 0.425)
                best = np.where(pane, 0.0, best)
    depth = np.where(np.isfinite(best), np.round(best * 1000.0), 0).astype(np.uint16)
    return cls, depth


def door_candidate():
    cands, _ = candidates(one_wall(), DOOR_SCENE)
    return cands[0]


class TestStereo:

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

    def test_stereo_support_raises_the_confidence_and_is_never_required(self):
        lidar_only = dd.DoorTracker()
        both = dd.DoorTracker()
        for i in range(3):
            cand = door_candidate()
            lidar_only.step([cand], ORIGIN, 0.1 * i)
            cand = door_candidate()
            cand.stereo_support = True
            both.step([cand], ORIGIN, 0.1 * i)
        assert lidar_only.doors[0].state == dd.STATE_CONFIRMED
        assert lidar_only.doors[0].stereo_support is False
        assert both.doors[0].stereo_support is True
        assert both.doors[0].confidence > lidar_only.doors[0].confidence


# ==============================================================================
# End to end: the real extraction and tracker
# ==============================================================================

class TestThroughTheRealTracker:

    def test_a_doorway_in_a_tracked_wall_is_found(self):
        tracker = mwt.MultiWallTracker()
        tracker.laser_xy = LASER[:2]
        p = mwt.extraction_params(backend='python')
        doors = dd.DoorTracker()
        for i in range(6):
            ranges = raycast(ORIGIN, DOOR_SCENE, seed=i)
            lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=p)
            obs = [wall_fusion.Observation(mwt.line_to_wall_line(ln), 0.0, 2.5, False,
                                           wall_fusion.SOURCE_LIDAR, 0.8) for ln in lines]
            scan = scan_of(ranges)
            tracker.step(obs, 0.1 * i, ORIGIN, scan=scan)
            out = doors.step(dd.door_candidates(tracker.tracks, ORIGIN, scan), ORIGIN,
                             0.1 * i)
        assert len(out) == 1
        assert out[0].state == dd.STATE_CONFIRMED
        assert out[0].width == pytest.approx(0.85, abs=0.04)
        assert out[0].centre == pytest.approx([WALL_X, 0.0], abs=0.04)
        live = {t.id for t in tracker.tracks}
        assert set(out[0].parents) <= live


# ==============================================================================
# THE COLLINEAR GAP RULE (multi_wall_tracker): merge gap = door_width_min_m
# ==============================================================================

def run_real_tracker(segments, cycles=6):
    """Run the real extraction and MultiWallTracker on segments; return (tracker, doors)."""
    tracker = mwt.MultiWallTracker()
    tracker.laser_xy = LASER[:2]
    p = mwt.extraction_params(backend='python')
    doors = dd.DoorTracker()
    out = []
    for i in range(cycles):
        ranges = raycast(ORIGIN, segments, seed=i)
        lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=p)
        obs = [wall_fusion.Observation(mwt.line_to_wall_line(ln), 0.0, 2.5, False,
                                       wall_fusion.SOURCE_LIDAR, 0.8) for ln in lines]
        scan = scan_of(ranges)
        tracker.step(obs, 0.1 * i, ORIGIN, scan=scan)
        out = doors.step(dd.door_candidates(tracker.tracks, ORIGIN, scan), ORIGIN, 0.1 * i)
    return tracker, out


def walls_on_the_line(tracker):
    """Return the confirmed tracks on the x = 3 wall line, ordered along it."""
    on = [t for t in tracker.tracks if t.ever_confirmed
          and abs(t.rho - WALL_X) < 0.1 and abs(math.sin(t.theta)) < 0.1]
    return sorted(on, key=lambda t: t.t0)


class TestTheCollinearGapRule:

    def test_a_0_5_m_gap_is_merged_into_one_wall(self):
        tracker, doors = run_real_tracker(wall_with_gap(-0.25, 0.25) + back_wall())
        assert len(walls_on_the_line(tracker)) == 1
        assert doors == []

    def test_a_0_85_m_gap_stays_two_walls_with_one_door_between_them(self):
        tracker, doors = run_real_tracker(DOOR_SCENE)
        walls = walls_on_the_line(tracker)
        assert len(walls) == 2
        assert walls[0].t1 < -0.4 and walls[1].t0 > 0.4      # neither spans the door
        assert len(doors) == 1 and doors[0].state == dd.STATE_CONFIRMED
        assert set(doors[0].parents) == {walls[0].id, walls[1].id}

    def test_a_1_4_m_gap_stays_two_walls_and_is_no_door(self):
        tracker, doors = run_real_tracker(wall_with_gap(-0.7, 0.7) + back_wall())
        assert len(walls_on_the_line(tracker)) == 2
        assert doors == []

    def test_the_extractions_merge_reads_the_same_number(self):
        """The extraction alone: 0.5 m merged, 0.85 m not (one source of truth)."""
        p = mwt.extraction_params(backend='python')
        for half, n in ((0.25, 1), (0.425, 2)):
            ranges = raycast(ORIGIN, wall_with_gap(-half, half) + back_wall())
            lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, params=p)
            assert len([ln for ln in lines if abs(ln.rho - WALL_X) < 0.15]) == n

    def test_the_door_search_inside_one_track_is_kept_as_a_fallback(self):
        """A track that spans the doorway anyway (e.g. joined before the rule)."""
        cands, _ = candidates(one_wall(), DOOR_SCENE)
        assert len(cands) == 1 and cands[0].parents == (1, 1)


# ==============================================================================
# THE LEAF: an open door leaf hung on a jamb
# ==============================================================================

def leaf(deg, hinge=(WALL_X, -0.425), length=0.85, towards=1.0):
    """Return an open leaf hinged at `hinge`, `deg` from the wall line, as a segment."""
    th = math.radians(deg)
    end = (hinge[0] + length * math.sin(th), hinge[1] + towards * length * math.cos(th))
    return [(hinge, end)]


class TestTheLeaf:

    def test_a_leaf_open_at_90_deg_on_the_right_jamb_is_a_door_with_its_side(self):
        cands, rejects = candidates(one_wall(), DOOR_SCENE + leaf(90.0))
        assert len(cands) == 1, rejects
        assert cands[0].width == pytest.approx(0.85, abs=0.04)
        # Looking along the crossing normal (+x), y < 0 is the RIGHT jamb.
        assert cands[0].leaf_side == dd.LEAF_RIGHT

    def test_a_leaf_on_the_left_jamb_says_left(self):
        left_leaf = leaf(90.0, hinge=(WALL_X, 0.425), towards=-1.0)
        cands, _ = candidates(one_wall(), DOOR_SCENE + left_leaf)
        assert [c.leaf_side for c in cands] == [dd.LEAF_LEFT]

    def test_no_leaf_is_none(self):
        cands, _ = candidates(one_wall(), DOOR_SCENE)
        assert cands[0].leaf_side == dd.LEAF_NONE

    def test_a_leaf_open_at_45_deg_crosses_the_core_and_needs_a_smaller_core(self):
        """
        Reject a 45 deg leaf with the default core, accept it with a 0.3 core.

        At 45 deg the leaf passes through the central 60 % less than 0.30 m
        past the line.
        """
        cands, rejects = candidates(one_wall(), DOOR_SCENE + leaf(45.0))
        assert cands == []
        assert codes(rejects) == {'shallow'}
        cands, _ = candidates(one_wall(), DOOR_SCENE + leaf(45.0),
                              p=dd.DoorParams(core_fraction=0.3))
        assert [c.leaf_side for c in cands] == [dd.LEAF_RIGHT]

    def test_a_box_face_in_the_outer_part_is_not_a_leaf(self):
        """A box 0.2 m past the line beside the jamb: its face misses the hinge."""
        box = [((WALL_X + 0.2, -0.40), (WALL_X + 0.2, -0.30)),
               ((WALL_X + 0.2, -0.30), (WALL_X + 0.35, -0.30))]
        cands, rejects = candidates(one_wall(), DOOR_SCENE + box)
        assert cands == []
        assert codes(rejects) == {'not_a_leaf'}

    def test_a_box_corner_in_the_outer_part_is_not_a_leaf(self):
        """A box turned corner-on, in the outer part: two faces, not one leaf."""
        box = [((WALL_X + 0.28, -0.41), (WALL_X + 0.12, -0.34)),
               ((WALL_X + 0.12, -0.34), (WALL_X + 0.28, -0.27))]
        cands, rejects = candidates(one_wall(), DOOR_SCENE + box)
        assert cands == []
        assert codes(rejects) <= {'not_a_leaf', 'shallow'} and rejects

    def test_is_leaf_wants_a_straight_segment_starting_at_the_jamb(self):
        p = dd.DoorParams()
        jamb = np.array([0.0, 0.0])
        along = np.array([[0.0, 0.2], [0.0, 0.4], [0.0, 0.6]])
        assert dd.is_leaf(along, jamb, p)
        assert not dd.is_leaf(along + [0.2, 0.0], jamb, p)           # misses the hinge
        assert not dd.is_leaf(np.array([[0.0, -0.3], [0.0, 0.3]]), jamb, p)  # through it
        assert not dd.is_leaf(np.array([[0.0, 0.5], [0.0, 1.2]]), jamb, p)   # too long
        assert not dd.is_leaf(np.array([[0.0, 0.2], [0.1, 0.4], [0.0, 0.6]]), jamb, p)

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
# The stereo glass veto (stereo_glass_veto, OFF by default)
# ==============================================================================

def glass_check(**render_kw):
    labels, depth = render(**render_kw)
    return dd.stereo_glass_check(door_candidate(), labels, depth, FX, FX, CX, CY, ROT,
                                 CAM_T, depth_scale=0.001)


class TestGlassVeto:

    def test_it_is_off_by_default(self):
        assert dd.DoorParams().stereo_glass_veto is False

    def test_a_real_door_with_a_far_wall_behind_is_kept(self):
        """The room behind is WALL in the mask, but beyond the line: it never counts."""
        veto, suspect, detail = glass_check(gap=True)
        assert veto is False and suspect is False
        assert detail['on_line_frac'] < 0.1

    def test_glass_on_the_line_with_valid_depth_is_vetoed(self):
        veto, suspect, detail = glass_check(gap=False)
        assert veto is True and suspect is False
        assert detail['on_line_frac'] >= 0.5

    def test_glass_with_invalid_depth_is_kept_and_flagged(self):
        veto, suspect, detail = glass_check(gap=False, pane_depth=False)
        assert veto is False and suspect is True
        assert detail['valid_frac'] < 0.5

    def test_no_camera_view_is_no_opinion(self):
        labels = np.zeros((H, W), dtype=np.uint8)
        depth = np.zeros((H, W), dtype=np.uint16)
        cand = door_candidate()
        cand.jamb0, cand.jamb1 = -cand.jamb0, -cand.jamb1       # behind the camera
        veto, suspect, _ = dd.stereo_glass_check(cand, labels, depth, FX, FX, CX, CY, ROT,
                                                 CAM_T, depth_scale=0.001)
        assert veto is None and suspect is False

    def test_glass_suspect_rides_on_the_door(self):
        tracker = dd.DoorTracker()
        cand = door_candidate()
        cand.glass_suspect = True
        doors = tracker.step([cand], ORIGIN, 0.0)
        assert doors[0].glass_suspect is True
