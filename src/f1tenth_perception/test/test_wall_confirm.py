"""LiDAR confirmation of the camera's wall distance (M02 campaign fix A4).

R009: the wall stop fired on a person ~1.5 m ahead who had leaked into
/perception/front_distance. The stop now needs the forward /scan to show a
wall-like line where the camera says the wall is. Synthetic scans, ray-cast
against walls (segments) and people/plants (circles), on the Hokuyo's
0.25 deg grid. Sensor mounts as on the car: laser 0.12 m ahead of base_link
(description.launch.py's static base_link -> laser, 0.12 0.0 0.20, which the
recorded /tf_static agrees with -- see corridor_perception/replay.py; 0.20 is
its height), camera lens at 0.11 m (zed2_camera_link 0.12 m, ZED2 optical
centre ~1 cm behind it). front_clearance_node reads both from /tf_static.
"""

import math

import numpy as np
import pytest

from f1tenth_perception.wall_confirm import (
    REASON_DISAGREE, REASON_NO_CAMERA, REASON_NO_SCAN, REASON_OBLIQUE, REASON_OK,
    REASON_SHORT, WallConfirmConfig, WallConfirmTracker, confirm_wall, scan_points_base,
    visible_extent)

LASER = (0.12, 0.0, 0.0)
CAMERA_X = 0.11
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(0.25)
N_BEAMS = 1081


def _ray_segment(ox, oy, dx, dy, seg):
    (x1, y1), (x2, y2) = seg
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return math.inf
    t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else math.inf


def _ray_circle(ox, oy, dx, dy, circle):
    cx, cy, r = circle
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def scan(segments=(), circles=(), max_range=10.0):
    """Ranges (laser frame) for walls (segments) and round objects (circles), base_link."""
    lx, ly, _ = LASER
    ranges = []
    for k in range(N_BEAMS):
        a = ANGLE_MIN + k * ANGLE_INC
        dx, dy = math.cos(a), math.sin(a)
        hits = [_ray_segment(lx, ly, dx, dy, s) for s in segments]
        hits += [_ray_circle(lx, ly, dx, dy, c) for c in circles]
        ranges.append(min(hits + [math.inf]) if min(hits + [math.inf]) <= max_range
                      else math.inf)
    return ranges


def _points(ranges, cfg=WallConfirmConfig()):
    return scan_points_base(ranges, ANGLE_MIN, ANGLE_INC, 0.06, 10.0, LASER,
                            math.radians(cfg.half_angle_deg))


WALL_4M = ((4.0, -3.0), (4.0, 3.0))
PERSON_1_5 = [(1.5, 0.12, 0.07), (1.5, -0.12, 0.07)]   # two legs
PLANT_1_4 = [(1.4, 0.0, 0.15)]


def _confirm(ranges, camera_distance):
    return confirm_wall(_points(ranges), camera_distance, CAMERA_X)


def test_a_wall_where_the_camera_says_is_confirmed():
    c = _confirm(scan([WALL_4M]), 4.0 - CAMERA_X)
    assert c.confirmed, c.as_dict()
    assert c.reason == REASON_OK
    assert c.lidar_distance_m == pytest.approx(4.0, abs=0.02)
    assert c.extent_m > 2.0


def test_r009_a_person_leaking_into_front_distance_is_not_a_wall():
    """The camera reads the person (1.5 m), the LiDAR sees legs, not a wall."""
    c = _confirm(scan([WALL_4M], PERSON_1_5), 1.5 - CAMERA_X)
    assert not c.confirmed
    assert c.reason != REASON_OK


def test_r011_a_plant_is_not_a_wall():
    c = _confirm(scan([WALL_4M], PLANT_1_4), 1.4 - CAMERA_X)
    assert not c.confirmed


def test_the_wall_behind_a_person_is_still_confirmed():
    c = _confirm(scan([WALL_4M], PERSON_1_5), 4.0 - CAMERA_X)
    assert c.confirmed, c.as_dict()


def test_a_camera_distance_with_nothing_there_is_not_confirmed():
    """Leak into the middle of empty floor: no returns near 2 m at all."""
    c = _confirm(scan([WALL_4M]), 2.0)
    assert not c.confirmed


def test_a_wall_further_off_than_the_tolerance_disagrees():
    # Wall at 2.0 m, camera 2.58 m: inside the 0.6 m search window, outside
    # 0.30 + 10 % of 2.58 = 0.558 m. (2.6 sat exactly on the window edge.)
    c = _confirm(scan([((2.0, -3.0), (2.0, 3.0))]), 2.58 - CAMERA_X)
    assert not c.confirmed
    assert c.reason == REASON_DISAGREE


def test_an_oblique_wall_is_rejected():
    # 70 deg off square, crossing the axis at 3.0 m: a side wall angling across.
    t = math.tan(math.radians(70.0))
    seg = ((3.0 - 2.0 * t, -2.0), (3.0 + 2.0 * t, 2.0))
    c = _confirm(scan([seg]), 3.0 - CAMERA_X)
    assert not c.confirmed
    assert c.reason in (REASON_OBLIQUE, 'too_few_returns', 'no_line')


def test_side_walls_alone_are_not_a_wall_ahead():
    corridor = [((0.0, 0.6), (8.0, 0.6)), ((0.0, -0.6), (8.0, -0.6))]
    c = _confirm(scan(corridor), 3.0)
    assert not c.confirmed


def test_a_moderately_angled_wall_is_still_confirmed():
    # 20 deg off square, crossing the axis at 3.0 m.
    t = math.tan(math.radians(20.0))
    seg = ((3.0 - 3.0 * t, -3.0), (3.0 + 3.0 * t, 3.0))
    c = _confirm(scan([seg]), 3.0 - CAMERA_X)
    assert c.confirmed, c.as_dict()


def test_no_camera_reading_and_no_scan():
    assert confirm_wall(None, -1.0, CAMERA_X).reason == REASON_NO_CAMERA
    assert confirm_wall(None, math.nan, CAMERA_X).reason == REASON_NO_CAMERA
    assert confirm_wall(None, 2.0, CAMERA_X).reason == REASON_NO_SCAN


def test_the_sector_only_keeps_forward_returns():
    pts = _points(scan([((-2.0, -3.0), (-2.0, 3.0)), WALL_4M]))
    assert np.all(pts[:, 0] > 0.0)
    bearings = np.degrees(np.arctan2(pts[:, 1] - LASER[1], pts[:, 0] - LASER[0]))
    assert np.abs(bearings).max() <= 30.0 + 0.25


def test_the_shipped_parameters_match_the_dataclass():
    from f1tenth_params.param_defaults import get_value
    cfg = WallConfirmConfig()
    assert get_value('front_clearance_wall_confirm_half_angle_deg') == cfg.half_angle_deg
    assert get_value('front_clearance_wall_confirm_min_extent_m') == cfg.min_extent_m
    assert get_value('front_clearance_wall_confirm_tolerance_m') == cfg.tolerance_m
    assert get_value('front_clearance_wall_confirm_tolerance_frac') == cfg.tolerance_frac
    assert get_value('front_clearance_wall_confirm_search_window_m') == cfg.search_window_m
    assert get_value('front_clearance_wall_confirm_min_inliers') == cfg.min_inliers
    assert get_value('front_clearance_wall_confirm_near_extent_frac') == cfg.near_extent_frac
    assert get_value('front_clearance_wall_confirm_latch_hold_s') == cfg.latch_hold_s


# ---------------------------------------------------------------------------
# Close to the wall: the latch (2026-10-12). The +-30 deg sector shows only
# 2 d tan30 of a square-on wall, 0.57 m when the camera reads the 0.5 m stop,
# so the fixed 0.8 m could never confirm there and the wall stop never fired.
# ---------------------------------------------------------------------------

STOP_M = 0.5        # the wall stop distance (camera reading), e.g. boundary_constraint_diag_05m
RATE_HZ = 15.0      # front_clearance_node: one confirmation per depth frame
SPEED = 0.5         # m/s


def _shift(segments, circles, car_x):
    segs = [((x1 - car_x, y1), (x2 - car_x, y2)) for (x1, y1), (x2, y2) in segments]
    return segs, [(cx - car_x, cy, r) for cx, cy, r in circles]


def _approach(segments, circles, camera_reading, start_x=0.0, end_x=None, speed=SPEED):
    """Drive the car along +x; camera_reading(car_x) -> front_distance.

    Returns [(camera distance, WallConfirmation)] per depth frame, through a
    WallConfirmTracker as front_clearance_node runs it.
    """
    tracker = WallConfirmTracker()
    out, x, t = [], start_x, 0.0
    while end_x is None or x <= end_x:
        d = camera_reading(x)
        if d is not None and d < STOP_M - 0.1:
            break
        segs, circs = _shift(segments, circles, x)
        c = tracker.update(_points(scan(segs, circs)), d if d is not None else -1.0,
                           CAMERA_X, LASER[:2], t)
        out.append((d, c))
        x += speed / RATE_HZ
        t += 1.0 / RATE_HZ
    return out


def _wall_stop_fires(frames):
    """CheckStopCondition: front_wall_confirmed (-1 unless confirmed) < distance."""
    return any(c.confirmed and d <= STOP_M for d, c in frames)


WALL_X = 3.0 + CAMERA_X      # camera reads 3.0 m at the start
FLAT_WALL = ((WALL_X, -3.0), (WALL_X, 3.0))


def test_the_bug_a_fixed_0_8_m_never_confirms_at_the_0_5_m_stop():
    c = _confirm(scan([((STOP_M + CAMERA_X, -3.0), (STOP_M + CAMERA_X, 3.0))]), STOP_M)
    assert not c.confirmed and c.reason == REASON_SHORT
    assert c.extent_m == pytest.approx(2 * (STOP_M + CAMERA_X - LASER[0]) * math.tan(
        math.radians(30.0)), abs=0.01)


def test_visible_extent_of_a_square_on_wall_is_2_d_tan_half_angle():
    d = STOP_M + CAMERA_X - LASER[0]
    assert visible_extent(1.0, 0.0, STOP_M + CAMERA_X, LASER[:2], math.radians(30.0),
                          STOP_M + CAMERA_X, 0.6) == pytest.approx(
        2 * d * math.tan(math.radians(30.0)), abs=1e-9)


def test_approaching_a_flat_wall_from_3_m_is_confirmed_the_whole_way_and_stops_at_0_5():
    frames = _approach([FLAT_WALL], [], lambda x: WALL_X - x - CAMERA_X)
    assert frames[0][0] == pytest.approx(3.0)
    assert frames[-1][0] < STOP_M
    unconfirmed = [(round(d, 2), c.reason) for d, c in frames if not c.confirmed]
    assert unconfirmed == []
    assert _wall_stop_fires(frames)
    # Near the stop it is the latch that carries it, with a shorter requirement.
    d, c = min(frames, key=lambda f: abs(f[0] - STOP_M))
    assert c.latched
    assert c.required_extent_m == pytest.approx(0.7 * c.extent_m, abs=0.03)
    assert c.extent_m < 0.8


def test_a_person_1_m_before_the_wall_is_never_the_wall_stop():
    """The camera reads the wall while the person is masked out, then the
    person leaks into front_distance (R009) for the rest of the approach. The
    latched wall line does not carry over to the legs, so the stop never
    fires on the person, however close."""
    person_x = WALL_X - 1.0
    legs = [(person_x, 0.12, 0.07), (person_x, -0.12, 0.07)]

    def camera(x):
        to_person = person_x - 0.07 - x - CAMERA_X
        return WALL_X - x - CAMERA_X if to_person > 1.5 else to_person

    frames = _approach([FLAT_WALL], legs, camera)
    assert any(c.confirmed for d, c in frames)            # the wall, while far
    leaked = [(d, c) for d, c in frames if d < 1.5]
    assert leaked and not any(c.confirmed for d, c in leaked)
    assert not _wall_stop_fires(frames)


def test_a_person_leaking_from_the_start_is_never_confirmed():
    person_x = WALL_X - 1.0
    legs = [(person_x, 0.15, 0.08), (person_x, -0.15, 0.08)]   # wider stance
    frames = _approach([FLAT_WALL], legs, lambda x: person_x - 0.08 - x - CAMERA_X)
    assert not any(c.confirmed for d, c in frames)


def test_a_plant_close_ahead_is_never_confirmed():
    plant_x = WALL_X - 1.0
    frames = _approach([FLAT_WALL], [(plant_x, 0.0, 0.15)],
                       lambda x: plant_x - 0.15 - x - CAMERA_X)
    assert not any(c.confirmed for d, c in frames)


@pytest.mark.parametrize('angle_deg', [30.0, -30.0])
def test_a_wall_at_30_deg_is_confirmed_down_to_the_stop(angle_deg):
    # Crosses the car's axis at WALL_X, normal 30 deg off the heading.
    t = math.tan(math.radians(angle_deg))
    seg = ((WALL_X + 3.0 * t, -3.0), (WALL_X - 3.0 * t, 3.0))
    frames = _approach([seg], [], lambda x: WALL_X - x - CAMERA_X)
    unconfirmed = [(round(d, 2), c.reason) for d, c in frames if not c.confirmed]
    assert unconfirmed == []
    assert _wall_stop_fires(frames)


def test_a_short_wall_segment_is_never_a_wall():
    """A 0.6 m wide panel (a door leaf, a cabinet): never the full 0.8 m, so
    never confirmed and never latched -- the latch cannot start on it."""
    panel = ((WALL_X, -0.3), (WALL_X, 0.3))
    frames = _approach([panel], [], lambda x: WALL_X - x - CAMERA_X)
    assert not any(c.confirmed for d, c in frames)
    assert {c.reason for d, c in frames} <= {REASON_SHORT, 'too_few_returns', 'no_line'}


def test_a_doorway_ahead_confirms_the_wall_then_lets_go_in_the_opening():
    """A wall with a 0.9 m doorway on the car's axis, camera reading the wall
    plane: confirmed while the sector still holds wall either side of the
    opening; once the opening fills the sector there is no wall ahead and the
    confirmation is dropped (no wall stop in a doorway; the proximity e-stop
    covers the door frame)."""
    door = [((WALL_X, -3.0), (WALL_X, -0.45)), ((WALL_X, 0.45), (WALL_X, 3.0))]
    frames = _approach(door, [], lambda x: WALL_X - x - CAMERA_X)
    assert frames[0][1].confirmed
    # Laser within 0.45 / tan30 = 0.78 m of the wall plane: the sector sees
    # only the opening.
    blind = [c for d, c in frames if d + CAMERA_X - LASER[0] < 0.45 / math.tan(math.radians(30))]
    assert blind and not any(c.confirmed for c in blind)
    assert not _wall_stop_fires(frames)


def test_the_latch_expires_without_confirmations():
    tracker = WallConfirmTracker()
    far = _points(scan([((2.0, -3.0), (2.0, 3.0))]))
    assert tracker.update(far, 2.0 - CAMERA_X, CAMERA_X, LASER[:2], 0.0).confirmed
    near = _points(scan([((0.61, -3.0), (0.61, 3.0))]))
    # Too long a gap (> latch_hold_s): the near wall needs the full 0.8 m again.
    c = tracker.update(near, 0.5, CAMERA_X, LASER[:2], 1.0)
    assert not c.confirmed and not c.latched and c.reason == REASON_SHORT


def test_the_latch_does_not_jump_to_a_nearer_surface():
    """Confirmed wall at 2 m; next frame a surface 1.4 m nearer: not a
    continuation, full extent needed."""
    tracker = WallConfirmTracker()
    assert tracker.update(_points(scan([((2.0, -3.0), (2.0, 3.0))])), 2.0 - CAMERA_X,
                          CAMERA_X, LASER[:2], 0.0).confirmed
    c = tracker.update(_points(scan([((0.61, -3.0), (0.61, 3.0))])), 0.5, CAMERA_X,
                       LASER[:2], 1.0 / RATE_HZ)
    assert not c.latched and not c.confirmed
