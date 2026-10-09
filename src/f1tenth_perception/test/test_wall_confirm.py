"""LiDAR confirmation of the camera's wall distance (M02 campaign fix A4).

R009: the wall stop fired on a person ~1.5 m ahead who had leaked into
/perception/front_distance. The stop now needs the forward /scan to show a
wall-like line where the camera says the wall is. Synthetic scans, ray-cast
against walls (segments) and people/plants (circles), on the Hokuyo's
0.25 deg grid, laser 0.20 m ahead of base_link, camera lens at 0.12 m.
"""

import math

import numpy as np
import pytest

from f1tenth_perception.wall_confirm import (
    REASON_DISAGREE, REASON_NO_CAMERA, REASON_NO_SCAN, REASON_OBLIQUE, REASON_OK,
    WallConfirmConfig, confirm_wall, scan_points_base)

LASER = (0.20, 0.0, 0.0)
CAMERA_X = 0.12
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
    # Wall at 2.0 m, camera 2.6 m: inside the 0.6 m search window, outside
    # 0.30 + 10 % of 2.6 = 0.56 m.
    c = _confirm(scan([((2.0, -3.0), (2.0, 3.0))]), 2.6 - CAMERA_X + 0.0)
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
