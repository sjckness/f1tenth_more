"""
Coverage for mpc_controller/wall_turn_distance.py.

The 'semantic' wall_turn distance: a ray from the bumper along the heading
onto the /perception/walls segments, else the LiDAR forward-sector fit, else
unknown.

All geometry in odom; the car sits at the origin unless a test moves it.

Run standalone: python3 -m pytest test/test_wall_turn_distance.py -v
"""

import math

from mpc_controller import wall_boundaries as wb
from mpc_controller.wall_turn_distance import (
    SOURCE_LIDAR, SOURCE_NONE, SOURCE_SEMANTIC, lidar_forward_distance,
    nearest_wall_hit, ray_segment_distance, track_counts, wall_turn_distance)
import pytest

BUMPER = 0.4
MAX_COAST = 1.0


def _wall(track_id, p0, p1, state=wb.STATE_CONFIRMED, source=wb.SOURCE_LIDAR):
    return wb.WallTrackView(track_id, 'front', state, source, 0.0, 0.0, p0, p1)


def _pick(tracks, coast_ages=None, walls_age=0.1, lidar_wall=None, lidar_age=0.1,
          x=0.0, y=0.0, heading=0.0, frame='odom'):
    return wall_turn_distance(
        x, y, heading, tracks, coast_ages or {}, frame, walls_age, lidar_wall, lidar_age,
        bumper_x_m=BUMPER, odom_frame='odom', max_walls_age_s=0.5,
        max_coast_s=MAX_COAST, max_lidar_age_s=0.5)


# A wall square across the heading, 3 m ahead of base_link: 2.6 m from the bumper.
FRONT = _wall(7, (3.0, -1.0), (3.0, 1.0))


class TestTheRayCast:

    def test_a_wall_across_the_heading_is_hit_at_its_distance_from_the_bumper(self):
        hit = _pick([FRONT])
        assert hit.source == SOURCE_SEMANTIC
        assert hit.track_id == 7
        assert hit.distance == pytest.approx(3.0 - BUMPER)

    def test_a_segment_beside_the_ray_is_a_miss(self):
        assert ray_segment_distance(0.0, 0.0, 0.0, (3.0, 0.5), (3.0, 2.0)) is None
        assert _pick([_wall(1, (3.0, 0.5), (3.0, 2.0))]).source == SOURCE_NONE

    def test_a_wall_parallel_to_the_heading_is_a_miss_even_when_the_ray_runs_along_it(self):
        assert ray_segment_distance(0.0, 0.0, 0.0, (1.0, 0.8), (5.0, 0.8)) is None
        assert ray_segment_distance(0.0, 0.0, 0.0, (1.0, 0.0), (5.0, 0.0)) is None

    def test_a_wall_behind_the_bumper_is_a_miss(self):
        behind = _wall(2, (-1.0, -1.0), (-1.0, 1.0))
        assert _pick([behind]).source == SOURCE_NONE
        # Between base_link and the bumper: behind the ray's origin too.
        assert _pick([_wall(3, (0.2, -1.0), (0.2, 1.0))]).source == SOURCE_NONE

    def test_the_nearest_of_several_segments_wins_whatever_their_order(self):
        far = _wall(1, (6.0, -1.0), (6.0, 1.0))
        near = _wall(2, (2.0, -1.0), (2.5, 1.0))     # oblique, crosses y=0 at x=2.25
        for tracks in ([far, near], [near, far]):
            hit = _pick(tracks)
            assert hit.track_id == 2
            assert hit.distance == pytest.approx(2.25 - BUMPER)

    def test_an_endpoint_on_the_ray_counts_as_a_hit(self):
        assert ray_segment_distance(0.0, 0.0, 0.0, (3.0, 0.0), (3.0, 1.0)) == pytest.approx(3.0)

    def test_mid_turn_the_ray_follows_the_heading_not_the_move_start_line(self):
        # 45 degrees into a left turn, at (1, 0.5): the wall ahead of the
        # START heading (x = 3) is now met obliquely, the side wall y = 3 is
        # what the car points at.
        side = _wall(4, (0.0, 3.0), (6.0, 3.0))
        heading = math.radians(45.0)
        hit = _pick([FRONT, side], x=1.0, y=0.5, heading=heading)
        by = 0.5 + BUMPER * math.sin(heading)
        assert hit.track_id == 4
        assert hit.distance == pytest.approx((3.0 - by) / math.sin(heading))
        # And on the start heading it would have been the front wall.
        assert _pick([FRONT, side], x=1.0, y=0.5, heading=0.0).track_id == 7

    def test_the_bumper_offset_shortens_the_distance_by_exactly_its_length(self):
        bare = nearest_wall_hit(0.0, 0.0, 0.0, [FRONT], {}, bumper_x_m=0.0,
                                max_coast_s=MAX_COAST)
        bumped = nearest_wall_hit(0.0, 0.0, 0.0, [FRONT], {}, bumper_x_m=BUMPER,
                                  max_coast_s=MAX_COAST)
        assert bare[0] - bumped[0] == pytest.approx(BUMPER)


class TestWhichTracksCount:

    def test_confirmed_lidar_and_both_count_stereo_only_never(self):
        for source in (wb.SOURCE_LIDAR, wb.SOURCE_BOTH):
            assert track_counts(_wall(1, (0, 0), (1, 0), source=source), 0.0, MAX_COAST)
        assert not track_counts(
            _wall(1, (0, 0), (1, 0), source=wb.SOURCE_STEREO), 0.0, MAX_COAST)

    def test_a_tentative_track_is_not_a_wall(self):
        tentative = _wall(7, (3.0, -1.0), (3.0, 1.0), state=wb.STATE_TENTATIVE)
        assert _pick([tentative]).source == SOURCE_NONE

    def test_a_coasting_track_counts_until_the_wall_row_coast_cap_then_falls_to_lidar(self):
        coasting = _wall(7, (3.0, -1.0), (3.0, 1.0), state=wb.STATE_COASTING)
        lidar = (2.0, 0.0)
        assert _pick([coasting], {7: MAX_COAST}, lidar_wall=lidar).source == SOURCE_SEMANTIC
        stale = _pick([coasting], {7: MAX_COAST + 0.01}, lidar_wall=lidar)
        assert stale.source == SOURCE_LIDAR
        assert stale.distance == pytest.approx(2.0 - BUMPER)

    def test_a_stale_wall_message_counts_as_no_hit_and_the_lidar_applies(self):
        for age in (None, 0.51):
            pick = _pick([FRONT], walls_age=age, lidar_wall=(2.0, 0.0))
            assert pick.source == SOURCE_LIDAR

    def test_a_wall_message_in_another_frame_counts_as_no_hit(self):
        assert _pick([FRONT], frame='map').source == SOURCE_NONE


class TestTheLidarFallback:

    def test_square_on_it_is_the_fit_distance_less_the_bumper(self):
        assert lidar_forward_distance(2.0, 0.0, BUMPER) == pytest.approx(1.6)

    def test_it_measures_along_the_heading_like_the_ray_cast(self):
        # An oblique wall: the line x cos(a) + y sin(a) = d. The ray-cast on
        # the same line as a segment gives the same number, so a source
        # switch does not step.
        a, d = math.radians(20.0), 3.0
        p0 = (d * math.cos(a) - 2.0 * math.sin(a), d * math.sin(a) + 2.0 * math.cos(a))
        p1 = (d * math.cos(a) + 2.0 * math.sin(a), d * math.sin(a) - 2.0 * math.cos(a))
        semantic = _pick([_wall(9, p0, p1)])
        lidar = _pick([], lidar_wall=(d, a))
        assert semantic.source == SOURCE_SEMANTIC and lidar.source == SOURCE_LIDAR
        assert lidar.distance == pytest.approx(semantic.distance)

    def test_a_line_not_ahead_or_behind_the_bumper_or_nan_is_no_distance(self):
        assert lidar_forward_distance(2.0, math.pi / 2, BUMPER) is None
        assert lidar_forward_distance(0.3, 0.0, BUMPER) is None
        assert lidar_forward_distance(float('nan'), 0.0, BUMPER) is None

    def test_the_semantic_hit_wins_over_the_lidar(self):
        pick = _pick([FRONT], lidar_wall=(1.0, 0.0))
        assert pick.source == SOURCE_SEMANTIC

    def test_a_stale_lidar_estimate_leaves_the_distance_unknown(self):
        for age in (None, 0.51):
            pick = _pick([], lidar_wall=(2.0, 0.0), lidar_age=age)
            assert pick == (None, SOURCE_NONE, None)
