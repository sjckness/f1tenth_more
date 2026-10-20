"""
object_memory.py: visibility per sensor, the TRACKED / MEMORY lifecycle, growth.

Pure: no rclpy. The scenario tests drive the REAL tracker
(semantic_layer.update_tracks_batch with prune_confirmed=False) together with
object_memory.memory_cycle, one detection batch and one memory cycle every
0.1 s, so "keeps its id" and "is removed" are asserted on the same track list
the node publishes from.

The default lifecycle numbers (1.5 s, 15 s, 0.2 m/s, 0.6 m) are the task's and
stack_params.yaml's; test_object_memory_node.py pins that the two agree.

Run standalone: python3 -m pytest test/test_object_memory.py -v
"""

import itertools
import math
from types import SimpleNamespace

from f1tenth_costmap import object_memory as om
from f1tenth_costmap.semantic_layer import (
    compose_base_link_to_map, live_tracks, update_tracks_batch)
from f1tenth_params.semantic_objects import map_point_to_odom, odom_point_to_map
import pytest

DT = 0.1
VISIBLE = om.Visibility(lidar=om.VIS_VISIBLE, stereo=om.VIS_VISIBLE)
OUT_OF_VIEW = om.Visibility(lidar=om.VIS_OUT_OF_ANGLE, stereo=om.VIS_OUT_OF_ANGLE)
OCCLUDED = om.Visibility(lidar=om.VIS_OCCLUDED, stereo=om.VIS_OCCLUDED_DEPTH)


class World:
    """The tracker plus the memory, stepped like the node steps them."""

    def __init__(self, params=None, lost_miss_count=5, enabled=True):
        self.params = params or om.MemoryParams()
        self.tracks = []
        self.memories = {}
        self.ids = itertools.count()
        self.t = 0.0
        self.view = VISIBLE
        self.lost = lost_miss_count
        self.enabled = enabled

    def step(self, detections=()):
        """
        One camera batch (possibly empty) at t, then one memory cycle.

        view is a Visibility for every track, or a callable(track) ->
        Visibility for real geometry.
        """
        self.t += DT
        self.tracks = update_tracks_batch(
            self.tracks, list(detections), self.t, 0.5, 0.3, 3, self.lost,
            lambda: next(self.ids), prune_confirmed=not self.enabled)
        view = self.view if callable(self.view) else (lambda trk: self.view)
        self.tracks = om.memory_cycle(
            self.tracks, self.memories, DT, view, self.params, enabled=self.enabled)

    def run(self, seconds, detections=()):
        for _ in range(int(round(seconds / DT))):
            self.step(detections)

    def confirmed(self):
        return [t for t in self.tracks if t.confirmed]

    def only(self):
        tracks = self.confirmed()
        assert len(tracks) == 1, tracks
        return tracks[0], self.memories[tracks[0].track_id]


def _confirmed_world(class_id='chair', x=3.0, y=0.0, width=0.45, **kw):
    world = World(**kw)
    world.run(0.5, [(class_id, x, y, 0.9, None, width)])
    trk, mem = world.only()
    assert mem.state == om.TRACKED
    return world


# ---------------------------------------------------------------------------
# B5 scenarios

class TestLeavesTheViewPersistsThenExpires:

    def test_out_of_view_object_is_memory_for_about_15_s_then_removed(self):
        world = _confirmed_world()
        track_id = world.only()[0].track_id
        world.view = OUT_OF_VIEW          # the car turned away; batches keep coming
        world.run(7.5)
        trk, mem = world.only()
        assert trk.track_id == track_id
        assert mem.state == om.MEMORY
        assert mem.confidence == pytest.approx(0.5, abs=0.01)
        world.run(7.3)                    # 14.8 s in memory
        trk, mem = world.only()
        assert mem.state == om.MEMORY and 0.0 < mem.confidence < 0.02
        assert mem.remaining_s(world.params) == pytest.approx(0.2, abs=1e-6)
        world.run(0.3)                    # 15.1 s
        assert world.confirmed() == []
        assert world.memories == {}

    def test_the_tracker_miss_count_no_longer_drops_a_confirmed_track(self):
        world = _confirmed_world()
        world.view = OUT_OF_VIEW
        world.run(2.0)                    # 20 empty batches >> lost_miss_count 5
        trk, _mem = world.only()
        # ...but the pre-memory outputs no longer list it (live_tracks).
        assert live_tracks(world.tracks, world.lost) == []

    def test_confidence_decays_linearly(self):
        world = _confirmed_world()
        world.view = OUT_OF_VIEW
        seen = []
        for _ in range(5):
            world.run(2.0)
            seen.append(world.only()[1].confidence)
        diffs = [a - b for a, b in zip(seen, seen[1:])]
        assert all(d == pytest.approx(2.0 / 15.0, abs=1e-6) for d in diffs)


class TestInViewButNotDetected:

    def test_removed_after_1_5_s(self):
        world = _confirmed_world()
        world.run(1.4)                    # in view, no detection
        trk, mem = world.only()
        assert mem.state == om.TRACKED    # we are still looking at it
        assert mem.fov_unseen_s == pytest.approx(1.4, abs=1e-6)
        world.run(0.2)
        assert world.confirmed() == []

    def test_a_detection_resets_the_clock(self):
        world = _confirmed_world()
        world.run(1.2)
        world.run(0.1, [('chair', 3.0, 0.0, 0.9, None, 0.45)])
        world.run(1.2)
        _trk, mem = world.only()
        assert mem.fov_unseen_s == pytest.approx(1.2, abs=1e-6)

    def test_reentering_the_view_undetected_is_removed_after_1_5_s_not_15(self):
        world = _confirmed_world()
        world.view = OUT_OF_VIEW
        world.run(5.0)
        assert world.only()[1].state == om.MEMORY
        world.view = VISIBLE              # turned back: looking at the spot
        world.run(1.4)
        _trk, mem = world.only()
        assert mem.state == om.MEMORY     # not seen yet, decay goes on
        assert mem.time_in_memory_s == pytest.approx(6.4, abs=1e-6)
        world.run(0.2)
        assert world.confirmed() == []


class TestOccludedIsMemory:

    def test_occluded_object_is_kept_well_past_1_5_s(self):
        world = _confirmed_world()
        world.view = OCCLUDED
        world.run(5.0)
        _trk, mem = world.only()
        assert mem.state == om.MEMORY
        assert not mem.in_fov and not mem.in_fov_stereo and not mem.in_fov_lidar
        assert mem.confidence == pytest.approx(1.0 - 5.0 / 15.0, abs=0.01)

    def test_lidar_occlusion_alone_hides_it_from_the_camera_too(self):
        vis = om.object_visibility(
            0.25, (3.0, 0.0), _scan(occluder=(0.0, 1.5)), (3.0, 0.0), True, None, None,
            om.VisibilityParams())
        assert vis.lidar == om.VIS_OCCLUDED
        assert vis.stereo == om.VIS_OCCLUDED_LIDAR
        assert not om.in_fov_for_class('chair', vis)


class TestPersonGrowth:

    def test_person_radius_grows_0_2_mps_and_the_growth_caps_at_0_6(self):
        world = _confirmed_world('person', width=0.5)
        world.view = OUT_OF_VIEW
        trk, mem = world.only()
        assert om.effective_radius(trk, mem, world.params) == pytest.approx(0.25)
        radii = {}
        for t_mem in (1.0, 2.0, 3.0, 5.0, 10.0):
            world.run(t_mem - (world.t - 0.5))
            trk, mem = world.only()
            radii[t_mem] = om.effective_radius(trk, mem, world.params)
        assert radii[1.0] == pytest.approx(0.25 + 0.2, abs=1e-6)
        assert radii[2.0] == pytest.approx(0.25 + 0.4, abs=1e-6)
        assert radii[3.0] == pytest.approx(0.25 + 0.6, abs=1e-6)
        assert radii[5.0] == pytest.approx(0.25 + 0.6, abs=1e-6)
        assert radii[10.0] == pytest.approx(0.25 + 0.6, abs=1e-6)

    def test_static_classes_do_not_grow(self):
        world = _confirmed_world('chair', width=0.45)
        world.view = OUT_OF_VIEW
        world.run(5.0)
        trk, mem = world.only()
        assert om.effective_radius(trk, mem, world.params) == pytest.approx(0.225)

    def test_tracked_person_does_not_grow(self):
        world = _confirmed_world('person', width=0.5)
        trk, mem = world.only()
        assert om.memory_growth_m('person', mem, world.params) == 0.0

    def test_missing_width_falls_back_to_the_class_nominal(self):
        world = _confirmed_world('person', width=None)
        trk, mem = world.only()
        assert om.track_base_radius(trk) == pytest.approx(0.25)   # 0.50 m shoulders


class TestReappearsWithTheSameId:

    def test_redetected_memory_object_keeps_its_id_and_is_tracked_again(self):
        world = _confirmed_world()
        track_id = world.only()[0].track_id
        world.view = OUT_OF_VIEW
        world.run(5.0)
        assert world.only()[1].state == om.MEMORY
        world.view = VISIBLE
        world.run(0.1, [('chair', 3.2, 0.1, 0.9, None, 0.45)])
        trk, mem = world.only()
        assert trk.track_id == track_id
        assert mem.state == om.TRACKED
        assert mem.confidence == 1.0 and mem.time_in_memory_s == 0.0
        assert [t.track_id for t in live_tracks(world.tracks, world.lost)] == [track_id]

    def test_memory_zeroes_the_velocity_so_matching_uses_the_stored_position(self):
        world = World()
        # Seen sliding +x: the EMA velocity becomes clearly non-zero.
        for k in range(6):
            world.step([('chair', 3.0 + 0.05 * k, 0.0, 0.9, None, 0.45)])
        trk = world.confirmed()[0]
        assert trk.vx_map > 0.1
        world.view = OUT_OF_VIEW
        world.run(5.0)
        trk = world.confirmed()[0]
        assert (trk.vx_map, trk.vy_map) == (0.0, 0.0)
        assert trk.predict(world.t + 10.0) == (trk.x_map, trk.y_map)

    def test_a_remembered_person_is_reacquired_inside_the_grown_gate(self):
        world = _confirmed_world('person', width=0.5)
        track_id = world.only()[0].track_id
        world.view = OUT_OF_VIEW
        world.run(2.0)                    # growth 0.4: gate 0.5 + 0.4
        world.view = VISIBLE
        world.run(0.1, [('person', 3.8, 0.0, 0.9, None, 0.5)])
        trk, mem = world.only()
        assert trk.track_id == track_id and mem.state == om.TRACKED
        assert trk.extra_gate_m == 0.0    # tracked again: normal gate

    def test_a_static_object_that_moved_further_than_the_gate_is_a_new_track(self):
        world = _confirmed_world('chair')
        old_id = world.only()[0].track_id
        world.view = OUT_OF_VIEW
        world.run(2.0)
        world.view = VISIBLE
        world.run(0.3, [('chair', 3.8, 0.0, 0.9, None, 0.45)])
        ids = {t.track_id for t in world.confirmed()}
        assert old_id in ids and len(ids) == 2
        # The old one is in view and never detected: gone 1.5 s later.
        world.run(1.5, [('chair', 3.8, 0.0, 0.9, None, 0.45)])
        assert old_id not in {t.track_id for t in world.confirmed()}


class TestMapOdomJump:

    def test_memory_object_stays_put_in_map_through_a_pose_jump(self):
        """A map -> odom correction moves the car's map pose, not the object."""
        robot = [(0.0, 0.0, 0.0)]
        world = _confirmed_world(x=3.0, y=1.0)
        trk = world.confirmed()[0]
        stored = (trk.x_map, trk.y_map)

        def view(trk):
            # Real geometry: the camera looks along the robot's heading.
            x_b, y_b = om.map_to_base(trk.x_map, trk.y_map, robot[0])
            stereo = om.stereo_frustum((x_b, y_b), 0.225, om.VisibilityParams())
            return om.Visibility(lidar=om.VIS_NO_DATA, stereo=stereo)

        world.view = view
        robot[0] = (0.0, 0.0, math.pi)    # facing away: MEMORY
        world.run(2.0)
        assert world.only()[1].state == om.MEMORY
        robot[0] = (0.7, -0.4, math.pi + 0.3)   # SLAM correction: the pose jumps
        world.run(2.0)
        trk, mem = world.only()
        assert (trk.x_map, trk.y_map) == stored
        assert mem.state == om.MEMORY

    def test_odom_consumers_follow_the_jump_through_the_current_edge(self):
        before = (0.0, 0.0, 0.0)
        after = (0.5, -0.2, 0.1)           # the map -> odom edge after the jump
        p_map = (3.0, 1.0)
        odom_before = map_point_to_odom(*p_map, before)
        odom_after = map_point_to_odom(*p_map, after)
        assert odom_before != pytest.approx(odom_after)
        # Both are the same map point: re-expressing, not caching, is the rule.
        assert odom_point_to_map(*odom_after, after) == pytest.approx(p_map)
        assert odom_point_to_map(*odom_before, before) == pytest.approx(p_map)


# ---------------------------------------------------------------------------
# lifecycle details

class TestLifecycleRules:

    def test_detection_wins_over_the_visibility_verdict(self):
        world = _confirmed_world()
        world.view = OUT_OF_VIEW
        world.run(1.0, [('chair', 3.0, 0.0, 0.9, None, 0.45)])
        assert world.only()[1].state == om.TRACKED

    def test_tentative_tracks_are_left_to_the_tracker(self):
        world = World()
        world.step([('chair', 3.0, 0.0, 0.9, None, 0.45)])
        assert world.memories == {}
        world.view = OUT_OF_VIEW
        world.run(1.0)                    # 10 misses > lost_miss_count: pruned
        assert world.tracks == []

    def test_disabled_memory_never_moves_or_removes(self):
        world = _confirmed_world(enabled=False, lost_miss_count=1000)
        world.view = OUT_OF_VIEW
        world.run(20.0)
        trk, mem = world.only()
        assert mem.state == om.TRACKED and mem.confidence == 1.0
        assert mem.time_since_seen_s == pytest.approx(20.0, abs=1e-6)

    def test_negative_and_huge_dt_are_clamped(self):
        mem = om.ObjectMemory()
        params = om.MemoryParams()
        assert om.step_memory(mem, -5.0, False, False, params)
        assert mem.time_in_memory_s == 0.0
        assert om.step_memory(mem, 50.0, False, False, params)
        assert mem.time_in_memory_s == om.MAX_STEP_DT_S

    @pytest.mark.parametrize('decay', [9.9, 20.1, 0.0])
    def test_memory_decay_outside_10_to_20_s_is_rejected(self, decay):
        with pytest.raises(ValueError):
            om.MemoryParams(memory_decay_s=decay)

    @pytest.mark.parametrize('decay', [10.0, 15.0, 20.0])
    def test_memory_decay_inside_10_to_20_s_is_accepted(self, decay):
        assert om.MemoryParams(memory_decay_s=decay).memory_decay_s == decay

    def test_in_fov_uses_the_sensor_that_can_detect_the_class(self):
        lidar_only = om.Visibility(lidar=om.VIS_VISIBLE, stereo=om.VIS_OUT_OF_ANGLE)
        assert not om.in_fov_for_class('person', lidar_only)     # a YOLO class
        assert om.in_fov_for_class('box', lidar_only, lidar_only_classes={'box'})


class TestTrackerHooks:

    def test_prune_confirmed_false_keeps_confirmed_but_prunes_tentative(self):
        ids = itertools.count()
        tracks = update_tracks_batch(
            [], [('chair', 0.0, 0.0, 0.9), ('cup', 5.0, 0.0, 0.9)], 0.0, 0.5, 0.3, 1, 2,
            lambda: next(ids))
        tracks[1].confirmed = False
        for k in range(3):
            tracks = update_tracks_batch(
                tracks, [], 1.0 + k, 0.5, 0.3, 1, 2, lambda: next(ids),
                prune_confirmed=False)
        assert [t.class_id for t in tracks] == ['chair']

    def test_extra_gate_widens_only_that_track(self):
        ids = itertools.count()
        tracks = update_tracks_batch([], [('chair', 0.0, 0.0, 0.9)], 0.0, 0.5, 0.3, 1, 5,
                                     lambda: next(ids))
        tracks[0].extra_gate_m = 0.4
        tracks = update_tracks_batch(tracks, [('chair', 0.85, 0.0, 0.9)], 1.0, 0.5, 0.3,
                                     1, 5, lambda: next(ids))
        assert len(tracks) == 1 and tracks[0].hit_count == 2


# ---------------------------------------------------------------------------
# visibility

def _scan(angle_min=-math.radians(135.0), angle_max=math.radians(135.0),
          inc=math.radians(0.25), background=10.0, occluder=None, range_max=10.0):
    """Build a LaserScan stand-in; occluder = (bearing, range) over +-3 deg."""
    count = int(round((angle_max - angle_min) / inc)) + 1
    ranges = [background] * count
    if occluder is not None:
        bearing, rng = occluder
        for i in range(count):
            if abs(angle_min + i * inc - bearing) <= math.radians(3.0):
                ranges[i] = rng
    return SimpleNamespace(angle_min=angle_min, angle_max=angle_max, angle_increment=inc,
                           range_min=0.02, range_max=range_max, ranges=ranges)


class TestLidarVisibility:

    def test_object_ahead_with_free_beams_is_visible(self):
        assert om.lidar_visibility((3.0, 0.0), 0.25, _scan(), 0.15) == om.VIS_VISIBLE

    def test_rear_blind_cone_is_out_of_view_from_the_scan_limits(self):
        assert om.lidar_visibility((-3.0, 0.0), 0.25, _scan(), 0.15) == om.VIS_OUT_OF_ANGLE
        rear = (3.0 * math.cos(math.radians(140.0)), 3.0 * math.sin(math.radians(140.0)))
        assert om.lidar_visibility(rear, 0.25, _scan(), 0.15) == om.VIS_OUT_OF_ANGLE
        side = (3.0 * math.cos(math.radians(130.0)), 3.0 * math.sin(math.radians(130.0)))
        assert om.lidar_visibility(side, 0.25, _scan(), 0.15) == om.VIS_VISIBLE

    def test_the_sector_is_the_messages_own_not_the_devices(self):
        narrow = _scan(angle_min=-1.0, angle_max=1.0)
        p = (3.0 * math.cos(1.2), 3.0 * math.sin(1.2))
        assert om.lidar_visibility(p, 0.25, narrow, 0.15) == om.VIS_OUT_OF_ANGLE

    def test_a_scan_claiming_more_sector_than_it_has_beams_is_trimmed(self):
        scan = _scan(angle_min=-1.0, angle_max=1.0)
        scan.angle_max = 3.0              # the +-3.14 request, not the delivery
        p = (3.0 * math.cos(1.5), 3.0 * math.sin(1.5))
        assert om.lidar_visibility(p, 0.25, scan, 0.15) == om.VIS_OUT_OF_ANGLE

    def test_beyond_range_max_is_out_of_range(self):
        assert om.lidar_visibility((12.0, 0.0), 0.25, _scan(), 0.15) == om.VIS_OUT_OF_RANGE

    def test_a_closer_return_on_the_bearing_occludes(self):
        scan = _scan(occluder=(0.0, 1.5))
        assert om.lidar_visibility((3.0, 0.0), 0.25, scan, 0.15) == om.VIS_OCCLUDED

    def test_the_objects_own_surface_does_not_occlude_it(self):
        scan = _scan(occluder=(0.0, 2.8))    # near surface at 2.75
        assert om.lidar_visibility((3.0, 0.0), 0.25, scan, 0.15) == om.VIS_VISIBLE

    def test_no_return_beams_never_occlude(self):
        scan = _scan(background=math.inf)
        scan.ranges[540] = math.nan
        assert om.lidar_visibility((3.0, 0.0), 0.25, scan, 0.15) == om.VIS_VISIBLE

    def test_a_thin_occluder_across_part_of_the_object_does_not_hide_it(self):
        scan = _scan()
        scan.ranges[540] = 1.0               # one beam of the ~19 spanning it
        assert om.lidar_visibility((3.0, 0.0), 0.25, scan, 0.15) == om.VIS_VISIBLE

    def test_no_scan_is_no_data(self):
        assert om.lidar_visibility((3.0, 0.0), 0.25, None, 0.15) == om.VIS_NO_DATA


class TestStereoVisibility:

    VP = om.VisibilityParams()

    def test_ahead_inside_the_depth_window_is_visible(self):
        assert om.stereo_frustum((3.0, 0.0), 0.25, self.VP) == om.VIS_VISIBLE

    def test_outside_the_101_deg_frustum_is_out_of_angle(self):
        assert om.stereo_frustum((3.0, 4.0), 0.25, self.VP) == om.VIS_OUT_OF_ANGLE
        assert om.stereo_frustum((-3.0, 0.0), 0.25, self.VP) == om.VIS_OUT_OF_ANGLE

    def test_the_whole_footprint_must_be_inside(self):
        bearing = math.radians(48.0)         # centre inside 50.5, edge outside
        p = (3.0 * math.cos(bearing), 3.0 * math.sin(bearing))
        assert om.stereo_frustum(p, 0.0, self.VP) == om.VIS_VISIBLE
        assert om.stereo_frustum(p, 0.25, self.VP) == om.VIS_OUT_OF_ANGLE

    def test_outside_the_depth_window_is_out_of_range(self):
        assert om.stereo_frustum((0.5, 0.0), 0.25, self.VP) == om.VIS_OUT_OF_RANGE
        assert om.stereo_frustum((6.0, 0.0), 0.25, self.VP) == om.VIS_OUT_OF_RANGE

    def test_a_dead_camera_sees_nothing(self):
        vis = om.object_visibility(0.25, None, None, (3.0, 0.0), False, None, None, self.VP)
        assert vis.stereo == om.VIS_NO_DATA

    def test_a_grown_person_needs_the_grown_disk_inside(self):
        bearing = math.radians(35.0)
        p = (3.0 * math.cos(bearing), 3.0 * math.sin(bearing))
        assert om.object_visibility(0.25, None, None, p, True, None, None, self.VP).in_fov_stereo
        assert not om.object_visibility(
            0.25, None, None, p, True, None, None, self.VP, view_radius=0.85).in_fov_stereo


def _columns(distance, width=640, fx=320.0, cx=320.0):
    return SimpleNamespace(width=width, fx=fx, cx=cx, distance_m=[distance] * width)


class TestStereoColumnsOcclusion:

    def test_nearer_wall_depth_across_the_object_occludes(self):
        assert om.columns_occluded((0.0, 0.0, 3.0), 0.25, _columns(1.5), 0.15) is True

    def test_wall_behind_the_object_does_not(self):
        assert om.columns_occluded((0.0, 0.0, 3.0), 0.25, _columns(5.0), 0.15) is False

    def test_empty_columns_say_nothing(self):
        assert om.columns_occluded((0.0, 0.0, 3.0), 0.25, _columns(math.nan), 0.15) is False

    def test_outside_the_image_cannot_say(self):
        assert om.columns_occluded((10.0, 0.0, 1.0), 0.25, _columns(1.0), 0.15) is None
        assert om.columns_occluded((0.0, 0.0, -1.0), 0.25, _columns(1.0), 0.15) is None

    def test_object_visibility_reports_depth_occlusion(self):
        vis = om.object_visibility(0.25, None, None, (3.0, 0.0), True, (0.0, 0.0, 3.0),
                                   _columns(1.5), om.VisibilityParams())
        assert vis.stereo == om.VIS_OCCLUDED_DEPTH

    def test_lidar_occlusion_for_the_camera_can_be_switched_off(self):
        vp = om.VisibilityParams(stereo_occlusion_use_lidar=False)
        vis = om.object_visibility(0.25, (3.0, 0.0), _scan(occluder=(0.0, 1.5)), (3.0, 0.0),
                                   True, None, None, vp)
        assert vis.lidar == om.VIS_OCCLUDED and vis.stereo == om.VIS_VISIBLE


class TestGeometry:

    def test_map_to_base_inverts_compose_base_link_to_map(self):
        pose = (1.0, -2.0, 0.7)
        x_m, y_m = compose_base_link_to_map(2.5, 0.3, pose)
        assert om.map_to_base(x_m, y_m, pose) == pytest.approx((2.5, 0.3))

    def test_laser_lookup_puts_a_base_point_in_the_laser_frame(self):
        # lookup_transform('laser', 'base_link') for laser at (0.305, 0, 0.190).
        tf = ((-0.305, 0.0, -0.190), (0.0, 0.0, 0.0, 1.0))
        assert om.source_origin_height(tf) == pytest.approx(0.190)
        assert om.apply_transform(tf, (3.305, 0.0, 0.190)) == pytest.approx((3.0, 0.0, 0.0))

    def test_rotation_is_a_proper_quaternion_rotation(self):
        q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # yaw +90 deg
        assert om.rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
