"""
The merged semantic layer: object memory (Part B) and unknown objects (obstacles v2).

One track list; each track is removed by its own sensor's rule; a recognised
track is "current" (owns its LiDAR cells, is on /costmap/semantic_obstacles)
by one predicate; a remembered one is on neither and never gets a parallel
unknown track; one the memory lets go while the LiDAR still hits it goes on
as a confirmed unknown; one plausibility gate for live and remembered.
Pure functions only, the node's per-cycle order by hand.
"""

import itertools
import math

from f1tenth_costmap import object_memory as om
from f1tenth_costmap import semantic_layer as sl
from f1tenth_costmap import unknown_objects as uo
from f1tenth_params.object_geometry import obstacle_filter, passes_obstacle_filter

P = uo.UnknownParams()
MEM = om.MemoryParams()
POSE = (0.0, 0.0, 0.0)
LIMITS = obstacle_filter()          # stack_params' obstacle_z_* / *_obstacle_radius
Z_FLOOR_OBJECT = 0.45               # a chair's centre height


class World:
    """The node's lists and the order of its cycles, minus ROS."""

    def __init__(self):
        self.tracks = []
        self.memory = {}
        self.ids = itertools.count(1)
        self.t = 0.0

    def camera(self, detections):
        self.tracks = sl.update_tracks_batch(
            self.tracks, detections, self.t, 0.5, 0.3, 3, 5, lambda: next(self.ids),
            prune_confirmed=False)
        self.tracks, _ = sl.dedupe_tracks(self.tracks, 0.5, self.fold)

    def lidar(self, candidates, observe=lambda trk: (True, False)):
        self.tracks, stats = sl.update_unknown_batch(
            self.tracks, candidates, self.t, P, 0.5, 0.3, lambda: next(self.ids), observe,
            can_absorb=self.plausible)
        self.tracks, _ = sl.dedupe_tracks(self.tracks, 0.5, self.fold, self.plausible)
        return stats

    def remember(self, in_view, dt=0.1):
        self.tracks = om.memory_cycle(
            self.tracks, self.memory, dt,
            lambda trk: om.Visibility(stereo=om.VIS_VISIBLE if in_view else om.VIS_NO_DATA),
            MEM, expire=lambda trk: self.plausible(trk) and sl.hand_over_to_lidar(
                trk, self.t, P.decay_s))

    @staticmethod
    def plausible(trk):
        return uo.plausible(trk, LIMITS)

    def fold(self, survivor, folded):
        mem = self.memory.pop(folded.track_id, None)
        if mem is not None:
            mem.last_hit_count = survivor.hit_count
            self.memory[survivor.track_id] = mem

    def current(self):
        return uo.current_recognised(self.tracks, self.t, 2.0, self.memory, LIMITS)

    def tick(self, dt=0.1):
        self.t += dt


def chair_seen(world, cycles=4, z=Z_FLOOR_OBJECT):
    for _ in range(cycles):
        world.camera([('chair', 1.5, 0.0, 0.9, None, 0.4, z)])
        world.remember(in_view=True)
        world.tick()
    (chair,) = [t for t in world.tracks if t.class_id == 'chair']
    return chair


def test_a_seen_object_is_current_owns_its_cells_and_is_on_the_mpc_list():
    world = World()
    chair = chair_seen(world)
    assert world.memory[chair.track_id].state == om.TRACKED
    assert world.current() == [chair]
    ids = {t.track_id for t in world.current()}
    assert uo.obstacles_for_mpc(world.tracks, POSE, 'active', current_ids=ids)


def test_a_remembered_object_is_off_the_live_list_and_its_cluster_is_absorbed():
    world = World()
    chair = chair_seen(world)
    world.remember(in_view=False)                 # the camera looks away: MEMORY
    assert world.memory[chair.track_id].state == om.MEMORY
    assert world.current() == []
    assert uo.obstacles_for_mpc(world.tracks, POSE, 'active', current_ids=set()) == []
    # The LiDAR still sees it: absorbed into the remembered track, no unknown.
    for _ in range(10):
        stats = world.lidar([uo.Candidate(1.5, 0.0, 0.4, 0.4, 0.0, 12)])
        world.remember(in_view=False)
        world.tick()
    assert stats['absorbed'] == 1 and stats['spawned'] == 0
    assert [t.track_id for t in world.tracks] == [chair.track_id]
    # ...and the LiDAR never refreshes the memory: still MEMORY, still decaying.
    assert world.memory[chair.track_id].state == om.MEMORY
    assert world.memory[chair.track_id].time_in_memory_s > 0.9


def test_without_lidar_hits_the_expired_object_goes_and_a_later_cluster_is_new():
    world = World()
    chair = chair_seen(world)
    for _ in range(int(MEM.memory_decay_s) + 1):             # decayed: removed
        world.remember(in_view=False, dt=1.0)
    assert chair.track_id not in [t.track_id for t in world.tracks]
    for _ in range(8):
        world.lidar([uo.Candidate(1.5, 0.0, 0.4, 0.4, 0.0, 12)])
        world.tick()
    (unknown,) = [t for t in world.tracks if t.is_unknown]
    assert unknown.confirmed and unknown.track_id != chair.track_id


def test_the_memory_never_touches_an_unknown_track():
    world = World()
    for _ in range(8):
        world.lidar([uo.Candidate(1.5, 0.0, 0.3, 0.3, 0.0, 10)])
        world.tick()
    (unknown,) = [t for t in world.tracks if t.is_unknown and t.confirmed]
    for _ in range(30):                          # 3 s in view of a camera that never sees it
        world.remember(in_view=True)
        world.lidar([uo.Candidate(1.5, 0.0, 0.3, 0.3, 0.0, 10)])
        world.tick()
    assert unknown in world.tracks and unknown.track_id not in world.memory


def test_a_fold_into_an_older_unknown_keeps_the_recognised_memory():
    world = World()
    for _ in range(8):
        world.lidar([uo.Candidate(1.5, 0.0, 0.3, 0.3, 0.0, 10)])
        world.tick()
    (unknown,) = [t for t in world.tracks if t.is_unknown]
    # A recognised track the camera made apart from it, then remembered.
    cone = sl.SemanticObject(999, 'cone', 1.6, 0.05, 0.9, world.t, 1, width=0.3)
    world.tracks.append(cone)
    world.remember(in_view=True)
    world.remember(in_view=False)
    assert world.memory[999].state == om.MEMORY
    world.tracks, merged = sl.dedupe_tracks(world.tracks, 0.5, world.fold)
    assert merged == 1
    (survivor,) = world.tracks
    assert (survivor.track_id, survivor.class_id) == (unknown.track_id, 'cone')
    # The memory moved with the identity, and the fold is not a detection.
    world.remember(in_view=False)
    assert world.memory[survivor.track_id].state == om.MEMORY
    assert 999 not in world.memory


def test_a_recognised_survivor_does_not_count_the_lidar_hits():
    chair = sl.SemanticObject(1, 'chair', 1.5, 0.0, 0.9, 0.0, 1, width=0.4)
    chair.hit_count = 7
    unknown = sl.SemanticObject(2, uo.UNKNOWN_CLASS, 1.55, 0.0, 0.0, 0.0, 1, width=0.3)
    unknown.hit_count = 30
    tracks, merged = sl.dedupe_tracks([chair, unknown], 0.5)
    assert merged == 1 and tracks == [chair] and chair.hit_count == 7


def test_one_radius_for_every_consumer():
    person = sl.SemanticObject(1, 'person', 0.0, 0.0, 0.9, 0.0, 1)
    assert person.radius() == om.track_base_radius(person) == 0.25
    unknown = sl.SemanticObject(2, uo.UNKNOWN_CLASS, 0.0, 0.0, 0.0, 0.0, 1, width=0.3)
    unknown.extent = (1.0, 0.3)
    assert unknown.radius() == 0.5


def test_a_remembered_object_the_lidar_still_hits_goes_on_as_a_confirmed_unknown():
    """1a: MEMORY expiry -> confirmed unknown, same id, same place, no cycle without it."""
    world = World()
    chair = chair_seen(world)
    cluster = [uo.Candidate(1.5, 0.0, 0.4, 0.4, 0.0, 12)]
    on_list = []
    for _ in range(int(MEM.memory_decay_s / 1.0) + 3):
        world.lidar(cluster)                      # absorbed while remembered
        world.remember(in_view=False, dt=1.0)
        world.tick(1.0)
        ids = {t.track_id for t in world.current()}
        # live list (TRACKED + active unknowns) or the remembered path: never neither
        live = uo.obstacles_for_mpc(world.tracks, POSE, 'active', current_ids=ids)
        remembered = [t for t in world.tracks if t.track_id in world.memory
                      and world.memory[t.track_id].state == om.MEMORY]
        on_list.append(bool(live) or bool(remembered))
    assert all(on_list)
    (trk,) = world.tracks
    assert trk.track_id == chair.track_id and trk.is_unknown and trk.confirmed
    assert trk.track_id not in world.memory
    assert math.hypot(trk.x_map - 1.5, trk.y_map) < 0.05
    # From now on the LiDAR owns it: it goes once the rays pass through.
    for _ in range(int(P.decay_s / 0.1) + 3):
        world.lidar([], observe=lambda t: (True, True))
        world.tick()
    assert world.tracks == []


def test_one_plausibility_gate_for_live_and_remembered():
    """1b: an object outside the height band is no obstacle live or remembered."""
    world = World()
    lamp = chair_seen(world, z=2.6)               # above obstacle_z_max
    assert not passes_obstacle_filter(2.6, lamp.radius(), LIMITS)
    assert world.current() == []                  # not current: no cells, not live
    assert uo.obstacles_for_mpc(world.tracks, POSE, 'active', current_ids=set()) == []
    # Remembered, it fails the same function mpc_corr applies to /costmap/semantic_objects.
    world.remember(in_view=False)
    assert world.memory[lamp.track_id].state == om.MEMORY
    assert not passes_obstacle_filter(lamp.z_base, om.track_base_radius(lamp), LIMITS)
    # And it hides nothing: the cluster under it becomes an unknown of its own.
    for _ in range(8):
        stats = world.lidar([uo.Candidate(1.5, 0.0, 0.3, 0.3, 0.0, 10)])
        world.tick()
    assert stats['absorbed'] == 0
    assert [t.is_unknown for t in world.tracks if t.confirmed].count(True) == 1


def test_an_implausible_object_is_never_handed_over():
    world = World()
    lamp = chair_seen(world, z=2.6)
    for _ in range(int(MEM.memory_decay_s) + 2):
        world.remember(in_view=False, dt=1.0)
    assert lamp not in world.tracks
