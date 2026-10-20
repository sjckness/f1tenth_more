"""go_to_person when the detector loses the person close in: the target is never an obstacle.

The end-to-end rig of test_go_to_object_end_to_end (the real GoToObject,
CheckStopCondition, goal_object_callback, corridor, target and solver), with
a fake projector and semantic layer around it. Each DRIVE tick's obstacle
list is MPC_corr's own _planning_obstacles and the rear check's objects its
own _reverse_env, so what is tested is the shipped assembly:

- while detected, the person is on /perception/obstacles_2d (live) and TRACKED
  on /costmap/semantic_objects;
- from the moment the front bumper is `lost_gap` from the person's near edge
  (the go_to gap measure), the detector reports nothing: no live circle, and
  the semantic layer holds the person in MEMORY at once, its radius growing at
  the shipped person growth (the worst case: in view it would stay TRACKED
  for fov_miss_remove_s). The BT's /costmap/semantic_tracks keeps the track
  TRACK_HOLD_S longer, as semantic_layer_node does until lost_miss_count
  missed batches, so the approach keeps driving that long;
- optionally a DIFFERENT person (another id) in MEMORY beside the approach,
  which must stay an obstacle with its grown radius.

Every DRIVE tick is checked, before the loss too: while detected, the person
was on the live list, and before the target was excluded it was an obstacle
there on every tick of the approach.

THE GEOMETRY. With the shipped person gap of 1.1 m (0.6 until 2026-10-20,
before obstacle_class_margin_m {person: 0.5}) the stop latch trips at a gap
of about 1.34 m (object_reach_tol_m + object_stop_distance_m past it), so a
loss at 1.3 m comes during the stop-and-wait, where no solve runs: the
approach completes. A loss before the latch (here 1.7 m) gives DRIVE ticks on the
remembered, grown target -- none of which may carry it -- and then ends
target_lost: the handler waits in GRACE at speed 0 once its tracks are empty,
because /costmap/semantic_tracks does not carry MEMORY objects. Excluding the
target in mpc_corr does not change that (report, Part B fixes).
"""

import math

from f1tenth_messages.msg import SemanticObject, SemanticObjectArray
from f1tenth_params.object_geometry import class_margin_for, obstacle_filter
from f1tenth_params.param_defaults import get_value
from f1tenth_params.semantic_objects import map_point_to_odom, odom_point_to_map
from mpc_controller.MPC_corr import MPCController
import pytest

import test_go_to_object_end_to_end as e2e
import test_object_approach_closed_loop as rig

# The shipped person gap (1.1 m) + 0.2 and + 0.6: the same two places in the
# approach the 0.8 / 1.2 m losses were when the gap was 0.6 m.
LOST_GAP_M = 1.3
EARLY_LOST_GAP_M = 1.7
# semantic_layer_node keeps a lost confirmed track on /costmap/semantic_tracks
# until lost_miss_count (5) consecutive missed batches: ~0.4 s at 12.5 Hz.
TRACK_HOLD_S = 0.4
GATE = float(get_value('mpc_target_exclusion_gate_m'))
GROWTH_MPS = float(get_value('semantic_layer_person_memory_growth_mps'))
GROWTH_MAX_M = float(get_value('semantic_layer_person_memory_growth_max_m'))
PERSON_ODOM = map_point_to_odom(*e2e.PERSON_MAP, e2e.MAP_ODOM)
OTHER_RADIUS, OTHER_GROWN = 0.25, 0.55


def _other_person_odom(lateral_m):
    """Return a point beside the approach line's midpoint, `lateral_m` to its left."""
    px, py = PERSON_ODOM
    d = math.hypot(px, py)
    return 0.5 * px - lateral_m * py / d, 0.5 * py + lateral_m * px / d


def _semantic(oid, xy_map, r, r_eff, state, conf=1.0):
    o = SemanticObject()
    o.id, o.class_id = oid, 'person'
    o.x, o.y = float(xy_map[0]), float(xy_map[1])
    o.radius_m, o.effective_radius_m = float(r), float(r_eff)
    o.state, o.confidence, o.z_m = state, float(conf), 0.9
    return o


class _World:
    """The fake projector and semantic layer; records what MPC_corr assembled."""

    def __init__(self, other_odom=None, lost_gap=LOST_GAP_M):
        self.other_odom = other_odom
        self.lost_gap = lost_gap
        self.lost_t = None
        self.t = None
        self.drive_after_loss = 0
        self.target_grown = []
        self.planning = []
        self.rear = []
        self.car = []

    @property
    def detected(self):
        return self.lost_t is None

    def detect(self, x, spec):
        """Run the detector this tick; return whether the BT's tracks still carry the person."""
        self.t = self.t + e2e.TS if self.t is not None else 0.0
        gap = (math.hypot(PERSON_ODOM[0] - x[0], PERSON_ODOM[1] - x[1])
               - spec.front_x_m - e2e.PERSON_RADIUS)
        if self.lost_t is None and gap <= self.lost_gap:
            self.lost_t = self.t
        return self.lost_t is None or self.t - self.lost_t < TRACK_HOLD_S

    def obstacles(self, mpc, x, _tracks_seen):
        t = self.t
        stamp = mpc.clock.t
        self._wire(mpc)
        detected = self.detected
        if not detected:
            self.drive_after_loss += 1
        mpc.obstacles_global_live = (
            [(PERSON_ODOM[0], PERSON_ODOM[1], e2e.PERSON_RADIUS)] if detected else [])
        mpc.obstacles_live_time = stamp
        if detected:
            target = _semantic(7, e2e.PERSON_MAP, e2e.PERSON_RADIUS, e2e.PERSON_RADIUS,
                               SemanticObject.TRACKED)
        else:
            grown = e2e.PERSON_RADIUS + min(GROWTH_MPS * (t - self.lost_t), GROWTH_MAX_M)
            self.target_grown.append(grown)
            target = _semantic(7, e2e.PERSON_MAP, e2e.PERSON_RADIUS, grown,
                               SemanticObject.MEMORY, conf=0.9)
        msg = SemanticObjectArray()
        msg.objects = [target]
        if self.other_odom is not None:
            msg.objects.append(_semantic(
                9, odom_point_to_map(*self.other_odom, e2e.MAP_ODOM), OTHER_RADIUS,
                OTHER_GROWN, SemanticObject.MEMORY, conf=0.7))
        mpc._semantic_msg, mpc._semantic_t = msg, stamp
        planning = MPCController._planning_obstacles(mpc)
        self.planning.append(list(planning))
        self.rear.append(list(MPCController._reverse_env(mpc, []).objects))
        self.car.append((float(x[0]), float(x[1])))
        return planning

    @staticmethod
    def _wire(mpc):
        """Set the consumer's attributes MPC_corr's __init__ sets on the rig's stand-in."""
        if getattr(mpc, '_wired', False):
            return
        mpc.semantic_memory_enabled = True
        mpc.semantic_memory_max_age_s = float(get_value('mpc_semantic_memory_max_age_s'))
        mpc.semantic_memory_min_confidence = float(
            get_value('mpc_semantic_memory_min_confidence'))
        mpc._semantic_class_margins = get_value('obstacle_class_margin_m')
        mpc._obstacle_filter = obstacle_filter()
        mpc.target_exclusion_gate_m = GATE
        mpc._rear_grid = None
        mpc._bt_allow = mpc._bt_allow_t = None
        mpc._emergency_stop = False
        # obstacles v2: the live list is aged and capped (_solve_obstacles).
        mpc.obstacles_max_age_s, mpc.obstacles_max = 1.0, 8
        mpc.obstacles_topic = '/costmap/semantic_obstacles'
        mpc._wired = True


def _run(world):
    run = e2e._run_go_to_person(detect=world.detect, obstacles_fn=world.obstacles)
    run.world = world
    return run


def _near(circles, xy, tol):
    return [c for c in circles if math.hypot(c[0] - xy[0], c[1] - xy[1]) <= tol]


@pytest.fixture(scope='module')
def lost():
    return _run(_World())


@pytest.fixture(scope='module')
def lost_with_other():
    return _run(_World(other_odom=_other_person_odom(0.7)))


@pytest.fixture(scope='module')
def lost_early():
    return _run(_World(lost_gap=EARLY_LOST_GAP_M))


class TestTheTargetLostAtPointEightMetres:

    def test_the_detector_did_lose_it_after_the_latch(self, lost):
        assert lost.world.lost_t is not None
        assert lost.world.drive_after_loss == 0, 'the geometry this class pins changed'

    def test_the_car_still_completes_the_approach(self, lost):
        assert lost.reached_at is not None, 'object_reached never fired'
        assert lost.state.object_record.outcome == 'reached'
        d = math.hypot(PERSON_ODOM[0] - lost.x[0], PERSON_ODOM[1] - lost.x[1])
        gap = d - lost.spec.front_x_m - e2e.PERSON_RADIUS
        lo, hi = rig.stop_window()
        assert lost.spec.gap_m + lo <= gap <= lost.spec.gap_m + hi

    def test_the_target_was_never_an_obstacle_live_or_remembered(self, lost):
        assert lost.world.planning, 'no DRIVE tick ran'
        assert all(_near(c, PERSON_ODOM, GATE) == [] for c in lost.world.planning)

    def test_the_target_never_reached_the_rear_check(self, lost):
        assert all(_near(c, PERSON_ODOM, GATE) == [] for c in lost.world.rear)


class TestADifferentPersonInMemoryIsStillAvoided:

    def test_it_is_an_obstacle_every_tick_with_its_grown_radius(self, lost_with_other):
        other = lost_with_other.world.other_odom
        # The grown radius plus the person's class margin (objects_to_odom_circles).
        margin = class_margin_for(get_value('obstacle_class_margin_m'), 'person')
        for circles in lost_with_other.world.planning:
            [c] = _near(circles, other, 0.05)
            assert c[2] == pytest.approx(OTHER_GROWN + margin)
        assert all(len(_near(c, other, 0.05)) == 1 for c in lost_with_other.world.rear)

    def test_the_car_steers_away_from_it(self, lost, lost_with_other):
        other = lost_with_other.world.other_odom

        def closest(run):
            return min(math.hypot(cx - other[0], cy - other[1]) for cx, cy in run.world.car)

        assert closest(lost_with_other) > closest(lost) + 0.02
        assert closest(lost_with_other) >= OTHER_GROWN + rig.CAR_RADIUS

    def test_and_the_approach_still_completes(self, lost_with_other):
        assert lost_with_other.state.object_record.outcome == 'reached'
        assert all(_near(c, PERSON_ODOM, GATE) == [] for c in lost_with_other.world.planning)


class TestALossBeforeTheLatch:
    """The remembered target, grown, on DRIVE ticks: still never an obstacle."""

    def test_the_approach_drove_on_the_remembered_grown_target(self, lost_early):
        assert lost_early.world.drive_after_loss >= 3
        assert max(lost_early.world.target_grown) > e2e.PERSON_RADIUS + 0.05

    def test_no_drive_tick_carried_it(self, lost_early):
        assert all(_near(c, PERSON_ODOM, GATE) == [] for c in lost_early.world.planning)
        assert all(_near(c, PERSON_ODOM, GATE) == [] for c in lost_early.world.rear)

    def test_the_bt_then_ends_the_move_target_lost_unchanged_by_this_fix(self, lost_early):
        """A coupling with the go_to handler's GRACE: a handler that follows memory changes it."""
        assert lost_early.state.object_record.outcome == 'target_lost'
