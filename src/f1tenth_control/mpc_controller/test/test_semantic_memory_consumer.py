"""
mpc_corr's side of the object memory (Part B4).

MEMORY objects from /costmap/semantic_objects join the MPC's obstacle list
(TRACKED ones arrive live on /perception/obstacles_2d already), and every
semantic object reaches the reverse manoeuvre's rear pre-check, all
re-expressed through the current map -> odom. Every one passes
obstacle_projector_node's height/radius gate first, and the current object
move's target is in no list at all, live or remembered.
"""

import math
from types import SimpleNamespace

from f1tenth_messages.msg import SemanticObject, SemanticObjectArray
from f1tenth_params.object_geometry import obstacle_filter
from f1tenth_params.param_defaults import get_value
from f1tenth_params.semantic_objects import objects_to_odom_circles
from mpc_controller.MPC_corr import MPCController
import pytest

import reverse_manoeuvre_harness as H

GATE = 0.5


def _obj(oid, cls, x, y, r, r_eff, state, conf=1.0, z=0.8):
    o = SemanticObject()
    o.id, o.class_id, o.x, o.y = oid, cls, float(x), float(y)
    o.radius_m, o.effective_radius_m = float(r), float(r_eff)
    o.state, o.confidence = state, float(conf)
    o.z_m = float(z)
    return o


class _Clock:
    def __init__(self, t):
        self.t = t

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.t * 1e9))


class _Stub(SimpleNamespace):
    """The attributes these methods read; every other MPCController method bound to it."""

    def __getattr__(self, name):
        method = getattr(MPCController, name, None)
        if callable(method):
            return method.__get__(self, _Stub)
        raise AttributeError(name)


def _node(objects, age=0.1, map_odom=(1.0, 0.0, 0.0), min_conf=0.0, enabled=True,
          live=(), target=None):
    msg = SemanticObjectArray()
    msg.objects = objects
    node = _Stub(
        semantic_memory_enabled=enabled, semantic_memory_max_age_s=0.5,
        semantic_memory_min_confidence=min_conf, _semantic_class_margins={},
        _obstacle_filter=obstacle_filter(), target_exclusion_gate_m=GATE,
        _semantic_msg=msg, _semantic_t=10.0 - age, obstacles_global_live=list(live),
        _rear_grid=None, _bt_allow=None, _bt_allow_t=None, hold=False,
        _emergency_stop=False, object_psi_c=None, goal_object_map_xy=None,
        # obstacles v2: the live list is aged and capped (_solve_obstacles).
        obstacles_live_time=10.0, obstacles_max_age_s=1.0, obstacles_max=8,
        obstacles_topic='/costmap/semantic_obstacles', x=0.0, y=0.0)
    node.get_clock = lambda: _Clock(10.0)
    node._lookup_map_odom = lambda: map_odom
    if target is not None:
        _aim(node, *target, map_odom=map_odom)
    return node


def _aim(node, track_id, cls, map_xy, map_odom=(1.0, 0.0, 0.0)):
    """Put the stub in an object move whose target is `map_xy`, as goal_object_callback does."""
    tx, ty, tyaw = map_odom
    c, s = math.cos(tyaw), math.sin(tyaw)
    dx, dy = map_xy[0] - tx, map_xy[1] - ty
    node.object_psi_c = 0.0
    node.goal_object_track_id, node.goal_object_target_class = track_id, cls
    node.goal_object_map_xy = tuple(map_xy)
    node.goal_object_odom_xy = (c * dx + s * dy, -s * dx + c * dy)


OBJS = [_obj(1, 'chair', 3.0, 0.0, 0.2, 0.2, SemanticObject.TRACKED),
        _obj(2, 'person', 0.0, 1.0, 0.25, 0.65, SemanticObject.MEMORY, conf=0.4)]


def test_memory_objects_join_the_obstacle_list_tracked_ones_do_not():
    circles = MPCController._semantic_circles(_node(OBJS), memory_only=True)
    # map (0, 1) through map -> odom (1, 0, 0) is odom (-1, 1); the person's
    # GROWN radius, not its base one.
    assert circles == [pytest.approx((-1.0, 1.0, 0.65))]


def test_the_rear_check_gets_every_object():
    circles = MPCController._semantic_circles(_node(OBJS), memory_only=False)
    assert len(circles) == 2


@pytest.mark.parametrize('kw', [{'age': 0.8}, {'map_odom': None}, {'enabled': False}])
def test_stale_unplaceable_or_disabled_gives_nothing(kw):
    assert MPCController._semantic_circles(_node(OBJS, **kw), memory_only=False) == []


def test_low_confidence_memory_is_left_out():
    assert MPCController._semantic_circles(_node(OBJS, min_conf=0.5), memory_only=True) == []


def test_the_reverse_env_carries_the_live_obstacles_and_every_semantic_object():
    """MPC_corr's own wiring: _reverse_env appends ALL semantic objects, TRACKED and MEMORY."""
    live = [(5.0, 5.0, 0.3)]
    env = MPCController._reverse_env(_node(OBJS, live=live), [])
    # map -> odom (1, 0, 0): the chair (3, 0) -> (2, 0) and the person (0, 1) -> (-1, 1).
    assert env.objects == [live[0], pytest.approx((2.0, 0.0, 0.2)),
                           pytest.approx((-1.0, 1.0, 0.65))]


def test_the_reverse_env_gets_no_semantic_object_from_a_stale_message():
    live = [(5.0, 5.0, 0.3)]
    assert MPCController._reverse_env(_node(OBJS, age=0.8, live=live), []).objects == live


def test_the_planning_list_is_live_plus_memory_once():
    live = [(5.0, 5.0, 0.3)]
    obstacles = MPCController._planning_obstacles(_node(OBJS, live=live))
    assert obstacles == [live[0], pytest.approx((-1.0, 1.0, 0.65))]


def test_outside_an_object_move_the_live_list_is_passed_through_unchanged():
    node = _node([], live=[(5.0, 5.0, 0.3)])
    # Equal, not the same list: since obstacles v2 it is a fresh, capped copy.
    assert MPCController._live_obstacles(node) == node.obstacles_global_live


def test_a_remembered_person_behind_refuses_the_reverse():
    cfg = H.shipped_manoeuvre_config()
    # A person the car drove past: out of the camera's view now, in MEMORY,
    # radius grown to 0.65. In map = odom here; 0.3 m behind the rear bumper
    # of where the car will be stuck (base_link x ~ 1.66).
    person = _obj(7, 'person', 1.66 - cfg.car_rear_offset_m - 0.3 - 0.65, 0.0, 0.25, 0.65,
                  SemanticObject.MEMORY, conf=0.5)
    memory = objects_to_odom_circles([person], (0.0, 0.0, 0.0))
    r = H.run(H.Script(objects=lambda t: memory), duration=12.0, front_wall_x=2.0)
    assert H.reverses(r) == []
    stuck_ticks = [s for s in r.status if s['stuck']]
    assert stuck_ticks and all(s['rear_reason'] == 'rear_object' for s in stuck_ticks)


# -- the projector's gate, applied to semantic objects (Part B fix b) ---------

class TestTheSameGateAsTheProjector:
    """passes_obstacle_filter on (z_m, radius_m), with stack_params.yaml's bounds."""

    Z_MAX = float(get_value('obstacle_z_max'))
    Z_MIN = float(get_value('obstacle_z_min'))
    R_MIN = float(get_value('min_obstacle_radius'))
    R_MAX = float(get_value('max_obstacle_radius'))

    def _memory(self, **kw):
        base = {'oid': 5, 'cls': 'person', 'x': 0.0, 'y': 1.0, 'r': 0.25, 'r_eff': 0.25,
                'state': SemanticObject.MEMORY}
        base.update(kw)
        return _obj(base.pop('oid'), base.pop('cls'), base.pop('x'), base.pop('y'),
                    base.pop('r'), base.pop('r_eff'), base.pop('state'), **base)

    @pytest.mark.parametrize('kw', [
        {'z': Z_MAX + 0.01},                  # overhead: signage, a poster
        {'z': Z_MIN - 0.01},                  # below the floor: ground false positive
        {'z': math.nan},                      # no detection ever carried a height
        {'r': R_MAX + 0.01, 'r_eff': R_MAX + 0.01},
        {'r': R_MIN - 0.005, 'r_eff': 0.5},   # noise-sized: the GROWTH does not rescue it
    ])
    def test_what_the_projector_drops_live_is_dropped_from_memory(self, kw):
        node = _node([self._memory(**kw)])
        assert MPCController._semantic_circles(node, memory_only=True) == []
        assert MPCController._semantic_circles(node, memory_only=False) == []

    @pytest.mark.parametrize('kw', [
        {'z': Z_MAX}, {'z': Z_MIN}, {'r': R_MAX, 'r_eff': R_MAX}, {'r': R_MIN, 'r_eff': R_MIN},
    ])
    def test_the_bounds_themselves_pass(self, kw):
        assert len(MPCController._semantic_circles(_node([self._memory(**kw)]), True)) == 1

    def test_the_radius_gate_reads_the_base_radius_not_the_grown_one(self):
        """A person at r 1.4 grown to 2.0 is still an obstacle, with the grown radius."""
        circles = MPCController._semantic_circles(
            _node([self._memory(r=1.4, r_eff=2.0)]), memory_only=True)
        assert [c[2] for c in circles] == [pytest.approx(2.0)]


# -- the move's target is never an obstacle (Part B fix a) --------------------

PERSON_T = _obj(7, 'person', 4.0, 0.0, 0.25, 0.65, SemanticObject.MEMORY, conf=0.6)
PERSON_OTHER = _obj(9, 'person', 2.0, 1.2, 0.25, 0.55, SemanticObject.MEMORY, conf=0.6)


class TestTheTargetIsNeverAnObstacle:

    def test_the_remembered_target_is_in_no_semantic_list_by_track_id(self):
        node = _node([PERSON_T, PERSON_OTHER], target=('7', 'person', (4.0, 0.0)))
        for memory_only in (True, False):
            circles = MPCController._semantic_circles(node, memory_only=memory_only)
            # Only the OTHER person, odom (1, 1.2), with ITS grown radius.
            assert circles == [pytest.approx((1.0, 1.2, 0.55))]

    def test_a_tracked_target_is_out_of_the_rear_check_too(self):
        tracked = _obj(7, 'person', 4.0, 0.0, 0.25, 0.25, SemanticObject.TRACKED)
        node = _node([tracked], target=('7', 'person', (4.0, 0.0)))
        assert MPCController._semantic_circles(node, memory_only=False) == []

    def test_with_a_track_id_the_id_decides_not_the_position(self):
        """Another person standing ON the goal point under another id stays an obstacle."""
        impostor = _obj(11, 'person', 4.0, 0.0, 0.25, 0.25, SemanticObject.MEMORY)
        node = _node([impostor], target=('7', 'person', (4.0, 0.0)))
        assert len(MPCController._semantic_circles(node, memory_only=True)) == 1

    def test_without_a_track_id_class_and_position_decide(self):
        near_same = _obj(3, 'person', 4.3, 0.2, 0.25, 0.65, SemanticObject.MEMORY)
        far_same = _obj(4, 'person', 4.0, 1.0, 0.25, 0.25, SemanticObject.MEMORY)
        near_other = _obj(5, 'chair', 4.1, 0.0, 0.2, 0.2, SemanticObject.MEMORY)
        node = _node([near_same, far_same, near_other], target=('', 'person', (4.0, 0.0)))
        circles = MPCController._semantic_circles(node, memory_only=True)
        assert sorted(round(c[2], 2) for c in circles) == [0.2, 0.25]

    def test_the_live_target_circle_is_left_out_by_position(self):
        # odom: the target at (3, 0) (map (4, 0) through (1, 0, 0)).
        target_live = (3.1, 0.1, 0.25)
        beside = (3.0, 0.8, 0.25)
        node = _node([], live=[target_live, beside], target=('7', 'person', (4.0, 0.0)))
        assert MPCController._live_obstacles(node) == [beside]
        assert MPCController._planning_obstacles(node) == [beside]
        assert MPCController._reverse_env(node, []).objects == [beside]

    def test_the_target_does_not_make_the_car_stuck(self):
        """_reverse_front_distance, what stuck detection reads, ignores the target."""
        node = _node([], live=[(1.0, 0.0, 0.25)], target=('7', 'person', (1.1, 0.0)),
                     map_odom=(0.0, 0.0, 0.0))
        node.x, node.y, node.yaw = 0.0, 0.0, 0.0
        node._fresh_front_distance = lambda: None
        node._front_bumper_xy = lambda: (0.405, 0.0)
        assert MPCController._reverse_front_distance(node, []) is None
        node.object_psi_c = None                 # the move ended: an obstacle again
        assert MPCController._reverse_front_distance(node, []) is not None


# -- the startup WARN when B4 cannot run (Part B fix c) ----------------------

def _init_node(anchor, enabled=True):
    logged = []
    node = SimpleNamespace(
        use_map_frame_goal_anchor=anchor, subscriptions=[],
        get_logger=lambda: SimpleNamespace(warn=logged.append, info=lambda *a, **k: None))
    overrides = {'mpc_semantic_memory_enabled': enabled}
    node.declare_parameter = lambda key, default: SimpleNamespace(
        value=overrides.get(key, default))
    node.create_subscription = lambda *a, **k: node.subscriptions.append(a[1])
    node._semantic_objects_cb = None
    MPCController._init_semantic_memory(node)
    return node, logged


def test_one_warn_at_startup_names_the_anchor_when_b4_cannot_run():
    node, logged = _init_node(anchor=False)
    assert len(logged) == 1
    assert 'use_map_frame_goal_anchor is false' in logged[0]
    assert 'localization_source=' in logged[0]
    assert node.subscriptions == ['/costmap/semantic_objects']


@pytest.mark.parametrize('anchor, enabled', [(True, True), (False, False)])
def test_no_warn_when_b4_runs_or_is_switched_off(anchor, enabled):
    _, logged = _init_node(anchor=anchor, enabled=enabled)
    assert logged == []
