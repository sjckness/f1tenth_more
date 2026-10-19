"""
mpc_corr's side of the object memory (Part B4).

MEMORY objects from /costmap/semantic_objects join the MPC's obstacle list
(TRACKED ones arrive live on /perception/obstacles_2d already), and every
semantic object reaches the reverse manoeuvre's rear pre-check, all
re-expressed through the current map -> odom.
"""

from types import SimpleNamespace

from f1tenth_messages.msg import SemanticObject, SemanticObjectArray
from f1tenth_params.semantic_objects import objects_to_odom_circles
from mpc_controller.MPC_corr import MPCController
import pytest

import reverse_manoeuvre_harness as H


def _obj(oid, cls, x, y, r, r_eff, state, conf=1.0):
    o = SemanticObject()
    o.id, o.class_id, o.x, o.y = oid, cls, float(x), float(y)
    o.radius_m, o.effective_radius_m = float(r), float(r_eff)
    o.state, o.confidence = state, float(conf)
    return o


class _Clock:
    def __init__(self, t):
        self.t = t

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.t * 1e9))


def _node(objects, age=0.1, map_odom=(1.0, 0.0, 0.0), min_conf=0.0, enabled=True):
    msg = SemanticObjectArray()
    msg.objects = objects
    node = SimpleNamespace(
        semantic_memory_enabled=enabled, semantic_memory_max_age_s=0.5,
        semantic_memory_min_confidence=min_conf, _semantic_class_margins={},
        _semantic_msg=msg, _semantic_t=10.0 - age)
    node.get_clock = lambda: _Clock(10.0)
    node._lookup_map_odom = lambda: map_odom
    return node


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
