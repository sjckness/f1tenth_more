"""
/viz/doors marker building: jamb cylinders, colours by state, and dying doors.

Pure builder tests, the test_wall_markers.py convention: no node, no spin.
The node wiring (published only while subscribed) is in
test_lidar_front_wall_node.py's TestDoors.
"""

import collections

from builtin_interfaces.msg import Time

from f1tenth_perception import door_detection as dd
from f1tenth_perception import door_markers as dm

import numpy as np

import pytest

from visualization_msgs.msg import Marker


def _door(door_id, state=dd.STATE_CONFIRMED, occupied=False, j0=(3.0, -0.425),
          j1=(3.0, 0.425)):
    door = dd.DoorTrack(door_id, np.array(j0, dtype=float), np.array(j1, dtype=float),
                        np.array([1.0, 0.0]), born=0.0, last_seen=0.0, state=state)
    door.history = collections.deque([True], maxlen=5)
    door.occupied = occupied
    return door


def _build(builder, doors):
    return builder.build(doors, Time(sec=12), 'odom').markers


def _slots(markers, door_id):
    return {m.id - 4 * door_id: m for m in markers if 4 * door_id <= m.id < 4 * door_id + 4}


class TestJambs:

    def test_each_jamb_is_a_2_5_m_cylinder_standing_on_the_floor(self):
        markers = _slots(_build(dm.DoorMarkerBuilder(0.6), [_door(1)]), 1)
        for slot, (x, y) in ((dm.SLOT_JAMB0, (3.0, -0.425)), (dm.SLOT_JAMB1, (3.0, 0.425))):
            post = markers[slot]
            assert post.type == Marker.CYLINDER and post.action == Marker.ADD
            assert post.scale.z == pytest.approx(2.5)
            assert post.pose.position.z == pytest.approx(1.25)     # base on the floor
            assert post.scale.x == post.scale.y == pytest.approx(0.10)
            assert (post.pose.position.x, post.pose.position.y) == pytest.approx((x, y))

    def test_the_diameter_and_height_are_the_builders_parameters(self):
        builder = dm.DoorMarkerBuilder(0.6, height_m=2.0, jamb_diameter_m=0.2)
        post = _slots(_build(builder, [_door(1)]), 1)[dm.SLOT_JAMB0]
        assert post.scale.z == pytest.approx(2.0) and post.scale.x == pytest.approx(0.2)
        assert post.pose.position.z == pytest.approx(1.0)

    def test_the_threshold_runs_jamb_to_jamb_and_the_label_says_what_it_is(self):
        markers = _slots(_build(dm.DoorMarkerBuilder(0.6), [_door(3, occupied=True)]), 3)
        line = markers[dm.SLOT_LINE]
        assert line.type == Marker.LINE_STRIP
        ends = [(p.x, p.y) for p in line.points]
        assert ends == pytest.approx([(3.0, -0.425), (3.0, 0.425)])
        text = markers[dm.SLOT_TEXT]
        assert text.type == Marker.TEXT_VIEW_FACING
        assert text.text == 'door 3 0.85m confirmed OCCUPIED'
        assert text.pose.position.z > 2.5

    def test_every_marker_carries_the_lifetime(self):
        for m in _build(dm.DoorMarkerBuilder(0.6), [_door(1)]):
            assert m.lifetime.sec + m.lifetime.nanosec * 1e-9 == pytest.approx(0.6)
            assert m.ns == dm.NS and m.header.frame_id == 'odom'


class TestColour:

    def test_alpha_is_the_state(self):
        builder = dm.DoorMarkerBuilder(0.6)
        alpha = {}
        for state in (dd.STATE_TENTATIVE, dd.STATE_CONFIRMED, dd.STATE_COASTING):
            post = _slots(_build(builder, [_door(1, state=state)]), 1)[dm.SLOT_JAMB0]
            alpha[state] = post.color.a
        assert alpha[dd.STATE_TENTATIVE] < alpha[dd.STATE_COASTING]
        assert alpha[dd.STATE_COASTING] < alpha[dd.STATE_CONFIRMED] == 1.0

    def test_coasting_is_dimmed_and_occupied_is_its_own_colour(self):
        builder = dm.DoorMarkerBuilder(0.6)
        free = _slots(_build(builder, [_door(1)]), 1)[dm.SLOT_JAMB0].color
        coast = _slots(_build(builder, [_door(1, state=dd.STATE_COASTING)]),
                       1)[dm.SLOT_JAMB0].color
        busy = _slots(_build(builder, [_door(1, occupied=True)]), 1)[dm.SLOT_JAMB0].color
        assert coast.g < free.g
        assert (busy.r, busy.g, busy.b) == pytest.approx(dm.OCCUPIED_COLOUR)
        assert (free.r, free.g, free.b) == pytest.approx(dm.DOOR_COLOUR)


class TestDeletion:

    def test_a_dead_door_gets_a_delete_on_each_of_its_four_slots(self):
        builder = dm.DoorMarkerBuilder(0.6)
        _build(builder, [_door(1), _door(2)])
        markers = _build(builder, [_door(2)])
        deletes = [m for m in markers if m.action == Marker.DELETE]
        assert sorted(m.id for m in deletes) == [4, 5, 6, 7]
        assert all(m.ns == dm.NS for m in deletes)
        assert {m.id // 4 for m in markers if m.action == Marker.ADD} == {2}

    def test_the_delete_is_sent_once_not_forever(self):
        builder = dm.DoorMarkerBuilder(0.6)
        _build(builder, [_door(1)])
        _build(builder, [])
        assert _build(builder, []) == []

    def test_no_array_ever_clears_the_whole_namespace(self):
        builder = dm.DoorMarkerBuilder(0.6)
        for doors in ([_door(1)], [], [_door(2)]):
            assert Marker.DELETEALL not in [m.action for m in _build(builder, doors)]

    def test_forget_means_no_deletes_for_doors_no_client_saw(self):
        builder = dm.DoorMarkerBuilder(0.6)
        _build(builder, [_door(1)])
        builder.forget()
        assert _build(builder, []) == []
