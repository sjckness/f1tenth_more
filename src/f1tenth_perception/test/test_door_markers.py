"""
/viz/doors marker building: jamb cylinders, faded candidates, labels and dying doors.

Pure builder tests, the test_wall_markers.py convention: no node, no spin.
The node wiring (every live door, every cycle, subscribed or not) is in
test_lidar_front_wall_node.py's TestDoors.
"""

from builtin_interfaces.msg import Time

from f1tenth_perception import door_detection as dd
from f1tenth_perception import door_markers as dm

import numpy as np

import pytest

from visualization_msgs.msg import Marker


def _door(door_id, state=dd.STATE_CONFIRMED, occupied=False, j0=(3.0, -0.425),
          j1=(3.0, 0.425)):
    door = dd.DoorTrack(door_id, np.array(j0, dtype=float), np.array(j1, dtype=float),
                        np.array([1.0, 0.0]), born=0.0, last_positive=0.0, state=state)
    door.occupied = occupied
    door.parents = (7, 8)
    # Measured by the stereo, so the plain label carries no flag.
    door.jamb_height_unknown = False
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


class TestLabel:

    def test_a_candidate_s_label_names_its_state_leaf_and_every_flag(self):
        door = _door(4, state=dd.STATE_CANDIDATE, occupied=True)
        door.leaf_side, door.glass_suspect, door.jamb_height_unknown = dd.LEAF_LEFT, True, True
        text = _slots(_build(dm.DoorMarkerBuilder(0.6), [door]), 4)[dm.SLOT_TEXT].text
        assert text == 'door 4 0.85m candidate leaf left glass? jamb? OCCUPIED'

    def test_jamb_question_mark_means_the_stereo_never_measured_the_jambs(self):
        door = _door(5)
        assert dm.label_of(door) == 'door 5 0.85m confirmed'
        door.jamb_height_unknown = True
        assert dm.label_of(door) == 'door 5 0.85m confirmed jamb?'

    def test_the_label_no_longer_says_frame_or_square(self):
        """No frame jambs (parent 0) and no square looks by default any more."""
        door = _door(6, state=dd.STATE_CANDIDATE)
        door.parents, door.square_looks = (0, 9), 0
        assert dm.label_of(door) == 'door 6 0.85m candidate'


class TestColour:

    def test_a_candidate_is_faded_and_a_confirmed_door_solid_on_all_four_markers(self):
        builder = dm.DoorMarkerBuilder(0.6)
        for state, alpha in ((dd.STATE_CANDIDATE, dm.CANDIDATE_ALPHA),
                             (dd.STATE_CONFIRMED, 1.0)):
            markers = _build(builder, [_door(1, state=state)])
            assert len(markers) == 4
            assert [m.color.a for m in markers] == pytest.approx([alpha] * 4)
        assert dm.CANDIDATE_ALPHA == pytest.approx(0.35)

    def test_only_confirmed_is_solid_a_legacy_coasting_state_is_faded(self):
        post = _slots(_build(dm.DoorMarkerBuilder(0.6), [_door(1, state=dd.STATE_COASTING)]),
                      1)[dm.SLOT_JAMB0]
        assert post.color.a == pytest.approx(dm.CANDIDATE_ALPHA)

    def test_occupied_is_its_own_colour_in_either_state(self):
        builder = dm.DoorMarkerBuilder(0.6)
        free = _slots(_build(builder, [_door(1)]), 1)[dm.SLOT_JAMB0].color
        for state in (dd.STATE_CANDIDATE, dd.STATE_CONFIRMED):
            busy = _slots(_build(builder, [_door(1, state=state, occupied=True)]),
                          1)[dm.SLOT_JAMB0].color
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

    def test_a_candidate_turning_confirmed_keeps_its_slots_and_is_not_deleted(self):
        builder = dm.DoorMarkerBuilder(0.6)
        _build(builder, [_door(3, state=dd.STATE_CANDIDATE)])
        markers = _build(builder, [_door(3)])
        assert [m.action for m in markers] == [Marker.ADD] * 4
        assert sorted(m.id for m in markers) == [12, 13, 14, 15]
