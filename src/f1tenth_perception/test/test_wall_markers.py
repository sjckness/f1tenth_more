"""/viz/walls marker building: ids, colours, lifetimes, and dying tracks.

Pure builder tests -- no node, no rclpy spin. The builder's only state is the
set of track ids it drew last, which is what makes the DELETE-on-death tests
here worth having: the bug class they exclude is a track vanishing from the
array with nothing telling the client to remove its markers, which is what
the DELETEALL this topic deliberately does not send used to paper over.

The gates that decide WHETHER an array is built at all (the flag, the
subscription count, the rate cap) are node-level and live in
test_lidar_front_wall_node.py's TestVizWalls.
"""
import math

from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion
from f1tenth_perception import wall_markers as wm

from builtin_interfaces.msg import Time

import numpy as np

import pytest

from visualization_msgs.msg import Marker


def _stamp(sec=12, nanosec=340000000):
    return Time(sec=sec, nanosec=nanosec)


def _track(track_id, *, rho=2.0, theta=0.0, t0=-1.0, t1=1.0, role='front',
           ever_confirmed=True, state=mwt.STATE_CONFIRMED,
           verify=wall_fusion.VERIFY_UNVERIFIED, glass=False,
           z_min=0.0, z_max=2.5, height_measured=False):
    """One Track, built as the real dataclass so the builder sees real geometry."""
    return mwt.Track(
        id=track_id, x=np.array([rho, theta], dtype=float), P=np.eye(2) * 0.01,
        t0=t0, t1=t1, born=0.0, last_seen=1.0, ever_confirmed=ever_confirmed,
        state=state, role=role, verify=verify, glass=glass,
        z_min=z_min, z_max=z_max, height_measured=height_measured)


def _build(builder, tracks, distances=None):
    distances = distances or {t.id: 1.5 for t in tracks}
    return builder.build(tracks, _stamp(), 'odom', distances).markers


def _by_slot(markers, track_id):
    return {m.id - 4 * track_id: m for m in markers if 4 * track_id <= m.id <= 4 * track_id + 3}


class TestIds:
    """One track, four reserved slots, and never another track's marker."""

    def test_each_track_owns_four_consecutive_ids_from_four_times_its_id(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(0), _track(1), _track(7)])
        for track_id in (0, 1, 7):
            slots = _by_slot(markers, track_id)
            assert sorted(slots) == [0, 1, 2, 3]

    def test_no_two_tracks_share_a_marker_id(self):
        builder = wm.WallMarkerBuilder(0.6)
        # Adjacent ids are the case a 2-slot or 3-slot scheme would collide on.
        markers = _build(builder, [_track(i) for i in range(5)])
        ids = [m.id for m in markers]
        assert len(ids) == len(set(ids))

    def test_every_marker_is_in_the_one_namespace(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(3, glass=True)])
        assert {m.ns for m in markers} == {wm.NS}

    def test_the_stamp_and_frame_are_the_scan_s_not_now(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = builder.build([_track(1)], _stamp(99, 500000000), 'odom', {1: 1.0}).markers
        assert markers
        for m in markers:
            assert (m.header.stamp.sec, m.header.stamp.nanosec) == (99, 500000000)
            assert m.header.frame_id == 'odom'


class TestEveryTrackIsDrawn:
    """The reason this topic exists: published() drops these, this does not."""

    def test_an_other_track_is_drawn_in_its_own_dark_grey(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(1, role='other')])
        line = _by_slot(markers, 1)[wm.SLOT_LINE]
        assert (line.color.r, line.color.g, line.color.b) == pytest.approx(
            wm.ROLE_COLOUR['other'])

    def test_a_track_with_no_role_yet_reads_as_other(self):
        assert wm.role_of(_track(1, role='')) == 'other'

    def test_a_tentative_track_is_drawn_too(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(1, ever_confirmed=False,
                                          state=mwt.STATE_TENTATIVE)])
        assert _by_slot(markers, 1)[wm.SLOT_LINE].action == Marker.ADD


class TestColours:
    """Role picks the colour, state picks the alpha, and they never swap."""

    @pytest.mark.parametrize('role', ['front', 'left', 'right', 'back', 'other'])
    def test_the_line_takes_its_role_s_colour(self, role):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(2, role=role)])
        line = _by_slot(markers, 2)[wm.SLOT_LINE]
        assert (line.color.r, line.color.g, line.color.b) == pytest.approx(
            wm.ROLE_COLOUR[role])

    def test_front_is_red_left_is_blue_right_is_yellow(self):
        front, left, right = (wm.ROLE_COLOUR[k] for k in ('front', 'left', 'right'))
        assert front[0] > front[1] and front[0] > front[2]          # red
        assert left[2] > left[0] and left[2] > left[1]              # blue
        assert right[0] > 0.5 and right[1] > 0.5 and right[2] < 0.2  # yellow

    @pytest.mark.parametrize('kw,word,alpha', [
        ({'verify': wall_fusion.VERIFY_VERIFIED}, 'verified', 0.9),
        ({'verify': wall_fusion.VERIFY_UNVERIFIED}, 'unverified', 0.5),
        ({'verify': wall_fusion.VERIFY_REJECTED}, 'rejected', 0.2),
        ({'ever_confirmed': False, 'state': mwt.STATE_TENTATIVE}, 'tentative', 0.2),
    ])
    def test_alpha_follows_the_state_word(self, kw, word, alpha):
        trk = _track(1, **kw)
        assert wm.state_word(trk) == word
        assert wm.alpha_of(trk) == pytest.approx(alpha)

    def test_a_never_confirmed_track_is_tentative_whatever_the_rectangle_said(self):
        trk = _track(1, ever_confirmed=False, state=mwt.STATE_TENTATIVE,
                     verify=wall_fusion.VERIFY_VERIFIED)
        assert wm.state_word(trk) == 'tentative'

    def test_the_prism_is_always_fainter_than_its_own_line(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(1, verify=wall_fusion.VERIFY_VERIFIED)])
        slots = _by_slot(markers, 1)
        assert slots[wm.SLOT_PRISM].color.a < slots[wm.SLOT_LINE].color.a

    def test_the_label_says_id_role_state_dist(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(4, role='left',
                                          verify=wall_fusion.VERIFY_VERIFIED)],
                         distances={4: 1.25})
        text = _by_slot(markers, 4)[wm.SLOT_TEXT]
        assert text.type == Marker.TEXT_VIEW_FACING
        assert text.text.split() == ['4', 'left', 'verified', '1.25m']

    def test_the_label_cannot_contradict_the_alpha(self):
        builder = wm.WallMarkerBuilder(0.6)
        trk = _track(1, verify=wall_fusion.VERIFY_REJECTED)
        markers = _build(builder, [trk])
        slots = _by_slot(markers, 1)
        assert wm.state_word(trk) in slots[wm.SLOT_TEXT].text
        assert slots[wm.SLOT_LINE].color.a == pytest.approx(wm.ALPHA[wm.state_word(trk)])


class TestGeometry:
    """A 5 cm line on the floor and a prism standing on it."""

    def test_the_line_is_the_segment_s_two_endpoints_five_centimetres_wide(self):
        builder = wm.WallMarkerBuilder(0.6)
        trk = _track(1, rho=2.0, theta=0.0, t0=-1.0, t1=1.0)
        line = _by_slot(_build(builder, [trk]), 1)[wm.SLOT_LINE]
        assert line.type == Marker.LINE_STRIP
        assert line.scale.x == pytest.approx(0.05)
        (x0, y0), (x1, y1) = trk.endpoints()
        assert [(p.x, p.y) for p in line.points] == pytest.approx([(x0, y0), (x1, y1)])

    def test_the_prism_spans_zero_to_two_metres_when_nothing_measured_the_height(self):
        builder = wm.WallMarkerBuilder(0.6)
        prism = _by_slot(_build(builder, [_track(1, height_measured=False)]),
                         1)[wm.SLOT_PRISM]
        assert prism.scale.z == pytest.approx(2.0)
        assert prism.pose.position.z == pytest.approx(1.0)

    def test_the_prism_uses_the_measured_span_when_there_is_one(self):
        builder = wm.WallMarkerBuilder(0.6)
        prism = _by_slot(_build(builder, [_track(1, z_min=0.1, z_max=1.3,
                                                 height_measured=True)]),
                         1)[wm.SLOT_PRISM]
        assert prism.scale.z == pytest.approx(1.2)

    def test_the_prism_runs_along_the_wall_and_is_thin_across_it(self):
        builder = wm.WallMarkerBuilder(0.6)
        trk = _track(1, t0=-1.5, t1=1.5)
        prism = _by_slot(_build(builder, [trk]), 1)[wm.SLOT_PRISM]
        assert prism.scale.x == pytest.approx(3.0)
        assert prism.scale.y < 0.1

    def test_a_degenerate_segment_never_gets_a_zero_scale(self):
        builder = wm.WallMarkerBuilder(0.6)
        prism = _by_slot(_build(builder, [_track(1, t0=0.0, t1=0.0)]), 1)[wm.SLOT_PRISM]
        assert prism.scale.x > 0.0 and prism.scale.z > 0.0

    def test_a_missing_distance_prints_as_nan_rather_than_raising(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = builder.build([_track(1)], _stamp(), 'odom', {}).markers
        assert 'nan' in _by_slot(markers, 1)[wm.SLOT_TEXT].text


class TestGlass:
    """A pane gets a cyan outline, and loses it again when it stops being one."""

    def test_a_glass_track_gets_a_cyan_outline_at_the_top_of_the_pane(self):
        builder = wm.WallMarkerBuilder(0.6)
        trk = _track(1, glass=True, z_min=0.0, z_max=1.8, height_measured=True)
        glass = _by_slot(_build(builder, [trk]), 1)[wm.SLOT_GLASS]
        assert glass.action == Marker.ADD
        assert glass.type == Marker.LINE_STRIP
        assert (glass.color.r, glass.color.g, glass.color.b) == pytest.approx(
            wm.GLASS_COLOUR)
        assert glass.color.b > glass.color.r and glass.color.g > glass.color.r  # cyan
        assert [p.z for p in glass.points] == pytest.approx([1.8, 1.8])

    def test_the_outline_does_not_replace_the_role_colour(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(1, role='front', glass=True)])
        line = _by_slot(markers, 1)[wm.SLOT_LINE]
        assert (line.color.r, line.color.g, line.color.b) == pytest.approx(
            wm.ROLE_COLOUR['front'])

    def test_a_non_glass_track_actively_deletes_its_outline_slot(self):
        builder = wm.WallMarkerBuilder(0.6)
        glass_slot = _by_slot(_build(builder, [_track(1, glass=False)]), 1)[wm.SLOT_GLASS]
        assert glass_slot.action == Marker.DELETE

    def test_a_track_that_stops_being_glass_loses_the_outline(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1, glass=True)])
        after = _by_slot(_build(builder, [_track(1, glass=False)]), 1)
        assert after[wm.SLOT_GLASS].action == Marker.DELETE
        assert after[wm.SLOT_LINE].action == Marker.ADD


class TestLifetime:
    """Nothing is drawn forever, and no array clears the namespace."""

    def test_every_added_marker_carries_the_builder_s_lifetime(self):
        builder = wm.WallMarkerBuilder(0.6)
        markers = _build(builder, [_track(1, glass=True), _track(2)])
        added = [m for m in markers if m.action == Marker.ADD]
        assert added
        for m in added:
            assert m.lifetime.sec + m.lifetime.nanosec * 1e-9 == pytest.approx(0.6)

    def test_the_lifetime_is_at_least_three_publish_periods_at_five_hertz(self):
        builder = wm.WallMarkerBuilder(0.6)
        line = _by_slot(_build(builder, [_track(1)]), 1)[wm.SLOT_LINE]
        assert line.lifetime.sec + line.lifetime.nanosec * 1e-9 >= 3 * (1 / 5.0)

    def test_a_lifetime_over_a_second_splits_into_sec_and_nanosec(self):
        """nanosec is a uint32: packing the whole duration into it raises."""
        builder = wm.WallMarkerBuilder(4.5)
        line = _by_slot(_build(builder, [_track(1)]), 1)[wm.SLOT_LINE]
        assert line.lifetime.sec == 4
        assert line.lifetime.nanosec == 500000000
        assert line.lifetime.nanosec <= 4294967295

    def test_no_array_ever_clears_every_namespace(self):
        builder = wm.WallMarkerBuilder(0.6)
        first = _build(builder, [_track(1), _track(2)])
        second = _build(builder, [_track(1)])
        third = _build(builder, [])
        for markers in (first, second, third):
            assert Marker.DELETEALL not in [m.action for m in markers]


class TestDeleteOnDeath:
    """A track that goes away takes its markers with it, by id."""

    def test_a_dead_track_gets_a_delete_on_each_of_its_four_slots(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1), _track(2)])
        markers = _build(builder, [_track(1)])
        dead = _by_slot(markers, 2)
        assert sorted(dead) == [0, 1, 2, 3]
        assert all(m.action == Marker.DELETE for m in dead.values())

    def test_the_surviving_track_is_still_added(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1), _track(2)])
        markers = _build(builder, [_track(1)])
        assert _by_slot(markers, 1)[wm.SLOT_LINE].action == Marker.ADD

    def test_every_track_dying_at_once_deletes_every_id(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1), _track(2), _track(5)])
        markers = _build(builder, [])
        assert {m.id for m in markers} == {
            4 * i + slot for i in (1, 2, 5) for slot in wm.SLOTS}
        assert all(m.action == Marker.DELETE for m in markers)
        assert builder.drawn_ids == set()

    def test_a_track_is_deleted_only_once(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1)])
        _build(builder, [])
        again = _build(builder, [])
        assert again == []

    def test_a_track_that_comes_back_is_added_again(self):
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1)])
        _build(builder, [])
        markers = _build(builder, [_track(1)])
        assert _by_slot(markers, 1)[wm.SLOT_LINE].action == Marker.ADD

    def test_forget_drops_the_memory_without_emitting_deletes(self):
        """Nobody is subscribed any more: the markers expire on their own."""
        builder = wm.WallMarkerBuilder(0.6)
        _build(builder, [_track(1), _track(2)])
        builder.forget()
        assert builder.drawn_ids == set()
        markers = _build(builder, [_track(1)])
        assert all(m.action == Marker.ADD or m.id == 4 * 1 + wm.SLOT_GLASS
                   for m in markers)
        assert not any(m.id // 4 == 2 for m in markers)


def test_the_role_colours_cover_every_role_the_tracker_can_assign():
    """Every bucket published() knows, plus 'other', has a colour."""
    assert set(wm.ROLE_COLOUR) == {'front', 'left', 'right', 'back', 'other'}


def test_every_alpha_is_a_legal_opacity():
    assert all(0.0 < a <= 1.0 for a in wm.ALPHA.values())


def test_the_default_prism_span_is_zero_to_two_metres():
    assert wm.DEFAULT_Z_SPAN == (0.0, 2.0)
    assert not math.isnan(wm.DEFAULT_Z_SPAN[1])
