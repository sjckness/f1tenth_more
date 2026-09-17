"""mission/object_handler.py -- which track is the target, and when the move fails.

Pure: no rclpy, no py_trees. Times are plain floats.

Run standalone: python3 -m pytest test/test_object_handler.py -v
"""

import pytest

from f1tenth_behavior.mission.object_handler import (
    ACQUIRE,
    ENDED,
    FOLLOW,
    GRACE,
    OUTCOME_LOST,
    OUTCOME_NOT_FOUND,
    OUTCOME_UNREACHABLE,
    HandlerParams,
    ObjectHandler,
    Track,
    centre_distance_for_gap,
    gap_for_centre_distance,
    nearest_track,
    object_move_wire_id,
    track_radius,
)

PARAMS = HandlerParams(
    target_class='person', gap_m=0.5, nose_reach_m=0.5525, speed=0.4, acquire_timeout_sec=5.0,
    lost_grace_sec=1.5, follow_gate_m=0.5,
    tracks_max_gap_sec=0.5)


def _t(track_id, x, y, cls='person', stamp=0.0, width=0.5):
    return Track(track_id=track_id, class_id=cls, x=x, y=y, score=0.9, stamp_sec=stamp,
                 width=width)


def _tick(handler, now, tracks, received=None, vehicle=(0.0, 0.0), behind=False):
    return handler.update(now, tracks, now if received is None else received, vehicle,
                          behind_terminal=behind)


class TestAcquire:

    def test_the_nearest_track_of_the_class_to_the_vehicle_is_acquired(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 0.1, [_t('far', 5.0, 0.0), _t('near', 2.0, 1.0),
                              _t('chair', 1.0, 0.0, cls='chair')], vehicle=(0.0, 0.0))
        assert step.phase == FOLLOW
        assert step.track_id == 'near'
        assert step.target_xy == (2.0, 1.0)
        assert step.speed == pytest.approx(0.4)

    def test_nearest_is_measured_from_the_vehicle(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 0.1, [_t('a', 2.0, 0.0), _t('b', 6.0, 0.0)], vehicle=(5.5, 0.0))
        assert step.track_id == 'b'

    def test_nothing_is_published_while_acquiring(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 0.1, [_t('c', 1.0, 0.0, cls='chair')])
        assert step.phase == ACQUIRE
        assert step.target_xy is None and step.outcome is None

    def test_target_not_found_after_the_acquire_timeout(self):
        h = ObjectHandler(PARAMS, start_sec=10.0)
        assert _tick(h, 14.9, []).outcome is None
        step = _tick(h, 15.0, [])
        assert step.outcome == OUTCOME_NOT_FOUND
        assert step.phase == ENDED

    def test_no_vehicle_pose_means_no_acquisition(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = h.update(0.1, [_t('a', 2.0, 0.0)], 0.1, None)
        assert step.phase == ACQUIRE

    def test_the_capture_stamp_travels_with_the_point(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 3.0, [_t('a', 2.0, 0.0, stamp=2.7)])
        assert step.target_stamp_sec == pytest.approx(2.7)


class TestFollow:

    def _following(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        _tick(h, 0.1, [_t('7', 3.0, 0.0)])
        return h

    def test_a_track_id_change_inside_the_gate_keeps_the_target(self):
        h = self._following()
        step = _tick(h, 0.2, [_t('12', 3.3, 0.2)])
        assert step.phase == FOLLOW
        assert step.track_id == '12'
        assert step.target_xy == (3.3, 0.2)

    def test_a_track_outside_the_gate_is_rejected(self):
        """Same class, 0.6 m from the last point: a different object."""
        h = self._following()
        step = _tick(h, 0.2, [_t('7', 3.6, 0.0)])
        assert step.phase == GRACE
        assert step.target_xy == (3.0, 0.0), 'the last point is kept, not the new track'

    def test_the_gate_is_centred_on_the_last_point_not_the_vehicle(self):
        """A closer same-class track outside the gate must not steal the target."""
        h = self._following()
        step = _tick(h, 0.2, [_t('other', 1.0, 0.0), _t('9', 3.2, 0.1)])
        assert step.track_id == '9'

    def test_the_gate_moves_with_the_target(self):
        """A person walking: each refresh is within the gate of the previous one."""
        h = self._following()
        for i in range(1, 11):
            step = _tick(h, 0.1 + 0.1 * i, [_t('7', 3.0, 0.3 * i)])
            assert step.phase == FOLLOW
        assert step.target_xy == (3.0, pytest.approx(3.0))

    def test_another_class_inside_the_gate_is_ignored(self):
        h = self._following()
        step = _tick(h, 0.2, [_t('c', 3.0, 0.0, cls='chair')])
        assert step.phase == GRACE


class TestGraceAndLost:

    def _following(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        _tick(h, 0.1, [_t('7', 3.0, 0.0)])
        return h

    def test_grace_keeps_the_last_point_and_stops_to_wait(self):
        """Stop and wait, not a reduced speed: 0.2 m/s is below the operating floor."""
        h = self._following()
        step = _tick(h, 0.2, [])
        assert step.phase == GRACE
        assert step.target_xy == (3.0, 0.0)
        assert step.speed == 0.0
        assert step.outcome is None

    def test_a_track_returning_during_grace_drives_at_the_move_speed_again(self):
        h = self._following()
        _tick(h, 0.2, [])
        step = _tick(h, 0.5, [_t('9', 3.05, 0.0)])
        assert step.phase == FOLLOW
        assert step.speed == pytest.approx(0.4)

    def test_no_phase_ever_asks_for_a_speed_between_zero_and_the_floor(self):
        """ACQUIRE, FOLLOW, GRACE and ENDED over a whole lose-and-regain sequence."""
        h = ObjectHandler(PARAMS, start_sec=0.0)
        speeds = [_tick(h, 0.05, []).speed]
        speeds.append(_tick(h, 0.1, [_t('7', 3.0, 0.0)]).speed)
        speeds += [_tick(h, 0.2 + 0.1 * k, []).speed for k in range(5)]
        speeds.append(_tick(h, 0.8, [_t('8', 3.1, 0.0)]).speed)
        speeds += [_tick(h, 0.9 + 0.1 * k, []).speed for k in range(20)]
        assert all(s == 0.0 or s >= 0.4 for s in speeds), speeds

    def test_grace_then_lost(self):
        h = self._following()
        _tick(h, 0.2, [])
        assert _tick(h, 1.69, []).outcome is None
        step = _tick(h, 1.7, [])
        assert step.outcome == OUTCOME_LOST
        assert step.phase == ENDED

    def test_a_gated_track_during_grace_resumes_follow_and_resets_the_clock(self):
        h = self._following()
        _tick(h, 0.2, [])
        assert _tick(h, 1.0, [_t('8', 3.1, 0.0)]).phase == FOLLOW
        _tick(h, 1.1, [])
        assert _tick(h, 2.5, []).outcome is None, 'grace restarted at 1.1'
        assert _tick(h, 2.6, []).outcome == OUTCOME_LOST

    def test_stale_tracks_count_as_no_tracks(self):
        """A stalled perception pipeline is a lost target, not a frozen one."""
        h = self._following()
        step = h.update(1.0, [_t('7', 3.0, 0.0)], 0.4, (0.0, 0.0))
        assert step.phase == GRACE

    def test_zero_grace_loses_on_the_first_missing_tick(self):
        params = HandlerParams(**{**PARAMS.__dict__, 'lost_grace_sec': 0.0})
        h = ObjectHandler(params, start_sec=0.0)
        _tick(h, 0.1, [_t('7', 3.0, 0.0)])
        assert _tick(h, 0.2, []).outcome == OUTCOME_LOST

    def test_an_ended_handler_stays_ended(self):
        h = self._following()
        _tick(h, 0.2, [])
        _tick(h, 2.0, [])
        step = _tick(h, 2.1, [_t('7', 3.0, 0.0)])
        assert step.outcome == OUTCOME_LOST and step.phase == ENDED


class TestUnreachable:

    def test_a_terminal_target_behind_ends_the_move_unreachable(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        _tick(h, 0.1, [_t('7', 3.0, 0.0)])
        step = _tick(h, 0.2, [_t('7', 3.0, 0.0)], behind=True)
        assert step.outcome == OUTCOME_UNREACHABLE

    def test_it_is_not_judged_before_acquisition(self):
        """No goal has been sent, so the flag cannot be this move's."""
        h = ObjectHandler(PARAMS, start_sec=0.0)
        assert _tick(h, 0.1, [], behind=True).outcome is None


class TestCentreStandoff:
    """ObjectGoal.standoff is a centre distance: gap + nose_reach + target radius."""

    def test_it_uses_the_tracks_fused_width(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 0.1, [_t('7', 3.0, 0.0, width=0.46)])
        assert step.target_radius == pytest.approx(0.23)
        assert step.centre_standoff == pytest.approx(0.5 + 0.5525 + 0.23)

    def test_it_follows_the_width_as_the_track_updates(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        _tick(h, 0.1, [_t('7', 3.0, 0.0, width=0.46)])
        step = _tick(h, 0.2, [_t('7', 3.0, 0.0, width=0.60)])
        assert step.centre_standoff == pytest.approx(0.5 + 0.5525 + 0.30)

    @pytest.mark.parametrize('cls, width, radius', [
        ('person', 0.0, 0.25), ('chair', 0.0, 0.225), ('bottle', 0.0, 0.15)])
    def test_an_unknown_width_falls_back_to_the_class_nominal(self, cls, width, radius):
        params = HandlerParams(**{**PARAMS.__dict__, 'target_class': cls})
        h = ObjectHandler(params, start_sec=0.0)
        step = _tick(h, 0.1, [_t('7', 3.0, 0.0, cls=cls, width=width)])
        assert step.target_radius == pytest.approx(radius)

    def test_nothing_before_acquisition(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        assert _tick(h, 0.1, []).centre_standoff is None


def test_nearest_track_within_a_radius():
    tracks = [_t('a', 1.0, 0.0), _t('b', 0.4, 0.0)]
    assert nearest_track(tracks, 'person', (0.0, 0.0)).track_id == 'b'
    assert nearest_track(tracks, 'person', (2.0, 0.0), within=0.5) is None


def test_the_wire_id_is_distinct_across_runs_and_stable_within_one():
    a = object_move_wire_id('go_to_person', 3, 'move_0')
    assert a == object_move_wire_id('go_to_person', 3, 'move_0')
    assert a != object_move_wire_id('go_to_person', 5, 'move_0')


class TestTheGapRule:
    """One definition, shared with watch_objects.py: radius, centre distance, gap."""

    def test_a_measured_width_gives_the_radius(self):
        assert track_radius(_t('7', 3.0, 0.0, width=0.6)) == pytest.approx(0.3)

    def test_no_width_falls_back_to_the_class_nominal(self):
        assert track_radius(_t('7', 3.0, 0.0, width=0.0)) == pytest.approx(0.25)

    def test_centre_distance_and_gap_are_inverses(self):
        d = centre_distance_for_gap(0.5, 0.5525, 0.25)
        assert d == pytest.approx(1.3025)
        assert gap_for_centre_distance(d, 0.5525, 0.25) == pytest.approx(0.5)

    def test_the_handler_sends_the_centre_distance_of_the_rule(self):
        h = ObjectHandler(PARAMS, start_sec=0.0)
        step = _tick(h, 0.1, [_t('7', 3.0, 0.0, width=0.6)])
        assert step.centre_standoff == pytest.approx(
            centre_distance_for_gap(PARAMS.gap_m, PARAMS.nose_reach_m, 0.3))
