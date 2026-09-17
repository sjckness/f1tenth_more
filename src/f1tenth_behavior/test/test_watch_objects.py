"""scripts/watch_objects.py -- the floor session's object watcher: its numbers and its lines.

The script lives at the repository root (it is run by hand, not installed), so
it is loaded by path, the way the mpc_controller end-to-end tests reach across
packages. Only its pure part is exercised: no node is created, no topic
subscribed. Messages are fakes carrying exactly the fields the code reads.

The point of most of these tests is AGREEMENT: the watcher must print the gap
the go_to_object handler and its scoring use, so each check compares against
object_handler's own functions rather than against a number typed here.

Run standalone: python3 -m pytest test/test_watch_objects.py -v
"""

import importlib.util
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from f1tenth_behavior.behaviours.go_to_object import tracks_from_message
from f1tenth_behavior.mission.object_handler import (
    Track, centre_distance_for_gap, gap_for_centre_distance, track_radius)
from f1tenth_params.object_geometry import gap_limits

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'watch_objects.py'


@pytest.fixture(scope='module')
def watch():
    spec = importlib.util.spec_from_file_location('watch_objects', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


NOSE = gap_limits('person').nose_reach


def _track(track_id='12', cls='person', x=3.0, y=0.0, width=0.5, stamp=99.8, score=0.91):
    return Track(track_id=track_id, class_id=cls, x=x, y=y, score=score,
                 stamp_sec=stamp, width=width)


def _status(**overrides):
    base = dict(move_id='go_to_person_floor#4/move_0_go_to_person', target_class='person',
                track_id='12', r=0.123, gap=1.123, alpha=math.radians(3.2),
                target_age_s=0.21, speed_ref=0.4, stop_latched=False,
                inside_turn_radius=False, target_behind=False, target_behind_terminal=False,
                goal_watchdog=False, target_stale=False)
    base.update(overrides)
    return SimpleNamespace(**base)


class TestTheGapIsTheHandlers:

    def test_it_is_centre_distance_minus_nose_reach_minus_the_track_radius(self, watch):
        pose = watch.Pose(0.0, 0.0, 0.0, 100.0)
        (row,) = watch.track_rows([_track(x=3.0, width=0.5)], pose, 100.0, NOSE)
        assert row.range_m == pytest.approx(3.0)
        assert row.gap_m == pytest.approx(gap_for_centre_distance(3.0, NOSE, 0.25))
        assert row.gap_m == pytest.approx(3.0 - 0.5525 - 0.25)

    def test_a_track_without_a_width_uses_the_class_nominal(self, watch):
        pose = watch.Pose(0.0, 0.0, 0.0, 100.0)
        chair = _track(cls='chair', x=2.0, width=0.0)
        (row,) = watch.track_rows([chair], pose, 100.0, NOSE)
        assert row.gap_m == pytest.approx(2.0 - NOSE - track_radius(chair))
        assert track_radius(chair) == pytest.approx(0.225)

    def test_at_the_handlers_centre_distance_the_gap_is_the_commanded_one(self, watch):
        """Round trip with the distance GoToObject sends mpc_corr for gap 1.0."""
        track = _track(width=0.5)
        d = centre_distance_for_gap(1.0, NOSE, track_radius(track))
        pose = watch.Pose(3.0 - d, 0.0, 0.0, 100.0)
        (row,) = watch.track_rows([track], pose, 100.0, NOSE)
        assert row.gap_m == pytest.approx(1.0)

    def test_tracks_are_parsed_by_the_handlers_own_parser(self, watch):
        """tracks_from_message gives the width the radius rule reads."""
        stamp = SimpleNamespace(sec=99, nanosec=800000000)
        centre = SimpleNamespace(position=SimpleNamespace(x=3.0, y=0.0))
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id='map', stamp=stamp),
            detections=[SimpleNamespace(
                id='12', results=[SimpleNamespace(hypothesis=SimpleNamespace(
                    class_id='person', score=0.9))],
                bbox=SimpleNamespace(center=centre, size=SimpleNamespace(x=0.6, y=0.6, z=0.0)))])
        (track,) = tracks_from_message(msg)
        pose = watch.Pose(0.0, 0.0, 0.0, 100.0)
        (row,) = watch.track_rows([track], pose, 100.0, NOSE)
        assert row.gap_m == pytest.approx(3.0 - NOSE - 0.3)
        assert row.age_s == pytest.approx(0.2)


class TestGeometry:

    def test_bearing_is_relative_to_the_heading_and_positive_left(self, watch):
        pose = watch.Pose(0.0, 0.0, math.pi / 2, 100.0)
        (left,) = watch.track_rows([_track(x=-1.0, y=1.0)], pose, 100.0, NOSE)
        assert left.bearing_deg == pytest.approx(45.0)
        (right,) = watch.track_rows([_track(x=1.0, y=1.0)], pose, 100.0, NOSE)
        assert right.bearing_deg == pytest.approx(-45.0)

    def test_without_a_pose_there_are_no_numbers(self, watch):
        (row,) = watch.track_rows([_track()], None, 100.0, NOSE)
        assert (row.range_m, row.bearing_deg, row.gap_m) == (None, None, None)
        assert row.age_s == pytest.approx(0.2)

    def test_rows_come_nearest_first(self, watch):
        pose = watch.Pose(0.0, 0.0, 0.0, 100.0)
        rows = watch.track_rows([_track('1', x=5.0), _track('2', x=2.0)], pose, 100.0, NOSE)
        assert [r.track_id for r in rows] == ['2', '1']

    def test_the_class_filter(self, watch):
        pose = watch.Pose(0.0, 0.0, 0.0, 100.0)
        rows = watch.track_rows([_track('1'), _track('2', cls='chair')], pose, 100.0, NOSE,
                                class_filter='chair')
        assert [r.track_id for r in rows] == ['2']


class TestTheTarget:

    def _goal(self, watch, track_id='12', x=3.0, y=0.0):
        return watch.GoalView('m', 'person', track_id, x, y, 100.0)

    def test_the_goal_track_id_marks_the_target(self, watch):
        tracks = [_track('7', x=2.0), _track('12', x=3.0)]
        target = watch.locked_track_id(tracks, self._goal(watch), 0.5)
        rows = watch.track_rows(tracks, watch.Pose(0, 0, 0, 100.0), 100.0, NOSE,
                                target_id=target)
        assert [r.track_id for r in rows if r.target] == ['12']

    def test_without_a_track_id_the_handlers_follow_gate_picks_it(self, watch):
        tracks = [_track('7', x=2.0), _track('12', x=3.1)]
        assert watch.locked_track_id(tracks, self._goal(watch, track_id='', x=3.0), 0.5) == '12'
        assert watch.locked_track_id(tracks, self._goal(watch, track_id='', x=9.0), 0.5) is None

    def test_no_goal_no_target(self, watch):
        assert watch.locked_track_id([_track()], None, 0.5) is None


class TestLines:

    def test_a_track_line(self, watch):
        row = watch.TrackRow('12', 'person', 0.91, 0.18, 2.34, 12.3, 1.52, True, 0, 0)
        assert watch.format_track_row(row) == (
            '  #12    person         0.91  range  2.34 m  bearing  +12.3 deg'
            '  gap  1.52 m  age 0.18 s  TARGET')

    def test_a_track_line_without_a_pose(self, watch):
        row = watch.TrackRow('12', 'person', 0.91, 0.18, None, None, None, False, 0, 0)
        line = watch.format_track_row(row)
        assert 'NO POSE' in line and 'gap' not in line and 'TARGET' not in line

    def test_the_go_to_line_carries_the_live_values_and_the_goal(self, watch):
        goal = watch.GoalView('go_to_person_floor#4/move_0_go_to_person', 'person', '12',
                              3.0, 0.0, 100.0)
        line = watch.format_status_line(_status(stop_latched=True), goal)
        assert line == (
            '  GO_TO go_to_person_floor#4/move_0_go_to_person [person]  r +0.123'
            '  gap 1.123 m  alpha +3.2 deg  target_age 0.21 s  speed_ref 0.40'
            '  flags stop_latched')

    def test_an_unknown_gap_prints_nan_and_no_flags_print_a_dash(self, watch):
        line = watch.format_status_line(_status(gap=math.nan), None)
        assert 'gap nan m' in line and line.endswith('flags -')

    def test_every_flag_is_named(self, watch):
        status = _status(**{name: True for name in watch.STATUS_FLAGS})
        assert watch.status_flags(status) == list(watch.STATUS_FLAGS)


class TestMarkersAndCsv:

    def test_the_label_text(self, watch):
        row = watch.TrackRow('12', 'person', 0.9, 0.1, 1.5, 0.0, 0.93, False, 0, 0)
        assert watch.label_text(row) == 'person #12 gap 0.93 m'
        no_pose = watch.TrackRow('12', 'person', 0.9, 0.1, None, None, None, False, 0, 0)
        assert watch.label_text(no_pose) == 'person #12 NO POSE'

    def test_marker_ids_are_stable_and_numeric_ids_are_kept(self, watch):
        assert watch.marker_id('12') == 12
        assert watch.marker_id('a7') == watch.marker_id('a7')
        assert watch.marker_id('a7') >= 0

    def test_the_archive_is_refused_for_csv(self, watch, tmp_path):
        archive = os.path.expanduser('~/f1tenth_archive')
        assert watch.csv_path_allowed(os.path.join(archive, 'x.csv')) is False
        assert watch.csv_path_allowed(os.path.join(archive, 'complete', 'run', 'x.csv')) is False
        assert watch.csv_path_allowed(str(tmp_path / 'x.csv')) is True

    def test_a_symlink_into_the_archive_is_refused_too(self, watch, tmp_path):
        link = tmp_path / 'sneaky'
        link.symlink_to(os.path.expanduser('~/f1tenth_archive'))
        assert watch.csv_path_allowed(str(link / 'x.csv')) is False

    def test_main_refuses_an_archive_path_before_touching_ros(self, watch, capsys):
        code = watch.main(['--csv', os.path.expanduser('~/f1tenth_archive/x.csv')])
        assert code == 2
        assert 'refusing' in capsys.readouterr().err

    def test_csv_records_cover_every_printed_field(self, watch):
        row = watch.TrackRow('12', 'person', 0.91, 0.18, 2.34, 12.3, 1.52, True, 0, 0)
        record = watch.csv_track_record(1789000000.0, row)
        assert set(record) <= set(watch.CSV_FIELDS)
        assert record['gap_m'] == '1.520' and record['target'] == 1
        status = watch.csv_status_record(1789000000.0, _status(), None)
        assert set(status) <= set(watch.CSV_FIELDS)
        assert status['status_gap_m'] == '1.123'
