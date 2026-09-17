"""GoToObject (behaviours/go_to_object.py) and the go_to_person repeat run.

Same no-ROS-graph convention as test_goal_reached_latch_per_run.py: the real
behaviours, setup() bypassed, publishers replaced by recorders, the blackboard
by a plain object. Tracks and statuses are injected the way the wire delivers
them (the subscription callbacks), and time comes from a settable clock.

Run standalone: python3 -m pytest test/test_go_to_object_behaviour.py -v
"""

import time
from pathlib import Path
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.behaviours.go_to_object import (  # noqa: E402
    GoToObject,
    tracks_from_message,
)
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import load_mission_file  # noqa: E402
from f1tenth_behavior.mission.object_handler import object_move_wire_id  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    OBJECT_STATUS_KEY,
    MissionRuntimeState,
    MissionState,
)

MISSIONS = Path(__file__).resolve().parent.parent / 'missions'
S = py_trees.common.Status


class _Clock:
    """GoToObject's clock. Starts at time.monotonic() because CheckStopCondition
    measures move timeouts and status age on the real monotonic clock."""

    def __init__(self):
        self.t = time.monotonic()

    def __call__(self):
        return self.t


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, *_a, **_k):
        self.lines.append(str(msg))

    warn = warning = debug = error = info


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Blackboard:
    pass


def _tracks_msg(tracks, stamp=99.5, frame='map'):
    """A semantic_tracks-shaped message: (id, class, x, y)."""
    sec = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(
            sec=sec, nanosec=int(round((stamp - sec) * 1e9)))),
        detections=[SimpleNamespace(
            id=tid,
            results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id=cls, score=0.9))],
            bbox=SimpleNamespace(
                center=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
                size=SimpleNamespace(x=0.5, y=0.5, z=0.0)))
            for tid, cls, x, y in tracks])


def _rig(mission='go_to_person'):
    state = MissionRuntimeState()
    clock = _Clock()
    bb = _Blackboard()
    setattr(bb, MISSION_KEY, state)
    setattr(bb, GLOBAL_XY_KEY, (0.0, 0.0))
    setattr(bb, GLOBAL_YAW_KEY, 0.0)
    setattr(bb, GLOBAL_TURN_ACCUM_KEY, 0.0)
    setattr(bb, OBJECT_STATUS_KEY, None)

    goto = GoToObject(clock=clock)
    goto.node = _Node()
    goto.blackboard = bb
    goto.goal_pub, goto.end_pub, goto.hold_pub = _Pub(), _Pub(), _Pub()

    check = CheckStopCondition()
    check.node = goto.node
    check.blackboard = bb
    check.hold_pub, check.object_end_pub = _Pub(), _Pub()
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(bb, key, None)
    setattr(bb, DETECTED_CLASSES_KEY, {})
    check.x, check.y, check.yaw = 0.0, 0.0, 0.0
    check.global_x, check.global_y, check.global_yaw = 0.0, 0.0, 0.0

    config = load_mission_file(str(MISSIONS / f'{mission}.json'))
    return SimpleNamespace(state=state, clock=clock, bb=bb, goto=goto, check=check,
                           config=config)


def _start(rig):
    rig.state.load(rig.config, rig.clock.t)
    rig.state.begin(rig.clock.t)
    return object_move_wire_id(rig.config.mission_id, rig.state.run_generation,
                               rig.config.moves[0].id)


def _feed_tracks(rig, tracks, stamp=None):
    rig.goto._tracks_cb(_tracks_msg(tracks, stamp=rig.clock.t - 0.2 if stamp is None else stamp))


def _status(rig, move_id, r, behind=False, watchdog=False, age=0.2,
            latched=False, speed=0.0):
    """Feed a status stand-in carrying every field the callback reads."""
    rig.check._object_status_cb(SimpleNamespace(
        move_id=move_id, r=r, alpha=0.05, target_age_s=age,
        target_behind_terminal=behind, goal_watchdog=watchdog,
        stop_latched=latched, speed=speed))


def _tick(rig, dt=0.1):
    rig.clock.t += dt
    goto = rig.goto.update()
    if goto != S.SUCCESS:
        return goto, None
    return goto, rig.check.update()


class TestPublishing:

    def test_a_constant_move_id_and_the_capture_stamp(self):
        rig = _rig()
        wire = _start(rig)
        for i in range(5):
            _feed_tracks(rig, [('7', 'person', 3.0, 0.1 * i)], stamp=100.0 + 0.1 * i)
            _tick(rig)
        msgs = rig.goto.goal_pub.msgs
        assert len(msgs) == 5
        assert {m.move_id for m in msgs} == {wire}
        last = msgs[-1]
        assert (last.header.stamp.sec, last.header.stamp.nanosec) == (100, 400000000)
        assert last.header.frame_id == 'map'
        assert (last.point.x, last.point.y) == (3.0, pytest.approx(0.4))
        spec = rig.config.moves[0].go_to_object
        # mpc_corr's standoff is the CENTRE distance: default gap + nose_reach +
        # the track's 0.5 m width / 2.
        assert spec.gap_m == pytest.approx(0.5)
        assert last.standoff == pytest.approx(0.5 + spec.nose_reach_m + 0.25)
        assert last.speed == pytest.approx(0.4)
        assert last.target_class == 'person'

    def test_nothing_is_published_before_a_track_is_acquired(self):
        rig = _rig()
        _start(rig)
        _feed_tracks(rig, [('c', 'chair', 2.0, 0.0)])
        assert _tick(rig) == (S.SUCCESS, S.RUNNING)
        assert rig.goto.goal_pub.msgs == []

    def test_grace_republishes_the_last_point_at_reduced_speed(self):
        rig = _rig()
        _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        _feed_tracks(rig, [])
        _tick(rig)
        last = rig.goto.goal_pub.msgs[-1]
        assert (last.point.x, last.point.y) == (3.0, 0.0)
        assert last.speed == pytest.approx(0.2)

    def test_tracks_in_another_frame_are_ignored(self):
        rig = _rig()
        _start(rig)
        rig.goto._tracks_cb(_tracks_msg([('7', 'person', 3.0, 0.0)], frame='odom'))
        _tick(rig)
        assert rig.goto.goal_pub.msgs == []

    def test_tracks_from_message_reads_id_class_position_and_stamp(self):
        (track,) = tracks_from_message(_tracks_msg([('4', 'person', 1.5, -2.0)], stamp=12.25))
        assert (track.track_id, track.class_id, track.x, track.y) == ('4', 'person', 1.5, -2.0)
        assert track.stamp_sec == pytest.approx(12.25)


class TestFailureOutcomes:

    def _assert_failed(self, rig, wire, outcome, status):
        assert status == S.FAILURE
        assert rig.state.state == MissionState.ABORTED
        assert [m.data for m in rig.goto.end_pub.msgs] == [wire]
        assert [m.data for m in rig.goto.hold_pub.msgs] == [True]
        (record,) = rig.state.move_outcomes
        assert record.outcome == outcome
        assert rig.state.last_stop_reason == f'go_to_object:{outcome}'

    def test_target_not_found(self):
        rig = _rig()
        wire = _start(rig)
        assert _tick(rig)[0] == S.SUCCESS            # the handler starts here
        assert _tick(rig, dt=4.95)[0] == S.SUCCESS   # acquire_timeout_sec is 5.0
        self._assert_failed(rig, wire, 'target_not_found', _tick(rig, dt=0.1)[0])

    def test_grace_then_target_lost(self):
        rig = _rig()
        wire = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        rig.goto.tracks = []
        results = [_tick(rig)[0] for _ in range(16)]
        assert results[:15] == [S.SUCCESS] * 15
        self._assert_failed(rig, wire, 'target_lost', results[15])

    def test_target_unreachable_on_the_terminal_flag_for_this_move(self):
        rig = _rig()
        wire = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        _status(rig, 'some other move', r=1.0, behind=True)
        assert _tick(rig)[0] == S.SUCCESS, "another move's flag is not ours"
        _status(rig, wire, r=1.0, behind=True)
        self._assert_failed(rig, wire, 'target_unreachable', _tick(rig)[0])


class TestReachedAndTimeout:

    def test_object_reached_on_this_moves_status(self):
        rig = _rig()
        wire = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        assert _tick(rig) == (S.SUCCESS, S.RUNNING)
        _status(rig, wire, r=0.3)
        assert _tick(rig) == (S.SUCCESS, S.RUNNING)
        _status(rig, wire, r=0.05)
        assert _tick(rig) == (S.SUCCESS, S.SUCCESS)
        assert rig.state.object_record.outcome == 'reached'
        assert rig.state.object_record.last_status.r == pytest.approx(0.05)

    def test_a_timeout_abort_ends_the_object_move_and_holds(self):
        rig = _rig()
        wire = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        rig.check.node = rig.goto.node
        rig.state.move_start_time -= 61.0
        _, check = _tick(rig)
        assert check == S.FAILURE
        assert [m.data for m in rig.check.object_end_pub.msgs] == [wire]
        assert [m.data for m in rig.check.hold_pub.msgs] == [True]
        assert rig.state.move_outcomes[-1].outcome == 'timeout'

    def test_a_move_change_to_another_type_ends_the_object_move(self):
        rig = _rig()
        wire = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        rig.state.current_index = 1          # past the only move: no current move
        assert rig.goto.update() == S.SUCCESS
        assert [m.data for m in rig.goto.end_pub.msgs] == [wire]
        published = len(rig.goto.goal_pub.msgs)
        rig.goto.update()
        assert len(rig.goto.goal_pub.msgs) == published, 'it stops publishing'


class TestGoToPersonRepeatRun:
    """Two runs of go_to_person.json in one executor; run 2 must not complete instantly.

    The 2026-09-15 failure mode for goal_reached, now for object_reached: the
    last status of run 1 (r inside the tolerance) is still on the blackboard
    when run 2 starts. It carries run 1's wire id, so it cannot satisfy run 2.
    """

    def test_the_second_run_does_not_complete_on_its_first_tick(self):
        rig = _rig()
        wire1 = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        _status(rig, wire1, r=0.02)
        assert _tick(rig) == (S.SUCCESS, S.SUCCESS)

        wire2 = _start(rig)
        assert wire2 != wire1
        _feed_tracks(rig, [('9', 'person', 3.0, 0.0)])
        assert _tick(rig) == (S.SUCCESS, S.RUNNING), (
            "run 2 completed on its first tick on run 1's status")
        assert {m.move_id for m in rig.goto.goal_pub.msgs[-1:]} == {wire2}

    def test_run_2_still_completes_on_its_own_status(self):
        rig = _rig()
        wire1 = _start(rig)
        _feed_tracks(rig, [('7', 'person', 3.0, 0.0)])
        _tick(rig)
        _status(rig, wire1, r=0.02)
        _tick(rig)
        wire2 = _start(rig)
        _feed_tracks(rig, [('9', 'person', 3.0, 0.0)])
        _tick(rig)
        _status(rig, wire2, r=0.02)
        assert _tick(rig) == (S.SUCCESS, S.SUCCESS)
