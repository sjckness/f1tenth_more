"""CheckStopCondition's /mpc/goal_reached latch is per RUN, not per move id.

The bug this pins was observed live, not imagined. mpc_corr publishes
/mpc/goal_reached as `True` and never republishes `False` when a new goal
supersedes the old one, so the consumer has to decide for itself when a
latched True stops being relevant. It used to decide that on `move.id`
alone -- and a move id is unique within a mission, not across runs of one.

go_to_person.json is a single-move mission whose move is always
`move_0_go_to_person`. Re-running it in a live executor therefore kept the
previous run's latch, and the first tick of run 2 saw its own goal as already
reached. The three runs recorded on 2026-09-15 show exactly that: run 1 drove
0.548 m, runs 2 and 3 completed in 0.082 s having moved 0.0 m, both
mismatch_flagged.

WHY THIS IS A UNIT TEST AND NOT A NODE TEST. The whole mechanism is three
lines of bookkeeping over state the behaviour owns -- the subscription that
sets the latch is one line of rclpy and is not what broke. Driving a real
executor would need a live mpc_corr to publish the stale True, which is the
hardware path this suite must not take. So the stale True is injected the
same way the wire does it (`_goal_reached_cb`), and the reset is exercised
through the real `update()`.
"""

import time

import pytest

py_trees = pytest.importorskip('py_trees')

from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.mission.mission_config import (  # noqa: E402
    MissionConfig,
    Move,
    StopCondition,
)
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)


class _Bool:
    """std_msgs/Bool stand-in -- the callback only ever reads `.data`."""

    def __init__(self, data):
        self.data = data


class _Logger:

    def info(self, *_a, **_k):
        pass

    warn = warning = debug = error = info


class _Node:

    def get_logger(self):
        return _Logger()


class _Blackboard:
    """py_trees blackboard stand-in: the behaviour only get/setattr's it."""


def _single_move_mission():
    """go_to_person.json's shape: one terminal move, stop on goal_reached."""
    return MissionConfig(
        mission_id='go_to_person',
        moves=[
            Move(
                id='move_0_go_to_person',
                goal_distance=0.5,
                stop_condition=StopCondition(type='goal_reached', params={}),
                timeout_sec=60,
                on_timeout='abort',
                terminal=True,
            )
        ],
    )


def _behaviour(state):
    """A CheckStopCondition wired to `state`, with setup() bypassed.

    setup() only creates ROS subscriptions/publishers; every field it would
    populate that update() actually reads is set here instead, so the test
    touches no ROS graph at all.
    """
    behaviour = CheckStopCondition()
    behaviour.node = _Node()
    behaviour.blackboard = _Blackboard()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    # Blackboard keys update() reads unconditionally, by their real constants
    # rather than by their spelling -- a renamed key must break this test
    # loudly rather than leave it asserting against a stale name.
    setattr(behaviour.blackboard, MIN_OBSTACLE_DISTANCE_KEY, None)
    setattr(behaviour.blackboard, MIN_OBSTACLE_DISTANCE_FORWARD_KEY, None)
    setattr(behaviour.blackboard, FRONT_CLEARANCE_KEY, None)
    setattr(behaviour.blackboard, DETECTED_CLASSES_KEY, {})
    behaviour.x, behaviour.y, behaviour.yaw = 0.0, 0.0, 0.0
    behaviour.global_x, behaviour.global_y, behaviour.global_yaw = 0.0, 0.0, 0.0
    return behaviour


def _run(state, behaviour, config):
    """One full mission run: load -> begin -> a single tick."""
    now = time.monotonic()
    state.load(config, now)
    state.begin(now)
    return behaviour.update()


class TestLatchIsPerRun:

    def test_second_run_of_the_same_mission_does_not_complete_on_its_first_tick(self):
        """The regression itself. One executor, one behaviour, two runs."""
        config = _single_move_mission()
        state = MissionRuntimeState()
        behaviour = _behaviour(state)

        # --- run 1: mpc_corr confirms arrival, the move completes.
        assert _run(state, behaviour, config) == py_trees.common.Status.RUNNING
        behaviour._goal_reached_cb(_Bool(True))
        assert behaviour.update() == py_trees.common.Status.SUCCESS

        # --- run 2: nothing new on the wire. mpc_corr does NOT republish
        # False, so the only thing standing between the stale True and an
        # instant completion is the per-run reset.
        assert _run(state, behaviour, config) == py_trees.common.Status.RUNNING, (
            'run 2 completed on its first tick -- the goal_reached latch '
            'survived from run 1 (this is the 0.0 m / mismatch_flagged bug)'
        )
        assert behaviour._goal_reached_flag is False

    def test_run_2_still_completes_once_its_own_arrival_arrives(self):
        """The reset must not break the feature it protects."""
        config = _single_move_mission()
        state = MissionRuntimeState()
        behaviour = _behaviour(state)

        _run(state, behaviour, config)
        behaviour._goal_reached_cb(_Bool(True))
        behaviour.update()

        assert _run(state, behaviour, config) == py_trees.common.Status.RUNNING
        behaviour._goal_reached_cb(_Bool(True))
        assert behaviour.update() == py_trees.common.Status.SUCCESS

    def test_three_consecutive_runs_each_need_their_own_arrival(self):
        """The 2026-09-15 session was three runs, not two."""
        config = _single_move_mission()
        state = MissionRuntimeState()
        behaviour = _behaviour(state)

        for run in range(3):
            assert _run(state, behaviour, config) == py_trees.common.Status.RUNNING, (
                f'run {run + 1} completed on its first tick')
            behaviour._goal_reached_cb(_Bool(True))
            assert behaviour.update() == py_trees.common.Status.SUCCESS


class TestRunGeneration:

    def test_load_and_begin_each_bump_it(self):
        state = MissionRuntimeState()
        start = state.run_generation
        state.load(_single_move_mission(), 0.0)
        after_load = state.run_generation
        state.begin(0.0)
        assert after_load > start
        assert state.run_generation > after_load

    def test_it_never_repeats_across_runs(self):
        """Only inequality is tested by consumers, so monotonicity is enough."""
        state = MissionRuntimeState()
        config = _single_move_mission()
        seen = []
        for _ in range(5):
            state.load(config, 0.0)
            state.begin(0.0)
            seen.append(state.run_generation)
        assert seen == sorted(seen)
        assert len(set(seen)) == len(seen)

    def test_advancing_a_move_does_not_bump_it(self):
        """goto_move is a move change, not a run change -- the move id half of
        the key already covers that, and bumping here would be redundant."""
        state = MissionRuntimeState()
        state.load(_single_move_mission(), 0.0)
        state.begin(0.0)
        before = state.run_generation
        state.goto_move(0, 0.0)
        assert state.run_generation == before


class TestMoveIdHalfStillWorks:

    def test_a_move_change_within_one_run_still_resets_the_latch(self):
        """The original behaviour, unchanged: the key is a pair, not a swap."""
        config = MissionConfig(
            mission_id='two_moves',
            moves=[
                Move(id='m0', goal_distance=1.0,
                     stop_condition=StopCondition(type='goal_reached', params={}),
                     timeout_sec=60, on_timeout='abort'),
                Move(id='m1', goal_distance=1.0,
                     stop_condition=StopCondition(type='goal_reached', params={}),
                     timeout_sec=60, on_timeout='abort', terminal=True),
            ],
        )
        state = MissionRuntimeState()
        behaviour = _behaviour(state)
        _run(state, behaviour, config)

        behaviour._goal_reached_cb(_Bool(True))
        assert behaviour.update() == py_trees.common.Status.SUCCESS

        # Advance within the SAME run: run_generation is unchanged, so this
        # exercises the move-id half of the key on its own.
        generation = state.run_generation
        state.goto_move(1, time.monotonic())
        assert state.run_generation == generation
        assert behaviour.update() == py_trees.common.Status.RUNNING
        assert behaviour._goal_reached_flag is False
