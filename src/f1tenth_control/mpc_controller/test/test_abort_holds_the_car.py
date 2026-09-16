"""A goal_distance move that times out: hold, and zero /drive within one control tick.

The real CheckStopCondition times the move out and publishes /mpc/hold; the
real MPCController.hold_callback receives it; the next real control_loop tick
publishes /drive through the real _publish_drive. Only the node plumbing is a
stand-in. Before the fix the timeout-abort branch published nothing, so this
car kept driving its goal_distance corridor after the mission had aborted.

Run standalone: python3 -m pytest test/test_abort_holds_the_car.py -v
"""

import time
from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import parse_mission  # noqa: E402
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)

from mpc_controller.MPC_corr import MPCController  # noqa: E402


class _Pub:
    def __init__(self, sink=None):
        self.msgs = []
        self.sink = sink

    def publish(self, msg):
        self.msgs.append(msg)
        if self.sink is not None:
            self.sink(msg)

    def get_subscription_count(self):
        return 1


class _Logger:
    def info(self, *a, **k):
        pass

    warn = error = info


class _Mpc:
    """The instance state control_loop reads up to its hold branch."""

    def __init__(self):
        from builtin_interfaces.msg import Time
        self._logger = _Logger()
        self._time = Time
        self.x, self.y, self.yaw, self.v = 1.0, 0.0, 0.0, 0.45
        self.hold = False
        self.object_psi_c = None
        self.object_exit_ramping = False
        self.obstacles_global_live = []
        self.max_forward_speed, self.max_reverse_speed = 1.0, 0.0
        self.pub, self.drive_clamp_pub = _Pub(), _Pub()
        self.min_obstacle_distance_pub = _Pub()
        self.min_obstacle_distance_forward_pub = _Pub()
        self._last_published_steer = 0.0

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(time.monotonic() * 1e9), to_msg=lambda: self._time()))

    def _update_active_odom(self):
        pass

    def __getattr__(self, name):
        method = getattr(MPCController, name, None)
        if callable(method):
            return method.__get__(self, _Mpc)
        raise AttributeError(name)


@pytest.fixture(autouse=True)
def _no_report_files(monkeypatch):
    monkeypatch.setattr(check_module, 'write_mission_summary', lambda *a, **k: None)


def test_a_goal_distance_timeout_holds_and_zeroes_drive_within_one_tick():
    mpc = _Mpc()
    config = parse_mission({'mission_id': 't', 'moves': [{
        'id': 'straight', 'goal_distance': 3.0,
        'stop_condition': {'type': 'distance_reached'},
        'timeout_sec': 10, 'on_timeout': 'abort'}]})
    state = MissionRuntimeState()
    state.load(config, time.monotonic())
    state.begin(time.monotonic())

    check = CheckStopCondition()
    check.node = SimpleNamespace(get_logger=_Logger)
    check.blackboard = SimpleNamespace()
    setattr(check.blackboard, MISSION_KEY, state)
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(check.blackboard, key, None)
    setattr(check.blackboard, DETECTED_CLASSES_KEY, {})
    check.hold_pub = _Pub(sink=mpc.hold_callback)
    check.object_end_pub = _Pub()

    state.move_start_time -= 11.0
    assert check.update() == py_trees.common.Status.FAILURE
    assert mpc.hold is True, 'the abort did not reach mpc_corr as a hold'

    mpc.control_loop()
    (drive,) = mpc.pub.msgs
    assert drive.drive.speed == 0.0
    assert drive.drive.steering_angle == 0.0
