"""/mission/start_mission on a go_to_object mission answers instead of crashing.

The go_to_object preflight requirement reads GLOBAL_XY_KEY, which MissionLoader
had never registered for READ. py_trees raised AttributeError inside the
service callback and took behavior_executor_node down on every go_to_object
start (2026-09-17): load succeeded, start never answered. test_preflight.py
checked required_dependencies() on its own, where no blackboard is involved,
so these go through MissionLoader itself.

Same fake-node convention as test_mission_loader_hold_release.py.

Run standalone: python3 -m pytest test/test_start_mission_go_to_object.py -v
"""

from pathlib import Path
import types

import py_trees
import pytest

from f1tenth_behavior.mission.loader import MissionLoader
from f1tenth_behavior.mission.preflight import PREFLIGHT_BLACKBOARD_KEYS
from f1tenth_behavior.mission.runtime import (
    CURRENT_XY_KEY,
    GLOBAL_XY_KEY,
    MISSION_KEY,
    MissionState,
)

MISSION = str(Path(__file__).resolve().parent.parent / 'missions' / 'go_to_person_floor.json')


class _Node:
    def __init__(self, live_node_names):
        self._live_node_names = live_node_names

    def create_publisher(self, *a, **k):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **k):
        return None

    def create_service(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def declare_parameter(self, _name, default):
        return types.SimpleNamespace(value=default)

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None)

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=Time))

    def get_node_names(self):
        return self._live_node_names


@pytest.fixture
def loaded():
    loader = MissionLoader(_Node(['mpc_corr', 'ackermann_to_vesc_node', 'semantic_layer_node']))
    ok, message = loader._load(MISSION)
    assert ok, message
    # CheckStopCondition's construction: every key it owns starts at None.
    sensors = py_trees.blackboard.Client(name='FakeCheckStopCondition')
    for key in PREFLIGHT_BLACKBOARD_KEYS:
        sensors.register_key(key=key, access=py_trees.common.Access.WRITE)
        setattr(sensors, key, None)
    return loader, sensors


def _start(loader):
    return loader._on_start_mission_service(
        types.SimpleNamespace(), types.SimpleNamespace(success=None, message=None))


def _state(loader):
    return getattr(loader.blackboard, MISSION_KEY).state


def test_start_without_a_global_pose_is_refused_by_name(loaded):
    loader, sensors = loaded
    setattr(sensors, CURRENT_XY_KEY, (0.0, 0.0))
    resp = _start(loader)
    assert resp.success is False
    assert 'global localization' in resp.message
    assert _state(loader) is MissionState.LOADED


def test_start_with_every_dependency_live_runs(loaded):
    loader, sensors = loaded
    setattr(sensors, CURRENT_XY_KEY, (0.0, 0.0))
    setattr(sensors, GLOBAL_XY_KEY, (0.0, 0.0))
    resp = _start(loader)
    assert resp.success is True, resp.message
    assert _state(loader) is MissionState.RUNNING
