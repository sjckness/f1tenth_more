"""
The comms component (f1tenth_comms): registered, its own, and off by default.

wifi_coverage_node and link_manager report on /comms/* only. They must not
start unless enable_comms says so (a launch argument of both bringups,
default from stack_params.yaml), and must not share a component with
anything else, so `stackctl.py start/stop comms` and a crash loop of theirs
touch nothing but themselves.

YAML, text and set checks; the supervisor module is imported (rclpy must be
importable), never instantiated.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

import component_supervisor_node  # noqa: E402, I100
from component_supervisor_node import (  # noqa: E402, I100
    _ALWAYS_AUTO_START, _CONDITIONAL_AUTO_START, _NEVER_AUTO_START, ComponentSupervisorNode)

_HERE = os.path.dirname(__file__)
_COMPONENTS_YAML = os.path.join(_HERE, '..', 'config', 'components.yaml')
_STACK_PARAMS = os.path.join(
    _HERE, '..', '..', 'f1tenth_params', 'config', 'stack_params.yaml')
_STACK_BRINGUP = os.path.join(_HERE, '..', 'launch', 'stack_bringup.launch.py')
_SUPERVISOR_BRINGUP = os.path.join(_HERE, '..', 'launch', 'supervisor_bringup.launch.py')


def _components():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)['components']


def test_comms_is_a_component_of_its_own():
    entries = _components()['comms']
    assert entries == [{'package': 'f1tenth_comms', 'launch_file': 'comms.launch.py'}]
    for name, other in _components().items():
        if name != 'comms':
            assert 'comms.launch.py' not in [e['launch_file'] for e in other], name


def test_comms_is_gated_on_enable_comms():
    assert 'comms' not in _ALWAYS_AUTO_START
    assert 'comms' not in _NEVER_AUTO_START
    assert _CONDITIONAL_AUTO_START['comms'] == 'enable_comms'


def test_enable_comms_defaults_off():
    with open(_STACK_PARAMS) as f:
        assert yaml.safe_load(f)['enable_comms']['default'] is False


def test_enable_comms_is_a_launch_argument_of_both_bringups():
    with open(_SUPERVISOR_BRINGUP) as f:
        sup = f.read()
    assert "get_default('enable_comms')" in sup
    assert "'enable_comms': LaunchConfiguration('enable_comms')" in sup
    assert 'enable_comms_la,' in sup
    with open(_STACK_BRINGUP) as f:
        stack = f.read()
    assert "get_default('enable_comms')" in stack and 'enable_comms_la,' in stack
    assert stack.count("'comms.launch.py'") == 1
    assert "condition=IfCondition(LaunchConfiguration('enable_comms'))" in stack


class _Supervisor:
    def __init__(self, enable_intelligence, enable_comms):
        self.enable_intelligence = enable_intelligence
        self.enable_comms = enable_comms


def _auto_start(monkeypatch, stack_params, **params):
    monkeypatch.setattr(component_supervisor_node, 'get_value', stack_params.__getitem__)
    return ComponentSupervisorNode._auto_start_set(_Supervisor(**params))


def test_comms_follows_the_supervisor_parameter_not_the_yaml(monkeypatch):
    on_in_yaml = {'use_behavior_tree': True, 'enable_wall_distance': False,
                  'enable_swept_clearance': False, 'enable_comms': True}
    started = _auto_start(monkeypatch, on_in_yaml, enable_intelligence=False,
                          enable_comms=False)
    assert 'comms' not in started
    off_in_yaml = dict(on_in_yaml, enable_comms=False)
    started = _auto_start(monkeypatch, off_in_yaml, enable_intelligence=False,
                          enable_comms=True)
    assert 'comms' in started


def test_the_other_components_are_gated_as_before(monkeypatch):
    params = {'use_behavior_tree': True, 'enable_wall_distance': False,
              'enable_swept_clearance': False, 'enable_comms': False}
    for comms in (False, True):
        started = _auto_start(monkeypatch, params, enable_intelligence=True,
                              enable_comms=comms)
        assert started - {'comms'} == _ALWAYS_AUTO_START | {'behavior', 'intelligence'}
    started = _auto_start(monkeypatch, dict(params, use_behavior_tree=False,
                                            enable_wall_distance=True),
                          enable_intelligence=False, enable_comms=False)
    assert started == _ALWAYS_AUTO_START | {'wall_distance'}


def test_swept_clearance_starts_only_with_enable_swept_clearance(monkeypatch):
    """R3 (2026-10-23): out of the always-started set, gated on its stack_params key."""
    params = {'use_behavior_tree': False, 'enable_wall_distance': False,
              'enable_swept_clearance': False, 'enable_comms': False}
    started = _auto_start(monkeypatch, params, enable_intelligence=False, enable_comms=False)
    assert 'swept_clearance' not in started
    assert 'obstacle_clearance' in started          # the logger's contact source stays
    started = _auto_start(monkeypatch, dict(params, enable_swept_clearance=True),
                          enable_intelligence=False, enable_comms=False)
    assert started == _ALWAYS_AUTO_START | {'swept_clearance'}
