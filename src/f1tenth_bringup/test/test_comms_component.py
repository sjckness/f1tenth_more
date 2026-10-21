"""The comms component (f1tenth_comms): registered, its own, and off by default.

wifi_coverage_node and link_manager report on /comms/* only. They must not
start unless stack_params.yaml enable_comms says so, and must not share a
component with anything else, so `stackctl.py start/stop comms` and a crash
loop of theirs touch nothing but themselves.

Plain YAML and set checks, no rclpy.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402, I100
    _ALWAYS_AUTO_START, _CONDITIONAL_AUTO_START, _NEVER_AUTO_START)

_HERE = os.path.dirname(__file__)
_COMPONENTS_YAML = os.path.join(_HERE, '..', 'config', 'components.yaml')
_STACK_PARAMS = os.path.join(
    _HERE, '..', '..', 'f1tenth_params', 'config', 'stack_params.yaml')
_STACK_BRINGUP = os.path.join(_HERE, '..', 'launch', 'stack_bringup.launch.py')


def _components():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)['components']


def test_comms_is_a_component_of_its_own():
    entries = _components()['comms']
    assert entries == [{'package': 'f1tenth_comms', 'launch_file': 'comms.launch.py'}]
    for name, other in _components().items():
        if name != 'comms':
            assert 'comms.launch.py' not in [e['launch_file'] for e in other], name


def test_comms_auto_starts_only_on_enable_comms():
    assert 'comms' not in _ALWAYS_AUTO_START
    assert 'comms' not in _NEVER_AUTO_START
    assert _CONDITIONAL_AUTO_START['comms'] == 'enable_comms'


def test_enable_comms_defaults_off():
    with open(_STACK_PARAMS) as f:
        assert yaml.safe_load(f)['enable_comms']['default'] is False


def test_stack_bringup_includes_comms_only_behind_the_same_key():
    with open(_STACK_BRINGUP) as f:
        text = f.read()
    assert "if get_value('enable_comms'):" in text
    assert text.count("'comms.launch.py'") == 1
