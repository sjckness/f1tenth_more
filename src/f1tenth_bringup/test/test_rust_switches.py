"""guard:= (Rust ports, output/rust/): bringup -> supervisor -> component.

supervisor_bringup.launch.py and stack_bringup.launch.py declare the switch;
component_supervisor_node writes it into the args of the entry that launches
behavior_bringup.launch.py, found by launch_file. Plain YAML and launch
inspection, no rclpy node.
"""

import copy
import importlib.util
import os
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402, I100
    apply_rust_switches, validate_rust_switches)

_PKG = os.path.join(os.path.dirname(__file__), '..')


def _registry():
    with open(os.path.join(_PKG, 'config', 'components.yaml')) as f:
        raw = yaml.safe_load(f)['components']
    return {name: [dict(e, args=dict(e.get('args', {}))) for e in entries]
            for name, entries in raw.items()}


def _declared(launch_file):
    spec = importlib.util.spec_from_file_location(
        launch_file.replace('.', '_'), os.path.join(_PKG, 'launch', launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {e.name: e for e in module.generate_launch_description().entities
            if isinstance(e, DeclareLaunchArgument)}


@pytest.mark.parametrize('launch_file', ['supervisor_bringup.launch.py',
                                         'stack_bringup.launch.py'])
def test_both_bringups_declare_the_switches(launch_file):
    declared = _declared(launch_file)
    for name, default, choices in (('guard', 'off', ['off', 'shadow', 'active']),):
        arg = declared[name]
        assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == default
        assert arg.choices == choices


def test_bad_values_are_refused():
    validate_rust_switches('off')
    validate_rust_switches('active')
    with pytest.raises(ValueError):
        validate_rust_switches('on')


def test_the_switches_reach_exactly_their_launch_files():
    before = _registry()
    after = apply_rust_switches(copy.deepcopy(before), 'shadow')
    touched = 0
    for name, entries in after.items():
        for old, new in zip(before[name], entries):
            extra = {k: v for k, v in new['args'].items() if old['args'].get(k) != v}
            if new['launch_file'] == 'behavior_bringup.launch.py':
                assert extra == {'proximity_guard': 'shadow'}
                touched += 1
            else:
                assert extra == {}, (name, new['launch_file'])
    assert touched == 1


def test_the_defaults_are_written_too():
    after = apply_rust_switches(_registry(), 'off')
    args = {e['launch_file']: e['args'] for entries in after.values() for e in entries}
    assert args['behavior_bringup.launch.py']['proximity_guard'] == 'off'
