"""Supervisor side of obstacle_clearance: its own component, and never at boot.

obstacle_clearance_node reads /scan, so it follows the placement rule
test_wall_distance_component.py argues for the other three /scan consumers:
never under `perception` (whose urg_node is the e-stop's only /scan) and
never sharing a component with another consumer. Unlike those three it is
OPTIONAL -- a data source for the test-campaign logger that nothing controls
off -- so it is registered on demand only (_NEVER_AUTO_START) and started
with `./scripts/stackctl.py start obstacle_clearance`.

Plain YAML and set checks, no rclpy.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402
    _ALWAYS_AUTO_START, _CONDITIONAL_AUTO_START, _NEVER_AUTO_START)

_COMPONENTS_YAML = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'components.yaml')
_LAUNCH_FILE = 'obstacle_clearance.launch.py'


def _components():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)['components']


def test_obstacle_clearance_is_a_component_of_its_own():
    entries = _components()['obstacle_clearance']
    assert [e['launch_file'] for e in entries] == [_LAUNCH_FILE]
    assert [e['package'] for e in entries] == ['f1tenth_perception']


def test_no_other_component_launches_it():
    for name, entries in _components().items():
        if name == 'obstacle_clearance':
            continue
        assert _LAUNCH_FILE not in [e['launch_file'] for e in entries], name


def test_it_is_registered_but_never_started_at_boot():
    assert 'obstacle_clearance' in _NEVER_AUTO_START
    assert 'obstacle_clearance' not in _ALWAYS_AUTO_START
    assert 'obstacle_clearance' not in _CONDITIONAL_AUTO_START
