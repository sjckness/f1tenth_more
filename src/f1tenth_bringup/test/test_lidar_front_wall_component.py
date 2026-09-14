"""Supervisor side of the lidar_front_wall placement constraint.

component_supervisor_node restarts a component by SIGINTing every launch file
registered under it. lidar_front_wall.launch.py must therefore never share a
component with lidar.launch.py, whose urg_node is the only publisher of the
/scan the e-stop reads (see f1tenth_perception's lidar_front_wall.launch.py
module docstring). f1tenth_perception's test_lidar_front_wall_launch.py
guards the launch-file side of the same constraint. Plain YAML and set checks,
no rclpy.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import _ALWAYS_AUTO_START  # noqa: E402

_COMPONENTS_YAML = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'components.yaml')


def _components():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)['components']


def _launch_files(entries):
    return [entry['launch_file'] for entry in entries]


def test_lidar_front_wall_is_a_component_of_its_own():
    """Only the wall estimator's launch file lives under lidar_front_wall."""
    components = _components()
    assert _launch_files(components['lidar_front_wall']) == ['lidar_front_wall.launch.py']


def test_no_component_restarts_urg_node_with_the_wall_estimator():
    """No component may hold both lidar.launch.py and the wall estimator."""
    for name, entries in _components().items():
        files = _launch_files(entries)
        assert not ('lidar.launch.py' in files and 'lidar_front_wall.launch.py' in files), name


def test_the_wall_estimator_is_not_in_the_perception_component():
    """The perception component owns urg_node."""
    assert 'lidar_front_wall.launch.py' not in _launch_files(_components()['perception'])


def test_lidar_front_wall_auto_starts():
    """A registered but uncategorized component silently never starts."""
    assert 'lidar_front_wall' in _ALWAYS_AUTO_START
