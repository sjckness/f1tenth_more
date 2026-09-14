"""Supervisor side of the wall_distance and swept_clearance placement rules.

component_supervisor_node restarts a component by SIGINTing every launch file
registered under it. Any launch file that reads /scan must therefore never
share a component with lidar.launch.py, whose urg_node is the only publisher
of the /scan the e-stop reads (see f1tenth_perception's
lidar_front_wall.launch.py module docstring, which is where this rule is
argued at length). f1tenth_perception's test_wall_distance_launch.py guards
the launch-file side of the same constraint.

THREE /scan CONSUMERS NOW LIVE OUTSIDE `perception`: lidar_front_wall (guarded
by test_lidar_front_wall_component.py), wall_distance and swept_clearance
(guarded here). The rule is the same for all three and the test is written
over a LIST so a fourth cannot be added without either appearing here or
failing the sweep at the bottom.

Plain YAML and set checks, no rclpy.
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import _ALWAYS_AUTO_START  # noqa: E402

_COMPONENTS_YAML = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'components.yaml')

# component name -> its one launch file. Every /scan consumer that is
# deliberately NOT under `perception`.
_SCAN_CONSUMERS = {
    'lidar_front_wall': 'lidar_front_wall.launch.py',
    'wall_distance': 'wall_distance.launch.py',
    'swept_clearance': 'swept_clearance.launch.py',
}


def _components():
    with open(_COMPONENTS_YAML) as f:
        return yaml.safe_load(f)['components']


def _launch_files(entries):
    return [entry['launch_file'] for entry in entries]


def test_each_scan_consumer_is_a_component_of_its_own():
    """Only that consumer's own launch file lives under its component."""
    components = _components()
    for name, launch_file in _SCAN_CONSUMERS.items():
        assert name in components, f'{name} is not registered in components.yaml'
        assert _launch_files(components[name]) == [launch_file], name


def test_no_component_restarts_urg_node_with_a_scan_consumer():
    """No component may hold both lidar.launch.py and a /scan consumer."""
    for name, entries in _components().items():
        files = _launch_files(entries)
        if 'lidar.launch.py' not in files:
            continue
        for consumer in _SCAN_CONSUMERS.values():
            assert consumer not in files, f'{name} would restart urg_node with {consumer}'


def test_no_scan_consumer_is_in_the_perception_component():
    """The perception component owns urg_node."""
    perception = _launch_files(_components()['perception'])
    for consumer in _SCAN_CONSUMERS.values():
        assert consumer not in perception, consumer


def test_every_scan_consumer_auto_starts():
    """An uncategorized component silently never starts: no warning, no error."""
    for name in _SCAN_CONSUMERS:
        assert name in _ALWAYS_AUTO_START, name


def test_no_component_holds_two_scan_consumers_either():
    """No component may hold two of them, so they cannot restart each other."""
    for name, entries in _components().items():
        files = set(_launch_files(entries))
        held = files & set(_SCAN_CONSUMERS.values())
        assert len(held) <= 1, f'{name} holds {sorted(held)}'


def test_the_scan_consumer_list_here_is_complete():
    """Any component holding a known /scan launch file must be listed above."""
    known = {
        'lidar_front_wall.launch.py', 'wall_distance.launch.py',
        'swept_clearance.launch.py', 'lidar.launch.py',
    }
    for name, entries in _components().items():
        for launch_file in _launch_files(entries):
            if launch_file not in known:
                continue
            if launch_file == 'lidar.launch.py':
                assert name == 'perception', 'urg_node moved out of perception'
            else:
                assert _SCAN_CONSUMERS.get(name) == launch_file, (
                    f'{launch_file} is under {name}, which this test does not know about')
