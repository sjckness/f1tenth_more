"""behavior_bringup.launch.py: the Rust front-proximity guard switch.

proximity_guard:=off|shadow|active (supervisor_bringup/stack_bringup guard:=).
LaunchDescriptions built and inspected directly, no process spawned (the
test_detection_launch_config.py convention). Load-bearing: 'off' (the
default) launches nothing, and the guard gets the BT's own front threshold
and the URDF bumper.

Run standalone: python3 -m pytest test/test_proximity_guard_launch.py -v
"""

import importlib.util
import os

from f1tenth_description.vehicle_frames import base_front_x
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

_LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                       'launch', 'behavior_bringup.launch.py')


def _description():
    spec = importlib.util.spec_from_file_location('behavior_bringup_launch', _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _context(description, **overrides):
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    context.launch_configurations.update(overrides)
    return context


def _guard(description):
    nodes = [e for e in description.entities
             if isinstance(e, Node) and e.node_package == 'f1tenth_proximity_guard']
    assert len(nodes) == 1, nodes
    return nodes[0]


def test_the_switch_defaults_to_off_with_three_choices():
    declared = {e.name: e for e in _description().entities
                if isinstance(e, DeclareLaunchArgument)}
    arg = declared['proximity_guard']
    assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == 'off'
    assert arg.choices == ['off', 'shadow', 'active']


@pytest.mark.parametrize('mode, launched', [('off', False), ('shadow', True), ('active', True)])
def test_off_launches_nothing(mode, launched):
    description = _description()
    node = _guard(description)
    assert node.condition.evaluate(_context(description, proximity_guard=mode)) is launched


@pytest.mark.parametrize('mode', ['shadow', 'active'])
def test_the_guard_gets_the_bt_threshold_and_the_urdf_bumper(mode):
    description = _description()
    node = _guard(description)
    context = _context(description, proximity_guard=mode, proximity_front_threshold_m='0.23')
    params = evaluate_parameters(context, node._Node__parameters)[0]
    assert params['mode'] == mode
    assert params['base_front_x_m'] == base_front_x()
    assert params['front_distance_threshold_m'] == 0.23
    assert node.node_executable == 'proximity_guard_node'
    assert node._Node__node_name == 'proximity_guard_node'
