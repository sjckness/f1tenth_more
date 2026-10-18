"""costmap.launch.py: costmap_boundary_impl:=py|rust (supervisor/stack impl:=).

Exactly one costmap_boundary_node runs, never both (same node name, same
topics), and none without enable_slam. The Rust node gets every parameter the
Python node gets plus the three vehicle_frames values the Python imports.
LaunchDescriptions built and inspected directly, no process spawned.

Run standalone: python3 -m pytest test/test_costmap_boundary_impl_launch.py -v
"""

import importlib.util
import os

from f1tenth_description.vehicle_frames import (
    base_front_x, front_side_limit_m, MIN_FRONT_DISTANCE_M)
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

_LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                       'launch', 'costmap.launch.py')


def _description():
    spec = importlib.util.spec_from_file_location('costmap_launch', _LAUNCH)
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


def _boundary_nodes(description):
    return {e.node_package: e for e in description.entities
            if isinstance(e, Node) and e.node_executable == 'costmap_boundary_node'}


def test_the_switch_defaults_to_py():
    declared = {e.name: e for e in _description().entities
                if isinstance(e, DeclareLaunchArgument)}
    arg = declared['costmap_boundary_impl']
    assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == 'py'
    assert arg.choices == ['py', 'rust']


@pytest.mark.parametrize('impl', ['py', 'rust'])
@pytest.mark.parametrize('slam', ['true', 'True', 'false'])
def test_exactly_one_implementation_runs(impl, slam):
    description = _description()
    nodes = _boundary_nodes(description)
    assert set(nodes) == {'f1tenth_costmap', 'f1tenth_costmap_boundary_rs'}
    context = _context(description, costmap_boundary_impl=impl, enable_slam=slam)
    running = {pkg for pkg, node in nodes.items() if node.condition.evaluate(context)}
    if slam == 'false':
        assert running == set()
    else:
        assert running == {'f1tenth_costmap' if impl == 'py' else 'f1tenth_costmap_boundary_rs'}


def test_same_name_and_a_superset_of_the_parameters():
    description = _description()
    nodes = _boundary_nodes(description)
    context = _context(description)
    py = evaluate_parameters(context, nodes['f1tenth_costmap']._Node__parameters)[0]
    rs = evaluate_parameters(context, nodes['f1tenth_costmap_boundary_rs']._Node__parameters)[0]
    assert {k: rs[k] for k in py} == py
    assert set(rs) - set(py) == {'base_front_x_m', 'front_side_limit_m', 'min_front_distance_m'}
    assert rs['base_front_x_m'] == base_front_x()
    assert rs['front_side_limit_m'] == front_side_limit_m()
    assert rs['min_front_distance_m'] == MIN_FRONT_DISTANCE_M
    names = {n._Node__node_name for n in nodes.values()}
    assert names == {'costmap_boundary_node'}
