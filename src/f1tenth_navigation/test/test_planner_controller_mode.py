"""navigation.launch.py's planner_controller_mode branch: which launch file backs the lane.

Evaluates the launch description, launches nothing. See
f1tenth_params/planner_modes.py for the modes.

Run standalone: python3 -m pytest test/test_planner_controller_mode.py -v
"""

import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest


def _load(package, launch_file, name):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def nav():
    return _load('f1tenth_navigation', 'navigation.launch.py', 'nav_launch_mode_test')


def _text(value):
    if isinstance(value, str):
        return value
    if not isinstance(value, (list, tuple)):
        value = [value]
    return perform_substitutions(LaunchContext(), list(value))


def _files(actions):
    # .location only renders substitutions as reprs before the file is loaded;
    # the substitutions themselves are what the include will resolve.
    return [os.path.basename(_text(
        a.launch_description_source._LaunchDescriptionSource__location))
        for a in actions if isinstance(a, IncludeLaunchDescription)]


def _args(include):
    return {_text(k): _text(v) for k, v in (include.launch_arguments or [])}


@pytest.mark.parametrize('mode, launch_file', [
    ('corridor_mpc', 'mpc_corr.launch.py'),
    ('corridor_rrt', 'planner_controller.launch.py'),
    ('p2p_geometric', 'planner_controller.launch.py'),
])
def test_each_mode_starts_its_controller_and_the_map(nav, mode, launch_file):
    actions = nav.drive_lane_actions(mode)
    assert _files(actions) == [launch_file, 'map_only.launch.py']


@pytest.mark.parametrize('mode', ['corridor_rrt', 'p2p_geometric'])
def test_the_planner_include_is_told_its_mode(nav, mode):
    include = nav.drive_lane_actions(mode)[0]
    assert _args(include) == {'planner_controller_mode': mode}


def test_an_unknown_mode_fails_the_launch(nav):
    with pytest.raises(ValueError):
        nav.drive_lane_actions('rrt_please')


def test_the_argument_is_declared_with_the_stack_default(nav):
    ld = nav.generate_launch_description()
    declared = {e.name: perform_substitutions(LaunchContext(), e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert declared['planner_controller_mode'] == 'corridor_mpc'
    assert any(isinstance(e, OpaqueFunction) for e in ld.entities)


@pytest.mark.parametrize('mode', ['corridor_rrt', 'p2p_geometric'])
def test_the_planner_launch_file_starts_the_named_node(mode):
    planner = _load('f1tenth_planners', 'planner_controller.launch.py', f'planner_{mode}')
    context = LaunchContext()
    context.launch_configurations['planner_controller_mode'] = mode
    context.launch_configurations['cpu_affinity'] = '0'
    (node,) = planner._node(context)
    assert node.node_executable == mode
    assert node.node_package == 'f1tenth_planners'


def test_the_planner_launch_file_refuses_corridor_mpc():
    planner = _load('f1tenth_planners', 'planner_controller.launch.py', 'planner_mpc')
    context = LaunchContext()
    context.launch_configurations['planner_controller_mode'] = 'corridor_mpc'
    with pytest.raises(ValueError):
        planner._node(context)
