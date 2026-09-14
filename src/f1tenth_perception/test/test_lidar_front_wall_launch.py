"""lidar_front_wall.launch.py tests -- LaunchDescriptions built and inspected
directly, no process spawned (the test_detection_launch_config.py convention).

The load-bearing test is the placement one: lidar_front_wall_node must never
be launched from lidar.launch.py, because restarting that file restarts
urg_node, the e-stop's only /scan publisher (see lidar_front_wall.launch.py's
module docstring). f1tenth_bringup's test_lidar_front_wall_component.py guards
the supervisor half of the same constraint.

Run standalone: python3 -m pytest test/test_lidar_front_wall_launch.py -v
"""

import importlib.util
import os

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

from f1tenth_params.param_defaults import get_value

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PACKAGE_DIR = os.path.dirname(_THIS_DIR)
_SRC_DIR = os.path.dirname(_PACKAGE_DIR)


def _load(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wall_launch():
    return _load(os.path.join(_PACKAGE_DIR, 'launch', 'lidar_front_wall.launch.py'),
                 'lidar_front_wall_launch')


def _declared(launch_description):
    return {e.name: e for e in launch_description.entities
            if isinstance(e, DeclareLaunchArgument)}


def _nodes(launch_description):
    return [e for e in launch_description.entities if isinstance(e, Node)]


def _text(substitutions, context):
    return ''.join(s.perform(context) for s in substitutions)


class TestArguments:

    def test_the_arg_list_covers_exactly_the_stack_params_block(self):
        with open(os.path.join(_SRC_DIR, 'f1tenth_params', 'config', 'stack_params.yaml')) as f:
            keys = {k for k in yaml.safe_load(f) if k.startswith('lidar_front_wall_')}
        names = {'lidar_front_wall_' + name for name, _ in _wall_launch().LIDAR_FRONT_WALL_ARGS}
        assert names == keys

    def test_each_explicit_value_type_matches_its_stack_params_default(self):
        for name, value_type in _wall_launch().LIDAR_FRONT_WALL_ARGS:
            assert type(get_value('lidar_front_wall_' + name)) is value_type, name

    def test_defaults_resolve_to_stack_params(self):
        module = _wall_launch()
        declared = _declared(module.generate_launch_description())
        context = LaunchContext()
        for name, _ in module.LIDAR_FRONT_WALL_ARGS:
            arg = declared['lidar_front_wall_' + name]
            assert _text(arg.default_value, context) == str(get_value('lidar_front_wall_' + name))


class TestNode:

    def _launch(self):
        launch_description = _wall_launch().generate_launch_description()
        nodes = _nodes(launch_description)
        assert len(nodes) == 1
        return launch_description, nodes[0]

    def test_runs_the_wall_node(self):
        _, node = self._launch()
        assert node._Node__node_executable == 'lidar_front_wall_node'

    def test_gated_on_use_lidar(self):
        _, node = self._launch()
        assert isinstance(node.condition, IfCondition)
        context = LaunchContext()
        context.launch_configurations['use_lidar'] = 'false'
        assert not node.condition.evaluate(context)
        context.launch_configurations['use_lidar'] = 'true'
        assert node.condition.evaluate(context)

    def test_pinned_to_core_5_off_the_behavior_executor_through_taskset(self):
        launch_description, node = self._launch()
        context = LaunchContext()
        for arg in _declared(launch_description).values():
            arg.visit(context)
        assert _text(node.process_description.prefix, context) == 'taskset -c 5'


class TestPlacement:

    def test_lidar_launch_runs_urg_node_and_nothing_else(self):
        # See this file's module docstring: anything added to lidar.launch.py
        # restarts with urg_node.
        launch_description = _load(os.path.join(_PACKAGE_DIR, 'launch', 'lidar.launch.py'),
                                   'lidar_launch').generate_launch_description()
        assert [n._Node__node_executable for n in _nodes(launch_description)] == ['urg_node_driver']


class TestSeedCoupling:

    def test_seed_range_limit_equals_the_costmap_nodes_search_range(self):
        # costmap_boundary_node publishes max_range_m + 1.0 for "nothing within
        # range"; the node rejects seeds above seed_max_valid_m, so the two must
        # move together.
        costmap = _load(os.path.join(_SRC_DIR, 'f1tenth_costmap', 'launch', 'costmap.launch.py'),
                        'costmap_launch')
        arg = _declared(costmap.generate_launch_description())['costmap_boundary_max_range_m']
        max_range = float(_text(arg.default_value, LaunchContext()))
        assert max_range == get_value('lidar_front_wall_seed_max_valid_m')
