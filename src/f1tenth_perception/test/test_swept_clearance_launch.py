"""swept_clearance.launch.py tests -- LaunchDescriptions built and inspected
directly, no process spawned (the test_lidar_front_wall_launch.py convention).

Run standalone: python3 -m pytest test/test_swept_clearance_launch.py -v
"""

import importlib.util
import os

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from f1tenth_params.param_defaults import get_value

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PACKAGE_DIR = os.path.dirname(_THIS_DIR)
_SRC_DIR = os.path.dirname(_PACKAGE_DIR)


def _launch_module():
    path = os.path.join(_PACKAGE_DIR, 'launch', 'swept_clearance.launch.py')
    spec = importlib.util.spec_from_file_location('swept_clearance_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared(launch_description):
    return {e.name: e for e in launch_description.entities if isinstance(e, DeclareLaunchArgument)}


def _node(launch_description):
    nodes = [e for e in launch_description.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    return nodes[0]


def _text(substitutions, context):
    return ''.join(s.perform(context) for s in substitutions)


class TestArguments:

    def test_the_arg_list_covers_exactly_the_stack_params_block(self):
        with open(os.path.join(_SRC_DIR, 'f1tenth_params', 'config', 'stack_params.yaml')) as f:
            keys = {k for k in yaml.safe_load(f) if k.startswith('swept_clearance_')}
        names = {'swept_clearance_' + name for name, _ in _launch_module().SWEPT_CLEARANCE_ARGS}
        assert names == keys

    def test_each_explicit_value_type_matches_its_stack_params_default(self):
        for name, value_type in _launch_module().SWEPT_CLEARANCE_ARGS:
            assert type(get_value('swept_clearance_' + name)) is value_type, name

    def test_defaults_resolve_to_stack_params(self):
        module = _launch_module()
        declared = _declared(module.generate_launch_description())
        context = LaunchContext()
        for name, _ in module.SWEPT_CLEARANCE_ARGS:
            arg = declared['swept_clearance_' + name]
            assert _text(arg.default_value, context) == str(get_value('swept_clearance_' + name))


class TestNode:

    def test_runs_the_swept_clearance_node_pinned_off_the_e_stop_core(self):
        launch_description = _launch_module().generate_launch_description()
        node = _node(launch_description)
        assert node._Node__node_executable == 'swept_clearance_node'
        context = LaunchContext()
        for arg in _declared(launch_description).values():
            arg.visit(context)
        assert _text(node.process_description.prefix, context) == 'taskset -c 5'

    def test_the_camera_follows_camera_source_because_only_the_zed_has_depth(self):
        node = _node(_launch_module().generate_launch_description())
        context = LaunchContext()
        # launch_ros normalises parameter names into substitution tuples.
        params = {_text(key, context): value for key, value in node._Node__parameters[0].items()}
        assert params['use_camera'] is (get_value('camera_source') == 'zed')

    def test_if_registered_it_never_shares_a_component_with_urg_node(self):
        # It reads /scan; a component restart takes every launch file under it
        # down together (see the launch file's module docstring). Not
        # registered today, which passes trivially.
        path = os.path.join(_SRC_DIR, 'f1tenth_bringup', 'config', 'components.yaml')
        with open(path) as f:
            components = yaml.safe_load(f)['components']
        for name, entries in components.items():
            files = [entry['launch_file'] for entry in entries]
            if 'swept_clearance.launch.py' in files:
                assert files == ['swept_clearance.launch.py'], name
