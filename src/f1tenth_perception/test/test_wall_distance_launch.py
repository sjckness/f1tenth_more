"""wall_distance.launch.py tests -- LaunchDescriptions built and inspected
directly, no process spawned (the test_lidar_front_wall_launch.py convention).

The load-bearing test is the placement one: wall_distance_node must never be
launched from lidar.launch.py, because restarting that file restarts urg_node,
the e-stop's only /scan publisher (see wall_distance.launch.py's module
docstring). f1tenth_bringup's test_wall_distance_component.py guards the
supervisor half of the same constraint.

The second most load-bearing is TestArguments: the arg lists and the
stack_params.yaml blocks must cover each other exactly, in both directions. A
key in the YAML that no launch arg forwards is a parameter that silently does
nothing -- which is what every glass_* gate was before this pass, and what
DetectorConfig's docstring falsely claimed otherwise.

Run standalone: python3 -m pytest test/test_wall_distance_launch.py -v
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


def _wall_distance_launch():
    return _load(os.path.join(_PACKAGE_DIR, 'launch', 'wall_distance.launch.py'),
                 'wall_distance_launch')


def _stack_params():
    with open(os.path.join(_SRC_DIR, 'f1tenth_params', 'config', 'stack_params.yaml')) as f:
        return yaml.safe_load(f)


def _declared(launch_description):
    return {e.name: e for e in launch_description.entities
            if isinstance(e, DeclareLaunchArgument)}


def _nodes(launch_description):
    return [e for e in launch_description.entities if isinstance(e, Node)]


def _text(substitutions, context):
    return ''.join(s.perform(context) for s in substitutions)


class TestArguments:

    def test_the_wall_distance_arg_list_covers_exactly_its_stack_params_block(self):
        keys = {k for k in _stack_params() if k.startswith('wall_distance_')}
        names = {'wall_distance_' + n for n, _ in _wall_distance_launch().WALL_DISTANCE_ARGS}
        assert names == keys

    def test_the_glass_arg_list_covers_exactly_the_glass_block(self):
        keys = {k for k in _stack_params() if k.startswith('glass_')}
        names = {'glass_' + n for n, _ in _wall_distance_launch().GLASS_ARGS}
        assert names == keys

    def test_each_explicit_value_type_matches_its_stack_params_default(self):
        module = _wall_distance_launch()
        for prefix, args in (('wall_distance_', module.WALL_DISTANCE_ARGS),
                             ('glass_', module.GLASS_ARGS)):
            for name, value_type in args:
                assert type(get_value(prefix + name)) is value_type, prefix + name

    def test_defaults_resolve_to_stack_params(self):
        module = _wall_distance_launch()
        declared = _declared(module.generate_launch_description())
        context = LaunchContext()
        for prefix, args in (('wall_distance_', module.WALL_DISTANCE_ARGS),
                             ('glass_', module.GLASS_ARGS)):
            for name, _ in args:
                arg = declared[prefix + name]
                assert _text(arg.default_value, context) == str(get_value(prefix + name))

    def test_every_glass_gate_is_a_real_DetectorConfig_field(self):
        """A glass_ key that DetectorConfig does not have would be forwarded to
        the node and then rejected at construction -- at runtime, on the car."""
        from f1tenth_perception.glass_detect import DetectorConfig
        defaults = DetectorConfig()
        for name, _ in _wall_distance_launch().GLASS_ARGS:
            assert hasattr(defaults, name), name

    def test_every_DetectorConfig_field_has_a_stack_param(self):
        """The other direction, and the one that actually bit: a gate with no
        key is untunable without a rebuild. gain_lut is exempt -- it is an
        optional lookup table, not a scalar, and there is no sensible YAML
        spelling for it."""
        from dataclasses import fields
        from f1tenth_perception.glass_detect import DetectorConfig
        forwarded = {name for name, _ in _wall_distance_launch().GLASS_ARGS}
        scalar_fields = {f.name for f in fields(DetectorConfig)} | {'gain_lut'}
        missing = scalar_fields - forwarded - {'gain_lut'}
        assert not missing, f'DetectorConfig gates with no glass_ stack param: {missing}'


class TestFootprintIsSharedNotCopied:

    def test_the_footprint_comes_from_the_swept_clearance_keys(self):
        """One car, one rectangle. Two nodes computing swept clearance off two
        copies of the footprint is two clearances that disagree for reasons
        nobody can find."""
        module = _wall_distance_launch()
        for key, _node_name, _value_type in module.SWEPT_GEOMETRY_ARGS:
            assert key.startswith('swept_clearance_'), key
            assert key in _stack_params(), key

    def test_the_footprint_args_are_declared_so_a_cli_override_reaches_both(self):
        module = _wall_distance_launch()
        declared = _declared(module.generate_launch_description())
        context = LaunchContext()
        for key, _node_name, _value_type in module.SWEPT_GEOMETRY_ARGS:
            assert key in declared, key
            assert _text(declared[key].default_value, context) == str(get_value(key))

    def test_the_swept_clearance_launch_file_declares_the_same_keys(self):
        """If the two files ever stop declaring the same footprint arg names, a
        CLI override would move one node and not the other."""
        mine = {k for k, _, _ in _wall_distance_launch().SWEPT_GEOMETRY_ARGS}
        theirs = _load(os.path.join(_PACKAGE_DIR, 'launch', 'swept_clearance.launch.py'),
                       'swept_clearance_launch')
        theirs = {'swept_clearance_' + n for n, _ in theirs.SWEPT_CLEARANCE_ARGS}
        assert mine <= theirs, mine - theirs


class TestNode:

    def _launch(self):
        launch_description = _wall_distance_launch().generate_launch_description()
        nodes = _nodes(launch_description)
        assert len(nodes) == 1
        return launch_description, nodes[0]

    def test_runs_the_wall_distance_node(self):
        _, node = self._launch()
        assert node._Node__node_executable == 'wall_distance_node'

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

    def test_the_entry_point_exists(self):
        with open(os.path.join(_PACKAGE_DIR, 'setup.py')) as f:
            setup_py = f.read()
        assert ('wall_distance_node = f1tenth_perception.wall_distance_node:main'
                in setup_py)


class TestPlacement:

    def test_lidar_launch_runs_urg_node_and_nothing_else(self):
        # See this file's module docstring: anything added to lidar.launch.py
        # restarts with urg_node, and urg_node is the e-stop's only /scan.
        launch_description = _load(os.path.join(_PACKAGE_DIR, 'launch', 'lidar.launch.py'),
                                   'lidar_launch').generate_launch_description()
        assert [n._Node__node_executable for n in _nodes(launch_description)] == ['urg_node_driver']

    def test_this_launch_file_runs_only_its_own_node(self):
        """A second node in here would restart with wall_distance_node, which
        is the component most likely to be restarted on the car."""
        launch_description = _wall_distance_launch().generate_launch_description()
        nodes = _nodes(launch_description)
        assert [n._Node__node_executable for n in nodes] == ['wall_distance_node']


class TestSweptClearanceIsRegistered:
    """Phase 2: swept_clearance_node existed, was tested, had an entry point and
    a launch file, and appeared in NO component and no other launch file -- so
    it had never run on the car. These assert the registration, not that it
    works; it still has not run."""

    def test_swept_clearance_has_a_component_entry(self):
        with open(os.path.join(_SRC_DIR, 'f1tenth_bringup', 'config',
                               'components.yaml')) as f:
            components = yaml.safe_load(f)['components']
        assert 'swept_clearance' in components
        assert [e['launch_file'] for e in components['swept_clearance']] == [
            'swept_clearance.launch.py']

    def test_its_launch_file_no_longer_claims_to_be_unregistered(self):
        """The docstring said 'NOT REGISTERED IN components.yaml'. A stale
        comment that contradicts the config is exactly the failure
        docs/wall_turn_investigation.md catalogues three times over."""
        with open(os.path.join(_PACKAGE_DIR, 'launch', 'swept_clearance.launch.py')) as f:
            text = f.read()
        assert 'NOT REGISTERED' not in text
