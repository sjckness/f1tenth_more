"""The Rust-port switches (guard, impl, corridor_impl, mpc_solver_impl, hard_stop_impl,
wall_tracker_impl) end to end.

bringup -> supervisor -> component.

supervisor_bringup.launch.py and stack_bringup.launch.py declare the
switches; component_supervisor_node writes them into the args of the entries
that launch behavior_bringup.launch.py / costmap.launch.py /
navigation.launch.py / ackermann_mux.launch.py / lidar_front_wall.launch.py,
found by launch_file (_RUST_SWITCHES, keyed by the argument).
wall_tracker_impl (R6) is supervisor_bringup's only: stack_bringup.launch.py does
not start lidar_front_wall_node (a supervisor component, see its launch file), so
it has nothing to pass the switch to. Plain YAML and launch inspection, no rclpy
node.
"""

import copy
import importlib.util
import os
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402, I100
    _RUST_SWITCHES, apply_rust_switches, rust_switch_defaults, validate_rust_switches)

_PKG = os.path.join(os.path.dirname(__file__), '..')

#: (bringup argument, supervisor parameter / component argument, default, choices)
_SWITCHES = (
    ('guard', 'proximity_guard', 'off', ['off', 'shadow', 'active']),
    ('impl', 'costmap_boundary_impl', 'py', ['py', 'rust']),
    ('corridor_impl', 'corridor_impl', 'py', ['py', 'rust']),
    ('mpc_solver_impl', 'mpc_solver_impl', 'py', ['py', 'rust']),
    ('hard_stop_impl', 'hard_stop_impl', 'py', ['py', 'rust']),
    ('wall_tracker_impl', 'wall_tracker_impl', 'py', ['py', 'rust']),
)

#: Switches whose component only the supervisor starts (not stack_bringup).
_SUPERVISOR_ONLY = {'wall_tracker_impl'}


def _registry():
    with open(os.path.join(_PKG, 'config', 'components.yaml')) as f:
        raw = yaml.safe_load(f)['components']
    return {name: [dict(e, args=dict(e.get('args', {}))) for e in entries]
            for name, entries in raw.items()}


def _module(launch_file):
    spec = importlib.util.spec_from_file_location(
        launch_file.replace('.', '_'), os.path.join(_PKG, 'launch', launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared(launch_file):
    return {e.name: e for e in _module(launch_file).generate_launch_description().entities
            if isinstance(e, DeclareLaunchArgument)}


@pytest.mark.parametrize('launch_file', ['supervisor_bringup.launch.py',
                                         'stack_bringup.launch.py'])
def test_both_bringups_declare_the_switches(launch_file):
    declared = _declared(launch_file)
    for name, _param, default, choices in _SWITCHES:
        if launch_file == 'stack_bringup.launch.py' and name in _SUPERVISOR_ONLY:
            assert name not in declared, name
            continue
        arg = declared[name]
        assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == default
        assert arg.choices == choices


def test_the_table_matches_the_bringup_arguments():
    assert {param: (choices[0], list(choices))
            for param, (_lf, choices) in _RUST_SWITCHES.items()} == {
        param: (default, choices) for _arg, param, default, choices in _SWITCHES}
    assert rust_switch_defaults() == {
        'proximity_guard': 'off', 'costmap_boundary_impl': 'py', 'corridor_impl': 'py',
        'mpc_solver_impl': 'py', 'hard_stop_impl': 'py', 'wall_tracker_impl': 'py'}


def test_bad_values_are_refused():
    validate_rust_switches(rust_switch_defaults())
    validate_rust_switches({'proximity_guard': 'active', 'costmap_boundary_impl': 'rust',
                            'corridor_impl': 'rust', 'mpc_solver_impl': 'rust',
                            'hard_stop_impl': 'rust', 'wall_tracker_impl': 'rust'})
    with pytest.raises(ValueError):
        validate_rust_switches({'proximity_guard': 'on'})
    with pytest.raises(ValueError):
        validate_rust_switches({'costmap_boundary_impl': 'cpp'})
    with pytest.raises(ValueError):
        validate_rust_switches({'corridor_impl': 'c'})
    with pytest.raises(ValueError):
        validate_rust_switches({'mpc_solver_impl': 'osqp'})
    with pytest.raises(ValueError):
        validate_rust_switches({'hard_stop_impl': 'cpp'})
    with pytest.raises(ValueError):
        validate_rust_switches({'wall_tracker_impl': 'cpp'})
    with pytest.raises(ValueError):
        validate_rust_switches({'solver_impl': 'rust'})
    with pytest.raises(ValueError):
        apply_rust_switches(_registry(), {'corridor_impl': 'fast'})


def test_the_switches_reach_exactly_their_launch_files():
    before = _registry()
    after = apply_rust_switches(copy.deepcopy(before), {
        'proximity_guard': 'shadow', 'costmap_boundary_impl': 'rust', 'corridor_impl': 'rust',
        'mpc_solver_impl': 'rust', 'hard_stop_impl': 'rust', 'wall_tracker_impl': 'rust'})
    expected = {'behavior_bringup.launch.py': {'proximity_guard': 'shadow'},
                'costmap.launch.py': {'costmap_boundary_impl': 'rust'},
                'navigation.launch.py': {'corridor_impl': 'rust', 'mpc_solver_impl': 'rust'},
                'ackermann_mux.launch.py': {'hard_stop_impl': 'rust'},
                'lidar_front_wall.launch.py': {'wall_tracker_impl': 'rust'}}
    touched = 0
    for name, entries in after.items():
        for old, new in zip(before[name], entries):
            extra = {k: v for k, v in new['args'].items() if old['args'].get(k) != v}
            if new['launch_file'] in expected:
                assert extra == expected[new['launch_file']], (name, new['launch_file'])
                touched += 1
            else:
                assert extra == {}, (name, new['launch_file'])
    assert touched == 5


def test_the_defaults_are_written_too():
    after = apply_rust_switches(_registry(), rust_switch_defaults())
    args = {e['launch_file']: e['args'] for entries in after.values() for e in entries}
    assert args['behavior_bringup.launch.py']['proximity_guard'] == 'off'
    assert args['costmap.launch.py']['costmap_boundary_impl'] == 'py'
    assert args['navigation.launch.py']['corridor_impl'] == 'py'
    assert args['navigation.launch.py']['mpc_solver_impl'] == 'py'
    assert args['ackermann_mux.launch.py']['hard_stop_impl'] == 'py'
    assert args['lidar_front_wall.launch.py']['wall_tracker_impl'] == 'py'


def test_navigation_launch_accepts_what_the_supervisor_writes():
    """
    Navigation.launch.py declares the argument the supervisor writes.

    The supervisor runs `ros2 launch ... navigation.launch.py corridor_impl:=...
    mpc_solver_impl:=...`: an argument the file does not declare would be refused
    at launch time.
    """
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('f1tenth_navigation'), 'launch',
                        'navigation.launch.py')
    spec = importlib.util.spec_from_file_location('nav_for_switch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = {e.name for e in module.generate_launch_description().entities
             if isinstance(e, DeclareLaunchArgument)}
    assert 'corridor_impl' in names
    assert 'mpc_solver_impl' in names


def test_ackermann_mux_launch_accepts_what_the_supervisor_writes():
    """ackermann_mux.launch.py declares hard_stop_impl (the `control` component)."""
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('f1tenth_control'), 'launch',
                        'ackermann_mux.launch.py')
    spec = importlib.util.spec_from_file_location('mux_for_switch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = {e.name: e for e in module.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    arg = declared['hard_stop_impl']
    assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == 'py'
    assert arg.choices == ['py', 'rust']


def test_lidar_front_wall_launch_accepts_what_the_supervisor_writes():
    """
    lidar_front_wall.launch.py declares wall_tracker_impl, with the switch's choices.

    The supervisor runs `ros2 launch ... lidar_front_wall.launch.py
    wall_tracker_impl:=...`: an argument the file does not declare would be
    refused at launch time.
    """
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('f1tenth_perception'), 'launch',
                        'lidar_front_wall.launch.py')
    spec = importlib.util.spec_from_file_location('lfw_for_switch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = {e.name: e for e in module.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    assert declared['wall_tracker_impl'].choices == ['py', 'rust']


def test_stack_bringup_does_not_start_the_wall_node():
    """The reason wall_tracker_impl is supervisor_bringup's only (_SUPERVISOR_ONLY)."""
    ld = _module('stack_bringup.launch.py').generate_launch_description()
    ctx = LaunchContext()
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    assert not [e for e in includes if _location(e, ctx).endswith('/lidar_front_wall.launch.py')]


def _supervisor_parameters(overrides=None):
    """Return the parameters component_supervisor_node receives, resolved as launch does."""
    ld = _module('supervisor_bringup.launch.py').generate_launch_description()
    ctx = LaunchContext()
    for key, value in (overrides or {}).items():
        ctx.launch_configurations[key] = value
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
    op = [e for e in ld.entities if isinstance(e, OpaqueFunction)][-1]
    node = next(a for a in op.execute(ctx) if isinstance(a, Node))
    # Name-mangled: launch_ros has no public accessor (see f1tenth_control's
    # test_mpc_corr_launch_config.py, which reads the same attribute).
    params = evaluate_parameters(ctx, normalize_parameters(node._Node__parameters))
    flat = {}
    for item in params:
        if isinstance(item, dict):
            flat.update(item)
    return flat


@pytest.mark.parametrize('overrides', [None, {'guard': 'shadow', 'impl': 'rust',
                                              'corridor_impl': 'rust',
                                              'mpc_solver_impl': 'rust',
                                              'hard_stop_impl': 'rust',
                                              'wall_tracker_impl': 'rust'}])
def test_the_switches_reach_the_supervisor_as_strings(overrides):
    """
    Every switch reaches the supervisor as a string.

    guard's default `off` is the YAML 1.1 boolean false when untyped, and the
    supervisor (STRING parameter) then dies in __init__ with
    InvalidParameterTypeException. Found in R1, 2026-10-23.
    """
    params = _supervisor_parameters(overrides)
    for arg, param, default, _choices in _SWITCHES:
        value = params[param]
        assert isinstance(value, str), (param, value)
        assert value == (overrides or {}).get(arg, default)


def _location(include, ctx):
    # Name-mangled like _supervisor_parameters' read: the public `location`
    # is a repr of the substitutions until the include has been performed.
    return perform_substitutions(
        ctx, include.launch_description_source._LaunchDescriptionSource__location)


def test_stack_bringup_forwards_both_mpc_switches_to_navigation():
    ld = _module('stack_bringup.launch.py').generate_launch_description()
    ctx = LaunchContext()
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    nav = [e for e in includes if _location(e, ctx).endswith('/navigation.launch.py')]
    assert len(nav) == 1
    forwarded = {perform_substitutions(ctx, list(k)) if not isinstance(k, str) else k
                 for k, _v in nav[0].launch_arguments}
    assert forwarded == {'corridor_impl', 'mpc_solver_impl'}
    assert forwarded == {'corridor_impl'}


def test_stack_bringup_forwards_hard_stop_impl_to_the_mux():
    ld = _module('stack_bringup.launch.py').generate_launch_description()
    ctx = LaunchContext()
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    mux = [e for e in includes if _location(e, ctx).endswith('/ackermann_mux.launch.py')]
    assert len(mux) == 1
    forwarded = {perform_substitutions(ctx, list(k)) if not isinstance(k, str) else k
                 for k, _v in mux[0].launch_arguments}
    assert forwarded == {'hard_stop_impl'}
