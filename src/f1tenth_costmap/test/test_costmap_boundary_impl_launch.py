"""costmap.launch.py: costmap_boundary_impl:=py|rust (supervisor/stack impl:=).

Exactly one costmap_boundary_node runs, never both (same node name, same
topics), and none without enable_slam. The Rust node gets every parameter the
Python node gets plus the three vehicle_frames values the Python imports.
LaunchDescriptions built and inspected directly, no process spawned.

Key parity (R0, main-jazzy-rust): both nodes DECLARE the same parameters,
names, types and defaults (the Python's declare_parameter calls, read with
ast; the Rust node's `x("name", default)?` calls in read_config, read from
its source), and every key the launch file passes is declared by the node it
goes to. rclpy and rclrs both ignore an override nobody declared, so a key
the Rust node lacked would be dropped without a word: the costmap-mode
cleanup switches (component_filter, exclude_tracked_object_cells), which the
Rust node refuses when true, would then be silently ignored instead.

Run standalone: python3 -m pytest test/test_costmap_boundary_impl_launch.py -v
"""

import ast
import importlib.util
import math
import os
import re

from f1tenth_description.vehicle_frames import (
    base_front_x, front_side_limit_m, MIN_FRONT_DISTANCE_M)
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_LAUNCH = os.path.join(_PKG, 'launch', 'costmap.launch.py')
_PY_NODE = os.path.join(_PKG, 'f1tenth_costmap', 'costmap_boundary_node.py')
# The Rust port's source, next to this package in the workspace's src/.
_RS_MAIN = os.path.join(os.path.dirname(_PKG), 'rust', 'f1tenth_costmap_boundary_rs',
                        'src', 'main.rs')

# Declared by the Rust node only: the three vehicle_frames values the Python
# imports (the launch file passes them) and its own timing log.
_RUST_ONLY = {'base_front_x_m', 'front_side_limit_m', 'min_front_distance_m',
              'timing_report_period_s'}
# The costmap-mode cleanup parameters (main-jazzy), with the Python defaults.
_CLEANUP_DEFAULTS = {
    'component_filter': False,
    'min_component_length_m': 0.5,
    'min_component_scans': 2,
    'exclude_tracked_object_cells': False,
    'object_footprint_margin_m': 0.12,
}


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


# ---------------------------------------------------------------- key parity
def _python_declared():
    """{name: default} of CostmapBoundaryNode.__init__'s declare_parameter calls."""
    import f1tenth_costmap.costmap_boundary_node as module
    with open(_PY_NODE) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'CostmapBoundaryNode')
    out = {}
    for call in ast.walk(cls):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'declare_parameter'):
            name, default = call.args
            assert isinstance(name, ast.Constant), ast.dump(name)
            assert name.value not in out, f'{name.value} declared twice'
            out[name.value] = (getattr(module, default.id) if isinstance(default, ast.Name)
                               else ast.literal_eval(default))
    return out


_RS_DECL = re.compile(r'\b([sfib])\(\s*"([A-Za-z0-9_]+)"\s*,\s*([^)]*?)\s*\)\?')
_RS_TYPES = {'s': str, 'f': float, 'i': int, 'b': bool}


def _rust_declared():
    """{name: default} of read_config's s/f/i/b calls (main.rs)."""
    if not os.path.exists(_RS_MAIN):
        pytest.skip(f'Rust source not in this tree: {_RS_MAIN}')
    with open(_RS_MAIN) as f:
        src = f.read()
    body = src[src.index('fn read_config('):src.index('\nfn ', src.index('fn read_config('))]
    out = {}
    for kind, name, text in _RS_DECL.findall(body):
        assert name not in out, f'{name} declared twice'
        if kind == 's':
            assert text.startswith('"') and text.endswith('"'), text
            value = text[1:-1]
        elif kind == 'f':
            value = math.nan if text == 'f64::NAN' else float(text)
        elif kind == 'i':
            value = int(text)
        else:
            assert text in ('true', 'false'), text
            value = text == 'true'
        assert type(value) is _RS_TYPES[kind], (name, text)
        out[name] = value
    return out


def test_the_rust_declarations_are_read():
    rs = _rust_declared()
    # Guards the parser: a refactor of read_config must not make this vacuous.
    assert len(rs) >= 32, sorted(rs)
    assert {'map_topic', 'use_convex_polytope', 'polytope_max_faces',
            *_CLEANUP_DEFAULTS, *_RUST_ONLY} <= set(rs)


def test_both_nodes_declare_the_same_parameters_types_and_defaults():
    py = _python_declared()
    rs = _rust_declared()
    assert set(rs) - _RUST_ONLY == set(py), {
        'rust_lacks': sorted(set(py) - set(rs)),
        'python_lacks': sorted(set(rs) - _RUST_ONLY - set(py))}
    for name, default in py.items():
        assert type(rs[name]) is type(default), (name, rs[name], default)
        assert rs[name] == default, (name, rs[name], default)


def test_the_cleanup_parameters_have_the_python_defaults_in_both_nodes():
    py = _python_declared()
    rs = _rust_declared()
    for name, default in _CLEANUP_DEFAULTS.items():
        assert type(py[name]) is type(default) and py[name] == default, name
        assert type(rs[name]) is type(default) and rs[name] == default, name
    assert py['obstacles_topic'] == rs['obstacles_topic'] == '/perception/obstacles_2d'


def test_every_launch_key_is_declared_by_its_node_with_its_type():
    description = _description()
    nodes = _boundary_nodes(description)
    context = _context(description)
    declared = {'f1tenth_costmap': _python_declared(),
                'f1tenth_costmap_boundary_rs': _rust_declared()}
    for pkg, node in nodes.items():
        launched = evaluate_parameters(context, node._Node__parameters)[0]
        undeclared = set(launched) - set(declared[pkg])
        assert not undeclared, (pkg, sorted(undeclared))
        for key, value in launched.items():
            # A launch value of another type fails the declaration at startup.
            assert type(value) is type(declared[pkg][key]), (pkg, key, value)


def test_the_cleanup_keys_reach_both_nodes():
    description = _description()
    nodes = _boundary_nodes(description)
    for overrides, expected in (
            ({}, _CLEANUP_DEFAULTS),
            ({'costmap_boundary_component_filter': 'true',
              'costmap_boundary_min_component_length_m': '0.8',
              'costmap_boundary_min_component_scans': '3',
              'costmap_boundary_exclude_tracked_object_cells': 'true',
              'costmap_boundary_object_footprint_margin_m': '0.2'},
             {'component_filter': True, 'min_component_length_m': 0.8,
              'min_component_scans': 3, 'exclude_tracked_object_cells': True,
              'object_footprint_margin_m': 0.2})):
        context = _context(description, **overrides)
        for pkg, node in nodes.items():
            launched = evaluate_parameters(context, node._Node__parameters)[0]
            got = {k: launched[k] for k in expected}
            assert got == expected, (pkg, overrides)
            assert all(type(got[k]) is type(v) for k, v in expected.items()), (pkg, got)
