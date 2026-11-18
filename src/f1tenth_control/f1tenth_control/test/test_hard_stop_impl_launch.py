"""ackermann_mux.launch.py: hard_stop_impl:=py|rust (supervisor/stack hard_stop_impl:=).

Exactly one hard_stop_node runs, never both (same node name, same topics),
and the default is the Python one. The Rust node gets every parameter the
Python node gets, plus the two footprint values the Python node computes as
its own defaults (the URDF bumper, stack_params' body half-width).
LaunchDescriptions built and inspected directly, no process spawned.

Key parity (the R0 costmap pattern): both nodes DECLARE the same parameters,
names, types and defaults (the Python's declare_parameter calls, read with
ast; the Rust node's `x("name", default)?` calls in read_params, read from its
source), and every key the launch file passes is declared by the node it goes
to with its type. rclpy and rclrs both ignore an override nobody declared, so
a key the Rust node lacked would be dropped without a word.

Run standalone: python3 -m pytest test/test_hard_stop_impl_launch.py -v
"""

import ast
import importlib.util
import math
import os
import re

from f1tenth_control.hard_stop_node import _default_front_x, _default_half_width
from f1tenth_params.param_defaults import get_value
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_LAUNCH = os.path.join(_PKG, 'launch', 'ackermann_mux.launch.py')
_PY_NODE = os.path.join(_PKG, 'f1tenth_control', 'hard_stop_node.py')
# The Rust port's source: src/rust next to src/f1tenth_control.
_RS_MAIN = os.path.join(os.path.dirname(os.path.dirname(_PKG)), 'rust',
                        'f1tenth_hard_stop_rs', 'src', 'main.rs')

# The two values the Python computes as defaults and the launch file passes
# to the Rust node, which declares them without a usable default (NaN).
_LAUNCH_SUPPLIED = {'footprint_front_x_m', 'footprint_half_width_m'}


def _description():
    spec = importlib.util.spec_from_file_location('ackermann_mux_launch', _LAUNCH)
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


def _hard_stop_nodes(description):
    return {e.node_package: e for e in description.entities
            if isinstance(e, Node) and e.node_executable == 'hard_stop_node'}


def _launched(description, context, pkg):
    node = _hard_stop_nodes(description)[pkg]
    return evaluate_parameters(context, node._Node__parameters)[0]


def test_the_switch_defaults_to_py_from_stack_params():
    declared = {e.name: e for e in _description().entities
                if isinstance(e, DeclareLaunchArgument)}
    arg = declared['hard_stop_impl']
    assert ''.join(s.perform(LaunchContext()) for s in arg.default_value) == 'py'
    assert get_value('hard_stop_impl') == 'py'
    assert arg.choices == ['py', 'rust']


@pytest.mark.parametrize('impl', [None, 'py', 'rust'])
def test_exactly_one_implementation_runs(impl):
    description = _description()
    nodes = _hard_stop_nodes(description)
    assert set(nodes) == {'f1tenth_control', 'f1tenth_hard_stop_rs'}
    context = _context(description, **({} if impl is None else {'hard_stop_impl': impl}))
    running = {pkg for pkg, node in nodes.items() if node.condition.evaluate(context)}
    assert running == {'f1tenth_hard_stop_rs' if impl == 'rust' else 'f1tenth_control'}


def test_the_mux_runs_with_either():
    description = _description()
    mux = [e for e in description.entities
           if isinstance(e, Node) and e.node_package == 'ackermann_mux']
    assert len(mux) == 1 and mux[0].condition is None


def test_same_name_and_a_superset_of_the_parameters():
    description = _description()
    context = _context(description)
    py = _launched(description, context, 'f1tenth_control')
    rs = _launched(description, context, 'f1tenth_hard_stop_rs')
    assert {k: rs[k] for k in py} == py
    assert set(rs) - set(py) == _LAUNCH_SUPPLIED
    # Exactly what the Python node uses when nobody overrides it.
    assert rs['footprint_front_x_m'] == _default_front_x() == 0.405
    assert rs['footprint_half_width_m'] == _default_half_width() == 0.136
    names = {n._Node__node_name for n in _hard_stop_nodes(description).values()}
    assert names == {'hard_stop_node'}


def test_overrides_reach_both_nodes_with_their_types():
    description = _description()
    context = _context(description, hard_stop_enabled='false', hard_stop_range_m='0.25',
                       hard_stop_debounce_scans='3', hard_stop_clear_s='0.7',
                       hard_stop_allow_reverse='false')
    expected = {'hard_stop_enabled': False, 'hard_stop_range_m': 0.25,
                'hard_stop_debounce_scans': 3, 'hard_stop_clear_s': 0.7,
                'hard_stop_allow_reverse': False}
    for pkg in ('f1tenth_control', 'f1tenth_hard_stop_rs'):
        launched = _launched(description, context, pkg)
        got = {k: launched[k] for k in expected}
        assert got == expected, pkg
        assert all(type(got[k]) is type(v) for k, v in expected.items()), (pkg, got)


# ---------------------------------------------------------------- key parity
def _python_declared():
    """{name: default} of HardStopNode.__init__'s declare_parameter calls."""
    import f1tenth_control.hard_stop_node as module
    with open(_PY_NODE) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'HardStopNode')
    out = {}
    for call in ast.walk(cls):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'declare_parameter'):
            name, default = call.args
            assert isinstance(name, ast.Constant), ast.dump(name)
            assert name.value not in out, f'{name.value} declared twice'
            if isinstance(default, ast.Name):
                value = getattr(module, default.id)
            elif isinstance(default, ast.Call) and isinstance(default.func, ast.Name):
                value = getattr(module, default.func.id)()
            else:
                value = ast.literal_eval(default)
            out[name.value] = value
    return out


_RS_DECL = re.compile(r'\b([sfib])\(\s*"([A-Za-z0-9_]+)"\s*,\s*([^)]*?)\s*\)\?')
_RS_TYPES = {'s': str, 'f': float, 'i': int, 'b': bool}


def _rust_declared():
    """{name: default} of read_params's s/f/i/b calls (main.rs)."""
    if not os.path.exists(_RS_MAIN):
        pytest.skip(f'Rust source not in this tree: {_RS_MAIN}')
    with open(_RS_MAIN) as f:
        src = f.read()
    start = src.index('fn read_params(')
    body = src[start:src.index('\nfn ', start)]
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
    # Guards the parser: a refactor of read_params must not make this vacuous.
    assert len(rs) == 15, sorted(rs)
    assert _LAUNCH_SUPPLIED <= set(rs)


def test_both_nodes_declare_the_same_parameters_types_and_defaults():
    py = _python_declared()
    rs = _rust_declared()
    assert set(rs) == set(py), {'rust_lacks': sorted(set(py) - set(rs)),
                                'python_lacks': sorted(set(rs) - set(py))}
    for name, default in py.items():
        assert type(rs[name]) is type(default), (name, rs[name], default)
        if name in _LAUNCH_SUPPLIED:
            # No literal in the Python (computed); none in Rust (refused if absent).
            assert math.isnan(rs[name]), name
        else:
            assert rs[name] == default, (name, rs[name], default)


def test_every_launch_key_is_declared_by_its_node_with_its_type():
    description = _description()
    context = _context(description)
    declared = {'f1tenth_control': _python_declared(),
                'f1tenth_hard_stop_rs': _rust_declared()}
    for pkg in declared:
        launched = _launched(description, context, pkg)
        undeclared = set(launched) - set(declared[pkg])
        assert not undeclared, (pkg, sorted(undeclared))
        for key, value in launched.items():
            assert type(value) is type(declared[pkg][key]), (pkg, key, value)
