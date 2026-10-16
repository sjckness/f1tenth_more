"""sim:=isaac in the component supervisor (apply_sim_mode and the sim command wrapper).

The car path must be untouched: with sim 'none' a component's command is the plain
`ros2 launch <pkg> <file> k:=v` it always was. In sim, device drivers are gone, the frame
publishers they owned are replaced, and every launch file runs through
sim_component.launch.py (use_sim_time for every node). Plain Python, no rclpy.
"""
import ast
import copy
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402
    _ComponentProcess,
    _SIM_DRIVER_LAUNCH_FILES,
    _SIM_REPLACEMENTS,
    apply_sim_mode,
    load_sim_component_args,
)

_PKG = os.path.join(os.path.dirname(__file__), '..')
_CONFIG = os.path.join(_PKG, 'config')
_SRC = os.path.join(_PKG, '..')


def _registry():
    with open(os.path.join(_CONFIG, 'components.yaml')) as f:
        raw = yaml.safe_load(f)['components']
    return {name: [dict(e, args=e.get('args', {})) for e in entries]
            for name, entries in raw.items()}


def _files(registry, name):
    return [(e['package'], e['launch_file']) for e in registry[name]]


def test_no_device_driver_launch_file_survives_sim_mode():
    sim = apply_sim_mode(_registry())
    for name, entries in sim.items():
        for e in entries:
            assert (e['package'], e['launch_file']) not in _SIM_DRIVER_LAUNCH_FILES, name
            assert (e['package'], e['launch_file']) != \
                ('f1tenth_perception', 'camera.launch.py'), name


def test_every_launch_file_sim_mode_names_exists_in_the_registry():
    files = {(e['package'], e['launch_file']) for entries in _registry().values() for e in entries}
    assert _SIM_DRIVER_LAUNCH_FILES <= files
    assert {key[1:] for key in _SIM_REPLACEMENTS} <= files


def test_hardware_keeps_only_the_imu_tf_and_calibration_nothing():
    sim = apply_sim_mode(_registry())
    assert _files(sim, 'hardware') == [('f1tenth_bringup', 'sim_hardware_tf.launch.py')]
    assert sim['calibrate_hardware'] == []


def test_perception_swaps_the_camera_for_its_frames_and_keeps_detection():
    sim = apply_sim_mode(_registry())
    assert _files(sim, 'perception') == [
        ('f1tenth_bringup', 'sim_camera_tf.launch.py'),
        ('f1tenth_perception', 'detection.launch.py')]


def test_every_other_component_is_untouched():
    car = _registry()
    sim = apply_sim_mode(car)
    for name in set(car) - {'hardware', 'calibrate_hardware', 'perception'}:
        assert sim[name] == car[name], name


def test_apply_sim_mode_does_not_modify_its_input():
    car = _registry()
    before = copy.deepcopy(car)
    apply_sim_mode(car, {'diagnostics': {'mission_logger_runs_dir': '/tmp/x'}})
    assert car == before


def test_overlay_args_come_after_and_win_over_the_registry_args():
    car = {'c': [{'package': 'p', 'launch_file': 'f.launch.py', 'args': {'a': '1', 'b': '2'}}]}
    sim = apply_sim_mode(car, {'c': {'b': 'overlay', 'n': 3}})
    assert sim['c'][0]['args'] == {'a': '1', 'b': 'overlay', 'n': '3'}


def test_an_overlay_naming_an_unknown_component_is_an_error():
    with pytest.raises(ValueError, match='not in components.yaml'):
        apply_sim_mode(_registry(), {'no_such_component': {'x': 1}})


def test_the_shipped_overlays_name_real_components_and_args():
    overlays = [os.path.join(_CONFIG, 'sim', 'isaac.yaml'),
                os.path.join(_CONFIG, 'sim', 'hosts', 'linus.yaml')]
    args = load_sim_component_args(*overlays)
    sim = apply_sim_mode(_registry(), args)   # raises on an unknown component
    assert sim['diagnostics'][0]['args']['mission_logger_runs_dir'] == '~/f1tenth_sim_archive'
    assert all(e['args']['cpu_affinity'] == '6,7' for e in sim['navigation'])
    assert all(e['args']['yolo_cpu_affinity'] == '4,5' for e in sim['perception'])


def test_later_overlays_win():
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        a, b = os.path.join(d, 'a.yaml'), os.path.join(d, 'b.yaml')
        for path, value in ((a, 'A'), (b, 'B')):
            with open(path, 'w') as f:
                yaml.safe_dump({'component_supervisor_node': {'ros__parameters': {
                    'sim_component_args': {'c': {'x': value, path[-6]: 1}}}}}, f)
        assert load_sim_component_args(a, b) == {'c': {'x': 'B', 'a': 1, 'b': 1}}
        assert load_sim_component_args('', a) == {'c': {'x': 'A', 'a': 1}}


def test_cmd_without_sim_is_the_plain_ros2_launch():
    proc = _ComponentProcess('pkg', 'file.launch.py', {'a': '1', 'b': 'two'}, '/tmp/log')
    assert proc.cmd == ['ros2', 'launch', 'pkg', 'file.launch.py', 'a:=1', 'b:=two']


def test_cmd_with_sim_wraps_the_same_launch_file_and_args():
    proc = _ComponentProcess('pkg', 'file.launch.py', {'a': '1'}, '/tmp/log',
                             sim_wrapper_args={'sim_params_file': '/o.yaml'})
    assert proc.cmd == ['ros2', 'launch', 'f1tenth_bringup', 'sim_component.launch.py',
                        'component_package:=pkg', 'component_launch_file:=file.launch.py',
                        'sim_params_file:=/o.yaml', 'a:=1']


def _static_tf_node(path, name):
    """Return (executable, arguments) of the Node(name=<name>) in a launch file, by AST."""
    tree = ast.parse(open(path).read())
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and getattr(call.func, 'id', '') == 'Node':
            kw = {k.arg: k.value for k in call.keywords}
            if isinstance(kw.get('name'), ast.Constant) and kw['name'].value == name:
                args = kw.get('arguments')
                return (kw['executable'].value,
                        [a.value for a in args.elts] if isinstance(args, ast.List) else None)
    raise AssertionError(f'no Node {name} in {path}')


def test_sim_imu_tf_is_exactly_the_cars():
    car = _static_tf_node(os.path.join(
        _SRC, 'f1tenth_hardware', 'f1tenth_hardware', 'launch', 'vesc.launch.py'),
        'static_baselink_to_imu')
    sim = _static_tf_node(os.path.join(_PKG, 'launch', 'sim_hardware_tf.launch.py'),
                          'static_baselink_to_imu')
    assert sim == car


def test_sim_camera_tf_uses_the_cars_node_name_and_v4_frame_names():
    path = os.path.join(_PKG, 'launch', 'sim_camera_tf.launch.py')
    text = open(path).read()
    assert "name='static_baselink_to_zed2'" in text
    for frame in ('zed2_camera_link', 'zed2_camera_center', 'zed2_left_camera_frame',
                  'zed2_left_camera_optical_frame', 'zed2_right_camera_frame',
                  'zed2_right_camera_optical_frame'):
        assert f'name="{frame}"' in text
    assert 'frame_optical"' not in text   # no v5 link names (zed2_left_camera_frame_optical)
