"""
Sim mode: what it skips, what it keeps, and that sim=false changes nothing.

See component_supervisor_node's "Sim mode" paragraph.

Plain-Python checks on apply_sim_mode() and _ComponentProcess.cmd against the
real components.yaml, plus source-level checks that sim_hardware_tf.launch.py
still publishes the same base_link -> imu transform as vesc.launch.py, and
sim_camera_tf.launch.py the same camera TF as camera.launch.py. No rclpy
needed.
"""
import ast
import copy
import os
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..', 'f1tenth_bringup'))

from component_supervisor_node import (  # noqa: E402,I100
    _ComponentProcess, _SIM_SKIPPED_LAUNCH_FILES, apply_sim_mode)
import yaml  # noqa: E402,I100

COMPONENTS = os.path.join(HERE, '..', 'config', 'components.yaml')
SRC = os.path.join(HERE, '..', '..')
VESC_LAUNCH = os.path.join(
    SRC, 'f1tenth_hardware', 'f1tenth_hardware', 'launch', 'vesc.launch.py')
SIM_TF_LAUNCH = os.path.join(HERE, '..', 'launch', 'sim_hardware_tf.launch.py')
SIM_COMPONENT_LAUNCH = os.path.join(HERE, '..', 'launch', 'sim_component.launch.py')
PERCEPTION_LAUNCH = os.path.join(SRC, 'f1tenth_perception', 'launch')
CAMERA_LAUNCH = os.path.join(PERCEPTION_LAUNCH, 'camera.launch.py')
SIM_CAMERA_TF_LAUNCH = os.path.join(PERCEPTION_LAUNCH, 'sim_camera_tf.launch.py')
DESCRIPTION_LAUNCH = os.path.join(
    SRC, 'f1tenth_description', 'launch', 'description.launch.py')
SIM_MOUNTS_YAML = os.path.join(
    SRC, 'f1tenth_description', 'config', 'sim_sensor_mounts.yaml')


def _registry():
    with open(COMPONENTS) as f:
        raw = yaml.safe_load(f)['components']
    return {n: [dict(e, args=e.get('args', {})) for e in es] for n, es in raw.items()}


def _files(entries):
    return [(e['package'], e['launch_file']) for e in entries]


def test_no_driver_launch_file_survives_sim_mode():
    sim = apply_sim_mode(_registry())
    for name, entries in sim.items():
        assert not set(_files(entries)) & _SIM_SKIPPED_LAUNCH_FILES, name


def test_every_skipped_launch_file_exists_in_the_registry():
    # A renamed driver launch file would otherwise silently start in sim.
    present = {f for es in _registry().values() for f in _files(es)}
    assert _SIM_SKIPPED_LAUNCH_FILES <= present


def test_hardware_keeps_only_the_imu_static_tf_and_calibration_nothing():
    sim = apply_sim_mode(_registry())
    assert _files(sim['hardware']) == [('f1tenth_bringup', 'sim_hardware_tf.launch.py')]
    assert sim['calibrate_hardware'] == []


def test_perception_swaps_camera_for_its_tf_and_everything_else_is_untouched():
    reg = _registry()
    sim = apply_sim_mode(reg)
    assert _files(sim['perception']) == [
        ('f1tenth_perception', 'sim_camera_tf.launch.py'),
        ('f1tenth_perception', 'detection.launch.py')]
    for name in set(reg) - {'hardware', 'calibrate_hardware', 'perception'}:
        assert sim[name] == reg[name], name


def test_apply_sim_mode_does_not_modify_its_input():
    reg = _registry()
    before = copy.deepcopy(reg)
    apply_sim_mode(reg)
    assert reg == before


def test_cmd_without_sim_is_the_plain_ros2_launch():
    p = _ComponentProcess('f1tenth_logger', 'mission_logger.launch.py',
                          {'a': 'b'}, '/dev/null')
    assert p.cmd == ['ros2', 'launch', 'f1tenth_logger', 'mission_logger.launch.py', 'a:=b']


def test_cmd_with_sim_wraps_the_same_launch_file_and_args():
    p = _ComponentProcess('f1tenth_logger', 'mission_logger.launch.py',
                          {'a': 'b'}, '/dev/null', sim=True)
    assert p.cmd == ['ros2', 'launch', 'f1tenth_bringup', 'sim_component.launch.py',
                     'component_package:=f1tenth_logger',
                     'component_launch_file:=mission_logger.launch.py', 'a:=b']


def _static_tf_nodes(path):
    """Map node name -> list of each matching node's LITERAL argument list.

    Nodes whose `arguments` are built at runtime (e.g. splatted from the sim
    mounts yaml) have no literal list and are skipped; a name can map to more
    than one node (description.launch.py's car + sim laser branches).
    """
    out = {}
    for node in ast.walk(ast.parse(open(path).read())):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            if (isinstance(kw.get('executable'), ast.Constant)
                    and kw['executable'].value == 'static_transform_publisher'
                    and isinstance(kw['arguments'], ast.List)
                    and all(isinstance(e, ast.Constant) for e in kw['arguments'].elts)):
                out.setdefault(kw['name'].value, []).append(
                    [e.value for e in kw['arguments'].elts])
    return out


def test_sim_imu_tf_matches_vesc_launch():
    car = _static_tf_nodes(VESC_LAUNCH)
    sim = _static_tf_nodes(SIM_TF_LAUNCH)
    assert list(sim) == ['static_baselink_to_imu']
    assert sim['static_baselink_to_imu'] == car['static_baselink_to_imu']


def _sim_mounts():
    with open(SIM_MOUNTS_YAML) as f:
        return yaml.safe_load(f)


def test_sim_component_sets_the_sim_flag():
    # The supervisor's sim path sets `sim` true here (like use_sim_time), which
    # propagates into description.launch.py. On the car these files run directly.
    src = open(SIM_COMPONENT_LAUNCH).read()
    assert "SetLaunchConfiguration('sim', 'true')" in src


def test_sim_camera_tf_uses_the_sim_mount_not_the_cars():
    # Car keeps its own base_link->zed2_camera_link; the sim one reads the yaml.
    car = _static_tf_nodes(CAMERA_LAUNCH)['static_baselink_to_zed2']
    assert car == [['0.12', '0.0', '0.15', '0.0', '0.0', '0.0',
                    'base_link', 'zed2_camera_link']]
    sim_src = open(SIM_CAMERA_TF_LAUNCH).read()
    assert 'sim_sensor_mounts.yaml' in sim_src
    assert "mounts['zed2_camera_link']" in sim_src
    # and that is a different pose from the car's
    assert _sim_mounts()['zed2_camera_link']['xyz'] == [0.36, 0.0, 0.25]


def test_sim_laser_tf_uses_the_sim_mount_and_car_branch_is_unchanged():
    # description.launch.py has two static_baselink_to_laser nodes: the car one
    # (literal, UnlessCondition(sim)) and the sim one (yaml, IfCondition(sim)).
    laser = _static_tf_nodes(DESCRIPTION_LAUNCH)['static_baselink_to_laser']
    assert laser == [['0.12', '0.0', '0.20', '0.0', '0.0', '0.0',
                      'base_link', 'laser']]           # car branch, unchanged
    src = open(DESCRIPTION_LAUNCH).read()
    assert 'sim_sensor_mounts.yaml' in src
    assert "sim_mounts['laser']" in src
    assert 'UnlessCondition(sim)' in src and 'IfCondition(sim)' in src
    assert _sim_mounts()['laser']['xyz'] == [0.40, 0.0, 0.155]


def _zed_wrapper_args(path):
    """The camera_name / camera_model literals camera.launch.py passes to the wrapper."""
    for node in ast.walk(ast.parse(open(path).read())):
        if isinstance(node, ast.Dict):
            keys = [k.value for k in node.keys if isinstance(k, ast.Constant)]
            if 'camera_model' in keys:
                return {k.value: v.value for k, v in zip(node.keys, node.values)
                        if k.value in ('camera_name', 'camera_model')}
    raise AssertionError('no camera_model argument in ' + path)


def _module_constants(path):
    out = {}
    for node in ast.parse(open(path).read()).body:
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)
                and isinstance(node.targets[0], ast.Name)):
            out[node.targets[0].id] = node.value.value
    return out


def test_sim_camera_tree_uses_the_cars_zed_name_and_model():
    car = _zed_wrapper_args(CAMERA_LAUNCH)
    sim = _module_constants(SIM_CAMERA_TF_LAUNCH)
    assert (sim['CAMERA_NAME'], sim['CAMERA_MODEL']) == (car['camera_name'], car['camera_model'])
    assert car == {'camera_name': 'zed2', 'camera_model': 'zed2i'}
