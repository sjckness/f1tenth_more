"""
Sim mode: what it skips, what it keeps, and that sim=false changes nothing.

See component_supervisor_node's "Sim mode" paragraph.

Plain-Python checks on apply_sim_mode() and _ComponentProcess.cmd against the
real components.yaml, plus a source-level check that sim_hardware_tf.launch.py
still publishes the same base_link -> imu transform as vesc.launch.py. No
rclpy needed.
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
VESC_LAUNCH = os.path.join(
    HERE, '..', '..', 'f1tenth_hardware', 'f1tenth_hardware', 'launch', 'vesc.launch.py')
SIM_TF_LAUNCH = os.path.join(HERE, '..', 'launch', 'sim_hardware_tf.launch.py')


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


def test_perception_keeps_detection_and_everything_else_is_untouched():
    reg = _registry()
    sim = apply_sim_mode(reg)
    assert _files(sim['perception']) == [('f1tenth_perception', 'detection.launch.py')]
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
    """Map node name to arguments for every static_transform_publisher Node."""
    out = {}
    for node in ast.walk(ast.parse(open(path).read())):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            if (isinstance(kw.get('executable'), ast.Constant)
                    and kw['executable'].value == 'static_transform_publisher'):
                out[kw['name'].value] = [e.value for e in kw['arguments'].elts]
    return out


def test_sim_imu_tf_matches_vesc_launch():
    car = _static_tf_nodes(VESC_LAUNCH)
    sim = _static_tf_nodes(SIM_TF_LAUNCH)
    assert list(sim) == ['static_baselink_to_imu']
    assert sim['static_baselink_to_imu'] == car['static_baselink_to_imu']
