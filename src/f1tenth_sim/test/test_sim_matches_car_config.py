"""Sim defaults that copy a real-car config value must stay equal to it.

Each of these is a coupling (see the workspace CLAUDE.md): changing the car's
value in vesc.yaml / steering_calibration.yaml / mux.yaml means updating the
sim in the same commit, and this file is what notices when that is forgotten.
Reads the source trees directly, so it needs no built workspace or ROS.
"""
import ast
from pathlib import Path

import pytest
import yaml

from f1tenth_sim.kinematics import (
    servo_envelope,
    STEERING_MAX_RAD,
    STEERING_MIN_RAD,
)

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
VESC_YAML = SRC / 'f1tenth_bringup' / 'config' / 'vesc.yaml'
MUX_YAML = SRC / 'f1tenth_bringup' / 'config' / 'mux.yaml'
STEERING_YAML = (SRC / 'f1tenth_hardware' / 'f1tenth_hardware' / 'config'
                 / 'steering_calibration.yaml')


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _ros_params(path, node):
    return _load(path)[node]['ros__parameters']


def _drive_bridge_defaults():
    """declare_parameter(name, literal) defaults, read without importing rclpy."""
    tree = ast.parse((PKG / 'f1tenth_sim' / 'drive_bridge.py').read_text())
    out = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call)
                and getattr(call.func, 'attr', None) == 'declare_parameter'
                and isinstance(call.args[0], ast.Constant)
                and isinstance(call.args[1], ast.Constant)):
            out[call.args[0].value] = call.args[1].value
    return out


@pytest.fixture(scope='module')
def controller():
    return _ros_params(PKG / 'config' / 'controllers.yaml',
                       'ackermann_steering_controller')


def test_steering_clamp_equals_servo_limits_of_steering_calibration_yaml():
    cal = _ros_params(STEERING_YAML, '/**')
    lo, hi = servo_envelope(
        cal['servo_min'], cal['servo_max'],
        cal['steering_angle_to_servo_offset'],
        cal['steering_angle_to_servo_gain_left'],
        cal['steering_angle_to_servo_gain_right'])
    assert STEERING_MIN_RAD == pytest.approx(lo, abs=5e-5)
    assert STEERING_MAX_RAD == pytest.approx(hi, abs=5e-5)


@pytest.mark.parametrize('key', [
    'gyro_variance_x', 'gyro_variance_y', 'gyro_variance_z',
    'accel_variance_x', 'accel_variance_y', 'accel_variance_z'])
def test_sim_imu_covariance_default_equals_vesc_yaml(key):
    assert _drive_bridge_defaults()[key] == _ros_params(VESC_YAML, '/**')[key]


def test_sim_imu_frame_id_is_empty_like_vesc_driver():
    assert _drive_bridge_defaults()['imu_frame_id'] == ''


def test_sim_odom_pose_covariance_equals_vesc_to_odom(controller):
    odom = _ros_params(VESC_YAML, 'vesc_to_odom_node')
    cov = controller['pose_covariance_diagonal']
    assert (cov[0], cov[1], cov[5]) == (
        odom['x_variance'], odom['y_variance'], odom['yaw_variance'])


def test_sim_odom_vx_covariance_equals_vesc_to_odom(controller):
    odom = _ros_params(VESC_YAML, 'vesc_to_odom_node')
    assert controller['twist_covariance_diagonal'][0] == odom['vx_variance']


def test_controller_reference_timeout_equals_mux_timeout(controller):
    timeouts = {t['timeout'] for t in _ros_params(MUX_YAML, 'ackermann_mux')['topics'].values()}
    assert timeouts == {controller['reference_timeout']}


def test_drive_bridge_and_controller_share_the_urdf_wheelbase(controller):
    assert _drive_bridge_defaults()['wheelbase'] == controller['wheelbase'] == 0.325


def test_drive_bridge_listens_behind_ackermann_mux():
    assert _drive_bridge_defaults()['drive_topic'] == '/ackermann_drive'


def test_controller_does_not_broadcast_odom_tf(controller):
    assert controller['enable_odom_tf'] is False


def test_gz_bridge_never_publishes_tf():
    entries = _load(PKG / 'config' / 'ros_gz_bridge.yaml')
    assert {e['ros_topic_name'] for e in entries}.isdisjoint({'/tf', '/tf_static'})
    assert {e['direction'] for e in entries} == {'GZ_TO_ROS'}
    assert all(e['gz_type_name'].startswith('gz.msgs.') for e in entries)
