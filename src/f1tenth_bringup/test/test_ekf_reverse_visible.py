"""
The local EKF must not hide a reverse.

Companion to vesc_ackermann's test_odom_speed.cpp, which pins the sign at the
/odom end. This end checks the fusion config between /odom and
/odometry/filtered -- the topic mpc_corr reads as v_real. robot_localization
has no "forward only" switch, so what could hide a reverse here is config: vx
not fused at all, or a rejection threshold tight enough to throw out the step
change in vx when the car changes direction.
"""

import pathlib

import yaml

CONFIG = (pathlib.Path(__file__).resolve().parents[1] / 'config' / 'ekf.yaml')

# odomN_config / imuN_config order (robot_localization): the 15 booleans are
# x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az.
VX = 6


def _params():
    cfg = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    for node in cfg.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    raise AssertionError(f'no ros__parameters in {CONFIG}')


def test_the_wheel_odometry_vx_is_fused():
    """Without vx fused, /odometry/filtered's vx is an integration artefact."""
    params = _params()
    assert 'odom0' in params, 'the wheel odometry source is gone'
    assert params['odom0_config'][VX] is True, 'odom0 vx is not fused'


def test_no_rejection_threshold_can_discard_a_direction_change():
    """
    A tight Mahalanobis gate would reject the vx step at a gear change.

    Absent is the robot_localization default (numeric_limits<double>::max(),
    i.e. accept everything); a finite value here would need justifying against
    a reverse, so this fails rather than guessing what is safe.
    """
    params = _params()
    offenders = {k: v for k, v in params.items() if k.endswith('_rejection_threshold')}
    assert not offenders, (
        f'rejection thresholds present, check each against a reverse: {offenders}')


def test_nothing_in_the_config_names_a_forward_only_assumption():
    """two_d_mode zeroes z/roll/pitch only -- it must not be read as planar-forward."""
    params = _params()
    # two_d_mode is fine and expected; this only pins that it is the ONLY
    # dimension-reducing switch in force, so a future addition gets noticed.
    reducing = {k for k in params if k in ('two_d_mode', 'control_config', 'use_control')}
    assert reducing <= {'two_d_mode'}, (
        f'new motion assumptions to check against a reverse: {reducing}')
