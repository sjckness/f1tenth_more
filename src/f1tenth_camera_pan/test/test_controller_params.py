"""camera_pan_controller_node's dynamic parameters.

mode, fixed_angle_rad and track_when_stopped must change the aim at runtime
(`ros2 param set`), and bad values must be rejected without changing anything.
The validator is tested pure; the node is a real (never spun) rclpy Node whose
parameters are set the way the parameter service sets them, then _target() is
read directly. Needs f1tenth_params installed (the aim-law tuning comes from
stack_params.yaml).
"""
import math

from f1tenth_camera_pan.aim_law import aim_pan
from f1tenth_camera_pan.camera_pan_controller_node import (
    CameraPanController, validate_dynamic_params)
import pytest
import rclpy
from rclpy.parameter import Parameter

MAX = 0.2618


# -- pure validator -----------------------------------------------------------

@pytest.mark.parametrize('values', [
    {'mode': 'track_heading'}, {'mode': 'fixed'}, {'mode': 'scan'},
    {'fixed_angle_rad': 0.0}, {'fixed_angle_rad': -MAX}, {'fixed_angle_rad': 0.1},
    {'track_when_stopped': True}, {'track_when_stopped': False},
    {'mode': 'fixed', 'fixed_angle_rad': 0.2, 'track_when_stopped': True},
    {},
])
def test_validator_accepts_good_values(values):
    assert validate_dynamic_params(values, MAX) is None


@pytest.mark.parametrize('values, word', [
    ({'mode': 'track'}, 'mode'),
    ({'mode': ''}, 'mode'),
    ({'mode': 1}, 'mode'),
    ({'fixed_angle_rad': MAX + 0.01}, 'within'),
    ({'fixed_angle_rad': -1.0}, 'within'),
    ({'fixed_angle_rad': math.nan}, 'within'),
    ({'fixed_angle_rad': math.inf}, 'within'),
    ({'fixed_angle_rad': 'left'}, 'number'),
    ({'fixed_angle_rad': True}, 'number'),
    ({'track_when_stopped': 'true'}, 'bool'),
    ({'track_when_stopped': 1}, 'bool'),
    ({'mode': 'fixed', 'fixed_angle_rad': 9.0}, 'within'),
])
def test_validator_rejects_bad_values_with_a_reason(values, word):
    reason = validate_dynamic_params(values, MAX)
    assert reason is not None and word in reason


# -- live node ----------------------------------------------------------------

@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def node():
    n = CameraPanController()
    yield n
    n.destroy_node()


def _set(node, **kv):
    """Set parameters the way the /set_parameters service does; return the results."""
    return node.set_parameters([Parameter(k, value=v) for k, v in kv.items()])


def _stopped_and_steering(node, delta=0.3):
    node._v = 0.0
    node._omega = 0.0
    node._delta = delta


def test_defaults(node):
    assert node.mode == 'track_heading'
    assert node.fixed_angle == 0.0
    assert node.params.track_when_stopped is False


def test_track_when_stopped_toggles_at_runtime(node):
    _stopped_and_steering(node)
    assert node._target() == 0.0          # stopped, default: aim straight ahead

    [r] = _set(node, track_when_stopped=True)
    assert r.successful
    assert node.params.track_when_stopped is True
    aim = node._target()
    assert aim > 0.0                      # steering left -> camera left
    assert aim == pytest.approx(aim_pan(0.0, 0.0, 0.3, node.params))
    # The rest of the aim-law tuning is untouched by the replace().
    assert node.params.max_pan_rad == pytest.approx(MAX, abs=1e-3)

    [r] = _set(node, track_when_stopped=False)
    assert r.successful
    assert node._target() == 0.0


def test_mode_switch_to_fixed_and_back(node):
    _stopped_and_steering(node)
    assert all(r.successful for r in _set(node, mode='fixed', fixed_angle_rad=-0.2))
    assert node.mode == 'fixed'
    assert node._target() == pytest.approx(-0.2)

    [r] = _set(node, fixed_angle_rad=0.15)
    assert r.successful
    assert node._target() == pytest.approx(0.15)

    [r] = _set(node, mode='track_heading')
    assert r.successful
    assert node._target() == 0.0          # stopped, track_when_stopped off


def test_scan_falls_back_to_zero(node):
    [r] = _set(node, mode='scan')
    assert r.successful
    assert node._target() == 0.0


@pytest.mark.parametrize('kv', [
    {'mode': 'tracking'},
    {'fixed_angle_rad': 1.0},
    {'fixed_angle_rad': math.nan},
])
def test_bad_value_is_rejected_and_changes_nothing(node, kv):
    before = (node.mode, node.fixed_angle, node.params)
    [r] = _set(node, **kv)
    assert not r.successful
    assert r.reason
    assert (node.mode, node.fixed_angle, node.params) == before
    # The stored parameter is unchanged too (what `ros2 param get` reports).
    assert node.get_parameter('mode').value == 'track_heading'
    assert node.get_parameter('fixed_angle_rad').value == 0.0


def test_atomic_set_with_one_bad_value_applies_none(node):
    r = node.set_parameters_atomically([
        Parameter('mode', value='fixed'), Parameter('fixed_angle_rad', value=5.0)])
    assert not r.successful
    assert node.mode == 'track_heading'
    assert node.get_parameter('mode').value == 'track_heading'


def test_wrong_type_is_rejected(node):
    # Declared as bool / double: `ros2 param set ... track_when_stopped 1` must not
    # sneak an int in.
    [r] = _set(node, track_when_stopped=1)
    assert not r.successful
    assert node.params.track_when_stopped is False


@pytest.mark.parametrize('name, value', [
    ('pivot_x_m', 0.5), ('pivot_y_m', 0.1), ('publish_rate_hz', 10.0)])
def test_launch_only_params_are_read_only(node, name, value):
    [r] = _set(node, **{name: value})
    assert not r.successful


@pytest.mark.parametrize('override, word', [
    (Parameter('mode', value='bogus'), 'mode must be one of'),
    (Parameter('fixed_angle_rad', value=1.0), 'fixed_angle_rad must be within'),
])
def test_invalid_launch_value_fails_at_startup(override, word):
    with pytest.raises(ValueError, match=word):
        CameraPanController(parameter_overrides=[override])


def test_launch_values_are_used():
    n = CameraPanController(parameter_overrides=[
        Parameter('mode', value='fixed'), Parameter('fixed_angle_rad', value=0.1),
        Parameter('track_when_stopped', value=True)])
    try:
        assert n.mode == 'fixed'
        assert n._target() == pytest.approx(0.1)
        assert n.params.track_when_stopped is True
    finally:
        n.destroy_node()
