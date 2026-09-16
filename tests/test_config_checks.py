"""C5: startup validation refuses unsafe configurations."""

import pytest

from go_to_object.config_checks import (
    ConfigError,
    VehicleLimits,
    describe,
    raise_on_errors,
    validate,
)
from go_to_object.mission_state import MissionParams
from go_to_object.object_tracker import TrackerParams
from go_to_object.pursuit_geometry import PursuitParams


def _validate(**kwargs):
    return validate(kwargs.pop('pursuit', PursuitParams()),
                    kwargs.pop('tracker', TrackerParams()),
                    kwargs.pop('mission', MissionParams()),
                    kwargs.pop('max_kappa_rate', 0.5),
                    kwargs.pop('limits', VehicleLimits()))


def _errors(findings):
    return [f for f in findings if f.severity == 'error']


def test_the_shipped_defaults_raise_no_errors():
    """If this ever fails, the defaults ship a config the node refuses."""
    assert _errors(_validate()) == []


def test_a_curvature_rate_beyond_the_servo_is_refused():
    limits = VehicleLimits(wheelbase=0.33, steering_slew_rate=6.98)
    assert limits.max_kappa_rate() == pytest.approx(21.15, rel=1e-3)

    findings = _validate(max_kappa_rate=30.0, limits=limits)
    errors = _errors(findings)

    assert len(errors) == 1 and errors[0].check == 'max_kappa_rate'
    with pytest.raises(ConfigError):
        raise_on_errors(findings)


def test_a_rate_just_inside_the_servo_is_accepted():
    limits = VehicleLimits(wheelbase=0.33, steering_slew_rate=6.98)
    assert _errors(_validate(max_kappa_rate=limits.max_kappa_rate() * 0.99,
                             limits=limits)) == []


def test_an_r_min_tighter_than_the_vehicle_can_turn_is_refused():
    findings = _validate(pursuit=PursuitParams(r_min=0.4),
                         limits=VehicleLimits(min_turn_radius=1.2))
    errors = _errors(findings)

    assert [e.check for e in errors] == ['r_min']
    with pytest.raises(ConfigError):
        raise_on_errors(findings)


def test_a_state_timeout_inside_max_age_is_refused():
    """Otherwise the watchdog pre-empts every ordinary LOST ramp."""
    findings = _validate(tracker=TrackerParams(max_age=1.5),
                         mission=MissionParams(state_timeout=1.0))

    assert [e.check for e in _errors(findings)] == ['state_timeout']


@pytest.mark.parametrize('check', ['max_age', 'd_stop'])
def test_the_speed_dependent_checks_warn_rather_than_refuse(check):
    """Reported with the arithmetic; not fatal, because this node has no speed.

    Both fire on the shipped defaults. ``max_age`` 0.8 s at 2 m/s is 1.6 m of
    blind travel against a 0.5 m ``d_stop``, and ``d_stop`` 0.5 m sits inside
    ``d_lookahead_min`` 1.0 m. Making either fatal would refuse the shipped
    configuration, and neither is knowable without an operating speed that
    this node does not command.
    """
    findings = _validate()
    warnings = [f for f in findings if f.severity == 'warning']

    assert check in [w.check for w in warnings]
    assert _errors(findings) == []


def test_the_blind_travel_warning_states_the_arithmetic():
    findings = _validate(limits=VehicleLimits(cruise_speed=2.0))
    message = next(f.message for f in findings if f.check == 'max_age')

    assert '1.60 m' in message, 'the computed blind travel'
    assert '0.25 s' in message, 'and the max_age that would bound it'


def test_a_slow_enough_vehicle_clears_the_blind_travel_warning():
    findings = _validate(limits=VehicleLimits(cruise_speed=0.5))

    assert 'max_age' not in [f.check for f in findings]


def test_an_alarm_inside_the_sampling_noise_is_flagged():
    """C6: an alarm a few sigma from expectation fires on noise, not faults."""
    findings = _validate(tracker=TrackerParams(nis_window=4, nis_alarm_ratio=1.1))

    assert 'nis_alarm_ratio' in [f.check for f in findings]
    assert 'nis_alarm_ratio' not in [f.check for f in _validate()]


def test_describe_lists_every_loaded_parameter():
    lines = describe(PursuitParams(), TrackerParams(), MissionParams(), 0.5,
                     VehicleLimits())
    joined = '\n'.join(lines)

    for expected in ('max_kappa_rate=0.5', 'pursuit.r_min=1.5',
                     'tracker.max_age=0.8', 'mission.odom_timeout=0.2',
                     'vehicle.wheelbase=0.33'):
        assert expected in joined
