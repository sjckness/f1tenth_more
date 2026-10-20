"""READY: threshold validation and the pure judge/evaluate logic."""

import os
import re

from f1tenth_ctl import ready
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(PKG)

RATE = {'name': 'scan', 'kind': 'topic_rate', 'topic': '/scan',
        'type': 'sensor_msgs/msg/LaserScan', 'min_hz': 20.0}
TF = {'name': 'tf', 'kind': 'tf_age', 'parent': 'map', 'child': 'base_link',
      'max_age_s': 0.5}
SVC = {'name': 'mission', 'kind': 'service', 'service': '/mission/load_mission',
       'type': 'f1tenth_messages/srv/LoadMission'}
ESTOP = {'name': 'estop_latched', 'kind': 'estop', 'topic': '/mission/status',
         'type': 'f1tenth_messages/msg/MissionStatus'}
BATT = {'name': 'battery', 'kind': 'battery', 'topic': '/diagnostics/battery_status',
        'type': 'f1tenth_messages/msg/BatteryStatus', 'min_voltage': 10.8}


def test_validate_rejects_unknown_kind_and_missing_fields():
    with pytest.raises(ready.ReadyConfigError, match='unknown kind'):
        ready.validate({'checks': [{'name': 'x', 'kind': 'nope'}]})
    with pytest.raises(ready.ReadyConfigError, match="needs 'topic'"):
        ready.validate({'checks': [{'name': 'x', 'kind': 'topic_rate', 'type': 't'}]})
    with pytest.raises(ready.ReadyConfigError, match='duplicate'):
        ready.validate({'checks': [RATE, RATE]})


def test_validate_fills_defaults():
    data = ready.validate({'checks': [RATE]})
    assert data['window_s'] == 2.0 and data['use_sim_time'] is False


def test_topic_rate_passes_at_threshold_and_fails_below():
    assert ready.judge(RATE, {'count': 40, 'hz': 20.0})['ok']
    low = ready.judge(RATE, {'count': 10, 'hz': 9.5})
    assert not low['ok'] and low['why'] == '9.5 Hz'
    none = ready.judge(RATE, {'count': 0, 'hz': 0.0})
    assert not none['ok'] and none['why'] == 'no messages'


def test_topic_rate_with_zero_minimum_is_always_ok():
    check = dict(RATE, min_hz=0)
    assert ready.judge(check, {'count': 0, 'hz': 0.0})['ok']


def test_tf_age_needs_a_fresh_transform():
    assert ready.judge(TF, {'found': True, 'age_s': 0.05})['ok']
    stale = ready.judge(TF, {'found': True, 'age_s': 2.0})
    assert not stale['ok'] and 'old' in stale['why']
    missing = ready.judge(TF, {'found': False, 'error': 'LookupException: "map" missing'})
    assert not missing['ok'] and 'LookupException' in missing['why']


def test_service_available_or_not():
    assert ready.judge(SVC, {'available': True})['ok']
    assert not ready.judge(SVC, {'available': False})['ok']


def test_estop_latched_or_unknown_blocks_ready():
    assert ready.judge(ESTOP, {'received': True, 'emergency_stop_active': False,
                               'state': 'IDLE'})['ok']
    latched = ready.judge(ESTOP, {'received': True, 'emergency_stop_active': True})
    assert not latched['ok'] and 'latched' in latched['why']
    assert not ready.judge(ESTOP, {'received': False})['ok']


def test_ready_is_false_while_the_estop_is_latched():
    thr = ready.validate({'checks': [RATE, TF, SVC, ESTOP, BATT]})
    m = _all_good()
    m['estop_latched'] = {'received': True, 'emergency_stop_active': True, 'state': 'IDLE'}
    res = ready.evaluate(thr, m, stack_running=True, discovery_ok=True)
    assert res['ready'] is False and res['failing'] == ['estop_latched']
    check = next(c for c in res['checks'] if c['name'] == 'estop_latched')
    assert check['why'] == 'e-stop latched' and check['value'] is True
    assert ready.estop_latched(res['checks']) is True


def test_ready_again_once_the_latch_is_clear():
    thr = ready.validate({'checks': [RATE, TF, SVC, ESTOP, BATT]})
    res = ready.evaluate(thr, _all_good(), stack_running=True, discovery_ok=True)
    assert res['ready'] is True and ready.estop_latched(res['checks']) is False


def test_unknown_latch_is_not_ready_and_not_reported_as_clear():
    thr = ready.validate({'checks': [RATE, TF, SVC, ESTOP, BATT]})
    m = _all_good()
    m['estop_latched'] = {'received': False}
    res = ready.evaluate(thr, m, stack_running=True, discovery_ok=True)
    assert res['failing'] == ['estop_latched']
    assert ready.estop_latched(res['checks']) is None
    assert 'unknown' in next(c for c in res['checks'] if c['name'] == 'estop_latched')['why']
    # Without the stack nothing is measured: latched is unknown, not False.
    res = ready.evaluate(thr, None, stack_running=False, discovery_ok=True)
    assert ready.estop_latched(res['checks']) is None


def test_battery_needs_data_and_voltage():
    assert ready.judge(BATT, {'received': True, 'has_data': True, 'voltage': 11.9})['ok']
    low = ready.judge(BATT, {'received': True, 'has_data': True, 'voltage': 10.5})
    assert not low['ok'] and low['value'] == 10.5
    nodata = ready.judge(BATT, {'received': True, 'has_data': False, 'voltage': 0.0})
    assert not nodata['ok'] and nodata['value'] is None


def test_measurement_error_fails_the_check():
    res = ready.judge(RATE, {'error': 'ValueError: bad type'})
    assert not res['ok'] and 'bad type' in res['why']


def _all_good():
    return {'scan': {'count': 60, 'hz': 40.0}, 'tf': {'found': True, 'age_s': 0.02},
            'mission': {'available': True},
            'estop_latched': {'received': True, 'emergency_stop_active': False},
            'battery': {'received': True, 'has_data': True, 'voltage': 12.1}}


def test_evaluate_ready_when_everything_passes():
    thr = ready.validate({'checks': [RATE, TF, SVC, ESTOP, BATT]})
    res = ready.evaluate(thr, _all_good(), stack_running=True, discovery_ok=True)
    assert res['ready'] and res['failing'] == []


def test_evaluate_lists_every_failing_check():
    thr = ready.validate({'checks': [RATE, TF, SVC, ESTOP, BATT]})
    m = _all_good()
    m['scan'] = {'count': 0, 'hz': 0.0}
    m['battery'] = {'received': True, 'has_data': True, 'voltage': 9.9}
    res = ready.evaluate(thr, m, stack_running=True, discovery_ok=True)
    assert not res['ready'] and res['failing'] == ['scan', 'battery']


def test_report_only_check_never_blocks_ready():
    thr = ready.validate({'checks': [RATE, dict(BATT, required=False)]})
    m = {'scan': {'count': 60, 'hz': 40.0}, 'battery': {'received': False}}
    res = ready.evaluate(thr, m, stack_running=True, discovery_ok=None)
    assert res['ready']
    batt = next(c for c in res['checks'] if c['name'] == 'battery')
    assert not batt['ok'] and not batt['required']


def test_stack_and_discovery_are_preconditions():
    thr = ready.validate({'checks': [RATE]})
    m = {'scan': {'count': 60, 'hz': 40.0}}
    assert ready.evaluate(thr, m, False, True)['failing'] == ['stack']
    assert ready.evaluate(thr, m, True, False)['failing'] == ['discovery']
    names = [c['name'] for c in ready.evaluate(thr, m, True, None)['checks']]
    assert 'discovery' not in names


def test_probe_not_run_fails_every_required_check():
    thr = ready.validate({'checks': [RATE, dict(BATT, required=False)]})
    res = ready.evaluate(thr, None, stack_running=True, discovery_ok=True)
    assert res['failing'] == ['scan']


def test_key_values_skip_process_checks():
    thr = ready.validate({'checks': [RATE]})
    res = ready.evaluate(thr, {'scan': {'count': 60, 'hz': 39.96}}, True, True)
    assert ready.key_values(res['checks']) == {'scan': 40.0}


@pytest.mark.parametrize('name', ['ready.yaml', 'ready_sim.yaml'])
def test_shipped_threshold_files_are_valid(name):
    data = ready.load_thresholds(os.path.join(PKG, 'config', name))
    names = {c['name'] for c in data['checks']}
    assert {'odometry', 'scan', 'tf_map_base_link', 'mission_service',
            'estop_latched', 'battery'} <= names
    # status must answer in < 5 s, env sourcing and rclpy start-up included.
    assert data['window_s'] <= 2.0


def _stack_default(key):
    path = os.path.join(SRC, 'f1tenth_params', 'config', 'stack_params.yaml')
    if not os.path.isfile(path):
        pytest.skip('stack_params.yaml not next to this package')
    with open(path) as f:
        return yaml.safe_load(f)[key]['default']


def test_car_battery_threshold_is_the_stacks_min_battery_voltage():
    data = ready.load_thresholds(os.path.join(PKG, 'config', 'ready.yaml'))
    batt = next(c for c in data['checks'] if c['kind'] == 'battery')
    assert batt['min_voltage'] == _stack_default('min_battery_voltage')


def test_car_odometry_minimum_is_below_the_ekf_rate():
    data = ready.load_thresholds(os.path.join(PKG, 'config', 'ready.yaml'))
    odo = next(c for c in data['checks'] if c['name'] == 'odometry')
    assert odo['min_hz'] < _stack_default('ekf_frequency_hz')


def test_estop_topic_is_the_mux_highest_priority_lane():
    mux = os.path.join(SRC, 'f1tenth_bringup', 'config', 'mux.yaml')
    if not os.path.isfile(mux):
        pytest.skip('mux.yaml not next to this package')
    with open(mux) as f:
        lanes = yaml.safe_load(f)['ackermann_mux']['ros__parameters']['topics']
    top = max(lanes.values(), key=lambda lane: lane['priority'])
    for env in ('ctl.env', 'sim_linus.env'):
        with open(os.path.join(PKG, 'config', env)) as f:
            text = f.read()
        topic = re.search(r'^F1T_ESTOP_TOPIC=(\S+)', text, re.M).group(1)
        hz = float(re.search(r'^F1T_ESTOP_PUBLISH_HZ=(\S+)', text, re.M).group(1))
        assert topic == '/' + top['topic'].lstrip('/')
        # The lane drops out `timeout` seconds after its last message.
        assert 1.0 / hz < top['timeout']
