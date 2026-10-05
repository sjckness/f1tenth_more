"""Unit tests for f1tenth_bringup/topic_watchdog.py (fix batch 5, H1).

The topic-liveness watchdog's pure logic. No rclpy: times are plain floats.
"""

import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from topic_watchdog import (  # noqa: E402
    check_verdict, ComponentMonitor, ERROR, HealthConfigError, needs_message, ONCE_SETTLE_SEC,
    parse_health_config, topic_states, TopicState, WARN)

REGISTRY = {
    'swept_clearance': [{'launch_file': 'swept_clearance.launch.py'}],
    'slam': [{'launch_file': 'slam.launch.py'}, {'launch_file': 'costmap.launch.py'}],
    'dev_tools': [{'launch_file': 'foxglove_bridge.launch.py'}],
}
TYPES = {'/scan': 'sensor_msgs/msg/LaserScan',
         '/out': 'std_msgs/msg/Float32',
         '/map': 'nav_msgs/msg/OccupancyGrid',
         '/front': 'std_msgs/msg/Float32',
         '/status': 'diagnostic_msgs/msg/DiagnosticStatus'}


def doc(health, unwatched=None):
    return {'health_topic_types': TYPES, 'health': health, 'unwatched': unwatched or {}}


def swept(**kw):
    cfg = {'grace_sec': 10, 'fail_for_sec': 3, 'healthy_for_sec': 5,
           'checks': [{'topic': '/out', 'max_age_sec': 0.5, 'when_fresh': {'/scan': 0.5}}]}
    cfg.update(kw)
    health, _, _ = parse_health_config(doc({'swept_clearance': cfg}), REGISTRY)
    return health['swept_clearance']


def topics(*names):
    # Every when_fresh max age these tests use.
    return {n: TopicState((0.5, 15)) for n in names}


def feed(ts, now, *names):
    for n in names:
        ts[n].on_message(now)


RUNNING = {'swept_clearance.launch.py': (True, 0.0)}


# -- config parsing --------------------------------------------------------------

def test_single_launch_file_is_the_default_owner():
    assert swept().checks[0].launch_file == 'swept_clearance.launch.py'


def test_multi_launch_component_must_name_the_owner():
    with pytest.raises(HealthConfigError, match='launch_file is required'):
        parse_health_config(doc({'slam': {'checks': [{'topic': '/map', 'max_age_sec': 15}]}}),
                            REGISTRY)


def test_owner_must_belong_to_the_component():
    bad = {'checks': [{'topic': '/map', 'max_age_sec': 15, 'launch_file': 'nope.launch.py'}]}
    with pytest.raises(HealthConfigError, match='not one of'):
        parse_health_config(doc({'slam': bad}), REGISTRY)


def test_every_topic_needs_a_type():
    bad = {'checks': [{'topic': '/out', 'max_age_sec': 1, 'when_fresh': {'/untyped': 1}}]}
    with pytest.raises(HealthConfigError, match='/untyped has no entry'):
        parse_health_config(doc({'swept_clearance': bad}), REGISTRY)


def test_unknown_key_and_bad_action_are_rejected():
    with pytest.raises(HealthConfigError, match='unknown keys'):
        swept(grace=3)
    with pytest.raises(HealthConfigError, match='action'):
        swept(action='reboot')


def test_component_both_watched_and_unwatched_is_rejected():
    with pytest.raises(HealthConfigError, match='both'):
        parse_health_config(
            doc({'swept_clearance': {'checks': [{'topic': '/out', 'max_age_sec': 1}]}},
                {'swept_clearance': 'x'}), REGISTRY)


def test_needs_message_only_for_expect_checks():
    health, _, _ = parse_health_config(doc({
        'swept_clearance': {'checks': [{'topic': '/out', 'max_age_sec': 1}]},
        'slam': {'checks': [
            {'topic': '/map', 'max_age_sec': 15, 'launch_file': 'slam.launch.py',
             'min_rate_hz': 1},
            {'topic': '/status', 'max_age_sec': 1, 'launch_file': 'costmap.launch.py',
             'expect': {'level': [0]}}]},
    }), REGISTRY)
    assert not needs_message('/out', health)
    assert not needs_message('/map', health)     # a rate needs receipt times only
    assert needs_message('/status', health)
    assert not needs_message('/scan', health)


# -- one check ---------------------------------------------------------------------

def test_never_received_is_infinitely_old():
    assert TopicState().age(5.0) == math.inf


def test_fresh_output_is_ok_and_stale_output_with_fresh_input_fails():
    c = swept().checks[0]
    ts = topics('/scan', '/out')
    feed(ts, 10.0, '/scan', '/out')
    assert check_verdict(c, ts, 10.3)[0] == 'ok'
    for t in (10.2, 10.4, 10.6, 10.8):
        feed(ts, t, '/scan')
    v, why = check_verdict(c, ts, 11.0)
    assert v == 'fail' and 'while its input is fresh' in why


def test_stale_input_skips_the_check():
    """An upstream outage (no /scan) is not this component's fault."""
    c = swept().checks[0]
    ts = topics('/scan', '/out')
    v, why = check_verdict(c, ts, 100.0)
    assert v == 'skip' and '/scan' in why


def _ekf(min_rate=30):
    health, _, _ = parse_health_config(doc({'swept_clearance': {'checks': [
        {'topic': '/out', 'max_age_sec': 0.5, 'min_rate_hz': min_rate,
         'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    return health['swept_clearance'], topic_states(health)


def _publish(ts, topic, t0, t1, hz):
    n = int(round((t1 - t0) * hz))
    for i in range(n):
        ts[topic].on_message(t0 + i / hz)


def test_min_rate_catches_an_ekf_predicting_without_input():
    """Tell a starved EKF apart by its rate alone.

    Measured: robot_localization keeps publishing after /odom stops, 48 Hz
    fed, about 9 Hz starved, with a new header.stamp on every message.
    """
    cfg, ts = _ekf()
    c = cfg.checks[0]
    _publish(ts, '/scan', 0.0, 10.0, 50)      # the input, as the supervisor gets it
    _publish(ts, '/out', 0.0, 10.0, 48)       # fed
    assert check_verdict(c, ts, 9.99, started_at=0.0)[0] == 'ok'
    _publish(ts, '/scan', 10.0, 20.0, 50)
    _publish(ts, '/out', 10.0, 20.0, 9)       # starved: its subscription lost /odom
    v, why = check_verdict(c, ts, 19.99, started_at=0.0)
    assert v == 'fail' and '9.0 Hz' in why and 'below 30 Hz' in why


def test_min_rate_is_scaled_by_the_real_time_factor():
    """Scale the threshold in sim: at half real time a fed EKF gives 24 Hz."""
    cfg, ts = _ekf()
    _publish(ts, '/scan', 0.0, 10.0, 25)
    _publish(ts, '/out', 0.0, 10.0, 24)
    assert check_verdict(cfg.checks[0], ts, 9.9, started_at=0.0)[0] == 'fail'
    assert check_verdict(cfg.checks[0], ts, 9.9, started_at=0.0, rate_scale=0.5)[0] == 'ok'


def test_min_rate_waits_a_full_window_after_start_and_after_input_returns():
    cfg, ts = _ekf()
    c = cfg.checks[0]
    _publish(ts, '/scan', 0.0, 1.0, 50)
    _publish(ts, '/out', 0.0, 1.0, 48)
    assert check_verdict(c, ts, 0.99, started_at=0.0)[0] == 'ok'      # half a window
    # Input outage 1-10 s, then back: the output's rate is judged a window later.
    _publish(ts, '/scan', 10.0, 13.0, 50)
    _publish(ts, '/out', 10.3, 13.0, 48)
    assert check_verdict(c, ts, 11.5, started_at=0.0)[0] == 'ok'
    assert check_verdict(c, ts, 12.99, started_at=0.0)[0] == 'ok'     # full window: 48 Hz


def test_expect_reads_the_last_message_and_byte_fields():
    health, _, _ = parse_health_config(doc({'swept_clearance': {'checks': [
        {'topic': '/status', 'max_age_sec': 1, 'expect': {'level': [0]}}]}}), REGISTRY)
    c = health['swept_clearance'].checks[0]
    ts = topics('/status')
    ts['/status'].on_message(1.0, msg=SimpleNamespace(level=b'\x00'))
    assert check_verdict(c, ts, 1.1)[0] == 'ok'
    ts['/status'].on_message(1.2, msg=SimpleNamespace(level=b'\x02'))
    v, why = check_verdict(c, ts, 1.3)
    assert v == 'fail' and 'level = 2' in why


# -- the monitor -----------------------------------------------------------------------

def run(mon, ts, procs, t0, t1, dt=0.5, feeds=('/scan', '/out'), **kw):
    """Evaluate every dt from t0 to t1, feeding `feeds` each step.

    Returns [(time, launch_files_to_restart)] for every non-empty result. Stops
    at the first restart: a real restart gives the launch file a new start
    time, which the caller passes in its next run().
    """
    out = []
    t = t0
    while t <= t1 + 1e-9:
        feed(ts, t, *feeds)
        r = mon.evaluate(t, ts, procs, **kw)
        if r:
            out.append((t, r))
            break
        t += dt
    return out


def test_no_trigger_during_grace_even_with_nothing_received():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    assert run(mon, ts, RUNNING, 0.0, 9.5, feeds=('/scan',)) == []
    assert mon.status == 'STARTING'


def test_healthy_component_never_triggers():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    assert run(mon, ts, RUNNING, 0.0, 600.0) == []
    assert mon.status == 'OK' and mon.failures == 0


def test_silent_output_with_fresh_input_restarts_after_fail_for():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    # /out stops at 20.0; age passes 0.5 s at 20.5 -> failing; restart 3 s later.
    res = run(mon, ts, RUNNING, 20.5, 30.0, feeds=('/scan',))
    assert res[0] == (24.0, ['swept_clearance.launch.py'])
    assert mon.status == 'RESTARTING' and mon.restarts == 1
    assert any('LIVENESS FAILURE' in e for _, e in mon.events)


def test_stale_input_never_triggers():
    """Both input and output stop: an upstream outage, not isolation."""
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    assert run(mon, ts, RUNNING, 20.5, 120.0, feeds=()) == []
    assert mon.status == 'UPSTREAM_STALE'


def test_short_dropout_below_fail_for_does_not_trigger():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    assert run(mon, ts, RUNNING, 20.5, 22.5, feeds=('/scan',)) == []   # 2.5 s < 3 s
    assert mon.status == 'FAILING'
    assert run(mon, ts, RUNNING, 23.0, 40.0) == []
    assert mon.status == 'OK'


def test_not_running_launch_file_is_not_judged():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    procs = {'swept_clearance.launch.py': (False, None)}
    assert run(mon, ts, procs, 0.0, 60.0, feeds=('/scan',)) == []
    assert mon.status == 'NOT_RUNNING'


def test_alert_action_reports_once_and_never_restarts():
    mon = ComponentMonitor(swept(action='alert'))
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    assert run(mon, ts, RUNNING, 20.5, 80.0, feeds=('/scan',)) == []
    assert mon.status == 'ALERT' and mon.failures == 1
    assert sum('alert only' in e for _, e in mon.events) == 1


def test_gives_up_after_max_consecutive_restarts_and_stays_failed():
    """A component that fails after every restart: 3 restarts, then FAILED."""
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    t, restarts = 0.0, []
    for _ in range(6):
        started = t
        procs = {'swept_clearance.launch.py': (True, started)}
        res = run(mon, ts, procs, t, t + 30.0, feeds=('/scan',))
        restarts += res
        t = res[0][0] + 0.5 if res else t + 30.5
    assert len(restarts) == 3
    assert mon.status == 'FAILED' and mon.gave_up
    # Stays FAILED, even if the output comes back, until a manual reset.
    run(mon, ts, RUNNING, t, t + 60.0)
    assert mon.status == 'FAILED'
    mon.reset()
    run(mon, ts, RUNNING, t + 61, t + 70.0)
    assert mon.status == 'OK'


def test_healthy_period_between_failures_resets_the_consecutive_count():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    t = 0.0
    for _ in range(5):   # more than max_consecutive_restarts isolated episodes
        procs = {'swept_clearance.launch.py': (True, t)}
        run(mon, ts, procs, t, t + 20.0)                          # grace, then healthy
        res = run(mon, ts, procs, t + 20.5, t + 30.0, feeds=('/scan',))
        assert len(res) == 1
        t = res[0][0] + 0.5
    assert not mon.gave_up and mon.restarts == 5


def test_restart_refused_by_budget_marks_failed():
    mon = ComponentMonitor(swept())
    mon.restart_refused('restart budget exhausted')
    assert mon.status == 'FAILED' and mon.gave_up


def test_paused_clock_never_triggers_and_resume_gets_a_grace():
    """sim: /clock stops for 30 s, every sim-time node goes silent."""
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    assert run(mon, ts, RUNNING, 20.5, 50.0, feeds=(), paused=True) == []
    assert mon.status == 'PAUSED'
    mon.resume(50.5, 5.0)
    # The inputs need a moment to flow again: nothing at first, then /scan only.
    assert run(mon, ts, RUNNING, 50.5, 52.0, feeds=()) == []
    assert run(mon, ts, RUNNING, 52.5, 55.0, feeds=('/scan',)) == []
    assert run(mon, ts, RUNNING, 55.5, 70.0) == []
    assert mon.status == 'OK'


def test_without_pause_flag_a_silent_node_is_still_caught():
    """Judge the same silence normally when paused is never passed.

    On the car there is no /clock: the gate must not switch the watchdog off.
    """
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    assert run(mon, ts, RUNNING, 20.5, 30.0, feeds=('/scan',))


def test_supervisor_stall_clears_failure_timers():
    mon = ComponentMonitor(swept())
    ts = topics('/scan', '/out')
    run(mon, ts, RUNNING, 0.0, 20.0)
    feed(ts, 20.0, '/scan')
    # The supervisor was blocked 10 s: every age looks old on its first tick back.
    assert mon.evaluate(30.0, ts, RUNNING, stalled=True) == []
    run(mon, ts, RUNNING, 30.5, 40.0)
    assert mon.status == 'OK' and mon.failures == 0


def test_multi_launch_component_restarts_only_the_failing_launch_file():
    health, _, _ = parse_health_config(doc({'slam': {
        'grace_sec': 5, 'fail_for_sec': 2, 'checks': [
            {'topic': '/map', 'max_age_sec': 15, 'launch_file': 'slam.launch.py',
             'when_fresh': {'/scan': 0.5}},
            {'topic': '/front', 'max_age_sec': 1, 'launch_file': 'costmap.launch.py',
             'when_fresh': {'/map': 15}}]}}), REGISTRY)
    mon = ComponentMonitor(health['slam'])
    ts = topics('/scan', '/map', '/front')
    procs = {'slam.launch.py': (True, 0.0), 'costmap.launch.py': (True, 0.0)}
    run(mon, ts, procs, 0.0, 10.0, feeds=('/scan', '/map', '/front'))
    res = run(mon, ts, procs, 10.5, 20.0, feeds=('/scan', '/map'))
    assert res[0][1] == ['costmap.launch.py']


def test_partly_judged_component_is_ok_and_fully_unjudged_is_upstream_stale():
    """No VESC in sim: the /sensors/core-gated check is noted, not a warning."""
    health, _, _ = parse_health_config(doc({'swept_clearance': {'grace_sec': 0, 'checks': [
        {'topic': '/out', 'max_age_sec': 1},
        {'topic': '/front', 'max_age_sec': 1, 'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    mon = ComponentMonitor(health['swept_clearance'])
    ts = topics('/scan', '/out', '/front')
    run(mon, ts, RUNNING, 1.0, 5.0, feeds=('/out',))
    assert mon.status == 'OK' and 'not judged' in mon.message
    health, _, _ = parse_health_config(doc({'swept_clearance': {'grace_sec': 0, 'checks': [
        {'topic': '/front', 'max_age_sec': 1, 'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    mon = ComponentMonitor(health['swept_clearance'])
    run(mon, ts, RUNNING, 1.0, 5.0, feeds=('/out',))
    assert mon.status == 'UPSTREAM_STALE'


def test_output_gets_its_max_age_after_an_input_outage():
    """Smoke-run regression: /slam/map is published every 5 s, max age 15 s.

    After a 40 s /scan outage the map was 18 s old the moment /scan came back,
    and the check restarted slam 3 s later. The silence during the outage is
    not slam's: the output gets max_age_sec from the input's return.
    """
    health, _, _ = parse_health_config(doc({'slam': {'grace_sec': 0, 'fail_for_sec': 3, 'checks': [
        {'topic': '/map', 'max_age_sec': 15, 'launch_file': 'slam.launch.py',
         'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    mon = ComponentMonitor(health['slam'])
    ts = topics('/scan', '/map')
    procs = {'slam.launch.py': (True, 0.0)}
    run(mon, ts, procs, 1.0, 20.0, feeds=('/scan', '/map'))
    run(mon, ts, procs, 20.5, 60.0, feeds=())                # outage: both stop
    # /scan back, the map only on its next 5 s update.
    assert run(mon, ts, procs, 60.5, 64.5, feeds=('/scan',)) == []
    assert run(mon, ts, procs, 65.0, 120.0, feeds=('/scan', '/map')) == []
    # A map that does NOT come back within max age of /scan's return still fails.
    run(mon, ts, procs, 120.5, 160.0, feeds=())
    res = run(mon, ts, procs, 160.5, 200.0, feeds=('/scan',))
    assert res and 175.0 <= res[0][0] <= 179.0


def test_fresh_since_ignores_gaps_shorter_than_the_max_age():
    st = TopicState((0.5, 2.0))
    assert st.fresh_since(0.5) is None
    for t in (1.0, 1.1, 1.2, 1.6, 1.7, 3.0, 3.1):
        st.on_message(t)
    assert st.fresh_since(0.5) == 3.0      # the 1.3 s gap ended at 3.0
    assert st.fresh_since(2.0) == 1.0      # no gap longer than 2 s


def test_fresh_since_survives_many_ticks():
    """Smoke-run regression, take two: a bounded gap history lost the outage."""
    st = TopicState((0.5,))
    st.on_message(0.0)
    st.on_message(40.0)
    for i in range(1, 2000):
        st.on_message(40.0 + 0.025 * i)
    assert st.fresh_since(0.5) == 40.0


def test_topic_states_registers_every_when_fresh_threshold():
    health, _, _ = parse_health_config(doc({'slam': {'checks': [
        {'topic': '/map', 'max_age_sec': 15, 'launch_file': 'slam.launch.py',
         'when_fresh': {'/scan': 0.5}},
        {'topic': '/front', 'max_age_sec': 1, 'launch_file': 'costmap.launch.py',
         'when_fresh': {'/map': 15, '/scan': 1.0}}]}}), REGISTRY)
    ts = topic_states(health)
    assert sorted(ts) == ['/front', '/map', '/scan']
    ts['/scan'].on_message(1.0)
    assert ts['/scan'].fresh_since(0.5) == ts['/scan'].fresh_since(1.0) == 1.0
    ts['/map'].on_message(2.0)
    assert ts['/map'].fresh_since(15) == 2.0


def test_once_check_needs_a_message_since_the_launch_file_started():
    """Judge /slam/map by one message since (re)start, not by its age.

    slam_toolbox rebuilds the whole grid before every publish, so its interval
    grows with the session (18.5 s seen in a 3 min run). "Published at least
    once since (re)start" is reliable, and is exactly the never-activated
    failure of fix batch 4.
    """
    health, _, _ = parse_health_config(doc({'slam': {'grace_sec': 5, 'fail_for_sec': 3, 'checks': [
        {'topic': '/map', 'once': True, 'launch_file': 'slam.launch.py',
         'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    mon = ComponentMonitor(health['slam'])
    ts = topics('/scan', '/map')
    procs = {'slam.launch.py': (True, 0.0)}
    ts['/map'].on_message(4.0)
    # One map, then silence for minutes: fine.
    assert run(mon, ts, procs, 0.0, 300.0, feeds=('/scan',)) == []
    assert mon.status == 'OK'
    # Restarted at 300: the old map does not count for the new process.
    procs = {'slam.launch.py': (True, 300.0)}
    res = run(mon, ts, procs, 300.5, 320.0, feeds=('/scan',))
    assert res and res[0][0] == 308.0


def test_once_needs_no_max_age_but_age_checks_do():
    with pytest.raises(HealthConfigError, match='max_age_sec is required'):
        parse_health_config(doc({'swept_clearance': {'checks': [{'topic': '/out'}]}}), REGISTRY)


def test_infinite_when_fresh_means_received_at_least_once():
    health, _, _ = parse_health_config(doc({'swept_clearance': {'checks': [
        {'topic': '/out', 'max_age_sec': 0.5, 'when_fresh': {'/map': math.inf}}]}}), REGISTRY)
    c = health['swept_clearance'].checks[0]
    ts = {t: TopicState((math.inf,)) for t in ('/map', '/out')}
    assert check_verdict(c, ts, 10.0)[0] == 'skip'     # no map ever: not judged
    ts['/map'].on_message(10.0)
    assert check_verdict(c, ts, 10.3)[0] == 'ok'       # judged from the map's arrival
    assert check_verdict(c, ts, 500.0)[0] == 'fail'    # map long ago is still "received"


def test_expect_waits_max_age_after_its_input_returns():
    """Give the node max_age_sec to see an input that just came back.

    Live: /odometry/filtered reached the supervisor a moment before mpc_corr's
    own staleness test saw it, and its level was ERROR for 0.5 s.
    """
    health, _, _ = parse_health_config(doc({'swept_clearance': {'checks': [
        {'topic': '/status', 'max_age_sec': 1.5, 'expect': {'level': [0]},
         'when_fresh': {'/scan': 0.5}}]}}), REGISTRY)
    c = health['swept_clearance'].checks[0]
    ts = topics('/scan', '/status')
    for i in range(30):
        t = 10.0 + 0.1 * i
        ts['/scan'].on_message(t)
        ts['/status'].on_message(t, msg=SimpleNamespace(level=b'\x02'))
    assert check_verdict(c, ts, 11.0)[0] == 'ok'      # 1.0 s after /scan returned
    assert check_verdict(c, ts, 12.9)[0] == 'fail'    # 2.9 s: mpc_corr really has none


# -- `once` settle time (backlog M19, report B6) ---------------------------------------
#
# Live (discload/after_watchdog/run_05): slam ACTIVE 3.6 s after bringup, /scan
# first fed 60 s later, i.e. after slam's 30 s grace; 6 s after the feed the
# once check restarted a healthy slam -- its first map needs a scan plus one
# map_update_interval. A once check now waits settle_sec after its inputs
# (and its launch file) are there before it can fail.

SLAM_START = {'slam.launch.py': (True, 0.0)}


def _slam(**check_kw):
    check = {'topic': '/map', 'once': True, 'launch_file': 'slam.launch.py',
             'when_fresh': {'/scan': 0.5}}
    check.update(check_kw)
    health, _, _ = parse_health_config(doc({'slam': {
        'grace_sec': 30, 'fail_for_sec': 3, 'checks': [check]}}), REGISTRY)
    return ComponentMonitor(health['slam'])


def _slam_run(mon, ts, t0, t1, scan_from=None, map_at=None, procs=SLAM_START, **kw):
    """Evaluate every 0.5 s; /scan from scan_from on, one /map at map_at."""
    out = []
    t = t0
    while t <= t1 + 1e-9:
        if scan_from is not None and t >= scan_from:
            ts['/scan'].on_message(t)
        if map_at is not None and abs(t - map_at) < 1e-9:
            ts['/map'].on_message(t)
        r = mon.evaluate(t, ts, procs, **kw)
        if r:
            out.append((t, r))
            break
        t += 0.5
    return out


def test_once_settle_defaults_and_is_only_for_once_checks():
    assert _slam().cfg.checks[0].settle_sec == ONCE_SETTLE_SEC
    assert _slam(settle_sec=20).cfg.checks[0].settle_sec == 20.0
    with pytest.raises(HealthConfigError, match='settle_sec is only for once'):
        parse_health_config(doc({'swept_clearance': {'checks': [
            {'topic': '/out', 'max_age_sec': 0.5, 'settle_sec': 5}]}}), REGISTRY)
    with pytest.raises(HealthConfigError, match='settle_sec must be >= 0'):
        _slam(settle_sec=-1)


def test_once_input_inside_the_grace_map_in_time_never_triggers():
    mon = _slam()
    ts = topics('/scan', '/map')
    assert _slam_run(mon, ts, 0.0, 300.0, scan_from=2.0, map_at=8.0) == []
    assert mon.status == 'OK' and mon.failures == 0


def test_once_input_after_the_grace_gets_the_settle_time():
    """The B6 case: /scan first appears 60 s after slam started -- no restart."""
    mon = _slam()
    ts = topics('/scan', '/map')
    assert _slam_run(mon, ts, 0.0, 59.5) == []
    assert mon.status == 'UPSTREAM_STALE'
    # Map 7 s after the first scan (RTF 0.74 in sim: 5 s / 0.74 + the rebuild).
    assert _slam_run(mon, ts, 60.0, 66.5, scan_from=60.0) == []
    assert mon.status == 'STARTING' and 'settling' in mon.message
    assert _slam_run(mon, ts, 67.0, 300.0, scan_from=60.0, map_at=67.0) == []
    assert mon.status == 'OK' and mon.failures == 0


def test_once_without_settle_reproduces_b6():
    """settle_sec 0 is the old behaviour: the late input restarts a healthy slam."""
    mon = _slam(settle_sec=0)
    ts = topics('/scan', '/map')
    _slam_run(mon, ts, 0.0, 59.5)
    res = _slam_run(mon, ts, 60.0, 70.0, scan_from=60.0, map_at=67.0)
    assert res == [(63.0, ['slam.launch.py'])]


def test_once_input_never_is_upstream_stale_not_a_restart():
    """No /scan at all: not slam's fault, so slam is never restarted.

    The watchdog reports it as slam UPSTREAM_STALE (WARN, naming /scan); the
    lidar itself is judged by perception's own /scan check (its alert).
    """
    mon = _slam()
    ts = topics('/scan', '/map')
    assert _slam_run(mon, ts, 0.0, 600.0) == []
    assert mon.status == 'UPSTREAM_STALE' and mon.level == WARN
    assert '/scan' in mon.message
    lidar = ComponentMonitor(swept(checks=[{'topic': '/scan', 'max_age_sec': 0.5}],
                                   action='alert'))
    assert run(lidar, ts, RUNNING, 0.0, 30.0, feeds=()) == []
    assert lidar.status == 'ALERT' and lidar.level == ERROR


def test_once_output_never_after_settle_triggers():
    """Input after the grace, map never: restart at input + settle + fail_for."""
    mon = _slam()
    ts = topics('/scan', '/map')
    _slam_run(mon, ts, 0.0, 59.5)
    res = _slam_run(mon, ts, 60.0, 120.0, scan_from=60.0)
    assert res == [(60.0 + ONCE_SETTLE_SEC + 3.0, ['slam.launch.py'])]
    assert any('/map: no message never' in e for _, e in mon.events)


def test_once_output_never_with_input_inside_the_grace_triggers_after_the_grace():
    """Settle runs from the input's arrival; it does not extend the grace."""
    mon = _slam()
    ts = topics('/scan', '/map')
    res = _slam_run(mon, ts, 0.0, 60.0, scan_from=1.0)
    assert res == [(33.0, ['slam.launch.py'])]


def test_once_settle_restarts_after_an_input_outage():
    """/scan drops out before the first map: the settle time starts again."""
    mon = _slam()
    ts = topics('/scan', '/map')
    _slam_run(mon, ts, 0.0, 59.5)
    assert _slam_run(mon, ts, 60.0, 65.0, scan_from=60.0) == []        # 5 s of scans
    assert _slam_run(mon, ts, 65.5, 80.0) == []                         # outage
    assert _slam_run(mon, ts, 80.5, 80.5 + ONCE_SETTLE_SEC - 0.5, scan_from=80.5) == []
    assert mon.status == 'STARTING'


def test_once_settle_covers_a_late_sim_clock():
    """sim: paused until the simulator starts at 60 s, /scan with its clock."""
    mon = _slam()
    ts = topics('/scan', '/map')
    assert _slam_run(mon, ts, 0.0, 59.5, paused=True) == []
    mon.resume(60.0, 5.0)
    assert _slam_run(mon, ts, 60.0, 67.5, scan_from=60.0) == []
    assert _slam_run(mon, ts, 68.0, 200.0, scan_from=60.0, map_at=68.0) == []
    assert mon.status == 'OK' and mon.failures == 0
