"""reconnect.py: retry triggers, rate limit, and what one attempt runs (nmcli mocked)."""

from concurrent.futures import Future

from f1tenth_comms.reconnect import (NmcliReconnector, parse_ssid_list, parse_terse,
                                     ReconnectDriver, ReconnectPolicy)
import pytest


def policy(**kw):
    args = {'retry_period_s': 2.5, 'retry_backoff_max_s': 0.0, 'retry_distance_m': 3.0,
            'retry_on_new_cell': True, 'retry_min_interval_s': 2.0}
    args.update(kw)
    return ReconnectPolicy(**args)


# ---- triggers ---------------------------------------------------------------------------

def test_nothing_fires_while_the_link_is_up():
    p = policy()
    p.update_link(0.0, lost=False)
    p.add_distance(100.0)
    p.entered_new_cell()
    assert p.due(10.0) is None


def test_time_trigger_counts_from_lost_then_from_each_attempt():
    p = policy()
    p.update_link(10.0, lost=True)
    assert p.due(12.4) is None
    assert p.due(12.5) == 'time'
    p.started(12.5)
    assert p.due(14.9) is None
    assert p.due(15.0) == 'time'


def test_distance_trigger_and_its_reset():
    p = policy(retry_period_s=0.0)
    p.update_link(0.0, lost=True)
    p.add_distance(2.0)
    assert p.due(5.0) is None
    p.add_distance(1.0)
    assert p.due(5.0) == 'distance'
    p.started(5.0)
    assert p.distance_m == 0.0 and p.due(8.0) is None


def test_new_cell_trigger_waits_for_the_min_interval_but_is_not_forgotten():
    p = policy(retry_period_s=0.0)
    p.update_link(0.0, lost=True)
    p.entered_new_cell()
    assert p.due(1.0) is None            # 1 s after LOST began < 2 s
    assert p.due(2.0) == 'new_cell'
    p.started(2.0)
    p.entered_new_cell()
    assert p.due(3.9) is None
    assert p.due(4.0) == 'new_cell'


def test_link_back_resets_everything():
    p = policy()
    p.update_link(0.0, lost=True)
    p.add_distance(10.0)
    p.entered_new_cell()
    p.started(2.0)
    p.update_link(3.0, lost=False)
    assert (p.lost_since, p.outage_attempts, p.distance_m, p.new_cell_pending) == \
        (None, 0, 0.0, False)
    p.add_distance(10.0)                 # not counted while the link is up
    p.update_link(4.0, lost=True)
    assert p.distance_m == 0.0 and p.due(5.0) is None


def test_backoff_doubles_the_time_trigger_up_to_the_cap():
    p = policy(retry_backoff_max_s=10.0)
    p.update_link(0.0, lost=True)
    intervals = []
    for _ in range(5):
        intervals.append(p.interval())
        p.started(0.0)
    assert intervals == [2.5, 5.0, 10.0, 10.0, 10.0]
    assert policy().interval() == 2.5    # 0: no backoff


def test_in_flight_blocks_every_trigger():
    p = policy()
    p.update_link(0.0, lost=True)
    p.entered_new_cell()
    assert p.due(10.0, in_flight=True) is None
    assert p.due(10.0) == 'new_cell'


# ---- one attempt ----------------------------------------------------------------------------

class FakeNmcli:
    """Answers nmcli like NetworkManager would, records every command."""

    def __init__(self, state='100 (connected)', connection='eduroam', ssids=('eduroam',),
                 profile_ssid='f1tenth-base', up_rc=0):
        self.state, self.connection, self.ssids = state, connection, list(ssids)
        self.profile_ssid, self.up_rc = profile_ssid, up_rc
        self.calls = []

    def __call__(self, cmd, timeout):
        self.calls.append(cmd)
        args = cmd[1:]
        if args[:3] == ['-t', '-f', 'GENERAL.STATE,GENERAL.CONNECTION']:
            return 0, f'GENERAL.STATE:{self.state}\nGENERAL.CONNECTION:{self.connection}\n', ''
        if args[:3] == ['-t', '-f', '802-11-wireless.ssid']:
            if self.profile_ssid is None:
                return 10, '', 'Error: f1tenth-base - no such connection profile.'
            return 0, f'802-11-wireless.ssid:{self.profile_ssid}\n', ''
        if args[:3] == ['-t', '-f', 'SSID']:
            return 0, ''.join(s + '\n' for s in self.ssids), ''
        if 'up' in args:
            if self.up_rc:
                return self.up_rc, '', ('Error: Connection activation failed: '
                                        'No suitable device found.')
            return 0, 'Connection successfully activated', ''
        raise AssertionError(cmd)

    @property
    def ups(self):
        return [c for c in self.calls if 'up' in c]


def reconnector(fake, **kw):
    return NmcliReconnector('wlP1p1s0', 'f1tenth-base', run=fake, **kw)


def test_parsers():
    assert parse_terse('GENERAL.STATE:100 (connected)\nGENERAL.CONNECTION:a\\:b\n') == \
        {'GENERAL.STATE': '100 (connected)', 'GENERAL.CONNECTION': 'a:b'}
    assert parse_ssid_list('eduroam\n\nf1tenth-base\n') == ['eduroam', 'f1tenth-base']


def test_from_eduroam_only_if_the_hotspot_is_in_the_scan():
    fake = FakeNmcli(ssids=['eduroam', 'TIM-71892397'])
    assert reconnector(fake).attempt(-50.0) == "skipped: on eduroam, 'f1tenth-base' not in scan"
    assert fake.ups == []
    fake.ssids.append('f1tenth-base')
    assert reconnector(fake).attempt(-50.0) == 'ok'
    assert fake.ups == [['nmcli', '--wait', '15', 'connection', 'up', 'id', 'f1tenth-base',
                         'ifname', 'wlP1p1s0']]


def test_on_the_hotspot_with_a_good_signal_it_does_not_reassociate():
    fake = FakeNmcli(connection='f1tenth-base')
    assert reconnector(fake).attempt(-50.0).startswith('skipped: on f1tenth-base at -50 dBm')
    assert reconnector(fake).attempt(-75.0) == 'ok'
    assert reconnector(fake).attempt(None) == 'ok'          # no reading: try
    assert reconnector(fake, skip_if_rssi_above_dbm=0.0).attempt(-40.0) == 'ok'
    assert len(fake.ups) == 3


def test_disconnected_goes_straight_to_connection_up():
    fake = FakeNmcli(state='30 (disconnected)', connection='')
    assert reconnector(fake).attempt(None) == 'ok'
    assert not [c for c in fake.calls if 'list' in c]       # no scan needed


def test_busy_or_unavailable_device_is_left_alone():
    for state in ('70 (connecting (getting IP configuration))', '20 (unavailable)'):
        fake = FakeNmcli(state=state, connection='')
        assert reconnector(fake).attempt(None).startswith('skipped:')
        assert fake.ups == []


def test_missing_profile_and_failed_activation():
    fake = FakeNmcli(state='30 (disconnected)', connection='', profile_ssid=None)
    assert reconnector(fake).attempt(None).startswith('skipped: no NetworkManager profile')
    fake = FakeNmcli(state='30 (disconnected)', connection='', up_rc=4)
    assert reconnector(fake).attempt(None) == \
        'failed: Error: Connection activation failed: No suitable device found.'


def test_dry_run_never_runs_connection_up():
    fake = FakeNmcli(state='30 (disconnected)', connection='')
    r = reconnector(fake, dry_run=True).attempt(None)
    assert r.startswith('dry-run: would run nmcli --wait 15 connection up id f1tenth-base')
    assert fake.ups == []


# ---- the rate limit, end to end with a mocked nmcli ------------------------------------------

class ManualPool:
    """submit() that runs the attempt only when the test says it is finished."""

    def __init__(self):
        self.pending = []

    def __call__(self, fn, *args):
        f = Future()
        self.pending.append((f, fn, args))
        return f

    def finish_all(self):
        for f, fn, args in self.pending:
            f.set_result(fn(*args))
        self.pending = []


@pytest.mark.parametrize('attempt_ticks', [0, 1, 7, 40])
def test_nmcli_up_rate_limit_under_constant_triggers(attempt_ticks):
    """
    60 s LOST, a new cell and 10 m of travel every 0.5 s tick: every trigger always due.

    attempt_ticks: how many ticks one `nmcli connection up` takes (0 = instant,
    40 = 20 s, longer than nmcli_wait_s). Whatever the triggers do, no two
    attempts are closer than retry_min_interval_s and never two at once.
    """
    fake = FakeNmcli(state='30 (disconnected)', connection='')
    pool = ManualPool()
    p = policy(retry_min_interval_s=2.0)
    d = ReconnectDriver(p, reconnector(fake).attempt, submit=pool)
    starts, running_since = [], None
    for i in range(121):
        now = i * 0.5
        p.update_link(now, lost=True)
        p.add_distance(10.0)
        p.entered_new_cell()
        if running_since is not None and i - running_since >= attempt_ticks:
            pool.finish_all()
            running_since = None
        assert len(pool.pending) <= 1
        trig = d.tick(now)
        if trig:
            assert running_since is None, 'started while one was running'
            starts.append(now)
            running_since = i
            if attempt_ticks == 0:
                pool.finish_all()
                running_since = None
    pool.finish_all()
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert starts and min(gaps) >= 2.0 - 1e-9
    assert len(fake.ups) == len(starts) == d.attempts
    if attempt_ticks <= 1:
        assert len(starts) == 30           # 60 s / 2 s
    p.update_link(61.0, lost=False)      # link back: nothing new starts, the last is collected
    d.tick(61.0)
    assert d.last_result == 'ok' and d.completed == d.attempts


def test_driver_collects_exceptions_and_respects_enabled():
    p = policy()
    p.update_link(0.0, lost=True)

    def boom(rssi):
        raise RuntimeError('nmcli vanished')

    pool = ManualPool()
    d = ReconnectDriver(p, boom, submit=pool)
    assert d.tick(3.0) == 'time'
    pool.pending[0][0].set_exception(RuntimeError('nmcli vanished'))
    d.tick(3.5)
    assert d.last_result == 'failed: RuntimeError: nmcli vanished'
    off = ReconnectDriver(policy(), boom, enabled=False, submit=pool)
    off.policy.update_link(0.0, lost=True)
    assert off.tick(100.0) is None and off.attempts == 0
