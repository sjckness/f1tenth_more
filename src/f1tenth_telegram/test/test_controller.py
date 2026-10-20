"""The panel state machine against a fake transport."""

import threading

from f1tenth_telegram import config, view
from f1tenth_telegram.controller import Controller, summary_estop, summary_startstop
import pytest


def status_answer(stack='stopped', planner='stopped', logger='stopped', ready=False,
                  latched=False):
    procs = {'discovery': {'state': 'running', 'address': '127.0.0.1:11811'}}
    for name, state in (('stack', stack), ('planner', planner), ('logger', logger)):
        procs[name] = {'state': state, 'managed': True}
    checks = [{'name': 'scan', 'kind': 'topic_rate', 'ok': ready, 'required': True,
               'value': 40.0 if ready else 0.0},
              {'name': 'estop_clear', 'kind': 'estop_clear', 'ok': not latched,
               'required': True, 'value': latched, 'mission_state': 'IDLE'}]
    return {'ok': True, 'cmd': 'status', 'ready': ready,
            'failing': [] if ready else ['scan'], 'processes': procs, 'checks': checks}


class FakeTransport:
    def __init__(self, online=True):
        self.online = online
        self.calls = []
        self.answers = {}
        self.state = {'stack': 'stopped', 'planner': 'stopped', 'logger': 'stopped'}

    def reachability(self, timeout):
        return {'online': self.online, 'ssh': self.online,
                **({} if self.online else {'error': 'ssh: Connection timed out'})}

    def ctl(self, args, timeout):
        self.calls.append(args)
        if not self.online:
            return {'ok': False, 'transport_error': 'ssh', 'error': 'ssh: timed out'}
        if args[0] in self.answers:
            return self.answers[args[0]]
        if args[0] == 'connect':
            return {'ok': True, 'discovery': {'address': '127.0.0.1:11811'}}
        if args[0] == 'status':
            return status_answer(**self.state, ready=self.state['stack'] == 'running')
        if args[0] in ('start', 'stop'):
            self.state[args[1]] = 'running' if args[0] == 'start' else 'stopped'
            return {'ok': True, 'pid': 42, 'signals': ['SIGINT'], 'elapsed_s': 1.0}
        if args[0] == 'killall':
            self.state = dict.fromkeys(self.state, 'stopped')
            return {'ok': True, 'units': {}, 'strays': {'count': 0},
                    'discovery': {'state': 'running'}}
        if args[0] == 'estop':
            return {'ok': True, 'service': {'ok': True, 'call_s': 0.1, 'message': 'engaged'},
                    'publish': {'ok': True, 'after_match': 100, 'rate_hz': 50.0,
                                'subscribers': 1}}
        return {'ok': False, 'error': 'unexpected'}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def ctl():
    cfg = config.from_dict({'telegram': {'allowed_user_ids': [1]}})
    clock = Clock()
    fake = FakeTransport()
    c = Controller(cfg, fake, clock=clock)
    c.clock_ = clock
    return c, fake


def actions(panel):
    return [b.action for row in view.buttons(panel) for b in row]


def test_offline_shows_only_refresh(ctl):
    c, fake = ctl
    fake.online = False
    panel = c.handle('refresh')
    assert panel.online is False and not panel.connected
    assert actions(panel) == ['refresh']
    assert '🔴 offline' in view.render(panel)[0]


def test_online_offers_connect(ctl):
    c, fake = ctl
    panel = c.handle('refresh')
    assert actions(panel) == ['connect', 'refresh']
    assert fake.calls == []


def test_connect_then_full_panel(ctl):
    c, fake = ctl
    panel = c.handle('connect')
    assert panel.connected and fake.calls == [['connect'], ['status']]
    assert actions(panel) == ['start:stack', 'start:planner', 'start:logger', 'status',
                              'killall', 'estop']
    assert 'ready to launch' in panel.last


def test_start_stack_needs_a_confirmation(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.calls.clear()
    panel = c.handle('start:stack')
    assert fake.calls == [] and panel.confirm[1] == 'start:stack'
    assert actions(panel) == [f'confirm:{panel.confirm[0]}', f'cancel:{panel.confirm[0]}',
                              'estop']
    panel = c.handle(f'confirm:{panel.confirm[0]}')
    assert fake.calls == [['start', 'stack'], ['status']]
    assert 'stop:stack' in actions(panel) and 'stack started' in panel.last
    assert view.render(panel)[0].count('READY ✅') == 1


def test_late_confirmation_does_not_run(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.calls.clear()
    nonce = c.handle('killall').confirm[0]
    c.clock_.t += 11
    panel = c.handle(f'confirm:{nonce}')
    assert fake.calls == [] and 'ask again' in panel.last and panel.confirm is None


def test_cancel_does_not_run(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.calls.clear()
    nonce = c.handle('start:planner').confirm[0]
    panel = c.handle(f'cancel:{nonce}')
    assert fake.calls == [] and panel.last == '✖ cancelled'


def test_stop_and_logger_need_no_confirmation(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.calls.clear()
    c.handle('start:logger')
    c.handle('stop:logger')
    assert fake.calls == [['start', 'logger'], ['status'], ['stop', 'logger'], ['status']]


def test_estop_runs_while_another_action_holds_the_lock(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.calls.clear()
    c._lock.acquire()
    try:
        panel = c.handle('estop')
        assert fake.calls == [['estop']] and 'E-STOP sent' in panel.last
        panel = c.handle('status')
        assert 'not queued' in panel.last and fake.calls == [['estop']]
    finally:
        c._lock.release()


def test_concurrent_actions_are_refused_not_queued(ctl):
    c, fake = ctl
    c.handle('connect')
    gate = threading.Event()
    release = threading.Event()
    orig = fake.ctl

    def slow(args, timeout):
        if args[0] == 'stop':
            gate.set()
            release.wait(5)
        return orig(args, timeout)

    fake.ctl = slow
    th = threading.Thread(target=c.handle, args=('stop:stack',))
    th.start()
    gate.wait(5)
    panel = c.handle('start:logger')
    assert 'not queued' in panel.last
    release.set()
    th.join(5)
    assert ['start', 'logger'] not in fake.calls


def test_ssh_failure_marks_the_car_offline(ctl):
    c, fake = ctl
    c.handle('connect')
    fake.online = False
    panel = c.handle('status')
    assert panel.online is False and not panel.connected and panel.status is None
    assert actions(panel) == ['refresh']


def test_summaries():
    assert 'LATE 1.2 s' in summary_estop(
        {'ok': True, 'service': {'ok': True, 'late': True, 'call_s': 1.2, 'message': 'm'},
         'publish': {'ok': True, 'after_match': 100, 'rate_hz': 50.0, 'subscribers': 1}})
    assert 'E-STOP NOT SENT' in summary_estop({'ok': False, 'transport_error': 'timeout',
                                               'error': 'no answer within 15 s'})
    assert 'NOT started' in summary_startstop('start', 'stack',
                                              {'ok': False, 'error': 'already running'})
    assert 'SIGINT→SIGTERM' in summary_startstop(
        'stop', 'logger', {'ok': True, 'signals': ['SIGINT', 'SIGTERM'], 'elapsed_s': 11})
