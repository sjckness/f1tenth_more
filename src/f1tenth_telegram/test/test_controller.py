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
              {'name': 'estop_latched', 'kind': 'estop', 'ok': not latched,
               'required': True, 'value': latched, 'mission_state': 'IDLE'}]
    failing = ([] if ready else ['scan']) + (['estop_latched'] if latched else [])
    return {'ok': True, 'cmd': 'status', 'ready': ready and not latched, 'failing': failing,
            'estop_latched': latched, 'processes': procs, 'checks': checks}


def estop_answer(zero_ok=True, late=False):
    """f1tenth_ctl estop's JSON: one record per step, and car_held."""
    def step(ok, msg, ms, **kw):
        return {'status': 'ok' if ok else 'failed', 'ok': ok, 'message': msg,
                'duration_ms': ms, **kw}
    steps = {'zero_drive': step(zero_ok, '100 msgs to 1 subscriber(s)' if zero_ok
                                else 'no subscriber matched (is ackermann_mux up?)', 2250),
             'emergency_stop': step(True, 'emergency stop engaged', 640,
                                    **({'late': True} if late else {})),
             'abort_mission': step(True, 'no mission to abort (state=IDLE)', 9)}
    return {'ok': zero_ok, 'car_held': zero_ok, 'steps': steps}


class FakeTransport:
    def __init__(self, online=True):
        self.online = online
        self.calls = []
        self.answers = {}
        self.state = {'stack': 'stopped', 'planner': 'stopped', 'logger': 'stopped'}
        self.latched = False

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
            return status_answer(**self.state, ready=self.state['stack'] == 'running',
                                 latched=self.latched)
        if args[0] in ('start', 'stop'):
            self.state[args[1]] = 'running' if args[0] == 'start' else 'stopped'
            return {'ok': True, 'pid': 42, 'signals': ['SIGINT'], 'elapsed_s': 1.0}
        if args[0] == 'killall':
            self.state = dict.fromkeys(self.state, 'stopped')
            return {'ok': True, 'units': {}, 'strays': {'count': 0},
                    'discovery': {'state': 'running'}}
        if args[0] == 'estop-reset':
            self.latched = False
            return {'ok': True, 'path': 'restart_behavior', 'latched_before': True,
                    'latched_after': False, 'verified': True}
        if args[0] == 'estop':
            self.latched = True
            return estop_answer()
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
        assert fake.calls == [['estop']] and 'car held ✅' in panel.last
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
    held = summary_estop(estop_answer(late=True)).splitlines()
    assert held[0] == '🛑 E-STOP: car held ✅' and len(held) == 4
    assert held[1].startswith('✅ zero drive') and 'LATE' in held[2]
    assert held[3] == ('✅ abort_mission (mission + MPC hold): '
                       'no mission to abort (state=IDLE) (9 ms)')
    lost = summary_estop(estop_answer(zero_ok=False)).splitlines()
    assert lost[0] == '🛑 E-STOP: car NOT confirmed held ❌' and lost[1].startswith('❌')
    assert 'E-STOP NOT SENT' in summary_estop({'ok': False, 'transport_error': 'timeout',
                                               'error': 'no answer within 15 s'})
    assert 'NOT started' in summary_startstop('start', 'stack',
                                              {'ok': False, 'error': 'already running'})
    assert 'SIGINT→SIGTERM' in summary_startstop(
        'stop', 'logger', {'ok': True, 'signals': ['SIGINT', 'SIGTERM'], 'elapsed_s': 11})


def test_estop_latch_shows_and_reset_needs_a_confirmation(ctl):
    c, fake = ctl
    c.handle('connect')
    c.handle('confirm:' + c.handle('start:stack').confirm[0])
    assert 'estop-reset' not in actions(c.panel)
    panel = c.handle('estop')
    assert panel.status['estop_latched'] is True
    assert 'estop-reset' in actions(panel)
    text = view.render(panel)[0]
    assert '🔴 E-STOP LATCHED' in text and 'READY ❌ estop_latched' in text
    fake.calls.clear()
    panel = c.handle('estop-reset')
    assert fake.calls == [] and panel.confirm[1] == 'estop-reset'
    panel = c.handle(f'confirm:{panel.confirm[0]}')
    assert fake.calls == [['estop-reset'], ['status']]
    assert 'e-stop reset (behavior restarted)' in panel.last
    assert 'estop-reset' not in actions(panel) and 'READY ✅' in view.render(panel)[0]


def test_failed_reset_is_reported():
    from f1tenth_telegram.controller import summary_estop_reset
    msg = summary_estop_reset({'ok': False, 'path': 'stack_restart',
                               'error': 'this stack was not started by f1tenth_ctl'})
    assert 'FAILED (stack restarted)' in msg and 'not started' in msg
    assert 'nothing to do' in summary_estop_reset({'ok': True, 'path': 'none'})
