"""Planner commands: validation, the output parsing, trimming, the exchange."""

from f1tenth_telegram import planner
import pytest

from test_controller import FakeTransport, PLANNER_ANSWER


def test_validate_collapses_lines_and_allows_500_chars():
    assert planner.validate('  go ahead 2 meters ') == ('go ahead 2 meters', None)
    assert planner.validate('x' * 500)[0] == 'x' * 500
    assert 'at most 500' in planner.validate('x' * 501)[1]
    assert planner.validate('go\n ahead\r\n') == ('go ahead', None)
    assert planner.validate('   ')[1] == 'empty text'
    assert planner.validate(None)[1] == 'empty text'


def test_parse_finds_the_mission_json_and_the_result():
    p = planner.parse(PLANNER_ANSWER)
    assert p['done'] and p['result']['status'] == 'ok'
    assert p['mission_id'] == 'llm_go_ahead_2m'
    assert p['mission_json'].splitlines()[0] == '{'
    assert p['mission_json'].splitlines()[-1] == '}'
    assert not any(line.startswith(('RESULT', 'JSON (', '{', '}')) for line in p['body'])
    assert 'scritta in: /tmp/m.json' in p['body']


def test_parse_without_an_end_is_not_done():
    assert not planner.parse(PLANNER_ANSWER[:5])['done']
    # The interactive prompt sticks to the next line and is removed.
    assert planner.parse(['comando> comando> hello'])['body'] == ['hello']


def test_render_highlights_the_mission_and_escapes_html():
    out = planner.render('go <fast> & far', PLANNER_ANSWER, planner.Exchange.DONE, 3.4)
    assert out.startswith('📝 «go &lt;fast&gt; &amp; far» → planner')
    assert '🏁 planner answered in 3 s' in out and '✅ result: ok' in out
    assert '<pre><code class="language-json">{' in out and '&quot;mission_id&quot;' in out


def test_render_trims_to_the_last_3500_chars():
    lines = [f'line {i:04d} ' + 'x' * 60 for i in range(400)]
    out = planner.render('cmd', lines, planner.Exchange.WAITING, 10)
    assert len(out) <= planner.MAX_CHARS
    assert 'line 0399' in out and 'line 0000' not in out and '…' in out


def test_render_failed_results():
    refused = planner.parse(['RESULT {"status": "refused", "error_type": "EmptyPlanError", '
                             '"error_message": "ambiguous"}'])
    assert planner.result_line(refused['result']) == (
        '⚠️ result: refused (EmptyPlanError: ambiguous)')
    assert planner.result_line({'status': 'llm_error', 'error_message': 'down'}) == (
        '❌ result: llm_error (down)')


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_exchange_follows_the_log_to_the_result():
    fake = FakeTransport()
    fake.state['planner'] = 'running'
    fake.planner_batches = [PLANNER_ANSWER[:3], [], PLANNER_ANSWER[3:]]
    ex = planner.Exchange('go ahead 2 meters', clock=Clock())
    assert ex.send(fake, 5) and fake.sent == ['go ahead 2 meters']
    assert ex.poll(fake, 5) and not ex.done
    assert ex.poll(fake, 5) is False and not ex.done
    assert ex.poll(fake, 5) and ex.done and ex.state == planner.Exchange.DONE
    assert fake.calls[1] == ['planner-log', '--since', '100']
    assert fake.calls[2] == ['planner-log', '--since', '130']
    assert ex.poll(fake, 5) is False   # done: no more polling


def test_exchange_gives_up_after_60_s():
    fake = FakeTransport()
    fake.state['planner'] = 'running'
    fake.planner_batches = []
    fake.answer_next = []              # the planner says nothing at all
    clock = Clock()
    ex = planner.Exchange('cmd', clock=clock)
    ex.send(fake, 5)
    clock.t = 59.0
    assert not ex.poll(fake, 5) and not ex.done
    clock.t = 60.5
    assert ex.poll(fake, 5) and ex.done and 'no RESULT line after 60 s' in ex.render()


def test_exchange_stops_when_the_planner_dies_or_send_fails():
    fake = FakeTransport()
    fake.planner_batches = []      # it printed nothing, and is gone
    fake.answer_next = []
    ex = planner.Exchange('cmd', clock=Clock())
    ex.send(fake, 5)
    assert ex.poll(fake, 5) and ex.done and 'not running any more' in ex.state
    fake.answers['send'] = {'ok': False, 'error': 'planner is not running'}
    ex = planner.Exchange('cmd', clock=Clock())
    assert ex.send(fake, 5) is False and ex.done
    assert '❌ not sent: planner is not running' in ex.render()


# ---- long-poll: planner-log --wait, and the fallback to the 2 s polling ----

class WaitTransport(FakeTransport):
    """A car whose ctl may or may not know `planner-log --wait`."""

    def __init__(self, knows_wait=True):
        super().__init__()
        self.state['planner'] = 'running'
        self.knows_wait = knows_wait
        self.timeouts = []

    def ctl(self, args, timeout):
        self.timeouts.append(timeout)
        if '--wait' not in args:
            return super().ctl(args, timeout)
        self.calls.append(args)
        if not self.knows_wait:
            return {'ok': False, 'transport_error': 'parse', '_rc': 2,
                    'error': 'no JSON from f1tenth_ctl (exit 2): f1tenth_ctl: error: '
                             'unrecognized arguments: --wait 8'}
        return self._waited(args, timeout)

    def _waited(self, args, timeout):
        i = args.index('--wait')
        calls = len(self.calls)
        out = FakeTransport.ctl(self, args[:i] + args[i + 2:], timeout)
        del self.calls[calls:]               # keep the call as the bot made it
        return out


def test_a_long_poll_asks_the_car_to_wait_and_widens_the_timeout():
    fake = WaitTransport()
    fake.planner_batches = [PLANNER_ANSWER]
    ex = planner.Exchange('go ahead', clock=Clock(), wait_s=8.0)
    assert ex.send(fake, 5)
    assert ex.poll_delay() == 0.0
    assert ex.poll(fake, 12) and ex.done and ex.state == planner.Exchange.DONE
    assert fake.calls[-1] == ['planner-log', '--since', '100', '--wait', '8']
    assert fake.timeouts[-1] == 20            # the ssh timeout covers the wait


def test_an_old_ctl_that_rejects_wait_is_polled_the_old_way():
    fake = WaitTransport(knows_wait=False)
    fake.planner_batches = [PLANNER_ANSWER[:3], PLANNER_ANSWER[3:]]
    ex = planner.Exchange('go ahead', clock=Clock(), wait_s=8.0)
    ex.send(fake, 5)
    assert ex.poll_delay() == 0.0
    assert ex.poll(fake, 12) and not ex.done         # the rejected call was repeated, plain
    assert fake.calls[-2:] == [['planner-log', '--since', '100', '--wait', '8'],
                               ['planner-log', '--since', '100']]
    assert ex.wait_s == 0.0 and ex.poll_delay() == planner.POLL_S
    assert ex.poll(fake, 12) and ex.done
    # from now on no more --wait
    assert all('--wait' not in c for c in fake.calls[-1:])


def test_other_failures_are_not_mistaken_for_an_old_ctl():
    assert not planner.wait_flag_rejected({'transport_error': 'ssh', '_rc': 255,
                                           'error': 'ssh: timed out'})
    assert not planner.wait_flag_rejected({'transport_error': 'parse', '_rc': 1,
                                           'error': 'no JSON from f1tenth_ctl (exit 1): boom'})
    assert not planner.wait_flag_rejected({'ok': True})
    # a real transport failure keeps wait mode on and is reported, not retried
    fake = WaitTransport()
    fake.answers['planner-log'] = {'ok': False, 'transport_error': 'ssh', 'error': 'ssh: down'}
    ex = planner.Exchange('cmd', clock=Clock(), wait_s=8.0)
    ex.send(fake, 5)
    ex.poll(fake, 12)
    assert ex.wait_s == 8.0 and ex.error == 'ssh: down'


def test_without_wait_the_exchange_is_exactly_as_before():
    fake = FakeTransport()
    fake.state['planner'] = 'running'
    ex = planner.Exchange('cmd', clock=Clock())
    ex.send(fake, 5)
    assert ex.wait_s == 0.0 and ex.poll_delay() == planner.POLL_S
    ex.poll(fake, 5)
    assert fake.calls[-1] == ['planner-log', '--since', '100']


def test_the_planner_dying_during_a_long_poll_still_ends_the_exchange():
    fake = WaitTransport()
    fake.state['planner'] = 'stopped'
    fake.planner_batches = []
    fake.answer_next = []
    ex = planner.Exchange('cmd', clock=Clock(), wait_s=8.0)
    ex.send(fake, 5)
    assert ex.poll(fake, 12) and ex.done and 'not running any more' in ex.state


def test_the_config_default_and_validation():
    from f1tenth_telegram import config
    base = {'target': {'ctl_command': '/x'}}
    assert config.from_dict(base).planner_wait_s == 8.0
    assert config.from_dict({**base, 'telegram': {'planner_wait_s': 0}}).planner_wait_s == 0.0
    with pytest.raises(config.ConfigError):
        config.from_dict({**base, 'telegram': {'planner_wait_s': -1}})
    with pytest.raises(config.ConfigError):
        config.from_dict({**base, 'telegram': {'planner_wait_s': 'soon'}})
