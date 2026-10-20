"""Planner commands: validation, the output parsing, trimming, the exchange."""

from f1tenth_telegram import planner

from test_controller import FakeTransport, PLANNER_ANSWER


def test_validate_one_line_up_to_500_chars():
    assert planner.validate('  go ahead 2 meters ') == ('go ahead 2 meters', None)
    assert planner.validate('x' * 500)[0] == 'x' * 500
    assert 'at most 500' in planner.validate('x' * 501)[1]
    assert planner.validate('go\nahead')[1].startswith('one line only')
    assert planner.validate('   ')[1] == 'empty text'
    assert planner.validate(None)[1] == 'empty text'


def test_parse_finds_the_mission_json_and_the_result():
    p = planner.parse(PLANNER_ANSWER)
    assert p['done'] and p['result']['status'] == 'ok'
    assert p['mission_id'] == 'llm_go_ahead_2m'
    assert p['mission_json'].splitlines()[0] == '{'
    assert p['mission_json'].splitlines()[-1] == '}'
    assert not any(line.startswith(('RESULT', 'JSON (', '{', '}')) for line in p['body'])
    assert p['body'][-1] == 'scritta in: /tmp/m.json'


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
    ex = planner.Exchange('cmd', clock=Clock())
    ex.send(fake, 5)
    assert ex.poll(fake, 5) and ex.done and 'not running any more' in ex.state
    fake.answers['send'] = {'ok': False, 'error': 'planner is not running'}
    ex = planner.Exchange('cmd', clock=Clock())
    assert ex.send(fake, 5) is False and ex.done
    assert '❌ not sent: planner is not running' in ex.render()
