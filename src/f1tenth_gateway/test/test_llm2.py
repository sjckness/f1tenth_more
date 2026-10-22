"""
Llm2State: LLM2's state for the app from /llm2/status and the /llm2/set_active answers.

The rules are output/two_llm/README.md section 2: no status for 5 s is LLM2
not running (not paused); the service's answer wins over a status that
arrives less than one period after it (the two are separate channels).
"""

import json

from f1tenth_gateway.llm2 import Llm2State


def status(state, latency_ms=None, outcome='ok'):
    return json.dumps({'seq': 1, 'outcome': outcome, 'latency_ms': latency_ms, 'state': state,
                       'counters': {'ok': 3, 'invalid': 0, 'timeout': 1}})


def test_no_status_for_5_s_is_not_running_not_paused():
    s = Llm2State()
    assert s.view(0.0)['state'] == 'not_running'
    assert s.on_status(status('paused'), 10.0)
    assert s.view(14.9)['state'] == 'paused'
    v = s.view(15.1)
    assert v['state'] == 'not_running' and 'da 5 s' in v['why']
    assert s.view(11.0, service_ready=False)['state'] == 'not_running'
    assert not s.on_status('{not json', 11.0) and not s.on_status('{"state": "off"}', 11.0)
    assert not s.on_status('[' * 100000 + ']' * 100000, 11.0)    # RecursionError
    # An answer alone also says LLM2 runs, for as long as a status would.
    s.forget()
    s.on_response(True, True, 'active', 20.0)
    assert s.view(24.9)['state'] == 'active' and s.view(25.1)['state'] == 'not_running'


def test_the_answer_wins_over_a_status_within_one_period_then_the_status_rules():
    s = Llm2State(trust_s=2.0)
    s.on_status(status('active', latency_ms=412.0), 0.0)
    assert s.on_response(False, True, 'paused', 1.0) == {'ok': True, 'state': 'paused'}
    s.on_status(status('active', latency_ms=430.0), 1.1)    # published before the call
    assert s.view(1.2)['state'] == 'paused'
    s.on_status(status('paused', outcome='paused'), 2.9)
    v = s.view(3.0)
    assert v['state'] == 'paused' and v['latency_ms'] == 430.0 and v['counters']['ok'] == 3
    s.on_status(status('active'), 3.5)                       # resumed by someone else
    assert s.view(3.6)['state'] == 'active'
    bad = s.on_response(True, True, 'paused', 4.0)
    assert not bad['ok'] and bad['state'] == 'paused' and s.view(4.1)['state'] == 'paused'
    refused = s.on_response(False, False, 'paused', 5.0)
    assert not refused['ok'] and 'success=False' in refused['message']
