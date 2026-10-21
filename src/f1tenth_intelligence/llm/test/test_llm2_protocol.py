"""
LLM2 request and reply logic (llm2_protocol), without ROS and without a model.

The parts that can break silently: a reply that is accepted although it is
not exactly the parameter set (a missing key, a NaN, a boolean, trailing
text), a second request started while one is still running, a request body
that quietly loses temperature 0, the key schema or the prompt cache, and a
slow server that hangs the node instead of counting a timeout. The HTTP
tests run against conftest.py's fake /completion endpoint on 127.0.0.1.
"""

import json
import math
import socket
import threading
import time

from llm.llm2_protocol import (
    build_request_body,
    completion_url,
    ERROR,
    INVALID,
    MOCK_PARAMS,
    mock_params_from,
    OK,
    params_json,
    PROMPT_PREFIX,
    request_params,
    RequestGate,
    SKIPPED,
    TIMEOUT,
    validate_reply,
)
import pytest

PARAMS = dict(MOCK_PARAMS)
NAMES = list(PARAMS)


# -- reply validation ---------------------------------------------------------

def test_an_exact_echo_is_valid_and_comes_back_equal():
    check = validate_reply(params_json(PARAMS), NAMES)
    assert check.reason == ''
    assert check.values == PARAMS


def test_a_changed_value_is_valid_but_differs_from_what_was_sent():
    reply = dict(PARAMS, mpc_w_v=5.0)
    check = validate_reply(json.dumps(reply), NAMES)
    assert check.values == reply
    assert check.values != PARAMS


def test_values_come_back_as_floats_in_the_configured_order():
    reversed_reply = {name: int(PARAMS[name]) for name in reversed(NAMES)}
    check = validate_reply(json.dumps(reversed_reply), NAMES)
    assert list(check.values) == NAMES
    assert all(type(value) is float for value in check.values.values())


def _with(**changes):
    return json.dumps(dict(PARAMS, **changes))


def _without(name):
    return json.dumps({k: v for k, v in PARAMS.items() if k != name})


@pytest.mark.parametrize('content, reason_part', [
    ('', 'not one JSON value'),
    ('sure, here it is: ' + params_json(PARAMS), 'not one JSON value'),
    (params_json(PARAMS) + ' done', 'not one JSON value'),
    (json.dumps([PARAMS]), 'not an object'),
    (_without('mpc_w_v'), "missing ['mpc_w_v']"),
    (_with(car_radius=0.1), "unexpected ['car_radius']"),
    (params_json(PARAMS)[:-1] + ', "mpc_w_v": 99.0}', 'repeated keys'),
    (_with(mpc_w_v='4.2857'), 'not a number'),
    (_with(mpc_w_v=True), 'not a number'),
    (_with(mpc_w_v=None), 'not a number'),
    (_with(mpc_w_v={'value': 4.2857}), 'not a number'),
    (_with(mpc_w_v=math.nan), 'not finite'),           # json.dumps writes NaN
    (_with(mpc_w_v=math.inf), 'not finite'),           # ... and Infinity
    (params_json(PARAMS).replace('4.2857', '1e999'), 'not finite'),
    (params_json(PARAMS).replace('4.2857', '1' + '0' * 400), 'not finite'),
], ids=[
    'empty', 'leading_text', 'trailing_text', 'array', 'missing_key', 'extra_key',
    'repeated_key', 'string_value', 'bool_value', 'null_value', 'object_value',
    'nan', 'infinity', 'float_overflow', 'int_overflow',
])
def test_an_invalid_reply_is_rejected_with_its_reason(content, reason_part):
    check = validate_reply(content, NAMES)
    assert check.values is None
    assert reason_part in check.reason


@pytest.mark.parametrize('names, values, reason_part', [
    (['a', 'b'], [1.0], 'must pair up'),
    ([], [], 'empty'),
    (['a', 'a'], [1.0, 2.0], 'repeats'),
    (['a', ''], [1.0, 2.0], 'empty name'),
    (['a'], [math.nan], 'not finite'),
])
def test_a_mock_set_the_node_must_not_start_with_is_refused(names, values, reason_part):
    with pytest.raises(ValueError, match=reason_part):
        mock_params_from(names, values)


# -- request body -------------------------------------------------------------

def test_the_request_is_greedy_cached_and_constrained_to_exactly_these_keys():
    body = build_request_body(PARAMS, 256)
    assert body['temperature'] == 0.0
    assert body['cache_prompt'] is True
    assert body['n_predict'] == 256
    schema = body['json_schema']
    assert list(schema['properties']) == NAMES
    assert schema['required'] == NAMES
    assert schema['additionalProperties'] is False
    assert all(spec == {'type': 'number'} for spec in schema['properties'].values())
    assert params_json(PARAMS) in body['prompt']


def test_the_prompt_starts_with_the_same_fixed_prefix_for_any_parameter_set():
    other = {'mpc_w_v': 1.0}
    for params in (PARAMS, other):
        prompt = build_request_body(params, 256)['prompt']
        assert prompt.startswith(PROMPT_PREFIX)
        assert prompt.index(params_json(params)) == len(PROMPT_PREFIX)


# -- one request in flight at most --------------------------------------------

def test_a_tick_while_a_request_is_in_flight_is_skipped_and_counted():
    gate = RequestGate()
    assert gate.try_start(1).started
    second = gate.try_start(2)
    third = gate.try_start(3)
    assert not second.started and not third.started
    assert second.in_flight == 1 and third.in_flight == 1
    assert third.counters[SKIPPED] == 2
    assert gate.in_flight == 1


def test_the_gate_reopens_once_the_request_finishes_and_counts_its_outcome():
    gate = RequestGate()
    gate.try_start(1)
    counters = gate.finish(OK, echo_match=True)
    assert counters[OK] == 1 and counters['echo_match'] == 1
    assert gate.in_flight is None
    assert gate.try_start(2).started
    counters = gate.finish(TIMEOUT, None)
    assert counters[TIMEOUT] == 1 and counters['echo_match'] == 1


def test_concurrent_ticks_never_start_two_requests():
    gate = RequestGate()
    barrier = threading.Barrier(8)
    started = []

    def tick(seq):
        barrier.wait()
        if gate.try_start(seq).started:
            started.append(seq)

    threads = [threading.Thread(target=tick, args=(seq,)) for seq in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(started) == 1


# -- against a fake llama-server (conftest.py) ---------------------------------

def test_an_exact_echo_from_the_server_is_ok_with_its_timings(fake_server):
    server = fake_server()
    result = request_params(completion_url(server.url + '/'), PARAMS, 5.0, 256)
    assert result.outcome == OK, result.reason
    assert result.values == PARAMS and result.echo_match is True
    assert result.timings == {
        'prompt_n': 1, 'prompt_ms': 12.5, 'predicted_n': 82, 'predicted_ms': 880.0}
    assert result.tokens_cached == 214
    (body,) = server.bodies
    assert body == build_request_body(PARAMS, 256)


def test_a_server_that_answers_late_is_a_timeout_not_a_hang(fake_server):
    server = fake_server(delay_s=5.0)
    t0 = time.monotonic()
    result = request_params(completion_url(server.url), PARAMS, 0.3, 256)
    elapsed = time.monotonic() - t0
    assert result.outcome == TIMEOUT
    assert result.values is None and result.echo_match is None
    assert 0.3 <= elapsed < 2.0


def test_a_reply_that_fails_validation_is_invalid_and_carries_no_values(fake_server):
    server = fake_server(answer=lambda body: (200, {'content': '{"mpc_w_v": 4.2857}'}))
    result = request_params(completion_url(server.url), PARAMS, 5.0, 256)
    assert result.outcome == INVALID
    assert result.values is None and result.echo_match is False
    assert 'missing' in result.reason


def test_a_server_still_loading_its_model_is_an_error(fake_server):
    server = fake_server(answer=lambda body: (503, {'error': 'Loading model'}))
    result = request_params(completion_url(server.url), PARAMS, 5.0, 256)
    assert result.outcome == ERROR and 'HTTP 503' in result.reason


def test_nothing_listening_is_an_error_not_a_timeout():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    result = request_params(f'http://127.0.0.1:{port}/completion', PARAMS, 5.0, 256)
    assert result.outcome == ERROR and 'ConnectionError' in result.reason
