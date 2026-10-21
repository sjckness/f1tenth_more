"""
LLM2 request and reply logic, free of rclpy so it tests without a ROS graph.

llm2_mpc_param_node (the ROS side) asks its own llama-server for an MPC
parameter set once per tick. Everything between "these are the parameters"
and "this reply is a valid parameter set" lives here: the prompt, the request
body, the HTTP call and how its outcome is classified, the reply validation,
and the gate that keeps two requests from ever overlapping.

The mock (two-llm-arch, for now): the prompt carries a fixed parameter set
and asks the model to return it unchanged. A real tuner later changes what
the prompt asks for. The request contract, the validation and the gate stay
as they are.
"""

import json
import math
import threading
import time
from typing import Dict, NamedTuple, Optional, Sequence

import requests

#: MpcParamUpdate.source of every message the node publishes.
SOURCE = 'llm2_mock'

# Outcome of one tick. Also the keys of the cumulative counters on
# /llm2/status (plus 'echo_match', a subset of 'ok').
OK = 'ok'                # valid reply, published
INVALID = 'invalid'      # the server answered, the content failed validation
TIMEOUT = 'timeout'      # no reply within the request's timeout
ERROR = 'error'          # no usable reply: connection refused, HTTP error, no 'content'
SKIPPED = 'skipped'      # active, but the previous request was still in flight
PAUSED = 'paused'        # the node is paused (/llm2/set_active): nothing was sent
DISCARDED = 'discarded'  # the request was in flight when the node was paused: its
#                          reply, whatever it was, is neither published nor
#                          counted as ok / invalid / timeout / error
OUTCOMES = (OK, INVALID, TIMEOUT, ERROR, SKIPPED, PAUSED, DISCARDED)

#: The mock parameter set: MPC_corr ROS parameter name -> its value on the
#: launch path (the stack_params.yaml defaults, output/two_llm/recon.md
#: section 3). The node sends it every tick and, on an exact echo, MPC_corr
#: applies the same values it already runs, so the round trip changes nothing.
#: mpc_w_du_delta is 8.33, the launch-path value, NOT the 8.5714 that the
#: comments around it quote (output/mpc_known_failures.md #13).
#: Left out on purpose: vdes (per-move mission state, not a declared
#: parameter: a "current" 0.5 would change a move commanded at 0.4),
#: mpc_w_obs (safety term), mpc_w_line (always overridden where it acts),
#: mpc_w_delta0 / mpc_w_u_a / mpc_w_du_a (zero on purpose).
#: config/llm2_mpc_param_params.yaml carries the same set; the node's tests
#: hold both equal to stack_params.yaml.
MOCK_PARAMS = (
    ('mpc_w_term', 9.0),
    ('mpc_w_psi', 4.5),
    ('mpc_w_psi_stage', 4.7736),
    ('mpc_w_corr', 3.5714),
    ('mpc_w_v', 4.2857),
    ('mpc_w_du_delta', 8.33),
)

# The prompt is ChatML, Qwen2.5-Instruct's own chat template, written out by
# hand because /completion takes a raw prompt. Everything before the JSON is
# fixed text and stays byte-identical between requests, so llama-server's
# prompt cache (cache_prompt, one slot) reuses it: after the first request
# only what follows the prefix is ever evaluated again (in the mock nothing
# changes, measured: 1 prompt token on a warm request). The wording is the one
# stage 1 measured on linus (echo_match on every request).
PROMPT_PREFIX = (
    '<|im_start|>system\n'
    'You tune the cost weights of a model predictive controller. Reply with '
    'one JSON object and nothing else.<|im_end|>\n'
    '<|im_start|>user\n'
    'Return the following JSON object exactly as given: same keys, same '
    'values, no other text.\n'
)
PROMPT_SUFFIX = '<|im_end|>\n<|im_start|>assistant\n'

#: llama-server's own per-request timings carried onto /llm2/status:
#: prompt_ms is the prefill, predicted_ms the generation.
TIMING_KEYS = ('prompt_n', 'prompt_ms', 'predicted_n', 'predicted_ms')


def mock_params_from(names: Sequence[str], values: Sequence[float]) -> Dict[str, float]:
    """
    Build the name -> value set from the node's two parallel parameter lists.

    Raises ValueError on a set the node must not start with: different
    lengths, no names, a repeated or empty name, a value that is not finite.
    """
    names = [str(name) for name in names]
    values = [float(value) for value in values]
    if len(names) != len(values):
        raise ValueError(
            f'mock_param_names has {len(names)} entries, mock_param_values '
            f'{len(values)}: they are one name -> value list and must pair up')
    if not names:
        raise ValueError('mock_param_names is empty: nothing to ask for')
    repeated = sorted({name for name in names if names.count(name) > 1})
    if repeated:
        raise ValueError(f'mock_param_names repeats {repeated}')
    if any(not name for name in names):
        raise ValueError('mock_param_names contains an empty name')
    not_finite = [name for name, value in zip(names, values) if not math.isfinite(value)]
    if not_finite:
        raise ValueError(f'mock_param_values not finite for {not_finite}')
    return dict(zip(names, values))


def params_json(params: Dict[str, float]) -> str:
    """Return the JSON text the prompt carries (keys in the configured order)."""
    return json.dumps(params)


def build_prompt(params: Dict[str, float]) -> str:
    """Return the full prompt: the fixed prefix, the JSON, the assistant turn."""
    return PROMPT_PREFIX + params_json(params) + PROMPT_SUFFIX


def json_schema_for(names: Sequence[str]) -> dict:
    """
    Return the JSON schema of a reply: one object, exactly these keys, numbers.

    llama-server compiles it into a grammar, so the model cannot emit text
    around the object, a missing or extra key, or a non-number. A JSON number
    can still be 1e999 (infinite once parsed), and the server is not the
    validator anyway: validate_reply() checks everything again.
    """
    return {
        'type': 'object',
        'properties': {name: {'type': 'number'} for name in names},
        'required': list(names),
        'additionalProperties': False,
    }


def build_request_body(params: Dict[str, float], n_predict: int) -> dict:
    """Return the /completion request body for one tick."""
    return {
        'prompt': build_prompt(params),
        'temperature': 0.0,
        'n_predict': int(n_predict),
        'cache_prompt': True,
        'json_schema': json_schema_for(list(params)),
    }


class ReplyCheck(NamedTuple):
    """validate_reply()'s verdict: values is None exactly when the reply is invalid."""

    values: Optional[Dict[str, float]]
    reason: str


def _object_without_repeated_keys(pairs):
    # json.loads keeps the LAST of two equal keys without a word, which would
    # let a reply carry a value nobody sees. Reject it instead.
    keys = [key for key, _ in pairs]
    repeated = sorted({key for key in keys if keys.count(key) > 1})
    if repeated:
        raise ValueError(f'repeated keys {repeated}')
    return dict(pairs)


def validate_reply(content: str, expected_names: Sequence[str]) -> ReplyCheck:
    """
    Check one model reply: one JSON object, exactly the expected keys, finite numbers.

    Strict on purpose: no text before or after the object (the grammar
    already forbids it, so any is a server-side surprise worth seeing), no
    repeated keys, no booleans (true is an int to Python), no NaN/Infinity
    tokens, no number too large to be finite. Returns the values as floats,
    in expected_names order, or None and the reason.
    """
    try:
        parsed = json.loads(content, object_pairs_hook=_object_without_repeated_keys)
    except ValueError as e:     # JSONDecodeError is a ValueError, so is a repeated key
        return ReplyCheck(None, f'not one JSON value ({e}): {content[:120]!r}')
    if not isinstance(parsed, dict):
        return ReplyCheck(None, f'top level is a {type(parsed).__name__}, not an object')
    missing = sorted(set(expected_names) - set(parsed))
    unexpected = sorted(set(parsed) - set(expected_names))
    if missing or unexpected:
        return ReplyCheck(
            None, f'key set differs: missing {missing}, unexpected {unexpected}')
    values = {}
    for name in expected_names:
        value = parsed[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return ReplyCheck(None, f'{name}: {value!r} is not a number')
        try:
            number = float(value)
        except OverflowError:   # an int literal beyond float range
            number = math.inf
        if not math.isfinite(number):
            return ReplyCheck(None, f'{name}: {value!r} is not finite')
        values[name] = number
    return ReplyCheck(values, '')


class Completion(NamedTuple):
    """The outcome of one request (request_params())."""

    outcome: str                         # OK, INVALID, TIMEOUT or ERROR
    reason: str                          # '' for OK
    values: Optional[Dict[str, float]]   # the validated reply; None unless OK
    echo_match: Optional[bool]           # reply == sent; None when there was no reply
    latency_ms: float                    # wall time of the HTTP call
    timings: Dict[str, Optional[float]]  # TIMING_KEYS from the server's reply
    tokens_cached: Optional[int]         # prompt tokens the server reused


def completion_url(server_url: str) -> str:
    """Return llama-server's /completion endpoint under server_url."""
    return server_url.rstrip('/') + '/completion'


def request_params(url: str, params: Dict[str, float], timeout_s: float,
                   n_predict: int, post=requests.post,
                   clock=time.monotonic) -> Completion:
    """
    POST one tick's request and classify the outcome. Never raises for a server problem.

    timeout_s is requests' timeout, which bounds the connect and then each
    wait for bytes. llama-server sends nothing until a non-streamed
    completion is done, so in practice it bounds the whole request.

    TIMEOUT is caught before the other request errors: requests' ConnectTimeout
    is both a Timeout and a ConnectionError.
    """
    body = build_request_body(params, n_predict)
    no_timings = {key: None for key in TIMING_KEYS}
    t0 = clock()
    try:
        response = post(url, json=body, timeout=timeout_s)
    except requests.exceptions.Timeout:
        return Completion(TIMEOUT, f'no reply within {timeout_s:.1f} s', None, None,
                          (clock() - t0) * 1000.0, no_timings, None)
    except requests.exceptions.RequestException as e:
        return Completion(ERROR, f'{type(e).__name__}: {e}', None, None,
                          (clock() - t0) * 1000.0, no_timings, None)
    latency_ms = (clock() - t0) * 1000.0
    if response.status_code != 200:
        # 503 while the server is still loading the model, for one.
        return Completion(ERROR, f'HTTP {response.status_code}: {response.text[:200]!r}',
                          None, None, latency_ms, no_timings, None)
    try:
        reply = response.json()
        content = reply['content']
    except (ValueError, KeyError, TypeError) as e:
        return Completion(ERROR, f'server reply has no content ({type(e).__name__}: {e})',
                          None, None, latency_ms, no_timings, None)
    server_timings = reply.get('timings') or {}
    timings = {key: server_timings.get(key) for key in TIMING_KEYS}
    tokens_cached = reply.get('tokens_cached')
    check = validate_reply(content, list(params))
    if check.values is None:
        return Completion(INVALID, check.reason, None, False, latency_ms, timings,
                          tokens_cached)
    return Completion(OK, '', check.values, check.values == params, latency_ms, timings,
                      tokens_cached)


class GateState(NamedTuple):
    """RequestGate.try_start()'s or count_paused()'s answer, taken under one lock."""

    started: bool              # False: the tick is skipped or paused (and was counted)
    in_flight: Optional[int]   # seq of the request running at a tick that did not start one
    counters: Dict[str, int]


class RequestGate:
    """
    At most one request in flight; a tick that finds one running is skipped and counted.

    The LLM2 server has one slot, and a queued request would answer a tick
    that is already stale, so a skipped tick is dropped, never deferred.
    Thread-safe: try_start() and count_paused() run on the executor (the
    timer callback), finish() on the worker thread that ran the request.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._in_flight = None
        self._counters = {key: 0 for key in OUTCOMES + ('echo_match',)}

    def try_start(self, seq: int) -> GateState:
        """Claim the gate for request seq, or count the tick as skipped."""
        with self._lock:
            if self._in_flight is not None:
                self._counters[SKIPPED] += 1
                return GateState(False, self._in_flight, dict(self._counters))
            self._in_flight = seq
            return GateState(True, None, dict(self._counters))

    def count_paused(self) -> GateState:
        """Count a tick of a paused node: it never tries the gate, and is not a skip."""
        with self._lock:
            self._counters[PAUSED] += 1
            return GateState(False, self._in_flight, dict(self._counters))

    def finish(self, outcome: str, echo_match: Optional[bool] = None) -> Dict[str, int]:
        """Release the gate after a request; count its outcome. Returns the counters."""
        with self._lock:
            self._counters[outcome] += 1
            if echo_match:
                self._counters['echo_match'] += 1
            self._in_flight = None
            return dict(self._counters)

    @property
    def in_flight(self) -> Optional[int]:
        """Seq of the request in flight, or None."""
        with self._lock:
            return self._in_flight
