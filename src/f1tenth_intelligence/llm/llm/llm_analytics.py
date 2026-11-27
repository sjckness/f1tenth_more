r"""Record and analyse the traffic llm_planner_node sends to llama-server.

WHAT THIS IS
------------
A recording reverse proxy, not an instrumented node. It listens on a second
localhost port, the planner node is pointed at it with one ROS parameter, and
every byte of every /completion exchange is written down on the way through:

    llm_planner_node --> :8084 llm_analytics --> :8083 llama-server

The proxy captures the payload it forwards, so what lands in the log IS the
request the node built -- not a re-serialisation of a dict some hook happened
to see. On the response side the same holds for `content`, `timings` and
`generation_settings`, which is why this sits *outside* the node: the raw model
output is recorded before plan_translate.py's JSON airlock has had a chance to
accept or reject it, which is exactly the string worth looking at when a plan
comes out wrong.

Nothing in the node, the translator, llm.launch.py or models.yaml changes.

WHAT IT ANSWERS
---------------
1. Input      -- the exact prompt string, split into system prompt / command /
                 retry feedback, plus the request's generation parameters.
2. Timing     -- prompt-eval and generation tokens/second and milliseconds from
                 llama-server's own `timings`, the wall clock the proxy measured,
                 and the difference between the two (HTTP + queueing).
3. Thought    -- the model's raw output verbatim, whether it parses as JSON, how
                 much trailing text the node's raw_decode() had to discard, and
                 (with --airlock) what plan_translate.translate() says about it.
4. Load       -- n_predict requested vs applied, how much of that budget the
                 answer used, prompt-vs-decode shares of the server time, KV
                 cache occupancy from /slots and /metrics, and the server
                 process's RSS from /proc.

Usage
-----
    # terminal 1: the recorder runs the node itself (dry-run unless --live)
    ros2 run llm llm_analytics --exec 'vai dritto al muro e gira a destra'
    # or watch a node started by hand -- the recorder prints the exact command
    ros2 run llm llm_analytics
    ros2 run llm llm_planner_node 'vai dritto' --dry-run \
        --ros-args -p llm_url:=http://127.0.0.1:8084/completion

    # re-analyse a previous capture with no server and no ROS at all
    ros2 run llm llm_analytics --replay llm_analytics.jsonl

    # what does THIS llama-server build actually return?
    ros2 run llm llm_analytics --probe

    # sweep a file of commands through the node's own request, node not started
    ros2 run llm llm_analytics --exec-file commands.txt

    # --out report.json also writes report.md beside it, in the same folder

Deliberate limits
-----------------
* /metrics is only served when llama-server was started with --metrics, and
  llm.launch.py does not pass it. The poller therefore tries once, gives up
  permanently and says so; /slots and RSS still work, and the report never
  invents a number it could not read.
* GPU utilisation is not measured (no nvidia-smi dependency). KV-cache
  occupancy and RSS are the proxy for how hard the box is working.
* The proxy adds one localhost hop, a few hundred microseconds against a
  generation that takes seconds.
* --airlock predicts the verdict of one attempt, with go_to_enabled as
  configured here; the node's retry loop is still the node's business.
* --exec-file sends the node's v2 request, which is its default planner_path,
  and only that. It never starts the node, never reaches a mission service, and
  does not reproduce the legacy SYSTEM_PROMPT path: that prompt pairs with
  phases_to_mission(), so a sweep of it would be measured by an airlock built
  for the other vocabulary and every verdict would be a rejection.

Run standalone: python3 -m llm.llm_analytics --help
"""

import argparse
import datetime
import hashlib
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import re
import shlex
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request


# ---------------------------------------------------------------------------
# Defaults and the node's own spellings
# ---------------------------------------------------------------------------
DEFAULT_UPSTREAM = 'http://127.0.0.1:8083'
DEFAULT_LISTEN = '127.0.0.1:8084'
DEFAULT_OUT = 'llm_analytics.json'
DEFAULT_EVENTS = 'llm_analytics.jsonl'

# Copied from llm_planner_node._completion() and get_intent_from_llm(). If the
# node ever changes how it wraps the command, parse_prompt() stops matching and
# says so (`well_formed: false`) instead of silently mis-splitting a prompt --
# which is why these live here as named constants and not inline.
PROMPT_HEAD = '\ncomando: "'
PROMPT_TAIL = '"\nrisposta:'
RETRY_MARK = "\n(il tentativo precedente e' stato rifiutato: "
RETRY_TAIL = '. Correggi e rispondi di nuovo solo con il JSON.)'

# The rest of the request _completion() builds: the token ceiling, the
# temperature, and the schema that makes llama-server stop at the closing brace.
# Copied for the same reason as the two constants above -- importing the node
# would pull rclpy into a tool that must also run with no ROS at all (--replay,
# --probe). A drift is not silent: every record carries `request.params` as it
# was really sent, so an --exec capture and an --exec-file one are compared
# field by field rather than taken on trust.
N_PREDICT = 512
TEMPERATURE = 0.0
INTENT_REQUEST_SCHEMA = {
    'type': 'object',
    'properties': {
        'plan': {'type': 'array', 'items': {'type': 'object'}},
        'unsupported': {'type': 'array', 'items': {'type': 'string'}},
    },
    'required': ['plan', 'unsupported'],
}

# The readiness pass (llm_planner_node._wait_for_llama_server) posts this exact
# prompt with n_predict=8 once per node start. Recognising it keeps the warm-up
# latency out of the per-command averages instead of hiding it inside them.
WARMUP_PROMPT = 'ciao'

# The node's machine-readable outcome line (llm_planner_node._emit_result).
RESULT_PREFIX = 'RESULT '

# Response headers worth passing back. Everything else is either hop-by-hop,
# recomputed (Content-Length) or this server's own (Server, Date).
FORWARDED_HEADERS = frozenset(('content-type', 'content-encoding'))

# The three kinds of exchange this proxy can see.
KIND_WARMUP = 'warmup'
KIND_COMPLETION = 'completion'
KIND_OTHER = 'other'
KINDS = (KIND_WARMUP, KIND_COMPLETION, KIND_OTHER)


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def _now_iso(when=None):
    """Local time as ISO-8601 with milliseconds, for human-readable logs.

    Accepts a datetime, a POSIX timestamp (what time.time() gives, which is what
    every caller has) or nothing for "now".
    """
    if when is None:
        moment = datetime.datetime.now().astimezone()
    elif isinstance(when, (int, float)) and not isinstance(when, bool):
        moment = datetime.datetime.fromtimestamp(when).astimezone()
    else:
        moment = when
    return moment.isoformat(timespec='milliseconds')


def _sha256(text):
    """sha256 of a string, or None. Lets two prompts be compared cheaply."""
    if not isinstance(text, str):
        return None
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def _number(value):
    """float(value) when it is a real number, else None. Booleans are not numbers.

    Every field read from llama-server goes through this: a build that reports
    a value as a string, or omits it, must degrade to null rather than raise.
    """
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _integer(value):
    """int(value) when it is an integer-valued number, else None."""
    number = _number(value)
    return None if number is None else int(number)


def _to_int(value):
    """int() of a command-line string or a number, or None.

    Separate from _integer(): an argv token is the string '8083', which is not
    a number and must not be silently discarded as one.
    """
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


def _ratio(part, whole):
    """Return part / whole, or None when whole is zero/None."""
    if part is None or not whole:
        return None
    return part / whole


def _round(value, digits=2):
    """round() that tolerates None."""
    return None if value is None else round(value, digits)


def _decode_json(payload):
    """Parse bytes/str JSON, returning None instead of raising.

    A response that is not JSON is a real and interesting outcome here (a 502
    from this proxy, an error page from somewhere else), so it must not be an
    exception that throws away the rest of the record.
    """
    if payload is None:
        return None
    if isinstance(payload, bytes):
        try:
            payload = payload.decode('utf-8')
        except UnicodeDecodeError:
            return None
    if not isinstance(payload, str) or not payload.strip():
        return None
    try:
        return json.loads(payload)
    except ValueError:
        return None


def _truncate(text, limit):
    """Cut `text` to `limit` characters, marking the cut. limit 0 means no cut."""
    if not isinstance(text, str) or not limit or len(text) <= limit:
        return text
    return text[:limit] + f'... [{len(text) - limit} chars cut]'


def _split_url(url):
    """Split an http URL into (host, port, path-prefix)."""
    parts = urllib.parse.urlsplit(url)
    if parts.scheme not in ('http', ''):
        raise ValueError(f'only http:// upstreams are supported, got {url!r}')
    if not parts.hostname:
        raise ValueError(f'no host in {url!r}')
    return parts.hostname, parts.port or 80, parts.path.rstrip('/')


def _split_listen(value):
    """Split 'host:port' (the --listen value) into (host, port)."""
    host, _, port = value.rpartition(':')
    if not host or not port:
        raise ValueError(f'--listen needs HOST:PORT, got {value!r}')
    try:
        return host, int(port)
    except ValueError:
        raise ValueError(f'port in --listen is not a number: {value!r}') from None


# ---------------------------------------------------------------------------
# 1. The input: the exact prompt and the request's own parameters
# ---------------------------------------------------------------------------
def parse_prompt(prompt):
    r"""Split a /completion prompt into the node's three parts.

    llm_planner_node builds exactly one shape:

        <system prompt>\ncomando: "<command>"\nrisposta:

    and, on a translator rejection, wraps the command instead of the prompt:

        <command>\n(il tentativo precedente e' stato rifiutato: <validator
        text>. Correggi e rispondi di nuovo solo con il JSON.)

    rpartition on the last marker, not split(): the system prompt is a file and
    may legitimately contain the word "comando". What comes back as `command`
    is what the node put between the quotes, byte for byte, with the retry
    suffix split out separately into `retry_feedback`.
    """
    out = {
        'prompt': prompt,
        'prompt_chars': len(prompt) if isinstance(prompt, str) else None,
        'well_formed': False,
        'system_prompt_chars': None,
        'system_prompt_sha256': None,
        'command': None,
        'retry_feedback': None,
    }
    if not isinstance(prompt, str):
        return out
    head, separator, tail = prompt.rpartition(PROMPT_HEAD)
    if not separator or not tail.endswith(PROMPT_TAIL):
        return out
    command_block = tail[:-len(PROMPT_TAIL)]
    command, retry_separator, retry_text = command_block.partition(RETRY_MARK)
    feedback = None
    if retry_separator:
        feedback = retry_text
        if feedback.endswith(RETRY_TAIL):
            feedback = feedback[:-len(RETRY_TAIL)]
    out.update({
        'well_formed': True,
        'system_prompt_chars': len(head),
        'system_prompt_sha256': _sha256(head),
        'command': command,
        'retry_feedback': feedback,
    })
    return out


def request_parameters(request_obj):
    """Return the generation parameters the node actually sent, and nothing else."""
    if not isinstance(request_obj, dict):
        return {'n_predict': None, 'temperature': None, 'stop': None,
                'json_schema': False, 'prompt_chars': None}
    prompt = request_obj.get('prompt')
    return {
        'n_predict': _integer(request_obj.get('n_predict')),
        'temperature': _number(request_obj.get('temperature')),
        'stop': request_obj.get('stop'),
        'json_schema': isinstance(request_obj.get('json_schema'), dict),
        'prompt_chars': len(prompt) if isinstance(prompt, str) else None,
    }


def classify_exchange(request_obj, path=''):
    """Classify an exchange as warmup / completion / other, from the payload.

    The warm-up is identified by its prompt because that is the only thing that
    distinguishes it: same endpoint, same content type, one eighth the budget.
    """
    if isinstance(request_obj, dict):
        prompt = request_obj.get('prompt')
        if isinstance(prompt, str):
            if prompt.strip() == WARMUP_PROMPT:
                return KIND_WARMUP
            if PROMPT_HEAD in prompt:
                return KIND_COMPLETION
    if path.startswith('/completion'):
        return KIND_COMPLETION
    return KIND_OTHER


# ---------------------------------------------------------------------------
# 3a. The thought flow: the raw model output, before the airlock
# ---------------------------------------------------------------------------
def parse_thought(content):
    """Describe the model's raw output without interpreting it.

    `raw_output` is stored verbatim and is the point of the exercise: it is the
    string llm_planner_node hands to json.JSONDecoder().raw_decode() *before*
    plan_translate.py sees anything. `trailing_chars` counts what raw_decode
    had to discard -- the model sometimes keeps writing comando:/risposta:
    pairs after the closing brace, and knowing how often is how you tell
    whether the json_schema or the prompt is doing the constraining.

    The same decoder the node uses (raw_decode, not json.loads), so that
    `json_valid` means exactly what the node would have done with this string.
    """
    raw = content if isinstance(content, str) else ''
    out = {
        'raw_output': raw,
        'raw_chars': len(raw),
        'raw_sha256': _sha256(raw),
        'empty': not raw.strip(),
        'json_valid': False,
        'json_value_type': None,
        'trailing_chars': None,
        'moves': None,
        'unsupported': None,
        'airlock': None,
    }
    stripped = raw.strip()
    if not stripped:
        return out
    try:
        value, end = json.JSONDecoder().raw_decode(stripped)
    except ValueError:
        return out
    out['json_valid'] = True
    out['json_value_type'] = type(value).__name__
    out['trailing_chars'] = len(stripped[end:].strip())
    if isinstance(value, dict):
        if isinstance(value.get('plan'), list):
            out['moves'] = len(value['plan'])
        if isinstance(value.get('unsupported'), list):
            out['unsupported'] = len(value['unsupported'])
    elif isinstance(value, list):
        out['moves'] = len(value)
    return out


def airlock_verdict(content, go_to_enabled=True):
    """Ask plan_translate.translate() what it makes of this raw output.

    Returns (verdict, warning). The verdict is a string rather than an exception
    because it goes into a JSONL file and gets grepped:
    'accepted:3 moves' or 'rejected:IntentSchemaError: <message>'.

    Imported lazily and guarded: the airlock is a convenience, and a machine
    where the package or jsonschema cannot be imported must still be able to
    record traffic. translate() is called exactly as llm_planner_node calls it,
    on the same value raw_decode would have produced.
    """
    try:
        from llm.plan_translate import translate
    except Exception as error:  # noqa: BLE001 -- an unavailable airlock is not fatal
        return None, f'airlock unavailable ({type(error).__name__}: {error})'

    thought = parse_thought(content)
    if not thought['json_valid']:
        return 'rejected:JSONDecodeError: no JSON value in the model output', None
    value, _ = json.JSONDecoder().raw_decode(content.strip())
    if not isinstance(value, dict):
        return (f'rejected:ValueError: intent is a {type(value).__name__}, '
                'not an object'), None
    try:
        result = translate(value, go_to_enabled=go_to_enabled)
    except Exception as error:  # noqa: BLE001 -- every rejection is a verdict
        return f'rejected:{type(error).__name__}: {_truncate(str(error), 300)}', None
    suffix = ''
    rejected = len(result.unsupported or ())
    if rejected:
        suffix += f', {rejected} unsupported'
    if result.requires_confirmation:
        suffix += ', requires_confirmation'
    moves = len(result.mission.get('moves') or ())
    return f'accepted:{moves} moves{suffix}', None


# ---------------------------------------------------------------------------
# 2 + 4. One exchange, fully described
# ---------------------------------------------------------------------------
# Top-level keys a llama.cpp /completion response is known to carry. Used only
# to flag drift on the first completion of a session: a new or renamed field is
# worth seeing once, not once per call.
KNOWN_RESPONSE_KEYS = frozenset({
    'content', 'generation_settings', 'model', 'prompt', 'stop', 'stop_type',
    'stopped_eos', 'stopped_limit', 'stopped_word', 'stopping_word', 'timings',
    'tokens_cached', 'tokens_evaluated', 'tokens_predicted', 'truncated',
    'id_slot', 'slot_id', 'multibyte_fix', 'has_new_line', 'n_prompt_tokens',
    'next_token', 'error', 'message',
})


def summarise_timings(timings, wall_ms=None):
    """llama-server's own numbers, plus the gap to the wall clock we measured.

    `prompt_per_second` and `predicted_per_second` are reported by the server;
    the fallbacks divide by the server's own milliseconds so a build that omits
    the derived fields still yields a speed. `overhead_ms` is everything the
    node waited for that the server did not spend generating: connection setup,
    queueing behind another slot, JSON encoding, the loopback hop.
    """
    timings = timings if isinstance(timings, dict) else {}
    prompt_n = _integer(timings.get('prompt_n'))
    prompt_ms = _number(timings.get('prompt_ms'))
    predicted_n = _integer(timings.get('predicted_n'))
    predicted_ms = _number(timings.get('predicted_ms'))
    prompt_tps = _number(timings.get('prompt_per_second'))
    if prompt_tps is None and prompt_ms:
        prompt_tps = _ratio(prompt_n, prompt_ms / 1000.0)
    predicted_tps = _number(timings.get('predicted_per_second'))
    if predicted_tps is None and predicted_ms:
        predicted_tps = _ratio(predicted_n, predicted_ms / 1000.0)
    server_ms = None
    if prompt_ms is not None and predicted_ms is not None:
        server_ms = prompt_ms + predicted_ms
    overhead_ms = None
    if server_ms is not None and wall_ms is not None:
        overhead_ms = wall_ms - server_ms
    return {
        'prompt_n': prompt_n,
        'prompt_ms': _round(prompt_ms, 1),
        'prompt_per_second': _round(prompt_tps, 1),
        'predicted_n': predicted_n,
        'predicted_ms': _round(predicted_ms, 1),
        'predicted_per_second': _round(predicted_tps, 1),
        'server_ms': _round(server_ms, 1),
        'overhead_ms': _round(overhead_ms, 1),
        'prompt_share': _round(_ratio(prompt_ms, server_ms), 4),
        'decode_share': _round(_ratio(predicted_ms, server_ms), 4),
    }


def summarise_load(parameters, response_obj, thought, timings):
    """How much of the generation budget this answer used, and on what.

    `budget_used` is tokens generated divided by the n_predict the server
    actually applied -- 1.0 means the answer was cut off by the cap rather than
    finished, which is the difference between "the model was concise" and "the
    model was still talking when the budget ran out".
    """
    generation = response_obj.get('generation_settings')
    generation = generation if isinstance(generation, dict) else {}
    applied = _integer(generation.get('n_predict'))
    predicted_n = _integer(response_obj.get('tokens_predicted'))
    if predicted_n is None:
        predicted_n = timings.get('predicted_n')
    budget_used = _ratio(predicted_n, applied)
    stop_type = response_obj.get('stop_type')
    truncated = response_obj.get('truncated')
    hit_limit = bool(truncated) or stop_type == 'limit'
    if not hit_limit and applied and predicted_n is not None:
        hit_limit = predicted_n >= applied
    return {
        'n_predict_requested': parameters.get('n_predict'),
        'n_predict_applied': applied,
        'tokens_cached': _integer(response_obj.get('tokens_cached')),
        'budget_used': _round(budget_used, 3),
        'hit_token_limit': hit_limit,
        'stop_type': stop_type,
        'truncated': truncated,
        'eos': response_obj.get('stopped_eos'),
        'json_constrained': parameters.get('json_schema'),
        'moves': thought.get('moves'),
    }


def analyse_exchange(index, kind, request_body, response_status, response_body,
                     wall_ms, *, path='/completion', t_start=None, sample=None,
                     go_to_enabled=True, airlock=False, occurrence=1):
    """Build the complete record for one request/response pair.

    Pure by design -- bytes in, dict out, no sockets and no clock unless asked
    -- so the whole analysis path is testable against a canned llama.cpp
    response without starting a server.

    Nothing here raises on unexpected input: a response that is not JSON, a
    missing `timings`, a renamed field all become nulls and, where it matters,
    a warning. The alternative is a recorder that dies on the exact anomaly it
    was deployed to observe.
    """
    request_obj = _decode_json(request_body)
    parameters = request_parameters(request_obj)
    prompt = request_obj.get('prompt') if isinstance(request_obj, dict) else None
    parsed_prompt = parse_prompt(prompt)
    parsed_prompt['occurrence'] = occurrence
    parsed_prompt['is_retry'] = bool(parsed_prompt.get('retry_feedback'))

    response_obj = _decode_json(response_body)
    response_obj = response_obj if isinstance(response_obj, dict) else {}
    content = response_obj.get('content')
    thought = parse_thought(content)
    warnings = []
    if airlock:
        verdict, warning = airlock_verdict(content, go_to_enabled)
        thought['airlock'] = verdict
        if warning:
            warnings.append(warning)
    if not response_obj:
        warnings.append('response body is not a JSON object')
    unknown = sorted(set(response_obj) - KNOWN_RESPONSE_KEYS)

    timings = summarise_timings(response_obj.get('timings'), wall_ms)
    load = summarise_load(parameters, response_obj, thought, timings)
    tokens = {
        'evaluated': _integer(response_obj.get('tokens_evaluated')),
        'predicted': _integer(response_obj.get('tokens_predicted')),
        'cached': _integer(response_obj.get('tokens_cached')),
    }
    memory = dict(sample) if isinstance(sample, dict) else {}

    request_bytes = request_body or b''
    return {
        'index': index,
        'kind': kind,
        't_start': _now_iso(t_start),
        'wall_ms': _round(wall_ms, 1),
        'request': {
            'path': path,
            'body_chars': len(request_bytes),
            'body_sha256': hashlib.sha256(request_bytes).hexdigest(),
            'params': parameters,
        },
        'input': parsed_prompt,
        'response': {
            'http_status': response_status,
            'error': response_obj.get('error'),
            'model': response_obj.get('model'),
            'stop_type': response_obj.get('stop_type'),
            'truncated': response_obj.get('truncated'),
            'slot_id': response_obj.get('slot_id', response_obj.get('id_slot')),
            'generation_settings': response_obj.get('generation_settings'),
            'unknown_keys': unknown,
        },
        'tokens': tokens,
        'timings': timings,
        'thought': thought,
        'load': load,
        'memory': memory,
        'warnings': warnings,
    }


# ---------------------------------------------------------------------------
# 4b. The server itself: KV cache, /proc and the Prometheus endpoint
# ---------------------------------------------------------------------------
_METRIC_LINE = re.compile(
    r'^(?P<name>[A-Za-z_:][A-Za-z0-9_:]*)(?P<labels>\{[^}]*\})?[ \t]+'
    r'(?P<value>[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?'
    r'|[-+]?Inf|NaN)')


def parse_prometheus(text):
    """Flatten a Prometheus exposition into {metric_name: last float value}.

    llama.cpp's /metrics is unlabelled today (llamacpp:kv_cache_usage_ratio,
    llamacpp:tokens_predicted_total, ...), so a flat dict is enough. Labelled
    series keep their last sample: good enough for a gauge, and the alternative
    -- averaging histograms nobody asked for -- would be a lie dressed up as a
    number.
    """
    metrics = {}
    if not isinstance(text, str):
        return metrics
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        match = _METRIC_LINE.match(line)
        if not match:
            continue
        try:
            metrics[match.group('name')] = float(match.group('value'))
        except ValueError:
            continue
    return metrics


def metric_value(metrics, suffix):
    """Value of the first metric whose name ends with `suffix`, else None.

    Matched by suffix because the family prefix has moved between llama.cpp
    versions (llamacpp: vs llama_, and a build may drop the prefix entirely).
    """
    if not isinstance(metrics, dict):
        return None
    for name, value in metrics.items():
        if name.endswith(suffix):
            return value
    return None


def kv_from_slots(slots):
    """KV-cache occupancy and busy slots, from llama-server's /slots array.

    `n_ctx` is the slot's own context size and `n_past` how much of it this
    conversation currently occupies, which is the only per-slot figure a
    default build exposes. Newer builds nest both under `next_token`, so both
    shapes are read. tokens staying None (no slot reported occupancy at all) is
    NOT the same as 0: an unknown is reported as an unknown, never as an empty
    cache.
    """
    tokens = None
    capacity = 0
    processing = 0
    if not isinstance(slots, list):
        return {}
    for slot in slots:
        if not isinstance(slot, dict):
            continue
        state = slot.get('next_token')
        state = state if isinstance(state, dict) else {}
        context = _integer(slot.get('n_ctx'))
        if context is None:
            context = _integer(state.get('n_ctx'))
        if context:
            capacity += context
        used = None
        for holder in (slot, state):
            for key in ('n_past', 'prompt_n', 'n_decoded'):
                used = _integer(holder.get(key))
                if used is not None:
                    break
            if used is not None:
                break
        if used is not None:
            tokens = used if tokens is None else tokens + used
        if slot.get('is_processing'):
            processing += 1
    return {
        'kv_tokens': tokens,
        'kv_capacity': capacity or None,
        'kv_ratio': _round(_ratio(tokens, capacity), 4),
        'processing_slots': processing,
    }


def _read_cmdline(path):
    """Read argv of a process from /proc/<pid>/cmdline, or [] if it is gone."""
    try:
        with open(path, 'rb') as handle:
            raw = handle.read()
    except OSError:
        return []
    return [part.decode('utf-8', 'replace') for part in raw.split(b'\0') if part]


def _is_llama_server(argv):
    """Say whether this argv looks like llama-server, wherever the binary lives."""
    if not argv:
        return False
    candidates = [os.path.basename(argv[0])] + [os.path.basename(a) for a in argv[1:3]]
    return any(name.startswith('llama-server') for name in candidates)


def _port_from_argv(argv):
    """Return the --port an argv asked for, or None when it did not say."""
    for index, arg in enumerate(argv):
        if arg.startswith('--port='):
            return _to_int(arg.split('=', 1)[1])
        if arg == '--port' and index + 1 < len(argv):
            return _to_int(argv[index + 1])
    return None


def _listening_inodes(port, proc_root='/proc'):
    """Socket inodes listening on `port`, from the kernel's own tables."""
    inodes = set()
    for table in ('net/tcp', 'net/tcp6'):
        path = os.path.join(proc_root, table)
        try:
            with open(path, 'r', encoding='utf-8', errors='replace') as handle:
                lines = handle.readlines()
        except OSError:
            continue
        for line in lines[1:]:
            fields = line.split()
            # sl local_address rem_address st ... uid timeout inode
            if len(fields) < 10 or fields[3] != '0A':         # 0A == LISTEN
                continue
            _, _, hex_port = fields[1].rpartition(':')
            try:
                if int(hex_port, 16) != int(port):
                    continue
            except ValueError:
                continue
            inodes.add(fields[9])
    return inodes


def _pid_by_socket(port, proc_root='/proc'):
    """PID owning a socket listening on `port`, by scanning /proc/<pid>/fd."""
    wanted = {f'socket:[{inode}]' for inode in _listening_inodes(port, proc_root)}
    if not wanted:
        return None
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return None
    for entry in entries:
        if not entry.isdigit():
            continue
        fd_dir = os.path.join(proc_root, entry, 'fd')
        try:
            descriptors = os.listdir(fd_dir)
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(os.path.join(fd_dir, descriptor))
            except OSError:
                continue
            if target in wanted:
                return int(entry)
    return None


def find_server_pid(port, proc_root='/proc'):
    """PID of the llama-server serving `port`, or None.

    Command line first -- it names both the binary and the port, so it cannot
    pick the wrong process -- and the listening-socket scan only as a fallback
    for a wrapper that execs llama-server under another name. Both are plain
    /proc reads: no psutil, no nvidia-smi, nothing to install on the car.
    """
    try:
        entries = sorted(os.listdir(proc_root))
    except OSError:
        return None
    fallback = None
    for entry in entries:
        if not entry.isdigit():
            continue
        argv = _read_cmdline(os.path.join(proc_root, entry, 'cmdline'))
        if not _is_llama_server(argv):
            continue
        declared = _port_from_argv(argv)
        if declared == port:
            return int(entry)
        # llama.cpp's own default port is 8080, so an argv without --port is a
        # match for that port only.
        if fallback is None and declared is None and port == 8080:
            fallback = int(entry)
    if fallback is not None:
        return fallback
    return _pid_by_socket(port, proc_root)


def read_proc_memory(pid, proc_root='/proc'):
    """RSS, peak RSS and address space of a PID, in MB, from /proc/<pid>/status."""
    path = os.path.join(proc_root, str(pid), 'status')
    fields = {'VmRSS': 'rss_mb', 'VmHWM': 'rss_peak_mb', 'VmSize': 'vm_size_mb'}
    out = {}
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            lines = handle.readlines()
    except OSError:
        return out
    for line in lines:
        key, _, rest = line.partition(':')
        field = fields.get(key.strip())
        if not field:
            continue
        parts = rest.split()
        if not parts:
            continue
        try:
            out[field] = round(float(parts[0]) / 1024.0, 1)
        except ValueError:
            continue
    return out


def summarise_props(props):
    """Extract the bits of /props worth putting in a report header."""
    if not isinstance(props, dict):
        return {}
    generation = props.get('default_generation_settings')
    generation = generation if isinstance(generation, dict) else {}
    n_ctx = generation.get('n_ctx')
    if n_ctx is None:
        n_ctx = props.get('n_ctx')
    return {
        'model_path': props.get('model_path'),
        'n_ctx': _integer(n_ctx),
        'total_slots': _integer(props.get('total_slots')),
        'build_info': props.get('build_info'),
    }


class MemoryPoller(threading.Thread):
    """Samples llama-server's own state on a timer, off the request path.

    Every call is wrapped: a poller that dies, or a server that answers 404,
    must cost the recording nothing. That is the whole reason this is a
    separate thread with its own timeouts rather than work done per request.
    """

    # 0.5 s over ~2.8 hours; beyond that the oldest samples are dropped. A run
    # long enough to matter is one where per-request samples exist anyway.
    MAX_SAMPLES = 20000

    def __init__(self, upstream, port=None, interval=0.5, timeout=2.0,
                 proc_root='/proc', on_warning=None):
        super().__init__(name='llm-analytics-memory', daemon=True)
        self.upstream = upstream.rstrip('/')
        self.port = port
        self.interval = max(0.05, float(interval))
        self.timeout = float(timeout)
        self.proc_root = proc_root
        self.on_warning = on_warning or (lambda message: None)
        self.props = {}
        self.metrics = {}
        self.metrics_first = {}
        self.metrics_enabled = True
        self.pid = None
        self.samples = []
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._warned = set()

    def run(self):
        """Poll until stop() is called. Never raises out of the thread."""
        self.props = self._get_json('/props')
        while not self._stop_event.is_set():
            try:
                self._sample_once()
            except Exception as error:  # noqa: BLE001 -- polling must not escape
                self._warn(f'memory poll failed ({type(error).__name__}: {error})')
            self._stop_event.wait(self.interval)

    def stop(self):
        """Ask the thread to finish and wait for it, briefly."""
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=max(1.0, self.interval * 4))

    def _warn(self, message):
        """Record a warning once and forward it to the session's reporter."""
        if message in self._warned:
            return
        self._warned.add(message)
        self.on_warning(message)

    def _get(self, path):
        """GET one endpoint: (body, status, error). Never raises."""
        try:
            with urllib.request.urlopen(self.upstream + path,
                                        timeout=self.timeout) as response:
                return response.read(), response.status, None
        except urllib.error.HTTPError as error:
            return None, error.code, None
        except Exception as error:  # noqa: BLE001 -- polling must not escape
            return None, None, f'{type(error).__name__}: {error}'

    def _get_json(self, path):
        """GET one endpoint and parse it, or None."""
        body, _, _ = self._get(path)
        return None if body is None else _decode_json(body)

    def _sample_once(self):
        """One row of the memory series: KV cache, metrics and process RSS."""
        sample = {'t': time.time(), 'at': _now_iso()}
        sample.update(kv_from_slots(self._get_json('/slots')))
        if self.metrics_enabled:
            self._sample_metrics(sample)
        if self.pid is None and self.port is not None:
            self.pid = find_server_pid(self.port, self.proc_root)
        if self.pid is not None:
            sample.update(read_proc_memory(self.pid, self.proc_root))
        with self._lock:
            self.samples.append(sample)
            if len(self.samples) > self.MAX_SAMPLES:
                del self.samples[0]

    def _sample_metrics(self, sample):
        """Read /metrics, or disable it permanently and say why."""
        body, status, error = self._get('/metrics')
        if body is None:
            self.metrics_enabled = False
            if status == 404:
                self._warn('llama-server has no /metrics (restart it with --metrics '
                           'for KV-cache and token counters); RSS and /slots are still '
                           'sampled')
            elif status is not None:
                self._warn(f'/metrics returned HTTP {status}; metrics polling disabled')
            elif error:
                self._warn(f'/metrics unreachable ({error}); metrics polling disabled')
            return
        parsed = parse_prometheus(body.decode('utf-8', 'replace'))
        if not parsed:
            self.metrics_enabled = False
            self._warn('/metrics returned nothing parseable; metrics polling disabled')
            return
        if not self.metrics_first:
            self.metrics_first = dict(parsed)
        self.metrics = parsed
        ratio = metric_value(parsed, 'kv_cache_usage_ratio')
        if ratio is not None:
            sample['kv_ratio_metrics'] = _round(ratio, 4)

    def snapshot(self, when=None):
        """Return the sample nearest `when` (or the latest), as a plain dict."""
        with self._lock:
            samples = list(self.samples)
        if not samples:
            return {}
        if when is None:
            return dict(samples[-1])
        return dict(min(samples, key=lambda sample: abs(sample['t'] - when)))

    def summary(self):
        """Aggregate RSS movement and KV-cache occupancy for the report."""
        with self._lock:
            samples = list(self.samples)
        out = {
            'samples': len(samples),
            'pid': self.pid,
            'metrics_available': bool(self.metrics),
            'props': summarise_props(self.props),
            'warnings': sorted(self._warned),
        }
        rss = [sample['rss_mb'] for sample in samples if sample.get('rss_mb')]
        if rss:
            out['rss_baseline_mb'] = rss[0]
            out['rss_peak_mb'] = max(rss)
            out['rss_last_mb'] = rss[-1]
            out['rss_delta_mb'] = round(rss[-1] - rss[0], 1)
        ratios = [sample['kv_ratio'] for sample in samples if sample.get('kv_ratio')]
        if ratios:
            out['kv_ratio_peak'] = max(ratios)
        if self.metrics:
            throughput = {
                name: round(value - self.metrics_first.get(name, value), 3)
                for name, value in self.metrics.items()
                if name.endswith('_total')
            }
            if throughput:
                out['metrics_delta'] = throughput
            out['kv_ratio_metrics_last'] = metric_value(
                self.metrics, 'kv_cache_usage_ratio')
        return out


# ---------------------------------------------------------------------------
# The recording session: proxy callback, event file, console
# ---------------------------------------------------------------------------
def _one_line(text, limit=70):
    """Flatten a string to one short line, for console output."""
    if not isinstance(text, str):
        return '-'
    flat = ' '.join(text.split())
    return flat if len(flat) <= limit else flat[:max(1, limit - 3)] + '...'


def _fmt(value, spec='.1f', suffix=''):
    """Format a number for the console, or a placeholder when it is None."""
    if value is None:
        return '-'
    return format(value, spec) + suffix


class Session:
    """One recording run: the records, the event file and the console.

    Thread-safe on the record list because ThreadingHTTPServer handles requests
    concurrently -- in practice the node is sequential, but the recorder is not
    allowed to depend on that.
    """

    def __init__(self, upstream, listen, *, airlock=False, go_to_enabled=True,
                 events_path=None, raw_chars=0, show_raw=True, verbose=False,
                 poller=None):
        self.upstream = upstream
        self.listen = listen
        self.airlock = airlock
        self.go_to_enabled = go_to_enabled
        self.events_path = events_path
        self.raw_chars = raw_chars
        self.show_raw = show_raw
        self.verbose = verbose
        self.poller = poller
        self.records = []
        self.warnings = []
        self.runs = []
        self.started = time.time()
        self.started_at = _now_iso()
        self._lock = threading.Lock()
        self._occurrences = {}
        self._events = None
        if events_path:
            # Truncated on purpose, like the JSON report: one file per run, so
            # a stale capture can never be mistaken for this one.
            self._events = open(events_path, 'w', encoding='utf-8')

    def warn(self, message):
        """Say something once, on stderr, and keep it for the report."""
        if message in self.warnings:
            return
        self.warnings.append(message)
        print(f'WARNING: {message}', file=sys.stderr, flush=True)

    def close(self):
        """Close the event file, if one was opened."""
        if self._events is not None:
            self._events.close()
            self._events = None

    def _occurrence(self, command):
        """How many times this exact command has been sent in this session."""
        if command is None:
            return 1
        count = self._occurrences.get(command, 0) + 1
        self._occurrences[command] = count
        return count

    def add(self, path, request_body, response_status, response_body, wall_ms,
            t_start=None, sample=None):
        """Record one exchange. Called from the proxy's request thread."""
        request_obj = _decode_json(request_body)
        prompt = request_obj.get('prompt') if isinstance(request_obj, dict) else None
        kind = classify_exchange(request_obj, path)
        command = parse_prompt(prompt).get('command')
        with self._lock:
            index = len(self.records) + 1
            record = analyse_exchange(
                index, kind, request_body, response_status, response_body, wall_ms,
                path=path, t_start=t_start, sample=sample,
                go_to_enabled=self.go_to_enabled, airlock=self.airlock,
                occurrence=self._occurrence(command))
            self.records.append(record)
            self._write_event(record)
        for warning in record['warnings']:
            self.warn(warning)
        self._print_exchange(record)
        return record

    def _write_event(self, record):
        """Append one JSON line and flush it: a crash must not cost the log."""
        if self._events is None:
            return
        self._events.write(json.dumps(record, ensure_ascii=False, default=str) + '\n')
        self._events.flush()

    def _print_exchange(self, record):
        """One compact console block per exchange, raw output included.

        The console is the point of `--exec`: an operator watching a run should
        not have to open the JSON to see what the model actually wrote.
        """
        index = record['index']
        timings = record['timings']
        response = record['response']
        thought = record['thought']
        kind = record['kind']
        wall = _fmt(record['wall_ms'])
        if kind == KIND_WARMUP:
            print(f'[{index:>3}] warmup   wall {wall} ms   '
                  f'prefill {_fmt(timings["prompt_per_second"])} t/s', flush=True)
            return
        if kind != KIND_COMPLETION:
            print(f'[{index:>3}] {kind:<8} wall {wall} ms   '
                  f'HTTP {response["http_status"]}', flush=True)
            return

        command = record['input'].get('command')
        occurrence = record['input'].get('occurrence') or 1
        header = f'[{index:>3}] command #{occurrence}  '
        header += '(prompt not recognised)' if command is None else repr(_one_line(command))
        print(header, flush=True)
        print(
            f'      wall {wall} ms  |  '
            f'prefill {_fmt(timings["prompt_n"], "d")} t @ '
            f'{_fmt(timings["prompt_per_second"])} t/s '
            f'({_fmt(timings["prompt_ms"])} ms)  |  '
            f'decode {_fmt(timings["predicted_n"], "d")} t @ '
            f'{_fmt(timings["predicted_per_second"])} t/s '
            f'({_fmt(timings["predicted_ms"])} ms)  |  '
            f'overhead {_fmt(timings["overhead_ms"])} ms', flush=True)
        load = record['load']
        print(
            f'      budget {_fmt(load["budget_used"], ".0%")} of '
            f'{_fmt(load["n_predict_applied"], "d")} tokens  '
            f'stop={load["stop_type"]}  hit_limit={load["hit_token_limit"]}  '
            f'prompt_tokens={_fmt(record["tokens"]["evaluated"], "d")}', flush=True)
        memory = record['memory']
        bits = []
        if memory.get('kv_ratio') is not None:
            bits.append(f'kv {memory["kv_ratio"]:.0%} '
                        f'({memory.get("kv_tokens")}/{memory.get("kv_capacity")} tokens)')
        elif memory.get('kv_ratio_metrics') is not None:
            bits.append(f'kv {memory["kv_ratio_metrics"]:.0%} (/metrics)')
        elif memory.get('kv_capacity') is not None:
            bits.append(f'kv unknown of {memory["kv_capacity"]} tokens ('
                        'no slot reported n_past)')
        if memory.get('rss_mb') is not None:
            bits.append(f'rss {memory["rss_mb"]:.0f} MiB '
                        f'(peak {_fmt(memory.get("rss_peak_mb"), ".0f")})')
        if bits:
            print('      ' + '  |  '.join(bits), flush=True)
        detail = (f'{thought["raw_chars"]} chars, '
                  f'{"json ok" if thought["json_valid"] else "NOT json"}')
        if thought['json_valid']:
            detail += (f', type={thought["json_value_type"]}'
                       f', moves={thought["moves"]}'
                       f', trailing={thought["trailing_chars"]}')
        print(f'      thought: {detail}', flush=True)
        if thought.get('airlock'):
            print(f'      airlock: {thought["airlock"]}', flush=True)
        feedback = record['input'].get('retry_feedback')
        if feedback:
            print(f'      retry feedback: {_one_line(feedback, 100)!r}', flush=True)
        if self.show_raw:
            raw = _truncate(thought['raw_output'], self.raw_chars)
            for line in (raw or '(empty)').splitlines() or ['(empty)']:
                print(f'      | {line}', flush=True)

    def report(self, extra=None):
        """Build the whole run as one JSON-serialisable dict."""
        with self._lock:
            records = list(self.records)
        payload = {
            'tool': 'llm_analytics',
            'session': {
                'started_at': self.started_at,
                'finished_at': _now_iso(),
                'duration_s': round(time.time() - self.started, 1),
                'upstream': self.upstream,
                'proxy': self.listen,
                'airlock': self.airlock,
                'go_to_enabled': self.go_to_enabled,
                'events_path': self.events_path,
                'warnings': list(self.warnings),
            },
            'summary': build_summary(records),
            'server': self.poller.summary() if self.poller is not None else {},
            'runs': list(self.runs),
            'exchanges': records,
        }
        if extra:
            payload['session'].update(extra)
        return payload


# ---------------------------------------------------------------------------
# The aggregate view
# ---------------------------------------------------------------------------
def _percentile(ordered, fraction):
    """Nearest-rank percentile of an already-sorted list."""
    if not ordered:
        return None
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[max(0, min(len(ordered) - 1, index))]


def _stats(values):
    """mean/median/p95/max of a list of numbers, or None when there is nothing."""
    ordered = sorted(value for value in values if value is not None)
    if not ordered:
        return None
    return {
        'n': len(ordered),
        'mean': round(statistics.fmean(ordered), 2),
        'median': round(statistics.median(ordered), 2),
        'p95': round(_percentile(ordered, 0.95), 2),
        'max': round(ordered[-1], 2),
    }


def _tally(items):
    """{item: count}, most frequent first then alphabetical."""
    counts = {}
    for item in items:
        counts[item] = counts.get(item, 0) + 1
    return dict(sorted(counts.items(), key=lambda pair: (-pair[1], pair[0])))


def build_summary(records):
    """Aggregate the exchanges into the numbers worth reading first.

    Warm-up exchanges are counted and timed separately: they are one call at
    node start against a cold server, so folding ~58 s of first-call latency
    into the per-command averages would misreport every run.
    """
    records = list(records or ())
    completions = [record for record in records if record['kind'] == KIND_COMPLETION]
    warmups = [record for record in records if record['kind'] == KIND_WARMUP]
    by_kind = {kind: 0 for kind in KINDS}
    for record in records:
        by_kind[record['kind']] = by_kind.get(record['kind'], 0) + 1

    def block(items, section, key):
        return [record.get(section, {}).get(key) for record in items]

    summary = {
        'exchanges': len(records),
        'by_kind': by_kind,
        'failures': sum(1 for record in records
                        if record['response'].get('http_status') != 200),
        'retries': sum(1 for record in completions if record['input'].get('is_retry')),
        'prompt_parsed': sum(1 for record in completions
                             if record['input'].get('well_formed')),
        'json_valid': sum(1 for record in completions
                          if record['thought'].get('json_valid')),
        'json_with_trailing': sum(
            1 for record in completions
            if (record['thought'].get('trailing_chars') or 0) > 0),
        'budget_limited': sum(1 for record in completions
                              if record['load'].get('hit_token_limit')),
    }
    if completions:
        summary['wall_ms'] = _stats([record['wall_ms'] for record in completions])
        summary['wall_total_ms'] = round(
            sum(record['wall_ms'] or 0.0 for record in completions), 1)
        summary['prompt_per_second'] = _stats(
            block(completions, 'timings', 'prompt_per_second'))
        summary['predicted_per_second'] = _stats(
            block(completions, 'timings', 'predicted_per_second'))
        summary['prompt_ms'] = _stats(block(completions, 'timings', 'prompt_ms'))
        summary['predicted_ms'] = _stats(block(completions, 'timings', 'predicted_ms'))
        summary['overhead_ms'] = _stats(block(completions, 'timings', 'overhead_ms'))
        summary['prompt_share'] = _stats(block(completions, 'timings', 'prompt_share'))
        summary['budget_used'] = _stats(block(completions, 'load', 'budget_used'))
        summary['tokens'] = {
            key: sum(value for value in block(completions, 'tokens', key)
                     if value is not None)
            for key in ('evaluated', 'predicted', 'cached')
        }
        commands = [record['input'].get('command') for record in completions]
        summary['commands'] = _tally(
            command if command is not None else '<unparsed>' for command in commands)
        verdicts = [record['thought'].get('airlock') for record in completions
                    if record['thought'].get('airlock')]
        if verdicts:
            rejected = [verdict.split(':', 2)[1] if ':' in verdict else verdict
                        for verdict in verdicts if verdict.startswith('rejected')]
            summary['airlock'] = {
                'accepted': sum(1 for v in verdicts if v.startswith('accepted')),
                'rejected': len(rejected),
                'by_error': _tally(rejected),
            }
    if warmups:
        summary['warmup_wall_ms'] = _stats([record['wall_ms'] for record in warmups])
    return summary


# ---------------------------------------------------------------------------
# Printing and files
# ---------------------------------------------------------------------------
def _row(label, text):
    """One aligned 'label   value' line."""
    return f'{label:<18}{text}'


def _stats_line(stats, spec='.1f', suffix=''):
    """'mean .. median .. p95 .. max .. (n=..)' from a _stats() dict."""
    if not stats:
        return '-'
    return (f'mean {format(stats["mean"], spec)}{suffix}  '
            f'median {format(stats["median"], spec)}{suffix}  '
            f'p95 {format(stats["p95"], spec)}{suffix}  '
            f'max {format(stats["max"], spec)}{suffix}  '
            f'(n={stats["n"]})')


def _server_lines(server):
    """Build the llama-server side of the summary: identity, RSS, KV, counters."""
    lines = []
    props = server.get('props') or {}
    bits = []
    if server.get('pid'):
        bits.append(f'pid {server["pid"]}')
    if props.get('n_ctx'):
        bits.append(f'n_ctx {props["n_ctx"]}')
    if props.get('total_slots'):
        bits.append(f'slots {props["total_slots"]}')
    if props.get('model_path'):
        bits.append(os.path.basename(props['model_path']))
    if bits:
        lines.append(_row('server', '  '.join(bits)))
    if server.get('rss_baseline_mb') is not None:
        lines.append(_row('rss MiB', f'baseline {server["rss_baseline_mb"]:.0f}  '
                                     f'peak {server["rss_peak_mb"]:.0f}  '
                                     f'last {server["rss_last_mb"]:.0f}  '
                                     f'delta {server["rss_delta_mb"]:+.0f}'))
    if server.get('kv_ratio_peak') is not None:
        lines.append(_row('kv cache peak', f'{server["kv_ratio_peak"]:.0%} of n_ctx'))
    elif server.get('kv_ratio_metrics_last') is not None:
        lines.append(_row('kv cache', f'{server["kv_ratio_metrics_last"]:.0%} (/metrics)'))
    if server.get('metrics_delta'):
        detail = '  '.join(f'{name.split(":")[-1]} {value:+.0f}'
                           for name, value in server['metrics_delta'].items())
        lines.append(_row('metrics delta', detail))
    elif server and not server.get('metrics_available'):
        lines.append(_row('metrics', 'not served (llama-server without --metrics)'))
    return lines


def format_summary(summary, server=None):
    """Render the aggregate view as printable lines. Pure, so it can be asserted."""
    lines = []
    by_kind = summary.get('by_kind') or {}
    kinds = ', '.join(f'{kind} {by_kind.get(kind, 0)}' for kind in KINDS)
    lines.append(_row('exchanges', f'{summary.get("exchanges", 0)}  ({kinds})'))
    lines.append(_row('failures', str(summary.get('failures', 0))))
    lines.append(_row('retries', str(summary.get('retries', 0))))
    lines.append(_row('prompt parsed', str(summary.get('prompt_parsed', 0))))
    lines.append(_row('json valid', f'{summary.get("json_valid", 0)}  '
                                    f'(trailing text in '
                                    f'{summary.get("json_with_trailing", 0)})'))
    lines.append(_row('budget limited', str(summary.get('budget_limited', 0))))
    if summary.get('warmup_wall_ms'):
        lines.append(_row('warmup wall ms', _stats_line(summary['warmup_wall_ms'])))
    for label, key, spec, suffix in (
            ('wall ms', 'wall_ms', '.1f', ''),
            ('prompt eval', 'prompt_per_second', '.1f', ' t/s'),
            ('generation', 'predicted_per_second', '.1f', ' t/s'),
            ('prefill ms', 'prompt_ms', '.1f', ''),
            ('decode ms', 'predicted_ms', '.1f', ''),
            ('overhead ms', 'overhead_ms', '.1f', ''),
            ('prefill share', 'prompt_share', '.0%', ''),
            ('budget used', 'budget_used', '.0%', '')):
        if summary.get(key):
            lines.append(_row(label, _stats_line(summary[key], spec, suffix)))
    tokens = summary.get('tokens')
    if tokens:
        tokens_line = (f'evaluated {tokens["evaluated"]}  '
                       f'predicted {tokens["predicted"]}  '
                       f'cached {tokens["cached"]}')
        lines.append(_row('tokens total', tokens_line))
    airlock = summary.get('airlock')
    if airlock:
        detail = ', '.join(f'{name} {count}'
                           for name, count in airlock['by_error'].items())
        accepted = airlock['accepted']
        rejected = airlock['rejected']
        lines.append(_row('airlock', f'accepted {accepted}  rejected {rejected}'
                                     + (f'  ({detail})' if detail else '')))
    if server:
        lines.extend(_server_lines(server))
    return lines


def print_summary(summary, server=None, stream=None):
    """Print the summary block, framed. Used by both live mode and --replay."""
    stream = stream or sys.stdout
    print('=' * 60, file=stream)
    print('llm_analytics summary', file=stream)
    print('=' * 60, file=stream)
    for line in format_summary(summary, server):
        print(line, file=stream)
    print('=' * 60, file=stream, flush=True)


def write_report(path, payload):
    """Write JSON through a temporary file: a half-written report is worse than none."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False, default=str)
        handle.write('\n')
    os.replace(temporary, path)
    return path


def _dict(value):
    """Return `value` when it is a dict, else an empty one.

    Records come back from disk, from older captures, and from `--replay` of a
    file somebody edited by hand: every section may be missing. The table must
    degrade to a dash, never to a stack trace.
    """
    return value if isinstance(value, dict) else {}


def markdown_path(json_path):
    """Return the Markdown twin's path: `report.json` -> `report.md`.

    Derived rather than flagged: `--out` names one artifact, and the human
    summary belongs in the same directory, so a capture folder stays
    self-contained and can be read without a JSON viewer.
    """
    base, extension = os.path.splitext(json_path)
    if extension.lower() == '.md':
        return json_path
    if extension.lower() in ('.json', '.jsonl'):
        return f'{base}.md'
    return f'{json_path}.md'


def markdown_cell(value, spec=None):
    """Render one table cell, escaping what would break the row.

    Commands come from a human, so they can contain the two characters that
    end a Markdown cell: a newline and a pipe. Both are escaped rather than
    dropped -- the column promises the string the node really sent.
    """
    if value is None:
        return '-'
    if spec is not None and isinstance(value, (int, float)):
        return format(value, spec)
    text = str(value)
    text = text.replace('\\', '\\\\').replace('|', '\\|')
    for newline in ('\r\n', '\r', '\n'):
        text = text.replace(newline, '<br>')
    return text or '-'


def markdown_verdict(verdict):
    """Render an airlock verdict for a human, e.g. `accepted:3 moves`.

    The record's own spelling is kept after the colon (`rejected:<Exception>:
    <message>`) because the exception name is what the summary tallies and what
    an operator greps for; only the leading state is capitalised.
    """
    if not isinstance(verdict, str) or not verdict.strip():
        return 'not run (no verdict recorded)'
    state, _, reason = verdict.partition(':')
    state = state.strip().capitalize()
    reason = reason.strip()
    return f'{state}: {reason}' if reason else state


def markdown_table(records):
    """Build the per-exchange table: one row per exchange, in capture order.

    Seven columns, fixed: which command, at what temperature, whether the
    string the node would have parsed is JSON, what the airlock said, and what
    it cost in wall time, tokens and decode speed. Everything else stays in the
    JSON, where it is queryable.
    """
    lines = [
        '| Command Prompt | Temp | JSON Valid | Airlock Verdict '
        '| Latency | Tokens | Tok/s |',
        '|---|---|---:|---|---:|---:|---:|',
    ]
    for record in records or ():
        record = _dict(record)
        parsed = _dict(record.get('input'))
        params = _dict(_dict(record.get('request')).get('params'))
        thought = _dict(record.get('thought'))
        tokens = _dict(record.get('tokens'))
        timings = _dict(record.get('timings'))
        command = parsed.get('command')
        if command is None:
            command = ('(warm-up)' if record.get('kind') == KIND_WARMUP
                       else '(prompt not parsed)')
        counts = [value for value in (tokens.get('evaluated'),
                                      tokens.get('predicted'))
                  if isinstance(value, int)]
        lines.append('| ' + ' | '.join((
            markdown_cell(command),
            markdown_cell(params.get('temperature'), '.1f'),
            'Pass' if thought.get('json_valid') else 'Fail',
            markdown_cell(markdown_verdict(thought.get('airlock'))),
            markdown_cell(record.get('wall_ms'), '.1f'),
            markdown_cell(sum(counts) if counts else None),
            markdown_cell(timings.get('predicted_per_second'), '.1f'),
        )) + ' |')
    return lines


def format_markdown_report(payload):
    """Render a whole report as Markdown: what ran, what it cost, what it said.

    The summary block is `format_summary()` verbatim, so the Markdown file and
    the console can never disagree; only the per-exchange table is new here.
    The system-prompt sha256 is printed because it is the one key that makes
    two runs comparable at all. The input snapshot is named beside it: a
    capture whose commands are gone cannot be reproduced, and its numbers then
    describe nothing.
    """
    payload = payload if isinstance(payload, dict) else {}
    session = _dict(payload.get('session'))
    summary = _dict(payload.get('summary'))
    server = _dict(payload.get('server'))
    records = payload.get('exchanges')
    records = records if isinstance(records, list) else []

    lines = ['# llm_analytics report', '']
    replayed = session.get('replayed_from')
    lines.append(f'- replayed from: `{replayed}`' if replayed
                 else '- mode: live recording')
    for label, key in (('started', 'started_at'), ('finished', 'finished_at')):
        if session.get(key):
            lines.append(f'- {label}: {session[key]}')
    if session.get('duration_s') is not None:
        duration = markdown_cell(session['duration_s'], '.1f')
        lines.append(f'- duration: {duration} s')
    for label in ('upstream', 'proxy'):
        if session.get(label):
            lines.append(f'- {label}: {session[label]}')
    if session.get('airlock') is not None:
        lines.append(f'- airlock: {"on" if session["airlock"] else "off"}')
    if session.get('go_to_enabled') is not None:
        state = 'enabled' if session['go_to_enabled'] else 'disabled'
        lines.append(f'- go_to: {state}')
    if session.get('exec_file'):
        lines.append(f'- exec file: {session["exec_file"]}')
    if session.get('commands_snapshot'):
        name = os.path.basename(session['commands_snapshot'])
        parts = []
        count = session.get('commands_count')
        if count is not None:
            parts.append(f'{count} commands')
        sha = session.get('commands_sha256')
        if sha:
            parts.append(f'sha256 `{sha[:12]}`')
        suffix = f'  ({", ".join(parts)})' if parts else ''
        lines.append(f'- input: `{name}`{suffix}')
    for record in records:
        parsed = _dict(_dict(record).get('input'))
        if parsed.get('system_prompt_sha256'):
            sha = parsed['system_prompt_sha256'][:12]
            lines.append(f'- system prompt sha256: `{sha}`')
            break
    lines.append(f'- exchanges: {len(records)}')
    for warning in session.get('warnings') or ():
        lines.append(f'- warning: {warning}')
    lines.extend(['', '## Summary', '', '```'])
    lines.extend(format_summary(summary, server))
    lines.extend(['```', '', '## Exchanges', ''])
    lines.extend(markdown_table(records))
    lines.append('')
    return '\n'.join(lines)


def write_markdown_report(path, payload):
    """Write the Markdown twin of a report, atomically like its JSON."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        handle.write(format_markdown_report(payload))
    os.replace(temporary, path)
    return path


def write_reports(out_path, payload):
    """Write the JSON report and its Markdown twin; return both paths.

    Every run goes through here -- live and `--replay` alike -- so `--out` can
    never produce a report whose summary has to be read through a JSON viewer.
    """
    json_path = write_report(out_path, payload)
    markdown_file = write_markdown_report(markdown_path(json_path), payload)
    return json_path, markdown_file


def commands_snapshot_path(events_path):
    """Where a run's input is kept: `commands.txt` beside the capture.

    Derived rather than flagged, for the same reason as `markdown_path()`:
    `--events` already names the run's folder, and a copy somebody has to
    remember to make is the copy that goes missing. One run, one directory
    (decision D-B), so the folder is the only place where a capture and the
    exact commands that produced it can stay paired.
    """
    return os.path.join(os.path.dirname(events_path), 'commands.txt')


def write_commands_snapshot(events_path, exec_file=None, commands=None):
    """Keep this run's input beside its capture; return the session keys.

    `--exec-file` is copied byte for byte -- comments, blank lines and the
    original line endings included. The copy exists to be handed straight back
    to `--exec-file` on the next run, so any rewriting, or re-serialising a
    parsed command list, would make a different input look like the same one.

    `--exec` has no file, so the same commands are written one per line, which
    is the format `load_command_file()` reads back. A command containing a
    newline cannot survive that round trip; `--exec` strings never do.

    Returns an empty dict when there is nothing to copy (an idle recorder) or
    no capture to copy it beside (`--no-events`): an input file sitting next to
    no log is an input file that gets lost.
    """
    if not events_path or not (exec_file or commands):
        return {}
    if exec_file:
        with open(exec_file, 'rb') as handle:
            data = handle.read()
        source = 'exec-file'
        count = len(load_command_file(exec_file))
    else:
        data = ''.join(f'{command}\n' for command in commands).encode('utf-8')
        source = 'exec'
        count = len(commands)
    path = commands_snapshot_path(events_path)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    temporary = f'{path}.tmp'
    with open(temporary, 'wb') as handle:
        handle.write(data)
    os.replace(temporary, path)
    return {
        'commands_snapshot': path,
        'commands_source': source,
        'commands_count': count,
        'commands_sha256': hashlib.sha256(data).hexdigest(),
    }


def read_events(path):
    """Exchanges from a JSONL capture, skipping lines that are not objects."""
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except ValueError as error:
                print(f'WARNING: {path}:{number}: not JSON ({error})', file=sys.stderr)
                continue
            if isinstance(value, dict):
                records.append(value)
    return records


# ---------------------------------------------------------------------------
# The recording proxy
# ---------------------------------------------------------------------------
class _ProxyHandler(BaseHTTPRequestHandler):
    """Relays one request upstream verbatim, records it, relays the answer back."""

    protocol_version = 'HTTP/1.1'
    server_version = 'llm_analytics'
    sys_version = ''

    def do_GET(self):
        self._relay('GET')

    def do_POST(self):
        self._relay('POST')

    def _relay(self, method):
        started = time.time()
        body = self._read_body()
        content_type = self.headers.get('Content-Type')
        error = None
        try:
            status, headers, payload = self.server.forward(
                method, self.path, body, content_type)
        except Exception as failure:  # noqa: BLE001 -- a dead upstream is an outcome
            error = f'{type(failure).__name__}: {failure}'
            status, headers = 502, []
            payload = json.dumps({'error': error}).encode('utf-8')
        wall_ms = (time.time() - started) * 1000.0
        try:
            self.server.record(method, self.path, body, status, payload, wall_ms,
                               started, error)
        except Exception as failure:  # noqa: BLE001 -- recording must not break the proxy
            # Said loudly, not only under --verbose: a recorder that silently
            # stops recording is the one failure this tool cannot have.
            self.server.session.warn(
                f'recording {method} {self.path} failed: '
                f'{type(failure).__name__}: {failure}')
        self._respond(status, headers, payload)

    def _read_body(self):
        """Read the request body, by Content-Length, exactly as sent."""
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except (TypeError, ValueError):
            return b''
        return self.rfile.read(length) if length > 0 else b''

    def _respond(self, status, headers, payload):
        """Echo the upstream status and body back to the node."""
        self.send_response(status)
        for name, value in headers:
            self.send_header(name, value)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        if not payload:
            return
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            # The node gave up before the answer arrived (its own timeout).
            self.server.say('client closed the connection mid-response')

    def log_message(self, fmt, *args):
        """Route the base class's access log through --verbose, not stderr."""
        message = fmt % args if args else fmt
        self.server.say(f'{self.command} {self.path}: {message}')


class CompletionProxy(ThreadingHTTPServer):
    """Threaded HTTP server that relays to llama-server and records what it saw.

    Threaded because the node's readiness pass and its commands are separate
    connections and the recorder must never serialise them into a timeout.
    """

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, listen, upstream, session, timeout=120.0, verbose=False):
        self.upstream = upstream
        self.session = session
        self.timeout = float(timeout)
        self.verbose = verbose
        self.upstream_host, self.upstream_port, self.upstream_prefix = _split_url(upstream)
        super().__init__(tuple(listen), _ProxyHandler)

    @property
    def url(self):
        """Give the URL to hand the node as its llm_url parameter."""
        host, port = self.server_address[:2]
        return f'http://{host}:{port}/completion'

    def say(self, message):
        """Console chatter that only appears with --verbose."""
        if self.verbose:
            print(f'[proxy] {message}', file=sys.stderr, flush=True)

    def forward(self, method, path, body, content_type=None):
        """Send one request upstream verbatim: (status, headers, body).

        The body is passed through untouched, which is the whole contract: what
        gets recorded is the bytes the node produced, so a change in the node's
        prompt shows up here and nowhere else. Accept-Encoding is forced to
        identity so the body can also be read by eye and by json.loads().
        """
        connection = HTTPConnection(
            self.upstream_host, self.upstream_port, timeout=self.timeout)
        headers = {'Accept-Encoding': 'identity'}
        if content_type:
            headers['Content-Type'] = content_type
        try:
            connection.request(method, self.upstream_prefix + path,
                               body=body or None, headers=headers)
            response = connection.getresponse()
            payload = response.read()
            forwarded = [(name, value) for name, value in response.getheaders()
                         if name.lower() in FORWARDED_HEADERS]
            return response.status, forwarded, payload
        finally:
            connection.close()

    def record(self, method, path, request_body, status, response_body, wall_ms,
               t_start, error):
        """Hand the exchange to the session, with the nearest memory sample."""
        sample = None
        if self.session.poller is not None:
            sample = self.session.poller.snapshot(t_start)
        record = self.session.add(path, request_body, status, response_body,
                                  wall_ms, t_start=t_start, sample=sample)
        if error:
            self.session.warn(f'upstream error on {method} {path}: {error}')
        return record


# ---------------------------------------------------------------------------
# Driving the node itself (--exec)
# ---------------------------------------------------------------------------
TAIL_CHARS = 4000


def http_get(url, timeout=5.0):
    """GET an absolute URL: (status, body). Raises on a transport failure."""
    parts = urllib.parse.urlsplit(url)
    connection = HTTPConnection(parts.hostname, parts.port or 80, timeout=timeout)
    try:
        connection.request('GET', parts.path or '/',
                           headers={'Accept-Encoding': 'identity'})
        response = connection.getresponse()
        return response.status, response.read()
    finally:
        connection.close()


def http_post(url, payload, timeout=120.0):
    """POST raw bytes to an absolute URL: (status, body). Raises on a failure."""
    parts = urllib.parse.urlsplit(url)
    connection = HTTPConnection(parts.hostname, parts.port or 80, timeout=timeout)
    try:
        connection.request('POST', parts.path or '/', body=payload,
                           headers={'Content-Type': 'application/json',
                                    'Accept-Encoding': 'identity'})
        response = connection.getresponse()
        return response.status, response.read()
    finally:
        connection.close()


def _setup_bash():
    """Find this workspace's install/setup.bash, inferred from the environment."""
    candidates = []
    for variable in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for entry in os.environ.get(variable, '').split(os.pathsep):
            if not entry:
                continue
            prefix = os.path.dirname(entry.rstrip(os.sep))
            candidates.append(os.path.join(prefix, 'setup.bash'))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return None


def build_node_command(command, llm_url, dry_run=True, go_to=None, extra_params=()):
    """Build argv that plans one command with this recorder in between.

    Same shape as test_campaign/run_dry_campaign.build_command(): a dry run by
    default, so nothing reaches the mission services, and llm_url pointed at the
    proxy instead of llama-server. `go_to` is only added when asked for, so the
    node's own default (stack_params.yaml) stays in charge otherwise.
    """
    node_args = ['ros2', 'run', 'llm', 'llm_planner_node', command]
    if dry_run:
        node_args.append('--dry-run')
    parameters = [f'llm_url:={llm_url}']
    if go_to is not None:
        parameters.append(f'go_to_enabled:={"true" if go_to else "false"}')
    parameters.extend(extra_params)
    node_args.append('--ros-args')
    for parameter in parameters:
        node_args.extend(('-p', parameter))
    if shutil.which('ros2'):
        return node_args
    # Started from an unsourced shell (a plain terminal, a cron-like runner):
    # source the workspace's own setup.bash, as run_dry_campaign does.
    setup_bash = _setup_bash()
    if not setup_bash:
        raise RuntimeError(
            'ros2 is not on PATH and no install/setup.bash was found; source the '
            'workspace before using --exec')
    quoted = ' '.join(shlex.quote(arg) for arg in node_args)
    return ['bash', '-c', f'source {shlex.quote(setup_bash)} >/dev/null && exec {quoted}']


def _signal_group(process, number):
    """Signal a subprocess's whole group, so ros2's children die with it."""
    try:
        os.killpg(os.getpgid(process.pid), number)
        return
    except OSError:
        pass
    try:
        process.send_signal(number)
    except OSError:
        pass


def parse_result_line(stdout):
    """Parse the node's last `RESULT {...}` line, or return None.

    llm_planner_node prints exactly one per command (see _emit_result), always
    last. Reading it instead of scraping the Italian prose is what makes the
    node's verdict and the proxy's own view of the same call comparable.
    """
    for line in reversed((stdout or '').splitlines()):
        line = line.strip()
        if not line.startswith(RESULT_PREFIX):
            continue
        value = _decode_json(line[len(RESULT_PREFIX):])
        return value if isinstance(value, dict) else None
    return None


def run_node_command(argv, timeout_s=180.0):
    """Run the node once and collect what it said. Never raises on failure."""
    started = time.time()
    process = subprocess.Popen(
        argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, start_new_session=True)
    timed_out = False
    try:
        stdout, stderr = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        # SIGINT first: rclpy still gets to shut down and print its RESULT line.
        _signal_group(process, signal.SIGINT)
        try:
            stdout, stderr = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            _signal_group(process, signal.SIGKILL)
            stdout, stderr = process.communicate()
    return {
        'argv': argv,
        'exit_code': process.returncode,
        'timed_out': timed_out,
        'duration_s': round(time.time() - started, 2),
        'result': parse_result_line(stdout),
        'stdout_tail': _truncate(stdout, TAIL_CHARS),
        'stderr_tail': _truncate(stderr, TAIL_CHARS),
    }


# ---------------------------------------------------------------------------
# --exec-file: the node's request, without the node
# ---------------------------------------------------------------------------
def load_system_prompt(go_to_enabled=True):
    """Return (system_prompt, filename) exactly as llm_planner_node loads it.

    The node's default planner_path is 'v2', so its system prompt is a FILE and
    not the inline SYSTEM_PROMPT: plan_translate owns which file, and this asks
    it rather than repeating a name that has already been changed once (the A4
    prompt change kept the name and altered the text).

    go_to_enabled chooses between two files because the node pairs the prompt
    with the translator: sweeping commands against the wrong one would measure a
    system nobody runs.

    Imported lazily and guarded, like the airlock, so a machine where the
    package's data files are missing says so instead of dying mid-batch.
    """
    try:
        from llm.plan_translate import intent_prompt_filename, load_intent_prompt
    except Exception as error:  # noqa: BLE001 -- report it, do not crash
        raise RuntimeError(
            f"cannot build the node's prompt: "
            f'{type(error).__name__}: {error}') from error
    filename = intent_prompt_filename(go_to_enabled)
    try:
        return load_intent_prompt(filename), filename
    except Exception as error:  # noqa: BLE001 -- same reasoning
        raise RuntimeError(
            f'cannot read prompts/{filename}: '
            f'{type(error).__name__}: {error}') from error


def build_intent_request(command, system_prompt, n_predict=N_PREDICT,
                         temperature=TEMPERATURE, json_schema=None):
    """Build the /completion body llm_planner_node._completion() would post.

    Byte for byte the node's request: the same system prompt, the same
    `comando: "<command>"` / `risposta:` framing around an unwrapped command,
    the same n_predict ceiling, temperature and json_schema, and no `stop` list.
    Nothing is added for the batch's convenience -- if this request differs from
    the node's, the timings in the log describe a system nobody runs.
    """
    return {
        'prompt': f'{system_prompt}{PROMPT_HEAD}{command}{PROMPT_TAIL}',
        'n_predict': n_predict,
        'temperature': temperature,
        'json_schema': INTENT_REQUEST_SCHEMA if json_schema is None else json_schema,
    }


def load_command_file(path):
    r"""Read one command per line; blank lines and `#` comments are skipped.

    A command file is edited by hand, so it will collect comments and stray
    newlines: refusing to run over them would be a trap, and sending them would
    cost a generation each. Only leading and trailing whitespace is removed --
    the interior of the line is the command and is passed through untouched.
    """
    commands = []
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            text = line.strip()
            if not text or text.startswith('#'):
                continue
            commands.append(text)
    return commands


# ---------------------------------------------------------------------------
# --probe: what does this build actually expose?
# ---------------------------------------------------------------------------
def probe(upstream, timeout=120.0):
    """Report what the running llama-server is and what it really returns.

    Read-mostly and one-shot: /health, /props, /slots and /metrics, then one
    8-token generation with the same shape as the node's warm-up. Everything
    printed here is exactly what the analyser keys on, so this is the first
    thing to run when a field in the report comes back null.
    """
    upstream = upstream.rstrip('/')
    print(f'upstream    {upstream}')
    for path in ('/health', '/props', '/slots', '/metrics'):
        try:
            status, body = http_get(upstream + path, timeout=5.0)
        except Exception as error:  # noqa: BLE001 -- a diagnostic reports failures
            print(f'{path:<11} unreachable ({type(error).__name__}: {error})')
            continue
        text = body.decode('utf-8', 'replace')
        if path == '/metrics':
            metrics = parse_prometheus(text)
            names = ', '.join(sorted(metrics)[:12]) or 'no parseable samples'
            print(f'{path:<11} HTTP {status}  {len(metrics)} metrics: {names}')
            continue
        parsed = _decode_json(text)
        if isinstance(parsed, dict):
            print(f'{path:<11} HTTP {status}  keys: {", ".join(sorted(parsed))}')
        else:
            print(f'{path:<11} HTTP {status}  {_one_line(text, 100)}')

    payload = json.dumps({'prompt': WARMUP_PROMPT, 'n_predict': 8,
                          'temperature': 0.0}).encode('utf-8')
    try:
        status, body = http_post(upstream + '/completion', payload, timeout=timeout)
    except Exception as error:  # noqa: BLE001 -- a diagnostic reports failures
        print(f'POST /completion unreachable ({type(error).__name__}: {error})')
        return 2
    parsed = _decode_json(body)
    parsed = parsed if isinstance(parsed, dict) else {}
    print(f'POST /completion  HTTP {status}  {len(body)} bytes')
    print(f'  keys                 {", ".join(sorted(parsed))}')
    for key in ('model', 'stop_type', 'tokens_evaluated', 'tokens_predicted',
                'tokens_cached', 'truncated'):
        if key in parsed:
            print(f'  {key:<20} {parsed[key]!r}')
    for key in ('timings', 'generation_settings'):
        if key in parsed:
            print(f'  {key:<20} {json.dumps(parsed[key], sort_keys=True)}')
    print(f'  content              {_one_line(parsed.get("content"), 120)!r}')
    return 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser():
    """Build the recorder's argument parser."""
    parser = argparse.ArgumentParser(
        prog='llm_analytics',
        description='Record, and then analyse, the traffic between '
                    'llm_planner_node and llama-server. Read-only: the node is '
                    'pointed at this proxy and nothing else changes.')
    parser.add_argument('--upstream', default=DEFAULT_UPSTREAM,
                        help=f'llama-server base URL (default {DEFAULT_UPSTREAM})')
    parser.add_argument('--listen', default=DEFAULT_LISTEN,
                        help=f'address to record on (default {DEFAULT_LISTEN})')
    parser.add_argument('--out', default=DEFAULT_OUT,
                        help='JSON report path; a Markdown summary of the '
                             'same run is written beside it '
                             f'(default {DEFAULT_OUT})')
    parser.add_argument('--events', default=DEFAULT_EVENTS,
                        help='JSONL event log path, one line per exchange '
                             f'(default {DEFAULT_EVENTS})')
    parser.add_argument('--no-events', action='store_true',
                        help='do not write the JSONL event log')
    parser.add_argument('--exec', dest='commands', action='append', metavar='COMMAND',
                        help='run llm_planner_node with this command, in order '
                             '(repeatable). Dry run unless --live.')
    parser.add_argument('--live', action='store_true',
                        help='with --exec: let the node load and start the mission '
                             '(default is --dry-run)')
    parser.add_argument('--exec-file', metavar='PATH', dest='exec_file',
                        help='send every command in this file (one per line, '
                             'blank lines and # comments skipped) in order, as '
                             "the node's own request, without starting the node")
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='per-command timeout in seconds (default 180)')
    parser.add_argument('--airlock', action='store_true',
                        help='also run each raw output through '
                             'plan_translate.translate() and record the verdict')
    parser.add_argument('--no-go-to', dest='go_to', action='store_false', default=None,
                        help='treat go_to as disabled, for --exec and --airlock')
    parser.add_argument('--no-memory', action='store_true',
                        help='do not poll /slots, /metrics or /proc')
    parser.add_argument('--poll-interval', type=float, default=0.5,
                        help='seconds between memory samples (default 0.5)')
    parser.add_argument('--upstream-timeout', type=float, default=120.0,
                        help='proxy to llama-server timeout in seconds (default 120, '
                             "above the node's own 90 so the node times out first)")
    parser.add_argument('--raw-chars', type=int, default=0,
                        help='truncate the raw output on the console to this many '
                             'characters (0 = whole, the default)')
    parser.add_argument('--no-raw', action='store_true',
                        help='do not print the raw model output on the console')
    parser.add_argument('--replay', metavar='JSONL',
                        help='analyse a saved event log instead of recording')
    parser.add_argument('--probe', action='store_true',
                        help='report what llama-server exposes and returns, then exit')
    parser.add_argument('--verbose', action='store_true',
                        help='log every proxied request on stderr')
    return parser


def _print_banner(server, upstream, opts):
    """Print what is being recorded, where, and how to point the node at it."""
    host, port = server.server_address[:2]
    print(f'llm_analytics   {upstream}   ->   http://{host}:{port}')
    print(f'  llm_url    {server.url}')
    if not opts.commands and not opts.exec_file:
        print('  run        ros2 run llm llm_planner_node "<command>" --dry-run \\')
        print(f'             --ros-args -p llm_url:={server.url}')
        print('  (Ctrl-C to stop and write the report)')
    if opts.exec_file:
        print(f'  exec-file  {opts.exec_file}  '
              "(the node's payload; the node is not started)")
    if opts.airlock:
        print('  airlock    on: every raw output goes through translate()')
    print(f'  events     {opts.events if not opts.no_events else "off"}')
    print(f'  report     {opts.out}')
    print()


def _run_commands(opts, session, server):
    """Run the node once per --exec command, in order, and report each verdict."""
    exit_code = 0
    total = len(opts.commands)
    for number, command in enumerate(opts.commands, 1):
        try:
            argv = build_node_command(command, server.url, dry_run=not opts.live,
                                      go_to=opts.go_to)
        except RuntimeError as error:
            print(f'ERRORE: {error}', file=sys.stderr)
            return 2
        print(f'--- [{number}/{total}] {command}', flush=True)
        run = run_node_command(argv, timeout_s=opts.timeout)
        session.runs.append(dict(run, command=command))
        result = run.get('result') or {}
        verdict = result.get('status') or 'no RESULT line'
        suffix = '  TIMED OUT' if run['timed_out'] else ''
        print(f'    node: {verdict}   exit {run["exit_code"]}   '
              f'{run["duration_s"]}s{suffix}', flush=True)
        if result.get('error_message'):
            print('    ' + _one_line(f'{result.get("error_type")}: '
                                     f'{result["error_message"]}', 110), flush=True)
        if run['timed_out'] or run['exit_code'] not in (0, None):
            exit_code = exit_code or 1
        if verdict in ('llm_error', 'translator_error', 'node_error'):
            exit_code = exit_code or 1
    return exit_code


def _report_batch_answer(session, number, total, command, status, body, wall_ms):
    """Echo one command's latency, speed and JSON verdict; 1 if it went wrong.

    The record is written by the proxy on the way through, with the same
    analyser and the same fields as a node-driven exchange. What is computed
    here is the batch's own progress line, so a sweep of forty commands can be
    watched as it runs instead of by tailing the JSONL afterwards.
    """
    response_obj = _decode_json(body)
    response_obj = response_obj if isinstance(response_obj, dict) else {}
    thought = parse_thought(response_obj.get('content'))
    timings = summarise_timings(response_obj.get('timings'), wall_ms)
    fields = [f'http {status}', f'{wall_ms:.0f} ms']
    if timings['predicted_per_second'] is not None:
        fields.append(f'{timings["predicted_per_second"]:.1f} tok/s')
    fields.append('JSON ok' if thought['json_valid'] else 'JSON INVALID')
    if thought['json_valid'] and thought['trailing_chars']:
        fields.append(f'+{thought["trailing_chars"]} chars after the JSON')
    print('    ' + '  |  '.join(fields), flush=True)
    if 200 <= status < 300 and thought['json_valid']:
        return 0
    if not 200 <= status < 300:
        snippet = response_obj.get('error')
        if not snippet:
            text = body.decode('utf-8', 'replace') if isinstance(body, bytes) else ''
            snippet = _one_line(text, 100)
        detail = f'HTTP {status}: {snippet}'
    elif thought['empty']:
        detail = 'HTTP ok, but the model returned nothing'
    else:
        detail = 'HTTP ok, but the output holds no JSON value'
    # Both places on purpose: the warning ends up in the report's session block,
    # the print tells whoever is watching the sweep which line just went wrong.
    session.warn(f'command {number}/{total} {command!r}: {detail}')
    print(f'    warning  {_one_line(detail, 100)}', flush=True)
    return 1


def _run_batch(opts, session, server):
    """Send every --exec-file command through the proxy, in order.

    No node and no ROS: each line becomes the request _completion() would have
    posted and goes through the recording proxy, so the exchange lands in the
    JSONL exactly as a node-driven one does -- same fields, same analyser, same
    summary. That is what makes a sweep affordable: forty commands cost forty
    generations instead of forty rclpy starts.

    Nothing here may end the batch. A refused connection, a timeout, an HTTP
    error, a body that is not JSON, a body that is JSON but not an intent: each
    is recorded (proxy-side, or as a warning) and the next line runs. The exit
    code says at the end whether any line went wrong: 1 for that, 2 for a
    problem that stopped the batch before it began (no prompt, no command file).
    """
    try:
        system_prompt, filename = load_system_prompt(opts.go_to is not False)
    except RuntimeError as error:
        print(f'ERRORE: {error}', file=sys.stderr)
        return 2
    try:
        commands = load_command_file(opts.exec_file)
    except (OSError, UnicodeDecodeError) as error:
        print(f'ERRORE: cannot read {opts.exec_file}: {error}', file=sys.stderr)
        return 2
    if not commands:
        print(f'ERRORE: {opts.exec_file} holds no commands', file=sys.stderr)
        return 2
    # Which prompt, by name and by hash, exactly as the node logs it at startup:
    # two runs of the same command file are only comparable when this matches.
    print(f'  prompt     prompts/{filename}  '
          f'sha256={_sha256(system_prompt)[:12]}', flush=True)
    print(f'  commands   {len(commands)} from {opts.exec_file}', flush=True)
    total = len(commands)
    exit_code = 0
    for number, command in enumerate(commands, 1):
        body = json.dumps(build_intent_request(
            command, system_prompt)).encode('utf-8')
        print(f'--- [{number}/{total}] {command}', flush=True)
        started = time.time()
        try:
            status, payload = http_post(server.url, body, timeout=opts.timeout)
        except Exception as error:  # noqa: BLE001 -- one line cannot end the batch
            exit_code = exit_code or 1
            detail = f'{type(error).__name__}: {error}'
            session.warn(f'command {number}/{total} {command!r}: {detail}')
            print(f'    failed   {_one_line(detail, 100)}', flush=True)
            continue
        wall_ms = (time.time() - started) * 1000.0
        outcome = _report_batch_answer(
            session, number, total, command, status, payload, wall_ms)
        exit_code = exit_code or outcome
    return exit_code


def _replay(path, out_path):
    """Summarise a saved event log. No server, no ROS, no network."""
    try:
        records = read_events(path)
    except OSError as error:
        print(f'ERRORE: cannot read {path}: {error}', file=sys.stderr)
        return 2
    summary = build_summary(records)
    print_summary(summary)
    payload = {
        'tool': 'llm_analytics',
        'session': {'replayed_from': path, 'finished_at': _now_iso()},
        'summary': summary,
        'server': {},
        'runs': [],
        'exchanges': records,
    }
    json_path, markdown_file = write_reports(out_path, payload)
    print(f'report     {json_path}   ({len(records)} exchanges)')
    print(f'markdown   {markdown_file}')
    return 0


def _wait_for_interrupt():
    """Block the main thread until Ctrl-C."""
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        print('\nstopping', file=sys.stderr)


def _record(opts, upstream):
    """Serve, optionally drive the node, then write the report."""
    try:
        listen = _split_listen(opts.listen)
    except ValueError as error:
        print(f'ERRORE: {error}', file=sys.stderr)
        return 2
    session = Session(upstream, opts.listen, airlock=opts.airlock,
                      go_to_enabled=opts.go_to is not False,
                      events_path=None if opts.no_events else opts.events,
                      raw_chars=opts.raw_chars, show_raw=not opts.no_raw,
                      verbose=opts.verbose)
    poller = None
    if not opts.no_memory:
        poller = MemoryPoller(upstream, port=_split_url(upstream)[1],
                              interval=opts.poll_interval,
                              on_warning=session.warn)
        session.poller = poller
    try:
        server = CompletionProxy(listen, upstream, session,
                                 timeout=opts.upstream_timeout,
                                 verbose=opts.verbose)
    except OSError as error:
        print(f'ERRORE: cannot listen on {opts.listen}: {error}', file=sys.stderr)
        session.close()
        return 2
    thread = threading.Thread(target=server.serve_forever, daemon=True,
                              name='llm-analytics-proxy')
    thread.start()
    if poller is not None:
        poller.start()
    _print_banner(server, upstream, opts)
    # Before the first command goes out: the input is part of the capture. A
    # failure here is a warning, never an abort -- the measurement is the point.
    extra = {'exec_file': opts.exec_file} if opts.exec_file else {}
    try:
        extra.update(write_commands_snapshot(
            session.events_path, exec_file=opts.exec_file,
            commands=opts.commands))
    except OSError as error:
        session.warn(f'cannot keep the input beside the capture: {error}')
    if extra.get('commands_snapshot'):
        print(f'  input      {extra["commands_snapshot"]}  '
              f'sha256={extra["commands_sha256"][:12]}', flush=True)
    exit_code = 0
    try:
        if opts.commands:
            exit_code = _run_commands(opts, session, server)
        if opts.exec_file:
            exit_code = _run_batch(opts, session, server) or exit_code
        if not opts.commands and not opts.exec_file:
            _wait_for_interrupt()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5.0)
        if poller is not None:
            poller.stop()
        payload = session.report(extra or None)
        session.close()
    path, markdown_file = write_reports(opts.out, payload)
    print()
    print_summary(payload['summary'], payload['server'])
    print(f'report     {path}')
    print(f'markdown   {markdown_file}')
    if payload['session']['warnings']:
        for warning in payload['session']['warnings']:
            print(f'warning    {warning}')
    if exit_code == 0 and payload['summary'].get('failures'):
        return 1
    return exit_code


def main(argv=None):
    """Entry point: record (serve, --exec or --exec-file), replay, or probe."""
    opts = build_parser().parse_args(argv)
    upstream = opts.upstream.rstrip('/')
    if opts.exec_file and opts.live and not opts.commands:
        print('ERRORE: --live applies to --exec only; --exec-file posts the '
              "node's payload itself and never loads a mission",
              file=sys.stderr)
        return 2
    if opts.probe:
        return probe(upstream, timeout=opts.upstream_timeout)
    if opts.replay:
        return _replay(opts.replay, opts.out)
    return _record(opts, upstream)


if __name__ == '__main__':
    sys.exit(main())
