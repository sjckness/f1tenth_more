#!/usr/bin/env python3
"""
llm2_mpc_param_node: LLM2 of two-llm-arch, the MPC-parameter interrogation (a MOCK).

Two-stage split: LLM1 (llm_planner_node) translates a mission, LLM2 tunes the
controller. For now LLM2 is a mock. Every tick (rate_hz, default 0.5 Hz) it
sends its own llama-server a fixed MPC parameter set as JSON and asks for it
back verbatim. A valid reply goes out as an f1tenth_messages/MpcParamUpdate on
/llm2/mpc_params (source 'llm2_mock'), which MPC_corr applies only when its own
llm2_params_enabled is true (default false). The default set is today's
launch-path values, so an exact echo changes nothing.

One tick:
  - paused (below): nothing is sent; the tick is counted as 'paused', not
    as 'skipped', and still gets its status;
  - a request still in flight: the tick is skipped and counted, never queued.
    Requests never overlap (the server has one slot, and a queued request
    would answer a tick that is already stale);
  - otherwise one POST /completion on a worker thread, so the executor never
    blocks on HTTP: temperature 0, a json_schema of exactly these keys
    (llama-server turns it into a grammar), cache_prompt with a fixed prompt
    prefix (llm2_protocol.build_request_body);
  - the reply is validated (one JSON object, the same key set, finite
    numbers: llm2_protocol.validate_reply). Valid: published. Invalid:
    nothing is published and the reason is logged;
  - one status per tick on /llm2/status (std_msgs/String, JSON), published
    when the tick's outcome is known: at once for a skipped or paused tick,
    when the reply, the timeout or the error comes back otherwise. It carries the
    outcome, echo_match (reply == sent), the wall latency, llama-server's
    prefill (prompt_ms) and generation (predicted_ms) times, the node's
    state when the status is published ('active' / 'paused'; a tick's
    outcome can predate a /llm2/set_active call that came in before its
    status went out), and the cumulative counters ok / invalid /
    timeout / error / skipped / paused / discarded / echo_match.

Pause / resume: the service /llm2/set_active (std_srvs/SetBool; true =
active, false = paused), and start_active (true) for the state at start. Paused, the
timer keeps ticking and publishing /llm2/status ('state': 'paused'), but no
request goes to the server and nothing is published on /llm2/mpc_params. A
request already in flight when the pause comes is left to finish; its reply is
then discarded (outcome and counter 'discarded'), even if the node was resumed
meanwhile. Pausing reverts nothing: mpc_corr keeps the values it last
applied. The llama-server keeps running with its model loaded. Setting the
state the node is already in is accepted (success, nothing changes). The
response's message is the new state, 'active' or 'paused'.

Parameters (config/llm2_mpc_param_params.yaml; llm2.launch.py injects
server_url from its llm2_port argument):
  start_active       start active (true) or paused (false); read-only, read
                     once at start: /llm2/set_active switches at run time
  rate_hz            tick rate (0.5)
  server_url         the LLM2 llama-server (http://127.0.0.1:8084; 8083 is LLM1's)
  request_timeout_s  timeout of a request once the server has answered (10.0)
  warmup_timeout_s   timeout of the first request, and of the next one after
                     any timeout or error (90.0): a fresh server builds its
                     GPU pipelines on first use (26.4 s measured on linus)
  n_predict          generation cap (256; the six-weight echo is ~82 tokens)
  mock_param_names   MPC_corr parameter names, in prompt order
  mock_param_values  their values, paired by index (a double array)

Run by hand against a server on another port:
  ros2 run llm llm2_mpc_param_node --ros-args -p server_url:=http://127.0.0.1:8083
"""

import json
import threading

from f1tenth_messages.msg import MpcParamUpdate

from llm.llm2_protocol import (
    Completion,
    completion_url,
    DISCARDED,
    ERROR,
    INVALID,
    MOCK_PARAMS,
    mock_params_from,
    OK,
    PAUSED,
    request_params,
    RequestGate,
    SKIPPED,
    SOURCE,
    TIMEOUT,
    TIMING_KEYS,
)

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool

PARAMS_TOPIC = '/llm2/mpc_params'
STATUS_TOPIC = '/llm2/status'
SET_ACTIVE_SERVICE = '/llm2/set_active'
#: The node's state: 'state' on every /llm2/status, and the message of every
#: /llm2/set_active response.
STATE_ACTIVE = 'active'
STATE_PAUSED = 'paused'
#: Every /llm2/status message has every one of these keys (null where the
#: tick has no value, e.g. latency_ms of a skipped tick), plus 'state',
#: 'source' and 'counters', so a bag or a log reads as one table.
STATUS_KEYS = ('seq', 'outcome', 'reason', 'echo_match', 'latency_ms', 'timeout_s',
               'in_flight_seq', 'tokens_cached') + TIMING_KEYS


class Llm2MpcParamNode(Node):
    """Ask the LLM2 server for the MPC parameter set every tick; publish valid replies."""

    def __init__(self, **kwargs):
        # kwargs: rclpy Node options, e.g. parameter_overrides in the tests.
        super().__init__('llm2_mpc_param_node', **kwargs)
        # Read-only: a runtime `ros2 param set ... start_active false` would
        # otherwise be accepted and do nothing. The runtime switch is the
        # /llm2/set_active service.
        self.declare_parameter('start_active', True, ParameterDescriptor(
            read_only=True,
            description=f'read once at start; pause/resume with {SET_ACTIVE_SERVICE}'))
        self.declare_parameter('rate_hz', 0.5)
        self.declare_parameter('server_url', 'http://127.0.0.1:8084')
        self.declare_parameter('request_timeout_s', 10.0)
        self.declare_parameter('warmup_timeout_s', 90.0)
        self.declare_parameter('n_predict', 256)
        # Two parallel lists rather than one map: a ROS parameter cannot be a
        # map, and this is the shape MpcParamUpdate carries (names[]/values[]).
        # The defaults are non-empty on purpose: rclpy types an empty list
        # default as a byte array.
        self.declare_parameter('mock_param_names', [name for name, _ in MOCK_PARAMS])
        self.declare_parameter('mock_param_values', [value for _, value in MOCK_PARAMS])

        def p(name):
            return self.get_parameter(name).value

        rate_hz = float(p('rate_hz'))
        self._server_url = str(p('server_url'))
        self._request_timeout_s = float(p('request_timeout_s'))
        self._warmup_timeout_s = float(p('warmup_timeout_s'))
        self._n_predict = int(p('n_predict'))
        for name, value in (('rate_hz', rate_hz),
                            ('request_timeout_s', self._request_timeout_s),
                            ('warmup_timeout_s', self._warmup_timeout_s),
                            ('n_predict', self._n_predict)):
            if not value > 0:
                raise ValueError(f'{name}={value} is not valid: it must be > 0')
        self._params = mock_params_from(p('mock_param_names'), p('mock_param_values'))
        self._url = completion_url(self._server_url)

        self._gate = RequestGate()
        self._seq = 0
        # Warm: the last request got an answer, so the next one gets the
        # short timeout. Cold at start and after any timeout or error (the
        # server may be loading, or restarted and cold again). Written by the
        # worker before it releases the gate and read by the timer only after
        # it has claimed the gate, so the two never race.
        self._warm = False
        self._started_at = self.get_clock().now()
        self._first_ok_logged = False
        self._closing = False

        # Pause / resume. _pauses counts the pauses: a request takes its
        # value when it starts, and a reply that comes back after a later
        # pause is discarded, even if the node was resumed meanwhile.
        # _state_lock makes a pause atomic against the timer starting a
        # request and against the worker publishing its update, so nothing
        # reaches /llm2/mpc_params once /llm2/set_active has answered
        # 'paused'; every status reads 'state' and is published under it too.
        # Lock order: _state_lock, then the gate's own lock.
        self._state_lock = threading.Lock()
        self._active = bool(p('start_active'))
        self._pauses = 0

        self._update_pub = self.create_publisher(MpcParamUpdate, PARAMS_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._set_active_srv = self.create_service(
            SetBool, SET_ACTIVE_SERVICE, self._on_set_active)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_tick)

        self.get_logger().info(
            f'LLM2 mock: {len(self._params)} parameters every {1.0 / rate_hz:g} s from '
            f'{self._url} -> {PARAMS_TOPIC} (status {STATUS_TOPIC}); timeout '
            f'{self._request_timeout_s:.1f} s, {self._warmup_timeout_s:.1f} s until the '
            f'server first answers. Set: {json.dumps(self._params)}')
        if not self._active:
            self.get_logger().info(
                f'starting PAUSED (start_active false): no request until '
                f'{SET_ACTIVE_SERVICE} true')

    # -- one tick -----------------------------------------------------------

    def _on_tick(self):
        self._seq = (self._seq + 1) & 0xFFFFFFFF    # MpcParamUpdate.seq is a uint32
        seq = self._seq
        # Under the state lock: a pause lands either before this tick (no
        # request) or after its request has started (the pause then
        # discards the reply), never in between.
        with self._state_lock:
            active, pauses = self._active, self._pauses
            gate = self._gate.try_start(seq) if active else self._gate.count_paused()
            if gate.started:
                timeout_s = self._request_timeout_s if self._warm else self._warmup_timeout_s
                threading.Thread(
                    target=self._run_request, args=(seq, timeout_s, pauses),
                    name=f'llm2_request_{seq}', daemon=True).start()
        if gate.started:
            return
        if active:
            outcome, reason = SKIPPED, f'request {gate.in_flight} still in flight'
        else:
            outcome, reason = PAUSED, f'paused, nothing sent ({SET_ACTIVE_SERVICE} true resumes)'
        self._publish_status({
            'seq': seq, 'outcome': outcome, 'reason': reason,
            'in_flight_seq': gate.in_flight,
        }, gate.counters)

    def _run_request(self, seq, timeout_s, pauses):
        """Worker thread: one request, its publication and its status. Always releases the gate."""
        try:
            result = request_params(self._url, self._params, timeout_s, self._n_predict)
        except Exception as e:
            # request_params() classifies every server problem itself; this
            # is a bug, and a bug must not leave the gate claimed for good
            # (every later tick would be skipped, silently).
            result = Completion(ERROR, f'unexpected {type(e).__name__}: {e}', None, None,
                                0.0, {key: None for key in TIMING_KEYS}, None)
        # Warm or cold describes the server, so it holds whatever the node's state.
        self._warm = result.outcome in (OK, INVALID)
        outcome, reason, echo_match = result.outcome, result.reason, result.echo_match
        try:
            with self._state_lock:
                if self._active and self._pauses == pauses:
                    if result.outcome == OK:
                        self._publish_update(seq, result.values)
                else:
                    # Paused while in flight: the reply answers a request
                    # the pause cancelled. Counted as discarded only, so ok /
                    # invalid / timeout / error / echo_match count what was used.
                    outcome, echo_match = DISCARDED, None
                    reason = (f'paused while in flight: its result ({result.outcome}) '
                              f'was discarded, nothing published')
            if outcome == DISCARDED:
                self.get_logger().info(f'seq {seq}: {reason}')
            else:
                self._log_outcome(seq, result)
        finally:
            counters = self._gate.finish(outcome, echo_match)
        status = {
            'seq': seq, 'outcome': outcome, 'reason': reason,
            'echo_match': echo_match,
            'latency_ms': round(result.latency_ms, 1),
            'timeout_s': timeout_s,
            'tokens_cached': result.tokens_cached,
        }
        status.update(result.timings)
        self._publish_status(status, counters)

    # -- pause / resume -----------------------------------------------------

    def _on_set_active(self, request, response):
        """/llm2/set_active: true = active, false = paused. Idempotent; always succeeds."""
        active = bool(request.data)
        with self._state_lock:
            changed = active != self._active
            if changed and not active:
                self._pauses += 1
            self._active = active
            in_flight = self._gate.in_flight
        state = STATE_ACTIVE if active else STATE_PAUSED
        log = self.get_logger()
        # A request in flight here started before a pause (a paused node
        # starts none), so its reply will be discarded either way.
        in_flight_note = ('' if in_flight is None else
                          f'; request {in_flight} is still in flight, its reply '
                          f'will be discarded')
        if not changed:
            log.info(f'{SET_ACTIVE_SERVICE} {active}: already {state}, nothing changes')
        elif active:
            log.info(f'LLM2 resumed ({SET_ACTIVE_SERVICE}): requests restart at the '
                     f'next tick that finds none in flight{in_flight_note}')
        else:
            log.info(f'LLM2 paused ({SET_ACTIVE_SERVICE}): no requests, nothing on '
                     f'{PARAMS_TOPIC} until resumed; mpc_corr keeps the values it '
                     f'last applied{in_flight_note}')
        response.success = True
        response.message = state
        return response

    def _log_outcome(self, seq, result):
        log = self.get_logger()
        if result.outcome == OK:
            if not self._first_ok_logged:
                self._first_ok_logged = True
                since_start = (self.get_clock().now() - self._started_at).nanoseconds * 1e-9
                log.info(
                    f'first valid reply (seq {seq}, {since_start:.1f} s after start, '
                    f'request {result.latency_ms:.0f} ms, echo_match {result.echo_match})')
            if not result.echo_match:
                changed = {name: (self._params[name], value)
                           for name, value in result.values.items()
                           if value != self._params[name]}
                log.warn(f'seq {seq}: valid reply, published, but NOT an echo: '
                         f'sent -> returned {changed}')
            else:
                log.debug(f'seq {seq}: echo ok, {result.latency_ms:.0f} ms')
        elif result.outcome == INVALID:
            log.warn(f'seq {seq}: invalid reply, nothing published: {result.reason}')
        elif result.outcome == TIMEOUT:
            log.warn(f'seq {seq}: {result.reason} from {self._url}, nothing published')
        else:
            # Connection refused repeats every tick while the server loads.
            log.warn(f'seq {seq}: no usable reply from {self._url}: {result.reason}',
                     throttle_duration_sec=10.0)

    # -- publication --------------------------------------------------------

    def _publish_update(self, seq, values):
        msg = MpcParamUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = seq
        msg.source = SOURCE
        msg.names = list(values)
        msg.values = [float(value) for value in values.values()]
        self._publish(self._update_pub, msg)

    def _publish_status(self, fields, counters):
        # 'state' is read and published under the state lock, so a status
        # published after /llm2/set_active has answered carries the new state
        # (the worker's status goes out after its outcome is decided, and a
        # call can land in between). The caller must not hold the lock.
        status = dict.fromkeys(STATUS_KEYS)
        status.update(fields)
        with self._state_lock:
            status['state'] = STATE_ACTIVE if self._active else STATE_PAUSED
            status['source'] = SOURCE
            status['counters'] = counters
            self._publish(self._status_pub, String(data=json.dumps(status)))

    def _publish(self, publisher, msg):
        # The worker can finish after shutdown began; its last publish then
        # has nowhere to go and is dropped.
        if self._closing:
            return
        try:
            publisher.publish(msg)
        except Exception:
            if not self._closing:
                raise

    def destroy_node(self):
        self._closing = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Llm2MpcParamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
