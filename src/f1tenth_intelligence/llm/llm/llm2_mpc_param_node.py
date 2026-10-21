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
    when the tick's outcome is known: at once for a skipped tick, when the
    reply, the timeout or the error comes back otherwise. It carries the
    outcome, echo_match (reply == sent), the wall latency, llama-server's
    prefill (prompt_ms) and generation (predicted_ms) times, and the
    cumulative counters ok / invalid / timeout / error / skipped / echo_match.

Parameters (config/llm2_mpc_param_params.yaml; llm2.launch.py injects
server_url from its llm2_port argument):
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
    ERROR,
    INVALID,
    MOCK_PARAMS,
    mock_params_from,
    OK,
    request_params,
    RequestGate,
    SKIPPED,
    SOURCE,
    TIMEOUT,
    TIMING_KEYS,
)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

PARAMS_TOPIC = '/llm2/mpc_params'
STATUS_TOPIC = '/llm2/status'
#: Every /llm2/status message has every one of these keys (null where the
#: tick has no value, e.g. latency_ms of a skipped tick), plus 'source' and
#: 'counters', so a bag or a log reads as one table.
STATUS_KEYS = ('seq', 'outcome', 'reason', 'echo_match', 'latency_ms', 'timeout_s',
               'in_flight_seq', 'tokens_cached') + TIMING_KEYS


class Llm2MpcParamNode(Node):
    """Ask the LLM2 server for the MPC parameter set every tick; publish valid replies."""

    def __init__(self, **kwargs):
        # kwargs: rclpy Node options, e.g. parameter_overrides in the tests.
        super().__init__('llm2_mpc_param_node', **kwargs)
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

        self._update_pub = self.create_publisher(MpcParamUpdate, PARAMS_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_tick)

        self.get_logger().info(
            f'LLM2 mock: {len(self._params)} parameters every {1.0 / rate_hz:g} s from '
            f'{self._url} -> {PARAMS_TOPIC} (status {STATUS_TOPIC}); timeout '
            f'{self._request_timeout_s:.1f} s, {self._warmup_timeout_s:.1f} s until the '
            f'server first answers. Set: {json.dumps(self._params)}')

    # -- one tick -----------------------------------------------------------

    def _on_tick(self):
        self._seq = (self._seq + 1) & 0xFFFFFFFF    # MpcParamUpdate.seq is a uint32
        seq = self._seq
        gate = self._gate.try_start(seq)
        if not gate.started:
            self._publish_status({
                'seq': seq, 'outcome': SKIPPED,
                'reason': f'request {gate.in_flight} still in flight',
                'in_flight_seq': gate.in_flight,
            }, gate.counters)
            return
        timeout_s = self._request_timeout_s if self._warm else self._warmup_timeout_s
        threading.Thread(
            target=self._run_request, args=(seq, timeout_s),
            name=f'llm2_request_{seq}', daemon=True).start()

    def _run_request(self, seq, timeout_s):
        """Worker thread: one request, its publication and its status. Always releases the gate."""
        try:
            result = request_params(self._url, self._params, timeout_s, self._n_predict)
        except Exception as e:
            # request_params() classifies every server problem itself; this
            # is a bug, and a bug must not leave the gate claimed for good
            # (every later tick would be skipped, silently).
            result = Completion(ERROR, f'unexpected {type(e).__name__}: {e}', None, None,
                                0.0, {key: None for key in TIMING_KEYS}, None)
        self._warm = result.outcome in (OK, INVALID)
        try:
            if result.outcome == OK:
                self._publish_update(seq, result.values)
            self._log_outcome(seq, result)
        finally:
            counters = self._gate.finish(result.outcome, result.echo_match)
        status = {
            'seq': seq, 'outcome': result.outcome, 'reason': result.reason,
            'echo_match': result.echo_match,
            'latency_ms': round(result.latency_ms, 1),
            'timeout_s': timeout_s,
            'tokens_cached': result.tokens_cached,
        }
        status.update(result.timings)
        self._publish_status(status, counters)

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
        status = dict.fromkeys(STATUS_KEYS)
        status.update(fields)
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
