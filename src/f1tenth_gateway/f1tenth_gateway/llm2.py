"""
LLM2 (the MPC-parameter tuner of two-llm-arch) as the app sees it: active, paused, not running.

llm2_mpc_param_node publishes /llm2/status (std_msgs/String, one JSON per
tick, every 1/rate_hz = 2 s, paused or not) and serves /llm2/set_active
(std_srvs/SetBool, the response's message is exactly 'active' or 'paused').
The contract is output/two_llm/README.md, section 2.

Llm2State turns those into what lamacchinina shows. No rclpy in here: the ROS
port (ros_bridge.py) and the pretend car (mock.py) feed it the same way.

- not_running: no status for stale_s (5 s) and no service answer either, or
  the service not there at all. That is LLM2 not started (enable_llm2 false)
  or dead, which the app shows differently from paused;
- otherwise the state of the latest status, except right after a
  /llm2/set_active answer. Status and service are separate channels, so a
  status published just before the call can arrive after the response; the
  response's message then wins over any status received less than trust_s
  (one status period) after it.
"""

import json
import threading
from typing import Dict, Optional

ACTIVE = 'active'
PAUSED = 'paused'
NOT_RUNNING = 'not_running'
STATES = (ACTIVE, PAUSED)


class Llm2State:
    def __init__(self, service: str = '/llm2/set_active', stale_s: float = 5.0,
                 trust_s: float = 2.0):
        self.service = service
        self.stale_s = float(stale_s)
        self.trust_s = float(trust_s)
        self._lock = threading.Lock()
        self._status: Optional[Dict] = None
        self._status_t: Optional[float] = None
        self._answer: Optional[tuple] = None     # (t, state) of the last service answer
        self._latency_ms = None                  # the last tick that had one

    def forget(self) -> None:
        """Back to nothing heard (the mock's reset: a new pretend LLM2)."""
        with self._lock:
            self._status = self._status_t = self._answer = self._latency_ms = None

    def on_status(self, data, now: float) -> bool:
        """Take one /llm2/status (its JSON text, or the parsed dict); False if unusable."""
        # RecursionError (deeply nested JSON) is not a ValueError: escaping
        # the subscription callback, it would stop the whole ROS executor.
        try:
            status = json.loads(data) if isinstance(data, (str, bytes)) else data
        except (ValueError, RecursionError):
            return False
        if not isinstance(status, dict) or status.get('state') not in STATES:
            return False
        with self._lock:
            self._status, self._status_t = status, now
            if status.get('latency_ms') is not None:
                self._latency_ms = status['latency_ms']
        return True

    def on_response(self, requested: bool, success: bool, message: str, now: float) -> Dict:
        """
        Take a /llm2/set_active response; return {ok, state[, message]}.

        A message that names a state is the node's state from now on (it
        answered after switching), whatever was asked: it is applied even
        when it is not the one requested.
        """
        want = ACTIVE if requested else PAUSED
        state = message if message in STATES else None
        if state is not None:
            with self._lock:
                self._answer = (now, state)
        if success and state == want:
            return {'ok': True, 'state': state}
        return {'ok': False, 'state': state,
                'message': f'risposta inattesa da {self.service}: success={bool(success)}, '
                           f'message={message!r}'}

    def view(self, now: float, service_ready: bool = True) -> Dict:
        """Return {state, why, latency_ms, outcome, counters, age_s} for the status message."""
        with self._lock:
            status, status_t, answer = self._status, self._status_t, self._answer
            latency = self._latency_ms
        age = None if status_t is None else round(now - status_t, 1)
        out = {'state': NOT_RUNNING, 'why': None, 'latency_ms': latency,
               'outcome': (status or {}).get('outcome'),
               'counters': (status or {}).get('counters'), 'age_s': age}
        fresh = status_t is not None and now - status_t <= self.stale_s
        answered = answer is not None and now - answer[0] <= self.stale_s
        if not service_ready:
            out['why'] = f'servizio {self.service} non disponibile'
            return out
        if not fresh and not answered:
            out['why'] = ('nessun /llm2/status ricevuto' if status_t is None else
                          f'nessun /llm2/status da {now - status_t:.0f} s')
            return out
        state = status['state'] if status is not None else None
        if answer is not None and (status_t is None or status_t < answer[0] + self.trust_s):
            state = answer[1]
        out['state'] = state
        return out
