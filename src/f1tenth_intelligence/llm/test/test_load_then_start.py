"""
Delivery: start_mission the moment load_mission confirms, never without it.

Mocked services, no ROS graph. LLMPlannerNode._deliver_mission runs unbound
on a stub (the pattern of test_planner_path.py) whose load client answers
like f1tenth_behavior's mission loader -- at once, late, with a refusal, or
never -- and rclpy.spin_until_future_complete is replaced by a stand-in that
keeps rclpy's contract: return when the future is done or when timeout_sec
has passed. The real _call_load_mission and wait_for_load_confirmation run.

llm_load_confirm_timeout_s is only the ceiling on the wait for the
confirmation: nothing in this path waits a fixed time.
"""

import time
from types import SimpleNamespace

from llm import llm_planner_node
from llm.llm_planner_node import (
    LLMPlannerNode,
    load_confirm_timeout_default,
    wait_for_load_confirmation,
)
import pytest

MISSION_PATH = '/nonexistent/llm_generated/llm_0123456789ab.json'
LOADED = "loaded 'llm_0123456789ab', 2 moves"


class _Future:
    """A client future: done once the stand-in spin delivers the response."""

    def __init__(self, response, delay_s):
        self.response = response        # None: the loader never answers
        self.delay_s = delay_s
        self.answered_at = None
        self._done = False

    def done(self):
        return self._done

    def result(self):
        return self.response if self._done else None

    def answer(self):
        self._done = True
        self.answered_at = time.monotonic()


class _LoadClient:
    """/mission/load_mission as the loader serves it."""

    def __init__(self, success=True, message=LOADED, delay_s=0.0,
                 available=True, answers=True):
        self.success = success
        self.message = message
        self.delay_s = delay_s
        self.available = available
        self.answers = answers
        self.paths = []
        self.future = None

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, request):
        self.paths.append(request.path)
        response = (SimpleNamespace(success=self.success, message=self.message)
                    if self.answers else None)
        self.future = _Future(response, self.delay_s)
        return self.future


def _spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    """Keep rclpy's contract, in real time: done, or timeout_sec gone by."""
    assert timeout_sec is not None and timeout_sec >= 0.0, \
        'None or a negative timeout would make rclpy wait forever'
    if future.response is not None and future.delay_s <= timeout_sec:
        time.sleep(future.delay_s)
        future.answer()
    else:
        time.sleep(timeout_sec)


class _Logger:

    def __init__(self):
        self.lines = {'info': [], 'warn': [], 'error': []}

    def info(self, msg):
        self.lines['info'].append(str(msg))

    def warn(self, msg):
        self.lines['warn'].append(str(msg))

    def error(self, msg):
        self.lines['error'].append(str(msg))


class _Stub:
    """What _deliver_mission touches on self; the load path is the real one."""

    def __init__(self, load_client, timeout_s=15.0):
        self.load_client = load_client
        self._load_confirm_timeout_s = timeout_s
        self._logger = _Logger()
        self.calls = []
        self.started_at = None
        self.failures = []

    def get_logger(self):
        return self._logger

    def _announce_plan(self):
        self.calls.append('announce')

    def _call_abort_mission(self):
        self.calls.append('abort')

    def _call_load_mission(self, path):
        self.calls.append('load')
        return LLMPlannerNode._call_load_mission(self, path)

    def _call_start_mission(self):
        self.started_at = time.monotonic()
        self.calls.append('start')
        return True, 'starting in 15.0 s'

    def _report_delivery_failure(self, stage, message):
        self.failures.append((stage, message))


@pytest.fixture
def spin(monkeypatch):
    monkeypatch.setattr(llm_planner_node.rclpy, 'spin_until_future_complete',
                        _spin_until_future_complete)


def _deliver(stub):
    t0 = time.monotonic()
    started = LLMPlannerNode._deliver_mission(stub, MISSION_PATH)
    return started, time.monotonic() - t0


# -- start right after a successful load ----------------------------------

def test_start_is_called_within_0_1_s_of_a_successful_load(spin):
    client = _LoadClient()
    stub = _Stub(client)
    started, elapsed = _deliver(stub)

    assert started is True
    assert stub.calls == ['announce', 'abort', 'load', 'start']
    assert client.paths == [MISSION_PATH]
    assert stub.started_at - client.future.answered_at < 0.1
    assert elapsed < 0.1, 'no fixed delay anywhere in the delivery path'
    assert stub.failures == []
    assert any(line.startswith('start_mission OK: starting in 15.0 s')
               for line in stub._logger.lines['info'])


def test_a_late_confirmation_starts_on_arrival_not_at_the_timeout(spin):
    """The timeout is a ceiling: an answer after 0.3 s starts at 0.3 s, not at 15 s."""
    client = _LoadClient(delay_s=0.3)
    stub = _Stub(client, timeout_s=15.0)
    started, elapsed = _deliver(stub)

    assert started is True
    assert stub.started_at - client.future.answered_at < 0.1
    assert 0.3 <= elapsed < 1.0


# -- load failure -> no start ---------------------------------------------

def test_a_refused_load_never_calls_start_and_logs_why(spin):
    reason = "invalid mission: move 'm0' has no stop_condition"
    stub = _Stub(_LoadClient(success=False, message=reason))
    started, _ = _deliver(stub)

    assert started is False
    assert 'start' not in stub.calls
    assert stub.failures == [('load_mission', reason)]
    [error] = stub._logger.lines['error']
    assert error.startswith('load_mission FALLITO')
    assert reason in error
    assert 'start_mission NON chiamato' in error


# -- no confirmation within the timeout -> no start -----------------------

def test_no_confirmation_within_the_timeout_never_calls_start(spin):
    stub = _Stub(_LoadClient(answers=False), timeout_s=0.2)
    started, elapsed = _deliver(stub)

    assert started is False
    assert 'start' not in stub.calls
    assert 0.2 <= elapsed < 0.7, 'gives up at the timeout, not before or long after'
    [(stage, reason)] = stub.failures
    assert stage == 'load_mission'
    assert reason == '/mission/load_mission: nessuna conferma entro 0.2s'
    [error] = stub._logger.lines['error']
    assert error.startswith('load_mission NON CONFERMATO')
    assert 'start_mission NON chiamato' in error


def test_a_confirmation_later_than_the_timeout_never_calls_start(spin):
    client = _LoadClient(delay_s=0.5)
    stub = _Stub(client, timeout_s=0.2)
    started, elapsed = _deliver(stub)

    assert started is False
    assert 'start' not in stub.calls
    assert client.future.answered_at is None
    assert elapsed < 0.5


def test_an_unreachable_loader_never_calls_start(spin):
    stub = _Stub(_LoadClient(available=False))
    started, _ = _deliver(stub)

    assert started is False
    assert 'start' not in stub.calls
    [(stage, reason)] = stub.failures
    assert stage == 'load_mission'
    assert 'non disponibile entro 15.0s' in reason


# -- the deadline itself, with a fake clock -----------------------------------

class _Clock:

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_discovery_and_answer_share_one_deadline():
    """Discovery that took 4 s of a 15 s budget leaves the answer 11 s."""
    clock = _Clock()
    spun = []

    class _SlowDiscovery(_LoadClient):
        def wait_for_service(self, timeout_sec=None):
            clock.now += 4.0
            return True

    wait_for_load_confirmation(
        _SlowDiscovery(answers=False), SimpleNamespace(path=MISSION_PATH),
        lambda future, timeout_sec: spun.append(timeout_sec), 15.0, clock)
    assert spun == [pytest.approx(11.0)]


def test_spin_never_gets_a_negative_timeout_after_an_overrun_discovery():
    """The wait_for_service of rclpy polls every 0.25 s and can overrun its timeout."""
    clock = _Clock()
    spun = []

    class _OverrunDiscovery(_LoadClient):
        def wait_for_service(self, timeout_sec=None):
            clock.now += timeout_sec + 0.25
            return True

    result = wait_for_load_confirmation(
        _OverrunDiscovery(answers=False), SimpleNamespace(path=MISSION_PATH),
        lambda future, timeout_sec: spun.append(timeout_sec), 15.0, clock)
    assert spun == [0.0]
    assert result.loaded is False and result.timed_out is True


# -- the shipped default (a coupling with stack_params.yaml, see CLAUDE.md) ---

def test_the_load_confirmation_timeout_ships_as_15_s_in_stack_params():
    assert load_confirm_timeout_default() == 15.0
