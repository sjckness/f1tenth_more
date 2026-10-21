"""
llm2_mpc_param_node against a fake llama-server: what it publishes, and when.

The node is built for real (rclpy, its own parameters) but never spun: each
test calls its timer callback by hand and replaces the two publishers with
recorders, so nothing depends on DDS discovery or on timing of the 0.5 Hz
timer. The request runs on the node's own worker thread, as on the car.
/llm2/set_active is called the same way, through its callback (one test
pins the name and type it is served under).

Also here: the mock set is a coupling with stack_params.yaml (CLAUDE.md). It
must stay the launch-path values, or the "round trip changes nothing" premise
of the mock is false and MPC_corr would apply a different weight.
"""

import json
import pathlib
import time

from llm.llm2_mpc_param_node import Llm2MpcParamNode
from llm.llm2_protocol import MOCK_PARAMS, SOURCE
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool
import yaml

LLM_DIR = pathlib.Path(__file__).resolve().parents[1]
STACK_PARAMS = LLM_DIR.parents[1] / 'f1tenth_params' / 'config' / 'stack_params.yaml'


class Recorder:
    """Stands in for a publisher: keeps what would have been published."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def make_node(ros):
    made = []

    def make(server, **params):
        overrides = [Parameter('server_url', value=server.url)]
        overrides += [Parameter(name, value=value) for name, value in params.items()]
        node = Llm2MpcParamNode(parameter_overrides=overrides)
        node.destroy_timer(node._timer)     # ticks only by hand
        node._update_pub = Recorder()
        node._status_pub = Recorder()
        made.append(node)
        return node

    yield make
    for node in made:
        node.destroy_node()


def statuses(node):
    return [json.loads(msg.data) for msg in node._status_pub.msgs]


def wait_for_statuses(node, count, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while len(node._status_pub.msgs) < count:
        assert time.monotonic() < deadline, f'only {statuses(node)} after {timeout_s} s'
        time.sleep(0.01)
    return statuses(node)


def set_active(node, active):
    """Call /llm2/set_active's callback, as the service would."""
    return node._on_set_active(SetBool.Request(data=active), SetBool.Response())


def test_a_valid_echo_is_published_on_llm2_mpc_params_as_sent(make_node, fake_server):
    node = make_node(fake_server())
    node._on_tick()
    (status,) = wait_for_statuses(node, 1)
    (msg,) = node._update_pub.msgs
    assert msg.source == SOURCE == 'llm2_mock'
    assert msg.seq == status['seq'] == 1
    assert list(msg.names) == [name for name, _ in MOCK_PARAMS]
    assert list(msg.values) == [value for _, value in MOCK_PARAMS]
    assert status['outcome'] == 'ok' and status['echo_match'] is True
    assert status['prompt_ms'] == 12.5 and status['predicted_ms'] == 880.0
    assert status['counters']['ok'] == 1 and status['counters']['echo_match'] == 1


def test_a_valid_reply_that_is_not_an_echo_is_published_as_returned_with_echo_match_false(
        make_node, fake_server, echo_answer):
    # Valid but changed: published (MPC_corr's whitelist and ranges are the
    # gate on values), and the status says it was not an echo.
    def changed(body):
        status, reply = echo_answer(body)
        values = json.loads(reply['content'])
        values['mpc_w_v'] = 5.0
        return status, dict(reply, content=json.dumps(values))

    node = make_node(fake_server(answer=changed))
    node._on_tick()
    (status,) = wait_for_statuses(node, 1)
    assert status['outcome'] == 'ok' and status['echo_match'] is False
    assert status['counters']['ok'] == 1 and status['counters']['echo_match'] == 0
    (msg,) = node._update_pub.msgs
    sent = dict(MOCK_PARAMS)
    assert sent['mpc_w_v'] != 5.0
    assert dict(zip(msg.names, msg.values)) == dict(sent, mpc_w_v=5.0)


def test_a_tick_while_the_request_is_in_flight_is_skipped_not_sent(make_node, fake_server):
    server = fake_server(delay_s=0.5)
    node = make_node(server)
    node._on_tick()
    node._on_tick()
    node._on_tick()
    skipped = statuses(node)
    assert [s['outcome'] for s in skipped] == ['skipped', 'skipped']
    assert [s['in_flight_seq'] for s in skipped] == [1, 1]
    done = wait_for_statuses(node, 3)[-1]
    assert done['seq'] == 1 and done['outcome'] == 'ok'
    assert set(skipped[0]) == set(done)     # one key set for every tick
    assert done['counters']['skipped'] == 2
    assert len(server.bodies) == 1 and server.max_active == 1
    node._on_tick()     # the gate is open again
    assert wait_for_statuses(node, 4)[-1]['outcome'] == 'ok'
    assert len(server.bodies) == 2


def test_an_invalid_reply_publishes_nothing_and_says_why(make_node, fake_server):
    node = make_node(fake_server(answer=lambda body: (200, {'content': 'no JSON'})))
    node._on_tick()
    (status,) = wait_for_statuses(node, 1)
    assert node._update_pub.msgs == []
    assert status['outcome'] == 'invalid' and 'not one JSON value' in status['reason']
    assert status['counters']['invalid'] == 1


def test_the_short_timeout_applies_only_after_the_server_has_answered(
        make_node, fake_server, echo_answer):
    answers = iter([(503, {}), (200, None), (200, None)])

    def answer(body):
        status, _ = next(answers)
        return (status, {}) if status != 200 else echo_answer(body)

    node = make_node(fake_server(answer=answer),
                     request_timeout_s=2.0, warmup_timeout_s=7.0)
    for count in (1, 2, 3):
        node._on_tick()
        wait_for_statuses(node, count)
    assert [(s['outcome'], s['timeout_s']) for s in statuses(node)] == [
        ('error', 7.0),     # cold at start
        ('ok', 7.0),        # still cold: the last request got no answer
        ('ok', 2.0),        # warm
    ]


def test_a_paused_node_sends_and_publishes_nothing_until_resumed(make_node, fake_server):
    server = fake_server()
    node = make_node(server)
    for _ in range(2):      # pausing a paused node is accepted, nothing changes
        response = set_active(node, False)
        assert response.success and response.message == 'paused'
    node._on_tick()
    node._on_tick()
    paused = statuses(node)     # published at once: a paused tick waits for nothing
    # A tick that started a request would have claimed the gate already.
    assert node._gate.in_flight is None and server.bodies == []
    assert node._update_pub.msgs == []
    assert [(s['outcome'], s['state']) for s in paused] == [('paused', 'paused')] * 2
    assert paused[-1]['counters']['paused'] == 2 and paused[-1]['counters']['skipped'] == 0

    response = set_active(node, True)
    assert response.success and response.message == 'active'
    node._on_tick()
    done = wait_for_statuses(node, 3)[-1]
    assert (done['seq'], done['outcome'], done['state']) == (3, 'ok', 'active')
    assert set(done) == set(paused[0])     # one key set, paused or not
    assert len(server.bodies) == 1
    (msg,) = node._update_pub.msgs
    assert msg.seq == 3


@pytest.mark.parametrize('resumed_before_the_reply', [False, True])
def test_a_reply_to_a_request_in_flight_at_the_pause_is_discarded_not_published(
        make_node, fake_server, resumed_before_the_reply):
    server = fake_server(delay_s=0.5)
    node = make_node(server)
    node._on_tick()             # request 1 goes out
    assert set_active(node, False).message == 'paused'
    node._on_tick()             # paused while 1 is in flight: not a skip
    if resumed_before_the_reply:
        set_active(node, True)
    tick_2, tick_1 = wait_for_statuses(node, 2)
    assert (tick_2['outcome'], tick_2['in_flight_seq']) == ('paused', 1)
    assert (tick_1['seq'], tick_1['outcome'], tick_1['echo_match']) == (1, 'discarded', None)
    assert '(ok)' in tick_1['reason']   # the server did answer an exact echo
    assert tick_1['state'] == ('active' if resumed_before_the_reply else 'paused')
    assert node._update_pub.msgs == []
    counters = tick_1['counters']
    assert counters['discarded'] == 1 and counters['ok'] == counters['echo_match'] == 0
    assert counters['paused'] == 1 and counters['skipped'] == 0
    if resumed_before_the_reply:
        node._on_tick()         # the next request goes out and is used
        assert wait_for_statuses(node, 3)[-1]['outcome'] == 'ok'
        assert len(node._update_pub.msgs) == 1
    assert len(server.bodies) == (2 if resumed_before_the_reply else 1)


def test_a_status_published_after_the_pause_answer_says_paused(make_node, fake_server):
    # The worker publishes its status after its outcome is decided; a pause
    # answered in between must show in that status, or a UI that mirrors
    # 'state' flips back to active for a period.
    node = make_node(fake_server())
    answers = []
    log_outcome = node._log_outcome

    def log_then_pause(seq, result):     # runs between the outcome and the status
        log_outcome(seq, result)
        answers.append(set_active(node, False).message)

    node._log_outcome = log_then_pause
    node._on_tick()
    (status,) = wait_for_statuses(node, 1)
    assert answers == ['paused']
    assert len(node._update_pub.msgs) == 1     # published before the pause answer
    assert (status['outcome'], status['state']) == ('ok', 'paused')


def test_pause_resume_is_served_as_std_srvs_setbool_on_llm2_set_active(make_node, fake_server):
    # The app's gateway calls it by this name: the other tests call the
    # callback directly and would not notice a rename.
    node = make_node(fake_server())
    service = node._set_active_srv
    assert service.srv_name == '/llm2/set_active' and service.srv_type is SetBool
    assert service.callback == node._on_set_active


def test_start_active_false_starts_paused_and_param_set_cannot_resume_it(
        make_node, fake_server):
    server = fake_server()
    node = make_node(server, start_active=False)
    node._on_tick()
    (status,) = statuses(node)
    assert (status['outcome'], status['state']) == ('paused', 'paused')
    assert node._gate.in_flight is None and server.bodies == []
    # Read-only: the runtime switch is /llm2/set_active, not the parameter.
    (result,) = node.set_parameters([Parameter('start_active', value=True)])
    assert not result.successful
    node._on_tick()
    assert statuses(node)[-1]['outcome'] == 'paused'


def test_unpaired_mock_lists_stop_the_node_at_start(ros, fake_server):
    with pytest.raises(ValueError, match='must pair up'):
        Llm2MpcParamNode(parameter_overrides=[
            Parameter('server_url', value=fake_server().url),
            Parameter('mock_param_values', value=[9.0])])


# -- config couplings -----------------------------------------------------------

def test_the_mock_set_is_the_launch_path_value_of_every_weight_it_names():
    with open(LLM_DIR / 'config' / 'llm2_mpc_param_params.yaml') as f:
        node_params = yaml.safe_load(f)['/**']['ros__parameters']
    from_file = dict(zip(node_params['mock_param_names'], node_params['mock_param_values']))
    assert len(from_file) == len(node_params['mock_param_values'])
    assert from_file == dict(MOCK_PARAMS)
    if not STACK_PARAMS.is_file():
        pytest.skip('stack_params.yaml not reachable from the source tree')
    with open(STACK_PARAMS) as f:
        stack = yaml.safe_load(f)
    assert {name: stack[name]['default'] for name in from_file} == from_file


def test_the_shipped_params_file_starts_llm2_active():
    # enable_llm2 is the opt-in; a shipped false would start an LLM2 that
    # silently never asks anything.
    with open(LLM_DIR / 'config' / 'llm2_mpc_param_params.yaml') as f:
        node_params = yaml.safe_load(f)['/**']['ros__parameters']
    assert node_params['start_active'] is True


def test_the_llm2_interrogation_names_an_installed_executable_and_params_file():
    with open(LLM_DIR / 'config' / 'interrogations.yaml') as f:
        entry = yaml.safe_load(f)['llm2_mpc_params']
    assert (LLM_DIR / 'config' / entry['params_file']).is_file()
    setup_py = (LLM_DIR / 'setup.py').read_text()
    assert f"'{entry['executable']} = llm.llm2_mpc_param_node:main'" in setup_py
