"""
llm2_mpc_param_node against a fake llama-server: what it publishes, and when.

The node is built for real (rclpy, its own parameters) but never spun: each
test calls its timer callback by hand and replaces the two publishers with
recorders, so nothing depends on DDS discovery or on timing of the 0.5 Hz
timer. The request runs on the node's own worker thread, as on the car.

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


def test_the_llm2_interrogation_names_an_installed_executable_and_params_file():
    with open(LLM_DIR / 'config' / 'interrogations.yaml') as f:
        entry = yaml.safe_load(f)['llm2_mpc_params']
    assert (LLM_DIR / 'config' / entry['params_file']).is_file()
    setup_py = (LLM_DIR / 'setup.py').read_text()
    assert f"'{entry['executable']} = llm.llm2_mpc_param_node:main'" in setup_py
