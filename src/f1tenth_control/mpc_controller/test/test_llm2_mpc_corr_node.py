"""
MPC_corr's LLM2 block on the real node (MPC_corr._llm2_*).

Builds the real MPCController (stack_params defaults, the real control_loop)
on a private, localhost-only DDS domain, like test_speed_channel_node.py: no
topics, no stack. Updates go straight into the subscription callback; the
/mpc/applied_params publisher is replaced by a recorder. What it pins:

  - llm2_params_enabled false (the default): no subscription, no publisher,
    no whitelist parameters, weights untouched;
  - true: an update is applied at the NEXT control_loop tick, not by the
    callback -- even one arriving in the middle of a solve -- and then shows
    in self.weights (what the solver reads), in the declared parameter (what
    `ros2 param get` reads) and on /mpc/applied_params with the same seq;
  - a rejected update changes nothing, and does not displace a valid one;
  - with allow_reverse on, an update raising mpc_w_v past w_rev / (2 vdes)
    is rejected;
  - the switch and the whitelist are read-only: a runtime `ros2 param set`
    is refused, not accepted and ignored;
  - a whitelist naming a safety parameter, or mpc_w_obs, refuses the start.

Run standalone: python3 -m pytest test/test_llm2_mpc_corr_node.py -v
"""

import math
import os

from f1tenth_params.param_defaults import get_value
import pytest


@pytest.fixture
def make_node(monkeypatch):
    """Build an MPCController with `overrides` (ROS args), torn down after the test."""
    rclpy = pytest.importorskip('rclpy')
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '147'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    made = []

    def _make(**overrides):
        from mpc_controller.MPC_corr import MPCController
        args = ['--ros-args', '--log-level', 'fatal']
        for key, value in overrides.items():
            args += ['-p', f'{key}:={value}']
        if not rclpy.ok():
            rclpy.init(args=args)
        node = MPCController()
        made.append(node)
        return node

    yield _make
    for node in made:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _update(seq, **values):
    from f1tenth_messages.msg import MpcParamUpdate
    msg = MpcParamUpdate()
    msg.seq = seq
    msg.source = 'test'
    msg.names = list(values)
    msg.values = [float(v) for v in values.values()]
    return msg


def _enabled(make_node):
    node = make_node(llm2_params_enabled='true')
    applied = []
    node.llm2_applied_pub.publish = applied.append
    return node, applied


_W_V = float(get_value('mpc_w_v'))
_W_TERM = float(get_value('mpc_w_term'))


def test_disabled_by_default_means_no_subscription_and_no_publisher(make_node):
    node = make_node()
    assert node.get_parameter('llm2_params_enabled').value is False
    assert '/llm2/mpc_params' not in [s.topic_name for s in node.subscriptions]
    assert '/mpc/applied_params' not in [p.topic_name for p in node.publishers]
    assert not node.has_parameter('llm2_param_whitelist')
    weights = dict(node.weights)
    node.control_loop()
    assert node.weights == weights


def test_enabled_an_update_is_applied_at_the_next_tick_not_by_the_callback(make_node):
    node, applied = _enabled(make_node)
    assert [s.topic_name for s in node.subscriptions].count('/llm2/mpc_params') == 1

    node._llm2_params_callback(_update(7, mpc_w_v=5.0, mpc_w_term=12.0))
    # Parked only: neither the solver's dict nor the parameter has moved.
    assert node.weights['w_v'] == _W_V and node.weights['w_term'] == _W_TERM
    assert node.get_parameter('mpc_w_v').value == _W_V
    assert applied == []

    node.control_loop()
    assert node.weights['w_v'] == 5.0 and node.weights['w_term'] == 12.0
    assert node.get_parameter('mpc_w_v').value == 5.0
    assert node.get_parameter('mpc_w_term').value == 12.0
    (msg,) = applied
    assert msg.seq == 7 and msg.source == 'test'
    assert dict(zip(msg.names, msg.values)) == {'mpc_w_v': 5.0, 'mpc_w_term': 12.0}

    node.control_loop()
    assert len(applied) == 1, 'applied again with nothing pending'


def test_an_update_arriving_during_a_solve_waits_for_the_next_tick(make_node):
    from f1tenth_messages.msg import DriveCommand
    from nav_msgs.msg import Odometry
    node, applied = _enabled(make_node)
    node.pub.publish = lambda m: None
    solve = node._mpc_solve
    seen = []

    def spying_solve(**kw):
        seen.append((kw['weights']['w_v'], node.weights['w_v']))
        if len(seen) == 1:
            node._llm2_params_callback(_update(3, mpc_w_v=6.0))
            seen.append(('after inject', node.weights['w_v']))
        return solve(**kw)

    node._mpc_solve = spying_solve

    def tick(k):
        m = Odometry()
        m.header.stamp.sec = k
        m.pose.pose.orientation.w = 1.0
        node.hw_odom_callback(m)
        node.control_loop()

    tick(1)
    cmd = DriveCommand()
    cmd.mode = 'straight'
    cmd.speed = 0.4
    cmd.d_safe = -1.0
    node.goal_drive_callback(cmd)
    k = 2
    while not seen and k < 10:
        tick(k)
        k += 1
    assert seen, 'control_loop never reached the solve'
    assert seen[:2] == [(_W_V, _W_V), ('after inject', _W_V)]
    assert node.weights['w_v'] == _W_V, 'applied before the tick ended'
    assert applied == []
    tick(k)
    assert seen[2] == (6.0, 6.0), 'the next solve did not get the update'
    assert [m.seq for m in applied] == [3]


@pytest.mark.parametrize('values', [
    {'mpc_w_v': 5.0, 'car_radius': 0.1},       # one non-whitelisted name
    {'mpc_w_v': 5.0, 'mpc_w_obs': 8.0},        # a weight, but not whitelisted
    {'mpc_w_v': math.nan},
    {'mpc_w_v': 5.0, 'mpc_w_term': 1000.0},    # out of range
])
def test_a_rejected_update_changes_nothing(make_node, values):
    node, applied = _enabled(make_node)
    weights = dict(node.weights)
    node._llm2_params_callback(_update(9, **values))
    node.control_loop()
    assert node.weights == weights
    assert node.get_parameter('mpc_w_v').value == _W_V
    assert applied == []


def test_a_rejected_update_does_not_displace_a_parked_valid_one(make_node):
    node, applied = _enabled(make_node)
    node._llm2_params_callback(_update(1, mpc_w_v=5.0))
    node._llm2_params_callback(_update(2, mpc_w_v=50.0))
    node.control_loop()
    assert node.weights['w_v'] == 5.0
    assert [m.seq for m in applied] == [1]


def test_with_allow_reverse_on_an_update_raising_w_v_past_the_invariant_is_rejected(
        make_node):
    # reverse_mpc: w_rev >= 2 * w_v * vdes. Shipped: 4.3 / (2 * 0.5) = 4.3,
    # inside the whitelist's [1, 10] for mpc_w_v.
    node = make_node(llm2_params_enabled='true', allow_reverse='true',
                     reverse_auto_enabled='false')
    assert node.reverse_planner is not None
    applied = []
    node.llm2_applied_pub.publish = applied.append
    node._llm2_params_callback(_update(1, mpc_w_v=10.0))
    node.control_loop()
    assert node.weights['w_v'] == _W_V and applied == []
    node._llm2_params_callback(_update(2, mpc_w_v=4.0))
    node.control_loop()
    assert node.weights['w_v'] == 4.0 and [m.seq for m in applied] == [2]


def test_a_runtime_param_set_cannot_switch_llm2_off_or_change_the_whitelist(make_node):
    from rclpy.parameter import Parameter
    node, applied = _enabled(make_node)
    max_w_v = list(node.get_parameter('llm2_param_max').value)
    tighter = [min(v, 5.0) for v in max_w_v]
    results = node.set_parameters([
        Parameter('llm2_params_enabled', Parameter.Type.BOOL, False),
        Parameter('llm2_param_max', Parameter.Type.DOUBLE_ARRAY, tighter)])
    assert [r.successful for r in results] == [False, False]
    assert all('read-only' in r.reason for r in results)
    # What `ros2 param get` shows is what is in force.
    assert node.get_parameter('llm2_params_enabled').value is True
    assert list(node.get_parameter('llm2_param_max').value) == max_w_v
    node._llm2_params_callback(_update(4, mpc_w_v=9.0))
    node.control_loop()
    assert node.weights['w_v'] == 9.0 and [m.seq for m in applied] == [4]


def test_a_runtime_param_set_cannot_switch_llm2_on(make_node):
    from rclpy.parameter import Parameter
    node = make_node()
    (result,) = node.set_parameters([Parameter('llm2_params_enabled', Parameter.Type.BOOL, True)])
    assert not result.successful
    assert node.get_parameter('llm2_params_enabled').value is False


@pytest.mark.parametrize('name, match', [
    ('car_radius', 'not an MPC cost weight'),
    ('mpc_w_obs', 'never tunable'),
])
def test_a_whitelist_naming_a_safety_parameter_refuses_the_start(make_node, name, match):
    with pytest.raises(ValueError, match=match):
        make_node(llm2_params_enabled='true', llm2_param_whitelist=f"['{name}']",
                  llm2_param_min='[0.0]', llm2_param_max='[1.0]')
