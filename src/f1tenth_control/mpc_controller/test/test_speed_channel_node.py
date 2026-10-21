"""
The speed law inside the real node: MPCController, both modes, a few seconds each.

Builds the real MPCController (stack_params defaults, the real control_loop),
feeds odometry through its own callback and closes the loop with a small
first-order plant here -- no topics, no stack: the publishers are stubbed and
the node runs on a private, localhost-only DDS domain. What it pins is the
wiring the rig cannot see: the parameters, the /ackermann_drive
subscription, the per-publish /mpc/speed_channel status, the resets, and that
legacy mode still publishes v + a * ts.

Run standalone: python3 -m pytest test/test_speed_channel_node.py -v
"""

import math
import os

import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def make_node(monkeypatch):
    """Build an MPCController with `overrides` (ROS args), torn down after the test."""
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '147'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    made = []

    def _make(**overrides):
        from mpc_controller.MPC_corr import MPCController
        args = ['--ros-args', '--log-level', 'fatal']
        for key, value in overrides.items():
            args += ['-p', f'{key}:={value}']
        rclpy.init(args=args)
        node = MPCController()
        made.append(node)
        return node

    yield _make
    for node in made:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


class _Loop:
    """Odometry in through hw_odom_callback, /drive out through a stub, a lag plant between."""

    def __init__(self, node, gain=0.86, tau=0.176):
        from nav_msgs.msg import Odometry
        self.node, self.gain, self.alpha = node, gain, 1.0 - math.exp(-0.1 / tau)
        self.Odometry = Odometry
        self.v, self.x, self.k = 0.0, 0.0, 0
        self.sent, self.status = [], []
        node.pub.publish = lambda m: self.sent.append(float(m.drive.speed))
        node.speed_channel_pub.publish = lambda m: self.status.append(m)

    def tick(self, n=1):
        for _ in range(n):
            self.k += 1
            m = self.Odometry()
            m.header.stamp.sec = self.k
            m.pose.pose.position.x = self.x
            m.pose.pose.orientation.w = 1.0
            m.twist.twist.linear.x = self.v
            self.node.hw_odom_callback(m)
            self.node.control_loop()
            c = self.sent[-1]
            self.v += self.alpha * (self.gain * c - self.v)
            self.x += self.v * 0.1

    def drive(self, speed=0.5):
        from f1tenth_messages.msg import DriveCommand
        cmd = DriveCommand()
        cmd.mode = 'straight'
        cmd.speed = float(speed)
        cmd.d_safe = -1.0
        self.node.goal_drive_callback(cmd)


def test_offset_free_is_the_default_and_beats_the_fixed_point(make_node):
    node = make_node()
    assert node.speed_channel.cfg.mode == 'offset_free'
    loop = _Loop(node, gain=0.80)
    loop.tick()
    loop.drive(0.4)
    loop.tick(60)
    # 0.4 / 0.8 = 0.5: exactly the ceiling. Legacy would sit near 0.25.
    assert loop.v == pytest.approx(0.4, abs=0.02)
    assert max(loop.sent) <= node.max_forward_speed + 1e-9
    st = loop.status[-1]
    assert st.mode == 'offset_free'
    assert st.d_hat == pytest.approx(-0.1, abs=0.03)
    assert len(loop.status) == len(loop.sent)       # one status per /drive


def test_the_start_jumps_to_the_breakaway_bound(make_node):
    node = make_node()
    loop = _Loop(node)
    loop.tick()
    loop.drive(0.4)
    loop.tick(4)
    # Tick 1 of the move plans from rest; tick 2 sees the plan wanted to move
    # and jumps to v_breakaway instead of ramping from zero.
    assert loop.sent[-4] < 0.05
    assert max(loop.sent[-3:]) >= node.speed_channel.cfg.breakaway_mps - 1e-9


def test_a_hold_publishes_zero_resets_and_restarts_cleanly(make_node):
    from std_msgs.msg import Bool
    node = make_node()
    loop = _Loop(node)
    loop.tick()
    loop.drive(0.4)
    loop.tick(30)
    node.hold_callback(Bool(data=True))
    loop.tick(10)
    assert loop.sent[-1] == 0.0
    assert loop.status[-1].last_reset == 'hold' and loop.status[-1].d_hat == 0.0
    assert 'hold' in loop.status[-1].freeze_reasons
    node.hold_callback(Bool(data=False))
    loop.tick(1)
    assert loop.sent[-1] < 0.05              # from the zero the hold left in force
    # Back to vdes within 2.5 s. Not longer: this loop runs far faster than
    # real time, and the node rebuilds its corridor on the WALL clock
    # (corridor_update_period), so a long run here drives into a corridor end
    # the real 10 Hz loop would have moved long before.
    loop.tick(25)
    assert loop.v == pytest.approx(0.4, abs=0.02)


def test_another_lane_on_the_mux_is_seen(make_node):
    node = make_node()
    loop = _Loop(node)
    loop.tick()
    loop.drive(0.4)
    loop.tick(20)
    now = node.get_clock().now().nanoseconds * 1e-9
    node.speed_channel.note_applied(0.0, now)       # safety_stop owns the mux
    loop.tick(1)
    assert loop.status[-1].override and 'override' in loop.status[-1].freeze_reasons


def test_legacy_mode_publishes_v_plus_a_ts(make_node):
    node = make_node(speed_command_mode='legacy')
    assert node.speed_channel.cfg.mode == 'legacy'
    loop = _Loop(node, gain=0.758)
    loop.tick()
    loop.drive(0.5)
    loop.tick(5)
    for _ in range(20):
        v_before = loop.v
        loop.tick(1)
        expected = min(v_before + float(node.last_u[1]) * node.ts, node.max_forward_speed)
        assert loop.sent[-1] == pytest.approx(max(expected, 0.0), abs=1e-9)
    assert loop.status[-1].mode == 'legacy'
