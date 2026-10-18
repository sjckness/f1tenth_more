"""
reset_manager costs nothing between resets (cpu-cleanup, 2026-10-16).

It used to hold four Odometry/Pose subscriptions and a 20 Hz timer for its
whole life (36% of a core on the parked Orin). Now both exist only while a
/reset_all runs. These tests build the real node under rclpy, with every step
switched off so no other node is needed.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

rclpy = pytest.importorskip('rclpy')
reset_manager_node = pytest.importorskip('reset_manager_node')


@pytest.fixture
def node():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'stop.enabled:=false', '-p', 'wheel_odom.enabled:=false',
        '-p', 'ekf_local.enabled:=false', '-p', 'slam.enabled:=false',
        '-p', 'ekf_global.enabled:=false', '-p', "stop.abort_mission_service:=''",
        '-p', "hooks.restart_components:=['']",
        '-p', 'service_name:=/reset_all_idle_test',
    ])
    manager = reset_manager_node.ResetManager()
    yield manager
    manager.destroy_node()
    rclpy.shutdown()


def test_idle_node_has_no_subscriptions_and_no_timers(node):
    assert list(node.subscriptions) == []
    assert list(node.timers) == []


def test_watches_open_for_a_reset_and_close_after_it(node):
    node._open_watches()
    topics = sorted(sub.topic_name for sub in node.subscriptions)
    assert topics == sorted({
        '/odometry/filtered', '/odom', '/ekf_global/odometry/filtered', '/slam/pose'})
    node._close_watches()
    assert list(node.subscriptions) == []


def test_a_whole_reset_leaves_nothing_behind(node):
    from std_srvs.srv import Trigger
    response = node._on_reset_all(Trigger.Request(), Trigger.Response())
    assert response.success, response.message
    assert list(node.subscriptions) == []
    assert list(node.timers) == []


def test_the_hold_timer_exists_only_while_holding(node):
    node._holding = False
    node._open_watches()
    # What _step_stop does before it waits for a standstill.
    node.stop_rate = 20.0
    node._holding = True
    node._hold_timer = node.create_timer(
        1.0 / node.stop_rate, node._hold_tick, callback_group=node._group)
    assert len(list(node.timers)) == 1
    assert node._hold_timer.timer_period_ns == pytest.approx(50_000_000)
    node._holding = False
    node._close_watches()
    assert list(node.timers) == []
