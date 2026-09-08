"""Fires the runtime battery e-stop end to end, over real pub/sub.

WHY THIS FILE EXISTS. f1tenth_diagnostics' battery_voltage_check_node used to
fail closed: a low reading, or no reading at all, exited 1 and
f1tenth_hardware/vesc.launch.py then skipped ackermann_to_vesc_node and
vesc_to_odom_node for that entire boot. That startup gate is now advisory
(see that node's own module docstring), which is only defensible if the
RUNTIME battery cutout genuinely works -- otherwise the change would leave the
pack with no protection at all.

The archives could not answer that. Across 26 archived runs carrying
/behavior/tree_status, IsProximityTooClose trips 65 ticks in 3 runs and
IsBatteryLow trips zero -- the emergency lane demonstrably works, but the
battery condition specifically had never fired in a recorded run, and had no
test either. This file is that missing evidence, exercising the whole chain
the way the live stack wires it:

  vesc_driver_node  --VescStateStamped--> /sensors/core
    -> diagnostics_server_node (voltage vs. min_battery_voltage)
    --BatteryStatus--> /diagnostics/battery_status
      -> IsBatteryLow (BT emergency lane, unconditional)
        -> Stop --AckermannDriveStamped--> /safety_stop
          -> ackermann_mux (priority 200, beats navigation's 10)

The /sensors/core -> BatteryStatus half is covered on the publisher's own side
by f1tenth_diagnostics' test_diagnostics_server_battery.py. This file covers
BatteryStatus -> a zero-speed command actually on /safety_stop, and builds the
lane with the real create_root() rather than a hand-assembled copy, so a
future edit that drops IsBatteryLow out of the emergency lane fails here.

Real DDS, not direct callback calls: "is it wired" is the question, so the
message has to travel. Publisher and subscriber share one node, which is how
the delivery stays immediate under this stack's Fast-DDS Discovery Server
configuration without a discovery server being up.
"""
import time

import py_trees
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

from f1tenth_behavior.behavior_executor_node import create_root
from f1tenth_behavior.behaviours.is_battery_low import IsBatteryLow

from f1tenth_messages.msg import BatteryStatus

_EMERGENCY_FRAME_ID = 'base_link/emergency'


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _emergency_lane(root):
    """Return the root Selector's emergency lane."""
    lane = root.children[0]
    assert lane.name == 'emergency', (
        f'emergency must stay the highest-priority lane, found "{lane.name}"')
    return lane


def _find(subtree, behaviour_type):
    """Return the single behaviour of the given type inside a subtree."""
    found = [b for b in subtree.iterate() if isinstance(b, behaviour_type)]
    assert len(found) == 1, f'expected one {behaviour_type.__name__}, got {len(found)}'
    return found[0]


def _battery_status(voltage, ok, has_data=True, min_voltage=10.8):
    msg = BatteryStatus()
    msg.voltage = voltage
    msg.min_voltage = min_voltage
    msg.has_data = has_data
    msg.ok = ok
    return msg


def _spin(node, predicate, timeout_sec=5.0):
    """Spin the node until predicate() is true or the timeout expires."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


class _Harness:
    """A live emergency lane plus the topics on either side of it."""

    def __init__(self, name):
        self.node = Node(name)
        self.root = create_root()
        self.lane = _emergency_lane(self.root)
        for behaviour in self.lane.iterate():
            behaviour.setup(node=self.node)
        self.condition = _find(self.lane, IsBatteryLow)
        self.stops = []
        self.node.create_subscription(
            AckermannDriveStamped, 'safety_stop', self.stops.append, 10)
        self.battery_pub = self.node.create_publisher(
            BatteryStatus, self.condition.battery_status_topic, 10)

    def deliver(self, status):
        """Publish a BatteryStatus and wait for the condition to receive it."""
        self.battery_pub.publish(status)
        assert _spin(self.node, lambda: self.condition.latest is not None), (
            'BatteryStatus was never delivered to IsBatteryLow')

    def tick(self):
        self.lane.tick_once()
        _spin(self.node, lambda: bool(self.stops), timeout_sec=1.0)

    def close(self):
        self.node.destroy_node()


class TestEmergencyLaneWiring:
    """IsBatteryLow is present and highest-priority, whatever the toggles."""

    def test_battery_condition_is_first_in_the_emergency_lane(self):
        lane = _emergency_lane(create_root())
        condition_selector = lane.children[0]
        assert condition_selector.name == 'emergency_condition'
        # First child of a memory=False Selector: battery is evaluated before
        # any other emergency condition can short-circuit the lane.
        assert isinstance(condition_selector.children[0], IsBatteryLow)

    @pytest.mark.parametrize('flags', [
        {'enable_camera_obstacle_stop': False, 'enable_lidar_safety_stop': False},
        {'enable_camera_obstacle_stop': True, 'enable_lidar_safety_stop': False},
        {'enable_camera_obstacle_stop': False, 'enable_lidar_safety_stop': True},
        {'enable_camera_obstacle_stop': True, 'enable_lidar_safety_stop': True},
    ])
    def test_battery_condition_is_not_gateable(self, flags):
        """No config toggle removes it -- the property is_battery_low.py claims."""
        lane = _emergency_lane(create_root(**flags))
        assert _find(lane, IsBatteryLow) is not None

    def test_lane_ends_in_a_stop_on_the_mux_safety_lane(self):
        lane = _emergency_lane(create_root())
        stop = lane.children[-1]
        assert stop.output_topic == 'safety_stop'
        assert stop.frame_id == _EMERGENCY_FRAME_ID


class TestLowBatteryStopsTheCar:
    """THE verification: a low pack really does put a stop on the mux."""

    def test_low_battery_publishes_a_zero_speed_emergency_stop(self):
        h = _Harness('test_battery_estop_low')
        try:
            h.deliver(_battery_status(voltage=9.6, ok=False))
            h.tick()

            assert h.condition.status == py_trees.common.Status.SUCCESS, (
                'IsBatteryLow did not trip on a low BatteryStatus')
            assert h.lane.status == py_trees.common.Status.SUCCESS, (
                'the emergency lane did not run its Stop')
            assert h.stops, 'nothing was published on /safety_stop'

            stop = h.stops[-1]
            assert stop.drive.speed == 0.0
            assert stop.drive.steering_angle == 0.0
            # The mux republishes the winning message verbatim, so this is what
            # attributes the stop to the battery lane in a recorded bag.
            assert stop.header.frame_id == _EMERGENCY_FRAME_ID
        finally:
            h.close()


class TestHealthyBatteryDoesNotStopTheCar:
    """The other half: the condition must not trip when it should not."""

    def test_healthy_battery_leaves_the_condition_failing(self):
        h = _Harness('test_battery_estop_ok')
        try:
            h.deliver(_battery_status(voltage=12.4, ok=True))
            h.tick()

            assert h.condition.status == py_trees.common.Status.FAILURE
            assert not [s for s in h.stops
                        if s.header.frame_id == _EMERGENCY_FRAME_ID], (
                'a healthy battery must not raise an emergency stop')
        finally:
            h.close()

    def test_no_data_yet_does_not_trip_the_lane(self):
        """A cold-start tick before any /sensors/core sample is not "low"."""
        h = _Harness('test_battery_estop_nodata')
        try:
            h.deliver(_battery_status(voltage=0.0, ok=False, has_data=False))
            h.tick()

            assert h.condition.status == py_trees.common.Status.FAILURE
            assert not [s for s in h.stops
                        if s.header.frame_id == _EMERGENCY_FRAME_ID]
        finally:
            h.close()


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
