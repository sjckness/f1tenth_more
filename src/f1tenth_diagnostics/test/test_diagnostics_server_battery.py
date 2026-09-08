"""Covers the /sensors/core -> BatteryStatus half of the runtime battery cutout.

Companion to f1tenth_behavior's test_battery_low_runtime_estop.py, which covers
the other half (BatteryStatus -> BT emergency lane -> a zero-speed command on
/safety_stop). Together they are the evidence that made it defensible to turn
battery_voltage_check_node's STARTUP gate advisory: the check that actually
protects the pack is this continuous one, so it had to be shown to work rather
than assumed to. It had no test at all before, and had never been observed
firing in any archived run.

diagnostics_server_node is constructed for real but never spun; its callbacks
and publish tick are driven directly, the same convention this package's other
node tests use.
"""
import pytest
import rclpy
from vesc_msgs.msg import VescState, VescStateStamped

from f1tenth_diagnostics.diagnostics_server_node import DiagnosticsServerNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture()
def server():
    node = DiagnosticsServerNode()
    published = []
    node._battery_pub.publish = published.append
    node.published = published
    yield node
    node.destroy_node()


def _core_sample(voltage):
    msg = VescStateStamped()
    msg.state = VescState()
    msg.state.voltage_input = voltage
    return msg


class TestBatteryStatusFromCoreSamples:
    """The verdict the BT's IsBatteryLow reads is computed here, not there."""

    def test_low_voltage_publishes_not_ok(self, server):
        server.min_battery_voltage = 10.8
        server._vesc_callback(_core_sample(9.6))
        server._publish_battery_status()

        status = server.published[-1]
        assert status.has_data is True
        assert status.ok is False
        assert status.voltage == pytest.approx(9.6)
        assert status.min_voltage == pytest.approx(10.8)

    def test_healthy_voltage_publishes_ok(self, server):
        server.min_battery_voltage = 10.8
        server._vesc_callback(_core_sample(12.4))
        server._publish_battery_status()

        status = server.published[-1]
        assert status.has_data is True
        assert status.ok is True

    def test_no_samples_yet_is_not_reported_as_low(self, server):
        """The cold-start guard: "no data" must never look like "battery low"."""
        server._publish_battery_status()

        status = server.published[-1]
        assert status.has_data is False
        # ok is false too, but IsBatteryLow gates on has_data first -- see
        # is_battery_low.py. What matters is that it is distinguishable.
        assert status.ok is False

    def test_verdict_tracks_the_pack_draining_across_ticks(self, server):
        """A pack that sags below the threshold flips the verdict on that tick."""
        server.min_battery_voltage = 10.8

        server._vesc_callback(_core_sample(11.4))
        server._publish_battery_status()
        assert server.published[-1].ok is True

        # Self-resetting bucket: the next tick averages only new samples, so a
        # drained pack is not held up by earlier healthy ones.
        server._vesc_callback(_core_sample(10.2))
        server._publish_battery_status()
        assert server.published[-1].ok is False
        assert server.published[-1].voltage == pytest.approx(10.2)
