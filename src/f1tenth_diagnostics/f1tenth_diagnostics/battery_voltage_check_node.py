"""One-time startup battery-voltage gate for the VESC drive stack.

Samples VescStateStamped.state.voltage_input on state_topic (default
/sensors/core, published by vesc_driver_node) over a short averaging window
(default 2.0s at vesc_driver_node's 50Hz telemetry poll rate -> ~100
samples), compares the mean against min_battery_voltage (default 10.8V),
logs the result, and exits with returncode 0 (pass) or 1 (fail).

This is a ONE-TIME pre-flight check, not continuous monitoring: it is meant
to be launched once, alongside a standalone vesc_driver_node instance,
before the rest of the drive stack (ackermann_to_vesc_node, the rest of the
sensor driver group, ekf/Nav2) -- see f1tenth_hardware/launch/vesc.launch.py,
which gates that launch on this node's exit code via a launch event handler
inspecting ProcessExited.returncode. There is deliberately no BT condition
node or Stop-node integration here -- continuous voltage monitoring during
operation is a separate, later feature.

A short averaging window (rather than a single sample) is used because no
historical noise data for voltage_input exists in this repo to justify
trusting one reading near a hard safety cutoff -- VESC UART telemetry can
have occasional dropped/garbled packets, and averaging ~100 samples over 2s
is cheap relative to the cost of a false pass/fail at boot.
"""

import statistics
import sys

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped


class BatteryVoltageCheckNode(Node):

    def __init__(self):
        super().__init__('battery_voltage_check_node')

        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        self.min_battery_voltage = float(
            self.declare_parameter('min_battery_voltage', 10.8).value)
        self.sample_window_sec = float(
            self.declare_parameter('sample_window_sec', 2.0).value)

        self.exit_code = 0
        self.done = False
        self._samples = []

        self._sub = self.create_subscription(
            VescStateStamped, self.state_topic, self._state_callback, 50)
        self._timer = self.create_timer(self.sample_window_sec, self._finish)

        self.get_logger().info(
            f'Sampling battery voltage on "{self.state_topic}" for '
            f'{self.sample_window_sec:.1f}s (min_battery_voltage='
            f'{self.min_battery_voltage:.1f}V).')

    def _state_callback(self, msg: VescStateStamped):
        self._samples.append(msg.state.voltage_input)

    def _finish(self):
        self._timer.cancel()
        self._sub.destroy()

        if not self._samples:
            self.get_logger().error(
                f'STARTUP ABORTED: no samples received on "{self.state_topic}" -- is '
                'vesc_driver_node publishing? Cannot verify battery voltage.')
            self.exit_code = 1
            self.done = True
            return

        voltage = statistics.mean(self._samples)
        if voltage >= self.min_battery_voltage:
            self.get_logger().info(
                f'Battery check passed: {voltage:.1f}V >= '
                f'{self.min_battery_voltage:.1f}V minimum '
                f'(n={len(self._samples)} samples).')
            self.exit_code = 0
        else:
            self.get_logger().error(
                f'STARTUP ABORTED: battery voltage {voltage:.1f}V is below minimum '
                f'{self.min_battery_voltage:.1f}V -- charge or replace battery before '
                f'starting (n={len(self._samples)} samples).')
            self.exit_code = 1
        self.done = True


def main():
    rclpy.init()
    node = BatteryVoltageCheckNode()
    try:
        # NOT rclpy.spin(node): calling rclpy.shutdown() from inside a callback
        # running under the executor deadlocks (executor.shutdown() waits for that
        # same callback to finish -- confirmed live). Callbacks only set node.done;
        # shutdown happens here, in the main thread, once the loop notices it.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
