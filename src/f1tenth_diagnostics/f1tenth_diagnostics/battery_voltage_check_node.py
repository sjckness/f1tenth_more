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

max_wait_for_first_sample_sec (startup-race fix): sample_window_sec used to
ALSO be the deadline for the very first sample ever arriving, which silently
conflated two different things -- "how long the VESC serial handshake takes
before ANY telemetry exists" (cold-start, happens once, duration unknown up
front) vs. "how many samples to average once telemetry is already flowing"
(steady-state, ~2s is plenty for noise reduction). Confirmed live (this
node's own log, two independent cold boots) that the handshake alone can
exceed the old fixed 2.0s window with zero samples the entire time --
vesc_driver_node logged "Connected to VESC" ~0.9-1.0s in, and the following
full 2.0s sampling window still saw not one message -- which made this check
fail-close on a hardware/firmware timing artifact, not a real "no VESC
connected" or "battery low" condition, permanently blocking
ackermann_to_vesc_node/vesc_to_odom_node for that whole boot (no retry).
Fixed by splitting into two phases (see _poll() below): poll for the first
sample, up to max_wait_for_first_sample_sec (default 10.0s -- generous
margin over the ~2.9s already observed as insufficient); only once a first
sample lands does the unchanged sample_window_sec averaging window start.
This still fails closed -- same safety intent, same abort behavior -- it
just no longer mistakes "still warming up" for "no VESC".
"""

import statistics
import sys

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped

_POLL_PERIOD_SEC = 0.2


class BatteryVoltageCheckNode(Node):

    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node -- lets tests pass
        # parameter_overrides directly (see gyro_bias_calibration_node.py's
        # own __init__ for the same pattern already established in this
        # package), not otherwise used by main() below.
        super().__init__('battery_voltage_check_node', **kwargs)

        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        self.min_battery_voltage = float(
            self.declare_parameter('min_battery_voltage', 10.8).value)
        self.sample_window_sec = float(
            self.declare_parameter('sample_window_sec', 2.0).value)
        self.max_wait_for_first_sample_sec = float(
            self.declare_parameter('max_wait_for_first_sample_sec', 10.0).value)

        self.exit_code = 0
        self.done = False
        self._samples = []
        self._start_time = self.get_clock().now()
        self._first_sample_time = None

        self._sub = self.create_subscription(
            VescStateStamped, self.state_topic, self._state_callback, 50)
        # Polls rather than a single one-shot timer: the deadline for "first
        # sample ever" and the deadline for "averaging window complete" are
        # two different, sequential conditions now (see module docstring) --
        # _poll() checks whichever one is currently active every tick instead
        # of needing two separately-scheduled timer objects.
        self._poll_timer = self.create_timer(_POLL_PERIOD_SEC, self._poll)

        self.get_logger().info(
            f'Waiting up to {self.max_wait_for_first_sample_sec:.1f}s for the first '
            f'telemetry sample on "{self.state_topic}", then averaging over '
            f'{self.sample_window_sec:.1f}s (min_battery_voltage='
            f'{self.min_battery_voltage:.1f}V).')

    def _state_callback(self, msg: VescStateStamped):
        if self._first_sample_time is None:
            self._first_sample_time = self.get_clock().now()
            elapsed = (self._first_sample_time - self._start_time).nanoseconds / 1e9
            self.get_logger().info(
                f'First telemetry sample received after {elapsed:.2f}s -- '
                f'averaging over the next {self.sample_window_sec:.1f}s.')
        self._samples.append(msg.state.voltage_input)

    def _poll(self):
        now = self.get_clock().now()

        if self._first_sample_time is None:
            elapsed = (now - self._start_time).nanoseconds / 1e9
            if elapsed >= self.max_wait_for_first_sample_sec:
                self._poll_timer.cancel()
                self._sub.destroy()
                self.get_logger().error(
                    f'STARTUP ABORTED: no telemetry received on "{self.state_topic}" '
                    f'within {self.max_wait_for_first_sample_sec:.1f}s of startup -- '
                    'check vesc_driver_node\'s own log for a "Connected to VESC" line '
                    '(no such line: VESC not connected/responding; a "Connected" line '
                    'with still no telemetry here: raise max_wait_for_first_sample_sec, '
                    'the handshake is just slower than expected). Cannot verify '
                    'battery voltage.')
                self.exit_code = 1
                self.done = True
            return

        elapsed_since_first = (now - self._first_sample_time).nanoseconds / 1e9
        if elapsed_since_first >= self.sample_window_sec:
            self._finish()

    def _finish(self):
        self._poll_timer.cancel()
        self._sub.destroy()

        # Always non-empty by construction: _finish() is only reached via
        # _poll()'s second branch, which requires _first_sample_time to
        # already be set, which _state_callback only sets after appending
        # that same first sample.
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
