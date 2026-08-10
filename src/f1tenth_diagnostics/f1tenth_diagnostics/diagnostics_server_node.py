"""Persistent diagnostics coordinator: continuous battery-voltage safety monitoring
plus an on-demand snapshot service, deliberately decoupled from system_observer_node's
enable_sys_obs gate -- battery safety monitoring must keep running even with sys_obs
disabled (see f1tenth_params/config/stack_params.yaml's enable_sys_obs comment).

Publishes f1tenth_messages/BatteryStatus on /diagnostics/battery_status at
battery_check_rate_hz (default 2.0 Hz), computed from VescStateStamped.state.
voltage_input samples received on state_topic (default /sensors/core -- same source
battery_voltage_check_node samples). This node does NOT replace that one-time startup
gate (see f1tenth_hardware/vesc.launch.py, which still runs it exactly as before): this
adds continuous monitoring for f1tenth_behavior's BT emergency lane (see
f1tenth_behavior/behaviours/is_battery_low.py), a genuinely different concern (during-
operation safety trip vs. pre-flight veto). Averages samples received since the last
publish tick (self-resetting bucket, same noise-rejection rationale
battery_voltage_check_node's own docstring gives -- VESC UART telemetry can have
occasional dropped/garbled packets). has_data stays false (and ok stays false) until the
first /sensors/core sample has actually been received, so a cold-start BT tick or an
early run_diagnostics call can't misread "no data yet" as "battery critically low."

Also serves ~run_diagnostics (f1tenth_messages/RunDiagnostics) for on-demand checks
rather than only at startup: returns the current battery status plus the last-received
SystemStatus, if enable_sys_obs is true (sys_obs_enabled in the response tells the
caller whether system_status is meaningful or just the zero-valued default -- nothing
publishes /diagnostics/system_status at all when sys_obs is disabled, so this node
can't distinguish "sys_obs is off" from "no message has arrived yet" any other way).
"""

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped

from f1tenth_messages.msg import BatteryStatus, SystemStatus
from f1tenth_messages.srv import RunDiagnostics
from f1tenth_params.param_defaults import get_value


class DiagnosticsServerNode(Node):

    def __init__(self):
        super().__init__('diagnostics_server_node')

        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        self.min_battery_voltage = float(
            self.declare_parameter('min_battery_voltage', 10.8).value)
        self.battery_check_rate_hz = float(
            self.declare_parameter('battery_check_rate_hz', 2.0).value)
        # Plain get_value() read, not a ROS parameter -- see stack_params.yaml's
        # enable_sys_obs comment for why this node reads it the same way
        # behavior_executor_node does, rather than declaring it as its own parameter.
        self.sys_obs_enabled = bool(get_value('enable_sys_obs'))

        self._voltage_samples = []
        self._has_data = False

        self._battery_pub = self.create_publisher(
            BatteryStatus, '/diagnostics/battery_status', 10)
        self._vesc_sub = self.create_subscription(
            VescStateStamped, self.state_topic, self._vesc_callback, 50)

        self._latest_system_status = SystemStatus()
        if self.sys_obs_enabled:
            self._system_status_sub = self.create_subscription(
                SystemStatus, '/diagnostics/system_status', self._system_status_callback, 10)
        else:
            self._system_status_sub = None

        period = 1.0 / max(self.battery_check_rate_hz, 1e-3)
        self.create_timer(period, self._publish_battery_status)

        self._run_diagnostics_srv = self.create_service(
            RunDiagnostics, '~/run_diagnostics', self._on_run_diagnostics)

        self.get_logger().info(
            f'[diagnostics_server] Publishing BatteryStatus on /diagnostics/battery_status '
            f'at {self.battery_check_rate_hz:.2f} Hz (min_battery_voltage='
            f'{self.min_battery_voltage:.1f}V). sys_obs '
            f'{"enabled" if self.sys_obs_enabled else "disabled"} -- run_diagnostics '
            f'service ready.')

    def _vesc_callback(self, msg: VescStateStamped):
        self._voltage_samples.append(msg.state.voltage_input)
        self._has_data = True

    def _system_status_callback(self, msg: SystemStatus):
        self._latest_system_status = msg

    def _current_battery_status(self) -> BatteryStatus:
        status = BatteryStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.min_voltage = self.min_battery_voltage
        status.has_data = self._has_data
        status.voltage = (
            sum(self._voltage_samples) / len(self._voltage_samples)
            if self._voltage_samples else 0.0)
        status.ok = status.has_data and status.voltage >= self.min_battery_voltage
        return status

    def _publish_battery_status(self):
        status = self._current_battery_status()
        self._battery_pub.publish(status)
        # Self-resetting bucket: only averages samples received since the last tick,
        # not a running mean over the node's whole lifetime.
        self._voltage_samples = []

    def _on_run_diagnostics(self, request, response):
        response.battery = self._current_battery_status()
        response.sys_obs_enabled = self.sys_obs_enabled
        response.system_status = self._latest_system_status

        if not response.battery.has_data:
            response.message = f'battery: no data yet (waiting on "{self.state_topic}")'
        elif not response.battery.ok:
            response.message = (
                f'LOW BATTERY: {response.battery.voltage:.1f}V < '
                f'{response.battery.min_voltage:.1f}V minimum')
        else:
            response.message = f'battery OK: {response.battery.voltage:.1f}V'
        if not self.sys_obs_enabled:
            response.message += ' | sys_obs disabled, system_status not available'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
