"""Startup sequence: steer right, steer left, center, then wait for Enter
and activate the MPC at zero velocity.

Publishes AckermannDriveStamped to the high-priority mux input topic
(`/teleop` by default) so the sequence preempts whatever the MPC publishes
on `/drive` (mux gives joystick priority 100 vs navigation 10).

After the user presses Enter, the node "activates" the MPC by setting its
`v_ref` parameter to 0.0 via the ROS 2 parameter service.

TODO: the original spec asked to send the activation command to "the MPC
input topic" with target velocity + angular velocity. frenet_mpc_node has
no such command topic (only /odom in, /drive out). The closest equivalent
is setting the `v_ref` parameter — done here. If a dedicated command
topic gets added to frenet_mpc_node later, switch this to publish on it.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from ackermann_msgs.msg import AckermannDriveStamped


class StackStartupSequence(Node):

    def __init__(self):
        super().__init__('stack_startup_sequence')

        self.declare_parameter('startup_delay', 5.0)
        self.declare_parameter('right_duration', 2.0)
        self.declare_parameter('left_duration', 2.0)
        self.declare_parameter('center_duration', 1.0)
        self.declare_parameter('max_steering_angle', 0.18)
        self.declare_parameter('command_topic', '/teleop')
        self.declare_parameter('mpc_node_name', '/frenet_mpc_controller')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.right_duration = float(self.get_parameter('right_duration').value)
        self.left_duration = float(self.get_parameter('left_duration').value)
        self.center_duration = float(self.get_parameter('center_duration').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.mpc_node_name = str(self.get_parameter('mpc_node_name').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            self.command_topic,
            10,
        )

        self.set_param_cli = self.create_client(
            SetParameters,
            f'{self.mpc_node_name}/set_parameters',
        )

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _publish_cmd(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.pub.publish(msg)

    def _hold(self, steering, speed, duration):
        period = 1.0 / max(self.publish_rate, 1.0)
        end = self.get_clock().now().nanoseconds + int(duration * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            self._publish_cmd(steering, speed)
            threading.Event().wait(period)

    def _set_mpc_vref(self, value):
        if not self.set_param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Parameter service {self.mpc_node_name}/set_parameters not '
                f'available; cannot activate MPC'
            )
            return False

        req = SetParameters.Request()
        req.parameters = [
            Parameter('v_ref', Parameter.Type.DOUBLE, float(value))
            .to_parameter_msg()
        ]
        future = self.set_param_cli.call_async(req)
        # Spin in this worker thread until the future completes.
        # The node is spun by the executor on another thread.
        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)

        if future.result() is None:
            self.get_logger().error('Failed to call set_parameters on MPC')
            return False

        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'unknown'
            self.get_logger().error(f'MPC rejected v_ref set: {reason}')
            return False

        return True

    def _run(self):
        self.get_logger().info(
            f'[stack_bringup] Waiting {self.startup_delay:.1f}s for nodes to come up...'
        )
        threading.Event().wait(self.startup_delay)

        self.get_logger().info('[stack_bringup] Steering RIGHT (max) for 2s...')
        self._hold(-self.max_steer, 0.0, self.right_duration)

        self.get_logger().info('[stack_bringup] Steering LEFT (max) for 2s...')
        self._hold(self.max_steer, 0.0, self.left_duration)

        self.get_logger().info('[stack_bringup] Centering steering for 1s...')
        self._hold(0.0, 0.0, self.center_duration)

        # Release the mux input so the MPC's /drive can win arbitration.
        self._publish_cmd(0.0, 0.0)

        print(
            '\n[stack_bringup] Start sequence complete. '
            'Press ENTER to activate MPC.',
            flush=True,
        )

        try:
            sys.stdin.readline()
        except Exception:
            pass

        # Activate MPC with zero target velocity.
        # See module-level TODO: frenet_mpc has no command topic; we use its
        # v_ref ROS parameter as the closest equivalent.
        ok = self._set_mpc_vref(0.0)
        if ok:
            self.get_logger().info(
                '[stack_bringup] MPC activated with v_ref = 0.0'
            )
        else:
            self.get_logger().error(
                '[stack_bringup] MPC activation failed'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StackStartupSequence()
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
