"""Camera pan bridge: the sim's stand-in for the real Pico servo driver.

Hardware-agnostic contract (f1tenth_camera_pan): the stack's
camera_pan_controller publishes /camera_pan/command (std_msgs/Float64, rad) and
camera_pan_tf_node consumes /camera_pan/joint_state (sensor_msgs/JointState, the
MEASURED angle). This node makes Gazebo satisfy both, the way the real driver
will on the car:

  /camera_pan/command (Float64, rad)
        --> /camera_pan_position_controller/commands (Float64MultiArray)
            the gz_ros2_control position controller (controllers.yaml); the
            camera_pan_joint URDF velocity limit (~6 rad/s) caps the real speed.

  /sim/joint_states (from joint_state_broadcaster)
        --> /camera_pan/joint_state (JointState, just camera_pan_joint)
            the MEASURED angle, stamped with the joint-state sample time, so
            camera_pan_tf_node drives the TF from where the camera ACTUALLY is.

Nothing here touches /tf: camera_pan_tf_node (on the stack) owns the pan TF.
"""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class CameraPanBridge(Node):

    def __init__(self):
        super().__init__('f1tenth_sim_camera_pan_bridge')
        self.joint_name = self.declare_parameter(
            'joint_name', 'camera_pan_joint').value
        self.declare_parameter('command_topic', '/camera_pan/command')
        self.declare_parameter(
            'controller_command_topic', '/camera_pan_position_controller/commands')
        self.declare_parameter('joint_states_topic', '/sim/joint_states')
        self.declare_parameter('joint_state_out_topic', '/camera_pan/joint_state')
        p = self.get_parameter

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, p('controller_command_topic').value, 10)
        self.create_subscription(
            Float64, p('command_topic').value, self.on_command, 10)

        # Only the pan joint, reported as the car's real driver would.
        self.js_pub = self.create_publisher(
            JointState, p('joint_state_out_topic').value, qos_profile_sensor_data)
        self.create_subscription(
            JointState, p('joint_states_topic').value, self.on_joint_states,
            qos_profile_sensor_data)

        self.get_logger().info(
            'camera_pan_bridge up: %s -> %s (position controller); %s[%r] -> %s' % (
                p('command_topic').value, p('controller_command_topic').value,
                p('joint_states_topic').value, self.joint_name,
                p('joint_state_out_topic').value))

    def on_command(self, msg: Float64):
        self.cmd_pub.publish(Float64MultiArray(data=[float(msg.data)]))

    def on_joint_states(self, msg: JointState):
        if self.joint_name not in msg.name:
            return
        i = msg.name.index(self.joint_name)
        out = JointState()
        out.header.stamp = msg.header.stamp          # the measurement's own time
        out.name = [self.joint_name]
        out.position = [msg.position[i]]
        if i < len(msg.velocity):
            out.velocity = [msg.velocity[i]]
        self.js_pub.publish(out)


def main():
    rclpy.init()
    node = CameraPanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Same teardown as drive_bridge.main().
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
