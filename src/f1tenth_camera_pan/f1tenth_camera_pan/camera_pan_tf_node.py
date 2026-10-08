"""Publish the camera mount TF from the MEASURED pan angle.

So every perception TF lookup (stamped at the image time) sees where the camera
ACTUALLY was, not where it was commanded.

Frame chain (replaces the single static base_link -> zed2_camera_link):
    base_link --(static, pivot xyz)--> camera_pan_base
    camera_pan_base --(revolute, yaw = measured angle)--> zed2_camera_link
The ZED's own internal subtree (zed2_camera_link -> ... -> *_optical) is still
published by the ZED wrapper RSP (car) / sim_camera_tf RSP (sim), unchanged.

When the measured angle is 0 the composed base_link -> zed2_camera_link is
EXACTLY the old static transform (pivot xyz, identity rotation), so pan=0 is
bit-identical to today -- see test_camera_pan_tf.py.

The dynamic transform is stamped with the joint_state's OWN measurement time and
re-broadcast on every /camera_pan/joint_state message (the servo reports at >=
50 Hz), so tf2 interpolates cleanly at any image stamp. A watchdog republishes
the last angle if the measurement stream goes quiet, so lookups never run off
the end of the buffer.
"""
from f1tenth_camera_pan.frames import (
    base_to_pan_base,
    CAMERA_FRAME,
    joint_state_is_fresh,
    PAN_BASE_FRAME,
    pan_base_to_camera,
)
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _to_msg(content, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = content['parent']
    t.child_frame_id = content['child']
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
        content['translation']
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = content['rotation']
    return t


class CameraPanTf(Node):

    def __init__(self):
        super().__init__('camera_pan_tf_node')
        pivot = (
            float(self.declare_parameter('pivot_x_m', 0.36).value),
            float(self.declare_parameter('pivot_y_m', 0.0).value),
            float(self.declare_parameter('pivot_z_m', 0.25).value),
        )
        self.joint_name = self.declare_parameter('joint_name', 'camera_pan_joint').value
        # A2: never re-stamp an old angle. The dynamic TF is published ONLY when
        # a measurement arrives, with that measurement's OWN stamp, and only if
        # it is fresher than this. A stuck/laggy servo -> no new TF -> the TF
        # goes stale and lookups fail (A1 never falls back to latest) rather than
        # the camera TF silently lying. There is deliberately NO watchdog
        # re-broadcast (that would re-stamp an old angle).
        self.max_stale_ns = int(
            float(self.declare_parameter('max_joint_state_age_s', 0.2).value) * 1e9)

        self._static_bc = StaticTransformBroadcaster(self)
        self._bc = TransformBroadcaster(self)

        # base_link -> camera_pan_base: fixed, latched once.
        self._static_bc.sendTransform(
            _to_msg(base_to_pan_base(pivot), self.get_clock().now().to_msg()))

        self.create_subscription(JointState, '/camera_pan/joint_state',
                                 self._on_joint_state, qos_profile_sensor_data)
        self.get_logger().info(
            f'camera_pan_tf up: base_link -> {PAN_BASE_FRAME} (pivot {pivot}) -> '
            f'{CAMERA_FRAME} from measured "{self.joint_name}" '
            f'(drop if older than {self.max_stale_ns / 1e9:g} s).')

    def _on_joint_state(self, msg: JointState):
        yaw = None
        if self.joint_name in msg.name:
            yaw = msg.position[msg.name.index(self.joint_name)]
        elif len(msg.position) == 1:
            yaw = msg.position[0]
        if yaw is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if not joint_state_is_fresh(now_ns, stamp_ns, self.max_stale_ns):
            self.get_logger().warn(
                f'/camera_pan/joint_state is {(now_ns - stamp_ns) / 1e9:.3f} s old '
                f'(> {self.max_stale_ns / 1e9:g} s) -- not publishing the pan TF',
                throttle_duration_sec=1.0)
            return
        self._bc.sendTransform(_to_msg(pan_base_to_camera(float(yaw)), msg.header.stamp))


def main():
    rclpy.init()
    node = CameraPanTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
