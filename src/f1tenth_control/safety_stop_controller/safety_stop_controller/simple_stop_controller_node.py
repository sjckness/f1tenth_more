#!/usr/bin/env python3
"""Simple reactive safety-stop supervisor for the F1TENTH perception stack.

Watches vision_msgs/Detection3DArray (from f1tenth_perception's
detection_3d_node, the YOLO+depth bbox fusion node) and, when a detection
falls inside a fixed forward/lateral/vertical corridor in front of the car,
calls the standard `<mpc_node_name>/set_parameters` service to set the
running MPC controller's `v_ref` speed-reference parameter to 0.0. Once the
corridor has been clear for `clear_frames_required` consecutive
Detection3DArray messages, `v_ref` is restored to `forward_v_ref`.

This deliberately does NOT publish AckermannDriveStamped itself: the MPC
(andre_mpc_node.py) already reads its own `v_ref` parameter every control
loop tick and publishes /drive from it, so supervising that one parameter
lets this node act as a safety layer without a second publisher racing the
MPC on /drive. `mpc_node_name` reuses the launch argument already declared
(previously unused) in stack_bringup_launch.py for exactly this purpose.

This is a simple reactive safety layer, NOT a replacement for the MPC
controller -- it is launchable/testable standalone (against any already-
running MPC node) and is wired into stack_bringup_launch.py gated by the
`enable_safety_stop` launch argument (default 'false': opt-in, since this
overrides the MPC's speed reference and is new/not yet road-tested).

Axis convention (IMPORTANT -- do not assume optical-frame axes here):
detection_3d_node publishes Detection3DArray with header.frame_id ==
`zed2_left_camera_frame` (its `output_frame` parameter), the ZED's
non-optical camera frame, NOT the optical frame. That frame follows REP-103
(x-forward, y-left, z-up), confirmed from detection_3d_node.py's own tf2
transform (optical -> output_frame) and its module docstring. So here:
    bbox.center.position.x -> forward distance (>= 0 is in front of the camera)
    bbox.center.position.y -> lateral offset (positive == left of the optical axis)
    bbox.center.position.z -> vertical offset (positive == above the optical axis)
This is DIFFERENT from the ZED optical-frame convention (z-forward, x-right,
y-down) that upstream instructions may casually assume -- always confirm
against detection_3d_node's `output_frame` parameter/frame_id before reusing
this logic elsewhere.

Stop logic (center-point-only, no bbox-size accounting): a detection "blocks"
the corridor if its center satisfies, simultaneously:
    0 <= x <= stop_distance
    |y| <= corridor_half_width
    |z| <= corridor_half_height
If ANY detection in the array blocks, v_ref is immediately set to 0.0 (on the
stopped<-clear transition only, not resent every frame). Once stopped,
`clear_frames_required` consecutive Detection3DArray messages with no
blocking detection are required before restoring v_ref=forward_v_ref
(debounce, avoids flicker from single noisy frames). Conservative startup
default: v_ref is left untouched (whatever the MPC's own config/default is)
until the corridor has first been confirmed clear for clear_frames_required
frames -- this node never blind-starts the car before it has seen real
perception data.
"""

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from vision_msgs.msg import Detection3DArray


class SimpleStopControllerNode(Node):
    def __init__(self):
        super().__init__('simple_stop_controller_node')

        # ---- parameters ------------------------------------------------
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections_3d').value)
        # Fully-qualified MPC node name whose `v_ref` parameter this node
        # supervises via SetParameters. Matches the launch argument already
        # declared (previously unused) in stack_bringup_launch.py.
        mpc_node_name = str(
            self.declare_parameter('mpc_node_name', '/andre_mpc_controller').value)
        self.mpc_node_name = (
            mpc_node_name if mpc_node_name.startswith('/') else '/' + mpc_node_name)
        self.forward_v_ref = float(
            self.declare_parameter('forward_v_ref', 0.5).value)
        self.stop_distance = float(
            self.declare_parameter('stop_distance', 1.0).value)
        self.corridor_half_width = float(
            self.declare_parameter('corridor_half_width', 0.25).value)
        self.corridor_half_height = float(
            self.declare_parameter('corridor_half_height', 0.25).value)
        self.clear_frames_required = int(
            self.declare_parameter('clear_frames_required', 3).value)

        # ---- state -------------------------------------------------------
        # Conservative default: treat as "stopped" (i.e. don't push
        # forward_v_ref) until the corridor has actually been confirmed
        # clear for clear_frames_required frames.
        self.is_stopped = True
        self.consecutive_clear = 0

        # ---- ROS interfaces ------------------------------------------------
        self.set_params_client = self.create_client(
            SetParameters, f'{self.mpc_node_name}/set_parameters')
        self.det_sub = self.create_subscription(
            Detection3DArray, self.detections_topic, self._detections_callback, 10)

        self.get_logger().info(
            f'simple_stop_controller_node up: subscribing '
            f'"{self.detections_topic}", supervising v_ref on '
            f'"{self.mpc_node_name}" -- forward_v_ref={self.forward_v_ref} m/s, '
            f'corridor=[x: 0..{self.stop_distance}m, '
            f'y: ±{self.corridor_half_width}m, '
            f'z: ±{self.corridor_half_height}m], '
            f'clear_frames_required={self.clear_frames_required}. Leaving '
            "v_ref untouched until the corridor is first confirmed clear.")

    def _find_blocker(self, msg: Detection3DArray):
        """Return the first detection whose bbox center falls inside the
        stop corridor (center-point-only check; ignores bbox extent), or
        None if the frame is clear."""
        for det in msg.detections:
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            if (0.0 <= x <= self.stop_distance
                    and abs(y) <= self.corridor_half_width
                    and abs(z) <= self.corridor_half_height):
                return det
        return None

    def _detections_callback(self, msg: Detection3DArray):
        blocker = self._find_blocker(msg)

        if blocker is not None:
            self.consecutive_clear = 0
            if not self.is_stopped:
                class_id = blocker.results[0].hypothesis.class_id if blocker.results else '?'
                score = blocker.results[0].hypothesis.score if blocker.results else 0.0
                p = blocker.bbox.center.position
                self.get_logger().warn(
                    f'STOP triggered by class_id="{class_id}" (score={score:.2f}) '
                    f'at (x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}) m in frame '
                    f'"{msg.header.frame_id}" -- setting v_ref=0.0 on '
                    f'"{self.mpc_node_name}"')
                self.is_stopped = True
                self._set_mpc_v_ref(0.0)
        else:
            self.consecutive_clear += 1
            if self.is_stopped and self.consecutive_clear >= self.clear_frames_required:
                self.get_logger().info(
                    f'Corridor clear for {self.consecutive_clear} consecutive '
                    f'frames -- restoring v_ref={self.forward_v_ref} on '
                    f'"{self.mpc_node_name}"')
                self.is_stopped = False
                self._set_mpc_v_ref(self.forward_v_ref)

    def _set_mpc_v_ref(self, value: float):
        if not self.set_params_client.service_is_ready():
            self.get_logger().error(
                f'"{self.mpc_node_name}/set_parameters" service not '
                f'available -- cannot set v_ref={value}')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='v_ref',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))]
        future = self.set_params_client.call_async(request)
        future.add_done_callback(
            lambda f, v=value: self._on_set_v_ref_done(f, v))

    def _on_set_v_ref_done(self, future, value):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - report, don't crash the node
            self.get_logger().error(f'set_parameters(v_ref={value}) call failed: {exc}')
            return
        if not response.results or not response.results[0].successful:
            reason = response.results[0].reason if response.results else 'no result'
            self.get_logger().error(f'MPC rejected v_ref={value}: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStopControllerNode()
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
