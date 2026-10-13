"""
Poses of fixed sensor frames in base_link, from /tf_static alone.

The two behaviours that measure from the front bumper (IsProximityTooClose's
front cone, IsObstacleDetected's stop distance) need where their sensor sits
on the car: the LiDAR and camera mounts are static transforms, published on
/tf_static (description.launch.py, camera.launch.py, the ZED wrapper's
robot_state_publisher). Same approach as front_clearance_node and
lidar_front_wall_node: a tf2 Buffer filled from /tf_static only, no
TransformListener, which would deserialize every dynamic /tf message in Python
to read a handful of fixed edges.

A frame that reaches base_link only through a DYNAMIC joint (the camera pan,
when camera_pan_enabled is true) is not resolvable here; callers fall back to
a stated conservative pose.
"""

import math

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)


class StaticFramePoses:
    """(x, y, yaw) of a frame in base_frame, cached once resolved."""

    def __init__(self, base_frame='base_link'):
        self.base_frame = base_frame
        self._buffer = Buffer()
        self._poses = {}

    def subscribe(self, node):
        """Feed this from node's /tf_static subscription."""
        return node.create_subscription(TFMessage, '/tf_static', self.on_tf_static,
                                        TF_STATIC_QOS)

    def on_tf_static(self, msg: TFMessage):
        for transform in msg.transforms:
            self._buffer.set_transform_static(transform, 'static_frame_poses')

    def pose(self, frame_id):
        """Return (x, y, yaw) of frame_id in base_frame, or None until /tf_static has it."""
        if not frame_id:
            return None
        cached = self._poses.get(frame_id)
        if cached is not None:
            return cached
        try:
            stamped = self._buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            return None
        t = stamped.transform
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (float(t.translation.x), float(t.translation.y), yaw)
        self._poses[frame_id] = pose
        return pose
