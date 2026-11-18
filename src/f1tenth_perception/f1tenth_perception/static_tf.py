"""A tf2 Buffer fed from the latched /tf_static only, for fixed-joint lookups.

detection_3d_node (depth optical frame -> its output_frame) and
obstacle_projector_node (detections frame -> base_link) each look up one chain
of FIXED URDF joints (sensors.xacro: zed2_camera_joint,
zed2_left_camera_joint, zed2_left_camera_optical_joint, plus the ZED
wrapper's own fixed joints), all of it on /tf_static on the car and in the
Isaac sim (output/jazzy_port/e2e/tf.json: every camera edge 1 static message,
0 dynamic ones). A TransformListener also deserialises every /tf message (both
EKFs, the wheel joints, slam_toolbox: 20-105 messages/s on the car) on the
node's executor, ~0.35 ms of a core each in rclpy whatever the callback does,
to read edges that never change. So, like front_clearance_node,
lidar_front_wall_node, semantic_layer_node and
f1tenth_behavior/behaviours/static_frames.py, the buffer is filled from
/tf_static alone.

The lookups themselves are unchanged: lookup_transform(target, source,
Time()) on a static-only chain returns the same TransformStamped from a
static-only buffer as from a full listener's buffer, because the listener's
/tf_static callback stores the same messages with the same call
(set_transform_static).

A frame that moves -- map and odom are always /tf here -- cannot be reached
this way, so an output_frame that is one of them is refused at startup
(refuse_dynamic_frame) instead of dropping every frame at runtime. A chain
that runs through a moving joint (a camera pan joint, on branches that have
one) needs tf_static_only:=false, which restores the full TransformListener;
the lookup-failure warning says so.
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# The profile tf2_ros.TransformListener itself uses for /tf_static.
TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)

#: Frames that are only ever published on /tf in this stack (slam_toolbox /
#: the global EKF own map -> odom, the local EKF odom -> base_link). A
#: /tf_static-only buffer can never resolve a lookup into them.
DYNAMIC_FRAMES = frozenset({'map', 'odom'})

STATIC_ONLY_HINT = (
    'this node reads /tf_static only (tf_static_only true): every joint between '
    'the two frames must be fixed; set tf_static_only:=false for a chain through '
    'a moving joint')


def refuse_dynamic_frame(frame, param='output_frame'):
    """Raise ValueError if frame is one of DYNAMIC_FRAMES (startup check)."""
    name = str(frame).lstrip('/')
    if name in DYNAMIC_FRAMES:
        raise ValueError(
            f'{param} {frame!r} is a moving frame (published on /tf, not /tf_static) '
            f'and cannot be resolved with tf_static_only true: {STATIC_ONLY_HINT}')


class StaticTf:
    """The TF state of one node: a Buffer and how it is fed.

    tf_static_only True: a /tf_static subscription on the node feeds
    `buffer` (listener None). False: a TransformListener, as before
    (static_sub None).
    """

    def __init__(self, node, tf_static_only, authority):
        self.buffer = Buffer()
        self.authority = authority
        self.listener = None
        self.static_sub = None
        if tf_static_only:
            self.static_sub = node.create_subscription(
                TFMessage, '/tf_static', self.on_tf_static, TF_STATIC_QOS)
        else:
            self.listener = TransformListener(self.buffer, node)

    def on_tf_static(self, msg: TFMessage):
        for transform in msg.transforms:
            self.buffer.set_transform_static(transform, self.authority)
