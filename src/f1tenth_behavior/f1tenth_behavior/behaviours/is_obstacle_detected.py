"""py_trees port of f1tenth_behavior's old BT.CPP IsObstacleDetectedCondition.

Corridor check ported from safety_stop_controller's simple_stop_controller_node.py
(_find_blocker): a detection blocks if its bbox center falls inside a fixed
forward/lateral/vertical corridor ahead of the camera (frame zed2_left_camera_frame,
REP-103: x=forward, y=left, z=up). Center-point only, first blocker wins. Like the old
BT.CPP condition (and unlike simple_stop_controller_node), this does not debounce
consecutive clear frames -- the root Selector re-ticking every cycle plus the
ackermann_mux lane's own timeout provide the hysteresis instead.

corridor_half_width/corridor_half_height: normally passed in by
behavior_executor_node.create_root(), derived from stack_params.yaml's shared
car_radius + obstacle_safety_margin_m (safety-margin unification pass -- see
that key's own comment for the full 4-mechanism picture and why 0.32 replaces
this behaviour's old standalone 0.25 default, which had drifted from
safety_stop_controller's own, separate 0.4). The 0.32 default here is that
same derivation's result (0.20 + 0.12), kept in sync for standalone
construction/tests, not re-derived independently.
"""

import py_trees
from vision_msgs.msg import Detection3DArray


class IsObstacleDetected(py_trees.behaviour.Behaviour):

    def __init__(
        self,
        name='IsObstacleDetected',
        detections_topic='/camera/detections_3d',
        stop_distance=1.0,
        corridor_half_width=0.32,
        corridor_half_height=0.32,
    ):
        super().__init__(name=name)
        self.detections_topic = detections_topic
        self.stop_distance = stop_distance
        self.corridor_half_width = corridor_half_width
        self.corridor_half_height = corridor_half_height
        self.node = None
        self.detections_sub = None
        self.obstacle_present = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "IsObstacleDetected.setup() didn't find 'node' in kwargs"
            ) from e
        self.detections_sub = self.node.create_subscription(
            Detection3DArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )

    def _detections_callback(self, msg):
        for detection in msg.detections:
            p = detection.bbox.center.position
            if (0.0 <= p.x <= self.stop_distance
                    and abs(p.y) <= self.corridor_half_width
                    and abs(p.z) <= self.corridor_half_height):
                self.obstacle_present = True
                return
        self.obstacle_present = False

    def update(self):
        return (
            py_trees.common.Status.SUCCESS
            if self.obstacle_present
            else py_trees.common.Status.FAILURE
        )
