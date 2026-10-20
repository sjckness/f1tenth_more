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

stop_distance IS FROM THE FRONT BUMPER (2026-10-12), not from the camera: the
centre is moved into base_link through the camera frame's pose from
/tf_static and measured ahead of base_front (the URDF frame
f1tenth_description/urdf/base_front.xacro). Everything in front of the lens
still counts (p.x >= 0), so nothing the old check caught is dropped; the
stopping point only moves out, by the lens-to-bumper offset. Without the
camera's transform the lens is assumed at base_link's origin, which can only
trip early.
"""

import math

from f1tenth_behavior.behaviours.static_frames import StaticFramePoses
from f1tenth_description.vehicle_frames import base_front_x
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
        base_front_x_m=None,
        reverse_mode=None,
        front_hard_floor_m=0.20,
    ):
        super().__init__(name=name)
        # Direction-aware, as IsProximityTooClose's front cone (Part D,
        # reverse_integration.py): while the MPC stops for or drives a
        # reverse, only a blocker under front_hard_floor_m stops the car.
        self.reverse_mode = reverse_mode
        self.front_hard_floor_m = float(front_hard_floor_m)
        # The nearest blocker's distance ahead of the bumper, None if none.
        self.nearest_ahead_m = None
        self.detections_topic = detections_topic
        # stop_distance is measured from here (module docstring).
        self.front_x = base_front_x() if base_front_x_m is None else float(base_front_x_m)
        self._frames = StaticFramePoses()
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
        self.tf_static_sub = self._frames.subscribe(self.node)

    def _detections_callback(self, msg):
        cx, _cy, cyaw = self._frames.pose(msg.header.frame_id) or (0.0, 0.0, 0.0)
        nearest = None
        for detection in msg.detections:
            p = detection.bbox.center.position
            # Ahead of the front bumper, along base_link x (module docstring).
            ahead_of_front = cx + p.x * math.cos(cyaw) - p.y * math.sin(cyaw) - self.front_x
            if (p.x >= 0.0 and ahead_of_front <= self.stop_distance
                    and abs(p.y) <= self.corridor_half_width
                    and abs(p.z) <= self.corridor_half_height):
                nearest = ahead_of_front if nearest is None else min(nearest, ahead_of_front)
        self.obstacle_present = nearest is not None
        self.nearest_ahead_m = nearest

    def blocks_reverse(self) -> bool:
        """Return True for a blocker that stops the car in every direction.

        Any, without the direction-aware gate; else only one under
        front_hard_floor_m.
        """
        return self.obstacle_present and (
            self.reverse_mode is None or self.nearest_ahead_m < self.front_hard_floor_m)

    def should_stop(self) -> bool:
        released = self.reverse_mode is not None and self.reverse_mode.reverse_active()
        return self.blocks_reverse() or (self.obstacle_present and not released)

    def update(self):
        return (
            py_trees.common.Status.SUCCESS
            if self.should_stop()
            else py_trees.common.Status.FAILURE
        )
