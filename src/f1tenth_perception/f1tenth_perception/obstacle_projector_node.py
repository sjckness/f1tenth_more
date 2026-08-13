#!/usr/bin/env python3
"""3D-detections-to-2D-obstacles projection for the F1TENTH perception stack.

Consumes detection_3d_node's vision_msgs/Detection3DArray (default topic
/camera/detections_3d, published in its own output_frame -- default
zed2_left_camera_frame, a camera frame, not base_link) and republishes each
detection as a ground-plane disk (x, y, r) in output_frame (default base_link,
the frame mpc_controller's MPC_corr.py assumes for its own robot-frame obstacle
math), on f1tenth_messages/Obstacle2DArray.

One detections message in, one obstacle list out -- no persistence or
timeout/decay across frames; that livens/decays only downstream, in whatever
consumes /perception/obstacles_2d. Overlap merging (obstacle_merge_distance,
see below) is the one exception: it's within-frame only (collapsing multiple
projected detections that are really the same object into one disk), not
tracking/matching across frames.

Each detection's bbox.center pose is transformed via tf2 (same
lookup_transform(..., rclpy.time.Time()) pattern detection_3d_node.py already
uses for its own optical -> camera_frame step, since this is likewise a fixed
joint chain -- base_link -> zed2_camera_link is a static transform published by
f1tenth_perception/camera.launch.py, and zed2_camera_link -> zed2_left_camera_frame
is a fixed joint broadcast by the ZED wrapper's own robot_state_publisher).
Detections whose transformed z falls outside [obstacle_z_min, obstacle_z_max]
are dropped -- rejects ground-plane and overhead false positives.

Per-detection radius is bbox.size.x/y (the object's real-world width/height,
back-projected from the 2D box + depth by detection_3d_node) -- max(size.x,
size.y) / 2.0, a simple top-down bounding-disk radius. bbox.size.z is NOT used:
it's detection_3d_node's own default_depth_extent placeholder, not a measurement
(depth-axis extent isn't observable from a single monocular depth view).
"""

import math

import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers Pose transform support
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray

from f1tenth_messages.msg import Obstacle2D, Obstacle2DArray

from f1tenth_perception.cpu_affinity import (
    apply_cpu_affinity_and_priority,
    declare_cpu_affinity_params,
)


class ObstacleProjectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_projector_node')

        # cpu_affinity/nice: see cpu_affinity.py. The perception-latency audit
        # measured this node at ~56-59% CPU with no pinning -- shares
        # detection_3d_node's "perception" core pair rather than getting its
        # own dedicated one, set via detection.launch.py's
        # obstacle_projector_cpu_affinity arg.
        declare_cpu_affinity_params(self)

        self.detections_3d_topic = str(
            self.declare_parameter('detections_3d_topic', '/camera/detections_3d').value)
        self.obstacles_topic = str(
            self.declare_parameter('obstacles_topic', '/perception/obstacles_2d').value)
        # base_link: the frame mpc_controller's MPC_corr.py assumes for its own
        # robot-frame obstacle math (robot_to_global takes x_r/y_r as robot-frame
        # forward/left).
        self.output_frame = str(
            self.declare_parameter('output_frame', 'base_link').value)
        self.obstacle_z_min = float(
            self.declare_parameter('obstacle_z_min', -0.1).value)
        self.obstacle_z_max = float(
            self.declare_parameter('obstacle_z_max', 2.0).value)
        # Two or more projected obstacles with centers within this distance of
        # each other are collapsed into one (centroid position, radius = max of
        # the merged set) before publishing -- detection_3d_node has no
        # temporal/spatial dedup of its own, so one real object can otherwise
        # show up as several adjacent disks per frame (e.g. a person split
        # across two overlapping YOLO boxes, or depth noise splitting one box
        # into two slightly different back-projected positions).
        self.obstacle_merge_distance = float(
            self.declare_parameter('obstacle_merge_distance', 0.2).value)
        # Reject implausible radii for what's actually expected on this track
        # (bottles/cones through person-sized) rather than trusting every
        # back-projected size: min rejects near-zero noise-sized boxes, max
        # rejects absurdly large ones (e.g. a bad depth read inflating a
        # box's back-projected real-world size). Revisit these two bounds if
        # the track's real obstacle set turns out to need a wider range.
        self.min_obstacle_radius = float(
            self.declare_parameter('min_obstacle_radius', 0.03).value)
        self.max_obstacle_radius = float(
            self.declare_parameter('max_obstacle_radius', 1.5).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obstacles_pub = self.create_publisher(
            Obstacle2DArray, self.obstacles_topic, 10)

        self.det_sub = self.create_subscription(
            Detection3DArray, self.detections_3d_topic, self._detections_callback, 10)

        apply_cpu_affinity_and_priority(self)

        self.get_logger().info(
            f'obstacle_projector_node up: "{self.detections_3d_topic}" -> '
            f'"{self.obstacles_topic}" in frame "{self.output_frame}", '
            f'z-band=[{self.obstacle_z_min}, {self.obstacle_z_max}]')

    def _detections_callback(self, msg: Detection3DArray):
        if not msg.detections:
            self._publish([], msg.header.stamp)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{msg.header.frame_id}" -> "{self.output_frame}" failed: '
                f'{exc} -- skipping frame', throttle_duration_sec=5.0)
            return

        obstacles = []
        for det in msg.detections:
            pose_out = tf2_geometry_msgs.do_transform_pose(det.bbox.center, transform)

            if not (self.obstacle_z_min <= pose_out.position.z <= self.obstacle_z_max):
                continue

            radius = max(float(det.bbox.size.x), float(det.bbox.size.y)) / 2.0
            if not (self.min_obstacle_radius <= radius <= self.max_obstacle_radius):
                continue

            obstacle = Obstacle2D()
            obstacle.x = float(pose_out.position.x)
            obstacle.y = float(pose_out.position.y)
            obstacle.r = radius
            obstacles.append(obstacle)

        obstacles = self._merge_close_obstacles(obstacles, self.obstacle_merge_distance)

        self._publish(obstacles, msg.header.stamp)

    @staticmethod
    def _merge_close_obstacles(obstacles, merge_distance):
        """Collapse obstacles whose centers are within merge_distance of each
        other into one: position = centroid of the merged set, radius = max of
        the merged radii. Greedy/transitive (single-linkage) -- if A is close
        to B and B is close to C, all three merge into one group even if A and
        C themselves aren't within merge_distance -- deliberately, since a
        detection_3d_node "chain" split across two overlapping YOLO boxes plus
        a slightly-offset depth re-read is exactly this shape, not just
        isolated pairs. O(n^2) in the obstacle count, which per frame is small
        (a handful of detections), so this is not a hot-path concern."""
        n = len(obstacles)
        if n < 2:
            return obstacles

        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj

        for i in range(n):
            for j in range(i + 1, n):
                dx = obstacles[i].x - obstacles[j].x
                dy = obstacles[i].y - obstacles[j].y
                if math.hypot(dx, dy) <= merge_distance:
                    union(i, j)

        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(obstacles[i])

        merged = []
        for group in groups.values():
            if len(group) == 1:
                merged.append(group[0])
                continue
            centroid_x = sum(o.x for o in group) / len(group)
            centroid_y = sum(o.y for o in group) / len(group)
            merged_obstacle = Obstacle2D()
            merged_obstacle.x = float(centroid_x)
            merged_obstacle.y = float(centroid_y)
            merged_obstacle.r = float(max(o.r for o in group))
            merged.append(merged_obstacle)
        return merged

    def _publish(self, obstacles, stamp):
        out = Obstacle2DArray()
        out.header = Header(stamp=stamp, frame_id=self.output_frame)
        out.obstacles = obstacles
        self.obstacles_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProjectorNode()
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
