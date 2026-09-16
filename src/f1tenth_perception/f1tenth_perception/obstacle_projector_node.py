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

Per-detection radius: see obstacle_radius() and the obstacle_radius_source
parameter. The obstacle is a disk on the GROUND PLANE, so its radius must come
from the object's horizontal extent. detection_3d_node fills bbox.size.x with
the back-projected image WIDTH, bbox.size.y with the back-projected image
HEIGHT (a vertical extent), and bbox.size.z with its default_depth_extent
parameter, a fixed placeholder rather than a measurement.

The original rule, max(size.x, size.y) / 2.0, therefore sized the disk by the
object's HEIGHT whenever it was taller than wide: a standing person 1.75 m tall
and 0.5 m wide became a 0.875 m disk instead of 0.25 m. "footprint" (default)
uses the width only; "legacy" keeps the height-inflated rule for comparison
against runs recorded before the fix.

Per-class clearance (obstacle_class_margin_m, a JSON class -> metres map,
default {}) is added to the radius HERE and nowhere else, in both radius modes.
Obstacle2D carries no class label, so this node is the last place the class
is known; every consumer of /perception/obstacles_2d then inherits it through
r exactly once (MPC_corr's R_safe = r + car_radius + avoidance_margin, the
solver's d_front = |p - o| - r, front_clearance_node's |y| - r and x - r).
The margin is added AFTER the [min, max] radius reject filter, which judges
the plausibility of the measured size, not of the clearance wanted around it.
"""

import json
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
    apply_nice,
    declare_nice_param,
)

RADIUS_SOURCES = ('footprint', 'legacy')

# detection_3d_node._build_detection3d sets bbox.size.z to its
# default_depth_extent parameter: depth-axis extent is not observable from one
# depth view. Footprint mode may fold size.z in only once that node measures it.
DETECTION_3D_DEPTH_EXTENT_IS_MEASURED = False


def obstacle_radius(size_x, size_y, size_z, source,
                    depth_extent_is_measured=DETECTION_3D_DEPTH_EXTENT_IS_MEASURED):
    """Ground-plane disk radius [m] for one detection's bbox.size.

    footprint: max(size.x, size.z) / 2 when size.z is a real depth estimate,
    otherwise size.x / 2. size.y is never used, since it is the object's height.
    legacy: max(size.x, size.y) / 2, the pre-fix rule, which inflates the disk
    to half the object's HEIGHT for anything taller than it is wide.
    """
    if source == 'footprint':
        if depth_extent_is_measured:
            return max(float(size_x), float(size_z)) / 2.0
        return float(size_x) / 2.0
    if source == 'legacy':
        return max(float(size_x), float(size_y)) / 2.0
    raise ValueError(
        f'obstacle_radius_source must be one of {RADIUS_SOURCES}, got {source!r}')


def parse_class_margins(text):
    """Parse obstacle_class_margin_m (JSON object, class -> metres) to a dict.

    Empty text means no margins. Raises ValueError on anything that is not an
    object of string keys to finite non-negative numbers, so a bad value fails
    at node startup instead of silently dropping the clearance.
    """
    if not str(text).strip():
        return {}
    try:
        raw = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f'obstacle_class_margin_m is not valid JSON: {exc}') from exc
    if not isinstance(raw, dict):
        raise ValueError(
            f'obstacle_class_margin_m must be a JSON object, got {type(raw).__name__}')
    margins = {}
    for class_id, value in raw.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(
                f'obstacle_class_margin_m[{class_id!r}] must be a number, got {value!r}')
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                f'obstacle_class_margin_m[{class_id!r}] must be finite and >= 0, '
                f'got {value!r}')
        margins[str(class_id)] = float(value)
    return margins


class ObstacleProjectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('obstacle_projector_node', **kwargs)

        # nice: see cpu_affinity.py. The perception-latency audit measured
        # this node at ~56-59% CPU with no pinning. CPU AFFINITY is now a
        # `taskset -c` launch prefix, not an in-process self-pin -- see
        # detection.launch.py's own obstacle_projector_cpu_affinity comment
        # (thread-pinning-leak fix, Step 6 reintroduction investigation: the
        # old self-pin left 21 of this node's 22 threads fully unpinned,
        # confirmed live executing on reserved cores).
        declare_nice_param(self)

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
        # max_obstacle_radius: tuned against legacy height radii, re-validate on floor
        self.max_obstacle_radius = float(
            self.declare_parameter('max_obstacle_radius', 1.5).value)
        # See obstacle_radius(). Validated here so a typo fails at startup, not
        # on the first detection.
        self.obstacle_radius_source = str(
            self.declare_parameter('obstacle_radius_source', 'footprint').value)
        if self.obstacle_radius_source not in RADIUS_SOURCES:
            raise ValueError(
                f'obstacle_radius_source must be one of {RADIUS_SOURCES}, '
                f'got {self.obstacle_radius_source!r}')
        # See the module docstring: added to r once, after the reject filter.
        self.obstacle_class_margin_m = parse_class_margins(
            self.declare_parameter('obstacle_class_margin_m', '{}').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obstacles_pub = self.create_publisher(
            Obstacle2DArray, self.obstacles_topic, 10)

        self.det_sub = self.create_subscription(
            Detection3DArray, self.detections_3d_topic, self._detections_callback, 10)

        apply_nice(self)

        self.get_logger().info(
            f'obstacle_projector_node up: "{self.detections_3d_topic}" -> '
            f'"{self.obstacles_topic}" in frame "{self.output_frame}", '
            f'z-band=[{self.obstacle_z_min}, {self.obstacle_z_max}], '
            f'radius_source={self.obstacle_radius_source}, '
            f'class_margin_m={self.obstacle_class_margin_m}')

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

            radius = obstacle_radius(
                det.bbox.size.x, det.bbox.size.y, det.bbox.size.z,
                self.obstacle_radius_source)
            if not (self.min_obstacle_radius <= radius <= self.max_obstacle_radius):
                continue
            class_id = det.results[0].hypothesis.class_id if det.results else ''
            radius += self.obstacle_class_margin_m.get(class_id, 0.0)

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
