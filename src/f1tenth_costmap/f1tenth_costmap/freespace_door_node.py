"""freespace_door_node: doors from the occupancy grid, no walls involved.

A THIN NODE. Every decision lives in freespace_doors.py (pure numpy/scipy,
no rclpy) and every output in semantic_door_markers.py; this file only
subscribes, builds the parameter object, drives one cycle on a timer and
publishes. That split is the same one semantic_doors.py/semantic_layer_node
already uses, and it is why the detector is testable on a hand-drawn grid.

INPUTS
  /slam/map                        nav_msgs/OccupancyGrid, MAP frame,
                                   0.05 m/cell, RELIABLE + TRANSIENT_LOCAL
                                   (latched), republished every 5 s --
                                   slam_toolbox's own grid, the stack's only
                                   live one. See output/doors/freespace.md.
  /ekf_global/odometry/filtered    the GLOBAL EKF's map-frame pose, the same
                                   input costmap_boundary_node pairs the grid
                                   with (NOT /slam/pose). BEST_EFFORT.
  /scan                            sensor_msgs/LaserScan, for the live
                                   confirmation only -- never for geometry.
  /tf_static                       base_link -> the scan frame, once.

OUTPUTS -- THE SAME FOUR THE SEMANTIC DETECTOR PUBLISHES, deliberately:
  /perception/doors_semantic       f1tenth_messages/DoorArray, MAP frame
  /viz/doors_semantic              MarkerArray: jambs, threshold, the
                                   crossing-direction ARROW, a label
  /costmap/semantic_doors          SemanticObjectArray, class_id 'door'
  TF map -> door_<id>              one per confirmed door
plus two of its own, which no mission reads:
  /viz/doors_freespace_debug       the skeleton and every candidate, coloured
                                   by why it was rejected
  /costmap/freespace_doors/debug   std_msgs/String, one JSON object per cycle

The four shared names are not a mistake and the word "semantic" in them is
not a claim about this detector. door_source makes the three detectors
EXCLUSIVE -- at most one of them is ever running -- so a consumer
(f1tenth_behavior's EnterDoor reads /perception/doors_semantic and nothing
else) should not have to know, or be reconfigured for, which one found a
door. Renaming the topic per source would mean every consumer growing a
door_source branch for no gain.

WHY A NODE OF ITS OWN rather than a third branch inside semantic_layer_node:
nothing here needs anything that node holds. The semantic detector is hosted
there because it needs that node's wall tracks, its scan, its laser pose, its
pose history and its object list, all already in the map frame. This one
needs an occupancy grid, which that node does not subscribe to at all, and
none of the rest. Hosting it there would mean a /slam/map subscription and a
0.2 Hz grid inside a node that ticks at 10 Hz on scans.

THE RATE. 2 Hz by default, against a grid that only changes every 5 s. The
geometry is therefore re-measured from unchanged cells most ticks; what is
genuinely new each tick is the /scan confirmation, and that is what the
confirm count counts. See output/doors/freespace.md.

FAILURE. A cycle that raises is logged (throttled), counted (door_errors in
the debug JSON) and never allowed to kill the node; after
FREESPACE_MAX_ERRORS consecutive failures the detector turns itself off for
the life of the process, the same guard the other two detectors carry.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from f1tenth_messages.msg import Door, DoorArray, SemanticObject, SemanticObjectArray
from f1tenth_params.param_defaults import get_value

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap import semantic_door_markers as sdm
from f1tenth_costmap import semantic_doors as sd


def yaw_from_quaternion(q):
    """Yaw [rad] of a geometry_msgs/Quaternion. Same formula as costmap_boundary."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(pose, offset):
    """(x, y, yaw) of `offset` (in base_link) expressed in map, given `pose`."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * offset[0] - s * offset[1],
            y + s * offset[0] + c * offset[1],
            yaw + offset[2])


def beams_from_scan(scan, laser_map):
    """Map-frame beam endpoints of one LaserScan, as a fd.ScanBeams.

    A BEAM WITH NO RETURN IS PLACED AT range_max, not dropped. A beam that
    goes through a doorway and hits nothing is the strongest evidence the
    doorway is open; dropping it (which is what unknown_objects.scan_points
    does, correctly, for its own question) would make an open door look shut.
    Only a reading below range_min -- a genuinely invalid sample -- is
    discarded.
    """
    n = len(scan.ranges)
    if n == 0:
        return fd.ScanBeams(np.zeros((0, 2)), laser_map[:2])
    r = np.asarray(scan.ranges, dtype=float)
    bearings = scan.angle_min + np.arange(n, dtype=float) * scan.angle_increment
    no_return = ~np.isfinite(r) | (r >= scan.range_max)
    r = np.where(no_return, scan.range_max, r)
    keep = no_return | (r >= scan.range_min)
    r, bearings = r[keep], bearings[keep]
    theta = laser_map[2] + bearings
    pts = np.column_stack([laser_map[0] + r * np.cos(theta),
                           laser_map[1] + r * np.sin(theta)])
    return fd.ScanBeams(pts, laser_map[:2])


class FreespaceDoorNode(Node):
    """The grid in, the four door outputs out, once per timer tick."""

    #: Consecutive failing cycles after which this node stops detecting for
    #: good. Same number and same reasoning as the other two detectors'
    #: guards: a detector that has started raising every cycle is broken, and
    #: a broken one filling the log at 2 Hz is worse than a silent one.
    FREESPACE_MAX_ERRORS = 50

    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node -- lets the tests pass
        # parameter_overrides directly, the same pattern semantic_layer_node
        # uses.
        super().__init__('freespace_door_node', **kwargs)

        def stack(key):
            """Declare a parameter whose default is stack_params.yaml's."""
            return self.declare_parameter(key, get_value(key)).value

        self.door_source = str(stack('door_source'))
        if self.door_source not in sd.DOOR_SOURCES:
            raise ValueError('door_source must be one of '
                             f'{sd.DOOR_SOURCES}, got {self.door_source!r}')
        self.flag = bool(stack('freespace_doors_enabled'))
        self.enabled = self.flag and self.door_source == 'freespace'

        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.map_topic = str(self.declare_parameter('map_topic', '/slam/map').value)
        self.pose_topic = str(self.declare_parameter(
            'pose_topic', '/ekf_global/odometry/filtered').value)
        self.scan_topic = str(self.declare_parameter('scan_topic', '/scan').value)

        if not self.enabled:
            # Nothing is subscribed, nothing is published, nothing is
            # computed. The node still spins so that the component
            # supervisor sees it alive rather than crash-looping, and so a
            # `ros2 param set` can only ever be a no-op rather than a
            # half-initialised detector.
            self._tracker = None
            self.get_logger().info(
                f'freespace doors OFF (door_source={self.door_source!r}, '
                f'freespace_doors_enabled={self.flag}) -- this node '
                'computes and publishes nothing')
            return

        rate_hz = float(stack('freespace_doors_rate_hz'))
        if not rate_hz > 0.0:
            raise ValueError(f'freespace_doors_rate_hz must be > 0, got {rate_hz}')
        self.params = fd.FreespaceParams(
            window_m=float(stack('freespace_doors_window_m')),
            occupied_threshold=int(stack('freespace_doors_occupied_threshold')),
            cleanup_iterations=int(stack('freespace_doors_cleanup_iterations')),
            width_min_m=float(stack('freespace_doors_width_min_m')),
            width_max_m=float(stack('freespace_doors_width_max_m')),
            rise_ratio=float(stack('freespace_doors_rise_ratio')),
            rise_within_m=float(stack('freespace_doors_rise_within_m')),
            spur_prune_m=float(stack('freespace_doors_spur_prune_m')),
            local_min_radius_m=float(stack('freespace_doors_local_min_radius_m')),
            tangent_span_m=float(stack('freespace_doors_tangent_span_m')),
            symmetry_tol_m=float(stack('freespace_doors_symmetry_tol_m')),
            jamb_radius_m=float(stack('freespace_doors_jamb_radius_m')),
            jamb_min_cells=int(stack('freespace_doors_jamb_min_cells')),
            min_beams=int(stack('freespace_doors_min_beams')),
            beyond_m=float(stack('freespace_doors_beyond_m')),
            jamb_exclude_m=float(stack('freespace_doors_jamb_exclude_m')),
            assoc_max_m=float(stack('freespace_doors_assoc_max_m')),
            confirm_hits=int(stack('freespace_doors_confirm_hits')),
            smooth_alpha=float(stack('freespace_doors_smooth_alpha')),
            candidate_forget_s=float(stack('freespace_doors_candidate_forget_s')),
            forget_unseen_s=float(stack('freespace_doors_forget_unseen_s')),
            max_candidates=int(stack('freespace_doors_max_candidates')))
        self.grid_max_age_s = float(stack('freespace_doors_grid_max_age_s'))
        self.jamb_diameter_m = float(stack('semantic_doors_jamb_viz_diameter_m'))
        self.viz_height_m = float(stack('semantic_doors_viz_height_m'))
        self.publish_tf = bool(stack('semantic_doors_publish_tf'))
        self.debug_markers_enabled = bool(stack('freespace_doors_debug_markers'))

        self._tracker = sd.DoorTracker(self.params.door_params())
        self._grid = None
        self._grid_stamp = None
        self._pose = None
        self._scan = None
        self._laser_bl = None          # (frame_id, (x, y, yaw)) in base_link
        self._errors = 0
        self._ms = math.nan
        self._codes: dict = {}
        self._skeleton_cells = 0
        self._window_cells = 0
        self._viz_ids: set = set()

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        fast_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(Odometry, self.pose_topic, self._pose_cb, fast_qos)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, fast_qos)

        self.doors_pub = self.create_publisher(
            DoorArray,
            self.declare_parameter('doors_topic', '/perception/doors_semantic').value, 10)
        self.viz_pub = self.create_publisher(
            MarkerArray,
            self.declare_parameter('viz_doors_topic', '/viz/doors_semantic').value, 10)
        self.objects_pub = self.create_publisher(
            SemanticObjectArray,
            self.declare_parameter(
                'door_objects_topic', '/costmap/semantic_doors').value, 10)
        self.debug_pub = self.create_publisher(
            String, '/costmap/freespace_doors/debug', 10)
        self.debug_viz_pub = self.create_publisher(
            MarkerArray, '/viz/doors_freespace_debug', 10) \
            if self.debug_markers_enabled else None
        self._tf = TransformBroadcaster(self) if self.publish_tf else None

        self.timer = self.create_timer(1.0 / rate_hz, self._cycle)
        self.get_logger().info(
            f'freespace doors on: {self.params.window_m:.1f} m window on '
            f'{self.map_topic} at {rate_hz:.1f} Hz, '
            f'{self.params.width_min_m:.2f}-{self.params.width_max_m:.2f} m openings, '
            f'confirmed after {self.params.confirm_hits} scan confirmations')

    # ------------------------------------------------------------------
    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg):
        self._grid = msg
        self._grid_stamp = self._now_sec()

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _scan_cb(self, msg):
        self._scan = msg

    def _laser_in_base(self, frame_id):
        """(x, y, yaw) of the scan frame in base_frame, from /tf_static, or None."""
        if self._laser_bl is not None and self._laser_bl[0] == frame_id:
            return self._laser_bl[1]
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        self._laser_bl = (frame_id, (t.translation.x, t.translation.y,
                                     yaw_from_quaternion(t.rotation)))
        return self._laser_bl[1]

    # ------------------------------------------------------------------
    def _cycle(self):
        """One detection cycle. Never raises; see the module docstring."""
        if self._tracker is None:
            return
        if self._grid is None or self._pose is None:
            self.get_logger().info(
                f'freespace doors: waiting for {self.map_topic} and {self.pose_topic}',
                throttle_duration_sec=10.0)
            return
        now = self._now_sec()
        age = now - self._grid_stamp
        if self.grid_max_age_s > 0.0 and age > self.grid_max_age_s:
            # The grid is latched, so an old one keeps arriving in every new
            # subscriber's queue forever. Publishing doors off a grid whose
            # publisher died minutes ago would be inventing geometry; the
            # tracker keeps whatever it already confirmed and this cycle
            # simply does not happen.
            self.get_logger().warn(
                f'freespace doors: {self.map_topic} is {age:.1f} s old '
                f'(> {self.grid_max_age_s:.1f}) -- cycle skipped',
                throttle_duration_sec=10.0)
            return
        t0 = time.perf_counter()
        try:
            self._detect(now)
            self._errors = 0
        except Exception as exc:                        # noqa: BLE001
            self._errors += 1
            self.get_logger().warn(
                f'freespace doors: cycle failed ({exc!r})',
                throttle_duration_sec=5.0)
            if self._errors >= self.FREESPACE_MAX_ERRORS:
                self._tracker = None
                self.timer.cancel()
                self.get_logger().error(
                    f'freespace doors: {self._errors} consecutive failures -- '
                    'door detection is off for the life of this node')
        finally:
            self._ms = (time.perf_counter() - t0) * 1e3
            self._publish_debug(age)

    def _detect(self, now):
        grid, pose = self._grid, self._pose
        info = grid.info
        window = fd.occupancy_window(
            grid.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, pose[:2], self.params)
        skel = fd.skeleton(window, self.params)
        self._window_cells = int(window.free.sum())
        self._skeleton_cells = int(skel.sum())

        beams = None
        if self._scan is not None:
            laser_bl = self._laser_in_base(self._scan.header.frame_id)
            if laser_bl is None:
                self.get_logger().warn(
                    'freespace doors: no base_link -> '
                    f'{self._scan.header.frame_id} transform yet -- no door can '
                    'be confirmed without one', throttle_duration_sec=10.0)
            else:
                beams = beams_from_scan(self._scan, compose(pose, laser_bl))

        cands = fd.candidate_doors(window, skel, beams, pose[:2], self.params)
        self._tracker.update(cands, (pose[0], pose[1]), now)
        self._codes = {}
        for c in cands:
            key = c.code or 'ok'
            self._codes[key] = self._codes.get(key, 0) + 1
        self._publish(now, window, skel, cands)

    # ------------------------------------------------------------------
    def _publish(self, now, window, skel, cands):
        """The four shared outputs, then the debug markers."""
        stamp = self.get_clock().now().to_msg()
        confirmed = self._tracker.confirmed()

        msg = DoorArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.doors = [sdm.door_message(d, Door, stamp) for d in confirmed]
        self.doors_pub.publish(msg)

        objects = SemanticObjectArray()
        objects.header.stamp = stamp
        objects.header.frame_id = self.map_frame
        objects.objects = [sdm.door_object(d, SemanticObject) for d in confirmed]
        self.objects_pub.publish(objects)

        markers, self._viz_ids = sdm.marker_array(
            self._tracker.tracks, self._viz_ids, self.map_frame, stamp,
            self.jamb_diameter_m, self.viz_height_m)
        self.viz_pub.publish(markers)

        if self._tf is not None and confirmed:
            self._tf.sendTransform([
                sdm.door_transform(d, TransformStamped, self.map_frame, stamp)
                for d in confirmed])

        if self.debug_viz_pub is not None:
            self.debug_viz_pub.publish(sdm.freespace_debug_markers(
                fd.skeleton_points(window, skel), cands, self.map_frame, stamp,
                window.res))

    def _publish_debug(self, grid_age_s):
        """One JSON object per cycle, SAME KEY NAMES as the semantic detector's.

        door_ms / door_errors / door_codes / doors_confirmed are the four keys
        semantic_layer_node already puts on /costmap/unknown_objects/debug, so
        a reader that plots a door detector's health does not change when
        door_source changes -- only the topic it reads does. The three
        freespace-only keys are added, not substituted.
        """
        self.debug_pub.publish(String(data=json.dumps({
            'source': 'freespace',
            'door_ms': None if not math.isfinite(self._ms) else round(self._ms, 2),
            'door_errors': self._errors,
            'door_codes': self._codes,
            'doors_confirmed': (0 if self._tracker is None
                                else len(self._tracker.confirmed())),
            'door_candidates': (0 if self._tracker is None
                                else len(self._tracker.tracks)),
            'free_cells': self._window_cells,
            'skeleton_cells': self._skeleton_cells,
            'grid_age_s': round(float(grid_age_s), 2),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = FreespaceDoorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
