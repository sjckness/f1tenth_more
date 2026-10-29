"""freespace_door_node: doors from the occupancy grid, no walls involved.

A THIN NODE. Every decision lives in freespace_doors.py (pure numpy/scipy,
no rclpy) and every output in semantic_door_markers.py; this file only
subscribes, builds the parameter object, drives one cycle per scan and
publishes. That split is the same one semantic_doors.py/semantic_layer_node
already uses, and it is why the detector is testable on a hand-drawn grid.

INPUTS -- /scan IS THE PRIMARY ONE
  /scan                            sensor_msgs/LaserScan. THE evidence: a
                                   doorway is a near-far-near step in the
                                   range profile and that signature is there
                                   on the first scan that sees the opening.
                                   The cycle is driven by this topic.
  /slam/map                        nav_msgs/OccupancyGrid, MAP frame,
                                   0.05 m/cell, latched, republished every
                                   5 s. A CROSS-CHECK only, and an optional
                                   one: before the first grid arrives the
                                   detector runs anyway and flags its
                                   candidates `no_grid`.
  /ekf_global/odometry/filtered    the GLOBAL EKF's map-frame pose, the same
                                   input costmap_boundary_node pairs the grid
                                   with (NOT /slam/pose). BEST_EFFORT.
                                   REQUIRED: everything is published in map.
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

THE RATE. Scan-driven, throttled to `freespace_doors_rate_hz` (10 Hz), not
on a timer: the scan is the evidence, so a cycle without a new one would
re-measure the same beams. The throttle exists because the lidar runs at
~40 Hz and nothing downstream needs doors that often.

CONFIRMATION IS ABOUT WHERE THE CAR WAS STANDING, not how many times it
looked -- two hits from poses `confirm_pose_spread_m` apart, or
`confirm_same_pose_hits` hits from anywhere. A reflection or a beam that
slipped past a corner is perfectly repeatable from one standing position;
counting it ten times would prove nothing. That policy lives in
semantic_doors.DoorTracker behind two parameters that the wall detectors
leave at zero, so there is still one tracker for all three door sources.

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
    """A scan in, the four door outputs out, once per scan (throttled)."""

    #: Consecutive failing cycles after which this node stops detecting for
    #: good. Same number and same reasoning as the other two detectors'
    #: guards: a detector that has started raising every cycle is broken, and
    #: a broken one filling the log at 2 Hz is worse than a silent one.
    FREESPACE_MAX_ERRORS = 50
    #: Consecutive candidates-but-nothing-confirmed cycles before saying so.
    FREESPACE_STILL_CYCLES = 10

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
        self.min_period_s = 1.0 / rate_hz
        # EVERY FreespaceParams field comes from stack_params.yaml under its
        # own name with a `freespace_doors_` prefix -- no hand-written list
        # that can drift out of step with the dataclass, and the type of the
        # shipped default is what the YAML value is cast to.
        # test_every_freespace_param_is_in_stack_params guards the mapping.
        proto = fd.FreespaceParams()
        self.params = fd.FreespaceParams(**{
            slot: type(getattr(proto, slot))(stack('freespace_doors_' + slot))
            for slot in fd.FreespaceParams.__slots__})
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
        self._still = 0
        self._ms = math.nan
        self._codes: dict = {}
        self._flags: dict = {}
        self._jumps = 0
        self._grid_age_s = math.nan
        self._last_cycle_sec = None
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

        how = 'medial axis' if self.params.use_skeleton else 'scan jumps'
        self.get_logger().info(
            f'freespace doors on ({how}): driven by {self.scan_topic} at up to '
            f'{rate_hz:.1f} Hz, openings '
            f'{self.params.width_min_detect_m:.2f}-'
            f'{self.params.width_max_detect_m:.2f} m, confirmed on two hits '
            f'{self.params.confirm_pose_spread_m:.2f} m apart or '
            f'{self.params.confirm_same_pose_hits} from one pose, cross-checked '
            f'against {self.map_topic} when one is available')

    # ------------------------------------------------------------------
    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg):
        self._grid = msg
        self._grid_stamp = self._now_sec()

    def _pose_cb(self, msg):
        pos = msg.pose.pose.position
        self._pose = (pos.x, pos.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def _scan_cb(self, msg):
        """THE cycle trigger. Throttled, because the lidar outruns the need."""
        self._scan = msg
        now = self._now_sec()
        if self._last_cycle_sec is not None and now - self._last_cycle_sec < self.min_period_s:
            return
        self._last_cycle_sec = now
        self._cycle()

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

    def _usable_grid(self, now):
        """The latest grid, or None when there is not a usable one.

        None is a normal state, not a failure: the scan is the primary
        evidence and candidates built without a grid are flagged `no_grid`
        rather than withheld. The age check matters because the grid is
        LATCHED -- an old one keeps arriving in every new subscriber's queue
        forever, so without it a dead slam_toolbox would go on silently
        cross-checking doors against a map minutes out of date.
        """
        if self._grid is None:
            return None
        age = now - self._grid_stamp
        self._grid_age_s = age
        if self.grid_max_age_s > 0.0 and age > self.grid_max_age_s:
            self.get_logger().warn(
                f'freespace doors: {self.map_topic} is {age:.1f} s old '
                f'(> {self.grid_max_age_s:.1f}) -- cross-check skipped',
                throttle_duration_sec=10.0)
            return None
        return self._grid

    def _cycle(self):
        """One detection cycle. Never raises; see the module docstring."""
        if self._tracker is None:
            return
        if self._scan is None or self._pose is None:
            self.get_logger().info(
                f'freespace doors: waiting for {self.scan_topic} and {self.pose_topic}',
                throttle_duration_sec=10.0)
            return
        laser_bl = self._laser_in_base(self._scan.header.frame_id)
        if laser_bl is None:
            self.get_logger().warn(
                'freespace doors: no base_link -> '
                f'{self._scan.header.frame_id} transform yet -- the scan cannot '
                'be placed in the map', throttle_duration_sec=10.0)
            return
        now = self._now_sec()
        t0 = time.perf_counter()
        try:
            self._detect(now, laser_bl)
            self._errors = 0
        except Exception as exc:                        # noqa: BLE001
            self._errors += 1
            self.get_logger().warn(
                f'freespace doors: cycle failed ({exc!r})',
                throttle_duration_sec=5.0)
            if self._errors >= self.FREESPACE_MAX_ERRORS:
                self._tracker = None
                self.get_logger().error(
                    f'freespace doors: {self._errors} consecutive failures -- '
                    'door detection is off for the life of this node')
        finally:
            self._ms = (time.perf_counter() - t0) * 1e3
            self._publish_debug()

    def _detect(self, now, laser_bl):
        scan, pose = self._scan, self._pose
        laser_map = compose(pose, laser_bl)
        profile = fd.scan_profile(
            scan.angle_min + np.arange(len(scan.ranges), dtype=float)
            * scan.angle_increment,
            scan.ranges, scan.range_min, scan.range_max, laser_map, self.params)

        grid = self._usable_grid(now)
        window = None
        if grid is not None:
            info = grid.info
            window = fd.occupancy_window(
                grid.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, pose[:2],
                self.params)

        skel = None
        if self.params.use_skeleton:
            if window is None:
                self.get_logger().warn(
                    'freespace doors: use_skeleton is on but there is no grid '
                    '-- the medial-axis detector has nothing to read',
                    throttle_duration_sec=10.0)
                cands = []
            else:
                skel = fd.skeleton(window, self.params)
                beams = fd.ScanBeams(profile.points, profile.laser)
                cands = fd.skeleton_candidates(
                    window, skel, beams, pose[:2], self.params)
        else:
            cands = fd.scan_candidates(profile, window, pose[:2], self.params)

        self._tracker.update(cands, (pose[0], pose[1]), now)
        self._codes, self._flags = {}, {}
        for c in cands:
            key = c.code or 'ok'
            self._codes[key] = self._codes.get(key, 0) + 1
            for flag in c.flags:
                self._flags[flag] = self._flags.get(flag, 0) + 1
        self._jumps = len(fd.jump_points(profile, self.params))
        self._window_cells = 0 if window is None else int(window.free.sum())
        self._skeleton_cells = 0 if skel is None else int(skel.sum())
        self._publish(now, profile, window, skel, cands)
        self._warn_if_standing_still()

    def _publish(self, now, profile, window, skel, cands):
        """The four shared outputs, then this detector's own debug view."""
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

        # EVERY live track, candidates included -- sdm.marker_array draws an
        # unconfirmed one faded, which is exactly what "unconfirmed
        # candidates stay visible in Foxglove" asks for.
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
                fd.jump_points(profile, self.params), cands, self.map_frame,
                stamp,
                skeleton_xy=(fd.skeleton_points(window, skel)
                             if skel is not None else None)))

    def _warn_if_standing_still(self):
        """Say so when the only thing missing is that the car has not moved.

        The shipped confirmation policy is two sightings from robot positions
        confirm_pose_spread_m apart with NO standing-still fallback
        (confirm_same_pose_hits 0), which is deliberate: a one-scan artefact
        is perfectly repeatable from a parked car, so counting it five times
        proves nothing. The cost is that a bench test looks exactly like a
        broken detector -- openings found on every cycle, nothing confirmed,
        an empty door list -- and that cost was paid once already. So this
        says which of the two it is, at most once a minute.

        After FREESPACE_STILL_CYCLES consecutive such cycles, not the first
        one: every run starts parked, so a warning on the first scan would be
        noise rather than a finding.
        """
        p = self.params
        if p.confirm_pose_spread_m <= 0.0 or p.confirm_same_pose_hits:
            return
        tracks = self._tracker.tracks
        spread = max((t.pose_spread() for t in tracks), default=0.0)
        if (not tracks or self._tracker.confirmed()
                or spread >= p.confirm_pose_spread_m):
            self._still = 0
            return
        self._still += 1
        if self._still < self.FREESPACE_STILL_CYCLES:
            return
        self.get_logger().warn(
            f'{len(tracks)} door candidate(s), none confirmed: the furthest '
            f'apart two sightings of one have been is {spread:.2f} m and a '
            f'door needs {p.confirm_pose_spread_m:.2f} m. DRIVE A COUPLE OF '
            'METRES -- freespace_doors_confirm_same_pose_hits is 0, so a '
            'standing car never confirms a door. They are on '
            f'{self.viz_pub.topic_name} already, drawn faded.',
            throttle_duration_sec=60.0)

    def _publish_debug(self):
        """One JSON object per cycle, SAME KEY NAMES as the semantic detector's.

        door_ms / door_errors / door_codes / doors_confirmed are the four keys
        semantic_layer_node already puts on /costmap/unknown_objects/debug, so
        a reader that plots a door detector's health does not change when
        door_source changes -- only the topic it reads does. The three
        freespace-only keys are added, not substituted.
        """
        self.debug_pub.publish(String(data=json.dumps({
            'source': 'freespace',
            'method': 'skeleton' if self.params.use_skeleton else 'scan_jumps',
            'door_ms': None if not math.isfinite(self._ms) else round(self._ms, 2),
            'door_errors': self._errors,
            'door_codes': self._codes,
            'door_flags': self._flags,
            'doors_confirmed': (0 if self._tracker is None
                                else len(self._tracker.confirmed())),
            'door_candidates': (0 if self._tracker is None
                                else len(self._tracker.tracks)),
            'scan_jumps': self._jumps,
            'free_cells': self._window_cells,
            'skeleton_cells': self._skeleton_cells,
            'grid_age_s': (None if not math.isfinite(self._grid_age_s)
                           else round(float(self._grid_age_s), 2)),
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
