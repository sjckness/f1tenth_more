#!/usr/bin/env python3
"""
semantic_layer_node.py

Accumulates detected-object positions into a persistent MAP-FRAME semantic
layer, independent of (not merged into) slam_toolbox's own occupancy grid --
see this package's own README-equivalent, the module docstring of
semantic_layer.py, for the "two distinct sources of truth, combined only at
render time" design this deliberately follows (costmap_renderer_node.py is
the one place that actually combines them, visually, into one PNG).

Source: /camera/detections_3d (vision_msgs/Detection3DArray), NOT
/perception/obstacles_2d. Reasoning (the task's own explicit ask -- "your
call which is the more appropriate source, report the reasoning"):
  - Obstacle2DArray (f1tenth_perception's obstacle_projector_node) carries
    NO class label at all -- Obstacle2D.msg is x/y/r only, stripped during
    that node's own z-band/radius-filter/merge-close projection stage.
    Structurally unable to feed a genuinely SEMANTIC (class-aware) layer --
    every object would render identically, indistinguishable from every
    other, which defeats the actual point of a "semantic" layer as opposed
    to a second occupancy-style layer.
  - Detection3DArray (f1tenth_perception's detection_3d_node) carries
    class_id + score per detection (vision_msgs' own ObjectHypothesisWithPose)
    -- the one thing obstacles_2d cannot offer. Trade-offs accepted for this:
    one extra tf2 hop (detections_3d's own frame is zed2_left_camera_frame,
    not base_link, unlike obstacles_2d which is already base_link-relative)
    -- done here via a live tf2 lookup, same pattern wall_detector_node.py's
    own _transform_to_robot_frame/detection_3d_node.py's own optical_to_
    output lookup already use; and no upstream deduplication (obstacles_2d's
    own merge-close pass doesn't apply here) -- handled instead by this
    node's own batch tracker (see semantic_layer.py's own
    update_tracks_batch).

Two-hop transform per detection (see semantic_layer.py's own module
docstring for the pure-function half of this):
  1. camera_frame -> base_link: real tf2 lookup, done here.
  2. base_link -> map: NOT tf2 (see slam.launch.py's own module docstring --
     slam_toolbox's transform_publish_period is deliberately 0.0, so no
     map->odom edge exists in the TF tree to look up at all). Applied
     directly from the temporally-nearest /slam/pose message instead --
     "at the time it was seen" (this node's own bounded pose history,
     matched against each detection batch's own header.stamp via
     semantic_layer.py's find_nearest_pose_by_stamp, not just "whatever
     pose happens to be latest by the time this callback runs").

Real per-frame tracking pass (batch association + confirm/lost lifecycle,
see semantic_layer.py's own module docstring for the full "why" -- this file
only covers what changed here, at the ROS-glue layer):
  - _detections_cb now gathers the WHOLE batch of this message's detections
    (still per-detection tf2/pose math, unchanged) before calling
    update_tracks_batch() ONCE, instead of merging detections one at a time
    in arrival order.
  - An EMPTY Detection3DArray is no longer skipped outright: detection_3d_
    node.py always publishes one message per processed frame (even with
    zero detections in it -- see that node's own _detections_cb), so "this
    message has zero detections" is itself a real, once-per-frame "nothing
    matched any track" signal the lifecycle needs to see (miss-streak
    tracks; ticking them is the whole point of confirm/lost) -- the tf2/
    pose lookups are still skipped in that case (nothing to project), just
    not the miss-tick itself.
  - _publish() now only emits markers for CONFIRMED tracks (an unconfirmed
    one-off false detection never gets a marker at all -- see
    SemanticObject's own confirm_hit_count) and DELETE-diffs tracks that
    drop out between ticks (pruning is new -- previously self._objects only
    ever grew, so there was nothing to delete; RViz/Foxglove otherwise
    caches a marker forever once ADD'd, regardless of it no longer being
    present in a later MarkerArray -- same technique wall_detector_node.py's
    own _publish() already uses for the same reason).
  - Marker id is now track_id directly (SemanticObject's own caller-assigned,
    never-reused id), not the old hash(class_id)+list-index scheme -- that
    scheme relied on list position being stable, which pruning now breaks
    (removing an earlier track shifts every later one's index, which would
    silently reassign marker identities out from under RViz/Foxglove).

Step-0 diagnostic (pose-jump-vs-spawn correlation, see this pass's own
report): every NEWLY SPAWNED track is logged alongside the pose used to
project it and the most recent pose-to-pose jump magnitude/dt at that time
(_recent_pose_jump()) -- cheap, always-on instrumentation (not a new topic/
message type) so a live run can be grepped afterward to check whether spawns
cluster around moments of larger pose jitter rather than being uniformly
distributed, without needing to re-run anything.

Published as visualization_msgs/MarkerArray on /costmap/semantic_markers
(map frame) -- not a new custom message type: MarkerArray already carries
everything a first-pass semantic layer needs (position, a per-class color,
a readable label), is directly Foxglove/RViz-viewable on its own without
needing costmap_renderer_node.py at all, and needs no f1tenth_messages
build-system changes. class_id is carried in EACH marker's own `ns` field
(one namespace per class, `id` is the per-track id, see above) --
costmap_renderer_node.py reads `ns` back as the authoritative class_id (not
the separate TEXT_VIEW_FACING label marker, which exists purely for direct
RViz/Foxglove readability, same convention wall_detector_node.py's own
_wall_markers already uses for its own label markers).
"""

import itertools
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_costmap.semantic_layer import (
    compose_base_link_to_map, find_nearest_pose_by_stamp, pose_to_xytheta,
    update_tracks_batch)

# How many /slam/pose samples to keep for find_nearest_pose_by_stamp -- /slam/
# pose only updates on a real scan-matched pose change (minimum_travel_
# distance/heading in slam_toolbox_params.yaml), so this is a generous window,
# not a tight one; sized in samples (not seconds) since the underlying rate is
# itself irregular (event-driven, not periodic).
_POSE_HISTORY_MAXLEN = 200

# Deterministic per-class colors -- a small, fixed palette cycled by class_id's
# own hash so the same class always renders the same color across a whole run
# (and across restarts, since it's a pure function of the string, not
# insertion order) without needing a hardcoded class_id -> color table that
# would need updating every time the YOLO model's own class list changes.
_PALETTE = (
    (0.90, 0.10, 0.10), (0.10, 0.60, 0.90), (0.15, 0.80, 0.15),
    (0.95, 0.65, 0.05), (0.65, 0.15, 0.85), (0.95, 0.90, 0.10),
)


def _color_for_class(class_id: str) -> tuple:
    """Deterministic (r, g, b) in [0, 1] for `class_id` -- see module-level
    _PALETTE comment. Pure function, shared with costmap_renderer_node.py so
    the PNG render and the MarkerArray always agree on which color means
    which class."""
    idx = hash(class_id) % len(_PALETTE)
    return _PALETTE[idx]


class SemanticLayerNode(Node):
    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node -- lets tests pass
        # parameter_overrides directly (same pattern battery_voltage_check_
        # node.py's own __init__ just picked up this session).
        super().__init__('semantic_layer_node', **kwargs)

        self.declare_parameter('detections_topic', '/camera/detections_3d')
        self.declare_parameter('pose_topic', '/slam/pose')
        self.declare_parameter('output_topic', '/costmap/semantic_markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('score_threshold', 0.5)
        # Renamed in spirit only (still the max association distance, now
        # gating a predicted-position match instead of a raw-position one --
        # see semantic_layer.py's own module docstring) -- kept as the same
        # param name/default so existing launch configs aren't broken by
        # this pass.
        self.declare_parameter('merge_distance_m', 0.5)
        self.declare_parameter('ema_alpha', 0.3)
        # Real per-frame tracking pass -- both new. Initial, reasoned-but-
        # not-yet-live-tuned defaults (flagged explicitly, not buried):
        # confirm_hit_count=3 means a one-off false detection needs 3
        # CONSECUTIVE frames of the same class landing within
        # merge_distance_m of each other before it ever becomes a visible
        # marker; lost_miss_count=5 means a track survives up to 5
        # consecutive frames with no matching detection (camera briefly
        # occluded/missed a frame) before being dropped. Both are in units
        # of "detection batches received" (detection_3d_node.py's own
        # publish rate, i.e. roughly camera FPS), not seconds -- retune once
        # Step 0's live logging below gives real numbers, see this pass's
        # own report.
        self.declare_parameter('confirm_hit_count', 3)
        self.declare_parameter('lost_miss_count', 5)

        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.score_threshold = p('score_threshold').value
        self.merge_distance_m = p('merge_distance_m').value
        self.ema_alpha = p('ema_alpha').value
        self.confirm_hit_count = int(p('confirm_hit_count').value)
        self.lost_miss_count = int(p('lost_miss_count').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /slam/pose has no natural QoS precedent to match against (unlike
        # /odom, where MPC_corr.py's own BEST_EFFORT/VOLATILE choice matches
        # what robot_localization/vesc_to_odom already publish) -- slam_
        # toolbox's own pose publisher uses the rclpy default (RELIABLE,
        # VOLATILE, KEEP_LAST depth 10), confirmed via `ros2 node info` on a
        # live instance during this pass's own feasibility check, so matched
        # here explicitly rather than assumed.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, p('pose_topic').value, self._pose_cb, pose_qos)
        self.det_sub = self.create_subscription(
            Detection3DArray, p('detections_topic').value, self._detections_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, p('output_topic').value, 10)

        self._pose_history: list = []  # [(stamp_sec, (x, y, yaw)), ...]
        self._objects: list = []       # [SemanticObject, ...] (tracks, confirmed + tentative)
        # Caller-owned, never-reused id source for update_tracks_batch()'s
        # own next_track_id param -- see semantic_layer.py's own docstring
        # for why that function takes a callable instead of owning a counter
        # itself (keeps it a pure function of its arguments).
        self._track_id_counter = itertools.count()
        # (ns, id) marker keys published on the PREVIOUS tick -- diffed
        # against the current tick's set in _publish() to emit DELETE
        # markers for tracks that dropped out (pruned) since. Empty at
        # startup, same as _objects.
        self._last_published_marker_keys: set = set()

        self.get_logger().info('semantic_layer_node started')

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xytheta = pose_to_xytheta(msg.pose.pose.position, msg.pose.pose.orientation)
        self._pose_history.append((stamp_sec, xytheta))
        if len(self._pose_history) > _POSE_HISTORY_MAXLEN:
            self._pose_history.pop(0)

    def _recent_pose_jump(self):
        """Step-0 diagnostic only (see module docstring): distance and dt
        between the two most recent /slam/pose samples, as a cheap proxy for
        "how much has the estimated pose been jumping around lately" at the
        moment a new track spawns. (None, None) if fewer than 2 pose samples
        exist yet."""
        if len(self._pose_history) < 2:
            return None, None
        (t0, (x0, y0, _yaw0)), (t1, (x1, y1, _yaw1)) = self._pose_history[-2:]
        return math.hypot(x1 - x0, y1 - y0), (t1 - t0)

    # ------------------------------------------------------------------
    def _detections_cb(self, msg: Detection3DArray):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not msg.detections:
            # Still a real "nothing matched any track this frame" signal for
            # the lifecycle (see module docstring) -- just nothing to
            # project, so the tf2/pose lookups below are skipped.
            before_ids = {t.track_id for t in self._objects}
            self._objects = update_tracks_batch(
                self._objects, [], stamp_sec, self.merge_distance_m, self.ema_alpha,
                self.confirm_hit_count, self.lost_miss_count, self._next_track_id)
            self._log_new_spawns(before_ids, map_pose=None)
            self._publish()
            return

        try:
            cam_to_base = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{msg.header.frame_id}" -> "{self.base_frame}" failed: '
                f'{exc} -- skipping this detection batch', throttle_duration_sec=5.0)
            return

        map_pose = find_nearest_pose_by_stamp(self._pose_history, stamp_sec)
        if map_pose is None:
            self.get_logger().warn(
                'no /slam/pose received yet -- skipping this detection batch '
                '(nothing to anchor the map-frame transform to)',
                throttle_duration_sec=5.0)
            return

        t = cam_to_base.transform.translation
        q = cam_to_base.transform.rotation

        batch = []  # [(class_id, x_map, y_map, score), ...] -- the WHOLE frame,
        # gathered before a single update_tracks_batch() call (see module
        # docstring's "batch association" paragraph for why this must not
        # merge one detection at a time as it's produced).
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            class_id = str(hyp.class_id)
            score = float(hyp.score)
            if score < self.score_threshold:
                continue

            pos = det.bbox.center.position
            # Full 3D rotate+translate for the camera->base_link hop (a real
            # quaternion rotation, not a planar/yaw-only shortcut -- unlike
            # compose_base_link_to_map's own base_link -> map hop, where
            # map_pose_xytheta is already flat, the camera can be mounted
            # with real pitch/roll relative to base_link, so this hop needs
            # the actual 3D rotation to be correct).
            x_cam, y_cam, z_cam = pos.x, pos.y, pos.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            # Standard quaternion-rotate-a-vector formula (v' = q v q^-1),
            # expanded closed-form -- same result tf2_geometry_msgs.do_
            # transform_pose gives, done inline here to keep this a plain
            # numeric hop (no extra Pose message construction needed for
            # just a position, unlike detection_3d_node.py's own do_
            # transform_pose call, which also needs the orientation).
            # Only the X/Y output components are computed -- the rotated Z
            # (rz = 2*dot_uv*uz + (ww-dot(u,u))*z_cam + 2*qw*cross_z) is
            # dropped deliberately, not an oversight: compose_base_link_to_
            # map (the next hop) is a 2D-only composition, so nothing
            # downstream of this node ever needs a map-frame Z coordinate.
            ux, uy, uz = qx, qy, qz
            dot_uv = ux * x_cam + uy * y_cam + uz * z_cam
            cross_x = uy * z_cam - uz * y_cam
            cross_y = uz * x_cam - ux * z_cam
            ww = qw * qw
            rx = (2.0 * dot_uv * ux + (ww - (ux * ux + uy * uy + uz * uz)) * x_cam
                  + 2.0 * qw * cross_x)
            ry = (2.0 * dot_uv * uy + (ww - (ux * ux + uy * uy + uz * uz)) * y_cam
                  + 2.0 * qw * cross_y)
            x_base = rx + t.x
            y_base = ry + t.y

            x_map, y_map = compose_base_link_to_map(x_base, y_base, map_pose)
            batch.append((class_id, x_map, y_map, score))

        before_ids = {t.track_id for t in self._objects}
        self._objects = update_tracks_batch(
            self._objects, batch, stamp_sec, self.merge_distance_m, self.ema_alpha,
            self.confirm_hit_count, self.lost_miss_count, self._next_track_id)
        self._log_new_spawns(before_ids, map_pose=map_pose)

        self._publish()

    def _next_track_id(self) -> int:
        return next(self._track_id_counter)

    def _log_new_spawns(self, before_ids: set, map_pose):
        """Step-0 diagnostic (see module docstring) -- one log line per
        newly-spawned track this tick, with the pose used for this batch's
        projection and the most recent pose-to-pose jump, so a live run can
        be grepped afterward to check whether spawns cluster around pose
        jitter."""
        new_tracks = [t for t in self._objects if t.track_id not in before_ids]
        if not new_tracks:
            return
        jump_m, jump_dt = self._recent_pose_jump()
        jump_str = (f'{jump_m:.3f}m over {jump_dt:.2f}s'
                    if jump_m is not None else 'n/a (< 2 pose samples yet)')
        pose_str = (f'({map_pose[0]:.2f}, {map_pose[1]:.2f}, {map_pose[2]:.2f}rad)'
                    if map_pose is not None else 'n/a')
        for trk in new_tracks:
            self.get_logger().info(
                f'new track spawned: id={trk.track_id} class={trk.class_id} '
                f'pos=({trk.x_map:.2f}, {trk.y_map:.2f}) pose_used={pose_str} '
                f'recent_pose_jump={jump_str}')

    # ------------------------------------------------------------------
    def _publish(self):
        # Only CONFIRMED tracks are ever published (see module docstring --
        # a one-off false detection stays tentative and gets pruned before
        # confirm_hit_count is reached, so it never becomes a marker at
        # all). DELETE-diffed against the previous tick's published keys --
        # pruning is new as of this pass (tracks used to only ever grow),
        # so without this, RViz/Foxglove would keep showing a marker for a
        # track that's since been dropped (they cache ADD'd markers by
        # (ns, id) until explicitly told otherwise -- same reasoning wall_
        # detector_node.py's own _publish() already documents for its own
        # DELETE-diffing).
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        current_keys = set()

        for obj in self._objects:
            if not obj.confirmed:
                continue
            class_id = obj.class_id
            r, g, b = _color_for_class(class_id)
            marker_id = obj.track_id

            disk = Marker()
            disk.header.frame_id = self.map_frame
            disk.header.stamp = now
            disk.ns = class_id
            disk.id = marker_id
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position.x = obj.x_map
            disk.pose.position.y = obj.y_map
            disk.pose.position.z = 0.05
            disk.pose.orientation.w = 1.0
            disk.scale.x = disk.scale.y = 0.3
            disk.scale.z = 0.1
            disk.color = ColorRGBA(r=r, g=g, b=b, a=0.8)
            marker_array.markers.append(disk)
            current_keys.add((disk.ns, disk.id))

            label = Marker()
            label.header = disk.header
            label.ns = f'{class_id}_label'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = obj.x_map
            label.pose.position.y = obj.y_map
            label.pose.position.z = 0.35
            label.scale.z = 0.15
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{class_id} ({obj.hit_count})'
            marker_array.markers.append(label)
            current_keys.add((label.ns, label.id))

        for (ns, marker_id) in (self._last_published_marker_keys - current_keys):
            delete_marker = Marker()
            delete_marker.header.frame_id = self.map_frame
            delete_marker.header.stamp = now
            delete_marker.ns = ns
            delete_marker.id = marker_id
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self._last_published_marker_keys = current_keys
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
