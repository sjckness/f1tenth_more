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
     directly from the temporally-nearest pose message instead --
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

UNKNOWN OBJECTS (obstacles v2, unknown_objects_mode; f1tenth_costmap/
unknown_objects.py has the cell rules). A timer at unknown_rate_hz (10 Hz)
takes the latest /scan, splits its occupied cells into WALL (a confirmed
/perception/walls segment), OBJECT (a confirmed recognised track's footprint)
and ISLAND, turns persistent islands into tracks of class "unknown" in the
SAME track list the detections feed (semantic_layer.update_unknown_batch),
and folds duplicates (dedupe_tracks). On every tick it also publishes, for
mpc_corr, /costmap/semantic_obstacles (f1tenth_messages/Obstacle2DArray,
base_link): every confirmed track, recognised always and unknown only in
'active' mode, r = disk radius + obstacle_class_margin_m -- the ONE obstacle
path the MPC uses since v2 -- and /costmap/unknown_speed_cap (Float32, -1.0 =
none). /costmap/unknown_objects/debug (JSON) carries the per-cycle counts and
cost; /costmap/cell_owners (OccupancyGrid) the ownership grid, only while
something is subscribed.

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

OBJECT MEMORY (Part B; the logic and its reasoning are object_memory.py's,
this file only wires sensors and topics). A timer at memory_rate_hz (10 Hz)
runs one memory cycle: for every CONFIRMED track it decides whether the LiDAR
(/scan, its own sector and range, occlusion along the bearing) and the camera
(live detection batches, frustum, depth window, StereoColumns depth and LiDAR
occlusion) could see it from the car's CURRENT map pose (latest global EKF
pose), then applies the TRACKED / MEMORY / removed lifecycle, and publishes
  /costmap/semantic_objects  f1tenth_messages/SemanticObjectArray, map frame,
                             every cycle: TRACKED and MEMORY objects, with
                             state, confidence, timers, in_fov per sensor and
                             the grown radius of a remembered person;
  /viz/semantic_objects      MarkerArray for Foxglove, only while subscribed:
                             MEMORY objects semi-transparent (alpha follows
                             confidence) and labelled with their remaining time.
The tracker keeps confirmed tracks for the memory to expire
(update_tracks_batch(prune_confirmed=False)), which is also what keeps a
remembered object's id when it is detected again. The two outputs that predate
the memory, /costmap/semantic_markers and /costmap/semantic_tracks, are built
from live_tracks() -- the set the old lost_miss_count lifecycle held -- so
their content is unchanged, except that an object that comes back after
being remembered reappears at once under its old id instead of as a new,
not-yet-confirmed track. memory_enabled:=false restores the old pruning
exactly.

All stored positions stay in the MAP frame. A map -> odom jump changes the
car's map pose and hence what is in view, never where a remembered object is.

BOTH IN ONE NODE (merge, 2026-10-20). One track list (self._tracks), one
/scan subscription read by both cycles, and two 10 Hz timers on the one
executor: the LiDAR cycle (_cycle: unknown objects, dedupe, the MPC's list)
and the memory cycle (_memory_step). semantic_layer.py's docstring says which
rule removes which track. A recognised track is CURRENT (_current_recognised:
TRACKED in the memory, or fresh with the memory off) for both of its
privileges at once -- it owns its LiDAR cells and it is on
/costmap/semantic_obstacles; a remembered (MEMORY) one has neither and
reaches mpc_corr through /costmap/semantic_objects instead, so no object is
ever on both obstacle paths. When the memory lets a track go while the LiDAR
still hits it, _hand_over makes it a confirmed unknown (same id, no gap).
Both paths apply the same plausibility gate (obstacle_z_*,
*_obstacle_radius: unknown_objects.plausible here, passes_obstacle_filter
in mpc_corr).
"""

import array
import itertools
import json
import math
import time

from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32, Int32MultiArray, String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_messages.msg import (
    Door, DoorArray, MultiWallTrack, MultiWallTrackArray, Obstacle2D, Obstacle2DArray,
    SemanticObject, SemanticObjectArray, StereoColumns, WallObservationArray)
from f1tenth_params.object_geometry import footprint_radius
from f1tenth_params.param_defaults import get_value
from f1tenth_costmap import unknown_objects as uo
from f1tenth_costmap.wall_raster import rasterize_walls, transform_segment
from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.object_geometry import ObstacleFilter

from f1tenth_costmap import object_memory as om
from f1tenth_costmap import semantic_door_markers as sdm
from f1tenth_costmap import semantic_doors as sd
from f1tenth_costmap.semantic_layer import (
    compose_base_link_to_map, dedupe_tracks, find_nearest_pose_by_stamp, hand_over_to_lidar,
    live_tracks, pose_to_xytheta, update_tracks_batch, update_unknown_batch)

# How many pose samples to keep for find_nearest_pose_by_stamp. Sized when
# this node read slam_toolbox's own /slam/pose, which only updated on a real
# scan-matched pose change (minimum_travel_distance/heading in slam_toolbox_
# params.yaml) -- generous for that source, and still correct (just a shorter
# wall-clock window) now the source is the global EKF's own continuous output
# instead; sized in samples (not seconds) since the underlying rate is
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


#: An unknown (LiDAR-only) object: grey, a colour no recognised class gets.
UNKNOWN_COLOUR = (0.55, 0.55, 0.55)


def _color_for_class(class_id: str) -> tuple:
    """Deterministic (r, g, b) in [0, 1] for `class_id` -- see module-level
    _PALETTE comment. Pure function, shared with costmap_renderer_node.py so
    the PNG render and the MarkerArray always agree on which color means
    which class."""
    if class_id == uo.UNKNOWN_CLASS:
        return UNKNOWN_COLOUR
    idx = hash(class_id) % len(_PALETTE)
    return _PALETTE[idx]


def _class_margins(value):
    """obstacle_class_margin_m as a dict, from the yaml's dict or a JSON string."""
    if isinstance(value, dict):
        return {str(k): float(v) for k, v in value.items()}
    raw = json.loads(str(value) or '{}')
    return {str(k): float(v) for k, v in raw.items()}


class SemanticLayerNode(Node):
    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node -- lets tests pass
        # parameter_overrides directly (same pattern battery_voltage_check_
        # node.py's own __init__ just picked up this session).
        super().__init__('semantic_layer_node', **kwargs)

        self.declare_parameter('detections_topic', '/camera/detections_3d')
        # POSE SOURCE SWAP (pose-source-arbitration pass) -- the GLOBAL EKF's
        # own fused map-frame output, NOT slam_toolbox's raw /slam/pose.
        # Same map-frame base_link pose either way (this message's
        # header.frame_id is 'map', child_frame_id 'base_link'), so every
        # consumer of _pose_history below is unaffected -- but /slam/pose is
        # silent until the car physically moves past slam_toolbox's own
        # minimum_travel_distance/minimum_travel_heading gates (NORMAL, not a
        # fault -- see costmap_boundary_node.py's own PERIODIC-PUBLISH
        # paragraph), which meant this node skipped EVERY non-empty detection
        # batch for the entire time the car sat still, logging 'no /slam/pose
        # received yet' forever and publishing no semantic markers at all --
        # confirmed live, and exactly the symptom that sent three separate
        # debugging passes looking for a missing-markers bug that was never
        # in this package. The global EKF publishes continuously at its own
        # frequency= from startup (it is seeded by ekf_global.yaml's own
        # initial_state -- see that key's comment for why it had to be, and
        # what it fixed), dead-reckoning on the local EKF between sparse
        # /slam/pose corrections and folding each one in through a normal
        # covariance-weighted Kalman update, so this node now gets a
        # continuous, never-teleporting map-frame pose in every state.
        self.declare_parameter('pose_topic', '/ekf_global/odometry/filtered')
        self.declare_parameter('output_topic', '/costmap/semantic_markers')
        # The same confirmed tracks as the markers, but as DATA rather than
        # as a view. The MarkerArray was the only output for a long time,
        # which meant a fully-associated, map-frame, class-labelled track set
        # existed in this node and was readable by nothing but RViz. Anything
        # that wants to ACT on a tracked object -- drive to it, avoid it,
        # count it -- needs this topic instead of re-deriving tracking from
        # raw per-frame detections, which is how duplicate/multiplying objects
        # got reintroduced once already (see this module's own docstring).
        self.declare_parameter('tracks_topic', '/costmap/semantic_tracks')
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
        # ---- uncertainty-aware tracking (2026-09-01 analysis follow-up) ----
        # See semantic_layer.py's update_tracks_batch/_effective_alpha/
        # mark_missed docstrings for what each of these does and the measured
        # behaviour that motivated it. All three default to ON here (unlike in
        # semantic_layer.py, where they default to the old behaviour so the
        # pure functions stay backward compatible for their own tests).
        #
        # confirm_grace_misses=2: a track keeps its progress toward
        # confirmation across up to 2 consecutive missed frames. Sized against
        # per-class detection flicker within non-empty frames (measured: 24
        # on/off toggles for `person`, 20 for `bottle`, 31 for `tv` across 275
        # frames in run 15-04-45), NOT against the "35-45% frame dropout"
        # figure this was first justified by -- see mark_missed()'s own
        # CORRECTED 2026-09-02 note for why that figure does not apply here.
        self.declare_parameter('confirm_grace_misses', 2)
        # Sigma at which a detection receives the full ema_alpha. 0.05 m is
        # detection_3d_node's own position_sigma_base_m, i.e. "a close-range,
        # high-confidence detection is fully trusted"; anything noisier is
        # down-weighted proportionally.
        self.declare_parameter('alpha_sigma_ref_m', 0.05)
        self.declare_parameter('min_alpha_scale', 0.25)
        # Association gate widening per metre of sigma. 2.0 means a detection
        # with 0.2 m sigma gets a 0.4 m wider gate than merge_distance_m --
        # roughly the measured low-confidence frame-to-frame spread, which is
        # exactly the population that was failing to match its own track.
        self.declare_parameter('gate_sigma_scale', 2.0)

        p = self.get_parameter
        self.pose_topic = p('pose_topic').value
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.score_threshold = p('score_threshold').value
        self.merge_distance_m = p('merge_distance_m').value
        self.ema_alpha = p('ema_alpha').value
        self.confirm_hit_count = int(p('confirm_hit_count').value)
        self.lost_miss_count = int(p('lost_miss_count').value)
        self.confirm_grace_misses = int(p('confirm_grace_misses').value)
        self.alpha_sigma_ref_m = float(p('alpha_sigma_ref_m').value)
        self.min_alpha_scale = float(p('min_alpha_scale').value)
        self.gate_sigma_scale = float(p('gate_sigma_scale').value)

        # The one TF lookup here is camera -> base_link (_detections_cb), and
        # every link of that chain is a fixed URDF joint (sensors.xacro:
        # zed2_camera_joint, zed2_left_camera_joint, ..._optical_joint), i.e.
        # /tf_static. A TransformListener also deserialises every /tf message
        # (both EKFs at 50 Hz) on this node's executor for nothing, so by
        # default (cpu-cleanup, 2026-10-16) the buffer is fed from the latched
        # /tf_static alone -- the same "a handful of fixed edges, no
        # TransformListener" choice yolo_detector_node and front_clearance_node
        # already make. tf_static_only:=false restores the full listener, for
        # a detections frame reached through a moving joint -- REQUIRED if the
        # camera pan joint is ever enabled (stack_params.yaml
        # semantic_layer_tf_static_only, where the default lives).
        self.declare_parameter('tf_static_only', get_value('semantic_layer_tf_static_only'))
        self.tf_buffer = Buffer()
        self.tf_listener = None
        self.tf_static_sub = None
        if bool(self.get_parameter('tf_static_only').value):
            self.tf_static_sub = self.create_subscription(
                TFMessage, '/tf_static', self._tf_static_cb, QoSProfile(
                    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST))
        else:
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # Same BEST_EFFORT/VOLATILE match costmap_boundary_node.py in this
        # same package already uses for this exact topic, which in turn matches
        # MPC_corr.py's own odom subscription -- one consistent QoS choice per
        # publisher across the package. (Was RELIABLE/VOLATILE while this node
        # read slam_toolbox's own /slam/pose, whose publisher uses the rclpy
        # default; that precedent no longer applies now the publisher is the
        # vendored robot_localization ekf_node instead -- see pose_topic above.)
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            Odometry, p('pose_topic').value, self._pose_cb, pose_qos)
        self.det_sub = self.create_subscription(
            Detection3DArray, p('detections_topic').value, self._detections_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, p('output_topic').value, 10)
        self.tracks_pub = self.create_publisher(
            Detection3DArray, p('tracks_topic').value, 10)

        # --- the wall layer (step 7) -------------------------------------
        # Confirmed walls from the tracker, rasterised into the SAME semantic
        # layer this node already owns: a tracked wall is "a thing of class
        # wall is here", which is this layer's kind of truth, not the
        # occupancy grid's "something reflected here once". The grid value is
        # the ROLE (wall_raster.ROLE_VALUE) so it reads on its own; the ids
        # go out beside it, because an int8 cell cannot hold both.
        self.declare_parameter('wall_tracks_topic', '/perception/walls')
        self.declare_parameter('wall_cells_topic', '/costmap/wall_cells')
        self.declare_parameter('wall_ids_topic', '/costmap/wall_cell_ids')
        self.declare_parameter('wall_layer_size_m', 12.0)
        self.declare_parameter('wall_layer_resolution_m', 0.05)
        self.declare_parameter('wall_thickness_m', 0.06)
        self.wall_layer_size_m = float(p('wall_layer_size_m').value)
        self.wall_layer_res_m = float(p('wall_layer_resolution_m').value)
        self.wall_thickness_m = float(p('wall_thickness_m').value)
        self.wall_cells_pub = self.create_publisher(
            OccupancyGrid, p('wall_cells_topic').value, 1)
        self.wall_ids_pub = self.create_publisher(
            Int32MultiArray, p('wall_ids_topic').value, 1)
        # cpu-cleanup (2026-10-16): the wall layer is rebuilt from the LATEST
        # /perception/walls message at most wall_layer_rate_hz, on a timer,
        # instead of on every message (~10 Hz: lidar_front_wall_node's
        # tracker steps 1 scan in 4). Its two outputs have no live consumer:
        # /costmap/wall_cells is only recorded by the mission logger and
        # /costmap/wall_cell_ids is read by nothing. Each rebuild is a fresh
        # raster of the newest message (never accumulated, see _walls_cb),
        # so skipping an intermediate message loses nothing. 5 Hz, below
        # mpc_corr's 10 Hz.
        self.declare_parameter('wall_layer_rate_hz', 5.0)
        self.wall_layer_rate_hz = float(p('wall_layer_rate_hz').value)
        self._pending_walls = None
        self.wall_sub = self.create_subscription(
            MultiWallTrackArray, p('wall_tracks_topic').value, self._walls_msg_cb, 1)
        self.wall_timer = self.create_timer(
            1.0 / self.wall_layer_rate_hz, self._wall_layer_tick)

        # ---- unknown objects (obstacles v2, module docstring) ---------------
        def stack(name, key=None):
            return self.declare_parameter(name, get_value(key or name)).value
        self.unknown_mode = str(stack('unknown_objects_mode'))
        if self.unknown_mode not in uo.MODES:
            raise ValueError(f'unknown_objects_mode must be one of {uo.MODES}, '
                             f'got {self.unknown_mode!r}')
        half_width = float(get_value('swept_clearance_body_half_width_m'))
        self.unknown_params = uo.UnknownParams(
            range_m=float(stack('unknown_range_m')),
            wall_tol_m=float(stack('unknown_wall_tol_m')),
            object_margin_m=float(stack('unknown_object_margin_m')),
            cluster_gap_m=float(stack('unknown_cluster_gap_m')),
            min_cells=int(stack('unknown_min_cells')),
            persist_s=float(stack('unknown_persist_s')),
            decay_s=float(stack('unknown_decay_s')),
            unobserved_decay_s=float(stack('unknown_unobserved_decay_s')),
            front_x_m=float(base_front_x()),
            half_width_m=half_width)
        self.unknown_rate_hz = float(stack('unknown_rate_hz'))
        self.recognised_max_age_s = float(stack('semantic_recognised_max_age_s'))
        self.wall_obs_min_span_m = float(stack('unknown_wall_obs_min_span_m'))
        self.speed_cap_enabled = bool(stack('unknown_speed_cap_enabled'))
        self.speed_cap_mps = float(stack('unknown_speed_cap_mps'))
        self.speed_cap_range_m = float(stack('unknown_speed_cap_range_m'))
        self.strip_half_m = half_width + float(
            get_value('lidar_front_wall_front_strip_margin_m'))
        # A JSON string, as obstacle_projector_node takes it (a ROS parameter
        # cannot be a dict); the yaml default is a dict.
        margins = get_value('obstacle_class_margin_m')
        self.class_margin = _class_margins(self.declare_parameter(
            'obstacle_class_margin_m',
            json.dumps(margins) if isinstance(margins, dict) else str(margins)).value)
        # The ONE plausibility gate (unknown_objects.plausible): the same
        # stack_params keys obstacle_projector_node and mpc_corr read.
        self.obstacle_limits = ObstacleFilter(
            float(stack('obstacle_z_min')), float(stack('obstacle_z_max')),
            float(stack('min_obstacle_radius')), float(stack('max_obstacle_radius')))
        self.declare_parameter('obstacles_topic', '/costmap/semantic_obstacles')
        # /scan is subscribed once, in _init_object_memory: the memory's
        # visibility and this LiDAR cycle read the same latest scan.
        self._scan_unprocessed = False
        self._laser_pose = None       # (frame_id, (x, y, yaw) in base_frame)
        self._latest_walls = None     # the latest MultiWallTrackArray
        self._latest_wall_obs = None  # the latest WallObservationArray
        self._owner_counts = [0, 0, 0, 0]
        self._n_candidates = 0
        self._unknown_stats = {}
        self._n_upgrades = 0
        self._n_dedupe = 0
        self._unknown_ms = math.nan
        self.obstacles_pub = self.create_publisher(
            Obstacle2DArray, p('obstacles_topic').value, 10)
        self.speed_cap_pub = self.create_publisher(Float32, '/costmap/unknown_speed_cap', 10)
        self.unknown_debug_pub = self.create_publisher(
            String, '/costmap/unknown_objects/debug', 10)
        self.cell_owners_pub = self.create_publisher(OccupancyGrid, '/costmap/cell_owners', 1)
        if self.unknown_mode != 'off':
            self.wall_obs_sub = self.create_subscription(
                WallObservationArray, '/perception/wall_observations',
                self._wall_obs_cb, 1)
        self.cycle_timer = self.create_timer(1.0 / self.unknown_rate_hz, self._cycle)

        self._pose_history: list = []  # [(stamp_sec, (x, y, yaw)), ...]
        # [SemanticObject, ...]: tentative + confirmed, including the confirmed
        # ones only the object memory is keeping (see _objects for the view
        # the pre-memory outputs are built from).
        self._tracks: list = []
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

        self._init_object_memory()
        self._init_semantic_doors(stack)

        self.get_logger().info('semantic_layer_node started')

    # ------------------------------------------------------------------
    # Doors from the semantic layer (f1tenth_costmap/semantic_doors.py)
    # ------------------------------------------------------------------
    #: Consecutive failing door cycles after which door detection is turned
    #: off for this node's life. The semantic layer must never be stopped by
    #: an overlay; the same number and the same reasoning as
    #: lidar_front_wall_node's own door guard.
    DOOR_MAX_ERRORS = 50

    def _init_semantic_doors(self, stack):
        """Declare the door parameters and wire the four outputs.

        `stack` is __init__'s own declare-from-stack_params helper, passed in
        rather than rebuilt, so every door default comes from the one
        stack_params.yaml entry and from nowhere else.

        NOTHING IS CREATED WHEN THE DETECTOR IS OFF -- no publisher, no
        broadcaster, no tracker: `semantic_doors_enabled` false, or
        `door_source` naming one of the other two detectors (freespace_door_
        node's grid one, or lidar_front_wall_node's profile one), leaves this
        node exactly as
        it was. Both default to running it (true / 'semantic').
        """
        self.door_source = str(stack('door_source'))
        if self.door_source not in sd.DOOR_SOURCES:
            raise ValueError('door_source must be one of '
                             f'{sd.DOOR_SOURCES}, got {self.door_source!r}')
        self.doors_enabled = (bool(stack('semantic_doors_enabled'))
                              and self.door_source == 'semantic')
        self._door_tracker = None
        self._door_errors = 0
        self._door_ms = math.nan
        self._door_codes: dict = {}
        self._door_viz_ids: set = set()
        if not self.doors_enabled:
            return

        level_name = str(stack('semantic_doors_min_wall_level'))
        if level_name not in sd.LEVEL_BY_NAME:
            raise ValueError('semantic_doors_min_wall_level must be one of '
                             f'{sorted(sd.LEVEL_BY_NAME)}, got {level_name!r}')
        self.door_params = sd.DoorParams(
            min_wall_level=sd.LEVEL_BY_NAME[level_name],
            width_min_m=float(stack('semantic_doors_width_min_m')),
            width_max_m=float(stack('semantic_doors_width_max_m')),
            parallel_angle_deg=float(stack('semantic_doors_parallel_angle_deg')),
            parallel_offset_m=float(stack('semantic_doors_parallel_offset_m')),
            perp_tol_deg=float(stack('semantic_doors_perp_tol_deg')),
            foot_inset_m=float(stack('semantic_doors_foot_inset_m')),
            end_fov_margin_rad=math.radians(
                float(stack('semantic_doors_end_fov_margin_deg'))),
            end_range_margin_m=float(stack('semantic_doors_end_range_margin_m')),
            end_occlusion_margin_m=float(stack('semantic_doors_end_occlusion_margin_m')),
            end_probe_beams=int(stack('semantic_doors_end_probe_beams')),
            min_beams=int(stack('semantic_doors_min_beams')),
            through_frac=float(stack('semantic_doors_through_frac')),
            behind_margin_m=float(stack('semantic_doors_behind_margin_m')),
            gap_clear_m=float(stack('semantic_doors_gap_clear_m')),
            jamb_exclude_m=float(stack('semantic_doors_jamb_exclude_m')),
            assoc_max_m=float(stack('semantic_doors_assoc_max_m')),
            confirm_hits=int(stack('semantic_doors_confirm_hits')),
            smooth_alpha=float(stack('semantic_doors_smooth_alpha')),
            candidate_forget_s=float(stack('semantic_doors_candidate_forget_s')),
            forget_unseen_s=float(stack('semantic_doors_forget_unseen_s')),
            max_walls=int(stack('semantic_doors_max_walls')))
        self.door_jamb_diameter_m = float(stack('semantic_doors_jamb_viz_diameter_m'))
        self.door_viz_height_m = float(stack('semantic_doors_viz_height_m'))
        self.door_publish_tf = bool(stack('semantic_doors_publish_tf'))
        self._door_tracker = sd.DoorTracker(self.door_params)
        self.doors_pub = self.create_publisher(
            DoorArray,
            self.declare_parameter('doors_topic', '/perception/doors_semantic').value, 10)
        self.viz_doors_pub = self.create_publisher(
            MarkerArray,
            self.declare_parameter('viz_doors_topic', '/viz/doors_semantic').value, 10)
        self.door_objects_pub = self.create_publisher(
            SemanticObjectArray,
            self.declare_parameter('door_objects_topic', '/costmap/semantic_doors').value,
            10)
        self._door_tf = TransformBroadcaster(self) if self.door_publish_tf else None
        self.get_logger().info(
            f'semantic doors on: walls at or above {level_name!r}, '
            f'{self.door_params.width_min_m:.2f}-{self.door_params.width_max_m:.2f} m, '
            f'confirmed after {self.door_params.confirm_hits} sightings')

    def _door_walls_map(self, pose):
        """The semantic layer's wall TRACKS as map-frame sd.Wall records.

        The tracks only -- not `_observed_walls_map`'s raw observations. A
        doorway's jamb has to be a wall with an identity and a confirmation
        level behind it, and a raw observation has neither: it is this scan's
        run of returns, with no id to track a door against and nothing to
        compare door_min_wall_level to.
        """
        msg = self._latest_walls
        if msg is None:
            return []
        out = []
        for t in msg.tracks:
            p0, p1 = transform_segment((t.p0_bl.x, t.p0_bl.y),
                                       (t.p1_bl.x, t.p1_bl.y), pose)
            out.append(sd.Wall(t.id, p0, p1,
                               level=sd.wall_level(t.state, t.source),
                               role=t.role or t.label))
        return out

    def _door_step(self, scan):
        """Prepare one scan for the door cycle and run it.

        Cheap enough to stand on its own (one uo.scan_points over the scan,
        then the pair loop): the replay measures the whole cycle at 0.08 ms
        median on a laptop, against the 100 ms the 10 Hz tick allows.
        """
        if not self.doors_enabled or self._door_tracker is None:
            return
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        pose = find_nearest_pose_by_stamp(self._pose_history, stamp)
        laser = self._laser_pose_for(scan.header.frame_id)
        if pose is None or laser is None:
            self.get_logger().warn(
                'semantic doors: no pose or no laser transform yet -- scan skipped',
                throttle_duration_sec=5.0)
            return
        _pts, bearings, ranges = uo.scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, laser)
        self._door_cycle(scan, pose, laser, bearings, ranges, stamp)

    def _door_cycle(self, scan, pose, laser, bearings, ranges, stamp_sec):
        """One door cycle: candidates, the tracker, and the four outputs.

        Never raises into the LiDAR cycle. A door is an OVERLAY: whatever
        goes wrong here, the walls, the obstacle list and the speed cap of
        this same cycle are exactly what they would have been without it.
        """
        if not self.doors_enabled or self._door_tracker is None:
            return
        t0 = time.perf_counter()
        try:
            walls = self._door_walls_map(pose)
            view = sd.ScanView(bearings, ranges, laser[:2], scan.range_max)
            objects = [(float(xy[0]), float(xy[1]), float(t.radius()))
                       for t, xy in ((t, uo.to_base_link([(t.x_map, t.y_map)], pose)[0])
                                     for t in self._tracks if t.confirmed)]
            cands = sd.candidate_doors(
                walls, view, objects, self.door_params,
                to_base_link=lambda xy: uo.to_base_link([xy], pose)[0])
            self._door_tracker.update(cands, (pose[0], pose[1]), stamp_sec)
            self._door_codes = {}
            for c in cands:
                key = c.code or 'ok'
                self._door_codes[key] = self._door_codes.get(key, 0) + 1
            self._publish_doors(scan.header.stamp, stamp_sec)
            self._door_errors = 0
        except Exception as exc:                        # noqa: BLE001
            self._door_errors += 1
            self.get_logger().warn(
                f'semantic doors: cycle failed ({exc!r}); the semantic layer is '
                'unaffected', throttle_duration_sec=5.0)
            if self._door_errors >= self.DOOR_MAX_ERRORS:
                self.doors_enabled = False
                self.get_logger().error(
                    f'semantic doors: {self._door_errors} consecutive failures -- '
                    'door detection is off for the life of this node')
        finally:
            self._door_ms = (time.perf_counter() - t0) * 1e3

    def _publish_doors(self, stamp_msg, stamp_sec):
        """The DoorArray, the markers, the TFs and the semantic-layer objects."""
        tracker = self._door_tracker
        confirmed = tracker.confirmed()

        msg = DoorArray()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.map_frame
        msg.doors = [sdm.door_message(d, Door, stamp_msg) for d in confirmed]
        self.doors_pub.publish(msg)

        objects = SemanticObjectArray()
        objects.header.stamp = stamp_msg
        objects.header.frame_id = self.map_frame
        objects.objects = [sdm.door_object(d, SemanticObject) for d in confirmed]
        self.door_objects_pub.publish(objects)

        markers, self._door_viz_ids = sdm.marker_array(
            tracker.tracks, self._door_viz_ids, self.map_frame, stamp_msg,
            self.door_jamb_diameter_m, self.door_viz_height_m)
        self.viz_doors_pub.publish(markers)

        if self._door_tf is not None and confirmed:
            self._door_tf.sendTransform([
                sdm.door_transform(d, TransformStamped, self.map_frame, stamp_msg)
                for d in confirmed])

    # ------------------------------------------------------------------
    def _init_object_memory(self):
        """
        Declare the memory parameters and wire its inputs and outputs.

        Defaults are single-sourced in stack_params.yaml (semantic_layer_*),
        the same way tf_static_only above is; topic and frame names stay
        node-local like the other topics of this node.
        """
        def declare(name, key):
            return self.declare_parameter(name, get_value(key)).value

        def declare_float(name, key, descriptor=None):
            if descriptor is None:
                return float(self.declare_parameter(name, float(get_value(key))).value)
            return float(self.declare_parameter(
                name, float(get_value(key)), descriptor).value)

        lo, hi = om.MEMORY_DECAY_RANGE_S
        decay_descriptor = ParameterDescriptor(
            description='Seconds a MEMORY object takes to decay to confidence 0.',
            floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])
        self.memory_enabled = bool(declare('memory_enabled', 'semantic_layer_memory_enabled'))
        rate_hz = declare_float('memory_rate_hz', 'semantic_layer_memory_rate_hz')
        if not rate_hz > 0.0:
            raise ValueError(f'memory_rate_hz must be > 0, got {rate_hz}')
        self.memory_params = om.MemoryParams(
            fov_miss_remove_s=declare_float(
                'fov_miss_remove_s', 'semantic_layer_fov_miss_remove_s'),
            memory_decay_s=declare_float(
                'memory_decay_s', 'semantic_layer_memory_decay_s', decay_descriptor),
            person_growth_mps=declare_float(
                'person_memory_growth_mps', 'semantic_layer_person_memory_growth_mps'),
            person_growth_max_m=declare_float(
                'person_memory_growth_max_m', 'semantic_layer_person_memory_growth_max_m'))
        self.visibility_params = om.VisibilityParams(
            stereo_hfov_rad=math.radians(
                declare_float('stereo_hfov_deg', 'semantic_layer_stereo_hfov_deg')),
            stereo_depth_min_m=declare_float(
                'stereo_depth_min_m', 'semantic_layer_stereo_depth_min_m'),
            stereo_depth_max_m=declare_float(
                'stereo_depth_max_m', 'semantic_layer_stereo_depth_max_m'),
            occlusion_margin_m=declare_float(
                'occlusion_margin_m', 'semantic_layer_occlusion_margin_m'),
            stereo_occlusion_use_lidar=bool(declare(
                'stereo_occlusion_use_lidar', 'semantic_layer_stereo_occlusion_use_lidar')))
        self.sensor_stale_s = declare_float('sensor_stale_s', 'semantic_layer_sensor_stale_s')
        self.stereo_columns_max_age_s = declare_float(
            'stereo_columns_max_age_s', 'semantic_layer_stereo_columns_max_age_s')
        # The detections' own frame (detection_3d_node's output_frame): a
        # BODY-convention frame, x forward and y left, which is what the
        # frustum test assumes. Not the optical frame.
        self.stereo_camera_frame = str(
            self.declare_parameter('stereo_camera_frame', 'zed2_left_camera_frame').value)
        self.viz_objects_max_hz = float(self.declare_parameter('viz_objects_max_hz', 5.0).value)

        # Clock read through one attribute so the tests can drive time.
        self._now_sec = self._clock_now_sec
        self._memory: dict = {}               # {track_id: object_memory.ObjectMemory}
        self._last_memory_sec = None
        self._last_pose_rx = None
        self._last_detections_rx = None       # last batch the TRACKER processed
        self._scan = None                     # (receipt sec, LaserScan)
        self._columns = None                  # (receipt sec, StereoColumns)
        self._viz_drawn_ids: set = set()
        self._viz_next_sec = None

        latest_only = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, self.declare_parameter('scan_topic', '/scan').value,
            self._scan_cb, latest_only)
        self.columns_sub = self.create_subscription(
            StereoColumns,
            self.declare_parameter('stereo_columns_topic', '/perception/stereo_columns').value,
            self._stereo_columns_cb, latest_only)
        self.objects_pub = self.create_publisher(
            SemanticObjectArray,
            self.declare_parameter('objects_topic', '/costmap/semantic_objects').value, 10)
        self.viz_objects_pub = self.create_publisher(
            MarkerArray,
            self.declare_parameter('viz_objects_topic', '/viz/semantic_objects').value, 10)
        self.memory_timer = self.create_timer(1.0 / rate_hz, self._memory_tick)

    @property
    def _objects(self) -> list:
        """
        Return the tracks the pre-memory lifecycle would hold (live_tracks).

        /costmap/semantic_markers and /costmap/semantic_tracks are built from
        this, so the object memory does not change what they carry.
        """
        return live_tracks(self._tracks, self.lost_miss_count)

    def _clock_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _tf_static_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'semantic_layer_tf_static')

    def _walls_msg_cb(self, msg):
        """Keep only the newest wall message; _wall_layer_tick rasterises it."""
        self._pending_walls = msg
        self._latest_walls = msg

    def _wall_obs_cb(self, msg):
        self._latest_wall_obs = msg

    def _current_recognised(self, now_sec):
        """The recognised tracks that own their cells and reach the MPC live (one rule)."""
        return uo.current_recognised(self._tracks, now_sec, self.recognised_max_age_s,
                                     self._memory if self.memory_enabled else None,
                                     self.obstacle_limits)

    def _plausible(self, trk):
        return uo.plausible(trk, self.obstacle_limits)

    def _hand_over(self, trk, now_sec):
        """The memory let go of trk: the LiDAR takes it on if it still sees it (no gap)."""
        if self.unknown_mode == 'off' or not uo.plausible(trk, self.obstacle_limits):
            return False
        cls = trk.class_id
        if hand_over_to_lidar(trk, now_sec, self.unknown_params.decay_s):
            self.get_logger().info(
                f'{cls} track {trk.track_id} left the object memory while the LiDAR still '
                'sees it: now a confirmed unknown, same id')
            return True
        return False

    def _fold_memory(self, survivor, folded):
        """dedupe_tracks gave `survivor` the recognised `folded`'s identity: its memory too."""
        mem = self._memory.pop(folded.track_id, None)
        if mem is not None:
            mem.last_hit_count = survivor.hit_count
            self._memory[survivor.track_id] = mem

    def _laser_pose_for(self, frame_id):
        """Return (x, y, yaw) of the scan frame in base_frame, from /tf_static, or None."""
        if self._laser_pose is not None and self._laser_pose[0] == frame_id:
            return self._laser_pose[1]
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._laser_pose = (frame_id, (t.translation.x, t.translation.y, yaw))
        return self._laser_pose[1]

    def _confirmed_walls_map(self):
        """Return the CONFIRMED wall segments of the latest /perception/walls, map frame."""
        msg = self._latest_walls
        if msg is None:
            return []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = find_nearest_pose_by_stamp(self._pose_history, stamp)
        if pose is None:
            return []
        segs = [transform_segment((t.p0_bl.x, t.p0_bl.y), (t.p1_bl.x, t.p1_bl.y), pose)
                for t in msg.tracks if t.state == MultiWallTrack.STATE_CONFIRMED]
        return segs + self._observed_walls_map()

    def _observed_walls_map(self):
        """
        Return this cycle's long LiDAR wall observations as map segments (D5).

        /perception/walls carries at most 4 tracks and never 'other', so the
        confirmed walls alone leave much of a corridor's own returns as
        islands; a LiDAR-backed observation at least unknown_wall_obs_min_span_m
        long -- what the tracker needs to START a wall -- owns its cells too.
        """
        msg = self._latest_wall_obs
        if msg is None or self.wall_obs_min_span_m <= 0.0:
            return []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = find_nearest_pose_by_stamp(self._pose_history, stamp)
        if pose is None:
            return []
        return [transform_segment((o.p0.x, o.p0.y), (o.p1.x, o.p1.y), pose)
                for o in msg.observations
                if o.source != o.SOURCE_STEREO
                and math.hypot(o.p1.x - o.p0.x, o.p1.y - o.p0.y) >= self.wall_obs_min_span_m]

    # ------------------------------------------------------------------
    def _cycle(self):
        """
        Run one LiDAR cycle (unknown objects) and publish the MPC's obstacle list.

        The obstacle list and the speed cap go out on EVERY tick, also in
        mode 'off' and also when no new scan arrived: they are the MPC's only
        obstacle input and it drops a list older than mpc_obstacles_max_age_s.
        """
        if not self._pose_history:
            return
        if self._scan_unprocessed and self._scan is not None:
            if self.unknown_mode != 'off':
                self._lidar_step(self._scan[1])
            # DOORS DO NOT RIDE ON THE UNKNOWN-OBJECT CYCLE. They used to be
            # called from _lidar_step, which runs only while
            # unknown_objects_mode is 'shadow' or 'active' -- and that
            # parameter ships 'off', so the door detector would have been
            # dead on the car with no way to tell from its own flags. The two
            # read the same scan and the same walls but answer different
            # questions, and neither should be able to switch the other off.
            self._door_step(self._scan[1])
        self._scan_unprocessed = False
        pose_now = self._pose_history[-1][1]
        out = Obstacle2DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        now_sec = self._pose_history[-1][0]
        current = {t.track_id for t in self._current_recognised(now_sec)}
        obstacles = uo.obstacles_for_mpc(self._tracks, pose_now, self.unknown_mode,
                                         self.class_margin, now_sec, self.recognised_max_age_s,
                                         current_ids=current)
        out.obstacles = [Obstacle2D(x=x, y=y, r=r) for x, y, r in obstacles]
        self.obstacles_pub.publish(out)
        cap = uo.speed_cap(self._tracks, pose_now, self.unknown_mode, self.speed_cap_enabled,
                           self.speed_cap_mps, self.speed_cap_range_m,
                           self.unknown_params.front_x_m, self.strip_half_m)
        self.speed_cap_pub.publish(Float32(data=float(cap)))
        unknown = [t for t in self._tracks if t.is_unknown]
        self.unknown_debug_pub.publish(String(data=json.dumps({
            'mode': self.unknown_mode,
            'cells': dict(zip(('free', 'wall', 'object', 'island'),
                              (int(c) for c in self._owner_counts))),
            'candidates': self._n_candidates,
            'door_ms': round(self._door_ms, 2) if self._door_ms == self._door_ms else None,
            'door_errors': self._door_errors,
            'door_codes': self._door_codes,
            'doors_confirmed': (0 if self._door_tracker is None
                                else len(self._door_tracker.confirmed())),
            'unknown_tentative': sum(1 for t in unknown if not t.confirmed),
            'unknown_confirmed': sum(1 for t in unknown if t.confirmed),
            'recognised_confirmed': sum(
                1 for t in self._tracks if t.confirmed and not t.is_unknown),
            'stats': self._unknown_stats,
            'upgrades_total': self._n_upgrades,
            'duplicates_folded_total': self._n_dedupe,
            'mpc_obstacles': len(obstacles),
            'speed_cap': cap,
            'lidar_ms': None if not math.isfinite(self._unknown_ms)
            else round(self._unknown_ms, 3)})))

    def _lidar_step(self, scan):
        t0 = time.perf_counter()
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        pose = find_nearest_pose_by_stamp(self._pose_history, stamp)
        laser = self._laser_pose_for(scan.header.frame_id)
        if pose is None or laser is None:
            self.get_logger().warn(
                'unknown objects: no pose or no laser transform yet -- scan skipped',
                throttle_duration_sec=5.0)
            return
        p = self.unknown_params
        pts, bearings, ranges = uo.scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, laser)
        objects = [(t.x_map, t.y_map, t.radius()) for t in self._current_recognised(stamp)]
        view = uo.cell_view(pts, pose, p)
        walls = self._confirmed_walls_map()
        owner = uo.cell_ownership(view, walls, objects, p)
        cands = uo.island_candidates(view, owner, pose, p)
        # First obstacle (D6): a cluster seen through a gap in a wall is not
        # in the car's way.
        c, s = math.cos(pose[2]), math.sin(pose[2])
        cands = uo.drop_behind_walls(cands, walls, (pose[0] + c * laser[0] - s * laser[1],
                                                    pose[1] + s * laser[0] + c * laser[1]))

        island_xy = view.centres(owner == uo.OWNER_ISLAND)[2]

        def observe(trk):
            bl = uo.to_base_link([(trk.x_map, trk.y_map)], pose)[0]
            seen = uo.rays_pass_through(bl, trk.radius(), bearings, ranges, laser[:2], p)
            return seen + (uo.cells_hit((trk.x_map, trk.y_map), trk.radius(), island_xy,
                                        view.resolution),)
        self._tracks, self._unknown_stats = update_unknown_batch(
            self._tracks, cands, stamp, p, self.merge_distance_m, self.ema_alpha,
            self._next_track_id, observe, can_absorb=self._plausible)
        self._tracks, folded = dedupe_tracks(self._tracks, self.merge_distance_m,
                                             self._fold_memory, self._plausible)
        self._n_dedupe += folded
        self._n_candidates = len(cands)
        self._owner_counts = np.bincount(owner.ravel(), minlength=4).tolist()
        self._unknown_ms = (time.perf_counter() - t0) * 1e3
        if self.cell_owners_pub.get_subscription_count() > 0:
            self._publish_owners(view, owner, scan.header.stamp)
        self._publish(scan.header.stamp)

    def _publish_owners(self, view, owner, stamp):
        """Publish the ownership grid for a viewer: free 0, wall 40, object 70, island 100."""
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.map_frame
        grid.info.resolution = view.resolution
        grid.info.height, grid.info.width = owner.shape
        grid.info.origin.position.x, grid.info.origin.position.y = view.origin
        grid.info.origin.orientation.w = 1.0
        value = np.array([0, 40, 70, 100], dtype=np.int8)[owner]
        grid.data = array.array('b', value.ravel().tobytes())
        self.cell_owners_pub.publish(grid)

    def _wall_layer_tick(self):
        msg, self._pending_walls = self._pending_walls, None
        if msg is not None:
            self._walls_cb(msg)

    # ------------------------------------------------------------------
    def _walls_cb(self, msg):
        """Rasterise this cycle's CONFIRMED walls into the map-frame layer.

        Rebuilt from scratch every message, never accumulated: that is what
        makes a track the lifecycle deleted or merged simply stop having
        cells. An accumulating layer would need its own deletion logic, a
        second lifecycle to keep in step with the tracker's, and the one that
        drifts is the one that leaves a wall on the map after the tracker
        decided it was never there.

        The segments are taken in BASE_LINK (p0_bl/p1_bl) and placed with the
        pose nearest the message's own stamp, the same "where the car was when
        it saw this" rule the object tracks already follow -- not whatever
        pose happens to be latest when this callback runs.
        """
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = find_nearest_pose_by_stamp(self._pose_history, stamp_sec)
        if pose is None:
            return
        walls = []
        for track in msg.tracks:
            if track.state != MultiWallTrack.STATE_CONFIRMED:
                continue        # a tentative or coasting wall is not a fact
            p0, p1 = transform_segment(
                (track.p0_bl.x, track.p0_bl.y), (track.p1_bl.x, track.p1_bl.y), pose)
            walls.append((int(track.id), str(track.role or track.label), p0, p1))

        cells = max(1, int(round(self.wall_layer_size_m / self.wall_layer_res_m)))
        origin_x = pose[0] - 0.5 * self.wall_layer_size_m
        origin_y = pose[1] - 0.5 * self.wall_layer_size_m
        roles, ids = rasterize_walls(
            walls, origin_x, origin_y, self.wall_layer_res_m, cells, cells,
            self.wall_thickness_m)

        grid = OccupancyGrid()
        grid.header.stamp = msg.header.stamp
        grid.header.frame_id = self.get_parameter('map_frame').value
        grid.info.resolution = self.wall_layer_res_m
        grid.info.width = grid.info.height = cells
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        # array.array, not .tolist(): the same values, but the generated
        # message setters take an array of the right typecode as-is instead
        # of building and checking a 57,600-element Python list per field
        # (~4 ms of the ~4.3 ms this callback took on linus with no walls).
        grid.data = array.array('b', roles.ravel().astype(np.int8).tobytes())
        self.wall_cells_pub.publish(grid)
        # The ids beside the grid, same row-major order, so a consumer can ask
        # the tracker about a cell. -1 where no wall claims it.
        self.wall_ids_pub.publish(Int32MultiArray(
            data=array.array('i', ids.ravel().astype(np.int32).tobytes())))

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: Odometry):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xytheta = pose_to_xytheta(msg.pose.pose.position, msg.pose.pose.orientation)
        self._pose_history.append((stamp_sec, xytheta))
        if len(self._pose_history) > _POSE_HISTORY_MAXLEN:
            self._pose_history.pop(0)
        self._last_pose_rx = self._now_sec()

    def _scan_cb(self, msg: LaserScan):
        self._scan = (self._now_sec(), msg)
        self._scan_unprocessed = True

    def _stereo_columns_cb(self, msg: StereoColumns):
        self._columns = (self._now_sec(), msg)

    def _recent_pose_jump(self):
        """Step-0 diagnostic only (see module docstring): distance and dt
        between the two most recent pose samples, as a cheap proxy for
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
            before_ids = {t.track_id for t in self._tracks}
            # confirm_grace_misses matters here too: an empty frame is the
            # single most common miss, so omitting it would let the empty-batch
            # path keep zeroing hit_streak and defeat the grace window entirely.
            # The alpha/gate params are irrelevant with no detections to match.
            self._tracks = update_tracks_batch(
                self._tracks, [], stamp_sec, self.merge_distance_m, self.ema_alpha,
                self.confirm_hit_count, self.lost_miss_count, self._next_track_id,
                confirm_grace_misses=self.confirm_grace_misses,
                prune_confirmed=not self.memory_enabled)
            # The camera LOOKED (an empty batch is a processed frame): the
            # object memory's "camera live" test reads this.
            self._last_detections_rx = self._now_sec()
            self._log_new_spawns(before_ids, map_pose=None)
            self._publish(msg.header.stamp)
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
                f'no pose received yet on "{self.pose_topic}" -- skipping this '
                'detection batch '
                '(nothing to anchor the map-frame transform to)',
                throttle_duration_sec=5.0)
            return

        t = cam_to_base.transform.translation
        q = cam_to_base.transform.rotation

        batch = []  # [(class_id, x_map, y_map, score, sigma), ...] -- the WHOLE frame,
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

            # Per-detection position uncertainty, published by
            # detection_3d_node as the x/y variance of the hypothesis pose
            # covariance (see its position_sigma_* params). Taken as the
            # larger of the two axes -- a single scalar is all the tracker
            # consumes, and the conservative axis is the right one to carry.
            #
            # 0.0 means "not populated" (an older detection_3d_node, or any
            # other publisher on this topic) and is mapped to None, which
            # semantic_layer.py treats as "no confidence weighting" rather
            # than as a perfectly precise detection -- so this node degrades
            # to exactly its previous behaviour against an upstream that
            # doesn't provide covariance, instead of silently over-trusting it.
            cov = det.results[0].pose.covariance
            var_xy = max(float(cov[0]), float(cov[7]))
            sigma = math.sqrt(var_xy) if var_xy > 0.0 else None

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
            # compose_base_link_to_map (the next hop) is a 2D-only
            # composition, so nothing downstream needs a map-frame Z. The
            # rotated Z is kept for one thing only: z_base, the centre's
            # height above the floor, which obstacle_projector_node's height
            # band tests and mpc_corr applies to these objects too
            # (passes_obstacle_filter).
            ux, uy, uz = qx, qy, qz
            dot_uv = ux * x_cam + uy * y_cam + uz * z_cam
            cross_x = uy * z_cam - uz * y_cam
            cross_y = uz * x_cam - ux * z_cam
            cross_z = ux * y_cam - uy * x_cam
            ww = qw * qw
            rx = (2.0 * dot_uv * ux + (ww - (ux * ux + uy * uy + uz * uz)) * x_cam
                  + 2.0 * qw * cross_x)
            ry = (2.0 * dot_uv * uy + (ww - (ux * ux + uy * uy + uz * uz)) * y_cam
                  + 2.0 * qw * cross_y)
            rz = (2.0 * dot_uv * uz + (ww - (ux * ux + uy * uy + uz * uz)) * z_cam
                  + 2.0 * qw * cross_z)
            x_base = rx + t.x
            y_base = ry + t.y
            z_base = rz + t.z

            x_map, y_map = compose_base_link_to_map(x_base, y_base, map_pose)
            # Footprint WIDTH by obstacle_projector_node's own rule (shared in
            # f1tenth_params.object_geometry): the back-projected width, never
            # size.y (the height). Fused per track in semantic_layer.py.
            width = 2.0 * footprint_radius(det.bbox.size.x, det.bbox.size.z)
            batch.append((class_id, x_map, y_map, score, sigma, width, z_base))

        before_ids = {t.track_id for t in self._tracks}
        unknown_before = {t.track_id for t in self._tracks if t.is_unknown}
        self._tracks = update_tracks_batch(
            self._tracks, batch, stamp_sec, self.merge_distance_m, self.ema_alpha,
            self.confirm_hit_count, self.lost_miss_count, self._next_track_id,
            confirm_grace_misses=self.confirm_grace_misses,
            alpha_sigma_ref_m=self.alpha_sigma_ref_m,
            min_alpha_scale=self.min_alpha_scale,
            gate_sigma_scale=self.gate_sigma_scale,
            prune_confirmed=not self.memory_enabled)
        self._last_detections_rx = self._now_sec()
        for trk in self._tracks:
            if trk.track_id in unknown_before and not trk.is_unknown:
                self._n_upgrades += 1
                self.get_logger().info(
                    f'unknown track {trk.track_id} upgraded to {trk.class_id}')
        self._tracks, folded = dedupe_tracks(self._tracks, self.merge_distance_m,
                                             self._fold_memory, self._plausible)
        self._n_dedupe += folded
        self._log_new_spawns(before_ids, map_pose=map_pose)

        self._publish(msg.header.stamp)

    def _next_track_id(self) -> int:
        return next(self._track_id_counter)

    def _log_new_spawns(self, before_ids: set, map_pose):
        """Step-0 diagnostic (see module docstring) -- one log line per
        newly-spawned track this tick, with the pose used for this batch's
        projection and the most recent pose-to-pose jump, so a live run can
        be grepped afterward to check whether spawns cluster around pose
        jitter."""
        new_tracks = [t for t in self._tracks if t.track_id not in before_ids]
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
    def _publish(self, capture_stamp):
        """Publish the markers and the tracks for one detection batch.

        `capture_stamp` is that batch's own header.stamp -- the instant the
        frame was captured, not the instant this runs. It goes on the TRACKS
        message (see _publish_tracks); the markers keep the current clock,
        unchanged, because a marker's stamp is what RViz resolves its TF
        against and back-dating it would make every marker look stale to a
        viewer that has since moved.
        """
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
        self._publish_tracks(capture_stamp)

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
            disk.scale.x = disk.scale.y = (2.0 * obj.radius() if obj.is_unknown else 0.3)
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


    def _publish_tracks(self, capture_stamp):
        """Confirmed tracks as Detection3DArray, in the map frame.

        Detection3DArray rather than a new message type: it already carries
        exactly what a track is (class_id, score, a 3-D centre) on a topic
        whose frame_id says these are map-frame, and both this package and
        every plausible consumer already depend on vision_msgs. A bespoke
        message would add a build dependency for no extra information.

        Only CONFIRMED tracks, matching the markers exactly -- a consumer that
        drives at one of these must never see a track that was too tentative
        to draw.

        THE STAMP IS THE DETECTION BATCH'S CAPTURE TIME, not the clock at
        publish. It used to be the latter, which threw away the one quantity
        a consumer cannot reconstruct. A track's position is an estimate of
        where the object was when the frame was CAPTURED; everything between
        capture and here -- the detector's own inference, the two-hop
        transform, the association pass -- is latency this node knows about
        and the consumer does not. Stamping at publish reported every track
        as zero-age no matter how far behind the pipeline had fallen, so a
        downstream staleness check could only ever measure its own receive
        gap, never the real age of the estimate.

        That matters most for exactly the consumer this topic was added for:
        something that drives at a track. A stale track stamped `now` is
        indistinguishable from a fresh one, and the car keeps steering at
        where a person was a second ago while the age check reads healthy.

        This node already matches detections against the pose history by this
        same stamp (find_nearest_pose_by_stamp in _detections_cb), so the
        value is the one the estimate was actually built against, not a
        second opinion about when the frame happened.
        """
        message = Detection3DArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = self.map_frame

        for obj in self._objects:
            if not obj.confirmed:
                continue
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = obj.class_id
            hypothesis.hypothesis.score = float(obj.score)

            detection = Detection3D()
            detection.header = message.header
            detection.id = str(obj.track_id)
            detection.results = [hypothesis]
            detection.bbox.center.position.x = float(obj.x_map)
            detection.bbox.center.position.y = float(obj.y_map)
            detection.bbox.center.orientation.w = 1.0
            # A top-down disk: x and y both carry the fused footprint width
            # (0.0 when no detection has provided one yet, which consumers
            # read as "unknown"; GoToObject then uses a per-class nominal).
            # z is unused. This was a fixed 0.3 cube for every track.
            width = obj.width if obj.width is not None else 0.0
            detection.bbox.size.x = float(width)
            detection.bbox.size.y = float(width)
            detection.bbox.size.z = 0.0
            message.detections.append(detection)

        self.tracks_pub.publish(message)

    # ------------------------------------------------------------------
    # object memory (Part B) -- see the module docstring and object_memory.py
    def _memory_tick(self):
        self._memory_step(self._now_sec())

    def _memory_step(self, now_sec: float):
        """One memory cycle at now_sec: visibility, lifecycle, publish."""
        dt = 0.0 if self._last_memory_sec is None else now_sec - self._last_memory_sec
        self._last_memory_sec = now_sec
        views = self._sensor_views(now_sec)
        self._tracks = om.memory_cycle(
            self._tracks, self._memory, dt,
            lambda trk: self._visibility(trk, views),
            self.memory_params, enabled=self.memory_enabled,
            expire=lambda trk: self._hand_over(trk, now_sec))
        self._publish_semantic_objects(now_sec)
        self._publish_viz_objects(now_sec)

    def _fresh(self, rx_sec, now_sec: float, max_age_s: float) -> bool:
        return rx_sec is not None and now_sec - rx_sec <= max_age_s

    def _lookup(self, target: str, source: str):
        """Return lookup_transform(target, source) as ((t), (q)), or None."""
        try:
            ts = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'object memory: tf2 lookup "{source}" -> "{target}" failed: {exc} '
                '-- that sensor counts as not looking', throttle_duration_sec=5.0)
            return None
        t, q = ts.transform.translation, ts.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _sensor_views(self, now_sec: float) -> dict:
        """Collect what each sensor can contribute this cycle; None where it cannot."""
        views = {'pose': None, 'scan': None, 'laser_tf': None, 'camera_live': False,
                 'cam_tf': None, 'columns': None, 'opt_tf': None}
        if not self._pose_history or not self._fresh(
                self._last_pose_rx, now_sec, self.sensor_stale_s):
            return views          # no current pose: nothing can be placed in any view
        views['pose'] = self._pose_history[-1][1]
        if self._scan is not None and self._fresh(self._scan[0], now_sec, self.sensor_stale_s):
            scan = self._scan[1]
            views['laser_tf'] = self._lookup(scan.header.frame_id, self.base_frame)
            views['scan'] = scan if views['laser_tf'] is not None else None
        if self._fresh(self._last_detections_rx, now_sec, self.sensor_stale_s):
            views['cam_tf'] = self._lookup(self.stereo_camera_frame, self.base_frame)
            views['camera_live'] = views['cam_tf'] is not None
        if views['camera_live'] and self._columns is not None and self._fresh(
                self._columns[0], now_sec, self.stereo_columns_max_age_s):
            columns = self._columns[1]
            views['opt_tf'] = self._lookup(columns.camera_frame, self.base_frame)
            views['columns'] = columns if views['opt_tf'] is not None else None
        return views

    def _visibility(self, trk, views: dict) -> om.Visibility:
        """Place one track in every sensor frame and ask object_memory."""
        if views['pose'] is None:
            return om.Visibility()
        x_b, y_b = om.map_to_base(trk.x_map, trk.y_map, views['pose'])

        def in_frame(tf):
            # At the sensor's own height, i.e. in its horizontal plane.
            if tf is None:
                return None
            return om.apply_transform(tf, (x_b, y_b, om.source_origin_height(tf)))

        p_laser = in_frame(views['laser_tf'])
        p_cam = in_frame(views['cam_tf'])
        radius = om.track_base_radius(trk)
        return om.object_visibility(
            radius, p_laser, views['scan'], p_cam, views['camera_live'],
            in_frame(views['opt_tf']), views['columns'], self.visibility_params,
            view_radius=radius + trk.extra_gate_m)

    def _memory_entries(self):
        """Yield (track, ObjectMemory) for every confirmed track the memory holds."""
        for trk in self._tracks:
            mem = self._memory.get(trk.track_id)
            if trk.confirmed and mem is not None:
                yield trk, mem

    def _publish_semantic_objects(self, now_sec: float):
        """Publish /costmap/semantic_objects: every remembered object, map frame."""
        msg = SemanticObjectArray()
        msg.header.stamp = rclpy.time.Time(nanoseconds=int(now_sec * 1e9)).to_msg()
        msg.header.frame_id = self.map_frame
        params = self.memory_params
        msg.fov_miss_remove_s = float(params.fov_miss_remove_s)
        msg.memory_decay_s = float(params.memory_decay_s)
        for trk, mem in self._memory_entries():
            obj = SemanticObject()
            obj.id = int(trk.track_id)
            obj.class_id = str(trk.class_id)
            obj.x = float(trk.x_map)
            obj.y = float(trk.y_map)
            obj.radius_m = float(om.track_base_radius(trk))
            z_base = getattr(trk, 'z_base', None)
            obj.z_m = float(z_base) if z_base is not None else math.nan
            obj.effective_radius_m = float(om.effective_radius(trk, mem, params))
            obj.state = int(mem.state)
            obj.confidence = float(mem.confidence)
            obj.score = float(trk.score)
            obj.time_in_memory_s = float(mem.time_in_memory_s)
            obj.memory_remaining_s = float(mem.remaining_s(params))
            obj.time_since_seen_s = float(mem.time_since_seen_s)
            obj.fov_unseen_s = float(mem.fov_unseen_s)
            obj.in_fov = bool(mem.in_fov)
            obj.in_fov_lidar = bool(mem.in_fov_lidar)
            obj.in_fov_stereo = bool(mem.in_fov_stereo)
            msg.objects.append(obj)
        self.objects_pub.publish(msg)

    def _publish_viz_objects(self, now_sec: float):
        """
        Publish /viz/semantic_objects for Foxglove, built only while watched.

        Same bargain as lidar_front_wall_node's /viz/walls: the subscription
        count, then a rate cap, and markers that carry a lifetime, so nothing
        lingers if this node stops.
        """
        if self.viz_objects_pub.get_subscription_count() == 0:
            self._viz_drawn_ids = set()    # the lifetime already cleared them
            self._viz_next_sec = None
            return
        if self._viz_next_sec is not None and now_sec < self._viz_next_sec:
            return
        period = 1.0 / self.viz_objects_max_hz if self.viz_objects_max_hz > 0.0 else 0.0
        self._viz_next_sec = now_sec + period
        markers, self._viz_drawn_ids = build_object_markers(
            list(self._memory_entries()), self.memory_params, self.map_frame,
            self.get_clock().now().to_msg(), max(0.5, 3.0 * period), self._viz_drawn_ids)
        self.viz_objects_pub.publish(markers)


#: /viz/semantic_objects namespaces: one for the disks, one for the labels.
VIZ_NS_DISK = 'semantic_objects'
VIZ_NS_LABEL = 'semantic_objects_label'
#: Alpha of a TRACKED disk; a MEMORY disk fades from VIZ_MEMORY_ALPHA_MAX
#: at confidence 1 down to VIZ_MEMORY_ALPHA_MIN at 0, so it is never mistaken
#: for one that is being seen and never quite vanishes before it is removed.
VIZ_TRACKED_ALPHA = 0.8
VIZ_MEMORY_ALPHA_MIN = 0.12
VIZ_MEMORY_ALPHA_MAX = 0.45


def object_label(trk, mem, params) -> str:
    """Return the Foxglove label: class, id, state, remaining time, views."""
    views = ('S' if mem.in_fov_stereo else '-') + ('L' if mem.in_fov_lidar else '-')
    text = f'{trk.class_id} #{trk.track_id}'
    if mem.state == om.MEMORY:
        text += f' MEM {mem.remaining_s(params):.1f}s'
        growth = om.memory_growth_m(trk.class_id, mem, params)
        if growth > 0.0:
            text += f' r+{growth:.2f}'
    elif mem.fov_unseen_s > 0.0:
        text += f' unseen {mem.fov_unseen_s:.1f}s'
    return f'{text} [{views}]'


def build_object_markers(entries, params, frame_id, stamp, lifetime_s, drawn_ids):
    """
    Build the MarkerArray for [(track, ObjectMemory)]: a disk and a label each.

    The disk's diameter is the EFFECTIVE radius, so a remembered person is
    drawn as large as the obstacle the planner is given. Returns the array
    and the set of ids it drew; ids in drawn_ids that are gone get DELETEs.
    """
    out = MarkerArray()
    seconds = int(lifetime_s)
    lifetime = (seconds, int(round((lifetime_s - seconds) * 1e9)))
    drawn = set()
    for trk, mem in entries:
        r, g, b = _color_for_class(trk.class_id)
        if mem.state == om.MEMORY:
            alpha = VIZ_MEMORY_ALPHA_MIN + (
                VIZ_MEMORY_ALPHA_MAX - VIZ_MEMORY_ALPHA_MIN) * mem.confidence
        else:
            alpha = VIZ_TRACKED_ALPHA
        disk = Marker()
        disk.header.frame_id = frame_id
        disk.header.stamp = stamp
        disk.ns = VIZ_NS_DISK
        disk.id = int(trk.track_id)
        disk.type = Marker.CYLINDER
        disk.action = Marker.ADD
        disk.pose.position.x = float(trk.x_map)
        disk.pose.position.y = float(trk.y_map)
        disk.pose.position.z = 0.05
        disk.pose.orientation.w = 1.0
        disk.scale.x = disk.scale.y = 2.0 * float(om.effective_radius(trk, mem, params))
        disk.scale.z = 0.1
        disk.color = ColorRGBA(r=r, g=g, b=b, a=float(alpha))
        disk.lifetime.sec, disk.lifetime.nanosec = lifetime

        label = Marker()
        label.header = disk.header
        label.ns = VIZ_NS_LABEL
        label.id = disk.id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = disk.pose.position.x
        label.pose.position.y = disk.pose.position.y
        label.pose.position.z = 0.4
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0 if mem.state == om.TRACKED else 0.6)
        label.text = object_label(trk, mem, params)
        label.lifetime.sec, label.lifetime.nanosec = lifetime
        out.markers.extend([disk, label])
        drawn.add(disk.id)
    for gone in sorted(drawn_ids - drawn):
        for ns in (VIZ_NS_DISK, VIZ_NS_LABEL):
            delete = Marker()
            delete.header.frame_id = frame_id
            delete.header.stamp = stamp
            delete.ns = ns
            delete.id = int(gone)
            delete.action = Marker.DELETE
            out.markers.append(delete)
    return out, drawn


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
