#!/usr/bin/env python3
"""Jitter-hardened front wall / front clearance signal for the F1TENTH stack.

Every output of this node is FILTERED. A previous attempt at this signal
published raw per-frame values straight to the topic with nothing damping
them, and flapped: a single pixel-count crossing flipped the wall boolean,
and a single noisy depth frame moved the distance. Nothing here reaches a
topic without passing through either an EMA (`EmaFilter`) or a
dead-band-plus-dwell latch (`HysteresisLatch`), including the per-pixel
object-exclusion mask.

PUBLISHES, one message each per DEPTH frame -- never gated on a
message_filters synchronizer, so a missing detections/mask/obstacles message
degrades the estimate rather than stopping the topic (same "one message per
frame including the empty ones" discipline yolo_detector_node's own
masks_topic invariant follows, and for the same reason: a consumer that
stops receiving cannot tell "nothing in front" from "node wedged"):

  /perception/front_distance   Float32  EMA-smoothed BACKGROUND distance
  /perception/front_wall       Bool     hysteresis + dwell on the PIXEL COUNT
  /perception/front_clearance  Float32  EMA of min(front_distance, nearest
                                        in-corridor obstacle)
  /perception/front_blocked    Bool     hysteresis + dwell, inverted sense

front_distance and front_clearance are NOT interchangeable, and conflating
them is the mistake this node exists to make impossible:

  * front_distance answers "is there a WALL" and nothing else. Detected
    objects are removed from it by construction (see the exclusion mask
    below), so a person standing 40 cm in front of the car does not lower
    it -- the wall six metres behind them is what it reports.
  * front_clearance is the STOP/SLOW-DOWN signal: min(background distance,
    nearest detected obstacle inside the corridor). front_blocked is its
    latched boolean.

Anything making a safety decision wants front_clearance/front_blocked.

THREE DIFFERENT THINGS ARE CALLED "front_clearance" IN THIS STACK
-----------------------------------------------------------------
Read this before wiring anything to the topic below, because the name is
reused and the sources are not interchangeable:

  * /perception/front_clearance -- THIS node. Camera-derived: ZED depth plus
    YOLO objects. Sees things that are not on the map (a person, a chair
    someone just moved) and does not need SLAM to be converged. Blind to
    anything outside the camera's field of view, and inherits every failure
    mode of stereo depth (glass, texureless surfaces, direct sun).
  * /costmap/front_clearance -- f1tenth_costmap's costmap_boundary_node,
    derived from slam_toolbox's occupancy grid. This is the one the mission
    `front_clearance` stop_condition actually reads (see
    f1tenth_behavior's check_stop_condition.py, whose front_clearance_topic
    defaults to it, and mission/condition_eval.py). Writing
    `stop_type: front_clearance` in a mission JSON gets the MAP-derived
    value, NOT this node's.
  * /perception/front_clearance as published by the RETIRED
    wall_detector_node (Open3D RANSAC plane segmentation on the ZED point
    cloud). That node was deleted -- see detection.launch.py's own docstring
    -- so this node reuses a topic name that had a different meaning and a
    different producer historically. Anything found in an old bag, an old
    Foxglove layout, or a pre-retirement doc under this name is that node's
    output, not this one's.

Nothing subscribes to /perception/front_clearance or /perception/front_blocked
as of this pass. Migrating a consumer onto them is a deliberate decision with
its own validation, not a rename.

WHY THE OBJECT-EXCLUSION MASK IS AN EMA AND NOT A FRESHNESS WINDOW
------------------------------------------------------------------
The obvious way to exclude detected objects from a background estimate is a
binary hold: mark the pixels of the latest detections, keep them for N
seconds, drop them when stale. That only survives a FULL detection dropout.
The larger jitter source measured on this stack is the frame-to-frame edge
wobble that happens WHILE an object is being continuously detected -- the
silhouette breathes by a few pixels every frame, and those boundary pixels
alternate between "object" (excluded) and "background" (included in the
percentile band) at frame rate, moving the distance estimate even though
nothing in the scene moved and no detection was ever missed.

So the mask is a per-pixel confidence map in [0, 1], EMA-updated EVERY depth
frame whether or not detections arrived, with a dt-normalized alpha
(`1 - exp(-dt / mask_time_constant)`) so its time constant is a real
wall-clock constant rather than a frame-count one -- this node's rate is set
by the depth stream and is not fixed. Background weight is `1 - confidence`.
A dropout decays the exclusion smoothly instead of deleting it; edge wobble
averages out instead of switching.

OBJECT-EXCLUSION SOURCE, in preference order
--------------------------------------------
  1. `masks_topic` (mono8 instance-index label image, 0 = background,
     value N = detections[N-1] -- see yolo_detector_node.py's own module
     docstring for the encoding). Pixel-accurate, so preferred whenever it
     is actually available. Only a segment-task model publishes it.
  2. `detections_topic` bounding boxes, rasterized as filled rectangles.
     The fallback for the deployed detect-task engine, which publishes no
     masks at all.

Which one is live is decided per frame by comparing header stamps, NOT by a
timeout: yolo_detector_node publishes a mask and a Detection2DArray with the
identical header for the same frame, so "the latest mask belongs to the
latest detections" is an exact test with no tunable in it. A detect-task
model never publishes a mask, the stamps never match, and every frame takes
the box path -- no configuration needed to switch between them.

Box pixel coordinates are used directly against the depth image, i.e. the
detection image and the depth image are assumed to share a resolution. That
is the SAME assumption detection_3d_node.py already makes deliberately (both
are ZED-published under one `general.pub_resolution`/`pub_downscale_factor`,
see zed2_perception.yaml) -- reused here rather than a second, independent
convention. Unlike that node this one cannot verify it, because a
Detection2DArray carries no image dimensions; the mask path CAN and does
(it is a real image), and resizes on mismatch. Rectangles are clamped to the
depth image either way, so a mismatch degrades the exclusion's accuracy
rather than raising.

THE CAR'S OWN LIDAR HOUSING IS CUT OUT OF THE ROI
-------------------------------------------------
It sits in the ZED's field of view (bottom-right of frame) and is a
close-range PHYSICAL object -- it returns real, valid, very small depths.
A low-percentile background estimate would be dominated by it. It is
excluded geometrically, before any statistics, using the same fractional
`lidar_exclusion_x_min/_x_max/_y_min/_y_max` rectangle
yolo_detector_node.py already applies to detections (one calibration, two
consumers -- see stack_params.yaml's own lidar_exclusion_x_min comment for
its provenance and the trade-off). Fractions rather than pixels, so it
stays correct across a resolution change and needs no rescaling between the
RGB and depth images.

WHY THE WALL LATCH IS FED PIXELS AND THE BLOCKED LATCH IS FED METRES
--------------------------------------------------------------------
`front_wall` is a question about COVERAGE ("is a surface filling the ROI"),
so its latch input is the valid background pixel COUNT, not the distance --
a wall that is far away is still a wall, and a distance threshold would say
otherwise. `front_blocked` is a question about RANGE, so its latch input is
the clearance in metres, with `higher_enters=False` (smaller distance means
more triggered).

The blocked latch is fed the SMOOTHED clearance, never the raw one. Feeding
a latch a raw signal wastes the EMA sitting in front of it: the dead band
would be re-crossed by exactly the per-frame noise the filter exists to
remove, and the dwell counter would then be the only thing left holding the
state, which is what the original implementation effectively relied on.

INSUFFICIENT DATA IS None, NOT -1.0
------------------------------------
A frame with too few background pixels yields `None`, which makes both the
EMA hold its last value and the latch reset its dwell counter without
counting toward a flip in either direction. Substituting a sentinel like
-1.0 would instead feed a fictitious "very close" reading into the filter
chain and drag the estimate toward it. -1.0 appears on a topic ONLY when a
filter has never held a value at all (nothing published yet this run) --
i.e. it means "no reading", never "a reading of -1".
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from vision_msgs.msg import Detection2DArray

from f1tenth_messages.msg import Obstacle2DArray

from f1tenth_perception.cpu_affinity import (
    apply_nice,
    declare_nice_param,
)


# ==============================================================================
# Filters -- pure, no rclpy/ROS dependency, independently unit-testable (same
# "pure logic separate from ROS glue" convention yolo_detector_node.py's own
# _bbox_overlap_fraction/_mask_overlap_fraction and is_proximity_too_close.py's
# own _in_lidar_window/_in_front_cone already follow).
# ==============================================================================

class EmaFilter:
    """Exponential moving average that HOLDS on missing input.

    `update(None)` returns the last value unchanged rather than seeding,
    decaying, or resetting -- "no measurement this frame" must not be
    allowed to move the estimate (see the module docstring's "insufficient
    data is None" note). `.value` is None until the first real sample seeds
    it, which is what callers test to decide whether anything can be
    published at all.
    """

    def __init__(self, alpha):
        self.alpha = float(alpha)
        self.value = None

    def update(self, x):
        if x is None:
            return self.value
        if self.value is None:
            self.value = float(x)
        else:
            self.value = self.alpha * float(x) + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


class HysteresisLatch:
    """Boolean state behind BOTH a dead band and a minimum-dwell counter.

    The two guard different failure modes and neither subsumes the other,
    which is why both are required rather than offered as alternatives:

      * The dead band (`enter_threshold` / `exit_threshold`) stops a signal
        parked near ONE threshold from flipping on every crossing -- but a
        signal sweeping cleanly across the whole band still flips instantly.
      * The dwell counter stops a single outlier frame anywhere from
        flipping the state -- but on its own it only delays a flapping
        signal, it does not stop it.

    `higher_enters=True`: value >= enter_threshold proposes True, value <=
    exit_threshold proposes False (a pixel COUNT -- more means present).
    `higher_enters=False` inverts it: value <= enter_threshold proposes True,
    value >= exit_threshold proposes False (a DISTANCE -- smaller means
    triggered). Inside the band, the current state is held.

    `update(None)` resets the dwell counter and returns the state unchanged:
    a gap in the data must not accumulate toward a flip in either direction,
    and must not be mistaken for evidence of the state it interrupts.
    """

    def __init__(self, enter_threshold, exit_threshold, min_dwell_frames=1,
                 initial=False, higher_enters=True):
        self.enter_threshold = enter_threshold
        self.exit_threshold = exit_threshold
        self.min_dwell_frames = max(1, int(min_dwell_frames))
        self.higher_enters = bool(higher_enters)
        self.state = bool(initial)
        self._candidate = bool(initial)
        self._dwell_count = 0

    def update(self, value):
        if value is None:
            self._dwell_count = 0
            return self.state

        if self.higher_enters:
            if value >= self.enter_threshold:
                candidate = True
            elif value <= self.exit_threshold:
                candidate = False
            else:
                candidate = self.state
        else:
            if value <= self.enter_threshold:
                candidate = True
            elif value >= self.exit_threshold:
                candidate = False
            else:
                candidate = self.state

        if candidate == self.state:
            # Already there -- a run of agreeing frames is not evidence for
            # some future opposite flip, so the counter starts clean.
            self._dwell_count = 0
            return self.state

        if candidate == self._candidate:
            self._dwell_count += 1
        else:
            self._candidate = candidate
            self._dwell_count = 1

        if self._dwell_count >= self.min_dwell_frames:
            self.state = candidate
            self._dwell_count = 0

        return self.state


def _stamps_equal(a, b):
    """True when two builtin_interfaces/Time stamps are bit-identical.

    Used to decide whether the latest mask image describes the latest
    Detection2DArray -- yolo_detector_node publishes both with the same
    `msg.header` object for a given frame, so this is an exact identity
    test, deliberately not a tolerance (see module docstring). Anything
    else, including a detect-task model that never publishes masks at all,
    falls through to the bounding-box path.
    """
    return a.sec == b.sec and a.nanosec == b.nanosec


class FrontClearanceNode(Node):
    def __init__(self, **kwargs):
        # **kwargs: lets tests pass parameter_overrides=[...] straight through
        # to rclpy.Node, same convention as yolo_detector_node.py and
        # f1tenth_costmap's costmap_boundary_node.py -- no effect on normal
        # launch-file construction, which never passes kwargs here.
        super().__init__('front_clearance_node', **kwargs)

        declare_nice_param(self)

        # ---- topics --------------------------------------------------------
        # depth_topic defaults to the canonical /camera/depth/image_raw for the
        # same reason image_topic does in yolo_detector_node: the node stays
        # agnostic to which camera is the active source and the bringup launch
        # remaps the selected one onto it.
        self.depth_topic = str(
            self.declare_parameter('depth_topic', '/camera/depth/image_raw').value)
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections').value)
        self.masks_topic = str(
            self.declare_parameter('masks_topic', '/camera/detection_masks').value)
        self.obstacles_topic = str(
            self.declare_parameter('obstacles_topic', '/perception/obstacles_2d').value)

        # ---- ROI -----------------------------------------------------------
        self.roi_half_w = int(self.declare_parameter('roi_half_width_px', 90).value)
        self.roi_half_h = int(self.declare_parameter('roi_half_height_px', 35).value)
        self.min_bg_pixels = int(
            self.declare_parameter('min_bg_pixels_for_reading', 60).value)

        # ---- car's-own-LiDAR exclusion rectangle ---------------------------
        # Same five fractional params yolo_detector_node applies to detections,
        # read from the same stack_params.yaml entries -- see module docstring.
        self.lidar_exclusion_x_min = float(
            self.declare_parameter('lidar_exclusion_x_min', 0.75).value)
        self.lidar_exclusion_x_max = float(
            self.declare_parameter('lidar_exclusion_x_max', 1.0).value)
        self.lidar_exclusion_y_min = float(
            self.declare_parameter('lidar_exclusion_y_min', 0.55).value)
        self.lidar_exclusion_y_max = float(
            self.declare_parameter('lidar_exclusion_y_max', 1.0).value)

        # ---- front_wall latch (input: background PIXEL COUNT) --------------
        self.wall_enter_px = int(self.declare_parameter('wall_enter_px', 150).value)
        self.wall_exit_px = int(self.declare_parameter('wall_exit_px', 50).value)
        self.wall_min_dwell_frames = int(
            self.declare_parameter('wall_min_dwell_frames', 2).value)

        # ---- smoothing ------------------------------------------------------
        self.distance_ema_alpha = float(
            self.declare_parameter('distance_ema_alpha', 0.25).value)
        self.mask_time_constant = float(
            self.declare_parameter('mask_time_constant', 0.35).value)
        self.bg_weight_threshold = float(
            self.declare_parameter('background_weight_threshold', 0.5).value)
        self.bg_pct_low = float(
            self.declare_parameter('background_percentile_low', 5.0).value)
        self.bg_pct_high = float(
            self.declare_parameter('background_percentile_high', 50.0).value)

        # ---- front_clearance / front_blocked -------------------------------
        self.corridor_half_width_m = float(
            self.declare_parameter('corridor_half_width_m', 0.35).value)
        self.obstacle_max_age_s = float(
            self.declare_parameter('obstacle_max_age_s', 0.4).value)
        self.clearance_enter_m = float(
            self.declare_parameter('clearance_enter_m', 0.5).value)
        self.clearance_exit_m = float(
            self.declare_parameter('clearance_exit_m', 0.7).value)
        self.clearance_min_dwell_frames = int(
            self.declare_parameter('clearance_min_dwell_frames', 2).value)

        self.publish_debug_raw = bool(
            self.declare_parameter('publish_debug_raw', False).value)

        # ---- filters / latches ---------------------------------------------
        self.distance_filter = EmaFilter(self.distance_ema_alpha)
        self.clearance_filter = EmaFilter(self.distance_ema_alpha)
        self.wall_latch = HysteresisLatch(
            enter_threshold=self.wall_enter_px,
            exit_threshold=self.wall_exit_px,
            min_dwell_frames=self.wall_min_dwell_frames,
            initial=False, higher_enters=True)
        self.blocked_latch = HysteresisLatch(
            enter_threshold=self.clearance_enter_m,
            exit_threshold=self.clearance_exit_m,
            min_dwell_frames=self.clearance_min_dwell_frames,
            initial=False, higher_enters=False)

        # ---- state ----------------------------------------------------------
        self.bridge = CvBridge()
        # Per-pixel object confidence in [0, 1] at DEPTH resolution (never at
        # RGB resolution: it is only ever consumed against the depth image, so
        # it is built there once instead of resized on every use).
        self.object_conf_mask = None
        self._last_mask_update_time = None
        self._latest_detections = None
        self._latest_mask_msg = None
        self._latest_obstacles = None
        self._latest_obstacles_time = None
        # Previous latch states, so the info log fires on TRANSITIONS only --
        # this node runs at the depth stream's rate and a per-frame info log
        # would bury every other node's output in the shared console.
        self._prev_wall = None
        self._prev_blocked = None

        # ---- ROS interfaces -------------------------------------------------
        self.front_distance_pub = self.create_publisher(
            Float32, '/perception/front_distance', 10)
        self.front_wall_pub = self.create_publisher(
            Bool, '/perception/front_wall', 10)
        self.front_clearance_pub = self.create_publisher(
            Float32, '/perception/front_clearance', 10)
        self.front_blocked_pub = self.create_publisher(
            Bool, '/perception/front_blocked', 10)

        # Debug topics exist only when asked for: they are for tuning the
        # filters against a recorded bag (the raw value the EMA smoothed, and
        # the exact count the wall latch saw) rather than guessing at
        # thresholds, and there is no reason to pay for them in normal
        # operation.
        self.front_distance_raw_pub = None
        self.bg_pixel_count_pub = None
        if self.publish_debug_raw:
            self.front_distance_raw_pub = self.create_publisher(
                Float32, '/perception/debug/front_distance_raw', 10)
            self.bg_pixel_count_pub = self.create_publisher(
                Float32, '/perception/debug/bg_pixel_count', 10)

        # Depth LAST, so the first depth frame cannot arrive before the
        # publishers and the other subscriptions exist.
        self.detections_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._detections_callback, 10)
        self.masks_sub = self.create_subscription(
            Image, self.masks_topic, self._masks_callback, 10)
        self.obstacles_sub = self.create_subscription(
            Obstacle2DArray, self.obstacles_topic, self._obstacles_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10)

        apply_nice(self)

        self.get_logger().info(
            f'front_clearance_node up: depth "{self.depth_topic}", detections '
            f'"{self.detections_topic}", masks "{self.masks_topic}", obstacles '
            f'"{self.obstacles_topic}" -> /perception/front_distance, '
            'front_wall, front_clearance, front_blocked'
            + (' (+ /perception/debug/*)' if self.publish_debug_raw else ''))

    # ------------------------------------------------------------------------
    # Subscriptions that only cache. All the work happens on the depth frame,
    # which is what drives publishing.
    # ------------------------------------------------------------------------

    def _detections_callback(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _masks_callback(self, msg: Image):
        self._latest_mask_msg = msg

    def _obstacles_callback(self, msg: Obstacle2DArray):
        self._latest_obstacles = msg
        self._latest_obstacles_time = self._now_seconds()

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------------
    # Object-exclusion mask
    # ------------------------------------------------------------------------

    def _raw_object_mask(self, depth_shape):
        """This frame's BINARY object footprint (float32 {0.0, 1.0}) at depth
        resolution, from the mask image when it describes the current
        detections and from the bounding boxes otherwise. See the module
        docstring's source-preference section.

        An all-zero mask is a legitimate, meaningful result (nothing detected
        this frame) and is returned as such -- it must still be fed to the
        EMA so a genuinely empty scene decays the exclusion away. Only a
        total absence of detection data returns None ("no information",
        which holds the confidence map instead of decaying it).
        """
        h, w = depth_shape
        det_msg = self._latest_detections
        if det_msg is None:
            return None

        mask_msg = self._latest_mask_msg
        if (mask_msg is not None
                and _stamps_equal(mask_msg.header.stamp, det_msg.header.stamp)):
            # Zero-size mask == "this frame had no detections", the empty-mask
            # invariant yolo_detector_node maintains (see _publish_empty_mask
            # there). Not an error, and not a reason to fall back to boxes:
            # there are none.
            if mask_msg.height == 0 or mask_msg.width == 0:
                return np.zeros((h, w), dtype=np.float32)
            try:
                label_img = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
            except CvBridgeError as exc:
                self.get_logger().warn(
                    f'cv_bridge mask decode failed: {exc} -- falling back to '
                    'bounding boxes for this frame.', throttle_duration_sec=5.0)
            else:
                raw = (label_img > 0).astype(np.float32)
                if raw.shape[:2] != (h, w):
                    # INTER_NEAREST: this is a binarized discrete label map,
                    # and any blending interpolation would invent fractional
                    # membership along every instance edge. Same choice, same
                    # reason, as detection_3d_node's own mask resize.
                    raw = cv2.resize(
                        raw, (w, h), interpolation=cv2.INTER_NEAREST)
                    self.get_logger().warn(
                        f'masks_topic resolution {label_img.shape[:2]} != depth '
                        f'resolution {(h, w)} -- nearest-neighbor-resized.',
                        throttle_duration_sec=5.0)
                return raw

        # Bounding-box fallback. Coordinates are used directly against the
        # depth image and clamped to it -- see the module docstring for why
        # that assumption is inherited rather than invented here.
        raw = np.zeros((h, w), dtype=np.float32)
        for det in det_msg.detections:
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            half_w = float(det.bbox.size_x) / 2.0
            half_h = float(det.bbox.size_y) / 2.0
            x1 = max(int(math.floor(cx - half_w)), 0)
            y1 = max(int(math.floor(cy - half_h)), 0)
            x2 = min(int(math.ceil(cx + half_w)), w)
            y2 = min(int(math.ceil(cy + half_h)), h)
            if x2 > x1 and y2 > y1:
                raw[y1:y2, x1:x2] = 1.0
        return raw

    def _update_object_confidence(self, raw, now):
        """EMA the binary footprint into the persistent confidence map, with a
        dt-normalized alpha so `mask_time_constant` is wall-clock seconds and
        not a frame count -- this node's rate follows the depth stream and is
        neither fixed nor known at startup. See the module docstring for why
        this is an EMA rather than a freshness window.
        """
        dt = 0.0 if self._last_mask_update_time is None \
            else (now - self._last_mask_update_time)
        self._last_mask_update_time = now

        if raw is None:
            # No detection data at all -- hold. Decaying here would slowly
            # re-admit a still-present object into the background estimate.
            return self.object_conf_mask

        if self.object_conf_mask is None or self.object_conf_mask.shape != raw.shape:
            # First frame, or the depth resolution changed underneath us:
            # seed rather than blend incompatible shapes.
            self.object_conf_mask = raw.copy()
            return self.object_conf_mask

        # dt <= 0 (first update, or a bag whose clock jumped backwards) ->
        # alpha 1.0, i.e. take this frame outright instead of producing a
        # negative or undefined weight.
        alpha = 1.0 - math.exp(-dt / self.mask_time_constant) \
            if dt > 0.0 and self.mask_time_constant > 0.0 else 1.0
        self.object_conf_mask = (
            alpha * raw + (1.0 - alpha) * self.object_conf_mask).astype(np.float32)
        return self.object_conf_mask

    # ------------------------------------------------------------------------
    # Background distance
    # ------------------------------------------------------------------------

    def _robust_background_distance(self, depths_1d):
        """Trimmed percentile-band mean: a robust "nearest consistent surface".

        Takes the values between the `background_percentile_low` and
        `background_percentile_high` percentiles and means them. The low end
        drops the speckle outliers a bare minimum would latch onto; the high
        end drops the far-field tail, so a doorway or a gap in the wall does
        not pull the estimate backwards. Averaging the surviving band rather
        than taking a single order statistic (a bare median) is what actually
        damps frame-to-frame movement, since every sample in the band
        contributes instead of one pixel deciding the whole reading.

        Falls back to the mean of the whole set if the band comes out empty,
        which can happen on a degenerate sample where both percentiles land
        on the same value.
        """
        if depths_1d.size == 0:
            return None
        lo = np.percentile(depths_1d, self.bg_pct_low)
        hi = np.percentile(depths_1d, self.bg_pct_high)
        band = depths_1d[(depths_1d >= lo) & (depths_1d <= hi)]
        if band.size == 0:
            band = depths_1d
        return float(np.mean(band))

    def _nearest_corridor_obstacle(self, now):
        """Forward clearance to the nearest obstacle whose disk intersects the
        central corridor, or None.

        Reads /perception/obstacles_2d, which obstacle_projector_node has
        already projected into metric base_link coordinates -- object geometry
        is deliberately NOT re-derived from camera intrinsics here. That work
        exists once, in one node, and duplicating it would produce a second
        set of obstacle positions that could disagree with the ones the MPC is
        actually avoiding.

        An obstacle counts as in-corridor when its own footprint reaches the
        corridor (`|y| - r`), not merely when its centre does: a wide object
        straddling the edge blocks the car exactly as much as a narrow one
        dead ahead. The returned distance is likewise to the obstacle's near
        FACE (`x - r`), floored at 0, so it is directly comparable with a
        depth reading -- and obstacles behind the car (x <= 0) are skipped.
        """
        msg = self._latest_obstacles
        if msg is None or self._latest_obstacles_time is None:
            return None
        if (now - self._latest_obstacles_time) > self.obstacle_max_age_s:
            return None

        nearest = None
        for obs in msg.obstacles:
            x, y, r = float(obs.x), float(obs.y), float(obs.r)
            if x <= 0.0:
                continue
            if abs(y) - r >= self.corridor_half_width_m:
                continue
            candidate = max(0.0, x - r)
            if nearest is None or candidate < nearest:
                nearest = candidate
        return nearest

    # ------------------------------------------------------------------------

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as exc:
            self.get_logger().error(
                f'cv_bridge depth conversion failed: {exc}',
                throttle_duration_sec=5.0)
            return

        now = self._now_seconds()
        h, w = depth.shape[:2]

        # 1. Object-exclusion confidence, refreshed every frame (see module
        #    docstring) -- before the ROI statistics that consume it.
        conf = self._update_object_confidence(self._raw_object_mask((h, w)), now)

        # 2. ROI, centered on the depth image.
        cy, cx = h // 2, w // 2
        y0 = max(0, cy - self.roi_half_h)
        y1 = min(h, cy + self.roi_half_h)
        x0 = max(0, cx - self.roi_half_w)
        x1 = min(w, cx + self.roi_half_w)
        roi = depth[y0:y1, x0:x1]

        # 3. Valid == finite, positive, and not covered by an object.
        valid = np.isfinite(roi) & (roi > 0)
        if conf is not None:
            bg_weight = 1.0 - conf[y0:y1, x0:x1]
            valid &= (bg_weight >= self.bg_weight_threshold)

        # 4. Cut out the car's own LiDAR housing BEFORE any statistics -- it is
        #    a real close-range surface and would dominate the low percentile.
        valid &= ~self._lidar_exclusion_roi_mask((h, w), (y0, y1, x0, x1))

        bg_depths = roi[valid]
        n_valid = int(bg_depths.size)

        raw_distance = (self._robust_background_distance(bg_depths)
                        if n_valid >= self.min_bg_pixels else None)

        # 5. The WALL latch is fed the pixel COUNT, not the distance -- see the
        #    module docstring. Note this is deliberately NOT gated on
        #    min_bg_pixels: a count below that floor is a real, meaningful
        #    observation of "the ROI is empty", and is exactly the evidence
        #    that should drive the latch toward False.
        front_wall = self.wall_latch.update(n_valid)

        # 6. Background distance: EMA, holding on a None frame.
        self.distance_filter.update(raw_distance)
        smoothed_distance = self.distance_filter.value

        # 7-8. Clearance: the nearer of the background and the nearest
        #      in-corridor obstacle, smoothed, then latched on the SMOOTHED
        #      value (see module docstring).
        object_distance = self._nearest_corridor_obstacle(now)
        candidates = [d for d in (smoothed_distance, object_distance) if d is not None]
        raw_clearance = min(candidates) if candidates else None
        self.clearance_filter.update(raw_clearance)
        smoothed_clearance = self.clearance_filter.value
        front_blocked = self.blocked_latch.update(smoothed_clearance)

        self.front_distance_pub.publish(
            Float32(data=self._publishable(smoothed_distance)))
        self.front_wall_pub.publish(Bool(data=bool(front_wall)))
        self.front_clearance_pub.publish(
            Float32(data=self._publishable(smoothed_clearance)))
        self.front_blocked_pub.publish(Bool(data=bool(front_blocked)))

        if self.publish_debug_raw:
            self.front_distance_raw_pub.publish(
                Float32(data=self._publishable(raw_distance)))
            self.bg_pixel_count_pub.publish(Float32(data=float(n_valid)))

        self._log_transitions(front_wall, front_blocked, n_valid,
                              smoothed_distance, smoothed_clearance)

    def _lidar_exclusion_roi_mask(self, depth_shape, roi_bounds):
        """Boolean array shaped like the ROI, True where the ROI overlaps the
        car's-own-LiDAR exclusion rectangle. Fractions of the DEPTH image's own
        dimensions (not the RGB image's), so no cross-resolution rescaling is
        involved -- see module docstring.
        """
        h, w = depth_shape
        y0, y1, x0, x1 = roi_bounds
        ex1 = int(self.lidar_exclusion_x_min * w)
        ex2 = int(math.ceil(self.lidar_exclusion_x_max * w))
        ey1 = int(self.lidar_exclusion_y_min * h)
        ey2 = int(math.ceil(self.lidar_exclusion_y_max * h))

        out = np.zeros((max(0, y1 - y0), max(0, x1 - x0)), dtype=bool)
        # Intersect the rectangle with the ROI, then re-express it in the ROI's
        # own coordinates.
        ix1, iy1 = max(ex1, x0), max(ey1, y0)
        ix2, iy2 = min(ex2, x1), min(ey2, y1)
        if ix2 > ix1 and iy2 > iy1:
            out[iy1 - y0:iy2 - y0, ix1 - x0:ix2 - x0] = True
        return out

    @staticmethod
    def _publishable(value):
        """A filter's value as a float, or -1.0 when it has never held one.

        -1.0 means "no reading yet", never "a reading of -1" -- see the module
        docstring. Everything downstream should treat a negative value as
        absent rather than as a distance.
        """
        return float(value) if value is not None else -1.0

    def _log_transitions(self, front_wall, front_blocked, n_valid,
                         smoothed_distance, smoothed_clearance):
        """Info-log ONLY on an actual latch state change. This node runs at the
        depth stream's rate; a per-frame info line would drown every other
        node sharing the console, and the whole point of the latches is that
        these two booleans are now rare, meaningful events. Per-frame values
        are available on the debug topics (publish_debug_raw) instead, where
        they can be plotted against a bag rather than read in a log.
        """
        if front_wall != self._prev_wall:
            self.get_logger().info(
                f'front_wall {self._prev_wall} -> {front_wall} '
                f'(bg_pixels={n_valid}, front_distance='
                f'{self._publishable(smoothed_distance):.2f} m)')
            self._prev_wall = front_wall
        if front_blocked != self._prev_blocked:
            self.get_logger().info(
                f'front_blocked {self._prev_blocked} -> {front_blocked} '
                f'(front_clearance={self._publishable(smoothed_clearance):.2f} m)')
            self._prev_blocked = front_blocked

    # ------------------------------------------------------------------------
    # FUTURE: LIDAR RANGE FUSION -- deliberately not in this pass.
    #
    # Next step, when it happens: subscribe /scan, take the MEDIAN range over a
    # narrow forward sector matching the camera's own FOV, and use it as a
    # CROSS-CHECK against front_distance/front_clearance -- not as a
    # replacement and not silently merged in. Publish/flag the disagreement
    # rather than picking a winner: the cases where the two differ (glass,
    # out-of-plane obstacles, the LiDAR's single scan plane missing a low or
    # overhanging object) are exactly the cases worth surfacing, and averaging
    # them away would hide the one measurement here that is immune to
    # segmentation jitter entirely.
    # ------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = FrontClearanceNode()
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
