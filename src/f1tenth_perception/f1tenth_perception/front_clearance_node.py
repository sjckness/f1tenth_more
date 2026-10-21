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
  /perception/debug/front_wall String   JSON, every intermediate of this
                                        frame's wall/clearance decision (ROI,
                                        pixel counts, raw and smoothed
                                        distances, objects over the ROI, the
                                        latest LiDAR front-wall fit). Only
                                        when publish_debug_wall; LOGGED, never
                                        consumed for control. Layout:
                                        perception_debug.build_wall_debug().

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
    derived from slam_toolbox's occupancy grid. This USED to be the one the
    mission `front_clearance` stop_condition read; it is not any more. The map-
    derived topic is left to the safety/diagnostic side: f1tenth_diagnostics'
    steering_offset_calibration_node abort, and f1tenth_logger's recording.

WHAT THE MISSION `front_clearance` STOP_CONDITION READS: this node's
/perception/front_DISTANCE, NOT the identically-named front_clearance above
(see f1tenth_behavior's check_stop_condition.py, whose front_clearance_topic
default points at front_distance, and mission/condition_eval.py). The
stop_condition type name is a misnomer kept for schema compatibility. A
mission means "stop N metres from the WALL", so it needs the object-excluded
background distance; routing it to front_clearance would make any object in
the corridor -- the very thing front_clearance exists to notice -- hijack the
wall guard and stop the car short. The two are one letter apart in the config
and opposite in meaning, which is why this section is here.
  * /perception/front_clearance as published by the RETIRED
    wall_detector_node (Open3D RANSAC plane segmentation on the ZED point
    cloud). That node was deleted -- see detection.launch.py's own docstring
    -- so this node reuses a topic name that had a different meaning and a
    different producer historically. Anything found in an old bag, an old
    Foxglove layout, or a pre-retirement doc under this name is that node's
    output, not this one's.

/perception/front_distance has two subscribers: MPC_corr.py (pre-existing, via
the front_depth_monitor_node swap) and f1tenth_behavior's CheckStopCondition,
for the mission `front_clearance` stop_condition. Note what that guard
inherits: every tuning default in this node is still unfitted, so a mission
that stops on front_distance is only as accurate as the ROI/percentile/EMA
constants below.

Nothing subscribes to /perception/front_clearance or /perception/front_blocked,
and that is DELIBERATE, not an unfinished wiring job. The safety path trips on
LiDAR /scan instead (f1tenth_behavior's IsProximityTooClose), because the ZED
front check was deliberately REMOVED from it: stereo depth is unreliable
against large flat/featureless walls and was a live suspect for the
intermittent mid-mission stop-then-resume, where a noisy near-threshold
reading trips an e-stop that self-clears next frame. That behaviour's own
module docstring carries the full reasoning, and a regression test
(test_is_proximity_too_close.py's test_no_camera_front_distance_subscription_
remains) asserts the camera path stays out. This node's filtering is aimed at
exactly that flapping, so re-wiring it is defensible -- but it is a decision
to make against measured jitter on a real run, and it means deleting that
test on purpose.

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

THE FILTER IS ASYMMETRIC: attack instant, release EMA'd. Confidence is
`max(this frame's raw footprint, the EMA)`, so a pixel that is object NOW is
excluded NOW and only the DECAY is smoothed. Both jitter sources above are
release-side -- wobble and dropouts are about not RE-ADMITTING a pixel -- so
ramping the attack side bought nothing and cost the exclusion its purpose: a
symmetric EMA left a newly-detected object counted as background for its
first ~294 ms (alpha ~0.155 at 17 Hz), long enough for a mission guard with
debounce_ticks 3 to stop the car at a person while the wall sat metres
behind. See _update_object_confidence for the full reasoning.

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

ZED INVALID DEPTH: -Inf AND NaN/+Inf ARE NOT THE SAME THING
------------------------------------------------------------
They both fail `np.isfinite`, they both look like "bad pixel", and treating
them alike is a FALSE-SAFE bug -- one that was live in this node until the
too-close short-circuit below was added. Read this before touching
`_too_close_fraction` or the validity mask, because collapsing the two back
into one `np.isfinite` test is the obvious "simplification" and it silently
restores the bug.

  * `-Inf` means CLOSER THAN THE CAMERA'S MINIMUM MEASURABLE RANGE (~0.2 m
    on this rig -- the same floor front_depth_monitor_node/detection_3d_node
    spell as min_valid_depth). It is not missing data. It is a positive
    measurement that something is right in front of the lens, and it is the
    single most safety-relevant reading this node can receive.
  * `NaN` / `+Inf` mean UNKNOWN -- no stereo match. Textureless wall, glass,
    direct sun, an occlusion boundary. Genuinely no information.

WHY DROPPING -Inf INVERTS THE ANSWER, in three independent places, which is
why the fix is a short-circuit and not a smaller patch:

  1. The percentile band is taken over the pixels that SURVIVE the validity
     mask -- and every survivor is FURTHER AWAY than the object that produced
     the -Inf. An object entering minimum range therefore makes the reported
     distance go UP.
  2. Losing those pixels drops the valid count. Below `min_bg_pixels_for_
     reading` the frame yields None, and `EmaFilter.update(None)` HOLDS its
     last -- far -- value.
  3. `wall_latch` is fed that same collapsing pixel count and is deliberately
     NOT gated on that floor, so it is driven toward front_wall = False,
     "no wall here".

An object pressed against the lens thus reads as far, stale-far, and
wall-free at once. Nothing downstream of the validity mask can recover that,
so the too-close case is detected BEFORE it and published directly, past the
percentile band, past both EMAs and past both latches.

`too_close_min_pixel_fraction` IS NOT A NOISE KNOB. It exists to separate a
real object from isolated speckle -Inf pixels, nothing else. It is not a
smoothing control and must never be raised to quieten the output -- raising
it raises the size an object has to reach before the car notices it is
touching one. If the signal is noisy, that is a depth-quality problem, and
the honest fixes are the ROI bounds or the camera configuration.

The car's own LiDAR housing is excluded from the too-close count for the same
reason it is excluded from the background statistics (see above): it is a
permanent close-range fixture in the ZED's field of view, and counting it
would hold the short-circuit on forever.

INSUFFICIENT DATA IS None, NOT -1.0
------------------------------------
A frame with too few background pixels yields `None`, which makes both the
EMA hold its last value and the latch reset its dwell counter without
counting toward a flip in either direction. Substituting a sentinel like
-1.0 would instead feed a fictitious "very close" reading into the filter
chain and drag the estimate toward it. -1.0 appears on a topic ONLY when a
filter has never held a value at all (nothing published yet this run) --
i.e. it means "no reading", never "a reading of -1".

SCENE LABELS (enable_scene_labels; default true) -- scene_labels.py
--------------------------------------------------------------------
With the switch on, every 1/scene_rate_hz a scene_grid_width_px x
scene_grid_height_px sample of the depth frame is back-projected into
base_link and labelled GROUND / WALL / CEILING / IGNORE / OTHER:
  GROUND   the floor, RANSAC-fitted around the TF-predicted plane
           (ground_prior_z_m) and accepted only within
           scene_ground_max_angle_deg / scene_ground_max_offset_m of it.
  CEILING  the histogram peak of heights in [scene_ceiling_min_m,
           scene_ceiling_max_m], EMA'd and step-gated.
  WALL     within scene_wall_band_m of a LiDAR wall plane from
           /perception/wall_planes (no older than scene_wall_max_age_s) and
           inside its span. Never a YOLO-mask pixel. With no LiDAR wall in the
           ROI, a stereo vertical plane may stand in, flagged as fallback.
  IGNORE   above scene_max_relevant_height_m, never a YOLO-mask pixel.
  OTHER    the rest (objects, masked or not).
Published: /perception/scene_labels (mono8 label image at the grid size, the
depth header), /perception/scene_labels_debug (JSON: planes, timing, wall
outlines in depth px), /perception/stereo_wall (JSON: the median WALL distance
in the ROI from the bumper, scene_labels.stereo_wall_distance) and
/perception/unclassified_obstacles (JSON: OTHER clusters outside every mask;
log only). The background statistics (front_distance and the background term
of front_clearance) then use WALL and OTHER pixels only, with the latest
labels no older than scene_wall_max_age_s; the floor-removal path above is the
fallback when there are none.

front_distance is NOT the wall truth when the switch is on: the virtual wall
(lidar_front_wall_node, /perception/virtual_wall) is, and every wall consumer
reads that instead (f1tenth_params.param_defaults.get_wall_distance_topic()).

EVERY PUBLISHED DISTANCE IS FROM THE FRONT BUMPER (2026-10-12)
--------------------------------------------------------------
front_distance and front_clearance are measured from base_front, the URDF
frame f1tenth_description/urdf/base_front.xacro defines 0.405 m ahead of
base_link (measured 2026-10-13; 0.443 before) -- not from the camera lens, and not (for the obstacle term of
front_clearance) from base_link. They used to be both at once:
front_clearance took the min of a lens distance and a base_link one.

The depth estimate itself (the ROI, the percentile band, both EMAs on it, the
wall latch) is computed exactly as before, in the camera's terms. Only what
is PUBLISHED is moved: minus (base_front x - the depth frame's x in
base_link), the lens-to-bumper offset, read from /tf_static (the camera is a
fixed mount). No camera transform yet = no reference = -1.0 on
front_distance, with a throttled warning. The obstacle term is the near
face's x minus base_front x.

A REAL READING IS NEVER PUBLISHED AT OR BELOW ZERO. Every consumer reads
<= 0 as "no reading" (condition_eval's front_clearance guard, wall_turn), so
a wall the offset puts at or behind the bumper -- something closer to the
lens than the bumper is, which the geometry says is touching -- is published
as MIN_FRONT_DISTANCE_M and the stop fires, instead of a value that would
switch the stop off.
"""

import array
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from f1tenth_params.param_defaults import get_value
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from vision_msgs.msg import Detection2DArray

from f1tenth_description.vehicle_frames import (
    base_front_x, beside_body_behind_bumper, front_side_limit_m, MIN_FRONT_DISTANCE_M)
from f1tenth_messages.msg import Obstacle2DArray, StereoColumns, WallLineFit

from f1tenth_perception import (
    inferred_floor, perception_debug, scene_labels, scene_mask, wall_fusion)
from f1tenth_perception.ground_plane import (
    back_project, fit_ground_plane, ground_mask, quat_to_matrix)
from f1tenth_perception.latest_frame_qos import LATEST_FRAME_QOS
from f1tenth_perception.cpu_affinity import (
    apply_nice,
    declare_nice_param,
)


def _parse_polygons(text, logger=None):
    """Fractional self-mask polygons from a JSON parameter string.

    A ROS parameter cannot hold a ragged nested list, so the polygons travel
    as JSON: ``[[[x, y], ...], ...]`` in fractions of width/height. Empty or
    malformed means "no polygon" -- a bad string must not stop the node, it
    must only cost the masking it would have done, loudly.
    """
    text = (text or '').strip()
    if not text:
        return []
    try:
        data = json.loads(text)
        return [[(float(x), float(y)) for x, y in poly] for poly in data]
    except (ValueError, TypeError) as exc:
        if logger is not None:
            logger.error(f'self-mask polygons {text!r} are not [[[x, y], ...], ...]: '
                         f'{exc} -- masking nothing')
        return []


def from_front(distance, offset):
    """
    Return a lens-referenced distance [m] re-referenced to the front bumper.

    offset: base_front x minus the camera's x in base_link (how far the
    bumper sits ahead of the lens). None stays None (no reading); a real
    reading never comes out below MIN_FRONT_DISTANCE_M.
    """
    if distance is None:
        return None
    return max(float(distance) - float(offset), MIN_FRONT_DISTANCE_M)


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


# What the distances in /perception/debug/front_wall are measured from, as
# this node publishes them (module docstring, "EVERY PUBLISHED DISTANCE IS
# FROM THE FRONT BUMPER"): front_distance, the obstacle term and the
# clearance from base_front; the raw pre-EMA stereo value still from the lens.
_WALL_DEBUG_STEREO_ORIGIN = perception_debug.ORIGIN_FRONT
_WALL_DEBUG_OBSTACLE_ORIGIN = perception_debug.ORIGIN_FRONT
_WALL_DEBUG_STEREO_RAW_ORIGIN = perception_debug.ORIGIN_CAMERA


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
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        # Every published distance is from here (module docstring): the URDF's
        # base_front frame, the one base_link -> front offset in the stack.
        self.front_x = base_front_x()

        # ---- ROI -----------------------------------------------------------
        self.roi_half_w = int(self.declare_parameter('roi_half_width_px', 90).value)
        self.roi_half_h = int(self.declare_parameter('roi_half_height_px', 35).value)
        # Rows the ROI centre sits ABOVE the image centre (0 = centred).
        self.roi_shift_up = int(self.declare_parameter('roi_shift_up_px', 0).value)
        self.min_bg_pixels = int(
            self.declare_parameter('min_bg_pixels_for_reading', 60).value)

        # ---- ground removal (ground_plane.py) -------------------------------
        # Floor pixels are dropped from the background statistics: without
        # this the floor below the horizon is the nearest consistent surface
        # and the "wall" reads the floor. Default OFF here so a node built
        # without the yaml keeps the old behaviour; stack_params turns it on.
        self.ground_enabled = bool(
            self.declare_parameter('ground_removal_enabled', False).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic',
            self.depth_topic.rsplit('/', 1)[0] + '/camera_info').value)
        self.ground_prior_z = float(self.declare_parameter('ground_prior_z_m', 0.0).value)
        self.ground_band = float(self.declare_parameter('ground_band_m', 0.15).value)
        self.ground_inlier = float(self.declare_parameter('ground_inlier_m', 0.03).value)
        self.ground_height = float(self.declare_parameter('ground_height_m', 0.06).value)
        self.ground_max_tilt = float(
            self.declare_parameter('ground_max_tilt_deg', 15.0).value)
        self.ground_iters = int(self.declare_parameter('ground_ransac_iters', 60).value)
        self.ground_min_inliers = int(
            self.declare_parameter('ground_min_inliers', 150).value)
        self.ground_stride = max(1, int(
            self.declare_parameter('ground_fit_stride_px', 4).value))
        self._camera_k = None      # (fx, fy, cx, cy, width, height)
        self._ground_rng = np.random.default_rng(0)
        self._last_ground = None   # debug: plane summary and pixel count

        # ---- scene labels (module docstring) --------------------------------
        def p(name, default):
            return self.declare_parameter(name, default).value
        self.scene_enabled = bool(p('enable_scene_labels', get_value('enable_scene_labels')))
        scene_rate = float(p('scene_rate_hz', 5.0))
        self.scene_period = 1.0 / scene_rate if scene_rate > 0.0 else math.inf
        self.scene_w = int(p('scene_grid_width_px', 160))
        self.scene_h = int(p('scene_grid_height_px', 90))
        self.scene_max_age = float(p('scene_wall_max_age_s', 0.3))
        self.scene_ground_angle = float(p('scene_ground_max_angle_deg', 5.0))
        self.scene_ground_offset = float(p('scene_ground_max_offset_m', 0.05))
        self.scene_ground_band = float(p('scene_ground_band_m', 0.04))
        self.scene_ceiling_band = float(p('scene_ceiling_band_m', 0.08))
        self.scene_max_height = float(p('scene_max_relevant_height_m', 1.0))
        self.scene_wall_band = float(p('scene_wall_band_m', 0.06))
        self.scene_wall_min_span = float(p('scene_wall_min_span_m', 0.8))
        self.scene_stereo_min_px = int(p('scene_stereo_wall_min_px', 30))
        self._ceiling = scene_labels.CeilingTracker(
            min_h=float(p('scene_ceiling_min_m', 2.0)), max_h=float(p('scene_ceiling_max_m', 4.0)),
            alpha=float(p('scene_ceiling_alpha', 0.3)),
            max_step_m=float(p('scene_ceiling_max_step_m', 0.25)),
            prior_h=float(p('scene_ceiling_prior_m', 0.0)))
        self._scene = None         # latest labels: {'stamp', 'labels' (h, w) uint8}
        self._wall_planes = None   # (stamp, [WallLine]) from /perception/wall_planes

        # ---- scene MASK (scene_mask.py) -------------------------------------
        # The whole-frame, ROI-free per-pixel classification that the rebuilt
        # wall pipeline consumes. Independent of the scene LABELS above (which
        # still feed the ROI stereo wall distance); the two coexist until the
        # ROI path is retired.
        mask_rate = float(p('scene_mask_rate_hz', 5.0))
        self.scene_mask_enabled = bool(p('scene_mask_enabled', True))
        self.scene_mask_period = 1.0 / mask_rate if mask_rate > 0.0 else math.inf
        self.scene_mask_floor_tol = float(p('scene_mask_floor_tol_m', 0.05))
        self.scene_mask_ceiling_min_z = float(p('scene_mask_ceiling_min_z_m', 2.0))
        self.scene_mask_min_depth = float(p('scene_mask_min_depth_m', 0.2))
        self.scene_mask_max_depth = float(p('scene_mask_max_depth_m', 20.0))
        self.scene_mask_strip_half_w = float(
            p('scene_mask_strip_half_width_m', front_side_limit_m()))
        self.scene_mask_image_topic = str(p('scene_mask_image_topic', '/camera/image_raw'))
        # The published cloud is decimated: it is for inspection and the bag,
        # not the pipeline (StereoColumns.msg is), and the full set is ~50k
        # points a frame, about 3 MB/s at 5 Hz straight into every recording.
        self.scene_mask_cloud_stride = max(1, int(p('scene_mask_cloud_stride', 4)))
        self.stereo_col_percentile = float(p('scene_mask_column_percentile', 20.0))
        self.stereo_col_min_px = int(p('scene_mask_column_min_px', 5))
        # Row decimation of the column summary. The per-column percentile is
        # the single most expensive stage of the tick -- 8.5 ms of 21.8 on an
        # idle Orin, and 5.8 of that is one sort over every WALL pixel
        # (tools/mask_stage_profile.py). The sort is linear in that count and
        # the count is ~110 px per column against min_px 5, so a stride is the
        # one lever that moves it. 1 keeps every pixel.
        self.stereo_col_row_stride = max(1, int(p('scene_mask_column_row_stride', 1)))
        # The car's own LiDAR housing is NOT a second calibration: it is the
        # lidar_exclusion_* rectangle yolo_detector_node already applies to
        # detections. An explicit polygon list overrides it; empty derives it,
        # so the rectangle has exactly one definition in the workspace.
        override = _parse_polygons(p('scene_mask_self_polygons', ''), self.get_logger())
        self._self_polygons = (override or [scene_mask.rect_polygon(
            float(get_value('lidar_exclusion_x_min')),
            float(get_value('lidar_exclusion_y_min')),
            float(get_value('lidar_exclusion_x_max')),
            float(get_value('lidar_exclusion_y_max')))])
        self._self_polygons += _parse_polygons(
            p('scene_mask_occluded_polygons',
              '[[[0.0, 0.78], [0.16, 0.78], [0.16, 1.0], [0.0, 1.0]]]'),
            self.get_logger())
        self._self_mask = None      # cached per (h, w): the polygons are static
        self._next_scene_mask = 0.0
        self._latest_image_msg = None
        self._latest_scan = None    # (receipt time, LaserScan)
        self._next_scene = 0.0

        # ---- LiDAR-inferred floor (inferred_floor.py) -----------------------
        # The floor the LiDAR can see where the stereo matched nothing. Its own
        # class, never FLOOR: see that module's docstring for why, and for what
        # r_eff is and why it can only ever REFUSE to infer.
        self.inferred_floor_enabled = bool(p('inferred_floor_enabled', True))
        self.inferred_floor_r_max = float(p('inferred_floor_r_max_m', 3.0))
        self.inferred_floor_margin = float(p('inferred_floor_margin_m', 0.15))
        self.inferred_floor_window_k = int(p('inferred_floor_window_k', 3))
        self.inferred_floor_sigma_beams = float(p('inferred_floor_sigma_beams', 1.5))
        self.inferred_floor_max_scan_age = float(p('inferred_floor_max_scan_age_s', 0.15))
        self._floor_lut = None      # inferred_floor.FloorLut, rebuilt on any change
        self._inferred_floor_stats = None

        # ---- the scene-mask watchdog ----------------------------------------
        # A topic that stops is invisible to its consumers: the LiDAR node
        # holds the last profile it received and reports "LiDAR only" per
        # cycle, which reads the same whether the camera is 0.2 s late or has
        # been dead for 12 s. This says which, every tick, whether or not a
        # depth frame ever arrives again.
        self.watchdog_age_s = float(p('scene_mask_watchdog_age_s', 1.0))
        watchdog_period = float(p('scene_mask_watchdog_period_s', 0.5))
        self.watchdog_period = watchdog_period if watchdog_period > 0.0 else 0.0
        self._last_depth_time = None     # node clock, a depth MESSAGE arrived
        self._last_tick_time = None      # node clock, a mask tick completed
        self._last_profile_stamp = None  # the IMAGE stamp of the last profile
        self._watchdog_stale = False

        # ---- too-close (-Inf) short-circuit --------------------------------
        # See the module docstring's own "-Inf AND NaN/+Inf ARE NOT THE SAME
        # THING" section for the ZED semantics and for why this has to bypass
        # every filter rather than feed one.
        #
        # 0.2 m matches the ~0.2 m ZED minimum-stereo-range floor
        # front_depth_monitor_node and detection_3d_node already spell as
        # min_valid_depth -- one number for "the closest this camera can
        # actually measure", three consumers. A -Inf pixel means the true
        # distance is somewhere BELOW this, so publishing it is conservative
        # in the right direction: it is an upper bound on a distance we know
        # only by its ceiling.
        self.too_close_clearance_m = float(
            self.declare_parameter('too_close_clearance_m', 0.2).value)
        # Fraction of the ROI (excluding the car's own LiDAR housing) that
        # must read -Inf before the short-circuit fires. NOT a noise knob --
        # see the module docstring. It separates a real object from isolated
        # speckle; raising it raises how big an object must be before the car
        # notices it is touching one.
        self.too_close_min_fraction = float(
            self.declare_parameter('too_close_min_pixel_fraction', 0.02).value)

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
        # |y| beyond which an obstacle behind the bumper face is beside the
        # car, not in front of it (_nearest_corridor_obstacle).
        self.front_side_limit_m = float(
            self.declare_parameter('front_side_limit_m', front_side_limit_m()).value)
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
        # /perception/debug/front_wall (see module docstring). Also the only
        # reason this node reads /perception/lidar_front_wall: the payload
        # carries the LiDAR's own fit next to the stereo value, for the log.
        self.publish_debug_wall = bool(
            self.declare_parameter('publish_debug_wall', True).value)

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
        # Which object footprint the last _raw_object_mask() used, and the
        # latest LiDAR front-wall fit -- both only for the debug payload.
        self._latest_object_source = 'none'
        self._latest_lidar_wall = None
        # Previous latch states, so the info log fires on TRANSITIONS only --
        # this node runs at the depth stream's rate and a per-frame info log
        # would bury every other node's output in the shared console.
        self._prev_wall = None
        self._prev_blocked = None
        # The depth frame's pose in base_frame, from /tf_static only (no
        # TransformListener: a handful of fixed edges, not every /tf message).
        self._tf_buffer = Buffer()
        self._frame_poses = {}   # frame_id -> (x, y, yaw) in base_frame
        self._frame_transforms = {}   # frame_id -> (R 3x3, t 3) into base_frame

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
        self.wall_debug_pub = None
        if self.publish_debug_wall:
            self.wall_debug_pub = self.create_publisher(
                String, '/perception/debug/front_wall', 10)
            self.lidar_wall_sub = self.create_subscription(
                WallLineFit, '/perception/lidar_front_wall', self._lidar_wall_callback, 10)

        # Depth LAST, so the first depth frame cannot arrive before the
        # publishers and the other subscriptions exist.
        self.detections_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._detections_callback, 10)
        self.masks_sub = self.create_subscription(
            Image, self.masks_topic, self._masks_callback, LATEST_FRAME_QOS)
        self.obstacles_sub = self.create_subscription(
            Obstacle2DArray, self.obstacles_topic, self._obstacles_callback, 10)
        # The depth frame's pose is what moves every published distance from
        # the lens to the bumper.
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_callback, QoSProfile(
                depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST))
        if self.scene_enabled:
            self.scene_labels_pub = self.create_publisher(Image, '/perception/scene_labels', 10)
            self.scene_debug_pub = self.create_publisher(
                String, '/perception/scene_labels_debug', 10)
            self.stereo_wall_pub = self.create_publisher(String, '/perception/stereo_wall', 10)
            self.unclassified_pub = self.create_publisher(
                String, '/perception/unclassified_obstacles', 10)
            self.create_subscription(
                String, '/perception/wall_planes', self._wall_planes_callback, 10)
        if self.scene_mask_enabled:
            # The overlay is rendered only when something is subscribed (the
            # same discipline the annotated detection image follows): holding
            # the newest RGB message is cheap, decoding and tinting it is not.
            self.scene_mask_image_pub = self.create_publisher(
                Image, '/perception/scene_labels/image', 10)
            # The CLASS MASK AS DATA, and the depth it was computed from, for
            # the rectangle verification in lidar_front_wall_node
            # (wall_fusion, THE RECTANGLE). Published as a PAIR and only when
            # something is subscribed, for two reasons: the verification needs
            # both to be the same tick -- a mask paired with a depth frame
            # from a different one would put WALL pixels at the wrong
            # distances -- and at 640x360 they are 230 KB + 460 KB a tick,
            # which nothing should pay for unless it is reading them. mono16
            # millimetres rather than 32FC1 metres halves the depth's bytes;
            # 1 mm is two orders of magnitude finer than the tolerance the
            # verification compares it against.
            self.scene_mask_classes_pub = self.create_publisher(
                Image, '/perception/scene_labels/classes', 10)
            self.scene_mask_depth_pub = self.create_publisher(
                Image, '/perception/scene_labels/depth_mm', 10)
            self.wall_points_pub = self.create_publisher(
                PointCloud2, '/perception/wall_points', 10)
            self.scene_mask_stats_pub = self.create_publisher(
                String, '/perception/scene_labels/stats', 10)
            self.stereo_columns_pub = self.create_publisher(
                StereoColumns, '/perception/stereo_columns', 10)
            self.mask_health_pub = self.create_publisher(
                String, '/perception/scene_labels/health', 10)
            self.create_subscription(
                Image, self.scene_mask_image_topic, self._image_callback, LATEST_FRAME_QOS)
            self.create_subscription(
                LaserScan, '/scan', self._scan_callback, LATEST_FRAME_QOS)
        if self.ground_enabled or self.scene_enabled or self.scene_mask_enabled:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self._camera_info_callback, 10)
        # Newest depth frame only (latest_frame_qos.py): the callback does
        # the work, and with KEEP_LAST 10 a slow tick made every later frame
        # queue behind the backlog (the front_wall payload was 164 ms old on
        # the 2026-10-13 bags against 52-55 ms for the depth itself).
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, LATEST_FRAME_QOS)

        # The watchdog is a TIMER, not a check inside depth_callback: the
        # failure it exists to report is depth_callback not being called, and
        # a check that lives there cannot report its own absence.
        self.watchdog_timer = None
        if self.scene_mask_enabled and self.watchdog_period > 0.0:
            self.watchdog_timer = self.create_timer(
                self.watchdog_period, self._publish_mask_health)

        apply_nice(self)

        self.get_logger().info(
            f'front_clearance_node up: depth "{self.depth_topic}", detections '
            f'"{self.detections_topic}", masks "{self.masks_topic}", obstacles '
            f'"{self.obstacles_topic}" -> /perception/front_distance, '
            'front_wall, front_clearance, front_blocked'
            + (' (+ /perception/debug/*)' if self.publish_debug_raw else '')
            + f'; distances from base_front, {self.front_x:.3f} m ahead of '
            f'{self.base_frame}')

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

    def _lidar_wall_callback(self, msg: WallLineFit):
        self._latest_lidar_wall = msg

    def _wall_planes_callback(self, msg: String):
        try:
            d = json.loads(msg.data)
            walls = [scene_labels.WallLine.from_dict(w) for w in d.get('walls', [])]
            self._wall_planes = (float(d['stamp']), walls)
        except (ValueError, KeyError, TypeError):
            return

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tf_static_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'front_clearance_node')

    def _frame_pose(self, frame_id):
        """(x, y, yaw) of frame_id in base_frame from /tf_static, cached; None if unknown."""
        if not frame_id:
            return None
        pose = self._frame_poses.get(frame_id)
        if pose is not None:
            return pose
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            return None
        t = stamped.transform
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (float(t.translation.x), float(t.translation.y), yaw)
        self._frame_poses[frame_id] = pose
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: x={pose[0]:.3f} y={pose[1]:.3f} '
            f'yaw={math.degrees(yaw):.1f} deg')
        return pose

    def _frame_transform(self, frame_id):
        """(R, t) taking frame_id points into base_frame, cached; None if unknown."""
        if not frame_id:
            return None
        cached = self._frame_transforms.get(frame_id)
        if cached is not None:
            return cached
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            return None
        t, q = stamped.transform.translation, stamped.transform.rotation
        cached = (quat_to_matrix(q.x, q.y, q.z, q.w),
                  np.array([t.x, t.y, t.z], dtype=float))
        self._frame_transforms[frame_id] = cached
        return cached

    def _camera_info_callback(self, msg: CameraInfo):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0:
            self._camera_k = (float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                              int(msg.width), int(msg.height))

    def _ground_roi_mask(self, depth, frame_id, bounds):
        """ROI-shaped bool, True on floor pixels; None when it cannot be computed.

        The plane is fitted on a strided sample of everything from the ROI's
        top row to the image bottom (more floor than the ROI alone holds), then
        every finite ROI pixel is tested against it.
        """
        if not self.ground_enabled or self._camera_k is None:
            return None
        tf = self._frame_transform(frame_id)
        if tf is None:
            return None
        h, w = depth.shape[:2]
        fx, fy, cx, cy, kw, kh = self._camera_k
        if kw > 0 and kh > 0 and (kw != w or kh != h):
            sx, sy = w / kw, h / kh
            fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy
        rot, trans = tf
        y0, y1, x0, x1 = bounds

        rows, cols = np.mgrid[y0:h:self.ground_stride, x0:x1:self.ground_stride]
        rows, cols = rows.ravel(), cols.ravel()
        z = depth[rows, cols]
        keep = np.isfinite(z) & (z > 0)
        plane = fit_ground_plane(
            back_project(depth, rows[keep], cols[keep], fx, fy, cx, cy, rot, trans),
            prior_z=self.ground_prior_z, band_m=self.ground_band,
            inlier_m=self.ground_inlier, max_tilt_deg=self.ground_max_tilt,
            iters=self.ground_iters, min_inliers=self.ground_min_inliers,
            rng=self._ground_rng)

        roi = depth[y0:y1, x0:x1]
        out = np.zeros(roi.shape, dtype=bool)
        finite = np.isfinite(roi) & (roi > 0)
        rr, cc = np.nonzero(finite)
        if rr.size:
            pts = back_project(depth, rr + y0, cc + x0, fx, fy, cx, cy, rot, trans)
            out[rr, cc] = ground_mask(pts, plane, self.ground_height)
        self._last_ground = {
            'fitted': plane.fitted, 'inliers': plane.inliers,
            'tilt_deg': round(plane.tilt_deg(), 2),
            'z0_m': round(plane.z_at_origin(), 3),
            'px': int(np.count_nonzero(out))}
        return out

    def _intrinsics(self, w, h):
        """(fx, fy, cx, cy) for a w x h depth image, or None before camera_info."""
        if self._camera_k is None:
            return None
        fx, fy, cx, cy, kw, kh = self._camera_k
        if kw > 0 and kh > 0 and (kw != w or kh != h):
            sx, sy = w / kw, h / kh
            fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy
        return fx, fy, cx, cy

    def _scene_roi_valid(self, stamp_sec, bounds, shape_hw):
        """ROI-shaped bool, True on WALL/OTHER pixels; None without fresh labels."""
        scene = self._scene
        if scene is None or abs(stamp_sec - scene['stamp']) > self.scene_max_age:
            return None
        y0, y1, x0, x1 = bounds
        h, w = shape_hw
        lab = scene['labels']
        gh, gw = lab.shape
        roi_lab = lab[np.ix_(np.arange(y0, y1) * gh // h, np.arange(x0, x1) * gw // w)]
        return (roi_lab == scene_labels.LABEL_WALL) | (roi_lab == scene_labels.LABEL_OTHER)

    def _label_scene(self, depth, header, conf, bounds):
        """One labelling tick (module docstring, SCENE LABELS). Publishes its
        four topics and caches the label image for the ROI statistics."""
        t_start = time.perf_counter()
        stamp = perception_debug.stamp_to_sec(header.stamp)
        tf = self._frame_transform(header.frame_id)
        h, w = depth.shape[:2]
        k = self._intrinsics(w, h)
        if tf is None or k is None:
            return
        fx, fy, cx, cy = k
        rot, trans = tf
        gh, gw = self.scene_h, self.scene_w
        rows = np.arange(gh) * h // gh + h // (2 * gh)
        cols = np.arange(gw) * w // gw + w // (2 * gw)
        rr, cc = np.meshgrid(rows, cols, indexing='ij')
        rr, cc = rr.ravel(), cc.ravel()
        z = depth[rr, cc]
        valid = np.isfinite(z) & (z > 0)
        rv, cv = rr[valid], cc[valid]
        pts = back_project(depth, rv, cv, fx, fy, cx, cy, rot, trans)
        if conf is not None:
            in_mask = conf[rv, cv] > (1.0 - self.bg_weight_threshold)
        else:
            in_mask = np.zeros(rv.shape, dtype=bool)

        ground, fitted = scene_labels.fit_ground(
            pts[~in_mask], prior_z=self.ground_prior_z, candidate_band_m=self.ground_band,
            inlier_m=self.ground_inlier, max_angle_deg=self.scene_ground_angle,
            max_offset_m=self.scene_ground_offset, iters=self.ground_iters,
            min_inliers=self.ground_min_inliers, rng=self._ground_rng)
        heights = ground.height(pts)
        ceiling_h = self._ceiling.update(heights[~in_mask])
        planes = self._wall_planes
        walls = []
        planes_age = None
        if planes is not None:
            planes_age = stamp - planes[0]
            if abs(planes_age) <= self.scene_max_age:
                walls = list(planes[1])
        labels, wall_idx = scene_labels.label_points(
            pts, in_mask, ground, ceiling_h, walls, ground_band_m=self.scene_ground_band,
            ceiling_band_m=self.scene_ceiling_band,
            max_relevant_height_m=self.scene_max_height, wall_band_m=self.scene_wall_band)

        y0, y1, x0, x1 = bounds
        in_roi = (rv >= y0) & (rv < y1) & (cv >= x0) & (cv < x1)
        fallback = False
        if not np.any((labels == scene_labels.LABEL_WALL) & in_roi):
            fw = scene_labels.fallback_wall(
                pts, in_roi & (labels == scene_labels.LABEL_OTHER) & ~in_mask,
                inlier_m=self.scene_wall_band, min_span_m=self.scene_wall_min_span)
            if fw is not None:
                lab2, _ = scene_labels.label_points(
                    pts, in_mask, ground, ceiling_h, [fw], ground_band_m=self.scene_ground_band,
                    ceiling_band_m=self.scene_ceiling_band,
                    max_relevant_height_m=self.scene_max_height,
                    wall_band_m=self.scene_wall_band)
                sel = (labels == scene_labels.LABEL_OTHER) & (lab2 == scene_labels.LABEL_WALL)
                walls.append(fw)
                labels[sel] = scene_labels.LABEL_WALL
                wall_idx[sel] = len(walls) - 1
                fallback = bool(np.any(sel))
        d_stereo, n_wall_px, k_wall = scene_labels.stereo_wall_distance(
            pts, labels, wall_idx, walls, in_roi, self.front_x, self.scene_stereo_min_px)
        clusters = scene_labels.cluster_other(pts, labels, in_mask)

        grid = np.zeros(gh * gw, dtype=np.uint8)
        grid[valid] = labels
        grid = grid.reshape(gh, gw)
        self._scene = {'stamp': stamp, 'labels': grid}
        ms = (time.perf_counter() - t_start) * 1e3

        img = self.bridge.cv2_to_imgmsg(grid, encoding='mono8')
        img.header = header
        self.scene_labels_pub.publish(img)
        used_source = walls[k_wall].source if k_wall is not None else None
        self.stereo_wall_pub.publish(String(data=json.dumps({
            'stamp': stamp, 'origin': 'front', 'distance_m': (
                None if d_stereo is None else round(d_stereo, 4)),
            'n_px': n_wall_px, 'source': used_source,
            'fallback': used_source == 'stereo'})))
        cands = []
        for members, summary in clusters:
            summary['bbox_px'] = [int(cv[members].min()), int(rv[members].min()),
                                  int(cv[members].max()), int(rv[members].max())]
            cands.append(summary)
        self.unclassified_pub.publish(String(data=json.dumps({
            'stamp': stamp, 'frame_id': self.base_frame, 'candidates': cands})))

        outlines = []
        for wall in walls:
            (ax, ay), (bx, by) = wall.endpoints()
            z0 = -ground.d / float(ground.normal[2])
            corners = np.array([[ax, ay, z0], [bx, by, z0],
                                [bx, by, z0 + self.scene_max_height],
                                [ax, ay, z0 + self.scene_max_height]])
            opt = (corners - trans) @ rot      # base_link -> optical
            if np.all(opt[:, 2] > 0.1):
                outlines.append([[round(float(fx * x / zz + cx), 1),
                                  round(float(fy * y / zz + cy), 1)] for x, y, zz in opt])
            else:
                outlines.append(None)
        counts = np.bincount(grid.ravel(), minlength=6)
        self.scene_debug_pub.publish(String(data=json.dumps({
            'stamp': stamp, 'ms': round(ms, 2), 'grid': [gw, gh], 'img_wh': [w, h],
            'ground': {'fitted': fitted, 'tilt_deg': round(ground.tilt_deg(), 2),
                       'z0_m': round(ground.z_at_origin(), 3)},
            'ceiling_m': None if ceiling_h is None else round(ceiling_h, 3),
            'planes_age_s': None if planes_age is None else round(planes_age, 3),
            'walls': [dict(wl.to_dict(), outline_px=o) for wl, o in zip(walls, outlines)],
            'stereo_wall_m': None if d_stereo is None else round(d_stereo, 4),
            'stereo_wall_px': n_wall_px, 'fallback': fallback,
            'counts': {scene_labels.LABEL_NAMES[i]: int(c) for i, c in enumerate(counts)},
            'n_candidates': len(cands)})))

    # ------------------------------------------------------------------------
    # Scene MASK (scene_mask.py): whole frame, no ROI
    # ------------------------------------------------------------------------

    def _image_callback(self, msg: Image):
        self._latest_image_msg = msg

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = (self._now_seconds(), msg)

    def _scan_points_base(self, scan):
        """(N, 3) base_frame points of a scan's valid returns, z = the laser's.

        The scan plane is a plane in base_link, so the z of every return is
        the laser's own mount height; the cross-check only ever uses x and y,
        and the overlay wants the real height to land on the right rows.
        scene_mask.scan_to_base does it, for the run report's overlay too.
        """
        pose = self._frame_transform(scan.header.frame_id)
        if pose is None:
            return None
        pts, _ranges = scene_mask.scan_to_base(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, *pose)
        return pts

    def _scene_mask_tick(self, depth, header, now):
        """One whole-frame classification (scene_mask.py). Publishes the class
        stats, the WALL point cloud and -- only when subscribed -- the overlay.
        """
        t_start = time.perf_counter()
        tf = self._frame_transform(header.frame_id)
        h, w = depth.shape[:2]
        k = self._intrinsics(w, h)
        if tf is None or k is None:
            return
        fx, fy, cx, cy = k
        rot, trans = tf

        if self._self_mask is None or self._self_mask.shape != (h, w):
            self._self_mask = scene_mask.polygon_mask((h, w), self._self_polygons)
        raw = self._raw_object_mask((h, w))
        object_mask = None if raw is None else (raw > 0.5)

        labels, _z = scene_mask.classify(
            depth, fx, fy, cx, cy, rot, trans,
            self_mask=self._self_mask, object_mask=object_mask,
            floor_tol=self.scene_mask_floor_tol,
            ceiling_min_z=self.scene_mask_ceiling_min_z,
            min_depth=self.scene_mask_min_depth, max_depth=self.scene_mask_max_depth)
        if self.inferred_floor_enabled:
            self._infer_floor(labels, header, (fx, fy, cx, cy), rot, trans)
        points, rows, cols = scene_mask.wall_points(
            depth, labels, fx, fy, cx, cy, rot, trans)

        # The compact per-column profile is the PIPELINE's interface (see
        # StereoColumns.msg); the cloud below is for inspection and recording.
        self._publish_stereo_columns(header, depth, rows, cols, points, w, h, k)

        # create_cloud_xyz32 ALIASES the header it is given, so overriding
        # the cloud's frame_id used to rewrite the depth frame's own header to
        # 'base_link' for everything later in this tick. The points really are
        # in the base frame, so the cloud needs its own header.
        cloud_header = Header(stamp=header.stamp, frame_id=self.base_frame)
        cloud = point_cloud2.create_cloud_xyz32(
            cloud_header, points[::self.scene_mask_cloud_stride].astype(np.float32))
        self.wall_points_pub.publish(cloud)

        stats = scene_mask.class_stats(
            labels, depth=depth, points=points, front_x=self.front_x,
            strip_half_width_m=self.scene_mask_strip_half_w)
        stats.update({
            'stamp': perception_debug.stamp_to_sec(header.stamp),
            'frame_id': self.base_frame,
            'object_source': self._latest_object_source,
            'img_wh': [w, h],
            'strip_half_width_m': self.scene_mask_strip_half_w,
            'ms': round((time.perf_counter() - t_start) * 1e3, 2),
            'inferred_floor': self._inferred_floor_stats,
        })
        self.scene_mask_stats_pub.publish(String(data=json.dumps(stats)))
        # The watchdog's "a tick completed" mark: after the publishes, so a
        # tick that died half way through does not count as one.
        self._last_tick_time = now

        if (self.scene_mask_classes_pub.get_subscription_count() > 0
                or self.scene_mask_depth_pub.get_subscription_count() > 0):
            self._publish_scene_mask_classes(labels, depth, header)
        if self.scene_mask_image_pub.get_subscription_count() > 0:
            self._publish_scene_mask_overlay(labels, header, (fx, fy, cx, cy), rot, trans)

    def _publish_mask_health(self):
        """The scene mask's own liveness, on /perception/scene_labels/health.

        Three ages, because they fail separately and the difference between
        them is the whole diagnosis:

          depth_age_s    since a depth MESSAGE arrived. Large means the camera
                         or the ZED wrapper stopped, and nothing this node
                         does can help. On corridor10m_20261015T171753 this is
                         the one that went: the wrapper's own depth
                         camera_info -- a few hundred bytes, under no recorder
                         pressure -- fell from 65 messages per 10 s to 6
                         between t = 30 s and t = 40 s, while /scan and
                         /odometry/filtered dropped by a third over the same
                         window. The camera stopped; the node did not.
          tick_age_s     since a mask tick COMPLETED. Large while depth_age_s
                         is small is the node's own fault: the tick is
                         throttled to scene_mask_rate_hz, is raising, or is
                         taking longer than the budget.
          profile_age_s  the published profile's own IMAGE stamp against now.
                         This is the age the LiDAR node's pairing actually
                         sees, so it is the one to compare against
                         stereo_profile_max_age_s over there.

        `stale` is the published verdict, and the WARN is throttled: a stall
        lasting 12 s must be one log line a second at most, not one per tick.
        """
        now = self._now_seconds()

        def age(then):
            return None if then is None else round(now - then, 3)

        profile_age = None
        if self._last_profile_stamp is not None:
            profile_age = round(now - self._last_profile_stamp, 3)
        payload = {
            'stamp': round(now, 3),
            'depth_age_s': age(self._last_depth_time),
            'tick_age_s': age(self._last_tick_time),
            'profile_age_s': profile_age,
            'max_age_s': self.watchdog_age_s,
            'depth_topic': self.depth_topic,
            'rate_hz': (None if not math.isfinite(self.scene_mask_period)
                        else round(1.0 / self.scene_mask_period, 2)),
        }
        stale = (self._last_tick_time is None
                 or now - self._last_tick_time > self.watchdog_age_s)
        payload['stale'] = bool(stale)
        payload['reason'] = (
            None if not stale
            else 'never_ticked' if self._last_tick_time is None
            else 'no_depth' if (self._last_depth_time is None
                                or now - self._last_depth_time > self.watchdog_age_s)
            else 'tick_not_running')
        self.mask_health_pub.publish(String(data=json.dumps(payload)))
        if stale:
            self.get_logger().warn(
                f'scene mask STALE ({payload["reason"]}): last tick '
                f'{payload["tick_age_s"]} s ago, last depth frame '
                f'{payload["depth_age_s"]} s ago on "{self.depth_topic}", limit '
                f'{self.watchdog_age_s:.2f} s. Every consumer of '
                '/perception/stereo_columns is reading a profile that old.',
                throttle_duration_sec=1.0)
        elif self._watchdog_stale:
            self.get_logger().info(
                f'scene mask alive again: tick {payload["tick_age_s"]} s ago')
        self._watchdog_stale = bool(stale)

    def _infer_floor(self, labels, header, k, rot, trans):
        """Relabel the no-depth pixels the LiDAR says are empty floor.

        inferred_floor.py owns the geometry and the rules; this is the glue
        that finds the inputs, decides whether they may be used at all, and
        records what happened in the stats payload. `labels` is modified in
        place.

        It NEVER raises on a missing input: no scan yet, no laser transform,
        a scan from a different moment -- each leaves the classes exactly as
        classify produced them and says so in `skipped`. The scene mask is a
        diagnostic surface, and a wrong floor is worse than no floor.
        """
        t_start = time.perf_counter()
        stats = {'applied_px': 0, 'lut_px': 0, 'skipped': None}
        self._inferred_floor_stats = stats
        try:
            self._infer_floor_step(labels, header, k, rot, trans, stats)
        finally:
            # Timed even when it skipped: the skip is the common case while a
            # sensor is missing, and its cost belongs in the frame budget.
            stats['ms'] = round((time.perf_counter() - t_start) * 1e3, 2)

    def _infer_floor_step(self, labels, header, k, rot, trans, stats):
        """_infer_floor's body. Fills `stats`; see that method."""
        held = self._latest_scan
        if held is None:
            stats['skipped'] = 'no_scan'
            return
        scan = held[1]
        # The MASK's own stamp against the SCAN's own stamp, both from the
        # sensors: the receipt times are a property of the recorder and the
        # executor, not of when the two sensors looked.
        age = (perception_debug.stamp_to_sec(header.stamp)
               - perception_debug.stamp_to_sec(scan.header.stamp))
        stats['scan_age_s'] = round(age, 4)
        if abs(age) > self.inferred_floor_max_scan_age:
            stats['skipped'] = 'stale_scan'
            return
        laser = self._frame_pose(scan.header.frame_id)
        if laser is None:
            stats['skipped'] = 'no_laser_transform'
            return

        h, w = labels.shape[:2]
        if self._self_mask is None or self._self_mask.shape != (h, w):
            self._self_mask = scene_mask.polygon_mask((h, w), self._self_polygons)
        n_beams = len(scan.ranges)
        key = inferred_floor.lut_key(
            (h, w), k, rot, trans, laser,
            (scan.angle_min, scan.angle_increment, n_beams), self.inferred_floor_r_max)
        if self._floor_lut is None or not self._floor_lut.matches(key):
            self._floor_lut = inferred_floor.build_lut(
                (h, w), *k, rot, trans, laser, scan.angle_min, scan.angle_increment,
                n_beams, r_max=self.inferred_floor_r_max, self_mask=self._self_mask)
            self.get_logger().info(
                f'inferred floor: LUT rebuilt for {w}x{h}, laser at x={laser[0]:.3f} '
                f'yaw={math.degrees(laser[2]):.1f} deg, {n_beams} beams, r_max '
                f'{self.inferred_floor_r_max:.2f} m: {self._floor_lut.n_valid} of '
                f'{h * w} pixels can be inferred')
        lut = self._floor_lut
        stats['lut_px'] = lut.n_valid
        if lut.n_valid == 0:
            stats['skipped'] = 'empty_lut'
            return

        # One r_eff vector per cycle, shared by every pixel.
        r_eff = inferred_floor.effective_ranges(
            scan.ranges, scan.range_min, scan.range_max,
            far_value=self.inferred_floor_r_max + self.inferred_floor_margin + 1.0,
            window_k=self.inferred_floor_window_k,
            sigma_beams=self.inferred_floor_sigma_beams)
        _labels, applied = inferred_floor.apply(
            labels, lut, r_eff, margin=self.inferred_floor_margin)
        stats['applied_px'] = applied

    def _publish_stereo_columns(self, header, depth, rows, cols, points, width, height, k):
        """The per-column stereo summary the fusion consumes (StereoColumns.msg).

        scene_mask_column_row_stride decimates the WALL pixels first. The
        percentile and the extent are both order statistics over ~110 pixels
        per column, and the cost of both is linear in that count, so a stride
        of 2 halves the most expensive stage of the tick. `n_px` is the count
        the summary was actually taken over, not the pre-stride one: it is
        what min_px is compared against, and reporting the larger number would
        claim evidence that was not used.
        """
        fx, fy, cx, cy = k
        stride = self.stereo_col_row_stride
        if stride > 1 and rows.size:
            rows, cols, points = rows[::stride], cols[::stride], points[::stride]
        depths = depth[rows, cols] if rows.size else np.empty(0)
        distance, n_px = wall_fusion.column_ranges(
            cols, depths, width, percentile=self.stereo_col_percentile,
            min_px=self.stereo_col_min_px)
        z_lo, z_hi = wall_fusion.column_extent(
            cols, points[:, 2] if points.size else np.empty(0), width)
        msg = StereoColumns()
        # Its own header, not the depth frame's: this message's frame_id is
        # the BASE frame (the heights are measured in it) while camera_frame
        # is the optical one, and assigning the depth header here aliased it,
        # so every later consumer in this tick read 'base_link' as the depth
        # frame's own id.
        msg.camera_frame = header.frame_id
        msg.header = Header(stamp=header.stamp, frame_id=self.base_frame)
        msg.width, msg.height = int(width), int(height)
        msg.fx, msg.fy, msg.cx, msg.cy = float(fx), float(fy), float(cx), float(cy)
        msg.distance_m = distance.astype(np.float32).tolist()
        msg.z_min = z_lo.astype(np.float32).tolist()
        msg.z_max = z_hi.astype(np.float32).tolist()
        msg.n_px = n_px.astype(np.uint32).tolist()
        msg.percentile = float(self.stereo_col_percentile)
        msg.min_px = int(self.stereo_col_min_px)
        self.stereo_columns_pub.publish(msg)
        # The IMAGE stamp, which is the age the LiDAR node's pairing sees.
        self._last_profile_stamp = perception_debug.stamp_to_sec(msg.header.stamp)

    def _publish_scene_mask_classes(self, labels, depth, header):
        """The class mask and its depth, as one pair on the depth's own stamp.

        The intrinsics and the camera frame are NOT repeated here: the
        consumer already has them, at this resolution, on the StereoColumns
        message of the same tick (that message's own rationale), and the
        header stamp is what pairs the three.

        BOTH PAYLOADS GO IN AS array.array('B'). rclpy's uint8[] setter has
        exactly one fast path -- an array.array of typecode 'B', assigned
        without validation -- and for anything else, bytes included, it checks
        every element: 44 ms for the mask and 88 ms for the depth, measured,
        against 0.06 and 0.10 ms this way. (That same slow path is what makes
        cv_bridge's own cv2_to_imgmsg cost tens of milliseconds a frame here,
        which is most of what publishing the overlay costs when it is on.)
        """
        # The stamp and the frame COPIED, never the Header object: the depth
        # frame's own header is handed to point_cloud2.create_cloud_xyz32
        # earlier in this tick and then has its frame_id rewritten to the base
        # frame, so an aliased header arrives here saying 'base_link'. That
        # reads as an identity camera transform in the consumer and puts every
        # rectangle sample outside the image -- silently, because an empty
        # rectangle is exactly what a wall the camera cannot see looks like.
        classes = Image()
        classes.header.stamp = header.stamp
        classes.header.frame_id = header.frame_id
        classes.height, classes.width = labels.shape[:2]
        classes.encoding = 'mono8'
        classes.is_bigendian = 0
        classes.step = classes.width
        classes.data = array.array(
            'B', np.ascontiguousarray(labels, dtype=np.uint8).tobytes())
        self.scene_mask_classes_pub.publish(classes)

        d = np.asarray(depth, dtype=np.float32)
        with np.errstate(invalid='ignore'):
            mm = np.where(np.isfinite(d) & (d > 0.0), d * 1000.0, 0.0)
        mm = np.clip(mm, 0.0, 65535.0).astype(np.uint16)
        out = Image()
        out.header.stamp = header.stamp
        out.header.frame_id = header.frame_id
        out.height, out.width = mm.shape[:2]
        out.encoding = 'mono16'
        out.is_bigendian = 0
        out.step = 2 * out.width
        out.data = array.array('B', np.ascontiguousarray(mm).tobytes())
        self.scene_mask_depth_pub.publish(out)

    def _publish_scene_mask_overlay(self, labels, header, k, rot, trans):
        """The class-coloured overlay on the left image, scan points on top."""
        msg = self._latest_image_msg
        if msg is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge image decode failed: {exc}',
                                   throttle_duration_sec=5.0)
            return
        h, w = labels.shape[:2]
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)
        scan_px = None
        if self._latest_scan is not None:
            pts = self._scan_points_base(self._latest_scan[1])
            if pts is not None:
                scan_px = scene_mask.project_to_image(pts, *k, rot, trans)
        out = self.bridge.cv2_to_imgmsg(
            scene_mask.overlay(img, labels, scan_px=scan_px), encoding='bgr8')
        out.header = header
        self.scene_mask_image_pub.publish(out)

    def _front_offset(self, camera_frame):
        """
        Return how far the bumper sits ahead of the depth frame's origin [m], or None.

        base_front x minus the camera's x in base_frame, from /tf_static. None
        until that transform has arrived (nothing can be re-referenced yet).
        """
        cam = self._frame_pose(camera_frame)
        if cam is None:
            return None
        return self.front_x - cam[0]

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
        self._latest_object_source = 'none'
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
                self._latest_object_source = 'mask'
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
                self._latest_object_source = 'mask'
                return raw

        # Bounding-box fallback. Coordinates are used directly against the
        # depth image and clamped to it -- see the module docstring for why
        # that assumption is inherited rather than invented here.
        self._latest_object_source = 'boxes'
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
        ema = alpha * raw + (1.0 - alpha) * self.object_conf_mask

        # ATTACK IS INSTANT, RELEASE IS THE EMA -- and the asymmetry is the
        # whole point, not a shortcut.
        #
        # A symmetric EMA ramps a newly-detected object's pixels UP from 0,
        # so for the first few frames of its life they sit below
        # background_weight_threshold and are counted as BACKGROUND. At the
        # deployed rates that hole is not marginal: mask_time_constant 0.35 s
        # against a ~17 Hz depth stream gives alpha ~0.155, so confidence
        # needs 5 frames (~294 ms) to cross 0.5. For those 294 ms an object
        # that has been detected the entire time is inside the percentile
        # band, dragging front_distance from the wall down toward the object
        # -- and 294 ms is several BT ticks, so a mission guard with
        # debounce_ticks 3 fires on the transient. That is a wall guard
        # stopping at a person with the wall metres further back.
        #
        # Taking the elementwise max with this frame's raw footprint removes
        # the ramp without touching either behaviour the EMA exists for (see
        # the module docstring): a pixel that is object NOW is excluded NOW,
        # while a pixel that has stopped being reported still decays down
        # through `ema` over mask_time_constant instead of being deleted. Both
        # documented purposes are release-side -- edge wobble must not re-admit
        # a boundary pixel, and a detection dropout must not re-admit a
        # still-present object -- so damping the attack side was never buying
        # anything, and was costing the exclusion its whole reason to exist.
        self.object_conf_mask = np.maximum(raw, ema).astype(np.float32)
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
        FACE (`x - r`), floored at 0, in base_link -- depth_callback moves it
        to the front bumper with the background -- and obstacles behind the
        car (x <= 0) are skipped.

        BESIDE THE CAR IS NOT IN FRONT OF IT (2026-10-13): an obstacle whose
        near face is behind the bumper face is skipped when it stays clear of
        the car's width (|y| beyond front_side_limit_m) and does not stick out
        into the corridor ahead of the bumper -- vehicle_frames.
        beside_body_behind_bumper. Before, it was counted and its distance
        clamped to MIN_FRONT_DISTANCE_M, so someone standing next to the nose
        read 0.01 and BLOCKED. Inside the car's width it still counts and
        clamps: that is touching.
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
            if beside_body_behind_bumper(
                    x, y, r, front_x=self.front_x, side_limit_m=self.front_side_limit_m,
                    corridor_half_width_m=self.corridor_half_width_m):
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
        self._last_depth_time = now
        h, w = depth.shape[:2]

        # 1. Object-exclusion confidence, refreshed every frame (see module
        #    docstring) -- before the ROI statistics that consume it.
        conf = self._update_object_confidence(self._raw_object_mask((h, w)), now)

        # 2. ROI, centered on the depth image, raised by roi_shift_up_px.
        cy, cx = h // 2 - self.roi_shift_up, w // 2
        y0 = max(0, cy - self.roi_half_h)
        y1 = min(h, cy + self.roi_half_h)
        x0 = max(0, cx - self.roi_half_w)
        x1 = min(w, cx + self.roi_half_w)
        roi = depth[y0:y1, x0:x1]

        # 3. The car's own LiDAR housing. Computed HERE, before the too-close
        #    check rather than after the validity mask where it used to sit,
        #    because BOTH now need it: it is a permanent close-range fixture in
        #    the ZED's field of view, so counting it would dominate the low
        #    percentile (its original reason) AND hold the too-close
        #    short-circuit on forever (the new one). Same mask, one
        #    computation, two consumers.
        housing = self._lidar_exclusion_roi_mask((h, w), (y0, y1, x0, x1))

        # 3a. Scene mask, at scene_mask_rate_hz: the whole frame, no ROI.
        if self.scene_mask_enabled and now >= self._next_scene_mask:
            self._next_scene_mask = now + self.scene_mask_period
            try:
                self._scene_mask_tick(depth, msg.header, now)
            except Exception as exc:   # noqa: BLE001 - never kill the depth path
                self.get_logger().error(f'scene mask tick failed: {exc}',
                                        throttle_duration_sec=5.0)

        # 3b. Scene labels, at scene_rate_hz (module docstring, SCENE LABELS).
        if self.scene_enabled and now >= self._next_scene:
            self._next_scene = now + self.scene_period
            try:
                self._label_scene(depth, msg.header, conf, (y0, y1, x0, x1))
            except Exception as exc:  # noqa: BLE001 - labels must never stop the guard
                self.get_logger().error(
                    f'scene labelling failed: {exc}', throttle_duration_sec=5.0)

        # 4. TOO CLOSE -- the safety short-circuit, deliberately ahead of the
        #    validity mask and of every filter below it.
        #
        #    -Inf is NOT missing data: it is the ZED reporting something closer
        #    than it can measure. Read the module docstring's own "-Inf AND
        #    NaN/+Inf ARE NOT THE SAME THING" section before changing anything
        #    here -- it spells out the three independent ways the machinery
        #    below turns a dropped -Inf into evidence of CLEARANCE, which is
        #    why this returns instead of feeding a value forward.
        #
        #    Object-masked pixels are deliberately NOT excluded from this
        #    count: an object at too-close range is precisely the case being
        #    caught, so masking it out would defeat the check entirely. The
        #    housing is excluded, being the car's own hardware.
        considered = int(roi.size) - int(np.count_nonzero(housing))
        if considered > 0:
            n_too_close = int(np.count_nonzero(np.isneginf(roi) & ~housing))
            if (n_too_close / considered) >= self.too_close_min_fraction:
                too_close_front = self._publish_too_close(
                    n_too_close, considered, msg.header.frame_id)
                self._publish_wall_debug(
                    msg.header, (h, w), (y0, y1, x0, x1), roi, housing, conf, {
                        'too_close': True, 'n_valid': 0, 'raw_distance': None,
                        'front_distance': -1.0, 'front_wall': False,
                        'object_distance': None, 'raw_clearance': None,
                        'front_clearance': too_close_front,
                        'front_blocked': True})
                return

        # 5. Valid == finite, positive, and not covered by an object.
        #
        #    This is the NaN/+Inf path and its behaviour is UNCHANGED: both
        #    still fail np.isfinite and are dropped as "unknown, no stereo
        #    match", exactly like an object-masked pixel. -Inf also still fails
        #    here, which is now harmless rather than dangerous -- any frame
        #    carrying a meaningful amount of it already returned above, and a
        #    residual speckle -Inf below the threshold genuinely is noise and
        #    genuinely should be dropped.
        valid = np.isfinite(roi) & (roi > 0)
        if conf is not None:
            bg_weight = 1.0 - conf[y0:y1, x0:x1]
            valid &= (bg_weight >= self.bg_weight_threshold)

        # 6. Cut the housing out of the statistics -- see step 3.
        valid &= ~housing

        # 6b. Cut the floor out too (ground_plane.py): the floor below the
        #     horizon is otherwise the nearest consistent surface in the ROI.
        #     With scene labels on and fresh, WALL and OTHER only instead.
        self._last_ground = None
        scene_valid = None
        if self.scene_enabled:
            scene_valid = self._scene_roi_valid(
                perception_debug.stamp_to_sec(msg.header.stamp), (y0, y1, x0, x1), (h, w))
        if scene_valid is not None:
            valid &= scene_valid
        else:
            ground = self._ground_roi_mask(depth, msg.header.frame_id, (y0, y1, x0, x1))
            if ground is not None:
                valid &= ~ground

        bg_depths = roi[valid]
        n_valid = int(bg_depths.size)

        raw_distance = (self._robust_background_distance(bg_depths)
                        if n_valid >= self.min_bg_pixels else None)

        # 7. The WALL latch is fed the pixel COUNT, not the distance -- see the
        #    module docstring. Note this is deliberately NOT gated on
        #    min_bg_pixels: a count below that floor is a real, meaningful
        #    observation of "the ROI is empty", and is exactly the evidence
        #    that should drive the latch toward False.
        front_wall = self.wall_latch.update(n_valid)

        # 8. Background distance: EMA, holding on a None frame. Still from the
        #    lens, like everything above.
        self.distance_filter.update(raw_distance)
        smoothed_distance = self.distance_filter.value

        # FROM HERE ON, WHAT IS PUBLISHED IS FROM THE FRONT BUMPER (module
        # docstring). No camera transform yet: no reference, no reading.
        offset = self._front_offset(msg.header.frame_id)
        if offset is None:
            self.get_logger().warn(
                f'no {self.base_frame} <- "{msg.header.frame_id}" transform on '
                '/tf_static yet: front_distance cannot be referenced to the front '
                'bumper, publishing -1 (no reading).', throttle_duration_sec=5.0)
        front_distance = None if offset is None else from_front(smoothed_distance, offset)

        # 9-10. Clearance: the nearer of the background and the nearest
        #      in-corridor obstacle, BOTH from the bumper (the obstacle term was
        #      from base_link, the background from the lens), smoothed, then
        #      latched on the SMOOTHED value (see module docstring).
        object_distance = self._nearest_corridor_obstacle(now)
        object_front = (None if object_distance is None
                        else max(object_distance - self.front_x, MIN_FRONT_DISTANCE_M))
        candidates = [d for d in (front_distance, object_front) if d is not None]
        raw_clearance = min(candidates) if candidates else None
        self.clearance_filter.update(raw_clearance)
        smoothed_clearance = self.clearance_filter.value
        front_blocked = self.blocked_latch.update(smoothed_clearance)

        self.front_distance_pub.publish(
            Float32(data=self._publishable(front_distance)))
        self.front_wall_pub.publish(Bool(data=bool(front_wall)))
        self.front_clearance_pub.publish(
            Float32(data=self._publishable(smoothed_clearance)))
        self.front_blocked_pub.publish(Bool(data=bool(front_blocked)))

        if self.publish_debug_raw:
            self.front_distance_raw_pub.publish(
                Float32(data=self._publishable(raw_distance)))
            self.bg_pixel_count_pub.publish(Float32(data=float(n_valid)))

        self._publish_wall_debug(
            msg.header, (h, w), (y0, y1, x0, x1), roi, housing, conf, {
                'too_close': False, 'n_valid': n_valid, 'raw_distance': raw_distance,
                'front_distance': self._publishable(front_distance),
                'front_wall': front_wall, 'object_distance': object_front,
                'raw_clearance': raw_clearance,
                'front_clearance': self._publishable(smoothed_clearance),
                'front_blocked': front_blocked})

        self._log_transitions(front_wall, front_blocked, n_valid,
                              front_distance, smoothed_clearance)

    def _publish_too_close(self, n_too_close, considered, camera_frame=''):
        """Publish the too-close override for this frame and nothing else.

        WHAT IS BYPASSED, and why each one has to be. Every stage below the
        validity mask independently converts a dropped -Inf into evidence of
        clearance (see the module docstring), so routing the override through
        any of them would re-introduce the bug this exists to fix:

          * the trimmed percentile band -- its input is the surviving pixels,
            all of which are FURTHER than the object that produced the -Inf;
          * both EMAs -- `distance_ema_alpha` would ramp toward the floor over
            several frames while the object is already against the lens, and
            on a frame with too few valid pixels `EmaFilter.update(None)` would
            instead hold the last far value outright;
          * both latches -- their dwell counters would delay the flip by
            `*_min_dwell_frames`, and `wall_latch` would be reading a pixel
            count that is COLLAPSING toward "no wall" at exactly the moment a
            surface is closest.

        The filters are still stepped with None, which is their documented
        "no measurement this frame" path: it resets both dwell counters (so a
        partly-accumulated flip cannot survive across a too-close episode) and
        holds the EMAs without seeding them. Their held values are simply not
        what gets published this frame. This is deliberately NOT a retune of
        any of them -- the filter chain is adequate for real readings, and the
        fault was always upstream of it.

        WHAT IS PUBLISHED, one message per topic, preserving this node's
        "every depth frame produces a message on every topic" invariant:

          front_clearance  the floor (`too_close_clearance_m`) -- an upper
                           bound on a distance known only by its ceiling --
                           re-referenced to the front bumper like every other
                           published distance, never below
                           MIN_FRONT_DISTANCE_M (and exactly that when the
                           camera's pose is not known yet).
          front_blocked    True, immediately.
          front_distance   -1.0, the node's own "no reading" sentinel. NOT the
                           floor: front_distance means BACKGROUND/wall, and
                           with something inside minimum range the background
                           is genuinely unobserved. Publishing the floor here
                           would report a wall at 0.2 m that nobody measured;
                           publishing the held EMA value would be the stale
                           far reading this whole change exists to prevent.
          front_wall       False. The question it answers is "is a surface
                           filling the ROI at a measurable distance", and the
                           answer while something is inside minimum range is
                           no. Published directly rather than through the
                           latch, so no dwell delays it.

        Returns the front_clearance it published (for the debug payload).
        """
        self.distance_filter.update(None)
        self.clearance_filter.update(None)
        self.wall_latch.update(None)
        self.blocked_latch.update(None)

        offset = self._front_offset(camera_frame)
        too_close_front = (MIN_FRONT_DISTANCE_M if offset is None
                           else from_front(self.too_close_clearance_m, offset))
        self.front_distance_pub.publish(Float32(data=-1.0))
        self.front_wall_pub.publish(Bool(data=False))
        self.front_clearance_pub.publish(Float32(data=float(too_close_front)))
        self.front_blocked_pub.publish(Bool(data=True))

        if self.publish_debug_raw:
            # No background reading was computed at all this frame, and no
            # background pixels were counted -- the honest values, not zeros
            # standing in for numbers that were never produced.
            self.front_distance_raw_pub.publish(Float32(data=-1.0))
            self.bg_pixel_count_pub.publish(Float32(data=0.0))

        # Throttled WARN rather than the transition-only info the latches get:
        # this is a safety event and it should be visible while it persists,
        # but this node runs at the depth stream's rate and an unthrottled
        # line would bury the console.
        self.get_logger().warn(
            f'TOO CLOSE | {n_too_close}/{considered} ROI px below the ZED\'s '
            f'minimum measurable range ('
            f'{100.0 * n_too_close / considered:.1f}% >= '
            f'{100.0 * self.too_close_min_fraction:.1f}%): publishing '
            f'front_clearance={too_close_front:.2f} m (from the front bumper), '
            'front_blocked=True, bypassing all filters.',
            throttle_duration_sec=1.0)

        self._log_transitions(False, True, 0, None, too_close_front)
        return too_close_front

    def _publish_wall_debug(self, header, shape_hw, bounds, roi, housing, conf, values):
        """
        Publish this frame's wall/clearance intermediates on /perception/debug/front_wall.

        Pixel counts are recomputed here from the same ROI, housing and
        confidence arrays depth_callback used, so the published path above
        is untouched. Wrapped whole: a debug payload must never cost the
        wall guard a frame.
        """
        if self.wall_debug_pub is None:
            return
        try:
            y0, y1, x0, x1 = bounds
            finite = np.isfinite(roi) & (roi > 0)
            if conf is not None:
                covered = (1.0 - conf[y0:y1, x0:x1]) < self.bg_weight_threshold
            else:
                covered = np.zeros(roi.shape, dtype=bool)
            counts = {
                'roi': int(roi.size), 'housing': int(np.count_nonzero(housing)),
                'too_close': int(np.count_nonzero(np.isneginf(roi) & ~housing)),
                'invalid': int(np.count_nonzero(~finite & ~housing)),
                'object': int(np.count_nonzero(finite & ~housing & covered)),
                'bg': int(values['n_valid']), 'min_bg': int(self.min_bg_pixels)}
            objects = []
            det_stamp = None
            det_msg = self._latest_detections
            if det_msg is not None:
                det_stamp = perception_debug.stamp_to_sec(det_msg.header.stamp)
                for det in det_msg.detections:
                    cx = float(det.bbox.center.position.x)
                    cy = float(det.bbox.center.position.y)
                    half_w = float(det.bbox.size_x) / 2.0
                    half_h = float(det.bbox.size_y) / 2.0
                    n = perception_debug.box_overlap_px(
                        (cx - half_w, cy - half_h, cx + half_w, cy + half_h),
                        (x0, y0, x1, y1))
                    if n > 0 and det.results:
                        hyp = det.results[0].hypothesis
                        objects.append((hyp.class_id, hyp.score, n))
            lidar = None
            fit = self._latest_lidar_wall
            if fit is not None:
                lidar = perception_debug.lidar_fit_summary(
                    fit.valid, fit.reason, fit.distance, fit.inlier_count,
                    fit.valid_returns, perception_debug.stamp_to_sec(fit.header.stamp),
                    self._now_seconds())
            payload = perception_debug.build_wall_debug(
                perception_debug.stamp_to_sec(header.stamp), shape_hw, (x0, y0, x1, y1), counts,
                {'raw_m': values['raw_distance'],
                 'front_distance_m': values['front_distance'],
                 'front_wall': values['front_wall']},
                {'obstacle_m': values['object_distance'],
                 'corridor_half_width_m': self.corridor_half_width_m,
                 'raw_m': values['raw_clearance'],
                 'front_clearance_m': values['front_clearance'],
                 'front_blocked': values['front_blocked']},
                objects, self._latest_object_source, det_stamp, lidar, values['too_close'],
                frame_id=header.frame_id, stereo_origin=_WALL_DEBUG_STEREO_ORIGIN,
                obstacle_origin=_WALL_DEBUG_OBSTACLE_ORIGIN,
                stereo_raw_origin=_WALL_DEBUG_STEREO_RAW_ORIGIN,
                front_offset_m=self._front_offset(header.frame_id))
            payload['ground'] = self._last_ground
            # How much older the object mask is than the depth frame it was
            # applied to [ms]: the latest mask is used on every depth frame
            # (module docstring, OBJECT-EXCLUSION SOURCE), so this is the lag.
            mask_msg = self._latest_mask_msg
            payload['mask_age_ms'] = (None if mask_msg is None else round(
                (perception_debug.stamp_to_sec(header.stamp)
                 - perception_debug.stamp_to_sec(mask_msg.header.stamp)) * 1e3, 1))
            self.wall_debug_pub.publish(String(data=perception_debug.to_json(payload)))
        except Exception as exc:  # noqa: BLE001 - debug only, never stop the guard
            self.get_logger().error(
                f'front_wall debug payload failed: {exc}', throttle_duration_sec=5.0)

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
