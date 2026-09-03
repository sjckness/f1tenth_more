#!/usr/bin/env python3
"""2D-to-3D detection fusion node for the F1TENTH perception stack.

Synchronizes YOLO 2D detections (yolo_detector_node, vision_msgs/Detection2DArray,
pixel coords) with the ZED depth image (sensor_msgs/Image, 32FC1 meters) and its
camera_info, back-projects each 2D box into a 3D pose + size using the pinhole
model, and publishes:
  * vision_msgs/Detection3DArray on `detections_3d_topic` (default /camera/detections_3d)
  * visualization_msgs/MarkerArray on `markers_topic` (default /camera/detection_markers)
    -- CUBE markers, color-coded per class, short lifetime so stale boxes vanish
       in Foxglove/RViz if detections stop. Each box marker (id i) is paired with a
       TEXT_VIEW_FACING label marker (id i + LABEL_ID_OFFSET) showing
       "class_name (confidence)", floated above the box, same lifetime -- both
       vanish together.

Back-projection math happens in the depth image's own frame (ZED left camera
*optical* frame, e.g. `zed2_left_camera_optical_frame`), since that's the
convention camera_info/pinhole intrinsics use. Before publishing, each pose is
transformed via tf2 into `output_frame` (default `zed2_left_camera_frame`, the
ZED's non-optical camera frame) so downstream consumers (Foxglove 3D panel,
TF-tree-aligned tooling) get a REP-103-style axis convention (x-forward,
y-left, z-up) instead of the optical one (z-forward, x-right, y-down). The
optical -> camera_frame transform is a fixed joint already broadcast by the
ZED wrapper's own robot_state_publisher -- this node only looks it up, it does
not (re)publish it.

Detections with hypothesis.score below `confidence_threshold` are dropped
before either output is built.

Depth-axis (Z) extent of each box is NOT observable from a single monocular
depth view, so `default_depth_extent` is a configurable placeholder, not a
measurement -- see the comment at its use site below.

Mask-based depth sampling (use_mask_depth, default false -- config-selectable,
does NOT replace the box-region method below, only adds an alternative to it):
when true, this node additionally subscribes to yolo_detector_node's
`masks_topic` (a mono8 per-pixel instance-index label image, published ONLY
when yolo_detector_node's loaded model is a segment-task one -- see that
node's own module docstring) and 3-way-syncs it alongside detections_topic and
depth_topic. For each detection, `_mask_median_depth()` intersects that
detection's mask footprint (pixels whose label == its 1-based index) with
valid depth (finite, > 0 -- same convention `_median_depth()` already uses)
and takes the median of what's left -- robust to background/edge contamination
a box region would include, since the mask is the actual object silhouette,
not its bounding rectangle. Falls back to `_median_depth()` (the box-region
method, unchanged) for any individual detection whose mask has zero valid-
depth pixels left after that filtering (fully occluded, out of stereo range,
or -- with use_mask_depth true but a detect-task model upstream -- no mask
data published at all this frame), logging a throttled warning each time
rather than silently dropping the detection or crashing. This keeps
use_mask_depth true safe to leave on even against a detect-task model: every
detection just always takes the fallback path, identical output to
use_mask_depth false.

The mask image and the depth image are expected at the same resolution (both
are ZED-published images under the same `general.pub_resolution`/
`pub_downscale_factor` config, see zed2_perception.yaml) but this is NOT
assumed blindly: `_mask_median_depth()` checks shapes every frame and, on a
mismatch, nearest-neighbor-resizes the mask to the depth image's own
resolution (cv2.INTER_NEAREST -- the mask is a discrete instance-index label
map, not a continuous image; any other interpolation mode would blend index
values across instance boundaries into meaningless intermediate numbers) and
logs a throttled warning so a real, persistent mismatch stays visible rather
than silently degrading accuracy every frame.
"""

import hashlib

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers Pose transform support
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_perception.cpu_affinity import (
    apply_nice,
    declare_nice_param,
)

# Fixed offset applied to a box marker's id to get its paired text-label marker's id,
# so both live in the same MarkerArray/topic without id collisions.
LABEL_ID_OFFSET = 10000


class Detection3DNode(Node):
    def __init__(self, **kwargs):
        # **kwargs: lets tests pass parameter_overrides=[...] straight through
        # to rclpy.Node, same convention as every other node in this workspace
        # that has its own test file (e.g. f1tenth_costmap's
        # costmap_boundary_node.py/semantic_layer_node.py) -- no effect on
        # normal launch-file construction, which never passes kwargs here.
        super().__init__('detection_3d_node', **kwargs)

        # nice: see cpu_affinity.py. The perception-latency audit measured
        # this node at 75-82% CPU with no pinning (less severe than
        # yolo_detector_node's contention signature, but still substantial).
        # CPU AFFINITY is now a `taskset -c` launch prefix, not an in-process
        # self-pin -- see detection.launch.py's own detection_3d_cpu_affinity
        # comment (thread-pinning-leak fix, Step 6 reintroduction
        # investigation: the old self-pin left 32 of this node's 33 threads
        # fully unpinned, confirmed live executing on reserved cores).
        declare_nice_param(self)

        # ---- parameters ------------------------------------------------
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections').value)
        self.depth_topic = str(
            self.declare_parameter(
                'depth_topic', '/zed2/zed_node/depth/depth_registered').value)
        self.depth_info_topic = str(
            self.declare_parameter(
                'depth_info_topic', '/zed2/zed_node/depth/camera_info').value)
        self.detections_3d_topic = str(
            self.declare_parameter('detections_3d_topic', '/camera/detections_3d').value)
        self.markers_topic = str(
            self.declare_parameter('markers_topic', '/camera/detection_markers').value)
        self.sync_slop = float(
            self.declare_parameter('sync_slop', 0.1).value)
        # message_filters keeps this many of the *faster* topic's (depth, ~25Hz)
        # messages around while waiting for a match on the slower one
        # (detections -- header stamp is copied from the input frame, but the
        # message itself isn't published until inference finishes, ~1s later
        # on this hardware). The default queue_size=10 in message_filters only
        # covers ~0.4s of depth history at 25Hz, which is shorter than that
        # inference latency, so every detection's matching depth frame had
        # already been evicted before the detection arrived -- nothing ever
        # matched. 60 covers ~2.4s, comfortably above observed latency.
        self.sync_queue_size = int(
            self.declare_parameter('sync_queue_size', 60).value)
        self.default_depth_extent = float(
            self.declare_parameter('default_depth_extent', 0.3).value)
        self.marker_lifetime = float(
            self.declare_parameter('marker_lifetime', 0.2).value)
        self.center_fraction = float(
            self.declare_parameter('center_fraction', 0.5).value)
        # Non-optical ZED camera frame (REP-103 axes) that published poses are
        # transformed into. Confirmed against zed_macro.urdf.xacro: with
        # camera_name='zed2' the wrapper's robot_state_publisher broadcasts a
        # fixed zed2_left_camera_frame -> zed2_left_camera_optical_frame joint,
        # so this is the parent of the optical frame the depth image is in.
        self.output_frame = str(
            self.declare_parameter('output_frame', 'zed2_left_camera_frame').value)
        # Detections with hypothesis.score below this are dropped before
        # either output (Detection3DArray, MarkerArray) is built.
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.3).value)
        # The ZED2i's real minimum stereo sensing distance is ~0.2-0.3 m;
        # anything reporting a median depth closer than this is not a
        # trustworthy reading (stereo matching breaks down that close, not a
        # genuinely close obstacle) and is dropped in _synced_callback below,
        # same as a None (unreadable) depth already is.
        self.min_valid_depth = float(
            self.declare_parameter('min_valid_depth', 0.2).value)
        # Text label (class + confidence) floated above each box marker.
        self.label_scale = float(
            self.declare_parameter('label_scale', 0.15).value)
        self.label_z_offset = float(
            self.declare_parameter('label_z_offset', 0.15).value)
        # Mask-based depth sampling -- see module docstring. false: zero
        # behavior change from before this param existed (2-way sync,
        # _median_depth only). true: additionally subscribes to masks_topic
        # and 3-way-syncs it; per-detection depth sampling prefers the mask,
        # falling back to _median_depth per-detection (not node-wide) when a
        # detection's mask has no valid-depth pixels.
        self.use_mask_depth = bool(
            self.declare_parameter('use_mask_depth', False).value)

        # ---- position uncertainty (confidence- and range-scaled) ----------
        # The 2026-09-01 mission analysis measured raw frame-to-frame detection
        # displacement in THIS node's own output frame (ego-motion and
        # localisation both removed) and found it strongly confidence-
        # dependent:
        #
        #   confidence >= 0.75 : median 0.033-0.121 m
        #   confidence 0.5-0.75: median 0.001-0.221 m
        #   confidence <  0.50 : median 0.143-0.249 m   (2-7x the high bucket)
        #
        # Downstream consumers previously had no way to know this -- every
        # detection arrived equally precise, so semantic_layer_node's tracker
        # blended a 25 cm-noisy low-confidence detection with exactly the same
        # weight as a 3 cm-noisy high-confidence one. Publishing a real
        # covariance lets the tracker (and anything else) weight accordingly;
        # see semantic_layer_node's confidence_weighted_alpha.
        #
        # Model, deliberately simple and stated rather than tuned-in-secret:
        #
        #   sigma_xy = (base + range_coeff * z) * conf_scale(score)
        #   conf_scale = low_conf_sigma_scale ... 1.0, linear in score across
        #                [confidence_threshold, 1.0]
        #
        # base/range_coeff give the standard stereo behaviour that depth noise
        # grows with distance; conf_scale carries the measured confidence
        # dependence above. Defaults are seeded FROM the measured medians
        # (0.05 m base is roughly the high-confidence median at close range,
        # 4x scaling reproduces the observed low-confidence spread) and are
        # explicitly first-pass values to retune once this feed has been
        # recorded live -- not values with independent physical provenance.
        self.position_sigma_base_m = float(
            self.declare_parameter('position_sigma_base_m', 0.05).value)
        self.position_sigma_range_coeff = float(
            self.declare_parameter('position_sigma_range_coeff', 0.02).value)
        self.low_conf_sigma_scale = float(
            self.declare_parameter('low_conf_sigma_scale', 4.0).value)
        # Depth-axis extent is a placeholder, not a measurement (see
        # _build_detection3d), so its variance is deliberately large rather
        # than pretending the z estimate is as good as x/y.
        self.position_sigma_z_scale = float(
            self.declare_parameter('position_sigma_z_scale', 3.0).value)

        self.masks_topic = str(
            self.declare_parameter('masks_topic', '/camera/detection_masks').value)

        # ---- intrinsics (cached from the latest camera_info) -----------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.bridge = CvBridge()

        # ---- tf2 (optical frame -> output_frame) ------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS interfaces ---------------------------------------------
        self.det3d_pub = self.create_publisher(
            Detection3DArray, self.detections_3d_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.markers_topic, 10)

        self.info_sub = self.create_subscription(
            CameraInfo, self.depth_info_topic, self._camera_info_callback, 10)

        self.det_sub = message_filters.Subscriber(
            self, Detection2DArray, self.detections_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        # use_mask_depth gates a 3rd synced subscriber, same "Python-level
        # branching before the synchronizer is built" pattern detection.
        # launch.py's own is_zed gating uses -- see module docstring. false
        # (default): sync_subs stays exactly [det_sub, depth_sub], identical
        # to before this param existed.
        sync_subs = [self.det_sub, self.depth_sub]
        self.mask_sub = None
        if self.use_mask_depth:
            self.mask_sub = message_filters.Subscriber(self, Image, self.masks_topic)
            sync_subs.append(self.mask_sub)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            sync_subs, queue_size=self.sync_queue_size, slop=self.sync_slop)
        self.sync.registerCallback(self._synced_callback)

        # ---- 2D->3D yield accounting -------------------------------------
        # The 2026-09-01 mission analysis found only 55-65% of /camera/
        # detections frames produced a /camera/detections_3d message at all
        # (2D ~13 Hz in, 3D ~8 Hz out), with output gaps up to 6.16 s. That
        # yield loss is the dominant driver of marker flicker and track churn
        # downstream, but nothing in this node measured it, so the cause could
        # only be guessed at from message counts.
        #
        # It cannot be a per-detection filter: _synced_callback publishes
        # unconditionally at its end, even for an empty array. So every lost
        # frame is either (a) the synchronizer never firing for that 2D
        # message, or (b) one of this callback's early returns. These counters
        # separate those two, and separate the early-return reasons from each
        # other, so tomorrow's run answers the question directly instead of
        # leaving it to another offline reconstruction.
        #
        # `_raw_det_sub` is a second, plain subscription to the SAME topic the
        # synchronizer already wraps -- message_filters gives no visibility
        # into what it dropped, so counting arrivals independently is the only
        # way to compute a yield. Detection2DArray is small; this is a counter
        # increment per message, not a second copy of the pipeline.
        self._stats = {
            'det2d_in': 0, 'depth_in': 0, 'mask_in': 0, 'synced': 0, 'published': 0,
            'drop_no_camera_info': 0, 'drop_cv_bridge': 0, 'drop_tf2': 0,
            'det_drop_low_conf': 0, 'det_drop_bad_depth': 0,
            'det_drop_mask_fallback': 0, 'det_published': 0,
        }
        self._raw_det_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._count_det2d, 10)
        # Per-INPUT arrival counts. A 3-way ApproximateTimeSynchronizer fires
        # only when all three inputs line up, so "sync dropped it" is not yet
        # an actionable answer -- the actionable question is WHICH input is the
        # limiting one. These count arrivals on the synchronizer's OWN
        # subscriptions (message_filters.Subscriber.registerCallback reuses the
        # existing subscription) rather than opening second ones: a duplicate
        # subscription to the depth or mask image would deserialize a full
        # float32 image per frame just to increment a counter, which is enough
        # extra load on this box to perturb the very rates being measured.
        self.depth_sub.registerCallback(self._count_depth)
        if self.mask_sub is not None:
            self.mask_sub.registerCallback(self._count_mask)
        self.yield_report_period_sec = float(
            self.declare_parameter('yield_report_period_sec', 10.0).value)
        if self.yield_report_period_sec > 0.0:
            self._yield_timer = self.create_timer(
                self.yield_report_period_sec, self._report_yield)

        apply_nice(self)

        self.get_logger().info(
            f'detection_3d_node up: syncing "{self.detections_topic}" + '
            f'"{self.depth_topic}"' +
            (f' + "{self.masks_topic}"' if self.use_mask_depth else '') +
            f' (slop={self.sync_slop}s), intrinsics from '
            f'"{self.depth_info_topic}", publishing 3D detections on '
            f'"{self.detections_3d_topic}" and markers on "{self.markers_topic}" '
            f'in frame "{self.output_frame}", confidence_threshold='
            f'{self.confidence_threshold}, use_mask_depth={self.use_mask_depth}')

    def _count_det2d(self, msg):
        self._stats['det2d_in'] += 1

    def _count_depth(self, msg):
        self._stats['depth_in'] += 1

    def _count_mask(self, msg):
        self._stats['mask_in'] += 1

    def _report_yield(self):
        st = self._stats
        n_in = st['det2d_in']
        if n_in == 0:
            self.get_logger().warn(
                f'YIELD | no /camera/detections received in the last '
                f'{self.yield_report_period_sec:.0f}s -- upstream detector down?')
            return
        sync_pct = 100.0 * st['synced'] / n_in
        pub_pct = 100.0 * st['published'] / n_in
        # The synchronizer can only fire as often as its RAREST input arrives,
        # so this ceiling is what separates "sync policy is too strict" from
        # "an upstream input simply isn't being published often enough" -- two
        # causes with completely different fixes that a single sync-drop count
        # cannot tell apart.
        inputs = [('det2d', n_in), ('depth', st['depth_in'])]
        if self.use_mask_depth:
            inputs.append(('mask', st['mask_in']))
        limiter, limit_n = min(inputs, key=lambda kv: kv[1])
        ceiling = 100.0 * limit_n / n_in
        self.get_logger().info(
            f'YIELD | det2d_in={n_in} depth_in={st["depth_in"]} '
            + (f'mask_in={st["mask_in"]} ' if self.use_mask_depth else '')
            + f'-> synced={st["synced"]} ({sync_pct:.0f}%) '
            f'published={st["published"]} ({pub_pct:.0f}%) | '
            f'rarest input={limiter} n={limit_n} (ceiling {ceiling:.0f}% of det2d) | '
            f'frame drops: sync={n_in - st["synced"]} '
            f'no_camera_info={st["drop_no_camera_info"]} '
            f'cv_bridge={st["drop_cv_bridge"]} tf2={st["drop_tf2"]} | '
            f'detection drops: low_conf={st["det_drop_low_conf"]} '
            f'bad_depth={st["det_drop_bad_depth"]} '
            f'mask_fallback={st["det_drop_mask_fallback"]} '
            f'published={st["det_published"]} '
            f'(slop={self.sync_slop}s queue={self.sync_queue_size} '
            f'use_mask_depth={self.use_mask_depth})')
        for k in st:
            st[k] = 0

    def _camera_info_callback(self, msg: CameraInfo):
        # Intrinsics are effectively static per session; cache and move on.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def _synced_callback(self, det_msg, depth_msg, mask_msg=None):
        self._stats['synced'] += 1
        # mask_msg is only ever non-None when use_mask_depth is true (only
        # then is mask_sub registered with the synchronizer at all -- see
        # __init__) -- message_filters calls this with exactly as many
        # positional args as subscribers were registered, so the default
        # covers the use_mask_depth=false (2-way sync) case unchanged.
        if self.fx is None:
            self.get_logger().warn(
                'No camera_info received yet on '
                f'"{self.depth_info_topic}" -- skipping frame', throttle_duration_sec=5.0)
            self._stats['drop_no_camera_info'] += 1
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge depth conversion failed: {exc}')
            self._stats['drop_cv_bridge'] += 1
            return

        mask_img = None
        if mask_msg is not None:
            if mask_msg.height == 0 or mask_msg.width == 0:
                # ZERO-SIZE MASK = "this frame has no instance masks", the
                # deliberate marker yolo_detector_node._publish_empty_mask()
                # emits so a zero-detection frame still satisfies this node's
                # 3-way synchronizer and still produces an (empty)
                # Detection3DArray -- semantic_layer_node needs one message per
                # frame, including empty ones, to age its tracks out. See that
                # helper's docstring for the full chain.
                #
                # Checked explicitly rather than left to the conversion below
                # only because it states the intent: cv_bridge happens to
                # return None (NOT raise) for a 0x0 image, so the fallback
                # would land in the same place by accident. Relying on that
                # would make a deliberate empty-frame marker indistinguishable
                # from a genuinely malformed mask.
                mask_img = None
            else:
                try:
                    mask_img = self.bridge.imgmsg_to_cv2(
                        mask_msg, desired_encoding='mono8')
                except CvBridgeError as exc:
                    self.get_logger().error(
                        f'cv_bridge mask conversion failed: {exc} -- every '
                        'detection this frame falls back to box-region depth '
                        'sampling.')
                    mask_img = None
                if mask_img is None:
                    # Verified on this box: imgmsg_to_cv2(..., desired_encoding)
                    # can return None WITHOUT raising, so a `None` result is
                    # not implied by the absence of an exception and has to be
                    # handled on its own. Falls through to box-region sampling,
                    # same as a raised conversion error.
                    self.get_logger().error(
                        'cv_bridge returned no mask image for a '
                        f'{mask_msg.height}x{mask_msg.width} '
                        f'"{mask_msg.encoding}" mask -- every detection this '
                        'frame falls back to box-region depth sampling.',
                        throttle_duration_sec=5.0)

        h, w = depth.shape[:2]

        # This is a static fixed joint (see zed_macro.urdf.xacro), so look up
        # the latest available transform (Time()) rather than the depth
        # frame's exact stamp -- avoids extrapolation failures if /tf_static
        # hasn't been received for that exact timestamp yet.
        try:
            optical_to_output = self.tf_buffer.lookup_transform(
                self.output_frame, depth_msg.header.frame_id,
                rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{depth_msg.header.frame_id}" -> '
                f'"{self.output_frame}" failed: {exc} -- skipping frame',
                throttle_duration_sec=5.0)
            self._stats['drop_tf2'] += 1
            return

        out_header = Header(stamp=depth_msg.header.stamp, frame_id=self.output_frame)

        det3d_array = Detection3DArray()
        det3d_array.header = out_header

        markers = MarkerArray()

        for i, det in enumerate(det_msg.detections):
            class_id = det.results[0].hypothesis.class_id if det.results else ''
            score = det.results[0].hypothesis.score if det.results else 0.0
            if score < self.confidence_threshold:
                self._stats['det_drop_low_conf'] += 1
                continue

            cx_px = det.bbox.center.position.x
            cy_px = det.bbox.center.position.y
            box_w = det.bbox.size_x
            box_h = det.bbox.size_y

            # `i` is det_msg.detections' own index, which is exactly the
            # index yolo_detector_node used to build the mask label image
            # (label value i+1 -- see that node's masks_topic docstring), so
            # no separate lookup/renumbering is needed here.
            if self.use_mask_depth and mask_img is not None:
                z = self._mask_median_depth(
                    depth, mask_img, i, cx_px, cy_px, box_w, box_h, w, h)
            else:
                z = self._median_depth(depth, cx_px, cy_px, box_w, box_h, w, h)
            if z is None or z < self.min_valid_depth:
                self._stats['det_drop_bad_depth'] += 1
                continue

            x3d = (cx_px - self.cx) * z / self.fx
            y3d = (cy_px - self.cy) * z / self.fy
            width_3d = box_w * z / self.fx
            height_3d = box_h * z / self.fy

            # Position + orientation, transformed from the optical frame into
            # output_frame. Orientation starts identity (axis-aligned in the
            # optical frame); after the transform it carries the optical ->
            # camera_frame rotation, so scale.x/y/z (still the box's optical
            # width/height/depth-extent) render correctly oriented in
            # output_frame without needing to permute which axis is which.
            optical_pose = Pose()
            optical_pose.position.x = float(x3d)
            optical_pose.position.y = float(y3d)
            optical_pose.position.z = float(z)
            optical_pose.orientation.w = 1.0
            out_pose = tf2_geometry_msgs.do_transform_pose(
                optical_pose, optical_to_output)

            det3d = self._build_detection3d(
                header=out_header, pose=out_pose,
                width=width_3d, height=height_3d,
                class_id=class_id, score=score, z=z)
            det3d_array.detections.append(det3d)

            markers.markers.append(self._build_marker(
                header=out_header, marker_id=i, pose=out_pose,
                width=width_3d, height=height_3d, class_id=class_id))
            markers.markers.append(self._build_label_marker(
                header=out_header, marker_id=i, pose=out_pose,
                height=height_3d, class_id=class_id, score=score))

        self._stats['published'] += 1
        self._stats['det_published'] += len(det3d_array.detections)
        self.det3d_pub.publish(det3d_array)
        self.marker_pub.publish(markers)

    def _median_depth(self, depth, cx_px, cy_px, box_w, box_h, img_w, img_h):
        # Sample only the center `center_fraction` of the box (in each axis) to
        # avoid depth bleed from background pixels near the box edges.
        half_w = (box_w * self.center_fraction) / 2.0
        half_h = (box_h * self.center_fraction) / 2.0

        x1 = int(np.clip(cx_px - half_w, 0, img_w - 1))
        x2 = int(np.clip(cx_px + half_w, 0, img_w - 1))
        y1 = int(np.clip(cy_px - half_h, 0, img_h - 1))
        y2 = int(np.clip(cy_px + half_h, 0, img_h - 1))

        if x2 <= x1 or y2 <= y1:
            return None

        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            return None

        return float(np.median(valid))

    def _mask_median_depth(
            self, depth, mask_img, det_index, cx_px, cy_px, box_w, box_h, img_w, img_h):
        """Mask-based counterpart to `_median_depth()`: median depth over the
        pixels where `mask_img` equals this detection's label (`det_index +
        1` -- see yolo_detector_node.py's masks_topic docstring for why),
        restricted to valid depth (finite, > 0 -- identical filter to
        `_median_depth()`). Falls back to `_median_depth()` (the box-region
        method, completely unchanged) for THIS ONE detection if that leaves
        zero pixels -- fully occluded, out of stereo range, or (use_mask_depth
        true against a detect-task upstream model) no mask data published at
        all this frame -- logging a throttled warning rather than dropping
        the detection or crashing. See module docstring's "Mask-based depth
        sampling" paragraph for the resize-on-mismatch behavior below.
        """
        if mask_img.shape[:2] != depth.shape[:2]:
            orig_shape = mask_img.shape[:2]
            mask_img = cv2.resize(
                mask_img, (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST)
            self.get_logger().warn(
                f'masks_topic resolution {orig_shape} != depth resolution '
                f'{depth.shape[:2]} -- nearest-neighbor-resized the mask to '
                'match (see module docstring\'s resize-on-mismatch note).',
                throttle_duration_sec=5.0)

        label = det_index + 1
        if label > 255:
            # Not reachable in practice: yolo_detector_node caps mask labels
            # at MAX_MASK_INSTANCES (254) and det_index tracks that same
            # detections array -- guarded anyway rather than risking a wrapped
            # uint8 label matching the wrong instance.
            self._stats['det_drop_mask_fallback'] += 1
            return self._median_depth(depth, cx_px, cy_px, box_w, box_h, img_w, img_h)

        sel = (mask_img == label) & np.isfinite(depth) & (depth > 0.0)
        if not np.any(sel):
            # Counted, not just warned: the warning is throttled to once per 5s
            # so it cannot show how OFTEN the mask path is unusable, which is
            # exactly what decides whether the mask sampling is earning its
            # place. Note this is a FALLBACK, not a dropped detection -- it
            # still yields a depth via the box-region method.
            self._stats['det_drop_mask_fallback'] += 1
            self.get_logger().warn(
                f'Detection index {det_index} (mask label {label}): zero '
                'valid-depth pixels in its instance mask -- falling back to '
                'box-region depth sampling for this detection.',
                throttle_duration_sec=5.0)
            return self._median_depth(depth, cx_px, cy_px, box_w, box_h, img_w, img_h)

        return float(np.median(depth[sel]))

    def _position_sigma(self, score, z):
        """Per-detection 1-sigma position uncertainty [m], from detection
        confidence and range. See the position_sigma_* params in __init__ for
        the model and where the defaults come from.

        score is clamped into [confidence_threshold, 1.0] before scaling:
        anything below the threshold was already dropped by the caller, and a
        score above 1.0 (which some backends can emit) must not produce a
        sigma smaller than the high-confidence floor.
        """
        lo = self.confidence_threshold
        hi = 1.0
        if hi <= lo:
            # Degenerate config (threshold at or above 1.0) -- no meaningful
            # confidence range to interpolate over, so treat every surviving
            # detection as high-confidence rather than dividing by zero.
            frac = 1.0
        else:
            frac = (float(np.clip(score, lo, hi)) - lo) / (hi - lo)
        # frac 1.0 (max confidence) -> scale 1.0; frac 0.0 -> low_conf_sigma_scale.
        conf_scale = self.low_conf_sigma_scale + frac * (1.0 - self.low_conf_sigma_scale)
        return (self.position_sigma_base_m
                + self.position_sigma_range_coeff * max(0.0, float(z))) * conf_scale

    def _build_detection3d(self, header, pose, width, height, class_id, score, z):
        det3d = Detection3D()
        det3d.header = header

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score = float(score)
        hyp.pose.pose = pose

        # 6x6 row-major [x y z roll pitch yaw] covariance. Only the position
        # block is populated: this node estimates no orientation at all (the
        # pose's rotation is the fixed optical->output_frame transform, not a
        # measurement), so the rotation diagonal is left at 0.0 rather than
        # invented. Consumers that need it should treat 0.0 as "not estimated".
        sigma = self._position_sigma(score, z)
        var_xy = sigma * sigma
        var_z = (sigma * self.position_sigma_z_scale) ** 2
        hyp.pose.covariance[0] = float(var_xy)   # xx
        hyp.pose.covariance[7] = float(var_xy)   # yy
        hyp.pose.covariance[14] = float(var_z)   # zz

        det3d.results.append(hyp)

        bbox = BoundingBox3D()
        bbox.center = pose
        bbox.size.x = float(width)
        bbox.size.y = float(height)
        # Depth-axis (camera Z) extent is not observable from a single view;
        # `default_depth_extent` is a fixed placeholder, not a measurement.
        bbox.size.z = self.default_depth_extent
        det3d.bbox = bbox

        return det3d

    def _build_marker(self, header, marker_id, pose, width, height, class_id):
        marker = Marker()
        marker.header = header
        marker.ns = 'detection_3d'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = max(float(width), 0.01)
        marker.scale.y = max(float(height), 0.01)
        marker.scale.z = self.default_depth_extent

        r, g, b = self._class_color(class_id)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.5

        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)

        return marker

    def _build_label_marker(self, header, marker_id, pose, height, class_id, score):
        marker = Marker()
        marker.header = header
        marker.ns = 'detection_3d_label'
        marker.id = marker_id + LABEL_ID_OFFSET
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Same x/y as the box; z floats above it. The box marker's orientation
        # carries the optical -> output_frame rotation (see _synced_callback), so its
        # *world*-vertical extent is `height` (scale.y, the object's image-plane
        # height converted to 3D) -- not scale.z (default_depth_extent, a depth
        # placeholder that ends up along the forward axis after that rotation, not
        # up). Using `height` here is what actually puts the label above the box
        # rather than off to one side.
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z + float(height) / 2.0 + self.label_z_offset
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.label_scale

        # White, distinct from the box's per-class color, for readability against
        # any box color the hash in _class_color happens to produce.
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f'{class_id} ({score:.2f})'

        # Same lifetime as the box marker -- both disappear together when a
        # detection ages out.
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)

        return marker

    @staticmethod
    def _class_color(class_id):
        # Deterministic per-class color: hash the class name into a hue-ish RGB
        # triple so the same class always renders the same color across frames.
        digest = hashlib.md5(str(class_id).encode('utf-8')).digest()
        return (digest[0] / 255.0, digest[1] / 255.0, digest[2] / 255.0)


def main(args=None):
    rclpy.init(args=args)
    node = Detection3DNode()
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
