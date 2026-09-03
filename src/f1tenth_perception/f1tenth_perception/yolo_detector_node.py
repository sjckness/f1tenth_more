#!/usr/bin/env python3
"""YOLO 2D object detector node for the F1TENTH perception stack.

Subscribes to a raw RGB image stream, runs an Ultralytics YOLO model, and
publishes:
  * vision_msgs/Detection2DArray on `detections_topic` (default /camera/detections)
  * sensor_msgs/Image           on `annotated_topic`  (default /camera/image_annotated)
    with, for every detection, class name + confidence drawn on the frame, PLUS
    either a bounding box (detect-task model, e.g. the deployed yolo26s.engine)
    OR a semi-transparent per-instance mask overlay (segment-task model, e.g.
    yolo26s-seg.pt) -- never both, and never boxes for a seg model. See
    _draw_mask_overlay()'s own docstring for the seg-model path; the detect-
    task path is unchanged from before this pass (Ultralytics' own
    result.plot()). Branches on the SAME `masks_np is not None` check already
    used just above it to decide whether to publish masks_topic at all -- not
    a second, independent condition -- so this also correctly falls through to
    the box-drawing call (a no-op box draw, since there are none) on a
    segment-task model's own zero-detection frames, identical in effect to a
    detect-task model's zero-detection frames.
  * sensor_msgs/Image           on `masks_topic` (default /camera/detection_masks)
    -- ONLY when the loaded model's task is 'segment' (e.g. yolo26s-seg.pt): a
    mono8 per-pixel instance-index label image, same resolution as the input
    frame and same header/stamp as `detections_topic`'s message for that frame.
    Pixel value 0 = background/no detection; value (i+1) = index into that
    frame's Detection2DArray.detections (so pixel value N belongs to
    `detections[N-1]`). mono8 caps this at 254 simultaneous detections in one
    frame (255 reserved/unused) -- if that many appears, only the first 254 are
    written into the mask and a warning is logged; detections beyond that still
    publish normally on `detections_topic`, they just carry no mask pixels (a
    downstream consumer treats "no pixels for my index" as "no mask data",
    identically to a detect-task model publishing nothing here at all -- see
    detection_3d_node.py's box-region fallback). Where two instance masks
    overlap, the later detection's pixels win (last-write on the shared
    label image) -- a known limitation of a single-channel index encoding,
    not a bug. For a detect-task model (e.g. the deployed yolo26s.engine),
    this topic is never created/published at all -- zero added per-frame cost
    on the existing box-only path.

The input topic defaults to **/camera/image_raw** so the node is agnostic to the
active camera source (v4l2 webcam or ZED2 wrapper); the bringup launch remaps
whichever camera is selected onto that canonical topic.

If no model is configured (`model_path` empty) OR Ultralytics is not importable,
the node runs in **passthrough mode**: it publishes an empty Detection2DArray and
re-publishes the incoming frame on the annotated topic (so both topics stay live
and downstream tooling still sees a stream).

The `device` parameter (default `cuda`) selects the torch device YOLO runs on.
If `cuda` is requested but `torch.cuda.is_available()` is False (e.g. a CPU-only
torch wheel on a CUDA-capable host), the node raises at startup rather than
silently running inference on the CPU -- pass `device:=cpu` to opt into CPU
explicitly (the CPU path is still fully supported, just never used implicitly).

`model_path` may point at a native `*.pt` checkpoint (runs via torch/cuBLAS) or
a `*.engine` file built for this exact machine via TensorRT:
    yolo export model=yolo26s.pt format=onnx device=cpu simplify=True
    trtexec --onnx=yolo26s.onnx --saveEngine=yolo26s.engine --fp16
(ONNX export runs on CPU deliberately -- it only traces the graph once, and
doing so avoids depending on GPU torch/cuBLAS for the export step itself.)
Engines are NOT portable across devices/TensorRT versions, even same-model
Jetsons, and must be rebuilt per host -- see .gitignore, these are never
committed. TensorRT engines are CUDA-only and already bound to a device at
build time, so `Model.to(device)` is skipped for them (it would raise) and
`device:='cpu'` is rejected outright for a `*.engine` model_path.

Class names for a `*.engine` model_path (Foxglove-class-name pass): the raw
`trtexec` step above builds a bare TensorRT engine with none of Ultralytics'
own metadata (that only survives an engine built via `model.export(format=
'engine')`, which produces a `metadata.yaml` sidecar this manual two-step
recipe never creates) -- confirmed live: loading the deployed yolo26s.engine
directly, `model.names` comes back as a generic 999-entry `{0: 'class0', 1:
'class1', ...}` placeholder, not the real 80-class COCO set. Every
downstream consumer (detection_3d_node.py's/semantic_layer_node.py's own
Foxglove-facing MarkerArray text labels) was already displaying whatever
this node published as `class_id` correctly -- the string itself was just
wrong at the source, not misdisplayed downstream. Fixed here, not there:
_resolve_class_names() below loads real names from the `*.pt` checkpoint
sitting next to the engine (same stem, confirmed live to carry the correct
COCO names -- it's the exact checkpoint the engine was exported from) and
uses that dict instead of the engine's own placeholder one. Falls back to
the engine's own `model.names` if no sibling `.pt` is found or it fails to
load, so a hypothetical engine that WAS exported via Ultralytics' own
`.export()` (real metadata intact) still works, not just this one.

Task resolution (detect vs segment): a `*.pt` checkpoint carries its own task
in its embedded metadata (confirmed live: `YOLO('yolo26s-seg.pt').task ==
'segment'`, `YOLO('yolo26s.pt').task == 'detect'`) -- loaded with no explicit
`task=` override so Ultralytics' own metadata decides, same reasoning as not
forcing class names. A `*.engine` model_path has NO such metadata (identical
root cause to the missing class names above), so its task comes from the
`model_task` param instead (default 'detect', matching every engine this repo
has shipped so far) -- passed as `YOLO(model_path, task=model_task)`. Whichever
path resolved it, `self.task` (== `self.model.task` after load) is what
actually gates mask publishing below, not model_task or the file extension
directly -- so a hypothetical future segmentation `*.engine` (model_task:=
segment) drives the exact same mask-publishing code as a `*-seg.pt` does.

Car's-own-LiDAR exclusion (self-occlusion filter): the ZED's field of view
includes the car's own LiDAR housing, sitting in the bottom-right of frame --
a fixed self-occlusion artifact confirmed live via a Foxglove screenshot (a
spurious high-confidence "chair" detection sitting exactly on top of it).
`_lidar_exclusion_keep_mask()` runs once, right after inference, on the raw
Ultralytics `result` (BEFORE the Detection2DArray-building loop, masks_topic,
and the annotated image all read from it) -- a detection whose own box (or,
for a segment-task model, its own mask, more accurate than its box when
available) overlaps the configured exclusion rectangle above
`lidar_exclusion_overlap_threshold` never reaches ANY of those three outputs,
and therefore never reaches detection_3d_node.py's Detection3DArray/
obstacle_projector_node's Obstacle2DArray either -- one filter point, not one
per consumer. The rectangle itself (`lidar_exclusion_x_min/_x_max/_y_min/
_y_max`, fractions of image width/height, not raw pixels -- stays correct
across a camera resolution change) is a stack_params.yaml default,
screenshot-calibrated with margin (x in [0.75, 1.0], y in [0.55, 1.0] --
right 25%/bottom 45% of frame) -- NOT yet measured against a real-resolution
live frame with the ZED mounted on the car (attempted this pass; blocked,
the camera wasn't mounted on the car at the time -- see stack_params.yaml's
own lidar_exclusion_x_min comment for the full calibration writeup). See
that same comment for the trade-off: a genuine external obstacle overlapping
this same small, near-field region would also go unseen by this path.

Message field reference (vision_msgs, Humble == "4.x" layout):
    Detection2DArray.detections[]            -> Detection2D
    Detection2D.results[]                     -> ObjectHypothesisWithPose
    ObjectHypothesisWithPose.hypothesis       -> ObjectHypothesis
        .hypothesis.class_id  (string)        -> class name
        .hypothesis.score     (float)         -> confidence
    Detection2D.bbox                          -> BoundingBox2D
        .bbox.center          (vision_msgs/Pose2D)
        .bbox.center.position.x / .y          -> bbox center in pixels
        .bbox.size_x / .bbox.size_y           -> bbox width / height in pixels
"""

import hashlib
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge, CvBridgeError

from f1tenth_perception.cpu_affinity import (
    apply_nice,
    declare_nice_param,
)

# mono8 instance-index mask image: 0 is reserved for background, so at most
# 255 distinct detections are representable and 255 itself is left unused
# (kept as a visually-obvious "not a valid index" value rather than the last
# legal one) -- see module docstring's masks_topic paragraph.
MAX_MASK_INSTANCES = 254

# Mask-overlay opacity for the annotated-image seg-model path (_draw_mask_
# overlay) -- 0.5 matches Ultralytics' own default Annotator mask alpha, kept
# as a plain module constant rather than a ROS param since nothing else in
# this pass needed it tunable; bump to a declare_parameter if that changes.
MASK_OVERLAY_ALPHA = 0.5


# ==============================================================================
# Car's-own-LiDAR exclusion -- pure functions, no rclpy/Ultralytics dependency,
# independently unit-testable (same "pure logic separate from ROS glue"
# convention is_proximity_too_close.py's own _in_lidar_window/_in_front_cone
# and lidar_boundary_node.py's own _classify_side already use). See module
# docstring's own "Car's-own-LiDAR exclusion" paragraph for the full picture.
# ==============================================================================

def _bbox_overlap_fraction(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
    """Fraction of the [x1,y1,x2,y2] box's OWN area that falls inside the
    exclusion rectangle [ex1,ey1,ex2,ey2] (both in the same pixel-coordinate
    space) -- deliberately NOT an IoU (which would also weight the exclusion
    rectangle's own area, irrelevant here: a detection is excluded based on
    how much of ITSELF sits over the LiDAR housing, regardless of the
    housing's own footprint size). Returns 0.0 for a degenerate (zero-area)
    box or a non-overlapping pair, never divides by zero."""
    ix1, iy1 = max(x1, ex1), max(y1, ey1)
    ix2, iy2 = min(x2, ex2), min(y2, ey2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    box_area = (x2 - x1) * (y2 - y1)
    if box_area <= 0.0:
        return 0.0
    return ((ix2 - ix1) * (iy2 - iy1)) / box_area


def _mask_overlap_fraction(mask, ex1, ey1, ex2, ey2):
    """Fraction of `mask`'s (2D boolean array) OWN True pixels that fall
    inside the exclusion rectangle [ex1,ey1,ex2,ey2] (pixel coords, will be
    clamped to `mask`'s own shape and truncated to int here -- same
    half-open [ex1,ex2)x[ey1,ey2) convention array slicing already uses).
    Returns 0.0 for an empty mask or a rectangle that clamps to nothing,
    never divides by zero. More accurate than _bbox_overlap_fraction for a
    segment-task detection -- see module docstring."""
    total = int(np.count_nonzero(mask))
    if total == 0:
        return 0.0
    h, w = mask.shape[:2]
    cx1, cy1 = max(int(ex1), 0), max(int(ey1), 0)
    cx2, cy2 = min(int(ex2), w), min(int(ey2), h)
    if cx2 <= cx1 or cy2 <= cy1:
        return 0.0
    inside = int(np.count_nonzero(mask[cy1:cy2, cx1:cx2]))
    return inside / total


class YoloDetectorNode(Node):
    def __init__(self, **kwargs):
        # **kwargs: lets tests pass parameter_overrides=[...] straight through
        # to rclpy.Node, same convention as every other node in this workspace
        # that has its own test file (e.g. f1tenth_costmap's
        # costmap_boundary_node.py/semantic_layer_node.py) -- no effect on
        # normal launch-file construction, which never passes kwargs here.
        super().__init__('yolo_detector_node', **kwargs)

        # nice: declared early (see cpu_affinity.py), applied at the end of
        # __init__ once model loading is done. CPU AFFINITY (this node
        # showed the clearest contention signature in the perception-latency
        # audit -- nonvoluntary:voluntary context-switch ratio ~6:1,
        # /camera/detections lagging /camera/image_raw by ~278ms measured via
        # ros2 topic delay, despite only ~22ms of its own in-callback
        # inference time) is now a `taskset -c` launch prefix instead of an
        # in-process self-pin -- see detection.launch.py's own
        # yolo_cpu_affinity comment for why (thread-pinning-leak fix, Step 6
        # reintroduction investigation: the old self-pin, applied here AFTER
        # model loading, left 32 of this node's 36 threads -- including
        # CUDA/TensorRT inference threads spawned during that loading --
        # fully unpinned, confirmed live executing on the EKF pair's own
        # reserved cores).
        declare_nice_param(self)

        # ---- parameters ----------------------------------------------------
        # image_topic defaults to the canonical /camera/image_raw so the same
        # node works whether the webcam (v4l2_camera) or the ZED2 wrapper is the
        # active source; the bringup launch remaps the selected camera onto it.
        self.image_topic = str(
            self.declare_parameter('image_topic', '/camera/image_raw').value)
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections').value)
        self.annotated_topic = str(
            self.declare_parameter('annotated_topic', '/camera/image_annotated').value)
        # Only ever published when the loaded model resolves to task 'segment'
        # (see module docstring) -- the publisher itself is only created in that
        # case, so a detect-task model (the deployed default) pays zero per-frame
        # cost for this topic existing.
        self.masks_topic = str(
            self.declare_parameter('masks_topic', '/camera/detection_masks').value)
        self.model_path = str(self.declare_parameter('model_path', '').value)
        # Torch device YOLO runs inference on. Default 'cuda' matches the
        # Jetson Orin deployment target; pass device:=cpu to explicitly opt
        # into the (still fully supported) CPU path.
        self.device = str(self.declare_parameter('device', 'cuda').value)
        # Only consulted for a *.engine model_path -- see module docstring's
        # "Task resolution" paragraph. Ignored for *.pt, which carries its own
        # task in the checkpoint.
        self.model_task = str(self.declare_parameter('model_task', 'detect').value)
        # Same stack-wide param/default as detection_3d_node's own
        # confidence_threshold (f1tenth_params/config/stack_params.yaml) --
        # previously only reached detection_3d_node via detection.launch.py,
        # so this node's own /camera/detections (and the annotated image) still
        # carried every box down to Ultralytics' internal default (~0.25)
        # regardless of what confidence_threshold was set to. Passed straight
        # into inference below (conf=...) rather than filtered after the fact,
        # so low-confidence boxes never even reach cv2 annotation/publishing.
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.3).value)
        # Car's-own-LiDAR exclusion rectangle -- see module docstring's own
        # paragraph and stack_params.yaml's lidar_exclusion_x_min comment for
        # the full rationale/trade-off/calibration source. Fractions
        # (0.0-1.0) of image width/height, not raw pixels. Defaults here
        # match stack_params.yaml's own (same convention every other param
        # in this file already follows, e.g. confidence_threshold) --
        # screenshot-calibrated with margin, NOT yet measured against a
        # real-resolution live frame.
        self.lidar_exclusion_x_min = float(
            self.declare_parameter('lidar_exclusion_x_min', 0.75).value)
        self.lidar_exclusion_x_max = float(
            self.declare_parameter('lidar_exclusion_x_max', 1.0).value)
        self.lidar_exclusion_y_min = float(
            self.declare_parameter('lidar_exclusion_y_min', 0.55).value)
        self.lidar_exclusion_y_max = float(
            self.declare_parameter('lidar_exclusion_y_max', 1.0).value)
        self.lidar_exclusion_overlap_threshold = float(
            self.declare_parameter('lidar_exclusion_overlap_threshold', 0.5).value)

        # ---- model ---------------------------------------------------------
        self.model = None
        # class-index -> class-name, resolved once at startup (see
        # _resolve_class_names()'s own docstring for the *.engine case) and
        # used per-detection in image_callback() below instead of re-reading
        # result.names every frame.
        self.names = {}
        # 'detect' or 'segment', resolved once at startup from the loaded
        # model (see module docstring's "Task resolution" paragraph) -- gates
        # whether the masks publisher below is created at all.
        self.task = 'detect'
        if not self.model_path:
            self.get_logger().warn(
                'No YOLO model_path set — running in passthrough mode '
                '(empty detections, annotated == raw frame)')
        else:
            try:
                from ultralytics import YOLO
                import torch
            except Exception as exc:  # noqa: BLE001 - stay alive in passthrough
                self.get_logger().error(
                    f'Failed to import Ultralytics/torch: {exc} — falling '
                    'back to passthrough mode')
            else:
                is_engine = self.model_path.endswith('.engine')
                if is_engine and self.device == 'cpu':
                    raise RuntimeError(
                        f'model_path="{self.model_path}" is a TensorRT engine, '
                        'which is CUDA-only and already bound to a device at '
                        'build time -- device:="cpu" is not valid for it. Use '
                        'a *.pt checkpoint for CPU inference instead.')
                if self.device == 'cuda' and not torch.cuda.is_available():
                    # Deliberately NOT caught below / not a passthrough
                    # fallback: a CPU-only torch wheel silently serving
                    # "working" but far slower inference is worse than a
                    # loud startup failure. See module docstring.
                    raise RuntimeError(
                        f'device="cuda" requested but torch.cuda.is_available() '
                        f'is False (torch {torch.__version__}). On Jetson this '
                        'usually means a generic CPU-only wheel is installed '
                        'instead of a JetPack-matched CUDA build. Pass '
                        'device:=cpu to explicitly run on CPU instead.')
                try:
                    # *.engine has no embedded task metadata -- resolved from
                    # model_task instead. *.pt is loaded with no task= override
                    # so Ultralytics' own checkpoint metadata decides (forcing
                    # 'detect' here, as this used to do unconditionally, would
                    # silently mis-load a *-seg.pt as a detect-only model). See
                    # module docstring's "Task resolution" paragraph.
                    self.model = YOLO(
                        self.model_path, task=self.model_task if is_engine else None)
                    if not is_engine:
                        # TensorRT engines are already device-bound at build
                        # time; Model.to() only supports native *.pt models
                        # and raises TypeError otherwise.
                        self.model.to(self.device)
                    self.names = self._resolve_class_names(YOLO, self.model_path, is_engine)
                    self.task = self.model.task
                    self.get_logger().info(
                        f'Loaded YOLO model "{self.model_path}" on '
                        f'device="{self.device}" task="{self.task}" '
                        f'({len(self.names)} classes)')
                except Exception as exc:  # noqa: BLE001 - stay alive in passthrough
                    self.get_logger().error(
                        f'Failed to load Ultralytics YOLO ("{self.model_path}") '
                        f'on device="{self.device}": {exc} — falling back to '
                        'passthrough mode')
                    self.model = None

        # ---- ROS interfaces -----------------------------------------------
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, 10)
        # Only created for a segment-task model -- see module docstring and
        # masks_topic's own param comment above. None on the box-only path
        # (the deployed default), checked in image_callback() before use.
        self.masks_pub = (
            self.create_publisher(Image, self.masks_topic, 10)
            if self.task == 'segment' else None)
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        apply_nice(self)

        self.get_logger().info(
            f'yolo_detector_node up: subscribing "{self.image_topic}", '
            f'publishing detections "{self.detections_topic}" and annotated '
            f'"{self.annotated_topic}"' + (
                f', masks "{self.masks_topic}" (task=segment)'
                if self.masks_pub is not None else ''))

    def _resolve_class_names(self, YOLO, model_path, is_engine):  # noqa: N803 - YOLO is a class
        """class-index -> class-name dict to actually use for this loaded
        model. For a *.pt checkpoint, `model.names` (Ultralytics' own,
        embedded in the checkpoint) is already correct -- used directly.

        For a *.engine built via this file's own documented manual
        `trtexec` recipe (see module docstring's "Class names for a
        *.engine model_path" paragraph), `model.names` is a generic
        placeholder (confirmed live on yolo26s.engine: 999 entries,
        '{0: "class0", 1: "class1", ...}"', not real names) -- the engine
        simply never had Ultralytics' metadata to begin with. The sibling
        *.pt checkpoint (same stem, right next to the engine in models/)
        is the actual source the engine was exported from and still has
        the real names (confirmed live: yolo26s.pt's `.names` is the real
        80-class COCO set) -- loaded here ONLY for its `.names` attribute
        (no `.to(device)`, no inference on it), then used in place of the
        engine's own placeholder. Falls back to the engine's own
        `model.names` if no sibling *.pt exists or loading it fails, so an
        engine that WAS built via Ultralytics' own `.export()` (metadata
        intact, real names already) isn't penalized either way.
        """
        if not is_engine:
            return dict(self.model.names)

        sibling_pt = model_path[: -len('.engine')] + '.pt'
        if not os.path.isfile(sibling_pt):
            self.get_logger().warn(
                f'No sibling .pt checkpoint at "{sibling_pt}" for engine '
                f'"{model_path}" -- using the engine\'s own class names as-is '
                '(a manually-trtexec-built engine will show placeholder '
                '"classN" names; see module docstring).')
            return dict(self.model.names)
        try:
            # Loaded with the same model_task as the engine itself (this
            # method only runs when is_engine is True) -- a *_seg.pt sibling
            # loaded with a mismatched forced task='detect' risks the same
            # kind of silent misinterpretation this whole file is careful to
            # avoid elsewhere; .names is read-only metadata either way, never
            # used for inference on this sibling.
            names = dict(YOLO(sibling_pt, task=self.model_task).names)
            self.get_logger().info(
                f'Resolved real class names from sibling checkpoint '
                f'"{sibling_pt}" ("{model_path}" itself has no usable '
                'class-name metadata -- see module docstring).')
            return names
        except Exception as exc:  # noqa: BLE001 - names are a nice-to-have, not fatal
            self.get_logger().warn(
                f'Failed to load class names from sibling checkpoint '
                f'"{sibling_pt}": {exc} -- using the engine\'s own class '
                'names as-is.')
            return dict(self.model.names)

    def _lidar_exclusion_keep_mask(self, result, img_h, img_w):
        """Boolean array (len == len(result.boxes)) -- True means KEEP that
        detection (index-aligned with `result.boxes`/`result.masks`, same
        order `image_callback()`'s own loops already assume). Computes the
        configured exclusion rectangle (fractions of this frame's own
        img_h/img_w -- not a fixed pixel rectangle, so this stays correct
        even if the published camera resolution ever changes) in pixel
        coordinates once, then checks each detection's own overlap against
        it: `_mask_overlap_fraction()` (more accurate) when this result has
        real per-detection mask data (a segment-task model's frame),
        `_bbox_overlap_fraction()` otherwise (a detect-task model, or a
        segment-task model's own detection whose mask decode was empty --
        see _publish_masks()'s own MAX_MASK_INSTANCES-cap note for one way
        that can legitimately happen). See module docstring's own "Car's-
        own-LiDAR exclusion" paragraph for the full picture.
        """
        ex1 = self.lidar_exclusion_x_min * img_w
        ex2 = self.lidar_exclusion_x_max * img_w
        ey1 = self.lidar_exclusion_y_min * img_h
        ey2 = self.lidar_exclusion_y_max * img_h

        masks_np = (
            result.masks.data.cpu().numpy() if result.masks is not None else None)

        keep = np.ones(len(result.boxes), dtype=bool)
        for i, box in enumerate(result.boxes):
            if masks_np is not None and np.any(masks_np[i] > 0.5):
                overlap = _mask_overlap_fraction(masks_np[i] > 0.5, ex1, ey1, ex2, ey2)
            else:
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                overlap = _bbox_overlap_fraction(x1, y1, x2, y2, ex1, ey1, ex2, ey2)
            if overlap > self.lidar_exclusion_overlap_threshold:
                keep[i] = False
        return keep

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR (3-channel) for inference/drawing.
        # cv_bridge handles rgb8 (webcam) and bgra8 (ZED) -> bgr8 conversion.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        out = Detection2DArray()
        out.header = msg.header  # keep detections time/frame aligned to the image

        if self.model is None:
            # Passthrough: empty detections, re-publish the frame unmodified so
            # the annotated topic stays live.
            self.det_pub.publish(out)
            self._publish_annotated(cv_image, msg.header)
            return

        # ---- real inference ------------------------------------------------
        # retina_masks=True: for a segment-task model, makes Ultralytics itself
        # upsample result.masks.data from the internal inference resolution
        # (e.g. 640x480) back to cv_image's own resolution using its own known-
        # correct decode (this is exactly what retina_masks is for -- see
        # module docstring's "Task resolution" paragraph and the mask-fusion
        # design note in detection_3d_node.py for why that decode is trusted
        # rather than hand-rolled here). Confirmed live: with retina_masks=True,
        # result.masks.data.shape[1:] == cv_image.shape[:2] exactly, no manual
        # resize needed in this node. A no-op for a detect-task model (verified
        # live: result.masks stays None, no error) so this is always passed.
        results = self.model(
            cv_image, verbose=False, device=self.device,
            conf=self.confidence_threshold, retina_masks=True)
        result = results[0]

        # Car's-own-LiDAR exclusion -- see module docstring's own paragraph.
        # Applied here, on the raw Ultralytics result, BEFORE anything below
        # reads from it (Detection2DArray, masks_topic, annotated image all
        # derive from `result` from this point on) -- indexing `result` with
        # a boolean array re-subsets .boxes/.masks together, consistently
        # (confirmed live: Ultralytics' own Results.__getitem__), so no
        # downstream code needs its own awareness of this filter at all.
        if len(result.boxes) > 0:
            keep = self._lidar_exclusion_keep_mask(result, *cv_image.shape[:2])
            if not keep.all():
                self.get_logger().debug(
                    f'{int((~keep).sum())} detection(s) this frame suppressed '
                    '-- overlaps the car\'s-own-LiDAR exclusion rectangle '
                    'above lidar_exclusion_overlap_threshold.')
                result = result[keep]

        masks_np = None
        if self.masks_pub is not None and result.masks is not None:
            masks_np = result.masks.data.cpu().numpy()

        for i, box in enumerate(result.boxes):
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            cls_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            out.detections.append(self._build_detection(
                class_id=self.names.get(cls_id, str(cls_id)), score=score,
                cx=cx, cy=cy, w=(x2 - x1), h=(y2 - y1)))

        self.det_pub.publish(out)

        # PER-FRAME MASK INVARIANT (2026-09-02 stale-track fix)
        # ----------------------------------------------------------------
        # In segment mode this topic must carry EXACTLY ONE message per
        # processed frame, INCLUDING frames with no detections. It used to
        # publish only when `result.masks is not None`, which is never true on
        # a zero-detection frame, and that had a non-obvious consequence two
        # nodes downstream:
        #
        #   no mask -> detection_3d_node's 3-way ApproximateTimeSynchronizer
        #              (detections + depth + masks) cannot fire
        #           -> no Detection3DArray published for that frame
        #           -> semantic_layer_node's miss-streak lifecycle, which
        #              explicitly relies on one message per frame INCLUDING
        #              empty ones, never ticks
        #           -> a track whose object has left the scene is never aged
        #              out and its marker persists indefinitely.
        #
        # Measured on the 2026-09-01 bags: only 3.0% of zero-detection frames
        # produced a mask, vs 99.2% of frames containing a detection. (That
        # same split is why the headline "35-45% 2D->3D yield loss" was almost
        # entirely empty frames and cost avoidance nothing -- this fix is for
        # TRACK AGING, not avoidance, which reads /perception/obstacles_2d.)
        if self.masks_pub is not None:
            published = False
            if masks_np is not None:
                published = self._publish_masks(
                    masks_np, cv_image.shape[:2], msg.header)
            if not published:
                # Covers both "no detections this frame" and _publish_masks'
                # own error early-outs (shape mismatch, encode failure): the
                # invariant is per-FRAME, so a frame whose real mask could not
                # be built still has to emit something, or it silently becomes
                # another never-aged-out gap of exactly the kind above.
                self._publish_empty_mask(msg.header)
            # Segment-task model, this frame had detections: masks instead
            # of boxes -- see module docstring / _draw_mask_overlay()'s own
            # docstring. out.detections is already in the same order/index
            # as masks_np (both built from this same `result`, same i).
            annotated = self._draw_mask_overlay(cv_image, masks_np, out.detections)
        else:
            # Detect-task model (the deployed default), OR a segment-task
            # model's own zero-detection frame (result.masks is None then
            # too, identical to the detect-task case -- see module
            # docstring) -- unchanged from before this pass either way.
            # result.plot() reads result.names (copied from the model at
            # inference time) to draw its own "class conf" labels --
            # overwritten here so the annotated image's on-frame labels use
            # the same resolved self.names as Detection2DArray above, not
            # the model's own (possibly-placeholder, *.engine case) names.
            # See _resolve_class_names()'s own docstring.
            result.names = self.names
            annotated = result.plot()
        self._publish_annotated(annotated, msg.header)

    def _publish_annotated(self, cv_image, header):
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge annotated encode failed: {exc}')
            return
        annotated_msg.header = header
        self.annotated_pub.publish(annotated_msg)

    def _publish_empty_mask(self, header):
        """Publish a ZERO-SIZE mono8 mask carrying only `header`, to keep the
        one-mask-per-frame invariant on frames that have no instance masks.
        See the PER-FRAME MASK INVARIANT comment in image_callback().

        Zero-size rather than a full-resolution blank image: the only consumer,
        detection_3d_node, indexes the mask exclusively inside its
        per-detection loop, which does not execute on a frame with no
        detections. A 1280x720 blank would be ~920 KB at ~14 Hz (~13 MB/s) of
        pure waste on this box; this carries no pixels at all. Only the header
        stamp matters, since the mask's whole job here is to let the 3-way
        synchronizer fire.

        BUILT BY HAND, NOT VIA CvBridge, deliberately: cv2_to_imgmsg() on a
        0x0 array raises ZeroDivisionError computing its step (verified on this
        box, not assumed). detection_3d_node has a matching explicit guard
        treating height/width == 0 as "no mask this frame", falling back to
        box-region depth sampling -- which is a no-op on an empty frame, since
        it has no detections to sample for.
        """
        mask_msg = Image()
        mask_msg.header = header
        mask_msg.height = 0
        mask_msg.width = 0
        mask_msg.encoding = 'mono8'
        mask_msg.is_bigendian = 0
        mask_msg.step = 0
        mask_msg.data = []
        self.masks_pub.publish(mask_msg)

    def _publish_masks(self, masks_np, shape_hw, header):
        """Build and publish the mono8 instance-index label image for one
        frame. `masks_np` is `result.masks.data` as a numpy array (N, H, W) of
        {0., 1.} float/uint values (Ultralytics' own decode, retina_masks=True
        -- see image_callback()); `shape_hw` is cv_image's own (h, w), i.e.
        what the label image must match. See module docstring's masks_topic
        paragraph for the encoding (0=background, pixel N -> detections[N-1])
        and the MAX_MASK_INSTANCES cap.
        """
        h, w = shape_hw
        if masks_np.shape[1:] != (h, w):
            # Should not happen -- retina_masks=True is exactly what makes
            # Ultralytics upsample to cv_image's own resolution (confirmed
            # live, see image_callback()'s comment). Guarded anyway rather
            # than trusting it silently forever: skip this frame's mask
            # publish (detections_topic still publishes normally) instead of
            # emitting a label image that doesn't line up with the frame it
            # claims to describe.
            self.get_logger().error(
                f'result.masks shape {masks_np.shape[1:]} != cv_image shape '
                f'{(h, w)} despite retina_masks=True -- skipping masks_topic '
                'publish this frame (detections_topic still published).',
                throttle_duration_sec=5.0)
            return False

        n = masks_np.shape[0]
        if n > MAX_MASK_INSTANCES:
            self.get_logger().warn(
                f'{n} detections this frame exceeds the {MAX_MASK_INSTANCES}-'
                'instance mono8 mask cap -- only the first '
                f'{MAX_MASK_INSTANCES} get mask pixels; the rest still '
                f'publish normally on "{self.detections_topic}" with no mask '
                'data (see module docstring).', throttle_duration_sec=5.0)

        label_img = np.zeros((h, w), dtype=np.uint8)
        for i in range(min(n, MAX_MASK_INSTANCES)):
            # Higher index (later detection) wins where masks overlap -- see
            # module docstring's "last-write" note.
            label_img[masks_np[i] > 0.5] = i + 1

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(label_img, encoding='mono8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge mask encode failed: {exc}')
            return False
        mask_msg.header = header
        self.masks_pub.publish(mask_msg)
        return True

    def _draw_mask_overlay(self, cv_image, masks_np, detections):
        """Builds the annotated frame for a segment-task model's frame that
        had at least one detection: a semi-transparent (MASK_OVERLAY_ALPHA)
        colored mask per detected instance instead of a bounding box, plus
        the same "class_name (confidence)" label text the detect-task path
        already draws -- anchored at the mask's own topmost pixel (centered
        on its horizontal centroid), since a seg detection has no box to
        anchor a label against the way result.plot()'s own box-label
        placement does.

        `masks_np` (N, H, W) and `detections` (this frame's
        Detection2DArray.detections, length N) must be the same length and
        in the same per-instance order -- guaranteed by image_callback()'s
        own call site (both built from the same `result`, indexed by the
        same `i`; identical convention to _publish_masks()'s own masks_np/
        detection-index pairing).

        Color is keyed by CLASS (via _class_color_bgr()), not by instance --
        see that method's own docstring for why: this makes a given class
        render the same color here as it does in detection_3d_node.py's own
        Foxglove MarkerArray, one shared convention instead of a second,
        independently-invented one. Two overlapping instances of the SAME
        class are therefore indistinguishable by color alone in this
        overlay (a deliberate trade-off, not a bug -- masks_topic's own
        per-instance index encoding, used by the depth-fusion path, is
        unaffected either way).
        """
        annotated = cv_image.copy()
        overlay = cv_image.copy()

        instance_masks = []  # (mask, color, class_id, score), built once, reused below
        for i, det in enumerate(detections):
            mask = masks_np[i] > 0.5
            if not np.any(mask):
                continue  # e.g. this instance is beyond MAX_MASK_INSTANCES -- see _publish_masks
            class_id = det.results[0].hypothesis.class_id if det.results else ''
            score = det.results[0].hypothesis.score if det.results else 0.0
            color = self._class_color_bgr(class_id)
            overlay[mask] = color
            instance_masks.append((mask, color, class_id, score))

        # Single whole-image blend (not one per instance): unmasked pixels
        # blend overlay==cv_image against cv_image itself, i.e. alpha*x +
        # (1-alpha)*x == x -- unchanged, so background never picks up a
        # tint, only the pixels actually overwritten above do.
        cv2.addWeighted(
            overlay, MASK_OVERLAY_ALPHA, annotated, 1.0 - MASK_OVERLAY_ALPHA, 0.0, dst=annotated)

        img_h, img_w = annotated.shape[:2]
        for mask, color, class_id, score in instance_masks:
            ys, xs = np.where(mask)
            label_x = int(xs.mean())
            label_y = max(int(ys.min()) - 6, 12)  # small margin above the mask; clamped onscreen
            text = f'{class_id} ({score:.2f})'
            (text_w, text_h), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            box_w, box_h = text_w + 4, text_h + baseline + 2
            # Clamped on ALL four edges (not just top-left) -- a mask near
            # the frame's right/bottom edge, or one small enough that its
            # own label is wider than the gap to that edge, would otherwise
            # push top_left right/down past the frame or leave bottom_right
            # unclamped past it entirely (cv2 itself would just silently
            # clip the drawing, so this isn't a crash risk, but an unclamped
            # rectangle can span most of the frame width for a narrow
            # canvas/wide label combination -- confirmed while testing this
            # method, see test_two_instances_of_different_classes_get_
            # different_colors's own history). Clamping bottom_right first
            # against the frame, then re-deriving top_left from it, keeps
            # the box's actual on-screen SIZE correct (text_w/text_h+baseline
            # unchanged) instead of stretching it if only top_left were
            # clamped independently.
            bottom_right = (
                min(max(label_x - text_w // 2 - 2, 0) + box_w, img_w - 1),
                min(max(label_y - text_h - 2, 0) + box_h, img_h - 1))
            # max(...,0) here too: only reachable if box_w/box_h exceeds the
            # frame's own width/height outright (a label wider than the
            # entire image) -- the rectangle's drawn size shrinks in that
            # pathological case rather than top_left going negative again.
            top_left = (max(bottom_right[0] - box_w, 0), max(bottom_right[1] - box_h, 0))
            # Filled background rect in the mask's own class color, white
            # text on top -- mirrors detection_3d_node.py's own
            # _build_label_marker reasoning ("white, distinct from the box's
            # per-class color, for readability against any color the hash
            # happens to produce").
            cv2.rectangle(annotated, top_left, bottom_right, color, thickness=-1)
            cv2.putText(
                annotated, text, (top_left[0] + 2, bottom_right[1] - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return annotated

    @staticmethod
    def _class_color_bgr(class_id):
        """Deterministic per-class color, BGR uint8 (cv2's own channel
        order/dtype) -- mirrors detection_3d_node.py's own _class_color()
        EXACTLY (same MD5-digest-of-class-name formula, same digest byte
        indices mapped to the same R/G/B channels), so a given class_id
        renders as the same visual color in this 2D annotated image as it
        does in that node's Foxglove MarkerArray -- one shared color
        convention, not a second, independently-invented palette (see
        module docstring / _draw_mask_overlay()'s own docstring).

        Duplicated here rather than imported: detection_3d_node.py is out of
        scope for this pass (see this pass's own guardrails) -- this is a
        pure six-line hash formula with essentially zero drift risk, so
        duplication was chosen over touching that file. Keep both in sync if
        the formula itself is ever revisited. Only the OUTPUT format differs
        from the original: BGR uint8 tuple here (cv2 drawing calls), vs. RGB
        float 0.0-1.0 there (ROS Marker.color fields) -- same three digest
        bytes either way, just reordered/rescaled for the target API.
        """
        digest = hashlib.md5(str(class_id).encode('utf-8')).digest()
        r, g, b = int(digest[0]), int(digest[1]), int(digest[2])
        return (b, g, r)

    @staticmethod
    def _build_detection(class_id, score, cx, cy, w, h):
        """Build one Detection2D using the Humble vision_msgs API.

        cx, cy are the bbox center in pixels; w, h are width/height in pixels.
        """
        det = Detection2D()

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)

        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.center.theta = 0.0
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        return det


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
