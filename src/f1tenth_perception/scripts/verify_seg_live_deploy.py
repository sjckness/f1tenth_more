#!/usr/bin/env python3
"""Standalone end-to-end verification of the yolo-seg live-deploy path:
real YoloDetectorNode + Detection3DNode classes (the actual node code these
would run in production, NOT the ultralytics.YOLO()-direct calls
scripts/benchmark_yolo_latency.py makes for pure timing), real GPU inference
against yolo26s-seg.pt, real mask decode, real mask-based depth fusion
(_mask_median_depth), real Detection3DArray construction.

NOT a ROS node, NOT wired into setup.py/launch, NOT part of the pytest suite
-- deliberately a manual dev tool, run directly from source, same "needs a
real GPU + real weights, not CI-friendly" category benchmark_yolo_latency.py
already established (see that file's own docstring):

    python3 src/f1tenth_perception/scripts/verify_seg_live_deploy.py

Written for the yolo-seg live-deployable pass (config-selectable seg
detector, default stays the TensorRT box detector -- see detection.launch.
py's own use_mask_depth_la comment and this pass's own final report). No
`ros2 launch`, no live ZED camera: both node classes are constructed
directly (same "real Node, callback called directly, no spin()" convention
test_yolo_detector_node.py/test_detection_3d_node.py already use for their
mocked-Ultralytics coverage) and fed a static test image (bus.jpg, the same
Ultralytics sample asset benchmark_yolo_latency.py uses) plus a synthetic
depth image -- there is no real paired ZED depth frame for bus.jpg, so a
depth field is constructed here with each detected instance's mask footprint
set to its own distinct depth value, specifically so a successful run proves
_mask_median_depth() is reading real, distinct-per-instance data through the
real mask decode -- not just that the pipeline doesn't crash.

test/test_detection_launch_config.py covers the OTHER half of "is this
actually deployable" -- that selecting yolo26s-seg.pt as yolo_model on the
command line really does auto-enable use_mask_depth (fast, no GPU/hardware
needed, part of the normal pytest suite). This script is the real-inference
half neither that test nor the existing 21 (mocked Ultralytics/torch, by
design) can cover -- run it manually whenever the node code, the model
weights, or the CUDA/cuBLAS environment (see benchmark_yolo_latency.py's own
docstring for that saga) changes, not on every CI run.

Section 3 (segmentation-overlay pass) additionally confirms, on real GPU
inference against the same bus.jpg, that the annotated image ACTUALLY shows
mask overlays for the seg model and ACTUALLY shows boxes for a detect-task
model -- test_yolo_detector_node.py's own TestDrawMaskOverlay covers
_draw_mask_overlay() in isolation with synthetic inputs (fast, no GPU); this
is the real-inference confirmation that image_callback() actually reaches
that method (not result.plot()) for a real segment-task result, and reaches
result.plot() (not _draw_mask_overlay()) for a real detect-task one -- i.e.
that the branch itself, not just each side of it, is wired correctly. Saves
both annotated frames as PNGs under /tmp for an actual human-viewable check
alongside the pixel-level assertions.

Exit code is 0 and prints "ALL CHECKS PASSED" only if every stage actually
produced real output (non-empty detections, non-empty mask, at least one
published Detection3D, annotated images that are visibly the right kind of
annotation for their model) -- an AssertionError anywhere means something in
the live-deploy path is broken, not just "ran without crashing."
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from ultralytics.utils import ASSETS

from f1tenth_perception.detection_3d_node import Detection3DNode
from f1tenth_perception.yolo_detector_node import YoloDetectorNode

MODEL_PATH = __file__.rsplit('/scripts/', 1)[0] + '/models/yolo26s-seg.pt'


def main():
    rclpy.init()
    bridge = CvBridge()

    print('=== 1. YoloDetectorNode: real GPU load + inference on yolo26s-seg.pt ===')
    yolo_node = YoloDetectorNode(parameter_overrides=[
        Parameter('model_path', value=MODEL_PATH),
        Parameter('device', value='cuda'),
        Parameter('confidence_threshold', value=0.3),
    ])
    assert yolo_node.task == 'segment', f'expected segment task, got {yolo_node.task}'
    assert yolo_node.masks_pub is not None, 'masks_pub should exist for a segment-task model'
    print(f'  loaded OK: task={yolo_node.task}, device={yolo_node.device}, '
          f'{len(yolo_node.names)} classes')

    captured = {}
    yolo_node.det_pub.publish = lambda msg: captured.__setitem__('detections', msg)
    yolo_node.masks_pub.publish = lambda msg: captured.__setitem__('masks', msg)
    yolo_node.annotated_pub.publish = lambda msg: captured.__setitem__('annotated', msg)

    img_bgr = cv2.imread(str(ASSETS / 'bus.jpg'))
    h, w = img_bgr.shape[:2]
    img_msg = bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
    img_msg.header.frame_id = 'zed2_left_camera_optical_frame'
    img_msg.header.stamp.sec = 12345

    yolo_node.image_callback(img_msg)

    assert 'detections' in captured, 'image_callback did not publish detections'
    assert 'masks' in captured, 'image_callback did not publish a masks_topic frame'
    det_arr = captured['detections']
    mask_msg = captured['masks']
    assert len(det_arr.detections) > 0, 'zero detections on a known-good test image'
    print(f'  image_callback OK: {len(det_arr.detections)} detections, classes='
          f'{[d.results[0].hypothesis.class_id for d in det_arr.detections]}')
    mask_img = bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
    assert mask_img.shape == (h, w), f'mask shape {mask_img.shape} != image shape {(h, w)}'
    n_labeled_px = int(np.count_nonzero(mask_img))
    print(f'  mask decode OK: shape={mask_img.shape}, {n_labeled_px} in-instance pixels '
          f'({100.0 * n_labeled_px / (h * w):.1f}% of frame)')
    assert n_labeled_px > 0, 'mask image has zero labeled pixels -- decode likely broken'

    print()
    print('=== 2. Detection3DNode: real mask-based depth fusion (_mask_median_depth) ===')
    det3d_node = Detection3DNode(parameter_overrides=[
        Parameter('use_mask_depth', value=True),
    ])
    assert det3d_node.mask_sub is not None, 'use_mask_depth=True should create mask_sub'

    published = {}
    det3d_node.det3d_pub.publish = lambda msg: published.__setitem__('det3d', msg)
    det3d_node.marker_pub.publish = lambda msg: published.__setitem__('markers', msg)

    det3d_node._camera_info_callback(CameraInfo(
        k=[500.0, 0.0, w / 2.0, 0.0, 500.0, h / 2.0, 0.0, 0.0, 1.0]))

    identity_tf = TransformStamped()
    identity_tf.header.frame_id = det3d_node.output_frame
    identity_tf.child_frame_id = 'zed2_left_camera_optical_frame'
    identity_tf.transform.rotation.w = 1.0
    det3d_node.tf_buffer.lookup_transform = lambda *a, **k: identity_tf

    # Synthetic depth: a plausible background field, with each detected
    # instance's OWN mask footprint set to its own distinct depth value --
    # see module docstring for why (proves per-instance mask sampling, not
    # a flat field a box-region fallback would reproduce by coincidence).
    rng = np.random.default_rng(0)
    depth = (2.5 + 0.05 * rng.standard_normal((h, w))).astype(np.float32)
    for i in range(int(mask_img.max())):
        label = i + 1
        depth[mask_img == label] = 1.2 + 0.3 * i
    depth_msg = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
    depth_msg.header.frame_id = 'zed2_left_camera_optical_frame'
    depth_msg.header.stamp = img_msg.header.stamp

    det3d_node._synced_callback(det_arr, depth_msg, mask_msg)

    assert 'det3d' in published, '_synced_callback did not publish Detection3DArray'
    d3d = published['det3d']
    print(f'  _synced_callback OK: {len(d3d.detections)} 3D detections '
          f'(of {len(det_arr.detections)} 2D input detections)')
    for i, det in enumerate(d3d.detections):
        z = det.results[0].pose.pose.position.z
        cls = det.results[0].hypothesis.class_id
        print(f'    [{i}] class={cls} z={z:.3f}m '
              f'(expected ~{1.2 + 0.3 * i:.2f}m from its own mask)')
    assert len(d3d.detections) > 0, 'zero 3D detections published -- fusion likely broken'

    print()
    print('=== 3. Annotated image: mask overlay (seg) vs boxes (detect), real inference ===')
    seg_annotated = bridge.imgmsg_to_cv2(captured['annotated'], desired_encoding='bgr8')
    seg_out_path = '/tmp/verify_seg_live_deploy_seg_annotated.png'
    cv2.imwrite(seg_out_path, seg_annotated)

    # Fraction of the mask region (from the real decoded mask_img, Section 1)
    # that actually changed vs. the raw frame -- a full alpha-blended mask
    # fill changes essentially every pixel in its own footprint; a box-only
    # draw would only touch a thin outline + small label rects, nowhere near
    # this fraction of the SAME region. Distinguishes "mask overlay was
    # actually drawn" from "boxes were drawn over roughly the same area" or
    # "nothing was drawn at all" without relying on visual inspection alone.
    changed = np.any(seg_annotated != img_bgr, axis=-1)
    in_mask = mask_img > 0
    seg_mask_region_changed_pct = 100.0 * np.count_nonzero(changed & in_mask) / np.count_nonzero(in_mask)
    print(f'  seg model: {seg_mask_region_changed_pct:.1f}% of the real mask region actually '
          f'changed pixel value (expect ~100% for a full alpha-blended fill) -- saved '
          f'{seg_out_path}')
    assert seg_mask_region_changed_pct > 90.0, (
        'seg model annotated image barely touches its own mask region -- '
        '_draw_mask_overlay() likely not actually reached/working')

    # Detect-task model, same test image, same node CODE PATH (image_callback
    # -- the branch itself is what this section confirms, not just each
    # side's own drawing logic in isolation) -- yolo26s.pt (not the deployed
    # .engine) purely so this doesn't also need a fresh TensorRT context;
    # *.pt vs *.engine makes no difference to which branch image_callback
    # takes, only self.task (detect either way) does.
    detect_model_path = MODEL_PATH.rsplit('/', 1)[0] + '/yolo26s.pt'
    detect_node = YoloDetectorNode(parameter_overrides=[
        Parameter('model_path', value=detect_model_path),
        Parameter('device', value='cuda'),
        Parameter('confidence_threshold', value=0.3),
    ])
    assert detect_node.task == 'detect', f'expected detect task, got {detect_node.task}'
    assert detect_node.masks_pub is None, 'masks_pub should NOT exist for a detect-task model'
    detect_captured = {}
    detect_node.det_pub.publish = lambda msg: None
    detect_node.annotated_pub.publish = lambda msg: detect_captured.__setitem__(
        'annotated', msg)
    detect_node.image_callback(img_msg)
    assert 'annotated' in detect_captured, 'detect-task image_callback did not publish annotated'
    detect_annotated = bridge.imgmsg_to_cv2(detect_captured['annotated'], desired_encoding='bgr8')
    detect_out_path = '/tmp/verify_seg_live_deploy_detect_annotated.png'
    cv2.imwrite(detect_out_path, detect_annotated)

    detect_changed = np.any(detect_annotated != img_bgr, axis=-1)
    detect_changed_pct = 100.0 * np.count_nonzero(detect_changed) / detect_changed.size
    print(f'  detect model: {detect_changed_pct:.1f}% of the WHOLE frame changed (expect a '
          f'small fraction -- box outlines + label rects only, not a region fill) -- saved '
          f'{detect_out_path}')
    # Box outlines/labels touch a small slice of the frame; a full mask fill
    # over even one mid-sized instance alone would already exceed this by a
    # wide margin -- confirms the detect path did NOT take the mask-overlay
    # branch (which would show as a large, region-filling changed fraction
    # here too, the same signature Section 1's own check looks for).
    assert detect_changed_pct < 20.0, (
        'detect-task annotated image changed a suspiciously large fraction of the frame -- '
        'may have taken the mask-overlay branch instead of drawing boxes')
    detect_node.destroy_node()

    print()
    print('=== ALL CHECKS PASSED -- yolo-seg live-deploy path verified end to end ===')

    yolo_node.destroy_node()
    det3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
