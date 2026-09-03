"""yolo_detector_node.py tests -- constructs a real (but never spun) rclpy
Node and calls its methods directly, same "no live topics/hardware needed,
pure in-process method calls" convention as
f1tenth_costmap/test/test_costmap_boundary_node.py (see that file's own
module docstring) and this package's own test/test_detection_3d_node.py.
Publishers are never actually sent over DDS here (no spin() anywhere in this
file) -- publish() is directly mocked so tests can assert exactly what would
have been published.

Scope: three additions this pass made to this node --
  1. _publish_masks(): the mono8 instance-index label image builder (see
     module docstring's masks_topic paragraph). Exercised directly with
     synthetic `result.masks.data`-shaped numpy arrays; no real image, camera,
     or Ultralytics inference involved.
  2. Task resolution (segment vs detect) no longer being force-set to
     'detect' for every *.pt checkpoint -- see module docstring's "Task
     resolution" paragraph. This is the actual regression this pass fixes
     (before it, pointing model_path at a *-seg.pt would have silently loaded
     it as a detect-only model), so it gets its own dedicated coverage below
     rather than being left implicit.
  3. _draw_mask_overlay()/_class_color_bgr() (segmentation-overlay pass): the
     annotated-image path for a segment-task model -- masks instead of boxes.
     Exercised directly with synthetic masks_np + Detection2D-shaped inputs,
     same granularity as _publish_masks()'s own coverage above; no fake
     Ultralytics `result.plot()`/callable-inference setup needed, since
     _draw_mask_overlay() itself has no Ultralytics dependency at all (pure
     numpy/cv2, same "pure logic, separately testable" discipline
     is_proximity_too_close.py's own _in_lidar_window/_in_front_cone already
     follow). image_callback()'s own branch (masks_np is not None -> this
     path; None -> the unchanged result.plot() box path) is NOT re-tested
     here at the full-callback level -- deliberately, see this file's own
     final report: it reuses the exact same `masks_np is not None` condition
     _publish_masks()'s own call already relies on, so no new branch
     condition exists to regress independently of a case already covered.
  4. Car's-own-LiDAR exclusion (_bbox_overlap_fraction/_mask_overlap_fraction/
     _lidar_exclusion_keep_mask): filters a detection whose box (or, when
     real mask data exists for it, its own mask -- more accurate) overlaps
     the configured exclusion rectangle above lidar_exclusion_overlap_
     threshold. The two overlap-fraction functions are pure (no rclpy/
     Ultralytics dependency) and tested directly; _lidar_exclusion_keep_mask
     is exercised against a minimal duck-typed fake `result` object (just
     enough surface -- .boxes with .xyxy, .masks.data.cpu().numpy() -- for
     the method to run; NOT a real Ultralytics Results object, same
     "don't need the real inference pipeline for this" reasoning item 3
     above already uses). image_callback()'s own call site (result = result
     [keep], re-subsetting .boxes/.masks together via Ultralytics' own
     Results.__getitem__) is NOT re-tested here -- confirmed live instead,
     empirically, against real inference output (see this pass's own final
     report) rather than mocked, since faking Results.__getitem__ itself
     would test the fake, not the real indexing behavior this relies on.

TestModelTaskResolution never imports/loads a real Ultralytics model (no GPU,
no network, no live hardware -- keeps this test file fast and independent of
what's actually installed) -- `ultralytics`/`torch` are replaced in
sys.modules with minimal fakes via monkeypatch before constructing the node,
which is exactly the module __init__ already imports them from. Each fake
YOLO() call is recorded so tests can assert what `task=` argument this node
actually passed, not just the resulting `.task` -- the two used to always
agree by coincidence (every *.pt this repo ever loaded before this pass
happened to be detect-task already) which is exactly how the bug stayed
invisible; asserting the argument directly is what would have caught it.

Run standalone: python3 -m pytest test/test_yolo_detector_node.py -v
"""

import hashlib
import sys
import types
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from f1tenth_perception.yolo_detector_node import (
    MASK_OVERLAY_ALPHA,
    MAX_MASK_INSTANCES,
    YoloDetectorNode,
    _bbox_overlap_fraction,
    _mask_overlap_fraction,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = YoloDetectorNode(parameter_overrides=overrides)
    # Never actually sent over DDS in this test file (no spin()).
    node.det_pub.publish = MagicMock()
    node.annotated_pub.publish = MagicMock()
    return node


class _FakeYoloModel:
    """Stands in for ultralytics.YOLO's return value -- just enough surface
    (.task, .names, .to()) for __init__'s load path to run to completion."""

    def __init__(self, path, task=None):
        self.path = path
        # Mirrors real Ultralytics: a *.pt checkpoint's own task metadata
        # decides when task=None is passed (see this file's fake _YOLO()
        # below); an explicit task= always wins (the *.engine case).
        self.task = task if task is not None else ('segment' if 'seg' in path else 'detect')
        self.names = {0: 'fake-class'}

    def to(self, device):
        return self


def _install_fake_ultralytics(monkeypatch, calls):
    """Replaces sys.modules['ultralytics']/['torch'] with minimal fakes,
    recording every YOLO(path, task=...) call into `calls` so tests can
    assert on the exact task argument this node passed -- see module
    docstring."""
    fake_ultralytics = types.ModuleType('ultralytics')

    def _fake_yolo(path, task=None):
        calls.append({'path': path, 'task': task})
        return _FakeYoloModel(path, task)

    fake_ultralytics.YOLO = _fake_yolo
    monkeypatch.setitem(sys.modules, 'ultralytics', fake_ultralytics)

    fake_torch = types.ModuleType('torch')
    fake_torch.cuda = types.SimpleNamespace(is_available=lambda: True)
    fake_torch.__version__ = '0.0.0-fake'
    monkeypatch.setitem(sys.modules, 'torch', fake_torch)


class TestPublishMasks:

    def test_label_image_matches_mask_indices(self):
        node = _construct_with_params()
        try:
            node.masks_pub = MagicMock()
            masks_np = np.zeros((2, 6, 6), dtype=np.float32)
            masks_np[0, 0:3, 0:3] = 1.0  # detection 0 -> label 1
            masks_np[1, 3:6, 3:6] = 1.0  # detection 1 -> label 2

            node._publish_masks(masks_np, (6, 6), Header())

            node.masks_pub.publish.assert_called_once()
            published = node.masks_pub.publish.call_args[0][0]
            label_img = CvBridge().imgmsg_to_cv2(published, desired_encoding='mono8')
            assert np.all(label_img[0:3, 0:3] == 1)
            assert np.all(label_img[3:6, 3:6] == 2)
            assert np.all(label_img[0:3, 3:6] == 0)  # background elsewhere
        finally:
            node.destroy_node()

    def test_overlapping_masks_later_index_wins(self):
        node = _construct_with_params()
        try:
            node.masks_pub = MagicMock()
            masks_np = np.zeros((2, 4, 4), dtype=np.float32)
            masks_np[0, :, :] = 1.0  # detection 0 -> label 1, whole frame
            masks_np[1, 1:3, 1:3] = 1.0  # detection 1 -> label 2, overlapping center

            node._publish_masks(masks_np, (4, 4), Header())

            published = node.masks_pub.publish.call_args[0][0]
            label_img = CvBridge().imgmsg_to_cv2(published, desired_encoding='mono8')
            assert label_img[1, 1] == 2  # overlap: later (higher-index) detection wins
            assert label_img[0, 0] == 1  # untouched region: still detection 0's label
        finally:
            node.destroy_node()

    def test_shape_mismatch_skips_publish_entirely(self):
        node = _construct_with_params()
        try:
            node.masks_pub = MagicMock()
            masks_np = np.zeros((1, 6, 6), dtype=np.float32)

            node._publish_masks(masks_np, (8, 8), Header())  # shape_hw doesn't match

            node.masks_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_instance_count_beyond_cap_only_writes_first_254(self):
        node = _construct_with_params()
        try:
            node.masks_pub = MagicMock()
            n = MAX_MASK_INSTANCES + 6
            # A 1x1 "image": every one of the n instances covers the single
            # pixel, so the final value is whatever the LAST written index
            # was -- directly exposes the cap (should stop at 254) rather
            # than wrapping a uint8 past 255 or writing index 260 (== 4 after
            # a hypothetical wraparound, which would be wrong either way).
            masks_np = np.ones((n, 1, 1), dtype=np.float32)

            node._publish_masks(masks_np, (1, 1), Header())

            published = node.masks_pub.publish.call_args[0][0]
            label_img = CvBridge().imgmsg_to_cv2(published, desired_encoding='mono8')
            assert label_img[0, 0] == MAX_MASK_INSTANCES
        finally:
            node.destroy_node()


class TestModelTaskResolution:
    """See module docstring -- this is the regression coverage for the
    "*.pt task no longer force-set to 'detect'" fix."""

    def test_seg_pt_checkpoint_task_is_not_forced_and_masks_pub_created(self, monkeypatch):
        calls = []
        _install_fake_ultralytics(monkeypatch, calls)
        node = _construct_with_params(
            {'model_path': '/fake/models/yolo26s-seg.pt', 'device': 'cuda'})
        try:
            assert calls[0]['task'] is None  # not forced to 'detect'
            assert node.task == 'segment'
            assert node.masks_pub is not None
        finally:
            node.destroy_node()

    def test_detect_pt_checkpoint_still_resolves_detect_no_masks_pub(self, monkeypatch):
        calls = []
        _install_fake_ultralytics(monkeypatch, calls)
        node = _construct_with_params(
            {'model_path': '/fake/models/yolo26s.pt', 'device': 'cuda'})
        try:
            assert calls[0]['task'] is None
            assert node.task == 'detect'
            assert node.masks_pub is None
        finally:
            node.destroy_node()

    def test_engine_task_defaults_to_detect_matching_deployed_default(self, monkeypatch):
        calls = []
        _install_fake_ultralytics(monkeypatch, calls)
        node = _construct_with_params(
            {'model_path': '/fake/models/yolo26s.engine', 'device': 'cuda'})
        try:
            assert calls[0]['task'] == 'detect'  # model_task's own default
            assert node.task == 'detect'
            assert node.masks_pub is None
        finally:
            node.destroy_node()

    def test_engine_task_honors_model_task_override(self, monkeypatch):
        calls = []
        _install_fake_ultralytics(monkeypatch, calls)
        node = _construct_with_params({
            'model_path': '/fake/models/yolo26s-seg.engine', 'device': 'cuda',
            'model_task': 'segment'})
        try:
            assert calls[0]['task'] == 'segment'
            assert node.task == 'segment'
            assert node.masks_pub is not None
        finally:
            node.destroy_node()


def _fake_detection2d(class_id, score):
    det = Detection2D()
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = float(score)
    det.results.append(hyp)
    return det


class TestClassColorBgr:
    """_class_color_bgr() -- see its own docstring: must exactly mirror
    detection_3d_node.py's own _class_color() (same MD5-of-class-name
    formula, same digest-byte -> channel mapping), just reordered/rescaled
    for cv2's BGR-uint8 convention instead of ROS Marker's RGB-float one."""

    def test_matches_detection_3d_node_own_formula_reordered_for_bgr(self):
        digest = hashlib.md5(b'person').digest()
        expected_bgr = (int(digest[2]), int(digest[1]), int(digest[0]))
        assert YoloDetectorNode._class_color_bgr('person') == expected_bgr

    def test_deterministic_same_class_same_color_every_call(self):
        assert (YoloDetectorNode._class_color_bgr('bus')
                == YoloDetectorNode._class_color_bgr('bus'))

    def test_different_classes_very_likely_get_different_colors(self):
        # Not a hash-collision guarantee (impossible to guarantee for any
        # hash), just confirms this specific, permanently-fixed pair (used
        # by TestDrawMaskOverlay's own multi-class test below) differs.
        assert (YoloDetectorNode._class_color_bgr('person')
                != YoloDetectorNode._class_color_bgr('bus'))


class TestDrawMaskOverlay:
    """_draw_mask_overlay(): the segment-task annotated-image path (masks
    drawn instead of boxes) -- see module docstring's Scope item 3. Pure
    numpy/cv2, no Ultralytics/rclpy dependency in the method itself, but
    exercised here as a real bound method on a real (unspun) node, same
    "construct the real Node, call the method directly" convention this
    whole file already uses.

    Test canvases are realistically sized (200x200+, matching real camera
    frame proportions, not a tiny handful of pixels) -- a "person (0.90)"
    label is roughly 90-100px wide at the drawing font/scale used here, so a
    too-small canvas makes every label rectangle collide with itself/its
    neighbor/the frame edge regardless of correct code (this bit a first
    draft of this test file directly: two same-canvas, far-apart-looking
    mask blocks on a 40x40 canvas produced identical sampled colors because
    both labels' rectangles, each individually clamped-but-still-90px-wide
    on a 40px-wide canvas, fully overlapped -- not a real bug, but real
    enough to also motivate clamping the label rectangle's bottom-right
    corner in the method itself, not just its top-left, so a mask that IS
    genuinely close to a frame edge or another instance in a real, properly-
    sized image still degrades gracefully). Mask blocks below keep clear
    vertical headroom above them (for the label) and enough horizontal
    separation that neither label's rectangle reaches the other block.
    """

    def test_masked_pixels_are_tinted_background_pixels_are_untouched(self):
        node = _construct_with_params()
        try:
            cv_image = np.full((200, 200, 3), 100, dtype=np.uint8)  # flat gray frame
            masks_np = np.zeros((1, 200, 200), dtype=np.float32)
            masks_np[0, 80:150, 60:140] = 1.0  # one instance, plenty of headroom above
            detections = [_fake_detection2d('person', 0.9)]

            annotated = node._draw_mask_overlay(cv_image, masks_np, detections)

            assert annotated.shape == cv_image.shape
            assert annotated.dtype == cv_image.dtype
            # Corner of the frame: outside both the mask and its label.
            assert np.array_equal(annotated[5, 5], cv_image[5, 5])
            # Bottom-center of the mask block: inside the mask, well below
            # the label (anchored above row 80) -- must be tinted.
            assert not np.array_equal(annotated[145, 100], cv_image[145, 100])
        finally:
            node.destroy_node()

    def test_empty_mask_for_a_detection_is_skipped_without_crashing(self):
        node = _construct_with_params()
        try:
            cv_image = np.full((10, 10, 3), 50, dtype=np.uint8)
            masks_np = np.zeros((1, 10, 10), dtype=np.float32)  # no pixels set at all
            detections = [_fake_detection2d('person', 0.5)]

            annotated = node._draw_mask_overlay(cv_image, masks_np, detections)

            assert np.array_equal(annotated, cv_image)  # nothing drawn, no crash
        finally:
            node.destroy_node()

    def test_two_instances_of_different_classes_get_different_colors(self):
        node = _construct_with_params()
        try:
            cv_image = np.zeros((150, 300, 3), dtype=np.uint8)
            masks_np = np.zeros((2, 150, 300), dtype=np.float32)
            masks_np[0, 50:100, 20:70] = 1.0    # left block, centroid x=45
            masks_np[1, 50:100, 150:200] = 1.0  # right block, centroid x=175 -- 130px away
            detections = [
                _fake_detection2d('person', 0.9),
                _fake_detection2d('bus', 0.8),
            ]

            annotated = node._draw_mask_overlay(cv_image, masks_np, detections)

            color_a = annotated[90, 45].astype(np.int32)    # inside block 0, below its label
            color_b = annotated[90, 175].astype(np.int32)   # inside block 1, below its label
            assert not np.array_equal(color_a, color_b)
            # Background here is pure black (cv_image is all zeros), so each
            # sampled (blended) pixel is exactly MASK_OVERLAY_ALPHA times its
            # own class's raw _class_color_bgr -- confirms which color went
            # where, not just that the two differ. atol=1 for cv2's own
            # rounding in addWeighted's saturate_cast.
            expected_a = np.array(YoloDetectorNode._class_color_bgr('person')) * MASK_OVERLAY_ALPHA
            expected_b = np.array(YoloDetectorNode._class_color_bgr('bus')) * MASK_OVERLAY_ALPHA
            assert np.allclose(color_a, expected_a, atol=1)
            assert np.allclose(color_b, expected_b, atol=1)
        finally:
            node.destroy_node()

    def test_two_instances_of_the_same_class_get_the_same_color(self):
        node = _construct_with_params()
        try:
            cv_image = np.zeros((150, 300, 3), dtype=np.uint8)
            masks_np = np.zeros((2, 150, 300), dtype=np.float32)
            masks_np[0, 50:100, 20:70] = 1.0
            masks_np[1, 50:100, 150:200] = 1.0
            detections = [
                _fake_detection2d('person', 0.9),
                _fake_detection2d('person', 0.4),  # same class, different confidence
            ]

            annotated = node._draw_mask_overlay(cv_image, masks_np, detections)

            color_a = tuple(int(v) for v in annotated[90, 45])
            color_b = tuple(int(v) for v in annotated[90, 175])
            assert color_a == color_b
        finally:
            node.destroy_node()

    def test_label_rectangle_near_a_frame_edge_stays_within_the_frame(self):
        # A mask right at the top-left corner (worst case for the top-left
        # clamp) on a small-ish frame -- regression coverage for the
        # bottom-right clamp added alongside this test file (see class
        # docstring): must not raise, and must not paint the ENTIRE frame
        # the label's color (which an unclamped bottom-right previously
        # could, on a small enough canvas relative to the label's own
        # pixel width).
        node = _construct_with_params()
        try:
            cv_image = np.zeros((60, 60, 3), dtype=np.uint8)
            masks_np = np.zeros((1, 60, 60), dtype=np.float32)
            masks_np[0, 0:10, 0:10] = 1.0  # corner-hugging instance
            detections = [_fake_detection2d('person', 0.9)]

            annotated = node._draw_mask_overlay(cv_image, masks_np, detections)

            assert annotated.shape == cv_image.shape
            # The far corner, well outside a 60x60 frame's worth of label
            # bleed from a top-left mask, must stay untouched.
            assert np.array_equal(annotated[55, 55], cv_image[55, 55])
        finally:
            node.destroy_node()


class TestBboxOverlapFraction:
    """_bbox_overlap_fraction() -- pure function, see its own docstring:
    fraction of the BOX's own area inside the exclusion rectangle, not IoU."""

    def test_box_fully_inside_exclusion_rect_is_1(self):
        assert _bbox_overlap_fraction(
            10, 10, 20, 20, ex1=0, ey1=0, ex2=100, ey2=100) == pytest.approx(1.0)

    def test_box_fully_outside_exclusion_rect_is_0(self):
        assert _bbox_overlap_fraction(
            0, 0, 10, 10, ex1=50, ey1=50, ex2=100, ey2=100) == pytest.approx(0.0)

    def test_half_the_box_overlapping_is_0_5(self):
        # Box [0,0,20,20] (area 400); exclusion rect [10,0,100,100] -- the
        # right half of the box (x in [10,20]) overlaps, area 200 -> 0.5.
        assert _bbox_overlap_fraction(
            0, 0, 20, 20, ex1=10, ey1=0, ex2=100, ey2=100) == pytest.approx(0.5)

    def test_zero_area_box_is_0_not_a_zero_division_crash(self):
        assert _bbox_overlap_fraction(
            10, 10, 10, 10, ex1=0, ey1=0, ex2=100, ey2=100) == pytest.approx(0.0)

    def test_touching_edges_not_overlapping_is_0(self):
        # Box's right edge exactly at the exclusion rect's left edge --
        # zero-area intersection, not counted as overlap.
        assert _bbox_overlap_fraction(
            0, 0, 10, 10, ex1=10, ey1=0, ex2=20, ey2=10) == pytest.approx(0.0)


class TestMaskOverlapFraction:
    """_mask_overlap_fraction() -- pure function, see its own docstring."""

    def test_mask_fully_inside_exclusion_rect_is_1(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[5:10, 5:10] = True
        assert _mask_overlap_fraction(mask, 0, 0, 20, 20) == pytest.approx(1.0)

    def test_mask_fully_outside_exclusion_rect_is_0(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[0:5, 0:5] = True
        assert _mask_overlap_fraction(mask, 10, 10, 20, 20) == pytest.approx(0.0)

    def test_half_the_mask_overlapping_is_0_5(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[0:10, 0:10] = True  # 100 pixels total
        # Exclusion rect covers only the right half (cols 5-9) of the mask.
        assert _mask_overlap_fraction(mask, 5, 0, 20, 20) == pytest.approx(0.5)

    def test_empty_mask_is_0_not_a_zero_division_crash(self):
        mask = np.zeros((20, 20), dtype=bool)
        assert _mask_overlap_fraction(mask, 0, 0, 20, 20) == pytest.approx(0.0)

    def test_exclusion_rect_outside_mask_bounds_is_0(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[0:5, 0:5] = True
        assert _mask_overlap_fraction(mask, 50, 50, 100, 100) == pytest.approx(0.0)


class _FakeBox:
    """Minimal duck-typed stand-in for one Ultralytics `Boxes` entry -- only
    the `.xyxy[0].tolist()` surface _lidar_exclusion_keep_mask() actually
    reads. NOT a real ultralytics.engine.results.Boxes -- see this class's
    own section of the module docstring for why that's not needed here."""

    def __init__(self, x1, y1, x2, y2):
        self.xyxy = [np.array([x1, y1, x2, y2], dtype=float)]


class _FakeMasksData:
    """Stands in for `result.masks.data` -- only `.cpu().numpy()` is read."""

    def __init__(self, arr):
        self._arr = arr

    def cpu(self):
        return self

    def numpy(self):
        return self._arr


class _FakeMasks:
    def __init__(self, arr):
        self.data = _FakeMasksData(arr)


class _FakeResult:
    """Minimal duck-typed stand-in for one Ultralytics `Results` object --
    only `.boxes` (list of _FakeBox, so `len()`/`enumerate()` both work) and
    `.masks` (None, or a _FakeMasks wrapping an (N,H,W) array) are read by
    _lidar_exclusion_keep_mask()."""

    def __init__(self, boxes, masks_arr=None):
        self.boxes = boxes
        self.masks = _FakeMasks(masks_arr) if masks_arr is not None else None


class TestLidarExclusionKeepMask:
    """_lidar_exclusion_keep_mask() -- the car's-own-LiDAR self-occlusion
    filter, see module docstring's Scope item 4. Most tests below construct
    with an explicit EXCLUSION_KW rectangle (deliberately different from the
    real default, so these stay stable if the real calibration is ever
    retuned) rather than relying on the node's own real default; two tests
    (test_real_default_*) exercise that real default specifically."""

    EXCLUSION_KW = {
        # Bottom-right quadrant of a 100x100-normalized frame -- an
        # arbitrary but clearly-defined rectangle for these tests, NOT the
        # real car's-own-LiDAR footprint (never live-calibrated this pass,
        # see this pass's own final report).
        'lidar_exclusion_x_min': 0.5,
        'lidar_exclusion_x_max': 1.0,
        'lidar_exclusion_y_min': 0.5,
        'lidar_exclusion_y_max': 1.0,
        'lidar_exclusion_overlap_threshold': 0.5,
    }

    def test_real_default_excludes_a_detection_deep_in_the_corner(self):
        # No override -- the node's own real default (screenshot-calibrated
        # with margin: x in [0.75, 1.0], y in [0.55, 1.0] -- see
        # stack_params.yaml's own lidar_exclusion_x_min comment), not the
        # explicit EXCLUSION_KW rectangle the other tests below use.
        node = _construct_with_params()
        try:
            result = _FakeResult(boxes=[_FakeBox(85, 90, 100, 100)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [False]
        finally:
            node.destroy_node()

    def test_default_leaves_a_detection_outside_the_corner_untouched(self):
        # Same real default as above -- a detection well clear of the
        # bottom-right corner (e.g. centered in frame, like a real object
        # elsewhere in the room) must NOT be affected.
        node = _construct_with_params()
        try:
            result = _FakeResult(boxes=[_FakeBox(30, 30, 50, 50)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [True]
        finally:
            node.destroy_node()

    def test_explicit_all_zero_override_disables_the_filter(self):
        # The filter can still be turned off entirely via an explicit
        # all-zero override (an empty rectangle -- see _bbox_overlap_
        # fraction's own docstring: always 0.0 overlap, i.e. always kept),
        # same "override explicitly ... if ever needed" escape hatch this
        # codebase already uses elsewhere (e.g. use_mask_depth).
        node = _construct_with_params({
            'lidar_exclusion_x_min': 0.0, 'lidar_exclusion_x_max': 0.0,
            'lidar_exclusion_y_min': 0.0, 'lidar_exclusion_y_max': 0.0,
        })
        try:
            result = _FakeResult(boxes=[_FakeBox(85, 90, 100, 100)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [True]
        finally:
            node.destroy_node()

    def test_detection_entirely_inside_exclusion_zone_is_dropped(self):
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            result = _FakeResult(boxes=[_FakeBox(80, 80, 100, 100)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [False]
        finally:
            node.destroy_node()

    def test_detection_entirely_outside_exclusion_zone_is_kept(self):
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            result = _FakeResult(boxes=[_FakeBox(0, 0, 20, 20)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [True]
        finally:
            node.destroy_node()

    def test_overlap_exactly_at_threshold_is_kept_not_dropped(self):
        # Box [40,50,60,100] (area 20x50=1000); exclusion rect x:[50,100]
        # y:[50,100] -- overlap is x:[50,60] i.e. half the box's width ->
        # overlap fraction exactly 0.5, equal to (not above)
        # lidar_exclusion_overlap_threshold=0.5 -- must stay KEPT (strict
        # '>' in the implementation, not '>=').
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            result = _FakeResult(boxes=[_FakeBox(40, 50, 60, 100)])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [True]
        finally:
            node.destroy_node()

    def test_mask_data_preferred_over_bbox_when_available(self):
        # Box spans the WHOLE frame (would read as ~25% bbox-overlap with
        # the bottom-right-quadrant exclusion rect, below the 0.5
        # threshold -- kept if bbox were used) but this detection's own
        # MASK sits entirely inside the exclusion zone -- must be dropped,
        # proving the mask (not the box) drove the decision.
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            mask = np.zeros((1, 100, 100), dtype=np.float32)
            mask[0, 80:100, 80:100] = 1.0
            result = _FakeResult(boxes=[_FakeBox(0, 0, 100, 100)], masks_arr=mask)
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [False]
        finally:
            node.destroy_node()

    def test_empty_mask_for_a_detection_falls_back_to_its_bbox(self):
        # This result HAS mask data overall, but THIS detection's own mask
        # slice is empty (e.g. beyond MAX_MASK_INSTANCES -- see
        # _publish_masks()) -- must fall back to its bbox, not treat an
        # empty mask as "0% overlap, always keep" independent of the box.
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            mask = np.zeros((1, 100, 100), dtype=np.float32)  # all zero -- empty
            result = _FakeResult(boxes=[_FakeBox(80, 80, 100, 100)], masks_arr=mask)
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [False]  # bbox is fully inside -> dropped
        finally:
            node.destroy_node()

    def test_multiple_detections_only_the_overlapping_one_is_dropped(self):
        node = _construct_with_params(self.EXCLUSION_KW)
        try:
            result = _FakeResult(boxes=[
                _FakeBox(0, 0, 20, 20),        # index 0: outside -- kept
                _FakeBox(80, 80, 100, 100),    # index 1: inside -- dropped
                _FakeBox(0, 80, 20, 100),      # index 2: outside (bottom-left) -- kept
            ])
            keep = node._lidar_exclusion_keep_mask(result, img_h=100, img_w=100)
            assert keep.tolist() == [True, False, True]
        finally:
            node.destroy_node()


if __name__ == '__main__':
    import sys as _sys
    _sys.exit(pytest.main([__file__, '-v']))
