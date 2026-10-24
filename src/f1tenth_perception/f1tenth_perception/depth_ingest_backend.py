"""
depth_ingest_backend.py -- where detection_3d_node and front_clearance_node get their depth.

depth_source (stack_params.yaml depth_source; the node parameter of both nodes,
read-only; detection.launch.py and the bringups depth_source:=raw|ingest, the
supervisor passing it on to the `perception` component):

  'raw'     each node subscribes the ZED depth frame itself and decodes it with
            cv_bridge (the default, the stack as it always was);
  'ingest'  f1tenth_depth_ingest_rs's depth_ingest_node (Rust, rclrs) reads the
            depth, camera_info, masks and detections ONCE and publishes what the
            two nodes read from them (output/rust/depth_ingest_design.md):
            detection_3d_node takes f1tenth_messages/DetectionDepthArray (the
            synchronized detections and each one's median depth, its source and
            pixel counts) instead of its 3-way synchronizer; front_clearance_node
            takes f1tenth_messages/DepthRoi (its ROI and floor-fit pixels, and on
            a scene tick the whole frame) instead of the depth image. Everything
            either node computes from those stays in the node, unchanged.

The ingest path's results are the raw path's, bit for bit, frame by frame
(output/rust/parity_depth_ingest.md).

TEST HOOK. With the environment variable F1TENTH_DEPTH_INGEST_INPROCESS=1, a node
running depth_source raw routes every raw frame it is handed (tests call
`_synced_callback` / `depth_callback` directly) through the same Rust code the
ingest node runs, in-process (libf1tenth_depth_ingest_rs.so through
depth_ingest_native.py), into its ingest callback. Subscriptions are those of
raw, so a wiring test pins the same thing either way; every frame test runs the
ingest path. Nothing in launch/, scripts/ or docker/ sets it.

The message builders below are the node's (src/rust/f1tenth_depth_ingest_rs/
src/main.rs: detection_message, roi_message) field for field; the in-process
adapters carry no input counters (zeros), which the consumers only read as
deltas for their yield report.
"""

import os

from f1tenth_messages.msg import DepthRoi, DetectionDepth, DetectionDepthArray
from f1tenth_params.param_defaults import get_value
import numpy as np

#: The accepted depth_source values; the first is the default.
SOURCES = ('raw', 'ingest')

#: The stack_params.yaml key (also the node parameter and launch argument name).
STACK_KEY = 'depth_source'

#: Test hook (module docstring).
ENV_INPROCESS = 'F1TENTH_DEPTH_INGEST_INPROCESS'

#: The ingest node's default output topics (its detection_depths_topic /
#: front_roi_topic parameters; the consumers' parameters of the same names).
DETECTION_DEPTHS_TOPIC = '/perception/depth_ingest/detection_depths'
FRONT_ROI_TOPIC = '/perception/depth_ingest/front_roi'

#: DetectionDepth sources that went through the mask (the resize warning's set).
MASK_SOURCES = frozenset({
    DetectionDepth.SOURCE_MASK, DetectionDepth.SOURCE_SCENE_OTHER,
    DetectionDepth.SOURCE_SCENE_FALLBACK, DetectionDepth.SOURCE_MASK_EMPTY,
    DetectionDepth.SOURCE_LABEL_OVERFLOW})


def check_source(source):
    """Raise ValueError for a value depth_source does not accept."""
    if source not in SOURCES:
        raise ValueError(f'depth_source must be one of {SOURCES}, got {source!r}')
    return source


def default_source():
    """stack_params.yaml's depth_source (raw when the key is absent)."""
    try:
        return check_source(str(get_value(STACK_KEY)))
    except KeyError:
        return SOURCES[0]


def inprocess():
    """Is the in-process test hook on?"""
    return os.environ.get(ENV_INPROCESS, '').strip() not in ('', '0', 'false', 'False')


def stamp_sec(stamp):
    """`stamp.sec + stamp.nanosec * 1e-9`, as both nodes compute it."""
    return stamp.sec + stamp.nanosec * 1e-9


def _native():
    from f1tenth_perception import depth_ingest_native
    if not depth_ingest_native.available():
        raise RuntimeError(
            f'{ENV_INPROCESS} is set but libf1tenth_depth_ingest_rs.so is not usable: '
            f'{depth_ingest_native.load_error()}. Build it (colcon build --packages-select '
            'f1tenth_depth_ingest_rs) or unset the variable.')
    return depth_ingest_native


class InProcessDetectionIngest:
    """depth_ingest_node's detection side in-process: raw messages -> DetectionDepthArray."""

    def __init__(self, center_fraction, use_mask_depth, scene_enabled, scene_labels_max_age_s,
                 scene_min_other_px, sync_slop, sync_queue_size):
        self.settings = dict(
            use_mask_depth=bool(use_mask_depth), center_fraction=float(center_fraction),
            enable_scene_labels=bool(scene_enabled),
            scene_labels_max_age_s=float(scene_labels_max_age_s),
            scene_min_other_px=int(scene_min_other_px), sync_slop=float(sync_slop),
            sync_queue_size=int(sync_queue_size))
        self._ingest = _native().DetIngest(
            center_fraction, use_mask_depth, scene_enabled, scene_labels_max_age_s,
            scene_min_other_px, sync_slop, sync_queue_size)

    def camera_info(self, msg):
        self._ingest.camera_info(msg.k)

    def scene_labels(self, msg):
        self._ingest.scene_labels(stamp_sec(msg.header.stamp), msg)

    def frame(self, det_msg, depth_msg, mask_msg=None):
        boxes = [(float(d.bbox.center.position.x), float(d.bbox.center.position.y),
                  float(d.bbox.size_x), float(d.bbox.size_y)) for d in det_msg.detections]
        frame, dets, error = self._ingest.frame(
            depth_msg, stamp_sec(depth_msg.header.stamp), mask_msg, boxes)
        return detection_depth_array(det_msg, depth_msg, mask_msg, frame, dets, error,
                                     self.settings)


def detection_depth_array(det_msg, depth_msg, mask_msg, frame, dets, error, settings,
                          counts=None):
    """The DetectionDepthArray main.rs's detection_message builds."""
    msg = DetectionDepthArray()
    msg.header.stamp = depth_msg.header.stamp
    msg.header.frame_id = depth_msg.header.frame_id
    msg.detections = det_msg
    msg.depth_status = frame['depth_status']
    msg.mask_status = frame['mask_status']
    msg.error = error
    msg.depth_width = frame['depth_width']
    msg.depth_height = frame['depth_height']
    msg.fx, msg.fy, msg.cx, msg.cy = frame['fx'], frame['fy'], frame['cx'], frame['cy']
    if mask_msg is not None:
        msg.mask_stamp = mask_msg.header.stamp
        msg.mask_encoding = str(mask_msg.encoding)
    msg.mask_width = frame['mask_width']
    msg.mask_height = frame['mask_height']
    for key, value in settings.items():
        setattr(msg, key, value)
    for key, value in (counts or {}).items():
        setattr(msg, key, int(value))
    if msg.depth_status == DetectionDepthArray.DEPTH_OK:
        for d in dets:
            dd = DetectionDepth()
            dd.source = d['source']
            dd.valid = bool(d['valid'])
            dd.depth_m = float(d['depth_m'])
            dd.box_px = d['box_px']
            dd.mask_px = d['mask_px']
            dd.other_px = d['other_px']
            msg.depths.append(dd)
    return msg


class InProcessRoiIngest:
    """depth_ingest_node's front_clearance side in-process: depth Image -> DepthRoi."""

    def __init__(self, roi_half_width_px, roi_half_height_px, roi_shift_up_px,
                 ground_removal_enabled, ground_fit_stride_px, scene_mask_enabled,
                 scene_mask_rate_hz, scene_labels_enabled, scene_rate_hz):
        self.settings = dict(
            scene_mask_enabled=bool(scene_mask_enabled),
            scene_mask_rate_hz=float(scene_mask_rate_hz),
            scene_labels_enabled=bool(scene_labels_enabled),
            scene_rate_hz=float(scene_rate_hz))
        self._ingest = _native().RoiIngest(
            roi_half_width_px, roi_half_height_px, roi_shift_up_px, ground_removal_enabled,
            ground_fit_stride_px, scene_mask_enabled, scene_mask_rate_hz, scene_labels_enabled,
            scene_rate_hz)

    def frame(self, depth_msg, now):
        out, roi, ground, error = self._ingest.frame(depth_msg, now)
        return depth_roi(depth_msg, out, roi, ground, error, self.settings)


def depth_roi(depth_msg, out, roi, ground, error, settings, depth_in=0):
    """The DepthRoi main.rs's roi_message builds."""
    import array
    msg = DepthRoi()
    msg.header.stamp = depth_msg.header.stamp
    msg.header.frame_id = depth_msg.header.frame_id
    for key, value in settings.items():
        setattr(msg, key, value)
    msg.depth_in = int(depth_in)
    if out['status'] != DepthRoi.STATUS_OK:
        msg.status = DepthRoi.STATUS_DECODE_FAILED
        msg.error = error
        return msg
    msg.width, msg.height = out['width'], out['height']
    msg.roi_y0, msg.roi_y1, msg.roi_x0, msg.roi_x1 = out['y0'], out['y1'], out['x0'], out['x1']
    # array.array('f'): rclpy's fast path for float32[], bit for bit.
    msg.roi = array.array('f', np.ascontiguousarray(roi, dtype=np.float32).tobytes())
    msg.ground_stride = out['ground_stride']
    msg.ground = array.array('f', np.ascontiguousarray(ground, dtype=np.float32).tobytes())
    msg.scene_mask_tick = bool(out['scene_mask_tick'])
    msg.scene_labels_tick = bool(out['scene_labels_tick'])
    if msg.scene_mask_tick or msg.scene_labels_tick:
        msg.frame = depth_msg
    return msg


def sparse_depth(msg, bounds):
    """
    The depth array front_clearance reads on a non-tick DepthRoi frame.

    NaN everywhere except the ROI and the floor-fit sample, which are the only
    pixels the node reads on such a frame (output/rust/depth_ingest_design.md
    1.1). Returns None when the message's pixel counts do not fit `bounds`.
    """
    h, w = int(msg.height), int(msg.width)
    depth = np.full((h, w), np.nan, dtype=np.float32)
    y0, y1, x0, x1 = bounds
    view = depth[y0:y1, x0:x1]
    roi = np.frombuffer(msg.roi, dtype=np.float32) if len(msg.roi) else np.empty(0, np.float32)
    if roi.size != view.size:
        return None
    view[...] = roi.reshape(view.shape)
    stride = int(msg.ground_stride)
    if stride > 0:
        rows = np.arange(y0, h, stride)
        cols = np.arange(x0, x1, stride)
        g = (np.frombuffer(msg.ground, dtype=np.float32) if len(msg.ground)
             else np.empty(0, np.float32))
        if g.size != rows.size * cols.size:
            return None
        if g.size:
            depth[np.ix_(rows, cols)] = g.reshape(rows.size, cols.size)
    return depth
