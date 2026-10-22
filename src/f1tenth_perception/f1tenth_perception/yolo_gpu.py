"""Pure helpers for yolo_detector_node's CPU-cost work (no rclpy import).

- rate_gate: the stamp-driven frame drop at max_rate_hz.
- set_cuda_blocking_sync / cuda_primary_ctx_flags: CU_CTX_SCHED_BLOCKING_SYNC
  on the device's primary context through the CUDA driver API (ctypes), so a
  thread waiting for the GPU sleeps instead of spinning a core.
- engine_path_for: the TensorRT engine file name the node looks for and
  scripts/export_yolo_engine.sh writes.

torch is imported inside the functions that need it, so this module imports
without torch (the node's tests replace torch with a fake module).
"""

import ctypes
import os

# cuda.h: CUcontext scheduling flags.
CU_CTX_SCHED_SPIN = 0x01
CU_CTX_SCHED_YIELD = 0x02
CU_CTX_SCHED_BLOCKING_SYNC = 0x04
CU_CTX_SCHED_MASK = 0x07

# Fraction of the period a frame may arrive early and still pass the gate:
# camera stamps jitter by a few ms, and without it a 30 Hz stream gated to
# 15 Hz would drop to 10 Hz whenever the second frame lands just short.
RATE_GATE_TOLERANCE = 0.1


def rate_gate(stamp_sec, next_due_sec, period_sec, tolerance=RATE_GATE_TOLERANCE):
    """
    Return (process, next_due) for the frame drop at max_rate_hz.

    perception_debug.tick_due semantics (the deadline advances from the
    previous deadline, resets after a gap or a backwards stamp) plus an early
    tolerance of `tolerance` * period. period_sec <= 0 lets every frame pass.
    """
    if period_sec <= 0.0:
        return True, next_due_sec
    if (next_due_sec is None or stamp_sec - next_due_sec > period_sec
            or stamp_sec < next_due_sec - period_sec):
        return True, stamp_sec + period_sec
    if stamp_sec + tolerance * period_sec >= next_due_sec:
        return True, next_due_sec + period_sec
    return False, next_due_sec


def _libcuda():
    return ctypes.CDLL('libcuda.so.1')


def set_cuda_blocking_sync(device_index=0):
    """
    Set CU_CTX_SCHED_BLOCKING_SYNC on the primary context of device_index.

    Call before torch creates its CUDA context (the first CUDA op). Returns
    (ok, message); never raises.
    """
    try:
        cuda = _libcuda()
        dev = ctypes.c_int()
        rc = cuda.cuInit(0)
        if rc != 0:
            return False, f'cuInit returned {rc}'
        rc = cuda.cuDeviceGet(ctypes.byref(dev), int(device_index))
        if rc != 0:
            return False, f'cuDeviceGet({device_index}) returned {rc}'
        set_flags = getattr(cuda, 'cuDevicePrimaryCtxSetFlags_v2', None) \
            or cuda.cuDevicePrimaryCtxSetFlags
        rc = set_flags(dev, ctypes.c_uint(CU_CTX_SCHED_BLOCKING_SYNC))
        if rc != 0:
            return False, f'cuDevicePrimaryCtxSetFlags returned {rc}'
        return True, 'CU_CTX_SCHED_BLOCKING_SYNC set'
    except Exception as exc:  # noqa: BLE001 - optional optimisation
        return False, str(exc)


def cuda_primary_ctx_flags(device_index=0):
    """Return (flags, active) of the primary context, or (None, None)."""
    try:
        cuda = _libcuda()
        dev, flags, active = ctypes.c_int(), ctypes.c_uint(), ctypes.c_int()
        if cuda.cuDeviceGet(ctypes.byref(dev), int(device_index)) != 0:
            return None, None
        if cuda.cuDevicePrimaryCtxGetState(dev, ctypes.byref(flags), ctypes.byref(active)) != 0:
            return None, None
        return int(flags.value), bool(active.value)
    except Exception:  # noqa: BLE001 - diagnostics only
        return None, None


def cuda_device_index(device):
    """'cuda' / 'cuda:1' / '0' -> device index, None for anything else (cpu)."""
    device = str(device).strip().lower()
    if device == 'cuda':
        return 0
    if device.startswith('cuda:'):
        device = device[len('cuda:'):]
    return int(device) if device.isdigit() else None


def engine_path_for(model_path, trt_version):
    """<dir>/<stem>.trt<trt_version>.fp16.engine next to model_path."""
    stem = os.path.splitext(os.path.basename(model_path))[0]
    return os.path.join(os.path.dirname(model_path), f'{stem}.trt{trt_version}.fp16.engine')


# ---- GPU pipeline (gpu_pipeline:=true) -----------------------------------
# Mirrors Ultralytics 8.4.90's predictor (engine/predictor.py preprocess and
# pre_transform, data/augment.py LetterBox, models/yolo/{detect,segment}/
# predict.py postprocess) on tensors that stay on the GPU.

# sensor_msgs/Image encodings the GPU path reads: channels and the index of
# R, G, B in each pixel.
ENCODING_RGB_INDEX = {
    'bgra8': (4, (2, 1, 0)),
    'bgr8': (3, (2, 1, 0)),
    'rgba8': (4, (0, 1, 2)),
    'rgb8': (3, (0, 1, 2)),
}


def letterbox_params(h, w, new_shape, auto, stride, scaleup=True):
    """LetterBox.get_params (center=True): ((unpad_w, unpad_h), top, bottom, left, right)."""
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / h, new_shape[1] / w)
    if not scaleup:
        r = min(r, 1.0)
    new_unpad = (round(w * r), round(h * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = dw % stride, dh % stride
    dw, dh = dw / 2, dh / 2
    top, bottom = round(dh - 0.1), round(dh + 0.1)
    left, right = round(dw - 0.1), round(dw + 0.1)
    return new_unpad, top, bottom, left, right


def image_view(msg_data, height, width, step, encoding):
    """(H, W, C) uint8 numpy view of an Image message's data, or None if unsupported."""
    import numpy as np
    spec = ENCODING_RGB_INDEX.get(encoding)
    if spec is None:
        return None
    channels = spec[0]
    flat = np.frombuffer(msg_data, dtype=np.uint8)
    if step < width * channels or flat.size < step * height:
        return None
    return flat[:step * height].reshape(height, step)[:, :width * channels].reshape(
        height, width, channels)


def to_model_input(frame, encoding, device, new_shape, auto, stride, half, pad_value=114):
    """
    Upload a (H, W, C) uint8 frame once and build the model input on device.

    Same tensor as the predictor builds from the BGR frame: RGB, letterboxed
    with pad_value, (1, 3, H', W'), half or float, / 255. A resize (only when
    the frame is not already at the letterbox size) is bilinear rounded back
    to uint8 levels, close to but not bit-identical with cv2.INTER_LINEAR.
    Returns (input tensor, rgb uint8 (3, H, W) on device).
    """
    import torch
    import torch.nn.functional as F
    h, w = frame.shape[:2]
    rgb_index = list(ENCODING_RGB_INDEX[encoding][1])
    hwc = torch.from_numpy(frame).to(device)
    rgb = hwc[:, :, rgb_index].permute(2, 0, 1)
    (uw, uh), top, bottom, left, right = letterbox_params(h, w, new_shape, auto, stride)
    img = rgb
    if (uw, uh) != (w, h):
        img = F.interpolate(img[None].float(), size=(uh, uw), mode='bilinear',
                            align_corners=False)[0].round_().clamp_(0, 255).to(torch.uint8)
    out = torch.full((3, uh + top + bottom, uw + left + right), pad_value,
                     dtype=torch.uint8, device=img.device)
    out[:, top:top + uh, left:left + uw] = img
    out = out[None]
    out = out.half() if half else out.float()
    out /= 255
    return out, rgb


def postprocess(predictor, preds, im_shape, orig_hw, conf, task):
    """
    Return detections (N, 6+), boxes in frame pixels, and masks (N, H, W) uint8 on device.

    DetectionPredictor/SegmentationPredictor.postprocess + construct_result
    with retina_masks=True, minus the Results objects; masks is None for a
    detect model or an empty frame, like result.masks.
    """
    from ultralytics.utils import nms, ops
    model, args = predictor.model, predictor.args
    protos = None
    if task == 'segment':
        protos = preds[0][1] if isinstance(preds[0], tuple) else preds[1]
        preds = preds[0]
    det = nms.non_max_suppression(
        preds, conf, args.iou, args.classes, args.agnostic_nms, max_det=args.max_det,
        nc=0 if task == 'detect' else len(model.names),
        end2end=getattr(model, 'end2end', False))[0]
    if det.shape[0] == 0 or protos is None:
        if det.shape[0]:
            det[:, :4] = ops.scale_boxes(im_shape, det[:, :4], orig_hw)
        return det, None
    det[:, :4] = ops.scale_boxes(im_shape, det[:, :4], orig_hw)
    masks = ops.process_mask_native(protos[0], det[:, 6:], det[:, :4], orig_hw)
    # Ultralytics keeps only detections whose mask is not empty.
    keep = masks.amax((-2, -1)) > 0
    det, masks = det[keep], masks[keep]
    return det, masks


def mask_stats(masks, rect):
    """Per-instance (total, inside rect) mask pixel counts as float32 on device.

    rect = (cx1, cy1, cx2, cy2) already clamped ints, as _mask_overlap_fraction does.
    """
    import torch
    total = masks.sum((1, 2), dtype=torch.float32)
    cx1, cy1, cx2, cy2 = rect
    if cx2 <= cx1 or cy2 <= cy1:
        inside = torch.zeros_like(total)
    else:
        inside = masks[:, cy1:cy2, cx1:cx2].sum((1, 2), dtype=torch.float32)
    return total, inside


def label_image(masks):
    """mono8 label image (H, W) on device: pixel = i + 1 of the last mask i covering it."""
    import torch
    n = masks.shape[0]
    labels = torch.arange(1, n + 1, dtype=torch.uint8, device=masks.device)
    return (masks * labels[:, None, None]).amax(0)
