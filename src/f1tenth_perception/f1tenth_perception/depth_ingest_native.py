"""
ctypes binding for libf1tenth_depth_ingest_rs.so (src/rust/f1tenth_depth_ingest_rs).

WHAT IT IS. The depth ingest node's logic without ROS, behind a C ABI
(src/rust/f1tenth_depth_ingest_rs/src/ffi.rs): the per-detection depth sampling
of detection_3d_node, the Python message_filters ApproximateTimeSynchronizer,
np.median of float32, cv2's INTER_NEAREST resize, and front_clearance_node's
ROI / floor sample / tick gates -- the same Rust the node binary runs. Used by
the parity harness (tools/rust_parity/depth_ingest) and by the in-process test
hook (depth_ingest_backend.ENV_INPROCESS). A live node with depth_source:=ingest
never loads it: it receives the node's messages.

WHERE THE LIBRARY COMES FROM: ament_index,
`get_package_prefix('f1tenth_depth_ingest_rs')/lib/f1tenth_depth_ingest_rs/
libf1tenth_depth_ingest_rs.so` (cargo-ament-build's install location), or the
file named by F1TENTH_DEPTH_INGEST_LIB. The loader checks the ABI version and
the index lists of every flat result array; available() / load_error() say
whether it loaded.
"""

from __future__ import annotations

import ctypes
import os

import numpy as np

#: Must equal the library's di_abi_version().
ABI_VERSION = 1

#: Overrides the ament_index location.
LIB_ENV = 'F1TENTH_DEPTH_INGEST_LIB'

LAYOUT_FRAME_I64 = 1
LAYOUT_FRAME_F64 = 2
LAYOUT_DET_I64 = 3
LAYOUT_ROI_I64 = 4

#: The index lists this binding was written against (ffi.rs).
LAYOUTS = {
    LAYOUT_FRAME_I64: ('depth_status', 'mask_status', 'depth_width', 'depth_height',
                       'mask_width', 'mask_height', 'has_scene'),
    LAYOUT_FRAME_F64: ('fx', 'fy', 'cx', 'cy', 'scene_stamp'),
    LAYOUT_DET_I64: ('source', 'valid', 'box_px', 'mask_px', 'other_px'),
    LAYOUT_ROI_I64: ('status', 'width', 'height', 'y0', 'y1', 'x0', 'x1', 'roi_len',
                     'ground_stride', 'ground_rows', 'ground_cols', 'scene_mask_tick',
                     'scene_labels_tick'),
}

_LIB = None
_LOAD_ERROR = None
_PATH = None

_VP = ctypes.c_void_p
_D = ctypes.c_double
_L = ctypes.c_int64
_I = ctypes.c_int32
_CP = ctypes.c_char_p
#: One image: data, length, height, width, step, is_bigendian, encoding.
_IMG = [_VP, _L, _L, _L, _L, _L, _CP]


def _default_path():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('f1tenth_depth_ingest_rs'), 'lib',
                        'f1tenth_depth_ingest_rs', 'libf1tenth_depth_ingest_rs.so')


def _sig(lib, name, restype, argtypes):
    fn = getattr(lib, name)
    fn.restype = restype
    fn.argtypes = argtypes
    return fn


def _load():
    global _LIB, _LOAD_ERROR, _PATH
    if _LIB is not None or _LOAD_ERROR is not None:
        return _LIB
    try:
        path = os.environ.get(LIB_ENV) or _default_path()
        _PATH = path
        lib = ctypes.CDLL(path)
        abi = int(_sig(lib, 'di_abi_version', ctypes.c_uint32, [])())
        if abi != ABI_VERSION:
            raise RuntimeError(f'{path}: ABI version {abi}, this binding is {ABI_VERSION}')
        layout = _sig(lib, 'di_layout', _CP, [_I])
        for kind, expected in LAYOUTS.items():
            got = tuple((layout(kind) or b'').decode().split(','))
            if got != expected:
                raise RuntimeError(f'{path}: layout {kind} is {got}, binding {expected}')
        _sig(lib, 'di_version', _CP, [])
        _sig(lib, 'di_build_info', _L, [ctypes.c_char_p, _L])
        _sig(lib, 'di_median_f32', _I, [_VP, _L, _VP])
        _sig(lib, 'di_resize_nearest_u8', _I, [_VP, _L, _L, _L, _VP, _L, _L])
        _sig(lib, 'di_det_new', _VP, [_D, _I, _I, _D, _L, _D, _L, ctypes.c_char_p, _L])
        _sig(lib, 'di_det_free', None, [_VP])
        _sig(lib, 'di_det_camera_info', _I, [_VP, _VP])
        _sig(lib, 'di_det_scene_labels', _I, [_VP, _D] + _IMG)
        _sig(lib, 'di_det_frame', _I,
             [_VP] + _IMG + [_D, _I] + _IMG + [_VP, _L, _VP, _VP, _VP, _VP,
                                              ctypes.c_char_p, _L])
        _sig(lib, 'di_sync_new', _VP, [_L, _L, _D])
        _sig(lib, 'di_sync_free', None, [_VP])
        _sig(lib, 'di_sync_add', _I, [_VP, _L, _L, _L, _VP])
        _sig(lib, 'di_roi_new', _VP, [_VP, _VP])
        _sig(lib, 'di_roi_free', None, [_VP])
        _sig(lib, 'di_roi_frame', _I,
             [_VP] + _IMG + [_D, _VP, _VP, _L, _VP, _L, ctypes.c_char_p, _L])
        _LIB = lib
    except Exception as exc:                      # noqa: BLE001 - any of them
        _LOAD_ERROR = f'{type(exc).__name__}: {exc}'
    return _LIB


def available() -> bool:
    """Is the library usable here?"""
    return _load() is not None


def load_error() -> str | None:
    """Return why it is not usable (None when it is)."""
    _load()
    return _LOAD_ERROR


def library_path() -> str | None:
    """Return the file that was (or failed to be) loaded."""
    _load()
    return _PATH


def _lib():
    lib = _load()
    if lib is None:
        raise RuntimeError(f'libf1tenth_depth_ingest_rs.so is not usable: {_LOAD_ERROR}')
    return lib


def version() -> str | None:
    lib = _load()
    return None if lib is None else lib.di_version().decode()


def build_info() -> str | None:
    lib = _load()
    if lib is None:
        return None
    buf = ctypes.create_string_buffer(256)
    lib.di_build_info(buf, len(buf))
    return buf.value.decode()


# ---------------------------------------------------------------- images

def _image_args(msg):
    """The seven image arguments of a sensor_msgs/Image, and the buffer to keep alive."""
    data = msg.data
    if isinstance(data, (bytes, bytearray, memoryview)) or hasattr(data, 'buffer_info'):
        buf = np.frombuffer(data, dtype=np.uint8)
    else:
        buf = np.asarray(data, dtype=np.uint8)
    buf = np.ascontiguousarray(buf)
    args = [buf.ctypes.data if buf.size else None, int(buf.size), int(msg.height),
            int(msg.width), int(msg.step), int(msg.is_bigendian),
            str(msg.encoding).encode()]
    return args, buf


def _err_buf():
    return ctypes.create_string_buffer(512)


# ---------------------------------------------------------------- small pieces

def median_f32(values) -> np.float32 | None:
    """np.median of float32 values as the library computes it (None if empty)."""
    v = np.ascontiguousarray(values, dtype=np.float32).ravel()
    out = np.zeros(1, dtype=np.float32)
    rc = _lib().di_median_f32(v.ctypes.data if v.size else None, int(v.size), out.ctypes.data)
    if rc == 1:
        return None
    if rc != 0:
        raise ValueError(f'di_median_f32: {rc}')
    return out[0]


def resize_nearest_u8(img, height, width) -> np.ndarray:
    """cv2.resize(img, (width, height), interpolation=cv2.INTER_NEAREST) of a uint8 image."""
    src = np.ascontiguousarray(img, dtype=np.uint8)
    dst = np.zeros((int(height), int(width)), dtype=np.uint8)
    rc = _lib().di_resize_nearest_u8(src.ctypes.data, src.shape[0], src.shape[1],
                                     src.strides[0], dst.ctypes.data, dst.shape[0],
                                     dst.shape[1])
    if rc != 0:
        raise ValueError(f'di_resize_nearest_u8: {rc}')
    return dst


class Sync:
    """The message_filters ApproximateTimeSynchronizer port over message ids."""

    def __init__(self, inputs, queue_size, slop):
        self._lib = _lib()
        self.inputs = int(inputs)
        self._h = self._lib.di_sync_new(self.inputs, int(queue_size), float(slop))
        if not self._h:
            raise ValueError(f'di_sync_new({inputs}, {queue_size}, {slop}) refused')

    def __del__(self):
        h, self._h = getattr(self, '_h', None), None
        if h:
            self._lib.di_sync_free(h)

    def add(self, input_index, stamp_ns, msg_id):
        """The fired set's ids (input order) or None."""
        out = np.zeros(self.inputs, dtype=np.int64)
        rc = self._lib.di_sync_add(self._h, int(input_index), int(stamp_ns), int(msg_id),
                                   out.ctypes.data)
        if rc < 0:
            raise ValueError(f'di_sync_add: {rc}')
        return tuple(int(x) for x in out) if rc == 1 else None


# ---------------------------------------------------------------- detection side

class DetIngest:
    """detection_3d_node's depth side (ingest.rs DetectionIngest), in-process."""

    def __init__(self, center_fraction, use_mask_depth, scene_enabled, scene_labels_max_age_s,
                 scene_min_other_px, sync_slop, sync_queue_size):
        self._lib = _lib()
        err = _err_buf()
        self._h = self._lib.di_det_new(
            float(center_fraction), int(bool(use_mask_depth)), int(bool(scene_enabled)),
            float(scene_labels_max_age_s), int(scene_min_other_px), float(sync_slop),
            int(sync_queue_size), err, len(err))
        if not self._h:
            raise ValueError(err.value.decode())

    def __del__(self):
        h, self._h = getattr(self, '_h', None), None
        if h:
            self._lib.di_det_free(h)

    def camera_info(self, k):
        kk = np.ascontiguousarray([float(x) for x in list(k)[:9]], dtype=np.float64)
        if kk.size != 9:
            raise ValueError('camera_info k needs 9 values')
        self._lib.di_det_camera_info(self._h, kk.ctypes.data)

    def scene_labels(self, stamp, msg):
        args, keep = _image_args(msg)
        self._lib.di_det_scene_labels(self._h, float(stamp), *args)
        del keep

    def frame(self, depth_msg, depth_stamp, mask_msg, boxes):
        """One synchronized set: (frame dict, per-detection dicts, error text)."""
        d_args, d_keep = _image_args(depth_msg)
        if mask_msg is not None:
            m_args, m_keep = _image_args(mask_msg)
            has_mask = 1
        else:
            m_args, m_keep = [None, 0, 0, 0, 0, 0, b''], None
            has_mask = 0
        bx = np.ascontiguousarray(boxes, dtype=np.float64).reshape(-1, 4)
        n = bx.shape[0]
        fi = np.zeros(len(LAYOUTS[LAYOUT_FRAME_I64]), dtype=np.int64)
        ff = np.zeros(len(LAYOUTS[LAYOUT_FRAME_F64]), dtype=np.float64)
        di = np.zeros((max(n, 1), len(LAYOUTS[LAYOUT_DET_I64])), dtype=np.int64)
        df = np.zeros(max(n, 1), dtype=np.float32)
        err = _err_buf()
        rc = self._lib.di_det_frame(
            self._h, *d_args, float(depth_stamp), has_mask, *m_args,
            bx.ctypes.data if n else None, n, fi.ctypes.data, ff.ctypes.data,
            di.ctypes.data, df.ctypes.data, err, len(err))
        del d_keep, m_keep
        if rc != 0:
            raise ValueError(f'di_det_frame: {rc}')
        frame = dict(zip(LAYOUTS[LAYOUT_FRAME_I64], (int(x) for x in fi)))
        frame.update(zip(LAYOUTS[LAYOUT_FRAME_F64], (float(x) for x in ff)))
        dets = []
        for k in range(n):
            row = dict(zip(LAYOUTS[LAYOUT_DET_I64], (int(x) for x in di[k])))
            row['depth_m'] = df[k]
            dets.append(row)
        return frame, dets, err.value.decode()


# ---------------------------------------------------------------- front_clearance side

class RoiIngest:
    """front_clearance_node's depth side (ingest.rs RoiIngest), in-process."""

    def __init__(self, roi_half_width_px, roi_half_height_px, roi_shift_up_px,
                 ground_removal_enabled, ground_fit_stride_px, scene_mask_enabled,
                 scene_mask_rate_hz, scene_labels_enabled, scene_rate_hz):
        self._lib = _lib()
        ints = np.array([int(roi_half_width_px), int(roi_half_height_px), int(roi_shift_up_px),
                         int(bool(ground_removal_enabled)), int(ground_fit_stride_px),
                         int(bool(scene_mask_enabled)), int(bool(scene_labels_enabled))],
                        dtype=np.int64)
        rates = np.array([float(scene_mask_rate_hz), float(scene_rate_hz)], dtype=np.float64)
        self._h = self._lib.di_roi_new(ints.ctypes.data, rates.ctypes.data)
        if not self._h:
            raise ValueError('di_roi_new refused')

    def __del__(self):
        h, self._h = getattr(self, '_h', None), None
        if h:
            self._lib.di_roi_free(h)

    def frame(self, depth_msg, now):
        """(out dict, roi float32 array, ground float32 array, error text)."""
        args, keep = _image_args(depth_msg)
        cap = max(int(depth_msg.height) * int(depth_msg.width), 1)
        out = np.zeros(len(LAYOUTS[LAYOUT_ROI_I64]), dtype=np.int64)
        roi = np.empty(cap, dtype=np.float32)
        ground = np.empty(cap, dtype=np.float32)
        err = _err_buf()
        rc = self._lib.di_roi_frame(self._h, *args, float(now), out.ctypes.data,
                                    roi.ctypes.data, cap, ground.ctypes.data, cap, err, len(err))
        del keep
        if rc != 0:
            raise ValueError(f'di_roi_frame: {rc}')
        o = dict(zip(LAYOUTS[LAYOUT_ROI_I64], (int(x) for x in out)))
        n_ground = o['ground_rows'] * o['ground_cols']
        return o, roi[:o['roi_len']].copy(), ground[:n_ground].copy(), err.value.decode()
