"""
ctypes binding for f1tenth_wall_tracker_core (src/rust/f1tenth_wall_tracker_core).

WHAT IT IS. libf1tenth_wall_tracker_core.so is a ROS-free Rust library with a C
ABI (src/rust/f1tenth_wall_tracker_core/src/ffi.rs): multi_wall_tracker's
MultiWallTracker (state machine, association, extent, roles, FRONT) behind a
handle, and wall_fusion's per-cycle functions, ported operation for operation
so their results are bit-identical to the Python on linus (parity:
tools/rust_parity/wall_tracker, output/rust/parity_wall_tracker.md).
wall_tracker_backend.py is the dispatcher and the Python-facing class; nothing
else should call this module.

WHY ctypes. The extraction_c.py / mpc_native.py reasoning: a few flat-array
functions do not justify an extension module. The price is that the layouts
are asserted here, not by a compiler: the loader checks the ABI version, every
struct's field list and size, and the parameter name lists against the ones
written below, and refuses a mismatch.

WHERE THE LIBRARY COMES FROM: ament_index,
`get_package_prefix('f1tenth_wall_tracker_core')/lib/f1tenth_wall_tracker_core/
libf1tenth_wall_tracker_core.so` (cargo-ament-build's install location). The
environment variable F1TENTH_WALL_TRACKER_CORE_LIB, when set, names the file
instead (the parity harness uses it for a fresh `cargo build`); the path
actually loaded is in library_path(). If nothing loads, available() is False and
load_error() says why; wall_tracker_impl:=rust then refuses to start.

THREAD SAFETY. A handle serializes its own calls (a mutex in the library);
ctypes releases the GIL during a call.
"""

from __future__ import annotations

import ctypes
import os

import numpy as np

#: Must equal the library's f1tenth_wall_tracker_core_abi_version().
ABI_VERSION = 1

#: Overrides the ament_index location (see the module docstring).
LIB_ENV = 'F1TENTH_WALL_TRACKER_CORE_LIB'

STRUCT_OBSERVATION = 1
STRUCT_LINE = 2
STRUCT_TRACK = 3
STRUCT_SCALARS = 4
STRUCT_REJECTION = 5
STRUCT_SIZES = 6
STRUCT_FUSION_IN = 7
STRUCT_FUSION_OBS = 8
STRUCT_RECT_OUT = 9

#: The struct layouts this binding was written against (ffi.rs). Every field
#: is 8 bytes; 'f' float64, 'i' int64.
FIELDS = {
    STRUCT_OBSERVATION: (
        'nx:f,ny:f,d:f,t0:f,t1:f,inliers:i,rms:f,z_min:f,z_max:f,height_measured:i,'
        'source:i,confidence:f,stereo_weight:f'),
    STRUCT_LINE: 'rho:f,alpha:f,cov00:f,cov01:f,cov10:f,cov11:f,sx:f,sy:f,ex:f,ey:f,n_points:i',
    STRUCT_TRACK: (
        'id:i,hits:i,state:i,source:i,verify:i,role:i,strip_verdict:i,rect_reject_streak:i,'
        'miss:i,strip_miss:i,ever_confirmed:i,has_lidar:i,has_both:i,height_measured:i,glass:i,'
        'rect_informative:i,occluded:i,contradicted:i,x0:f,x1:f,p00:f,p01:f,p10:f,p11:f,t0:f,'
        't1:f,born:f,last_seen:f,z_min:f,z_max:f,confidence:f,rect_coverage:f,rect_support:f,'
        'rect_beyond_share:f,odo_at_last_seen:f,strip_pred_m:f,confirm_maxlen:i,confirm_off:i,'
        'confirm_len:i,lidar_maxlen:i,lidar_off:i,lidar_len:i,conf_maxlen:i,conf_off:i,'
        'conf_len:i,ext_maxlen:i,ext_off:i,ext_len:i,hash:i'),
    STRUCT_SCALARS: (
        'laser_x:f,laser_y:f,has_laser:i,camera_x:f,camera_y:f,has_camera:i,camera_fresh:i,'
        'next_id:i,last_stamp:f,has_last_stamp:i,last_yaw:f,has_last_yaw:i,odometer:f,'
        'last_car_x:f,last_car_y:f,has_last_car_t:i,n_short_dropped:i,n_negative_killed:i,'
        'n_gap_rejected:i,n_merge_gap_refused:i,n_stereo_near_rejected:i,n_camera_rejected:i,'
        'strip_range_m:f,strip_has_range:i,strip_n_returns:i,strip_n_cut:i,'
        'strip_n_no_return:i,front_id:i,front_has_id:i,front_reason:i,front_distance_m:f,'
        'n_rejections:i,n_front_rejects:i,n_tracks:i'),
    STRUCT_REJECTION: 'code:i,has_id:i,id:i,value:f',
    STRUCT_SIZES: 'n_tracks:i,n_confirm:i,n_lidar:i,n_conf:i,n_ext:i',
    STRUCT_FUSION_IN: (
        'fx:f,fy:f,cx:f,cy:f,r00:f,r01:f,r02:f,r10:f,r11:f,r12:f,r20:f,r21:f,r22:f,t0:f,t1:f,'
        't2:f,laser_z:f,width:i,has_comp:i,scan_x:f,scan_y:f,scan_theta:f,image_x:f,image_y:f,'
        'image_theta:f,tol_base:f,tol_quad:f,disagree_reject_frac:f,stereo_weight:f,inlier_ref:f,'
        'rms_ref:f,range_ref_m:f,min_inlier_ref:f,max_rms_scale:f,min_span_columns:i,inlier_m:f,'
        'min_points:i,min_span_m:f,default_height:f,max_pairs:i'),
    STRUCT_FUSION_OBS: (
        'line_index:i,nx:f,ny:f,d:f,t0:f,t1:f,inliers:i,rms:f,z_min:f,z_max:f,height_measured:i,'
        'source:i,confidence:f,col_begin:i,col_end:i,n_agree:i,n_lidar_only:i,n_stereo_only:i,'
        'n_disagree:i,stereo_weight:f'),
    STRUCT_RECT_OUT: (
        'coverage:f,support:f,beyond_share:f,n_samples:i,n_in_image:i,at:f,beyond:f,against:f'),
}

#: TrackerParams fields as the library takes them: float64 and int64 arrays.
PARAM_F64 = (
    'process_noise_rho', 'process_noise_theta', 'k_yaw', 'meas_sigma_rho_min',
    'meas_sigma_theta_min', 'gate_chi2', 'collinear_gap_m', 'extent_decay_s',
    'confirm_min_length_m', 'confirm_max_dist_jitter_m', 'confirm_max_angle_jitter_rad',
    'delete_after_s', 'max_sigma_rho_m', 'label_half_sector_rad', 'role_hysteresis_rad',
    'ray_margin_m', 'front_max_coast_s', 'front_max_coast_m', 'lidar_sigma_base_m',
    'lidar_sigma_per_m', 'stereo_sigma_base_m', 'stereo_sigma_quad_per_m2',
    'min_span_for_angle_m', 'min_span_birth_m', 'min_span_update_m', 'front_x_m',
    'side_edge_half_width_m', 'front_strip_margin_m', 'stereo_front_max_normal_rad',
    'front_min_confidence', 'front_contradict_margin_m', 'stereo_min_rho_m',
    'camera_seg_min_ray_angle_rad', 'camera_seg_min_bearing_span_rad', 'stereo_max_rms_m',
    'stereo_min_vertical_extent_m', 'stereo_min_length_m', 'back_x_m', 'side_x_min_m',
    'side_x_max_m', 'side_max_normal_rad', 'camera_side_x_max_m', 'camera_side_max_normal_rad',
    'camera_side_offset_min_m', 'camera_side_offset_max_m', 'role_hysteresis_m',
    'occlusion_margin_m', 'negative_tol_m', 'negative_pass_frac', 'coast_max_odom_m')
PARAM_I64 = (
    'confirm_min_consecutive_hits', 'confirm_require_lidar', 'max_published',
    'front_lidar_recent_cycles', 'front_contradict_cycles', 'occlusion_points',
    'occlusion_min_blocked', 'negative_shrink_beams', 'negative_min_beams',
    'negative_miss_to_kill')
#: wall_fusion.RectParams' decision part, which the tracker's verify state reads.
RECT_F64 = ('coverage_min', 'support_verify', 'support_reject', 'glass_beyond_share',
            'glass_at_share')
RECT_I64 = ('reject_cycles',)

WT_OK = 0
WT_ERR_SINGULAR = -2
WT_ERR_LSA_INVALID = -3
WT_ERR_LSA_INFEASIBLE = -4
WT_ERR_CAPACITY = -6
WT_ERR_OVERFLOW = -10
_ERRORS = {
    -1: 'null pointer',
    -2: 'Singular matrix',
    -3: 'matrix contains invalid numeric entries',
    -4: 'cost matrix is infeasible',
    -5: 'panic in f1tenth_wall_tracker_core (a bug)',
    -6: 'buffer too small',
    -7: 'parameter arrays of the wrong length',
    -8: 'a tracker state code no Python value maps to',
    -9: 'length or offset out of range',
    -10: 'cannot convert float infinity to integer',
}

_LIB = None
_LOAD_ERROR = None
_PATH = None
#: kind -> numpy dtype, built from FIELDS once the library agreed to them.
DTYPES = {}

_DP = ctypes.POINTER(ctypes.c_double)
_LP = ctypes.POINTER(ctypes.c_int64)
_VP = ctypes.c_void_p
_D = ctypes.c_double
_L = ctypes.c_int64
_I = ctypes.c_int32


def _dtype(fields):
    return np.dtype([(name, '<f8' if kind == 'f' else '<i8')
                     for name, kind in (f.split(':') for f in fields.split(','))])


def _default_path():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('f1tenth_wall_tracker_core'), 'lib',
                        'f1tenth_wall_tracker_core', 'libf1tenth_wall_tracker_core.so')


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
        abi = int(_sig(lib, 'f1tenth_wall_tracker_core_abi_version', ctypes.c_uint32, [])())
        if abi != ABI_VERSION:
            raise RuntimeError(f'{path}: ABI version {abi}, this binding is {ABI_VERSION}')
        size = _sig(lib, 'f1tenth_wall_tracker_core_struct_size', _L, [_I])
        fields = _sig(lib, 'f1tenth_wall_tracker_core_fields', ctypes.c_char_p, [_I])
        for kind, expected in FIELDS.items():
            got = fields(kind).decode()
            if got != expected:
                raise RuntimeError(f'{path}: struct {kind} fields {got!r}, binding {expected!r}')
            dt = _dtype(expected)
            if int(size(kind)) != dt.itemsize:
                raise RuntimeError(f'{path}: struct {kind} is {int(size(kind))} bytes, the '
                                   f'binding {dt.itemsize}')
            DTYPES[kind] = dt
        names = _sig(lib, 'wt_param_names', ctypes.c_char_p, [_I])
        for kind, expected in enumerate((PARAM_F64, PARAM_I64, RECT_F64, RECT_I64)):
            got = tuple(names(kind).decode().split(','))
            if got != expected:
                raise RuntimeError(f'{path}: parameter list {kind} is {got}, binding {expected}')
        _sig(lib, 'f1tenth_wall_tracker_core_version', ctypes.c_char_p, [])
        _sig(lib, 'f1tenth_wall_tracker_core_build_info', _L, [ctypes.c_char_p, _L])
        _sig(lib, 'wt_tracker_new', _VP, [_DP, _L, _LP, _L, _DP, _L, _LP, _L,
                                          ctypes.POINTER(_I)])
        _sig(lib, 'wt_tracker_free', None, [_VP])
        _sig(lib, 'wt_tracker_set_params', _I, [_VP, _DP, _L, _LP, _L, _DP, _L, _LP, _L])
        _sig(lib, 'wt_tracker_get_scalars', _I, [_VP, _VP])
        _sig(lib, 'wt_tracker_set_scalars', _I, [_VP, _VP])
        _sig(lib, 'wt_tracker_export_tracks', _I,
             [_VP, _VP, _L, _DP, _L, _LP, _L, _DP, _L, _DP, _LP, _L, _VP])
        _sig(lib, 'wt_tracker_import_tracks', _I,
             [_VP, _L, _VP, _DP, _L, _LP, _L, _DP, _L, _DP, _LP, _L])
        _sig(lib, 'wt_tracker_step', _I,
             [_VP, _VP, _L, _D, _D, _D, _D, _I, _DP, _DP, _L, _D, _D, _VP, _I, _LP, _L, _LP])
        _sig(lib, 'wt_tracker_update_roles', _I, [_VP, _D, _D, _D])
        _sig(lib, 'wt_tracker_published', _I, [_VP, _D, _D, _D, _LP, _L, _LP])
        _sig(lib, 'wt_tracker_front_track', _I,
             [_VP, _D, _D, _D, _D, _LP, _LP, _LP, _L, _LP])
        _sig(lib, 'wt_tracker_apply_rect_verdicts', _I, [_VP, _L, _LP, _DP])
        _sig(lib, 'wt_tracker_spawn', _I, [_VP, _VP, _D, _VP])
        _sig(lib, 'wt_tracker_merge', _I, [_VP])
        _sig(lib, 'wt_tracker_rejections', _I, [_VP, _VP, _L, _LP])
        _sig(lib, 'wt_debug_solve2', _I, [_DP, _DP, _DP])
        _sig(lib, 'wt_debug_inv2', _I, [_DP, _DP])
        _sig(lib, 'wt_debug_lsa', _I, [_L, _L, _DP, _LP, _LP])
        _sig(lib, 'wt_debug_hypot', _D, [_D, _D])
        _sig(lib, 'wt_debug_pow2', _D, [_D])
        _sig(lib, 'wt_debug_wrap_pi', _D, [_D])
        _sig(lib, 'wt_debug_sum', _D, [_DP, _L])
        _sig(lib, 'wt_debug_arange', _L, [_D, _D, _D, _DP, _L])
        _sig(lib, 'wt_debug_searchsorted', _I, [_DP, _L, _DP, _L, _LP])
        _sig(lib, 'wt_debug_svd_vt', _I, [_DP, _L, _DP])
        _sig(lib, 'wt_debug_robust_line', _I, [_DP, _L, _D, _L, _DP])
        _sig(lib, 'wt_fusion_cycle', _I,
             [_VP, _DP, _L, _DP, _L, _DP, _L, _DP, _L, _DP, _L, _DP, _L, _VP, _DP, _DP, _DP, _VP,
              _L, _LP, _LP])
        _sig(lib, 'wt_verify_rectangles', _I,
             [_DP, _L, _VP, _VP, _L, _L, ctypes.c_float, _L, _L, _DP, _DP, _VP, _VP, _L, _VP])
        _LIB = lib
    except Exception as exc:                      # noqa: BLE001 - any of them
        _LOAD_ERROR = f'{type(exc).__name__}: {exc}'
    return _LIB


def available() -> bool:
    """Is the Rust tracker usable here? False: wall_tracker_impl:=rust refuses to start."""
    return _load() is not None


def load_error() -> str | None:
    """Return why it is not usable (None when it is)."""
    _load()
    return _LOAD_ERROR


def library_path() -> str | None:
    """Return the file that was (or failed to be) loaded."""
    _load()
    return _PATH


def version() -> str | None:
    lib = _load()
    return None if lib is None else lib.f1tenth_wall_tracker_core_version().decode()


def build_info() -> str | None:
    """Return the library's build line, e.g. 'build: cargo profile debug, opt-level 3, ...'."""
    lib = _load()
    if lib is None:
        return None
    buf = ctypes.create_string_buffer(256)
    lib.f1tenth_wall_tracker_core_build_info(buf, 256)
    return buf.value.decode()


def require():
    lib = _load()
    if lib is None:
        raise RuntimeError(f'f1tenth_wall_tracker_core is not loadable: {_LOAD_ERROR}')
    return lib


def raise_for(status):
    """Raise the Python exception the reference raises at the same place."""
    if status == WT_ERR_SINGULAR:
        raise np.linalg.LinAlgError('Singular matrix')
    if status in (WT_ERR_LSA_INVALID, WT_ERR_LSA_INFEASIBLE):
        raise ValueError(_ERRORS[status])
    if status == WT_ERR_OVERFLOW:
        raise OverflowError(_ERRORS[status])
    raise RuntimeError(f'f1tenth_wall_tracker_core: {_ERRORS.get(status, f"status {status}")}')


def _ptr(a, kind=_DP):
    return a.ctypes.data_as(kind)


def f64(a):
    return np.ascontiguousarray(a, dtype=np.float64)


def i64(a):
    return np.ascontiguousarray(a, dtype=np.int64)


class Handle:
    """One library tracker (MultiWallTracker state behind a mutex)."""

    def __init__(self, pf, pi, rf, ri):
        lib = require()
        self._lib = lib
        pf, pi, rf, ri = f64(pf), i64(pi), f64(rf), i64(ri)
        status = ctypes.c_int32(0)
        h = lib.wt_tracker_new(_ptr(pf), pf.size, _ptr(pi, _LP), pi.size, _ptr(rf), rf.size,
                               _ptr(ri, _LP), ri.size, ctypes.byref(status))
        if not h:
            raise_for(status.value)
        self._h = h
        self._scalars = np.zeros(1, dtype=DTYPES[STRUCT_SCALARS])
        self._sizes = np.zeros(1, dtype=DTYPES[STRUCT_SIZES])
        self._ids = np.zeros(64, dtype=np.int64)
        self._n = ctypes.c_int64(0)
        self._bufs = None
        self._grow(32, 160, 96, 160, 2048)

    def __del__(self):
        h, self._h = getattr(self, '_h', None), None
        if h:
            self._lib.wt_tracker_free(h)

    def _grow(self, nt, nc, nl, nf, ne):
        self._bufs = (np.zeros(nt, dtype=DTYPES[STRUCT_TRACK]), np.zeros((nc, 3)),
                      np.zeros(nl, dtype=np.int64), np.zeros(nf), np.zeros((ne, 5)),
                      np.zeros(ne, dtype=np.int64))

    def _check(self, status):
        if status != WT_OK:
            raise_for(status)

    def set_params(self, pf, pi, rf, ri):
        pf, pi, rf, ri = f64(pf), i64(pi), f64(rf), i64(ri)
        self._check(self._lib.wt_tracker_set_params(
            self._h, _ptr(pf), pf.size, _ptr(pi, _LP), pi.size, _ptr(rf), rf.size,
            _ptr(ri, _LP), ri.size))

    def get_scalars(self):
        self._check(self._lib.wt_tracker_get_scalars(self._h, self._scalars.ctypes.data))
        return self._scalars[0]

    def set_scalars(self, row):
        self._check(self._lib.wt_tracker_set_scalars(self._h, row.ctypes.data))

    def export_tracks(self):
        """(tracks, confirm, lidar, conf, ext, ext_serial) views, sized to the state."""
        while True:
            t, c, li, f, e, s = self._bufs
            st = self._lib.wt_tracker_export_tracks(
                self._h, t.ctypes.data, t.size, _ptr(c), c.shape[0], _ptr(li, _LP), li.size,
                _ptr(f), f.size, _ptr(e), _ptr(s, _LP), s.size, self._sizes.ctypes.data)
            z = self._sizes[0]
            if st == WT_ERR_CAPACITY:
                self._grow(*(2 * max(int(z[k]), 1) for k in range(5)))
                continue
            self._check(st)
            return (t[:int(z['n_tracks'])], c[:int(z['n_confirm'])], li[:int(z['n_lidar'])],
                    f[:int(z['n_conf'])], e[:int(z['n_ext'])], s[:int(z['n_ext'])])

    def import_tracks(self, tracks, confirm, lidar, conf, ext, ext_serial):
        tracks = np.ascontiguousarray(tracks, dtype=DTYPES[STRUCT_TRACK])
        confirm, lidar, conf = f64(confirm).reshape(-1, 3), i64(lidar), f64(conf)
        ext, ext_serial = f64(ext).reshape(-1, 5), i64(ext_serial)
        self._check(self._lib.wt_tracker_import_tracks(
            self._h, tracks.size, tracks.ctypes.data, _ptr(confirm), confirm.shape[0],
            _ptr(lidar, _LP), lidar.size, _ptr(conf), conf.size, _ptr(ext),
            _ptr(ext_serial, _LP), ext_serial.size))

    def _ids_out(self, call):
        while True:
            st = call(self._ids)
            n = int(self._n.value)
            if st == WT_ERR_CAPACITY:
                self._ids = np.zeros(2 * n, dtype=np.int64)
                continue
            self._check(st)
            return self._ids[:n].tolist()

    def step(self, obs, stamp, car, scan, defer_roles):
        """`scan` is (ranges f64, bearings f64, laser (x, y), cut u8 or None) or None."""
        if scan is None:
            r = b = np.zeros(0)
            n, lx, ly, cut, has = 0, 0.0, 0.0, None, 0
        else:
            r, b, (lx, ly), cut = scan
            n, has = r.size, 1
        cut_p = None if cut is None else cut.ctypes.data
        return self._ids_out(lambda ids: self._lib.wt_tracker_step(
            self._h, obs.ctypes.data, obs.size, stamp, car[0], car[1], car[2], has, _ptr(r),
            _ptr(b), n, lx, ly, cut_p, int(defer_roles), _ptr(ids, _LP), ids.size,
            ctypes.byref(self._n)))

    def update_roles(self, car):
        self._check(self._lib.wt_tracker_update_roles(self._h, car[0], car[1], car[2]))

    def published(self, car):
        return self._ids_out(lambda ids: self._lib.wt_tracker_published(
            self._h, car[0], car[1], car[2], _ptr(ids, _LP), ids.size, ctypes.byref(self._n)))

    def front_track(self, car, front_x):
        """(front id or None, [(id, code)] rejects)."""
        fid, has = ctypes.c_int64(0), ctypes.c_int64(0)
        cap = 32
        while True:
            buf = np.zeros(2 * cap, dtype=np.int64)
            st = self._lib.wt_tracker_front_track(
                self._h, car[0], car[1], car[2], front_x, ctypes.byref(fid), ctypes.byref(has),
                _ptr(buf, _LP), cap, ctypes.byref(self._n))
            if st == WT_ERR_CAPACITY:
                cap = 2 * int(self._n.value)
                continue
            self._check(st)
            n = int(self._n.value)
            pairs = buf[:2 * n].reshape(-1, 2).tolist()
            return (int(fid.value) if has.value else None), [(int(a), int(c)) for a, c in pairs]

    def apply_rect_verdicts(self, ids, values):
        ids, values = i64(ids), f64(values).reshape(-1, 3)
        self._check(self._lib.wt_tracker_apply_rect_verdicts(
            self._h, ids.size, _ptr(ids, _LP), _ptr(values)))

    def spawn(self, line_row, stamp, obs_row):
        self._check(self._lib.wt_tracker_spawn(
            self._h, line_row.ctypes.data, stamp,
            None if obs_row is None else obs_row.ctypes.data))

    def merge(self):
        self._check(self._lib.wt_tracker_merge(self._h))

    def rejections(self):
        buf = np.zeros(32, dtype=DTYPES[STRUCT_REJECTION])
        self._check(self._lib.wt_tracker_rejections(
            self._h, buf.ctypes.data, buf.size, ctypes.byref(self._n)))
        return buf[:int(self._n.value)].tolist()


# ---- debug entry points (tools/rust_parity/wall_tracker/numerics.py) -----

def debug_solve2(a, b):
    lib = require()
    a, b, out = f64(a).reshape(4), f64(b).reshape(2), np.zeros(2)
    st = lib.wt_debug_solve2(_ptr(a), _ptr(b), _ptr(out))
    if st != WT_OK:
        raise_for(st)
    return out


def debug_inv2(a):
    lib = require()
    a, out = f64(a).reshape(4), np.zeros(4)
    st = lib.wt_debug_inv2(_ptr(a), _ptr(out))
    if st != WT_OK:
        raise_for(st)
    return out.reshape(2, 2)


def debug_lsa(cost):
    lib = require()
    cost = f64(cost)
    nr, nc = cost.shape
    k = min(nr, nc)
    rows, cols = np.zeros(k, dtype=np.int64), np.zeros(k, dtype=np.int64)
    st = lib.wt_debug_lsa(nr, nc, _ptr(cost), _ptr(rows, _LP), _ptr(cols, _LP))
    if st != WT_OK:
        raise_for(st)
    return rows, cols


def debug_hypot(x, y):
    return require().wt_debug_hypot(float(x), float(y))


def debug_pow2(x):
    return require().wt_debug_pow2(float(x))


def debug_wrap_pi(x):
    return require().wt_debug_wrap_pi(float(x))


def debug_sum(v):
    v = f64(v)
    return require().wt_debug_sum(_ptr(v), v.size)


def debug_arange(start, stop, step):
    lib = require()
    n = int(lib.wt_debug_arange(float(start), float(stop), float(step), None, 0))
    out = np.zeros(max(n, 0))
    lib.wt_debug_arange(float(start), float(stop), float(step), _ptr(out), out.size)
    return out


def debug_searchsorted(arr, keys):
    lib = require()
    arr, keys = f64(arr), f64(keys)
    out = np.zeros(keys.size, dtype=np.int64)
    st = lib.wt_debug_searchsorted(_ptr(arr), arr.size, _ptr(keys), keys.size, _ptr(out, _LP))
    if st != WT_OK:
        raise_for(st)
    return out


def debug_svd_vt(x):
    lib = require()
    x = f64(x).reshape(-1, 2)
    out = np.zeros(4)
    st = lib.wt_debug_svd_vt(_ptr(x), x.shape[0], _ptr(out))
    if st != WT_OK:
        raise_for(st)
    return out.reshape(2, 2)


# ---- the per-cycle fusion ---------------------------------------------------

def fusion_cycle(inp_row, lines, pts, distance, z_min, z_max, cloud):
    """
    wt_fusion_cycle: (status, lidar_m, stereo_m, tol_m, observation rows, n_rejected).

    `inp_row` a one-element DTYPES[STRUCT_FUSION_IN] array; `lines` (n, 7)
    float64 (nx, ny, d, t0, t1, inliers, rms); `pts` (N, 2); `distance`,
    `z_min`, `z_max` float64; `cloud` (M, 3) float64 or None.
    """
    lib = require()
    width = int(inp_row[0]['width'])
    lines, pts = f64(lines).reshape(-1, 7), f64(pts).reshape(-1, 2)
    distance, z_min, z_max = f64(distance), f64(z_min), f64(z_max)
    cloud_p, n_cloud = None, 0
    if cloud is not None:
        cloud = f64(cloud).reshape(-1, 3)
        cloud_p, n_cloud = _ptr(cloud), cloud.shape[0]
    status = np.zeros(width, dtype=np.uint8)
    lidar_m, stereo_m, tol_m = np.zeros(width), np.zeros(width), np.zeros(width)
    cap = max(8, 2 * lines.shape[0] + 8)
    n_obs, n_rej = ctypes.c_int64(0), ctypes.c_int64(0)
    while True:
        obs = np.zeros(cap, dtype=DTYPES[STRUCT_FUSION_OBS])
        st = lib.wt_fusion_cycle(
            inp_row.ctypes.data, _ptr(lines), lines.shape[0], _ptr(pts), pts.shape[0],
            _ptr(distance), distance.size, _ptr(z_min), z_min.size, _ptr(z_max), z_max.size,
            cloud_p, n_cloud, status.ctypes.data, _ptr(lidar_m), _ptr(stereo_m), _ptr(tol_m),
            obs.ctypes.data, cap, ctypes.byref(n_obs), ctypes.byref(n_rej))
        if st == WT_ERR_CAPACITY:
            cap = 2 * int(n_obs.value)
            continue
        if st != WT_OK:
            raise_for(st)
        return status, lidar_m, stereo_m, tol_m, obs[:int(n_obs.value)], int(n_rej.value)


def verify_rectangles(segs, labels, depth, cam, p, grid_z, grid_w, depth_scale, wall_class,
                      object_class):
    """wt_verify_rectangles: the per-segment WtRectOut rows."""
    lib = require()
    segs = f64(segs).reshape(-1, 4)
    labels = np.ascontiguousarray(labels, dtype=np.uint8)
    depth = np.ascontiguousarray(depth, dtype=np.uint16)
    height, width = labels.shape[:2]
    if depth.shape[:2] != (height, width):
        raise ValueError('labels and depth must be one size')
    cam, p = f64(cam).reshape(17), f64(p).reshape(6)
    grid_z = np.ascontiguousarray(grid_z, dtype=np.float32)
    grid_w = np.ascontiguousarray(grid_w, dtype=np.float32)
    out = np.zeros(segs.shape[0], dtype=DTYPES[STRUCT_RECT_OUT])
    st = lib.wt_verify_rectangles(
        _ptr(segs), segs.shape[0], labels.ctypes.data, depth.ctypes.data, height, width,
        float(depth_scale), int(wall_class), int(object_class), _ptr(cam), _ptr(p),
        grid_z.ctypes.data, grid_w.ctypes.data, grid_z.size, out.ctypes.data)
    if st != WT_OK:
        raise_for(st)
    return out


def debug_robust_line(points, inlier_m, max_pairs):
    """lidar_front_wall._robust_line by the library: the same tuple, or None."""
    lib = require()
    pts = f64(points).reshape(-1, 2)
    out = np.zeros(5)
    st = lib.wt_debug_robust_line(_ptr(pts), pts.shape[0], float(inlier_m), int(max_pairs),
                                  _ptr(out))
    if st < 0:
        raise_for(st)
    if st == 0:
        return None
    return float(out[0]), float(out[1]), float(out[2]), int(out[3]), float(out[4])
