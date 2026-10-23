"""
ctypes binding for f1tenth_mpc_core (src/rust/f1tenth_mpc_core), the Rust corridor.

WHAT IT IS. libf1tenth_mpc_core.so is a ROS-free Rust library with a C ABI
(src/rust/f1tenth_mpc_core/src/ffi.rs): f1tenth_params.corridor_geometry's two
corridor builders and mpc_solver.py's corridor queries, ported operation for
operation so their arrays are bit-identical to the Python on linus (parity:
tools/rust_parity/corridor, output/rust/parity_corridor.md). corridor_backend.py
is the dispatcher; nothing else should call this module.

WHY ctypes AND NOT AN EXTENSION MODULE. Same reasoning as corridor_perception's
extraction_c.py: a few flat-array functions do not justify maturin, pybind11 or a
per-interpreter ABI. The price is that the argument types are asserted here, not
by a compiler, so the loader checks the library's ABI version and the size of
every struct against the ctypes mirrors below, and refuses a mismatch.

WHERE THE LIBRARY COMES FROM: ament_index,
`get_package_prefix('f1tenth_mpc_core')/lib/f1tenth_mpc_core/libf1tenth_mpc_core.so`
(cargo-ament-build's install location), never LD_LIBRARY_PATH. The environment
variable F1TENTH_MPC_CORE_LIB, when set, names the file instead (the parity
harness uses it to test a fresh `cargo build`); the path actually loaded is in
library_path(). If nothing loads, available() is False and load_error() says why;
corridor_impl:=rust then refuses to start rather than running the Python silently.

THREAD SAFETY. Every library call is pure (no static state), so concurrent calls
are safe. The ctypes structs here are allocated per call.

DIFFERENCES FROM THE PYTHON, all outside what any caller does:
- inputs are converted with float()/int() first, so a numpy float32 scalar is
  widened before the arithmetic instead of inside it;
- xL/yL/xR/yR are contiguous arrays, not column views of one (n, 2) array (the
  values are identical; nothing reads the layout). 'ny' is the 'tx' array, as in
  the Python dict;
- numpy's RuntimeWarnings (overflow, invalid) are not emitted;
- n above MAX_SAMPLES (1e6) is refused (ValueError) instead of attempted.
"""

from __future__ import annotations

import ctypes
import os

from f1tenth_params.corridor_geometry import (
    CORRIDOR_HANDLE_FRAC, CORRIDOR_WALL_SHAPES, POSE_HANDLE_FRAC)
import numpy as np

#: Must equal the library's f1tenth_mpc_core_abi_version() (ffi.rs ABI_VERSION).
ABI_VERSION = 1

#: corridor.rs MAX_SAMPLES: a larger n is refused before allocating.
MAX_SAMPLES = 1_000_000

#: Overrides the ament_index location (see the module docstring).
LIB_ENV = 'F1TENTH_MPC_CORE_LIB'

_WALLS = {'bezier': 0, 'trapezoid': 1}

MPC_OK = 0
_ERRORS = {
    -1: 'null pointer',
    -2: 'too few samples',
    -3: 'invalid walls code',
    -4: 'buffer length mismatch',
    -5: 'math domain error',
    -6: 'float division by zero',
    -7: 'panic in f1tenth_mpc_core (a bug)',
    -8: 'too many samples',
    -9: 'empty corridor',
}


class _CurvesArgs(ctypes.Structure):
    _fields_ = [
        ('x0', ctypes.c_double), ('y0', ctypes.c_double),
        ('psi_start', ctypes.c_double), ('psi_end', ctypes.c_double),
        ('length', ctypes.c_double), ('dpsi', ctypes.c_double),
        ('u_start', ctypes.c_double), ('u_end', ctypes.c_double),
        ('w0', ctypes.c_double), ('w1', ctypes.c_double),
        ('handle_frac', ctypes.c_double), ('r_min', ctypes.c_double),
        ('length_ref', ctypes.c_double), ('n', ctypes.c_int64),
        ('has_dpsi', ctypes.c_int32), ('has_r_min', ctypes.c_int32),
        ('has_length_ref', ctypes.c_int32), ('walls', ctypes.c_int32),
    ]


class _PoseArgs(ctypes.Structure):
    _fields_ = [
        ('x0', ctypes.c_double), ('y0', ctypes.c_double),
        ('psi_start', ctypes.c_double), ('x1', ctypes.c_double),
        ('y1', ctypes.c_double), ('psi_end', ctypes.c_double),
        ('handle_a', ctypes.c_double), ('handle_b', ctypes.c_double),
        ('w0', ctypes.c_double), ('w1', ctypes.c_double),
        ('handle_frac', ctypes.c_double), ('r_min', ctypes.c_double),
        ('length_ref', ctypes.c_double), ('n', ctypes.c_int64),
        ('has_r_min', ctypes.c_int32), ('has_length_ref', ctypes.c_int32),
        ('walls', ctypes.c_int32), ('reserved', ctypes.c_int32),
    ]


class _Info(ctypes.Structure):
    _fields_ = [
        ('pend', ctypes.c_double * 2), ('dpsi', ctypes.c_double),
        ('length', ctypes.c_double), ('u_end_eff', ctypes.c_double),
        ('w1_eff', ctypes.c_double), ('kappa_max', ctypes.c_double),
        ('ctrl_left', ctypes.c_double * 8), ('ctrl_right', ctypes.c_double * 8),
        ('ctrl_centre', ctypes.c_double * 8),
        ('ramp_clamped', ctypes.c_int32), ('feasible', ctypes.c_int32),
        ('has_ctrl_walls', ctypes.c_int32), ('has_ctrl_centre', ctypes.c_int32),
    ]


_DP = ctypes.POINTER(ctypes.c_double)

#: The output arrays, in _Buffers order; the dict's 'ny' is 'tx'.
_ARRAYS = ('xc', 'yc', 'xL', 'yL', 'xR', 'yR', 'tx', 'ty', 'nx', 'halfWidth')


class _Buffers(ctypes.Structure):
    _fields_ = [('n', ctypes.c_int64)] + [(name, _DP) for name in _ARRAYS]


class _Frame(ctypes.Structure):
    _fields_ = [
        ('idx', ctypes.c_int64), ('d2', ctypes.c_double),
        ('pc', ctypes.c_double * 2), ('n', ctypes.c_double * 2),
        ('half_w', ctypes.c_double), ('d_lat', ctypes.c_double),
        ('heading', ctypes.c_double),
    ]


#: ffi.rs STRUCT_* kinds and their ctypes mirrors.
_STRUCTS = {1: _CurvesArgs, 2: _PoseArgs, 3: _Info, 4: _Buffers, 5: _Frame}

_LIB = None
_LOAD_ERROR = None
_PATH = None


def _default_path():
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('f1tenth_mpc_core'), 'lib',
                        'f1tenth_mpc_core', 'libf1tenth_mpc_core.so')


def _load():
    global _LIB, _LOAD_ERROR, _PATH
    if _LIB is not None or _LOAD_ERROR is not None:
        return _LIB
    try:
        path = os.environ.get(LIB_ENV) or _default_path()
        _PATH = path
        lib = ctypes.CDLL(path)
        lib.f1tenth_mpc_core_abi_version.restype = ctypes.c_uint32
        lib.f1tenth_mpc_core_abi_version.argtypes = []
        abi = int(lib.f1tenth_mpc_core_abi_version())
        if abi != ABI_VERSION:
            raise RuntimeError(f'{path}: ABI version {abi}, this binding is {ABI_VERSION}')
        lib.f1tenth_mpc_core_struct_size.restype = ctypes.c_int64
        lib.f1tenth_mpc_core_struct_size.argtypes = [ctypes.c_int32]
        for kind, struct in _STRUCTS.items():
            size = int(lib.f1tenth_mpc_core_struct_size(kind))
            if size != ctypes.sizeof(struct):
                raise RuntimeError(
                    f'{path}: {struct.__name__} is {size} bytes in the library, '
                    f'{ctypes.sizeof(struct)} in mpc_native.py')
        lib.f1tenth_mpc_core_version.restype = ctypes.c_char_p
        lib.f1tenth_mpc_core_version.argtypes = []
        lib.f1tenth_mpc_core_build_info.restype = ctypes.c_int64
        lib.f1tenth_mpc_core_build_info.argtypes = [ctypes.c_char_p, ctypes.c_int64]
        lib.mpc_corridor_curves.restype = ctypes.c_int32
        lib.mpc_corridor_curves.argtypes = [
            ctypes.POINTER(_CurvesArgs), ctypes.POINTER(_Info), ctypes.POINTER(_Buffers)]
        lib.mpc_corridor_curves_to_pose.restype = ctypes.c_int32
        lib.mpc_corridor_curves_to_pose.argtypes = [
            ctypes.POINTER(_PoseArgs), ctypes.POINTER(_Info), ctypes.POINTER(_Buffers)]
        lib.mpc_nearest_centerline_index.restype = ctypes.c_int32
        lib.mpc_nearest_centerline_index.argtypes = [
            _DP, _DP, ctypes.c_int64, ctypes.c_double, ctypes.c_double,
            _DP, ctypes.POINTER(ctypes.c_int64)]
        lib.mpc_corridor_frame.restype = ctypes.c_int32
        lib.mpc_corridor_frame.argtypes = [
            _DP, _DP, _DP, _DP, _DP, ctypes.c_int64, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(_Frame)]
        _LIB = lib
    except Exception as exc:                      # noqa: BLE001 - any of them
        _LOAD_ERROR = f'{type(exc).__name__}: {exc}'
    return _LIB


def available() -> bool:
    """Is the Rust corridor usable here? False: corridor_impl:=rust refuses to start."""
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
    return None if lib is None else lib.f1tenth_mpc_core_version().decode()


def build_info() -> str | None:
    """Return the library's build line, e.g. 'build: cargo profile debug, opt-level 3, ...'."""
    lib = _load()
    if lib is None:
        return None
    buf = ctypes.create_string_buffer(256)
    lib.f1tenth_mpc_core_build_info(buf, 256)
    return buf.value.decode()


def _require():
    lib = _load()
    if lib is None:
        raise RuntimeError(f'f1tenth_mpc_core is not loadable: {_LOAD_ERROR}')
    return lib


def _check(n, walls):
    """
    Check n and walls as corridor_geometry does, then the size cap.

    Same order and messages as the Python; the cap comes before anything is
    allocated.
    """
    n = int(n)
    if n < 2:
        raise ValueError(f'a corridor needs at least 2 samples, got {n}')
    if walls not in CORRIDOR_WALL_SHAPES:
        raise ValueError(f'walls must be one of {CORRIDOR_WALL_SHAPES}, got {walls!r}')
    if n > MAX_SAMPLES:
        raise ValueError(f'f1tenth_mpc_core refuses more than {MAX_SAMPLES} samples, got {n}')
    return n, _WALLS[walls]


def _raise_for(status):
    if status == -5:
        raise ValueError('math domain error')
    if status == -6:
        raise ZeroDivisionError('float division by zero')
    raise RuntimeError(f'f1tenth_mpc_core: {_ERRORS.get(status, f"status {status}")}')


def _buffers(n):
    arrays = {name: np.empty(n, dtype=np.float64) for name in _ARRAYS}
    buf = _Buffers(n, *[arrays[name].ctypes.data_as(_DP) for name in _ARRAYS])
    return arrays, buf


def _ctrl(values, present):
    if not present:
        return None
    v = [float(x) for x in values]
    return [v[0:2], v[2:4], v[4:6], v[6:8]]


def _dict(arrays, info, pose):
    """Build the corridor dict: keys, order and value types as corridor_geometry's."""
    out = {
        'xc': arrays['xc'], 'yc': arrays['yc'],
        'xL': arrays['xL'], 'yL': arrays['yL'],
        'xR': arrays['xR'], 'yR': arrays['yR'],
        'tx': arrays['tx'], 'ty': arrays['ty'],
        'nx': arrays['nx'], 'ny': arrays['tx'],
        'halfWidth': arrays['halfWidth'],
        'Pend': np.array([info.pend[0], info.pend[1]], dtype=float),
        'dpsi': float(info.dpsi),
    }
    if pose is not None:
        out['length'] = float(info.length)
        out['handle_a'] = pose[0]
        out['handle_b'] = pose[1]
    out.update({
        'u_end_eff': float(info.u_end_eff),
        'w1_eff': float(info.w1_eff),
        'ramp_clamped': bool(info.ramp_clamped),
        'feasible': bool(info.feasible),
        'kappa_max': float(info.kappa_max),
    })
    if pose is not None:
        out['ctrl_centre'] = _ctrl(info.ctrl_centre, info.has_ctrl_centre)
    out['ctrl_left'] = _ctrl(info.ctrl_left, info.has_ctrl_walls)
    out['ctrl_right'] = _ctrl(info.ctrl_right, info.has_ctrl_walls)
    return out


def corridor_curves(x0, y0, psi_start, psi_end, length, n,
                    dpsi=None, u_start=0.0, u_end=0.40,
                    w0=0.4333, w1=0.7667,
                    handle_frac=CORRIDOR_HANDLE_FRAC,
                    r_min=None, length_ref=None, walls='bezier'):
    """corridor_geometry.corridor_curves, computed by the Rust library."""
    n, walls_code = _check(n, walls)
    length = float(length)
    lib = _require()
    args = _CurvesArgs(
        float(x0), float(y0), float(psi_start), float(psi_end), length,
        0.0 if dpsi is None else float(dpsi),
        float(u_start), float(u_end), float(w0), float(w1), float(handle_frac),
        0.0 if r_min is None else float(r_min),
        0.0 if length_ref is None else float(length_ref),
        n, int(dpsi is not None), int(r_min is not None), int(length_ref is not None),
        walls_code)
    arrays, buf = _buffers(n)
    info = _Info()
    status = lib.mpc_corridor_curves(ctypes.byref(args), ctypes.byref(info), ctypes.byref(buf))
    if status != MPC_OK:
        _raise_for(status)
    return _dict(arrays, info, None)


def corridor_curves_to_pose(x0, y0, psi_start, x1, y1, psi_end, n,
                            handle_a=POSE_HANDLE_FRAC,
                            handle_b=POSE_HANDLE_FRAC,
                            w0=0.4333, w1=0.7667,
                            handle_frac=CORRIDOR_HANDLE_FRAC,
                            r_min=None, length_ref=None, walls='bezier'):
    """corridor_geometry.corridor_curves_to_pose, computed by the Rust library."""
    n, walls_code = _check(n, walls)
    handle_a = float(handle_a)
    handle_b = float(handle_b)
    lib = _require()
    args = _PoseArgs(
        float(x0), float(y0), float(psi_start), float(x1), float(y1), float(psi_end),
        handle_a, handle_b, float(w0), float(w1), float(handle_frac),
        0.0 if r_min is None else float(r_min),
        0.0 if length_ref is None else float(length_ref),
        n, int(r_min is not None), int(length_ref is not None), walls_code, 0)
    arrays, buf = _buffers(n)
    info = _Info()
    status = lib.mpc_corridor_curves_to_pose(
        ctypes.byref(args), ctypes.byref(info), ctypes.byref(buf))
    if status != MPC_OK:
        _raise_for(status)
    return _dict(arrays, info, (handle_a, handle_b))


def _column(corridor, key):
    return np.ascontiguousarray(corridor[key], dtype=np.float64)


def nearest_centerline_index(p, corridor):
    """mpc_solver.nearest_centerline_index: (d2[idx], idx)."""
    lib = _require()
    xc = _column(corridor, 'xc')
    yc = _column(corridor, 'yc')
    if xc.shape != yc.shape or xc.ndim != 1:
        raise ValueError('xc and yc must be 1-D arrays of one length')
    d2 = ctypes.c_double()
    idx = ctypes.c_int64()
    status = lib.mpc_nearest_centerline_index(
        xc.ctypes.data_as(_DP), yc.ctypes.data_as(_DP), xc.size,
        float(p[0]), float(p[1]), ctypes.byref(d2), ctypes.byref(idx))
    if status == -9:
        raise ValueError('attempt to get argmin of an empty sequence')
    if status != MPC_OK:
        _raise_for(status)
    return float(d2.value), int(idx.value)


def corridor_frame(p, corridor):
    """
    Return _nearest_corridor_frame plus its callers' values, as a dict.

    Keys: idx, d2, pc (array), n (array), half_w, d_lat, heading.
    """
    lib = _require()
    cols = [_column(corridor, k) for k in ('xc', 'yc', 'nx', 'ny', 'halfWidth')]
    size = cols[0].size
    if any(c.ndim != 1 or c.size != size for c in cols):
        raise ValueError('the corridor arrays must be 1-D and of one length')
    frame = _Frame()
    status = lib.mpc_corridor_frame(
        *[c.ctypes.data_as(_DP) for c in cols], size, float(p[0]), float(p[1]),
        ctypes.byref(frame))
    if status == -9:
        raise ValueError('attempt to get argmin of an empty sequence')
    if status != MPC_OK:
        _raise_for(status)
    return {
        'idx': int(frame.idx), 'd2': float(frame.d2),
        'pc': np.array([frame.pc[0], frame.pc[1]], dtype=float),
        'n': np.array([frame.n[0], frame.n[1]], dtype=float),
        'half_w': float(frame.half_w), 'd_lat': float(frame.d_lat),
        'heading': float(frame.heading),
    }
