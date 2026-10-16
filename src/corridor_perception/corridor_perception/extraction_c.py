"""ctypes binding for corridor_perception_c's libextract_lines.so.

WHY ctypes AND NOT AN EXTENSION MODULE. The library has no Python in it and no
ROS in it: it is one function over flat arrays. A pybind11 or C-API module
would add a build dependency, a per-interpreter ABI and a second thing to
rebuild on every Python bump, to replace nine ctypes declarations. The cost is
that the argument types are asserted here rather than by a compiler, which is
what the equivalence test over a whole recording is for.

WHERE THE LIBRARY COMES FROM: ament_index, never a hard-coded path and never
LD_LIBRARY_PATH. `get_package_prefix('corridor_perception_c') + /lib` is the
installed location, and this package does NOT depend on
corridor_perception_c -- if it is not installed, `available()` is False, the
caller warns once and the Python runs. That is the whole fallback: no import
error, no partial state.

NOT THREAD SAFE. The C holds one static workspace, so two concurrent calls
would interleave in it. Every call here happens under the GIL between
`ascontiguousarray` and the return, and the one caller is a node's scan
callback, so there is no second caller to serialise against. If that ever
changes the workspace has to move into a caller-owned handle.
"""

from __future__ import annotations

import ctypes
import math
import os

import numpy as np

from .geometry import Line

MAX_BEAMS = 2048
MAX_SEGMENTS = 256


class _Params(ctypes.Structure):
    _fields_ = [
        ('sigma_0', ctypes.c_double), ('sigma_r', ctypes.c_double),
        ('lambda_abd', ctypes.c_double), ('abd_sigma_mult', ctypes.c_double),
        ('split_dist', ctypes.c_double), ('merge_alpha', ctypes.c_double),
        ('merge_rho', ctypes.c_double), ('merge_gap', ctypes.c_double),
        ('min_points', ctypes.c_int32), ('min_length', ctypes.c_double),
        ('max_point_gap', ctypes.c_double), ('quality_gates', ctypes.c_int32),
        ('seed_base_points', ctypes.c_int32), ('seed_max_points', ctypes.c_int32),
        ('seed_range_m', ctypes.c_double), ('seed_range_step_m', ctypes.c_double),
        ('max_rms_m', ctypes.c_double), ('min_fill', ctypes.c_double),
        ('occlusion_margin_m', ctypes.c_double), ('min_incidence', ctypes.c_double),
    ]


class _Segment(ctypes.Structure):
    _fields_ = [
        ('rho', ctypes.c_double), ('alpha', ctypes.c_double),
        ('cov00', ctypes.c_double), ('cov01', ctypes.c_double),
        ('cov11', ctypes.c_double),
        ('p0x', ctypes.c_double), ('p0y', ctypes.c_double),
        ('p1x', ctypes.c_double), ('p1y', ctypes.c_double),
        ('n_points', ctypes.c_int32),
        ('rms', ctypes.c_double), ('fill', ctypes.c_double),
    ]


class _Stats(ctypes.Structure):
    _fields_ = [('seed', ctypes.c_int32), ('rms', ctypes.c_int32),
                ('fill', ctypes.c_int32), ('support', ctypes.c_int32),
                ('kept', ctypes.c_int32)]


ERRORS = {-1: 'null argument', -2: 'too many beams', -3: 'workspace overflow'}

_LIB = None
_LOAD_ERROR = None
#: Reused across calls: the output buffer, the stats and the params struct are
#: the same objects every scan, so a call allocates nothing on either side.
_OUT = (_Segment * MAX_SEGMENTS)()
_STATS = _Stats()
_PARAMS = _Params()


def _load():
    global _LIB, _LOAD_ERROR
    if _LIB is not None or _LOAD_ERROR is not None:
        return _LIB
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(get_package_prefix('corridor_perception_c'), 'lib',
                            'libextract_lines.so')
        lib = ctypes.CDLL(path)
        lib.el_extract.restype = ctypes.c_int32
        lib.el_extract.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_int32, ctypes.c_double,
            ctypes.c_double, ctypes.POINTER(_Params), ctypes.POINTER(_Segment),
            ctypes.c_int32, ctypes.POINTER(_Stats)]
        lib.el_default_params.restype = None
        lib.el_default_params.argtypes = [ctypes.POINTER(_Params)]
        lib.el_version.restype = ctypes.c_char_p
        lib.el_version.argtypes = []
        _LIB = lib
    except Exception as exc:                      # noqa: BLE001 - any of them
        _LOAD_ERROR = f'{type(exc).__name__}: {exc}'
    return _LIB


def available() -> bool:
    """Is the C backend usable? False means the caller runs the Python."""
    return _load() is not None


def load_error() -> str | None:
    """Why it is not usable, for the one warning the caller logs."""
    _load()
    return _LOAD_ERROR


def library_path() -> str | None:
    lib = _load()
    return None if lib is None else lib._name


def version() -> str | None:
    lib = _load()
    return None if lib is None else lib.el_version().decode()


def _fill_params(p):
    for name, _type in _Params._fields_:
        setattr(_PARAMS, name, getattr(p, name) if name != 'quality_gates'
                else int(bool(p.quality_gates)))
    return _PARAMS


#: How far an angle array may stray from the arithmetic progression the C
#: rebuilds before the C cannot be used on it. A real LaserScan is exact here:
#: the caller computed the array as angle_min + arange(n) * angle_increment in
#: the first place, and the C redoes those same two operations in the same
#: order, so the reconstruction is bit for bit. The tolerance exists for a
#: synthesised array, not for a sensor.
ANGLE_GRID_TOL_RAD = 1e-12


def uniform_angles(angles):
    """Is this an arithmetic progression the C's (min, increment) can rebuild?

    THE C'S API TAKES angle_min AND angle_increment, not an angle array, so it
    can only ever see a uniform grid -- which is what a LaserScan is, by the
    message definition. A caller that passes arbitrary bearings (a unit test
    synthesising points, corridor_perception.replay re-deriving angles from a
    point cloud) is asking a question the C cannot answer, and the answer must
    be the reference implementation rather than a quietly resampled scan.
    """
    a = np.asarray(angles, dtype=float)
    if a.size < 3:
        return True
    grid = a[0] + np.arange(a.size) * ((a[-1] - a[0]) / (a.size - 1))
    return bool(np.max(np.abs(a - grid)) <= ANGLE_GRID_TOL_RAD)


def extract_segments_c(ranges, angles, p, stats=None):
    """The C backend of extract_segments. Returns the same [Line] it does.

    `angles` is NOT passed down: the C rebuilds angle_min + i * angle_increment
    in the same order numpy does, so the two agree bit for bit, and passing
    the array as well would be a second source of truth for the same numbers.
    Raises ValueError if the array is not that grid -- see uniform_angles; the
    dispatcher in extraction.py checks first and runs the reference instead.
    """
    a = np.asarray(angles, dtype=float)
    r = np.ascontiguousarray(ranges, dtype=np.float32)
    n = int(r.size)
    if n == 0:
        if stats is not None:
            stats.update({'seed': 0, 'rms': 0, 'fill': 0, 'support': 0, 'kept': 0})
        return []
    if n > MAX_BEAMS or a.size != n:
        raise ValueError(f'extract_segments_c: {n} beams against a limit of {MAX_BEAMS}')
    if not uniform_angles(a):
        raise ValueError('extract_segments_c: the angles are not a uniform grid, '
                         'which is the only thing (angle_min, angle_increment) '
                         'can express')
    angle_min = float(a[0])
    angle_increment = float((a[-1] - a[0]) / (n - 1)) if n > 1 else 0.0
    lib = _load()
    if lib is None:
        raise RuntimeError(f'libextract_lines.so not loaded: {_LOAD_ERROR}')
    count = lib.el_extract(
        r.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), n, angle_min,
        angle_increment, ctypes.byref(_fill_params(p)), _OUT, MAX_SEGMENTS,
        ctypes.byref(_STATS))
    if count < 0:
        raise RuntimeError('el_extract: ' + ERRORS.get(count, f'error {count}'))
    out = []
    for i in range(count):
        s = _OUT[i]
        out.append(Line(s.rho, s.alpha,
                        np.array([[s.cov00, s.cov01], [s.cov01, s.cov11]]),
                        np.array([s.p0x, s.p0y]), np.array([s.p1x, s.p1y]),
                        int(s.n_points), 'sensor'))
    if stats is not None:
        stats.update({'seed': int(_STATS.seed), 'rms': int(_STATS.rms),
                      'fill': int(_STATS.fill), 'support': int(_STATS.support),
                      'kept': int(_STATS.kept)})
    return out


def segment_quality(index):
    """(rms, fill) of the index-th segment of the LAST call. Reporting only."""
    s = _OUT[int(index)]
    return float(s.rms), (float(s.fill) if math.isfinite(s.fill) else math.inf)
