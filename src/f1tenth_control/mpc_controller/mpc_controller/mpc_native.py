"""
ctypes binding for f1tenth_mpc_core (src/rust/f1tenth_mpc_core): Rust corridor + solver core.

WHAT IT IS. libf1tenth_mpc_core.so is a ROS-free Rust library with a C ABI
(src/rust/f1tenth_mpc_core/src/ffi.rs): f1tenth_params.corridor_geometry's two
corridor builders and mpc_solver.py's corridor queries (R1), ported operation for
operation so their arrays are bit-identical to the Python on linus (parity:
tools/rust_parity/corridor, output/rust/parity_corridor.md), and the RTI solver
core (R2): mpc_solver._solve_rti for stage_spec None, both speed models, with
OSQP C v1.0.0 linked in (solve_rti; parity tools/rust_parity/mpc,
output/rust/parity_mpc_solver.md). corridor_backend.py and solver_backend.py /
mpc_solver.solve_mpc_step are the dispatchers; nothing else should call this
module (the parity harness uses build_qp / osqp_solve_csc too).

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

DIFFERENCES FROM THE PYTHON (corridor), all outside what any caller does:
- inputs are converted with float()/int() first, so a numpy float32 scalar is
  widened before the arithmetic instead of inside it;
- xL/yL/xR/yR are contiguous arrays, not column views of one (n, 2) array (the
  values are identical; nothing reads the layout). 'ny' is the 'tx' array, as in
  the Python dict;
- numpy's RuntimeWarnings (overflow, invalid) are not emitted;
- n above MAX_SAMPLES (1e6) is refused (ValueError) instead of attempted;
- with numpy-scalar u_start/u_end the Python returns kappa_max as numpy.float64,
  this binding always a Python float (the same value; no caller passes them).

DIFFERENCES FROM THE PYTHON (solver, solve_rti), all outside what any caller does:
- the same values, the same result dict (keys in the same order, the same types);
  the same exception types for the inputs _solve_rti fails on (x0 of the wrong
  size, math domain errors, a zero wheelbase or ts, an overflowing Python float
  square, an empty corridor, a missing 'obstacles_world' / 'psiRef', an OSQP
  setup error as osqp.OSQPException(code));
- inputs pass through float() / np.asarray(dtype=float64) first (a float32
  corridor is widened before the arithmetic instead of inside it), and an n-D
  x0 / warm start is raveled;
- horizon < 1, a QP above MAX_QP_VARIABLES (6000: N up to ~850; a Rust
  allocation failure would abort the process where numpy raises MemoryError),
  a negative boundary_max_sources, last_u / pref_nom / a target
  point that is not 2 numbers, obstacle or boundary rows that are not 3 numbers
  and corridor arrays of different lengths are refused (ValueError) up front,
  where the Python fails later or builds a degenerate QP; speed-model arrays
  shorter than the horizon raise IndexError, as in the Python;
- it does not need the Python osqp package (OSQP C is linked into the library),
  so it runs where _solve_rti would raise "solver='rti' requires the 'osqp'
  package";
- numpy's RuntimeWarnings are not emitted.
"""

from __future__ import annotations

import ctypes
import operator
import os

from f1tenth_params.corridor_geometry import (
    CORRIDOR_HANDLE_FRAC, CORRIDOR_WALL_SHAPES, POSE_HANDLE_FRAC)
from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
import numpy as np

#: Must equal the library's f1tenth_mpc_core_abi_version() (ffi.rs ABI_VERSION).
#: 2: the solver core (R2).
ABI_VERSION = 2

#: corridor.rs MAX_SAMPLES: a larger n is refused before allocating.
MAX_SAMPLES = 1_000_000

#: qp.rs MAX_VARIABLES: a larger RTI QP is refused (ValueError) before
#: allocating its dense P (8 n^2 bytes). The MPC builds 120-145.
MAX_QP_VARIABLES = 6000

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
    -10: 'x0 size does not match the speed model',
    -11: 'numerical result out of range',
    -12: 'OSQP setup error',
    -13: 'horizon must be >= 1',
    -14: "corridor['psiRef'] missing",
    -15: 'QP too large',
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


_D = ctypes.c_double
_I64 = ctypes.c_int64
_I32 = ctypes.c_int32
_IP = ctypes.POINTER(ctypes.c_int64)

# ---- the solver core (R2): mirrors of src/ffi/solver.rs, field for field

#: MpcRtiArgs scalar fields after the pointers, in order (all c_double).
_RTI_SCALARS = (
    'ts', 'control_period', 'wheelbase', 'lr', 'vdes',
    'delta_min', 'delta_max', 'a_min', 'a_max', 'd_delta_min', 'd_delta_max',
    'd_a_min', 'd_a_max', 'v_min', 'v_max',
    'w_v', 'w_delta0', 'w_u_a', 'w_du_delta', 'w_du_a', 'w_obs', 'w_corr', 'w_line',
    'w_psi_stage', 'w_term', 'w_psi',
    'car_radius', 'avoidance_margin', 'psi_ref', 'psi_ref_turn', 'psi_start', 'line_psi',
    'boundary_slack_weight', 'boundary_slack_max', 'sm_alpha', 'sm_d', 'sm_slack_weight',
)
_RTI_FLAGS = (
    'has_warm_start', 'has_control_period', 'has_psi_ref', 'has_psi_ref_turn',
    'has_psi_start', 'has_target_line', 'boundary_use_slack', 'has_speed_model',
    'sm_delay', 'has_v_hi',
)


class _RtiArgs(ctypes.Structure):
    _fields_ = (
        [(name, _I64) for name in (
            'horizon', 'n_x0', 'n_corridor', 'n_obstacles', 'n_boundaries',
            'boundary_max_sources', 'n_warm_start', 'n_front_offsets', 'n_speed')]
        + [(name, _DP) for name in (
            'x0', 'warm_start', 'xc', 'yc', 'nx', 'ny', 'half_width', 'obstacles',
            'boundaries', 'front_offsets', 'c_lo', 'c_hi', 'v_hi')]
        + [('last_u', _D * 2), ('pref_nom', _D * 2), ('line_p', _D * 2)]
        + [(name, _D) for name in _RTI_SCALARS]
        + [(name, _I32) for name in _RTI_FLAGS])


class _RtiResult(ctypes.Structure):
    _fields_ = (
        [(name, _I64) for name in (
            'status', 'iterations', 'status_polish', 'rho_updates', 'n', 'm', 'nnz_p',
            'nnz_a', 'n_boundary_slack', 'osqp_error', 'x0_expected')]
        + [(name, _D) for name in (
            'cost', 'speed_slack', 'obj_val', 'prim_res', 'dual_res', 'rho_estimate')]
        + [('success', _I32), ('has_speed_slack', _I32),
           ('status_message', ctypes.c_char * 32)])


class _RtiBuffers(ctypes.Structure):
    _fields_ = [('n_zopt', _I64), ('zopt', _DP), ('n_x_pred', _I64), ('x_pred', _DP),
                ('n_v_cmd', _I64), ('v_cmd_pred', _DP), ('n_slack', _I64),
                ('boundary_slack', _DP)]


class _QpSizes(ctypes.Structure):
    _fields_ = [(name, _I64) for name in ('n', 'm', 'nnz_p', 'nnz_a', 'n_slack', 'n_vs')]


class _QpBuffers(ctypes.Structure):
    _fields_ = [('sizes', _QpSizes), ('p_colptr', _IP), ('p_rowind', _IP), ('p_val', _DP),
                ('q', _DP), ('a_colptr', _IP), ('a_rowind', _IP), ('a_val', _DP),
                ('l', _DP), ('u', _DP), ('z_guess', _DP)]


class _OsqpArgs(ctypes.Structure):
    _fields_ = [('n', _I64), ('m', _I64), ('nnz_p', _I64), ('nnz_a', _I64),
                ('p_colptr', _IP), ('p_rowind', _IP), ('p_val', _DP), ('q', _DP),
                ('a_colptr', _IP), ('a_rowind', _IP), ('a_val', _DP), ('l', _DP),
                ('u', _DP), ('warm_x', _DP)]


#: Every OSQPSettings field in OSQP's order: (name, is_float).
OSQP_SETTING_FIELDS = (
    ('device', False), ('linsys_solver', False), ('allocate_solution', False),
    ('verbose', False), ('profiler_level', False), ('warm_starting', False),
    ('scaling', False), ('polishing', False), ('rho', True), ('rho_is_vec', False),
    ('sigma', True), ('alpha', True), ('cg_max_iter', False), ('cg_tol_reduction', False),
    ('cg_tol_fraction', True), ('cg_precond', False), ('adaptive_rho', False),
    ('adaptive_rho_interval', False), ('adaptive_rho_fraction', True),
    ('adaptive_rho_tolerance', True), ('max_iter', False), ('eps_abs', True),
    ('eps_rel', True), ('eps_prim_inf', True), ('eps_dual_inf', True),
    ('scaled_termination', False), ('check_termination', False), ('check_dualgap', False),
    ('time_limit', True), ('delta', True), ('polish_refine_iter', False),
)


class _OsqpSettings(ctypes.Structure):
    _fields_ = [(name, _D if is_float else _I64) for name, is_float in OSQP_SETTING_FIELDS]


class _OsqpResult(ctypes.Structure):
    _fields_ = (
        [(name, _I64) for name in (
            'status_val', 'status_polish', 'iter', 'rho_updates', 'setup_error', 'has_x')]
        + [(name, _D) for name in ('obj_val', 'prim_res', 'dual_res', 'rho_estimate')]
        + [('settings', _OsqpSettings), ('status', ctypes.c_char * 32)])


#: ffi.rs STRUCT_* kinds and their ctypes mirrors.
_STRUCTS = {1: _CurvesArgs, 2: _PoseArgs, 3: _Info, 4: _Buffers, 5: _Frame,
            6: _RtiArgs, 7: _RtiResult, 8: _RtiBuffers, 9: _QpSizes, 10: _QpBuffers,
            11: _OsqpArgs, 12: _OsqpResult, 13: _OsqpSettings}

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
        # The solver structs: every field at the library's offset (two fields
        # of one type swapped would keep the size).
        lib.f1tenth_mpc_core_field_offset.restype = ctypes.c_int64
        lib.f1tenth_mpc_core_field_offset.argtypes = [ctypes.c_int32, ctypes.c_char_p]
        for kind, struct in _STRUCTS.items():
            if kind < 6:
                continue
            for name, *_type in struct._fields_:
                off = int(lib.f1tenth_mpc_core_field_offset(kind, name.encode()))
                if off != getattr(struct, name).offset:
                    raise RuntimeError(
                        f'{path}: {struct.__name__}.{name} is at offset {off} in the '
                        f'library, {getattr(struct, name).offset} in mpc_native.py')
        lib.mpc_solve_rti.restype = ctypes.c_int32
        lib.mpc_solve_rti.argtypes = [
            ctypes.POINTER(_RtiArgs), ctypes.POINTER(_RtiResult), ctypes.POINTER(_RtiBuffers)]
        lib.mpc_build_qp.restype = ctypes.c_int32
        lib.mpc_build_qp.argtypes = [
            ctypes.POINTER(_RtiArgs), ctypes.POINTER(_QpSizes), ctypes.POINTER(_QpBuffers)]
        lib.mpc_osqp_solve_csc.restype = ctypes.c_int32
        lib.mpc_osqp_solve_csc.argtypes = [
            ctypes.POINTER(_OsqpArgs), ctypes.POINTER(_OsqpResult), _DP]
        lib.mpc_osqp_settings.restype = ctypes.c_int32
        lib.mpc_osqp_settings.argtypes = [ctypes.c_int32, ctypes.POINTER(_OsqpSettings)]
        lib.mpc_osqp_version.restype = ctypes.c_int64
        lib.mpc_osqp_version.argtypes = [ctypes.c_char_p, ctypes.c_int64]
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


# ===========================================================================
# The solver core (R2): mpc_solver._solve_rti, stage_spec None
# ===========================================================================

#: The limits keys _solve_rti reads, and their MpcRtiArgs fields. vMin / vMax
#: only without a speed model.
_LIMIT_FIELDS = (('delta_min', 'delta_min'), ('delta_max', 'delta_max'),
                 ('a_min', 'a_min'), ('a_max', 'a_max'),
                 ('dDeltaMin', 'd_delta_min'), ('dDeltaMax', 'd_delta_max'),
                 ('dAMin', 'd_a_min'), ('dAMax', 'd_a_max'))
#: weights[key] (KeyError when absent) and weights.get(key, 0.0).
_WEIGHTS_REQUIRED = ('w_v', 'w_delta0', 'w_u_a', 'w_du_delta', 'w_du_a', 'w_obs', 'w_term')
_WEIGHTS_OPTIONAL = ('w_corr', 'w_line', 'w_psi_stage', 'w_psi')


def _f64(values):
    """Return a contiguous 1-D float64 array of `values` (np.asarray(dtype=float))."""
    return np.ascontiguousarray(np.asarray(values, dtype=np.float64).ravel())


def _pair(values, name):
    arr = _f64(values)
    if arr.size != 2:
        raise ValueError(f'{name} must have 2 entries, got {arr.size}')
    return float(arr[0]), float(arr[1])


def _rows3(values, what):
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return np.zeros((0, 3))
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f'{what} rows must have 3 entries, got shape {arr.shape}')
    return np.ascontiguousarray(arr)


def _rti_args(x0, last_u, pref_nom, corridor, horizon, ts, params, limits, weights, vdes,
              boundaries, boundary_max_sources, boundary_hard, boundary_slack_weight,
              boundary_slack_max, warm_start_z, control_period, speed_model, front_offsets):
    """
    Return (args, keep, use_slack, max_sources) for one tick.

    `keep` holds every array the args point into; it must outlive the call.
    The reads mirror _solve_rti's and planner_cost_corridor's: the same keys,
    the same defaults, read only where they read them.
    """
    keep = []
    a = _RtiArgs()

    def put(field, values, count=None):
        arr = values if isinstance(values, np.ndarray) and values.dtype == np.float64 \
            and values.flags.c_contiguous else _f64(values)
        keep.append(arr)
        setattr(a, field, arr.ctypes.data_as(_DP))
        if count is not None:
            setattr(a, count, arr.shape[0] if arr.ndim else 1)
        return arr

    sm = speed_model
    n_x = 4 if sm is None else 5
    x0 = np.asarray(x0, dtype=float)
    if x0.size != n_x:
        raise ValueError(f'x0 has {x0.size} entries, the speed model needs {n_x}')
    n = operator.index(horizon)
    if n < 1:
        raise ValueError(f'horizon must be >= 1, got {n}')
    a.horizon = n
    put('x0', _f64(x0), 'n_x0')
    a.last_u = (_D * 2)(*_pair(last_u, 'last_u'))
    a.pref_nom = (_D * 2)(*_pair(pref_nom, 'pref_nom'))
    if warm_start_z is not None:
        put('warm_start', _f64(warm_start_z), 'n_warm_start')
        a.has_warm_start = 1

    # corridor: the arrays, obstacles_world (planner_cost_corridor reads it
    # with [], so its absence is a KeyError there), the scalars
    cols = [_f64(corridor[k]) for k in ('xc', 'yc', 'nx', 'ny', 'halfWidth')]
    if any(c.size != cols[0].size for c in cols):
        raise ValueError('the corridor arrays xc, yc, nx, ny, halfWidth differ in length')
    for field, col in zip(('xc', 'yc', 'nx', 'ny', 'half_width'), cols):
        put(field, col)
    a.n_corridor = cols[0].size
    put('obstacles', _rows3(corridor['obstacles_world'], 'obstacles_world'), 'n_obstacles')
    a.car_radius = float(corridor.get('car_radius', 0.0))
    a.avoidance_margin = float(corridor.get('avoidance_margin', 0.12))
    put('front_offsets', _f64(front_offsets), 'n_front_offsets')

    for key, field in _LIMIT_FIELDS:
        setattr(a, field, float(limits[key]))
    if sm is None:
        a.v_min = float(limits['vMin'])
        a.v_max = float(limits['vMax'])
    else:
        a.v_min = a.v_max = float('nan')
    for key in _WEIGHTS_REQUIRED:
        setattr(a, key, float(weights[key]))
    for key in _WEIGHTS_OPTIONAL:
        setattr(a, key, float(weights.get(key, 0.0)))

    # terminal yaw: psiRef (planner_cost_corridor: corridor["psiRef"]),
    # psiRefTurn, psiStart -- read only with w_psi on, as the solver does
    if a.w_psi:
        if 'psiRef' not in corridor:
            raise KeyError('psiRef')
        a.psi_ref = float(corridor['psiRef'])
        a.has_psi_ref = 1
        turn = corridor.get('psiRefTurn')
        if turn is not None:
            a.psi_ref_turn = float(turn)
            a.has_psi_ref_turn = 1
            if 'psiStart' in corridor:
                a.psi_start = float(corridor['psiStart'])
                a.has_psi_start = 1
    line = corridor.get('targetLine')
    if line is not None and a.w_line:
        a.line_psi = float(line['psi'])
        a.line_p = (_D * 2)(*_pair(line['p'], "targetLine['p']"))
        a.has_target_line = 1

    max_sources = operator.index(boundary_max_sources)
    if max_sources < 0:
        raise ValueError(f'boundary_max_sources must be >= 0, got {max_sources}')
    a.boundary_max_sources = max_sources
    rows = list(boundaries or [])[:max_sources]
    put('boundaries', _rows3(rows, 'boundary'), 'n_boundaries')
    use_slack = bool(boundaries) and not boundary_hard
    a.boundary_use_slack = int(use_slack)
    a.boundary_slack_weight = float(boundary_slack_weight)
    a.boundary_slack_max = float(boundary_slack_max)

    a.ts = float(ts)
    if control_period is not None:
        a.control_period = float(control_period)
        a.has_control_period = 1
    a.wheelbase = float(params['L'])
    a.lr = float(params['lr'])
    a.vdes = float(vdes)

    if sm is not None:
        a.has_speed_model = 1
        a.sm_alpha = float(sm['alpha'])
        a.sm_d = float(sm['d'])
        a.sm_delay = 1 if int(sm.get('delay', 0)) else 0
        c_lo = [float(v) for v in sm['c_lo']]
        c_hi = [float(v) for v in sm['c_hi']]
        v_hi = sm.get('v_hi')
        v_hi = None if v_hi is None else [float(v) for v in v_hi]
        if min(len(c_lo), len(c_hi), n if v_hi is None else len(v_hi)) < n:
            raise IndexError('list index out of range')
        put('c_lo', _f64(c_lo[:n]))
        put('c_hi', _f64(c_hi[:n]))
        if v_hi is not None:
            put('v_hi', _f64(v_hi[:n]))
            a.has_v_hi = 1
        a.n_speed = n
        a.sm_slack_weight = float(sm.get('slack_weight', 1.0e3))
    return a, keep, use_slack, max_sources


def _raise_solver(status, osqp_error=None):
    """Raise the exception the Python solver raises for `status`."""
    if status == -5:
        raise ValueError('math domain error')
    if status == -6:
        raise ZeroDivisionError('float division by zero')
    if status == -11:
        raise OverflowError(34, 'Numerical result out of range')
    if status == -9:
        raise ValueError('attempt to get argmin of an empty sequence')
    if status == -13:
        raise ValueError('horizon must be >= 1')
    if status == -14:
        raise KeyError('psiRef')
    if status == -15:
        raise ValueError(f'the QP would have more than {MAX_QP_VARIABLES} variables: '
                         'refused by f1tenth_mpc_core')
    if status == -12:
        try:
            from osqp import OSQPException  # noqa: PLC0415 - only on this error
        except ImportError:
            raise RuntimeError(f'OSQP setup error {osqp_error}') from None
        raise OSQPException(osqp_error)
    raise RuntimeError(f'f1tenth_mpc_core: {_ERRORS.get(status, f"status {status}")}')


def solve_rti(x0, last_u, pref_nom, corridor, horizon, ts, params, limits, weights,
              obstacles=None, dmin=None, vdes=0.0, boundaries=None,
              boundary_max_sources=3, boundary_hard=False, boundary_slack_weight=1.0e4,
              boundary_slack_max=0.5, warm_start_z=None, control_period=None,
              speed_model=None, front_offsets=SOLVER_FRONT_POINT_OFFSETS_M):
    """
    Return (u0, info) of one RTI tick (mpc_solver._solve_rti, stage_spec None), by Rust.

    `weights` must ALREADY be scaled (scale_stage_weights): solve_mpc_step
    does that before it picks an implementation. `obstacles` and `dmin` are
    accepted for the signature and unused, as in _solve_rti (it reads
    corridor['obstacles_world']). The info dict has _solve_rti's keys in its
    order: success, status, status_message, message, cost, iterations, zopt,
    x_pred (N x 4), boundary_slack, and with a speed model v_cmd_pred and
    speed_slack.
    """
    lib = _require()
    args, keep, use_slack, max_sources = _rti_args(
        x0, last_u, pref_nom, corridor, horizon, ts, params, limits, weights, vdes,
        boundaries, boundary_max_sources, boundary_hard, boundary_slack_weight,
        boundary_slack_max, warm_start_z, control_period, speed_model, front_offsets)
    n = args.horizon
    zopt = np.empty(2 * n, dtype=np.float64)
    x_pred = np.empty((n, 4), dtype=np.float64)
    v_cmd = np.empty(n, dtype=np.float64) if speed_model is not None else None
    slack = np.empty(max(max_sources, 1), dtype=np.float64)
    buf = _RtiBuffers(
        2 * n, zopt.ctypes.data_as(_DP), 4 * n, x_pred.ctypes.data_as(_DP),
        0 if v_cmd is None else n, None if v_cmd is None else v_cmd.ctypes.data_as(_DP),
        max_sources, slack.ctypes.data_as(_DP))
    res = _RtiResult()
    status = lib.mpc_solve_rti(ctypes.byref(args), ctypes.byref(res), ctypes.byref(buf))
    del keep
    if status != MPC_OK:
        _raise_solver(status, int(res.osqp_error))
    message = res.status_message.decode('utf-8', 'replace')
    info = {
        'success': bool(res.success),
        'status': int(res.status),
        'status_message': message,
        'message': message,
        'cost': float(res.cost),
        'iterations': int(res.iterations),
        'zopt': zopt.copy(),
        'x_pred': x_pred,
        'boundary_slack': ([float(v) for v in slack[:int(res.n_boundary_slack)]]
                           if use_slack else []),
    }
    if speed_model is not None:
        info['v_cmd_pred'] = v_cmd
        info['speed_slack'] = float(res.speed_slack)
    return zopt[:2].copy(), info


def build_qp(x0, last_u, pref_nom, corridor, horizon, ts, params, limits, weights,
             obstacles=None, dmin=None, vdes=0.0, boundaries=None,
             boundary_max_sources=3, boundary_hard=False, boundary_slack_weight=1.0e4,
             boundary_slack_max=0.5, warm_start_z=None, control_period=None,
             speed_model=None, front_offsets=SOLVER_FRONT_POINT_OFFSETS_M):
    """
    Return the tick's QP exactly as the Rust core hands it to OSQP C (debug, parity L1).

    A dict: n, m, n_slack, n_vs; P_indptr, P_indices, P_data (upper triangle,
    CSC, int64 indices), q, A_indptr, A_indices, A_data, l, u (clamped to
    +-OSQP_INFTY), z_guess. Same arguments as solve_rti.
    """
    lib = _require()
    args, keep, _use_slack, _max_sources = _rti_args(
        x0, last_u, pref_nom, corridor, horizon, ts, params, limits, weights, vdes,
        boundaries, boundary_max_sources, boundary_hard, boundary_slack_weight,
        boundary_slack_max, warm_start_z, control_period, speed_model, front_offsets)
    sizes = _QpSizes()
    status = lib.mpc_build_qp(ctypes.byref(args), ctypes.byref(sizes), None)
    if status != MPC_OK:
        _raise_solver(status)
    n, m, nnz_p, nnz_a = int(sizes.n), int(sizes.m), int(sizes.nnz_p), int(sizes.nnz_a)
    out = {
        'P_indptr': np.empty(n + 1, dtype=np.int64),
        'P_indices': np.empty(nnz_p, dtype=np.int64), 'P_data': np.empty(nnz_p),
        'q': np.empty(n),
        'A_indptr': np.empty(n + 1, dtype=np.int64),
        'A_indices': np.empty(nnz_a, dtype=np.int64), 'A_data': np.empty(nnz_a),
        'l': np.empty(m), 'u': np.empty(m), 'z_guess': np.empty(n),
    }
    ip = {k: v.ctypes.data_as(_IP if v.dtype == np.int64 else _DP) for k, v in out.items()}
    buf = _QpBuffers(sizes, ip['P_indptr'], ip['P_indices'], ip['P_data'], ip['q'],
                     ip['A_indptr'], ip['A_indices'], ip['A_data'], ip['l'], ip['u'],
                     ip['z_guess'])
    status = lib.mpc_build_qp(ctypes.byref(args), ctypes.byref(sizes), ctypes.byref(buf))
    del keep
    if status != MPC_OK:
        _raise_solver(status)
    out.update({'n': n, 'm': m, 'n_slack': int(sizes.n_slack), 'n_vs': int(sizes.n_vs)})
    return out


def _settings_dict(s):
    return {name: (float(getattr(s, name)) if is_float else int(getattr(s, name)))
            for name, is_float in OSQP_SETTING_FIELDS}


def osqp_solve_csc(P, q, A, l, u, warm_x=None):  # noqa: E741 - OSQP's own names
    """
    Solve one QP with the Rust core's OSQP and settings (debug, parity layer L2).

    P (upper triangle) and A are CSC matrices (anything with indptr, indices,
    data and shape, e.g. scipy.sparse); l / u as OSQP C should see them. A
    fresh workspace, warm-started in x when warm_x is given, exactly as
    solve_rti runs OSQP. Returns a dict: x (or None), status_val, status,
    status_polish, iter, rho_updates, rho_estimate, obj_val, prim_res,
    dual_res, settings (read back from the solver after setup).
    """
    lib = _require()
    n, m = int(P.shape[1]), int(A.shape[0])
    arrays = {
        'p_colptr': np.ascontiguousarray(P.indptr, dtype=np.int64),
        'p_rowind': np.ascontiguousarray(P.indices, dtype=np.int64),
        'p_val': np.ascontiguousarray(P.data, dtype=np.float64),
        'q': _f64(q),
        'a_colptr': np.ascontiguousarray(A.indptr, dtype=np.int64),
        'a_rowind': np.ascontiguousarray(A.indices, dtype=np.int64),
        'a_val': np.ascontiguousarray(A.data, dtype=np.float64),
        'l': _f64(l), 'u': _f64(u),
    }
    warm = None if warm_x is None else _f64(warm_x)
    if (arrays['q'].size != n or arrays['l'].size != m or arrays['u'].size != m
            or (warm is not None and warm.size != n)):
        raise ValueError('q / l / u / warm_x do not match the matrix sizes')
    ptrs = {k: v.ctypes.data_as(_IP if v.dtype == np.int64 else _DP)
            for k, v in arrays.items()}
    args = _OsqpArgs(n, m, arrays['p_val'].size, arrays['a_val'].size,
                     ptrs['p_colptr'], ptrs['p_rowind'], ptrs['p_val'], ptrs['q'],
                     ptrs['a_colptr'], ptrs['a_rowind'], ptrs['a_val'], ptrs['l'],
                     ptrs['u'], None if warm is None else warm.ctypes.data_as(_DP))
    res = _OsqpResult()
    x = np.empty(n, dtype=np.float64)
    status = lib.mpc_osqp_solve_csc(ctypes.byref(args), ctypes.byref(res),
                                    x.ctypes.data_as(_DP))
    if status != MPC_OK:
        _raise_solver(status, int(res.setup_error))
    return {
        'x': x if res.has_x else None,
        'status_val': int(res.status_val), 'status': res.status.decode('utf-8', 'replace'),
        'status_polish': int(res.status_polish), 'iter': int(res.iter),
        'rho_updates': int(res.rho_updates), 'rho_estimate': float(res.rho_estimate),
        'obj_val': float(res.obj_val), 'prim_res': float(res.prim_res),
        'dual_res': float(res.dual_res), 'settings': _settings_dict(res.settings),
    }


def osqp_settings(which='solver'):
    """Return the OSQP settings 'solver' sets, or OSQP C's 'default' ones, as a dict."""
    lib = _require()
    codes = {'solver': 0, 'default': 1}
    if which not in codes:
        raise ValueError(f'which must be one of {tuple(codes)}, got {which!r}')
    out = _OsqpSettings()
    status = lib.mpc_osqp_settings(codes[which], ctypes.byref(out))
    if status != MPC_OK:
        _raise_for(status)
    return _settings_dict(out)


def osqp_version():
    """Return the OSQP C version linked into the library ('1.0.0')."""
    lib = _require()
    buf = ctypes.create_string_buffer(64)
    lib.mpc_osqp_version(buf, 64)
    return buf.value.decode()
