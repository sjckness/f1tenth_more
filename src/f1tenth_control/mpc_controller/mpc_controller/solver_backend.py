"""
solver_backend.py -- which implementation solves the MPC's RTI tick.

mpc_solver_impl (stack_params.yaml; mpc_corr.launch.py / navigation.launch.py /
supervisor_bringup.launch.py / stack_bringup.launch.py mpc_solver_impl:=py|rust):

  'py'    mpc_solver._solve_rti, the reference (the default);
  'rust'  f1tenth_mpc_core's solver core (src/rust/f1tenth_mpc_core) through
          mpc_native.solve_rti: the same QP built bit for bit, the same OSQP C
          v1.0.0 with the same settings, the same rollout and true cost
          (output/rust/parity_mpc_solver.md).

ONLY THE FORWARD RTI TICK. mpc_solver.solve_mpc_step sends a call to Rust when
the implementation is 'rust', solver is 'rti' and stage_spec is None (both speed
models). SLSQP and the stage_spec paths (reverse_mpc's direction candidates,
reverse_manoeuvre.py) always run the Python. Both return the same (u0, info).

THE DEFAULT. DEFAULT_IMPL is what solve_mpc_step uses without an explicit impl=
(the closed-loop rigs, the tests, every stand-in). It is the stack_params.yaml
value unless the environment variable F1TENTH_MPC_SOLVER_IMPL is set: the test
hook that runs every solver test against the Rust core
(`F1TENTH_MPC_SOLVER_IMPL=rust colcon test ...`). MPC_corr declares its
mpc_solver_impl parameter with default_impl() as the default, so the hook
reaches a node built without overrides; an explicit parameter always wins.

RUST REQUESTED BUT NOT LOADABLE IS A STARTUP ERROR (require()) in the node, and a
RuntimeError per call elsewhere: never a silent fall back to the Python.

CAPTURE (parity tooling, off by default). With F1TENTH_MPC_CAPTURE set to a
directory, every Rust-eligible tick (rti, stage_spec None) appends its inputs
and outputs to <dir>/rti_capture_<pid>.jsonl.gz, one gzip member per line, in
the record format of tools/rust_parity/mpc/rti_fixture.py (inputs with the
UNSCALED weights, the corridor cut to the solver's keys; plus 'impl'). Capped
at F1TENTH_MPC_CAPTURE_MAX records per process (default 20000). The capture
only reads; the solve and its result are unchanged.
"""

import gzip
import json
import math
import os

from f1tenth_params.param_defaults import get_value
from mpc_controller import mpc_native
import numpy as np

#: The accepted mpc_solver_impl values.
IMPLS = ('py', 'rust')

#: Test hook: overrides the stack_params.yaml default (see the module docstring).
ENV_IMPL = 'F1TENTH_MPC_SOLVER_IMPL'

#: Capture directory (see the module docstring), and its per-process cap.
ENV_CAPTURE = 'F1TENTH_MPC_CAPTURE'
ENV_CAPTURE_MAX = 'F1TENTH_MPC_CAPTURE_MAX'


def check_impl(impl):
    """Raise ValueError for a value mpc_solver_impl does not accept."""
    if impl not in IMPLS:
        raise ValueError(f'mpc_solver_impl must be one of {IMPLS}, got {impl!r}')
    return impl


def default_impl():
    """F1TENTH_MPC_SOLVER_IMPL if set, else stack_params.yaml's mpc_solver_impl."""
    env = os.environ.get(ENV_IMPL, '').strip()
    return check_impl(env or str(get_value('mpc_solver_impl')))


#: Read once, at import. Tests may assign it; solve_mpc_step reads it per call.
DEFAULT_IMPL = default_impl()


def resolve(impl):
    """Return the implementation a call runs: DEFAULT_IMPL for None, else impl (checked)."""
    return DEFAULT_IMPL if impl is None else check_impl(impl)


def runs_rust(impl, solver, stage_spec):
    """Report whether a solve_mpc_step call goes to the Rust core."""
    return impl == 'rust' and solver == 'rti' and stage_spec is None


def require(impl):
    """
    For a node's startup: a valid impl whose backend is usable, or raise.

    ValueError for an unknown value; RuntimeError for 'rust' when
    libf1tenth_mpc_core.so does not load (missing, wrong ABI, wrong struct
    layout), with the loader's reason.
    """
    check_impl(impl)
    if impl == 'rust' and not mpc_native.available():
        raise RuntimeError(
            'mpc_solver_impl:=rust but the Rust solver core is not usable: '
            f'{mpc_native.load_error()}. Build it (colcon build --packages-select '
            'f1tenth_mpc_core) or run mpc_solver_impl:=py.')
    return impl


def describe(impl):
    """One line for the startup log: which solver runs, and from where."""
    if impl == 'rust':
        return (f'rust for rti ticks without stage_spec (f1tenth_mpc_core '
                f'{mpc_native.version()}, OSQP C {mpc_native.osqp_version()}, '
                f'{mpc_native.library_path()}, {mpc_native.build_info()}); '
                'slsqp and stage_spec ticks stay py')
    return 'py (mpc_solver._solve_rti)'


# ---------------------------------------------------------------------------
# capture (tools/rust_parity/mpc: live ticks for the parity replay)
# ---------------------------------------------------------------------------

#: The corridor keys the solver reads (rti_fixture.CORRIDOR_KEYS).
_CORRIDOR_KEYS = ('xc', 'yc', 'nx', 'ny', 'halfWidth', 'psiRef', 'psiRefTurn', 'psiStart',
                  'car_radius', 'avoidance_margin', 'targetLine', 'obstacles_world')
_OUTPUT_KEYS = ('success', 'status', 'status_message', 'message', 'cost', 'iterations',
                'zopt', 'x_pred', 'boundary_slack', 'v_cmd_pred', 'speed_slack')

_capture_state = {'count': 0}


def capture_dir():
    """Return the capture directory, or None when capture is off (read per call)."""
    return os.environ.get(ENV_CAPTURE) or None


def _jsonable(obj):
    """Return obj as JSON-safe values (rti_fixture.jsonable's encoding)."""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        v = float(obj)
        if math.isfinite(v):
            return v
        return {'$float': 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')}
    if isinstance(obj, np.ndarray):
        return _jsonable(obj.tolist())
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if hasattr(obj, 'value') and hasattr(obj, 'name'):
        return int(obj.value)
    return repr(obj)


def capture(inputs, impl, u0, info):
    """Append one tick to this process's capture file (when capture is on)."""
    directory = capture_dir()
    if directory is None:
        return
    limit = int(os.environ.get(ENV_CAPTURE_MAX, '20000'))
    if _capture_state['count'] >= limit:
        return
    _capture_state['count'] += 1
    enc = dict(inputs)
    enc['corridor'] = {k: inputs['corridor'][k] for k in _CORRIDOR_KEYS
                       if k in inputs['corridor']}
    outputs = {'u0': np.asarray(u0, dtype=float)}
    outputs.update({k: info[k] for k in _OUTPUT_KEYS if k in info})
    record = {'id': f'capture/{os.getpid()}/{_capture_state["count"]:06d}',
              'group': 'capture', 'impl': impl,
              'inputs': _jsonable(enc), 'outputs': _jsonable(outputs)}
    line = json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n'
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, f'rti_capture_{os.getpid()}.jsonl.gz')
    with gzip.open(path, 'at', encoding='utf-8') as fh:
        fh.write(line)
