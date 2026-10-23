"""
corridor_backend.py -- which implementation builds MPC_corr's corridor geometry.

corridor_impl (stack_params.yaml; mpc_corr.launch.py / navigation.launch.py /
supervisor_bringup.launch.py / stack_bringup.launch.py corridor_impl:=py|rust):

  'py'    f1tenth_params.corridor_geometry, the reference (the default);
  'rust'  f1tenth_mpc_core (src/rust/f1tenth_mpc_core) through mpc_native.py:
          the same functions ported to Rust, bit-identical on linus
          (output/rust/parity_corridor.md).

Both return the SAME dict (keys, key order, value types; 'ny' is the 'tx'
array in both), so build_straight_corridor does not know which one ran.
reverse_manoeuvre.py, wall_turn.py and object_approach.py keep calling the
Python directly: their geometry is microseconds of scalar math or a one-off.

THE DEFAULT. DEFAULT_IMPL is what a caller without an explicit impl gets: the
corridor tests bind build_straight_corridor onto duck-typed stand-ins that
carry no corridor_impl attribute, and MPC_corr reads it with
getattr(self, 'corridor_impl', corridor_backend.DEFAULT_IMPL). It is the
stack_params.yaml value, unless the environment variable F1TENTH_CORRIDOR_IMPL
is set: that is the test hook that runs every corridor test, stand-ins and real
nodes alike, against the Rust library (`F1TENTH_CORRIDOR_IMPL=rust colcon test
...`). The node also takes its parameter default from default_impl(), so the
hook reaches a node built without overrides; an explicit corridor_impl
parameter always wins.

RUST REQUESTED BUT NOT LOADABLE IS A STARTUP ERROR (require()), never a silent
fall back to the Python: a launch line that says rust must run rust.
"""

import os

from f1tenth_params import corridor_geometry
from f1tenth_params.param_defaults import get_value

from mpc_controller import mpc_native

#: The accepted corridor_impl values.
IMPLS = ('py', 'rust')

#: Test hook: overrides the stack_params.yaml default (see the module docstring).
ENV_IMPL = 'F1TENTH_CORRIDOR_IMPL'


def check_impl(impl):
    """Raise ValueError for a value corridor_impl does not accept."""
    if impl not in IMPLS:
        raise ValueError(f'corridor_impl must be one of {IMPLS}, got {impl!r}')
    return impl


def default_impl():
    """F1TENTH_CORRIDOR_IMPL if set, else stack_params.yaml's corridor_impl."""
    env = os.environ.get(ENV_IMPL, '').strip()
    return check_impl(env or str(get_value('corridor_impl')))


#: Read once, at import. Tests may assign it; the call sites read it per call.
DEFAULT_IMPL = default_impl()


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
            'corridor_impl:=rust but the Rust corridor library is not usable: '
            f'{mpc_native.load_error()}. Build it (colcon build --packages-select '
            'f1tenth_mpc_core) or run corridor_impl:=py.')
    return impl


def describe(impl):
    """One line for the startup log: which geometry runs, and from where."""
    if impl == 'rust':
        return (f'rust (f1tenth_mpc_core {mpc_native.version()}, '
                f'{mpc_native.library_path()}, {mpc_native.build_info()})')
    return 'py (f1tenth_params.corridor_geometry)'


def _impl(impl):
    return DEFAULT_IMPL if impl is None else impl


def corridor_curves(*args, impl=None, **kwargs):
    """corridor_geometry.corridor_curves by the selected implementation."""
    impl = _impl(impl)
    if impl == 'py':
        return corridor_geometry.corridor_curves(*args, **kwargs)
    if impl == 'rust':
        return mpc_native.corridor_curves(*args, **kwargs)
    raise ValueError(f'corridor_impl must be one of {IMPLS}, got {impl!r}')


def corridor_curves_to_pose(*args, impl=None, **kwargs):
    """corridor_geometry.corridor_curves_to_pose by the selected implementation."""
    impl = _impl(impl)
    if impl == 'py':
        return corridor_geometry.corridor_curves_to_pose(*args, **kwargs)
    if impl == 'rust':
        return mpc_native.corridor_curves_to_pose(*args, **kwargs)
    raise ValueError(f'corridor_impl must be one of {IMPLS}, got {impl!r}')


def nearest_centerline_index(p, corridor, impl=None):
    """
    mpc_solver.nearest_centerline_index by the selected implementation.

    Not on the controller's path in R1 (the Python solver calls its own); here
    for the parity harness and the R2 solver core.
    """
    impl = _impl(impl)
    if impl == 'py':
        from mpc_controller import mpc_solver  # noqa: PLC0415 - solver imports are heavy
        return mpc_solver.nearest_centerline_index(p, corridor)
    if impl == 'rust':
        return mpc_native.nearest_centerline_index(p, corridor)
    raise ValueError(f'corridor_impl must be one of {IMPLS}, got {impl!r}')
