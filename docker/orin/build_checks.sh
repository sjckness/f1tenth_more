#!/bin/bash
# Build-time checks of the Orin Jazzy image (run in the Dockerfile's ws stage,
# with the host's L4T libs bind-mounted at /usr/lib/aarch64-linux-gnu/nvidia).
# Any failure fails the build.
set -u
fail=0
bad() { echo "BUILD CHECK FAILED: $*" >&2; fail=1; }

echo "== pip check"
pip check || bad "pip check"

echo "== no PyPI CUDA (nvidia-*, cuda-*, triton; nvidia-ml-py is pure-Python NVML bindings)"
cuda_pkgs=$(pip list --format=freeze 2>/dev/null | grep -i -E '^(nvidia-|cuda-|triton)' | grep -v -i '^nvidia-ml-py==' || true)
[ -z "$cuda_pkgs" ] || bad "PyPI CUDA packages installed: $cuda_pkgs"

echo "== apt Python not shadowed by pip (Thor set, F6)"
python3 - <<'EOF' || bad "apt Python shadowed"
import importlib, sys
bad = []
for mod in ('numpy', 'scipy', 'matplotlib', 'jsonschema', 'requests', 'yaml', 'psutil', 'pydot', 'ruamel.yaml'):
    m = importlib.import_module(mod)
    where = getattr(m, '__file__', '') or ''
    ok = where.startswith('/usr/lib/python3/dist-packages/')
    print(f'  {mod:12s} {getattr(m, "__version__", "?"):10s} {where}')
    if not ok:
        bad.append(mod)
sys.exit(f'not from apt: {bad}' if bad else 0)
EOF

echo "== stack Python closure imports"
python3 -c '
import numpy, scipy, osqp, cv2, torch, torchvision, ultralytics, pyarrow, psutil, jsonschema
import matplotlib, pydot, yaml, ruamel.yaml, requests, py_trees, jtop, tensorrt
import rclpy, cv_bridge, rosbag2_py
print("  torch", torch.__version__, "cuda", torch.version.cuda, "| tensorrt", tensorrt.__version__,
      "| ultralytics", ultralytics.__version__, "| cv2", cv2.__version__, "| numpy", numpy.__version__)
' || bad "Python closure import"

echo "== CUDA nvJPEG dev symlink absent (F3)"
[ ! -e /usr/local/cuda/lib64/libnvjpeg.so ] || bad "/usr/local/cuda/lib64/libnvjpeg.so exists (shadows L4T libnvjpeg.so)"

echo "== /usr/local/zed group is the host zed GID (F5)"
[ "$(stat -c %g /usr/local/zed)" = "${ZED_GID}" ] || bad "/usr/local/zed group $(stat -c %g /usr/local/zed) != ${ZED_GID}"

echo "== Rust binaries (RUST=${RUST:-skip}; Dockerfile rust-build stage, NOT BUILT YET)"
# RUST=build: each replay binary prints its build on stderr (build_info.rs);
# an opt-level 0 build is the regression R0 fixed. The node binaries are in
# the ldd -r scan below. RUST=skip: the ament_cargo packages were ignored.
if [ "${RUST:-skip}" = build ]; then
  for pb in f1tenth_proximity_guard/proximity_guard_node f1tenth_proximity_guard/proximity_guard_replay \
            f1tenth_costmap_boundary_rs/costmap_boundary_node f1tenth_costmap_boundary_rs/costmap_boundary_replay \
            f1tenth_hard_stop_rs/hard_stop_node f1tenth_hard_stop_rs/hard_stop_replay; do
    [ -x "/opt/f1tenth_ws/install/lib/$pb" ] || bad "Rust binary missing: install/lib/$pb"
  done
  for pb in f1tenth_proximity_guard/proximity_guard_replay f1tenth_costmap_boundary_rs/costmap_boundary_replay \
            f1tenth_hard_stop_rs/hard_stop_replay; do
    build=$(echo | "/opt/f1tenth_ws/install/lib/$pb" 2>&1 >/dev/null | head -1)
    echo "  $pb: $build"
    case "$build" in *"opt-level 3,"*) ;; *) bad "$pb not built at opt-level 3: $build" ;; esac
  done
  # The corridor library (R1): mpc_controller loads it with ctypes when
  # corridor_impl is rust; check that the loader accepts it (ABI version,
  # struct sizes) and that it is an opt-level 3 build.
  mpc_core=$(python3 -c 'from mpc_controller import mpc_native as m; print(m.available(), m.load_error(), m.build_info())' 2>&1 | tail -1)
  echo "  f1tenth_mpc_core: $mpc_core"
  case "$mpc_core" in "True None "*"opt-level 3,"*) ;; *) bad "f1tenth_mpc_core not loadable or not opt-level 3: $mpc_core" ;; esac
  # The solver core (R2, same library): OSQP C 1.0.0 linked in, and one RTI
  # tick through mpc_solver_impl rust agrees with the Python bit for bit (an
  # aarch64 build may differ in the last bits: then this prints the largest
  # difference instead of failing; the tolerance gates are in
  # output/rust/parity_mpc_solver.md).
  solver=$(python3 - <<'PY' 2>&1 | tail -1
import numpy as np
from f1tenth_params import corridor_geometry as cg
from mpc_controller import mpc_native, mpc_solver
c = cg.corridor_curves(0.0, 0.0, 0.0, 0.5, 3.0, 120, r_min=0.923)
cor = {k: c[k] for k in ('xc', 'yc', 'nx', 'ny', 'halfWidth')}
cor.update(psiRef=0.5, obstacles_world=[(1.4, 0.15, 0.18)], car_radius=0.2, avoidance_margin=0.12)
kw = dict(x0=[0.0, 0.0, 0.0, 0.3], last_u=[0.0, 0.0], pref_nom=[2.5, 0.8], corridor=cor,
          horizon=20, ts=0.1, params={'L': 0.305, 'lr': 0.17},
          limits={'delta_min': -0.28, 'delta_max': 0.28, 'a_min': -2.0, 'a_max': 3.0,
                  'dDeltaMin': -0.5, 'dDeltaMax': 0.5, 'dAMin': -2.0, 'dAMax': 2.0,
                  'vMin': 0.0, 'vMax': 1.0},
          weights={'w_term': 9.0, 'w_v': 4.3, 'w_psi': 4.5, 'w_u_a': 0.0, 'w_du_delta': 8.6,
                   'w_du_a': 0.0, 'w_delta0': 0.0, 'w_obs': 8.0},
          obstacles=[], dmin=0.32, vdes=0.5, boundaries=[(0.0, 1.0, 0.9)],
          boundary_max_sources=4, control_period=0.1)
u_py, i_py = mpc_solver.solve_mpc_step(**kw, impl='py')
u_rs, i_rs = mpc_solver.solve_mpc_step(**kw, impl='rust')
d = float(np.max(np.abs(i_py['zopt'] - i_rs['zopt'])))
print(mpc_native.osqp_version(), i_py['status'] == i_rs['status'], d)
PY
)
  echo "  f1tenth_mpc_core solver (OSQP, status equal, max |dz|): $solver"
  case "$solver" in "1.0.0 True "*) ;; *) bad "f1tenth_mpc_core solver core check failed: $solver" ;; esac
fi

echo "== ldd -r scan (CLAUDE.md): workspace install, ZED SDK, llama.cpp; must print nothing"
scan=$(find /opt/f1tenth_ws/install /usr/local/zed/lib /opt/llama.cpp \( -path '*/lib/*' -o -path '*/bin/*' \) \( -type f -o -type l \) 2>/dev/null |
  while read -r f; do
    file -L "$f" | grep -q ELF || continue
    ldd -r "$f" 2>&1 | grep 'undefined symbol\|not found' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' | sed "s|^|$f: |"
  done)
if [ -n "$scan" ]; then echo "$scan" | head -40; bad "ldd -r scan not empty"; fi

[ $fail = 0 ] && echo "== all build checks passed"
exit $fail
