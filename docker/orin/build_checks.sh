#!/bin/bash
# Build-time checks of the Orin Jazzy image (run in the Dockerfile's ws stage
# as root, with the host's L4T libs bind-mounted at
# /usr/lib/aarch64-linux-gnu/nvidia and /opt/ros/jazzy + the workspace sourced).
# Any failure fails the build. Environment (from the Dockerfile's ARGs):
#   USER_NAME USER_UID ZED_GID JTOP_GID VIDEO_GID DIALOUT_GID RENDER_GID INPUT_GID RUST
set -u
# No __pycache__ from these imports in the image's source tree.
export PYTHONDONTWRITEBYTECODE=1
fail=0
bad() { echo "BUILD CHECK FAILED: $*" >&2; fail=1; }
WS=/opt/f1tenth_ws
VENV=/opt/f1tenth_gateway/venv
USER_NAME=${USER_NAME:-fabiocar}

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

echo "== engine export deps: onnx, onnxslim, onnxruntime at the pinned versions"
python3 -c '
import sys, onnx, onnxslim, onnxruntime
got = {"onnx": onnx.__version__, "onnxslim": onnxslim.__version__, "onnxruntime": onnxruntime.__version__}
want = {"onnx": "1.22.0", "onnxslim": "0.1.94", "onnxruntime": "1.23.2"}
print("  ", got)
sys.exit(0 if got == want else f"expected {want}")
' || bad "onnx / onnxslim / onnxruntime"

echo "== gateway venv $VENV: exactly gateway-requirements.txt, pip check, imports, no PyPI CUDA"
if [ -x "$VENV/bin/python" ]; then
  "$VENV/bin/pip" check || bad "gateway venv: pip check"
  site=$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
  # name==version lines, names normalised as pip compares them (PEP 503).
  norm() { awk -F'==' '{n = tolower($1); gsub(/[-_.]+/, "-", n); print n "==" $2}' | sort; }
  got=$("$VENV/bin/pip" list --path "$site" --format=freeze 2>/dev/null | grep -v -i -E '^(pip|setuptools|wheel)==' | norm)
  want=$(sed -e 's/#.*//' -e 's/[[:space:]]//g' /opt/f1tenth_gateway/requirements.txt | grep . | norm)
  if [ "$got" != "$want" ]; then
    bad "gateway venv differs from gateway-requirements.txt: $(diff <(echo "$want") <(echo "$got") | grep '^[<>]' | tr '\n' ' ')"
  else
    echo "  $(echo "$got" | wc -l) packages, equal to gateway-requirements.txt"
  fi
  if echo "$got" | grep -E '^(nvidia-|cuda-|triton)'; then bad "PyPI CUDA package in the gateway venv"; fi
  "$VENV/bin/python" -c '
import fastapi, uvicorn, websockets, multipart, qrcode, faster_whisper, ctranslate2, av, tokenizers, huggingface_hub
from faster_whisper.vad import get_vad_model
get_vad_model()
print("  fastapi", fastapi.__version__, "| uvicorn", uvicorn.__version__, "| faster_whisper", faster_whisper.__version__,
      "| ctranslate2", ctranslate2.__version__, "| av", av.__version__, "| Silero VAD loads")
' || bad "gateway venv: imports"
  # The gateway's own modules with the venv's interpreter and the ROS
  # environment (rclpy from /opt/ros/jazzy), as bin/f1tenth_gateway runs them.
  PYTHONPATH="$WS/src/f1tenth_gateway:$WS/src/f1tenth_ctl:$WS/src/f1tenth_telegram${PYTHONPATH:+:$PYTHONPATH}" \
    "$VENV/bin/python" -c 'import f1tenth_gateway.app, f1tenth_gateway.ros_bridge, f1tenth_gateway.transcribe_worker; print("  f1tenth_gateway modules import")' \
    || bad "f1tenth_gateway does not import with the venv"
else
  bad "no gateway venv at $VENV"
fi

echo "== ffmpeg (transcription fallback, mission videos)"
if command -v ffmpeg > /dev/null; then
  ffmpeg -hide_banner -version 2>/dev/null | head -1 | sed 's/^/  /'
  ffmpeg -hide_banner -decoders 2>/dev/null | grep -q ' opus ' || bad "ffmpeg has no opus decoder (Telegram and PWA voice is OGG/Opus)"
else
  bad "ffmpeg missing"
fi

echo "== group IDs: the host's (build.sh passes them; compose's group_add relies on them)"
for pair in "video:${VIDEO_GID:-}" "render:${RENDER_GID:-}" "dialout:${DIALOUT_GID:-}" "input:${INPUT_GID:-}" \
            "zed:${ZED_GID:-}" "jtop:${JTOP_GID:-}"; do
  name=${pair%%:*}; want=${pair#*:}
  have=$(getent group "$name" | cut -d: -f3)
  echo "  $name ${have:-missing}${want:+ (asked $want)}"
  if [ -z "$have" ]; then bad "group $name missing"; continue; fi
  [ -z "$want" ] || [ "$have" = "$want" ] || bad "group $name has GID $have, the build asked for $want"
done
user_groups=" $(id -nG "$USER_NAME" 2>/dev/null) "
for g in video render dialout input zed jtop; do
  case "$user_groups" in *" $g "*) ;; *) bad "$USER_NAME is not in group $g" ;; esac
done
[ "$(id -u "$USER_NAME" 2>/dev/null)" = "${USER_UID:-1004}" ] || bad "$USER_NAME uid is not ${USER_UID:-1004}"

echo "== writable directories owned by $USER_NAME"
for d in "$WS/src/f1tenth_control/corridors_jsons" "$WS/src/f1tenth_behavior/mission_reports" "$WS/mission_videos" \
         "$WS/install/share/f1tenth_behavior/missions/llm_generated" /var/cache/f1tenth/cuda /var/cache/f1tenth/trt \
         /usr/local/zed/resources \
         "/home/$USER_NAME/.config/f1tenth_gateway" "/home/$USER_NAME/.config/f1tenth_car" \
         "/home/$USER_NAME/.local/state/f1tenth_ctl" "/home/$USER_NAME/.local/state/f1tenth_gateway" \
         "/home/$USER_NAME/.local/state/f1tenth_archive_trigger" "/home/$USER_NAME/.local/share/f1tenth_gateway/www" \
         "/home/$USER_NAME/.cache/huggingface" "/home/$USER_NAME/.ros" "/home/$USER_NAME/f1tenth_archive" \
         "/home/$USER_NAME/projects/llm/models"; do
  if [ ! -d "$d" ]; then bad "missing directory $d"; continue; fi
  [ "$(stat -c %U "$d")" = "$USER_NAME" ] || bad "$d is owned by $(stat -c %U:%G "$d"), not $USER_NAME"
done
# No directory in HOME that the user does not own (a root-owned mount parent).
not_owned=$(find "/home/$USER_NAME" -xdev -type d ! -user "$USER_NAME" 2>/dev/null)
[ -z "$not_owned" ] || bad "directories under /home/$USER_NAME not owned by $USER_NAME: $(echo "$not_owned" | tr '\n' ' ')"

echo "== f1tenth_ctl in the container: ctl_container.env selects the proc backend, no comms"
ctl_env=$WS/src/f1tenth_ctl/config/ctl_container.env
if [ -f "$ctl_env" ]; then
  # As f1tenth_ctl sources it: bash, set -a, on a minimal environment.
  # shellcheck disable=SC2016  # expanded by the inner bash, after sourcing
  ctl_vals=$(env -i HOME="/home/$USER_NAME" USER="$USER_NAME" PATH=/usr/bin:/bin bash -c \
    'set -a; . "$1" > /dev/null 2>&1; printf "%s\n" "backend=${F1T_PROCESS_BACKEND:-}" "stack=${F1T_STACK_CMD:-}"' _ "$ctl_env")
  printf '  %s\n' "$ctl_vals"
  echo "$ctl_vals" | grep -qx 'backend=proc' || bad "ctl_container.env does not set F1T_PROCESS_BACKEND=proc"
  echo "$ctl_vals" | grep -q '^stack=ros2 launch f1tenth_bringup supervisor_bringup.launch.py' \
    || bad "ctl_container.env: F1T_STACK_CMD is not the supervisor bringup"
  if echo "$ctl_vals" | grep -q 'enable_comms:=true'; then
    bad "ctl_container.env turns comms on (f1tenth_comms needs the host's NetworkManager; not supported in v1)"
  fi
else
  bad "missing $ctl_env"
fi
python3 - <<'EOF' || bad "installed stack_params.yaml: enable_comms must default to false"
import yaml
d = yaml.safe_load(open('/opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml'))
print('  installed enable_comms default:', d['enable_comms']['default'])
raise SystemExit(0 if d['enable_comms']['default'] is False else 1)
EOF

echo "== every built package is in the ament index"
skip=(f1tenth_sim f1tenth_more husarion_gz_worlds)
if [ "${RUST:-skip}" != build ]; then
  read -r -a rust_pkgs < "$WS/rust_packages.txt" 2>/dev/null || rust_pkgs=()
  skip+=("${rust_pkgs[@]}")
fi
expected=$(cd "$WS" && colcon --log-base /dev/null list -n --packages-skip "${skip[@]}" 2>/dev/null | sort)
indexed=$(find "$WS/install/share/ament_index/resource_index/packages" -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | sort)
missing=$(comm -23 <(echo "$expected") <(echo "$indexed"))
echo "  $(echo "$expected" | grep -c .) built, $(echo "$indexed" | grep -c .) in the index"
[ -n "$expected" ] || bad "colcon list found no packages in $WS/src"
[ -z "$missing" ] || bad "packages not in the ament index: $(echo "$missing" | tr '\n' ' ')"

echo "== CUDA nvJPEG dev symlink absent (F3)"
[ ! -e /usr/local/cuda/lib64/libnvjpeg.so ] || bad "/usr/local/cuda/lib64/libnvjpeg.so exists (shadows L4T libnvjpeg.so)"

echo "== CUDA static libraries dropped except cudart_static, cudadevrt, culibos (L17)"
left=$(find /usr/local/cuda-12.9 -name 'lib*_static*.a' ! -name 'libcudart_static.a')
[ -z "$left" ] || bad "static libraries left: $(echo "$left" | tr '\n' ' ')"
for f in libcudart_static.a libcudadevrt.a; do
  [ -e "/usr/local/cuda-12.9/targets/aarch64-linux/lib/$f" ] || bad "$f missing"
done
echo "  kept: $(find /usr/local/cuda-12.9/targets/aarch64-linux/lib -maxdepth 1 \( -name libcudart_static.a -o -name libcudadevrt.a -o -name libculibos.a \) -printf '%f ')"

echo "== /usr/local/zed group is the host zed GID (F5)"
[ "$(stat -c %g /usr/local/zed)" = "${ZED_GID}" ] || bad "/usr/local/zed group $(stat -c %g /usr/local/zed) != ${ZED_GID}"

echo "== container files (docker/orin/container, the engine export script)"
for f in "$WS/docker/orin/container/f1tenth_container_main.py" "$WS/docker/orin/container/healthcheck.sh" \
         "$WS/docker/orin/container/gateway.yaml" "$WS/scripts/export_yolo_engine.sh"; do
  [ -f "$f" ] || bad "missing $f"
done
[ -x "$WS/docker/orin/container/healthcheck.sh" ] || bad "healthcheck.sh is not executable"
python3 -c 'import ast, sys; ast.parse(open(sys.argv[1]).read())' "$WS/docker/orin/container/f1tenth_container_main.py" \
  || bad "f1tenth_container_main.py does not parse"
bash -n "$WS/scripts/export_yolo_engine.sh" || bad "export_yolo_engine.sh: syntax error"

echo "== Rust (RUST=${RUST:-skip})"
# RUST=build: each replay binary prints its build on stderr (build_info.rs);
# an opt-level 0 build is the regression R0 fixed. No Rust binary may need
# test_msgs at run time (the ros2-rust#557 workaround goes with the rclrs 0.8
# port, plan P3.1). The node binaries are in the ldd -r scan below. RUST=skip:
# the ament_cargo packages were ignored.
if [ "${RUST:-skip}" = build ]; then
  for pb in f1tenth_proximity_guard/proximity_guard_node f1tenth_proximity_guard/proximity_guard_replay \
            f1tenth_costmap_boundary_rs/costmap_boundary_node f1tenth_costmap_boundary_rs/costmap_boundary_replay \
            f1tenth_hard_stop_rs/hard_stop_node f1tenth_hard_stop_rs/hard_stop_replay; do
    if [ ! -x "$WS/install/lib/$pb" ]; then bad "Rust binary missing: install/lib/$pb"; continue; fi
    if ldd "$WS/install/lib/$pb" 2>/dev/null | grep -q 'libtest_msgs'; then
      bad "$pb links test_msgs (a run-time dependency the rclrs 0.8 port removes)"
    fi
  done
  for pb in f1tenth_proximity_guard/proximity_guard_replay f1tenth_costmap_boundary_rs/costmap_boundary_replay \
            f1tenth_hard_stop_rs/hard_stop_replay; do
    build=$(echo | "$WS/install/lib/$pb" 2>&1 >/dev/null | head -1)
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
  # The wall tracker library (R6): lidar_front_wall_node loads it with ctypes
  # when wall_tracker_impl is rust; the same check (ABI, struct layouts,
  # parameter lists, opt-level 3).
  wall_core=$(python3 -c 'from f1tenth_perception import wall_tracker_native as m; print(m.available(), m.load_error(), m.build_info())' 2>&1 | tail -1)
  echo "  f1tenth_wall_tracker_core: $wall_core"
  case "$wall_core" in "True None "*"opt-level 3,"*) ;; *) bad "f1tenth_wall_tracker_core not loadable or not opt-level 3: $wall_core" ;; esac
fi

echo "== ldd -r scan (CLAUDE.md): workspace install, ZED SDK, llama.cpp; must print nothing"
scan=$(find "$WS/install" /usr/local/zed/lib /opt/llama.cpp \( -path '*/lib/*' -o -path '*/bin/*' \) \( -type f -o -type l \) 2>/dev/null |
  while read -r f; do
    file -L "$f" | grep -q ELF || continue
    ldd -r "$f" 2>&1 | grep 'undefined symbol\|not found' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' | sed "s|^|$f: |"
  done)
if [ -n "$scan" ]; then echo "$scan" | head -40; bad "ldd -r scan not empty"; fi

[ $fail = 0 ] && echo "== all build checks passed"
exit $fail
