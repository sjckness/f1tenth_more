#!/usr/bin/env bash
# build_hydra_ws.sh -- Hydra (MIT-SPARK) for ROS 2 Jazzy, built from the pinned submodules.
#
# Usage: scripts/hydra/build_hydra_ws.sh [--ws DIR] [--jobs N] [--models] [--check]
#   --ws DIR   build/install space (default ~/hydra_ws); the SOURCES are in this repo
#   --jobs N   parallel build jobs (default: nproc; Hydra needs ~2 GB RAM per job)
#   --models   also download the MobileNetV2 ADE20K ONNX model to ~/.semantic_inference
#   --check    only report what is missing (no build)
#
# The sources are the submodules in third_party/hydra_ros/, pinned to the SHAs in
# src/f1tenth_hydra/config/hydra/hydra_jazzy.repos (still the single source of truth for the
# version; scripts/hydra/check_hydra_pins.py fails if the two diverge). Nothing is cloned here:
#   git submodule update --init third_party/hydra_ros
#
# Never uses sudo. The apt/rosdep/TensorRT steps that need it are printed and listed LAST in
# docs/hydra_hil.md ("sudo steps"). Run them once, then this script.
#
# third_party/ carries COLCON_IGNORE, so the stack's own `colcon build` never builds Hydra or
# GTSAM; this script builds it on purpose with --base-paths, into its own build/install space.
# To run: source ~/hydra_ws/install/setup.bash, THEN the f1tenth workspace.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
REPOS="$REPO/src/f1tenth_hydra/config/hydra/hydra_jazzy.repos"
SRC="$REPO/third_party/hydra_ros"
WS="$HOME/hydra_ws"; JOBS="$(nproc)"; MODELS=false; CHECK=false
MOBILENET_URL='https://www.dropbox.com/scl/fi/dlcizojblgaq8dnhd94o4/ade20k-mobilenetv2dilated-c1_deepsup.onnx?rlkey=5m7tv9x8bt0gsg1q77tert9ic&st=ixu66280&dl=1'
while [ $# -gt 0 ]; do
  case "$1" in
    --ws) WS="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --models) MODELS=true; shift ;;
    --check) CHECK=true; shift ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
  esac
done
log() { echo "[build_hydra_ws] $*"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash || { echo "ROS 2 Jazzy not found in /opt/ros/jazzy" >&2; exit 1; }
missing=()
for c in colcon rosdep; do command -v "$c" >/dev/null || missing+=("$c"); done
dpkg -s ros-jazzy-gtsam >/dev/null 2>&1 || missing+=("ros-jazzy-gtsam (rosdep installs it)")
dpkg -s libnvinfer-dev >/dev/null 2>&1 || log "TensorRT (libnvinfer-dev) not installed: seg_source:=mobilenet will not work (gt still does)"
command -v nvcc >/dev/null || [ -x /usr/local/cuda/bin/nvcc ] || log "nvcc not found: semantic_inference builds without CUDA/TensorRT"

# the pinned sources: submodules, checked out, at the SHAs the .repos names
uninit=()
while read -r name; do
  [ -e "$SRC/$name/.git" ] || uninit+=("$name")
done < <(git -C "$REPO" config -f .gitmodules --get-regexp 'submodule\..*\.path' \
         | awk '$2 ~ /^third_party\/hydra_ros\//{ n = split($2, p, "/"); print p[n] }')
if [ ${#uninit[@]} -gt 0 ]; then
  log "submodules not initialized: ${uninit[*]}"
  log "run:  git -C $REPO submodule update --init third_party/hydra_ros"
  missing+=("third_party/hydra_ros submodules")
elif ! "$HERE/check_hydra_pins.py" --quiet; then
  log "the submodules do not match $REPOS (see above)"
  missing+=("hydra pins")
fi

if [ ${#missing[@]} -gt 0 ]; then
  log "missing: ${missing[*]} -- run the sudo steps in docs/hydra_hil.md first"
  $CHECK && exit 1
  case " ${missing[*]} " in *" colcon "*|*"submodules"*|*"hydra pins"*) exit 1 ;; esac
fi
$CHECK && { log "prerequisites present"; exit 0; }

mkdir -p "$WS"
cd "$WS"
[ -f colcon_defaults.yaml ] || cat > colcon_defaults.yaml <<'YAML'
build:
  symlink-install: true
  cmake-args: [-DCMAKE_BUILD_TYPE=Release]
YAML
log "sources: $SRC (submodules, pinned by $REPOS)"
log "rosdep check (installing needs sudo: docs/hydra_hil.md)"
if ! rosdep check --from-paths "$SRC" --ignore-src -r; then
  log "missing system dependencies: run (sudo)  rosdep install --from-paths $SRC --ignore-src -r -y"
  log "then run this script again (the sources are in place; nothing was built)"
  exit 1
fi

log "colcon build ($JOBS jobs) in $WS"
MAKEFLAGS="-j$JOBS" colcon build --base-paths "$SRC" \
  --continue-on-error --parallel-workers "$(( JOBS > 4 ? 4 : JOBS ))" \
  --packages-up-to hydra_ros hydra_visualizer semantic_inference_ros spark_dsg
log "built: $(ls "$WS/install" | tr '\n' ' ')"

# spark_dsg Python bindings from the SAME pinned source (colcon builds them OFF by default):
# the viewer, the logger's scene graph saver and hydra_cost read graphs with them.
# --target: no system site-packages are touched (PEP 668); sim_isaac.sh / hydra_env.sh put it
# on PYTHONPATH. Measured ~90 s on linus.
log "spark_dsg python bindings -> $WS/install/spark_dsg_python"
python3 -m pip install --quiet --no-deps --upgrade --target "$WS/install/spark_dsg_python" "$SRC/spark_dsg"

if $MODELS; then
  mkdir -p "$HOME/.semantic_inference"
  f="$HOME/.semantic_inference/ade20k-mobilenetv2dilated-c1_deepsup.onnx"
  if [ -s "$f" ]; then log "model present: $f"; else
    log "downloading MobileNetV2 ADE20K ONNX -> $f"
    wget -q -O "$f" "$MOBILENET_URL"
  fi
  log "the TensorRT engine is built from it on the first mobilenet run (about a minute)"
fi
log "done. Use: source $WS/install/setup.bash && source $REPO/install/setup.bash"
