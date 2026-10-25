#!/usr/bin/env bash
# The TensorRT engine: built by `docker_orin.sh engine`, loads in TensorRT,
# used by yolo_detector_node with yolo_backend:=engine, and a missing engine
# falls back to the .pt.
#
# Runs on: the Orin host, f1tenth_car up with the stack DOWN (the detection
# launch below runs alone, on an isolated domain; no camera needed: the model
# loads at startup). Plan P8: "`docker_orin.sh engine` builds the TRT engine,
# `yolo_backend:=engine` loads it, missing engine falls back to `.pt`".
#
#   yolo_engine_check.sh OUT_DIR [--skip-build] [--engine PATH]
#
#   build     `docker_orin.sh engine` exits 0 (--skip-build: the engine exists)
#   trt       g2_trt.py --engine PATH in the container: toy engines run, the
#             engine deserializes (default PATH: the installed yolo_engine_path,
#             /var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine)
#   engine    detection.launch.py yolo_backend:=engine logs
#             'yolo_backend=engine: running "PATH"'
#   fallback  ... yolo_engine_path:=<missing file> logs
#             'yolo_backend=engine: engine "..." not used' (the node keeps the .pt)
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
shift
BUILD=1
ENGINE=/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine
while [ $# -gt 0 ]; do
  case "$1" in --skip-build) BUILD=0; shift ;; --engine) ENGINE=$2; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;; esac
done
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
host_context "$OUT" host
if [ "$BUILD" = 1 ]; then
  "$DOCKER_ORIN" engine > "$OUT/engine_build.log" 2>&1
  rc=$?
  record "$OUT" "engine build" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "docker_orin.sh engine exit $rc"
fi
"$DOCKER" exec -i "$CONTAINER" python3 - --engine "$ENGINE" < "$HERE/g2_trt.py" > "$OUT/g2_trt.json" 2>&1
rc=$?
record "$OUT" "engine loads in TensorRT" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "g2_trt.py exit $rc (g2_trt.json)"

run_detection() {  # LOG ARGS...: detection.launch.py alone for 45 s, isolated
  local log=$1
  shift
  cexec env -u ROS_DISCOVERY_SERVER -u ROS_SUPER_CLIENT ROS_DOMAIN_ID=98 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout -s INT -k 15 45 \
    ros2 launch f1tenth_perception detection.launch.py "$@" > "$log" 2>&1
}
run_detection "$OUT/detection_engine.log" yolo_backend:=engine "yolo_engine_path:=$ENGINE"
if grep -q 'yolo_backend=engine: running' "$OUT/detection_engine.log"; then
  record "$OUT" "yolo_backend:=engine" PASS "$(grep -o 'yolo_backend=engine: running[^,]*' "$OUT/detection_engine.log" | head -1)"
else
  record "$OUT" "yolo_backend:=engine" FAIL "no 'yolo_backend=engine: running' line (detection_engine.log)"
fi
run_detection "$OUT/detection_fallback.log" yolo_backend:=engine \
  "yolo_engine_path:=/var/cache/f1tenth/trt/does-not-exist.engine"
if grep -q 'yolo_backend=engine: engine .* not used' "$OUT/detection_fallback.log"; then
  record "$OUT" "missing engine falls back to .pt" PASS "$(grep -o 'not used ([^)]*)' "$OUT/detection_fallback.log" | head -1)"
else
  record "$OUT" "missing engine falls back to .pt" FAIL "no 'engine ... not used' line (detection_fallback.log)"
fi
overall "$OUT"
