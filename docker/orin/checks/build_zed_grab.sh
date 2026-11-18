#!/usr/bin/env bash
# Build zed_grab (zed_grab.cpp) against THIS side's ZED SDK.
#
# Runs on: the Orin host (the host SDK 4.2.5 in /usr/local/zed: the native
# side) and inside the container (the image's SDK 5.4.0: the container side).
# Needs g++ and the CUDA headers; the image's ws stage has them if the final
# image kept build-essential (if not, build it in the harness container from
# a stage that has, or run `docker_orin.sh exec` on a dev image).
#
#   build_zed_grab.sh OUT_BINARY
#
# The binary records the SDK version it was built against (`zed_grab
# --help` does not open the camera; `zed_grab --frames 1` prints the SDK).
set -euo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ZED_DIR=${ZED_DIR:-/usr/local/zed}
CUDA_DIR=${CUDA_DIR:-/usr/local/cuda}
[ -f "$ZED_DIR/include/sl/Camera.hpp" ] || { echo "no ZED SDK headers in $ZED_DIR" >&2; exit 2; }
grep -h -E '#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' "$ZED_DIR/include/sl/Camera.hpp" || true
g++ -O2 -std=c++17 "$HERE/zed_grab.cpp" -o "$1" \
  -I"$ZED_DIR/include" -I"$CUDA_DIR/include" \
  -L"$ZED_DIR/lib" -lsl_zed -L"$CUDA_DIR/lib64" -lcudart -Wl,-rpath,"$ZED_DIR/lib"
echo "built $1"
