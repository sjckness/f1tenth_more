#!/bin/bash
# R6 colcon test gate: f1tenth_perception, f1tenth_wall_tracker_core and
# f1tenth_bringup, once with the stack default (py) and once with the test hook
# F1TENTH_WALL_TRACKER_IMPL=rust (every MultiWallTracker() and every
# lidar_front_wall_node in the tests then runs the Rust library), then
# compare_test_results.py: the same outcome for every test ID, and
# tools/r3_failing_ids.py: no failing ID outside the jazzy_main baseline.
#
#   tools/rust_parity/wall_tracker/run_gate_tests.sh <out dir>
#
# Isolated DDS (localhost, domain 95). Never two colcon runs at once in a
# workspace: run nothing else colcon while this runs.
set -euo pipefail
OUT=${1:?usage: run_gate_tests.sh <out dir>}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
ENV_FILE=${ENV_FILE:-$REPO/output/rust_main/env.sh}
set +u
# shellcheck source=/dev/null
source "$ENV_FILE"
set -u
cd "$REPO"
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
unset F1TENTH_WALL_TRACKER_IMPL F1TENTH_WALL_TRACKER_CORE_LIB F1TENTH_WALL_TRACKER_VERIFY_SYNC
export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
PKGS=(f1tenth_perception f1tenth_wall_tracker_core f1tenth_bringup)
for mode in py rust; do
  if [ "$mode" = rust ]; then export F1TENTH_WALL_TRACKER_IMPL=rust; fi
  echo "== colcon test ($mode)"
  colcon test --packages-select "${PKGS[@]}" --packages-ignore ${SKIP:-} \
    --test-result-base "$OUT/$mode" > "$OUT/$mode.log" 2>&1 || true
  colcon test-result --all --test-result-base "$OUT/$mode" > "$OUT/$mode.result.txt" 2>&1 || true
  tail -1 "$OUT/$mode.result.txt"
done
unset F1TENTH_WALL_TRACKER_IMPL
python3 "$HERE/compare_test_results.py" "$OUT/py" "$OUT/rust" --json "$OUT/compare.json" \
  | tee "$OUT/compare.txt"
for mode in py rust; do
  echo "== failing IDs ($mode) against output/jazzy_main/test_failing_ids.txt"
  python3 tools/r3_failing_ids.py "$OUT/$mode" output/jazzy_main/test_failing_ids.txt \
    --out "$OUT/$mode.failing_ids.txt" | tail -3 || true
done
