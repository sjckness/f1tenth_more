#!/usr/bin/env bash
# colcon test of a whole workspace on this side, reduced to failing test IDs.
#
# Runs on: inside the harness container (P9.1: the image's sources, built and
# tested there), and on linus for the reference at the same commit (point
# --src at a checkout of that commit). The image has no build/ (only
# install/), and its src is root-owned, so the sources are COPIED into
# WORK_DIR/src and built there exactly as linus builds them (--symlink-install,
# isolated install); every product stays under WORK_DIR.
#
#   colcon_test_ids.sh WORK_DIR [--src DIR] [--skip "PKG ..."] [--skip-cargo]
#                      [--jobs N] [--domain D] [--copy-results DIR]
#
#   --src     sources to copy (default /opt/f1tenth_ws/src, the image's)
#   --skip    packages skipped for build and test (default: f1tenth_sim
#             f1tenth_more husarion_gz_worlds, as the image build)
#   --skip-cargo  also skip every ament_cargo (Rust) package. Without a Rust
#             toolchain (no `cargo` on PATH) the script refuses unless this is
#             given; then pass it on BOTH sides, so both test the same set
#   --jobs    parallel workers and MAKEFLAGS (default 3)
#   --domain  ROS_DOMAIN_ID for the tests (default 96; localhost discovery, no
#             Discovery Server: the tests never join a running graph)
#   --copy-results DIR  at the end, copy the small results (condition.txt,
#             context.json, summary.md, failing_ids.txt, test_result_all.txt,
#             the tail of build.log) into DIR (new or empty)
#
# Writes WORK_DIR/{build.log, test.log, test_result_all.txt, summary.md,
# failing_ids.txt}. Then, against the reference IDs:
#   parity_gates.py tests REFERENCE_IDS failing_ids.txt [--allow FILE]
# A long run on the Orin (a full build; budget an hour): start it from the
# host with `docker_orin.sh exec` in the background and read the logs. In
# the harness container put WORK_DIR in the container's own /tmp (the docker
# data root is on the NVMe; the build is large) and --copy-results into the
# D3 results root under output/ (the only host directory it can write).
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
need_empty_dir "$1"
WORK=$(parity_abs "$1")
shift
SRC=/opt/f1tenth_ws/src
SKIP="f1tenth_sim f1tenth_more husarion_gz_worlds"
JOBS=3
DOMAIN=96
SKIP_CARGO=0
COPY_TO=""
while [ $# -gt 0 ]; do
  case "$1" in
    --src) SRC=$2; shift 2 ;;
    --skip) SKIP=$2; shift 2 ;;
    --skip-cargo) SKIP_CARGO=1; shift ;;
    --jobs) JOBS=$2; shift 2 ;;
    --domain) DOMAIN=$2; shift 2 ;;
    --copy-results) COPY_TO=$2; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
[ -d "$SRC" ] || { echo "no sources at $SRC" >&2; exit 2; }
command -v colcon > /dev/null || { echo "colcon not on PATH (source the ROS setup)" >&2; exit 2; }
if [ "$SKIP_CARGO" = 0 ] && ! command -v cargo > /dev/null; then
  echo "no Rust toolchain (cargo) here: pass --skip-cargo, and pass it on the other side too" >&2
  exit 2
fi
if [ -n "$COPY_TO" ]; then
  need_empty_dir "$COPY_TO"
  COPY_TO=$(parity_abs "$COPY_TO")
fi
parity_context "$WORK"
cp -a "$SRC" "$WORK/src" || exit 2
cd "$WORK" || exit 2
if [ "$SKIP_CARGO" = 1 ]; then
  # Identified from package.xml: "ros.ament_cargo" (or "ament_cargo" with
  # colcon-ros-cargo installed), as W2's image build does; a plain crate
  # ("cargo", colcon-cargo) needs the toolchain too.
  cargo_pkgs=$(colcon --log-base /dev/null list -t --base-paths "$WORK/src" 2>/dev/null \
    | awk '$3 ~ /^\((ros\.)?(ament_)?cargo\)$/ {printf "%s ", $1}')
  SKIP="$SKIP $cargo_pkgs"
fi
echo "src=$SRC skip=$SKIP skip_cargo=$SKIP_CARGO jobs=$JOBS domain=$DOMAIN" > "$WORK/condition.txt"
export MAKEFLAGS=-j$JOBS
# shellcheck disable=SC2086  # SKIP is a list of package names
colcon build --symlink-install --parallel-workers "$JOBS" --base-paths "$WORK/src" \
  --packages-skip $SKIP --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct- console_cohesion+ > "$WORK/build.log" 2>&1
BUILD_RC=$?
echo "build exit $BUILD_RC" >> "$WORK/condition.txt"
isolated_dds "$DOMAIN"
set +u
# shellcheck source=/dev/null
source "$WORK/install/setup.bash"
set -u
# shellcheck disable=SC2086
colcon test --parallel-workers "$JOBS" --base-paths "$WORK/src" --packages-skip $SKIP \
  --test-result-base "$WORK/results" --event-handlers console_direct- console_cohesion+ \
  > "$WORK/test.log" 2>&1
echo "test exit $?" >> "$WORK/condition.txt"
colcon test-result --all --test-result-base "$WORK/results" > "$WORK/test_result_all.txt" 2>&1
python3 "$HERE/test_results_summary.py" "$WORK/results" "$WORK/test_result_all.txt" \
  --ids "$WORK/failing_ids.txt" > "$WORK/summary.md"
tail -3 "$WORK/summary.md"
echo "failing/erroring/skipped IDs: $(grep -c . "$WORK/failing_ids.txt") -> $WORK/failing_ids.txt"
[ "$BUILD_RC" -eq 0 ] || echo "WARNING: the build failed (see build.log); tests of unbuilt packages are missing" >&2
if [ -n "$COPY_TO" ]; then
  cp -p "$WORK"/{condition.txt,summary.md,failing_ids.txt,test_result_all.txt} "$COPY_TO"/
  cp -p "$WORK/context.json" "$COPY_TO"/ 2>/dev/null
  tail -200 "$WORK/build.log" > "$COPY_TO/build_tail.log"
  echo "results copied to $COPY_TO"
fi
