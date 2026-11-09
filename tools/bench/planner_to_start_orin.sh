#!/bin/bash
# Run tools/bench/planner_to_start.py on the Orin, inside the car container,
# under the stack's own f1tenth_ctl environment (ctl_container.env: Discovery
# Server, Fast DDS profile, ROS_DOMAIN_ID), against the LIVE stack.
#
#   tools/bench/planner_to_start_orin.sh [--llm-overlay] -- <planner_to_start.py args>
#   tools/bench/planner_to_start_orin.sh -- --car-is-safe --suite --runs 10 --cold-first
#
# --llm-overlay  also copy this checkout's src/f1tenth_intelligence/llm into the
#                container and put it first on the PLANNER's PYTHONPATH, to time
#                planner code the image was not built with. Without it the
#                planner is the image's own (/opt/f1tenth_ws/SOURCES says which).
#
# The results JSON (--out NAME) is written in the container's /tmp/f1t_bench and
# copied back to output/bench/ here. Ctrl-C / SIGTERM here is forwarded to the
# tool inside the container, whose own handler stops the planner and aborts the
# mission (docker exec without a tty would not forward it).
#
# STARTS REAL MISSIONS: the tool refuses without --car-is-safe.
set -euo pipefail

CONTAINER="${F1T_BENCH_CONTAINER:-f1tenth_car}"
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
DEST=/tmp/f1t_bench
OVERLAY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --llm-overlay) OVERLAY=1; shift ;;
    --) shift; break ;;
    *) break ;;
  esac
done

HEAD="$(git -C "$REPO" rev-parse --short HEAD)"
if ! git -C "$REPO" diff --quiet -- tools/bench src/f1tenth_intelligence/llm; then
  HEAD="$HEAD+dirty"
fi

docker exec "$CONTAINER" mkdir -p "$DEST"
docker cp "$REPO/tools/bench/planner_to_start.py" "$CONTAINER:$DEST/planner_to_start.py" >/dev/null

EXTRA=()
if [ "$OVERLAY" = 1 ]; then
  docker exec "$CONTAINER" rm -rf "$DEST/llm_src"
  docker cp "$REPO/src/f1tenth_intelligence/llm" "$CONTAINER:$DEST/llm_src" >/dev/null
  EXTRA=(--planner-pythonpath "$DEST/llm_src")
fi

# --out: keep only the file name; it is written in $DEST and copied back.
ARGS=()
OUT=""
while [ $# -gt 0 ]; do
  if [ "$1" = "--out" ]; then OUT="$(basename "$2")"; ARGS+=(--out "$DEST/$OUT"); shift 2
  else ARGS+=("$1"); shift; fi
done

forward() {
  docker exec "$CONTAINER" bash -c "kill -INT \$(cat $DEST/tool.pid) 2>/dev/null" || true
}
trap forward INT TERM

set +e
docker exec -i -e F1T_BENCH_GIT_HEAD="$HEAD" -e F1T_BENCH_PID_FILE="$DEST/tool.pid" \
  "$CONTAINER" bash -c '
    set -a; . "${F1T_CTL_ENV:-/opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env}" || exit 97; set +a
    exec python3 '"$DEST"'/planner_to_start.py "$@"' bench "${ARGS[@]}" "${EXTRA[@]}" &
PID=$!
wait $PID
RC=$?
# wait returns early when a trapped signal arrives: wait for the tool to finish its abort.
while kill -0 $PID 2>/dev/null; do wait $PID; RC=$?; done
set -e

if [ -n "$OUT" ]; then
  mkdir -p "$REPO/output/bench"
  docker cp "$CONTAINER:$DEST/$OUT" "$REPO/output/bench/$OUT" >/dev/null 2>&1 \
    && echo "results: $REPO/output/bench/$OUT"
fi
exit $RC
