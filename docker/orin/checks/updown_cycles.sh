#!/usr/bin/env bash
# Container up/down cycles without leftovers, and the healthcheck.
#
# Runs on: the Orin host, the car in container mode (stack_mode.sh container
# done once before; no actuation: ESC off / VESC unplugged). Plan P8:
# "up/down x5 without leftovers" and "healthcheck".
#
#   updown_cycles.sh OUT_DIR [N] [-- UP_ARGS ...]
#
# Per cycle: `docker_orin.sh up UP_ARGS` (it waits for healthy itself; this
# script checks again and times it), the container's health state, then
# `docker_orin.sh down` (graceful) and host_leftovers.sh against the /dev/shm
# baseline taken before the first up. Pass: N ups reach `healthy`, N downs
# exit 0 and leave nothing (container, ROS processes, ports, Fast DDS SHM).
# Default N = 5. Every command's output in OUT_DIR/cycle_NN/.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
need_empty_dir "$1"
OUT=$(parity_abs "$1")
shift
N=5
if [ $# -gt 0 ] && [ "$1" != -- ]; then N=$1; shift; fi
[ "${1:-}" = -- ] && shift
UP_ARGS=("$@")
[ -x "$DOCKER_ORIN" ] || { echo "no $DOCKER_ORIN" >&2; exit 2; }
if [ "$(container_state)" = running ]; then
  echo "refusing: $CONTAINER already runs; start from down" >&2; exit 2
fi
host_context "$OUT" start
find "$SHM_DIR" -mindepth 1 -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' -o -name 'fast_datasharing*' \) \
  -printf '%f\n' 2>/dev/null | sort > "$OUT/shm_baseline.txt"
for i in $(seq 1 "$N"); do
  C="$OUT/cycle_$(printf %02d "$i")"
  mkdir -p "$C"
  t0=$(date +%s.%N)
  "$DOCKER_ORIN" up "${UP_ARGS[@]}" > "$C/up.log" 2>&1
  rc=$?
  t1=$(date +%s.%N)
  health=$(container_health)
  dt=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')
  if [ $rc -eq 0 ] && [ "$health" = healthy ]; then
    record "$OUT" "cycle $i up" PASS "healthy after ${dt} s"
  else
    record "$OUT" "cycle $i up" FAIL "exit $rc, health '$health' after ${dt} s (cycle_$(printf %02d "$i")/up.log)"
  fi
  "$DOCKER" exec "$CONTAINER" "$F1T_HEALTHCHECK" > "$C/healthcheck.log" 2>&1
  hc=$?
  record "$OUT" "cycle $i healthcheck" "$([ $hc -eq 0 ] && echo PASS || echo FAIL)" \
    "healthcheck.sh in the container exit $hc: $(tail -1 "$C/healthcheck.log")"
  sleep "${HOLD_UP_S:-10}"
  t0=$(date +%s.%N)
  "$DOCKER_ORIN" down > "$C/down.log" 2>&1
  rc=$?
  dt=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')
  record "$OUT" "cycle $i down" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "exit $rc after ${dt} s"
  sleep 2
  bash "$HERE/host_leftovers.sh" "$C" --shm-baseline "$OUT/shm_baseline.txt" --label "cycle_$i" > /dev/null
  grep "^[A-Z]* cycle_$i " "$C/results.txt" >> "$OUT/results.txt"
done
overall "$OUT"
