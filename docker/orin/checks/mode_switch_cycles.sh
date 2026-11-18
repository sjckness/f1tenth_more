#!/usr/bin/env bash
# native <-> container switches, timed, with the guard refusing the wrong side.
#
# Runs on: the Orin host (docker/orin/host installed). Plan P8: "native <->
# container x3 with the guard refusing the wrong direction" and P6.5
# "stack_mode.sh ... <= 60 s". Leaves the car in the mode it found it in
# (native unless --end container).
#
#   mode_switch_cycles.sh OUT_DIR [N] [--end native|container] [-- MODE_ARGS ...]
#
# Per cycle: `stack_mode.sh container MODE_ARGS` (exit 0, <= 60 s), then
# `orin_stack_guard.sh --for humble` must REFUSE (non-zero) while f1tenth_car
# runs; `stack_mode.sh native` (exit 0, <= 60 s), f1tenth_car gone, then
# `orin_stack_guard.sh --for jazzy` recorded (it refuses while a native ROS
# process -- the native gateway's discovery server -- runs). Default N = 3.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
need_empty_dir "$1"
OUT=$(parity_abs "$1")
shift
N=3 END=native
if [ $# -gt 0 ] && [ "$1" != -- ] && [ "${1#-}" = "$1" ]; then N=$1; shift; fi
while [ $# -gt 0 ] && [ "$1" != -- ]; do
  case "$1" in --end) END=$2; shift 2 ;; *) echo "unknown argument $1" >&2; exit 2 ;; esac
done
[ "${1:-}" = -- ] && shift
MODE_ARGS=("$@")
for f in "$STACK_MODE" "$ORIN_STACK_GUARD"; do [ -x "$f" ] || { echo "no $f" >&2; exit 2; }; done
host_context "$OUT" start

switch() {  # LABEL MODE ARGS... -> records exit + duration
  local label=$1 t0 rc dt
  shift
  t0=$(date +%s.%N)
  "$STACK_MODE" "$@" > "$OUT/$label.log" 2>&1
  rc=$?
  dt=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')
  if [ $rc -eq 0 ] && awk -v d="$dt" 'BEGIN { exit !(d <= 60) }'; then
    record "$OUT" "$label" PASS "exit 0 in ${dt} s"
  else
    record "$OUT" "$label" FAIL "exit $rc in ${dt} s (<= 60 s wanted; $label.log)"
  fi
}
for i in $(seq 1 "$N"); do
  switch "cycle${i}_to_container" container "${MODE_ARGS[@]}"
  "$ORIN_STACK_GUARD" --for humble > "$OUT/cycle${i}_guard_humble.log" 2>&1
  rc=$?
  record "$OUT" "cycle$i guard --for humble while the container runs" \
    "$([ $rc -ne 0 ] && echo PASS || echo FAIL)" "exit $rc (must refuse)"
  "$STACK_MODE" status > "$OUT/cycle${i}_status_container.log" 2>&1
  switch "cycle${i}_to_native" native
  st=$(container_state)
  record "$OUT" "cycle$i container gone after native" "$([ "$st" != running ] && echo PASS || echo FAIL)" \
    "f1tenth_car state '${st:-none}'"
  "$ORIN_STACK_GUARD" --for jazzy > "$OUT/cycle${i}_guard_jazzy.log" 2>&1
  record "$OUT" "cycle$i guard --for jazzy in native mode" INFO "exit $? (refuses while native ROS runs)"
  "$STACK_MODE" status > "$OUT/cycle${i}_status_native.log" 2>&1
done
[ "$END" = container ] && switch "final_to_container" container "${MODE_ARGS[@]}"
overall "$OUT"
