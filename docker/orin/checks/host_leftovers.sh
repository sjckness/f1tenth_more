#!/usr/bin/env bash
# What a stopped stack left on the Orin host: the container, ROS processes,
# bound ports, Fast DDS shared memory.
#
# Runs on: the Orin host, after `docker_orin.sh down` (or `stack_mode.sh
# native` before the native side starts). Plan P8 "up/down x5 without
# leftovers" (updown_cycles.sh calls it after every down) and the switch
# checks. Independent of docker_orin.sh's own leftovers report, so the two can
# be compared.
#
#   host_leftovers.sh OUT_DIR [--shm-baseline FILE] [--label L]
#
# Checks (PASS / FAIL lines in OUT_DIR/results.txt):
#   container  no f1tenth_car container in `docker ps` (running);
#   processes  no ROS stack process on the host (proc_audit.py snapshot: the
#              container's processes are visible from the host too);
#   ports      UDP 11811 and TCP 8770 / 8083 / 8765 not bound;
#   shm        no Fast DDS file in /dev/shm that was not there in the
#              baseline (--shm-baseline: a `ls /dev/shm` taken before the
#              first up); without a baseline the fastrtps_* files are listed
#              (WARN, not FAIL: a previous native run may have left them).
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
shift
BASELINE="" LABEL=leftovers
while [ $# -gt 0 ]; do
  case "$1" in
    --shm-baseline) BASELINE=$2; shift 2 ;;
    --label) LABEL=$2; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done

if [ "$(container_state)" = running ]; then
  record "$OUT" "$LABEL container" FAIL "$CONTAINER still running"
else
  record "$OUT" "$LABEL container" PASS "$CONTAINER not running ($(container_state || true))"
fi

procs=$(python3 "$REPO/scripts/jazzy_parity/proc_audit.py" snapshot)
printf '%s\n' "$procs" > "$OUT/${LABEL}_procs.txt"
if [ -n "$procs" ]; then
  record "$OUT" "$LABEL processes" FAIL "$(printf '%s\n' "$procs" | grep -c .) ROS stack processes left (${LABEL}_procs.txt)"
else
  record "$OUT" "$LABEL processes" PASS "no ROS stack process"
fi

bound=""
for p in $STACK_UDP_PORTS; do udp_port_bound "$p" && bound="$bound udp/$p"; done
for p in $STACK_TCP_PORTS; do
  ss -H -tln "sport = :$p" 2>/dev/null | grep -q . && bound="$bound tcp/$p"
done
if [ -n "$bound" ]; then
  record "$OUT" "$LABEL ports" FAIL "still bound:$bound"
else
  record "$OUT" "$LABEL ports" PASS "udp $STACK_UDP_PORTS, tcp $STACK_TCP_PORTS free"
fi

find "$SHM_DIR" -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | sort > "$OUT/${LABEL}_shm.txt"
dds=$(grep -E '^(sem\.)?fastrtps|^fast_datasharing' "$OUT/${LABEL}_shm.txt" || true)
if [ -n "$BASELINE" ]; then
  new=$(comm -13 <(sort "$BASELINE") <(printf '%s\n' "$dds" | sed '/^$/d' | sort))
  if [ -n "$new" ]; then
    record "$OUT" "$LABEL shm" FAIL "$(printf '%s\n' "$new" | grep -c .) Fast DDS files not in the baseline: $(printf '%s\n' "$new" | tr '\n' ' ' | cut -c1-200)"
  else
    record "$OUT" "$LABEL shm" PASS "no Fast DDS file beyond the baseline"
  fi
else
  record "$OUT" "$LABEL shm" WARN "no baseline; $(printf '%s\n' "$dds" | grep -c .) Fast DDS files in /dev/shm"
fi
grep -q "^FAIL $LABEL " "$OUT/results.txt" && exit 1
exit 0
