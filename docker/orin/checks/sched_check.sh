#!/usr/bin/env bash
# Scheduling inside the container: negative nice, real-time priority, CPU
# affinity, and the stack's CPU pinning.
#
# Runs on: the Orin host, with f1tenth_car up. Plan P8 "`os.nice(-5)` / `chrt`
# effective, CPU pinning" (compose: ulimits nice 40 / rtprio 99 / memlock -1,
# backlog F1).
#
#   sched_check.sh OUT_DIR [--pinning]
#
# In the container, as its user: the limits (/proc/self/limits), os.nice(-5)
# succeeds, `chrt -f 10 true` succeeds, `taskset -c 10,11 true` succeeds.
# --pinning (stack UP): scripts/check_cpu_pinning.py, run on the HOST (it
# scans /proc, where the container's processes are visible): every pinned
# node's every thread on its cores.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
PINNING=0
[ "${2:-}" = --pinning ] && PINNING=1
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
"$DOCKER" exec "$CONTAINER" cat /proc/self/limits > "$OUT/limits.txt" 2>&1
grep -E 'nice|realtime priority|locked memory' "$OUT/limits.txt" | sed 's/^/  /'

res=$("$DOCKER" exec "$CONTAINER" python3 -c 'import os
try:
    print("nice", os.nice(-5))
except OSError as exc:
    print("refused", exc)' 2>&1 | tail -1)
record "$OUT" "os.nice(-5)" "$(case "$res" in "nice -5") echo PASS ;; *) echo FAIL ;; esac)" "$res"
"$DOCKER" exec "$CONTAINER" chrt -f 10 true > "$OUT/chrt.log" 2>&1
rc=$?
record "$OUT" "chrt -f 10" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "exit $rc $(head -1 "$OUT/chrt.log")"
"$DOCKER" exec "$CONTAINER" taskset -c 10,11 true > "$OUT/taskset.log" 2>&1
rc=$?
record "$OUT" "taskset -c 10,11" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "exit $rc $(head -1 "$OUT/taskset.log")"
if [ "$PINNING" = 1 ]; then
  python3 "$REPO/scripts/check_cpu_pinning.py" > "$OUT/cpu_pinning.log" 2>&1
  rc=$?
  record "$OUT" "cpu pinning" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" \
    "check_cpu_pinning.py exit $rc ($(tail -1 "$OUT/cpu_pinning.log"))"
fi
overall "$OUT"
