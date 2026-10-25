#!/usr/bin/env bash
# A gateway crash leaves the stack running, and the gateway comes back.
#
# Runs on: the Orin host, f1tenth_car up in the car role with the stack
# started (f1tenth_ctl_car start stack). Plan P8: "gateway crash leaves the
# stack running" (the container main restarts the gateway, restart-always
# semantics, P4.2).
#
#   gateway_crash.sh OUT_DIR [--ctl CMD] [--wait S]
#
# SIGKILLs the gateway process (found with `docker top`, by its command line
# f1tenth_gateway), then: the stack's unit is still `running` in `status`
# (--ctl, default ~/.local/bin/f1tenth_ctl_car), and TCP 127.0.0.1:8770
# listens again within --wait (default 30) s, from a new process.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
shift
CTL=$HOME/.local/bin/f1tenth_ctl_car
WAIT=30
while [ $# -gt 0 ]; do
  case "$1" in --ctl) CTL=$2; shift 2 ;; --wait) WAIT=$2; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;; esac
done
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
stack_state() {
  $CTL status 2> /dev/null | python3 -c 'import json, sys
try:
    print(json.load(sys.stdin)["processes"]["stack"]["state"])
except Exception:
    print("unknown")'
}
gw_pid() {
  "$DOCKER" top "$CONTAINER" -eo pid,args 2>/dev/null \
    | awk '/f1tenth_gateway/ && !/awk/ { print $1; exit }'
}
before=$(stack_state)
pid=$(gw_pid)
echo "stack before: $before; gateway host pid: ${pid:-none}" | tee "$OUT/gateway_crash.txt"
[ "$before" = running ] || { record "$OUT" "gateway crash" FAIL "stack not running before the test ($before)"; overall "$OUT"; exit 1; }
[ -n "$pid" ] || { record "$OUT" "gateway crash" FAIL "no gateway process in $CONTAINER"; overall "$OUT"; exit 1; }
kill -KILL "$pid"
back=""
for _ in $(seq 1 "$WAIT"); do
  sleep 1
  new=$(gw_pid)
  if [ -n "$new" ] && [ "$new" != "$pid" ] && ss -H -tln 'sport = :8770' | grep -q .; then
    back=$new
    break
  fi
done
after=$(stack_state)
echo "gateway back: ${back:-no}; stack after: $after" | tee -a "$OUT/gateway_crash.txt"
record "$OUT" "gateway restarted" "$([ -n "$back" ] && echo PASS || echo FAIL)" \
  "${back:+new pid $back, }8770 listening: $(ss -H -tln 'sport = :8770' | grep -c .)"
record "$OUT" "stack survives gateway crash" "$([ "$after" = running ] && echo PASS || echo FAIL)" \
  "stack $before -> $after"
overall "$OUT"
