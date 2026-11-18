#!/usr/bin/env bash
# Drive f1tenth_ctl through the commands the Telegram bot and the gateway use,
# recording every JSON answer and exit code.
#
# Runs on: linus (over ssh, the bot's path) or the Orin host. Run it once with
# the car native and once in container mode, with the SAME prefix (the host
# wrapper ~/.local/bin/f1tenth_ctl_car picks the side), then
# `ctl_contract_compare.py NATIVE_DIR CONTAINER_DIR`. Plan P8: "ctl via the
# wrapper from linus over ssh (start, stop, status, estop, killall, send,
# missions; JSON + exit codes = native; a killed node reports crashed)". No
# actuation: ESC off / VESC unplugged (estop only publishes zero drive).
#
#   ctl_contract.sh OUT_DIR [--estop-reset] [--kill 'CMD {pid}'] -- PREFIX ...
#
#   PREFIX     how to call the ctl, e.g.
#              ssh fabiocar@orin ~/.local/bin/f1tenth_ctl_car
#   --kill     after the stack is up, run this command with {pid} replaced by
#              the stack's pid from `status` (native: `ssh fabiocar@orin kill
#              -KILL {pid}`; container: `ssh fabiocar@orin docker exec
#              f1tenth_car kill -KILL {pid}`), then `status` must say crashed
#   --estop-reset  also clear the latch (restarts behavior: only with the
#              wheels up and no drive possible)
#
# Steps (OUT_DIR/NN_name.{json,rc,err}, steps.tsv): status, check, discovery
# status, missions -n 3, send planner (planner down: a refusal), killall
# --dry-run, start stack, status until the stack runs (180 s), estop, status,
# [estop-reset --force], [kill + status], stop stack, status, logs stack -n 5.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
mkdir -p "$1" || exit 2
OUT=$(cd "$1" && pwd)
[ -z "$(ls -A "$OUT")" ] || { echo "refusing: $OUT is not empty" >&2; exit 2; }
shift
RESET=0 KILL=""
while [ $# -gt 0 ] && [ "$1" != -- ]; do
  case "$1" in
    --estop-reset) RESET=1; shift ;;
    --kill) KILL=$2; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
[ "${1:-}" = -- ] && shift
[ $# -ge 1 ] || { echo "no ctl prefix after --" >&2; exit 2; }
PREFIX=("$@")
N=0

step() {  # NAME ARGS... -> runs PREFIX ARGS, records, echoes the rc
  local name=$1 t0 rc
  shift
  N=$((N + 1))
  local base
  base=$(printf '%s/%02d_%s' "$OUT" "$N" "$name")
  t0=$(date +%s.%N)
  "${PREFIX[@]}" "$@" > "$base.json" 2> "$base.err" < /dev/null
  rc=$?
  echo "$rc" > "$base.rc"
  printf '%02d_%s\t%s\t%s\t%s\n' "$N" "$name" "$rc" \
    "$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }')" "$*" >> "$OUT/steps.tsv"
  return 0
}
json_get() {  # FILE DOTTED.KEY -> value or empty
  python3 -c 'import json, sys
d = json.load(open(sys.argv[1]))
for k in sys.argv[2].split("."):
    d = d.get(k) if isinstance(d, dict) else None
print("" if d is None else d)' "$1" "$2" 2>/dev/null
}
last() { find "$OUT" -maxdepth 1 -name '*.json' | sort | tail -1; }

step status status
step check check
step discovery_status discovery status
step missions missions -n 3
step send_planner_down send planner "go ahead 1 meter"
step killall_dry killall --dry-run
step start_stack start stack
for _ in $(seq 1 18); do
  sleep 10
  step status_wait status
  [ "$(json_get "$(last)" processes.stack.state)" = running ] && break
done
step status_up status
PID=$(json_get "$(last)" processes.stack.pid)
step estop estop
step status_estop status
[ "$RESET" = 1 ] && step estop_reset estop-reset --force
if [ -n "$KILL" ]; then
  if [ -n "$PID" ]; then
    cmd=${KILL//\{pid\}/$PID}
    echo "kill: $cmd" >> "$OUT/kill.txt"
    bash -c "$cmd" >> "$OUT/kill.txt" 2>&1
    sleep 8
    step status_killed status
    echo "stack state after the kill: $(json_get "$(last)" processes.stack.state)" | tee -a "$OUT/kill.txt"
  else
    echo "no stack pid in status: kill test skipped" | tee -a "$OUT/kill.txt"
  fi
fi
step stop_stack stop stack
step status_stopped status
step logs logs stack -n 5
cat "$OUT/steps.tsv"
