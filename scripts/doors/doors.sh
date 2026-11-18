#!/bin/bash
# Doors campaign on the car (Orin, Jazzy container f1tenth_car). One script,
# one subcommand per step:
#
#   scripts/doors/doors.sh up          container mode on the pinned image (no-op if already up)
#   scripts/doors/doors.sh status      image, pin, which side, stack/READY, last tests
#   scripts/doors/doors.sh start       start the stack through ctl (= f1tenth_ctl_car start stack:
#                                      yesterday's command + enable_test_campaign_logger:=true
#                                      from ~/.config/f1tenth_car/stack_args), then wait for
#                                      the logger's "auto_mode: True" line
#   scripts/doors/doors.sh start --fg  the same launch command in the foreground, Ctrl-C stops it
#                                      (ctl / the PWA then do not know the stack runs)
#   scripts/doors/doors.sh stop        stop the stack started with `start`
#   scripts/doors/doors.sh watch       follow the test_campaign lines of the stack log (Ctrl-C ends)
#   scripts/doors/doors.sh shell       a shell in the container: ROS sourced, f1tenth_ctl on PATH
#   scripts/doors/doors.sh results [ARGS]  analysis CSV over the campaign folder
#                                      (first_test_campaign/analysis_v2/doors_csv.py ARGS)
#   scripts/doors/doors.sh down        stop the container (scripts/docker_orin.sh down)
#
# Missions are loaded / started / stopped from Foxglove as usual. Nothing here
# moves the car; `start` starts the stack, `up` the container.
set -euo pipefail

HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
CTR=f1tenth_car
PIN=$HOME/.config/f1tenth/car_image
STACK_ARGS=$HOME/.config/f1tenth_car/stack_args
CAMPAIGN=/scratch/fabiocar/f1tenth_container/archive/doors_campaign
CTL_IN=/opt/f1tenth_ws/src/f1tenth_ctl/bin/f1tenth_ctl
CTL_CAR=$REPO/docker/orin/host/f1tenth_ctl_car
LOGGER_FLAG=enable_test_campaign_logger:=true

die() { echo "doors: $*" >&2; exit 1; }
pinned_tag() { awk '$1=="tag"{print $2}' "$PIN"; }
pinned_id() { awk '$1=="id"{print $2}' "$PIN"; }
ctr_state() { docker inspect -f '{{.State.Status}}' "$CTR" 2>/dev/null || echo absent; }
ctr_image_id() { docker inspect -f '{{.Image}}' "$CTR" 2>/dev/null || true; }
state_dir() {
  docker inspect -f '{{range .Mounts}}{{if eq .Destination "/home/fabiocar/.local/state/f1tenth_ctl"}}{{.Source}}{{end}}{{end}}' "$CTR"
}
stack_log() { echo "$(state_dir)/logs/stack.log"; }

# The container runs, on the pinned image. Everything but up/down/results needs it.
need_up() {
  [ -r "$PIN" ] || die "no pin in $PIN"
  [ "$(ctr_state)" = running ] || die "container $CTR is $(ctr_state): run 'scripts/doors/doors.sh up' first"
  [ "$(ctr_image_id)" = "$(pinned_id)" ] \
    || die "$CTR runs $(ctr_image_id), not the pinned $(pinned_tag) ($(pinned_id)): scripts/docker_orin.sh restart --allow-drift"
}

check_logger_flag() {
  if grep -qx "$LOGGER_FLAG" "$STACK_ARGS" 2>/dev/null; then
    echo "doors: stack_args: $LOGGER_FLAG"
  else
    echo "doors: WARNING: $STACK_ARGS does not hold '$LOGGER_FLAG': test_campaign_logger will NOT start" >&2
  fi
}

ctl_json() { docker exec "$CTR" /opt/f1tenth_ws/entrypoint.sh "$CTL_IN" --json "$@"; }

stack_state() {
  ctl_json status 2>/dev/null | python3 -c \
    'import json,sys; print(json.load(sys.stdin)["processes"]["stack"]["state"])' 2>/dev/null \
    || echo unknown
}

cmd_up() {
  [ -r "$PIN" ] || die "no pin in $PIN"
  if [ "$(ctr_state)" = running ] && [ "$(ctr_image_id)" = "$(pinned_id)" ]; then
    echo "doors: $CTR already up on $(pinned_tag)"
  elif [ "$(ctr_state)" = running ]; then
    die "$CTR runs $(ctr_image_id), not the pinned $(pinned_tag): restart it yourself (scripts/docker_orin.sh restart --allow-drift; it stops a running stack)"
  else
    echo "doors: switching the car to container mode on $(pinned_tag)"
    "$REPO/docker/orin/host/stack_mode.sh" container --allow-drift
  fi
  local which
  which=$("$CTL_CAR" --which)
  [ "$which" = container ] || die "f1tenth_ctl_car --which says '$which', not container"
  echo "doors: f1tenth_ctl_car --which: container; image $(docker inspect -f '{{.Config.Image}}' "$CTR")"
  check_logger_flag
}

cmd_status() {
  echo "pin:       $(pinned_tag) $(pinned_id | cut -c8-19)"
  echo "container: $CTR $(ctr_state) $(docker inspect -f '{{.Config.Image}} {{if .State.Health}}{{.State.Health.Status}}{{end}}' "$CTR" 2>/dev/null || true)"
  echo "which:     $("$CTL_CAR" --which 2>/dev/null || echo '?')"
  check_logger_flag
  if [ "$(ctr_state)" = running ]; then
    ctl_json status 2>/dev/null | python3 -c '
import json, sys
s = json.load(sys.stdin)
print("stack:    ", s["processes"]["stack"]["state"])
print("ready:    ", s.get("ready"), "failing:", ", ".join(s.get("failing") or []) or "-")
m = s.get("mission") or {}
print("mission:  ", m.get("state", "-"), m.get("id", "") if isinstance(m, dict) else m)
' || echo "stack:     (ctl status failed)"
  fi
  echo "last tests in $CAMPAIGN:"
  ls -dt "$CAMPAIGN"/*/P[0-9][0-9][0-9]-R[0-9][0-9][0-9]-* 2>/dev/null | head -5 | sed "s|$CAMPAIGN/|  |" || true
}

cmd_start() {
  need_up
  check_logger_flag
  if [ "${1:-}" = --fg ]; then
    echo "doors: foreground launch in $CTR (Ctrl-C stops it; ctl does not track it)"
    exec docker exec -it "$CTR" /opt/f1tenth_ws/entrypoint.sh bash -c \
      'source "$F1T_CTL_ENV" >/dev/null 2>&1; echo "+ $F1T_STACK_CMD"; exec $F1T_STACK_CMD'
  fi
  [ $# -eq 0 ] || die "start: unknown option '$1'"
  local st log off
  st=$(stack_state)
  [ "$st" != running ] || die "the stack already runs (stop it first: scripts/doors/doors.sh stop)"
  log=$(stack_log)
  off=$(stat -c %s "$log" 2>/dev/null || echo 0)
  "$CTL_CAR" start stack
  echo "doors: waiting up to 90 s for test_campaign_logger in $log"
  local i
  for i in $(seq 1 90); do
    if tail -c +"$((off + 1))" "$log" 2>/dev/null | grep -q '\[test_campaign\] auto_mode: True'; then
      tail -c +"$((off + 1))" "$log" | grep -E 'stack_args|\[supervisor_bringup\].*test_campaign|\[test_campaign\] (campaign folder|auto_mode)' | cut -c1-220
      echo "doors: logger up in auto mode. Load/start missions from Foxglove; 'scripts/doors/doors.sh watch' shows each test."
      return 0
    fi
    sleep 1
  done
  echo "doors: WARNING: no '[test_campaign] auto_mode: True' line after 90 s; last test_campaign lines:" >&2
  tail -c +"$((off + 1))" "$log" | grep -E 'test_campaign|stack_args' | tail -10 >&2 || true
  return 1
}

cmd_stop() {
  need_up
  "$CTL_CAR" stop stack
}

cmd_watch() {
  need_up
  local log
  log=$(stack_log)
  echo "doors: following [test_campaign] lines of $log (Ctrl-C ends)"
  tail -n 0 -F "$log" | grep --line-buffered -E '\[test_campaign\]|\[supervisor_bringup\].*test_campaign'
}

cmd_shell() {
  need_up
  echo "doors: shell in $CTR (ROS Jazzy sourced). f1tenth_ctl start|stop|status|logs stack; exit to leave."
  exec docker exec -it "$CTR" /opt/f1tenth_ws/entrypoint.sh bash -c \
    'export PATH=/opt/f1tenth_ws/src/f1tenth_ctl/bin:$PATH; exec bash -i'
}

cmd_results() {
  exec python3 "$REPO/first_test_campaign/analysis_v2/doors_csv.py" "$@"
}

cmd_down() {
  exec "$REPO/scripts/docker_orin.sh" down
}

sub=${1:-}
[ $# -eq 0 ] || shift
case "$sub" in
  up) cmd_up ;;
  status) cmd_status ;;
  start) cmd_start "$@" ;;
  stop) cmd_stop ;;
  watch) cmd_watch ;;
  shell) cmd_shell ;;
  results) cmd_results "$@" ;;
  down) cmd_down ;;
  ''|-h|--help|help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//' ;;
  *) die "unknown subcommand '$sub' (try --help)" ;;
esac
