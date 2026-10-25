#!/bin/bash
# Switch the car between its two stacks (plan P6.5): the native Humble stack
# (~/dev_ws/f1tenth_more, f1tenth-gateway.service) and the Jazzy container
# (f1tenth_car, f1tenth-car-container.service). One side at a time, always.
#
#   stack_mode.sh status
#   stack_mode.sh container [--archive separate|shared] [--allow-drift] [--dry-run]
#   stack_mode.sh native [--dry-run]
#
# container: stops the native side completely (its f1tenth_ctl killall
#   --reset-discovery: stack, planner, logger, monitor, any other ROS process
#   of this user, the discovery server, the ros2 daemon; then the native
#   gateway, which frees TCP 8770), waits for the guard
#   (scripts/orin_stack_guard.sh --for jazzy) to clear, then starts
#   f1tenth-car-container.service (docker_orin.sh up; the unit's
#   Conflicts=f1tenth-gateway.service keeps the native gateway down while it
#   is active). A unit still "active (exited)" from a container that is gone
#   is stopped first (`start` would do nothing). Without the unit installed:
#   docker_orin.sh up directly. Either way it then requires f1tenth_car to
#   run with role car, or fails. Docker not answering fails it before
#   anything is stopped.
#   --archive and --allow-drift are passed through the unit's EnvironmentFile
#   (~/.config/f1tenth/car_container.env); --allow-drift holds for this start
#   only.
# native: stops the container (the unit, or docker_orin.sh down), waits for the
#   guard (--for humble) to clear, starts f1tenth-gateway.service. The native
#   stack itself starts on demand, as before (PWA, Telegram, f1tenth_ctl).
# Target: <= 60 s either way (printed). --dry-run: every read-only probe runs,
# every step that changes something is printed instead.
# Exit: 0 done (or already in that mode), 1 a step failed (the car is left as
# the failing step found it; `status` says what runs), 2 usage.
# Env: F1T_NATIVE_CTL, F1T_DOCKER_ORIN, F1T_GUARD (paths of the tools),
# F1T_GUARD_WAIT_S (20).
set -uo pipefail
SELF=$(readlink -f "${BASH_SOURCE[0]}")
REPO=$(cd "$(dirname "$SELF")/../../.." && pwd)
NATIVE_CTL=${F1T_NATIVE_CTL:-$HOME/dev_ws/f1tenth_more/src/f1tenth_ctl/bin/f1tenth_ctl}
DOCKER_ORIN=${F1T_DOCKER_ORIN:-$REPO/scripts/docker_orin.sh}
GUARD=${F1T_GUARD:-$REPO/scripts/orin_stack_guard.sh}
UNIT_ENV=$HOME/.config/f1tenth/car_container.env
CTR=f1tenth_car
CAR_UNIT=f1tenth-car-container.service
GW_UNIT=f1tenth-gateway.service
DRY=0
T0=$(date +%s)

usage() { sed -n '2,/^set -uo/p' "$SELF" | sed '$d; s/^# \{0,1\}//'; }
say() { echo "stack_mode: $*"; }
fail() { echo "stack_mode: FAILED: $*" >&2; exit 1; }
# A step that changes something: printed under --dry-run, run otherwise.
run() {
  if [ "$DRY" = 1 ]; then printf 'stack_mode: dry-run: would run:'; printf ' %q' "$@"; echo; return 0; fi
  say "+ $*"
  "$@"
}

# The container's state as docker_orin.sh reads it: `docker ps -a`'s exit code
# and an exact name match, never Docker's message texts (Docker 29 changed
# them). absent: no such container; unknown: Docker did not answer (daemon
# down, no permission), the reason on stderr.
ctr_state() {
  local out
  if ! out=$(docker ps -a --filter "name=$CTR" --format '{{.Names}} {{.State}}' 2>&1); then
    echo "stack_mode: cannot ask Docker for $CTR's state: $(printf '%s\n' "$out" | grep -v '^[[:space:]]*$' | tail -1)" >&2
    echo unknown
    return 0
  fi
  printf '%s\n' "$out" | awk -v n="$CTR" '$1 == n {s = $2} END {print (s == "" ? "absent" : s)}'
}
ctr_role() {
  docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "$CTR" 2>/dev/null \
    | sed -n 's/^F1T_CONTAINER_ROLE=//p' | head -1
}
unit_known() { systemctl --user cat "$1" > /dev/null 2>&1; }
unit_active() { systemctl --user is-active --quiet "$1" 2>/dev/null; }
port_owner() { ss -Hlnp "( sport = :$1 )" 2>/dev/null | grep -o 'users:(([^)]*' | head -1 | sed 's/users:((//'; }
port_used() { ss -Hln "( sport = :$1 )" 2>/dev/null | grep -q .; }

# The guard, retried while the stopped side's processes and sockets go away.
wait_guard() {
  local for=$1 deadline=$(( $(date +%s) + ${F1T_GUARD_WAIT_S:-20} ))
  if [ "$DRY" = 1 ]; then
    say "dry-run: the guard right now (before any stop):"
    "$GUARD" --for "$for" 2>&1 | sed 's/^/  /'
    return 0
  fi
  local err
  while :; do
    if err=$("$GUARD" --for "$for" --quiet 2>&1 > /dev/null); then
      say "guard --for $for: clear"
      return 0
    fi
    if [ "$(date +%s)" -ge "$deadline" ]; then
      echo "$err" >&2
      return 1
    fi
    sleep 1
  done
}

write_unit_env() {
  local archive=$1 allow=$2
  if [ "$DRY" = 1 ]; then say "dry-run: would write $UNIT_ENV: F1T_ARCHIVE_MODE=$archive F1T_ALLOW_DRIFT=$allow"; return 0; fi
  mkdir -p "$(dirname "$UNIT_ENV")"
  printf '# Written by stack_mode.sh %s; read by %s (docker_orin.sh up).\nF1T_ARCHIVE_MODE=%s\nF1T_ALLOW_DRIFT=%s\n' \
    "$(date -Iseconds)" "$CAR_UNIT" "$archive" "$allow" > "$UNIT_ENV.tmp" && mv -f "$UNIT_ENV.tmp" "$UNIT_ENV"
}

cmd_status() {
  local st role gw native_ros=no mode act en
  st=$(ctr_state); role=$(ctr_role)
  gw=$(systemctl --user is-active "$GW_UNIT" 2>/dev/null || true)
  # Native ROS processes: the guard's process scan without the container ones.
  if [ "$st" != running ] && ! "$GUARD" --for humble --quiet > /dev/null 2>&1; then native_ros=yes; fi
  if [ "$st" = running ] && [ "$gw" = active ]; then mode="CONFLICT (container AND native gateway)"
  elif [ "$st" = running ]; then mode="container (${role:-car})"
  elif [ "$gw" = active ] || [ "$native_ros" = yes ]; then mode=native
  else mode=none
  fi
  say "mode: $mode"
  echo "  container $CTR: $st${role:+, role $role}$( [ "$st" = running ] && docker inspect --type container -f ', health {{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}, image {{.Config.Image}}' "$CTR" 2>/dev/null)"
  # is-active / is-enabled print their answer AND fail for anything but
  # active / enabled: keep the printed word, once.
  act=$(systemctl --user is-active "$CAR_UNIT" 2>/dev/null) || act=${act:-unknown}
  en=$(systemctl --user is-enabled "$CAR_UNIT" 2>/dev/null) || en=${en:-not installed}
  [ "$en" != not-found ] || en="not installed"
  echo "  $CAR_UNIT: $act ($en)$( [ "$act" = active ] && [ "$st" != running ] && echo "; STALE: active but $CTR is $st (stack_mode container restarts it)")"
  echo "  $GW_UNIT: ${gw:-unknown}"
  echo "  native ROS processes / DDS: $native_ros"
  local owner
  owner=$(port_owner 8770)
  if port_used 8770; then echo "  TCP 8770: listening${owner:+ ($owner)}"; else echo "  TCP 8770: free"; fi
  if port_used 11811; then echo "  UDP 11811: bound"; else echo "  UDP 11811: free"; fi
  [ "$mode" != "CONFLICT (container AND native gateway)" ] || return 1
  return 0
}

cmd_container() {
  local archive=separate allow=0 st
  while [ $# -gt 0 ]; do
    case "$1" in
      --archive) archive=${2:-}; shift ;;
      --archive=*) archive=${1#*=} ;;
      --allow-drift) allow=1 ;;
      --dry-run) DRY=1 ;;
      *) echo "stack_mode container: unknown option '$1'" >&2; exit 2 ;;
    esac
    shift
  done
  case "$archive" in separate|shared) ;; *) echo "stack_mode: --archive separate|shared" >&2; exit 2 ;; esac
  st=$(ctr_state)
  case "$st" in
    running)
      [ "$(ctr_role)" = car ] || fail "$CTR runs as '$(ctr_role)' (harness): docker_orin.sh down first"
      say "already in container mode ($CTR running)"; return 0 ;;
    unknown) fail "Docker does not answer (above): the container cannot start; nothing was changed" ;;
  esac
  say "-> container (archive $archive$( [ "$allow" = 1 ] && echo ', allow drift'); $CTR: $st)"
  # 1. the native side, completely
  if [ -x "$NATIVE_CTL" ]; then
    run "$NATIVE_CTL" killall --reset-discovery --json || fail "native f1tenth_ctl killall failed"
  else
    say "no native f1tenth_ctl at $NATIVE_CTL: nothing native to stop"
  fi
  if unit_active "$GW_UNIT"; then
    run systemctl --user stop "$GW_UNIT" || fail "could not stop $GW_UNIT"
  else
    say "$GW_UNIT is not active"
  fi
  # 2. the guard
  wait_guard jazzy || fail "the guard still refuses (above): the native side is not completely down"
  # 3. the container
  write_unit_env "$archive" "$allow"
  if unit_known "$CAR_UNIT"; then
    local rc=0
    # The unit is a oneshot with RemainAfterExit: it stays "active (exited)"
    # after its container exited or crashed, after a `docker_orin down` by
    # hand or a Docker restart, and `start` on an active unit does nothing.
    # Stop it first. Its ExecStop, `docker_orin down`, finds no running
    # container and only reports leftovers; exit 3 (stale Fast DDS SHM of the
    # dead container, listed in its history) leaves the unit "failed", which
    # `start` accepts. What counts is that the unit is no longer active.
    if unit_active "$CAR_UNIT"; then
      say "$CAR_UNIT is still active although $CTR is $st: stopping the unit first"
      run systemctl --user stop "$CAR_UNIT" \
        || say "WARN: stopping $CAR_UNIT reported a failure (its ExecStop is docker_orin down: docker_orin history)"
      if [ "$DRY" != 1 ] && unit_active "$CAR_UNIT"; then fail "could not stop the stale $CAR_UNIT: it is still active"; fi
    fi
    run systemctl --user start "$CAR_UNIT" || rc=$?
    [ "$allow" = 0 ] || write_unit_env "$archive" 0
    [ $rc = 0 ] || fail "$CAR_UNIT did not start (systemctl --user status $CAR_UNIT; docker_orin.sh history)"
  else
    say "$CAR_UNIT is not installed (docker/orin/host/install.sh): docker_orin.sh up directly"
    local args=(up --archive "$archive")
    [ "$allow" = 0 ] || args+=(--allow-drift)
    run "$DOCKER_ORIN" "${args[@]}" || fail "docker_orin.sh up failed"
  fi
  # 4. what really runs: a start that "succeeded" must have left the car's
  # container running.
  if [ "$DRY" != 1 ]; then
    local role
    st=$(ctr_state); role=$(ctr_role)
    if [ "$st" != running ] || [ "$role" != car ]; then
      fail "after the start $CTR is $st${role:+ (role $role)}, not running as the car: the car has NO stack now (docker_orin.sh status; docker_orin.sh history; stack_mode native)"
    fi
  fi
  [ "$DRY" = 1 ] || say "container mode in $(( $(date +%s) - T0 )) s$( [ $(( $(date +%s) - T0 )) -le 60 ] || echo ' (over the 60 s target)')"
}

cmd_native() {
  local st
  while [ $# -gt 0 ]; do
    case "$1" in --dry-run) DRY=1 ;; *) echo "stack_mode native: unknown option '$1'" >&2; exit 2 ;; esac
    shift
  done
  # Docker not answering ("unknown") does not stop the switch: the guard's
  # process scan below still sees a container stack that would be running.
  st=$(ctr_state)
  say "-> native ($CTR: $st)"
  # 1. the container (the unit's ExecStop is docker_orin.sh down)
  if unit_active "$CAR_UNIT"; then run systemctl --user stop "$CAR_UNIT" || fail "could not stop $CAR_UNIT"; fi
  # Started without the unit (or the unit did not stop it): docker_orin.sh
  # down directly; its exit 3 (leftovers, listed) is judged by the guard next.
  local need_down=0
  if [ "$DRY" = 1 ]; then
    if [ "$st" = running ] && ! unit_active "$CAR_UNIT"; then need_down=1; fi
  elif [ "$(ctr_state)" = running ]; then
    need_down=1
  fi
  if [ "$need_down" = 1 ]; then
    local rc=0
    run "$DOCKER_ORIN" down || rc=$?
    [ $rc = 0 ] || [ $rc = 3 ] || fail "docker_orin.sh down failed (exit $rc)"
  fi
  # 2. the guard
  wait_guard humble || fail "the guard still refuses (above)"
  # 3. the native gateway (the stack starts on demand)
  if unit_known "$GW_UNIT"; then
    run systemctl --user start "$GW_UNIT" || fail "could not start $GW_UNIT"
  else
    say "$GW_UNIT is not installed: nothing native to start"
  fi
  [ "$DRY" = 1 ] || say "native mode in $(( $(date +%s) - T0 )) s$( [ $(( $(date +%s) - T0 )) -le 60 ] || echo ' (over the 60 s target)')"
}

[ $# -gt 0 ] || { usage >&2; exit 2; }
sub=$1; shift
case "$sub" in
  status) cmd_status ;;
  container) cmd_container "$@" ;;
  native) cmd_native "$@" ;;
  -h|--help|help) usage ;;
  *) echo "stack_mode: unknown mode '$sub'" >&2; usage >&2; exit 2 ;;
esac
