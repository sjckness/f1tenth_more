#!/bin/bash
# The Orin at full speed while the stack runs: power mode MAXN plus
# jetson_clocks (CPU, GPU and EMC locked at their maximum), and the power
# mode from before once the stack is gone. Runs ON THE ORIN, as the root
# service f1tenth-perf.service; nothing in the stack calls it.
#
#   scripts/orin/perf_mode.sh status          # read-only: mode, clocks, stack, boost
#   sudo scripts/orin/perf_mode.sh install    # install, enable and start the service
#   sudo scripts/orin/perf_mode.sh uninstall  # stop it (restores the mode), remove it
#   sudo f1tenth-perf-mode boost|restore      # by hand, only with the service stopped
#   f1tenth-perf-mode watch                   # the service's loop
#
# "The stack runs" means a process whose command line matches
# F1T_PERF_STACK_MATCH, by default f1tenth_ctl's F1T_STACK_MATCH (ctl.env).
# That covers f1tenth_ctl, a launch typed by hand, and the Jazzy container
# (the host sees the container's processes).
#
# Boost: jetson_clocks --store, nvpmodel -m <F1T_PERF_RUN_MODE>, jetson_clocks.
# Restore: jetson_clocks --restore, nvpmodel -m <the mode before the boost>.
# The marker in the state dir holds that mode and survives a reboot, so a boost
# left behind by a crash, a reboot or a power cut is restored when the service
# next starts. A mode set by hand, with no boost of ours pending, is never
# touched.
#
# Settings (environment; the service reads /etc/default/f1tenth-perf):
#   F1T_PERF_RUN_MODE     nvpmodel mode while the stack runs (MAXN)
#   F1T_PERF_IDLE_MODE    mode to restore; empty = the one before the boost
#   F1T_PERF_CLOCKS       1 = jetson_clocks while boosted (1); 0 = the mode only
#   F1T_PERF_FAN          1 = jetson_clocks --fan, fan at full speed; 0 (default)
#                         leaves the fan to nvfancontrol
#   F1T_PERF_GRACE_S      restore after this long without the stack (30), so a
#                         stack restart keeps the boost
#   F1T_PERF_POLL_S       how often the watcher looks (2)
#   F1T_PERF_STACK_MATCH  pgrep -f regex for the stack
#
# Both default modes keep all 12 cores online. A mode that takes cores
# offline (MODE_15W, MODE_30W) can make nvpmodel ask for a reboot; this script
# never answers that prompt.
set -euo pipefail

RUN_MODE="${F1T_PERF_RUN_MODE:-MAXN}"
IDLE_MODE="${F1T_PERF_IDLE_MODE:-}"
CLOCKS="${F1T_PERF_CLOCKS:-1}"
FAN="${F1T_PERF_FAN:-0}"
GRACE_S="${F1T_PERF_GRACE_S:-30}"
POLL_S="${F1T_PERF_POLL_S:-2}"
STACK_MATCH="${F1T_PERF_STACK_MATCH:-ros2 launch f1tenth_bringup (supervisor_bringup|stack_bringup)\.launch\.py}"
NVPMODEL_CONF="${F1T_PERF_NVPMODEL_CONF:-/etc/nvpmodel.conf}"
# STATE_DIRECTORY: set by the unit's StateDirectory=, the same path.
STATE_DIR="${F1T_PERF_STATE_DIR:-${STATE_DIRECTORY:-/var/lib/f1tenth-perf}}"
MARKER="$STATE_DIR/boosted"     # a boost is pending; holds the mode before it
STORE="$STATE_DIR/clocks.conf"  # jetson_clocks --store, taken before the boost

SERVICE=f1tenth-perf
BIN=/usr/local/sbin/f1tenth-perf-mode
UNIT=/etc/systemd/system/$SERVICE.service

log() { printf 'f1tenth-perf: %s\n' "$*"; }
die() { log "$*" >&2; exit 1; }
bad_config() { log "$*" >&2; exit 2; }
need_root() { [ "$(id -u)" -eq 0 ] || die "$1 needs root: sudo $0 $1"; }
rd() { if [ -r "$1" ]; then printf '%s' "$(<"$1")"; else printf '?'; fi; }
mhz() {  # value, divisor
  if [[ $1 =~ ^[0-9]+$ ]]; then printf '%s' $(($1 / $2)); else printf '?'; fi
}

current_mode() { nvpmodel -q 2>/dev/null | sed -n 's/^NV Power Mode: *//p' || true; }

mode_id() {  # mode name -> its ID in nvpmodel.conf; nothing if unknown
  [[ $1 =~ ^[A-Za-z0-9_]+$ ]] || return 0
  local re="^< *POWER_MODEL +ID=([0-9]+) +NAME=$1 *>"
  sed -nE "/$re/{s/$re.*/\1/p;q}" "$NVPMODEL_CONF"
}

set_mode() {
  local id
  [ "$(current_mode)" != "$1" ] || return 0
  id=$(mode_id "$1")
  if [ -z "$id" ]; then
    log "WARN: no power mode $1 in $NVPMODEL_CONF; left at $(current_mode)"
    return 0
  fi
  # stdin /dev/null: a reboot prompt reads EOF and does not reboot.
  nvpmodel -m "$id" </dev/null || log "WARN: nvpmodel -m $id ($1) failed"
}

clocks() {  # one line: CPU0 and GPU, cur min-max
  local cpu=/sys/devices/system/cpu/cpu0/cpufreq gpus=(/sys/class/devfreq/*.gpu)
  local gpu=${gpus[0]}
  printf 'cpu0 %s MHz (%s-%s, %s), gpu %s MHz (%s-%s)' \
    "$(mhz "$(rd $cpu/scaling_cur_freq)" 1000)" "$(mhz "$(rd $cpu/scaling_min_freq)" 1000)" \
    "$(mhz "$(rd $cpu/scaling_max_freq)" 1000)" "$(rd $cpu/scaling_governor)" \
    "$(mhz "$(rd "$gpu/cur_freq")" 1000000)" "$(mhz "$(rd "$gpu/min_freq")" 1000000)" \
    "$(mhz "$(rd "$gpu/max_freq")" 1000000)"
}

stack_running() { pgrep -f -- "$STACK_MATCH" >/dev/null; }

boost() {  # why
  if [ ! -e "$MARKER" ]; then
    # What restore needs is written before anything changes.
    if [ "$CLOCKS" = 1 ]; then
      rm -f -- "${STORE:?}"  # --store asks before overwriting
      jetson_clocks --store "$STORE" >/dev/null ||
        log "WARN: jetson_clocks --store failed; restore will set the mode only"
    fi
    current_mode >"$MARKER"
  fi
  set_mode "$RUN_MODE"
  if [ "$CLOCKS" = 1 ]; then
    local args=()
    if [ "$FAN" = 1 ]; then args+=(--fan); fi
    jetson_clocks "${args[@]}" || log "WARN: jetson_clocks failed"
  fi
  log "boost ($1): $(current_mode); $(clocks)"
}

restore() {  # why
  local before
  before=$(rd "$MARKER")
  if [ -e "$STORE" ]; then
    jetson_clocks --restore "$STORE" >/dev/null || log "WARN: jetson_clocks --restore failed"
  fi
  local to="${IDLE_MODE:-$before}"
  if [ -n "$to" ] && [ "$to" != '?' ]; then
    set_mode "$to"
  else
    log "WARN: no mode to restore (empty marker, F1T_PERF_IDLE_MODE unset); left at $(current_mode)"
  fi
  rm -f -- "${MARKER:?}" "${STORE:?}"
  log "restore ($1): $(current_mode); $(clocks)"
}

watch_loop() {
  need_root watch
  mkdir -p -- "$STATE_DIR"
  # A bad setting exits 2, which the unit does not restart.
  local rc=0
  pgrep -f -- "$STACK_MATCH" >/dev/null || rc=$?
  [ "$rc" -le 1 ] || bad_config "pgrep refuses F1T_PERF_STACK_MATCH: $STACK_MATCH"
  [ -n "$(mode_id "$RUN_MODE")" ] || bad_config "no power mode $RUN_MODE in $NVPMODEL_CONF"
  [[ $POLL_S =~ ^[0-9]*\.?[0-9]+$ && $POLL_S != 0 && $GRACE_S =~ ^[0-9]+$ ]] ||
    bad_config "F1T_PERF_POLL_S ($POLL_S) must be seconds > 0, F1T_PERF_GRACE_S ($GRACE_S) whole seconds"
  local what=$RUN_MODE
  if [ "$CLOCKS" = 1 ]; then what="$what + jetson_clocks"; fi
  log "boost to $what while /$STACK_MATCH/ runs (every ${POLL_S}s), restore ${GRACE_S}s after"

  local stop=0 sleeper=''
  trap 'stop=1; if [ -n "$sleeper" ]; then kill "$sleeper" 2>/dev/null || true; fi' TERM INT
  if [ -e "$MARKER" ]; then
    if stack_running; then
      boost "stack already running at service start"
    else
      restore "boost left from before this service start"
    fi
  fi
  local seen=$SECONDS
  while [ "$stop" = 0 ]; do
    if stack_running; then
      seen=$SECONDS
      if [ ! -e "$MARKER" ]; then boost "stack started"; fi
    elif [ -e "$MARKER" ] && [ $((SECONDS - seen)) -ge "$GRACE_S" ]; then
      restore "no stack for ${GRACE_S}s"
    fi
    sleep "$POLL_S" &
    sleeper=$!
    wait "$sleeper" || true
    sleeper=''
  done
  if [ -e "$MARKER" ]; then restore "service stopping"; fi
}

status() {
  local procs
  echo "mode:    $(current_mode)"
  echo "clocks:  $(clocks)"
  procs=$(pgrep -af -- "$STACK_MATCH" || true)
  if [ -n "$procs" ]; then
    echo "stack:   running"
    cut -c1-160 <<<"$procs" | sed 's/^/  /'
  else
    echo "stack:   not running"
  fi
  if [ -e "$MARKER" ]; then
    echo "boost:   on (mode before: $(rd "$MARKER"))"
  else
    echo "boost:   off"
  fi
  echo "service: $(systemctl is-active $SERVICE 2>/dev/null || true)"
}

by_hand() {  # boost|restore
  need_root "$1"
  if systemctl is-active -q $SERVICE 2>/dev/null; then
    die "$SERVICE is running and would undo this: sudo systemctl stop $SERVICE first"
  fi
  mkdir -p -- "$STATE_DIR"
  if [ "$1" = boost ]; then
    boost "by hand"
  elif [ -e "$MARKER" ]; then
    restore "by hand"
  else
    log "no boost pending; nothing to restore"
  fi
}

do_install() {
  need_root install
  local here
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  [ -f "$here/$SERVICE.service" ] ||
    die "no $here/$SERVICE.service: run install from the checkout's scripts/orin/"
  install -m 0755 "$here/perf_mode.sh" "$BIN"
  install -m 0644 "$here/$SERVICE.service" "$UNIT"
  systemctl daemon-reload
  systemctl enable $SERVICE
  systemctl restart $SERVICE  # a reinstall runs the new copy
  sleep 1
  systemctl --no-pager --lines=5 status $SERVICE || true
}

do_uninstall() {
  need_root uninstall
  systemctl disable --now $SERVICE 2>/dev/null || true  # stopping restores
  rm -f -- "$UNIT" "$BIN"
  systemctl daemon-reload
  if [ -e "$MARKER" ]; then restore "uninstall"; fi
  log "removed $SERVICE (/etc/default/$SERVICE, if any, is kept)"
}

case "${1:-status}" in
  status) status ;;
  watch) watch_loop ;;
  boost | restore) by_hand "$1" ;;
  install) do_install ;;
  uninstall) do_uninstall ;;
  *) die "usage: $0 status|install|uninstall|boost|restore|watch" ;;
esac
