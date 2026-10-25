#!/usr/bin/env bash
# D3 launch harness: the full stack through the PRODUCTION
# supervisor_bringup.launch.py, started, measured and stopped by this script.
# (Port of the old jazzy line's Phase 5 harness to main-jazzy's components,
# nodes and topics.)
#
# Runs on: the Orin host with native Humble sourced (the native side of P9),
# inside the harness container (the Jazzy side; started from the host by
# docker/orin/checks/d3_side.sh, which records image ID, nvpmodel and clocks
# first) and on linus (bag-fed dry smoke). Same script, same arguments on
# both sides; the comparison is done afterwards over the two run directories
# by phase5_summary.py (each run) and parity_gates.py (native vs container).
#
#   phase5_bringup.sh step4     OUT_DIR      bring-up, rates, TF, graph, every
#                                            node's parameters, one mission
#                                            through the BT (mission logger),
#                                            the restart test, shutdown
#   phase5_bringup.sh cycles    OUT_DIR N    N bring-up / shutdown cycles (P9.2)
#   phase5_bringup.sh hold      OUT_DIR      bring-up, HOLD_SEC up: rates and
#                                            CPU/memory over RATE_S after
#                                            WARMUP_S, TF writers, a process
#                                            audit mid-way and at the end
#                                            (P9.2 hold, P9.3, P9.6 switches)
#   phase5_bringup.sh logger    OUT_DIR      bring-up, one mission, the run the
#                                            logger wrote (P9.2 "logger
#                                            recording")
#   phase5_bringup.sh snapshot  OUT_DIR [ARG:=V ...]  process list + every node's
#                                            parameters (compare_snapshots.py)
#   phase5_bringup.sh isolation OUT_DIR N    N bring-ups checked for discovery
#                                            isolation (isolation_check.py)
#
# A precheck refuses to start while any ROS stack process runs (CLAUDE.md:
# run nothing ROS while a batch runs), while the Discovery Server port is held
# by a process this PID namespace cannot see (the other side, seen from a
# container), and, on a host, while the f1tenth_car container runs (native and
# container never together). Every process it starts is stopped by PID or
# process group; after each shutdown the leftovers are AUDITED first
# (proc_audit.py), then swept so the next cycle starts clean.
#
# FEED:
#   bag  (default) hardware and perception registered with no launch files
#        (phase5_components.py --empty), BAG played with every header stamp set
#        to the wall clock (restamp_play.py), as the drivers would publish.
#        Nothing opens a serial port, a camera or the lidar. During a mission
#        a name-only stub stands in for ackermann_to_vesc_node (preflight).
#   live the production registry (only the logger's runs_dir redirected into
#        OUT_DIR): the real VESC, lidar and ZED drivers, the car on the stand.
#        No bag, no stub. step4 / logger run their 12 s hold mission only with
#        LIVE_MISSION=1 (/drive stays zero).
#        EVERY FEED=live run needs the ESC off or the VESC unplugged: the stack
#        runs on HARNESS_DOMAIN_ID, so no e-stop of the car's graph (ctl,
#        telegram, PWA: domain 0, native gateway stopped) reaches it. The
#        harness refuses FEED=live while /dev/sensors/vesc or a /dev/ttyACM*
#        exists unless LIVE_VESC_OK=1 says the ESC is off (wheels up).
#
# Archive sync: a mission makes the stack's mission logger request the real
# archive sync (native: `systemctl --user start f1tenth-archive.service`
# through PATH; container: the F1TENTH_ARCHIVE_TRIGGER file, which the host's
# .path unit turns into an rsync to linus). The harness redirects both, as
# logger_check.sh does: F1TENTH_ARCHIVE_TRIGGER=RUN/archive_trigger/request
# (set after CTL_ENV is sourced) and a `systemctl` shim first in the stack's
# PATH that only appends its arguments to RUN/systemctl_calls.txt. Nothing is
# synced from a harness run; phase5_summary.py reports what was requested.
#
# Environment:
#   CTL_ENV         an f1tenth_ctl env file (native: <ws>/src/f1tenth_ctl/config/
#                   ctl.env; container: the installed ctl_container.env). The
#                   stack is then started as f1tenth_ctl starts it: that file
#                   sourced with set -a and its F1T_STACK_CMD (with the car
#                   profile's launch arguments), plus this harness's arguments.
#                   Without it: `ros2 launch f1tenth_bringup
#                   supervisor_bringup.launch.py` in the caller's environment.
#   STACK_ARGS      launch arguments appended last (default FEED=bag:
#                   enable_intelligence:=false, FEED=live: none). Rust switch
#                   trials go here (P9.6), e.g. "wall_tracker_impl:=rust".
#   BAG             input bag for FEED=bag (filter_bag_for_layer.py output)
#   HARNESS_DOMAIN_ID (85), DS_PORT (11811): the stack's domain and Discovery
#                   Server port; every probe uses the same
#   HOLD_SEC (300), WARMUP_S (20), RATE_S (30), SETTLE_S (150), STABLE_S (15)
#   KEY_TOPICS      topics whose rates are measured (default below)
#   COMPONENTS_ARGS phase5_components.py arguments (default per FEED)
#   LIVE_MISSION=1  FEED=live: run step4's / logger's hold mission
#   LIVE_VESC_OK=1  FEED=live with a VESC attached: the ESC is off (see FEED)
#                   (LIVE_VESC_DEVS: the device paths checked, default
#                   "/dev/sensors/vesc /dev/ttyACM*"; the tests override it)
#   STACK_SUPER_CLIENT=1  ROS_SUPER_CLIENT=TRUE for the whole stack (when no
#                   CTL_ENV decides it; the native Humble ctl.env sets it)
#   ALLOW_RUNNING_DS=1  accept a Discovery Server already running on DS_PORT in
#                   this PID namespace (reuse it, as production does)
#   CPU_REMAP=1     linus only: the launch files pin nodes with `taskset -c`
#                   lists made for the Orin's 12 cores (mpc_corr 10,11); on a
#                   smaller host taskset refuses and the node never starts. A
#                   `taskset` shim first in the stack's PATH maps each CPU c to
#                   c % nproc. Never set on the Orin.
#   PARITY_IGNORE   linus only: a regex of process command lines that the
#                   precheck and every process audit ignore (other worktrees'
#                   isolated test runs). Never set on the Orin.
set -uo pipefail

usage() { sed -n '2,/^set -uo pipefail/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }
case "${1:-}" in -h|--help|'') usage; [ -n "${1:-}" ]; exit $? ;; esac
MODE=$1
[ $# -ge 2 ] || { usage >&2; exit 2; }
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
need_empty_dir "$2"
OUT=$(parity_abs "$2")
shift 2

FEED=${FEED:-bag}
case "$FEED" in
  bag)
    COMPONENTS_ARGS=${COMPONENTS_ARGS---empty hardware perception}
    STACK_ARGS=${STACK_ARGS-enable_intelligence:=false} ;;
  live)
    COMPONENTS_ARGS=${COMPONENTS_ARGS-}
    STACK_ARGS=${STACK_ARGS-}
    # No e-stop reaches the harness domain: refuse with a VESC attached unless
    # the operator vouches that the ESC is off (see FEED above).
    if [ "${LIVE_VESC_OK:-0}" != 1 ]; then
      # shellcheck disable=SC2086  # a list of paths and globs (tests override it)
      for d in ${LIVE_VESC_DEVS:-/dev/sensors/vesc /dev/ttyACM*}; do
        if [ -e "$d" ]; then
          echo "refusing FEED=live: $d exists and no e-stop reaches the harness domain;" \
               "switch the ESC off (wheels up) and set LIVE_VESC_OK=1, or unplug the VESC" >&2
          exit 2
        fi
      done
    fi ;;
  *) echo "FEED must be bag or live" >&2; exit 2 ;;
esac
case "$MODE" in
  step4|cycles|hold|logger|isolation)
    if [ "$FEED" = bag ]; then
      : "${BAG:?FEED=bag needs BAG=<input bag directory> (see filter_bag_for_layer.py)}"
      BAG=$(parity_abs "$BAG") || exit 2
    fi ;;
  snapshot) ;;
  *) echo "unknown mode $MODE" >&2; usage >&2; exit 2 ;;
esac
if [ -n "${CTL_ENV:-}" ] && [ ! -r "$CTL_ENV" ]; then
  echo "CTL_ENV=$CTL_ENV is not readable" >&2; exit 2
fi
export ROS_DOMAIN_ID=${HARNESS_DOMAIN_ID:-85}
DS_PORT=${DS_PORT:-11811}
export ROS_DISCOVERY_SERVER=127.0.0.1:$DS_PORT
unset ROS_SUPER_CLIENT
HARNESS_T0=$(date +%s)
LAUNCH_PID=""
PLAY=""
STUB=""
WATCH=""
CUR_RUN=""
SUPERVISOR_LOCK=${COMPONENT_SUPERVISOR_LOCK:-/tmp/component_supervisor.lock}
LOGGER_LOCK=${MISSION_LOGGER_LOCK:-/tmp/mission_logger.lock}
KEY_TOPICS=${KEY_TOPICS:-/scan /odom /odometry/filtered /ekf_global/odometry/filtered \
/slam/pose /slam/pose_calibrated /slam/map /tf /costmap/boundaries /costmap/front_clearance \
/drive /ackermann_drive /safety_stop /mpc/solver_status /behavior/tree_status /mission/status \
/diagnostics /diagnostics/system_status /perception/virtual_wall /perception/walls \
/perception/wall_observations /obstacle_clearance /joint_states /sensors/core /sensors/imu/raw \
/camera/detections /camera/detections_3d /perception/obstacles_2d /costmap/semantic_tracks}
CYCLE_TOPICS="/odometry/filtered /ekf_global/odometry/filtered /slam/pose \
/diagnostics/system_status /behavior/tree_status /mpc/solver_status /tf /scan /drive"
DS_RE='fast-discovery-server|fastdds(\.py)? discovery|ensure_discovery_server'

probe() { ROS_SUPER_CLIENT=TRUE python3 "$HERE/stack_probe.py" "$@"; }

# Every helper process this harness started, stopped on any exit; an
# interrupted run also takes the stack it started down (launch, its
# Discovery Server, every component group the supervisor tracked).
harness_cleanup() {
  local g groups
  for g in $WATCH $STUB $PLAY; do pg_alive "$g" && pg_stop "$g" 5 3 > /dev/null; done
  if [ -n "$LAUNCH_PID" ]; then
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "harness exit with the stack up: stopping it" >&2
      kill -INT "$LAUNCH_PID" 2>/dev/null
      pid_wait_gone "$LAUNCH_PID" 30 || kill -KILL "$LAUNCH_PID" 2>/dev/null
    fi
    groups="$LAUNCH_PID ${CUR_RUN:+$(tracked_pgids "$CUR_RUN")}"
    # shellcheck disable=SC2086  # a list of group ids
    python3 "$HERE/proc_audit.py" sweep --pgid $groups >&2
  fi
  WATCH="" STUB="" PLAY="" LAUNCH_PID=""
}
trap harness_cleanup EXIT
trap 'exit 130' INT TERM

# abort MSG...: the reason on stdout (the run's precheck.txt) and stderr, exit 2
abort() {
  printf 'ABORT: %s\n' "$*"
  [ -t 1 ] || printf 'ABORT: %s\n' "$*" >&2
  exit 2
}

precheck() {
  local snap busy el owner
  el=$(( $(date +%s) - HARNESS_T0 + 1 ))
  # (proc_audit.py honours PARITY_IGNORE, see the header.)
  snap=$(python3 "$HERE/proc_audit.py" snapshot)
  # A Discovery Server this harness run started (it may outlive a cycle's
  # launch) is fine; one from before only with ALLOW_RUNNING_DS=1.
  busy=$(printf '%s\n' "$snap" | awk -v el="$el" -v allow="${ALLOW_RUNNING_DS:-0}" -v re="$DS_RE" \
    'NF && !($0 ~ re && (allow == 1 || $4 <= el))')
  if [ -n "$busy" ]; then
    abort "ROS processes already running:" "$busy"
  fi
  if udp_port_bound "$DS_PORT" && [ -z "$(udp_port_owner "$DS_PORT")" ]; then
    abort "UDP $DS_PORT is bound by a process outside this PID namespace" \
          "(the other side's Discovery Server? native and container never together)"
  fi
  if [ ! -e /.dockerenv ] && [ ! -e /run/.containerenv ] && command -v docker > /dev/null 2>&1; then
    owner=$(timeout 10 docker ps -q -f 'name=^f1tenth_car$' 2>/dev/null)
    if [ -n "$owner" ]; then
      abort "container f1tenth_car is running on this host; the native side runs alone"
    fi
  fi
  # A lock naming a dead pid is stale and both nodes reclaim it at startup
  # (that path is part of what runs here). A live holder is a running stack.
  for l in "$SUPERVISOR_LOCK" "$LOGGER_LOCK"; do
    if [ -e "$l" ]; then
      if kill -0 "$(cat "$l")" 2>/dev/null; then abort "$l held by live pid $(cat "$l")"; fi
      echo "stale lock $l (dead pid $(cat "$l")), left for the node to reclaim"
    fi
  done
  for d in /dev/ttyACM* /dev/ttyUSB* /dev/sensors; do
    [ -e "$d" ] && echo "serial device present: $d"
  done
  true
}

shm_list() { find /dev/shm -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | sort; }

# make_taskset_shim RUN_DIR: the CPU_REMAP shim (see the header); prints its dir
make_taskset_shim() {
  local d="$1/shim" real
  real=$(command -v taskset) || { echo "no taskset on PATH" >&2; exit 2; }
  mkdir -p "$d"
  cat > "$d/taskset" <<EOF
#!/usr/bin/env python3
# phase5_bringup.sh CPU_REMAP shim: \`taskset -c LIST CMD...\` with every CPU of
# LIST mapped to cpu % nproc, then the real $real.
import os, sys
a = sys.argv[1:]
if len(a) >= 2 and a[0] == '-c':
    n = os.cpu_count() or 1
    cpus = set()
    for part in a[1].split(','):
        lo, _, hi = part.partition('-')
        cpus.update(range(int(lo), int(hi or lo) + 1))
    a[1] = ','.join(str(c) for c in sorted({c % n for c in cpus}))
os.execv('$real', ['$real'] + a)
EOF
  chmod +x "$d/taskset"
  echo "$d"
}

# make_archive_shim RUN_DIR: the `systemctl` shim of the archive-sync redirect
# (see the header) and the trigger file's directory; prints the shim's dir
make_archive_shim() {
  local d="$1/shim"
  mkdir -p "$d" "$1/archive_trigger"
  cat > "$d/systemctl" <<EOF
#!/bin/sh
# phase5_bringup.sh: a harness run never starts the archive sync; the call is
# only recorded.
echo "\$(date -Is) \$*" >> "$1/systemctl_calls.txt"
exit 0
EOF
  chmod +x "$d/systemctl"
  echo "$d"
}

# start_stack RUN_DIR [launch args...] -> LAUNCH_PID (its own process group), T0
start_stack() {
  local run=$1
  shift
  CUR_RUN=$run
  mkdir -p "$run/ros_log" "$run/runs" "$run/supervisor"
  parity_context "$run"
  # shellcheck disable=SC2086  # COMPONENTS_ARGS is a list of words
  if ! python3 "$HERE/phase5_components.py" "$run/components.yaml" $COMPONENTS_ARGS \
      --logger-runs-dir "$run/runs" > "$run/components_changes.txt" 2>&1; then
    cat "$run/components_changes.txt" >&2; exit 3
  fi
  shm_list > "$run/shm_before.txt"
  local args=("components_config:=$run/components.yaml" "log_dir:=$run/supervisor"
              "discovery_server_port:=$DS_PORT")
  # shellcheck disable=SC2206  # STACK_ARGS is a list of name:=value words
  args+=($STACK_ARGS "$@")
  {
    echo "FEED=$FEED"
    echo "CTL_ENV=${CTL_ENV:-}"
    echo "STACK_ARGS=$STACK_ARGS"
    echo "launch args: ${args[*]}"
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
    echo "STACK_SUPER_CLIENT=${STACK_SUPER_CLIENT:-0} ROS_DISTRO=${ROS_DISTRO:-?}"
    echo "PARITY_IGNORE=${PARITY_IGNORE:-} ALLOW_RUNNING_DS=${ALLOW_RUNNING_DS:-0}"
  } > "$run/stack_env.txt"
  local shim trigger="$run/archive_trigger/request"
  shim=$(make_archive_shim "$run")
  echo "archive sync redirected: F1TENTH_ARCHIVE_TRIGGER=$trigger, systemctl shim $shim/systemctl" \
    >> "$run/stack_env.txt"
  if [ "${CPU_REMAP:-0}" = 1 ]; then
    make_taskset_shim "$run" > /dev/null
    echo "CPU_REMAP=1: taskset shim $shim/taskset (nproc $(nproc))" >> "$run/stack_env.txt"
  fi
  T0=$(date +%s.%N)
  echo "$T0" > "$run/t0.txt"
  pushd "$run" > /dev/null || exit 2  # cwd of the stack: mpc_log.csv lands in the run
  if [ -n "${CTL_ENV:-}" ]; then
    # The redirect is set AFTER the env file, which sets its own trigger path.
    # shellcheck disable=SC2016  # expanded by the inner bash
    bg_start LAUNCH_PID "$run/launch.log" env ROS_LOG_DIR="$run/ros_log" \
      P5_DOMAIN="$ROS_DOMAIN_ID" P5_DS="$ROS_DISCOVERY_SERVER" P5_SHIM="$shim" \
      P5_TRIGGER="$trigger" \
      bash -c 'set -a; source "$1" || exit 97; set +a; shift
               export ROS_DOMAIN_ID=$P5_DOMAIN ROS_DISCOVERY_SERVER=$P5_DS
               export F1TENTH_ARCHIVE_TRIGGER=$P5_TRIGGER PATH="$P5_SHIM:$PATH"
               echo "[phase5_bringup] stack command: $F1T_STACK_CMD $*"
               eval "exec $F1T_STACK_CMD \"\$@\""' _ "$CTL_ENV" "${args[@]}"
  else
    local sc=()
    [ "${STACK_SUPER_CLIENT:-0}" = 1 ] && sc=(ROS_SUPER_CLIENT=TRUE)
    bg_start LAUNCH_PID "$run/launch.log" env ROS_LOG_DIR="$run/ros_log" \
      PATH="$shim:$PATH" F1TENTH_ARCHIVE_TRIGGER="$trigger" "${sc[@]}" \
      ros2 launch f1tenth_bringup supervisor_bringup.launch.py "${args[@]}"
  fi
  popd > /dev/null || exit 2
  echo "$LAUNCH_PID" > "$run/launch_pid.txt"
}

# wait_settled RUN_DIR: node set stable for STABLE_S, SETTLE_S at most
wait_settled() {
  probe --out "$1/nodes_startup.json" nodes --duration "${SETTLE_S:-150}" --t0 "$T0" \
    --stable "${STABLE_S:-15}"
  cp "$1/supervisor/tracked_pgids.json" "$1/tracked_pgids_settled.json" 2>/dev/null
  true
}

snapshot_procs() {  # RUN_DIR LABEL
  local p e
  python3 "$HERE/proc_audit.py" snapshot --out-all "$1/procs_all_$2.txt" --out-stack "$1/procs_$2.txt"
  python3 "$HERE/proc_audit.py" audit --procs-all "$1/procs_all_$2.txt" \
    --launch-pid "$LAUNCH_PID" --out "$1/audit_$2.json" > /dev/null
  # ROS_SUPER_CLIENT / ROS_DISCOVERY_SERVER as each process actually has them,
  # the archive-sync redirect (trigger path, first PATH entry = the shim dir).
  while read -r p _; do
    e=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null \
        | grep -E '^(ROS_(SUPER_CLIENT|DISCOVERY_SERVER|DOMAIN_ID)|FASTRTPS_DEFAULT_PROFILES_FILE|RMW_IMPLEMENTATION|F1TENTH_ARCHIVE_TRIGGER|PATH)=' \
        | sed 's/^\(PATH=[^:]*\):.*/\1:.../' | sort | tr '\n' ' ')
    printf '%s %s| %s\n' "$p" "${e:-<none>}" "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-160)"
  done < "$1/procs_$2.txt" > "$1/environ_$2.txt"
  while read -r p _; do
    printf '%s %s\n' "$p" "$(taskset -pc "$p" 2>/dev/null | sed 's/.*: //')"
  done < "$1/procs_$2.txt" > "$1/affinity_$2.txt"
}

# tracked component groups (latest copy), for the sweep after an audit
tracked_pgids() {
  python3 -c 'import json, sys
seen = set()
for f in sys.argv[1:]:
    try:
        for pids in json.load(open(f)).values():
            seen.update(pids)
    except (OSError, ValueError):
        pass
print(" ".join(str(p) for p in sorted(seen)))' "$1"/tracked_pgids_*.json "$1"/restart_*/tracked_pgids_after.json 2>/dev/null
}

# stop_stack RUN_DIR: SIGINT the launch (as f1tenth_ctl stop and Ctrl-C do),
# record how it went, audit what is left, then sweep it.
stop_stack() {
  local run=$1 t_stop rc groups l p
  t_stop=$(date +%s.%N)
  kill -INT "$LAUNCH_PID" 2>/dev/null
  if ! pid_wait_gone "$LAUNCH_PID" 30; then
    echo "launch still alive 30 s after SIGINT: SIGKILL" >> "$run/shutdown.txt"
    kill -KILL "$LAUNCH_PID" 2>/dev/null
  fi
  wait "$LAUNCH_PID"
  rc=$?
  sleep 2
  {
    echo "launch exit code: $rc"
    echo "shutdown duration: $(awk -v a="$t_stop" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a - 2 }') s"
  } >> "$run/shutdown.txt"
  python3 "$HERE/proc_audit.py" snapshot --out-all "$run/procs_all_after.txt" \
    --out-stack "$run/leftover_procs.txt"
  python3 "$HERE/proc_audit.py" audit --procs-all "$run/procs_all_after.txt" --expect-none \
    --out "$run/audit_after.json" > /dev/null
  shm_list > "$run/shm_after.txt"
  # A lock left after shutdown: its path, the pid it names, and whether that
  # pid lives (a dead one is reclaimed at the next start; a live one is not).
  for l in "$SUPERVISOR_LOCK" "$LOGGER_LOCK"; do
    [ -e "$l" ] || continue
    p=$(cat "$l" 2>/dev/null)
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "$l $p alive"; else echo "$l ${p:-?} dead"; fi
  done > "$run/leftover_locks.txt"
  groups="$LAUNCH_PID $(tracked_pgids "$run")"
  # shellcheck disable=SC2086  # a list of group ids
  python3 "$HERE/proc_audit.py" sweep --pgid $groups > "$run/sweep.txt" 2>&1
  LAUNCH_PID=""
}

discovery_errors() {  # RUN_DIR -> counts over every log of the run
  local tmp
  tmp=$(mktemp)
  find "$1" \( -name '*.log' -o -name '*.txt' \) ! -name discovery_errors.txt -print0 \
    | xargs -0 cat 2>/dev/null > "$tmp"
  {
    echo "Matching unexisting participant: $(grep -c 'Matching unexisting participant' "$tmp")"
    echo "DISCOVERY_DATABASE (any): $(grep -c 'DISCOVERY_DATABASE' "$tmp")"
    echo "[RTPS_ / [DISCOVERY / [PARTICIPANT errors (any): $(grep -cE '\[(RTPS_[A-Z_]+|DISCOVERY[A-Z_]*|PARTICIPANT) Error' "$tmp")"
    grep -hE 'DISCOVERY_DATABASE|\[(RTPS_[A-Z_]+|DISCOVERY[A-Z_]*|PARTICIPANT) Error' "$tmp" \
      | sed 's/^.*\(\[[A-Z_]* Error\]\)/\1/' | sort | uniq -c | sort -rn | head -20
  } > "$1/discovery_errors.txt"
  rm -f -- "$tmp"
}

component_nodes() {  # RUN_DIR LABEL -> component -> node names
  python3 - "$1/tracked_pgids_settled.json" "$1/procs_all_$2.txt" "$HERE" \
    > "$1/component_nodes_$2.json" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[3])
import proc_audit
try:
    pg = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    pg = {}
procs = proc_audit.parse_lines(open(sys.argv[2]))
out = {}
for comp, pids in pg.items():
    names = set()
    for p in procs:
        if p['pgid'] in pids:
            n = proc_audit.node_name(p['args'])
            if n:
                names.add(n)
    out[comp] = sorted(names)
print(json.dumps(out, indent=2))
PY
}

start_feed() {  # RUN_DIR LOOPS
  [ "$FEED" = bag ] || return 0
  bg_start PLAY "$1/play.log" python3 "$HERE/restamp_play.py" "$BAG" --loops "$2"
}
stop_feed() { [ -n "$PLAY" ] && pg_stop "$PLAY" 5 3 > /dev/null; PLAY=""; }

write_hold_mission() {  # FILE
  cat > "$1" <<'J'
{
  "mission_id": "phase5_logger_hold",
  "schema_version": "2.0",
  "moves": [
    {
      "id": "hold_12s",
      "goal_distance": 0.0,
      "stop_condition": { "type": "time_elapsed", "duration_sec": 12.0 },
      "timeout_sec": 30,
      "on_timeout": "abort"
    }
  ]
}
J
}

# run_mission RUN_DIR: the 12 s hold mission through the BT (logger records it)
run_mission() {
  local run=$1
  if [ "$FEED" = live ] && [ "${LIVE_MISSION:-0}" != 1 ]; then
    echo "FEED=live without LIVE_MISSION=1: mission skipped" > "$run/mission_skipped.txt"
    return 0
  fi
  write_hold_mission "$run/phase5_hold.json"
  # The preflight needs a node named ackermann_to_vesc_node (the VESC
  # converter, a hardware component): with FEED=bag a name-only stub.
  if [ "$FEED" = bag ]; then
    bg_start STUB "$run/stub_vesc.log" python3 "$HERE/graph_stub_node.py" ackermann_to_vesc_node
    sleep 3
  fi
  probe --out "$run/mission.json" mission --path "$run/phase5_hold.json" --wait 60
  [ -n "$STUB" ] && { pg_stop "$STUB" 5 3 > /dev/null; STUB=""; }
  sleep 15   # logger finalize (checksums, extract)
  find "$run/runs" -maxdepth 2 -mindepth 2 -type d | sort > "$run/logger_runs.txt"
}

measure_window() {  # RUN_DIR LABEL: rates + CPU/memory over the same RATE_S
  local run=$1 lb=$2 sp
  python3 "$HERE/sys_sample.py" --duration "${RATE_S:-30}" --out "$run/sys_$lb.json" \
    > "$run/sys_$lb.log" 2>&1 &
  sp=$!
  # shellcheck disable=SC2086  # KEY_TOPICS is a list of topics
  probe --out "$run/rates_$lb.json" rates --duration "${RATE_S:-30}" $KEY_TOPICS
  wait "$sp"
}

case "$MODE" in
step4)
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN"
  snapshot_procs "$RUN" settled
  component_nodes "$RUN" settled
  start_feed "$RUN" 12
  sleep 10
  # shellcheck disable=SC2086
  probe --out "$RUN/rates.json" rates --duration 20 $KEY_TOPICS
  probe --out "$RUN/graph.json" graph /tf /tf_static /diagnostics/system_status /mission/status \
    /scan /odom /drive
  probe --out "$RUN/tf.json" tf --duration 12
  (cd "$RUN" && timeout 60 ros2 run tf2_tools view_frames > view_frames.log 2>&1)
  probe --out "$RUN/params.json" params --skip-prefix /phase5_probe /restamp_player /isolation_watch
  run_mission "$RUN"
  probe --out "$RUN/rates_after_mission.json" rates --duration 10 /diagnostics/system_status \
    /odometry/filtered /tf
  snapshot_procs "$RUN" before_restart
  # Restart test: SIGKILL a component's whole process group, as a crash; the
  # supervisor's watchdog must respawn it.
  for comp in localization navigation diagnostics; do
    d="$RUN/restart_$comp"
    mkdir -p "$d"
    cp -r "$RUN/supervisor" "$d/supervisor_before"
    shm_list > "$d/shm_before.txt"
    pgids=$(python3 -c "import json, sys; print(' '.join(map(str, json.load(open(sys.argv[1])).get(sys.argv[2], []))))" \
            "$RUN/supervisor/tracked_pgids.json" "$comp")
    echo "$pgids" > "$d/killed_pgids.txt"
    [ -n "$pgids" ] || { echo "no tracked process group for $comp" > "$d/skipped.txt"; continue; }
    tk=$(date +%s.%N)
    echo "$tk" > "$d/t_kill.txt"
    for g in $pgids; do kill -KILL -- "-$g" 2>/dev/null; done
    sleep 1
    shm_list > "$d/shm_after_kill.txt"
    probe --out "$d/nodes.json" nodes --duration 60 --t0 "$tk" --stable 12
    shm_list > "$d/shm_after_respawn.txt"
    python3 "$HERE/proc_audit.py" snapshot --out-all "$d/procs_all_after.txt" --out-stack "$d/procs_after.txt"
    python3 "$HERE/proc_audit.py" audit --procs-all "$d/procs_all_after.txt" \
      --launch-pid "$LAUNCH_PID" --out "$d/audit_after.json" > /dev/null
    cp "$RUN/supervisor/tracked_pgids.json" "$d/tracked_pgids_after.json"
    probe --out "$d/rates.json" rates --duration 8 /odometry/filtered \
      /ekf_global/odometry/filtered /mpc/solver_status /drive /diagnostics/system_status \
      /behavior/tree_status /tf
  done
  stop_feed
  cp -r "$RUN/supervisor" "$RUN/supervisor_before_shutdown"
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
cycles)
  N=${1:-10}
  precheck
  for i in $(seq 1 "$N"); do
    RUN="$OUT/cycle_$(printf %02d "$i")"
    mkdir -p "$RUN"
    precheck > "$RUN/precheck.txt"
    start_stack "$RUN"
    wait_settled "$RUN"
    snapshot_procs "$RUN" settled
    component_nodes "$RUN" settled
    start_feed "$RUN" 1
    sleep 8
    # shellcheck disable=SC2086
    probe --out "$RUN/rates.json" rates --duration 10 $CYCLE_TOPICS
    stop_feed
    stop_stack "$RUN"
    discovery_errors "$RUN"
    echo "cycle $i done: $(grep -c . "$RUN/leftover_procs.txt") leftover stack processes," \
         "audit $(python3 -c "import json, sys; print('clean' if json.load(open(sys.argv[1]))['clean'] else 'NOT clean')" "$RUN/audit_after.json")"
    sleep 3
  done
  ;;
logger)
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN"
  snapshot_procs "$RUN" settled
  start_feed "$RUN" 3
  sleep 8
  run_mission "$RUN"
  stop_feed
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
hold)
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN"
  snapshot_procs "$RUN" settled
  component_nodes "$RUN" settled
  start_feed "$RUN" "${HOLD_LOOPS:-40}"
  T_READY=$(date +%s)
  touch "$RUN/READY"
  sleep "${WARMUP_S:-20}"
  measure_window "$RUN" hold
  probe --out "$RUN/tf.json" tf --duration 10
  snapshot_procs "$RUN" mid
  left=$(( ${HOLD_SEC:-300} - ($(date +%s) - T_READY) ))
  [ "$left" -gt 0 ] && sleep "$left"
  snapshot_procs "$RUN" end
  stop_feed
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
isolation)
  # N bring-ups, each checked for discovery isolation by isolation_check.py
  # (graph timeline + input-driven outputs + starvation logs). RUN_GAP (2 s)
  # between a shutdown and the next bring-up.
  N=${1:-30}
  precheck
  for i in $(seq 1 "$N"); do
    RUN="$OUT/run_$(printf %02d "$i")"
    mkdir -p "$RUN"
    precheck > "$RUN/precheck.txt"
    echo "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-<default>} RUN_GAP=${RUN_GAP:-2} FEED=$FEED STACK_ARGS=$STACK_ARGS" > "$RUN/condition.txt"
    head -1 /proc/stat > "$RUN/cpu_t0.txt"
    start_stack "$RUN"
    bg_start WATCH "$RUN/watch.log" env ROS_SUPER_CLIENT=TRUE \
      python3 "$HERE/isolation_check.py" watch --out "$RUN/graph_timeline.jsonl"
    wait_settled "$RUN"
    snapshot_procs "$RUN" settled
    start_feed "$RUN" 1
    T_FEED=$(date +%s.%N)
    echo "$T_FEED" > "$RUN/t_feed.txt"
    sleep 12
    ROS_SUPER_CLIENT=TRUE python3 "$HERE/isolation_check.py" check --run-dir "$RUN" \
      --timeline "$RUN/graph_timeline.jsonl" --t-feed "$T_FEED" --duration 8 \
      --feed "$FEED" --out "$RUN/isolation.json" > "$RUN/isolation.log" 2>&1
    head -1 /proc/stat > "$RUN/cpu_t1.txt"
    cat /proc/loadavg > "$RUN/loadavg.txt"
    stop_feed
    stop_stack "$RUN"
    pg_stop "$WATCH" 5 3 > /dev/null
    WATCH=""
    discovery_errors "$RUN"
    echo "run $i: $(tail -1 "$RUN/isolation.log" | cut -c1-300)"
    sleep "${RUN_GAP:-2}"
  done
  ;;
snapshot)
  precheck
  RUN=$OUT
  start_stack "$RUN" "$@"
  wait_settled "$RUN"
  snapshot_procs "$RUN" settled
  component_nodes "$RUN" settled
  if [ "$FEED" = bag ] && [ -n "${BAG:-}" ]; then
    BAG=$(parity_abs "$BAG") || exit 2
    start_feed "$RUN" 1
    sleep 8
    probe --out "$RUN/rates.json" rates --duration 10 /odometry/filtered \
      /ekf_global/odometry/filtered /slam/pose /diagnostics/system_status /behavior/tree_status /tf
  fi
  probe --out "$RUN/params.json" params --skip-prefix /phase5_probe /restamp_player /isolation_watch
  stop_feed
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
esac
