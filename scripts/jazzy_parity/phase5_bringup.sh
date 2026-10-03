#!/usr/bin/env bash
# Phase 5: the full stack through the PRODUCTION supervisor_bringup.launch.py
# on Thor, with no hardware, fed by a bag.
#
#   phase5_bringup.sh step4 OUT_DIR            bringup, health, TF, logger,
#                                              system_observer, restart test
#   phase5_bringup.sh cycles OUT_DIR N         N bringup/shutdown cycles
#   phase5_bringup.sh snapshot OUT_DIR [LAUNCH_ARGS...]
#                                              bringup, process list + every
#                                              node's parameters, shutdown
#                                              (Step 6 comparisons)
#
# Hardware: the 'hardware' (vesc.launch.py) and 'perception' (ZED, urg_node,
# YOLO/torch detection) components are registered with no launch files via a
# components.yaml generated from production (phase5_components.py), and
# enable_intelligence:=false (llama.cpp is not on Thor). Nothing that opens a
# serial port, camera or lidar is started. A precheck refuses to run if any
# ROS process is already up.
#
# Isolation: ROS_DOMAIN_ID 85. The Discovery Server is the production one
# (ensure_discovery_server.py, 127.0.0.1:11811). Every tool here is a client
# of it; graph queries use ROS_SUPER_CLIENT=TRUE.
#
# Env overrides: BAG (input bag), COMPONENTS_ARGS (phase5_components.py args).
set -uo pipefail
MODE=$1
OUT=$(mkdir -p "$2" && cd "$2" && pwd)
shift 2
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
BAG=${BAG:-$REPO/output/phase5/inputs/bringup_input}
export ROS_DOMAIN_ID=85
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
unset ROS_SUPER_CLIENT
SIGDFL=(python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')
probe() { ROS_SUPER_CLIENT=TRUE python3 "$HERE/stack_probe.py" "$@"; }
# Everything this stack can start, matched on the command line.
STACK_PAT='ros2 launch|/install/|/opt/ros/jazzy/lib/|fast-discovery-server|fastdds discovery|ensure_discovery_server'
stack_procs() { ps -eo pid,pgid,ppid,etimes,args --no-headers | grep -E "$STACK_PAT" | grep -vE "grep|stack_probe|ros2cli.daemon|phase5_bringup|bag play|graph_stub_node" ; }

precheck() {
  local busy
  busy=$(stack_procs | grep -vE "fast-discovery-server|fastdds discovery" || true)
  if [ -n "$busy" ]; then echo "ABORT: ROS processes already running:"; echo "$busy"; exit 2; fi
  # A lock naming a dead pid is stale, and both nodes reclaim it at startup
  # (that path is part of what runs here). A live holder is a running stack.
  for l in /tmp/component_supervisor.lock /tmp/mission_logger.lock; do
    if [ -e "$l" ]; then
      if kill -0 "$(cat "$l")" 2>/dev/null; then echo "ABORT: $l held by live pid $(cat "$l")"; exit 2; fi
      echo "stale lock $l (dead pid $(cat "$l")), left for the node to reclaim"
    fi
  done
  ls /dev/ttyACM* /dev/ttyUSB* /dev/sensors 2>/dev/null | sed 's/^/serial device present (not opened): /'
  true
}

shm_list() { ls -1 /dev/shm | sort; }

# start_stack RUN_DIR [launch args...] -> sets LAUNCH_PID, T0
start_stack() {
  local run=$1; shift
  mkdir -p "$run/ros_log" "$run/runs"
  python3 "$HERE/phase5_components.py" "$run/components.yaml" \
    ${COMPONENTS_ARGS:---empty hardware perception} --logger-runs-dir "$run/runs" > "$run/components_changes.txt"
  # A run that keeps the hardware components registered relies on sim mode to
  # skip the drivers: check that on the generated registry BEFORE launching.
  if printf '%s\n' "$@" | grep -qx 'sim:=true'; then
    python3 - "$run/components.yaml" "$REPO/src/f1tenth_bringup/f1tenth_bringup" <<'PY' || exit 3
import sys, yaml
sys.path.insert(0, sys.argv[2])
from component_supervisor_node import apply_sim_mode, _SIM_SKIPPED_LAUNCH_FILES
reg = {n: [dict(e, args=e.get('args', {})) for e in es]
       for n, es in yaml.safe_load(open(sys.argv[1]))['components'].items()}
left = [(n, e['launch_file']) for n, es in apply_sim_mode(reg).items() for e in es
        if (e['package'], e['launch_file']) in _SIM_SKIPPED_LAUNCH_FILES]
print('sim registry check:', 'driver launch files left: %s' % left if left else 'no driver launch file left')
sys.exit(1 if left else 0)
PY
  fi
  shm_list > "$run/shm_before.txt"
  T0=$(date +%s.%N)
  # STACK_SUPER_CLIENT=1: start the stack from a shell that exports
  # ROS_SUPER_CLIENT=TRUE (every node a super client), as the proposed env
  # script would.
  local sc=(); [ "${STACK_SUPER_CLIENT:-0}" = 1 ] && sc=(env ROS_SUPER_CLIENT=TRUE)
  echo "STACK_SUPER_CLIENT=${STACK_SUPER_CLIENT:-0}" > "$run/stack_env.txt"
  ROS_LOG_DIR="$run/ros_log" "${sc[@]}" "${SIGDFL[@]}" ros2 launch f1tenth_bringup supervisor_bringup.launch.py \
    components_config:="$run/components.yaml" enable_intelligence:=false \
    log_dir:="$run/supervisor" "$@" > "$run/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  echo "$T0" > "$run/t0.txt"
}

# wait_settled RUN_DIR MAX_S: nodes probe until the node set is stable 15 s
wait_settled() {
  probe --out "$1/nodes_startup.json" nodes --duration "$2" --t0 "$T0" --stable 15
}

snapshot_procs() {  # RUN_DIR LABEL
  stack_procs > "$1/procs_$2.txt"
  # ROS_SUPER_CLIENT / ROS_DISCOVERY_SERVER as each process actually has them.
  for p in $(awk '{print $1}' "$1/procs_$2.txt"); do
    e=$(tr '\0' '\n' < /proc/"$p"/environ 2>/dev/null | grep -E '^ROS_(SUPER_CLIENT|DISCOVERY_SERVER|DOMAIN_ID)=' | sort | tr '\n' ' ')
    printf '%s %s| %s\n' "$p" "${e:-<none>}" "$(tr '\0' ' ' < /proc/"$p"/cmdline 2>/dev/null | cut -c1-140)"
  done > "$1/environ_$2.txt"
  for p in $(awk '{print $1}' "$1/procs_$2.txt"); do
    printf '%s %s\n' "$p" "$(taskset -pc "$p" 2>/dev/null | sed 's/.*: //')"
  done > "$1/affinity_$2.txt"
}

# stop_stack RUN_DIR: SIGINT the launch (as Ctrl-C would), record how it went
stop_stack() {
  local run=$1 t_stop rc
  t_stop=$(date +%s.%N)
  kill -INT "$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 1 120); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.25; done
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "launch still alive 30 s after SIGINT" > "$run/shutdown.txt"
    kill -9 "$LAUNCH_PID"
  fi
  wait "$LAUNCH_PID"; rc=$?
  sleep 2
  {
    echo "launch exit code: $rc"
    echo "shutdown duration: $(echo "$(date +%s.%N) - $t_stop - 2" | bc) s"
  } >> "$run/shutdown.txt"
  stack_procs > "$run/leftover_procs.txt"
  shm_list > "$run/shm_after.txt"
  ls /tmp/component_supervisor.lock /tmp/mission_logger.lock 2>/dev/null > "$run/leftover_locks.txt"
}

discovery_errors() {  # RUN_DIR -> counts over every log of the run
  local files
  files=$(find "$1" -name '*.log' -o -name '*.txt' | grep -v discovery_errors)
  {
    echo "Matching unexisting participant: $(cat $files 2>/dev/null | grep -c 'Matching unexisting participant')"
    echo "DISCOVERY_DATABASE (any): $(cat $files 2>/dev/null | grep -c 'DISCOVERY_DATABASE')"
    echo "[RTPS_ / [DISCOVERY / [PARTICIPANT errors (any): $(cat $files 2>/dev/null | grep -cE '\[(RTPS_[A-Z_]+|DISCOVERY[A-Z_]*|PARTICIPANT) Error')"
    cat $files 2>/dev/null | grep -hE 'DISCOVERY_DATABASE|\[(RTPS_[A-Z_]+|DISCOVERY[A-Z_]*|PARTICIPANT) Error' | sed 's/^.*\(\[[A-Z_]* Error\]\)/\1/' | sort | uniq -c | sort -rn | head -20
  } > "$1/discovery_errors.txt"
}

KEY_TOPICS="/odom /scan /odometry/filtered /ekf_global/odometry/filtered /slam/pose /slam/pose_calibrated \
/slam/map /costmap/boundaries /costmap/front_clearance /tf /diagnostics/system_status /diagnostics/battery_status \
/behavior/tree_status /mission/status /mpc/solver_status /mpc/status /drive /ackermann_drive /perception/d_wall \
/perception/d_wall/segment /perception/swept_clearance /obstacle_clearance /perception/lidar_front_wall /diagnostics \
/calibration/in_progress /joint_states /robot_description"

component_nodes() {  # RUN_DIR LABEL -> component -> node names from the supervisor's pgid file
  python3 - "$1/supervisor/tracked_pgids.json" "$1/procs_$2.txt" > "$1/component_nodes_$2.json" <<'PY'
import json, re, sys
pg = json.load(open(sys.argv[1]))
procs = [l.split(None, 4) for l in open(sys.argv[2]) if l.strip()]
out = {}
for comp, pids in pg.items():
    names = []
    for pid, pgid, ppid, et, args in procs:
        if int(pgid) in pids:
            ns = re.search(r'__ns:=(\S+)', args)
            nm = re.search(r'__node:=(\S+)', args)
            if nm:
                names.append(((ns.group(1).rstrip('/') if ns else '') + '/' + nm.group(1)))
    out[comp] = sorted(set(names))
print(json.dumps(out, indent=2))
PY
}

case "$MODE" in
step4)
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN" 150
  snapshot_procs "$RUN" settled
  component_nodes "$RUN" settled
  # Feed it.
  # Re-stamped to wall time, as live drivers would (see restamp_play.py).
  "${SIGDFL[@]}" python3 "$HERE/restamp_play.py" "$BAG" --loops 12 > "$RUN/play.log" 2>&1 < /dev/null &
  PLAY=$!
  sleep 10
  probe --out "$RUN/rates.json" rates --duration 20 $KEY_TOPICS
  probe --out "$RUN/graph.json" graph /tf /tf_static /diagnostics/system_status /mission/status /scan /odom
  probe --out "$RUN/tf.json" tf --duration 12
  (cd "$RUN" && timeout 60 ros2 run tf2_tools view_frames > view_frames.log 2>&1)
  probe --out "$RUN/params.json" params --skip-prefix /phase5_probe
  # Mission logger inside the full stack: a real mission through the BT.
  # The preflight needs a node named ackermann_to_vesc_node (the VESC
  # converter, hardware component): a name-only stub, publishes nothing.
  python3 "$HERE/graph_stub_node.py" ackermann_to_vesc_node > "$RUN/stub_vesc.log" 2>&1 &
  STUB=$!
  cat > "$RUN/phase5_hold.json" <<'J'
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
  sleep 3
  probe --out "$RUN/mission.json" mission --path "$RUN/phase5_hold.json" --wait 60
  kill -INT $STUB; wait $STUB 2>/dev/null
  sleep 15   # logger finalize (extract)
  probe --out "$RUN/rates_after_mission.json" rates --duration 10 /diagnostics/system_status /odometry/filtered /tf
  snapshot_procs "$RUN" before_restart
  # Restart test: SIGKILL a component's whole process group, as a crash.
  for comp in localization navigation diagnostics; do
    d="$RUN/restart_$comp"; mkdir -p "$d"
    cp -r "$RUN/supervisor" "$d/supervisor_before"
    shm_list > "$d/shm_before.txt"
    pgids=$(python3 -c "import json;print(' '.join(map(str,json.load(open('$RUN/supervisor/tracked_pgids.json'))['$comp'])))")
    echo "$pgids" > "$d/killed_pgids.txt"
    tk=$(date +%s.%N); echo "$tk" > "$d/t_kill.txt"
    for g in $pgids; do kill -9 -- -"$g"; done
    sleep 1
    shm_list > "$d/shm_after_kill.txt"
    probe --out "$d/nodes.json" nodes --duration 60 --t0 "$tk" --stable 12
    shm_list > "$d/shm_after_respawn.txt"
    stack_procs > "$d/procs_after.txt"
    cp "$RUN/supervisor/tracked_pgids.json" "$d/tracked_pgids_after.json"
    probe --out "$d/rates.json" rates --duration 8 /odometry/filtered /ekf_global/odometry/filtered \
      /mpc/solver_status /mpc/status /diagnostics/system_status /behavior/tree_status /tf
  done
  kill -INT $PLAY 2>/dev/null; wait $PLAY 2>/dev/null
  cp -r "$RUN/supervisor" "$RUN/supervisor_before_shutdown"
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
cycles)
  N=${1:-10}
  precheck
  BAG=${BAG_CYCLE:-$REPO/output/phase5/inputs/bringup_input}
  for i in $(seq 1 "$N"); do
    RUN="$OUT/cycle_$(printf %02d "$i")"; mkdir -p "$RUN"
    start_stack "$RUN"
    wait_settled "$RUN" 150
    snapshot_procs "$RUN" settled
    component_nodes "$RUN" settled
    "${SIGDFL[@]}" python3 "$HERE/restamp_play.py" "$BAG" > "$RUN/play.log" 2>&1 < /dev/null &
    PLAY=$!
    sleep 8
    probe --out "$RUN/rates.json" rates --duration 10 /odometry/filtered /ekf_global/odometry/filtered \
      /slam/pose /diagnostics/system_status /behavior/tree_status /mpc/solver_status /tf
    kill -INT $PLAY 2>/dev/null; wait $PLAY 2>/dev/null
    stop_stack "$RUN"
    discovery_errors "$RUN"
    echo "cycle $i done: $(grep -c . "$RUN/leftover_procs.txt") leftover procs"
    sleep 3
  done
  ;;
logger)
  # Mission logger inside the full stack only: bringup, feed, one mission
  # through the BT, check what the bag holds, shutdown.
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN" 150
  snapshot_procs "$RUN" settled
  "${SIGDFL[@]}" python3 "$HERE/restamp_play.py" "$BAG" --loops 2 > "$RUN/play.log" 2>&1 < /dev/null &
  PLAY=$!
  python3 "$HERE/graph_stub_node.py" ackermann_to_vesc_node > "$RUN/stub_vesc.log" 2>&1 &
  STUB=$!
  cp "$REPO/output/phase5/step4/phase5_hold.json" "$RUN/phase5_hold.json"
  sleep 8
  probe --out "$RUN/mission.json" mission --path "$RUN/phase5_hold.json" --wait 60
  kill -INT $STUB; wait $STUB 2>/dev/null
  sleep 15
  kill -INT $PLAY 2>/dev/null; wait $PLAY 2>/dev/null
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
hold)
  # Bringup, feed, and keep the stack up HOLD_SEC (default 300) for checks
  # run from outside, then shut down.
  precheck
  RUN=$OUT
  start_stack "$RUN"
  wait_settled "$RUN" 150
  snapshot_procs "$RUN" settled
  "${SIGDFL[@]}" python3 "$HERE/restamp_play.py" "$BAG" --loops 20 > "$RUN/play.log" 2>&1 < /dev/null &
  PLAY=$!
  touch "$RUN/READY"
  sleep "${HOLD_SEC:-300}"
  kill -INT $PLAY 2>/dev/null; wait $PLAY 2>/dev/null
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
snapshot)
  precheck
  RUN=$OUT
  start_stack "$RUN" "$@"
  wait_settled "$RUN" 150
  snapshot_procs "$RUN" settled
  component_nodes "$RUN" settled
  if [ -n "${SNAPSHOT_BAG:-}" ]; then
    "${SIGDFL[@]}" ros2 bag play "$SNAPSHOT_BAG" ${SNAPSHOT_PLAY_ARGS:-} --disable-keyboard-controls > "$RUN/play.log" 2>&1 < /dev/null &
    PLAY=$!
    sleep 8
    probe --out "$RUN/rates.json" rates --duration 10 /clock /odometry/filtered /ekf_global/odometry/filtered \
      /slam/pose /diagnostics/system_status /behavior/tree_status /tf
  fi
  probe --out "$RUN/params.json" params --skip-prefix /phase5_probe
  [ -n "${PLAY:-}" ] && { kill -INT $PLAY 2>/dev/null; wait $PLAY 2>/dev/null; }
  stop_stack "$RUN"
  discovery_errors "$RUN"
  ;;
*) echo "unknown mode $MODE"; exit 2 ;;
esac
