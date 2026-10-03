#!/usr/bin/env bash
# Replay one localization-chain layer against a bag and record its outputs,
# on EITHER distro (Jazzy, this repo's current branch, or Humble at
# e47e646 on the Orin -- the commands below use only `ros2 run`/`ros2 bag`
# CLI, no distro-specific Python API, so the identical script runs on both;
# see the Phase 1 report's Step 1/3 for why e47e646 is the valid Humble
# reference point for this bag).
#
# Isolation: a dedicated ROS_DOMAIN_ID + ROS_AUTOMATIC_DISCOVERY_RANGE=
# LOCALHOST (no Discovery Server) -- this replay runs standalone, not as
# part of the live stack, so the stack's own Discovery Server setup
# (ensure_discovery_server.py, 127.0.0.1:11811) is neither needed nor
# started; LOCALHOST discovery range keeps this process's DDS traffic off
# any other ROS graph that might be running on the same machine. Pick a
# ROS_DOMAIN_ID not in everyday use here (default 77) to further avoid
# collision with anything else already running.
#
# Usage:
#   replay_localization.sh LAYER BAG_DIR OUT_DIR [DOMAIN_ID]
#     LAYER   = relay | ekf_global
#     BAG_DIR = path to the prepared input bag for that layer (see
#               filter_bag_for_layer.py -- relay needs no filtering,
#               ekf_global needs /tf's own map->odom entries dropped)
#     OUT_DIR = directory to record this run's output bag into
#
# relay layer:     plays /slam/pose, records /slam/pose_calibrated.
# ekf_global layer: plays /odometry/filtered + /slam/pose_calibrated +
#                   filtered /tf (odom->base_link only, map->odom dropped
#                   -- ekf_global needs the former for TF composition on
#                   its own map->odom output, and must never see the
#                   latter, which is its own output), records
#                   /ekf_global/odometry/filtered + /tf.
# slam layer:       plays /scan + /tf_static + filtered /tf (odom->base_link
#                   ONLY, via --tf-keep -- slam_toolbox needs that one edge
#                   and nothing else must be on the topic, since this
#                   layer's own Phase 2 job is partly to PROVE it adds
#                   nothing to /tf itself; see TF-silence check below).
#                   async_slam_toolbox_node is a LifecycleNode (Phase 0's
#                   4edcbbf fix taught slam.launch.py to drive it through
#                   configure->activate; this script does the same two
#                   transitions via the `ros2 lifecycle` CLI directly,
#                   since invoking the launch file itself would need a
#                   use_sim_time passthrough it doesn't currently expose --
#                   out of scope to add for this harness, see the Phase 2
#                   report). Records /slam/pose + /slam/map + /tf (the last
#                   one to prove TF silence, not because slam_toolbox is
#                   expected to produce anything there).
# semantic_layer:   plays /camera/detections_3d + /ekf_global/odometry/filtered
#                   + /tf_static (cam_to_base is a fully static chain --
#                   base_link->zed2_camera_link->...->zed2_left_camera_frame
#                   -- no dynamic /tf needed at all). Records
#                   /costmap/semantic_tracks.
# costmap_boundary: plays /slam/map + /ekf_global/odometry/filtered +
#                   /mpc/corridor_markers (present in the bag; only read
#                   when use_convex_polytope is true, which it isn't here
#                   -- included anyway for a faithful subscription set).
#                   No TF needed at all: its only tf_buffer.lookup_transform
#                   call (map->odom) is inside the convex-polytope path,
#                   which is off by default (confirmed: node's own
#                   declare_parameter default AND costmap.launch.py's own
#                   launch-arg default both say false) -- see the Phase 2
#                   report's Step 3 for the inflate_polytope confirmation
#                   this directly supports. Records /costmap/boundaries +
#                   /costmap/front_clearance.
# mpc:              plays the input bag built by filter_bag_for_layer.py for
#                   Phase 3 (/odometry/filtered, /costmap/boundaries,
#                   /perception/obstacles_2d, /mpc/hold, /scan, /tf,
#                   /tf_static, plus the injected /mpc/goal_drive -- see the
#                   Phase 3 report, Steps 1 and 3) into mpc_corr started by
#                   the PRODUCTION launch file (mpc_corr.launch.py) through
#                   mpc_replay.launch.py, which only adds use_sim_time.
#                   Records every MPC output topic, dumps the node's live
#                   parameters (params.yaml, reused by mpc_capture), and
#                   copies mpc_corr's own debug files (corridors_jsons/,
#                   mpc_log.csv) into OUT_DIR afterwards.
# mpc_capture:      same inputs, but mpc_corr run through
#                   mpc_capture_node.py (records every solve_mpc_step call
#                   for the function-level test) with MPC_PARAMS -- a
#                   params.yaml dumped by an `mpc` run -- so its configuration
#                   is the production one. Stopped with SIGINT so the capture
#                   file gets written.
# bt:               Phase 4. Plays the BT input bag (filter_bag_for_layer.py:
#                   /odometry/filtered, /ekf_global/odometry/filtered, /scan,
#                   /diagnostics/system_status, /camera/detections,
#                   /costmap/semantic_tracks, and /costmap/front_clearance
#                   renamed to /perception/front_distance as a documented
#                   stand-in -- see the Phase 4 report) into the PRODUCTION
#                   behavior_bringup.launch.py via bt_replay.launch.py
#                   (use_sim_time only). Two graph_stub_node.py stubs named
#                   mpc_corr and ackermann_to_vesc_node satisfy the
#                   start_mission preflight's node-existence check without
#                   running either (plus costmap_boundary_node and
#                   front_clearance_node, see the
#                   bt block below). bt_mission_driver.py loads and starts the
#                   mission at bag time BT_START_AT_BAG_SEC (default 36.5).
#                   Records every BT output; copies the mission report the
#                   run writes.
# ekf_cost_observer: Phase 4. Plays /odom, /odometry/filtered,
#                   /ekf_global/odometry/filtered, /slam/pose_calibrated and
#                   /diagnostics (robot_localization's own FrequencyStatus;
#                   the recorded ekf_cost_observer statuses removed by
#                   filter_bag_for_layer.py --diagnostics-drop-prefix) into
#                   ekf_cost_observer_node with its production defaults
#                   (localization.launch.py passes none) and NO use_sim_time:
#                   it measures wall-clock windows, as live. No EKF process
#                   runs, so its /proc CPU fields are unavailable by design.
#                   Records /diagnostics.
# mpc/mpc_capture environment:
#   MPC_CPU_AFFINITY  taskset core list (default 10,11 = mpc_corr.launch.py's
#                     own production default)
#   OSQP_TARGET       optional directory holding a different osqp, installed
#                     with `pip install --target` (never ~/.local); prepended
#                     to PYTHONPATH for the node only. The osqp version and
#                     file actually imported are written to osqp_version.txt.
#   PLAY_DELAY        seconds `ros2 bag play` waits after creating its
#                     publishers before the first message (default 3, mpc
#                     layers only) -- see the comment at the play call.
#   CLOCK_HZ          /clock publish rate during playback (default 1000 for
#                     the mpc layers). mpc_corr's 10 Hz timer runs on the sim
#                     clock and can only fire when a /clock message arrives,
#                     so at the 40 Hz default every tick is quantised to a
#                     25 ms grid; at 1 kHz the ticks land within 1 ms of
#                     their nominal times, as they do live. Same value on
#                     both distros. NOTE: this does NOT make
#                     /mpc/solver_status solve_dt_sec meaningful in a replay
#                     -- the node reads it from its own clock, and the
#                     /clock callback cannot run while control_loop holds the
#                     single-threaded executor, so sim time does not advance
#                     during a solve and solve_dt is 0.0. Solve timing comes
#                     from mpc_capture (perf_counter) and run_mpc_frozen.py.
# Both semantic_layer and costmap_boundary are launched via plain `ros2
# run` with ONLY use_sim_time overridden -- every other parameter's node-
# internal declare_parameter() default already matches costmap.launch.py's
# own declared launch-arg default exactly (checked line by line, not
# assumed), so no further -p overrides are needed for a faithful replay.
set -euo pipefail

LAYER=$1
BAG_DIR=$2
OUT_DIR=$3
DOMAIN_ID=${4:-77}

export ROS_DOMAIN_ID=$DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER 2>/dev/null || true

# F1TENTH_REPO overrides the repo root when this harness runs from a copy
# outside the repo (the Orin at e47e646 has no scripts/jazzy_parity/).
REPO_ROOT=${F1TENTH_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MPC_CPU_AFFINITY=${MPC_CPU_AFFINITY:-10,11}
EKF_GLOBAL_CONFIG="$REPO_ROOT/src/f1tenth_bringup/config/ekf_global.yaml"
SLAM_CONFIG="$REPO_ROOT/src/f1tenth_navigation/config/slam_toolbox_params.yaml"

# `ros2 run` spawns the actual node as a CHILD process rather than
# exec-replacing itself -- killing the PID bash's $! captures (the `ros2
# run` wrapper) orphans the real node instead of stopping it. Observed
# live: 3 consecutive relay runs recorded 2x/3x/4x the real message count,
# tracked to orphaned slam_pose_relay_node/ekf_node processes still
# subscribed/publishing on the (shared) ROS_DOMAIN_ID. Fix: match BOTH the
# wrapper's and the real node's cmdline with one broad substring pattern
# (both contain the plain node/executable name), SIGKILL (SIGTERM alone
# left live, CPU-active processes behind within the next command's
# observation window), and actively poll until no match remains instead of
# assuming a signal took effect immediately.
NODE_PATTERN_relay="slam_pose_relay_node"
NODE_PATTERN_ekf_global="ekf_node.*ekf_global_filter_node"
NODE_PATTERN_slam="async_slam_toolbox_node"
NODE_PATTERN_semantic_layer="semantic_layer_node"
NODE_PATTERN_costmap_boundary="costmap_boundary_node"
NODE_PATTERN_mpc="lib/mpc_controller/mpc_corr|mpc_replay\.launch\.py"
NODE_PATTERN_mpc_capture="mpc_capture_node\.py"
NODE_PATTERN_ekf_cost_observer="lib/f1tenth_diagnostics/ekf_cost_observer_node"
NODE_PATTERN_bt="behavior_executor_node|twist_to_ackermann_node|bt_replay\.launch\.py|graph_stub_node\.py|bt_mission_driver\.py"

case "$LAYER" in
  relay|ekf_global|slam|semantic_layer|costmap_boundary|mpc|mpc_capture|bt|ekf_cost_observer) ;;
  *)
    echo "unknown layer: $LAYER (expected relay, ekf_global, slam, semantic_layer, costmap_boundary, mpc or mpc_capture)" >&2
    exit 1
    ;;
esac

kill_and_wait() {
  local pattern=$1
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    pkill -9 -f "$pattern" 2>/dev/null || true
    sleep 0.3
    pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  done
  echo "WARNING: could not confirm all '$pattern' processes died" >&2
}

cleanup() {
  kill_and_wait "$NODE_PATTERN_relay"
  kill_and_wait "$NODE_PATTERN_ekf_global"
  kill_and_wait "$NODE_PATTERN_slam"
  kill_and_wait "$NODE_PATTERN_semantic_layer"
  kill_and_wait "$NODE_PATTERN_costmap_boundary"
  kill_and_wait "$NODE_PATTERN_mpc"
  kill_and_wait "$NODE_PATTERN_mpc_capture"
  kill_and_wait "$NODE_PATTERN_bt"
  kill_and_wait "$NODE_PATTERN_ekf_cost_observer"
  kill_and_wait "ros2 bag play"
  kill_and_wait "ros2 bag record"
}
trap cleanup EXIT

cleanup  # in case a previous run didn't exit cleanly

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
OUT_DIR=$(cd "$OUT_DIR" && pwd)  # absolute: the mpc layers cd into it
touch "$OUT_DIR/.run_start"

case "$LAYER" in
  relay)
    RECORD_TOPICS="/slam/pose_calibrated"
    ros2 run f1tenth_localization slam_pose_relay_node --ros-args \
      --params-file "$EKF_GLOBAL_CONFIG" -p use_sim_time:=true \
      > "$OUT_DIR/node.log" 2>&1 &
    ;;
  ekf_global)
    RECORD_TOPICS="/ekf_global/odometry/filtered /tf"
    ros2 run robot_localization ekf_node --ros-args \
      -r __name:=ekf_global_filter_node -r __ns:=/ekf_global \
      --params-file "$EKF_GLOBAL_CONFIG" -p use_sim_time:=true \
      > "$OUT_DIR/node.log" 2>&1 &
    ;;
  slam)
    RECORD_TOPICS="/slam/pose /slam/map /tf"
    # -r /map:=/slam/map etc. match slam.launch.py's own remappings exactly
    # -- without them the node publishes on its unremapped defaults
    # (/map, /pose), and this script would record nothing at all on
    # /slam/pose//slam/map despite the node working correctly. Found live:
    # `ros2 node info /slam_toolbox` showed /pose and /map as real
    # publishers the whole time this was missing.
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
      -r /map:=/slam/map -r /map_metadata:=/slam/map_metadata \
      -r /pose:=/slam/pose \
      --params-file "$SLAM_CONFIG" -p use_sim_time:=true \
      > "$OUT_DIR/node.log" 2>&1 &
    ;;
  semantic_layer)
    RECORD_TOPICS="/costmap/semantic_tracks /costmap/semantic_markers"
    ros2 run f1tenth_costmap semantic_layer_node --ros-args \
      -p use_sim_time:=true \
      > "$OUT_DIR/node.log" 2>&1 &
    ;;
  costmap_boundary)
    RECORD_TOPICS="/costmap/boundaries /costmap/front_clearance"
    ros2 run f1tenth_costmap costmap_boundary_node --ros-args \
      -p use_sim_time:=true \
      > "$OUT_DIR/node.log" 2>&1 &
    ;;
  ekf_cost_observer)
    RECORD_TOPICS="/diagnostics"
    ros2 run f1tenth_diagnostics ekf_cost_observer_node > "$OUT_DIR/node.log" 2>&1 &
    ;;
  bt)
    # Every topic the BT process publishes (Phase 4 report, Step 1), except
    # /bt/tree_visualization (PNG images, not compared).
    RECORD_TOPICS="/mpc/hold /mpc/goal_drive /mpc/goal_distance /mpc/goal_pose \
/mpc/goal_turn /mpc/goal_object /mpc/goal_object_end /mission/status \
/mission/move_outcome /behavior/tree_status /safety_stop /safety/event \
/test/mission_event /drive"
    python3 "$SCRIPT_DIR/graph_stub_node.py" mpc_corr > "$OUT_DIR/stub_mpc_corr.log" 2>&1 &
    python3 "$SCRIPT_DIR/graph_stub_node.py" ackermann_to_vesc_node > "$OUT_DIR/stub_vesc.log" 2>&1 &
    # preflight.py also requires a node by name for any front_clearance
    # stop_condition: costmap_boundary_node up to e47e646 (the Orin's Humble
    # reference), front_clearance_node from fix batch 1 on (the actual
    # /perception/front_distance producer since 63a6080). Both name-only
    # stubs run, so the same harness serves either code version; neither
    # publishes anything.
    python3 "$SCRIPT_DIR/graph_stub_node.py" costmap_boundary_node > "$OUT_DIR/stub_costmap.log" 2>&1 &
    python3 "$SCRIPT_DIR/graph_stub_node.py" front_clearance_node > "$OUT_DIR/stub_front_clearance.log" 2>&1 &
    (cd "$OUT_DIR" && exec ros2 launch "$SCRIPT_DIR/bt_replay.launch.py") > "$OUT_DIR/node.log" 2>&1 &
    ;;
  mpc|mpc_capture)
    # Every topic MPC_corr.py creates a publisher for (Phase 3 report,
    # Step 1), plus the injected goal so its delivery is on record.
    RECORD_TOPICS="/drive /mpc/drive_clamp /mpc/solver_status /mpc/status \
/mpc/corridor_markers /corridor /mpc/goal_reached /mpc/min_obstacle_distance \
/mpc/min_obstacle_distance_forward /mpc/predicted_min_clearance \
/mpc/object_status /mpc/wall_track /mpc/goal_drive"
    if [ -n "${OSQP_TARGET:-}" ]; then
      export PYTHONPATH="$OSQP_TARGET${PYTHONPATH:+:$PYTHONPATH}"
    fi
    python3 -c "import osqp, numpy, scipy, sys; print('osqp', osqp.__version__, osqp.__file__); print('numpy', numpy.__version__); print('scipy', scipy.__version__); print('python', sys.version)" \
      > "$OUT_DIR/osqp_version.txt" 2>&1
    # cwd = OUT_DIR so mpc_corr's relative model_log_path (mpc_log.csv)
    # lands with this run's outputs.
    if [ "$LAYER" = "mpc" ]; then
      (cd "$OUT_DIR" && exec ros2 launch "$SCRIPT_DIR/mpc_replay.launch.py" \
        cpu_affinity:="$MPC_CPU_AFFINITY") > "$OUT_DIR/node.log" 2>&1 &
    else
      : "${MPC_PARAMS:?mpc_capture needs MPC_PARAMS=<params.yaml dumped by an mpc run>}"
      (cd "$OUT_DIR" && exec taskset -c "$MPC_CPU_AFFINITY" python3 \
        "$SCRIPT_DIR/mpc_capture_node.py" --capture-out "$OUT_DIR/capture.npz" \
        --ros-args --params-file "$MPC_PARAMS" -p use_sim_time:=true) \
        > "$OUT_DIR/node.log" 2>&1 &
    fi
    ;;
esac

case "$LAYER" in
  relay) NODE_PATTERN=$NODE_PATTERN_relay ;;
  ekf_global) NODE_PATTERN=$NODE_PATTERN_ekf_global ;;
  slam) NODE_PATTERN=$NODE_PATTERN_slam ;;
  semantic_layer) NODE_PATTERN=$NODE_PATTERN_semantic_layer ;;
  costmap_boundary) NODE_PATTERN=$NODE_PATTERN_costmap_boundary ;;
  mpc) NODE_PATTERN=$NODE_PATTERN_mpc ;;
  mpc_capture) NODE_PATTERN=$NODE_PATTERN_mpc_capture ;;
  bt) NODE_PATTERN="behavior_executor_node" ;;
  ekf_cost_observer) NODE_PATTERN=$NODE_PATTERN_ekf_cost_observer ;;
esac

# sanity: the node must actually be up before we start recording. (Not an
# "exactly 1" check -- `ros2 run`'s wrapper process also matches this
# pattern alongside the real node it spawned, see this file's own header
# comment, so 2 is the normal running count; the real protection against
# the orphan-process contamination that comment describes is the
# kill-and-confirm cleanup() above/below, not this count.) Polled, not a
# single snapshot -- node startup time varies.
n_running=0
for _ in 1 2 3 4 5 6 7 8 9 10; do
  n_running=$(pgrep -cf "$NODE_PATTERN" || true)
  [ "$n_running" -ge 1 ] && break
  sleep 0.5
done
if [ "$n_running" -lt 1 ]; then
  echo "FATAL: $LAYER node never started" >&2
  exit 1
fi

if [ "$LAYER" = "slam" ]; then
  # Mirrors slam.launch.py's own configure->activate event pair (Phase 0's
  # 4edcbbf) via the `ros2 lifecycle` CLI instead of launch actions, since
  # driving the launch file itself here would need a use_sim_time
  # passthrough it doesn't currently expose (see this file's own header
  # comment). Before --clock starts moving (bag play hasn't started yet) --
  # confirmed live in Phase 0 that configure/activate complete fine without
  # sim time ticking; activate is what starts scan processing, so it must
  # finish before play begins or early scans would be missed.
  ros2 lifecycle set /slam_toolbox configure
  ros2 lifecycle set /slam_toolbox activate
  state=$(ros2 lifecycle get /slam_toolbox)
  echo "slam_toolbox lifecycle state after configure+activate: $state"
  case "$state" in
    active*) ;;
    *) echo "FATAL: slam_toolbox did not reach active (got: $state)" >&2; exit 1 ;;
  esac
fi

CLOCK_ARGS="--clock"
case "$LAYER" in
  mpc|mpc_capture)
    # mpc_corr imports numpy/scipy/osqp and builds its solver state before
    # it subscribes; wait for the node itself, not just its process.
    # (grep without -q: with pipefail, -q's early exit SIGPIPEs `ros2 node
    # list` and fails the pipeline even on a match once several nodes are up.)
    for _ in $(seq 1 60); do
      ros2 node list 2>/dev/null | grep -x "/mpc_corr" >/dev/null && break
      sleep 0.5
    done
    ros2 node list 2>/dev/null | grep -x "/mpc_corr" >/dev/null \
      || { echo "FATAL: /mpc_corr never appeared in the graph" >&2; exit 1; }
    if [ "$LAYER" = "mpc" ]; then
      ros2 param dump /mpc_corr > "$OUT_DIR/params.yaml"
    fi
    # --delay: the player creates every publisher, then waits before the
    # first message. Without it the injected /mpc/goal_drive -- a single
    # volatile message 0.5 s into the bag -- can go out before DDS discovery
    # has matched it to mpc_corr's subscription, and is then simply lost
    # (seen in an early run: goal never received, the MPC never left
    # "robot fermo in attesa"). The live BT publisher existed long before
    # it published, so this restores the original condition.
    CLOCK_ARGS="--clock ${CLOCK_HZ:-1000} --delay ${PLAY_DELAY:-3}"
    ;;
  bt)
    for _ in $(seq 1 60); do
      ros2 node list 2>/dev/null | grep -x "/behavior_executor_node" >/dev/null && break
      sleep 0.5
    done
    ros2 node list 2>/dev/null | grep -x "/behavior_executor_node" >/dev/null \
      || { echo "FATAL: /behavior_executor_node never appeared in the graph" >&2; exit 1; }
    ros2 param dump /behavior_executor_node > "$OUT_DIR/params.yaml" 2>/dev/null || true
    # BT_BAG_T0_NS (bag start, ns) can be passed explicitly -- the Orin
    # instructions do, because BagMetadata.starting_time's Python type is only
    # verified on Jazzy here.
    if [ -z "${BT_BAG_T0_NS:-}" ]; then
      BT_BAG_T0_NS=$(python3 -c "import sys; sys.path.insert(0, '$SCRIPT_DIR'); from bag_compat import open_reader; print(open_reader('$BAG_DIR').get_metadata().starting_time.nanoseconds)")
    fi
    echo "$BT_BAG_T0_NS" > "$OUT_DIR/bag_t0_ns.txt"
    python3 "$SCRIPT_DIR/bt_mission_driver.py" --repo "$REPO_ROOT" \
      --bag-start-ns "$BT_BAG_T0_NS" \
      --start-at-bag-sec "${BT_START_AT_BAG_SEC:-36.5}" --mission-dir "$OUT_DIR" \
      --log "$OUT_DIR/mission_driver.json" --ros-args -p use_sim_time:=true \
      > "$OUT_DIR/mission_driver.log" 2>&1 &
    CLOCK_ARGS="--clock ${CLOCK_HZ:-1000} --delay ${PLAY_DELAY:-3}"
    ;;
esac

ros2 bag record -o "$OUT_DIR/bag" $RECORD_TOPICS \
  --use-sim-time > "$OUT_DIR/record.log" 2>&1 &
RECORD_PID=$!
sleep 1  # let the recorder subscribe before playback starts

# mpc layers: the node's own CPU time across playback, from /proc/<pid>/stat
# (utime+stime, all threads) -- the measured load share used for the
# Phase 3 CPU-pinning proposal.
proc_cpu_ticks() { awk '{print $14 + $15}' "/proc/$1/stat" 2>/dev/null || echo 0; }
MPC_PID=""
case "$LAYER" in
  mpc) MPC_PID=$(pgrep -f "lib/mpc_controller/mpc_corr" | head -1 || true) ;;
  mpc_capture) MPC_PID=$(pgrep -f "python3 .*mpc_capture_node" | head -1 || true) ;;
esac
[ -n "$MPC_PID" ] && { CPU0=$(proc_cpu_ticks "$MPC_PID"); WALL0=$(date +%s.%N); }

set +e
ros2 bag play "$BAG_DIR" $CLOCK_ARGS --rate 1.0 > "$OUT_DIR/play.log" 2>&1
PLAY_STATUS=$?
set -e

if [ -n "$MPC_PID" ]; then
  CPU1=$(proc_cpu_ticks "$MPC_PID"); WALL1=$(date +%s.%N)
  python3 -c "import sys; c=(float(sys.argv[2])-float(sys.argv[1]))/float(sys.argv[5]); w=float(sys.argv[4])-float(sys.argv[3]); print('pid %s cpu_sec %.3f wall_sec %.3f cpu_percent_of_one_core %.1f' % (sys.argv[6], c, w, 100*c/w))" \
    "$CPU0" "$CPU1" "$WALL0" "$WALL1" "$(getconf CLK_TCK)" "$MPC_PID" > "$OUT_DIR/cpu_usage.txt"
fi

sleep 1  # drain in-flight messages before stopping the recorder

# GRACEFUL stop, by PID, not the pattern-based kill_and_wait the trap uses
# for the node processes: `ros2 bag record` only writes bag_0.db3's
# metadata.yaml on a clean shutdown (SIGTERM, its own signal handler) --
# SIGKILL-ing it (confirmed live: produced a bag directory with no
# metadata.yaml at all, "Could not find metadata in bag directory" from
# `ros2 bag info`) silently destroys the very output this script exists to
# capture. `ros2 bag record` is invoked directly here (not through `ros2
# run`), so it does not have the wrapper/orphan problem the node launches
# above do -- PID tracking is correct and sufficient for it.
kill "$RECORD_PID" 2>/dev/null || true
wait "$RECORD_PID" 2>/dev/null || true

case "$LAYER" in
  mpc_capture)
    # SIGINT, not the trap's SIGKILL: the capture file is written on shutdown.
    pkill -INT -f "$NODE_PATTERN_mpc_capture" 2>/dev/null || true
    for _ in $(seq 1 120); do
      pgrep -f "$NODE_PATTERN_mpc_capture" >/dev/null 2>&1 || break
      sleep 0.5
    done
    [ -f "$OUT_DIR/capture.npz" ] || { echo "FATAL: no capture.npz written" >&2; exit 1; }
    ;;
esac
case "$LAYER" in
  bt)
    mkdir -p "$OUT_DIR/mission_reports"
    find "$REPO_ROOT/src/f1tenth_behavior/mission_reports" -maxdepth 1 -type f \
      -newer "$OUT_DIR/.run_start" -exec cp -a {} "$OUT_DIR/mission_reports/" \; 2>/dev/null || true
    ;;
  mpc|mpc_capture)
    mkdir -p "$OUT_DIR/debug"
    # only what THIS run wrote (corridor snapshots are per-run timestamped
    # files that accumulate in that directory)
    find "$REPO_ROOT/src/f1tenth_control/corridors_jsons" -maxdepth 1 -type f \
      -newer "$OUT_DIR/.run_start" -exec cp -a {} "$OUT_DIR/debug/" \; 2>/dev/null || true
    ;;
esac

echo "layer=$LAYER play_status=$PLAY_STATUS out=$OUT_DIR/bag"
exit $PLAY_STATUS
