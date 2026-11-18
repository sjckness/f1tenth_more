#!/usr/bin/env bash
# Replay one layer of the stack against a bag and record its outputs.
#
# Runs on: the Orin host with native Humble sourced, inside the harness
# container (Jazzy), linus. Same script, same input bag on both sides (build
# the input once with filter_bag_for_layer.py and prove it equal with
# bag_digest.py); compare the outputs with compare_runs.py and the MPC timing
# with mpc_timing.py + parity_gates.py mpc-timing (P9.4).
#
#   replay_localization.sh LAYER BAG_DIR OUT_DIR [DOMAIN_ID]
#
# Each layer is the side's PRODUCTION launch file under use_sim_time
# (layer_replay.launch.py), so each side replays its own configuration:
#   ekf_global   f1tenth_localization ekf_global.launch.py (global EKF + the
#                slam pose relay). Input: /odometry/filtered, /slam/pose and
#                /tf WITHOUT map->odom (filter_bag_for_layer.py --tf-drop map
#                odom: the layer's own output must not come back in).
#   slam         f1tenth_navigation slam.launch.py (it configures and activates
#                slam_toolbox itself). Input: /scan, /tf_static and /tf with
#                odom->base_link only (--tf-keep odom base_link).
#   costmap      f1tenth_costmap costmap.launch.py (semantic layer, boundary,
#                renderer). Input: what it subscribes to (/slam/map,
#                /ekf_global/odometry/filtered, detections, ...).
#   mpc          f1tenth_control mpc_corr.launch.py (MPC_CPU_AFFINITY, default
#                the production 10,11; 8-core linus: 6,7). Input: a car bag
#                plus the goal injected with make_mpc_goal.py +
#                filter_bag_for_layer.py --inject (mpc_corr never solves
#                without one). Also writes params.yaml (ros2 param dump), the
#                wall-clock receive times of /mpc/solver_status
#                (solver_status_rx.json, for mpc_timing.py wallrx:) and the
#                node's CPU share over the playback (cpu_usage.txt).
#   mpc_capture  mpc_capture_node.py (solve_mpc_step timed in the node) with
#                MPC_PARAMS=<params.yaml of an mpc run>: capture.npz for
#                mpc_timing.py capture:.
#
# What is played and what is recorded is derived from the graph, not kept in
# lists: once the layer's main node is up, stack_probe.py io lists every
# node's subscriptions and publications; the bag topics the layer subscribes
# to and does not publish itself are played (`ros2 bag play --topics`), and
# everything the layer publishes is recorded (sqlite3).
#
# Environment: PLAY_TOPICS (the topics to play, overriding the derived list),
# CLOCK_HZ (/clock rate, default 1000: at the 40 Hz default a
# 10 Hz sim-time timer is quantised to a 25 ms grid), PLAY_DELAY (s between
# the player's publishers and its first message, default 3: a single
# volatile goal published before discovery matched it would be lost),
# PLAY_RATE (1.0), LAYER_ARGS (extra launch arguments), MPC_CPU_AFFINITY,
# MPC_PARAMS.
# Isolation: ROS_DOMAIN_ID DOMAIN_ID (default 77), localhost discovery, no
# Discovery Server. Every process is started in its own process group and
# stopped by group (never `pkill -f`); OUT_DIR must be new or empty.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
[ $# -ge 3 ] || { echo "usage: replay_localization.sh LAYER BAG_DIR OUT_DIR [DOMAIN_ID]" >&2; exit 2; }
LAYER=$1
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
BAG_DIR=$(parity_abs "$2") || exit 2
need_empty_dir "$3"
OUT_DIR=$(parity_abs "$3")
isolated_dds "${4:-77}"
MPC_CPU_AFFINITY=${MPC_CPU_AFFINITY:-10,11}

# FIXED_PLAY / FIXED_RECORD: layers whose node creates its subscriptions only
# once /clock runs (robot_localization's ekf_node: "Waiting for clock to
# start..."), so the graph cannot tell what to play before playing. The input
# bag is prepared for them (see the header); the graph still adds what it shows.
FIXED_PLAY="" FIXED_RECORD=""
case "$LAYER" in
  ekf_global) PKG=f1tenth_localization; LF=ekf_global.launch.py; MAIN=/ekf_global/ekf_global_filter_node
    FIXED_PLAY="/odometry/filtered /slam/pose /tf /tf_static"
    FIXED_RECORD="/ekf_global/odometry/filtered /slam/pose_calibrated /tf" ;;
  slam)       PKG=f1tenth_navigation;   LF=slam.launch.py;       MAIN=/slam_toolbox
    FIXED_PLAY="/scan /tf /tf_static"
    FIXED_RECORD="/slam/pose /slam/map /tf" ;;
  costmap)    PKG=f1tenth_costmap;      LF=costmap.launch.py;    MAIN=/costmap_boundary_node ;;
  mpc)        PKG=f1tenth_control;      LF=mpc_corr.launch.py;   MAIN=/mpc_corr ;;
  mpc_capture) MAIN=/mpc_corr
    : "${MPC_PARAMS:?mpc_capture needs MPC_PARAMS=<params.yaml written by an mpc run>}"
    [ -s "$MPC_PARAMS" ] || { echo "MPC_PARAMS=$MPC_PARAMS is missing or empty" >&2; exit 2; } ;;
  *) echo "unknown layer: $LAYER (ekf_global, slam, costmap, mpc, mpc_capture)" >&2; exit 2 ;;
esac

LAYER_PG="" REC_PG="" RX_PG=""
# shellcheck disable=SC2329  # invoked by the EXIT trap
cleanup() {
  local g
  for g in $RX_PG $REC_PG $LAYER_PG; do pg_alive "$g" && pg_stop "$g" 10 5 > /dev/null; done
  RX_PG="" REC_PG="" LAYER_PG=""
  # `ros2 lifecycle` / `ros2 param` started this domain's ros2cli daemon.
  ros2 daemon stop > /dev/null 2>&1
}
trap cleanup EXIT
trap 'exit 130' INT TERM
touch "$OUT_DIR/.run_start"
parity_context "$OUT_DIR"
{
  echo "layer=$LAYER bag=$BAG_DIR domain=$ROS_DOMAIN_ID CLOCK_HZ=${CLOCK_HZ:-1000}"
  echo "PLAY_DELAY=${PLAY_DELAY:-3} PLAY_RATE=${PLAY_RATE:-1.0} LAYER_ARGS=${LAYER_ARGS:-}"
  echo "MPC_CPU_AFFINITY=$MPC_CPU_AFFINITY MPC_PARAMS=${MPC_PARAMS:-}"
} > "$OUT_DIR/condition.txt"

# cwd = OUT_DIR so relative-path outputs (mpc_corr's mpc_log.csv) land here.
cd "$OUT_DIR" || exit 2
case "$LAYER" in
  mpc_capture)
    python3 -c "import osqp, numpy, sys; print('osqp', osqp.__version__, osqp.__file__); print('numpy', numpy.__version__); print('python', sys.version)" \
      > "$OUT_DIR/osqp_version.txt" 2>&1
    # shellcheck disable=SC2086  # LAYER_ARGS: extra --ros-args words
    bg_start LAYER_PG "$OUT_DIR/node.log" taskset -c "$MPC_CPU_AFFINITY" python3 \
      "$HERE/mpc_capture_node.py" --capture-out "$OUT_DIR/capture.npz" \
      --ros-args -r __node:=mpc_corr --params-file "$MPC_PARAMS" -p use_sim_time:=true ${LAYER_ARGS:-}
    ;;
  *)
    [ "$LAYER" = mpc ] && python3 -c "import osqp, numpy, sys; print('osqp', osqp.__version__, osqp.__file__); print('numpy', numpy.__version__); print('python', sys.version)" \
      > "$OUT_DIR/osqp_version.txt" 2>&1
    extra=()
    [ "$LAYER" = mpc ] && extra=("cpu_affinity:=$MPC_CPU_AFFINITY")
    # shellcheck disable=SC2086,SC2206  # LAYER_ARGS: extra name:=value words
    extra+=(${LAYER_ARGS:-})
    bg_start LAYER_PG "$OUT_DIR/node.log" ros2 launch "$HERE/layer_replay.launch.py" \
      package:="$PKG" launch_file:="$LF" "${extra[@]}"
    ;;
esac

# The layer's main node in the graph, then what it subscribes / publishes.
python3 "$HERE/stack_probe.py" --out "$OUT_DIR/io.json" io --wait-node "$MAIN" --timeout 90
if ! python3 -c "import json, sys; sys.exit(0 if json.load(open(sys.argv[1]))['found'] else 1)" "$OUT_DIR/io.json"; then
  echo "FATAL: $MAIN never appeared in the graph (see $OUT_DIR/node.log)" >&2
  exit 1
fi
if [ "$LAYER" = slam ]; then
  for _ in $(seq 1 60); do
    state=$(timeout 10 ros2 lifecycle get /slam_toolbox 2>/dev/null)
    case "$state" in active*) break ;; esac
    sleep 0.5
  done
  echo "slam_toolbox lifecycle state: ${state:-unknown}" >> "$OUT_DIR/condition.txt"
  case "${state:-}" in active*) ;; *) echo "FATAL: slam_toolbox not active (${state:-?})" >&2; exit 1 ;; esac
fi
if [ "$LAYER" = mpc ]; then
  # Without a daemon (a fresh one does not know the node yet: "Node not
  # found"), retried until the dump is not empty; the options each distro has.
  DUMP_HELP=$(ros2 param dump --help 2>/dev/null)
  DUMP_ARGS=()
  grep -q -- --no-daemon <<< "$DUMP_HELP" && DUMP_ARGS+=(--no-daemon)
  grep -q -- --spin-time <<< "$DUMP_HELP" && DUMP_ARGS+=(--spin-time 5)
  grep -q -- --timeout <<< "$DUMP_HELP" && DUMP_ARGS+=(--timeout 10)
  for _ in 1 2 3; do
    ros2 param dump /mpc_corr "${DUMP_ARGS[@]}" > "$OUT_DIR/params.yaml" 2>> "$OUT_DIR/param_dump.log"
    [ -s "$OUT_DIR/params.yaml" ] && break
    sleep 2
  done
  [ -s "$OUT_DIR/params.yaml" ] || echo "WARNING: empty params.yaml (param_dump.log)" | tee -a "$OUT_DIR/condition.txt" >&2
fi

python3 - "$OUT_DIR/io.json" "$BAG_DIR" "$HERE" "$OUT_DIR" "$FIXED_PLAY" "$FIXED_RECORD" \
  "${PLAY_TOPICS:-}" <<'PY' || exit 1
import json, os, sys
io_path, bag, here, out, fixed_play, fixed_record, override = sys.argv[1:8]
sys.path.insert(0, here)
from bag_compat import topics_of
io = json.load(open(io_path))['nodes']
skip = {'/rosout', '/parameter_events', '/clock'}
subs = ({t for n in io.values() for t in n.get('subscribes', {})} | set(fixed_play.split())) - skip
pubs = ({t for n in io.values() for t in n.get('publishes', {})} | set(fixed_record.split())) - skip
bag_topics = set(topics_of(bag))
# /tf is never left out because the layer publishes on it too: it is one
# multiplexed topic, and the input bag drops the layer's own edges
# (filter_bag_for_layer.py --tf-drop / --tf-keep).
play = sorted((subs & bag_topics) - (pubs - {'/tf'}))
if override:
    play = sorted(set(override.split()) & bag_topics)
# Everything the layer publishes, plus the goal / hold inputs it was given
# (mpc_metrics.py takes the analysis window from them).
record = sorted(pubs | {t for t in play if t.startswith('/mpc/goal') or t == '/mpc/hold'})
open(os.path.join(out, 'play_topics.txt'), 'w').write('\n'.join(play) + '\n')
open(os.path.join(out, 'record_topics.txt'), 'w').write('\n'.join(record) + '\n')
unfed = sorted(subs - bag_topics - pubs)
open(os.path.join(out, 'unfed_subscriptions.txt'), 'w').write('\n'.join(unfed) + '\n')
print('play %d topics, record %d, %d subscriptions not in the bag' % (len(play), len(record),
                                                                     len(unfed)))
if not play:
    sys.exit('nothing in the bag that this layer subscribes to')
PY
mapfile -t PLAY_TOPICS < "$OUT_DIR/play_topics.txt"
mapfile -t RECORD_TOPICS < "$OUT_DIR/record_topics.txt"

REC_ARGS=(-o "$OUT_DIR/bag" -s sqlite3)
ros2 bag record --help 2>/dev/null | grep -q -- '--use-sim-time' && REC_ARGS+=(--use-sim-time)
bg_start REC_PG "$OUT_DIR/record.log" ros2 bag record "${REC_ARGS[@]}" "${RECORD_TOPICS[@]}"
sleep 2  # let the recorder subscribe before playback starts

DUR=$(python3 -c "import sys, yaml; print(int(yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9 / float(sys.argv[2])) + 1)" \
      "$BAG_DIR/metadata.yaml" "${PLAY_RATE:-1.0}")
case "$LAYER" in
  mpc|mpc_capture)
    bg_start RX_PG "$OUT_DIR/solver_status_rx.log" python3 "$HERE/stack_probe.py" \
      --out "$OUT_DIR/solver_status_rx.json" --wait-graph 5 stamps --topic /mpc/solver_status \
      --duration $((DUR + ${PLAY_DELAY:-3} + 3)) ;;
esac

# The layer node's CPU share over the playback, from /proc (utime + stime).
MAIN_PID=$(python3 - "$LAYER_PG" "$MAIN" "$HERE" <<'PY'
import sys
sys.path.insert(0, sys.argv[3])
import proc_audit
pg, main = int(sys.argv[1]), sys.argv[2]
for p in proc_audit.read_proc():
    if p['pgid'] == pg and proc_audit.node_name(p['args']) == main:
        print(p['pid'])
        break
PY
)
ticks() { awk '{print $14 + $15}' "/proc/$1/stat" 2>/dev/null || echo 0; }
[ -n "$MAIN_PID" ] && { CPU0=$(ticks "$MAIN_PID"); WALL0=$(date +%s.%N); }

PLAY_HELP=$(ros2 bag play --help 2>/dev/null)
PLAY_ARGS=(--topics "${PLAY_TOPICS[@]}" --clock "${CLOCK_HZ:-1000}" --rate "${PLAY_RATE:-1.0}")
if grep -q -- '--delay' <<< "$PLAY_HELP"; then
  PLAY_ARGS+=(--delay "${PLAY_DELAY:-3}")
else
  echo "WARNING: this ros2 bag play has no --delay: an early single goal may be lost" \
    | tee -a "$OUT_DIR/condition.txt" >&2
fi
grep -q -- '--disable-keyboard-controls' <<< "$PLAY_HELP" && PLAY_ARGS+=(--disable-keyboard-controls)
echo "ros2 bag play $BAG_DIR ${PLAY_ARGS[*]}" > "$OUT_DIR/play_cmd.txt"
ros2 bag play "$BAG_DIR" "${PLAY_ARGS[@]}" > "$OUT_DIR/play.log" 2>&1 < /dev/null
PLAY_STATUS=$?

if [ -n "$MAIN_PID" ]; then
  CPU1=$(ticks "$MAIN_PID")
  WALL1=$(date +%s.%N)
  awk -v c0="$CPU0" -v c1="$CPU1" -v w0="$WALL0" -v w1="$WALL1" -v hz="$(getconf CLK_TCK)" -v p="$MAIN_PID" \
    'BEGIN { c = (c1 - c0) / hz; w = w1 - w0; printf "pid %s cpu_sec %.3f wall_sec %.3f cpu_percent_of_one_core %.1f\n", p, c, w, 100 * c / w }' \
    > "$OUT_DIR/cpu_usage.txt"
fi
sleep 1  # drain in-flight messages before stopping the recorder
# The recorder writes metadata.yaml only on a clean (signal-handled) stop.
pg_stop "$REC_PG" 15 5 > "$OUT_DIR/record_stop.txt"
REC_PG=""
[ -n "$RX_PG" ] && { pid_wait_gone "$RX_PG" 20 || pg_stop "$RX_PG" 5 3 > /dev/null; RX_PG=""; }
# SIGINT: mpc_capture_node writes capture.npz on shutdown.
pg_stop "$LAYER_PG" 30 10 > "$OUT_DIR/layer_stop.txt"
LAYER_PG=""
if [ "$LAYER" = mpc_capture ] && [ ! -f "$OUT_DIR/capture.npz" ]; then
  echo "FATAL: no capture.npz written" >&2; exit 1
fi
case "$LAYER" in
  mpc|mpc_capture)
    # mpc_corr's corridor snapshots go to the source tree it finds above its
    # install (MPC_corr.py _resolve_debug_output_path): copy only this run's.
    prefix=$(ros2 pkg prefix mpc_controller 2>/dev/null)
    mkdir -p "$OUT_DIR/debug"
    for d in "$prefix/../../src/f1tenth_control/corridors_jsons" "$prefix/../src/f1tenth_control/corridors_jsons"; do
      [ -d "$d" ] && find "$d" -maxdepth 1 -type f -newer "$OUT_DIR/.run_start" -exec cp -a {} "$OUT_DIR/debug/" \;
    done ;;
esac
echo "layer=$LAYER play_status=$PLAY_STATUS out=$OUT_DIR/bag"
exit $PLAY_STATUS
