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
set -euo pipefail

LAYER=$1
BAG_DIR=$2
OUT_DIR=$3
DOMAIN_ID=${4:-77}

export ROS_DOMAIN_ID=$DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER 2>/dev/null || true

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
EKF_GLOBAL_CONFIG="$REPO_ROOT/src/f1tenth_bringup/config/ekf_global.yaml"

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

case "$LAYER" in
  relay|ekf_global) ;;
  *)
    echo "unknown layer: $LAYER (expected relay or ekf_global)" >&2
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
  kill_and_wait "ros2 bag play"
  kill_and_wait "ros2 bag record"
}
trap cleanup EXIT

cleanup  # in case a previous run didn't exit cleanly

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

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
esac

case "$LAYER" in
  relay) NODE_PATTERN=$NODE_PATTERN_relay ;;
  ekf_global) NODE_PATTERN=$NODE_PATTERN_ekf_global ;;
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

ros2 bag record -o "$OUT_DIR/bag" $RECORD_TOPICS \
  --use-sim-time > "$OUT_DIR/record.log" 2>&1 &
RECORD_PID=$!
sleep 1  # let the recorder subscribe before playback starts

ros2 bag play "$BAG_DIR" --clock --rate 1.0 > "$OUT_DIR/play.log" 2>&1
PLAY_STATUS=$?

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

echo "layer=$LAYER play_status=$PLAY_STATUS out=$OUT_DIR/bag"
exit $PLAY_STATUS
