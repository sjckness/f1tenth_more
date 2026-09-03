#!/bin/bash
# replay_bag_through_mpc.sh <mission_bag_dir> [out_bag_dir]
#
# Feeds a recorded mission's INPUTS back into a live mpc_corr and records what
# that node publishes, producing a bag that contains /mpc/corridor_markers and
# the predicted-horizon fields of /mpc/solver_status for a run recorded before
# either existed. That is the only way to get the reference-corridor funnel and
# the predicted horizon into f1tenth_logger's mission_replay_video output for an
# OLD bag -- new missions record both directly (mission_logger_node's topic
# list), so this is for backfill, not for normal use.
#
# WHAT IT IS AND IS NOT: the corridor and horizon it produces are computed by
# the real solver from the real recorded inputs, but they are an OPEN-LOOP
# re-solve -- this mpc_corr's /drive output never moved the car, so the pose
# stream is still the original run's. Timing also differs from the original
# tick alignment. Treat the geometry as "what the MPC computes for this
# situation", not as a byte-exact replay of what it computed live.
#
# SAFETY: runs on an isolated ROS_DOMAIN_ID with the discovery server disabled,
# so the /drive commands this mpc_corr publishes cannot reach the VESC (or any
# other node) even if the real stack happens to be up.
#
# SIDE EFFECT: mpc_corr truncates its own debug logs on startup
# (src/f1tenth_control/corridors_jsons/{corridor_debug.jsonl,
# mpc_control_compare.csv,mpc_odom_error.csv}). Back them up first if the ones
# from the last live run still matter.
set +u
SRC="${1:?usage: $0 <mission_bag_dir> [out_bag_dir]}"
OUT="${2:-${SRC%/}_mpcreplay}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$(mktemp -d)"

rm -rf "$OUT"
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID:-77}" ROS_LOCALHOST_ONLY=1
cd "$WS" || exit 1
source install/setup.bash

# Recorded: everything mission_replay_video.py reads, plus the two new streams.
# Played: the MPC's inputs only -- /drive and /mpc/solver_status are left out so
# the recorded copies come from the live node, not from the bag.
REC_TOPICS="/tf /tf_static /odom /odometry/filtered /ekf_global/odometry/filtered /slam/map \
/costmap/boundaries /costmap/front_clearance /costmap/semantic_markers /camera/detections_3d \
/perception/obstacles_2d /behavior/tree_status /safety_stop /mission/status /drive \
/mpc/solver_status /mpc/corridor_markers"
PLAY_TOPICS="/tf /tf_static /odom /odometry/filtered /ekf_global/odometry/filtered /slam/map \
/costmap/boundaries /costmap/front_clearance /costmap/semantic_markers /camera/detections_3d \
/perception/obstacles_2d /behavior/tree_status /safety_stop /mission/status \
/mpc/goal_turn /mpc/goal_distance /mpc/hold"

# The BEST_EFFORT publishers of the original stack: bag play re-offers that
# profile, and a default (RELIABLE) recorder subscription would silently
# capture nothing from them -- the same QoS trap mission_logger_node documents.
cat > "$LOGDIR/qos.yaml" <<QOS
/costmap/boundaries: {reliability: best_effort, durability: volatile, history: keep_last, depth: 100}
/costmap/front_clearance: {reliability: best_effort, durability: volatile, history: keep_last, depth: 100}
/odometry/filtered: {reliability: best_effort, durability: volatile, history: keep_last, depth: 100}
/ekf_global/odometry/filtered: {reliability: best_effort, durability: volatile, history: keep_last, depth: 100}
/odom: {reliability: best_effort, durability: volatile, history: keep_last, depth: 100}
QOS

ros2 run mpc_controller mpc_corr > "$LOGDIR/mpc.log" 2>&1 &
MPC=$!
sleep 5
ros2 bag record -o "$OUT" --storage sqlite3 \
    --qos-profile-overrides-path "$LOGDIR/qos.yaml" $REC_TOPICS > "$LOGDIR/record.log" 2>&1 &
REC=$!
sleep 4
ros2 bag play "$SRC" --rate "${REPLAY_RATE:-1.0}" --topics $PLAY_TOPICS > "$LOGDIR/play.log" 2>&1
sleep 3
kill -INT $REC 2>/dev/null; sleep 3; kill -TERM $REC 2>/dev/null
kill -INT $MPC 2>/dev/null; sleep 2; kill -TERM $MPC 2>/dev/null
wait 2>/dev/null
echo "replay bag: $OUT   (logs in $LOGDIR)"
