#!/usr/bin/env bash
# run_replay.sh -- the whole stack against a recorded bag, measured by probe.py
# (perf baseline, level B). Measure only: it starts the car's own bringup and
# changes no file of it.
#
#   scripts/perf/run_replay.sh <bag_dir> <out_dir> [options]
#
#   --runs N            total runs; the first is a warm-up and is not summarised (default 3)
#   --no-warmup         every run is measured (a warm-up was done separately)
#   --play-s S          seconds of bag per run = the measured window (default 60)
#   --start-offset S    where in the bag each run starts (default 15)
#   --settle-s S        wall seconds between starting the stack and the bag (default 60)
#   --camera-hz HZ      rate the camera feeder republishes image and depth at (default 30,
#                       zed2_perception.yaml's pub_frame_rate); see replay_feeder.py
#   --goal-distance M   one /mpc/goal_distance sent 8 s into the bag, so the MPC solves
#                       (default 8; 0 = none, the MPC idles)
#   --clock-hz HZ       /clock rate of `ros2 bag play --clock` (default 50)
#   --domain N, --port P   ROS_DOMAIN_ID and Discovery Server port of this run
#                       (default 77 and 11911: never the car's 0/11811 nor the sim's 42)
#   --min-mem-mb MB     refuse to start with less MemAvailable than this (default 3000)
#                       A run is also aborted (stack stopped) as soon as MemAvailable falls
#                       under 500 MB or free swap under 250 MB: this machine is shared, and
#                       an OOM kill would take the largest process on it, not this stack.
#   --dry-run           print the commands, start nothing
#
# What runs: `ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac
# use_sim_time:=true` -- the car's launch with the device drivers off, as in the
# Isaac sim -- with enable_mission_logger:=false and enable_intelligence:=false, on
# its own loopback-only DDS (config/sim/dds/local.xml), plus:
#   ros2 bag play --clock   /scan, /odom, both camera_info, the 5 Hz depth and clean JPEG
#   replay_feeder.py        image + depth back at the camera rate (and the MPC goal)
#   probe.py                the one measuring process
# Not in the bag, so not in the measurement: the IMU (the EKFs fuse wheel odometry
# only), the VESC, the ZED wrapper, urg_node, and any mission.
#
# The stack is stopped the way Ctrl-C stops it (SIGINT to the launch); whatever is
# left of ITS process tree after 40 s is killed by pid. Nothing else on the machine
# is signalled: no pkill, no scripts/kill_ros2.py.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
log() { echo "[run_replay $(date +%H:%M:%S)] $*"; }
die() { echo "[run_replay] ERROR: $*" >&2; exit 1; }

RUNS=3; PLAY_S=60; START_OFFSET=15; SETTLE_S=60; CAMERA_HZ=30; GOAL=8; CLOCK_HZ=50
DOMAIN=77; PORT=11911; MIN_MEM_MB=3000; DRY=false; WARMUP=true
POS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --play-s) PLAY_S="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --settle-s) SETTLE_S="$2"; shift 2 ;;
    --camera-hz) CAMERA_HZ="$2"; shift 2 ;;
    --goal-distance) GOAL="$2"; shift 2 ;;
    --clock-hz) CLOCK_HZ="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --min-mem-mb) MIN_MEM_MB="$2"; shift 2 ;;
    --dry-run) DRY=true; shift ;;
    --no-warmup) WARMUP=false; shift ;;
    *) POS+=("$1"); shift ;;
  esac
done
[ ${#POS[@]} -eq 2 ] || { usage; exit 2; }
BAG="$(cd "${POS[0]}" && pwd)"; OUT="${POS[1]}"
[ -f "$BAG/metadata.yaml" ] || die "$BAG is not a rosbag2 directory"
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"

IMAGE_TOPIC="${BAG_IMAGE_TOPIC:-/camera/debug_image/clean/compressed}"
DEPTH_TOPIC="${BAG_DEPTH_TOPIC:-/zed2/zed_node/depth/depth_registered_5hz}"
for t in /scan /odom "$IMAGE_TOPIC" "$DEPTH_TOPIC"; do
  grep -q "name: $t\$" "$BAG/metadata.yaml" || die "the bag has no $t"
done

# shellcheck disable=SC1091
source "$HERE/perf_env.sh"
PROFILE="$REPO/src/f1tenth_bringup/config/sim/dds/local.xml"
OVERLAY="$REPO/src/f1tenth_bringup/config/sim/hosts/linus.yaml"
[ "$(nproc)" -ge 12 ] && OVERLAY=""
export ROS_DOMAIN_ID="$DOMAIN" RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="127.0.0.1:$PORT" FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_SUPER_CLIENT

descendants() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do descendants "$c"; echo "$c"; done; }

STACK_PID=""; FEED_PID=""; PLAY_PID=""; GUARD_PID=""
mem_guard() {  # <launch pid> <run dir>: SIGINT the launch when the machine runs out of memory
  local avail swap
  while kill -0 "$1" 2>/dev/null; do
    avail=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
    swap=$(awk '/SwapFree/ {print int($2/1024)}' /proc/meminfo)
    if [ "$avail" -lt 500 ] || { [ "$(awk '/SwapTotal/ {print $2}' /proc/meminfo)" -gt 0 ] && [ "$swap" -lt 250 ]; }; then
      echo "MemAvailable ${avail} MB, SwapFree ${swap} MB: aborting the run" > "$2/ABORTED_LOW_MEMORY"
      kill -INT "$1" 2>/dev/null || true
      return
    fi
    sleep 2
  done
}
stop_stack() {
  [ -n "$GUARD_PID" ] && kill "$GUARD_PID" 2>/dev/null || true
  GUARD_PID=""
  [ -n "$FEED_PID" ] && kill -INT "$FEED_PID" 2>/dev/null || true
  [ -n "$PLAY_PID" ] && kill -INT "$PLAY_PID" 2>/dev/null || true
  FEED_PID=""; PLAY_PID=""
  [ -n "$STACK_PID" ] || return 0
  local tree; tree="$(descendants "$STACK_PID")"
  if kill -0 "$STACK_PID" 2>/dev/null; then
    log "stopping the stack (SIGINT to the launch, up to 40 s)"
    kill -INT "$STACK_PID" 2>/dev/null || true
    for _ in $(seq 1 80); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$STACK_PID" 2>/dev/null && kill -TERM "$STACK_PID" 2>/dev/null || true
  fi
  sleep 2
  local left=() p
  for p in $tree; do kill -0 "$p" 2>/dev/null && left+=("$p"); done
  if [ ${#left[@]} -gt 0 ]; then
    log "${#left[@]} process(es) of the stack survived: SIGTERM, then SIGKILL, by pid"
    kill -TERM "${left[@]}" 2>/dev/null || true; sleep 3
    for p in "${left[@]}"; do kill -0 "$p" 2>/dev/null && kill -KILL "$p" 2>/dev/null || true; done
  fi
  STACK_PID=""
}
trap 'stop_stack' EXIT
trap 'exit 130' INT TERM

LAUNCH=(ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac use_sim_time:=true
  enable_mission_logger:=false enable_intelligence:=false
  discovery_server_address:=127.0.0.1 discovery_server_port:="$PORT" fastdds_profile:="$PROFILE")
[ -n "$OVERLAY" ] && LAUNCH+=(sim_overlay_extra:="$OVERLAY")
PLAY=(ros2 bag play "$BAG" --clock "$CLOCK_HZ" --start-offset "$START_OFFSET"
  --playback-duration "$PLAY_S" --disable-keyboard-controls
  --topics /scan /odom /camera/camera_info /zed2/zed_node/depth/camera_info "$IMAGE_TOPIC" "$DEPTH_TOPIC"
  --remap "$IMAGE_TOPIC:=/perf/image_clean/compressed" "$DEPTH_TOPIC:=/perf/depth_in")

{
  echo "bag: $BAG"; echo "repo: $REPO @ $(git -C "$REPO" rev-parse --short HEAD) ($(git -C "$REPO" rev-parse --abbrev-ref HEAD))"
  echo "domain $DOMAIN, discovery server 127.0.0.1:$PORT, profile $PROFILE"
  echo "runs $RUNS (warm-up first: $WARMUP), play ${PLAY_S}s from ${START_OFFSET}s, settle ${SETTLE_S}s, camera ${CAMERA_HZ} Hz, clock ${CLOCK_HZ} Hz, goal ${GOAL} m"
  echo "launch: ${LAUNCH[*]}"; echo "play: ${PLAY[*]}"
} | tee "$OUT/replay_setup.txt"
$DRY && exit 0

for r in $(seq 0 $((RUNS - 1))); do
  label="run$r"; [ "$r" -eq 0 ] && $WARMUP && label="warmup"
  $WARMUP || label="run$((r + 1))"
  RUN="$OUT/$label"; mkdir -p "$RUN"
  [ -e /tmp/component_supervisor.lock ] && die "/tmp/component_supervisor.lock exists: another supervisor is running on this machine"
  avail=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
  [ "$avail" -ge "$MIN_MEM_MB" ] || die "only ${avail} MB available (< ${MIN_MEM_MB}): not starting a stack on a machine this full"
  log "$label: starting the stack (MemAvailable ${avail} MB, load $(cut -d' ' -f1 /proc/loadavg))"
  env --default-signal=INT "${LAUNCH[@]}" log_dir:="$RUN/supervisor_log" > "$RUN/launch.log" 2>&1 &
  STACK_PID=$!
  mem_guard "$STACK_PID" "$RUN" &
  GUARD_PID=$!
  sleep "$SETTLE_S"
  kill -0 "$STACK_PID" 2>/dev/null || die "the launch exited during the settle time: see $RUN/launch.log"
  ps -o pid,ppid,pcpu,rss,etimes,args --no-headers -p "$(descendants "$STACK_PID" | paste -sd, -)" > "$RUN/ps_before.txt" 2>/dev/null || true
  log "$label: $(wc -l < "$RUN/ps_before.txt") processes up; starting feeder, probe and bag"
  python3 "$HERE/replay_feeder.py" --rate "$CAMERA_HZ" --goal-distance "$GOAL" > "$RUN/feeder.log" 2>&1 &
  FEED_PID=$!
  sleep 2
  "${PLAY[@]}" > "$RUN/bag_play.log" 2>&1 &
  PLAY_PID=$!
  python3 "$HERE/probe.py" run --root-pid "$STACK_PID" --harness-pid "$FEED_PID" --harness-pid "$PLAY_PID" \
    --duration "$PLAY_S" --skip 8 --out "$OUT" --label "$label" > "$RUN/probe.log" 2>&1 || log "probe failed: see $RUN/probe.log"
  wait "$PLAY_PID" 2>/dev/null || true; PLAY_PID=""
  [ -e "$RUN/ABORTED_LOW_MEMORY" ] && { stop_stack; die "$label: $(cat "$RUN/ABORTED_LOW_MEMORY")"; }
  ps -o pid,ppid,pcpu,rss,etimes,args --no-headers -p "$(descendants "$STACK_PID" | paste -sd, -)" > "$RUN/ps_after.txt" 2>/dev/null || true
  stop_stack
  sleep 5
done
python3 "$HERE/probe.py" summarise "$OUT" > "$OUT/summarise.log" 2>&1 || log "summarise failed: see $OUT/summarise.log"
log "done: $OUT/nodes.csv, topics.csv, summary.json"
