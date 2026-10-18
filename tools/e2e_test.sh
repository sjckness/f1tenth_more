#!/usr/bin/env bash
# e2e_test.sh -- end-to-end test of the stack against the Isaac sim on ONE machine (linus).
# Starts `scripts/sim/sim_isaac.sh all`, runs the checks of tools/sim/e2e_checks.py and the
# graph tools, samples CPU/RAM/VRAM, stops everything and writes a summary.
#
# Usage:
#   tools/e2e_test.sh [--scene base_room] [--out DIR] [--mission FILE] [--keep-running]
#     --out DIR        results (default output/jazzy_port/e2e)
#     --mission FILE   mission for the drive check (default f1tenth_behavior's
#                      missions/drive_test_straight_2m.json, 2.0 m straight at 0.4 m/s)
#     --keep-running   leave sim + stack up after the checks (default: stop and verify clean)
#     k:=v             passed to sim_isaac.sh all (supervisor_bringup), e.g. the Rust ports:
#                      guard:=shadow impl:=rust (output/rust/isaac_test.md)
# Steps (each one PASS/FAIL in DIR/summary.md, raw JSON in DIR/results.jsonl):
#   1 up         Isaac + stack up: /clock, then the stack's nodes and /odometry/filtered
#   2 graph      topic coverage from the running graph (tools/sim/topic_coverage.py)
#   3 hz         topic rates vs minimums
#   4 tf         TF tree: one parent per frame, one publisher per edge, odom->base_link =
#                EKF output, deployed sensor poses (tools/sim/tf_owners.py)
#   5 perception person detected (2D + 3D vs ground truth), LiDAR front wall vs ground truth
#                within 0.10 m
#   6 mission    load + start through the mission manager (--mission, default 2 m straight):
#                COMPLETE, ground-truth distance; the mission logger wrote a closed bag
#   7 reset      /reset_all (-> /sim/reset): car back at the start pose, EKFs restart at 0
#   8 person     mission go_to_person.json (perception + semantic tracks + MPC): COMPLETE,
#                ground-truth gap car front -> person; MPC CPU per tick; then /reset_all
#   9 perf       RTF (Isaac log), CPU/RAM (stack, Isaac), VRAM over the whole run
# Exit code: 0 if every step passed.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

SCENE=base_room
OUT="$REPO/output/jazzy_port/e2e"
MISSION=""
KEEP=false
STACK_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --scene) SCENE="${2:?}"; shift 2 ;;
    --out) OUT="${2:?}"; shift 2 ;;
    --mission) MISSION="${2:?}"; shift 2 ;;
    --keep-running) KEEP=true; shift ;;
    *:=*) STACK_ARGS+=("$1"); shift ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$OUT"
rm -f "$OUT"/results.jsonl "$OUT"/summary.md
log() { echo "[e2e $(date +%H:%M:%S)] $*" | tee -a "$OUT/e2e.log"; }
: > "$OUT/e2e.log"

# ---- environment: the same network as sim_isaac.sh --net local ---------------------------
set +e +u
# shellcheck disable=SC1090
[ -f "${F1TENTH_EXTRA_SETUP:-$HOME/dev_ws/jc_env.sh}" ] && source "${F1TENTH_EXTRA_SETUP:-$HOME/dev_ws/jc_env.sh}"
# shellcheck disable=SC1091
source "$REPO/scripts/env/jazzy.sh"
set -e
: "${F1TENTH_SIM_DISCOVERY_PORT:=11811}"; export F1TENTH_SIM_DISCOVERY_PORT   # sim_isaac.sh uses it too
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_E2E:-42}" ROS_DISCOVERY_SERVER="127.0.0.1:${F1TENTH_SIM_DISCOVERY_PORT}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/src/f1tenth_bringup/config/sim/dds/local.xml"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY
[ -n "$MISSION" ] || MISSION="$(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/drive_test_straight_2m.json"
SC() { ROS_SUPER_CLIENT=TRUE "$@"; }   # graph-wide observers

declare -A RESULT
record() {  # <step> <pass:0|1> <json file or ''>
  local step="$1" rc="$2" f="$3"
  if [ "$rc" = 0 ]; then RESULT[$step]=PASS; else RESULT[$step]=FAIL; fi
  log "step $step: ${RESULT[$step]}"
  if [ -n "$f" ] && [ -s "$f" ]; then
    python3 -c 'import json,sys; d=json.loads(open(sys.argv[1]).read().strip().splitlines()[-1]); d["step"]=sys.argv[2]; print(json.dumps(d))' \
      "$f" "$step" >> "$OUT/results.jsonl" 2>/dev/null || true
  fi
}

SIM_PID=""; PERF_PID=""
stop_all() {
  [ -n "$PERF_PID" ] && kill -TERM "$PERF_PID" 2>/dev/null && wait "$PERF_PID" 2>/dev/null
  PERF_PID=""
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    log "stopping sim_isaac.sh (SIGTERM -> its cleanup stops the stack, Isaac, the Discovery Server)"
    kill -TERM "$SIM_PID" 2>/dev/null || true
    for _ in $(seq 1 200); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 0.5; done   # cleanup: <= 30 + 40 + 2 s
  fi
  SIM_PID=""
}
# shellcheck disable=SC2329  # invoked by the EXIT trap
on_exit() {
  local rc=$?
  $KEEP || stop_all
  exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT TERM

# ---- 1 up --------------------------------------------------------------------------------
if docker ps -q -f name=^f1t_isaac$ | grep -q . || pgrep -f "component_supervisor_node" >/dev/null; then
  log "a sim or a stack is already running: stop it first (docker stop f1t_isaac; scripts/kill_ros2.py -y)"
  exit 2
fi
log "starting: scripts/sim/sim_isaac.sh all --scene $SCENE ${STACK_ARGS[*]} (log $OUT/sim_isaac.log)"
T0=$(date +%s)
# Own session: a Ctrl+C here reaches only this script, whose stop_all() then stops the sim
# once (SIGTERM -> sim_isaac.sh's cleanup), instead of both signals racing in its cleanup.
# --layout e2e_legacy: the base_room layout these checks were written for (car (1, 1.5), person
# (3, 3), no plant); base_room's default layout became the campaign room on 2026-10-16.
LAYOUT_ARGS=()
[ "$SCENE" = base_room ] && LAYOUT_ARGS=(--layout e2e_legacy)
F1T_SIM_LOG_DIR="$OUT" setsid "$REPO/scripts/sim/sim_isaac.sh" all --scene "$SCENE" "${LAYOUT_ARGS[@]}" \
  "${STACK_ARGS[@]}" > "$OUT/sim_isaac.log" 2>&1 < /dev/null &
SIM_PID=$!
python3 "$REPO/tools/sim/perf_sampler.py" --out "$OUT/perf.json" --period 5 &
PERF_PID=$!

up=1
for _ in $(seq 1 120); do   # Isaac 20-70 s to /clock, then the stack ~60 s
  kill -0 "$SIM_PID" 2>/dev/null || { log "sim_isaac.sh exited early"; break; }
  if grep -q "Started 'behavior'" "$OUT/sim_isaac.log" 2>/dev/null; then up=0; break; fi
  sleep 3
done
T_CLOCK=$(grep -m1 "/clock received" "$OUT/sim_isaac.log" 2>/dev/null \
          | sed 's/^\[sim_isaac \([0-9:]*\)\].*/\1/' || true)
if [ "$up" = 0 ]; then
  log "stack launched (/clock at $T_CLOCK); waiting for /odometry/filtered and detector warm-up"
  if SC timeout --foreground 120 ros2 topic echo --once --no-daemon /odometry/filtered \
        nav_msgs/msg/Odometry > /dev/null 2>&1 \
     && SC timeout --foreground 120 ros2 topic echo --once --no-daemon /camera/detections \
        vision_msgs/msg/Detection2DArray > /dev/null 2>&1; then
    up=0
  else
    up=1
  fi
  sleep 20   # SLAM's first map, both EKFs settled
fi
echo "{\"check\": \"up\", \"pass\": $([ "$up" = 0 ] && echo true || echo false), \"seconds_to_ready\": $(( $(date +%s) - T0 )), \"clock_at\": \"$T_CLOCK\"}" > "$OUT/up.json"
record 1_up "$up" "$OUT/up.json"
if [ "$up" != 0 ]; then
  log "not up: see $OUT/sim_isaac.log"; exit 1
fi

# ---- 2 graph ---------------------------------------------------------------------------
# /perception/d_wall/psi_correction: wall_distance_node is off by default since cpu-cleanup
# (686d911, enable_wall_distance); its subscribers stay.
ALLOW=(/sensors/core /imu /model/virtual_robot/odometry /test_campaign/logger_status /teleop
       /calibration_drive /cmd_vel_nav /set_pose /ekf_global/set_pose /mission/load_path
       /slam_toolbox/feedback /zed2/joint_states /perception/d_wall/psi_correction)
set +e
SC python3 "$REPO/tools/sim/topic_coverage.py" --spin 8 --allow-missing "${ALLOW[@]}" \
  --json "$OUT/topic_coverage.json" --md "$OUT/topic_coverage.md" > "$OUT/graph.json"
record 2_graph $? "$OUT/graph.json"

# ---- 3 hz ------------------------------------------------------------------------------
SC python3 "$REPO/tools/sim/e2e_checks.py" hz --window 15 > "$OUT/hz.json"
record 3_hz $? "$OUT/hz.json"

# ---- 4 tf ------------------------------------------------------------------------------
# Expected static poses: this branch's description (laser z 0.190, measured 2026-10-14, in
# campaign-final since the jazzy-rust merge; 0.20 before) and the deployed ZED mount.
SC python3 "$REPO/tools/sim/tf_owners.py" --listen 10 \
  --pose base_link:laser:0.305,0,0.190 \
         base_link:zed2_camera_link:0.2399,0,0.205,0,-0.01817399939,0,0.99983483923 \
         base_link:imu:0,0,0,0,0,0,1 \
  --json "$OUT/tf.json" --md "$OUT/tf.md" > "$OUT/tf_result.json"
record 4_tf $? "$OUT/tf_result.json"

# ---- 5 perception ------------------------------------------------------------------------
SC python3 "$REPO/tools/sim/e2e_checks.py" perception --window 15 > "$OUT/perception.json"
record 5_perception $? "$OUT/perception.json"

# ---- 6 mission ---------------------------------------------------------------------------
RUNS_DIR="$HOME/f1tenth_sim_archive"
before_runs=$(find "$RUNS_DIR" -mindepth 1 -maxdepth 2 -type d 2>/dev/null | sort)
SC python3 "$REPO/tools/sim/e2e_checks.py" mission --mission "$MISSION" --timeout 120 \
  --expect-distance 2.0 > "$OUT/mission.json"
rc=$?
sleep 15   # the logger closes the bag and moves the run to complete/
after_runs=$(find "$RUNS_DIR" -mindepth 1 -maxdepth 2 -type d 2>/dev/null | sort)
new_runs=$(comm -13 <(echo "$before_runs") <(echo "$after_runs") | grep -v '/incomplete$\|/complete$' || true)
echo "$new_runs" > "$OUT/mission_runs.txt"
run_ok=1
for d in $new_runs; do
  if find "$d" -name metadata.yaml | grep -q .; then run_ok=0; fi
done
python3 - "$OUT/mission.json" "$run_ok" "$new_runs" <<'PY' > "$OUT/mission_rec.json"
import json, sys
d = json.loads(open(sys.argv[1]).read().strip().splitlines()[-1])
d['logger_run_dirs'] = [x for x in sys.argv[3].split() if x]
d['logger_bag_closed'] = sys.argv[2] == '0'
d['pass'] = bool(d['pass'] and d['logger_bag_closed'])
print(json.dumps(d))
PY
[ "$rc" = 0 ] && [ "$run_ok" = 0 ] && rc=0 || rc=1
record 6_mission $rc "$OUT/mission_rec.json"

# ---- 7 reset -----------------------------------------------------------------------------
SC python3 "$REPO/tools/sim/e2e_checks.py" reset > "$OUT/reset.json"
record 7_reset $? "$OUT/reset.json"

# ---- 8 person ----------------------------------------------------------------------------
SC python3 "$REPO/tools/sim/e2e_checks.py" mission \
  --mission "$(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/go_to_person.json" \
  --timeout 120 --person-gap > "$OUT/person_mission.json"
record 8_person $? "$OUT/person_mission.json"
sleep 10
SC python3 "$REPO/tools/sim/e2e_checks.py" reset > "$OUT/reset2.json" || true

# ---- 9 perf ------------------------------------------------------------------------------
kill -TERM "$PERF_PID" 2>/dev/null; wait "$PERF_PID" 2>/dev/null; PERF_PID=""
python3 - "$OUT" <<'PY' > "$OUT/perf_result.json"
import json, re, sys
out = sys.argv[1]
rtf = [float(m.group(1)) for m in re.finditer(r'RTF ([0-9.]+) \(sim', open(f'{out}/isaac.log').read())]
perf = json.load(open(f'{out}/perf.json'))['summary']
last = lambda f: json.loads(open(f).read().strip().splitlines()[-1])
d = {'check': 'perf', 'rtf': {'n': len(rtf), 'min': min(rtf) if rtf else None,
                              'mean': round(sum(rtf) / len(rtf), 3) if rtf else None},
     'resources': perf, 'mpc_straight': last(f'{out}/mission.json').get('mpc'),
     'mpc_go_to_person': last(f'{out}/person_mission.json').get('mpc')}
d['pass'] = bool(rtf) and min(rtf) >= 0.9
print(json.dumps(d))
PY
record 9_perf $? "$OUT/perf_result.json"
set -e

# ---- summary -----------------------------------------------------------------------------
if ! $KEEP; then
  stop_all
  ros2 daemon stop > /dev/null 2>&1 || true   # started by the ros2 CLI calls above
  sleep 2
  left="$(docker ps -q -f name=^f1t_isaac$; pgrep -af 'component_supervisor|ros2 launch|fast-discovery-server|ros2-daemon' | grep -v pgrep || true)"
  if [ -n "$left" ]; then
    log "leftovers after stop: $left -- running scripts/kill_ros2.py -y"
    python3 "$REPO/scripts/kill_ros2.py" -y >> "$OUT/e2e.log" 2>&1 || true
    docker stop f1t_isaac >/dev/null 2>&1 || true
  else
    log "clean: no Isaac container, no ROS process left"
  fi
fi
{
  echo "# e2e $(date '+%Y-%m-%d %H:%M') scene $SCENE"
  echo
  echo "| step | result |"
  echo "|---|---|"
  for k in $(printf '%s\n' "${!RESULT[@]}" | sort); do echo "| $k | ${RESULT[$k]} |"; done
} > "$OUT/summary.md"
cat "$OUT/summary.md"
for k in "${!RESULT[@]}"; do [ "${RESULT[$k]}" = PASS ] || exit 1; done
exit 0
