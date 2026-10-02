#!/usr/bin/env bash
# Phase 4, Step 2: mission_logger_node end-to-end on this distro, with a short
# synthetic mission on an isolated domain. Prints a PASS/FAIL line per check.
#
#   logger_check.sh WORK_DIR SRC_BAG [STORAGE_ID] [PLAY_SEC]
#
# WORK_DIR gets runs/ (the logger's runs_dir), the lock file and the logs.
# SRC_BAG is played for PLAY_SEC (default 12) seconds while the mission is
# RUNNING, minus /mission/status and /mission/move_outcome, which the
# synthetic driver owns. STORAGE_ID is passed as the node's storage_id
# parameter (default: unset, i.e. the node's own default, mcap).
#
# Checks:
#   1. single-instance lock: a second logger refuses to start (exit 1) and
#      names the lock.
#   2. RUNNING opens active/<run_id>/ with a bag, a start manifest and the
#      params snapshot.
#   3. COMPLETE moves the run whole to complete/<run_id>/ (active/ left empty).
#   4. finalize: manifest has outcome/end_time/bag_bytes/bag_sha256; the
#      recomputed per-file sha256 matches; params snapshot is byte-identical to
#      the stack_params.yaml in effect; extract.parquet written.
#   5. the bag's storage format (mcap/sqlite3) is what was requested.
#   6. SIGINT shutdown releases the lock.
#
# Isolation: ROS_DOMAIN_ID 79, localhost discovery. f1tenth-archive.service
# (the node's post-run sync) must not exist on this machine -- checked first.
set -uo pipefail
WORK=$(mkdir -p "$1" && cd "$1" && pwd)
SRC=$2
STORAGE=${3:-}
PLAY_SEC=${4:-12}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER 2>/dev/null || true
export MISSION_LOGGER_LOCK="$WORK/mission_logger.lock"
RUNS="$WORK/runs"
rm -rf "$RUNS" "$MISSION_LOGGER_LOCK"
mkdir -p "$RUNS"
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILED=1; }
FAILED=0

if systemctl --user cat f1tenth-archive.service >/dev/null 2>&1; then
  echo "ABORT: f1tenth-archive.service exists here; the logger would sync runs off-machine."
  exit 2
fi

PARAMS=(-p "runs_dir:=$RUNS")
[ -n "$STORAGE" ] && PARAMS+=(-p "storage_id:=$STORAGE")
NODE_PAT="lib/f1tenth_logger/mission_logger_node"

ros2 run f1tenth_logger mission_logger_node --ros-args "${PARAMS[@]}" > "$WORK/logger1.log" 2>&1 &
for _ in $(seq 1 40); do [ -f "$MISSION_LOGGER_LOCK" ] && break; sleep 0.25; done
[ -f "$MISSION_LOGGER_LOCK" ] && pass "lock file created ($(cat "$MISSION_LOGGER_LOCK"))" || fail "no lock file"
for _ in $(seq 1 40); do grep -q "mission_logger_node up" "$WORK/logger1.log" && break; sleep 0.25; done

# 1. second instance
timeout 20 ros2 run f1tenth_logger mission_logger_node --ros-args "${PARAMS[@]}" > "$WORK/logger2.log" 2>&1
rc2=$?
if [ $rc2 -eq 1 ] && grep -q "refusing to start a second one" "$WORK/logger2.log"; then
  pass "second instance rejected (exit $rc2): $(grep -o 'another mission logger.*pid [0-9]*)' "$WORK/logger2.log")"
else
  fail "second instance not rejected (exit $rc2)"
fi

# 2. RUNNING
MISSION_JSON="$WORK/llm_2b356aac445f.json"
echo '{}' > "$MISSION_JSON"
python3 "$SCRIPT_DIR/publish_mission_status.py" RUNNING "$MISSION_JSON" 2.0 > "$WORK/status_running.log" 2>&1
sleep 1
ACTIVE_RUN=$(ls "$RUNS/active" 2>/dev/null | head -1)
if [ -n "$ACTIVE_RUN" ] && [ -f "$RUNS/active/$ACTIVE_RUN/$ACTIVE_RUN.manifest.json" ] \
   && [ -f "$RUNS/active/$ACTIVE_RUN/$ACTIVE_RUN.params.yaml" ]; then
  pass "RUNNING -> active/$ACTIVE_RUN with start manifest + params snapshot"
else
  fail "no active run after RUNNING"
fi

ros2 bag play "$SRC" --playback-duration "$PLAY_SEC" \
  --exclude-topics /mission/status /mission/move_outcome > "$WORK/play.log" 2>&1

# 3. COMPLETE
python3 "$SCRIPT_DIR/publish_mission_status.py" COMPLETE "$MISSION_JSON" 2.0 > "$WORK/status_complete.log" 2>&1
for _ in $(seq 1 240); do
  grep -q "extract written\|finalize step \"parquet extract\" failed" "$WORK/logger1.log" && break
  sleep 0.5
done
sleep 2
RUN_DIR="$RUNS/complete/$ACTIVE_RUN"
if [ -d "$RUN_DIR" ] && [ -z "$(ls -A "$RUNS/active")" ]; then
  pass "COMPLETE -> complete/$ACTIVE_RUN, active/ empty"
else
  fail "run not moved to complete/ (active: $(ls "$RUNS/active" 2>/dev/null))"
fi

# 4./5. finalize, checksums, params, extract, storage
python3 - "$RUN_DIR" "$ACTIVE_RUN" "${STORAGE:-mcap}" <<'PY'
import hashlib, json, os, sys
from ament_index_python.packages import get_package_share_directory
run_dir, run_id, want = sys.argv[1], sys.argv[2], sys.argv[3]
ok = True
def res(cond, text):
    global ok
    print(('PASS  ' if cond else 'FAIL  ') + text)
    ok = ok and cond
m = json.load(open(os.path.join(run_dir, run_id + '.manifest.json')))
res(m.get('outcome') == 'COMPLETE' and m.get('end_time'), 'manifest outcome=%s end_time=%s' % (m.get('outcome'), m.get('end_time')))
bag = os.path.join(run_dir, 'bag')
files = sorted(f for f in os.listdir(bag))
recomputed, total = {}, 0
for f in files:
    p = os.path.join(bag, f)
    recomputed[f] = hashlib.sha256(open(p, 'rb').read()).hexdigest()
    total += os.path.getsize(p)
stored = m.get('bag_sha256')
res(isinstance(stored, dict) and stored == recomputed,
    'bag_sha256 recomputed and equal for %d files (%s)' % (len(recomputed), ', '.join(files)))
res(m.get('bag_bytes') == total, 'bag_bytes %s == %d' % (m.get('bag_bytes'), total))
sp = os.path.realpath(os.path.join(get_package_share_directory('f1tenth_params'), 'config', 'stack_params.yaml'))
res(open(sp, 'rb').read() == open(os.path.join(run_dir, run_id + '.params.yaml'), 'rb').read(),
    'params snapshot byte-identical to %s' % sp)
res(m.get('storage_id') == want and any(f.endswith('.' + ('mcap' if want == 'mcap' else 'db3')) for f in files),
    'storage %s as requested (%s)' % (m.get('storage_id'), want))
ex = os.path.join(run_dir, run_id + '.extract.parquet')
res(os.path.isfile(ex), 'extract.parquet written (%s)' % ('%d bytes' % os.path.getsize(ex) if os.path.isfile(ex) else 'missing'))
sys.exit(0 if ok else 1)
PY
[ $? -eq 0 ] || FAILED=1
grep -h "finalize step\|ERROR\|Traceback" "$WORK/logger1.log" | head -5

# 6. shutdown
pkill -INT -f "$NODE_PAT"
for _ in $(seq 1 60); do pgrep -f "$NODE_PAT" >/dev/null || break; sleep 0.5; done
pgrep -f "$NODE_PAT" >/dev/null && { fail "logger did not exit on SIGINT"; pkill -9 -f "$NODE_PAT"; }
[ -f "$MISSION_LOGGER_LOCK" ] && fail "lock not released on shutdown" || pass "lock released on shutdown"
echo "RESULT $([ $FAILED -eq 0 ] && echo PASS || echo FAIL)  run=$ACTIVE_RUN"
