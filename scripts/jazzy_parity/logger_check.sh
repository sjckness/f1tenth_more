#!/usr/bin/env bash
# mission_logger_node end to end on this side, with a synthetic mission on an
# isolated domain. Prints one PASS/FAIL line per check and a RESULT line.
#
# Runs on: the Orin host with native Humble sourced, inside the harness
# container (Jazzy), linus. Run on both sides of P9 with the same SRC_BAG;
# compare_extracts.py then compares the two extract.parquet files (same
# content expected: same bag, same extractor).
#
#   logger_check.sh WORK_DIR SRC_BAG [STORAGE_ID] [PLAY_SEC]
#
# WORK_DIR (new or empty) gets runs/ (the logger's runs_dir), the lock file,
# the logs. SRC_BAG is played for PLAY_SEC (default 12) seconds while the
# mission is RUNNING, minus /mission/status and /mission/move_outcome, which
# the synthetic driver (publish_mission_status.py) owns. STORAGE_ID is the
# node's storage_id parameter (default sqlite3: what the car records).
#
# Checks:
#   1. single-instance lock: a second logger refuses to start (exit 1) and
#      names the lock;
#   2. RUNNING opens active/<run_id>/ with a start manifest and the params
#      snapshot;
#   3. COMPLETE moves the run whole to complete/<run_id>/ (active/ empty);
#   4. finalize: manifest outcome/end_time/bag_bytes/bag_sha256, recomputed
#      per-file sha256 equal, params snapshot byte-identical to the installed
#      stack_params.yaml in effect, extract.parquet written;
#   5. the bag's storage is the requested one;
#   6. every topic of SRC_BAG that the logger records (_DEFAULT_TOPICS of the
#      installed f1tenth_logger) is in the bag with messages;
#   7. the archive sync was requested: the trigger file was written
#      (F1TENTH_ARCHIVE_TRIGGER, the container contract) or `systemctl --user
#      start f1tenth-archive.service` was called -- captured by a PATH shim, so
#      nothing is ever synced from a check;
#   8. SIGINT shutdown releases the lock.
#
# Isolation: ROS_DOMAIN_ID 79, localhost discovery, no Discovery Server.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
[ $# -ge 2 ] || { echo "usage: logger_check.sh WORK_DIR SRC_BAG [STORAGE_ID] [PLAY_SEC]" >&2; exit 2; }
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
need_empty_dir "$1"
WORK=$(parity_abs "$1")
SRC=$(parity_abs "$2") || exit 2
STORAGE=${3:-sqlite3}
PLAY_SEC=${4:-12}
isolated_dds 79
export MISSION_LOGGER_LOCK="$WORK/mission_logger.lock"
RUNS="$WORK/runs"
mkdir -p "$RUNS" "$WORK/shim" "$WORK/archive_trigger"
# Archive sync capture (check 7): whatever the logger does to request a sync
# lands here and nowhere else.
cat > "$WORK/shim/systemctl" <<EOF
#!/bin/sh
echo "\$*" >> "$WORK/systemctl_calls.txt"
exit 0
EOF
chmod +x "$WORK/shim/systemctl"
export PATH="$WORK/shim:$PATH"
export F1TENTH_ARCHIVE_TRIGGER="$WORK/archive_trigger/request"
parity_context "$WORK"
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILED=1; }
FAILED=0
L1=""
# shellcheck disable=SC2329  # invoked by the EXIT trap
cleanup() { [ -n "$L1" ] && pg_alive "$L1" && pg_stop "$L1" 5 3 > /dev/null; }
trap cleanup EXIT

PARAMS=(-p "runs_dir:=$RUNS" -p "storage_id:=$STORAGE")
bg_start L1 "$WORK/logger1.log" ros2 run f1tenth_logger mission_logger_node --ros-args "${PARAMS[@]}"
for _ in $(seq 1 40); do [ -f "$MISSION_LOGGER_LOCK" ] && break; sleep 0.25; done
if [ -f "$MISSION_LOGGER_LOCK" ]; then pass "lock file created ($(cat "$MISSION_LOGGER_LOCK"))"; else fail "no lock file"; fi
for _ in $(seq 1 80); do grep -q "mission_logger_node up" "$WORK/logger1.log" && break; sleep 0.25; done

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
python3 "$HERE/publish_mission_status.py" RUNNING "$MISSION_JSON" 2.0 > "$WORK/status_running.log" 2>&1
sleep 1
ACTIVE_RUN=$(find "$RUNS/active" -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | head -1)
if [ -n "$ACTIVE_RUN" ] && [ -f "$RUNS/active/$ACTIVE_RUN/$ACTIVE_RUN.manifest.json" ] \
   && [ -f "$RUNS/active/$ACTIVE_RUN/$ACTIVE_RUN.params.yaml" ]; then
  pass "RUNNING -> active/$ACTIVE_RUN with start manifest + params snapshot"
else
  fail "no active run after RUNNING"
fi

# The source bag minus the topics the synthetic driver owns.
mapfile -t PLAY_TOPICS < <(python3 "$HERE/bag_compat.py" "$SRC" | awk '{print $1}' \
  | grep -vxE '/mission/status|/mission/move_outcome')
timeout --signal=INT "$PLAY_SEC" ros2 bag play "$SRC" --topics "${PLAY_TOPICS[@]}" \
  > "$WORK/play.log" 2>&1 < /dev/null
sleep 1

# 3. COMPLETE
python3 "$HERE/publish_mission_status.py" COMPLETE "$MISSION_JSON" 2.0 > "$WORK/status_complete.log" 2>&1
for _ in $(seq 1 240); do
  grep -q 'extract written\|finalize step "parquet extract" failed' "$WORK/logger1.log" && break
  sleep 0.5
done
sleep 2
RUN_DIR="$RUNS/complete/$ACTIVE_RUN"
if [ -n "$ACTIVE_RUN" ] && [ -d "$RUN_DIR" ] && [ -z "$(ls -A "$RUNS/active" 2>/dev/null)" ]; then
  pass "COMPLETE -> complete/$ACTIVE_RUN, active/ empty"
else
  fail "run not moved to complete/ (active: $(ls "$RUNS/active" 2>/dev/null))"
fi

# 4.-7. finalize, checksums, params, extract, storage, recorded topics, sync
LC_PLAY_SEC="$PLAY_SEC" python3 - "$RUN_DIR" "${ACTIVE_RUN:-none}" "$STORAGE" "$SRC" "$WORK" "$HERE" <<'PY' || FAILED=1
import hashlib, json, os, sys
import yaml
from ament_index_python.packages import get_package_share_directory
run_dir, run_id, want, src, work, here = sys.argv[1:7]
sys.path.insert(0, here)
import bag_compat  # noqa: F401 - on sys.path for the reader below
ok = True


def res(cond, text):
    global ok
    print(('PASS  ' if cond else 'FAIL  ') + text)
    ok = ok and bool(cond)


try:
    m = json.load(open(os.path.join(run_dir, run_id + '.manifest.json')))
except (OSError, ValueError) as exc:
    res(False, 'manifest unreadable: %s' % exc)
    sys.exit(1)
res(m.get('outcome') == 'COMPLETE' and m.get('end_time'),
    'manifest outcome=%s end_time=%s' % (m.get('outcome'), m.get('end_time')))
bag = os.path.join(run_dir, 'bag')
files = sorted(os.listdir(bag))
recomputed, total = {}, 0
for f in files:
    p = os.path.join(bag, f)
    recomputed[f] = hashlib.sha256(open(p, 'rb').read()).hexdigest()
    total += os.path.getsize(p)
stored = m.get('bag_sha256')
res(isinstance(stored, dict) and stored == recomputed,
    'bag_sha256 recomputed and equal for %d files (%s)' % (len(recomputed), ', '.join(files)))
res(m.get('bag_bytes') == total, 'bag_bytes %s == %d' % (m.get('bag_bytes'), total))
sp = os.path.realpath(os.path.join(get_package_share_directory('f1tenth_params'), 'config',
                                   'stack_params.yaml'))
res(open(sp, 'rb').read() == open(os.path.join(run_dir, run_id + '.params.yaml'), 'rb').read(),
    'params snapshot byte-identical to %s' % sp)
ext = '.mcap' if want == 'mcap' else '.db3'
res(m.get('storage_id') == want and any(f.endswith(ext) for f in files),
    'storage %s as requested (%s)' % (m.get('storage_id'), want))
meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
counts = {t['topic_metadata']['name']: t['message_count'] for t in meta['topics_with_message_count']}
from f1tenth_logger.mission_logger_node import _DEFAULT_TOPICS
from bag_compat import open_reader
# What was actually played: messages per topic in the first PLAY_SEC of the
# source bag. A topic counts when it had >= 3 there (one or two sparse
# messages right at the start can go out before discovery matched).
play_sec = float(os.environ.get('LC_PLAY_SEC', '12'))
reader = open_reader(src)
played, t_first = {}, None
while reader.has_next():
    topic, _, t = reader.read_next()
    t_first = t if t_first is None else t_first
    if t - t_first <= play_sec * 1e9:
        played[topic] = played.get(topic, 0) + 1
del reader
skip = {'/mission/status', '/mission/move_outcome'}
expected = sorted(t for t, n in played.items() if n >= 3 and t in set(_DEFAULT_TOPICS) - skip)
missing = [t for t in expected if counts.get(t, 0) == 0]
res(expected and not missing,
    'all %d logger topics played from the source bag are in the run with messages%s'
    % (len(expected), (': missing ' + ', '.join(missing)) if missing else ''))
ex = os.path.join(run_dir, run_id + '.extract.parquet')
res(os.path.isfile(ex), 'extract.parquet written (%s)'
    % ('%d bytes' % os.path.getsize(ex) if os.path.isfile(ex) else 'missing'))
trig = os.path.join(work, 'archive_trigger', 'request')
calls = os.path.join(work, 'systemctl_calls.txt')
how = []
if os.path.exists(trig):
    how.append('trigger file %s' % open(trig).read().strip()[:120])
if os.path.exists(calls):
    how.append('systemctl ' + open(calls).read().strip().replace('\n', ' | '))
res(bool(how), 'archive sync requested: %s' % ('; '.join(how) or 'neither trigger nor systemctl'))
sys.exit(0 if ok else 1)
PY
grep -h "finalize step\|ERROR\|Traceback" "$WORK/logger1.log" | head -5

# 8. shutdown
if pg_stop "$L1" 30 5 > "$WORK/logger1_stop.txt"; then :; else fail "logger did not exit on SIGINT/SIGTERM"; fi
L1=""
if [ -f "$MISSION_LOGGER_LOCK" ]; then fail "lock not released on shutdown"; else pass "lock released on shutdown"; fi
echo "RESULT $([ $FAILED -eq 0 ] && echo PASS || echo FAIL)  run=${ACTIVE_RUN:-none}"
exit $FAILED
