#!/usr/bin/env bash
# scenario.sh OUT MISSION [sim_isaac.sh all args ...] -- one Isaac scenario, recorded, on linus.
#
# Starts `scripts/sim/sim_isaac.sh all <args>` (own session), waits for the stack, then records
# for 75 s: tools/rust_parity/proximity/guard_vs_bt.py (guard vs BT front decision per scan) and
# ackermann_sources.py (/safety_stop and /ackermann_drive counted by frame_id). After 8 s it calls
# /sim/reset (the parked car creeps backwards, see isaac_test.md) and runs MISSION through
# tools/sim/e2e_checks.py mission. Stops sim + stack at the end. Results in OUT/.
#
#   tools/rust_parity/isaac/scenario.sh /tmp/a_active \
#       $(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/drive_test_straight_2m.json \
#       guard:=active impl:=rust
#   tools/rust_parity/isaac/scenario.sh /tmp/b2 tools/rust_parity/isaac/b2_straight_into_box_2p9m.json \
#       --layout obstacle guard:=shadow impl:=rust
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"
OUT=$(mkdir -p "$1" && cd "$1" && pwd); MISSION=$(realpath "$2"); shift 2
HERE="$REPO/tools/rust_parity/isaac"

F1T_SIM_LOG_DIR=$OUT setsid scripts/sim/sim_isaac.sh all "$@" > "$OUT/sim_isaac.log" 2>&1 < /dev/null &
SIM=$!
stop_sim() {   # SIGTERM to sim_isaac.sh: its cleanup stops the stack, Isaac, the Discovery Server
  kill -TERM "$SIM" 2>/dev/null
  for _ in $(seq 1 100); do kill -0 "$SIM" 2>/dev/null || break; sleep 1; done
}
trap stop_sim EXIT
up=1
for _ in $(seq 1 150); do
  grep -q "Started 'behavior'" "$OUT/sim_isaac.log" && { up=0; break; }
  kill -0 "$SIM" 2>/dev/null || break
  sleep 3
done
[ "$up" = 0 ] || { echo "stack not up: $OUT/sim_isaac.log"; exit 1; }
sleep 30   # detector warm-up, /tf_static, first map
R=(scripts/sim/sim_isaac.sh run --domain 42 --)
"${R[@]}" python3 "$REPO/tools/rust_parity/proximity/guard_vs_bt.py" --seconds 75 \
    --out "$OUT/guard_vs_bt.json" > "$OUT/rec.log" 2>&1 &
P1=$!
"${R[@]}" python3 "$HERE/ackermann_sources.py" 75 > "$OUT/sources.log" 2>&1 &
P2=$!
sleep 8
scripts/sim/sim_isaac.sh reset > "$OUT/reset.log" 2>&1
"${R[@]}" python3 "$REPO/tools/sim/e2e_checks.py" mission --mission "$MISSION" --timeout 50 \
    > "$OUT/mission.json" 2> "$OUT/mission.err"
wait "$P1" "$P2"
grep SOURCES "$OUT/sources.log"
tail -1 "$OUT/mission.json" | cut -c1-300
tail -1 "$OUT/rec.log" | cut -c1-600
