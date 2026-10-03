#!/usr/bin/env bash
# Fix batch 4 (A): start the steering calibration through its launch file
# N times against a running stack and record what its e-stop check found.
# amplitude_rad:=-0.1 makes preflight refuse on the steering-limit check, so
# every stage-1 check is evaluated and logged but nothing is ever commanded
# (the nudge and the drive are only reachable when ALL stage-1 checks pass).
#   estop_preflight_live.sh OUT_DIR LABEL N
set -uo pipefail
OUT=$(mkdir -p "$1" && cd "$1" && pwd); LABEL=$2; N=$3
for i in $(seq 1 "$N"); do
  log="$OUT/${LABEL}_$i.log"
  timeout 60 ros2 launch f1tenth_diagnostics steering_offset_calibration.launch.py \
    amplitude_rad:=-0.1 2>&1 | sed 's/\x1b\[[0-9;]*m//g' > "$log"
  g=$(grep -o 'e-stop graph check after.*' "$log" | head -1)
  if grep -q 'publisher(s) on /safety_stop (safety lane is live)' "$log"; then r=PASS
  elif grep -q 'NO publisher on /safety_stop' "$log"; then r='REFUSE(no /safety_stop publisher)'
  elif grep -q 'nothing is subscribed to' "$log"; then r='REFUSE(no mux lane)'
  else r=UNKNOWN; fi
  cmd=$(grep -c 'PREFLIGHT PASSED' "$log")
  echo "$LABEL $i: estop_check=$r | $g | refused_before_command=$(grep -c 'nothing was commanded' "$log") preflight_passed_lines=$cmd"
done | tee "$OUT/${LABEL}_summary.txt"
