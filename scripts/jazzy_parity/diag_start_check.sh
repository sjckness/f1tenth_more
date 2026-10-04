#!/usr/bin/env bash
# Phase 4, Step 3: start every f1tenth_diagnostics executable on this distro,
# with no hardware and no other stack running, and record how each one
# behaves: still running after RUN_SEC (and then stops cleanly on SIGINT), or
# exits on its own -- with what message, and whether there was a traceback.
#
#   diag_start_check.sh OUT_DIR [RUN_SEC]
#
# Plain `ros2 run <pkg> <exe>` with the node's own parameter defaults --
# deliberately NOT the calibration launch files, which can bring up the VESC
# driver group. Isolated ROS_DOMAIN_ID 80, localhost discovery: no driver, no
# mux, nothing that could carry a command to hardware. Each node's process
# tree is SIGINTed, then SIGKILLed if still alive (`ros2 run` orphans its
# child on a plain kill).
set -uo pipefail
OUT=$(mkdir -p "$1" && cd "$1" && pwd)
RUN_SEC=${2:-12}
export ROS_DOMAIN_ID=80 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER 2>/dev/null || true
EXES="battery_voltage_check_node diagnostics_server_node ekf_cost_observer_node \
system_observer_node gyro_bias_calibration_node sensor_covariance_calibration_node \
slam_pose_covariance_calibration_node steering_offset_calibration_node"

for exe in $EXES; do
  log="$OUT/$exe.log"
  # A background job of a non-interactive shell inherits SIGINT as IGNORED,
  # and a shell cannot un-ignore it. Python then never installs its
  # KeyboardInterrupt handler, and the later SIGINT surfaces as rclpy's
  # ExternalShutdownException -- on Humble (rclpy 3.3.21) and Jazzy alike,
  # checked with a probe. Restore SIG_DFL before exec so each node sees the
  # SIGINT a launch file or a terminal would deliver.
  python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    ros2 run f1tenth_diagnostics "$exe" > "$log" 2>&1 < /dev/null &
  wrapper=$!
  exited=""
  for _ in $(seq 1 $((RUN_SEC * 4))); do
    if ! kill -0 "$wrapper" 2>/dev/null; then exited=yes; break; fi
    sleep 0.25
  done
  if [ -n "$exited" ]; then
    wait "$wrapper"; rc=$?
    state="EXITED_ON_ITS_OWN rc=$rc"
  else
    pkill -INT -f "lib/f1tenth_diagnostics/$exe" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "$wrapper" 2>/dev/null || break; sleep 0.25; done
    if kill -0 "$wrapper" 2>/dev/null; then
      pkill -9 -f "lib/f1tenth_diagnostics/$exe"; kill -9 "$wrapper" 2>/dev/null
      state="RUNNING_AFTER_${RUN_SEC}s, needed SIGKILL"
    else
      wait "$wrapper"; rc=$?
      state="RUNNING_AFTER_${RUN_SEC}s, clean SIGINT stop rc=$rc"
    fi
  fi
  pkill -9 -f "lib/f1tenth_diagnostics/$exe" 2>/dev/null
  tb=$(grep -c "Traceback" "$log")
  last=$(grep -E "\[(ERROR|WARN|FATAL)\]|Error|refus|abort|not found|without" "$log" | head -2 | sed 's/^\[[^]]*\] \[[0-9.]*\] //' | cut -c1-230 | tr '\n' ' ')
  printf '%-40s %-45s tracebacks=%s | %s\n' "$exe" "$state" "$tb" "$last"
done
