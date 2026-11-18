#!/usr/bin/env bash
# Start every f1tenth_diagnostics executable on this side, with no hardware
# and no other stack running, and record how each one behaves: still running
# after RUN_SEC (and then stops cleanly on SIGINT), or exits on its own --
# with what message, and whether there was a traceback.
#
# Runs on: the Orin host with native Humble sourced, inside the harness
# container (Jazzy), linus. Run on both sides of P9 and diff the two
# states.txt files: the same executable must behave the same way. The
# system_observer line also says whether jtop answered (the GPU fields come
# through /run/jtop.sock; 0.0 everywhere means jtop is not reachable, M3).
#
#   diag_start_check.sh OUT_DIR [RUN_SEC]
#
# Plain `ros2 run <pkg> <exe>` with each node's own parameter defaults --
# deliberately NOT the calibration launch files, which can bring up the VESC
# driver group. Isolated domain 80, localhost discovery: no driver, no mux,
# nothing that could carry a command to hardware. Each node runs in its own
# process group; the group is SIGINTed, then SIGTERMed / SIGKILLed if still
# alive (never `pkill -f`).
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
need_empty_dir "$1"
OUT=$(parity_abs "$1")
RUN_SEC=${2:-12}
isolated_dds 80
parity_context "$OUT"
EXES="battery_voltage_check_node diagnostics_server_node system_observer_node \
gyro_bias_calibration_node sensor_covariance_calibration_node \
slam_pose_covariance_calibration_node steering_offset_calibration_node"
G=""
trap '[ -n "$G" ] && pg_stop "$G" 3 2 > /dev/null' EXIT

for exe in $EXES; do
  log="$OUT/$exe.log"
  bg_start G "$log" ros2 run f1tenth_diagnostics "$exe"
  exited=""
  extra=""
  for _ in $(seq 1 $((RUN_SEC * 4))); do
    if ! kill -0 "$G" 2>/dev/null; then exited=yes; break; fi
    sleep 0.25
  done
  if [ "$exe" = system_observer_node ] && [ -z "$exited" ]; then
    extra=$(timeout 20 python3 - <<'PY' 2>&1 | tail -1
import time, rclpy
from rosidl_runtime_py.utilities import get_message
rclpy.init()
n = rclpy.create_node('diag_check_status_probe')
got = []
n.create_subscription(get_message('f1tenth_messages/msg/SystemStatus'),
                      '/diagnostics/system_status', got.append, 10)
end = time.monotonic() + 8
while time.monotonic() < end and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    m = got[-1]
    f = {k: getattr(m, k) for k in ('gpu_percent', 'gpu_temp_c', 'emc_percent', 'cpu_temp_c')
         if hasattr(m, k)}
    print('system_status: ' + ' '.join('%s=%s' % kv for kv in sorted(f.items())))
else:
    print('system_status: no message in 8 s')
PY
)
  fi
  if [ -n "$exited" ]; then
    wait "$G"
    rc=$?
    state="EXITED_ON_ITS_OWN rc=$rc"; cls="EXITED_ON_ITS_OWN rc=$rc"
  else
    if pg_stop "$G" 10 5 > /dev/null; then
      wait "$G"
      rc=$?
      state="RUNNING_AFTER_${RUN_SEC}s, clean stop rc=$rc"; cls=RUNNING_CLEAN_STOP
    else
      state="RUNNING_AFTER_${RUN_SEC}s, needed SIGKILL"; cls=RUNNING_NEEDED_SIGKILL
    fi
  fi
  G=""
  tb=$(grep -c "Traceback" "$log")
  jt=""
  if [ "$exe" = system_observer_node ]; then
    if grep -q "jtop connection failed\|jtop.*not installed\|published as 0.0" "$log"; then
      jt="jtop=NO"
    else
      jt="jtop=yes"
    fi
  fi
  last=$(grep -E "\[(ERROR|WARN|FATAL)\]|Error|refus|abort|not found|without" "$log" | head -2 \
         | sed 's/^\[[^]]*\] \[[0-9.]*\] //' | cut -c1-230 | tr '\n' ' ')
  printf '%-40s %-45s tracebacks=%s %s %s| %s\n' "$exe" "$state" "$tb" "$jt" "$extra" "$last"
  # Comparable across sides: the state class only, no pids, times or rc noise.
  printf '%s %s tracebacks=%s %s\n' "$exe" "$cls" "$tb" "$jt" >> "$OUT/states.txt"
done
