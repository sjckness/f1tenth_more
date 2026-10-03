#!/usr/bin/env bash
# Fix batch 1, item 1: run the real system_observer_node (no jtop on Thor, so
# the sysfs fallback runs) for SOAK_SEC with the GPU idle, alongside
# system_observer_soak_monitor.py. Isolated ROS_DOMAIN_ID 81, localhost
# discovery, no other node, no hardware.
#
#   system_observer_soak.sh OUT_DIR [SOAK_SEC]
set -uo pipefail
OUT=$(mkdir -p "$1" && cd "$1" && pwd)
SOAK_SEC=${2:-330}
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=81 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER 2>/dev/null || true

# SIG_DFL before exec: background jobs of a non-interactive shell inherit
# SIGINT as ignored (Phase 4 finding).
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 run f1tenth_diagnostics system_observer_node > "$OUT/system_observer_node.log" 2>&1 < /dev/null &
wrapper=$!
sleep 2
python3 "$HERE/system_observer_soak_monitor.py" "$OUT" "$SOAK_SEC" > "$OUT/monitor.log" 2>&1
if kill -0 "$wrapper" 2>/dev/null; then alive=yes; else alive=no; fi
pkill -INT -f "lib/f1tenth_diagnostics/system_observer_node"
for _ in $(seq 1 40); do kill -0 "$wrapper" 2>/dev/null || break; sleep 0.25; done
wait "$wrapper"; rc=$?
pkill -9 -f "lib/f1tenth_diagnostics/system_observer_node" 2>/dev/null
n=$(($(wc -l < "$OUT/system_status.csv") - 1))
{
  echo "node alive at end of soak: $alive"
  echo "node exit code after SIGINT: $rc"
  echo "tracebacks in node log: $(grep -c Traceback "$OUT/system_observer_node.log")"
  echo "system_status messages received: $n in ${SOAK_SEC}s"
  echo "monitor: $(tail -1 "$OUT/monitor.log")"
} | tee "$OUT/result.txt"
