#!/usr/bin/env bash
# Phase 5, Step 3: the Fast DDS Discovery Server on this distro, outside the
# stack. Starts it exactly as supervisor_bringup.launch.py does (through
# ensure_discovery_server.py), puts two talker/listener clients on it, and
# records what the ros2 CLI sees as a plain client vs as a super client, with
# ROS_AUTOMATIC_DISCOVERY_RANGE at Jazzy's default (SUBNET) and at LOCALHOST.
# Then checks ensure_discovery_server.py's reuse branch (port already held).
#
#   ds_probe.sh OUT_DIR [PORT]
#
# Isolation: ROS_DOMAIN_ID 87, server on 127.0.0.1 (loopback only).
set -uo pipefail
OUT=$(mkdir -p "$1" && cd "$1" && pwd)
PORT=${2:-11811}
SHARE=$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup
export ROS_DOMAIN_ID=87
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
SIGDFL=(python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')

{
echo "--- versions"
dpkg -l ros-jazzy-fastrtps ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rmw-fastrtps-shared-cpp | awk '/^ii/{print $2, $3}'
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset, Jazzy default rmw_fastrtps_cpp>}"
python3 -c "import rclpy.utilities as u; print('rmw in use:', u.get_rmw_implementation_identifier())"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
echo "FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-<unset>}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE (shell default)=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
} > "$OUT/versions.txt" 2>&1

ss -lunp 2>/dev/null | grep ":$PORT " && { echo "ABORT: port $PORT already in use"; exit 2; }

echo "--- server start via ensure_discovery_server.py" > "$OUT/server.log"
"${SIGDFL[@]}" python3 "$SHARE/scripts/ensure_discovery_server.py" 127.0.0.1 "$PORT" >> "$OUT/server.log" 2>&1 &
SRV=$!
sleep 2
ps -o pid,args --no-headers -p $SRV >> "$OUT/server.log"
ss -lunp 2>/dev/null | grep ":$PORT " >> "$OUT/server.log"

echo "--- reuse branch: second ensure_discovery_server.py on the same port" > "$OUT/reuse.log"
"${SIGDFL[@]}" python3 "$SHARE/scripts/ensure_discovery_server.py" 127.0.0.1 "$PORT" >> "$OUT/reuse.log" 2>&1 &
REUSE=$!
sleep 2
kill -0 $REUSE 2>/dev/null && echo "second instance still alive (idling), rc n/a" >> "$OUT/reuse.log"
pkill -INT -P $REUSE 2>/dev/null; kill -INT $REUSE 2>/dev/null; wait $REUSE 2>/dev/null
echo "second instance exit code after SIGINT: $?" >> "$OUT/reuse.log"
pgrep -f "tail -f /dev/null" >/dev/null && { echo "leftover tail process found" >> "$OUT/reuse.log"; } || echo "no leftover tail process" >> "$OUT/reuse.log"

for RANGE in SUBNET LOCALHOST; do
  export ROS_AUTOMATIC_DISCOVERY_RANGE=$RANGE ROS_DISCOVERY_SERVER=127.0.0.1:$PORT
  "${SIGDFL[@]}" ros2 run demo_nodes_cpp talker --ros-args -r __node:=ds_talker > "$OUT/talker_$RANGE.log" 2>&1 &
  T=$!
  "${SIGDFL[@]}" ros2 run demo_nodes_py listener --ros-args -r __node:=ds_listener > "$OUT/listener_$RANGE.log" 2>&1 &
  L=$!
  sleep 6
  {
    echo "=== ROS_AUTOMATIC_DISCOVERY_RANGE=$RANGE, ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
    echo "listener received: $(grep -c 'I heard' "$OUT/listener_$RANGE.log") messages in 6 s"
    echo "--- ros2 node list --no-daemon, plain client:"
    ROS_SUPER_CLIENT= timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 topic list --no-daemon, plain client:"
    ROS_SUPER_CLIENT= timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 node list --no-daemon, ROS_SUPER_CLIENT=TRUE:"
    ROS_SUPER_CLIENT=TRUE timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 topic list --no-daemon, ROS_SUPER_CLIENT=TRUE:"
    ROS_SUPER_CLIENT=TRUE timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1
    echo "--- no ROS_DISCOVERY_SERVER at all (a simple-discovery participant):"
    env -u ROS_DISCOVERY_SERVER timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
  } > "$OUT/graph_$RANGE.txt"
  pkill -INT -f "__node:=ds_talker"; pkill -INT -f "__node:=ds_listener"
  wait $T $L 2>/dev/null
  pkill -9 -f "__node:=ds_" 2>/dev/null
done

# SIGINT to the launched wrapper ($SRV, `sh fastdds`) does not reach the
# server: sh does not forward it, and fastdds.py runs fast-discovery-server
# as a child. Recorded, then the real server process is stopped directly.
kill -INT $SRV; sleep 2
if pgrep -f "fast-discovery-server .*--udp-port $PORT" >/dev/null; then
  echo "SIGINT to the wrapper pid $SRV: fast-discovery-server still running" >> "$OUT/server.log"
fi
pkill -INT -f "fast-discovery-server .*--udp-port $PORT"; wait $SRV 2>/dev/null
echo "server chain exit code after SIGINT to fast-discovery-server: $?" >> "$OUT/server.log"
ros2 daemon stop >/dev/null 2>&1
cat "$OUT"/*.log | grep -c "Matching unexisting participant" > "$OUT/matching_unexisting_count.txt"
cat "$OUT"/graph_*.txt "$OUT/reuse.log" "$OUT/server.log"
