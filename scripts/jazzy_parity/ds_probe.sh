#!/usr/bin/env bash
# The Fast DDS Discovery Server of this side, outside the stack.
#
# Runs on: the Orin host with native Humble sourced, inside the harness
# container (Jazzy), linus. Run it on both sides of P9 and compare the
# summaries: the server must behave the same way.
#
# Starts the server exactly as supervisor_bringup.launch.py does (the
# installed ensure_discovery_server.py), puts a talker and a listener
# (docker/orin/checks/dds_probe.py) on it, and records what the ros2 CLI sees
# as a plain client, as a super client and without a server, with discovery
# limited to the host and not (Jazzy: ROS_AUTOMATIC_DISCOVERY_RANGE SUBNET /
# LOCALHOST; Humble: ROS_LOCALHOST_ONLY 0 / 1). Then checks
# ensure_discovery_server.py's reuse branch (port already held: it must idle,
# then exit on SIGINT without leaving its `tail -f /dev/null` behind).
#
#   ds_probe.sh OUT_DIR [PORT]      (PORT default 11811; on linus use another)
#
# Pass (summary.txt, exit 0): the listener received messages in both ranges;
# a plain client does not see the talker's topic (it does see node names:
# those travel on ros_discovery_info), a super client sees both nodes and the
# topic; the second server instance idled and its group was gone after
# SIGINT; 0 "Matching unexisting participant".
# Isolation: ROS_DOMAIN_ID 87, server on 127.0.0.1 only; refuses if PORT is
# already bound.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# shellcheck source=lib_parity.sh
. "$HERE/lib_parity.sh"
need_empty_dir "$1"
OUT=$(parity_abs "$1")
PORT=${2:-11811}
PROBE=$REPO/docker/orin/checks/dds_probe.py
SHARE=$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup
export ROS_DOMAIN_ID=87
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
SRV="" REUSE="" T="" L=""
cleanup() {
  local g
  for g in $T $L $REUSE $SRV; do pg_alive "$g" && pg_stop "$g" 3 2 > /dev/null; done
  ros2 daemon stop > /dev/null 2>&1
}
trap cleanup EXIT
parity_context "$OUT"

{
  echo "--- versions"
  dpkg -l "ros-${ROS_DISTRO}-fastrtps" "ros-${ROS_DISTRO}-rmw-fastrtps-cpp" \
    "ros-${ROS_DISTRO}-rmw-fastrtps-shared-cpp" 2>/dev/null | awk '/^ii/{print $2, $3}'
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  python3 -c "import rclpy.utilities as u; print('rmw in use:', u.get_rmw_implementation_identifier())"
  echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
  echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
} > "$OUT/versions.txt" 2>&1

if udp_port_bound "$PORT"; then echo "ABORT: UDP port $PORT already in use"; exit 2; fi

echo "--- server start via ensure_discovery_server.py" > "$OUT/server.log"
bg_start SRV "$OUT/server_out.log" python3 "$SHARE/scripts/ensure_discovery_server.py" 127.0.0.1 "$PORT"
sleep 2
ps -o pid,pgid,args --no-headers -s "$SRV" >> "$OUT/server.log"
ss -H -ulnp "sport = :$PORT" >> "$OUT/server.log" 2>&1

echo "--- reuse branch: second ensure_discovery_server.py on the same port" > "$OUT/reuse.log"
bg_start REUSE "$OUT/reuse_out.log" python3 "$SHARE/scripts/ensure_discovery_server.py" 127.0.0.1 "$PORT"
sleep 2
if pg_alive "$REUSE"; then
  echo "second instance idling: $(ps -o pid,args --no-headers -s "$REUSE" | tr '\n' ';')" >> "$OUT/reuse.log"
  REUSE_IDLED=1
else
  echo "second instance exited on its own" >> "$OUT/reuse.log"
  REUSE_IDLED=0
fi
pg_stop "$REUSE" 5 2 >> "$OUT/reuse.log"
REUSE_STOP=$?
REUSE=""

if [ "${ROS_DISTRO:-}" = humble ]; then RANGES="0 1"; else RANGES="SUBNET LOCALHOST"; fi
for RANGE in $RANGES; do
  if [ "${ROS_DISTRO:-}" = humble ]; then
    export ROS_LOCALHOST_ONLY=$RANGE; label="ROS_LOCALHOST_ONLY=$RANGE"
  else
    export ROS_AUTOMATIC_DISCOVERY_RANGE=$RANGE; label="ROS_AUTOMATIC_DISCOVERY_RANGE=$RANGE"
  fi
  export ROS_DISCOVERY_SERVER=127.0.0.1:$PORT
  # Both run until stopped below, after the CLI queries (the listener prints
  # its JSON when SIGINT ends it).
  bg_start T "$OUT/talker_$RANGE.log" python3 "$PROBE" talker 120 --node ds_talker
  bg_start L "$OUT/listener_$RANGE.log" python3 "$PROBE" listener 120 --node ds_listener
  sleep 6
  {
    echo "=== $label, ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
    echo "--- ros2 node list --no-daemon, plain client:"
    env -u ROS_SUPER_CLIENT timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 topic list --no-daemon, plain client:"
    env -u ROS_SUPER_CLIENT timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 node list --no-daemon, ROS_SUPER_CLIENT=TRUE:"
    ROS_SUPER_CLIENT=TRUE timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
    echo "--- ros2 topic list --no-daemon, ROS_SUPER_CLIENT=TRUE:"
    ROS_SUPER_CLIENT=TRUE timeout 20 ros2 topic list --no-daemon --spin-time 3 2>&1
    echo "--- no ROS_DISCOVERY_SERVER at all (a simple-discovery participant):"
    env -u ROS_DISCOVERY_SERVER timeout 20 ros2 node list --no-daemon --spin-time 3 2>&1
  } > "$OUT/graph_$RANGE.txt"
  pg_stop "$L" 5 2 > /dev/null
  pg_stop "$T" 5 2 > /dev/null
  T="" L=""
  echo "listener: $(cat "$OUT/listener_$RANGE.log")" >> "$OUT/graph_$RANGE.txt"
done

pg_stop "$SRV" 5 3 >> "$OUT/server.log"
SRV=""
ros2 daemon stop > /dev/null 2>&1
MUP=$(cat "$OUT"/*.log | grep -c "Matching unexisting participant")
echo "$MUP" > "$OUT/matching_unexisting_count.txt"

python3 - "$OUT" "$RANGES" "$REUSE_IDLED" "$REUSE_STOP" "$MUP" <<'PY'
import json, os, re, sys
out, ranges, idled, stop_rc, mup = sys.argv[1], sys.argv[2].split(), sys.argv[3], sys.argv[4], int(sys.argv[5])
bad, lines = [], []
for r in ranges:
    g = open(os.path.join(out, 'graph_%s.txt' % r)).read()
    m = re.search(r'listener: (\{.*\})', g)
    rx = json.loads(m.group(1))['received'] if m else 0
    sec = re.split(r'^--- ', g, flags=re.M)

    def part(prefix):
        return next((s for s in sec if s.startswith(prefix)), '')
    plain_nodes = part('ros2 node list --no-daemon, plain')
    plain_topics = part('ros2 topic list --no-daemon, plain')
    sup_nodes = part('ros2 node list --no-daemon, ROS_SUPER')
    sup_topics = part('ros2 topic list --no-daemon, ROS_SUPER')
    lines.append('%s: listener received %d; plain client: nodes %s, topic %s; super client: '
                 'nodes %s, topic %s' % (
                     r, rx, '/ds_talker' in plain_nodes, '/p5d_probe' in plain_topics,
                     '/ds_talker' in sup_nodes and '/ds_listener' in sup_nodes,
                     '/p5d_probe' in sup_topics))
    if rx == 0:
        bad.append('%s: no message through the server' % r)
    # A plain Discovery Server client learns node names (ros_discovery_info)
    # but only the endpoints matching its own: it must not see the topic.
    if '/p5d_probe' in plain_topics:
        bad.append('%s: a plain client sees the talker\'s topic' % r)
    if ('/ds_talker' not in sup_nodes or '/ds_listener' not in sup_nodes
            or '/p5d_probe' not in sup_topics):
        bad.append('%s: the super client does not see both nodes and the topic' % r)
if idled != '1':
    bad.append('reuse: the second ensure_discovery_server.py did not idle')
if stop_rc != '0':
    bad.append('reuse: the idling instance needed SIGKILL')
if mup:
    bad.append('%d "Matching unexisting participant"' % mup)
res = 'PASS' if not bad else 'FAIL'
text = '\n'.join(['%s ds_probe' % res] + lines + bad) + '\n'
open(os.path.join(out, 'summary.txt'), 'w').write(text)
print(text, end='')
sys.exit(0 if not bad else 1)
PY
