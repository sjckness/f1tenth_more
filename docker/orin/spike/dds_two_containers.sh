#!/bin/bash
# G5 (DDS), run on the HOST: two spike containers, both --network host
# --ipc host. Container A runs the Discovery Server (127.0.0.1:11811, as the
# stack's ensure_discovery_server.py does) and a 50 Hz talker; container B a
# listener. Both carry the stack's env and the 5 s lease profile. While they
# run, the host lists the Fast DDS shared-memory segments in /dev/shm: with
# ipc host they are the host's /dev/shm, so their presence shows SHM transport
# between two containers exactly as between two bare-metal processes.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OUT=$(cd "$HERE/../../.." && pwd)/output/phase5d_logs/spike
ENV='export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_DISCOVERY_SERVER=127.0.0.1:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/f1tenth_ws/install/share/f1tenth_bringup/config/fastdds_profile.xml
source /opt/ros/jazzy/setup.bash; source /opt/f1tenth_ws/install/setup.bash
test -f "$FASTRTPS_DEFAULT_PROFILES_FILE" || { echo "profile missing: $FASTRTPS_DEFAULT_PROFILES_FILE"; exit 1; }'

shm_before=$(ls /dev/shm | grep -c -i fastrtps || true)
"$HERE/spike.sh" run bash -c "$ENV
fastdds discovery --server-id 0 --udp-address 127.0.0.1 --udp-port 11811 > /tmp/ds.log 2>&1 &
sleep 3
python3 /spike/gates/dds_probe.py talker 35
kill %1; wait; echo '-- DS log'; tail -5 /tmp/ds.log" > "$OUT/g5_dds_containerA.log" 2>&1 &
a=$!
sleep 10
"$HERE/spike.sh" run bash -c "$ENV
python3 /spike/gates/dds_probe.py listener 20" > "$OUT/g5_dds_containerB.log" 2>&1 &
b=$!
sleep 8
{
  echo "fastrtps entries in /dev/shm before: $shm_before"
  echo "during (host view):"
  ls -l /dev/shm | grep -i fastrtps | awk '{print "  " $3, $5, $9}'
} > "$OUT/g5_dds_shm.log"
wait $b || true
wait $a || true
echo "after: $(ls /dev/shm | grep -c -i fastrtps || true) fastrtps entries" >> "$OUT/g5_dds_shm.log"
cat "$OUT/g5_dds_containerA.log" "$OUT/g5_dds_containerB.log" "$OUT/g5_dds_shm.log"
