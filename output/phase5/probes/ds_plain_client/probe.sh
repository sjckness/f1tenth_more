#!/usr/bin/env bash
# What a PLAIN Discovery Server client (no ROS_SUPER_CLIENT) learns about the
# graph: talker + listener on a server, then `ros2 topic list` as a plain
# client and as a super client. Run under each distro.
#   probe.sh LABEL PORT
LABEL=$1; PORT=$2
export ROS_DOMAIN_ID=91
unset ROS_SUPER_CLIENT ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY
fastdds discovery -i 0 -l 127.0.0.1 -p $PORT > /tmp/dsp_server_$LABEL.log 2>&1 &
sleep 2
export ROS_DISCOVERY_SERVER=127.0.0.1:$PORT
python3 $(dirname $0)/pubsub.py talker > /dev/null 2>&1 &
P1=$!
python3 $(dirname $0)/pubsub.py listener > /dev/null 2>&1 &
P2=$!
sleep 5
echo "=== $LABEL: $(dpkg -l | awk '/ros-[a-z]+-fastrtps /{print $2, $3}')"
echo "--- plain client: ros2 topic list"
timeout 20 ros2 topic list --no-daemon --spin-time 5
echo "--- plain client: ros2 topic info /chatter"
timeout 20 ros2 topic info /chatter --no-daemon --spin-time 5 2>&1 | head -3
echo "--- super client: ros2 topic list"
ROS_SUPER_CLIENT=TRUE timeout 20 ros2 topic list --no-daemon --spin-time 5
kill -INT $P1 $P2; pkill -INT -x fast-discovery-; sleep 1
