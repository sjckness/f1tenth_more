#!/usr/bin/env bash
# Fix batch 5: does robot_localization freeze header.stamp when its input
# stops (the premise of localization's new_stamp checks)? One hold run:
# stamp_probe.py as a super client for 20 s, the feed killed 8 s in.
set -uo pipefail
cd ~/dev_ws/f1tenth_more
R=output/fix_batch_5/ekf_stamps
export ROS_DOMAIN_ID=85 ROS_DISCOVERY_SERVER=127.0.0.1:11811
HOLD_SEC=90 bash scripts/jazzy_parity/phase5_bringup.sh hold $R > $R.out 2>&1 &
HP=$!
for _ in $(seq 1 120); do [ -e $R/READY ] && break; sleep 2; done
sleep 15
ROS_SUPER_CLIENT=TRUE python3 scripts/fix_batch_5/stamp_probe.py --duration 20 --bin 0.5 \
  /odometry/filtered /ekf_global/odometry/filtered /odom > $R/ekf_stamps.json &
PR=$!
sleep 8
date +%s.%N > $R/t_feed_stop.txt
FEED=$(ps -eo pid,args | awk '/restamp_play.py/ && !/awk/ {print $1; exit}')
kill -TERM "$FEED"
wait $PR
wait $HP
