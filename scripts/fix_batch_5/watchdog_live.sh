#!/usr/bin/env bash
# Fix batch 5 (H1): live tests of the supervisor's topic-liveness watchdog in
# the Phase 5 harness (scripts/jazzy_parity/phase5_bringup.sh, no hardware,
# bag-fed, ROS_DOMAIN_ID 85). TESTS picks which, in order:
#
#   normal   sim:=false, no /clock anywhere. (1) bringup, then 10 min with
#            nothing done to it: no watchdog trigger allowed. (2) SIGSTOP
#            swept_clearance_node: must be detected (FAILING) within its
#            max age, restarted after fail_for_sec, and back to OK. Then the
#            feed is stopped while stamp_probe.py watches both EKFs (premise
#            of the new_stamp check) -- every component must go
#            UPSTREAM_STALE, none restarted.
#   failing  (3) swept_clearance's check patched to a topic nothing
#            publishes: restarted 3 times, then FAILED, and stays FAILED.
#   sim      (4) sim:=true, /clock from `ros2 bag play --clock`; /clock is
#            paused for 30 s through /rosbag2_player/pause: no trigger
#            allowed, PAUSED while paused, OK again after.
#
# Every test records /supervisor/health changes (health_recorder.py) next to
# the harness output. Evaluated by watchdog_live_summary.py.
set -uo pipefail
cd ~/dev_ws/f1tenth_more
HERE=scripts/fix_batch_5
H=scripts/jazzy_parity/phase5_bringup.sh
OUT=output/fix_batch_5/watchdog_live
export ROS_DOMAIN_ID=85 ROS_DISCOVERY_SERVER=127.0.0.1:11811
mkdir -p "$OUT"

wait_ready() {  # RUN_DIR
  for _ in $(seq 1 120); do [ -e "$1/READY" ] && return 0; sleep 2; done
  echo "never READY: $1"; return 1
}
stamp() { date +%s.%N; }

for t in ${TESTS:-normal failing sim}; do
  R=$OUT/$t; mkdir -p "$R"
  case $t in
  normal)
    HOLD_SEC=${HOLD_NORMAL:-760} bash $H hold "$R" > "$R.out" 2>&1 &
    HP=$!
    wait_ready "$R" || { wait $HP; continue; }
    python3 $HERE/health_recorder.py --out "$R/health.jsonl" > "$R/recorder.log" 2>&1 &
    REC=$!
    stamp > "$R/t_ready.txt"
    # /clock: nothing may publish or subscribe to it in this run.
    ROS_SUPER_CLIENT=TRUE ros2 topic info /clock --no-daemon --spin-time 5 > "$R/clock_info.txt" 2>&1
    python3 $HERE/rx_probe.py --out "$R/slam_map_rx.json" /slam/map:nav_msgs/msg/OccupancyGrid \
      > "$R/rx_probe.log" 2>&1 &
    RX=$!
    sleep "${NORMAL_SEC:-600}"
    stamp > "$R/t_test1_end.txt"
    kill -INT $RX; wait $RX
    # (2) SIGSTOP swept_clearance_node.
    swept_pids() { ps -eo pid,args | awk '/__node:=swept_clearance_node/ && !/awk/ {print $1}'; }
    PID=$(swept_pids | head -1)
    echo "$PID" > "$R/sigstop_pid.txt"
    stamp > "$R/t_sigstop.txt"
    kill -STOP "$PID"
    sleep 60
    { kill -0 "$PID" 2>/dev/null && echo "old pid $PID still exists" || echo "old pid $PID gone"
      echo "new pid: $(swept_pids | tr '\n' ' ')"; } > "$R/sigstop_after.txt"
    # Feed stopped: EKF stamps, and every component UPSTREAM_STALE.
    FEED=$(pgrep -f 'restamp_play.py' | head -1)
    python3 $HERE/stamp_probe.py --duration 12 --bin 0.5 /odometry/filtered \
      /ekf_global/odometry/filtered /odom > "$R/ekf_stamps.json" 2> "$R/ekf_stamps.err" &
    PR=$!
    sleep 4; stamp > "$R/t_feed_stop.txt"; kill -TERM "$FEED"
    wait $PR
    sleep 20
    stamp > "$R/t_end.txt"
    wait $HP
    kill -INT $REC; wait $REC
    ;;
  failing)
    cat > "$R/health_patch.yaml" <<'Y'
health:
  swept_clearance:
    checks:
      - {topic: /perception/swept_clearance/never, max_age_sec: 0.5, when_fresh: {/scan: 0.5}}
health_topic_types:
  /perception/swept_clearance/never: std_msgs/msg/Float32
Y
    COMPONENTS_ARGS="--empty hardware perception --health-patch $R/health_patch.yaml" \
      HOLD_SEC=${HOLD_FAILING:-180} bash $H hold "$R" > "$R.out" 2>&1 &
    HP=$!
    wait_ready "$R" || { wait $HP; continue; }
    python3 $HERE/health_recorder.py --out "$R/health.jsonl" > "$R/recorder.log" 2>&1 &
    REC=$!
    stamp > "$R/t_ready.txt"
    wait $HP
    kill -INT $REC; wait $REC
    ;;
  sim)
    HOLD_SEC=${HOLD_SIM:-240} bash $H simhold "$R" > "$R.out" 2>&1 &
    HP=$!
    wait_ready "$R" || { wait $HP; continue; }
    python3 $HERE/health_recorder.py --out "$R/health.jsonl" > "$R/recorder.log" 2>&1 &
    REC=$!
    stamp > "$R/t_ready.txt"
    sleep 90
    stamp > "$R/t_pause.txt"
    ROS_SUPER_CLIENT=TRUE ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause > "$R/pause.log" 2>&1
    sleep 30
    stamp > "$R/t_resume.txt"
    ROS_SUPER_CLIENT=TRUE ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > "$R/resume.log" 2>&1
    wait $HP
    kill -INT $REC; wait $REC
    ;;
  esac
done
