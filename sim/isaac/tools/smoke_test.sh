#!/bin/bash
# Phase 7/8 automated smoke test (run on the HOST).
#   tools/smoke_test.sh [scene] [checks...]
#   scene  : base_room (default) | office | ...
#   checks : default "rates messages geometry drive reset rtf"
# Starts the sim headless in the Isaac container, runs the ROS checks with the same ROS distro as the
# sim's bridge (SIM_ROS_DISTRO, default jazzy: the host's /opt/ros/jazzy; humble: a throw-away
# f1tenth_more:cpu container), same domain, UDP-only FastDDS. Samples VRAM/CPU/RAM, saves PNGs,
# then stops the sim cleanly. Results: output/runs/<scene>_<timestamp>/ and a summary on stdout.
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENE="${1:-base_room}"; shift || true
CHECKS="${*:-rates messages geometry drive reset rtf}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN="$WS/output/runs/${SCENE}_${STAMP}"
NAME="f1t_smoke_${SCENE}"
HUMBLE_IMAGE="${HUMBLE_IMAGE:-f1tenth_more:cpu}"
SIM_ROS_DISTRO="${SIM_ROS_DISTRO:-jazzy}"
if [ "$SIM_ROS_DISTRO" = humble ]; then RWS=/ws; else RWS="$WS"; fi   # workspace path as the checker sees it
DOMAIN="$(grep -E '^\s*domain_id:' "$WS/config/ros_interface.yaml" | head -1 | awk '{print $2}')"
mkdir -p "$RUN"
log() { echo "[$(date +%T)] $*" | tee -a "$RUN/smoke.log"; }

ros() {  # run a command with the checker's ROS 2 (same distro as the sim bridge)
    if [ "$SIM_ROS_DISTRO" = humble ]; then
        docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp \
            -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_udp.xml -e WS=/ws \
            -v "$WS:/ws" --entrypoint bash "$HUMBLE_IMAGE" -c "source /opt/ros/humble/setup.bash && $*"
    else
        env -i HOME="$HOME" PATH=/usr/bin:/bin USER="$USER" ROS_LOG_DIR=/tmp/ros_checks_log \
            ROS_DOMAIN_ID="$DOMAIN" RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            FASTRTPS_DEFAULT_PROFILES_FILE="$WS/config/fastdds_udp.xml" WS="$WS" \
            bash -c "source /opt/ros/jazzy/setup.bash && $*"
    fi
}

cleanup() {
    [ -n "${MON_PID:-}" ] && kill "$MON_PID" 2>/dev/null
    if docker ps -q -f "name=^${NAME}$" | grep -q .; then
        log "stopping sim container $NAME"
        docker stop -t 40 "$NAME" >/dev/null 2>&1
    fi
}
trap cleanup EXIT

docker rm -f "$NAME" >/dev/null 2>&1
VRAM0=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
log "scene=$SCENE distro=$SIM_ROS_DISTRO domain=$DOMAIN checks=[$CHECKS] baseline VRAM ${VRAM0} MiB, disk $(df -h / | tail -1 | awk '{print $4}') free"
# extra sim args via SIM_ARGS, e.g. SIM_ARGS="--set sensors.stereo.semantic.enabled=true"
ISAAC_CONTAINER_NAME="$NAME" nohup "$WS/launch.sh" --ros-distro "$SIM_ROS_DISTRO" --scene "$SCENE" --rtf-log ${SIM_ARGS:-} \
    --stats-file "/home/isaac_ws/output/runs/${SCENE}_${STAMP}/sim_stats.json" > "$RUN/sim.log" 2>&1 &
T0=$(date +%s)

# resource monitor: VRAM (whole GPU), container CPU / RAM, every 2 s
(
    echo "t_s,vram_mib,cpu_pct,mem"
    while true; do
        v=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
        s=$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}}' "$NAME" 2>/dev/null | sed 's/ \/ .*//')
        echo "$(( $(date +%s) - T0 )),$v,${s:-,}"
        sleep 2
    done
) > "$RUN/resources.csv" &
MON_PID=$!

log "waiting for /clock ..."
if ! ros python3 "$RWS/tools/ros_checks.py" wait --scene "$SCENE" --out "$RWS/output/runs/${SCENE}_${STAMP}" --timeout 900 \
        >> "$RUN/smoke.log" 2>&1; then
    log "FAIL: sim did not come up; last sim log lines:"; tail -30 "$RUN/sim.log" | tee -a "$RUN/smoke.log"
    exit 1
fi
log "sim up after $(( $(date +%s) - T0 )) s"

# 1. topic list (CLI)
ros "ros2 topic list -t" > "$RUN/topic_list.txt" 2>&1
log "ros2 topic list: $(wc -l < "$RUN/topic_list.txt") topics"
# cross-check one rate with the CLI tool
ros "timeout 12 ros2 topic hz /scan --window 200" > "$RUN/hz_scan_cli.txt" 2>&1 || true
ros "timeout 15 ros2 topic echo --once /zed2/zed_node/left/camera_info" > "$RUN/echo_camera_info.txt" 2>&1 || true
ros "timeout 15 ros2 topic echo --once --no-arr /scan" > "$RUN/echo_scan.txt" 2>&1 || true

RESULT=0
for c in $CHECKS; do
    log "check: $c"
    if ros python3 "$RWS/tools/ros_checks.py" "$c" --scene "$SCENE" --out "$RWS/output/runs/${SCENE}_${STAMP}" \
            --seconds "$([ "$c" = rtf ] && echo 60 || echo 20)" 2>&1 | tail -1 | tee -a "$RUN/smoke.log" | grep -q "^\[PASS\]"; then
        :
    else
        RESULT=1
    fi
done

kill "$MON_PID" 2>/dev/null; MON_PID=""
PEAK=$(awk -F, 'NR>1 && $2>m {m=$2} END {print m}' "$RUN/resources.csv")
log "VRAM peak ${PEAK} MiB total GPU (baseline ${VRAM0} MiB) -> Isaac ~$(( PEAK - VRAM0 )) MiB"
echo "{\"vram_baseline_mib\": $VRAM0, \"vram_peak_mib\": $PEAK, \"vram_isaac_mib\": $(( PEAK - VRAM0 ))}" > "$RUN/vram.json"

cleanup
sleep 3
# plots (Isaac python has matplotlib)
if [ -f "$RUN/frames.npz" ]; then
    docker compose -f "$WS/.devcontainer/docker-compose.yaml" run --rm isaac-sim -c \
        "/isaac-sim/python.sh /home/isaac_ws/tools/save_frames.py /home/isaac_ws/output/runs/${SCENE}_${STAMP} /home/isaac_ws/output/runs/${SCENE}_${STAMP} && \
         /isaac-sim/python.sh /home/isaac_ws/tools/plot_scan.py /home/isaac_ws/output/runs/${SCENE}_${STAMP} /home/isaac_ws/config/scenes/${SCENE}.yaml /home/isaac_ws/config/car_spec.yaml /home/isaac_ws/output/runs/${SCENE}_${STAMP}/scan.png; \
         chown -R $(id -u):$(id -g) /home/isaac_ws/output" >> "$RUN/smoke.log" 2>&1
fi
log "done: result=$([ $RESULT = 0 ] && echo PASS || echo FAIL) -> $RUN"
exit $RESULT
