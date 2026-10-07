#!/bin/bash
# Phase 5d D1a spike: build the throwaway option-C image and run gates G1-G5
# in it. Usage:
#   docker/orin/spike/spike.sh build            # background-friendly, logs to output/phase5d_logs/
#   docker/orin/spike/spike.sh run <cmd...>     # one container, gate scripts at /spike/gates
#   docker/orin/spike/spike.sh guard            # print the guard's view and exit
#
# Every docker call here uses host networking: the Orin's dockerd runs with
# bridge:none / iptables:false (the custom 5.15.185-tegra kernel cannot load
# bridge.ko / nf_tables.ko), so a container or build step without
# --network=host has no network at all (CLAUDE.md, "Orin Docker").
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
IMAGE=${SPIKE_IMAGE:-f1tenth:jazzy-orin-spike}
DL=${SPIKE_DL:-/scratch/fabiocar/p5d_downloads}
SRC=${SPIKE_SRC:-/scratch/fabiocar/p5d_spike_ctx/src}
L4T=/usr/lib/aarch64-linux-gnu/nvidia
LOGS=$REPO/output/phase5d_logs
CONTAINER_PREFIX=f1tenth_spike

# Hard rule 2: the Humble stack and a Jazzy container never run together.
# Scans /proc (never pgrep -f, which matches this script's own command line).
# Processes inside this spike's own containers are not a conflict (the DDS
# probe runs two spike containers at once); anything else is.
in_spike_container() {
  local id
  for id in $spike_ids; do grep -q "$id" "/proc/$1/cgroup" 2>/dev/null && return 0; done
  return 1
}
guard() {
  local hits=() pid cmd spike_ids
  spike_ids=$(docker ps -q --no-trunc --filter "name=^${CONTAINER_PREFIX}")
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [ "$pid" = "$$" ] && continue
    cmd=$( { tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null) || continue
    [ -n "$spike_ids" ] && in_spike_container "$pid" && continue
    case "$cmd" in
      *spike.sh*|*docker\ *|*containerd*) continue ;;
      */opt/ros/humble/*|*dev_ws/f1tenth_more/install*|*component_supervisor_node*|*_ros2_daemon*|*ros2cli.daemon*|*fast-discovery-server*|*fastdds\ discovery*|*llama-server*|*"ros2 launch"*|*"ros2 run"*|*"ros2 bag"*)
        hits+=("$pid: ${cmd:0:120}") ;;
    esac
  done
  local binder
  for binder in $(ss -Hulpn 'sport = :11811' 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2); do
    [ -n "$spike_ids" ] && in_spike_container "$binder" && continue
    hits+=("UDP 11811 is bound by pid $binder")
  done
  if ss -Hulpn 'sport = :11811' 2>/dev/null | grep -q . && ! ss -Hulpn 'sport = :11811' | grep -q 'pid='; then
    hits+=("UDP 11811 is bound by a process of another user")
  fi
  local other
  other=$(docker ps --format '{{.Names}} {{.Image}}' | grep -E ' f1tenth:' | grep -v "^${CONTAINER_PREFIX}" || true)
  [ -n "$other" ] && hits+=("another f1tenth container runs: $other")
  if [ ${#hits[@]} -gt 0 ]; then
    echo "guard: REFUSING, a ROS stack or Discovery Server is up on the host:" >&2
    printf '  %s\n' "${hits[@]}" >&2
    return 1
  fi
  echo "guard: host clear (no Humble/ROS process, UDP 11811 free, no other f1tenth container)"
}

case "${1:-}" in
  build)
    mkdir -p "$LOGS"
    log="$LOGS/spike_build_$(date +%Y%m%d_%H%M%S).log"
    echo "build log: $log"
    docker build --network=host --progress=plain \
      --build-context dl="$DL" --build-context src="$SRC" --build-context l4t="$L4T" \
      ${SPIKE_TARGET:+--target "$SPIKE_TARGET"} \
      ${SPIKE_ZED_RUN:+--build-arg ZED_RUN="$SPIKE_ZED_RUN"} \
      -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE" > "$log" 2>&1
    echo "build OK: $(docker image inspect "$IMAGE" --format '{{.Id}} {{.Size}}')" | tee -a "$log"
    ;;
  run)
    shift
    guard
    hidraw_major=$(awk '$2=="hidraw"{print $1}' /proc/devices)
    mkdir -p "$LOGS/spike"
    exec docker run --rm --name "${CONTAINER_PREFIX}_$$" \
      --runtime nvidia --network host --ipc host \
      --user "$(id -u):$(id -g)" \
      --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)" \
      --group-add "$(getent group dialout | cut -d: -f3)" --group-add "$(getent group zed | cut -d: -f3)" \
      -e HOME=/tmp/home -e YOLO_CONFIG_DIR=/tmp/home/ultralytics -e YOLO_OFFLINE=True \
      -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
      -v /dev:/dev \
      --device-cgroup-rule 'c 81:* rmw' --device-cgroup-rule 'c 189:* rmw' \
      ${hidraw_major:+--device-cgroup-rule "c ${hidraw_major}:* rmw"} \
      -v "$HERE/gates:/spike/gates:ro" \
      -v "$LOGS/spike:/spike/out" \
      -v "$REPO/docker/orin/thor_capture:/spike/thor:ro" \
      -v "$REPO/src/f1tenth_perception/models:/spike/models:ro" \
      -v "${SPIKE_IMAGES:-/home/fabiocar/f1tenth_archive/verify/bottle_2026-09-02T15-27-43_frames}:/spike/images:ro" \
      ${SPIKE_EXTRA_ARGS:-} \
      "$IMAGE" "$@"
    ;;
  guard)
    guard
    ;;
  *)
    sed -n '2,12p' "$0"; exit 2 ;;
esac
