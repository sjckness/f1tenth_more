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
# The guard lives in docker/orin/lib_guard.sh (shared with the D1 scripts);
# this spike's own containers (the DDS probe runs two) are not a conflict.
source "$HERE/../lib_guard.sh"
guard() { GUARD_OWN_PREFIX=$CONTAINER_PREFIX orin_guard; }

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
