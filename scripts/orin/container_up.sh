#!/usr/bin/env bash
# Switch the Orin into container mode for a live stack run. A thin wrapper:
# all the native<->container switch logic (killing the native stack, stopping
# the native gateway, the drift/guard checks, starting the car-container
# systemd unit or docker_orin.sh up directly) already lives in
# docker/orin/host/stack_mode.sh and scripts/docker_orin.sh -- reused here,
# not duplicated.
#
#   scripts/orin/container_up.sh [--archive separate|shared] [--allow-drift] [--dry-run]
#
# Runs preflight.sh first (ZED/lidar/VESC) and refuses to switch if it fails.
# Starts the container with F1T_AUTOSTART=none (stack_mode.sh's own default):
# the ROS stack itself is NOT launched here -- run stack_live.sh next.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

echo "== preflight =="
"$HERE/preflight.sh"

echo
echo "== native -> container =="
"$REPO/docker/orin/host/stack_mode.sh" container "$@"

echo
echo "== status =="
"$REPO/scripts/docker_orin.sh" status || true

echo
echo "Container is up, stack NOT started (autostart=none). Next:"
echo "  scripts/orin/stack_live.sh"
