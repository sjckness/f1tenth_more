#!/usr/bin/env bash
# Run an F1TENTH dev container interactively.
# Usage:
#   ./scripts/docker_run.sh {cpu|gpu}
#   ./scripts/docker_run.sh {cpu|gpu} "<command to run instead of bash>"
# Example:
#   ./scripts/docker_run.sh gpu "ros2 launch f1tenth_bringup stack_bringup_launch.py"
set -euo pipefail

cd "$(dirname "$0")/.."   # repo root

TARGET="${1:-}"
shift || true

case "$TARGET" in
  cpu) SERVICE="f1tenth_cpu" ;;
  gpu) SERVICE="f1tenth_gpu" ;;
  *)
    echo "Usage: $0 {cpu|gpu} [command]" >&2
    exit 1
    ;;
esac

# Warn (don't fail) if the VESC serial device is missing on the host.
if [ ! -e /dev/ttyACM0 ]; then
  echo "WARNING: /dev/ttyACM0 not found on host. The VESC may be unplugged or" >&2
  echo "         on a different path (check: ls -l /dev/ttyACM*). Compose will" >&2
  echo "         fail to start until the device exists or the mapping is edited." >&2
fi

if [ "$#" -gt 0 ]; then
  # Run the user command through a shell so quoted multi-word commands work.
  # ROS is already sourced by the container entrypoint before this runs.
  exec docker compose run --rm "$SERVICE" bash -c "$*"
else
  exec docker compose run --rm "$SERVICE" bash
fi
