#!/usr/bin/env bash
# A shell in the running container, ROS sourced. A thin wrapper around
# scripts/docker_orin.sh shell (its /etc/bash.bashrc already sources
# scripts/env/jazzy.sh for `docker exec ... bash` -- see docker/orin/entrypoint.sh).
#
#   scripts/orin/container_shell.sh [CMD ...]
#   scripts/orin/container_shell.sh llm ["PROMPT"]
#   scripts/orin/container_shell.sh logger [EXTRA_LAUNCH_ARGS ...]
#
# No args: an interactive shell. With args: that command, through
# entrypoint.sh, non-interactively. `llm` and `logger` are shortcuts for the
# two one-shot commands most often run by hand against an already-live stack:
#   llm     ros2 run llm llm_planner_node ["PROMPT"]  -- llama-server
#           (llm.launch.py, part of components.yaml) is already running as a
#           stack component; this is just the one-shot CLI client. No
#           PROMPT: interactive, reads stdin (llm_planner_node's own mode).
#   logger  ros2 launch f1tenth_logger mission_logger.launch.py
#           enable_mission_logger:=true [EXTRA_LAUNCH_ARGS ...]  -- the
#           component supervisor only starts mission_logger_node when
#           enable_mission_logger is already true; this launches it
#           standalone for an ad hoc recording.
# Both run inside the container, so they share the stack's own install (not
# a separate workspace) and its ROS_DISCOVERY_SERVER/ROS_DOMAIN_ID -- the
# exact mismatch that made `ros2 run llm llm_planner_node` fail from a host
# shell ("sequence size exceeds remaining buffer"; load_mission ENOENT on
# the llm_generated path, a different, stale workspace -- see README).
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

case "${1:-}" in
  llm)
    shift
    set -- ros2 run llm llm_planner_node "$@" ;;
  logger)
    shift
    set -- ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=true "$@" ;;
esac

echo "== preflight (informational) ==" >&2
"$HERE/preflight.sh" >&2 || true

exec "$REPO/scripts/docker_orin.sh" shell "$@"
