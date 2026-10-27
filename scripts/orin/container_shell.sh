#!/usr/bin/env bash
# A shell in the running container, ROS sourced. A thin wrapper around
# scripts/docker_orin.sh shell (its /etc/bash.bashrc already sources
# scripts/env/jazzy.sh for `docker exec ... bash` -- see docker/orin/entrypoint.sh).
#
#   scripts/orin/container_shell.sh [CMD ...]
#   scripts/orin/container_shell.sh llm ["PROMPT"]
#
# No args: an interactive shell. With args: that command, through
# entrypoint.sh, non-interactively. `llm` is a shortcut for the one-shot
# command most often run by hand against an already-live stack:
#   llm   ros2 run llm llm_planner_node ["PROMPT"]  -- llama-server
#         (llm.launch.py, part of components.yaml) is already running as a
#         stack component; this is just the one-shot CLI client. No PROMPT:
#         interactive, reads stdin (llm_planner_node's own mode).
# It must run inside the container: load_mission/start_mission hand off
# through a FILE (install/share/.../missions/llm_generated/<id>.json), and
# the container's copy of that path is the ONLY one bind-mounted back to
# $F1T_STATE (compose.yaml). A native llm_planner_node writes to ITS OWN,
# different install path, which mission_manager inside the container never
# sees -- that mismatch is exactly "sequence size exceeds remaining buffer"
# / load_mission ENOENT (see README).
#
# Deliberately no `logger` shortcut here. test_campaign_logger
# (src/f1tenth_logger/TEST_CAMPAIGN.md) writes to <f1tenth_more
# checkout>/first_test_campaign, a git-tracked, native-only path with no
# matching bind mount into this container (compose.yaml has none for it):
# running it through docker exec would write into the container's
# ephemeral layer and lose the campaign on the next `down`. Run it natively
# against this same live stack instead (shared ROS_DISCOVERY_SERVER/
# ROS_DOMAIN_ID over host networking, so it sees the container's topics
# fine):
#   source scripts/env/jazzy.sh && ros2 launch f1tenth_logger test_campaign_logger.launch.py
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
    echo "container_shell.sh: no 'logger' shortcut -- test_campaign_logger writes to" >&2
    echo "  <f1tenth_more checkout>/first_test_campaign, which has no bind mount into" >&2
    echo "  this container (see this script's own header). Run it natively instead:" >&2
    echo "    source scripts/env/jazzy.sh && ros2 launch f1tenth_logger test_campaign_logger.launch.py" >&2
    exit 1 ;;
esac

echo "== preflight (informational) ==" >&2
"$HERE/preflight.sh" >&2 || true

exec "$REPO/scripts/docker_orin.sh" shell "$@"
