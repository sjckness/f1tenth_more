#!/usr/bin/env bash
# A shell in the running container, ROS sourced. A thin wrapper around
# scripts/docker_orin.sh shell (its /etc/bash.bashrc already sources
# scripts/env/jazzy.sh for `docker exec ... bash` -- see docker/orin/entrypoint.sh).
#
#   scripts/orin/container_shell.sh [CMD ...]
#
# No args: an interactive shell. With args: that command, through
# entrypoint.sh, non-interactively.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

echo "== preflight (informational) ==" >&2
"$HERE/preflight.sh" >&2 || true

exec "$REPO/scripts/docker_orin.sh" shell "$@"
