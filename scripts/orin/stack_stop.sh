#!/usr/bin/env bash
# Stop the ROS stack cleanly, container left running. A thin wrapper around
# the container's own f1tenth_ctl (reused, not duplicated): `f1tenth_ctl stop
# stack` SIGINTs the managed stack process the same way Ctrl-C would (see
# CLAUDE.md "ros2 run does not forward SIGINT"), not a container stop.
#
#   scripts/orin/stack_stop.sh
#
# preflight.sh runs first for the record but never blocks a stop.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# ros2 run would also find this (install/lib/<pkg>/<exe> is on its index),
# but the full path skips ros2 run's own startup cost for a one-shot CLI call.
CTL=/opt/f1tenth_ws/install/lib/f1tenth_ctl/f1tenth_ctl
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

echo "== preflight (informational; never blocks a stop) =="
"$HERE/preflight.sh" || true

echo
echo "== stopping the stack =="
"$REPO/scripts/docker_orin.sh" exec "$CTL" stop stack < /dev/null

echo
echo "== status =="
"$REPO/scripts/docker_orin.sh" exec "$CTL" status --human < /dev/null
