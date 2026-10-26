#!/usr/bin/env bash
# Stop the container and return the Orin to native mode. A thin wrapper
# around docker/orin/host/stack_mode.sh native (reused, not duplicated),
# which stops the container (its systemd unit, or docker_orin.sh down
# directly), waits for the guard, and restarts the native gateway.
#
#   scripts/orin/container_down.sh [--dry-run]
#
# preflight.sh runs first for the record but never blocks a return to
# native: you must always be able to get back, regardless of what it finds.
# Verifies the result with stack_mode.sh status.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

echo "== preflight (informational; never blocks a return to native) =="
"$HERE/preflight.sh" || true

echo
echo "== container -> native =="
"$REPO/docker/orin/host/stack_mode.sh" native "$@"

echo
echo "== status (verify) =="
"$REPO/docker/orin/host/stack_mode.sh" status
