#!/usr/bin/env bash
# clean.sh -- wipe all ROS 2 processes + runtime state on THIS machine.
# Thin wrapper around scripts/kill_ros2.py so "clear the machine" is one word on
# either host. kill_ros2.py escalates SIGINT->SIGTERM->SIGKILL and then purges
# the state that silently breaks the NEXT launch: Fast DDS /dev/shm segments and
# the supervisor singleton/pgid files (leftover SHM presents as "foxglove shows
# nothing", not as a stale-file error -- so a plain pkill is NOT enough).
#
# Usage:
#   scripts/sim/clean.sh            # kill everything, no prompt
#   scripts/sim/clean.sh -n         # dry run: list what would be killed
#   scripts/sim/clean.sh -t 10      # 10s grace per signal (default 5)
# Run this before a fresh sim.sh / stack.sh when a previous run left orphans
# (or just use sim.sh/stack.sh --clean, which calls this same path).
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
# Default to -y (no prompt) unless the caller passes their own flags.
if [[ $# -eq 0 ]]; then set -- -y; fi
exec python3 "$REPO/scripts/kill_ros2.py" "$@"
