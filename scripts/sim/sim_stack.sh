#!/usr/bin/env bash
# sim_stack.sh -- one-command Jazzy stack bringup in sim mode.
# Run on the STACK HOST (the Thor). Sets the ROS network env (see sim_net.sh)
# and launches supervisor_bringup.launch.py sim:=true with the standard args,
# so you never type exports or the long launch line.
# (Named sim_stack.sh so a stack.sh for the REAL car can live alongside it.)
#
# Usage:
#   scripts/sim/sim_stack.sh                       # sim mode, intelligence on, watchdog alert
#   scripts/sim/sim_stack.sh health_watchdog:=enforce
#   scripts/sim/sim_stack.sh --clean               # kill_ros2.py -y first (clean slate), then launch
#   scripts/sim/sim_stack.sh --dry-run             # print the env + command, launch nothing
# Any ros2-launch arg for supervisor_bringup.launch.py is passed through and, if
# it repeats a default below, overrides it (ros2 launch: last value wins).
#
# The supervisor STARTS the Discovery Server and binds it to
# discovery_server_address (the Thor's LAN IP, taken from the network config),
# so linus can reach it. Network overrides: see sim_net.sh.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

CLEAN=false
DRY=false
while [[ "${1:-}" == --* ]]; do
  case "$1" in
    --clean)   CLEAN=true ;;
    --dry-run) DRY=true ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

# shellcheck disable=SC1091
source "$HERE/sim_net.sh"
DS_IP="${F1TENTH_DISCOVERY_SERVER%%:*}"   # the server binds to this; must be the Thor's LAN IP

CMD=(ros2 launch f1tenth_bringup supervisor_bringup.launch.py
     sim:=true discovery_server_address:="$DS_IP"
     enable_intelligence:=true health_watchdog:=alert "$@")
echo "== stack host =="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER  bind=$DS_IP"
echo "cmd: ${CMD[*]}"
if $DRY; then echo "(dry run)"; exit 0; fi
if $CLEAN; then python3 "$REPO/scripts/kill_ros2.py" -y; fi
exec "${CMD[@]}"
