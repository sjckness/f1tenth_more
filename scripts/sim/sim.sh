#!/usr/bin/env bash
# sim.sh -- one-command Gazebo sim-host bringup for the two-machine sim.
# Run on the SIM HOST (linus). Sets the ROS network env (see sim_net.sh) and
# launches f1tenth_sim, so you never type exports or the launch line.
#
# Usage:
#   scripts/sim/sim.sh                 # headless, camera on, default spawn
#   scripts/sim/sim.sh gui:=true       # with the Gazebo GUI
#   scripts/sim/sim.sh yaw:=0          # face +x (at the office furniture)
#   scripts/sim/sim.sh --clean         # kill_ros2.py -y first (clean slate), then launch
#   scripts/sim/sim.sh --dry-run       # print the env + command, launch nothing
# Any ros2-launch arg for sim_bringup.launch.py is passed through and, if it
# repeats a default below, overrides it (ros2 launch: last value wins).
#
# Network overrides (export before running): F1TENTH_SIM_DISCOVERY_IP,
# F1TENTH_SIM_DISCOVERY_PORT, ROS_DOMAIN_ID -- see sim_net.sh.
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

CMD=(ros2 launch f1tenth_sim sim_bringup.launch.py gui:=false camera:=true "$@")
echo "== sim host =="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo "cmd: ${CMD[*]}"
if $DRY; then echo "(dry run)"; exit 0; fi
if $CLEAN; then python3 "$REPO/scripts/kill_ros2.py" -y; fi
exec "${CMD[@]}"
