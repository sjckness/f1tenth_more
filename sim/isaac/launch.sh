#!/bin/bash
# Launch the F1Tenth Isaac Sim scene (sim/isaac of the car repo; vendored from ~/isaac_ws,
# branch f1tenth, commit 4d2de1e, so a plain `git clone` of this repository is enough).
#   ./launch.sh --scene base_room [--gui] [--rtf-log] [--ros-distro jazzy|humble] [--dds-profile FILE.xml]
# On the host it starts the container (docker compose run); inside the container
# it runs sim/main.py with the ROS 2 bridge environment of the chosen distro.
#   --ros-distro   internal bridge libraries to use (both ship in the image). Default: jazzy
#                  (the stack on linus and the Thor is Jazzy; humble = the Orin's distro).
#   --dds-profile  Fast DDS XML profile on the host (default: config/fastdds_udp.xml). Its
#                  directory is mounted read-only at /dds in the container.
# Pass-through environment (if set on the host): ROS_DISCOVERY_SERVER, ROS_AUTOMATIC_DISCOVERY_RANGE,
# ROS_STATIC_PEERS, ROS_DOMAIN_ID (default: config/ros_interface.yaml ros.domain_id).
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

domain_id() {  # ros_interface.yaml -> ros.domain_id (default 42)
    local f="$1/config/ros_interface.yaml"
    local d
    d=$(grep -E '^\s*domain_id:' "$f" 2>/dev/null | head -1 | awk '{print $2}')
    echo "${d:-42}"
}

# --ros-distro / --dds-profile are consumed here; everything else goes to sim/main.py.
ROS_DISTRO_ARG="${ISAAC_ROS_DISTRO:-jazzy}"
DDS_PROFILE_ARG="${ISAAC_DDS_PROFILE:-}"
SIM_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --ros-distro) ROS_DISTRO_ARG="${2:?--ros-distro needs humble|jazzy}"; shift 2 ;;
        --ros-distro=*) ROS_DISTRO_ARG="${1#*=}"; shift ;;
        --dds-profile) DDS_PROFILE_ARG="${2:?--dds-profile needs a file}"; shift 2 ;;
        --dds-profile=*) DDS_PROFILE_ARG="${1#*=}"; shift ;;
        *) SIM_ARGS+=("$1"); shift ;;
    esac
done
case "$ROS_DISTRO_ARG" in humble|jazzy) ;; *) echo "launch.sh: --ros-distro must be humble or jazzy, got '$ROS_DISTRO_ARG'" >&2; exit 2 ;; esac

if [ ! -x /isaac-sim/python.sh ]; then
    # ---- host side ----
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(domain_id "$WS")}"
    export ISAAC_ROS_DISTRO="$ROS_DISTRO_ARG"
    if [ -n "$DDS_PROFILE_ARG" ]; then
        [ -f "$DDS_PROFILE_ARG" ] || { echo "launch.sh: DDS profile not found: $DDS_PROFILE_ARG" >&2; exit 2; }
        DDS_ABS="$(cd "$(dirname "$DDS_PROFILE_ARG")" && pwd)/$(basename "$DDS_PROFILE_ARG")"
        export ISAAC_DDS_DIR="$(dirname "$DDS_ABS")" ISAAC_DDS_FILE="$(basename "$DDS_ABS")"
    fi
    NAME="${ISAAC_CONTAINER_NAME:-f1t_isaac}"
    TTY=()
    [ -t 0 ] || TTY=(-T)
    # Paths the compose file needs (nothing outside this repository is assumed).
    export F1T_ISAAC_DIR="$WS"
    export F1T_REPO="${F1T_REPO:-$(cd "$WS/../.." && pwd)}"
    export F1T_ISAAC_IMAGE="${F1T_ISAAC_IMAGE:-f1tenth_isaac:5.1.0}"
    exec docker compose -f "$WS/docker/docker-compose.yaml" run --rm "${TTY[@]}" \
        --name "$NAME" isaac-sim /home/isaac_ws/launch.sh --ros-distro "$ROS_DISTRO_ARG" "${SIM_ARGS[@]}"
fi

# ---- inside the container ----
export ROS_DISTRO="$ROS_DISTRO_ARG"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(domain_id /home/isaac_ws)}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/isaac_ws/config/fastdds_udp.xml}"
# empty pass-through variables mean "not set" (an empty ROS_DISCOVERY_SERVER is not "no server" everywhere)
for v in ROS_DISCOVERY_SERVER ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_SUPER_CLIENT; do
    [ -z "${!v:-}" ] && unset "$v"
done
BRIDGE_LIB=/isaac-sim/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib
# drop the other distro's bridge libs if the compose default put them on the path
LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | { grep -v 'isaacsim.ros2.bridge/' || true; } | paste -sd: -)"
case ":${LD_LIBRARY_PATH:-}:" in *":$BRIDGE_LIB:"*) ;; *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$BRIDGE_LIB";; esac
# Isaac uses Python 3.11: never let host ROS (3.10/3.12) paths leak in.
export PYTHONPATH="$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: -)"

# Files written by the (root) container must stay editable by the host user.
OWNER="$(stat -c '%u:%g' /home/isaac_ws)"
trap 'chown -R "$OWNER" /home/isaac_ws/output 2>/dev/null || true' EXIT

# python.sh does not exec: forward SIGTERM/SIGINT (docker stop) to the Python process only, then wait for
# it to finish its clean shutdown (stats file, ROS node, app.close()).
echo "launch.sh: ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-<unset>} ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
/isaac-sim/python.sh /home/isaac_ws/sim/main.py "${SIM_ARGS[@]}" &
SIM_PID=$!
term() { pkill -TERM -f "bin/python3 .*/home/isaac_ws/sim/main.py" || true; }
trap term TERM INT
while kill -0 "$SIM_PID" 2>/dev/null; do wait "$SIM_PID" || true; done
