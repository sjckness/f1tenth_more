# Source from bash: the ROS environment of the tuning harness (same as scripts/sim/sim_isaac.sh's
# stack side on --net local), as a Discovery Server super client so introspection sees the graph.
#   ISAAC_TUNING_DOMAIN (default 78), ISAAC_TUNING_DS_PORT (default 11878): private, never the
#   car's 42 / 11811 (other sessions and the Thor's llm account use those).
# shellcheck shell=bash
_it_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_it_repo="$(cd "$_it_here/../../.." && pwd)"
set +eu
_it_extra="${F1TENTH_EXTRA_SETUP:-$HOME/dev_ws/jc_env.sh}"
# shellcheck disable=SC1090
[ -f "$_it_extra" ] && source "$_it_extra"
export ROS_DOMAIN_ID="${ISAAC_TUNING_DOMAIN:-78}"
export F1TENTH_DISCOVERY_SERVER="127.0.0.1:${ISAAC_TUNING_DS_PORT:-11878}"
# shellcheck disable=SC1091
source "$_it_repo/scripts/env/jazzy.sh"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER="$F1TENTH_DISCOVERY_SERVER"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export FASTRTPS_DEFAULT_PROFILES_FILE="$_it_repo/src/f1tenth_bringup/config/sim/dds/local.xml"
export ROS_SUPER_CLIENT=TRUE
unset _it_here _it_repo _it_extra
