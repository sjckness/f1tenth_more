# Source me (bash): ROS Jazzy + Hydra workspace + this f1tenth workspace + spark_dsg python.
#   source scripts/hydra/hydra_env.sh
# HYDRA_WS (default ~/hydra_ws). Network variables (ROS_DOMAIN_ID, Discovery Server, Fast DDS
# profile) are NOT set here: use scripts/sim/sim_isaac.sh run|shell for the sim graph.
_f1t_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_hws="${HYDRA_WS:-$HOME/hydra_ws}"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f "$_hws/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_hws/install/local_setup.bash"
else
  echo "hydra_env.sh: no $_hws/install (scripts/hydra/build_hydra_ws.sh)" >&2
fi
if [ -f "$_f1t_repo/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_f1t_repo/install/local_setup.bash"
else
  echo "hydra_env.sh: no $_f1t_repo/install (colcon build --packages-select f1tenth_hydra)" >&2
fi
[ -d "$_hws/install/spark_dsg_python" ] && export PYTHONPATH="$_hws/install/spark_dsg_python:$PYTHONPATH"
unset _f1t_repo _hws
