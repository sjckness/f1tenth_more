#!/usr/bin/env bash
# Run the review-item-6 scenario harness against the REAL mpc_corr node, on
# linus, without perception or Gazebo, on an isolated ROS domain.
#   tools/run_scenarios.sh [--true-centre] [--only NAME ...] [--out FILE]
# Needs: source install/setup.bash (and python3-osqp for mpc_corr).
set -eo pipefail
export ROS_DOMAIN_ID=${SCENARIO_DOMAIN_ID:-87}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
WORK=$(mktemp -d)
cd "$WORK"
PY=${PYTHON:-python3}
$PY -c "from mpc_controller.MPC_corr import main; main()" \
    --ros-args -p "model_log_path:=''" --log-level mpc_corr:=warn > "$WORK/mpc_corr.log" 2>&1 &
MPC=$!
trap 'kill -INT $MPC 2>/dev/null; wait $MPC 2>/dev/null; echo "mpc_corr log: $WORK/mpc_corr.log"' EXIT
sleep 3
$PY -c "from mpc_controller.campaign_scenario import main; main()" "$@"
