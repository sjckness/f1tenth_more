# PROPOSED scripts/env/jazzy.sh -- NOT applied (Phase 5, Step 2: awaiting OK).
#
# Jazzy environment for the f1tenth_more workspace, for interactive shells on
# a Jazzy machine (Thor today; the Jazzy container on the Orin later).
# Replaces the bare `source /opt/ros/jazzy/setup.bash` in ~/.bashrc.
#
# The stack itself does not need any of this: supervisor_bringup.launch.py and
# stack_bringup.launch.py start the Discovery Server and set
# ROS_DISCOVERY_SERVER for every process they spawn. This is for everything
# started BY HAND (ros2 CLI, ros2 run/launch, rosbag, scripts/stackctl.py):
# without it, a hand-started participant uses simple discovery and sees none
# of the stack (Phase 5, Step 3: a non-DS participant lists 0 nodes).

source /opt/ros/jazzy/setup.bash
_f1tenth_ws=${F1TENTH_WS:-$HOME/dev_ws/f1tenth_more}
if [ -f "$_f1tenth_ws/install/setup.bash" ]; then
  source "$_f1tenth_ws/install/setup.bash"
fi
unset _f1tenth_ws

# The Discovery Server is a Fast DDS feature. rmw_fastrtps_cpp is Jazzy's
# default already; set explicitly so a changed default cannot silently turn
# ROS_DISCOVERY_SERVER into a no-op.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Production domain: nothing in the repo sets ROS_DOMAIN_ID, so the stack runs
# on 0. Keep an already-exported value (test harnesses set their own).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Discovery Server client. Must match stack_params.yaml's
# discovery_server_address/_port (127.0.0.1:11811). For the two-machine sim
# setup this becomes the Thor's LAN address, on BOTH machines.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Super client: a plain DS client only learns endpoints that match its own,
# so `ros2 topic list` as a plain client shows /parameter_events and /rosout
# only (Phase 5, Step 3). A super client sees the whole graph. Costs extra
# discovery traffic for every hand-started node; acceptable for a dev shell.
export ROS_SUPER_CLIENT=TRUE

# ROS_AUTOMATIC_DISCOVERY_RANGE is left at Jazzy's default (SUBNET, set by
# ros_environment). With a Discovery Server it made no difference in Step 3
# (SUBNET and LOCALHOST gave identical graphs).

# The ros2 daemon keeps the environment it was started with. After changing
# any of the above, run `ros2 daemon stop` (or use --no-daemon).

# Escape hatch for an ad-hoc simple-discovery session (e.g. a bag replay on an
# isolated domain, the way scripts/jazzy_parity/*.sh do it):
#   f1tenth_simple_discovery 77
f1tenth_simple_discovery() {
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  export ROS_DOMAIN_ID=${1:-$ROS_DOMAIN_ID} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ros2 daemon stop >/dev/null 2>&1
  echo "simple discovery, localhost only, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
}
