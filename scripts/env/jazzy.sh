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

# ROS_SUPER_CLIENT is deliberately NOT exported. The stack is launched from
# this shell, and every node inherits its environment: exported here, every
# node would become a super client and receive the whole graph's discovery
# traffic, which defeats the Discovery Server. Only the nodes that need the
# whole graph set it in their own launch file (foxglove_bridge.launch.py,
# mission_logger.launch.py).
#
# A plain DS client only learns endpoints that match its own, so
# `ros2 topic list` as a plain client shows /parameter_events and /rosout
# only (Phase 5, Step 3). For introspection use ros2cli, which runs one ros2
# command as a super client:
#   ros2cli topic list
#   ros2cli node info /mpc_corr
# For the subcommands that read the graph (those that accept --no-daemon and
# --spin-time in Jazzy's ros2cli), two flags are added:
#   --no-daemon: the ros2 daemon keeps the environment it was started with,
#     so a daemon started by a plain `ros2` call would answer as a plain
#     client;
#   --spin-time 5 (ROS2CLI_SPIN_TIME; not added if you pass --spin-time):
#     a new super client needs a few seconds to be told the whole graph.
#     Measured on Thor against the full stack: the default and 1-2 s
#     returned the full graph in only some tries, 3 s and 5 s in 3 of 3
#     (output/fix_batch_2/ros2cli_spin_check.txt).
# The other subcommands (topic hz/pub/bw, service call, ...) build their own
# node and only need the super-client variable.
ros2cli() {
  case "$1 $2" in
    "node list"|"node info"|"topic list"|"topic info"|"topic echo"|"topic find"|\
    "topic type"|"service list"|"service info"|"param "*)
      local extra=(--no-daemon)
      case " $* " in *" --spin-time "*) ;; *) extra+=(--spin-time "${ROS2CLI_SPIN_TIME:-5}") ;; esac
      ROS_SUPER_CLIENT=TRUE ros2 "$@" "${extra[@]}" ;;
    *)
      ROS_SUPER_CLIENT=TRUE ros2 "$@" ;;
  esac
}

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
