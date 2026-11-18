#!/bin/bash
# Orin Jazzy image entrypoint: every command runs in the environment the Thor's
# interactive shells get from ~/.bashrc, i.e. scripts/env/jazzy.sh (Jazzy +
# the workspace install, rmw_fastrtps_cpp, the 5 s lease profile, ROS_DOMAIN_ID
# default 0, ROS_DISCOVERY_SERVER from F1TENTH_DISCOVERY_SERVER or
# 127.0.0.1:11811). The launch files set the stack's own environment as on the
# Thor. /etc/bash.bashrc sources the same file for `docker exec ... bash`.
source /opt/f1tenth_ws/scripts/env/jazzy.sh
exec "$@"
