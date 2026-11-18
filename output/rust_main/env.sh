# Build/test environment for branch main-jazzy-rust on linus (bash; source it).
#   source output/rust_main/env.sh; cd $WS
# WS is the worktree this file is in. Since 2026-10-24 (R0b) linus's ros-jazzy-* come from the
# ROS snapshot 2026-09-11 (tools/linus/ros_snapshot_move.sh): slam_toolbox, py_trees_ros,
# topic_tools, test_msgs and serial_driver are apt packages, so no user-space underlay is
# sourced (jc_deps_ws and rust_underlay were built against the old apt set; never source them
# again). jc_venv stays on PYTHONPATH for the pip-only deps (osqp, pyarrow, ultralytics, torch).
# src/rust/build_env.sh: QDLDL for osqp-sys from src/rust/third_party (no network).
set +u
source /opt/ros/jazzy/setup.bash
_site="$HOME/dev_ws/jc_venv/lib/python3.12/site-packages"
case ":${PYTHONPATH:-}:" in *":$_site:"*) ;; *) export PYTHONPATH="$_site${PYTHONPATH:+:$PYTHONPATH}";; esac
unset _site
export PATH="$HOME/.cargo/bin:$PATH"
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
export WS
SKIP="$(tr '\n' ' ' < "$WS/output/jazzy_port/linus_skip.txt")"
export SKIP
source "$WS/src/rust/build_env.sh"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
# Build:  colcon build --symlink-install --packages-ignore $SKIP        (never two colcon builds at once in $WS)
# Tests (isolated DDS, never the live graph):
#   unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
#   export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
#   colcon test --packages-select <pkgs> --packages-ignore $SKIP --test-result-base <dir>; colcon test-result --all --test-result-base <dir>
export CARGO_BUILD_JOBS=3 MAKEFLAGS=-j3
