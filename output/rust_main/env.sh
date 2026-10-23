# Build/test environment for branch main-jazzy-rust on linus (bash; source it).
#   source output/rust_main/env.sh; cd $WS
# jc_env: linus apt-gap underlay (slam_toolbox, py_trees_ros, topic_tools) + jc_venv on PYTHONPATH
# rust_underlay: test_msgs for rclrs 0.7 (ros2_rust#557 workaround; build time only)
set +u
source "$HOME/dev_ws/jc_env.sh"
source "$HOME/dev_ws/rust_underlay/install/local_setup.bash"
export PATH="$HOME/.cargo/bin:$PATH"
export WS=/home/andreas/dev_ws/f1tenth_more_rust
export SKIP="$(cat $WS/output/jazzy_port/linus_skip.txt | tr '\n' ' ')"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
# Build:  colcon build --symlink-install --packages-ignore $SKIP        (never two colcon builds at once in $WS)
# Tests (isolated DDS, never the live graph):
#   unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
#   export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
#   colcon test --packages-select <pkgs> --packages-ignore $SKIP --test-result-base <dir>; colcon test-result --all --test-result-base <dir>
export CARGO_BUILD_JOBS=3 MAKEFLAGS=-j3
