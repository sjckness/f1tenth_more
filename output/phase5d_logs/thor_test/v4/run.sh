#!/bin/bash
# Phase 5d: colcon test of jazzy@083c8e8 on the Thor, in an isolated clone
# (~/dev_ws/f1tenth_p5d_083c8e8). andre's workspace is not touched.
# Waits until no ROS stack has run for 5 consecutive minutes (gives up after 8 h).
# PYTHONNOUSERSITE=1 for build and test: ~/.local (PyPI torch cu130, setuptools
# 79/84) must not shadow apt, so the result is the clean apt-only reference.
set -u; set +u  # ROS setup files read unset variables (v3 died on AMENT_TRACE_SETUP_FILES)
D=$HOME/dev_ws/f1tenth_p5d_083c8e8
OUT=$HOME/p5d_thor_test_083c8e8
log() { echo "[$(date '+%F %T')] $*" >> $OUT/run.log; }
stack_up() {
  ps -eo cmd | grep -E 'component_supervisor_node|fast-discovery-server|ros2 launch|gz sim|--ros-args' | grep -v grep >/dev/null
}
log "waiting for a quiet Thor (v4: PYTHONNOUSERSITE=1, set +u)"
quiet=0; waited=0
while [ $quiet -lt 5 ]; do
  if stack_up; then quiet=0; else quiet=$((quiet+1)); fi
  sleep 60; waited=$((waited+1))
  if [ $waited -ge 480 ]; then log "gave up: stack still up after 8 h"; exit 2; fi
done
log "quiet for 5 min, starting"
ps -eo pid,ppid,etime,cmd > $OUT/ps_before.txt
cd $D || exit 3
# What ~/.local holds (recorded, then excluded).
python3 -m pip list --user --format=freeze > $OUT/pip_user_at_test.txt 2>&1
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=$D/src/f1tenth_bringup/config/fastdds_profile.xml
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
env | sort > $OUT/env.txt
python3 -m pip list --format=freeze > $OUT/pip_freeze_at_test.txt 2>&1
log "python: $(python3 -c 'import setuptools, sys; print("setuptools", setuptools.__version__, "no_user_site", sys.flags.no_user_site)')"
git log -1 --format='%H %s' > $OUT/commit.txt; git submodule status >> $OUT/commit.txt
colcon list > $OUT/colcon_list.txt 2>&1
log "build start"
nice -n 10 colcon build --symlink-install --test-result-base $OUT/test_results > $OUT/build.log 2>&1
log "build rc=$?"
source install/setup.bash
log "python after sourcing the install: $(python3 -c 'import setuptools, sys; print("setuptools", setuptools.__version__, "no_user_site", sys.flags.no_user_site)')"
log "test start"
nice -n 10 colcon test --test-result-base $OUT/test_results > $OUT/test.log 2>&1
log "test rc=$?"
colcon test-result --test-result-base $OUT/test_results --all > $OUT/test_result_all.txt 2>&1
colcon test-result --test-result-base $OUT/test_results --verbose > $OUT/test_result_verbose.txt 2>&1
log "test-result rc=$?"
ps -eo pid,ppid,etime,cmd > $OUT/ps_after.txt
stack_up && log "NOTE: a stack was up when the run ended"
log "done"
