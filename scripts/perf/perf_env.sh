# Source this (bash) before any scripts/perf tool: ROS 2 Jazzy, the host's extra setup
# (linus: ~/dev_ws/jc_env.sh puts osqp / torch / ultralytics on PYTHONPATH) and THIS
# worktree's install. Never another checkout's: with --symlink-install an install
# space points into its own sources.
#   F1TENTH_EXTRA_SETUP  host setup file (default ~/dev_ws/jc_env.sh, skipped if absent)
_perf_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
_perf_extra="${F1TENTH_EXTRA_SETUP:-$HOME/dev_ws/jc_env.sh}"
if [ -f "$_perf_extra" ]; then source "$_perf_extra"; else source /opt/ros/jazzy/setup.bash; fi
if [ ! -f "$_perf_ws/install/setup.bash" ]; then
  echo "perf_env: no $_perf_ws/install -- build the workspace first (see scripts/perf/README.md)" >&2
  return 1 2>/dev/null || exit 1
fi
source "$_perf_ws/install/setup.bash"
export F1T_PERF_WS="$_perf_ws"
unset _perf_ws _perf_extra
