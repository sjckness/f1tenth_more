#!/bin/bash
# The corridor test suites, once per implementation mode (main-jazzy-rust R1):
#
#   py          the reference: plain pytest, corridor_impl default py
#   rust        F1TENTH_CORRIDOR_IMPL=rust: MPC_corr's geometry call sites (real
#               nodes and the duck-typed stand-ins) use the Rust library
#   substitute  corridor_capture_plugin MODE substitute: corridor_geometry's two
#               functions and mpc_solver's four queries are REPLACED by Rust for
#               every caller (test_corridor_geometry.py, reverse_manoeuvre,
#               corridor_def, the solver)
#   capture     corridor_capture_plugin MODE capture: the Python runs, and every
#               call is replayed through Rust in process and compared
#               (summary.json, corpus.pkl)
#
# Then junit_compare.py: every test ID must have the same outcome in all four.
#
#   tools/rust_parity/corridor/run_corridor_tests.sh <out dir> [mode ...]
#
# Isolated DDS (localhost, domain 96), like every test run in this workspace.
set -euo pipefail
OUT=${1:?usage: run_corridor_tests.sh <out dir> [mode ...]}
shift
MODES=("$@")
[ ${#MODES[@]} -gt 0 ] || MODES=(py rust substitute capture)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
set +u
# shellcheck source=/dev/null
source "$REPO/output/rust_main/env.sh"
set -u
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE F1TENTH_CORRIDOR_IMPL
export ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$HERE:${PYTHONPATH:-}"
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

# package -> its directory (pytest runs from there, as the docstrings say)
declare -A PKG_DIR=(
  [mpc_controller]=src/f1tenth_control/mpc_controller
  [f1tenth_params]=src/f1tenth_params
  [f1tenth_logger]=src/f1tenth_logger
)

for mode in "${MODES[@]}"; do
  mkdir -p "$OUT/$mode"
  for pkg in mpc_controller f1tenth_params f1tenth_logger; do
    plugin=()
    env_impl=()
    case "$mode" in
      py) ;;
      rust) env_impl=(F1TENTH_CORRIDOR_IMPL=rust) ;;
      substitute|capture)
        plugin=(-p corridor_capture_plugin)
        env_impl=(F1TENTH_CORRIDOR_PARITY_MODE="$mode" F1TENTH_CORRIDOR_PARITY_OUT="$OUT/$mode/$pkg") ;;
      *) echo "unknown mode $mode" >&2; exit 2 ;;
    esac
    echo "== $mode $pkg ($(date +%H:%M:%S))"
    start=$(date +%s)
    (cd "$REPO/${PKG_DIR[$pkg]}" && env "${env_impl[@]}" python3 -m pytest -q -p no:cacheprovider \
        "${plugin[@]}" --junitxml="$OUT/$mode/$pkg.xml" test > "$OUT/$mode/$pkg.log" 2>&1) || true
    echo "   $(tail -1 "$OUT/$mode/$pkg.log") [$(( $(date +%s) - start )) s]"
  done
done
python3 "$HERE/junit_compare.py" "$OUT"
