#!/usr/bin/env bash
# sim_stack.sh -- one command for the stack side of the Isaac sim (run it on the Thor).
# A thin wrapper around `sim_isaac.sh stack` with the two-machine defaults: --net eth and
# enable_intelligence:=true (llama-server = the 'intelligence' component, as on the car).
#
# Usage:
#   scripts/sim/sim_stack.sh                          # --net eth enable_intelligence:=true
#   scripts/sim/sim_stack.sh enable_intelligence:=false   # any supervisor_bringup k:=v passes through
#   scripts/sim/sim_stack.sh --net local              # stack and Isaac on this one machine
#   scripts/sim/sim_stack.sh --domain 43 --clean --dry-run
#   scripts/sim/sim_stack.sh --gazebo                 # where the Gazebo version went (see below)
# Every option and k:=v argument goes to sim_isaac.sh stack (see `sim_isaac.sh --help`); a
# k:=v given here overrides the default above (the default is not added then).
#
# Two machines (direct Ethernet link, Discovery Server on the Thor):
#   1. linus:  scripts/sim/sim_isaac.sh isaac --net eth [--scene base_room --layout campaign] [--gui]
#   2. Thor:   scripts/sim/sim_stack.sh
#   3. Thor:   scripts/sim/llm.sh "go to the person"         # one LLM command
#   Either side may start first; the stack waits up to 30 s for /clock.
#
# --gazebo: the Gazebo sim_stack.sh (sim:=true + sim_net.sh, f1tenth_sim) lives on branch
# 'jazzy' (git show jazzy:scripts/sim/sim_stack.sh). This branch's supervisor_bringup only
# knows sim:=none|isaac, so it cannot run here; --gazebo says so instead of failing oddly.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }

NET_SET=false; INTEL_SET=false
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --gazebo)
      echo "[sim_stack] the Gazebo stack (sim:=true, sim_net.sh) is on branch 'jazzy':" >&2
      echo "            git show jazzy:scripts/sim/sim_stack.sh   (or check out jazzy and run it there)" >&2
      echo "            this branch (jazzy-campaign) supports sim:=isaac only." >&2
      exit 2 ;;
    --net|--domain) ARGS+=("$1" "${2:?$1 needs a value}"); [ "$1" = --net ] && NET_SET=true; shift 2 ;;
    --net=*) ARGS+=("$1"); NET_SET=true; shift ;;
    enable_intelligence:=*) ARGS+=("$1"); INTEL_SET=true; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
$NET_SET || ARGS=(--net eth "${ARGS[@]}")
$INTEL_SET || ARGS+=(enable_intelligence:=true)
exec "$HERE/sim_isaac.sh" stack "${ARGS[@]}"
