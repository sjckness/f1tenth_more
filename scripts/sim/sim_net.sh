# sim_net.sh -- shared ROS 2 network config for the two-machine sim.
#
# MUST be sourced (sim.sh / sim_stack.sh do), never executed: it only sets
# variables, so running it would change nothing (and it has no exec bit).
# Sets the Discovery-Server client env for THIS shell/launcher, in one place,
# then sources scripts/env/jazzy.sh (which turns F1TENTH_DISCOVERY_SERVER into
# ROS_DISCOVERY_SERVER, loads the Fast DDS profile, and defines the `ros2cli`
# introspection helper). For an interactive sim shell, bash or zsh:
#   source scripts/sim/sim_net.sh
#   ros2cli topic list
#
# Defaults are the Phase S sim setup: Discovery Server on the Thor
# (10.42.0.2:11811), ROS_DOMAIN_ID 42. Override any of them by exporting before
# launch, e.g.:
#   F1TENTH_SIM_DISCOVERY_IP=10.42.0.9 scripts/sim/sim_stack.sh
#   ROS_DOMAIN_ID=7 scripts/sim/sim.sh
: "${F1TENTH_SIM_DISCOVERY_IP:=10.42.0.2}"     # LAN IP of the Discovery Server host (the Thor)
: "${F1TENTH_SIM_DISCOVERY_PORT:=11811}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export F1TENTH_DISCOVERY_SERVER="${F1TENTH_DISCOVERY_SERVER:-${F1TENTH_SIM_DISCOVERY_IP}:${F1TENTH_SIM_DISCOVERY_PORT}}"

# This file's directory, whichever shell sources it (zsh has no BASH_SOURCE;
# same idiom as jazzy.sh, whose zsh branch bash never evaluates).
if [ -n "${ZSH_VERSION:-}" ]; then
  _net_dir=${${(%):-%x}:A:h}
else
  _net_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
# ROS setup scripts are not `set -u`/`set -e` clean; relax only around the source.
_net_had_u=1; case "$-" in *u*) ;; *) _net_had_u=0 ;; esac
_net_had_e=1; case "$-" in *e*) ;; *) _net_had_e=0 ;; esac
set +u +e
# shellcheck disable=SC1091
source "$_net_dir/../env/jazzy.sh"
[ "$_net_had_u" = 1 ] && set -u
[ "$_net_had_e" = 1 ] && set -e
unset _net_dir _net_had_u _net_had_e
