# shellcheck shell=bash
# Shared helpers for the D3 parity harness (scripts/jazzy_parity/*.sh) and the
# Orin container checks (docker/orin/checks/*.sh). Sourced, never run.
#
# Runs wherever its caller runs: the Orin host with native Humble sourced, the
# harness container (Jazzy, docker/orin/compose.harness.yaml) or linus (dry
# smoke). Nothing here assumes a distro: ROS_DISTRO comes from the sourced
# setup files, and only bash, coreutils, procps and python3 are used.
#
# Two rules from CLAUDE.md "Harness gotchas" are enforced here for every
# caller:
#   - processes the harness starts are stopped by PID or process group, never
#     by `pkill -f PATTERN` (it matches the calling shell's own command line);
#   - the harness deletes nothing recursively: a run directory must be new or
#     empty (need_empty_dir), so no `rm -rf` of a variable path exists at all.

# The exec wrapper every background command goes through.
# - A background job of a non-interactive shell inherits SIGINT as IGNORED and
#   cannot un-ignore it, so a node started with `cmd &` would see the SIGINT a
#   terminal or launch delivers as rclpy's ExternalShutdownException (checked
#   on Humble and Jazzy in Phase 4). SIG_DFL is restored before exec.
# - os.setsid() gives the command its own process group (pgid == its pid), so
#   the harness can stop everything it started by group. It fails only for a
#   process that already leads a group, whose pgid is then its pid anyway.
PARITY_EXEC=(python3 -c 'import os, signal, sys
try:
    os.setsid()
except OSError:
    pass
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])')

# bg_start VAR LOG CMD...: run CMD in the background through PARITY_EXEC,
# stdout and stderr to LOG, stdin from /dev/null; its pid (which is also its
# process group id) goes into the variable named VAR.
bg_start() {
  local __var=$1 __log=$2
  shift 2
  "${PARITY_EXEC[@]}" "$@" > "$__log" 2>&1 < /dev/null &
  printf -v "$__var" '%s' "$!"
}

# pg_alive PGID: true while any process of that process group exists.
pg_alive() { kill -0 -- "-$1" 2>/dev/null; }

# pg_stop PGID [INT_WAIT_S] [TERM_WAIT_S]: SIGINT the whole group, wait up to
# INT_WAIT_S (default 10); SIGTERM, wait up to TERM_WAIT_S (default 5);
# SIGKILL. Prints which signal ended it. Returns 0 when SIGINT or SIGTERM was
# enough, 1 when SIGKILL was needed.
pg_stop() {
  local g=$1 wi=${2:-10} wt=${3:-5} i
  pg_alive "$g" || { echo "pg $g: already gone"; return 0; }
  kill -INT -- "-$g" 2>/dev/null
  for ((i = 0; i < wi * 4; i++)); do
    pg_alive "$g" || { echo "pg $g: stopped by SIGINT"; return 0; }
    sleep 0.25
  done
  kill -TERM -- "-$g" 2>/dev/null
  for ((i = 0; i < wt * 4; i++)); do
    pg_alive "$g" || { echo "pg $g: stopped by SIGTERM"; return 0; }
    sleep 0.25
  done
  kill -KILL -- "-$g" 2>/dev/null
  echo "pg $g: needed SIGKILL"
  return 1
}

# pid_wait_gone PID SECONDS: wait until PID is gone; returns 1 on timeout.
pid_wait_gone() {
  local p=$1 s=$2 i
  for ((i = 0; i < s * 4; i++)); do
    kill -0 "$p" 2>/dev/null || return 0
    sleep 0.25
  done
  return 1
}

# need_empty_dir DIR: create DIR, or accept it if it exists and is empty;
# refuse (exit 2) otherwise. The harness never deletes a previous run: pick a
# new directory instead.
need_empty_dir() {
  mkdir -p "$1" || exit 2
  if [ -n "$(ls -A "$1")" ]; then
    echo "refusing: $1 is not empty; pass a new run directory (nothing is deleted)" >&2
    exit 2
  fi
}

# parity_abs DIR: absolute path of an existing directory.
parity_abs() { (cd "$1" && pwd); }

# udp_port_bound PORT: true if a UDP socket is bound to PORT (any address).
udp_port_bound() {
  ss -H -uln "sport = :$1" 2>/dev/null | grep -q .
}

# udp_port_owner PORT: "pid=N,..." of the process holding PORT, if this PID
# namespace can see it; empty if nobody holds it or the holder is invisible
# (another container, or the host seen from a container).
udp_port_owner() {
  ss -H -ulnp "sport = :$1" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1
}

# isolated_dds DOMAIN: the ground-rule environment for a tool that must not
# join any other graph (no discovery server, localhost only, own domain).
isolated_dds() {
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_DOMAIN_ID=$1
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  # Humble has no ROS_AUTOMATIC_DISCOVERY_RANGE; its equivalent.
  [ "${ROS_DISTRO:-}" = humble ] && export ROS_LOCALHOST_ONLY=1
  return 0
}

# parity_context OUT_DIR [LABEL]: record what this run ran on (distro, image,
# power mode, clocks) as OUT_DIR/context[_LABEL].json. Never fails the caller.
parity_context() {
  local here
  here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  python3 "$here/run_context.py" --out "$1/context${2:+_$2}.json" > /dev/null 2>&1 \
    || echo "run_context.py failed (context not recorded)" >&2
  return 0
}
