# Sourceable guard for Phase 5d (hard rule 2): the Humble stack and a Jazzy
# container never run at the same time on the Orin (shared devices, the
# Discovery Server's UDP 11811, the same ROS_DOMAIN_ID).
#
#   source docker/orin/lib_guard.sh
#   GUARD_OWN_PREFIX=f1tenth_spike orin_guard || exit 1
#
# Refuses (returns 1, says why on stderr) if any of these is true:
#   - a host process runs ROS: /opt/ros/humble, the Humble workspace install,
#     a component supervisor, a ros2 daemon, a Discovery Server, llama-server,
#     `ros2 launch|run|bag`. Scans /proc, never `pgrep -f` (which matches the
#     calling shell's own command line, CLAUDE.md "Harness gotchas");
#   - UDP 11811 is bound;
#   - a container from an f1tenth:* image runs.
# Processes inside containers whose name starts with $GUARD_OWN_PREFIX are not
# a conflict (a caller that runs several of its own containers at once).
# scripts/orin_stack_guard.sh (D2) wraps this for docker_orin.sh and the
# D3/D4 harnesses.

_orin_guard_own_ids=""

_orin_guard_in_own_container() {
  local id
  for id in $_orin_guard_own_ids; do
    grep -q "$id" "/proc/$1/cgroup" 2>/dev/null && return 0
  done
  return 1
}

orin_guard() {
  local prefix=${GUARD_OWN_PREFIX:-}
  local hits=() d pid cmd binder other
  _orin_guard_own_ids=""
  [ -n "$prefix" ] && _orin_guard_own_ids=$(docker ps -q --no-trunc --filter "name=^${prefix}")
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [ "$pid" = "$$" ] && continue
    cmd=$( { tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null) || continue
    [ -n "$_orin_guard_own_ids" ] && _orin_guard_in_own_container "$pid" && continue
    case "$cmd" in
      *lib_guard.sh*|*spike.sh*|*image_report.sh*|*docker\ *|*containerd*) continue ;;
      */opt/ros/humble/*|*dev_ws/f1tenth_more/install*|*component_supervisor_node*|*_ros2_daemon*|*ros2cli.daemon*|*fast-discovery-server*|*fastdds\ discovery*|*llama-server*|*"ros2 launch"*|*"ros2 run"*|*"ros2 bag"*)
        hits+=("$pid: ${cmd:0:120}") ;;
    esac
  done
  for binder in $(ss -Hulpn 'sport = :11811' 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2); do
    [ -n "$_orin_guard_own_ids" ] && _orin_guard_in_own_container "$binder" && continue
    hits+=("UDP 11811 is bound by pid $binder")
  done
  if ss -Hulpn 'sport = :11811' 2>/dev/null | grep -q . && ! ss -Hulpn 'sport = :11811' | grep -q 'pid='; then
    hits+=("UDP 11811 is bound by a process of another user")
  fi
  other=$(docker ps --format '{{.Names}} {{.Image}}' | grep -E ' f1tenth:' || true)
  [ -n "$prefix" ] && other=$(echo "$other" | grep -v "^${prefix}" || true)
  [ -n "$other" ] && hits+=("an f1tenth container runs: $other")
  if [ ${#hits[@]} -gt 0 ]; then
    echo "guard: REFUSING, a ROS stack, Discovery Server or f1tenth container is up:" >&2
    printf '  %s\n' "${hits[@]}" >&2
    return 1
  fi
  echo "guard: host clear (no Humble/ROS process, UDP 11811 free, no other f1tenth container)"
}
