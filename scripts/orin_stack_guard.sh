#!/bin/bash
# The Orin's stack guard (plan P6.4; hard rule: the native Humble stack and the
# Jazzy container never run at the same time, they share the devices, the
# Discovery Server's UDP 11811, the gateway's TCP 8770 and ROS domain 0).
#
#   scripts/orin_stack_guard.sh [--for jazzy|humble] [--quiet]
#
# --for jazzy (default): before starting the car container (docker_orin.sh up,
#   stack_mode.sh container). Refuses while
#   1. a host ROS process runs, UDP 11811 is bound, or any f1tenth container
#      runs (docker/orin/lib_guard.sh: /proc scan, never pgrep -f);
#   2. a /dev/shm Fast DDS segment is mapped by a live process (a ROS 2
#      participant that the process patterns did not name, e.g. the native
#      gateway's rclpy); stale segments are fine;
#   3. TCP 8770 (gateway), 8083 (llama-server) or 8765 (foxglove_bridge) is
#      listened on.
# --for humble: before starting anything native (stack_mode.sh native). The
#   same checks, and refuses while the container f1tenth_car runs (whatever
#   its health).
# Exit 0 clear, 1 refused (each reason on stderr), 2 usage.
# Env: F1T_GUARD_PORTS (TCP ports of check 3, default "8770 8083 8765").
set -uo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
LIB=$HERE/../docker/orin/lib_guard.sh
FOR=jazzy
QUIET=0
usage() { sed -n '2,/^set -uo/p' "$0" | sed '$d; s/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --for) FOR=${2:-}; shift 2 || { usage >&2; exit 2; } ;;
    --for=*) FOR=${1#*=}; shift ;;
    --quiet) QUIET=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "orin_stack_guard: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done
case "$FOR" in jazzy|humble) ;; *) echo "orin_stack_guard: --for jazzy|humble, got '$FOR'" >&2; exit 2 ;; esac
[ -f "$LIB" ] || { echo "orin_stack_guard: $LIB missing" >&2; exit 2; }
# shellcheck source=docker/orin/lib_guard.sh
source "$LIB"

reasons=()

# 1. processes, UDP 11811, f1tenth containers (lib_guard.sh prints its own list)
if ! out=$(orin_guard 2>&1); then
  reasons+=("$(echo "$out" | sed -n '2,$p' | sed 's/^ *//' | paste -sd ';' -)")
fi

# 2. Fast DDS shared memory mapped by a live process. /proc/<pid>/maps is
# readable for this user's processes (the container runs as the same uid);
# other users' are skipped and counted.
shm_hits=()
unreadable=0
for d in /proc/[0-9]*; do
  pid=${d#/proc/}
  [ "$pid" = "$$" ] && continue
  if ! maps=$(cat "$d/maps" 2>/dev/null); then
    [ -e "$d" ] && unreadable=$((unreadable + 1))
    continue
  fi
  seg=$(printf '%s\n' "$maps" | grep -o -m1 -E '/dev/shm/(fastrtps_|fast_datasharing|sem\.fastrtps_)[^ ]*' || true)
  if [ -n "$seg" ]; then
    shm_hits+=("pid $pid ($(tr '\0' ' ' < "$d/cmdline" 2>/dev/null | cut -c1-80)) maps $seg")
  fi
done
[ ${#shm_hits[@]} -eq 0 ] || reasons+=("live Fast DDS shared memory: ${shm_hits[*]:0:3}")

# 3. the stack's TCP ports
for port in ${F1T_GUARD_PORTS:-8770 8083 8765}; do
  if ss -Htln "sport = :$port" 2>/dev/null | grep -q .; then
    who=$(ss -Htlnp "sport = :$port" 2>/dev/null | grep -o 'users:(([^)]*' | head -1 | sed 's/users:((//')
    reasons+=("TCP $port is listened on${who:+ by $who}")
  fi
done

# 4. --for humble: the car container
if [ "$FOR" = humble ]; then
  state=$(docker inspect --type container -f '{{.State.Status}}' f1tenth_car 2>/dev/null || true)
  [ "$state" != running ] || reasons+=("the Jazzy container f1tenth_car runs (stack_mode.sh native stops it)")
fi

if [ ${#reasons[@]} -gt 0 ]; then
  echo "orin_stack_guard --for $FOR: REFUSING:" >&2
  printf '  - %s\n' "${reasons[@]}" >&2
  exit 1
fi
note=""
[ "$unreadable" -eq 0 ] || note="; the memory maps of $unreadable processes of other users were not readable"
[ "$QUIET" = 1 ] || echo "orin_stack_guard --for $FOR: clear (no ROS process, UDP 11811 free, no live Fast DDS SHM, TCP ${F1T_GUARD_PORTS:-8770 8083 8765} free, no f1tenth container$note)"
exit 0
