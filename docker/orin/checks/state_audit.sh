#!/usr/bin/env bash
# Ownership of every piece of container state, and what the container wrote
# outside its mounts.
#
# Runs on: the Orin host, after the container ran >= 10 min (P8: "ownership of
# all state + `docker diff` after a 10 min run lists only intended paths").
#
#   state_audit.sh OUT_DIR [--uid 1004]
#
#   host state   every file under F1T_STATE_ROOT (/scratch/fabiocar/
#                f1tenth_container: archive, runs, .ros, ctl / gateway state,
#                archive trigger, ZED settings, mission dirs) is owned by
#                --uid (fabiocar): nothing root-owned that the next run or the
#                host archive timer could not write or delete;
#   volumes      the same inside the container for the named volumes
#                (/var/cache/f1tenth/{cuda,trt}, /usr/local/zed/resources) and
#                the mission dirs at compose.yaml's mount targets
#                (src/f1tenth_control/corridors_jsons,
#                src/f1tenth_behavior/mission_reports, mission_videos,
#                install/share/f1tenth_behavior/missions/llm_generated);
#   docker diff  docker_diff_check.py against docker_diff_allow.txt.
#
# The car role only (compose.yaml alone): the harness override adds the
# checkout's mounts, whose mount points are not in the image.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
UID_WANT=1004
[ "${2:-}" = --uid ] && UID_WANT=$3
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
role=$(container_role)
[ "$role" = car ] || { echo "$CONTAINER runs as '$role', not 'car' (stack_mode.sh container)" >&2; exit 2; }
host_context "$OUT" host

if [ -d "$F1T_STATE_ROOT" ]; then
  find "$F1T_STATE_ROOT" ! -uid "$UID_WANT" -printf '%u:%g %m %p\n' 2> "$OUT/host_find_errors.txt" \
    > "$OUT/host_not_owned.txt"
  n=$(grep -c . "$OUT/host_not_owned.txt")
  e=$(grep -c . "$OUT/host_find_errors.txt")
  if [ "$n" -eq 0 ] && [ "$e" -eq 0 ]; then
    record "$OUT" "host state ownership" PASS "every file under $F1T_STATE_ROOT owned by uid $UID_WANT"
  else
    record "$OUT" "host state ownership" FAIL "$n not owned by uid $UID_WANT, $e unreadable (host_not_owned.txt)"
  fi
else
  record "$OUT" "host state ownership" FAIL "no $F1T_STATE_ROOT"
fi

# shellcheck disable=SC2016  # expanded inside the container
# (Each exists in the image, owned by the user: a missing one is a finding.)
"$DOCKER" exec "$CONTAINER" sh -c 'for d in /var/cache/f1tenth \
    /usr/local/zed/resources \
    /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons \
    /opt/f1tenth_ws/src/f1tenth_behavior/mission_reports \
    /opt/f1tenth_ws/mission_videos \
    /opt/f1tenth_ws/install/share/f1tenth_behavior/missions/llm_generated; do
  if [ -e "$d" ]; then find "$d" ! -uid "$1" -printf "%u:%g %m %p\n"; else echo "missing $d"; fi
  done' _ "$UID_WANT" > "$OUT/container_not_owned.txt" 2>&1
n=$(grep -c . "$OUT/container_not_owned.txt")
record "$OUT" "container volume ownership" "$([ "$n" -eq 0 ] && echo PASS || echo FAIL)" \
  "$n entries not owned by uid $UID_WANT (container_not_owned.txt)"

"$DOCKER" diff "$CONTAINER" > "$OUT/docker_diff.txt" 2>&1
python3 "$HERE/docker_diff_check.py" "$OUT/docker_diff.txt" --out "$OUT/docker_diff_check.json" \
  > "$OUT/docker_diff_check.txt"
rc=$?
record "$OUT" "docker diff" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "$(head -1 "$OUT/docker_diff_check.txt")"
overall "$OUT"
