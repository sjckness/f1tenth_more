#!/usr/bin/env bash
# The mission logger's archive trigger reaches the host's rsync to linus.
#
# Runs on: the Orin host, f1tenth_car up, the host units installed
# (docker/orin/host/install.sh: f1tenth-archive-trigger.path ->
# f1tenth-archive.service). Plan P8: "archive trigger -> host rsync".
#
#   archive_trigger_check.sh OUT_DIR [--wait S]
#
# Writes the trigger file from INSIDE the container exactly as the logger
# does (JSON, atomic rename, at $F1TENTH_ARCHIVE_TRIGGER of the container's
# ctl env: the bind mount is part of what is tested), then waits up to --wait
# (60) s for f1tenth-archive.service to start (its ExecMainStartTimestamp
# changes) and up to 300 s for it to finish with Result=success. The path
# unit's watched path is printed for the record.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
WAIT=60
[ "${2:-}" = --wait ] && WAIT=$3
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
SVC=f1tenth-archive.service
PATHU=f1tenth-archive-trigger.path
prop() { systemctl --user show "$SVC" -p "$1" --value 2>/dev/null; }
systemctl --user cat "$PATHU" > "$OUT/path_unit.txt" 2>&1
echo "path unit active: $(systemctl --user is-active "$PATHU" 2>&1)" >> "$OUT/path_unit.txt"
grep -E '^Path(Modified|Changed|Exists)' "$OUT/path_unit.txt" | sed 's/^/watched: /'
start0=$(prop ExecMainStartTimestampMonotonic)
# shellcheck disable=SC2016  # expanded inside the container
cexec bash -c '[ -n "${F1TENTH_ARCHIVE_TRIGGER:-}" ] || { echo "F1TENTH_ARCHIVE_TRIGGER unset"; exit 3; }
  mkdir -p "$(dirname "$F1TENTH_ARCHIVE_TRIGGER")"
  printf "{\"run_id\": \"d2_archive_trigger_check\", \"time\": \"%s\"}\n" "$(date -u +%FT%T+00:00)" \
    > "$F1TENTH_ARCHIVE_TRIGGER.tmp" && mv "$F1TENTH_ARCHIVE_TRIGGER.tmp" "$F1TENTH_ARCHIVE_TRIGGER"
  echo "written $F1TENTH_ARCHIVE_TRIGGER"' > "$OUT/trigger.log" 2>&1
rc=$?
record "$OUT" "trigger written in the container" "$([ $rc -eq 0 ] && echo PASS || echo FAIL)" "$(tail -1 "$OUT/trigger.log")"
started=""
for _ in $(seq 1 "$WAIT"); do
  sleep 1
  s=$(prop ExecMainStartTimestampMonotonic)
  if [ -n "$s" ] && [ "$s" != "$start0" ] && [ "$s" != 0 ]; then started=$s; break; fi
done
record "$OUT" "archive service started" "$([ -n "$started" ] && echo PASS || echo FAIL)" \
  "ExecMainStartTimestampMonotonic $start0 -> ${started:-unchanged after ${WAIT} s}"
if [ -n "$started" ]; then
  for _ in $(seq 1 300); do
    [ "$(prop ActiveState)" = activating ] || [ "$(prop ActiveState)" = active ] || break
    sleep 1
  done
  result=$(prop Result)
  status=$(prop ExecMainStatus)
  record "$OUT" "archive sync result" "$([ "$result" = success ] && [ "$status" = 0 ] && echo PASS || echo FAIL)" \
    "Result=$result ExecMainStatus=$status"
fi
journalctl --user -u "$SVC" -n 30 --no-pager > "$OUT/archive_journal.txt" 2>&1
overall "$OUT"
