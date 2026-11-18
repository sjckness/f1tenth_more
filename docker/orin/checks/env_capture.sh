#!/usr/bin/env bash
# The environment the running stack actually has: /proc/<pid>/environ of the
# component supervisor (every component inherits it).
#
# Runs on: the Orin host, with the stack running on either side (the
# container's processes are visible from the host and run as the same user,
# so both are read from the host). Plan P8 "environment table vs native":
# capture once per side, then `env_compare.py native.env container.env`.
#
#   env_capture.sh OUT_FILE native|container
#
# Picks the component_supervisor_node process whose command line lies under
# /opt/f1tenth_ws (container) or not (native). Values of variables whose name
# holds TOKEN, SECRET, PASSWORD or KEY are masked.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
[ $# -eq 2 ] || { echo "usage: env_capture.sh OUT_FILE native|container" >&2; exit 2; }
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
case "$2" in native|container) ;; *) echo "side must be native or container" >&2; exit 2 ;; esac
pid=$(python3 "$REPO/scripts/jazzy_parity/proc_audit.py" snapshot \
  | awk -v side="$2" '/component_supervisor_node/ && !/ros2 launch/ {
      inside = ($0 ~ /\/opt\/f1tenth_ws\//); if ((side == "container") == inside) { print $1; exit } }')
[ -n "$pid" ] || { echo "no component_supervisor_node process on the $2 side" >&2; exit 1; }
{
  echo "# env_capture.sh $2: pid $pid, $(date -Is)"
  echo "# cmdline: $(tr '\0' ' ' < "/proc/$pid/cmdline" | cut -c1-200)"
  tr '\0' '\n' < "/proc/$pid/environ" | sort \
    | sed -E 's/^([^=]*(TOKEN|SECRET|PASSWORD|KEY)[^=]*)=.*/\1=<masked>/'
} > "$1"
echo "$(grep -vc '^#' "$1") variables of pid $pid -> $1"
