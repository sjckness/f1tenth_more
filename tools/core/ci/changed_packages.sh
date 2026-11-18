#!/usr/bin/env bash
# desc: package ROS cambiati tra un ref e HEAD (una riga per nome), per la CI e per f1t build
#
#   f1t ci changed-packages [BASE]        default BASE = F1T_MAIN_REF (origin/main); usa BASE...HEAD
# Un file appartiene al package della cartella con package.xml piu' vicina; esclusi i submodule
# esterni (F1T_MODULES_EXCLUDE_RE). Nessun output = nessun package cambiato.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
F1T_LOG_DIR=${F1T_LOG_DIR:-/dev/null}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
cfg_load
BASE=${1:-$F1T_MAIN_REF}
cd "$F1T_ROOT"
git diff --name-only "$BASE...HEAD" -- src | while read -r f; do
  d=$(dirname "$f")
  while [ "$d" != . ] && [ ! -f "$d/package.xml" ]; do d=$(dirname "$d"); done
  [ -f "$d/package.xml" ] && echo "$d"
done | sort -u | grep -v -E "$F1T_MODULES_EXCLUDE_RE" | while read -r d; do
  sed -n 's:.*<name>[[:space:]]*\([^<[:space:]]*\).*:\1:p' "$d/package.xml" | head -1
done
