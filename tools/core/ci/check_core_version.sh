#!/usr/bin/env bash
# desc: una modifica a tools/core rispetto a un ref deve avere bump di VERSION e voce nel CHANGELOG (CI)
#
#   f1t ci check-core-version [BASE]      default BASE = F1T_MAIN_REF; confronta BASE...HEAD
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
F1T_LOG_DIR=${F1T_LOG_DIR:-/dev/null}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
cfg_load
BASE=${1:-$F1T_MAIN_REF}
cd "$F1T_ROOT"
changed=$(git diff --name-only "$BASE...HEAD" -- tools/core tools/f1t)
[ -n "$changed" ] || { echo "tools/core invariato rispetto a $BASE"; exit 0; }
old=$(git show "$BASE:tools/core/VERSION" 2>/dev/null || echo none)
new=$(cat tools/core/VERSION)
bad=0
if [ "$old" = "$new" ]; then echo "FAIL: tools/core cambiato ma VERSION e' ancora $new (bump: MAJOR se cambia un comando)"; bad=1; fi
grep -qx tools/core/CHANGELOG.md <<< "$changed" || { echo "FAIL: manca la voce in tools/core/CHANGELOG.md"; bad=1; }
grep -q "^## \[$new\]" tools/core/CHANGELOG.md || { echo "FAIL: tools/core/CHANGELOG.md non ha la sezione ## [$new]"; bad=1; }
[ "$bad" = 1 ] || echo "OK: tools/core $old -> $new con voce nel CHANGELOG"
exit "$bad"
