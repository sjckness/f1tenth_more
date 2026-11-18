#!/usr/bin/env bash
# desc: test rapidi di tools/core (bash puro, < 30 s, niente rete reale): tools/core/tests/run.sh
# alias: selftest
#
#   f1t selftest [-v] [FILTRO]     FILTRO = parte del nome di un test
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
[ -x "$F1T_CORE/tests/run.sh" ] || { echo "selftest: manca $F1T_CORE/tests/run.sh" >&2; exit 1; }
exec "$F1T_CORE/tests/run.sh" "$@"
