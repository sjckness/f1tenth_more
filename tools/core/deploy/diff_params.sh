#!/usr/bin/env bash
# desc: diff dei parametri (yaml dei package) tra la versione deployata sull'Orin e un commit
# alias: diff-params
#
#   f1t diff-params [--from REV] [--to REV] [--max-lines N] [--quiet]
#
# --from  default: il commit dell'immagine che gira sull'Orin (deployed.json o label del container)
# --to    default: HEAD di questo checkout
# Mostra i file yaml cambiati sotto src/ (esclusi i submodule esterni) e il diff, poi l'override a
# caldo attivo sull'Orin. Exit 0 anche se ci sono differenze; 1 se un commit e' sconosciuto.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

FROM='' TO=HEAD MAX=300 QUIET=0
while [ $# -gt 0 ]; do
  case "$1" in
    --from) FROM=${2:?}; shift ;;
    --to) TO=${2:?}; shift ;;
    --max-lines) MAX=${2:?}; shift ;;
    --quiet) QUIET=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $1 (f1t diff-params -h)" 2 ;;
  esac
  shift
done
cfg_load
cfg_require config || exit 1

OVERRIDE=''
if [ -z "$FROM" ]; then
  orin_facts || log_die "Orin non raggiungibile e nessun --from: ${ORIN[error]:-}"
  FROM=$(printf '%s' "${ORIN[deployed_json]:-}" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("sha",""))' 2>/dev/null || true)
  FROM=${FROM:-${ORIN[ctr_revision]:-}}
  OVERRIDE=${ORIN[override_stack_args]:-}
  [ -n "$FROM" ] || log_die "non so cosa gira sull'Orin (nessun deployed.json ne' container): usa --from"
fi
git -C "$F1T_ROOT" cat-file -e "${FROM}^{commit}" 2>/dev/null || log_die "commit ${FROM:0:12} sconosciuto su linus (f1t orin status --fetch)"
git -C "$F1T_ROOT" cat-file -e "${TO}^{commit}" 2>/dev/null || log_die "commit $TO sconosciuto"
# shellcheck disable=SC2034  # stampate da cfg_print (lettura indiretta ${!k})
F1T_DIFF_FROM="$(git -C "$F1T_ROOT" log -1 --format='%h %s' "$FROM" | cut -c1-70)"
# shellcheck disable=SC2034  # stampata da cfg_print (lettura indiretta)
F1T_DIFF_TO="$(git -C "$F1T_ROOT" log -1 --format='%h %s' "$TO" | cut -c1-70)"
[ "$QUIET" = 1 ] || cfg_print F1T_DIFF_FROM F1T_DIFF_TO

mapfile -t files < <(git -C "$F1T_ROOT" diff --name-only "$FROM" "$TO" -- 'src/*.yaml' 'src/*.yml' \
  | grep -v -E "$F1T_MODULES_EXCLUDE_RE" || true)
if [ ${#files[@]} -eq 0 ]; then
  log_ok "nessun yaml dei package cambiato tra ${FROM:0:8} e $(git -C "$F1T_ROOT" rev-parse --short "$TO")"
else
  log_warn "${#files[@]} file yaml cambiati (deployato -> nuovo):"
  git --no-pager -C "$F1T_ROOT" diff --stat=100 "$FROM" "$TO" -- "${files[@]}" >&2
  out=$(git --no-pager -C "$F1T_ROOT" diff --no-color -U1 "$FROM" "$TO" -- "${files[@]}")
  n=$(printf '%s\n' "$out" | wc -l)
  printf '%s\n' "$out" | head -n "$MAX"
  [ "$n" -le "$MAX" ] || log_warn "diff troncato a $MAX righe su $n (--max-lines)"
fi
[ -z "$OVERRIDE" ] || log_info "override a caldo attivo sull'Orin ($F1T_ORIN_OVERRIDE_DIR/stack_args): $OVERRIDE"
