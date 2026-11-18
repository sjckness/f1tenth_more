#!/usr/bin/env bash
# desc: allinea tools/core (e tools/f1t) di questo branch a origin/main e committa "chore: sync tools/core vX.Y.Z"
# alias: core sync
#
#   f1t core sync [--dry-run] [--fetch]
#
# Rimuove la copia del branch e prende quella di F1T_MAIN_REF (anche i file cancellati su main), poi
# committa SOLO tools/core e tools/f1t. Rifiuta su main (li' si modifica direttamente) e se ci sono
# altre modifiche in stage. --fetch fa prima git fetch origin.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
DRY=0 FETCH=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --fetch) FETCH=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $a" 2 ;;
  esac
done
cfg_load
G=(git -C "$F1T_ROOT")
BRANCH=$("${G[@]}" rev-parse --abbrev-ref HEAD)
[ "$BRANCH" != "$F1T_CORE_BRANCH" ] || log_die "sei su $F1T_CORE_BRANCH: qui tools/core si modifica direttamente (git pull)" 1
[ "$FETCH" = 0 ] || "${G[@]}" fetch -q origin
"${G[@]}" rev-parse -q --verify "$F1T_MAIN_REF:tools/core" >/dev/null || log_die "$F1T_MAIN_REF non ha tools/core" 1
"${G[@]}" diff --cached --quiet || log_die "ci sono altre modifiche in stage: il commit di sync deve contenere solo tools/core" 1
VER=$("${G[@]}" show "$F1T_MAIN_REF:tools/core/VERSION")
# shellcheck disable=SC2034  # stampata da cfg_print (lettura indiretta)
F1T_SYNC="$BRANCH: tools/core v$(cat "$F1T_CORE/VERSION" 2>/dev/null || echo '?') -> v$VER ($F1T_MAIN_REF)"
cfg_print F1T_SYNC
if [ "$("${G[@]}" rev-parse HEAD:tools/core 2>/dev/null || echo none)" = "$("${G[@]}" rev-parse "$F1T_MAIN_REF:tools/core")" ] \
   && "${G[@]}" diff --quiet HEAD -- tools/core tools/f1t; then
  log_ok "gia' allineato a $F1T_MAIN_REF (v$VER)"; exit 0
fi
"${G[@]}" diff --stat "HEAD" "$F1T_MAIN_REF" -- tools/core tools/f1t >&2 || true
if [ "$DRY" = 1 ]; then log_warn "DRY-RUN: niente cambiato"; exit 0; fi
"${G[@]}" diff --quiet HEAD -- tools/core tools/f1t || log_die "modifiche locali in tools/core: committale (e riportale su main) o scartale prima" 1
# git rm toglie dal working tree solo i file tracciati; il checkout rimette esattamente l'albero di main.
if [ -n "$("${G[@]}" ls-files -- tools/core)" ]; then "${G[@]}" rm -r -q -- tools/core; fi
"${G[@]}" checkout "$F1T_MAIN_REF" -- tools/core tools/f1t
F1T_CORE_EDIT=1 "${G[@]}" commit -q -m "chore: sync tools/core v$VER"
log_ok "commit $("${G[@]}" rev-parse --short HEAD): chore: sync tools/core v$VER"
