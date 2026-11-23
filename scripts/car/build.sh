#!/usr/bin/env bash
# scripts/car/build.sh - da linus: l'immagine dell'Orin per COMMIT, pronta per il deploy. Non tocca
# l'auto (il container in esecuzione resta com'e', salvo --stop-stack quando serve una build).
#
#   scripts/car/build.sh [--dry-run] [--force-untested] [--stop-stack] [COMMIT]   default COMMIT = HEAD
#
#   --dry-run          controlla tutto, non costruisce niente
#   --force-untested   accetta un'immagine i cui test falliscono (label tested assente)
#   --stop-stack       se serve una build e il container dell'auto gira: lo ferma (f1t build lo rifiuta
#                      con il container su). Senza: si ferma prima di toccare qualcosa
#   BUILD_MAX_S (7200) durata massima della build seguita; BUILD_STALE_S (1800) log fermi = build morta
#
# L'immagine: si riusa un'immagine gia' sull'Orin con lo stesso contenuto di COMMIT (stessi src/,
# docker/orin, scripts/env: un commit che cambia solo gli script o i profili non si ricostruisce),
# prima quella fissata. Se non c'e': tools/f1t build dell'ultimo commit che cambia quel contenuto
# (bundle git -> build SULL'ORIN -> pytest per package -> label tested), seguita con una barra.
# Poi l'alias f1tenth:<branch>-<sha7> sulla stessa immagine.
#
# Passi: 1 stato dell'Orin, 2 immagine, 3 alias. Log: logs/tools/car_build_<data>.log.
# Exit: 0 immagine pronta (ultima riga su stdout: il tag), 1 fallito, 2 argomenti.
# Dopo: scripts/car/deploy.sh COMMIT.
set -euo pipefail
export GIT_PAGER=cat PAGER=cat
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=car_build
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/car/lib/orin_ui.sh
source "$ROOT/scripts/car/lib/orin_ui.sh"
# shellcheck source=scripts/car/lib/orin_common.sh
source "$ROOT/scripts/car/lib/orin_common.sh"
# shellcheck source=scripts/car/lib/car_build.sh
source "$ROOT/scripts/car/lib/car_build.sh"

DRY=() FORCE=() STOP=() COMMIT=HEAD
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=(--dry-run) ;;
    --force-untested) FORCE=(--force-untested) ;;
    --stop-stack) STOP=(--stop-stack) ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a (scripts/car/build.sh -h)" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
cfg_require config || exit 1
SHA=$(git -C "$ROOT" rev-parse -q --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2

ui_section "car build: configurazione"
ui_kv "commit" "$SHA"
ui_kv "messaggio" "$(git -C "$ROOT" log -1 --format=%s "$SHA")"
ui_kv "branch" "$(branch_of "$SHA")"
ui_kv "contenuto immagine da" "$(image_commit "$SHA" | cut -c1-12) (ultimo commit che cambia ${IMAGE_PATHS[*]})"
ui_kv "Orin" "$(orin_target)"
ui_kv "opzioni" "${DRY[*]:-reale} ${FORCE[*]:-} ${STOP[*]:-}"
ui_kv "log" "$F1T_LOG_FILE"
ui_steps 3
if ! gate=$(car_commit_gate "$SHA" image); then
  log_die "${COMMIT} non e' pronto per l'Orin, niente e' stato toccato:
$gate
Committa e rilancia."
fi

car_state_step
car_image_step "$SHA"
car_alias_step "$SHA"
ui_bar 3 3 "finito in $(ui_elapsed)"; ui_bar_done
log_ok "immagine pronta: $TAG (= $IMAGE). Dopo: scripts/car/deploy.sh ${COMMIT}"
echo "$TAG"
