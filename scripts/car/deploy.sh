#!/usr/bin/env bash
# scripts/car/deploy.sh - da linus: COMMIT sull'Orin (immagine, script dell'host, profili), verificato.
#
#   scripts/car/deploy.sh [--dry-run] [--force-untested] [--no-allow-drift] [--stop-stack]
#                         [--allow-dirty-profiles] [COMMIT]                    default COMMIT = HEAD
#   scripts/car/deploy.sh --tools-only [--dry-run] [COMMIT]
#
#   --dry-run          controlla tutto, non cambia niente
#   --force-untested   accetta un'immagine senza label tested (f1t deploy lo scrive in deployed.json)
#   --no-allow-drift   NON passa --allow-drift: il container non parte se il checkout nativo dell'Orin
#                      ha modifiche che l'immagine non ha. Default: --allow-drift, scritto nel log
#   --stop-stack       se serve una build e il container dell'auto gira: lo ferma
#   --allow-dirty-profiles   copia anche profili con modifiche non committate (vedi profiles.sh)
#   --tools-only       solo gli script dell'host (passo 4) e la loro verifica: immagine, container e
#                      profili restano come sono. Per un commit che cambia solo scripts/car/orin (es.
#                      car-hil del batch HIL) quando l'immagine fissata ha gia' lo stesso contenuto
#
# Passi (log completo in logs/tools/car_deploy_<data>.log):
#   1 stato    Orin, immagine fissata, modalita', container, drift, stack_args
#   2 immagine come scripts/car/build.sh: riuso di un'immagine equivalente o build sull'Orin
#   3 alias    f1tenth:<branch>-<sha7>
#   4 script   car-stack, car-container, car-preflight di COMMIT -> ~/.local/bin dell'Orin
#   5 profili  src/f1tenth_bringup/config/profiles -> ~/.config/f1tenth_car/profiles dell'Orin
#   6 deploy   tools/f1t deploy --switch-mode --allow-drift: pin, avvio del container, health check,
#              rollback automatico. Saltato se l'immagine e' gia' fissata e il container gira sano
#   7 verifica pin == alias, container sano con quell'immagine, SOURCES equivalente a COMMIT,
#              script == COMMIT, profili == linus
# Lo stack NON viene avviato. Dopo: scripts/car/ready.sh [--profile P], poi sull'Orin
# car-stack start P.
# Exit: 0 OK, 1 fallito (l'auto resta sull'immagine di prima), 2 argomenti.
set -euo pipefail
export GIT_PAGER=cat PAGER=cat      # git diff non deve aprire less dentro f1t deploy
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=car_deploy
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

DRY=() FORCE=() DRIFT=(--allow-drift) STOP=() PDIRTY=() COMMIT=HEAD TOOLS_ONLY=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=(--dry-run) ;;
    --force-untested) FORCE=(--force-untested) ;;
    --no-allow-drift) DRIFT=() ;;
    --stop-stack) STOP=(--stop-stack) ;;
    --allow-dirty-profiles) PDIRTY=(--allow-dirty) ;;
    --tools-only) TOOLS_ONLY=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a (scripts/car/deploy.sh -h)" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
cfg_require config || exit 1
SHA=$(git -C "$ROOT" rev-parse -q --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
dry() { [ ${#DRY[@]} = 1 ]; }
ctr_now() {
  orin_ssh "docker inspect -f '{{.Image}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{end}}' $F1T_ORIN_CONTAINER" 2>/dev/null || echo none
}

ui_section "car deploy: configurazione"
ui_kv "commit" "$SHA"
ui_kv "messaggio" "$(git -C "$ROOT" log -1 --format=%s "$SHA")"
ui_kv "branch" "$(branch_of "$SHA")"
ui_kv "contenuto immagine da" "$(image_commit "$SHA" | cut -c1-12)"
ui_kv "Orin" "$(orin_target)"
ui_kv "opzioni" "${DRY[*]:-reale} ${FORCE[*]:-} ${DRIFT[*]:---no-allow-drift} ${STOP[*]:-} ${PDIRTY[*]:-}"
ui_kv "log" "$F1T_LOG_FILE"
ui_steps "$([ "$TOOLS_ONLY" = 1 ] && echo 1 || echo 7)"
if ! gate=$(car_commit_gate "$SHA" all ${PDIRTY[@]:+--allow-dirty-profiles}); then
  log_die "${COMMIT} non e' pronto per l'Orin, niente e' stato toccato:
$gate
Committa e rilancia."
fi

if [ "$TOOLS_ONLY" = 1 ]; then
  ui_step "solo script dell'host da ${SHA:0:12}: ${CAR_TOOLS[*]##*:}"
  pin_tag=$(orin_ssh "awk '\$1==\"tag\"{print \$2}' $F1T_ORIN_PIN_FILE" 2>/dev/null || true)
  if same_image "$(orin_ssh "docker image inspect -f '{{index .Config.Labels \"org.opencontainers.image.revision\"}}' $pin_tag" 2>/dev/null || echo x)" "$SHA"; then
    log_ok "immagine fissata $pin_tag: stesso contenuto di ${SHA:0:12}, non serve un'immagine nuova"
  else
    log_warn "immagine fissata ${pin_tag:-nessuna}: contenuto DIVERSO da ${SHA:0:12} (gli script vanno, l'immagine no: deploy completo)"
  fi
  if dry; then
    for t in "${CAR_TOOLS[@]}"; do log_info "dry-run: git show ${SHA:0:7}:${t%%:*} -> ~/.local/bin/${t##*:}"; done
  else
    car_tools_install "$SHA" || log_die "installazione degli script sull'Orin fallita"
  fi
  while read -r n st s; do check_record "$st" "$n = commit" "$s"; done < <(car_tools_check "$SHA")
  ui_step_end
  if summary "car deploy --tools-only ${SHA:0:12}"; then log_ok "script dell'host == ${SHA:0:12}"
  else dry || log_die "script dell'host != ${SHA:0:12} (righe FAIL sopra)"; fi
  exit 0
fi

car_state_step
car_image_step "$SHA"
car_alias_step "$SHA"

# ---------------------------------------------------------------- 4 script dell'host
ui_step "script dell'host da ${SHA:0:12}: ${CAR_TOOLS[*]##*:}"
if dry; then
  for t in "${CAR_TOOLS[@]}"; do log_info "dry-run: git show ${SHA:0:7}:${t%%:*} -> ~/.local/bin/${t##*:}"; done
else
  car_tools_install "$SHA" || log_die "installazione degli script sull'Orin fallita"
  car_tools_check "$SHA" | while read -r n st s; do ui_kv "$n" "$st $s"; done
fi
ui_step_end

# ---------------------------------------------------------------- 5 profili
ui_step "profili -> ~/$CAR_PROFILES_DIR"
if dry; then ui_run "$ROOT/scripts/car/profiles.sh" diff || true
else ui_run "$ROOT/scripts/car/profiles.sh" push "${PDIRTY[@]}" || log_die "copia dei profili fallita (sopra)"; fi
ui_step_end

# ---------------------------------------------------------------- 6 deploy
ui_step "deploy (tools/f1t deploy --switch-mode ${DRIFT[*]:-} ${FORCE[*]:-})"
pin_tag=$(orin_ssh "awk '\$1==\"tag\"{print \$2}' $F1T_ORIN_PIN_FILE" 2>/dev/null || true)
if [ "$pin_tag" = "$IMAGE" ] && [ "$(ctr_now)" = "$(img_id "$IMAGE") running healthy" ]; then
  log_ok "$IMAGE e' gia' fissata e il container gira sano con lei: nessun riavvio"
else
  ui_run "$ROOT/tools/f1t" deploy "${DRY[@]}" "${FORCE[@]}" "${DRIFT[@]}" --switch-mode "$IMAGE" \
    || { dry && log_warn "dry-run: f1t deploy non pronto (sopra)"; } \
    || log_die "deploy fallito (f1t deploy fa gia' il rollback all'immagine precedente; log sopra)"
fi
ui_step_end

# ---------------------------------------------------------------- 7 verifica
ui_step "verifica linus == Orin"
if dry; then log_info "dry-run: nessuna verifica"; ui_bar 7 7 "dry-run finito in $(ui_elapsed)"; ui_bar_done; exit 0; fi
pin_id=$(orin_ssh "awk '\$1==\"id\"{print \$2}' $F1T_ORIN_PIN_FILE")
tag_id=$(img_id "$TAG")
ctr=$(ctr_now)
sources=$(orin_ssh "docker exec $F1T_ORIN_CONTAINER cat /opt/f1tenth_ws/SOURCES" 2>/dev/null || echo "commit none")
src_sha=$(awk '$1=="commit"{print $2}' <<<"$sources")
ui_section "SOURCES nel container ($TAG)"
grep -v '^submodule' <<<"$sources" | sed 's/^/  /' >&2
ui_kv "label tested" "${TESTED:-no} $(img_label "$TAG" f1tenth.tested_at)"
ok() { if "$@"; then echo OK; else echo FAIL; fi; }
check_record "$(ok [ "$pin_id" = "$tag_id" ])" "pin = $TAG" "${tag_id:7:12}"
check_record "$(ok [ "$ctr" = "$tag_id running healthy" ])" "container sano con l'immagine" "${ctr:7:12} ${ctr#* }"
check_record "$(ok same_image "${src_sha:-x}" "$SHA")" "SOURCES equivalente al commit" "${src_sha:0:12} ~ ${SHA:0:12}"
while read -r n st s; do check_record "$st" "$n = commit" "$s"; done < <(car_tools_check "$SHA")
pdiff=$(diff <(car_profiles_sums) <(car_profiles_remote_sums) || true)
check_record "$(ok [ -z "$pdiff" ])" "profili = linus" "$(car_profiles_sums | wc -l) profili${pdiff:+, diversi: $(grep -c '^[<>]' <<<"$pdiff")}"
ui_bar 7 7 "finito in $(ui_elapsed)"; ui_bar_done
if summary "car deploy $TAG"; then
  log_ok "linus ${SHA:0:12} == Orin $TAG: OK. Dopo: scripts/car/ready.sh --profile <nome>; sull'Orin: car-stack start <nome>"
else
  log_die "linus ${SHA:0:12} != Orin $TAG: FAIL (righe FAIL sopra)"
fi
