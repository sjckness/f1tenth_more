#!/usr/bin/env bash
# orin_deploy.sh - da linus: immagine Jazzy dell'Orin per la campagna a 3 comandi, deploy, verifica.
#
#   scripts/orin_deploy.sh [--dry-run] [--force-untested] [COMMIT]     default COMMIT = HEAD (committato)
#
#   --dry-run          controlla tutto, non cambia niente
#   --force-untested   usa l'immagine anche se il passo test di f1t build e' fallito (o non c'e' la
#                      label tested); lo passa a f1t deploy, che lo scrive in deployed.json
#
# Passi (barra generale + barra/attesa per ogni passo, ogni comando stampato prima di eseguirlo,
# log completo in logs/tools/orin_deploy_<data>.log):
#   1 stato    commit, immagini e modalita' dell'Orin prima di toccare qualcosa
#   2 build    tools/f1t build --detach COMMIT: bundle git -> build SULL'ORIN (il Dockerfile linka le
#              librerie L4T r36.5 dell'host, JetPack 6.2.x: su linus non si costruisce) -> pytest
#              nell'immagine -> label f1tenth.tested=true. Barra dai passi [K/M] di BuildKit.
#              Tag f1tenth:jazzy-orin-<sha7>-norust. Nessun registry (F1T_REGISTRY vuoto).
#   3 alias    f1tenth:<branch>-<sha7> sulla stessa immagine (stesso ID)
#   4 deploy   tools/f1t deploy --switch-mode: pin, restart della unit, health check, rollback automatico
#   5 script   scripts/orin_run_tests.sh dello stesso commit -> ~/.local/bin/orin_run_tests sull'Orin
#   6 verifica /opt/f1tenth_ws/SOURCES nell'immagine (il file di versione: commit, branch, tree hash)
#              == commit di linus, pin == alias, script installato == commit -> OK / FAIL
set -euo pipefail
# f1t deploy stampa il diff dei parametri con git diff: senza questo git apre less e il deploy
# resta fermo in attesa di 'q' a meta' (prima del pin).
export GIT_PAGER=cat PAGER=cat
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_deploy
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/lib/orin_ui.sh
source "$ROOT/scripts/lib/orin_ui.sh"

DRY=() FORCE=() COMMIT=HEAD POLL_S=${POLL_S:-15}
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=(--dry-run) ;;
    --force-untested) FORCE=(--force-untested) ;;
    -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
SHA=$(git -C "$ROOT" rev-parse --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
SHORT=${SHA:0:7}
BRANCH=$(git -C "$ROOT" branch --points-at "$SHA" --format='%(refname:short)' | head -1)
BRANCH=${BRANCH:-detached}
SUFFIX=; [ "$F1T_IMAGE_RUST" = skip ] && SUFFIX=-norust
IMAGE=$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX$SHORT$SUFFIX
TAG=$F1T_IMAGE_REPO:${BRANCH//\//-}-$SHORT
dry() { [ ${#DRY[@]} = 1 ]; }
img_label() { orin_ssh "docker image inspect -f '{{index .Config.Labels \"$2\"}}' $1" 2>/dev/null || true; }

ui_section "orin_deploy: configurazione"
ui_kv "commit" "$SHA"
ui_kv "messaggio" "$(git -C "$ROOT" log -1 --format=%s "$SHA")"
ui_kv "branch" "$BRANCH"
ui_kv "immagine (f1t build)" "$IMAGE"
ui_kv "alias" "$TAG"
ui_kv "Orin" "$(orin_target) (ssh: $F1T_ORIN_SSH_OPTS)"
ui_kv "modalita'" "${DRY[*]:-reale} ${FORCE[*]:-}"
ui_kv "log" "$F1T_LOG_FILE"
ui_steps 6

ui_step "stato dell'Orin prima del deploy"
orin_up || log_die "Orin irraggiungibile: ssh $(orin_target)"
ui_kv "host" "$(orin_ssh 'hostname; uptime -p' | tr '\n' ' ')"
ui_kv "immagine fissata" "$(orin_ssh "awk '{printf \"%s=%s \", \$1, substr(\$2,1,19)}' ~/.config/f1tenth/car_image" 2>/dev/null || echo assente)"
ui_kv "modalita' auto" "$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?')"
ui_kv "container f1tenth_car" "$(orin_ssh "docker ps -a --filter name=^f1tenth_car\$ --format '{{.Image}} {{.Status}}'" || echo ?)"
ui_kv "stack_args" "$(orin_ssh 'cat ~/.config/f1tenth_car/stack_args 2>/dev/null || echo assente')"
ui_kv "disco Docker / scratch" "$(orin_ssh 'df -h --output=avail /var/lib/docker /scratch/fabiocar | tail -2 | tr -s " \n" " "')"
ui_step_end

ui_step "build sull'Orin (tools/f1t build)"
tested=$(img_label "$IMAGE" f1tenth.tested)
exists=$(orin_ssh "docker image inspect -f '{{.Id}}' $IMAGE" 2>/dev/null || true)
ui_kv "immagine sull'Orin" "${exists:-assente} ${tested:+(tested=$tested)}"
if [ -n "$exists" ] && { [ "$tested" = true ] || [ ${#FORCE[@]} = 1 ]; }; then
  log_ok "build saltata: $IMAGE c'e' gia' ($([ "$tested" = true ] && echo testata || echo 'NON testata, --force-untested'))"
elif dry; then
  ui_run "$ROOT/tools/f1t" build --dry-run "$SHA"
else
  ui_run "$ROOT/tools/f1t" build --detach "$SHA"
  t0=$(date +%s)
  while :; do
    IFS='|' read -r st phase k m text < <(orin_script "$ROOT/scripts/lib/orin_build_progress.sh" "$F1T_ORIN_BUILD_ROOT" "$SHORT" 2>/dev/null || echo "ssh?|?|0|0|ssh lento, riprovo")
    case "$phase" in
      build) if [ "${m:-0}" -gt 0 ]; then ui_bar "$k" "$m" "build $(ui_elapsed "$t0") ${text}"; else ui_spin "$t0" "build: $text"; fi ;;
      test)  ui_spin "$t0" "pytest nell'immagine: $text" ;;
      label) ui_spin "$t0" "label tested: $text" ;;
      *)     ui_spin "$t0" "$st: $text" ;;
    esac
    case "$st" in ok*|fail*|none) break ;; esac
    sleep "$POLL_S"
  done
  ui_bar_done
  ui_run "$ROOT/tools/f1t" build --follow "$SHORT" || true     # copia il log completo in logs/build/
  case "$st" in
    ok*) log_ok "build + test + label OK in $(ui_elapsed "$t0")" ;;
    *) if [ ${#FORCE[@]} = 1 ] && [ -n "$(orin_ssh "docker image inspect -f '{{.Id}}' $IMAGE" 2>/dev/null)" ]; then
         log_warn "f1t build: $st. Immagine costruita, si prosegue per --force-untested"
       else
         log_die "f1t build: $st (log in logs/build/). Immagine costruita ma non testata? riprova con --force-untested"
       fi ;;
  esac
fi
ui_step_end

ui_step "alias $TAG"
if dry; then log_info "dry-run: docker tag $IMAGE $TAG"
else ui_run orin_ssh docker tag "$IMAGE" "$TAG"; ui_kv "ID" "$(orin_ssh "docker image inspect -f '{{.Id}}' $TAG")"; fi
ui_step_end

ui_step "deploy (tools/f1t deploy --switch-mode ${FORCE[*]:-})"
ui_run "$ROOT/tools/f1t" deploy "${DRY[@]}" "${FORCE[@]}" --switch-mode "$IMAGE" \
  || { dry && log_warn "dry-run: deploy non pronto (atteso finche' l'immagine non esiste)"; } \
  || log_die "deploy fallito (f1t deploy fa gia' il rollback all'immagine precedente)"
ui_step_end

ui_step "orin_run_tests dal commit $SHORT"
if dry; then
  log_info "dry-run: git show $SHORT:scripts/orin_run_tests.sh -> ~/.local/bin/orin_run_tests"
else
  git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | orin_ssh 'install -D -m 755 /dev/stdin ~/.local/bin/orin_run_tests'
  ui_kv "installato" "\$HOME/.local/bin/orin_run_tests ($(orin_ssh 'sha256sum .local/bin/orin_run_tests' | cut -c1-12))"
fi
ui_step_end

ui_step "verifica linus == Orin"
if dry; then log_info "dry-run: nessuna verifica"; ui_bar 6 6 "dry-run finito in $(ui_elapsed)"; ui_bar_done; exit 0; fi
pin_id=$(orin_ssh "awk '\$1==\"id\"{print \$2}' ~/.config/f1tenth/car_image")
tag_id=$(orin_ssh "docker image inspect -f '{{.Id}}' $TAG" 2>/dev/null || echo none)
sources=$(orin_ssh docker run --rm --network none --entrypoint cat "$TAG" /opt/f1tenth_ws/SOURCES 2>/dev/null || echo "commit none")
img_sha=$(awk '$1=="commit"{print $2}' <<<"$sources")
want_sum=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have_sum=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || echo none)
ui_section "SOURCES nell'immagine ($TAG)"
grep -v '^submodule' <<<"$sources" | sed 's/^/  /' >&2
ui_kv "label tested" "$(img_label "$TAG" f1tenth.tested) $(img_label "$TAG" f1tenth.tested_at)"
ui_kv "container" "$(orin_ssh "docker ps --filter name=^f1tenth_car\$ --format '{{.Image}} {{.Status}}'")"
check_record "$([ "$pin_id" = "$tag_id" ] && echo OK || echo FAIL)" "pin = $TAG" "${tag_id:7:12}"
check_record "$([ "$img_sha" = "$SHA" ] && echo OK || echo FAIL)" "SOURCES nell'immagine" "${img_sha:0:12}"
check_record "$([ "$want_sum" = "$have_sum" ] && echo OK || echo FAIL)" "orin_run_tests" "${have_sum:0:12}"
ui_bar 6 6 "finito in $(ui_elapsed)"; ui_bar_done
if summary "orin_deploy $TAG"; then log_ok "linus $TAG == Orin $TAG: OK"; else log_die "linus $TAG != Orin: FAIL"; fi
