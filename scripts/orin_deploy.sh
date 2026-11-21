#!/usr/bin/env bash
# orin_deploy.sh - da linus: immagine Jazzy dell'Orin per la campagna a 3 comandi, deploy, verifica.
#
#   scripts/orin_deploy.sh [--dry-run] [--force-untested] [--no-allow-drift] [COMMIT]
#                                                        default COMMIT = HEAD (committato)
#
#   --dry-run          controlla tutto, non cambia niente
#   --force-untested   accetta un'immagine senza label tested (oggi serve sempre: f1tenth_behavior e
#                      f1tenth_bringup hanno test che falliscono gia' nell'immagine c0c1397); f1t deploy
#                      lo scrive in deployed.json
#   --no-allow-drift   NON passa --allow-drift: il container non parte se il checkout nativo dell'Orin
#                      ha modifiche che l'immagine non ha. Default: --allow-drift, scritto nel log
#
# L'immagine: si riusa un'immagine gia' sull'Orin con lo stesso contenuto di COMMIT (stessi src/,
# docker/orin, scripts/env: un commit che cambia solo questi script non si ricostruisce), prima quella
# fissata. Se non c'e': tools/f1t build dell'ultimo commit che cambia quel contenuto.
#
# Passi (barra generale + barra/attesa per ogni passo, ogni comando stampato, log completo in
# logs/tools/orin_deploy_<data>.log):
#   1 stato    Orin, immagine fissata, modalita', container in esecuzione (un container f1tenth
#              estraneo, es. di test, ferma tutto prima di toccare l'auto), drift, stack_args
#   2 immagine riuso, oppure tools/f1t build --detach: bundle git -> build SULL'ORIN (il Dockerfile
#              linka le librerie L4T r36.5 dell'host: su linus non si costruisce) -> pytest per
#              package -> label tested. Barra dai passi [K/M] di BuildKit. Nessun registry.
#   3 alias    f1tenth:<branch>-<sha7> sulla stessa immagine (stesso ID)
#   4 script   scripts/orin_run_tests.sh di COMMIT -> ~/.local/bin/orin_run_tests (prima del deploy)
#   5 deploy   tools/f1t deploy --switch-mode --allow-drift: pin, avvio, health check, rollback
#              automatico. Saltato se quell'immagine e' gia' fissata e il container gira sano
#   6 verifica pin == alias, container sano con quell'immagine, SOURCES nell'immagine (il file di
#              versione) equivalente a COMMIT, orin_run_tests == COMMIT -> OK / FAIL
# Exit: 0 OK, 1 fallito (l'auto resta sull'immagine di prima), 2 argomenti.
set -euo pipefail
export GIT_PAGER=cat PAGER=cat      # git diff non deve aprire less dentro f1t deploy
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_deploy
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/lib/orin_ui.sh
source "$ROOT/scripts/lib/orin_ui.sh"
# shellcheck source=scripts/lib/orin_common.sh
source "$ROOT/scripts/lib/orin_common.sh"

DRY=() FORCE=() DRIFT=(--allow-drift) COMMIT=HEAD POLL_S=${POLL_S:-15}
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=(--dry-run) ;;
    --force-untested) FORCE=(--force-untested) ;;
    --no-allow-drift) DRIFT=() ;;
    -h|--help) sed -n '2,31p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a (scripts/orin_deploy.sh -h)" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
cfg_require config || exit 1
SHA=$(git -C "$ROOT" rev-parse -q --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
dry() { [ ${#DRY[@]} = 1 ]; }
img_label() { orin_ssh "docker image inspect -f '{{index .Config.Labels \"$2\"}}' $1" 2>/dev/null || true; }
img_id() { orin_ssh "docker image inspect -f '{{.Id}}' $1" 2>/dev/null || true; }

ui_section "orin_deploy: configurazione"
ui_kv "commit" "$SHA"
ui_kv "messaggio" "$(git -C "$ROOT" log -1 --format=%s "$SHA")"
ui_kv "branch" "$(branch_of "$SHA")"
ui_kv "contenuto immagine da" "$(image_commit "$SHA" | cut -c1-12) (ultimo commit che cambia ${IMAGE_PATHS[*]})"
ui_kv "Orin" "$(orin_target)"
ui_kv "opzioni" "${DRY[*]:-reale} ${FORCE[*]:-} ${DRIFT[*]:---no-allow-drift}"
ui_kv "log" "$F1T_LOG_FILE"
ui_steps 6

# ---------------------------------------------------------------- 1 stato
ui_step "stato dell'Orin prima del deploy"
orin_up || log_die "Orin irraggiungibile: ssh $(orin_target) (Tailscale? ConnectTimeout in F1T_ORIN_SSH_OPTS)"
ui_kv "host" "$(orin_ssh 'hostname; uptime -p' | tr '\n' ' ')"
ui_kv "immagine fissata" "$(orin_ssh "awk '\$1==\"tag\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || echo assente)"
ui_kv "modalita' auto" "$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?')"
ui_kv "stack_args" "$(orin_ssh 'cat ~/.config/f1tenth_car/stack_args 2>/dev/null || echo assente')"
ui_kv "drift" "$(orin_drift_summary)$( [ ${#DRIFT[@]} = 1 ] && echo ': si parte con --allow-drift')"
ui_kv "spazio Docker / scratch" "$(orin_ssh 'df -h --output=avail /var/lib/docker /scratch/fabiocar | tail -2 | tr -s " \n" " "')"
running=$(orin_running_containers)
ui_kv "container in esecuzione" "${running:-nessuno}"
stray=$(awk -v c="$F1T_ORIN_CONTAINER" -v r="$F1T_IMAGE_REPO:" '$1 != c && index($2, r) == 1 {print $1}' <<<"$running")
[ -z "$stray" ] || log_die "container f1tenth estranei in esecuzione: $stray. Il guard di build e deploy li tratta come uno stack: fermali (docker stop $stray) e rilancia"
ui_step_end

# ---------------------------------------------------------------- 2 immagine
ui_step "immagine"
if reuse=$(orin_reusable_image "$SHA"); then
  IMAGE=${reuse%% *} IMG_SHA=${reuse##* }
  log_ok "riuso $IMAGE: stesso contenuto di ${SHA:0:12} (costruita da ${IMG_SHA:0:12}), nessuna build"
else
  IMG_SHA=$(image_commit "$SHA") IMAGE=$(image_tag_of "$IMG_SHA")
  log_info "nessuna immagine equivalente sull'Orin: build di $IMAGE"
  if dry; then
    ui_run "$ROOT/tools/f1t" build --dry-run "$IMG_SHA" || log_warn "dry-run: f1t build non pronto (sopra)"
  else
    short=${IMG_SHA:0:7}
    ui_run "$ROOT/tools/f1t" build --detach "$IMG_SHA"
    t0=$(date +%s) st=running
    while :; do
      IFS='|' read -r st phase k m text < <(orin_script "$ROOT/scripts/lib/orin_build_progress.sh" "$F1T_ORIN_BUILD_ROOT" "$short" 2>/dev/null \
        || echo "running|ssh|0|0|ssh lento, riprovo")
      case "$phase" in
        build) if [ "${m:-0}" -gt 0 ]; then ui_bar "$k" "$m" "build $(ui_elapsed "$t0") $text"; else ui_spin "$t0" "build: $text"; fi ;;
        test)  ui_spin "$t0" "pytest nell'immagine (un package per volta): $text" ;;
        label) ui_spin "$t0" "label tested: $text" ;;
        *)     ui_spin "$t0" "$st: $text" ;;
      esac
      case "$st" in ok*|fail*|none) break ;; esac
      sleep "$POLL_S"
    done
    ui_bar_done
    ui_run "$ROOT/tools/f1t" build --follow "$short" >/dev/null 2>&1 || true   # copia il log in logs/build/
    # shellcheck disable=SC2012  # nomi <sha7>_<data>.log, senza spazi
    blog=$(ls -t "$ROOT"/logs/build/"$short"_*.log 2>/dev/null | head -1 || true)
    [ -z "$blog" ] || { ui_section "test per package ($blog)"; grep -E '^PKG |^    (FAILED|ERROR)' "$blog" | sed 's/^/  /' >&2 || true; }
    case "$st" in
      ok*) log_ok "build + test + label OK in $(ui_elapsed "$t0")" ;;
      "fail test"*) [ -n "$(img_id "$IMAGE")" ] || log_die "f1t build: $st e immagine assente"
                    [ ${#FORCE[@]} = 1 ] || log_die "f1t build: test falliti (righe PKG sopra). L'immagine c'e': rilancia con --force-untested"
                    log_warn "f1t build: test falliti, si prosegue per --force-untested" ;;
      *) log_die "f1t build: $st (log in ${blog:-logs/build/})" ;;
    esac
  fi
fi
TESTED=$(img_label "$IMAGE" f1tenth.tested)
ui_kv "immagine" "$IMAGE $(img_id "$IMAGE" | cut -c8-19) tested=${TESTED:-no}"
if ! dry && [ "$TESTED" != true ] && [ ${#FORCE[@]} = 0 ]; then
  log_die "$IMAGE non ha la label tested: rilancia con --force-untested (l'auto non e' stata toccata)"
fi
BRANCH=$(branch_of "$SHA")
TAG=$F1T_IMAGE_REPO:${BRANCH//\//-}-${IMG_SHA:0:7}
ui_step_end

# ---------------------------------------------------------------- 3 alias
ui_step "alias $TAG"
if dry; then log_info "dry-run: docker tag $IMAGE $TAG"
else ui_run orin_ssh docker tag "$IMAGE" "$TAG"; ui_kv "ID" "$(img_id "$TAG")"; fi
ui_step_end

# ---------------------------------------------------------------- 4 script
ui_step "orin_run_tests da ${SHA:0:12}"
if dry; then
  log_info "dry-run: git show ${SHA:0:7}:scripts/orin_run_tests.sh -> ~/.local/bin/orin_run_tests"
else
  git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | orin_ssh 'install -D -m 755 /dev/stdin ~/.local/bin/orin_run_tests'
  ui_kv "installato" "~fabiocar/.local/bin/orin_run_tests ($(orin_ssh 'sha256sum .local/bin/orin_run_tests' | cut -c1-12))"
fi
ui_step_end

# ---------------------------------------------------------------- 5 deploy
ui_step "deploy (tools/f1t deploy --switch-mode ${DRIFT[*]:-} ${FORCE[*]:-})"
pin_tag=$(orin_ssh "awk '\$1==\"tag\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || true)
ctr=$(orin_ssh "docker inspect -f '{{.Image}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{end}}' $F1T_ORIN_CONTAINER" 2>/dev/null || true)
if [ "$pin_tag" = "$IMAGE" ] && [ "$ctr" = "$(img_id "$IMAGE") running healthy" ]; then
  log_ok "$IMAGE e' gia' fissata e il container gira sano con lei: nessun riavvio"
else
  ui_run "$ROOT/tools/f1t" deploy "${DRY[@]}" "${FORCE[@]}" "${DRIFT[@]}" --switch-mode "$IMAGE" \
    || { dry && log_warn "dry-run: f1t deploy non pronto (sopra)"; } \
    || log_die "deploy fallito (f1t deploy fa gia' il rollback all'immagine precedente; log sopra)"
fi
ui_step_end

# ---------------------------------------------------------------- 6 verifica
ui_step "verifica linus == Orin"
if dry; then log_info "dry-run: nessuna verifica"; ui_bar 6 6 "dry-run finito in $(ui_elapsed)"; ui_bar_done; exit 0; fi
pin_id=$(orin_ssh "awk '\$1==\"id\"{print \$2}' ~/.config/f1tenth/car_image")
tag_id=$(img_id "$TAG")
ctr=$(orin_ssh "docker inspect -f '{{.Image}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{end}}' $F1T_ORIN_CONTAINER" 2>/dev/null || echo "none")
sources=$(orin_ssh "docker exec $F1T_ORIN_CONTAINER cat /opt/f1tenth_ws/SOURCES" 2>/dev/null || echo "commit none")
src_sha=$(awk '$1=="commit"{print $2}' <<<"$sources")
want_sum=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have_sum=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || echo none)
ui_section "SOURCES nel container ($TAG)"
grep -v '^submodule' <<<"$sources" | sed 's/^/  /' >&2
ui_kv "label tested" "${TESTED:-no} $(img_label "$TAG" f1tenth.tested_at)"
ok() { if "$@"; then echo OK; else echo FAIL; fi; }
check_record "$(ok [ "$pin_id" = "$tag_id" ])" "pin = $TAG" "${tag_id:7:12}"
check_record "$(ok [ "$ctr" = "$tag_id running healthy" ])" "container sano con l'immagine" "${ctr:7:12} ${ctr#* }"
check_record "$(ok same_image "${src_sha:-x}" "$SHA")" "SOURCES equivalente al commit" "${src_sha:0:12} ~ ${SHA:0:12}"
check_record "$(ok [ "$want_sum" = "$have_sum" ])" "orin_run_tests = commit" "${have_sum:0:12}"
ui_bar 6 6 "finito in $(ui_elapsed)"; ui_bar_done
if summary "orin_deploy $TAG"; then
  log_ok "linus ${SHA:0:12} == Orin $TAG: OK. Sull'Orin (shell dell'host, non nel container): ~/.local/bin/orin_run_tests"
else
  log_die "linus ${SHA:0:12} != Orin $TAG: FAIL (righe FAIL sopra)"
fi
