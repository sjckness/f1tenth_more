# shellcheck shell=bash
# scripts/car/lib/car_build.sh - i passi comuni di scripts/car/build.sh e scripts/car/deploy.sh (linus).
# Dopo tools/core/lib/{config,remote}.sh, orin_ui.sh, orin_common.sh.
#
#   car_state_step                 passo "stato dell'Orin": stampa pin, modalita', stack_args, drift,
#                                  spazio, container; si ferma se gira un container f1tenth estraneo.
#                                  Imposta RUNNING (righe "nome immagine stato").
#   car_image_step SHA             passo "immagine": riuso di un'immagine equivalente gia' sull'Orin,
#                                  oppure tools/f1t build --detach seguita con la barra. Imposta IMAGE,
#                                  IMG_SHA, TESTED. Legge DRY, FORCE, STOP (array, dal chiamante).
#   car_alias_step SHA             passo "alias": f1tenth:<branch>-<sha7> sulla stessa immagine. TAG.
# Variabili lette: POLL_S, BUILD_MAX_S, BUILD_STALE_S.
[ -n "${_CAR_BUILD_SH:-}" ] && return 0
_CAR_BUILD_SH=1
POLL_S=${POLL_S:-15} BUILD_MAX_S=${BUILD_MAX_S:-7200} BUILD_STALE_S=${BUILD_STALE_S:-1800}

_dry() { [ "${#DRY[@]}" = 1 ]; }

car_state_step() {
  ui_step "stato dell'Orin"
  orin_up || log_die "Orin irraggiungibile: ssh $(orin_target) (Tailscale? ConnectTimeout in F1T_ORIN_SSH_OPTS)"
  ui_kv "host" "$(orin_ssh 'hostname; uptime -p' | tr '\n' ' ')"
  ui_kv "immagine fissata" "$(orin_ssh "awk '\$1==\"tag\"{print \$2}' $F1T_ORIN_PIN_FILE" 2>/dev/null || echo assente)"
  ui_kv "modalita' auto" "$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?')"
  ui_kv "stack_args" "$(orin_ssh 'cat ~/.config/f1tenth_car/stack_args 2>/dev/null || echo assente')"
  ui_kv "drift" "$(orin_drift_summary)"
  ui_kv "spazio Docker / scratch" "$(orin_ssh "df -h --output=avail /var/lib/docker $F1T_ORIN_BUILD_ROOT | tail -2 | tr -s ' \n' ' '" 2>/dev/null || echo '?')"
  RUNNING=$(orin_running_containers)
  ui_kv "container in esecuzione" "${RUNNING:-nessuno}"
  local stray
  stray=$(awk -v c="$F1T_ORIN_CONTAINER" -v r="$F1T_IMAGE_REPO:" '$1 != c && index($2, r) == 1 {print $1}' <<<"$RUNNING")
  [ -z "$stray" ] || log_die "container f1tenth estranei in esecuzione: $stray. Il guard di build e deploy li tratta come uno stack: fermali (docker stop $stray) e rilancia"
  ui_step_end
}

_car_follow_build() {   # _car_follow_build SHA: segue la build staccata fino alla fine; imposta BUILD_ST
  local short=${1:0:7} t0 st=running phase k m age text nssh=0 el
  t0=$(date +%s)
  while :; do
    IFS='|' read -r st phase k m age text < <(orin_script "$F1T_ROOT/scripts/car/lib/orin_build_progress.sh" "$F1T_ORIN_BUILD_ROOT" "$short" 2>/dev/null \
      || echo "running|ssh|0|0|-1|ssh non risponde, riprovo")
    if [ "$phase" = ssh ]; then nssh=$((nssh + 1)); else nssh=0; fi
    case "$phase" in
      build) if [ "${m:-0}" -gt 0 ]; then ui_bar "$k" "$m" "build $(ui_elapsed "$t0") $text"; else ui_spin "$t0" "build: $text"; fi ;;
      test)  ui_spin "$t0" "pytest nell'immagine (un package per volta): $text" ;;
      label) ui_spin "$t0" "label tested: $text" ;;
      *)     ui_spin "$t0" "$st: $text" ;;
    esac
    case "$st" in ok*|fail*|none) break ;; esac
    el=$(( $(date +%s) - t0 ))
    if [ "$el" -ge "$BUILD_MAX_S" ] || [ "${age:--1}" -ge "$BUILD_STALE_S" ] || [ $((nssh * POLL_S)) -ge 600 ]; then
      ui_bar_done
      log_die "build non conclusa: $(ui_elapsed "$t0") trascorsi, log fermi da ${age}s, ssh fallito ${nssh} volte. La build gira staccata: tools/f1t build --follow $short; stato sull'Orin in $F1T_ORIN_BUILD_ROOT/$short.status"
    fi
    sleep "$POLL_S"
  done
  ui_bar_done
  ui_run "$F1T_ROOT/tools/f1t" build --follow "$short" >/dev/null 2>&1 || true   # copia il log in logs/build/
  local blog
  # shellcheck disable=SC2012  # nomi <sha7>_<data>.log, senza spazi
  blog=$(ls -t "$F1T_ROOT"/logs/build/"$short"_*.log 2>/dev/null | head -1 || true)
  [ -z "$blog" ] || { ui_section "test per package ($blog)"; grep -E '^PKG |^    (FAILED|ERROR)' "$blog" | sed 's/^/  /' >&2 || true; }
  log_info "build finita in $(ui_elapsed "$t0"): $st"
  BUILD_ST=$st BUILD_LOG=${blog:-logs/build/}
}

car_image_step() {
  local sha=$1 reuse
  ui_step "immagine"
  if reuse=$(orin_reusable_image "$sha"); then
    IMAGE=${reuse%% *} IMG_SHA=${reuse##* }
    log_ok "riuso $IMAGE: stesso contenuto di ${sha:0:12} (costruita da ${IMG_SHA:0:12}), nessuna build"
  else
    IMG_SHA=$(image_commit "$sha") IMAGE=$(image_tag_of "$IMG_SHA")
    log_info "nessuna immagine equivalente sull'Orin: build di $IMAGE"
    if ! _dry && [ "${#STOP[@]}" = 0 ] && grep -q "^$F1T_ORIN_CONTAINER " <<<"${RUNNING:-}"; then
      log_die "serve una build ma il container $F1T_ORIN_CONTAINER gira (f1t build lo rifiuta): rilancia con --stop-stack per fermarlo, oppure fermalo tu (car-container stop)"
    fi
    if _dry; then
      ui_run "$F1T_ROOT/tools/f1t" build --dry-run "$IMG_SHA" || log_warn "dry-run: f1t build non pronto (sopra)"
    else
      ui_run "$F1T_ROOT/tools/f1t" build --detach "${STOP[@]}" "$IMG_SHA"
      _car_follow_build "$IMG_SHA"
      case "$BUILD_ST" in
        ok*) log_ok "build + test + label OK" ;;
        "fail test"*) [ -n "$(img_id "$IMAGE")" ] || log_die "f1t build: $BUILD_ST e immagine assente"
                      [ "${#FORCE[@]}" = 1 ] || log_die "f1t build: test falliti (righe PKG sopra). L'immagine c'e': rilancia con --force-untested"
                      log_warn "f1t build: test falliti, si prosegue per --force-untested" ;;
        *) log_die "f1t build: $BUILD_ST (log in $BUILD_LOG)" ;;
      esac
    fi
  fi
  TESTED=$(img_label "$IMAGE" f1tenth.tested)
  ui_kv "immagine" "$IMAGE $(img_id "$IMAGE" | cut -c8-19) tested=${TESTED:-no}"
  if ! _dry && [ "$TESTED" != true ] && [ "${#FORCE[@]}" = 0 ]; then
    log_die "$IMAGE non ha la label tested: rilancia con --force-untested (l'auto non e' stata toccata)"
  fi
  ui_step_end
}

car_alias_step() {
  local sha=$1 branch
  branch=$(branch_of "$sha")
  TAG=$F1T_IMAGE_REPO:${branch//\//-}-${IMG_SHA:0:7}
  ui_step "alias $TAG"
  if _dry; then log_info "dry-run: docker tag $IMAGE $TAG"
  else ui_run orin_ssh docker tag "$IMAGE" "$TAG"; ui_kv "ID" "$(img_id "$TAG")"; fi
  ui_step_end
}
