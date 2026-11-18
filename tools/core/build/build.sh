#!/usr/bin/env bash
# desc: build dell'immagine dell'Orin da un commit di linus (build remota sull'Orin), test e label tested=true
# alias: container-build
#
#   f1t build [--dry-run] [--dev] [--rust build|skip] [--test changed|all|none] [--stop-stack]
#             [--detach] [COMMIT]                       default COMMIT = HEAD di questo checkout
#   f1t build --follow SHA7                             segue una build gia' partita
#
# L'immagine si costruisce SULL'ORIN: il Dockerfile (docker/orin) linka le librerie L4T r36.5
# dell'host, che su linus non esistono (vedi docker/orin/Dockerfile, "BUILD REQUIREMENTS"). Linus
# decide COSA si costruisce (un commit, mai file non committati); l'Orin riceve il commit in un
# bundle git, lo costruisce in un worktree temporaneo con il docker/orin/build.sh di quel commit,
# esegue pytest dei package cambiati nell'immagine (rete none) e solo se passano la etichetta
# f1tenth.tested=true. Tag: f1tenth:jazzy-orin-<sha7>[-norust]. Cache Docker: solo i sorgenti
# cambiati si ricostruiscono (~10 min; ~1 h a freddo).
#
# --dry-run    solo letture: controlli locali e sull'Orin, cosa verrebbe trasferito e lanciato
# --dev        accetta un working tree sporco (l'immagine contiene comunque solo il commit)
# --stop-stack la build rifiuta se uno stack gira: ferma il container dell'auto (unit systemd)
# --detach     non aspetta la fine (poi: f1t build --follow SHA7)
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

DRY=0 DEV=0 STOP=0 DETACH=0 FOLLOW='' RUST='' TEST='' COMMIT=HEAD
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --dev) DEV=1 ;;
    --rust) RUST=${2:?--rust build|skip}; shift ;;
    --test) TEST=${2:?--test changed|all|none}; shift ;;
    --stop-stack) STOP=1 ;;
    --detach) DETACH=1 ;;
    --follow) FOLLOW=${2:?--follow SHA7}; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $1 (f1t build -h)" 2 ;;
    *) COMMIT=$1 ;;
  esac
  shift
done

cfg_load
cfg_require config || exit 1
RUST=${RUST:-$F1T_IMAGE_RUST} TEST=${TEST:-$F1T_BUILD_TEST}
case "$RUST" in build|skip) ;; *) log_die "--rust deve essere build o skip, non '$RUST'" 2 ;; esac
case "$TEST" in changed|all|none) ;; *) log_die "--test deve essere changed, all o none, non '$TEST'" 2 ;; esac
XFER=$F1T_ORIN_XFER ROOT=$F1T_ORIN_BUILD_ROOT
LOGDIR=$F1T_ROOT/logs/build

follow() {
  local short=$1 line st
  log_info "seguo la build $short sull'Orin (Ctrl-C non la ferma: gira staccata)"
  while :; do
    line=$(orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" status "$ROOT" "$short" 2>&1) || line="ssh fallito | $line"
    st=${line%% *}
    case "$st" in
      ok) log_ok "build finita: ${line%% |*}"; break ;;
      fail|none) log_fail "build: $line"; break ;;
      *) log_info "${line#*| }" ;;
    esac
    sleep 30
  done
  mkdir -p "$LOGDIR"
  orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" log "$ROOT" "$short" > "$LOGDIR/${short}_$(date +%Y%m%d_%H%M%S).log" 2>/dev/null || true
  log_info "log della build copiato in $LOGDIR/"
  [ "$st" = ok ]
}

if [ -n "$FOLLOW" ]; then
  cfg_print F1T_ORIN_HOST F1T_ORIN_BUILD_ROOT
  follow "$FOLLOW"; exit $?
fi

SHA=$(git -C "$F1T_ROOT" rev-parse --verify "${COMMIT}^{commit}" 2>/dev/null) || log_die "commit sconosciuto: $COMMIT" 2
SHORT=${SHA:0:7}
SUFFIX=; [ "$RUST" = skip ] && SUFFIX=-norust
IMAGE=$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX$SHORT$SUFFIX
# shellcheck disable=SC2034  # stampate da cfg_print (lettura indiretta ${!k})
F1T_BUILD_COMMIT="$SHA ($(git -C "$F1T_ROOT" log -1 --format='%s' "$SHA" | cut -c1-60))"
# shellcheck disable=SC2034  # stampate da cfg_print (lettura indiretta ${!k})
F1T_BUILD_IMAGE=$IMAGE F1T_BUILD_TESTS=$TEST
cfg_print F1T_ORIN_HOST F1T_BUILD_COMMIT F1T_BUILD_IMAGE F1T_BUILD_TESTS F1T_ORIN_BUILD_REPO F1T_ORIN_BUILD_ROOT F1T_REGISTRY
[ "$DRY" = 0 ] || log_warn "DRY-RUN: nessun trasferimento, nessuna build"
log_steps 5

log_step "Controlli su linus"
if [ "$DEV" = 1 ]; then check_warn "working tree committato" probe_git_clean "$F1T_ROOT"
else check "working tree committato" probe_git_clean "$F1T_ROOT"; fi
check "docker/orin/build.sh nel commit" bash -c "git -C '$F1T_ROOT' cat-file -e '$SHA:docker/orin/build.sh' && echo presente || { echo 'assente: il commit non sa costruire l immagine dell Orin'; exit 1; }"
if ! checks_ok; then summary "f1t build"; log_fail "fermo: committa (o --dev) e usa un commit con docker/orin/build.sh"; exit 1; fi

log_step "Orin"
if ! orin_up; then
  check_record FAIL "ssh $(orin_target)" "non risponde"
  summary "f1t build"; exit 1
fi
check_record OK "ssh $(orin_target)" "ok"
declare -A PRE=()
while IFS='=' read -r k v; do [ -n "$k" ] && PRE[$k]=$v; done < <(orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" pre "$F1T_ORIN_BUILD_REPO" "$SHA" 2>/dev/null || true)
case "${PRE[l4t]:-}" in *"R36 (release), REVISION: 5"*) check_record OK "host L4T r36.5" "${PRE[l4t]}" ;; *) check_record FAIL "host L4T r36.5" "${PRE[l4t]:-?}" ;; esac
[ "${PRE[docker_free_gb]:-0}" -ge 40 ] && check_record OK "spazio Docker" "${PRE[docker_free_gb]} GB" || check_record FAIL "spazio Docker" "${PRE[docker_free_gb]:-?} GB (< 40)"
[ "${PRE[root_free_gb]:-0}" -ge 5 ] && check_record OK "spazio /" "${PRE[root_free_gb]} GB" || check_record FAIL "spazio /" "${PRE[root_free_gb]:-?} GB (< 5)"
[ "${PRE[inputs]:-0}" -gt 0 ] && check_record OK "input binari (p5d_downloads)" "${PRE[inputs]} file" || check_record FAIL "input binari (p5d_downloads)" "assenti"
[ "${PRE[running_builds]:-0}" = 0 ] && check_record OK "altre build f1t in corso" "nessuna" || check_record FAIL "altre build f1t in corso" "${PRE[running_builds]}"
stack="${PRE[stack_procs]:-0} processi ROS/DS, container auto: ${PRE[car_container]:-assente}"
if [ "${PRE[stack_procs]:-0}" = 0 ] && [ -z "${PRE[car_container]:-}" ]; then check_record OK "nessuno stack in esecuzione" "$stack"
elif [ "$STOP" = 1 ]; then check_record WARN "nessuno stack in esecuzione" "$stack: --stop-stack lo ferma"
else check_record FAIL "nessuno stack in esecuzione" "$stack: la build carica tutti i core ~1 h; --stop-stack"; fi

log_step "Commit sull'Orin"
if orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" has "$F1T_ORIN_BUILD_REPO" "$SHA"; then
  check_record OK "commit nel repo di build" "gia' presente"
  NEED_BUNDLE=0
else
  NEED_BUNDLE=1
  n=$(git -C "$F1T_ROOT" rev-list --count "$SHA" --not --remotes=orin 2>/dev/null || echo '?')
  check_record OK "commit nel repo di build" "da trasferire: bundle di $n commit (--not --remotes=orin)"
fi
if ! checks_ok; then summary "f1t build"; exit 1; fi

if [ "$DRY" = 1 ]; then
  log_step "Build (dry-run)"
  log_info "verrebbe lanciato sull'Orin: orin_build_job.sh $SHA $RUST $TEST $F1T_ORIN_BUILD_REPO $ROOT/$SHORT"
  log_info "  = worktree temporaneo, ORIN_RUST=$RUST docker/orin/build.sh $SHA, pytest ($TEST), label f1tenth.tested=true"
  [ "${PRE[build_sh_dry]:-0}" -gt 0 ] && log_info "  il build.sh del commit ha --dry-run: per i suoi controlli usalo sull'Orin dopo il trasferimento"
  check_skip "build" "dry-run"
  log_step "Risultato"
  summary "f1t build (dry-run)"; exit $?
fi

log_step "Trasferimento e avvio"
if [ "$NEED_BUNDLE" = 1 ]; then
  tmpd=$(mktemp -d); trap 'git -C "$F1T_ROOT" update-ref -d "refs/f1t/build/$SHORT" 2>/dev/null || true; rm -rf -- "${tmpd:?}"' EXIT
  git -C "$F1T_ROOT" update-ref "refs/f1t/build/$SHORT" "$SHA"
  git -C "$F1T_ROOT" bundle create -q "$tmpd/f1t-$SHORT.bundle" "refs/f1t/build/$SHORT" --not --remotes=orin \
    || log_die "git bundle fallito"
  orin_ssh mkdir -p "$XFER"
  orin_scp "$tmpd/f1t-$SHORT.bundle" "$XFER/f1t-$SHORT.bundle" || log_die "scp del bundle fallito"
  orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" fetch "$F1T_ORIN_BUILD_REPO" "$XFER/f1t-$SHORT.bundle" "$SHORT" \
    | while read -r l; do log_info "Orin: $l"; done
fi
if [ "$STOP" = 1 ] && { [ "${PRE[stack_procs]:-0}" != 0 ] || [ -n "${PRE[car_container]:-}" ]; }; then
  log_warn "fermo il container dell'auto ($F1T_ORIN_UNIT): torna su con f1t deploy o stack_mode container"
  orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" stop-stack "$F1T_ORIN_UNIT" | while read -r l; do log_info "Orin: $l"; done
fi
orin_scp "$F1T_CORE/build/remote/orin_build_job.sh" "$XFER/f1t_build_job.sh"
orin_script "$F1T_CORE/build/remote/orin_build_ctl.sh" start "$ROOT" "$XFER/f1t_build_job.sh" \
  "$SHA" "$RUST" "$TEST" "$F1T_ORIN_BUILD_REPO" "$ROOT" "$F1T_BUILD_KEEP_WORKTREES" "$F1T_TEST_DOMAIN_ID" "$F1T_MODULES_EXCLUDE_RE" \
  | while read -r l; do log_info "Orin: $l"; done
check_record OK "build avviata" "$IMAGE"

if [ "$DETACH" = 1 ]; then
  log_info "staccata: f1t build --follow $SHORT"
  summary "f1t build"; exit $?
fi
log_step "Attesa della build"
if follow "$SHORT"; then
  check_record OK "immagine testata" "$IMAGE (f1tenth.tested=true)"
  mkdir -p "$LOGDIR"
  printf '{"sha": "%s", "image": "%s", "tests": "%s", "built_at": "%s"}\n' "$SHA" "$IMAGE" "$TEST" "$(date -Iseconds)" > "$LOGDIR/$SHORT.json"
  log_info "prossimo passo: f1t deploy $IMAGE"
else
  check_record FAIL "immagine testata" "vedi il log in $LOGDIR/"
fi
summary "f1t build"
