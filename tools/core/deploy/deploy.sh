#!/usr/bin/env bash
# desc: deploy sull'Orin di un'immagine testata: pin, restart, health check, rollback automatico
# alias: orin deploy
#
#   f1t deploy [--dry-run] [--dev] [--after-hil] [--force-untested] [--force-busy] [--switch-mode]
#              [IMAGE|COMMIT]                       default: l'immagine di HEAD di questo checkout
#   f1t deploy --rollback [--dry-run]               torna all'immagine precedente (deployed.json)
#
# Cancelli, in ordine (il primo che fallisce ferma tutto, l'auto resta com'e'):
#   working tree committato (--dev lo salta) -> immagine presente sull'Orin, costruita da f1t build
#   (label f1tenth.tested=true; --force-untested lo salta, e resta scritto in deployed.json) ->
#   --after-hil: l'ultimo `f1t hil run smoke` su quell'immagine e' OK -> nessuno stack in esecuzione
#   nel container (--force-busy) -> Orin in modalita' container (--switch-mode esegue stack_mode container).
# Poi: diff dei parametri (f1t diff-params), docker_orin pin + restart della unit, health check entro
# F1T_HEALTH_TIMEOUT_S (container healthy con l'immagine giusta). Se fallisce: rollback automatico
# all'immagine di prima, stesso health check, exit 1. Se riesce: deployed.json sull'Orin
# (sha, immagine, versioni dei moduli lette dal container, data, chi, immagine precedente).
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

DRY=0 DEV=0 AFTER_HIL=0 FORCE_UNTESTED=0 FORCE_BUSY=0 SWITCH=0 ROLLBACK=0 TARGET=HEAD
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --dev) DEV=1 ;;
    --after-hil) AFTER_HIL=1 ;;
    --force-untested) FORCE_UNTESTED=1 ;;
    --force-busy) FORCE_BUSY=1 ;;
    --switch-mode) SWITCH=1 ;;
    --rollback) ROLLBACK=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $1 (f1t deploy -h)" 2 ;;
    *) TARGET=$1 ;;
  esac
  shift
done
cfg_load
cfg_require config || exit 1
DCTL=$F1T_CORE/deploy/remote/orin_deploy.sh

jget() { python3 -c 'import json,sys; d=json.loads(sys.argv[1] or "{}"); print(d.get(sys.argv[2], "") or "")' "$1" "$2" 2>/dev/null || true; }

# shellcheck disable=SC2034  # stampata da cfg_print
F1T_DEPLOY_TARGET=$( [ "$ROLLBACK" = 1 ] && echo "rollback (immagine precedente)" || echo "$TARGET")
cfg_print F1T_ORIN_HOST F1T_DEPLOY_TARGET F1T_IMAGE_RUST F1T_HEALTH_TIMEOUT_S F1T_DEPLOY_REQUIRE_TESTED F1T_ORIN_DEPLOYED_JSON
[ "$DRY" = 0 ] || log_warn "DRY-RUN: si controlla tutto, non si cambia niente"
log_steps 6
log_step "Cosa gira sull'Orin"
orin_facts || { check_record FAIL "ssh $(orin_target)" "${ORIN[error]:-non risponde}"; summary "f1t deploy"; exit 1; }
check_record OK "ssh $(orin_target)" "${ORIN[hostname]}"
CUR_IMAGE=${ORIN[ctr_image]:-${ORIN[pin_tag]:-}} CUR_ID=${ORIN[ctr_image_id]:-${ORIN[pin_id]:-}}
CUR_REV=${ORIN[ctr_revision]:-}
check_record OK "immagine attuale" "${CUR_IMAGE:-nessuna} (${CUR_ID:7:12})"

# ---- immagine da deployare
SHA=
if [ "$ROLLBACK" = 1 ]; then
  IMAGE=$(jget "${ORIN[deployed_json]:-}" previous_image)
  [ -n "$IMAGE" ] || log_die "rollback: deployed.json non dice l'immagine precedente (usa f1t deploy <immagine>; docker_orin history sull'Orin la mostra)"
  DEV=1 FORCE_UNTESTED=1
elif [[ "$TARGET" == *:* ]]; then
  IMAGE=$TARGET
else
  SHA=$(git -C "$F1T_ROOT" rev-parse --verify "${TARGET}^{commit}" 2>/dev/null) || log_die "ne' immagine ne' commit: $TARGET" 2
  SUFFIX=; [ "$F1T_IMAGE_RUST" = skip ] && SUFFIX=-norust
  IMAGE=$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX${SHA:0:7}$SUFFIX
fi
log_info "immagine da deployare: $IMAGE (ora gira: ${CUR_IMAGE:-nessuna})"

log_step "Cancelli"
if [ "$DEV" = 1 ]; then check_skip "working tree committato" "--dev/--rollback"
else check "working tree committato" probe_git_clean "$F1T_ROOT"; fi
declare -A IMG=()
while IFS='=' read -r k v; do [ -n "$k" ] && IMG[$k]=$v; done < <(orin_script "$DCTL" image-info "$IMAGE" 2>/dev/null || true)
if [ "${IMG[exists]:-no}" != yes ]; then
  check_record FAIL "immagine sull'Orin" "$IMAGE assente: prima f1t build"
else
  check_record OK "immagine sull'Orin" "${IMG[id]:7:12} ${IMG[revision]:0:8} ${IMG[branch]}"
  if [ -n "$SHA" ] && [ "${IMG[revision]}" != "$SHA" ]; then
    check_record FAIL "revisione = commit" "label ${IMG[revision]:0:12}, commit ${SHA:0:12}"
  fi
  if [ "${IMG[tested]:-}" = true ]; then check_record OK "testata (f1tenth.tested)" "${IMG[tested_at]:0:16} ${IMG[tests]:0:40}"
  elif [ "$FORCE_UNTESTED" = 1 ] || [ "$F1T_DEPLOY_REQUIRE_TESTED" = 0 ]; then check_record WARN "testata (f1tenth.tested)" "no: forzato"
  else check_record FAIL "testata (f1tenth.tested)" "no: f1t build la testa (o --force-untested)"; fi
  [ "${IMG[id]:-}" != "$CUR_ID" ] || check_record WARN "diversa da quella attuale" "e' gia' quella in esecuzione: solo restart"
fi
if [ "$AFTER_HIL" = 1 ]; then
  hil=$(python3 - "$F1T_ROOT/$F1T_HIL_LOG_DIR" "${IMG[id]:-none}" <<'EOF'
import glob, json, os, sys
best = None
for f in glob.glob(os.path.join(sys.argv[1], '*', 'result.json')):
    try:
        d = json.load(open(f))
    except (OSError, ValueError):
        continue
    if d.get('scenario') == 'smoke' and d.get('image_id') == sys.argv[2]:
        if best is None or d.get('time', 0) > best[0].get('time', 0):
            best = (d, f)
print(f"{best[0].get('status')} {best[1]}" if best else 'NONE')
EOF
)
  case "$hil" in
    OK\ *) check_record OK "HIL smoke su questa immagine" "${hil#OK }" ;;
    NONE) check_record FAIL "HIL smoke su questa immagine" "nessun run: f1t hil run smoke -disp <host>" ;;
    *) check_record FAIL "HIL smoke su questa immagine" "$hil" ;;
  esac
fi
busy=${ORIN[ctr_stack_procs]:-0}
if [ "$busy" -gt 0 ] 2>/dev/null; then
  if [ "$FORCE_BUSY" = 1 ]; then check_record WARN "stack fermo" "$busy processi dello stack: forzato"
  else check_record FAIL "stack fermo" "$busy processi dello stack nel container (missione?): fermalo o --force-busy"; fi
else
  check_record OK "stack fermo" "nessun processo dello stack"
fi
MODE_OK=1
if [ "${ORIN[unit_container]:-}" != active ]; then
  MODE_OK=0
  if [ "$SWITCH" = 1 ]; then check_record WARN "modalita' container" "no: --switch-mode esegue stack_mode container"
  else check_record FAIL "modalita' container" "no (unit ${ORIN[unit_container]:-?}, gateway nativo ${ORIN[unit_gateway]:-?}): --switch-mode"; fi
else
  check_record OK "modalita' container" "$F1T_ORIN_UNIT attiva"
fi
if ! checks_ok; then summary "f1t deploy"; log_fail "deploy rifiutato: l'Orin non e' stato toccato"; exit 1; fi

log_step "Parametri: deployato -> nuovo"
TO_REV=${IMG[revision]:-}
if [ -n "$CUR_REV" ] && [ -n "$TO_REV" ] && git -C "$F1T_ROOT" cat-file -e "${CUR_REV}^{commit}" 2>/dev/null \
   && git -C "$F1T_ROOT" cat-file -e "${TO_REV}^{commit}" 2>/dev/null; then
  "$F1T_CORE/deploy/diff_params.sh" --from "$CUR_REV" --to "$TO_REV" --quiet --max-lines 120 || true
else
  log_warn "diff dei parametri non disponibile (commit ${CUR_REV:0:8} -> ${TO_REV:0:8} non entrambi su linus: f1t orin status --fetch)"
fi

if [ "$DRY" = 1 ]; then
  log_step "Deploy (dry-run)"
  log_info "verrebbe eseguito sull'Orin: docker_orin pin $IMAGE; systemctl --user restart $F1T_ORIN_UNIT; health check ${F1T_HEALTH_TIMEOUT_S}s"
  [ "$MODE_OK" = 1 ] || log_info "  prima: stack_mode container"
  check_skip "deploy" "dry-run"
  summary "f1t deploy (dry-run)"; exit $?
fi

switch_to() {
  local img=$1 mode=
  [ "$MODE_OK" = 1 ] || mode=$F1T_ORIN_STACK_MODE
  orin_script "$DCTL" switch "$img" "$F1T_ORIN_DOCKER_ORIN" "$F1T_ORIN_UNIT" ${mode:+"$mode"} 2>&1 | while read -r l; do log_info "Orin: $l"; done
  MODE_OK=1
}
health() {
  orin_script "$DCTL" health "$F1T_ORIN_CONTAINER" "$F1T_ORIN_DOCKER_ORIN" "$F1T_HEALTH_TIMEOUT_S" "$1"
}

log_step "Deploy: pin + restart"
rc=0; switch_to "$IMAGE" || rc=$?
[ "$rc" = 0 ] || log_warn "switch terminato con exit $rc: controllo comunque lo stato"

log_step "Health check (${F1T_HEALTH_TIMEOUT_S} s)"
if out=$(health "${IMG[id]}" 2>&1); then
  check_record OK "health check" "$(printf '%s\n' "$out" | tail -1)"
else
  check_record FAIL "health check" "$(printf '%s\n' "$out" | head -1)"
  printf '%s\n' "$out" >&2
  if [ -n "$CUR_IMAGE" ] && [ "$ROLLBACK" = 0 ]; then
    log_warn "rollback automatico a $CUR_IMAGE"
    rc=0; switch_to "$CUR_IMAGE" || rc=$?
    if out=$(health "$CUR_ID" 2>&1); then check_record WARN "rollback" "tornato a $CUR_IMAGE: $(printf '%s\n' "$out" | tail -1)"
    else check_record FAIL "rollback" "anche $CUR_IMAGE non e' sano: f1t orin status"; fi
  fi
  summary "f1t deploy"; exit 1
fi

# ---- deployed.json
tmp=$(mktemp); trap 'rm -f "$tmp"' EXIT
orin_script "$F1T_CORE/orin/remote/modules.sh" --running "$F1T_ORIN_CONTAINER" "$F1T_MODULES_EXCLUDE_RE" > "$tmp" 2>/dev/null || true
json=$(python3 - "$tmp" image="$IMAGE" image_id="${IMG[id]}" sha="${IMG[revision]}" branch="${IMG[branch]}" \
  tested="${IMG[tested]:-}" forced_untested="$FORCE_UNTESTED" deployed_at="$(date -Iseconds)" \
  deployed_by="$(id -un)@$(hostname)" \
  linus_checkout="$(git -C "$F1T_ROOT" describe --always --dirty 2>/dev/null) $(git -C "$F1T_ROOT" rev-parse --abbrev-ref HEAD)" \
  previous_image="$CUR_IMAGE" previous_image_id="$CUR_ID" rollback="$ROLLBACK" <<'EOF'
import json, sys
mods = {}
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) >= 2:
        mods[p[0]] = p[1]
d = dict(a.split('=', 1) for a in sys.argv[2:])
d['tested'] = d['tested'] == 'true'
d['forced_untested'] = d['forced_untested'] == '1'
d['rollback'] = d['rollback'] == '1'
d['modules'] = mods
print(json.dumps(d, indent=2, sort_keys=True))
EOF
)
if where=$(printf '%s\n' "$json" | orin_script "$DCTL" write-json "$F1T_ORIN_DEPLOYED_JSON" "$F1T_ORIN_DEPLOYED_JSON_FALLBACK" 2>&1); then
  if [ "$where" = "$F1T_ORIN_DEPLOYED_JSON" ]; then check_record OK "deployed.json" "$where"
  else check_record WARN "deployed.json" "$where ($F1T_ORIN_DEPLOYED_JSON non scrivibile: tools/SUDO_TODO.md)"; fi
else
  check_record WARN "deployed.json" "non scritto: $where"
fi
check_record OK "deployato" "$IMAGE ($(wc -l < "$tmp") moduli)"
summary "f1t deploy"
