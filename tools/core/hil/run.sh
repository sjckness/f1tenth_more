#!/usr/bin/env bash
# desc: avvia un run HIL (sim -> bridge -> stack sull'Orin) con attesa attiva su ogni passo
# alias: hil up
#
#   f1t hil run <scenario> -disp <linus|work> [--image IMMAGINE] [--allow-mismatch]
#
# Rifiuta di partire se l'ultimo precheck (logs/tools/hil_precheck_last.json) non e' OK, e' di un
# altro host/scenario o ha piu' di F1T_PRECHECK_MAX_AGE_S secondi: in quel caso lo rilancia da solo.
# Ordine, ognuno con attesa dei topic attesi entro il suo timeout (scenario):
#   1 rete/DS  container HIL sull'Orin (stessa immagine dell'auto, mai f1tenth_car), profilo DDS,
#              Discovery Server su HOST_ORIN_LINK_IP:HIL_DS_PORT
#   2 sim      SIM_CMD in HOST_HIL_WORKTREE (Isaac in Docker), attende SIM_TOPICS
#   3 bridge   BRIDGE_CMD (se c'e'), poi i BRIDGE_TOPICS devono arrivare DENTRO il container dell'Orin
#   4 stack    supervisor_bringup nel container HIL, attende STACK_TOPICS
#   5 guida    DRIVE_S secondi a DRIVE_SPEED, spostamento su /odom >= DRIVE_MIN_DIST_M (se DRIVE_S > 0)
# Un passo che fallisce: stop pulito di tutto (f1t hil down), log del passo, exit 1. Tutto finisce in
# logs/hil/<data>_<disp>_<scenario>/ (config, precheck, versioni, log di ogni componente, result.json).
# AUTO_DOWN=1 (smoke) chiude da solo; altrimenti resta su fino a: f1t hil down -disp <host>.
set -Eeuo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
HIL_USAGE="f1t hil run <scenario> -disp <linus|work> [--image IMMAGINE] [--allow-mismatch]"
# shellcheck source=tools/core/hil/lib/hil.sh
source "$F1T_CORE/hil/lib/hil.sh"

hil_parse_args "$@"
IMAGE_ARG='' ALLOW=()
set -- "${HIL_REST[@]}"
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE_ARG=${2:?--image IMMAGINE}; shift ;;
    --allow-mismatch) ALLOW=(--allow-mismatch) ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_fail "opzione sconosciuta: $1"; echo "uso: $HIL_USAGE" >&2; exit 2 ;;
    *) [ -z "$SCENARIO" ] || { log_fail "uno scenario solo"; exit 2; }; SCENARIO=$1 ;;
  esac
  shift
done
[ -n "$SCENARIO" ] || { log_fail "manca lo scenario"; echo "uso: $HIL_USAGE" >&2; echo "scenari: $(cfg_names "$F1T_CORE/hil/scenarios" | tr '\n' ' ')" >&2; exit 2; }
hil_load
[ -z "$IMAGE_ARG" ] || STACK_IMAGE=$IMAGE_ARG
export F1T_ORIN_HOST  # il precheck rilanciato vede gli stessi override
cfg_print F1T_DISP F1T_SCENARIO SCENARIO_DESC HOST_HIL_WORKTREE SIM_CMD STACK_CONTAINER STACK_IMAGE STACK_LAUNCH_ARGS HIL_DOMAIN_ID HIL_DS
STATE=$HIL_LOGROOT/current.env
if [ -f "$STATE" ]; then
  log_die "c'e' gia' un run HIL aperto ($(sed -n 's/^RUN=//p' "$STATE")): f1t hil down -disp $DISP" 1
fi
log_steps 7

log_step "Precheck (max ${F1T_PRECHECK_MAX_AGE_S} s)"
LAST=$F1T_ROOT/logs/tools/hil_precheck_last.json
fresh=$(python3 - "$LAST" "$DISP" "$F1T_SCENARIO" "$F1T_PRECHECK_MAX_AGE_S" <<'EOF' 2>/dev/null || echo "illeggibile"
import json, sys, time
d = json.load(open(sys.argv[1]))
age = time.time() - d.get('time', 0)
if d.get('status') != 'OK': print(f"ultimo precheck {d.get('status')}")
elif d.get('disp') != sys.argv[2] or d.get('scenario') != sys.argv[3]: print(f"ultimo precheck per {d.get('disp')}/{d.get('scenario')}")
elif age > float(sys.argv[4]): print(f"ultimo precheck di {age:.0f} s fa")
else: print('ok')
EOF
)
if [ "$fresh" != ok ]; then
  log_info "$fresh: rilancio il precheck"
  pre=(-disp "$DISP" --scenario "$F1T_SCENARIO" "${ALLOW[@]}")
  if ! F1T_SCRIPT=hil_precheck "$F1T_CORE/hil/precheck.sh" "${pre[@]}"; then
    log_die "precheck FAIL: il run non parte (logs/tools/hil_precheck_last.json)" 1
  fi
fi
check_record OK "precheck" "OK ($(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["time_iso"])' "$LAST"))"

RUN=$HIL_LOGROOT/$(date +%Y%m%d_%H%M%S)_${DISP}_${F1T_SCENARIO}
mkdir -p "$RUN"
cp "${F1T_CFG_FILES[@]}" "$LAST" "$RUN/" 2>/dev/null || true
cp "$F1T_LOG_FILE" "$RUN/run_start.log" 2>/dev/null || true
{
  echo "RUN=$RUN"; echo "DISP=$DISP"; echo "SCENARIO=$F1T_SCENARIO"
  echo "STACK_CONTAINER=$STACK_CONTAINER"; echo "SIM_CONTAINER=$SIM_CONTAINER"
} > "$STATE"
STEP=init
fail_step() {
  local rc=$?
  trap - ERR INT TERM
  check_record FAIL "passo $STEP" "${1:-exit $rc}"
  log_fail "passo '$STEP' fallito: chiudo tutto"
  write_result FAIL
  F1T_SCRIPT=hil_down "$F1T_CORE/hil/down.sh" -disp "$DISP" || true
  summary "hil run $F1T_SCENARIO"
  exit 1
}
write_result() {
  python3 - "$RUN/result.json" "$1" "$F1T_SCENARIO" "$DISP" "${IMAGE:-}" "${IMAGE_ID:-}" "$STEP" <<'EOF'
import json, sys, time
f, status, scen, disp, img, iid, step = sys.argv[1:8]
json.dump({'time': int(time.time()), 'time_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'status': status,
           'scenario': scen, 'disp': disp, 'image': img, 'image_id': iid, 'last_step': step},
          open(f, 'w'), indent=2)
EOF
}
trap 'fail_step "interrotto"' INT TERM
trap 'fail_step' ERR

HIL_SIM_PROFILE=$RUN/dds_sim_side.xml
hil_profile "$HIL_SIM_PROFILE" 0
hil_profile "$RUN/dds_stack_side.xml" 1
OHIL=$F1T_CORE/hil/remote/orin_hil.sh

log_step "Rete e Discovery Server sull'Orin"
STEP=rete
declare -A IM=()
while IFS='=' read -r k v; do [ -n "$k" ] && IM[$k]=$v; done < <(orin_script "$OHIL" image "$F1T_ORIN_PIN_FILE" "${STACK_IMAGE:-}")
IMAGE=${IM[image]:-} IMAGE_ID=${IM[id]:-}
[ -n "$IMAGE_ID" ] || fail_step "immagine dello stack assente"
orin_script "$OHIL" facts "$STACK_CONTAINER" "$HIL_DS_PORT" "$HOST_LINK_IP" > "$RUN/orin_facts.txt" 2>&1 || true
orin_script "$F1T_CORE/orin/remote/modules.sh" --image "$IMAGE" "$F1T_MODULES_EXCLUDE_RE" > "$RUN/versions_image.txt" 2>&1 || true
git -C "$F1T_ROOT" describe --always --dirty > "$RUN/versions_linus.txt" 2>&1 || true
git -C "$HOST_HIL_WORKTREE" describe --always --dirty >> "$RUN/versions_linus.txt" 2>&1 || true
orin_script "$OHIL" up "$STACK_CONTAINER" "$IMAGE" | while read -r l; do log_info "Orin: $l"; done
orin_script "$OHIL" put "$STACK_CONTAINER" /tmp/f1tenth_hil_profile.xml < "$RUN/dds_stack_side.xml" >/dev/null
orin_script "$OHIL" put "$STACK_CONTAINER" /tmp/ros_probe.py < "$F1T_CORE/hil/lib/ros_probe.py" >/dev/null
orin_script "$OHIL" ds "$STACK_CONTAINER" "$HOST_ORIN_LINK_IP" "$HIL_DS_PORT" | while read -r l; do log_info "Orin: $l"; done
check_record OK "rete/DS" "$STACK_CONTAINER da $IMAGE, DS $HIL_DS"

log_step "Sim: $SIM_CMD"
STEP=sim
( cd "$HOST_HIL_WORKTREE" && exec setsid bash -c "exec $SIM_CMD" ) > "$RUN/sim.log" 2>&1 < /dev/null &
SIM_PID=$!
echo "SIM_PID=$SIM_PID" >> "$STATE"
log_info "sim pid $SIM_PID (gruppo di processi), log $RUN/sim.log; attendo $SIM_TOPICS (max ${SIM_TIMEOUT_S} s)"
# shellcheck disable=SC2086  # SIM_TOPICS e' una lista di parole
if out=$(hil_probe wait --timeout "$SIM_TIMEOUT_S" $SIM_TOPICS 2>&1); then
  check_record OK "sim: topic" "$(printf '%s\n' "$out" | tail -1)"
else
  printf '%s\n' "$out" >&2
  kill -0 "$SIM_PID" 2>/dev/null || log_fail "la sim e' terminata: $(tail -n 3 "$RUN/sim.log" | tr '\n' ' ')"
  fail_step "topic della sim mancanti: $(printf '%s\n' "$out" | grep FAIL | head -2 | tr '\n' ' ')"
fi

log_step "Bridge: topic della sim visti dall'Orin"
STEP=bridge
if [ -n "${BRIDGE_CMD:-}" ]; then
  ( cd "$HOST_HIL_WORKTREE" && exec setsid bash -c "exec $BRIDGE_CMD" ) > "$RUN/bridge.log" 2>&1 < /dev/null &
  echo "BRIDGE_PID=$!" >> "$STATE"
fi
# shellcheck disable=SC2086
if out=$(orin_script "$OHIL" exec "$STACK_CONTAINER" "$HIL_DOMAIN_ID" "$HOST_ORIN_LINK_IP" "$HIL_DS_PORT" \
           python3 /tmp/ros_probe.py wait --timeout 60 $BRIDGE_TOPICS 2>&1); then
  check_record OK "bridge: topic sull'Orin" "$(printf '%s\n' "$out" | tail -1)"
else
  printf '%s\n' "$out" >&2
  fail_step "l'Orin non vede i topic della sim: $(printf '%s\n' "$out" | grep FAIL | head -2 | tr '\n' ' ')"
fi

log_step "Stack sull'Orin"
STEP=stack
# shellcheck disable=SC2086  # STACK_LAUNCH_ARGS e' una lista di argomenti
orin_script "$OHIL" stack "$STACK_CONTAINER" "$HIL_DOMAIN_ID" "$HOST_ORIN_LINK_IP" "$HIL_DS_PORT" $STACK_LAUNCH_ARGS \
  | while read -r l; do log_info "Orin: $l"; done
# shellcheck disable=SC2086
if out=$(hil_probe wait --timeout "$STACK_TIMEOUT_S" $STACK_TOPICS 2>&1); then
  check_record OK "stack: topic" "$(printf '%s\n' "$out" | tail -1)"
else
  printf '%s\n' "$out" >&2
  orin_script "$OHIL" logs "$STACK_CONTAINER" > "$RUN/stack.log" 2>&1 || true
  fail_step "topic dello stack mancanti (log: $RUN/stack.log)"
fi

log_step "Guida"
STEP=guida
if [ "${DRIVE_S:-0}" != 0 ]; then
  if out=$(hil_probe drive --seconds "$DRIVE_S" --speed "${DRIVE_SPEED:-0.3}" --topic "${DRIVE_TOPIC:-/ackermann_drive}" \
             --min-dist "${DRIVE_MIN_DIST_M:-1.0}" 2>&1); then
    check_record OK "guida in rettilineo" "$(printf '%s\n' "$out" | tail -1)"
  else
    fail_step "guida: $(printf '%s\n' "$out" | tail -1)"
  fi
else
  check_skip "guida in rettilineo" "DRIVE_S=0"
fi

log_step "Risultato"
STEP=fine
trap - ERR INT TERM
write_result OK
orin_script "$OHIL" logs "$STACK_CONTAINER" > "$RUN/stack.log" 2>&1 || true
summary_json "$RUN/checks.json" scenario="$F1T_SCENARIO" disp="$DISP" image="$IMAGE" image_id="$IMAGE_ID"
if [ "${AUTO_DOWN:-0}" = 1 ]; then
  F1T_SCRIPT=hil_down "$F1T_CORE/hil/down.sh" -disp "$DISP" || log_warn "down non pulito: f1t hil down -disp $DISP"
else
  log_info "HIL su. Per chiudere: f1t hil down -disp $DISP"
fi
cp "$F1T_LOG_FILE" "$RUN/run.log" 2>/dev/null || true
summary "hil run $F1T_SCENARIO -disp $DISP"
