#!/usr/bin/env bash
# desc: chiude il run HIL (stack sull'Orin, sim, bridge) e conferma che non resta niente
#
#   f1t hil down -disp <linus|work> [--scenario NOME]
#
# Legge logs/hil/current.env (scritto da f1t hil run); senza, usa i nomi dello scenario (default
# smoke). Ordine: log dello stack nella cartella del run, SIGINT allo stack e rimozione del container
# HIL sull'Orin, SIGINT/SIGTERM al gruppo di processi della sim e del bridge, docker stop del
# container di Isaac. Poi verifica: nessun container (HIL, sim), nessun processo del gruppo, porta
# del DS libera sull'Orin. Exit 1 se resta qualcosa (elencato).
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
HIL_USAGE="f1t hil down -disp <linus|work> [--scenario NOME]"
# shellcheck source=tools/core/hil/lib/hil.sh
source "$F1T_CORE/hil/lib/hil.sh"
hil_parse_args "$@"
for a in "${HIL_REST[@]}"; do
  case "$a" in
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_fail "argomento sconosciuto: $a"; echo "uso: $HIL_USAGE" >&2; exit 2 ;;
  esac
done
STATE_FILE=$F1T_ROOT/${F1T_HIL_LOG_DIR:-logs/hil}/current.env
RUN='' SIM_PID='' BRIDGE_PID=''
if [ -f "$STATE_FILE" ]; then
  RUN=$(sed -n 's/^RUN=//p' "$STATE_FILE"); SIM_PID=$(sed -n 's/^SIM_PID=//p' "$STATE_FILE")
  BRIDGE_PID=$(sed -n 's/^BRIDGE_PID=//p' "$STATE_FILE")
  [ -n "$SCENARIO" ] || SCENARIO=$(sed -n 's/^SCENARIO=//p' "$STATE_FILE")
fi
hil_load
cfg_print F1T_DISP F1T_SCENARIO STACK_CONTAINER SIM_CONTAINER RUN SIM_PID
log_steps 3

log_step "Stack sull'Orin ($STACK_CONTAINER)"
if orin_up; then
  [ -z "$RUN" ] || orin_script "$F1T_CORE/hil/remote/orin_hil.sh" logs "$STACK_CONTAINER" > "$RUN/stack.log" 2>&1 || true
  check "container HIL rimosso" orin_script "$F1T_CORE/hil/remote/orin_hil.sh" down "$STACK_CONTAINER"
else
  check_record FAIL "container HIL rimosso" "Orin non raggiungibile: $(orin_target)"
fi

log_step "Sim e bridge (host $DISP)"
stop_group() {
  local pid=$1 name=$2 i
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || { echo "$name: non in esecuzione"; return 0; }
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for i in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  if kill -0 "$pid" 2>/dev/null; then kill -TERM -- "-$pid" 2>/dev/null || true; sleep 5; fi
  kill -0 "$pid" 2>/dev/null && { echo "$name: il gruppo $pid non si ferma"; return 1; }
  echo "$name: fermato (gruppo $pid, ${i}s)"
}
check "processo della sim fermo" stop_group "$SIM_PID" sim
[ -z "$BRIDGE_PID" ] || check "processo del bridge fermo" stop_group "$BRIDGE_PID" bridge
stop_ctr() {
  docker ps -a --format '{{.Names}}' | grep -qx "$SIM_CONTAINER" || { echo "$SIM_CONTAINER assente"; return 0; }
  docker stop -t 40 "$SIM_CONTAINER" >/dev/null 2>&1 || true
  docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
  docker ps -a --format '{{.Names}}' | grep -qx "$SIM_CONTAINER" && { echo "$SIM_CONTAINER c'e' ancora"; return 1; }
  echo "$SIM_CONTAINER fermato e rimosso"
}
check "container della sim fermo" stop_ctr

log_step "Verifica: non resta niente"
left_local() {
  local n=0
  [ -z "$SIM_PID" ] || n=$(pgrep -g "$SIM_PID" 2>/dev/null | wc -l)
  [ "$n" = 0 ] || { echo "$n processi nel gruppo $SIM_PID"; return 1; }
  echo "nessun processo, nessun container $SIM_CONTAINER"
}
check "host della sim pulito" left_local
if orin_up; then
  declare -A OH=()
  while IFS='=' read -r k v; do [ -n "$k" ] && OH[$k]=$v; done < <(orin_script "$F1T_CORE/hil/remote/orin_hil.sh" facts "$STACK_CONTAINER" "$HIL_DS_PORT" "$HOST_LINK_IP" 2>/dev/null || true)
  if [ -z "${OH[hil_ctr]:-}" ] && [ "${OH[ds_port_busy]:-0}" = 0 ]; then check_record OK "Orin pulito" "nessun $STACK_CONTAINER, porta $HIL_DS_PORT libera"
  else check_record FAIL "Orin pulito" "container ${OH[hil_ctr]:-assente}, porta $HIL_DS_PORT occupata: ${OH[ds_port_busy]:-?}"; fi
fi
if summary "hil down -disp $DISP"; then
  rm -f "$STATE_FILE"
  [ -z "$RUN" ] || cp "$F1T_LOG_FILE" "$RUN/down.log" 2>/dev/null || true
  exit 0
fi
log_warn "resta qualcosa (sopra): $STATE_FILE non rimosso"
exit 1
