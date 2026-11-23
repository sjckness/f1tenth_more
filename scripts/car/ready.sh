#!/usr/bin/env bash
# scripts/car/ready.sh - da linus: l'Orin e' pronta a lanciare lo stack? READY / NOT READY.
#
#   scripts/car/ready.sh [--profile NOME] [--json FILE] [COMMIT]          default COMMIT = HEAD
#
# Solo letture (due chiamate ssh), non avvia e non ferma niente. READY se e solo se nessun
# controllo e' FAIL:
#   container  modalita' container; container su e sano (discovery server + gateway); container =
#              immagine fissata; immagine con lo stesso contenuto di COMMIT (WARN: label tested assente)
#   script     car-stack, car-container, car-preflight sull'Orin == quelli di COMMIT
#   libero     nessuno stack nel container, nessun processo ROS sull'host fuori dal container
#   sensori    car-preflight: ZED su USB3, il lidar risponde, VESC enumerato
#   profilo    stack_args applicabile (WARN se ha token che vincono sul profilo); con --profile: il
#              profilo e' sull'Orin, uguale a quello di linus, e valido nel container (ogni launch
#              argument, componente e argomento di componente che nomina esiste)
#   risorse    /scratch >= 20 GB, / >= 5 GB (FAIL); RAM >= 4000 MB (WARN); temperatura < 70 C OK,
#              < 85 C WARN, oltre FAIL
#   avvisi     orologio sincronizzato, file del container cambiati a mano
# Quello che si misura solo con lo stack acceso (frequenza di odometria e scan, TF, e-stop, batteria)
# lo controlla car-stack start, che aspetta il READY di f1tenth_ctl.
#
# --json FILE: i controlli anche in JSON (default logs/tools/car_ready_last.json).
# Exit: 0 READY, 1 NOT READY, 2 argomenti.
set -euo pipefail
export GIT_PAGER=cat PAGER=cat
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=car_ready
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"
# shellcheck source=scripts/car/lib/orin_common.sh
source "$ROOT/scripts/car/lib/orin_common.sh"
# shellcheck source=scripts/car/lib/car_ready.sh
source "$ROOT/scripts/car/lib/car_ready.sh"

PROFILE=- JSON=$ROOT/logs/tools/car_ready_last.json COMMIT=HEAD
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) [ $# -ge 2 ] || log_die "--profile vuole un nome" 2; PROFILE=$2; shift ;;
    --json) [ $# -ge 2 ] || log_die "--json vuole un file" 2; JSON=$2; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $1 (scripts/car/ready.sh -h)" 2 ;;
    *) COMMIT=$1 ;;
  esac
  shift
done
[ "$PROFILE" = - ] || [[ $PROFILE =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]*$ ]] \
  || log_die "--profile: un nome semplice, senza percorso e senza .yaml" 2
cfg_load
cfg_require config || exit 1
SHA=$(git -C "$ROOT" rev-parse -q --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
[ "$PROFILE" = - ] || [ -f "$ROOT/$CAR_PROFILES_SRC/$PROFILE.yaml" ] \
  || log_die "profilo $PROFILE assente in $CAR_PROFILES_SRC (ci sono: $(car_profiles_sums | awk '{printf "%s ", $1}'))" 2

log_info "car ready: Orin $(orin_target), commit ${SHA:0:12} ($(git -C "$ROOT" log -1 --format=%s "$SHA" | cut -c1-60)), profilo ${PROFILE/#-/nessuno}"
declare -A R=()
if ! orin_facts; then
  check_record FAIL "Orin raggiungibile" "ssh $(orin_target): ${ORIN[error]:-nessuna risposta}"
else
  check_record OK "Orin raggiungibile" "${ORIN[hostname]:-?}"
  # shellcheck disable=SC2088  # la ~ si espande sull'Orin (remote_ready.sh), apposta
  out=$(orin_script "$ROOT/scripts/car/lib/remote_ready.sh" "$F1T_ORIN_CONTAINER" "$PROFILE" \
          "~/$CAR_PROFILES_DIR" "$F1T_ORIN_BUILD_ROOT" 2>&1) || log_warn "remote_ready.sh: exit $? ($(tail -1 <<<"$out"))"
  while IFS='=' read -r k v; do [ -n "$k" ] && R[$k]=$v; done <<<"$out"
  ready_eval "$SHA" "$PROFILE"
fi
summary_json "$JSON" commit="$SHA" profile="$PROFILE" >/dev/null 2>&1 || true
if summary "car ready ${PROFILE/#-/}"; then
  log_ok "READY: sull'Orin car-stack start ${PROFILE/#-/} (attende il READY di f1tenth_ctl con lo stack acceso)"
  echo READY
else
  log_fail "NOT READY: le righe FAIL sopra dicono cosa sistemare"
  echo "NOT READY"
  exit 1
fi
