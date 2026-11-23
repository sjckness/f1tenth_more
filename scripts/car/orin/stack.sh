#!/usr/bin/env bash
# car-stack - lo stack ROS nel container dell'auto, SULL'ORIN, dalla shell dell'HOST (non nel container).
# Installato da scripts/car/deploy.sh in ~/.local/bin/car-stack (stesso commit dell'immagine).
#
#   car-stack start [PROFILO] [--wait S | --no-wait]   avvia lo stack (con il profilo) e aspetta READY
#   car-stack stop                                     ferma lo stack (il container resta su)
#   car-stack restart [PROFILO] [--wait S | --no-wait] stop + start
#   car-stack status                                   stato, profilo in uso, READY o cosa manca
#   car-stack profiles                                 i profili disponibili (con sha256)
#
# PROFILO e' il nome di un file di ~/.config/f1tenth_car/profiles senza .yaml (es. test_orin_rrt),
# copiato da linus con scripts/car/profiles.sh push. Formato: launch_arguments (argomenti di
# supervisor_bringup.launch.py) e component_args (argomenti dei launch file dei componenti); regole in
# src/f1tenth_bringup/f1tenth_bringup/launch_profile.py. Senza PROFILO: i default di stack_params.yaml.
#
# start: rifiuta se lo stack gira gia' (usa restart); verifica il profilo DENTRO il container
# (launch_profile check: ogni nome che usa deve esistere); scrive in ~/.config/f1tenth_car/stack_args i
# token profile:=P profiles_dir:=<cartella> lasciando gli altri token com'erano (stack_args e' riletto a
# ogni avvio: niente riavvio del container); f1tenth_ctl start stack; poi aspetta il READY di
# f1tenth_ctl (odometria, scan, TF, servizio missioni, e-stop, batteria), max --wait secondi (240).
# stop: f1tenth_ctl stop stack (SIGINT al gruppo di processi, poi SIGTERM, poi SIGKILL). Non prende
# un profilo: ferma lo stack qualunque esso sia. stack_args resta com'e'.
#
# Nessun movimento: lo stack parte senza missione.
# Exit: 0 fatto e READY (status: READY), 1 fallito o non READY, 2 argomenti, 3 lanciato dentro il
# container o container fermo (car-container start), 4 status: stack fermo.
set -euo pipefail
CTR=${F1T_CAR_CONTAINER:-f1tenth_car}
ARGS_FILE=${CAR_STACK_ARGS_FILE:-$HOME/.config/f1tenth_car/stack_args}
PROFILES_DIR=${CAR_PROFILES_DIR:-$HOME/.config/f1tenth_car/profiles}
CTL=${CAR_CTL:-$HOME/.local/bin/f1tenth_ctl_car}
CONTAINER_MARKERS=${F1T_CONTAINER_MARKERS:-/.dockerenv /opt/f1tenth_ws/SOURCES}
# La stessa regola di src/f1tenth_ctl/config/ctl_container.env: una riga, solo token name:=value.
TOKEN_RE='^[A-Za-z_][A-Za-z0-9_]*:=[A-Za-z0-9_./,:+@%-]*$'
NAME_RE='^[A-Za-z0-9_][A-Za-z0-9_.-]*$'

usage() { sed -n '2,/^set -euo/p' "$(readlink -f "$0")" | sed '$d; s/^# \{0,1\}//'; }
log() { printf '%s %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
kv() { printf '  %-22s %s\n' "$1" "$2" >&2; }
die() { log "ERRORE: $*"; exit "${2:-1}"; }

# stack_args_line FILE PROFILO DIR: la riga nuova di stack_args su stdout. Toglie i token profile:= e
# profiles_dir:=, tiene gli altri, aggiunge quelli del profilo (se PROFILO non e' vuoto). Exit 1 se il
# file attuale non rispetta la regola (piu' righe, token non validi): non lo riscrive alla cieca.
stack_args_line() {
  local file=$1 profile=$2 dir=$3 lines line tok out=()
  if [ -r "$file" ]; then
    lines=$(grep -c '[^[:space:]]' "$file" || true)
    [ "${lines:-0}" -le 1 ] || { echo "$file ha $lines righe: ne e' ammessa una" >&2; return 1; }
    line=$(grep -m1 '[^[:space:]]' "$file" || true)
    for tok in $line; do
      [[ $tok =~ $TOKEN_RE ]] || { echo "$file: token non valido '$tok' (solo name:=value)" >&2; return 1; }
      case "$tok" in profile:=*|profiles_dir:=*) ;; *) out+=("$tok") ;; esac
    done
  fi
  if [ -n "$profile" ]; then out+=("profile:=$profile" "profiles_dir:=$dir"); fi
  echo "${out[*]}"
}

# write_stack_args LINE: scrittura atomica (il container legge il file in sola lettura).
write_stack_args() {
  mkdir -p "$(dirname "$ARGS_FILE")"
  if [ -z "$1" ]; then rm -f "$ARGS_FILE"; return; fi
  printf '%s\n' "$1" > "$ARGS_FILE.tmp.$$" && mv -f "$ARGS_FILE.tmp.$$" "$ARGS_FILE"
}

in_ctr() { docker exec -i "$CTR" /opt/f1tenth_ws/entrypoint.sh "$@" </dev/null; }
ctr_running() { [ "$(docker inspect -f '{{.State.Running}}' "$CTR" 2>/dev/null)" = true ]; }
ctl_json() { "$CTL" status --json 2>/dev/null </dev/null || true; }
# Da un JSON di f1tenth_ctl status: "STATO_STACK READY|NOT_READY failing..."
ctl_summary() {
  python3 -c 'import json, sys
try:
    d = json.loads(sys.stdin.read())
except Exception:
    print("? ?"); sys.exit()
st = ((d.get("processes") or {}).get("stack") or {}).get("state", "?")
print(st, "READY" if d.get("ready") else "NOT_READY", " ".join(d.get("failing") or []))'
}
stack_state() { ctl_json | ctl_summary | awk '{print $1}'; }
# Il profilo con cui gira lo stack: dalla riga del launch nel suo log (nome, percorso, sha256).
running_profile() {
  in_ctr bash -c 'grep -h "\[supervisor_bringup\] profile " ~/.local/state/f1tenth_ctl/logs/stack.log 2>/dev/null | tail -1 | sed "s/.*\[supervisor_bringup\] //" | cut -c1-200' 2>/dev/null || true
}

check_profile() {   # check_profile NOME: esiste sull'host e passa launch_profile check nel container
  local name=$1 out
  [[ $name =~ $NAME_RE ]] || die "profilo '$name': serve un nome semplice (senza percorso, senza .yaml)" 2
  [ -f "$PROFILES_DIR/$name.yaml" ] || die "profilo '$name' assente in $PROFILES_DIR (car-stack profiles; da linus: scripts/car/profiles.sh push)"
  out=$(in_ctr python3 -m f1tenth_bringup.launch_profile check "$name" --profiles-dir "$PROFILES_DIR" 2>&1 | grep -E '^(OK|ERROR) ' | tail -1 || true)
  case "$out" in
    "OK "*) kv "profilo" "$name sha256 $(awk '{print substr($3,1,12)}' <<<"$out") ($PROFILES_DIR/$name.yaml)" ;;
    "ERROR "*) die "profilo '$name' rifiutato: ${out#ERROR }" ;;
    *) die "profilo '$name': verifica nel container non riuscita (immagine senza f1tenth_bringup.launch_profile? serve f1tenth_bringup >= 0.3.0: scripts/car/deploy.sh)" ;;
  esac
}

wait_ready() {   # wait_ready S: 0 READY entro S secondi
  local wait=$1 t0 now st rd failing
  t0=$(date +%s)
  while :; do
    read -r st rd failing < <(ctl_json | ctl_summary)
    now=$(( $(date +%s) - t0 ))
    if [ "$rd" = READY ]; then log "READY dopo ${now}s"; return 0; fi
    printf '\r\033[K  %3ds/%ds  stack %s, manca: %s' "$now" "$wait" "$st" "${failing:-?}" >&2
    if [ "$now" -ge 10 ]; then
      case "$st" in crashed|stopped) printf '\n' >&2; log "lo stack e' $st ($CTL logs stack -n 80)"; return 1 ;; esac
    fi
    if [ "$now" -ge "$wait" ]; then printf '\n' >&2; log "non READY dopo ${wait}s, manca: ${failing:-?}"; return 1; fi
    sleep 3
  done
}

cmd_status() {
  local st rd failing args
  ctr_running || { log "container $CTR fermo: car-container start"; return 3; }
  read -r st rd failing < <(ctl_json | ctl_summary)
  args=$(cat "$ARGS_FILE" 2>/dev/null || echo '(assente)')
  kv "container" "$CTR $(docker inspect -f '{{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{end}} {{.Config.Image}}' "$CTR")"
  kv "stack" "$st"
  kv "stack_args" "$args"
  if [ "$st" = running ]; then
    kv "profilo in uso" "$(running_profile)"
    kv "READY" "$( [ "$rd" = READY ] && echo si || echo "no, manca: ${failing:-?}")"
    [ "$rd" = READY ] && return 0 || return 1
  fi
  return 4
}

cmd_profiles() {
  local f
  [ -d "$PROFILES_DIR" ] || die "$PROFILES_DIR assente (da linus: scripts/car/profiles.sh push)"
  for f in "$PROFILES_DIR"/*.yaml; do
    [ -e "$f" ] || continue
    printf '  %-28s %s\n' "$(basename "$f" .yaml)" "$(sha256sum "$f" | cut -c1-12)"
  done
}

cmd_start() {
  local profile=$1 wait=$2 st line out
  st=$(stack_state)
  case "$st" in running|stopping) die "lo stack e' gia' $st ($(running_profile)): car-stack restart ${profile}" ;; esac
  log "avvio dello stack${profile:+ con il profilo $profile}"
  [ -z "$profile" ] || check_profile "$profile"
  line=$(stack_args_line "$ARGS_FILE" "$profile" "$PROFILES_DIR") \
    || die "stack_args non modificabile (sopra): correggilo a mano o cancellalo ($ARGS_FILE)"
  kv "stack_args prima" "$(cat "$ARGS_FILE" 2>/dev/null || echo '(assente)')"
  write_stack_args "$line"
  kv "stack_args dopo" "${line:-(assente)}"
  log "\$ $CTL start stack"
  out=$("$CTL" start stack </dev/null 2>&1) || { printf '%s\n' "$out" >&2; die "f1tenth_ctl start stack fallito ($CTL logs stack -n 80)"; }
  [ "$wait" -gt 0 ] || { log "avviato (--no-wait): car-stack status"; return 0; }
  wait_ready "$wait" || { log "diagnosi: $CTL status --human; $CTL logs stack -n 120"; return 1; }
  kv "profilo in uso" "$(running_profile)"
  log "PRONTO. Fermare: car-stack stop"
}

cmd_stop() {
  local st out
  st=$(stack_state)
  if [ "$st" != running ] && [ "$st" != stopping ]; then log "lo stack e' gia' fermo ($st)"; return 0; fi
  log "\$ $CTL stop stack"
  out=$("$CTL" stop stack </dev/null 2>&1) || { printf '%s\n' "$out" >&2; die "f1tenth_ctl stop stack fallito ($CTL status --human)"; }
  st=$(stack_state)
  [ "$st" != running ] || die "lo stack gira ancora dopo lo stop ($CTL status --human)"
  log "stack fermo ($st). Il container resta su: car-container stop per fermarlo"
}

main() {
  local cmd=${1:-} profile='' wait=240
  [ $# -gt 0 ] && shift
  case "$cmd" in
    -h|--help|help) usage; exit 0 ;;
    '') usage >&2; exit 2 ;;
  esac
  while [ $# -gt 0 ]; do
    case "$1" in
      --wait) [ $# -ge 2 ] && [[ $2 =~ ^[0-9]+$ ]] || die "--wait vuole dei secondi" 2; wait=$2; shift ;;
      --no-wait) wait=0 ;;
      -*) die "opzione sconosciuta '$1' (car-stack -h)" 2 ;;
      *) [ -z "$profile" ] || die "un solo profilo (car-stack -h)" 2; profile=$1 ;;
    esac
    shift
  done
  case "$cmd" in
    start|restart) ;;
    stop|status|profiles) [ -z "$profile" ] || die "$cmd non prende un profilo (car-stack -h)" 2 ;;
    *) die "comando sconosciuto '$cmd' (car-stack -h)" 2 ;;
  esac
  # Il container usa la rete dell'host: il suo prompt e' identico (fabiocar@jetson-orin).
  local m
  for m in $CONTAINER_MARKERS; do
    [ ! -e "$m" ] || die "sei DENTRO il container ($m esiste): esci con 'exit' e rilancia dalla shell dell'Orin" 3
  done
  [ "$cmd" = profiles ] && { cmd_profiles; return; }
  [ -x "$CTL" ] || die "manca $CTL (docker/orin/host/install.sh)"
  ctr_running || { [ "$cmd" = status ] && { cmd_status; return; }; die "container $CTR fermo: car-container start" 3; }
  case "$cmd" in
    status) cmd_status ;;
    start) cmd_start "$profile" "$wait" ;;
    stop) cmd_stop ;;
    restart) cmd_stop; cmd_start "$profile" "$wait" ;;
  esac
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then main "$@"; fi
