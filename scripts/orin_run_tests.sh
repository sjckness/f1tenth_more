#!/usr/bin/env bash
# orin_run_tests.sh - campagna a 3 comandi (RRT + logger) SULL'ORIN, dalla shell dell'HOST (non nel
# container), solo con l'immagine fissata. Installato da scripts/orin_deploy.sh in
# ~/.local/bin/orin_run_tests (stesso commit dell'immagine).
#
#   orin_run_tests               avvia tutto e aspetta che sia pronto:
#                                1 stack_args := profile:=campaign3_rrt (il precedente in stack_args.bak_*)
#                                2 container: stack_mode container --allow-drift (ferma il lato nativo,
#                                  unit systemd, device, rete host, volumi; i run in
#                                  /home/fabiocar/f1tenth_archive come la campagna precedente)
#                                3 stack: avviato; se gira gia' con altri argomenti lo riavvia
#                                4 attesa dei nodi con barra; poi riepilogo completo
#   orin_run_tests --summary     solo il riepilogo (nessun avvio)
#   orin_run_tests --restart     come senza opzioni, ma riavvia lo stack anche se e' gia' giusto
#   orin_run_tests --stop        ferma lo stack (il container resta su)
#   orin_run_tests --no-allow-drift   senza --allow-drift (il container non parte se c'e' drift)
#
# Nessun movimento: lo stack parte senza missione (le missioni da Foxglove, ws://<orin>:8765).
# Exit: 0 pronto, 1 non pronto (nodi mancanti, container o stack non partiti), 2 argomenti,
# 3 lanciato dentro il container. Log: ~/.local/state/orin_run_tests/<data>.log.
# F1T_RUN_WAIT_S (240) = attesa massima dei nodi.
set -euo pipefail
PROFILE=campaign3_rrt
ARGS_FILE=$HOME/.config/f1tenth_car/stack_args
DO=$HOME/.local/bin/docker_orin
CTL=$HOME/.local/bin/f1tenth_ctl_car
SMODE=$HOME/.local/bin/stack_mode
CTR=f1tenth_car
WAIT_S=${F1T_RUN_WAIT_S:-240}
NODES="/component_supervisor_node /corridor_rrt /test_campaign_logger /mission_logger_node /foxglove_bridge"
CONTAINER_MARKERS=${F1T_CONTAINER_MARKERS:-/.dockerenv /opt/f1tenth_ws/SOURCES}

MODE=start DRIFT=(--allow-drift)
for a in "$@"; do
  case "$a" in
    --summary) MODE=summary ;;
    --stop) MODE=stop ;;
    --restart) MODE=restart ;;
    --no-allow-drift) DRIFT=() ;;
    -h|--help) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "orin_run_tests: opzione sconosciuta '$a' (orin_run_tests -h)" >&2; exit 2 ;;
  esac
done

# Il container usa la rete dell'host: il suo prompt e' identico (fabiocar@jetson-orin).
for m in $CONTAINER_MARKERS; do
  if [ -e "$m" ]; then
    echo "orin_run_tests: sei DENTRO il container ($m esiste). Esci con 'exit' e rilancia dalla shell dell'Orin." >&2
    exit 3
  fi
done
for t in "$DO" "$CTL" "$SMODE"; do
  [ -x "$t" ] || { echo "orin_run_tests: manca $t (docker/orin/host/install.sh)" >&2; exit 1; }
done

LOG=$HOME/.local/state/orin_run_tests/$(date +%Y%m%d_%H%M%S).log
mkdir -p "$(dirname "$LOG")"
exec 2> >(tee -a "$LOG" >&2)
T0=$(date +%s) LIVE=0 STEPS=4 STEP=0

el() { local s=$(( $(date +%s) - ${1:-$T0} )); printf '%02d:%02d' $((s / 60)) $((s % 60)); }
log() { if [ "$LIVE" = 1 ]; then printf '\n' >&2; LIVE=0; fi; printf '%s %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
kv() { printf '  %-30s %s\n' "$1" "$2" >&2; }
die() { log "ERRORE: $*"; log "log completo: $LOG"; exit 1; }
bar() {   # bar CUR TOT TESTO
  local cur=$1 tot=$2 w=32 n line; [ "$tot" -gt 0 ] || tot=1; [ "$cur" -le "$tot" ] || cur=$tot
  n=$(( cur * w / tot ))
  line=$(printf '[%s%s] %3d%% %s' "$(printf '%*s' "$n" '' | tr ' ' '#')" \
    "$(printf '%*s' $(( w - n )) '' | tr ' ' '.')" $(( cur * 100 / tot )) "$3")
  if [ -t 1 ]; then printf '\r\033[K%s' "${line:0:$(( $(tput cols 2>/dev/null || echo 120) - 1 ))}" >&2; LIVE=1
  else printf '%s\n' "$line" >&2; fi
}
step() { STEP=$((STEP + 1)); log ""; bar $((STEP - 1)) $STEPS "totale: passo $STEP di $STEPS, $(el)"; log "[PASSO $STEP/$STEPS] $*"; }
run() { log "\$ $*"; "$@"; }
ctr_state() { docker inspect -f '{{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{end}}' "$CTR" 2>/dev/null || echo absent; }
ctr_running() { [ "$(docker inspect -f '{{.State.Running}}' "$CTR" 2>/dev/null)" = true ]; }
in_ctr() { docker exec -i "$CTR" /opt/f1tenth_ws/entrypoint.sh "$@"; }
nodes_now() { in_ctr bash -c 'ROS_SUPER_CLIENT=TRUE timeout 12 ros2 node list --no-daemon 2>/dev/null | sort -u' 2>/dev/null || true; }
# [s]: il pattern non trova il bash -c che lo contiene (pgrep esclude se stesso, non il padre).
stack_cmd() { in_ctr bash -c "pgrep -af 'ros2 launch f1tenth_bringup [s]upervisor_bringup' | head -1 | cut -d' ' -f2-" 2>/dev/null || true; }
# Controlli READY di f1tenth_ctl che falliscono (stack, odometria, scan, tf, ...), in una riga.
ctl_failing() {
  "$CTL" status --json 2>/dev/null | python3 -c 'import json,sys
try: print(" ".join(json.load(sys.stdin).get("failing", [])) or "nessuno")
except Exception: print("?")' 2>/dev/null || echo "?"
}

# Riepilogo, calcolato nel container: righe "chiave valore" su stdout (le legge scripts/orin_precheck.sh).
summary() {
  ctr_running || { echo "container fermo"; return 1; }
  in_ctr bash -s -- "$NODES" <<'EOF'
set -uo pipefail
wanted=$1
export ROS_SUPER_CLIENT=TRUE
share() { echo "$(ros2 pkg prefix "$1")/share/$1"; }
prof=$(share f1tenth_bringup)/config/profiles/campaign3_rrt.yaml
params=$(share f1tenth_params)/config/stack_params.yaml
src=/opt/f1tenth_ws/SOURCES
echo "sha $(awk '$1=="commit"{print $2}' $src)"
echo "branch $(awk '$1=="branch"{print $2}' $src)"
echo "describe $(awk '$1=="describe"{print $2}' $src)"
echo "subject $(awk '$1=="subject"{$1=""; print substr($0,2)}' $src)"
echo "image ${F1TENTH_IMAGE_REF:-?} ${F1TENTH_IMAGE_ID:0:19}"
echo "stack_args $(cat ~/.config/f1tenth_car/stack_args 2>/dev/null || echo absent)"
echo "params_file $prof"
echo "stack_params $params"
echo "profile $(awk '/^  [a-z_]+:/{printf "%s=%s ", $1, $2}' "$prof" | tr -d ":'")"
cmd=$(pgrep -af 'ros2 launch f1tenth_bringup [s]upervisor_bringup' | head -1 | cut -d' ' -f2-)
echo "stack_cmd ${cmd:-non in esecuzione}"
nodes=$(timeout 15 ros2 node list --no-daemon 2>/dev/null | sort -u)
echo "nodes_total $(grep -c . <<<"$nodes")"
planner=''
if grep -qx /component_supervisor_node <<<"$nodes"; then
  planner=$(timeout 15 ros2 param get --no-daemon --hide-type /component_supervisor_node planner_controller_mode 2>/dev/null | tail -1 | tr -d ' ')
  [ -n "$planner" ] && planner="$planner (parametro del supervisor)"
fi
if [ -z "$planner" ] && [ -n "$cmd" ]; then
  case "$cmd" in
    *planner_controller_mode:=*) planner="$(sed 's/.*planner_controller_mode:=\([a-z_]*\).*/\1/' <<<"$cmd") (dal comando dello stack)" ;;
    *profile:=campaign3_rrt*) planner="$(awk '/planner_controller_mode:/{print $2}' "$prof") (dal profilo nel comando dello stack)" ;;
  esac
fi
echo "planner ${planner:-$(awk '/planner_controller_mode:/{print $2}' "$prof") (dal profilo; stack fermo)}"
for n in /test_campaign_logger /mission_logger_node; do
  if grep -qx "$n" <<<"$nodes"; then echo "logger ${n#/} on"; else echo "logger ${n#/} off"; fi
done
for n in $wanted; do if grep -qx "$n" <<<"$nodes"; then echo "node_ok $n"; else echo "missing $n"; fi; done
python3 - "$params" "$(share llm)/config/models.yaml" <<'PY'
import os, sys, urllib.request, yaml
key = yaml.safe_load(open(sys.argv[1]))['model']['default']
m = yaml.safe_load(open(sys.argv[2]))[key]
path = os.path.expanduser(m['model_path'])
size = f"{os.path.getsize(path) / 1e9:.1f}GB" if os.path.isfile(path) else ''
print(f"llm_model {key} {path} {'present' if size else 'MISSING'} {size}")
try:
    urllib.request.urlopen(f"http://127.0.0.1:{m.get('port', 8083)}/health", timeout=2)
    print(f"llama_server up (port {m.get('port')})")
except Exception as exc:
    print(f"llama_server down (port {m.get('port')}: {type(exc).__name__})")
PY
if (exec 3<>/dev/tcp/127.0.0.1/8765) 2>/dev/null; then echo "foxglove up (ws://<orin>:8765)"; else echo "foxglove down (8765)"; fi
arch=/home/fabiocar/f1tenth_archive
echo "archive $arch $(df -h --output=avail "$arch" | tail -1 | tr -d ' ') liberi"
echo "last_run $(ls -td "$arch"/*/*/ 2>/dev/null | head -1 || echo nessuno)"
EOF
}

print_summary() {   # stampa leggibile su stderr, righe chiave/valore su stdout
  local out
  out=$(summary 2>/dev/null) || { log "container $CTR non in esecuzione ($(ctr_state)): nessun riepilogo"; echo "container fermo"; return 1; }
  local arch
  arch=$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/home/fabiocar/f1tenth_archive"}}{{.Source}}{{end}}{{end}}' "$CTR" 2>/dev/null || true)
  out="$out
archive_host ${arch:-?} (montato in /home/fabiocar/f1tenth_archive nel container)"
  log "== riepilogo campagna 3 comandi ($PROFILE)"
  kv "container" "$(ctr_state)"
  kv "READY di f1tenth_ctl, falliscono" "$(ctl_failing)"
  while read -r k v; do
    case "$k" in
      missing) kv "!! nodo MANCANTE" "$v" ;;
      node_ok) kv "ok nodo" "$v" ;;
      *) kv "$k" "$v" ;;
    esac
  done <<<"$out"
  echo "$out"
}

case "$MODE" in
  summary) print_summary; exit ;;
  stop) run "$CTL" stop stack; exit ;;
esac

log "orin_run_tests: profilo $PROFILE, $(hostname), log $LOG"
kv "immagine fissata" "$(awk '$1=="tag"{print $2}' ~/.config/f1tenth/car_image 2>/dev/null || echo assente)"
kv "modalita' auto" "$(cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?)"
kv "container" "$(ctr_state)"
kv "drift" "${DRIFT[*]:-controllo stretto (--no-allow-drift)}"
kv "nodi attesi" "$NODES"

step "stack_args := profile:=$PROFILE"
kv "prima" "$(cat "$ARGS_FILE" 2>/dev/null || echo assente)"
if [ "$(cat "$ARGS_FILE" 2>/dev/null)" != "profile:=$PROFILE" ]; then
  if [ -f "$ARGS_FILE" ]; then bk=$ARGS_FILE.bak_$(date +%Y%m%d_%H%M%S); cp -p "$ARGS_FILE" "$bk"; kv "backup" "$bk"; fi
  echo "profile:=$PROFILE" > "$ARGS_FILE"
fi
kv "dopo" "$(cat "$ARGS_FILE")"

step "container $CTR (stack_mode container ${DRIFT[*]:-})"
if ctr_running; then
  log "gia' in esecuzione: $(ctr_state)"
else
  run "$SMODE" container "${DRIFT[@]}" || die "il container non e' partito (sopra). Diagnosi: $DO status; $DO history -n 5; $DO drift"
fi
pin_id=$(awk '$1=="id"{print $2}' ~/.config/f1tenth/car_image 2>/dev/null || true)
run_id=$(docker inspect -f '{{.Image}}' "$CTR" 2>/dev/null || true)
[ -z "$pin_id" ] || [ "$pin_id" = "$run_id" ] \
  || log "ATTENZIONE: il container gira con ${run_id:7:12}, il pin dice ${pin_id:7:12} (orin_deploy.sh lo riallinea)"
kv "stato" "$(ctr_state)"

step "stack (f1tenth_ctl)"
cmd=$(stack_cmd)
kv "stack in esecuzione" "${cmd:-no}"
want_tail=" $(cat "$ARGS_FILE")"
if [ -n "$cmd" ] && { [ "$MODE" = restart ] || [[ $cmd != *"$want_tail" ]]; }; then
  log "lo stack gira $( [ "$MODE" = restart ] && echo '(--restart)' || echo "con argomenti diversi da stack_args ($want_tail)"): lo fermo e lo riavvio"
  run "$CTL" stop stack || die "stop stack fallito ($CTL logs stack -n 80)"
  cmd=''
fi
if [ -z "$cmd" ]; then
  run "$CTL" start stack || die "start stack fallito ($CTL logs stack -n 80)"
fi

step "attesa dei nodi (max $WAIT_S s)"
want=$(wc -w <<<"$NODES") t1=$(date +%s) miss=''
while :; do
  now=$(nodes_now); got=0 miss=''
  for n in $NODES; do if grep -qx "$n" <<<"$now"; then got=$((got + 1)); else miss="$miss ${n#/}"; fi; done
  bar "$got" "$want" "nodi $got/$want, $(grep -c . <<<"$now" || true) in totale, $(el "$t1")${miss:+, mancano:$miss}"
  [ "$got" = "$want" ] && break
  if [ $(( $(date +%s) - t1 )) -ge "$WAIT_S" ]; then log "tempo scaduto dopo $WAIT_S s: mancano$miss"; break; fi
  sleep 5
done
log ""

out=$(print_summary || true)
bar $STEPS $STEPS "finito in $(el)"; log ""
if ! grep -q '^sha ' <<<"$out"; then
  log "NON PRONTO: il container non risponde ($(ctr_state)). Diagnosi: $DO status; $DO logs -n 100; log: $LOG"
  exit 1
fi
if grep -q '^missing' <<<"$out"; then
  log "NON PRONTO: nodi mancanti ($(grep '^missing' <<<"$out" | awk '{print $2}' | tr '\n' ' '))"
  log "diagnosi: $CTL logs stack -n 120   |   $DO logs -n 100   |   log: $LOG"
  exit 1
fi
log "PRONTO: missioni da Foxglove (ws://$(hostname):8765). Fermare: orin_run_tests --stop"
