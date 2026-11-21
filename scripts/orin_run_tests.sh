#!/usr/bin/env bash
# orin_run_tests.sh - campagna a 3 comandi (RRT + logger) SULL'ORIN, solo dall'immagine fissata.
# Installato da scripts/orin_deploy.sh in ~/.local/bin/orin_run_tests (stesso commit dell'immagine).
#
#   orin_run_tests             stack_args := profile:=campaign3_rrt (il precedente in stack_args.bak_*),
#                              container su (docker_orin up: device, rete host, volumi, archivio dei run
#                              in /home/fabiocar/f1tenth_archive come la campagna precedente), stack
#                              avviato, attesa dei nodi con barra, riepilogo completo
#   orin_run_tests --summary   solo il riepilogo (nessun avvio)
#   orin_run_tests --stop      ferma lo stack (il container resta su)
#
# Nessun movimento: lo stack parte senza missione (le missioni da Foxglove, porta 8765).
# Log: ~/.local/state/orin_run_tests/<data>.log. F1T_RUN_WAIT_S (240) = attesa massima dei nodi.
set -euo pipefail
PROFILE=campaign3_rrt
ARGS_FILE=$HOME/.config/f1tenth_car/stack_args
DO=$HOME/.local/bin/docker_orin
CTL=$HOME/.local/bin/f1tenth_ctl_car
WAIT_S=${F1T_RUN_WAIT_S:-240}
NODES="/component_supervisor_node /corridor_rrt /test_campaign_logger /mission_logger_node /foxglove_bridge"
LOG=$HOME/.local/state/orin_run_tests/$(date +%Y%m%d_%H%M%S).log
mkdir -p "$(dirname "$LOG")"
exec 2> >(tee -a "$LOG" >&2)
T0=$(date +%s) LIVE=0 STEPS=4 STEP=0

el() { local s=$(( $(date +%s) - ${1:-$T0} )); printf '%02d:%02d' $((s / 60)) $((s % 60)); }
log() { if [ "$LIVE" = 1 ]; then printf '\n' >&2; LIVE=0; fi; printf '%s %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
kv() { printf '  %-30s %s\n' "$1" "$2" >&2; }
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
ctr_running() { [ "$(docker inspect -f '{{.State.Running}}' f1tenth_car 2>/dev/null)" = true ]; }
nodes_now() { "$DO" exec bash -c 'ROS_SUPER_CLIENT=TRUE timeout 12 ros2 node list --no-daemon 2>/dev/null' 2>/dev/null || true; }

# Riepilogo, calcolato nel container: righe "chiave valore" su stdout (le legge scripts/orin_precheck.sh).
summary() {
  ctr_running || { echo "container fermo"; return 1; }
  "$DO" exec bash -s -- "$NODES" <<'EOF'
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
nodes=$(timeout 15 ros2 node list --no-daemon 2>/dev/null | sort -u)
echo "nodes_total $(grep -c . <<<"$nodes")"
if grep -qx /component_supervisor_node <<<"$nodes"; then
  echo "planner $(timeout 10 ros2 param get --no-daemon /component_supervisor_node planner_controller_mode 2>/dev/null | awk '{print $NF}')"
else
  echo "planner $(awk '/planner_controller_mode:/{print $2}' "$prof") (dal profilo; stack fermo)"
fi
for n in /test_campaign_logger /mission_logger_node; do
  if grep -qx "$n" <<<"$nodes"; then echo "logger ${n#/} on"; else echo "logger ${n#/} off"; fi
done
for n in $wanted; do if grep -qx "$n" <<<"$nodes"; then echo "node_ok $n"; else echo "missing $n"; fi; done
cmd=$(pgrep -af 'ros2 launch f1tenth_bringup supervisor_bringup' | head -1 | cut -d' ' -f2-)
echo "stack_cmd ${cmd:-non in esecuzione}"
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
  local out; out=$(summary 2>/dev/null) || { log "container fermo: nessun riepilogo"; return 1; }
  log "== riepilogo campagna 3 comandi ($PROFILE)"
  while read -r k v; do
    case "$k" in
      missing) kv "!! nodo MANCANTE" "$v" ;;
      node_ok) kv "ok nodo" "$v" ;;
      *) kv "$k" "$v" ;;
    esac
  done <<<"$out"
  echo "$out"
}

case "${1:-}" in
  --summary) print_summary; exit ;;
  --stop) run "$CTL" stop stack; exit ;;
  '') ;;
  *) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

log "orin_run_tests: profilo $PROFILE, log $LOG"
kv "host" "$(hostname) $(uptime -p)"
kv "immagine fissata" "$(awk '{printf "%s=%s ", $1, substr($2,1,19)}' ~/.config/f1tenth/car_image)"
kv "modalita' auto" "$(cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?)"
kv "container" "$(docker ps -a --filter 'name=^f1tenth_car$' --format '{{.Image}} {{.Status}}')"
kv "nodi attesi" "$NODES"

step "stack_args := profile:=$PROFILE"
kv "prima" "$(cat "$ARGS_FILE" 2>/dev/null || echo assente)"
if [ "$(cat "$ARGS_FILE" 2>/dev/null)" != "profile:=$PROFILE" ]; then
  if [ -f "$ARGS_FILE" ]; then bk=$ARGS_FILE.bak_$(date +%Y%m%d_%H%M%S); cp -p "$ARGS_FILE" "$bk"; kv "backup" "$bk"; fi
  echo "profile:=$PROFILE" > "$ARGS_FILE"
fi
kv "dopo" "$(cat "$ARGS_FILE")"

step "container f1tenth_car"
if ctr_running; then log "gia' su"; else run "$DO" up; fi
kv "stato" "$(docker ps --filter 'name=^f1tenth_car$' --format '{{.Image}} {{.Status}}')"

step "stack (f1tenth_ctl start stack)"
run "$CTL" start stack || log "start stack: exit $? (gia' avviato? decide l'attesa dei nodi)"

step "attesa dei nodi (max $WAIT_S s)"
want=$(wc -w <<<"$NODES") t1=$(date +%s)
while :; do
  now=$(nodes_now); got=0 miss=''
  for n in $NODES; do if grep -qx "$n" <<<"$now"; then got=$((got + 1)); else miss="$miss ${n#/}"; fi; done
  bar "$got" "$want" "nodi $got/$want, $(grep -c . <<<"$now" || true) in totale, $(el "$t1")${miss:+, mancano:$miss}"
  [ "$got" = "$want" ] && break
  if [ $(( $(date +%s) - t1 )) -ge "$WAIT_S" ]; then log "tempo scaduto: mancano$miss"; break; fi
  sleep 5
done
log ""

out=$(print_summary || true)
bar $STEPS $STEPS "finito in $(el)"; log ""
if grep -q '^missing' <<<"$out"; then log "NON PRONTO: nodi mancanti (f1tenth_ctl_car logs stack -n 80)"; exit 1; fi
log "PRONTO: missioni da Foxglove; per fermare: orin_run_tests --stop"
