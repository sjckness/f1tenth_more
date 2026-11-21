#!/usr/bin/env bash
# orin_run_tests.sh - campagna a 3 comandi (RRT + logger) SULL'ORIN, solo dall'immagine fissata.
# Installato da scripts/orin_deploy.sh in ~/.local/bin/orin_run_tests (stesso commit dell'immagine).
#
#   orin_run_tests             stack_args := profile:=campaign3_rrt (il precedente in stack_args.bak_*),
#                              container su (docker_orin up: device, rete host, volumi, archivio dei
#                              run in /home/fabiocar/f1tenth_archive come la campagna precedente),
#                              stack avviato, attesa dei nodi, riepilogo
#   orin_run_tests --summary   solo il riepilogo (nessun avvio)
#   orin_run_tests --stop      ferma lo stack (il container resta su)
#
# Nessun movimento: lo stack parte senza missione. I 3 comandi poi con
#   f1tenth_ctl_car send planner "..."; f1tenth_ctl_car mission-load <id>; f1tenth_ctl_car mission-start
set -euo pipefail
PROFILE=campaign3_rrt
ARGS_FILE=$HOME/.config/f1tenth_car/stack_args
DO=$HOME/.local/bin/docker_orin
CTL=$HOME/.local/bin/f1tenth_ctl_car
WAIT_S=${F1T_RUN_WAIT_S:-240}
NODES="/component_supervisor_node /corridor_rrt /test_campaign_logger /mission_logger_node"
log() { printf '%s %s\n' "$(date +%H:%M:%S)" "$*" >&2; }

ctr_running() { [ "$(docker inspect -f '{{.State.Running}}' f1tenth_car 2>/dev/null)" = true ]; }

# Riepilogo, calcolato nel container: righe "chiave valore" (le legge scripts/orin_precheck.sh).
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
echo "image ${F1TENTH_IMAGE_REF:-?}"
echo "stack_args $(cat ~/.config/f1tenth_car/stack_args 2>/dev/null || echo absent)"
echo "params_file $prof (+ $params)"
nodes=$(timeout 15 ros2 node list --no-daemon 2>/dev/null)
if grep -qx /component_supervisor_node <<<"$nodes"; then
  echo "planner $(timeout 10 ros2 param get --no-daemon /component_supervisor_node planner_controller_mode 2>/dev/null | awk '{print $NF}')"
  for n in /test_campaign_logger /mission_logger_node; do
    if grep -qx "$n" <<<"$nodes"; then echo "logger ${n#/} on"; else echo "logger ${n#/} off"; fi
  done
else
  echo "planner $(awk '/planner_controller_mode:/{print $2}' "$prof") (dal profilo; stack fermo)"
fi
for n in $wanted; do grep -qx "$n" <<<"$nodes" || echo "missing $n"; done
python3 - "$params" "$(share llm)/config/models.yaml" <<'PY'
import os, sys, yaml
key = yaml.safe_load(open(sys.argv[1]))['model']['default']
path = os.path.expanduser(yaml.safe_load(open(sys.argv[2]))[key]['model_path'])
print(f"llm_model {key} {path} {'present' if os.path.isfile(path) else 'MISSING'}")
PY
EOF
}

case "${1:-}" in
  --summary) summary; exit ;;
  --stop) "$CTL" stop stack; exit ;;
  '') ;;
  *) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

log "1/4 stack_args := profile:=$PROFILE"
if [ "$(cat "$ARGS_FILE" 2>/dev/null)" != "profile:=$PROFILE" ]; then
  [ -f "$ARGS_FILE" ] && cp -p "$ARGS_FILE" "$ARGS_FILE.bak_$(date +%Y%m%d_%H%M%S)"
  echo "profile:=$PROFILE" > "$ARGS_FILE"
fi
log "2/4 container (immagine fissata: $(awk '$1=="tag"{print $2}' ~/.config/f1tenth/car_image))"
ctr_running || "$DO" up
log "3/4 stack"
"$CTL" start stack || log "    start stack: exit $? (gia' avviato? decide l'attesa dei nodi)"
log "    attesa dei nodi ($WAIT_S s max): $NODES"
for _ in $(seq "$((WAIT_S / 10))"); do
  if out=$(summary 2>/dev/null) && ! grep -q '^missing' <<<"$out"; then break; fi
  sleep 10
done
log "4/4 riepilogo"
out=$(summary || true)
echo "$out"
if grep -q '^missing' <<<"$out"; then log "NODI MANCANTI"; exit 1; fi
log "pronto: f1tenth_ctl_car send planner \"<comando>\""
