#!/usr/bin/env bash
# orin_precheck.sh - da linus: l'Orin e' pronta per la campagna a 3 comandi (RRT + logger)? OK / NOT OK.
#
#   scripts/orin_precheck.sh [--keep] [--no-start] [COMMIT]     default COMMIT = HEAD
#
#   --keep       lascia lo stack su alla fine (pronto per la campagna); default: lo ferma
#   --no-start   solo i controlli statici, non avvia container/stack
#
# Controlli (barra per controllo, valore di ognuno, log in logs/tools/orin_precheck_<data>.log):
#   1 Orin raggiungibile           5 spazio libero per i log dei run (/scratch, >= MIN_FREE_GB=20)
#   2 immagine fissata == linus    6 container + stack su con orin_run_tests (output dal vivo),
#   3 label tested dell'immagine     nessuna missione, nessun movimento
#   4 orin_run_tests dal commit    7 dal riepilogo: SOURCES, planner=corridor_rrt, i due logger,
#                                    stack_args, modello LLM, llama-server, Foxglove, nodi
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_precheck
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/lib/orin_ui.sh
source "$ROOT/scripts/lib/orin_ui.sh"

KEEP=0 START=1 COMMIT=HEAD MIN_FREE_GB=${MIN_FREE_GB:-20}
for a in "$@"; do
  case "$a" in
    --keep) KEEP=1 ;;
    --no-start) START=0 ;;
    -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
SHA=$(git -C "$ROOT" rev-parse --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
N=16 I=0
rec() {   # rec STATO descrizione valore: un controllo, con la barra dei controlli
  I=$((I + 1)); ui_bar "$I" "$N" "controllo $I/$N: $2"; ui_bar_done; check_record "$1" "$2" "$3"
}
is() { [ "$1" = "$2" ] && echo OK || echo "${3:-FAIL}"; }

ui_section "orin_precheck: configurazione"
ui_kv "commit atteso" "$SHA ($(git -C "$ROOT" log -1 --format=%s "$SHA" | cut -c1-60))"
ui_kv "Orin" "$(orin_target)"
ui_kv "opzioni" "keep=$KEEP start=$START min_free=${MIN_FREE_GB}GB"
ui_kv "log" "$F1T_LOG_FILE"
ui_steps 3

ui_step "controlli statici"
if ! orin_up; then rec FAIL "Orin raggiungibile" "ssh $(orin_target) fallito"; summary; echo "NOT OK: Orin irraggiungibile"; exit 1; fi
rec OK "Orin raggiungibile" "$(orin_ssh 'hostname; uptime -p' | tr '\n' ' ')"
pin=$(orin_ssh "awk '\$1==\"revision\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || true)
tag=$(orin_ssh "awk '\$1==\"tag\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || true)
rec "$(is "$pin" "$SHA")" "immagine fissata = linus" "$tag ${pin:0:12} (linus ${SHA:0:12})"
tested=$(orin_ssh "docker image inspect -f '{{index .Config.Labels \"f1tenth.tested\"}}' $tag" 2>/dev/null || true)
rec "$(is "$tested" true WARN)" "label tested" "${tested:-assente} (WARN: deploy con --force-untested)"
want=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || true)
rec "$(is "$have" "$want")" "orin_run_tests dal commit" "${have:0:12} (atteso ${want:0:12})"
free=$(orin_ssh "df -BG --output=avail /scratch/fabiocar | tail -1" | tr -dc 0-9)
rec "$([ "${free:-0}" -ge "$MIN_FREE_GB" ] && echo OK || echo FAIL)" "spazio log (/scratch)" "${free:-?} GB (min $MIN_FREE_GB)"
rec "$(is "$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null')" container)" "modalita' auto" "$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null || echo ?')"
ui_step_end

ui_step "container + stack (orin_run_tests sull'Orin, output dal vivo)"
rc=skip
if [ "$START" = 1 ]; then
  rc=0
  orin_ssh .local/bin/orin_run_tests 2>&1 | while IFS= read -r l; do printf '  orin| %s\n' "$l" >&2; _log_to_file "  orin| $l"; done \
    || rc=$?
  rc=${PIPESTATUS[0]:-$rc}
fi
ui_step_end

ui_step "riepilogo dal container"
out=$(orin_ssh .local/bin/orin_run_tests --summary 2>/dev/null || true)
val() { awk -v k="$1" '$1==k {$1=""; sub(/^ /,""); print; exit}' <<<"$out"; }
missing=$(grep '^missing' <<<"$out" | awk '{print $2}' | tr '\n' ' ' || true)
if [ "$rc" = skip ]; then I=$((I + 1)); check_skip "container + nodi su" "--no-start"
else rec "$([ -z "$missing" ] && echo OK || echo FAIL)" "container + nodi su" "${missing:-tutti presenti} (exit $rc)"; fi
if [ -z "$(val sha)" ]; then
  for d in "SOURCES nel container" planner "logger (test_campaign + mission)" stack_args "modello LLM" \
           llama-server "Foxglove bridge" "nodi ROS in totale" "archivio dei run"; do
    I=$((I + 1)); check_skip "$d" "container fermo: nessun riepilogo"
  done
else
rec "$(is "$(val sha)" "$SHA")" "SOURCES nel container" "$(val sha | cut -c1-12) $(val branch)"
rec "$([[ $(val planner) == corridor_rrt* ]] && echo OK || echo FAIL)" "planner" "$(val planner)"
rec "$(is "$(grep -c '^logger .* on$' <<<"$out" || true)" 2)" "logger (test_campaign + mission)" "$(grep '^logger' <<<"$out" | awk '{print $2"="$3}' | tr '\n' ' ')"
rec "$(is "$(val stack_args)" "profile:=campaign3_rrt")" "stack_args" "$(val stack_args)"
llm=$(val llm_model)
rec "$([[ $llm == *present* ]] && echo OK || echo FAIL)" "modello LLM" "${llm:-?}"
rec "$([[ $(val llama_server) == up* ]] && echo OK || echo WARN)" "llama-server" "$(val llama_server)"
rec "$([[ $(val foxglove) == up* ]] && echo OK || echo FAIL)" "Foxglove bridge" "$(val foxglove)"
rec "$([ "$(val nodes_total)" -gt 10 ] 2>/dev/null && echo OK || echo WARN)" "nodi ROS in totale" "$(val nodes_total)"
rec OK "archivio dei run" "$(val archive)"
fi
ui_section "dettagli"
for k in image describe subject params_file stack_params profile stack_cmd last_run; do ui_kv "$k" "$(val "$k")"; done

if [ "$KEEP" = 0 ] && [ "$START" = 1 ]; then
  log_info "stack fermato (--keep per lasciarlo su)"; orin_ssh .local/bin/orin_run_tests --stop >/dev/null 2>&1 || log_warn "stop stack fallito"
fi
ui_bar 3 3 "precheck finito in $(ui_elapsed)"; ui_bar_done
if summary "orin_precheck ${SHA:0:7}"; then echo "OK"; else echo "NOT OK: vedi i FAIL sopra (log: $F1T_LOG_FILE)"; exit 1; fi
