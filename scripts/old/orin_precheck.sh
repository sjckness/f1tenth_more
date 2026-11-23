#!/usr/bin/env bash
# orin_precheck.sh - da linus: l'Orin e' pronta per la campagna a 3 comandi (RRT + logger)? OK / NOT OK.
#
#   scripts/orin_precheck.sh [--keep] [--no-start] [COMMIT]     default COMMIT = HEAD
#
#   --keep       lascia lo stack su alla fine (pronto per la campagna); default: lo ferma
#   --no-start   solo i controlli, non avvia container ne' stack (riepilogo se il container gira gia')
#
# Controlli (barra per controllo, valore di ognuno, log in logs/tools/orin_precheck_<data>.log):
#   statici   Orin raggiungibile; immagine fissata con lo stesso contenuto di COMMIT; label tested;
#             orin_run_tests installato == COMMIT; spazio per i run (/scratch >= MIN_FREE_GB=20);
#             modalita' container; drift (WARN: gli script partono con --allow-drift)
#   avvio     orin_run_tests sull'Orin (output dal vivo, nessuna missione, l'auto non si muove)
#   riepilogo container sano, SOURCES == COMMIT (contenuto), planner corridor_rrt, i due logger,
#             stack_args, nodi attesi, modello LLM, llama-server, Foxglove 8765, archivio dei run
# Exit: 0 OK, 1 NOT OK, 2 argomenti.
set -euo pipefail
export GIT_PAGER=cat PAGER=cat
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_precheck
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/lib/orin_ui.sh
source "$ROOT/scripts/lib/orin_ui.sh"
# shellcheck source=scripts/lib/orin_common.sh
source "$ROOT/scripts/lib/orin_common.sh"
# shellcheck source=scripts/lib/orin_summary.sh
source "$ROOT/scripts/lib/orin_summary.sh"

KEEP=0 START=1 COMMIT=HEAD MIN_FREE_GB=${MIN_FREE_GB:-20}
for a in "$@"; do
  case "$a" in
    --keep) KEEP=1 ;;
    --no-start) START=0 ;;
    -h|--help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a (scripts/orin_precheck.sh -h)" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
cfg_require config || exit 1
SHA=$(git -C "$ROOT" rev-parse -q --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
N=18 I=0
rec() {   # rec STATO descrizione valore: un controllo, con la barra dei controlli
  I=$((I + 1)); ui_bar "$I" "$N" "controllo $I/$N: $2"; ui_bar_done; check_record "$1" "$2" "$3"
}
skip() { I=$((I + 1)); check_skip "$1" "$2"; }
is() { if [ "$1" = "$2" ]; then echo OK; else echo "${3:-FAIL}"; fi; }

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
if [ -n "$pin" ] && same_image "$pin" "$SHA"; then rec OK "immagine fissata ~ linus" "$tag (contenuto = ${SHA:0:12})"
else rec FAIL "immagine fissata ~ linus" "$tag ${pin:0:12}: contenuto diverso da ${SHA:0:12} (scripts/orin_deploy.sh)"; fi
tested=$(orin_ssh "docker image inspect -f '{{index .Config.Labels \"f1tenth.tested\"}}' $tag" 2>/dev/null || true)
rec "$(is "$tested" true WARN)" "label tested" "${tested:-assente} (deploy con --force-untested)"
want=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || true)
rec "$(is "$have" "$want")" "orin_run_tests = commit" "${have:0:12} (atteso ${want:0:12})"
free=$( { orin_ssh "df -BG --output=avail /scratch/fabiocar | tail -1" 2>/dev/null || true; } | tr -dc 0-9)
rec "$([ "${free:-0}" -ge "$MIN_FREE_GB" ] && echo OK || echo FAIL)" "spazio run (/scratch)" "${free:-?} GB (min $MIN_FREE_GB)"
mode=$(orin_ssh 'cat ~/.config/f1tenth/car_mode 2>/dev/null' || true)
rec "$(is "$mode" container WARN)" "modalita' auto" "${mode:-?} (orin_run_tests passa a container da solo)"
rec WARN "drift del checkout nativo" "$(orin_drift_summary) -> --allow-drift"
ui_step_end

ui_step "container + stack (orin_run_tests sull'Orin, output dal vivo)"
rc=skip
before=$(orin_ssh .local/bin/orin_run_tests --summary 2>/dev/null || true)
STARTED=0
[[ $(summary_val "$before" stack_cmd) == *supervisor_bringup* ]] || STARTED=1
log_info "stack prima del precheck: $( [ "$STARTED" = 1 ] && echo fermo || echo 'gia in esecuzione: non lo fermo alla fine')"
if [ "$START" = 1 ]; then
  set +e
  orin_ssh .local/bin/orin_run_tests 2>&1 | while IFS= read -r l; do printf '  orin| %s\n' "$l" >&2; _log_to_file "  orin| $l"; done
  rc=${PIPESTATUS[0]}
  set -e
  log_info "orin_run_tests: exit $rc (0 pronto, 1 non pronto, 3 nel container)"
else
  log_info "--no-start: nessun avvio"
fi
ui_step_end

ui_step "riepilogo dal container"
out=$(orin_ssh .local/bin/orin_run_tests --summary 2>/dev/null || true)
summary_checks "$out" "$SHA" "$rc"
ui_section "dettagli"
for k in image describe subject params_file stack_params profile stack_cmd last_run; do ui_kv "$k" "$(summary_val "$out" "$k")"; done

if [ "$KEEP" = 0 ] && [ "$START" = 1 ] && [ "$STARTED" = 1 ]; then
  log_info "stack fermato (l'ha avviato il precheck; --keep per lasciarlo su)"
  orin_ssh .local/bin/orin_run_tests --stop >/dev/null 2>&1 || log_warn "stop dello stack fallito"
fi
ui_bar 3 3 "precheck finito in $(ui_elapsed)"; ui_bar_done
if summary "orin_precheck ${SHA:0:7}"; then echo "OK"; else echo "NOT OK: vedi i FAIL sopra (log: $F1T_LOG_FILE)"; exit 1; fi
