#!/usr/bin/env bash
# orin_precheck.sh - da linus: l'Orin e' pronta per la campagna a 3 comandi (RRT + logger)? OK / NOT OK.
#
#   scripts/orin_precheck.sh [--keep] [COMMIT]     default COMMIT = HEAD
#
# Controlli: Orin raggiungibile; immagine fissata = commit di linus (pin, SOURCES nel container);
# orin_run_tests installato da quel commit; spazio libero per i log dei run; modello LLM presente;
# poi avvia container + stack (orin_run_tests: profile:=campaign3_rrt, nessuna missione, nessun
# movimento) e controlla nodi, planner=corridor_rrt, i due logger. Alla fine ferma lo stack
# (--keep lo lascia su per la campagna).
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_precheck
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"

KEEP=0 COMMIT=HEAD MIN_FREE_GB=${MIN_FREE_GB:-20}
for a in "$@"; do
  case "$a" in
    --keep) KEEP=1 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
SHA=$(git -C "$ROOT" rev-parse --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
rec() { check_record "$1" "$2" "$3"; }   # rec OK|FAIL|WARN descrizione valore
is() { [ "$1" = "$2" ] && echo OK || echo "${3:-FAIL}"; }

if ! orin_up; then rec FAIL "Orin raggiungibile" "ssh $(orin_target) fallito"; summary; echo "NOT OK: Orin irraggiungibile"; exit 1; fi
rec OK "Orin raggiungibile" "$(orin_ssh hostname)"

pin=$(orin_ssh "awk '\$1==\"revision\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || true)
rec "$(is "$pin" "$SHA")" "immagine fissata = linus" "${pin:0:12} vs ${SHA:0:12}"
want=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || true)
rec "$(is "$have" "$want")" "orin_run_tests dal commit" "${have:0:12}"
free=$(orin_ssh "df -BG --output=avail /scratch/fabiocar | tail -1" | tr -dc 0-9)
rec "$([ "${free:-0}" -ge "$MIN_FREE_GB" ] && echo OK || echo FAIL)" "spazio log (/scratch)" "${free:-?} GB (min $MIN_FREE_GB)"

log_info "avvio container + stack (orin_run_tests, nessuna missione)"
if orin_ssh .local/bin/orin_run_tests > "$F1T_LOG_FILE.run" 2>&1; then rc=0; else rc=$?; fi
out=$(orin_ssh .local/bin/orin_run_tests --summary 2>/dev/null || true)
val() { awk -v k="$1" '$1==k {$1=""; sub(/^ /,""); print; exit}' <<<"$out"; }
missing=$(grep '^missing' <<<"$out" | tr '\n' ' ' || true)
rec "$(is "$rc" 0)" "container + nodi su" "${missing:-tutti presenti} (exit $rc)"
rec "$(is "$(val sha)" "$SHA")" "SOURCES nel container" "$(val sha | cut -c1-12) $(val branch)"
rec "$(is "$(val planner)" corridor_rrt)" "planner" "$(val planner)"
rec "$(is "$(grep -c '^logger .* on$' <<<"$out")" 2)" "logger (test_campaign + mission)" "$(grep '^logger' <<<"$out" | awk '{print $2"="$3}' | tr '\n' ' ')"
rec "$(is "$(val stack_args)" "profile:=campaign3_rrt")" "stack_args" "$(val stack_args)"
llm=$(val llm_model)
rec "$([[ $llm == *present ]] && echo OK || echo FAIL)" "modello LLM" "${llm:-?}"
log_info "params: $(val params_file)"

if [ "$KEEP" = 0 ]; then orin_ssh .local/bin/orin_run_tests --stop >/dev/null 2>&1 || log_warn "stop stack fallito"; fi
if summary "orin_precheck ${SHA:0:7}"; then echo "OK"; else echo "NOT OK: vedi i FAIL sopra (log avvio: $F1T_LOG_FILE.run)"; exit 1; fi
