# shellcheck shell=bash
# scripts/lib/orin_summary.sh - lettura del riepilogo di `orin_run_tests --summary` (righe
# "chiave valore") e i controlli che ne derivano. Usata da scripts/orin_precheck.sh e dai test
# (scripts/tests/orin_scripts_test.sh). Chi la usa definisce:
#   rec STATO descrizione valore     un controllo (OK/WARN/FAIL)
#   skip descrizione motivo          un controllo non eseguito
#   same_image A B                   (scripts/lib/orin_common.sh)
#
#   summary_val OUT CHIAVE           il valore della prima riga CHIAVE
#   summary_checks OUT SHA RC        11 controlli; RC = exit di orin_run_tests, o "skip"
[ -n "${_ORIN_SUMMARY_SH:-}" ] && return 0
_ORIN_SUMMARY_SH=1

SUMMARY_CHECKS=(
  "container + nodi su" "SOURCES ~ commit" "planner" "logger (test_campaign + mission)" "stack_args"
  "comando dello stack" "nodi ROS in totale" "modello LLM" "llama-server" "Foxglove bridge" "archivio dei run"
)

summary_val() { awk -v k="$2" '$1==k {$1=""; sub(/^ /,""); print; exit}' <<<"$1"; }

summary_checks() {
  local out=$1 sha=$2 rc=$3 v missing nlog d
  if [ -z "$(summary_val "$out" sha)" ]; then
    # Con un avvio fatto (RC non skip) un container fermo e' un fallimento, non un controllo saltato.
    if [ "$rc" != skip ]; then rec FAIL "${SUMMARY_CHECKS[0]}" "container fermo dopo orin_run_tests (exit $rc)"
    else skip "${SUMMARY_CHECKS[0]}" "container fermo: nessun riepilogo"; fi
    for d in "${SUMMARY_CHECKS[@]:1}"; do skip "$d" "container fermo: nessun riepilogo"; done
    return 0
  fi
  missing=$(awk '$1=="missing"{print $2}' <<<"$out" | tr '\n' ' ')
  if [ "$rc" = skip ]; then
    rec "$([ -z "$missing" ] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[0]}" "${missing:-tutti presenti} (non avviato: --no-start)"
  else
    rec "$([ -z "$missing" ] && [ "$rc" = 0 ] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[0]}" "${missing:-tutti presenti} (orin_run_tests exit $rc)"
  fi
  v=$(summary_val "$out" sha)
  rec "$(same_image "$v" "$sha" && echo OK || echo FAIL)" "${SUMMARY_CHECKS[1]}" "${v:0:12} $(summary_val "$out" branch)"
  v=$(summary_val "$out" planner)
  rec "$([[ $v == corridor_rrt* ]] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[2]}" "$v"
  nlog=$(grep -c '^logger .* on$' <<<"$out" || true)
  rec "$([ "$nlog" = 2 ] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[3]}" "$(awk '$1=="logger"{printf "%s=%s ", $2, $3}' <<<"$out")"
  v=$(summary_val "$out" stack_args)
  rec "$([ "$v" = "profile:=campaign3_rrt" ] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[4]}" "$v"
  v=$(summary_val "$out" stack_cmd)
  rec "$([[ $v == *profile:=campaign3_rrt* ]] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[5]}" "${v:-?}"
  v=$(summary_val "$out" nodes_total)
  rec "$([ "${v:-0}" -gt 10 ] 2>/dev/null && echo OK || echo WARN)" "${SUMMARY_CHECKS[6]}" "${v:-?}"
  v=$(summary_val "$out" llm_model)
  rec "$([[ $v == *" present"* ]] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[7]}" "${v:-?}"
  v=$(summary_val "$out" llama_server)
  rec "$([[ $v == up* ]] && echo OK || echo WARN)" "${SUMMARY_CHECKS[8]}" "${v:-?} (serve solo al planner LLM)"
  v=$(summary_val "$out" foxglove)
  rec "$([[ $v == up* ]] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[9]}" "${v:-?}"
  v=$(summary_val "$out" archive)
  rec "$([ -n "$v" ] && echo OK || echo FAIL)" "${SUMMARY_CHECKS[10]}" "${v:-?}; sull'host $(summary_val "$out" archive_host | cut -d' ' -f1)"
}
