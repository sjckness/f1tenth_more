#!/usr/bin/env bash
# desc: cosa gira sull'Orin: immagine, sha, moduli, uptime, differenze da main, drift (file modificati a mano)
# alias: status
#
#   f1t orin status [--fetch] [--json FILE]
#
# Solo letture (una connessione ssh). --fetch aggiorna prima i ref orin/* (git fetch orin, con
# keepalive) per conoscere il commit dell'immagine se linus non lo ha. Exit 1 se l'Orin non risponde
# o il container dell'auto dovrebbe girare e non e' sano; i WARN (drift, versioni) non bloccano.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

FETCH=0 JSON=
while [ $# -gt 0 ]; do
  case "$1" in
    --fetch) FETCH=1 ;;
    --json) JSON=${2:?--json FILE}; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $1 (f1t orin status -h)" 2 ;;
  esac
  shift
done

cfg_load
cfg_require config || exit 1
cfg_print F1T_ORIN_HOST F1T_ORIN_USER F1T_ORIN_CONTAINER F1T_ORIN_UNIT F1T_MAIN_REF F1T_ORIN_DEPLOYED_JSON
log_steps 4

log_step "Orin raggiungibile"
if ! orin_facts; then
  check_record FAIL "ssh $(orin_target)" "${ORIN[error]:-non risponde}"
  for c in "cosa gira" "versioni vs main" "drift"; do check_skip "$c" "Orin non raggiungibile"; done
  [ -z "$JSON" ] || summary_json "$JSON" host="$F1T_ORIN_HOST"
  summary "orin status"; exit 1
fi
up_h=$(( ${ORIN[uptime_s]:-0} / 3600 ))
check_record OK "ssh $(orin_target)" "${ORIN[hostname]}, acceso da ${up_h} h"

log_step "Cosa gira"
mode=none
[ "${ORIN[unit_container]:-}" = active ] && mode=container
[ "${ORIN[unit_gateway]:-}" = active ] && mode=native
check_record "$( [ "$mode" = none ] && echo WARN || echo OK)" "modalita' (stack_mode)" "$mode"
if [ "${ORIN[docker]:-}" != ok ]; then
  check_record FAIL "docker" "${ORIN[docker]:-?}"
else
  st=${ORIN[ctr_state]:-absent} hl=${ORIN[ctr_health]:-}
  if [ "$st" = running ] && [ "$hl" = healthy ]; then state=OK
  elif [ "$mode" = container ]; then state=FAIL
  else state=WARN; fi
  check_record "$state" "container $F1T_ORIN_CONTAINER" "$st${hl:+, $hl}${ORIN[ctr_started]:+, dal ${ORIN[ctr_started]:0:16}}"
  if [ "$st" != absent ]; then
    check_record OK "immagine" "${ORIN[ctr_image]} (${ORIN[ctr_image_id]:7:12})"
    if [ -n "${ORIN[pin_id]:-}" ] && [ "${ORIN[pin_id]}" = "${ORIN[ctr_image_id]}" ]; then
      check_record OK "pin = container" "${ORIN[pin_tag]} (fissato ${ORIN[pin_at]:0:16})"
    else
      check_record WARN "pin = container" "pin ${ORIN[pin_tag]:-nessuno}, gira ${ORIN[ctr_image]}"
    fi
    check_record OK "revisione" "${ORIN[ctr_revision]:0:12} ${ORIN[ctr_branch]:-} (${ORIN[ctr_describe]:-})"
    if [ "${ORIN[ctr_tested]:-}" = true ]; then check_record OK "testata (label)" "f1tenth.tested=true"
    else check_record WARN "testata (label)" "label f1tenth.tested assente: non costruita da f1t build"; fi
    procs=${ORIN[ctr_stack_procs]:-0}
    check_record OK "stack ROS nel container" "$( [ "$procs" -gt 0 ] 2>/dev/null && echo "attivo ($procs processi)" || echo fermo)"
  fi
  if [ -n "${ORIN[deployed_json]:-}" ]; then
    check_record OK "deployed.json" "${ORIN[deployed_json_path]}: $(printf '%s' "${ORIN[deployed_json]}" \
      | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("image","?"), d.get("deployed_at","?"), d.get("deployed_by",""))' 2>/dev/null || echo illeggibile)"
  else
    check_record WARN "deployed.json" "assente: nessun f1t deploy ancora"
  fi
fi

log_step "Versioni rispetto a $F1T_MAIN_REF (linus)"
rev=${ORIN[ctr_revision]:-}
if [ -n "$rev" ] && [ "$FETCH" = 1 ] && ! git -C "$F1T_ROOT" cat-file -e "${rev}^{commit}" 2>/dev/null; then
  log_info "git fetch orin (keepalive) per conoscere $rev"
  GIT_SSH_COMMAND="ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4" timeout 600 git -C "$F1T_ROOT" fetch -q orin || log_warn "git fetch orin fallito"
fi
if [ -z "$rev" ]; then
  check_skip "commit vs $F1T_MAIN_REF" "nessuna revisione (container assente)"
elif git -C "$F1T_ROOT" cat-file -e "${rev}^{commit}" 2>/dev/null; then
  lr=$(git -C "$F1T_ROOT" rev-list --left-right --count "$rev...$F1T_MAIN_REF")
  ahead=${lr%%[[:space:]]*} behind=${lr##*[[:space:]]}
  if [ "$ahead" = 0 ] && [ "$behind" = 0 ]; then check_record OK "commit vs $F1T_MAIN_REF" "uguale"
  else check_record WARN "commit vs $F1T_MAIN_REF" "${rev:0:8}: $ahead commit non in main, $behind di main mancanti"; fi
else
  check_record WARN "commit vs $F1T_MAIN_REF" "${rev:0:8} sconosciuto su linus (f1t orin status --fetch)"
fi
if [ "${ORIN[ctr_state]:-}" = running ]; then
  tmp=$(mktemp); trap 'rm -f "$tmp"' EXIT
  if orin_script "$F1T_CORE/orin/remote/modules.sh" --running "$F1T_ORIN_CONTAINER" "$F1T_MODULES_EXCLUDE_RE" > "$tmp" 2>/dev/null; then
    diffs=$(F1T_ROOT=$F1T_ROOT python3 "$F1T_CORE/lib/modules.py" diff "$tmp" "ref:$F1T_MAIN_REF" | awk '{printf "%s %s->%s, ", $1, $2, $3}')
    n=$(F1T_ROOT=$F1T_ROOT python3 "$F1T_CORE/lib/modules.py" diff "$tmp" "ref:$F1T_MAIN_REF" | wc -l)
    if [ "$n" = 0 ]; then check_record OK "moduli vs $F1T_MAIN_REF" "$(wc -l < "$tmp") moduli, versioni uguali"
    else check_record WARN "moduli vs $F1T_MAIN_REF" "$n diversi (Orin->main): ${diffs%, }"; fi
  else
    check_record WARN "moduli vs $F1T_MAIN_REF" "versioni non lette dal container"
  fi
else
  check_skip "moduli vs $F1T_MAIN_REF" "container non in esecuzione"
fi

log_step "Drift: modifiche fatte a mano sull'Orin"
n=${ORIN[ctr_fs_changes]:-}
if [ -z "$n" ]; then check_skip "file cambiati nel container" "container non in esecuzione"
elif [ "$n" = 0 ]; then check_record OK "file cambiati nel container" "0 sotto /opt e /etc"
else check_record WARN "file cambiati nel container" "$n: ${ORIN[ctr_fs_changes_files]}"; fi
for k in build_repo native; do
  label=$( [ "$k" = build_repo ] && echo "repo di build ($F1T_ORIN_BUILD_REPO)" || echo "checkout nativo ($F1T_ORIN_NATIVE_WS)")
  d=${ORIN[${k}_dirty]:-}
  if [ -z "$d" ]; then check_record WARN "$label" "${ORIN[${k}_branch]:-n/d}"
  elif [ "$d" = 0 ]; then check_record OK "$label" "pulito, ${ORIN[${k}_branch]} ${ORIN[${k}_head]}"
  else check_record WARN "$label" "$d file modificati: ${ORIN[${k}_dirty_files]}"; fi
done
check_record OK "override a caldo ($F1T_ORIN_OVERRIDE_DIR)" "${ORIN[override_stack_args]:-${ORIN[override_files]:-nessuno}}"
[ -z "${ORIN[override_files]:-}" ] || log_info "override: file presenti: ${ORIN[override_files]}"

[ -z "$JSON" ] || summary_json "$JSON" host="$F1T_ORIN_HOST" image="${ORIN[ctr_image]:-}" revision="${ORIN[ctr_revision]:-}"
summary "orin status"
