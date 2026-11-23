# shellcheck shell=bash
# scripts/car/lib/car_ready.sh - i controlli di scripts/car/ready.sh, calcolati su due array di fatti
# (nessun ssh qui: i test li chiamano con fatti finti). Dopo tools/core/lib/log.sh e orin_common.sh.
#
#   ready_eval SHA PROFILO|-      registra un check_record per controllo. Legge:
#                                 ORIN[...]  da orin_facts (tools/core/orin/remote/facts.sh)
#                                 R[...]     da scripts/car/lib/remote_ready.sh
# Soglie (variabili): READY_MIN_SCRATCH_GB (20), READY_MIN_ROOT_GB (5), READY_MIN_MEM_MB (4000),
# READY_TEMP_WARN_C (70), READY_TEMP_FAIL_C (85): le stesse del precheck HIL.
[ -n "${_CAR_READY_SH:-}" ] && return 0
_CAR_READY_SH=1
READY_MIN_SCRATCH_GB=${READY_MIN_SCRATCH_GB:-20} READY_MIN_ROOT_GB=${READY_MIN_ROOT_GB:-5}
READY_MIN_MEM_MB=${READY_MIN_MEM_MB:-4000}
READY_TEMP_WARN_C=${READY_TEMP_WARN_C:-70} READY_TEMP_FAIL_C=${READY_TEMP_FAIL_C:-85}

_ge() { [ "${1:-x}" -ge "$2" ] 2>/dev/null; }

ready_eval() {
  local sha=$1 profile=$2 t n path want have rev st val extra local_sha t_int
  # ---- container e immagine
  if [ "${R[car_mode]:-?}" = container ]; then check_record OK "modalita' container" container
  else check_record FAIL "modalita' container" "${R[car_mode]:-?} (car-container start)"; fi
  st="${ORIN[ctr_state]:-absent} ${ORIN[ctr_health]:-}"
  if [ "$st" = "running healthy" ]; then check_record OK "container sano" "$st"
  else check_record FAIL "container sano" "$st (car-container start; docker_orin logs -n 100)"; fi
  if [ -n "${ORIN[pin_id]:-}" ] && [ "${ORIN[pin_id]}" = "${ORIN[ctr_image_id]:-}" ]; then
    check_record OK "container = immagine fissata" "${ORIN[pin_tag]:-}"
  else
    check_record FAIL "container = immagine fissata" "pin ${ORIN[pin_id]:7:12} container ${ORIN[ctr_image_id]:7:12} (scripts/car/deploy.sh)"
  fi
  rev=${ORIN[ctr_revision]:-}
  if [ -z "$rev" ]; then
    check_record FAIL "immagine = commit" "revisione dell'immagine sconosciuta"
  elif ! git -C "$F1T_ROOT" cat-file -e "$rev^{commit}" 2>/dev/null; then
    check_record FAIL "immagine = commit" "${rev:0:12} assente su linus (git fetch orin)"
  elif same_image "$rev" "$sha"; then
    check_record OK "immagine = commit" "${rev:0:12} ~ ${sha:0:12}"
  else
    check_record FAIL "immagine = commit" "${rev:0:12} != ${sha:0:12} (scripts/car/deploy.sh ${sha:0:12})"
  fi
  if [ "${ORIN[ctr_tested]:-}" = true ]; then check_record OK "label tested" true
  else check_record WARN "label tested" "${ORIN[ctr_tested]:-assente}"; fi

  # ---- script dell'host
  for t in "${CAR_TOOLS[@]}"; do
    path=${t%%:*} n=${t##*:}
    want=$(car_tool_sum "$sha" "$path") have=${R[tool_$n]:-}
    if [ -z "$want" ]; then check_record FAIL "$n = commit" "$path non e' nel commit ${sha:0:12} (committa)"
    elif [ -n "$have" ] && [ "$have" = "$want" ]; then check_record OK "$n = commit" "${have:0:12}"
    elif [ -z "$have" ]; then check_record FAIL "$n = commit" "assente sull'Orin (scripts/car/deploy.sh)"
    else check_record FAIL "$n = commit" "${have:0:12} != ${want:0:12} (scripts/car/deploy.sh)"; fi
  done

  # ---- niente gia' in esecuzione
  n=${ORIN[ctr_stack_procs]:-}
  if [ "$n" = 0 ]; then check_record OK "stack fermo" "nessun processo dello stack"
  elif [ -z "$n" ]; then check_record FAIL "stack fermo" "non letto (container fermo?)"
  else check_record FAIL "stack fermo" "$n processi dello stack nel container (car-stack stop)"; fi
  if [ -z "${R[host_ros_procs]:-}" ]; then check_record OK "nessun ROS sull'host" "-"
  else check_record FAIL "nessun ROS sull'host" "pid ${R[host_ros_procs]} fuori dal container"; fi

  # ---- sensori
  case "${R[preflight_rc]:-absent}" in
    0) check_record OK "sensori (preflight)" "${R[preflight_msg]:-}" ;;
    absent) check_record FAIL "sensori (preflight)" "car-preflight assente (scripts/car/deploy.sh)" ;;
    *) check_record FAIL "sensori (preflight)" "${R[preflight_fail]:-${R[preflight_msg]:-rc ${R[preflight_rc]}}}" ;;
  esac

  # ---- stack_args e profilo
  if [ "${R[stack_args_ok]:-no}" = yes ]; then
    extra=$(tr ' ' '\n' <<<"${R[stack_args]:-}" | grep -v -E '^(profile|profiles_dir):=|^$' | tr '\n' ' ' || true)
    if [ -z "$extra" ]; then check_record OK "stack_args" "${R[stack_args]:-(vuoto)}"
    else check_record WARN "stack_args" "vincono sul profilo: $extra"; fi
  else
    check_record FAIL "stack_args" "ignorato da f1tenth_ctl (una riga di name:=value): ${R[stack_args]:-}"
  fi
  if [ "$profile" = - ]; then
    check_skip "profilo" "nessuno (--profile NOME)"
  elif [ "${R[profile_present]:-no}" != yes ]; then
    check_record FAIL "profilo $profile" "assente sull'Orin (ci sono: ${R[profiles]:-nessuno}; scripts/car/profiles.sh push)"
  else
    local_sha=$(sha256sum "$F1T_ROOT/$CAR_PROFILES_SRC/$profile.yaml" 2>/dev/null | cut -d' ' -f1 || true)
    if [ "$local_sha" != "${R[profile_sha]:-}" ]; then
      check_record FAIL "profilo $profile = linus" "Orin ${R[profile_sha]:0:12} linus ${local_sha:-assente} (scripts/car/profiles.sh push)"
    else
      check_record OK "profilo $profile = linus" "${local_sha:0:12}"
    fi
    val=${R[profile_check]:-}
    case "$val" in
      "OK "*) check_record OK "profilo $profile valido" "launch_profile check nel container" ;;
      "SKIP "*) check_record SKIP "profilo $profile valido" "${val#SKIP }" ;;
      *) check_record FAIL "profilo $profile valido" "${val#ERROR }" ;;
    esac
  fi

  # ---- risorse
  if _ge "${R[scratch_free_gb]:-}" "$READY_MIN_SCRATCH_GB"; then check_record OK "spazio per i run" "${R[scratch_free_gb]} GB"
  else check_record FAIL "spazio per i run" "${R[scratch_free_gb]:-?} GB (< $READY_MIN_SCRATCH_GB)"; fi
  if _ge "${ORIN[disk_root_free_gb]:-}" "$READY_MIN_ROOT_GB"; then check_record OK "spazio su /" "${ORIN[disk_root_free_gb]} GB"
  else check_record FAIL "spazio su /" "${ORIN[disk_root_free_gb]:-?} GB (< $READY_MIN_ROOT_GB)"; fi
  if _ge "${ORIN[mem_avail_mb]:-}" "$READY_MIN_MEM_MB"; then check_record OK "RAM libera" "${ORIN[mem_avail_mb]} MB"
  else check_record WARN "RAM libera" "${ORIN[mem_avail_mb]:-?} MB (< $READY_MIN_MEM_MB)"; fi
  t_int=${ORIN[temp_max_c]:-}; t_int=${t_int%.*}
  if ! _ge "$t_int" 0; then check_record WARN "temperatura" "non letta"
  elif [ "$t_int" -lt "$READY_TEMP_WARN_C" ]; then check_record OK "temperatura" "${ORIN[temp_max_c]} C"
  elif [ "$t_int" -lt "$READY_TEMP_FAIL_C" ]; then check_record WARN "temperatura" "${ORIN[temp_max_c]} C"
  else check_record FAIL "temperatura" "${ORIN[temp_max_c]} C (>= $READY_TEMP_FAIL_C)"; fi

  # ---- orologio e drift (solo avvisi)
  if [ "${ORIN[time_sync]:-}" = yes ]; then check_record OK "orologio sincronizzato" yes
  else check_record WARN "orologio sincronizzato" "${ORIN[time_sync]:-?} (i timestamp dei bag)"; fi
  if [ "${ORIN[ctr_fs_changes]:-0}" = 0 ]; then check_record OK "container senza modifiche a mano" 0
  else check_record WARN "container senza modifiche a mano" "${ORIN[ctr_fs_changes]} file: ${ORIN[ctr_fs_changes_files]:-}"; fi
}
