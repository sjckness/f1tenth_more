#!/usr/bin/env bash
# desc: precheck HIL: Orin, rete, orologi, versioni, risorse, GPU, discovery ROS tra i due host
#
#   f1t hil precheck -disp <linus|work> [--scenario NOME] [--allow-mismatch] [--no-discovery]
#
# -disp (obbligatorio) = la macchina che fa girare Isaac Sim; i suoi valori sono in
# tools/core/hosts/<nome>.env e il precheck rifiuta di girare su un'altra macchina. Lo scenario
# (default smoke) da' dominio, porta del Discovery Server e immagine. Ogni check stampa il valore
# trovato. Fine: "HIL PRECHECK: OK" (o FAIL con l'elenco) e logs/tools/hil_precheck_last.json.
# Solo letture, tranne il test di discovery: un container usa e getta sull'Orin (DS + sottoscrittore,
# rete host, porta HIL_DS_PORT) e un publisher qui, per F1T_DISCOVERY_TIMEOUT_S secondi.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
HIL_USAGE="f1t hil precheck -disp <linus|work> [--scenario NOME] [--allow-mismatch] [--no-discovery]"
# shellcheck source=tools/core/hil/lib/hil.sh
source "$F1T_CORE/hil/lib/hil.sh"

hil_parse_args "$@"
ALLOW=0 DISCOVERY=1
for a in "${HIL_REST[@]}"; do
  case "$a" in
    --allow-mismatch) ALLOW=1 ;;
    --no-discovery) DISCOVERY=0 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_fail "argomento sconosciuto: $a"; echo "uso: $HIL_USAGE" >&2; exit 2 ;;
  esac
done
hil_load
cfg_print F1T_DISP F1T_SCENARIO HOST_HOSTNAME HOST_LINK_IFACE HOST_LINK_IP HOST_ORIN_LINK_IP F1T_ORIN_HOST \
  HIL_DOMAIN_ID HIL_RMW HIL_DS HOST_ISAAC_IMAGE HOST_HIL_WORKTREE
LAST=$F1T_ROOT/logs/tools/hil_precheck_last.json
finish() {
  summary_json "$LAST" disp="$DISP" scenario="$F1T_SCENARIO" image="${IMAGE:-}" image_id="${IMAGE_ID:-}"
  if summary "hil precheck -disp $DISP"; then
    echo "HIL PRECHECK: OK"
    exit 0
  fi
  echo "HIL PRECHECK: FAIL ($(checks_count FAIL)):"
  for i in "${!_CHK_STATE[@]}"; do [ "${_CHK_STATE[$i]}" != FAIL ] || echo "  - ${_CHK_NAME[$i]}: ${_CHK_VAL[$i]}"; done
  exit 1
}
log_steps 7

log_step "Questa macchina e l'Orin"
check "macchina dichiarata (-disp $DISP)" probe_local_host "$HOST_HOSTNAME"
check "Orin sul collegamento (ping)" probe_ping "$HOST_ORIN_LINK_IP" 3
LINK_OK=$( [ "$(check_last)" = FAIL ] && echo 0 || echo 1)
ORIN_OK=1
if orin_facts; then
  check_record OK "ssh $(orin_target)" "${ORIN[hostname]}"
else
  check_record FAIL "ssh $(orin_target)" "${ORIN[error]:-non risponde}"
  ORIN_OK=0
fi

log_step "Rete e DDS"
check "interfaccia, IP, MTU (linus)" probe_iface "$HOST_LINK_IFACE" "$HOST_LINK_IP" "$HOST_LINK_MTU" "$HOST_ORIN_LINK_IP"
check "rotta diretta verso l'Orin" probe_route_direct "$HOST_ORIN_LINK_IP"
if [ "$LINK_OK" = 1 ]; then check "perdita pacchetti (${F1T_PING_COUNT}x)" probe_ping "$HOST_ORIN_LINK_IP" "$F1T_PING_COUNT"
else check_skip "perdita pacchetti (${F1T_PING_COUNT}x)" "Orin non risponde sul collegamento"; fi
declare -A OH=()
if [ "$ORIN_OK" = 1 ]; then
  while IFS='=' read -r k v; do [ -n "$k" ] && OH[$k]=$v; done < <(orin_script "$F1T_CORE/hil/remote/orin_hil.sh" facts "$STACK_CONTAINER" "$HIL_DS_PORT" "$HOST_LINK_IP" 2>/dev/null || true)
  if [ "${OH[route_direct]:-no}" = yes ]; then check_record OK "rotta diretta Orin -> sim" "${OH[route]:0:60}, mtu ${OH[mtu]}"
  else check_record FAIL "rotta diretta Orin -> sim" "${OH[route]:-nessuna rotta verso $HOST_LINK_IP}"; fi
  [ "${OH[mtu]:-}" = "$HOST_LINK_MTU" ] && check_record OK "MTU uguale ai due lati" "$HOST_LINK_MTU" \
    || check_record WARN "MTU uguale ai due lati" "linus $HOST_LINK_MTU, Orin ${OH[mtu]:-?}"
  [ "${OH[ds_port_busy]:-0}" = 0 ] && check_record OK "porta DS $HIL_DS_PORT libera sull'Orin" "libera" \
    || check_record WARN "porta DS $HIL_DS_PORT libera sull'Orin" "occupata (run precedente? f1t hil down)"
  need=$((HIL_SOCKET_BUFFER_MB * 1024 * 1024))
  for side in linus orin; do
    v=$( [ "$side" = linus ] && cat /proc/sys/net/core/rmem_max || echo "${OH[rmem_max]:-0}")
    [ "${v:-0}" -ge "$need" ] && check_record OK "rmem_max $side >= ${HIL_SOCKET_BUFFER_MB} MB" "$v" \
      || check_record WARN "rmem_max $side >= ${HIL_SOCKET_BUFFER_MB} MB" "$v (immagini a raffica perse: SUDO_TODO)"
  done
else
  check_skip "rete lato Orin" "ssh non riuscito"
fi
rmw_env=${RMW_IMPLEMENTATION:-}
if [ -n "$rmw_env" ] && [ "$rmw_env" != "$HIL_RMW" ]; then check_record FAIL "RMW coerente" "shell: $rmw_env, scenario: $HIL_RMW (Zenoh non va con Isaac 5.1)"
elif [ -e "/opt/ros/jazzy/lib/lib${HIL_RMW}.so" ]; then check_record OK "RMW coerente" "$HIL_RMW su entrambi (dominio $HIL_DOMAIN_ID, DS $HIL_DS)"
else check_record FAIL "RMW coerente" "/opt/ros/jazzy/lib/lib${HIL_RMW}.so assente su $(hostname)"; fi
if [ -n "${ROS_DOMAIN_ID:-}" ] && [ "$ROS_DOMAIN_ID" != "$HIL_DOMAIN_ID" ]; then
  log_info "la shell ha ROS_DOMAIN_ID=$ROS_DOMAIN_ID: gli script HIL usano $HIL_DOMAIN_ID (scenario)"
fi

log_step "Orologi"
if [ "$ORIN_OK" = 1 ]; then check "differenza orologio linus-Orin" probe_clock_offset
else check_skip "differenza orologio linus-Orin" "ssh non riuscito"; fi

log_step "Versioni: immagine dello stack vs $F1T_MAIN_REF"
IMAGE='' IMAGE_ID=''
if [ "$ORIN_OK" = 1 ]; then
  declare -A IM=()
  while IFS='=' read -r k v; do [ -n "$k" ] && IM[$k]=$v; done < <(orin_script "$F1T_CORE/hil/remote/orin_hil.sh" image "$F1T_ORIN_PIN_FILE" "${STACK_IMAGE:-}" 2>/dev/null || true)
  IMAGE=${IM[image]:-} IMAGE_ID=${IM[id]:-}
  if [ -z "$IMAGE_ID" ]; then
    check_record FAIL "immagine dello stack" "${IMAGE:-nessuna} non presente sull'Orin"
  else
    check_record OK "immagine dello stack" "$IMAGE (${IMAGE_ID:7:12})"
    tmp=$(mktemp); trap 'rm -f "$tmp"' EXIT
    if orin_script "$F1T_CORE/orin/remote/modules.sh" --image "$IMAGE" "$F1T_MODULES_EXCLUDE_RE" > "$tmp" 2>/dev/null && [ -s "$tmp" ]; then
      nd=$(python3 "$F1T_CORE/lib/modules.py" diff "$tmp" "ref:$F1T_MAIN_REF" | wc -l)
      first=$(python3 "$F1T_CORE/lib/modules.py" diff "$tmp" "ref:$F1T_MAIN_REF" | head -3 | awk '{printf "%s %s->%s ", $1, $2, $3}')
      dj=$(printf '%s' "${ORIN[deployed_json]:-}" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("sha","")[:8])' 2>/dev/null || true)
      val="sha ${dj:-${ORIN[ctr_revision]:0:8}}; $nd moduli diversi da main ${first}"
      if [ "$nd" = 0 ]; then check_record OK "versioni moduli = main" "$val"
      elif [ "$ALLOW" = 1 ]; then check_record WARN "versioni moduli = main" "$val (--allow-mismatch)"
      else check_record FAIL "versioni moduli = main" "$val (--allow-mismatch per procedere)"; fi
    else
      check_record FAIL "versioni moduli = main" "versioni non lette dall'immagine"
    fi
  fi
else
  check_skip "versioni moduli = main" "ssh non riuscito"
fi

log_step "Risorse dell'Orin"
if [ "$ORIN_OK" = 1 ]; then
  check_record "$( [ "${ORIN[docker]:-}" = ok ] && echo OK || echo FAIL)" "docker sull'Orin" "${ORIN[docker]:-?}"
  procs=${ORIN[ctr_stack_procs]:-0}
  [ "${procs:-0}" = 0 ] && check_record OK "stack dell'auto fermo" "container ${ORIN[ctr_state]:-assente}, nessun processo dello stack" \
    || check_record FAIL "stack dell'auto fermo" "$procs processi dello stack in $F1T_ORIN_CONTAINER: stessa CPU dell'HIL, fermalo"
  case "${OH[hil_ctr]:-}" in
    '') check_record OK "container HIL libero" "$STACK_CONTAINER assente" ;;
    *) check_record WARN "container HIL libero" "$STACK_CONTAINER ${OH[hil_ctr]} (run precedente: f1t hil down)" ;;
  esac
  [ "${ORIN[disk_root_free_gb]:-0}" -ge 5 ] && check_record OK "disco / Orin" "${ORIN[disk_root_free_gb]} GB liberi" \
    || check_record FAIL "disco / Orin" "${ORIN[disk_root_free_gb]:-?} GB liberi (< 5)"
  [ "${ORIN[mem_avail_mb]:-0}" -ge 4000 ] && check_record OK "RAM Orin" "${ORIN[mem_avail_mb]} MB liberi su ${ORIN[mem_total_mb]}" \
    || check_record FAIL "RAM Orin" "${ORIN[mem_avail_mb]:-?} MB liberi (< 4000)"
  t=${ORIN[temp_max_c]:-0}; t=${t%.*}
  if [ "${t:-0}" -lt 70 ]; then check_record OK "temperatura Orin" "${ORIN[temp_max_c]} C"
  elif [ "$t" -lt 85 ]; then check_record WARN "temperatura Orin" "${ORIN[temp_max_c]} C"
  else check_record FAIL "temperatura Orin" "${ORIN[temp_max_c]} C"; fi
  [ "${ORIN[nvpmodel]:-}" = MAXN ] && check_record OK "nvpmodel / clock" "${ORIN[nvpmodel]}, cpu0 ${ORIN[cpu_max_mhz]} MHz, perf ${ORIN[unit_perf]}" \
    || check_record WARN "nvpmodel / clock" "${ORIN[nvpmodel]:-?} (atteso MAXN), cpu0 ${ORIN[cpu_max_mhz]} MHz"
  e=${ORIN[ctr_log_errors_10m]:-}
  if [ -z "$e" ]; then check_record OK "errori recenti nel log" "container dell'auto non in esecuzione"
  elif [ "$e" = 0 ]; then check_record OK "errori recenti nel log" "0 negli ultimi 10 min"
  else check_record WARN "errori recenti nel log" "$e righe error/exception negli ultimi 10 min (docker_orin logs)"; fi
else
  check_skip "risorse dell'Orin" "ssh non riuscito"
fi

log_step "Host della sim ($DISP)"
gpu_val=$(probe_gpu "$HOST_GPU_NAME" "$HOST_MIN_VRAM_FREE_MB" 2>&1) && gpu_rc=0 || gpu_rc=$?
if [ "$gpu_rc" = 0 ]; then check_record OK "GPU e VRAM libera (>= $HOST_MIN_VRAM_FREE_MB MiB)" "$gpu_val"
else
  hint=
  for s in ${HOST_GPU_SERVICES:-}; do
    [ "$(systemctl --user is-active "$s" 2>/dev/null || true)" = active ] && hint="$hint; $s attivo: systemctl --user stop $s"
  done
  check_record FAIL "GPU e VRAM libera (>= $HOST_MIN_VRAM_FREE_MB MiB)" "$gpu_val$hint"
fi
iv=$(probe_docker_image "$HOST_ISAAC_IMAGE" 2>&1) && irc=0 || irc=$?
if [ "$irc" = 0 ]; then
  if docker image inspect -f '{{json .Config.Labels}}{{json .Config.Env}}{{json .RepoTags}}' "$HOST_ISAAC_IMAGE" 2>/dev/null | grep -q "$HOST_ISAAC_VERSION"; then
    check_record OK "Isaac Sim $HOST_ISAAC_VERSION" "$iv"
  else check_record WARN "Isaac Sim $HOST_ISAAC_VERSION" "$iv: versione non scritta nell'immagine"; fi
else
  check_record FAIL "Isaac Sim $HOST_ISAAC_VERSION" "$iv"
fi
sim_exe=${SIM_CMD%% *}
if [ -x "$HOST_HIL_WORKTREE/$sim_exe" ]; then check_record OK "script della sim" "$HOST_HIL_WORKTREE/$sim_exe ($(git -C "$HOST_HIL_WORKTREE" rev-parse --abbrev-ref HEAD 2>/dev/null))"
else check_record FAIL "script della sim" "$HOST_HIL_WORKTREE/$sim_exe assente (HOST_HIL_WORKTREE)"; fi
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$SIM_CONTAINER"; then check_record FAIL "container della sim libero" "$SIM_CONTAINER gia' in esecuzione"
else check_record OK "container della sim libero" "$SIM_CONTAINER assente"; fi
check "disco / ($DISP)" probe_disk / "$HOST_MIN_DISK_GB"

log_step "Discovery ROS tra i due host (< ${F1T_DISCOVERY_TIMEOUT_S} s)"
if [ "$DISCOVERY" = 0 ]; then
  check_skip "topic visto dall'altro host" "--no-discovery"
elif [ "$ORIN_OK" = 0 ] || [ "$LINK_OK" = 0 ] || [ -z "$IMAGE_ID" ]; then
  check_skip "topic visto dall'altro host" "Orin, collegamento o immagine non pronti"
elif [ "${OH[ds_port_busy]:-0}" != 0 ]; then
  check_skip "topic visto dall'altro host" "porta $HIL_DS_PORT occupata sull'Orin"
else
  orin_ssh mkdir -p "$F1T_ORIN_XFER"
  orin_scp "$F1T_CORE/hil/lib/ros_probe.py" "$F1T_ORIN_XFER/ros_probe.py"
  topic=/f1t_precheck_$$
  mapfile -t o < <(_orin_opts)
  remote=$(printf '%q ' bash -s -- probe "$IMAGE" "$F1T_ORIN_XFER" "$HOST_ORIN_LINK_IP" "$HIL_DS_PORT" "$HIL_DOMAIN_ID" "$topic" 15)
  dv=$(hil_probe pingpong "$topic" --timeout "$F1T_DISCOVERY_TIMEOUT_S" -- \
       bash -c "ssh $(printf '%q ' "${o[@]}") $(orin_target) $(printf '%q' "$remote") < $(printf '%q' "$F1T_CORE/hil/remote/orin_hil.sh")" 2>&1 | tail -1) && drc=0 || drc=$?
  case "$drc" in
    0) check_record OK "topic visto dall'altro host" "$dv" ;;
    10) check_record WARN "topic visto dall'altro host" "$dv (oltre ${F1T_DISCOVERY_TIMEOUT_S} s)" ;;
    *) check_record FAIL "topic visto dall'altro host" "$dv" ;;
  esac
fi
finish
