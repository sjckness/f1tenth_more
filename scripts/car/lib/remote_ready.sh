#!/usr/bin/env bash
# Gira SULL'ORIN (orin_script, da scripts/car/ready.sh): i fatti per "pronto a lanciare lo stack" che
# tools/core/orin/remote/facts.sh non raccoglie. Solo letture: non cambia niente.
#   remote_ready.sh CONTAINER PROFILO|- PROFILES_DIR SCRATCH_DIR
# Righe chiave=valore, valori su una riga:
#   car_mode            ~/.config/f1tenth/car_mode
#   tool_<nome>         sha256 di ~/.local/bin/<nome> (car-stack, car-container, car-preflight) o assente
#   preflight_rc/_msg   exit e righe di car-preflight (ZED USB3, lidar, VESC)
#   host_ros_procs      processi ROS dell'host FUORI da ogni container (stack nativo, strays): pid
#   stack_args/_ok      la riga di ~/.config/f1tenth_car/stack_args e se ctl_container.env la applica
#   scratch_free_gb     spazio libero dove finiscono run, bag e log
#   profile_*           il profilo chiesto: presente, sha256, launch_profile check nel container
set -euo pipefail
CTR=$1 PROFILE=$2 PDIR=${3/#\~/$HOME} SCRATCH=$4
kv() { printf '%s=%s\n' "$1" "$(printf '%s' "${2:-}" | tr '\n\t' '  ' | sed 's/  */ /g; s/ $//')"; }

kv car_mode "$(cat "$HOME/.config/f1tenth/car_mode" 2>/dev/null || echo '?')"
for t in car-stack car-container car-preflight; do
  kv "tool_$t" "$(sha256sum "$HOME/.local/bin/$t" 2>/dev/null | cut -d' ' -f1 || true)"
done

if [ -x "$HOME/.local/bin/car-preflight" ]; then
  rc=0; out=$("$HOME/.local/bin/car-preflight" 2>&1) || rc=$?
  kv preflight_rc "$rc"
  kv preflight_msg "$(grep -E '^(PASS|FAIL) ' <<<"$out" | awk '{print $1, $2}' | tr '\n' ';')"
  kv preflight_fail "$(grep -E '^FAIL ' <<<"$out" | cut -c7- | head -3 | tr '\n' ';')"
else
  kv preflight_rc absent
fi

strays=''
for p in $(pgrep -f 'supervisor_bringup|component_supervisor_node|stack_bringup|fast-discovery-server|fastdds discovery' 2>/dev/null || true); do
  grep -q -E 'docker|containerd|libpod' "/proc/$p/cgroup" 2>/dev/null && continue
  strays+="$p "
done
kv host_ros_procs "$strays"

af=$HOME/.config/f1tenth_car/stack_args
if [ -r "$af" ]; then
  line=$(grep -m1 '[^[:space:]]' "$af" || true)
  n=$(grep -c '[^[:space:]]' "$af" || true)
  ok=yes
  [ "${n:-0}" -le 1 ] || ok=no
  re='^[A-Za-z_][A-Za-z0-9_]*:=[A-Za-z0-9_./,:+@%-]*$'
  for tok in $line; do [[ $tok =~ $re ]] || ok=no; done
  kv stack_args "$line"
  kv stack_args_ok "$ok"
else
  kv stack_args ""
  kv stack_args_ok yes
fi

kv scratch_free_gb "$(df -P -BG "$SCRATCH" 2>/dev/null | awk 'NR==2{sub("G","",$4);print $4}')"

kv profiles "$(find "$PDIR" -maxdepth 1 -name '*.yaml' -printf '%f\n' 2>/dev/null | sed 's/\.yaml$//' | sort | tr '\n' ' ')"
if [ "$PROFILE" != - ]; then
  f=$PDIR/$PROFILE.yaml
  if [ -f "$f" ]; then
    kv profile_present yes
    kv profile_sha "$(sha256sum "$f" | cut -d' ' -f1)"
    if [ "$(docker inspect -f '{{.State.Running}}' "$CTR" 2>/dev/null)" = true ]; then
      chk=$(docker exec -i "$CTR" /opt/f1tenth_ws/entrypoint.sh python3 -m f1tenth_bringup.launch_profile \
              check "$PROFILE" --profiles-dir "$PDIR" </dev/null 2>&1 | grep -E '^(OK|ERROR) ' | tail -1 || true)
      kv profile_check "${chk:-ERROR verifica non eseguita (immagine senza f1tenth_bringup.launch_profile?)}"
    else
      kv profile_check "SKIP container fermo"
    fi
  else
    kv profile_present no
  fi
fi
