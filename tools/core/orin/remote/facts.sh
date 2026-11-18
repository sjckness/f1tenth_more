#!/bin/bash
# Gira SULL'ORIN (orin_script, da tools/core/lib/checks.sh orin_facts): stampa i fatti dell'Orin come
# righe chiave=valore, una per riga, valori su una riga. Solo letture: non cambia niente.
#   facts.sh CONTAINER UNIT PIN_FILE DEPLOYED_JSON DEPLOYED_FALLBACK BUILD_REPO NATIVE_WS OVERRIDE_DIR
set -euo pipefail
exp() { printf '%s' "${1/#\~/$HOME}"; }
CTR=$1 UNIT=$2
PIN=$(exp "$3") DJ=$(exp "$4") DJF=$(exp "$5") BUILD=$(exp "$6") NATIVE=$(exp "$7") OVR=$(exp "$8")
kv() { printf '%s=%s\n' "$1" "$(printf '%s' "${2:-}" | tr '\n\t' '  ' | sed 's/  */ /g; s/ $//')"; }
label() { docker inspect --type container -f "{{index .Config.Labels \"$1\"}}" "$CTR" 2>/dev/null || true; }

kv hostname "$(hostname)"
kv uptime_s "$(cut -d. -f1 /proc/uptime)"
kv unit_container "$(systemctl --user is-active "$UNIT" 2>/dev/null || true)"
kv unit_gateway "$(systemctl --user is-active f1tenth-gateway.service 2>/dev/null || true)"
kv unit_perf "$(systemctl is-active f1tenth-perf.service 2>/dev/null || true)"

if docker info >/dev/null 2>&1; then
  kv docker ok
  st=$(docker ps -a --filter "name=^${CTR}\$" --format '{{.State}}' | head -1)
  kv ctr_state "${st:-absent}"
  if [ -n "$st" ]; then
    kv ctr_health "$(docker inspect --type container -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CTR")"
    kv ctr_image "$(docker inspect --type container -f '{{.Config.Image}}' "$CTR")"
    kv ctr_image_id "$(docker inspect --type container -f '{{.Image}}' "$CTR")"
    kv ctr_started "$(docker inspect --type container -f '{{.State.StartedAt}}' "$CTR")"
    kv ctr_revision "$(label org.opencontainers.image.revision)"
    kv ctr_branch "$(label f1tenth.branch)"
    kv ctr_describe "$(label org.opencontainers.image.version)"
    kv ctr_tested "$(label f1tenth.tested)"
    kv ctr_role "$(docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "$CTR" | sed -n 's/^F1T_CONTAINER_ROLE=//p' | head -1)"
    kv ctr_autostart "$(docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "$CTR" | sed -n 's/^F1T_AUTOSTART=//p' | head -1)"
    if [ "$st" = running ]; then
      kv ctr_log_errors_10m "$(docker logs --since 10m "$CTR" 2>&1 | grep -c -i -E 'error|exception|traceback' || true)"
      kv ctr_stack_procs "$(docker exec "$CTR" sh -c 'ps -eo args' < /dev/null 2>/dev/null | grep -c -E 'supervisor_bringup|component_supervisor|stack_bringup' || true)"
      # Drift a mano: modifiche al filesystem del container sotto /opt (codice, install) e /etc,
      # escluse quelle del runtime nvidia, delle cache e dei percorsi di stato.
      changes=$(docker diff "$CTR" 2>/dev/null | awk '{print $2}' \
        | grep -E '^/(opt|etc)/' \
        | grep -v -E '__pycache__|^/etc/(localtime|vulkan|vulkansc|ld\.so\.cache|ld\.so\.conf\.d|nv_tegra_release|glvnd|nvidia|OpenCL)|^/opt/f1tenth_ws/(log|mission_videos)(/|$)' || true)
      kv ctr_fs_changes "$(printf '%s' "$changes" | grep -c . || true)"
      kv ctr_fs_changes_files "$(printf '%s\n' "$changes" | head -4 | tr '\n' ' ')"
    fi
  fi
  kv images_f1tenth "$(docker images --format '{{.Repository}}:{{.Tag}}' 'f1tenth' | head -8 | tr '\n' ' ')"
else
  kv docker "non risponde"
fi

if [ -f "$PIN" ]; then
  kv pin_tag "$(awk '$1=="tag"{print $2}' "$PIN")"
  kv pin_id "$(awk '$1=="id"{print $2}' "$PIN")"
  kv pin_at "$(awk '$1=="pinned"{print $2}' "$PIN")"
else
  kv pin_tag ""
fi
for f in "$DJ" "$DJF"; do
  if [ -f "$f" ]; then kv deployed_json_path "$f"; kv deployed_json "$(cat "$f")"; break; fi
done
kv etc_f1tenth_writable "$( [ -d "$(dirname "$DJ")" ] && [ -w "$(dirname "$DJ")" ] && echo yes || echo no)"

kv disk_root_free_gb "$(df -P -BG / | awk 'NR==2{sub("G","",$4);print $4}')"
kv disk_docker_free_gb "$(df -P -BG "$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)" 2>/dev/null | awk 'NR==2{sub("G","",$4);print $4}')"
kv mem_avail_mb "$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"
kv mem_total_mb "$(awk '/MemTotal/{print int($2/1024)}' /proc/meminfo)"
kv temp_max_c "$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1 | awk '{printf "%.1f", $1/1000}')"
kv nvpmodel "$(nvpmodel -q 2>/dev/null | sed -n 's/^NV Power Mode: //p' | head -1)"
kv cpu_max_mhz "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null | awk '{print int($1/1000)}')"
kv time_sync "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)"

for pair in "build_repo:$BUILD" "native:$NATIVE"; do
  k=${pair%%:*} d=${pair#*:}
  if git -C "$d" rev-parse --git-dir >/dev/null 2>&1; then
    kv "${k}_branch" "$(git -C "$d" rev-parse --abbrev-ref HEAD)"
    kv "${k}_head" "$(git -C "$d" rev-parse --short=12 HEAD)"
    kv "${k}_dirty" "$(git -C "$d" status --porcelain --untracked-files=no --ignore-submodules=all | wc -l)"
    kv "${k}_dirty_files" "$(git -C "$d" status --porcelain --untracked-files=no --ignore-submodules=all | awk '{print $2}' | head -6 | tr '\n' ' ')"
  else
    kv "${k}_branch" "n/d ($d)"
  fi
done

if [ -d "$OVR" ]; then
  kv override_files "$(ls -A "$OVR" | tr '\n' ' ')"
  if [ -f "$OVR/stack_args" ]; then kv override_stack_args "$(head -c 300 "$OVR/stack_args")"; fi
else
  kv override_files "n/d ($OVR assente)"
fi
