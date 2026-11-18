#!/bin/bash
# Gira SULL'ORIN (orin_script da tools/core/build/build.sh): piccoli passi della build remota.
#   orin_build_ctl.sh has REPO SHA                  exit 0 se il commit e' nel repo di build
#   orin_build_ctl.sh fetch REPO BUNDLE SHORT       importa il bundle in refs/remotes/linus/f1t-SHORT
#   orin_build_ctl.sh pre REPO SHA                  condizioni della build (righe chiave=valore)
#   orin_build_ctl.sh start ROOT JOB ARGS...        lancia JOB staccato (setsid nohup), log ROOT/<sha7>.log
#   orin_build_ctl.sh status ROOT SHORT             "<stato> | <ultima riga del log>"
#   orin_build_ctl.sh log ROOT SHORT                il log intero
#   orin_build_ctl.sh stop-stack UNIT               ferma il container dell'auto (systemctl --user stop)
set -euo pipefail
cmd=$1; shift
exp() { printf '%s' "${1/#\~/$HOME}"; }
case "$cmd" in
  has) git -C "$(exp "$1")" cat-file -e "$2^{commit}" 2>/dev/null ;;
  fetch)
    repo=$(exp "$1") bundle=$2 short=$3
    git -C "$repo" bundle verify -q "$bundle" >/dev/null
    git -C "$repo" fetch -q "$bundle" "refs/f1t/build/$short:refs/remotes/linus/f1t-$short"
    case "$bundle" in /scratch/fabiocar/xfer/*.bundle) rm -f -- "$bundle" ;; esac
    echo "importato refs/remotes/linus/f1t-$short" ;;
  pre)
    repo=$(exp "$1") sha=$2
    echo "l4t=$(head -1 /etc/nv_tegra_release 2>/dev/null | cut -c1-40)"
    echo "build_sh=$(git -C "$repo" cat-file -e "$sha:docker/orin/build.sh" 2>/dev/null && echo yes || echo no)"
    echo "build_sh_dry=$(git -C "$repo" show "$sha:docker/orin/build.sh" 2>/dev/null | grep -c -- '--dry-run' || true)"
    echo "stack_procs=$(pgrep -c -x -f '.*(supervisor_bringup|component_supervisor_node|fast-discovery-server|fastdds discovery).*' 2>/dev/null || echo 0)"
    echo "car_container=$(docker ps --filter name=^f1tenth_car\$ --format '{{.State}}' | head -1)"
    echo "root_free_gb=$(df -P -BG / | awk 'NR==2{sub("G","",$4);print $4}')"
    echo "docker_free_gb=$(df -P -BG "$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)" | awk 'NR==2{sub("G","",$4);print $4}')"
    echo "inputs=$( [ -d /scratch/fabiocar/p5d_downloads ] && ls /scratch/fabiocar/p5d_downloads | wc -l || echo 0)"
    echo "running_builds=$(grep -l '^running' /scratch/fabiocar/f1t_build/*.status 2>/dev/null | wc -l)" ;;
  start)
    root=$1 job=$2; shift 2
    short=${1:0:7}
    mkdir -p "$root"
    setsid nohup bash "$job" "$@" > "$root/$short.log" 2>&1 < /dev/null &
    echo "avviato pid $! log $root/$short.log" ;;
  status)
    root=$1 short=$2
    st=$(cat "$root/$short.status" 2>/dev/null || echo none)
    echo "$st | $(tail -n 1 "$root/$short.log" 2>/dev/null | cut -c1-160)" ;;
  log) cat "$1/$2.log" ;;
  stop-stack) systemctl --user stop "$1"; echo "fermato $1" ;;
  *) echo "uso: orin_build_ctl.sh has|fetch|pre|start|status|log|stop-stack ..." >&2; exit 2 ;;
esac
