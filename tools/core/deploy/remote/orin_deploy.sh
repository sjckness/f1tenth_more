#!/bin/bash
# Gira SULL'ORIN (orin_script da tools/core/deploy/deploy.sh). Usa gli strumenti dell'Orin
# (docker_orin pin/status, la unit systemd --user del container), non ne duplica la logica.
#   orin_deploy.sh image-info IMAGE                 righe chiave=valore (id, label) o exit 1
#   orin_deploy.sh switch IMAGE DOCKER_ORIN UNIT [STACK_MODE] [--allow-drift]
#                                                   docker_orin pin IMAGE + restart della unit; con
#                                                   STACK_MODE e la unit spenta: stack_mode container.
#                                                   --allow-drift (vuole STACK_MODE): pin + stop della
#                                                   unit + stack_mode container --allow-drift
#   orin_deploy.sh pin IMAGE DOCKER_ORIN            solo il pin (nessun restart)
#   orin_deploy.sh health CONTAINER DOCKER_ORIN TIMEOUT_S EXPECTED_ID
#   orin_deploy.sh write-json PATH FALLBACK         scrive stdin in PATH (o FALLBACK se PATH non e' scrivibile)
set -euo pipefail
exp() { printf '%s' "${1/#\~/$HOME}"; }
cmd=$1; shift
case "$cmd" in
  image-info)
    img=$1
    docker image inspect "$img" >/dev/null 2>&1 || { echo "exists=no"; exit 1; }
    lab() { docker image inspect -f "{{index .Config.Labels \"$1\"}}" "$img"; }
    echo "exists=yes"
    echo "id=$(docker image inspect -f '{{.Id}}' "$img")"
    echo "revision=$(lab org.opencontainers.image.revision)"
    echo "branch=$(lab f1tenth.branch)"
    echo "tested=$(lab f1tenth.tested)"
    echo "tested_at=$(lab f1tenth.tested_at)"
    echo "tests=$(lab f1tenth.tests | tr '\n' ' ')"
    echo "car_sync=$(lab f1tenth.car_sync.sha256)" ;;
  switch)
    img=$1 dorin=$(exp "$2") unit=$3 smode='' allow=0
    for a in "${@:4}"; do case "$a" in --allow-drift) allow=1 ;; *) smode=$a ;; esac; done
    "$dorin" pin "$img"
    if [ "$allow" = 1 ]; then
      [ -n "$smode" ] || { echo "--allow-drift vuole STACK_MODE" >&2; exit 2; }
      # restart della unit userebbe F1T_ALLOW_DRIFT=0: si ferma e la riavvia stack_mode, che scrive
      # F1T_ALLOW_DRIFT=1 solo per questo avvio.
      if [ "$(systemctl --user is-active "$unit" 2>/dev/null || true)" = active ]; then
        echo "stop $unit"
        systemctl --user stop "$unit" || echo "stop $unit: exit $? (ExecStop = docker_orin down)"
      fi
      # Un container avviato a mano (docker_orin up, senza la unit) resterebbe su con l'immagine
      # vecchia: stack_mode direbbe "already in container mode" e non applicherebbe il pin.
      if [ -n "$(docker ps -q --filter 'name=^f1tenth_car$')" ]; then
        echo "docker_orin down (container avviato fuori dalla unit)"
        "$dorin" down || echo "docker_orin down: exit $? (avanzi elencati sopra)"
      fi
      echo "stack_mode container --allow-drift"
      "$(exp "$smode")" container --allow-drift
    elif [ "$(systemctl --user is-active "$unit" 2>/dev/null || true)" = active ]; then
      echo "restart $unit"
      systemctl --user restart "$unit"
    elif [ -n "$smode" ]; then
      echo "stack_mode container"
      "$(exp "$smode")" container
    else
      echo "la unit $unit non e' attiva (l'Orin non e' in modalita' container)" >&2
      exit 3
    fi ;;
  pin) "$(exp "$2")" pin "$1" ;;
  health)
    ctr=$1 dorin=$(exp "$2") timeout=$3 want=$4
    t0=$(date +%s)
    while :; do
      rc=0; out=$("$dorin" status 2>&1) || rc=$?
      id=$(docker inspect --type container -f '{{.Image}}' "$ctr" 2>/dev/null || true)
      if [ "$rc" = 0 ] && [ "$id" = "$want" ]; then
        echo "sano in $(( $(date +%s) - t0 )) s: $(printf '%s\n' "$out" | grep '^container:' | cut -c1-120)"
        exit 0
      fi
      if [ $(( $(date +%s) - t0 )) -ge "$timeout" ]; then
        echo "NON sano dopo ${timeout} s (docker_orin status exit $rc, immagine ${id:7:12}, attesa ${want:7:12})"
        printf '%s\n' "$out" | sed 's/^/  /'
        docker logs --tail 20 "$ctr" 2>&1 | sed 's/^/  log: /'
        exit 1
      fi
      sleep 3
    done ;;
  write-json)
    p=$(exp "$1") f=$(exp "$2") tmp=$(mktemp)
    cat > "$tmp"
    for dst in "$p" "$f"; do
      if mkdir -p "$(dirname "$dst")" 2>/dev/null && [ -w "$(dirname "$dst")" ]; then
        mv -f "$tmp" "$dst"; chmod 644 "$dst"; echo "$dst"; exit 0
      fi
    done
    rm -f "$tmp"; echo "nessuna destinazione scrivibile ($p, $f)" >&2; exit 1 ;;
  *) echo "uso: orin_deploy.sh image-info|switch|pin|health|write-json ..." >&2; exit 2 ;;
esac
