#!/usr/bin/env bash
# car-container - il container dell'auto (f1tenth_car), SULL'ORIN, dalla shell dell'HOST.
# Installato da scripts/car/deploy.sh in ~/.local/bin/car-container (stesso commit dell'immagine).
#
#   car-container start [--allow-drift] [--archive separate|shared]   preflight, poi container su (sano)
#   car-container stop                                                 stack giu', container giu', lato nativo
#   car-container status                                               immagine fissata, container, modalita'
#
# start: car-preflight (ZED su USB3, link del lidar, VESC; se manca qualcosa si ferma), poi
# stack_mode container: ferma il lato nativo, avvia la unit f1tenth-car-container (docker_orin up con
# l'immagine fissata), aspetta il container sano (discovery server + gateway). Lo stack NON parte:
# car-stack start [PROFILO]. --allow-drift: parte anche se il checkout nativo ha modifiche che
# l'immagine non ha (vale per questo avvio).
# stop: car-stack stop se lo stack gira (SIGINT, come Ctrl-C), poi stack_mode native (container giu',
# gateway nativo su). Si torna sempre al nativo: il preflight qui non blocca.
# L'immagine si cambia da linus (scripts/car/deploy.sh), non qui.
# Exit: 0 fatto (status: container su e sano), 1 fallito, 2 argomenti, 3 lanciato dentro il container
# (status: container fermo), 4 status: container su ma non sano.
set -euo pipefail
CTR=${F1T_CAR_CONTAINER:-f1tenth_car}
BIN=${CAR_BIN:-$HOME/.local/bin}
SMODE=${CAR_STACK_MODE:-$BIN/stack_mode}
DO=${CAR_DOCKER_ORIN:-$BIN/docker_orin}
PREFLIGHT=${CAR_PREFLIGHT:-$BIN/car-preflight}
STACK=${CAR_STACK:-$BIN/car-stack}
CONTAINER_MARKERS=${F1T_CONTAINER_MARKERS:-/.dockerenv /opt/f1tenth_ws/SOURCES}

usage() { sed -n '2,/^set -euo/p' "$(readlink -f "$0")" | sed '$d; s/^# \{0,1\}//'; }
log() { printf '%s %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die() { log "ERRORE: $*"; exit "${2:-1}"; }
run() { log "\$ $*"; "$@"; }

cmd=${1:-}
[ $# -gt 0 ] && shift
case "$cmd" in
  -h|--help|help) usage; exit 0 ;;
  start) for a in "$@"; do case "$a" in --allow-drift|--archive|separate|shared) ;; *) die "opzione sconosciuta '$a' (car-container -h)" 2 ;; esac; done ;;
  stop|status) [ $# = 0 ] || die "$cmd non prende opzioni (car-container -h)" 2 ;;
  *) usage >&2; exit 2 ;;
esac
for m in $CONTAINER_MARKERS; do
  [ ! -e "$m" ] || die "sei DENTRO il container ($m esiste): esci con 'exit' e rilancia dalla shell dell'Orin" 3
done
for t in "$SMODE" "$DO"; do [ -x "$t" ] || die "manca $t (docker/orin/host/install.sh)"; done

case "$cmd" in
  status)
    log "modalita' auto: $(cat ~/.config/f1tenth/car_mode 2>/dev/null || echo '?')"
    "$DO" status ;;
  start)
    if [ -x "$PREFLIGHT" ]; then
      run "$PREFLIGHT" || die "preflight fallito (sopra): sensori mancanti, il container non parte"
    else
      log "ATTENZIONE: $PREFLIGHT assente (scripts/car/deploy.sh lo installa): preflight saltato"
    fi
    run "$SMODE" container "$@" || die "il container non e' partito (sopra). Diagnosi: $DO status; $DO history -n 5; $DO drift"
    "$DO" status || die "container su ma non sano ($DO logs -n 100)"
    log "container su. Lo stack: car-stack start [PROFILO] (car-stack profiles)" ;;
  stop)
    if [ -x "$STACK" ] && [ "$(docker inspect -f '{{.State.Running}}' "$CTR" 2>/dev/null)" = true ]; then
      run "$STACK" stop || log "ATTENZIONE: car-stack stop fallito, il container si ferma comunque (SIGTERM, poi il main del container ferma lo stack)"
    fi
    [ ! -x "$PREFLIGHT" ] || "$PREFLIGHT" || true     # solo per il log: non blocca il ritorno al nativo
    run "$SMODE" native || die "stack_mode native fallito (sopra): $SMODE status"
    "$SMODE" status || true ;;
esac
