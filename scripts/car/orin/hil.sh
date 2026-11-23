#!/usr/bin/env bash
# preflight: ok/ko/wa always return 0, so `cond && ok .. || ko ..` there is a real if-then-else.
# shellcheck disable=SC2015
# car-hil - the Orin side of the office HIL batch, from the shell of the HOST (not in a container).
# Installed by scripts/car/deploy.sh in ~/.local/bin/car-hil, together with car-hil-agent (the
# ROS side, scripts/car/orin/hil_agent.py), from the same commit. The workstation drives it over
# ssh (scripts/sim/win/hil_office_batch.py); nothing here starts Isaac.
#
#   car-hil preflight --orin-ip IP --ws-ip IP --port P [--min-free-gb N]
#   car-hil up SID --orin-ip IP --port P --domain D [--buffer-mb N] [--image IMG]
#   car-hil put SID NAME                          stdin -> the session's file NAME
#   car-hil wait-sim SID [--timeout S]            Isaac's topics seen from the container
#   car-hil stack-up SID MODE TAG [--timeout S] [k:=v ...]   launch the stack, wait READY
#   car-hil stack-down SID                        stop the stack (the container stays)
#   car-hil run SID RUN_ID MODE REP POSE_IDX [--timeout S]  one repetition (car-hil-agent run)
#   car-hil mark SID RUN_ID MODE REP POSE_IDX STATUS [NOTE] a repetition that could not run
#   car-hil done SID                              run ids that already have a result
#   car-hil analyze SID                           the session's analysis (car-hil-agent analyze)
#   car-hil down SID                              stack down, container stopped and removed
#   car-hil status                                container, stack, sessions
#
# SID is a session id ([A-Za-z0-9_.-]); its directory is ~/f1tenth_hil_batch/SID, mounted at the
# same path in the container: session.json, mission.json, map_geo.json (from the workstation),
# hil.env (written by `up`), versions.json, logs/, runs/RUN_ID/, analysis/.
#
# The container: f1tenth_hil, from the image f1t deploy pinned (~/.config/f1tenth/car_image) or
# --image; host network and IPC (the Orin has no bridge, CLAUDE.md), no devices, `sleep infinity`.
# It is never the car's container (f1tenth_car): the car's stack must be stopped (preflight), and
# with sim:=isaac the stack starts no device driver, so nothing here can move the car.
# The stack: supervisor_bringup.launch.py sim:=isaac use_sim_time:=true on the Discovery Server
# this script holds in the container for the whole session (ORIN_IP:PORT), with the HIL Fast DDS
# profile car-hil-agent renders (5 s lease, UDP bound to the link + loopback, SHM).
# Exit: 0 ok, 1 failed, 2 arguments.
set -euo pipefail
CTR=${F1T_HIL_CONTAINER:-f1tenth_hil}
ROOT=${F1T_HIL_ROOT:-$HOME/f1tenth_hil_batch}
PIN_FILE=${F1T_ORIN_PIN_FILE:-$HOME/.config/f1tenth/car_image}
AGENT_HOST=${F1T_HIL_AGENT:-$HOME/.local/bin/car-hil-agent}
AGENT=/tmp/car-hil-agent.py
PROFILE=/tmp/f1tenth_hil_profile.xml
DS_PY=/opt/f1tenth_ws/install/share/f1tenth_bringup/scripts/ensure_discovery_server.py
CAR_CTR=${F1T_CAR_CONTAINER:-f1tenth_car}

usage() { sed -n '2,/^set -euo/p' "$(readlink -f "$0")" | sed '$d; s/^# \{0,1\}//'; }
log() { printf '[car-hil %s] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die() { log "ERRORE: $*"; exit "${2:-1}"; }
[ "$CTR" != "$CAR_CTR" ] || die "il container HIL non puo' essere quello dell'auto ($CAR_CTR)" 2
[ ! -e /.dockerenv ] || die "car-hil gira sull'host dell'Orin, non in un container" 2

state() { docker ps -a --filter "name=^$1\$" --format '{{.State}}' | head -1; }
sdir() {
  [[ ${1:-} =~ ^[A-Za-z0-9_.-]+$ ]] || die "session id non valido: '${1:-}'" 2
  echo "$ROOT/$1"
}
load_env() {   # SID: ORIN_IP PORT DOMAIN IMAGE BUFFER_MB from hil.env (written by up)
  local d; d=$(sdir "$1")
  [ -r "$d/hil.env" ] || die "sessione $1 senza hil.env: car-hil up $1 ... prima"
  # shellcheck disable=SC1091
  source "$d/hil.env"
  [ "$(state "$CTR")" = running ] || die "il container $CTR non gira (car-hil up $1)"
  [ "$(docker inspect -f '{{index .Config.Labels "f1t.hil.session"}}' "$CTR")" = "$1" ] \
    || die "il container $CTR e' di un'altra sessione ($(docker inspect -f '{{index .Config.Labels "f1t.hil.session"}}' "$CTR"))"
}
# cexec CMD...: CMD in the container with the HIL DDS environment. jazzy.sh is sourced FIRST and
# then overridden: it sets FASTRTPS_DEFAULT_PROFILES_FILE to the car's profile unconditionally.
cexec() {
  docker exec -e "ROS_DOMAIN_ID=$DOMAIN" -e "F1TENTH_DISCOVERY_SERVER=$ORIN_IP:$PORT" "$CTR" bash -c \
    "source /opt/f1tenth_ws/scripts/env/jazzy.sh >/dev/null; export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE \
ROS_DISCOVERY_SERVER=$ORIN_IP:$PORT ROS_SUPER_CLIENT=TRUE ROS_DOMAIN_ID=$DOMAIN; unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; \
exec $(printf '%q ' "$@")" < /dev/null
}
stack_pids() {   # the stack's processes in the container (not the DS, not sleep, not us)
  docker exec "$CTR" ps -eo pid=,args= < /dev/null 2>/dev/null \
    | awk '/supervisor_bringup|component_supervisor|__node:=|ros2 launch|llama-server/ && !/ensure_discovery_server|fastdds discovery|ps -eo/ {print $1}'
}
car_stack_running() {   # the car's own stack in f1tenth_car
  [ "$(state "$CAR_CTR")" = running ] || return 1
  docker exec "$CAR_CTR" ps -eo args= < /dev/null 2>/dev/null | grep -q -E 'supervisor_bringup|component_supervisor'
}

cmd=${1:-}; [ $# -gt 0 ] && shift
case "$cmd" in
  -h|--help|help|'') usage; exit 0 ;;

  preflight)
    ORIN_IP='' WS_IP='' PORT='' MINGB=8
    while [ $# -gt 0 ]; do
      case "$1" in
        --orin-ip) ORIN_IP=$2; shift ;; --ws-ip) WS_IP=$2; shift ;; --port) PORT=$2; shift ;;
        --min-free-gb) MINGB=$2; shift ;; *) die "opzione sconosciuta: $1" 2 ;;
      esac; shift
    done
    [ -n "$ORIN_IP" ] && [ -n "$WS_IP" ] && [ -n "$PORT" ] || die "servono --orin-ip --ws-ip --port" 2
    bad=0
    ok() { printf 'OK    %s\n' "$*"; }
    ko() { printf 'FAIL  %s\n' "$*"; bad=1; }
    wa() { printf 'WARN  %s\n' "$*"; }
    docker info >/dev/null 2>&1 && ok "docker" || ko "docker non risponde"
    img=$(awk '$1=="tag"{print $2}' "$PIN_FILE" 2>/dev/null || true)
    if [ -n "$img" ] && docker image inspect "$img" >/dev/null 2>&1; then
      ok "immagine fissata $img ($(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$img" | cut -c1-12))"
    else ko "nessuna immagine fissata in $PIN_FILE (scripts/car/deploy.sh da linus)"; fi
    if car_stack_running; then ko "lo stack dell'auto gira in $CAR_CTR: car-stack stop"
    else ok "stack dell'auto fermo"; fi
    # ip -o -4 addr: no grep -q in a pipe (pipefail + SIGPIPE)
    addrs=$(ip -o -4 addr show)
    if [[ $addrs == *" inet $ORIN_IP/"* ]]; then ok "$ORIN_IP su un'interfaccia"
    else ko "$ORIN_IP non e' su nessuna interfaccia (il cavo verso la workstation)"; fi
    r=$(ip -o -4 route get "$WS_IP" 2>&1 || true)
    case "$r" in *" via "*|*nreachable*|'') ko "rotta verso $WS_IP non diretta: $r" ;; *) ok "rotta diretta verso $WS_IP" ;; esac
    if ping -c1 -W2 "$WS_IP" >/dev/null 2>&1; then ok "la workstation risponde al ping"
    else wa "la workstation non risponde al ping (Windows lo blocca: conta il resto)"; fi
    want=$((16 * 1024 * 1024)); have=$(cat /proc/sys/net/core/rmem_max)
    # WARN, not FAIL: the batch's door missions run on the lidar (a 4 kB scan fits any buffer); the
    # 0.92 MB camera frames are what a small buffer drops, i.e. YOLO starves. Fix once per boot.
    [ "$have" -ge "$want" ] && ok "rmem_max $have" || wa "net.core.rmem_max $have < $want: le immagini della camera si perdono a raffiche; sudo sysctl -w net.core.rmem_max=$want net.core.wmem_max=$want"
    if [ "$(ss -H -lun "sport = :$PORT" 2>/dev/null | wc -l)" -gt 0 ]; then
      if [ "$(state "$CTR")" = running ]; then wa "porta $PORT occupata dal container $CTR di una sessione precedente (up lo rimpiazza)"
      else ko "porta UDP $PORT gia' occupata da un altro processo"; fi
    else ok "porta UDP $PORT libera"; fi
    free=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
    [ "$free" -ge "$MINGB" ] && ok "disco: ${free} GB liberi" || ko "disco: ${free} GB liberi < ${MINGB}"
    for c in car-hil car-hil-agent; do
      [ -r "$HOME/.local/bin/$c" ] && ok "$c $(sha256sum "$HOME/.local/bin/$c" | cut -c1-12)" || ko "$c assente"
    done
    old=$(docker ps --format '{{.Names}}' | grep -E '^f1tenth_hil' || true)
    [ -z "$old" ] || wa "container HIL gia' attivi: ${old//$'\n'/ } (up rimpiazza $CTR)"
    exit "$bad" ;;

  up)
    sid=${1:?SID}; shift; d=$(sdir "$sid")
    ORIN_IP='' PORT='' DOMAIN='' BUFFER_MB=16 IMAGE=''
    while [ $# -gt 0 ]; do
      case "$1" in
        --orin-ip) ORIN_IP=$2; shift ;; --port) PORT=$2; shift ;; --domain) DOMAIN=$2; shift ;;
        --buffer-mb) BUFFER_MB=$2; shift ;; --image) IMAGE=$2; shift ;; *) die "opzione sconosciuta: $1" 2 ;;
      esac; shift
    done
    [ -n "$ORIN_IP" ] && [ -n "$PORT" ] && [ -n "$DOMAIN" ] || die "servono --orin-ip --port --domain" 2
    [ -n "$IMAGE" ] || IMAGE=$(awk '$1=="tag"{print $2}' "$PIN_FILE" 2>/dev/null || true)
    [ -n "$IMAGE" ] || die "nessuna immagine: niente in $PIN_FILE e nessun --image"
    docker image inspect "$IMAGE" >/dev/null 2>&1 || die "immagine $IMAGE assente"
    [ -r "$AGENT_HOST" ] || die "$AGENT_HOST assente (scripts/car/deploy.sh da linus)"
    car_stack_running && die "lo stack dell'auto gira in $CAR_CTR: car-stack stop"
    mkdir -p "$d/logs" "$d/runs"
    if [ -n "$(state "$CTR")" ]; then
      log "rimuovo il container $CTR esistente ($(docker inspect -f '{{index .Config.Labels "f1t.hil.session"}}' "$CTR" 2>/dev/null || echo ?))"
      docker exec "$CTR" pkill -INT -f supervisor_bringup < /dev/null 2>/dev/null || true
      sleep 3
      docker rm -f "$CTR" >/dev/null
      sleep 6     # past the 5 s lease of whatever it had on the Discovery Server
    fi
    docker run -d --name "$CTR" --network host --ipc host --runtime nvidia --init --restart no \
      -l "f1t.hil.session=$sid" -e F1T_CONTAINER_ROLE=hil -e F1T_AUTOSTART=none \
      -v "$ROOT:$ROOT" --entrypoint /opt/f1tenth_ws/entrypoint.sh "$IMAGE" sleep infinity >/dev/null
    [ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$CTR")" = host ] || die "il container non e' sulla rete host"
    docker cp "$AGENT_HOST" "$CTR:$AGENT" >/dev/null
    { echo "ORIN_IP=$ORIN_IP"; echo "PORT=$PORT"; echo "DOMAIN=$DOMAIN"; echo "IMAGE=$IMAGE"
      echo "BUFFER_MB=$BUFFER_MB"; echo "IMAGE_ID=$(docker image inspect -f '{{.Id}}' "$IMAGE")"
      echo "AGENT_SHA=$(sha256sum "$AGENT_HOST" | cut -c1-64)"; echo "HIL_SHA=$(sha256sum "$(readlink -f "$0")" | cut -c1-64)"
    } > "$d/hil.env"
    load_env "$sid"
    docker exec "$CTR" python3 "$AGENT" profile --ip "$ORIN_IP" --buffer-mb "$BUFFER_MB" --out "$PROFILE" < /dev/null
    cexec python3 "$AGENT" versions --out "$d/versions.json" >/dev/null 2>&1 || log "versions.json non scritto"
    docker exec -d "$CTR" bash -c "source /opt/f1tenth_ws/scripts/env/jazzy.sh >/dev/null; exec python3 $DS_PY $ORIN_IP $PORT > $d/logs/discovery_server.log 2>&1"
    for _ in $(seq 1 30); do
      [ "$(ss -H -lun "sport = :$PORT" 2>/dev/null | wc -l)" -gt 0 ] && { log "container $CTR da $IMAGE, Discovery Server $ORIN_IP:$PORT, sessione $d"; exit 0; }
      sleep 0.5
    done
    die "il Discovery Server non ascolta su $PORT dopo 15 s ($d/logs/discovery_server.log)" ;;

  put)
    sid=${1:?SID}; name=${2:?NAME}; d=$(sdir "$sid")
    [[ $name =~ ^[A-Za-z0-9_.-]+$ ]] || die "nome non valido: $name" 2
    mkdir -p "$d"; cat > "$d/$name.tmp"; mv "$d/$name.tmp" "$d/$name"; log "scritto $d/$name" ;;

  wait-sim)
    sid=${1:?SID}; shift; load_env "$sid"; to=240
    [ "${1:-}" = --timeout ] && to=$2
    cexec python3 "$AGENT" wait --timeout "$to" /clock:20 /scan:10 /odom:20 /sim/ground_truth/odom:20 ;;

  stack-up)
    sid=${1:?SID}; mode=${2:?MODE}; tag=${3:?TAG}; shift 3; load_env "$sid"; d=$(sdir "$sid"); to=240
    case "$mode" in corridor_mpc) node=mpc_corr ;; corridor_rrt) node=corridor_rrt ;; p2p_geometric) node=p2p_geometric ;;
      *) die "modo sconosciuto: $mode" 2 ;; esac
    [[ $tag =~ ^[A-Za-z0-9_.-]+$ ]] || die "tag non valido: $tag" 2
    extra=()
    while [ $# -gt 0 ]; do
      case "$1" in --timeout) to=$2; shift ;; *:=*) extra+=("$1") ;; *) die "argomento sconosciuto: $1" 2 ;; esac; shift
    done
    [ -z "$(stack_pids)" ] || die "uno stack gira gia' nel container (car-hil stack-down $sid)"
    launch=(ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac use_sim_time:=true
            "discovery_server_address:=$ORIN_IP" "discovery_server_port:=$PORT" "fastdds_profile:=$PROFILE"
            "planner_controller_mode:=$mode" "${extra[@]}")
    logf="$d/logs/stack_$tag.log"
    echo "# $(date -Is) ${launch[*]}" > "$logf"
    docker exec -d -e "ROS_DOMAIN_ID=$DOMAIN" -e "F1TENTH_DISCOVERY_SERVER=$ORIN_IP:$PORT" "$CTR" bash -c \
      "source /opt/f1tenth_ws/scripts/env/jazzy.sh >/dev/null; export ROS_DOMAIN_ID=$DOMAIN; exec $(printf '%q ' "${launch[@]}") >> $logf 2>&1"
    log "stack $mode avviato ($logf), attendo READY (max $to s)"
    cexec python3 "$AGENT" ready --node "$node" --timeout "$to" ;;

  stack-down)
    sid=${1:?SID}; load_env "$sid"
    [ -n "$(stack_pids)" ] || { log "nessuno stack nel container"; exit 0; }
    docker exec "$CTR" pkill -INT -f supervisor_bringup.launch.py < /dev/null || true
    for i in $(seq 1 80); do
      [ -z "$(stack_pids)" ] && break
      [ "$i" = 60 ] && { log "stack ancora vivo dopo 30 s: SIGTERM"; docker exec "$CTR" pkill -TERM -f '__node:=|component_supervisor|supervisor_bringup' < /dev/null || true; }
      sleep 0.5
    done
    if [ -n "$(stack_pids)" ]; then
      log "stack ancora vivo dopo SIGTERM: SIGKILL"
      docker exec "$CTR" pkill -KILL -f '__node:=|component_supervisor|supervisor_bringup|llama-server' < /dev/null || true
      sleep 1
    fi
    [ -z "$(stack_pids)" ] || die "processi dello stack ancora vivi: $(stack_pids | tr '\n' ' ')"
    sleep 7     # the 5 s participant lease: the next bringup must not match this one's endpoints
    log "stack fermo" ;;

  run)
    sid=${1:?SID}; rid=${2:?RUN_ID}; mode=${3:?MODE}; rep=${4:?REP}; pidx=${5:?POSE_IDX}; shift 5
    load_env "$sid"; d=$(sdir "$sid"); to=180
    [ "${1:-}" = --timeout ] && to=$2
    [[ $rid =~ ^[A-Za-z0-9_.-]+$ ]] || die "run id non valido: $rid" 2
    [ -r "$d/mission.json" ] || die "$d/mission.json assente (car-hil put $sid mission.json)"
    cexec python3 "$AGENT" run --session "$d" --run-id "$rid" --mode "$mode" --rep "$rep" --pose-idx "$pidx" \
      --mission "$d/mission.json" --timeout "$to" 2>&1 | tee -a "$d/logs/runs.log" ;;

  mark)
    sid=${1:?SID}; rid=${2:?RUN_ID}; mode=${3:?MODE}; rep=${4:?REP}; pidx=${5:?POSE_IDX}; st=${6:?STATUS}; note=${7:-}
    d=$(sdir "$sid")
    python3 "$AGENT_HOST" mark --session "$d" --run-id "$rid" --mode "$mode" --rep "$rep" --pose-idx "$pidx" \
      --status "$st" --note "$note" 2>&1 | tee -a "$d/logs/runs.log" ;;

  done)
    d=$(sdir "${1:?SID}")
    for f in "$d"/runs/*/result.json; do [ -e "$f" ] && basename "$(dirname "$f")"; done; true ;;

  analyze)
    sid=${1:?SID}; d=$(sdir "$sid")
    if [ "$(state "$CTR")" = running ]; then
      docker exec "$CTR" python3 "$AGENT" analyze --session "$d" < /dev/null
    else
      img=$(awk '$1=="tag"{print $2}' "$PIN_FILE")
      docker run --rm --network host -v "$ROOT:$ROOT" -v "$AGENT_HOST:$AGENT:ro" --entrypoint /opt/f1tenth_ws/entrypoint.sh \
        "$img" python3 "$AGENT" analyze --session "$d" < /dev/null
    fi ;;

  down)
    sid=${1:?SID}
    if [ "$(state "$CTR")" = running ]; then
      if [ -n "$(stack_pids)" ]; then "$0" stack-down "$sid" || true; fi
    fi
    [ -z "$(state "$CTR")" ] || docker rm -f "$CTR" >/dev/null
    [ -z "$(state "$CTR")" ] || die "il container $CTR c'e' ancora"
    log "container $CTR rimosso" ;;

  status)
    st=$(state "$CTR")
    echo "container   $CTR: ${st:-assente}$( [ -n "$st" ] && echo " (sessione $(docker inspect -f '{{index .Config.Labels "f1t.hil.session"}}' "$CTR"), $(docker inspect -f '{{.Config.Image}}' "$CTR"))")"
    if [ "$st" = running ]; then n=$(stack_pids | wc -l); echo "stack       $([ "$n" -gt 0 ] && echo "su ($n processi)" || echo fermo)"; fi
    echo "auto        $(car_stack_running && echo 'stack dell'"'"'auto ATTIVO' || echo 'stack dell'"'"'auto fermo')"
    echo "sessioni    $ROOT"
    for s in "$ROOT"/*/; do
      [ -d "$s" ] || continue
      n=$(find "$s/runs" -name result.json 2>/dev/null | wc -l)
      printf '  %-28s %4s run  %s\n' "$(basename "$s")" "$n" "$( [ -f "$s/analysis/report.md" ] && echo 'analisi presente')"
    done ;;

  *) die "comando sconosciuto: $cmd (car-hil -h)" 2 ;;
esac
