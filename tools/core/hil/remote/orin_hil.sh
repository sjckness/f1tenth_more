#!/bin/bash
# Gira SULL'ORIN (orin_script da tools/core/hil/*.sh): lato stack dell'HIL, in un container separato
# da quello dell'auto (stessa immagine), rete e IPC dell'host (l'Orin non ha bridge Docker).
#   orin_hil.sh facts CTR PORT SIM_IP          righe chiave=valore: rotta, MTU, rmem, porta DS, container
#   orin_hil.sh image PIN_FILE [IMAGE]         l'immagine da usare (IMAGE, o il tag fissato) e il suo id
#   orin_hil.sh up CTR IMAGE                   crea il container HIL (sleep infinity) se non c'e'
#   orin_hil.sh put CTR DEST                   copia stdin in CTR:DEST
#   orin_hil.sh ds CTR IP PORT                 Discovery Server nel container (ensure_discovery_server.py)
#   orin_hil.sh stack CTR DOMAIN DS_IP PORT ARGS...   supervisor_bringup sim:=isaac, staccato
#   orin_hil.sh exec CTR DOMAIN DS_IP PORT CMD...     comando nel container con l'ambiente DDS dell'HIL
#   orin_hil.sh probe IMAGE XFER IP PORT DOMAIN TOPIC TIMEOUT
#                                              DS + sottoscrittore in un container usa e getta (precheck)
#   orin_hil.sh logs CTR                       log dello stack e del DS
#   orin_hil.sh down CTR                       SIGINT allo stack, stop e rm del container, verifica
set -euo pipefail
cmd=$1; shift
PROFILE=/tmp/f1tenth_hil_profile.xml
DS_PY=/opt/f1tenth_ws/install/share/f1tenth_bringup/scripts/ensure_discovery_server.py
guard() { [ "$1" != f1tenth_car ] || { echo "rifiuto: $1 e' il container dell'auto" >&2; exit 1; }; }
state() { docker ps -a --filter "name=^$1\$" --format '{{.State}}' | head -1; }
denv() { printf '%s\n' -e "ROS_DOMAIN_ID=$1" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e "ROS_DISCOVERY_SERVER=$2:$3" \
  -e "FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE" -e ROS_SUPER_CLIENT=TRUE; }
case "$cmd" in
  facts)
    ctr=$1 port=$2 simip=$3
    r=$(ip route get "$simip" 2>/dev/null | head -1 || true)
    echo "route=$r"
    dev=$(sed -n 's/.* dev \([^ ]*\).*/\1/p' <<< "$r")
    echo "route_direct=$(case "$r" in *" via "*|'') echo no ;; *) echo yes ;; esac)"
    echo "mtu=$( [ -n "$dev" ] && cat "/sys/class/net/$dev/mtu" 2>/dev/null || echo '?')"
    echo "rmem_max=$(cat /proc/sys/net/core/rmem_max)"
    echo "ds_port_busy=$(ss -H -lun "sport = :$port" 2>/dev/null | wc -l)"
    echo "hil_ctr=$(state "$ctr")" ;;
  image)
    pin=${1/#\~/$HOME} img=${2:-}
    [ -n "$img" ] || img=$(awk '$1=="tag"{print $2}' "$pin" 2>/dev/null || true)
    [ -n "$img" ] || { echo "nessuna immagine (pin vuoto: $pin)" >&2; exit 1; }
    echo "image=$img"
    echo "id=$(docker image inspect -f '{{.Id}}' "$img")" ;;
  up)
    ctr=$1 img=$2; guard "$ctr"
    case "$(state "$ctr")" in
      running) echo "container $ctr gia' in esecuzione"; exit 0 ;;
      '') ;;
      *) docker rm -f "$ctr" >/dev/null ;;
    esac
    docker run -d --name "$ctr" --network host --ipc host --runtime nvidia --init --restart no \
      -e F1T_CONTAINER_ROLE=hil -e F1T_AUTOSTART=none \
      --entrypoint /opt/f1tenth_ws/entrypoint.sh "$img" sleep infinity > /dev/null
    [ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$ctr")" = host ] || { echo "il container non e' sulla rete host" >&2; exit 1; }
    echo "container $ctr avviato da $img" ;;
  put)
    ctr=$1 dest=$2 tmp=$(mktemp)
    cat > "$tmp"; docker cp "$tmp" "$ctr:$dest" >/dev/null; rm -f "$tmp"; echo "copiato $dest" ;;
  ds)
    ctr=$1 ip=$2 port=$3; guard "$ctr"
    docker exec -d "$ctr" bash -lc "exec python3 $DS_PY $ip $port > /tmp/hil_ds.log 2>&1"
    for _ in $(seq 1 20); do
      [ "$(ss -H -lun "sport = :$port" 2>/dev/null | wc -l)" -gt 0 ] && { echo "Discovery Server su $ip:$port"; exit 0; }
      sleep 0.5
    done
    echo "il Discovery Server non ascolta su $port dopo 10 s" >&2; docker exec "$ctr" cat /tmp/hil_ds.log < /dev/null >&2 || true; exit 1 ;;
  stack)
    ctr=$1 dom=$2 dsip=$3 port=$4; shift 4; guard "$ctr"
    mapfile -t E < <(denv "$dom" "$dsip" "$port")
    q=$(printf '%q ' ros2 launch f1tenth_bringup supervisor_bringup.launch.py \
      "discovery_server_address:=$dsip" "discovery_server_port:=$port" "fastdds_profile:=$PROFILE" "$@")
    docker exec -d "${E[@]}" "$ctr" bash -lc "exec $q > /tmp/hil_stack.log 2>&1"
    echo "stack avviato in $ctr: $q" ;;
  exec)
    ctr=$1 dom=$2 dsip=$3 port=$4; shift 4
    mapfile -t E < <(denv "$dom" "$dsip" "$port")
    docker exec "${E[@]}" "$ctr" bash -lc "$(printf '%q ' "$@")" < /dev/null ;;
  probe)
    img=$1 xfer=$2 ip=$3 port=$4 dom=$5 topic=$6 to=$7
    docker run --rm --network host --entrypoint /opt/f1tenth_ws/entrypoint.sh \
      -v "$xfer/ros_probe.py:/tmp/ros_probe.py:ro" \
      -e "ROS_DOMAIN_ID=$dom" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e "ROS_DISCOVERY_SERVER=$ip:$port" \
      "$img" bash -c "python3 $DS_PY $ip $port > /tmp/ds.log 2>&1 & sleep 2; python3 /tmp/ros_probe.py sub $topic --timeout $to" < /dev/null ;;
  logs)
    ctr=$1
    for f in /tmp/hil_ds.log /tmp/hil_stack.log; do
      echo "===== $f"; docker exec "$ctr" cat "$f" < /dev/null 2>/dev/null || echo "(assente)"
    done ;;
  down)
    ctr=$1; guard "$ctr"
    st=$(state "$ctr")
    if [ "$st" = running ]; then
      docker exec "$ctr" pkill -INT -f 'ros2 launch' < /dev/null 2>/dev/null || true
      for _ in $(seq 1 30); do
        n=$(docker exec "$ctr" sh -c 'ps -eo args' < /dev/null 2>/dev/null | grep -c -E 'ros2 launch|component_supervisor|_node' || true)
        [ "$n" = 0 ] && break
        sleep 1
      done
      docker stop -t 30 "$ctr" > /dev/null
    fi
    [ -z "$st" ] || docker rm -f "$ctr" > /dev/null 2>&1 || true
    [ -z "$(state "$ctr")" ] || { echo "il container $ctr c'e' ancora" >&2; exit 1; }
    echo "container $ctr rimosso (era: ${st:-assente})" ;;
  *) echo "uso: orin_hil.sh facts|image|up|put|ds|stack|exec|probe|logs|down ..." >&2; exit 2 ;;
esac
