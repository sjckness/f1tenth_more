# shellcheck shell=bash
# tools/core/hil/lib/hil.sh - parti comuni di f1t hil precheck/run/down.
#   hil_parse_args "$@"        -disp NOME (obbligatorio), --scenario NOME, resto in HIL_REST[]
#   hil_load                   config + host + scenario, validati (exit 2 con l'uso se -disp manca)
#   hil_profile OUT SHM        rende tools/core/hil/dds/hil.xml.in (SHM 1 = lato stack)
#   hil_ros CMD...             CMD sull'host della sim con l'ambiente DDS dell'HIL (Jazzy sourced)
#   hil_probe ARGS...          ros_probe.py ARGS sull'host della sim
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

HIL_USAGE=${HIL_USAGE:-"f1t hil <comando> -disp <$(cfg_names "$F1T_CORE/hosts" | tr '\n' '|' | sed 's/|$//')> [--scenario NOME]"}
DISP='' SCENARIO='' HIL_REST=()
hil_parse_args() {
  while [ $# -gt 0 ]; do
    case "$1" in
      -disp|--disp) DISP=${2:-}; [ $# -ge 2 ] && shift ;;
      -disp=*|--disp=*) DISP=${1#*=} ;;
      --scenario) SCENARIO=${2:-}; [ $# -ge 2 ] && shift ;;
      *) HIL_REST+=("$1") ;;
    esac
    shift
  done
}

hil_load() {
  cfg_load
  if [ -z "$DISP" ]; then
    log_fail "-disp e' obbligatorio: indica la macchina che fa girare la sim"
    echo "uso: $HIL_USAGE" >&2
    exit 2
  fi
  cfg_load_host "$DISP" || { echo "uso: $HIL_USAGE" >&2; exit 2; }
  cfg_load_scenario "${SCENARIO:-smoke}" || exit 2
  local bad=0
  cfg_require config || bad=1
  cfg_require host || bad=1
  cfg_require scenario || bad=1
  [ "$bad" = 0 ] || log_die "configurazione incompleta (sopra le chiavi mancanti)" 1
  HIL_DS="$HOST_ORIN_LINK_IP:$HIL_DS_PORT"
  # shellcheck disable=SC2034  # usata da run.sh
  HIL_LOGROOT=$F1T_ROOT/$F1T_HIL_LOG_DIR
}

hil_profile() {
  local out=$1 shm=$2 ip
  ip=$( [ "$shm" = 1 ] && echo "$HOST_ORIN_LINK_IP" || echo "$HOST_LINK_IP")
  python3 - "$F1T_CORE/hil/dds/hil.xml.in" "$out" "$ip" "$shm" "$((HIL_SOCKET_BUFFER_MB * 1024 * 1024))" <<'EOF'
import sys
import xml.etree.ElementTree as ET
src, out, ip, shm, buf = sys.argv[1:6]
text = open(src).read()
for k, v in (('@ETH_IP@', ip), ('@SHM_TRANSPORT@', '<transport_id>f1t_shm</transport_id>' if shm == '1' else ''),
             ('@BUFFER_BYTES@', buf), ('@FLOW_BYTES@', str(800 * 1000000 // 8 // 100)), ('@ASYNC_WRITER@', '')):
    text = text.replace(k, v)
open(out, 'w').write(text)
ET.parse(out)
EOF
}

hil_ros() {
  local prof=${HIL_SIM_PROFILE:-}
  if [ -z "$prof" ]; then
    prof=$(mktemp --suffix=.xml); hil_profile "$prof" 0
  fi
  env ROS_DOMAIN_ID="$HIL_DOMAIN_ID" RMW_IMPLEMENTATION="$HIL_RMW" ROS_DISCOVERY_SERVER="$HIL_DS" \
    ROS_SUPER_CLIENT=TRUE FASTRTPS_DEFAULT_PROFILES_FILE="$prof" ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    bash -c 'set +u; source /opt/ros/jazzy/setup.bash; exec "$@"' _ "$@"
}

hil_probe() { hil_ros python3 "$F1T_CORE/hil/lib/ros_probe.py" "$@"; }
