#!/usr/bin/env bash
# wsl_dds_profile.sh -- render the WSL side of config/sim/dds/wsl.xml.in and print its path.
#
# For Hydra running in a WSL2 Ubuntu distro against Isaac Sim native on the same Windows PC.
# The Windows side is rendered by scripts/sim/win/hil_win.py --net wsl from the same template.
#
# Usage:
#   source scripts/hydra/hydra_env.sh
#   export FASTRTPS_DEFAULT_PROFILES_FILE="$(scripts/hydra/wsl_dds_profile.sh)"
#   export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   ros2 launch f1tenth_hydra hydra.launch.py hydra_profile:=workstation_full \
#     seg_source:=gt pose_source:=isaac_gt use_sim_time:=true
#
#   --out FILE    where to write it (default $XDG_RUNTIME_DIR or /tmp, f1tenth_wsl_dds.xml)
#   --buffer-mb N socket buffer size (default: link.socket_buffer_mb of hil.yaml, else 16)
#   --show        also print the rendered addresses to stderr
#   --tcp [PORT]  render wsl_tcp.xml.in instead: THIS side is the TCP SERVER and Isaac connects
#                 out to it. Use when the Windows firewall has no inbound rule for the WSL
#                 network: measured here, unsolicited UDP from WSL to Windows is dropped, while a
#                 connection Windows starts gets its answers back. Start this side FIRST, then
#                 Isaac with `hil_win.py sim --net wsltcp`. Default port 42100.
#
# Both addresses are read fresh on every call: a WSL restart changes them. Everything this script
# needs is inside WSL, so it never has to ask Windows anything.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
DDS_DIR="$REPO/src/f1tenth_bringup/config/sim/dds"
OUT="${XDG_RUNTIME_DIR:-/tmp}"
OUT="${OUT%/}/f1tenth_wsl_dds.xml"       # XDG_RUNTIME_DIR may already end in a slash
BUF_MB=""
SHOW=false
TCP=false
TCP_PORT=42100

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --buffer-mb) BUF_MB="$2"; shift 2 ;;
    --show) SHOW=true; shift ;;
    --tcp) TCP=true
           case "${2:-}" in [0-9]*) TCP_PORT="$2"; shift 2 ;; *) shift ;; esac ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
  esac
done

if $TCP; then
  TEMPLATE="$DDS_DIR/wsl_tcp.xml.in"
  OUT="${OUT%.xml}_tcp.xml"
else
  TEMPLATE="$DDS_DIR/wsl.xml.in"
fi
[ -f "$TEMPLATE" ] || { echo "no template: $TEMPLATE" >&2; exit 1; }

# Refuse to render a profile that cannot work, instead of producing one that silently matches
# nothing: a malformed or unroutable profile is the hardest HIL failure to read.
grep -qi microsoft /proc/version 2>/dev/null \
  || { echo "not running inside WSL (/proc/version): this profile is only for the WSL path" >&2; exit 1; }

OWN_IP="$(hostname -I | awk '{print $1}')"
PEER_IP="$(ip route show default | awk '{print $3; exit}')"
[ -n "$OWN_IP" ]  || { echo "could not read this distro's address (hostname -I)" >&2; exit 1; }
[ -n "$PEER_IP" ] || { echo "could not read the Windows address (ip route show default)" >&2; exit 1; }

if [ -z "$BUF_MB" ]; then
  BUF_MB="$(python3 - "$REPO" <<'PY' 2>/dev/null || echo 16
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(sys.argv[1]) / 'scripts' / 'sim'))
try:
    import hil_config
    v = hil_config._get(hil_config.load(), 'link.socket_buffer_mb')
    print(int(v) if v else 16)
except Exception:
    print(16)
PY
)"
fi
BUF_BYTES=$(( BUF_MB * 1024 * 1024 ))

# The OS limit caps what Fast DDS actually gets, silently. Warn rather than fail: a smaller
# buffer still works for light topics, and only the images suffer.
RMEM="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [ "$RMEM" -lt "$BUF_BYTES" ]; then
  echo "wsl_dds_profile.sh: net.core.rmem_max is $RMEM, below the ${BUF_MB} MB this profile asks" >&2
  echo "  for. Fast DDS will be capped and may drop image fragments in bursts. Fix (root, once):" >&2
  echo "  printf 'net.core.rmem_max = %s\\nnet.core.wmem_max = %s\\n' $BUF_BYTES $BUF_BYTES \\" >&2
  echo "    > /etc/sysctl.d/99-f1tenth-dds.conf && sysctl -p /etc/sysctl.d/99-f1tenth-dds.conf" >&2
fi

mkdir -p "$(dirname "$OUT")"
if $TCP; then
  # This side is the SERVER: it listens and advertises its locator. No initial peers: Isaac is
  # the one that connects, and it already knows this address from hil_win.py.
  LISTEN="      <listening_ports>
        <port>${TCP_PORT}</port>
      </listening_ports>"
  META="        <metatrafficUnicastLocatorList>
          <locator><tcpv4>
            <address>${OWN_IP}</address>
            <physical_port>${TCP_PORT}</physical_port>
            <port>${TCP_PORT}</port>
          </tcpv4></locator>
        </metatrafficUnicastLocatorList>"
  python3 - "$TEMPLATE" "$OUT" "$LISTEN" "$META" "$BUF_BYTES" "$OWN_IP" "$TCP_PORT" <<'PY'
import sys
tpl, out, listen, meta, buf, own, port = sys.argv[1:8]
text = open(tpl, encoding='utf-8').read()
for k, v in (('@LISTEN_BLOCK@', listen), ('@META_BLOCK@', meta), ('@PEER_BLOCK@', ''),
             ('@OWN_IP@', own), ('@TCP_PORT@', port), ('@BUFFER_BYTES@', buf)):
    text = text.replace(k, v)
open(out, 'w', encoding='utf-8').write(text)
PY
else
  sed -e "s|@OWN_IP@|$OWN_IP|g" \
      -e "s|@PEER_IP@|$PEER_IP|g" \
      -e "s|@BUFFER_BYTES@|$BUF_BYTES|g" \
      "$TEMPLATE" > "$OUT"
fi

# A malformed profile is IGNORED by Fast DDS without a word, which looks exactly like a network
# fault. Fail here instead (this is how the flow_controller_descriptors mistake stayed hidden).
python3 -c 'import sys, xml.etree.ElementTree as ET; ET.parse(sys.argv[1])' "$OUT"

$SHOW && echo "own $OWN_IP  peer $PEER_IP  buffer ${BUF_MB} MB  -> $OUT" >&2
echo "$OUT"
