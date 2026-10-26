#!/usr/bin/env bash
# Read-only hardware preflight for the on-car validation pass: is the ZED on
# USB3, is the lidar's Ethernet link up, is the VESC enumerated. No ROS, no
# container, no device opened -- safe to run in any mode, at any time.
#
#   scripts/orin/preflight.sh [--lidar-ip IP]
#
# ZED: STEREOLABS (vendor 2b03) must appear under a lsusb -t branch at
# 5000M or 10000M, not 480M (the USB2 fallback that silently breaks the SDK
# -- 2026-10-25 report.md). lidar: ping -c2 the Hokuyo's IP
# (src/f1tenth_bringup/config/sensors.yaml's ip_address, default
# 192.168.0.10; override with --lidar-ip or F1T_LIDAR_IP). VESC: a
# ChibiOS/STM virtual COM port under /dev/serial/by-id, resolved to its
# /dev/ttyACM*.
#
# Exit 0: all three present. Exit 1: at least one missing or wrong (printed).
# Battery voltage is NOT checked here -- it is only observable through
# vesc_driver_node, which is not running until the stack is live; see
# stack_live.sh's own post-start voltage line.
set -euo pipefail
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

LIDAR_IP=${F1T_LIDAR_IP:-192.168.0.10}
while [ $# -gt 0 ]; do
  case "$1" in
    --lidar-ip) [ $# -ge 2 ] || { echo "preflight: --lidar-ip needs a value" >&2; exit 2; }
                LIDAR_IP=$2; shift ;;
    *) echo "preflight: unknown option '$1'" >&2; exit 2 ;;
  esac
  shift
done

fail=0
pass() { printf 'PASS  %-10s %s\n' "$1" "$2"; }
bad()  { printf 'FAIL  %-10s %s\n' "$1" "$2"; fail=1; }

# ---- ZED on USB3 -------------------------------------------------------------
zed_line=$(lsusb -t 2>/dev/null | awk '
  /Bus [0-9]+\.Port 1:/ { root = $0; sub(/.*, /, "", root) }
  /2b03:f880|STEREOLABS ZED/ { print root, $0 }
' 2>/dev/null)
if ! lsusb 2>/dev/null | grep -qi "stereolabs zed"; then
  bad ZED "not enumerated at all (lsusb): unplugged, or the hub is unpowered"
else
  speed=$(lsusb -t 2>/dev/null | awk '
    /Port [0-9]+: Dev/ { line = $0 }
    /Class=Video/ && /uvcvideo/ { print line }
  ' | grep -oE '[0-9]+M' | head -1)
  case "$speed" in
    5000M|10000M) pass ZED "USB3 ($speed): $(lsusb | grep -i "stereolabs zed 2i")" ;;
    480M|12M)      bad ZED "on USB2 ($speed): re-seat the cable into a blue (USB3) port" ;;
    *)             bad ZED "enumerated but no Video-class child speed found (lsusb -t below)
$(lsusb -t)" ;;
  esac
fi

# ---- lidar: Ethernet link to the Hokuyo -------------------------------------
if ping -c2 -W1 "$LIDAR_IP" > /dev/null 2>&1; then
  pass lidar "$LIDAR_IP replies (sensors.yaml's urg_node ip_address)"
else
  bad lidar "$LIDAR_IP does not reply: check the eno1 link and the lidar's power"
fi

# ---- VESC: enumerated over USB -----------------------------------------------
vesc_link=$(find /dev/serial/by-id -maxdepth 1 \( -iname '*vesc*' -o -iname '*chibios*' \) 2>/dev/null | head -1)
if [ -n "$vesc_link" ] && [ -e "$vesc_link" ]; then
  pass VESC "$vesc_link -> $(readlink -f "$vesc_link")"
else
  bad VESC "no ChibiOS/STM virtual COM port under /dev/serial/by-id: check the USB cable"
fi

exit $fail
