#!/usr/bin/env bash
# The car's devices, as the host has them and as the container user sees them.
#
# Runs on: the Orin host, with f1tenth_car up (car or harness role). Plan P8
# "devices (VESC /dev/ttyACM0 + /dev/sensors/vesc, joystick, ZED
# hidraw/video/usb, lidar /scan rate)"; run it again after the ZED unplug /
# replug with the container up ([U-car]): the replugged nodes must appear in
# the container without a restart (cgroup rules + /dev bind).
#
#   devices_check.sh OUT_DIR [--open] [--scan]
#
# For every device node of interest present on the HOST (/dev/ttyACM*,
# /dev/ttyUSB*, /dev/sensors/vesc, /dev/input/js*, /dev/input/event*,
# /dev/hidraw*, /dev/video*, /dev/bus/usb/*/*): present in the container and
# readable + writable by its user -> PASS; missing or no access -> FAIL; a
# class absent on the host -> INFO (not plugged).
#   --open  also open each serial port O_RDWR|O_NONBLOCK and close it at once
#           (only with the stack DOWN: the VESC driver holds the port)
#   --scan  /scan rate in the container (stack UP): > 30 Hz expected (the UST
#           lidar publishes 40 Hz)
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
mkdir -p "$1" || exit 2
OUT=$(parity_abs "$1")
shift
OPEN=0 SCAN=0
for a in "$@"; do
  case "$a" in --open) OPEN=1 ;; --scan) SCAN=1 ;; *) echo "unknown argument $a" >&2; exit 2 ;; esac
done
[ "$(container_state)" = running ] || { echo "$CONTAINER is not running" >&2; exit 2; }
host_context "$OUT" host
CLASSES="ttyACM:/dev/ttyACM* ttyUSB:/dev/ttyUSB* vesc_link:/dev/sensors/vesc js:/dev/input/js* \
event:/dev/input/event* hidraw:/dev/hidraw* video:/dev/video* usb:/dev/bus/usb/*/*"
for entry in $CLASSES; do
  cls=${entry%%:*}
  pattern=${entry#*:}
  # shellcheck disable=SC2086  # the pattern is a glob on purpose
  devs=$(ls -d $pattern 2>/dev/null)
  if [ -z "$devs" ]; then
    record "$OUT" "device $cls" INFO "none on the host"
    continue
  fi
  bad=""
  n=0
  for d in $devs; do
    n=$((n + 1))
    real=$(readlink -f "$d")
    printf '%s -> %s %s\n' "$d" "$real" "$(stat -c '%t:%T %U:%G %a' "$real" 2>/dev/null)" >> "$OUT/host_devices.txt"
    # shellcheck disable=SC2016  # expanded inside the container
    res=$("$DOCKER" exec "$CONTAINER" sh -c 'if [ ! -e "$1" ]; then echo missing;
      elif [ -r "$1" ] && [ -w "$1" ]; then echo rw; else echo "no-access($(id -u):$(id -G | tr " " ,))"; fi' _ "$d" 2>&1)
    echo "$d container:$res" >> "$OUT/container_devices.txt"
    [ "$res" = rw ] || bad="$bad $d:$res"
  done
  if [ -n "$bad" ]; then
    record "$OUT" "device $cls" FAIL "$n on the host; in the container:$bad"
  else
    record "$OUT" "device $cls" PASS "$n on the host, all present and rw in the container"
  fi
done
if [ "$OPEN" = 1 ]; then
  for d in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$d" ] || continue
    res=$("$DOCKER" exec "$CONTAINER" python3 -c 'import os, sys
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
os.close(fd)
print("opened")' "$d" 2>&1 | tail -1)
    record "$OUT" "open $d" "$([ "$res" = opened ] && echo PASS || echo FAIL)" "$res"
  done
fi
if [ "$SCAN" = 1 ]; then
  # `ros2 topic hz` from the image (the car role mounts no checkout), as a
  # Discovery Server super client (a plain client is not told the topic).
  cexec env ROS_SUPER_CLIENT=TRUE timeout -s INT 20 ros2 topic hz /scan --window 200 \
    > "$OUT/scan_hz.log" 2>&1
  hz=$(awk '/average rate:/ { r = $3 } END { print r + 0 }' "$OUT/scan_hz.log")
  record "$OUT" "lidar /scan" "$(awk -v h="$hz" 'BEGIN { print (h > 30) ? "PASS" : "FAIL" }')" \
    "$hz Hz in the container (> 30 expected; scan_hz.log)"
fi
overall "$OUT"
