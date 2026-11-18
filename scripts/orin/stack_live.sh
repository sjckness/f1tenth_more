#!/usr/bin/env bash
# Launch the full stack inside the running container, with live ZED, lidar
# and VESC, plus foxglove_bridge on 8765. No motion command: this starts
# perception, control and the behavior tree idle, exactly as
# `f1tenth_ctl start stack` always has -- nothing here drives the car.
#
# camera_source (zed) and localization_source (ekf) are stack-wide branching
# args with no launch-time override (f1tenth_params/param_defaults.py's own
# docstring): the stack's defaults already mean live hardware, not
# simulation, so there is nothing to pass for that. enable_foxglove defaults
# true (port 8765); the container runs network_mode: host, so no port
# mapping is needed.
#
#   scripts/orin/stack_live.sh [--trt] [--rust <list|all>] [--timeout SECONDS]
#
#   --trt          yolo_backend:=engine (the TensorRT engine) instead of the
#                  default .pt. Build the engine first: docker_orin.sh engine.
#   --rust <list>  comma-separated subset of corridor,hard_stop,mpc_solver,
#          |all    wall_tracker -- that stack_params *_impl switch to 'rust'.
#                  Default: none (every switch stays 'py', the validated set).
#   --timeout N    seconds to wait for f1tenth_ctl status to report ready
#                  (default 60).
#
# Mechanism: writes (or, with neither flag, clears) the ONE line of extra
# ros2 launch arguments at ~/.config/f1tenth_car/stack_args, which
# ctl_container.env appends after the car profile on every `f1tenth_ctl
# start stack` (read-only bind mount into the container; see that file's own
# header). Requires the container already up: scripts/orin/container_up.sh.
#
# After it reports ready, samples /diagnostics/battery_status once (the
# earliest point vesc_driver_node has told anyone anything -- battery
# voltage is only observable through the VESC) and compares it against
# min_battery_voltage (stack_params.yaml). A low or missing reading is
# reported, never hidden, but does not stop the already-running stack.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# ros2 run would also find this (install/lib/<pkg>/<exe> is on its index),
# but the full path skips ros2 run's own startup cost for a one-shot CLI call.
CTL=/opt/f1tenth_ws/install/lib/f1tenth_ctl/f1tenth_ctl
case "${1:-}" in -h|--help)
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
  exit 0 ;;
esac

RUST_NAMES=(corridor hard_stop mpc_solver wall_tracker)
TRT=0
RUST_SEL=()
TIMEOUT=60
while [ $# -gt 0 ]; do
  case "$1" in
    --trt) TRT=1 ;;
    --rust)
      [ $# -ge 2 ] || { echo "stack_live: --rust needs a value (list or 'all')" >&2; exit 2; }
      raw=$2; shift
      if [ "$raw" = all ]; then
        RUST_SEL=("${RUST_NAMES[@]}")
      else
        IFS=',' read -r -a RUST_SEL <<< "$raw"
        for n in "${RUST_SEL[@]}"; do
          ok=0
          for known in "${RUST_NAMES[@]}"; do [ "$n" = "$known" ] && ok=1; done
          [ "$ok" = 1 ] || { echo "stack_live: unknown --rust name '$n' (corridor,hard_stop,mpc_solver,wall_tracker,all)" >&2; exit 2; }
        done
      fi ;;
    --timeout)
      [ $# -ge 2 ] || { echo "stack_live: --timeout needs a value" >&2; exit 2; }
      TIMEOUT=$2; shift ;;
    *) echo "stack_live: unknown option '$1'" >&2; exit 2 ;;
  esac
  shift
done

echo "== preflight =="
"$HERE/preflight.sh"

echo
echo "== container must already be up =="
STATUS_OUT=$("$REPO/scripts/docker_orin.sh" status) || true
echo "$STATUS_OUT"
echo "$STATUS_OUT" | grep -q "^container: f1tenth_car running" \
  || { echo "stack_live: the container is not up (scripts/orin/container_up.sh first)" >&2; exit 1; }

ARGS_FILE="$HOME/.config/f1tenth_car/stack_args"
LINE=""
[ "$TRT" = 1 ] && LINE="$LINE yolo_backend:=engine"
for n in "${RUST_SEL[@]}"; do LINE="$LINE ${n}_impl:=rust"; done
LINE="${LINE# }"
mkdir -p "$(dirname "$ARGS_FILE")"
if [ -n "$LINE" ]; then
  printf '%s\n' "$LINE" > "$ARGS_FILE"
  echo
  echo "stack_args: $LINE"
else
  rm -f "$ARGS_FILE"
  echo
  echo "stack_args: none (python defaults, .pt model)"
fi

echo
echo "== starting the stack =="
"$REPO/scripts/docker_orin.sh" exec "$CTL" start stack < /dev/null

echo
echo "== waiting for ready (timeout ${TIMEOUT}s) =="
deadline=$((SECONDS + TIMEOUT))
ready=0
while :; do
  STATUS_JSON=$("$REPO/scripts/docker_orin.sh" exec "$CTL" status < /dev/null 2>&1) || true
  parsed=$(printf '%s' "$STATUS_JSON" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception as exc:
    print("parse-error", exc); sys.exit(0)
print(("ready" if d.get("ready") else "not-ready"), d.get("failing"), d.get("estop_latched"))
' 2>&1)
  echo "  $parsed"
  case "$parsed" in ready*) ready=1; break ;; esac
  [ "$SECONDS" -lt "$deadline" ] || break
  sleep 3
done
if [ "$ready" != 1 ]; then
  echo "stack_live: not ready after ${TIMEOUT}s -- last status above; docker_orin.sh exec $CTL status --human for detail" >&2
  exit 1
fi

echo
echo "== battery voltage (advisory; vesc_driver_node's own telemetry) =="
# A plain `ros2 topic echo` CLI client does not see this Fast DDS Discovery
# Server's publishers (CLAUDE.md "ros2 CLI blind under Discovery Server");
# probe with rclpy directly instead, piped in over stdin (not -c: the
# worktree sandbox treats a multi-line -c argument as unreviewable).
PROBE=$(mktemp)
trap 'rm -f "$PROBE"' EXIT
cat > "$PROBE" <<'PY'
import time
import rclpy
from f1tenth_messages.msg import BatteryStatus

rclpy.init()
n = rclpy.create_node('battery_probe')
got = []
n.create_subscription(BatteryStatus, '/diagnostics/battery_status', got.append, 10)
end = time.monotonic() + 6
while time.monotonic() < end and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    m = got[-1]
    print('has_data=%s voltage=%.2f min_voltage=%.2f ok=%s' % (m.has_data, m.voltage, m.min_voltage, m.ok))
else:
    print('no-sample')
PY
READING=$("$REPO/scripts/docker_orin.sh" exec python3 - < "$PROBE" 2>&1) || true
rm -f "$PROBE"
echo "$READING"
case "$READING" in
  has_data=True*)
    echo "$READING" | grep -q 'ok=True' \
      || echo ">>> BELOW min_battery_voltage: check the pack before any floor run <<<" ;;
  *)
    echo "voltage: no sample ($READING) -- vesc_driver_node may still be starting; re-check:"
    echo "  docker_orin.sh exec $CTL status --human" ;;
esac

echo
echo "== foxglove_bridge (port 8765, network_mode: host) =="
LAN_IP=$(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -v '^127\.' | head -1)
TS_IP=$(tailscale ip -4 2>/dev/null || true)
TS_NAME=$(tailscale status --json 2>/dev/null | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(d.get("Self", {}).get("DNSName", "").rstrip("."))
except Exception:
    pass
' 2>/dev/null || true)
[ -n "$LAN_IP" ] && echo "  ws://$LAN_IP:8765"
[ -n "$TS_IP" ] && echo "  ws://$TS_IP:8765   (Tailscale)"
[ -n "$TS_NAME" ] && echo "  ws://$TS_NAME:8765   (Tailscale name)"
echo "Open this in Foxglove's laptop app -> Open connection -> Foxglove WebSocket."
echo
echo "No motion command was sent. Stop with: scripts/orin/stack_stop.sh"
