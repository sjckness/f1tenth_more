#!/bin/bash
# ============================================================================
# Entrypoint for the F1TENTH Jetson AGX Thor dev container.
# Runs for the container CMD, `docker exec`, and devcontainer sessions, so ROS
# is sourced even for non-interactive invocations (lesson m). Interactive
# shells additionally pick these up via /etc/bash.bashrc.
# ============================================================================
set -e

# --- source ROS + workspace overlay (lesson m) ----------------------------
# transport_drivers is vendored in src/, so there is NO /opt/ros_external_install
# overlay to source here (Layer 11 omitted in the Dockerfile).
source /opt/ros/humble/setup.bash
[ -f /f1tenth_more/install/setup.bash ] && source /f1tenth_more/install/setup.bash

# --- VESC symlink (lesson l) ----------------------------------------------
# The udev symlink /dev/sensors/vesc does not propagate reliably into the
# container. Recreate it from the raw ttyACM0 if present; never block startup.
mkdir -p /dev/sensors 2>/dev/null || true
[ -e /dev/ttyACM0 ] && ln -sf /dev/ttyACM0 /dev/sensors/vesc 2>/dev/null || true

# --- best-effort rosdep install over the bind-mounted src/ (lesson j) -----
# src/ is only present at run time. Best-effort: never fail startup if src/ is
# not mounted yet or a key is unresolved.
if [ -d /f1tenth_more/src ]; then
  cd /f1tenth_more && \
    rosdep install --from-paths src --ignore-src -y -r --rosdistro humble || true
fi

exec "$@"
