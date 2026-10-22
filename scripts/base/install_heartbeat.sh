#!/bin/bash
# Install (or remove) the heartbeat responder as a systemd USER unit on the
# base PC (linus or the tpad). Run it there, as the user who owns the base
# session, by hand. No root needed, except the optional linger step.
#
#   scripts/base/install_heartbeat.sh            # install, enable, start
#   scripts/base/install_heartbeat.sh --uninstall
#
# What it does:
#   ~/.local/bin/f1tenth_heartbeat_responder.py   <- heartbeat_responder.py
#   ~/.config/systemd/user/f1tenth-heartbeat.service
#   systemctl --user enable f1tenth-heartbeat; restart it
# Then it prints what it could not do for you: linger (to run without a
# login session) and the firewall rule if ufw is active.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
bin="$HOME/.local/bin/f1tenth_heartbeat_responder.py"
unit_dir="$HOME/.config/systemd/user"
unit="$unit_dir/f1tenth-heartbeat.service"

if [ "${1:-}" = "--uninstall" ]; then
  systemctl --user disable --now f1tenth-heartbeat 2>/dev/null || true
  rm -f "$unit" "$bin"
  systemctl --user daemon-reload
  echo "removed f1tenth-heartbeat"
  exit 0
fi

mkdir -p "$(dirname "$bin")" "$unit_dir"
install -m 0755 "$here/heartbeat_responder.py" "$bin"
install -m 0644 "$here/f1tenth-heartbeat.service" "$unit"
systemctl --user daemon-reload
systemctl --user enable f1tenth-heartbeat
systemctl --user restart f1tenth-heartbeat       # a reinstall runs the new copy
sleep 1
systemctl --user --no-pager --lines=3 status f1tenth-heartbeat || true

echo
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
  echo "NOTE: linger is off: the responder stops when you log out. To keep it:"
  echo "  sudo loginctl enable-linger $USER"
fi
if grep -qs '^ENABLED=yes' /etc/ufw/ufw.conf; then
  echo "NOTE: ufw is enabled. Let the heartbeat in on Tailscale and on the hotspot:"
  echo "  sudo ufw allow in on tailscale0 to any port 47800 proto udp"
  echo "  sudo ufw allow in on <hotspot interface> to any port 47800 proto udp"
  echo "  (scripts/base/hotspot.sh status prints the hotspot interface)"
fi
