#!/bin/bash
# The car's (Orin's) side of the base hotspot. Run ON THE ORIN, by hand;
# nothing here runs from linus or from the stack.
#
#   scripts/orin/comms_network.sh check           # read-only: profiles, priorities, permissions
#   scripts/orin/comms_network.sh install [conf]  # create/update the hotspot client profile
#   scripts/orin/comms_network.sh polkit          # let this user run `nmcli connection up` (sudo)
#   scripts/orin/comms_network.sh remove          # delete the profile (eduroam untouched)
#
# conf: the base's hotspot.conf (scripts/base/hotspot.conf, copy it over);
# only HOTSPOT_CON, HOTSPOT_SSID and HOTSPOT_PSK are read.
#
# The profile autoconnects with priority COMMS_PRIORITY (default 50): above
# eduroam (0), so NetworkManager picks the hotspot whenever both are in
# range and the radio is free; eduroam stays as it is, the backup. NM does
# not leave a working eduroam for the hotspot on its own: that switch is
# link_manager's reconnect (`nmcli connection up id <profile>`), which needs
# the polkit rule below when the stack runs over ssh or as a service.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cmd="${1:-check}"
conf="${2:-$here/../base/hotspot.conf}"
PRIORITY="${COMMS_PRIORITY:-50}"
CAR_USER="${COMMS_USER:-$USER}"

die() { echo "comms_network: $*" >&2; exit 1; }
wifi_if() {
  for d in /sys/class/net/*/wireless; do
    [ -e "$d" ] && { basename "$(dirname "$d")"; return; }
  done
}
IFNAME="${COMMS_IFNAME:-$(wifi_if)}"

check() {
  echo "== Wi-Fi $IFNAME, user $CAR_USER"
  nmcli -t -f DEVICE,STATE,CONNECTION device | grep "^$IFNAME:" || true
  echo
  echo "== Wi-Fi profiles (autoconnect, priority, retries)"
  nmcli -t -f NAME,TYPE connection show | awk -F: '$2 == "802-11-wireless" {print $1}' |
  while read -r name; do
    printf '%-24s %s\n' "$name" "$(nmcli -g connection.autoconnect,connection.autoconnect-priority,connection.autoconnect-retries connection show id "$name" | paste -sd ' ' -)"
  done
  echo
  echo "== permissions of $USER (link_manager needs network-control and wifi.scan = yes)"
  nmcli general permissions | grep -E 'network-control|wifi.scan' || true
  echo
  echo "== signal"
  cat /proc/net/wireless 2>/dev/null | tail -n +3 || true
  local higher
  higher=$(nmcli -t -f NAME,TYPE connection show | awk -F: '$2 == "802-11-wireless" {print $1}' |
    while read -r name; do
      p=$(nmcli -g connection.autoconnect-priority connection show id "$name")
      if [ "${p:-0}" -gt "$PRIORITY" ]; then echo "$name ($p)"; fi
    done | paste -sd ',' -)
  if [ -n "$higher" ]; then
    echo
    echo "NOTE: above the hotspot's priority $PRIORITY: $higher. Where both are in range, NM picks those."
  fi
}

install() {
  [ -f "$conf" ] || die "no $conf: copy the base's scripts/base/hotspot.conf here"
  # shellcheck source=/dev/null
  source "$conf"
  : "${HOTSPOT_CON:?}" "${HOTSPOT_SSID:?}" "${HOTSPOT_PSK:?}"
  [ -n "$IFNAME" ] || die "no Wi-Fi interface (set COMMS_IFNAME)"
  if ! nmcli -t -f NAME connection show | grep -Fxq "$HOTSPOT_CON"; then
    nmcli connection add type wifi ifname "$IFNAME" con-name "$HOTSPOT_CON" \
      ssid "$HOTSPOT_SSID" >/dev/null
  fi
  # autoconnect-retries 0: retry forever. NM's default (4) blocks the profile
  # for 5 min after 4 failures, e.g. while the base is not up yet.
  # powersave 2 = off: no power-save latency on the heartbeat.
  nmcli connection modify "$HOTSPOT_CON" \
    connection.interface-name "$IFNAME" \
    connection.autoconnect yes connection.autoconnect-priority "$PRIORITY" \
    connection.autoconnect-retries 0 \
    802-11-wireless.mode infrastructure 802-11-wireless.ssid "$HOTSPOT_SSID" \
    802-11-wireless.powersave 2 \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$HOTSPOT_PSK" \
    ipv4.method auto ipv6.method auto
  echo "comms_network: profile $HOTSPOT_CON on $IFNAME, priority $PRIORITY (eduroam untouched)"
  echo "comms_network: it is NOT activated now. With the base's hotspot up:"
  echo "  nmcli connection up id $HOTSPOT_CON      # this ssh session drops if it runs over eduroam"
  check
}

polkit() {
  local version
  version=$(pkaction --version 2>/dev/null | awk '{print $NF}')
  [ -n "$version" ] || die "pkaction not found: is polkit installed?"
  # polkit before 0.106 (Ubuntu 22.04: 0.105) reads .pkla files, later ones JS rules.
  if [ "${version%%.*}" = 0 ] && [ "${version#0.}" -lt 106 ] 2>/dev/null; then
    local f=/etc/polkit-1/localauthority/50-local.d/50-f1tenth-comms.pkla
    echo "comms_network: polkit $version (.pkla), writing $f (sudo)"
    sudo mkdir -p "$(dirname "$f")"
    sudo tee "$f" >/dev/null <<EOF
[f1tenth_comms link_manager: activate connections and scan without a password]
Identity=unix-user:$CAR_USER
Action=org.freedesktop.NetworkManager.network-control;org.freedesktop.NetworkManager.wifi.scan
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOF
  else
    local f=/etc/polkit-1/rules.d/50-f1tenth-comms.rules
    echo "comms_network: polkit $version (JS rules), writing $f (sudo)"
    sudo tee "$f" >/dev/null <<EOF
// f1tenth_comms link_manager: activate connections and scan without a password.
polkit.addRule(function(action, subject) {
  if ((action.id == "org.freedesktop.NetworkManager.network-control" ||
       action.id == "org.freedesktop.NetworkManager.wifi.scan") &&
      subject.user == "$CAR_USER") {
    return polkit.Result.YES;
  }
});
EOF
  fi
  sleep 1
  nmcli general permissions | grep -E 'network-control|wifi.scan'
}

remove() {
  # shellcheck source=/dev/null
  [ -f "$conf" ] && source "$conf"
  local con="${HOTSPOT_CON:-f1tenth-base}"
  nmcli connection delete id "$con"
}

case "$cmd" in
  check) check ;;
  install) install ;;
  polkit) polkit ;;
  remove) remove ;;
  *) die "usage: $0 check|install [conf]|polkit|remove [conf]" ;;
esac
