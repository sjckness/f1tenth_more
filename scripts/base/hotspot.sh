#!/bin/bash
# The base's Wi-Fi hotspot for the car (f1tenth_comms), with NetworkManager.
# Run it on the base PC (linus or the tpad), by hand.
#
#   scripts/base/hotspot.sh up     [config]   # create/update the profile, bring it up
#   scripts/base/hotspot.sh down   [config]   # take it down (the profile stays)
#   scripts/base/hotspot.sh status [config]   # read-only: state, address, clients, responder
#
# config: scripts/base/hotspot.conf by default (from hotspot.conf.example).
#
# The profile is WPA2-PSK (RSN/CCMP) with ipv4.method shared at
# HOTSPOT_ADDRESS: NetworkManager runs DHCP and DNS for the car and NATs it
# to this PC's default route (ethernet or eduroam), so the car keeps
# internet and Tailscale through the base. Autoconnect is off: the hotspot
# is up only when you say so.
#
# Needs nmcli. HOTSPOT_VIRTUAL=1 also needs iw and sudo (it adds a virtual
# AP interface, see hotspot.conf.example).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cmd="${1:-status}"
conf="${2:-$here/hotspot.conf}"

die() { echo "hotspot: $*" >&2; exit 1; }
note() { echo "hotspot: $*"; }

[ -f "$conf" ] || die "no $conf: cp $here/hotspot.conf.example $here/hotspot.conf and edit it"
# shellcheck source=/dev/null
source "$conf"
: "${HOTSPOT_CON:?}" "${HOTSPOT_SSID:?}" "${HOTSPOT_PSK:?}" "${HOTSPOT_IFNAME:?}"
: "${HOTSPOT_ADDRESS:?}" "${HOTSPOT_BAND:=bg}" "${HOTSPOT_CHANNEL:=6}"
: "${HOTSPOT_VIRTUAL:=0}" "${HOTSPOT_VIF:=f1tap0}"
command -v nmcli >/dev/null || die "nmcli not found (NetworkManager)"

ap_if() { if [ "$HOTSPOT_VIRTUAL" = 1 ]; then echo "$HOTSPOT_VIF"; else echo "$HOTSPOT_IFNAME"; fi; }
profile_exists() { nmcli -t -f NAME connection show | grep -Fxq "$HOTSPOT_CON"; }
default_dev() { ip route show default 2>/dev/null | awk '{for (i=1;i<NF;i++) if ($i=="dev") {print $(i+1); exit}}'; }

check_psk() {
  local n=${#HOTSPOT_PSK}
  [ "$n" -ge 8 ] && [ "$n" -le 63 ] || die "HOTSPOT_PSK must be 8..63 characters"
  [ "$HOTSPOT_PSK" != 'change-me-8-to-63-chars' ] || die "set HOTSPOT_PSK in $conf"
}

# Channel of the uplink on HOTSPOT_IFNAME, refusing radar/no-IR channels.
uplink_channel() {
  command -v iw >/dev/null || die "HOTSPOT_VIRTUAL=1 needs iw: sudo apt install iw"
  local freq phy line
  freq=$(iw dev "$HOTSPOT_IFNAME" link | awk '/freq:/ {print int($2); exit}')
  [ -n "$freq" ] || die "$HOTSPOT_IFNAME is not connected: concurrent mode shares its channel"
  phy=$(basename "$(readlink -f "/sys/class/net/$HOTSPOT_IFNAME/phy80211")")
  line=$(iw phy "$phy" info | grep -E "\* ${freq}(\.0)? MHz" | head -1 | sed 's/^[[:space:]]*//')
  [ -n "$line" ] || die "channel of $freq MHz not found in iw phy $phy info"
  if echo "$line" | grep -qE 'radar detection|no IR|disabled'; then
    die "the uplink is on $freq MHz ($line): no AP allowed there. Use dedicated mode, a USB adapter or the travel router (output/comms/setup.md)"
  fi
  iw phy "$phy" info | grep -A3 'valid interface combinations' | grep -q 'AP' \
    || die "$phy lists no interface combination with an AP: no concurrent mode"
  echo "$line" | sed -E 's/.*\[([0-9]+)\].*/\1/'
  [ "$freq" -lt 3000 ] && echo bg || echo a
}

add_vif() {
  ip link show "$HOTSPOT_VIF" >/dev/null 2>&1 && return 0
  local mac first
  mac=$(cat "/sys/class/net/$HOTSPOT_IFNAME/address")
  first=$(printf '%02x' $(( (0x${mac:0:2} | 0x02) ^ 0x04 )))    # locally administered, != parent
  note "adding $HOTSPOT_VIF (type AP) on $HOTSPOT_IFNAME's radio (sudo)"
  sudo iw dev "$HOTSPOT_IFNAME" interface add "$HOTSPOT_VIF" type __ap addr "$first${mac:2}"
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    nmcli -t -f DEVICE device | grep -Fxq "$HOTSPOT_VIF" && break
    sleep 0.5
  done
  nmcli device set "$HOTSPOT_VIF" managed yes
}

up() {
  check_psk
  local ifname band channel
  ifname=$(ap_if)
  band=$HOTSPOT_BAND
  channel=$HOTSPOT_CHANNEL
  if [ "$HOTSPOT_VIRTUAL" = 1 ]; then
    local found
    found=$(uplink_channel)              # exits here (set -e) when it refuses
    channel=$(echo "$found" | sed -n 1p)
    band=$(echo "$found" | sed -n 2p)
    add_vif
  else
    [ -e "/sys/class/net/$ifname/wireless" ] || die "$ifname is not a Wi-Fi interface"
    local active
    active=$(nmcli -g GENERAL.CONNECTION device show "$ifname" 2>/dev/null || true)
    if [ -n "$active" ] && [ "$active" != "$HOTSPOT_CON" ]; then
      echo "hotspot: $ifname carries '$active' now. In dedicated mode the radio becomes the AP"
      echo "         and '$active' goes down$([ "$(default_dev)" = "$ifname" ] && echo ", WITH this PC's default route: no internet to share")."
      local ok=''
      read -r -p "Continue? [y/N] " ok || true
      [ "$ok" = y ] || [ "$ok" = Y ] || die "nothing changed"
    fi
  fi
  if profile_exists; then
    note "updating profile $HOTSPOT_CON"
  else
    note "creating profile $HOTSPOT_CON"
    nmcli connection add type wifi ifname "$ifname" con-name "$HOTSPOT_CON" \
      autoconnect no ssid "$HOTSPOT_SSID" >/dev/null
  fi
  nmcli connection modify "$HOTSPOT_CON" \
    connection.interface-name "$ifname" connection.autoconnect no \
    802-11-wireless.mode ap 802-11-wireless.ssid "$HOTSPOT_SSID" \
    802-11-wireless.band "$band" 802-11-wireless.channel "$channel" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
    wifi-sec.psk "$HOTSPOT_PSK" \
    ipv4.method shared ipv4.addresses "$HOTSPOT_ADDRESS" ipv6.method disabled
  note "up: SSID $HOTSPOT_SSID on $ifname, band $band channel $channel, $HOTSPOT_ADDRESS"
  nmcli --wait 30 connection up "$HOTSPOT_CON"
  status
}

down() {
  if nmcli -t -f NAME connection show --active | grep -Fxq "$HOTSPOT_CON"; then
    nmcli connection down "$HOTSPOT_CON"
  else
    note "$HOTSPOT_CON is not active"
  fi
  if [ "$HOTSPOT_VIRTUAL" = 1 ] && ip link show "$HOTSPOT_VIF" >/dev/null 2>&1; then
    note "removing $HOTSPOT_VIF (sudo)"
    sudo iw dev "$HOTSPOT_VIF" del
  fi
}

status() {
  local ifname state
  ifname=$(ap_if)
  echo "== hotspot $HOTSPOT_CON (SSID $HOTSPOT_SSID) on $ifname"
  if profile_exists; then
    state=$(nmcli -g GENERAL.STATE connection show "$HOTSPOT_CON" 2>/dev/null || true)
    echo "profile:   present, ${state:-inactive}"
  else
    echo "profile:   none (hotspot.sh up creates it)"
  fi
  echo "address:   $(ip -4 -br addr show dev "$ifname" 2>/dev/null | awk '{print $3}' || true)"
  if command -v iw >/dev/null && ip link show "$ifname" >/dev/null 2>&1; then
    echo "clients:   $(iw dev "$ifname" station dump 2>/dev/null | grep -c '^Station' || true)"
  fi
  local leases="/var/lib/NetworkManager/dnsmasq-$ifname.leases"
  [ -r "$leases" ] && echo "leases:    $(awk '{print $3, $4}' "$leases" | paste -sd ', ' -)"
  echo "uplink:    default route via $(default_dev)"
  if command -v tailscale >/dev/null; then
    echo "tailscale: $(tailscale ip -4 2>/dev/null | head -1)"
  fi
  local responder
  responder=$(systemctl --user is-active f1tenth-heartbeat 2>/dev/null || true)
  echo "heartbeat: f1tenth-heartbeat ${responder:-not installed}; UDP 47800 $(ss -lun 2>/dev/null | grep -q ':47800 ' && echo listening || echo 'NOT listening')"
  if systemctl is-active --quiet ufw 2>/dev/null; then
    echo "firewall:  ufw is active. The hotspot needs DHCP and DNS in, and the heartbeat:"
    echo "             sudo ufw allow in on $ifname to any port 67 proto udp"
    echo "             sudo ufw allow in on $ifname to any port 53"
    echo "             sudo ufw allow in on $ifname to any port 47800 proto udp"
    echo "             sudo ufw allow in on tailscale0 to any port 47800 proto udp"
    echo "             sudo ufw route allow in on $ifname    # the car's traffic to the internet"
  fi
}

case "$cmd" in
  up) up ;;
  down) down ;;
  status) status ;;
  *) die "usage: $0 up|down|status [config]" ;;
esac
