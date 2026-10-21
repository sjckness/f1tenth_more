#!/bin/bash
# The base's Wi-Fi hotspot for the car (f1tenth_comms), with NetworkManager.
# Run it on the base PC (linus or the tpad), by hand.
#
#   scripts/base/hotspot.sh up     [config]   # create/update the profile, bring it up
#   scripts/base/hotspot.sh down   [config]   # take it down (the profile stays)
#   scripts/base/hotspot.sh status [config]   # read-only: both radios, clients, responder
#
# config: scripts/base/hotspot.conf by default (from hotspot.conf.example).
#
# Two radios (linus since 2026-10-21): HOTSPOT_AP_IF is the access point,
# HOTSPOT_UPLINK_IF keeps eduroam. Both are given by MAC address or a
# stable interface name and resolved at run time, so a wlan0/wlan1 swap
# cannot put the hotspot on the uplink radio. The hotspot profile is bound
# to the AP radio (interface name AND MAC), and the client profiles in
# HOTSPOT_UPLINK_CONS (eduroam) are bound to the uplink radio, so
# NetworkManager never puts both on one radio.
#
# The profile is WPA2-PSK (RSN/CCMP) with ipv4.method shared at
# HOTSPOT_ADDRESS: NetworkManager runs DHCP and DNS for the car and NATs it
# to this PC's default route. Autoconnect is off: the hotspot is up only
# when you say so. `up` checks the uplink before and for HOTSPOT_GUARD_S
# seconds after, and takes the hotspot down again if the uplink dropped.
#
# One radio only: HOTSPOT_VIRTUAL=1 adds a virtual AP interface on the
# uplink radio (needs iw and sudo; see hotspot.conf.example).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cmd="${1:-status}"
conf="${2:-$here/hotspot.conf}"

die() { echo "hotspot: $*" >&2; exit 1; }
note() { echo "hotspot: $*"; }

[ -f "$conf" ] || die "no $conf: cp $here/hotspot.conf.example $here/hotspot.conf and edit it"
# Band, channel and guard can be overridden for one run from the environment:
#   HOTSPOT_BAND=bg HOTSPOT_CHANNEL=6 scripts/base/hotspot.sh up
env_band="${HOTSPOT_BAND:-}" env_channel="${HOTSPOT_CHANNEL:-}" env_guard="${HOTSPOT_GUARD_S:-}"
# shellcheck source=/dev/null
source "$conf"
HOTSPOT_BAND="${env_band:-${HOTSPOT_BAND:-}}"
HOTSPOT_CHANNEL="${env_channel:-${HOTSPOT_CHANNEL:-}}"
HOTSPOT_GUARD_S="${env_guard:-${HOTSPOT_GUARD_S:-}}"
HOTSPOT_AP_IF="${HOTSPOT_AP_IF:-${HOTSPOT_IFNAME:-}}"        # HOTSPOT_IFNAME: the old name
: "${HOTSPOT_CON:?}" "${HOTSPOT_SSID:?}" "${HOTSPOT_PSK:?}" "${HOTSPOT_ADDRESS:?}"
: "${HOTSPOT_UPLINK_IF:=}" "${HOTSPOT_UPLINK_CONS:=}" "${HOTSPOT_GUARD_S:=10}"
: "${HOTSPOT_BAND:=bg}" "${HOTSPOT_CHANNEL:=6}" "${HOTSPOT_VIRTUAL:=0}" "${HOTSPOT_VIF:=f1tap0}"
command -v nmcli >/dev/null || die "nmcli not found (NetworkManager)"

# ---- interfaces -------------------------------------------------------------------------

is_mac() { [[ ${1,,} =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; }

# Permanent MAC of an interface: NetworkManager randomises the current one
# while it scans unconnected.
perm_mac() {
  local line
  line=$(ip -o link show dev "$1" 2>/dev/null) || return 1
  if [[ $line =~ permaddr\ ([0-9a-f:]{17}) ]]; then echo "${BASH_REMATCH[1]}"; return; fi
  cat "/sys/class/net/$1/address"
}

# A MAC or an interface name -> the interface's current name ('' if absent).
resolve_if() {
  local want="${1,,}" d n
  [ -n "$want" ] || return 0
  if is_mac "$want"; then
    for d in /sys/class/net/*; do
      n=$(basename "$d")
      [ "$(perm_mac "$n" 2>/dev/null)" = "$want" ] && { echo "$n"; return; }
    done
    return 0
  fi
  [ -e "/sys/class/net/$1" ] && echo "$1"
  return 0
}

phy_of() { basename "$(readlink -f "/sys/class/net/$1/phy80211" 2>/dev/null)"; }
driver_of() { basename "$(readlink -f "/sys/class/net/$1/device/driver" 2>/dev/null)"; }
dev_state() { nmcli -g GENERAL.STATE device show "$1" 2>/dev/null | cut -d' ' -f1; }
dev_con() { nmcli -g GENERAL.CONNECTION device show "$1" 2>/dev/null || true; }
default_dev() { ip route show default 2>/dev/null | awk '{for (i=1;i<NF;i++) if ($i=="dev") {print $(i+1); exit}}'; }
profile_exists() { nmcli -t -f NAME connection show | grep -Fxq "$1"; }

UPLINK=$(resolve_if "$HOTSPOT_UPLINK_IF")
if [ "$HOTSPOT_VIRTUAL" = 1 ]; then
  [ -n "$UPLINK" ] || die "HOTSPOT_VIRTUAL=1 needs HOTSPOT_UPLINK_IF (the radio to share)"
  AP="$HOTSPOT_VIF"
else
  [ -n "$HOTSPOT_AP_IF" ] || die "set HOTSPOT_AP_IF in $conf"
  AP=$(resolve_if "$HOTSPOT_AP_IF")
fi

check_psk() {
  local n=${#HOTSPOT_PSK}
  [ "$n" -ge 8 ] && [ "$n" -le 63 ] || die "HOTSPOT_PSK must be 8..63 characters"
  [ "$HOTSPOT_PSK" != 'change-me-8-to-63-chars' ] || die "set HOTSPOT_PSK in $conf"
}

# ---- one radio: the uplink's channel, refusing radar/no-IR channels ------------------------

uplink_channel() {
  command -v iw >/dev/null || die "HOTSPOT_VIRTUAL=1 needs iw: sudo apt install iw"
  local freq phy line
  freq=$(iw dev "$UPLINK" link | awk '/freq:/ {print int($2); exit}')
  [ -n "$freq" ] || die "$UPLINK is not connected: concurrent mode shares its channel"
  phy=$(phy_of "$UPLINK")
  line=$(iw phy "$phy" info | grep -E "\* ${freq}(\.0)? MHz" | head -1 | sed 's/^[[:space:]]*//')
  [ -n "$line" ] || die "channel of $freq MHz not found in iw phy $phy info"
  if echo "$line" | grep -qE 'radar detection|no IR|disabled'; then
    die "the uplink is on $freq MHz ($line): no AP allowed there. Use a second radio or the travel router (output/comms/setup.md)"
  fi
  iw phy "$phy" info | grep -A3 'valid interface combinations' | grep -q 'AP' \
    || die "$phy lists no interface combination with an AP: no concurrent mode"
  echo "$line" | sed -E 's/.*\[([0-9]+)\].*/\1/'
  [ "$freq" -lt 3000 ] && echo bg || echo a
}

add_vif() {
  ip link show "$HOTSPOT_VIF" >/dev/null 2>&1 && return 0
  local mac first
  mac=$(perm_mac "$UPLINK")
  first=$(printf '%02x' $(( (0x${mac:0:2} | 0x02) ^ 0x04 )))    # locally administered, != parent
  note "adding $HOTSPOT_VIF (type AP) on $UPLINK's radio (sudo)"
  sudo iw dev "$UPLINK" interface add "$HOTSPOT_VIF" type __ap addr "$first${mac:2}"
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    nmcli -t -f DEVICE device | grep -Fxq "$HOTSPOT_VIF" && break
    sleep 0.5
  done
  nmcli device set "$HOTSPOT_VIF" managed yes
}

# ---- keeping the two radios apart --------------------------------------------------------

# Bind each uplink client profile (eduroam) to the uplink radio, unless it already is.
pin_uplink() {
  [ -n "$UPLINK" ] || return 0
  local con ifn mac want
  want=$(perm_mac "$UPLINK")
  for con in $HOTSPOT_UPLINK_CONS; do
    profile_exists "$con" || { note "no profile '$con' to pin"; continue; }
    ifn=$(nmcli -g connection.interface-name connection show id "$con")
    mac=$(nmcli -g 802-11-wireless.mac-address connection show id "$con" | tr -d '\\' | tr 'A-F' 'a-f')
    if [ "$ifn" = "$UPLINK" ] && { [ -z "$mac" ] || [ "$mac" = "$want" ]; }; then
      continue
    fi
    note "pinning '$con' to the uplink radio $UPLINK ($want); it was '${ifn:-any}' ${mac:-}"
    nmcli connection modify id "$con" connection.interface-name "$UPLINK" \
      802-11-wireless.mac-address "$want"
  done
}

# Autoconnecting client profiles that may take the AP radio when the hotspot is down.
warn_ap_clients() {
  local con type ifn auto
  while IFS=: read -r con type; do
    [ "$type" = 802-11-wireless ] && [ "$con" != "$HOTSPOT_CON" ] || continue
    ifn=$(nmcli -g connection.interface-name connection show id "$con")
    auto=$(nmcli -g connection.autoconnect connection show id "$con")
    if [ "$auto" = yes ] && { [ "$ifn" = "$AP" ] || [ -z "$ifn" ]; }; then
      note "WARNING: '$con' autoconnects on ${ifn:-any radio}: it can take the AP radio while the hotspot is down"
      note "         nmcli connection modify id '$con' connection.interface-name ${UPLINK:-<uplink>}"
    fi
  done < <(nmcli -t -f NAME,TYPE connection show)
}

uplink_ok() { [ -z "$UPLINK" ] || [ "$(dev_state "$UPLINK")" = 100 ]; }

# ---- commands -----------------------------------------------------------------------------

up() {
  check_psk
  local band=$HOTSPOT_BAND channel=$HOTSPOT_CHANNEL
  if [ "$HOTSPOT_VIRTUAL" = 1 ]; then
    local found
    found=$(uplink_channel)              # exits here (set -e) when it refuses
    channel=$(echo "$found" | sed -n 1p)
    band=$(echo "$found" | sed -n 2p)
    add_vif
  else
    [ -n "$AP" ] || die "HOTSPOT_AP_IF '$HOTSPOT_AP_IF' is not present (adapter plugged in?)"
    [ -e "/sys/class/net/$AP/wireless" ] || die "$AP is not a Wi-Fi interface"
    if [ -n "$UPLINK" ] && [ "$(phy_of "$AP")" = "$(phy_of "$UPLINK")" ]; then
      die "$AP and the uplink $UPLINK are the same radio: use another adapter or HOTSPOT_VIRTUAL=1"
    fi
    local active
    active=$(dev_con "$AP")
    if [ -n "$active" ] && [ "$active" != "$HOTSPOT_CON" ]; then
      echo "hotspot: $AP carries '$active' now. It becomes the AP and '$active' goes down$([ "$(default_dev)" = "$AP" ] && echo ", WITH this PC's default route: no internet to share")."
      local ok=''
      read -r -p "Continue? [y/N] " ok || true
      [ "$ok" = y ] || [ "$ok" = Y ] || die "nothing changed"
    fi
  fi
  pin_uplink
  warn_ap_clients
  local uplink_was_up=0
  if uplink_ok && [ -n "$UPLINK" ]; then
    uplink_was_up=1
  elif [ -n "$UPLINK" ]; then
    note "the uplink $UPLINK is not connected: the car will get no internet through this base"
  fi

  if profile_exists "$HOTSPOT_CON"; then
    note "updating profile $HOTSPOT_CON"
  else
    note "creating profile $HOTSPOT_CON"
    nmcli connection add type wifi ifname "$AP" con-name "$HOTSPOT_CON" \
      autoconnect no ssid "$HOTSPOT_SSID" >/dev/null
  fi
  local bind_mac=''
  [ "$HOTSPOT_VIRTUAL" = 1 ] || bind_mac=$(perm_mac "$AP")
  nmcli connection modify id "$HOTSPOT_CON" \
    connection.interface-name "$AP" 802-11-wireless.mac-address "$bind_mac" \
    connection.autoconnect no \
    802-11-wireless.mode ap 802-11-wireless.ssid "$HOTSPOT_SSID" \
    802-11-wireless.band "$band" 802-11-wireless.channel "$channel" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
    wifi-sec.psk "$HOTSPOT_PSK" \
    ipv4.method shared ipv4.addresses "$HOTSPOT_ADDRESS" ipv6.method disabled
  note "up: SSID $HOTSPOT_SSID on $AP ($(driver_of "$AP")), band $band channel $channel, $HOTSPOT_ADDRESS"
  nmcli --wait 30 connection up id "$HOTSPOT_CON" || { down; die "activation failed: down again"; }

  if [ "$uplink_was_up" = 1 ]; then
    # 3 s down in a row: eduroam re-authenticates about hourly with a ~1 s gap.
    local i bad=0
    for ((i = 0; i < HOTSPOT_GUARD_S; i++)); do
      if uplink_ok; then bad=0; else bad=$((bad + 1)); fi
      if [ "$bad" -ge 3 ]; then
        note "the uplink $UPLINK dropped after the hotspot came up: taking the hotspot down"
        down
        die "rolled back"
      fi
      sleep 1
    done
    note "uplink $UPLINK still connected after ${HOTSPOT_GUARD_S} s"
  fi
  status
}

down() {
  if nmcli -t -f NAME connection show --active | grep -Fxq "$HOTSPOT_CON"; then
    nmcli connection down id "$HOTSPOT_CON"
  else
    note "$HOTSPOT_CON is not active"
  fi
  if [ "$HOTSPOT_VIRTUAL" = 1 ] && ip link show "$HOTSPOT_VIF" >/dev/null 2>&1; then
    note "removing $HOTSPOT_VIF (sudo)"
    sudo iw dev "$HOTSPOT_VIF" del
  fi
}

radio_line() {
  local n=$1
  if [ -z "$n" ]; then echo "(not present)"; return; fi
  local link=''
  if command -v iw >/dev/null; then
    link=$(iw dev "$n" info 2>/dev/null | awk '/type/ {t=$2} /channel/ {c="channel "$2" "$3" "$4; sub(/,$/, "", c)} END {print t, c}')
  fi
  echo "$n $(perm_mac "$n") $(driver_of "$n") $(phy_of "$n"), NM: $(dev_state "$n") '$(dev_con "$n")' ${link}"
}

status() {
  echo "== hotspot $HOTSPOT_CON (SSID $HOTSPOT_SSID)"
  echo "AP radio:  $(radio_line "$AP")"
  echo "uplink:    $(radio_line "$UPLINK")"
  if profile_exists "$HOTSPOT_CON"; then
    local state
    state=$(nmcli -g GENERAL.STATE connection show id "$HOTSPOT_CON" 2>/dev/null || true)
    echo "profile:   present, ${state:-inactive}, bound to $(nmcli -g connection.interface-name connection show id "$HOTSPOT_CON") $(nmcli -g 802-11-wireless.mac-address connection show id "$HOTSPOT_CON" | tr -d '\\')"
  else
    echo "profile:   none (hotspot.sh up creates it)"
  fi
  [ -n "$AP" ] && echo "address:   $(ip -4 -br addr show dev "$AP" 2>/dev/null | awk '{print $3}' || true)"
  if [ -n "$AP" ] && command -v iw >/dev/null; then
    echo "clients:   $(iw dev "$AP" station dump 2>/dev/null | grep -c '^Station' || true)"
  fi
  echo "route:     default via $(default_dev)"
  if command -v tailscale >/dev/null; then
    echo "tailscale: $(tailscale ip -4 2>/dev/null | head -1)"
  fi
  local responder
  responder=$(systemctl --user is-active f1tenth-heartbeat 2>/dev/null || true)
  echo "heartbeat: f1tenth-heartbeat ${responder:-not installed}; UDP 47800 $(ss -lun 2>/dev/null | grep -q ':47800 ' && echo listening || echo 'NOT listening')"
  if grep -qs '^ENABLED=yes' /etc/ufw/ufw.conf; then
    echo "firewall:  ufw is enabled. The hotspot needs DHCP and DNS in, and the heartbeat:"
    echo "             sudo ufw allow in on ${AP:-<ap>} to any port 67 proto udp"
    echo "             sudo ufw allow in on ${AP:-<ap>} to any port 53"
    echo "             sudo ufw allow in on ${AP:-<ap>} to any port 47800 proto udp"
    echo "             sudo ufw allow in on tailscale0 to any port 47800 proto udp"
    echo "             sudo ufw route allow in on ${AP:-<ap>}    # the car's traffic to the internet"
  else
    echo "firewall:  ufw not enabled"
  fi
  if [ -e /sys/class/net/docker0 ]; then
    echo "docker:    present; if the car gets an address but no internet, Docker's FORWARD DROP is the"
    echo "           usual cause: sudo iptables -I DOCKER-USER -i ${AP:-<ap>} -j ACCEPT"
    echo "                        sudo iptables -I DOCKER-USER -o ${AP:-<ap>} -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT"
  fi
}

case "$cmd" in
  up) up ;;
  down) down ;;
  status) status ;;
  *) die "usage: $0 up|down|status [config]" ;;
esac
