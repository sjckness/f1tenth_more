#!/bin/bash
# Can this PC be the car's base? Read-only: changes nothing, needs no root.
# Run it on linus or the tpad before hotspot.sh.
#
#   scripts/base/check_ap.sh [wifi interface]     # default: the first Wi-Fi interface
#
# Prints the card and driver, whether it can be an access point (AP), whether
# it can be one WHILE it stays connected as a client (concurrent mode, to
# share eduroam), the uplink and its channel, and a verdict. `iw` gives the
# full answer; without it only NetworkManager's AP yes/no is known
# (sudo apt install iw).
set -uo pipefail
IW="${IW:-iw}"

ifname="${1:-}"
if [ -z "$ifname" ]; then
  for d in /sys/class/net/*/wireless; do
    [ -e "$d" ] && { ifname=$(basename "$(dirname "$d")"); break; }
  done
fi
[ -n "$ifname" ] || { echo "no Wi-Fi interface found"; exit 1; }
default_dev=$(ip route show default 2>/dev/null | awk '{for (i=1;i<NF;i++) if ($i=="dev") {print $(i+1); exit}}')

echo "== $ifname"
echo "card:       $(lspci -s "$(basename "$(readlink -f "/sys/class/net/$ifname/device")")" 2>/dev/null | cut -d' ' -f2- || true)"
echo "driver:     $(basename "$(readlink -f "/sys/class/net/$ifname/device/driver")")"
nm_ap=$(nmcli -g WIFI-PROPERTIES.AP device show "$ifname" 2>/dev/null || true)
echo "NM AP:      ${nm_ap:-unknown}"
echo "connection: $(nmcli -g GENERAL.CONNECTION device show "$ifname" 2>/dev/null || true)"
echo "uplink:     default route via ${default_dev:-none}"

if ! command -v "$IW" >/dev/null; then
  echo
  echo "iw is not installed: AP yes/no above is all there is. For interface"
  echo "combinations (concurrent mode): sudo apt install iw, then run this again."
  exit 0
fi

phy=$(basename "$(readlink -f "/sys/class/net/$ifname/phy80211")")
info=$("$IW" phy "$phy" info)
# The block under a one-tab header, up to the next one-tab header.
block() { echo "$info" | awk -v h="$1" 'index($0, h) {f=1; next} f && /^\t[^\t]/ {f=0} f'; }
modes=$(block 'Supported interface modes:' | grep '\* ' | sed 's/.*\* //' | paste -sd ' ' -)
combos=$(block 'valid interface combinations:' | sed 's/^[[:space:]]*//' | paste -sd ' ' -)
echo "phy:        $phy"
echo "modes:      $modes"
echo "combos:     ${combos:-none listed}"
link=$("$IW" dev "$ifname" link 2>/dev/null || true)
freq=$(echo "$link" | awk '/freq:/ {print int($2); exit}')
if [ -n "$freq" ]; then
  chan=$(echo "$info" | grep -E "\* ${freq}(\.0)? MHz" | head -1 | sed 's/^[[:space:]]*\* //')
  echo "client on:  $freq MHz: $chan"
fi
ap_channels=$(echo "$info" | grep -E '\* [0-9.]+ MHz \[[0-9]+\]' | grep -vE 'disabled|radar detection|no IR' \
  | sed -E 's/.*\[([0-9]+)\].*/\1/' | paste -sd ' ' -)
echo "AP chans:   $ap_channels   (no radar detection, no no-IR)"

echo
echo "== verdict"
ap=no; echo "$modes" | grep -qw AP && ap=yes
concurrent=no
echo "$combos" | grep -q 'AP' && echo "$combos" | grep -q 'managed' && concurrent=yes
echo "AP mode:                    $ap"
echo "AP while connected (STA+AP): $concurrent$([ "$concurrent" = yes ] && echo ', on the client channel only (#channels <= 1)')"
if [ "$ap" = yes ]; then
  if [ "$default_dev" != "$ifname" ]; then
    echo "dedicated mode:             OK: the internet comes from $default_dev, not from this radio"
  else
    echo "dedicated mode:             the radio carries the internet ($ifname): the car would get none"
  fi
  if [ "$concurrent" = yes ] && [ -n "$freq" ]; then
    if echo "$chan" | grep -qE 'radar detection|no IR|disabled'; then
      echo "concurrent mode:            NOT now: the client is on a radar/no-IR channel ($freq MHz)"
    else
      echo "concurrent mode:            possible now, on $freq MHz (until the client roams)"
    fi
  fi
fi
