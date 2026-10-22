#!/bin/bash
# Can this PC be the car's base? Read-only: changes nothing, needs no root.
# Run it on linus or the tpad before hotspot.sh.
#
#   scripts/base/check_ap.sh            # every Wi-Fi radio
#   scripts/base/check_ap.sh wlp4s0     # just this one
#
# Per radio: card, driver (in the stock kernel or not), MAC, whether it can
# be an access point (AP), bands and the channels an AP may use, whether it
# can be an AP while it stays a client (one-radio concurrent mode), and what
# it is connected to. Then a layout: which radio keeps the uplink, which
# one can be the hotspot. `iw` gives the full answer; without it only
# NetworkManager's AP yes/no is known (sudo apt install iw).
set -uo pipefail
IW="${IW:-iw}"
have_iw=1
command -v "$IW" >/dev/null || have_iw=0

perm_mac() {
  local line
  line=$(ip -o link show dev "$1" 2>/dev/null) || return 1
  if [[ $line =~ permaddr\ ([0-9a-f:]{17}) ]]; then echo "${BASH_REMATCH[1]}"; return; fi
  cat "/sys/class/net/$1/address"
}

radios=()
if [ $# -gt 0 ]; then
  radios=("$@")
else
  for d in /sys/class/net/*/wireless; do
    [ -e "$d" ] && radios+=("$(basename "$(dirname "$d")")")
  done
fi
[ ${#radios[@]} -gt 0 ] || { echo "no Wi-Fi interface found"; exit 1; }
default_dev=$(ip route show default 2>/dev/null | awk '{for (i=1;i<NF;i++) if ($i=="dev") {print $(i+1); exit}}')
echo "default route via ${default_dev:-none}"

declare -A AP CONC UP CHAN
for ifname in "${radios[@]}"; do
  echo
  echo "== $ifname"
  dev=$(readlink -f "/sys/class/net/$ifname/device")
  drv=$(basename "$(readlink -f "/sys/class/net/$ifname/device/driver")")
  case "$dev" in
    */usb*) bus=usb
            usbdir=$(dirname "$dev")
            card="$(cat "$usbdir/manufacturer" 2>/dev/null) $(cat "$usbdir/product" 2>/dev/null) [$(cat "$usbdir/idVendor" 2>/dev/null):$(cat "$usbdir/idProduct" 2>/dev/null)], USB $(cat "$usbdir/speed" 2>/dev/null) Mb/s" ;;
    *) bus=pci
       card=$(lspci -s "$(basename "$dev")" 2>/dev/null | cut -d' ' -f2-) ;;
  esac
  modfile=$(modinfo -F filename "$drv" 2>/dev/null)
  case "$modfile" in
    */kernel/drivers/*) origin="in the stock kernel" ;;
    */updates/*|*/extra/*) origin="OUT-OF-TREE ($modfile)" ;;
    *) origin="${modfile:-unknown}" ;;
  esac
  echo "card:       $card ($bus)"
  echo "driver:     $drv, $origin"
  echo "MAC:        $(perm_mac "$ifname")"
  nm_ap=$(nmcli -g WIFI-PROPERTIES.AP device show "$ifname" 2>/dev/null || true)
  con=$(nmcli -g GENERAL.CONNECTION device show "$ifname" 2>/dev/null || true)
  echo "NM:         AP ${nm_ap:-unknown}, connection '${con}'"
  AP[$ifname]=$nm_ap
  CONC[$ifname]=unknown
  UP[$ifname]=$con
  [ "$have_iw" = 1 ] || continue

  phy=$(basename "$(readlink -f "/sys/class/net/$ifname/phy80211")")
  info=$("$IW" phy "$phy" info)
  # The block under a one-tab header, up to the next one-tab header.
  block() { echo "$info" | awk -v h="$1" 'index($0, h) {f=1; next} f && /^\t[^\t]/ {f=0} f'; }
  modes=$(block 'Supported interface modes:' | grep '\* ' | sed 's/.*\* //' | paste -sd ' ' -)
  combos=$(block 'valid interface combinations:' | sed 's/^[[:space:]]*//' | paste -sd ' ' -)
  bands=$(echo "$info" | grep -oE 'Band [0-9]+' | sed 's/Band 1/2.4 GHz/; s/Band 2/5 GHz/; s/Band 4/6 GHz/' | paste -sd ',' -)
  echo "phy:        $phy, bands $bands"
  echo "modes:      $modes"
  echo "combos:     ${combos:-none listed (one interface at a time)}"
  ap_channels=$(echo "$info" | grep -E '\* [0-9.]+ MHz \[[0-9]+\]' | grep -vE 'disabled|radar detection|no IR' \
    | sed -E 's/.*\[([0-9]+)\].*/\1/' | paste -sd ' ' -)
  radar=$(echo "$info" | grep -E '\* [0-9.]+ MHz \[[0-9]+\].*(radar detection|no IR)' \
    | sed -E 's/.*\[([0-9]+)\].*/\1/' | paste -sd ' ' -)
  echo "AP chans:   $ap_channels"
  echo "radar/noIR: ${radar:-none}"
  freq=$("$IW" dev "$ifname" link 2>/dev/null | awk '/freq:/ {print int($2); exit}')
  if [ -n "$freq" ]; then
    CHAN[$ifname]=$(echo "$info" | grep -E "\* ${freq}(\.0)? MHz" | head -1 | sed 's/^[[:space:]]*\* //')
    echo "client on:  ${CHAN[$ifname]}"
  fi
  if echo "$modes" | grep -qE '(^| )AP( |$)'; then AP[$ifname]=yes; else AP[$ifname]=no; fi
  if echo "$combos" | grep -q 'AP' && echo "$combos" | grep -q 'managed'; then
    CONC[$ifname]=yes
  else
    CONC[$ifname]=no
  fi
done

echo
echo "== layout"
uplink=''
for r in "${radios[@]}"; do [ "$r" = "$default_dev" ] && uplink=$r; done
if [ -z "$uplink" ]; then
  for r in "${radios[@]}"; do [ -n "${UP[$r]}" ] && { uplink=$r; break; }; done
fi
candidates=()
for r in "${radios[@]}"; do
  [ "$r" != "$uplink" ] && [ "${AP[$r]}" = yes ] && candidates+=("$r")
done
if [ -n "$uplink" ]; then
  echo "uplink:     $uplink ('${UP[$uplink]}')$([ "$uplink" = "$default_dev" ] && echo ', carries the default route')"
else
  echo "uplink:     no Wi-Fi radio carries the default route (${default_dev:-none})"
fi
if [ ${#candidates[@]} -gt 0 ]; then
  echo "hotspot:    ${candidates[*]} (a radio of its own: the uplink is untouched)"
  echo "            HOTSPOT_AP_IF=$(perm_mac "${candidates[0]}")  HOTSPOT_UPLINK_IF=${uplink:+$(perm_mac "$uplink")}"
elif [ -n "$uplink" ] && [ "${CONC[$uplink]}" = yes ]; then
  if echo "${CHAN[$uplink]:-}" | grep -qE 'radar detection|no IR|disabled'; then
    echo "hotspot:    only concurrent mode on $uplink, and NOT now: the uplink is on a radar/no-IR channel"
  else
    echo "hotspot:    only concurrent mode on $uplink (HOTSPOT_VIRTUAL=1), on its channel, until it roams"
  fi
elif [ -n "$uplink" ] && [ "${AP[$uplink]}" = yes ]; then
  echo "hotspot:    only by giving up the uplink $uplink: the car would get no internet"
else
  echo "hotspot:    no radio here can be an access point"
fi
[ "$have_iw" = 1 ] || echo "(iw not installed: interface combinations unknown; sudo apt install iw)"
