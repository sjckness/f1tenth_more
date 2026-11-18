# shellcheck shell=bash disable=SC2034  # ORIN[] e le altre globali le leggono gli script che fanno source
# tools/core/lib/checks.sh - sonde riusabili per check (stampano un valore; exit 0 OK, 10 WARN, 1 FAIL).
#
#   probe_local_host NOME          hostname di questa macchina == NOME
#   probe_ping IP [N]              latenza media e perdita su N ping (WARN se perdita > 0)
#   probe_route_direct IP          la rotta verso IP non passa da un gateway
#   probe_iface IFACE|auto IP MTU  interfaccia con l'IP atteso e la MTU attesa (auto = quella della rotta)
#   probe_gpu NOME MIN_FREE_MB     GPU, driver, VRAM libera
#   probe_disk PERCORSO MIN_GB     spazio libero
#   probe_docker_image IMMAGINE    immagine presente in locale
#   probe_clock_offset             differenza di orologio con l'Orin (F1T_CLOCK_WARN_MS / FAIL_MS)
#   probe_git_clean [DIR]          working tree senza modifiche ai file tracciati
#   orin_facts                     carica in ORIN[...] i fatti dell'Orin (una sola connessione ssh)
[ -n "${_F1T_CHECKS_SH:-}" ] && return 0
_F1T_CHECKS_SH=1
# shellcheck source=tools/core/lib/remote.sh
source "$(dirname "${BASH_SOURCE[0]}")/remote.sh"

probe_local_host() {
  local h; h=$(hostname)
  [ "$h" = "$1" ] || { echo "hostname $h, atteso $1"; return 1; }
  echo "$h"
}

probe_ping() {
  local ip=$1 n=${2:-3} out loss avg
  out=$(ping -n -q -c "$n" -i 0.2 -W 1 "$ip" 2>&1) || true
  loss=$(printf '%s\n' "$out" | sed -n 's/.* \([0-9.]*\)% packet loss.*/\1/p')
  avg=$(printf '%s\n' "$out" | sed -n 's|^rtt [^=]*= [0-9.]*/\([0-9.]*\)/.*|\1|p')
  [ -n "$loss" ] || { echo "ping $ip: $(printf '%s\n' "$out" | tail -1)"; return 1; }
  if [ -z "$avg" ]; then echo "$ip non risponde ($loss% persi su $n)"; return 1; fi
  echo "$avg ms, persi $loss% su $n"
  [ "${loss%.*}" = 0 ] || return 10
}

probe_route_direct() {
  local r; r=$(ip route get "$1" 2>&1) || { echo "nessuna rotta verso $1"; return 1; }
  r=$(printf '%s\n' "$r" | head -1)
  case "$r" in *" via "*) echo "passa da un gateway: $r"; return 1 ;; esac
  echo "diretta: dev $(printf '%s\n' "$r" | sed -n 's/.* dev \([^ ]*\).*/\1/p')"
}

link_iface() {
  local iface=$1 ip=$2
  if [ "$iface" = auto ]; then
    ip route get "$ip" 2>/dev/null | sed -n '1s/.* dev \([^ ]*\).*/\1/p'
  else
    echo "$iface"
  fi
}

probe_iface() {
  local iface ip=$2 mtu=$3 peer=${4:-} addr m
  iface=$(link_iface "$1" "${peer:-$ip}")
  [ -n "$iface" ] || { echo "interfaccia non trovata (auto: nessuna rotta verso ${peer:-$ip})"; return 1; }
  addr=$(ip -o -4 addr show dev "$iface" 2>/dev/null | awk '{print $4}' | tr '\n' ' ')
  m=$(cat "/sys/class/net/$iface/mtu" 2>/dev/null || echo '?')
  case " $addr" in *" $ip/"*) ;; *) echo "$iface ha [${addr% }], manca $ip"; return 1 ;; esac
  echo "$iface $ip mtu $m"
  [ "$m" = "$mtu" ] || return 10
}

probe_gpu() {
  local want=$1 min=$2 line name drv free
  command -v nvidia-smi >/dev/null || { echo "nvidia-smi assente"; return 1; }
  line=$(nvidia-smi --query-gpu=name,driver_version,memory.free --format=csv,noheader,nounits 2>&1 | head -1)
  IFS=',' read -r name drv free <<< "$line"
  free=${free// /}
  [ -n "$free" ] && [ "$free" -eq "$free" ] 2>/dev/null || { echo "nvidia-smi: $line"; return 1; }
  echo "${name# } driver${drv} libera ${free} MiB"
  case "$name" in *"$want"*) ;; *) return 1 ;; esac
  [ "$free" -ge "$min" ] || return 1
}

probe_disk() {
  local p=$1 min=$2 free
  free=$(df -P -BG "$p" 2>/dev/null | awk 'NR==2 {sub("G", "", $4); print $4}')
  [ -n "$free" ] || { echo "df $p fallito"; return 1; }
  echo "$p: $free GB liberi (min $min)"
  [ "$free" -ge "$min" ] || return 1
}

probe_docker_image() {
  local id
  id=$(docker image inspect -f '{{.Id}}' "$1" 2>/dev/null) || { echo "$1 assente (docker image ls)"; return 1; }
  echo "$1 ${id:7:12}"
}

probe_clock_offset() {
  local -a o; mapfile -t o < <(_orin_opts)
  python3 "$F1T_CORE/lib/clock_offset.py" --warn-ms "$F1T_CLOCK_WARN_MS" --fail-ms "$F1T_CLOCK_FAIL_MS" \
    -- ssh "${o[@]}" "$(orin_target)"
}

probe_git_clean() {
  local d=${1:-$F1T_ROOT} n
  n=$(git -C "$d" status --porcelain --untracked-files=no --ignore-submodules=all | wc -l)
  [ "$n" = 0 ] || { echo "$n file tracciati modificati"; return 1; }
  echo "pulito ($(git -C "$d" rev-parse --abbrev-ref HEAD) $(git -C "$d" rev-parse --short HEAD))"
}

# ORIN[chiave]=valore dai fatti dell'Orin (orin/remote/facts.sh). Ritorna 1 se l'Orin non risponde.
# shellcheck disable=SC2034  # letta da chi fa source
declare -gA ORIN=()
orin_facts() {
  local out k v
  out=$(orin_script "$F1T_CORE/orin/remote/facts.sh" "$F1T_ORIN_CONTAINER" "$F1T_ORIN_UNIT" \
    "$F1T_ORIN_PIN_FILE" "$F1T_ORIN_DEPLOYED_JSON" "$F1T_ORIN_DEPLOYED_JSON_FALLBACK" \
    "$F1T_ORIN_BUILD_REPO" "$F1T_ORIN_NATIVE_WS" "$F1T_ORIN_OVERRIDE_DIR" 2>&1) || {
    ORIN[error]=$(printf '%s\n' "$out" | tail -1); return 1; }
  while IFS='=' read -r k v; do [ -n "$k" ] && ORIN[$k]=$v; done <<< "$out"
  return 0
}
