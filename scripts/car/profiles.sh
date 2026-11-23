#!/usr/bin/env bash
# scripts/car/profiles.sh - da linus: i profili di lancio sull'Orin.
#
#   scripts/car/profiles.sh push [--allow-dirty]   copia i profili del repo sull'Orin
#   scripts/car/profiles.sh diff                   quali profili differiscono (linus vs Orin)
#   scripts/car/profiles.sh list                   i profili sull'Orin, con sha256
#
# Sorgente: src/f1tenth_bringup/config/profiles/<nome>.yaml (formato e regole:
# src/f1tenth_bringup/f1tenth_bringup/launch_profile.py). Destinazione: ~/.config/f1tenth_car/profiles
# sull'Orin, montata in sola lettura nel container e riletta a ogni avvio dello stack: per usare un
# profilo cambiato basta `car-stack restart <nome>`, niente rebuild ne' riavvio del container.
# push e' rsync --delete: sull'Orin restano esattamente i profili del repo. Rifiuta profili con
# modifiche non committate (lo stack.log registra lo sha256 del profilo usato: deve esistere in git),
# salvo --allow-dirty.
# Exit: 0 OK (diff: nessuna differenza), 1 fallito o diversi, 2 argomenti.
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=car_profiles
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"
# shellcheck source=scripts/car/lib/orin_common.sh
source "$ROOT/scripts/car/lib/orin_common.sh"

CMD=${1:-} DIRTY=0
[ $# -gt 0 ] && shift
for a in "$@"; do
  case "$a" in
    --allow-dirty) DIRTY=1 ;;
    *) log_die "opzione sconosciuta: $a (scripts/car/profiles.sh -h)" 2 ;;
  esac
done
case "$CMD" in
  push|diff|list) ;;
  -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
  *) log_die "uso: scripts/car/profiles.sh push|diff|list (-h)" 2 ;;
esac
cfg_load
orin_up || log_die "Orin irraggiungibile: ssh $(orin_target)"

show_diff() {
  local out
  out=$(join -a1 -a2 -e - -o 0,1.2,2.2 <(car_profiles_sums | sort) <(car_profiles_remote_sums | sort) \
    | awk '{ st = ($2 == $3) ? "uguale" : ($3 == "-" ? "solo su linus" : ($2 == "-" ? "solo sull Orin" : "DIVERSO"));
             printf "  %-28s %-14s linus %.12s  Orin %.12s\n", $1, st, $2, $3; if (st != "uguale") d = 1 }
           END { exit d }') && { printf '%s\n' "$out" >&2; return 0; }
  printf '%s\n' "$out" >&2; return 1
}

case "$CMD" in
  list) car_profiles_remote_sums | sed 's/^/  /'; exit 0 ;;
  diff) show_diff; exit ;;
esac

dirty=$(git -C "$ROOT" status --porcelain -- "$CAR_PROFILES_SRC")
if [ -n "$dirty" ]; then
  [ "$DIRTY" = 1 ] || log_die "profili con modifiche non committate (committa, o --allow-dirty):
$dirty"
  log_warn "profili non committati copiati (--allow-dirty): $(tr '\n' ' ' <<<"$dirty")"
fi
car_profiles_push || log_die "rsync verso $(orin_target):~/$CAR_PROFILES_DIR fallito"
if show_diff; then log_ok "profili sull'Orin == linus ($(car_profiles_sums | wc -l))"
else log_die "dopo la copia i profili differiscono ancora"; fi
