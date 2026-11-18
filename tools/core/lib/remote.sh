# shellcheck shell=bash
# tools/core/lib/remote.sh - accesso all'Orin da linus (dopo cfg_load).
#
#   orin_ssh CMD...                comando sull'Orin (connessione condivisa: ControlMaster, 60 s)
#   orin_script FILE [ARGS...]     esegue uno script LOCALE sull'Orin (ssh bash -s < FILE): il lavoro
#                                  remoto passa sempre da uno script, mai da un here-doc dentro ssh '...'
#   orin_up                        0 se l'Orin risponde via ssh entro F1T_ORIN_SSH_OPTS ConnectTimeout
#   orin_scp SRC DEST              copia verso l'Orin (DEST relativo alla home o assoluto)
# Gli script remoti stanno in tools/core/*/remote/ e iniziano con set -euo pipefail.
[ -n "${_F1T_REMOTE_SH:-}" ] && return 0
_F1T_REMOTE_SH=1

_orin_opts() {
  local -a o
  read -r -a o <<< "${F1T_ORIN_SSH_OPTS:-}"
  o+=(-o ControlMaster=auto -o "ControlPath=${XDG_RUNTIME_DIR:-/tmp}/f1t-ssh-%C" -o ControlPersist=60)
  printf '%s\n' "${o[@]}"
}
orin_target() { echo "${F1T_ORIN_USER}@${F1T_ORIN_HOST}"; }

orin_ssh() {
  local -a o; mapfile -t o < <(_orin_opts)
  # shellcheck disable=SC2029  # il comando si espande sull'Orin, apposta
  ssh "${o[@]}" "$(orin_target)" "$@"
}

orin_script() {
  local file=$1; shift
  local -a o; mapfile -t o < <(_orin_opts)
  local q='' a
  for a in "$@"; do q+=" $(printf '%q' "$a")"; done
  # shellcheck disable=SC2029  # the quoted arguments are meant to expand on the Orin
  ssh "${o[@]}" "$(orin_target)" "bash -s --$q" < "$file"
}

orin_scp() {
  local -a o; mapfile -t o < <(_orin_opts)
  scp -q "${o[@]}" "$1" "$(orin_target):$2"
}

orin_up() { orin_ssh true >/dev/null 2>&1; }
