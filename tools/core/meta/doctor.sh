#!/usr/bin/env bash
# desc: salute del tooling: tools/core uguale a origin/main, hook installato, config valida, strumenti
# alias: doctor
#
#   f1t doctor [--offline]
#
# tools/core si modifica solo su main: qui si confronta con F1T_MAIN_REF (VERSION + hash della
# cartella). Se e' indietro: WARN con il comando `f1t core sync`. --offline salta l'ssh all'Orin.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"
OFFLINE=0
for a in "$@"; do
  case "$a" in
    --offline) OFFLINE=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $a" 2 ;;
  esac
done
cfg_load
G=(git -C "$F1T_ROOT")
BRANCH=$("${G[@]}" rev-parse --abbrev-ref HEAD)
F1T_CORE_VERSION=$(cat "$F1T_CORE/VERSION")
cfg_print F1T_CORE_VERSION F1T_MAIN_REF F1T_CORE_BRANCH
log_steps 3

log_step "tools/core rispetto a $F1T_MAIN_REF"
core_state() {
  local here main mver
  if ! "${G[@]}" rev-parse -q --verify "$F1T_MAIN_REF:tools/core" >/dev/null; then
    echo "$F1T_MAIN_REF non ha tools/core (non ancora pushato su main?)"; return 10
  fi
  mver=$("${G[@]}" show "$F1T_MAIN_REF:tools/core/VERSION" 2>/dev/null || echo '?')
  if ! "${G[@]}" diff --quiet HEAD -- tools/core tools/f1t 2>/dev/null || [ -n "$("${G[@]}" ls-files --others --exclude-standard -- tools/core)" ]; then
    echo "modifiche non committate in tools/core (v$F1T_CORE_VERSION, main v$mver)"; return 10
  fi
  here=$("${G[@]}" rev-parse HEAD:tools/core 2>/dev/null || echo none)
  main=$("${G[@]}" rev-parse "$F1T_MAIN_REF:tools/core")
  if [ "$here" = "$main" ]; then echo "uguale a $F1T_MAIN_REF (v$F1T_CORE_VERSION)"; return 0; fi
  if [ "$BRANCH" = "$F1T_CORE_BRANCH" ]; then
    echo "$BRANCH locale v$F1T_CORE_VERSION diverso da $F1T_MAIN_REF v$mver: git push (o pull) di $BRANCH"; return 10
  fi
  if [ "$(printf '%s\n%s\n' "$F1T_CORE_VERSION" "$mver" | sort -V | head -1)" = "$F1T_CORE_VERSION" ] && [ "$F1T_CORE_VERSION" != "$mver" ]; then
    echo "indietro: v$F1T_CORE_VERSION, main v$mver -> f1t core sync"; return 10
  fi
  echo "diverso da main (v$F1T_CORE_VERSION, main v$mver): modificato su $BRANCH? riportalo su main; poi f1t core sync"; return 10
}
check "tools/core = $F1T_MAIN_REF" core_state

log_step "Installazione"
hook_state() {
  local h; h=$("${G[@]}" rev-parse --git-common-dir)/hooks/pre-commit
  [ -f "$h" ] && grep -q 'f1t-pre-commit' "$h" || { echo "hook pre-commit non installato: f1t setup"; return 10; }
  echo "installato"
}
check "hook pre-commit" hook_state
launcher_state() {
  local p; p=$(command -v f1t 2>/dev/null) || { echo "f1t non e' nel PATH: f1t setup (o usa tools/f1t)"; return 10; }
  echo "$p"
}
check "f1t nel PATH" launcher_state
check "config.env valida" cfg_require config

log_step "Strumenti"
# shellcheck disable=SC2016  # espanso dalla bash figlia
check "bash >= 4.3" bash -c '[ "${BASH_VERSINFO[0]}${BASH_VERSINFO[1]}" -ge 43 ] && echo "$BASH_VERSION"'
check "python3" bash -c 'python3 --version'
check "git" bash -c 'git --version'
check_warn "docker" bash -c 'docker version --format "{{.Client.Version}}" 2>/dev/null || { echo "assente"; exit 1; }'
check_warn "shellcheck" bash -c 'command -v shellcheck >/dev/null && shellcheck --version | sed -n "s/^version: //p" || { echo "assente: python3 -m venv <dir> && <dir>/bin/pip install shellcheck-py"; exit 1; }'
check_warn "ROS 2 Jazzy (HIL)" bash -c '[ -f /opt/ros/jazzy/setup.bash ] && echo /opt/ros/jazzy || { echo "assente: servono solo f1t hil"; exit 1; }'
if [ "$OFFLINE" = 0 ]; then
  check_warn "ssh Orin ($(orin_target))" bash -c "$(declare -f _orin_opts orin_target orin_ssh); F1T_ORIN_SSH_OPTS='$F1T_ORIN_SSH_OPTS' F1T_ORIN_USER='$F1T_ORIN_USER' F1T_ORIN_HOST='$F1T_ORIN_HOST' orin_ssh hostname"
fi
summary "f1t doctor"
