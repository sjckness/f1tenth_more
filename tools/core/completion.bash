# shellcheck shell=bash
# Completamento bash di f1t: source tools/core/completion.bash (f1t setup lo spiega).
_f1t_complete() {
  local cur=${COMP_WORDS[COMP_CWORD]}
  local words=("${COMP_WORDS[@]:1:COMP_CWORD-1}")
  local f1t=${COMP_WORDS[0]}
  command -v "$f1t" >/dev/null 2>&1 || f1t=f1t
  mapfile -t COMPREPLY < <(compgen -W "$("$f1t" __complete "${words[@]}" 2>/dev/null)" -- "$cur")
}
complete -F _f1t_complete f1t tools/f1t ./tools/f1t
