# shellcheck shell=bash
# scripts/car/lib/orin_ui.sh - output degli script di scripts/car/ su linus: barre di avanzamento,
# tempi per passo, comandi stampati prima di eseguirli, coppie chiave/valore. Dopo tools/core/lib/log.sh
# (usa log_*, _log_to_file): tutto va su stderr e nel log logs/tools/<script>_<data>.log.
#
#   ui_steps N                      N passi in tutto (barra generale)
#   ui_step "testo"                 nuovo passo: barra generale, numero, tempo totale
#   ui_step_end                     tempo del passo
#   ui_bar CUR TOT "testo"          barra (sul terminale si riscrive sulla stessa riga)
#   ui_bar_done                     chiude una barra che si riscriveva
#   ui_spin T0 "testo"              riga di attesa con tempo trascorso (attese senza totale)
#   ui_run CMD...                   stampa "$ CMD..." poi lo esegue
#   ui_kv CHIAVE VALORE             riga "  chiave ........ valore"
#   ui_section "titolo"             intestazione di blocco
[ -n "${_ORIN_UI_SH:-}" ] && return 0
_ORIN_UI_SH=1
UI_T0=$(date +%s) UI_STEP=0 UI_STEPS=1 UI_STEP_T0=$(date +%s) _UI_LIVE=0 _UI_SPIN=0
UI_W=${UI_W:-32}

ui_elapsed() {
  local s=$(( $(date +%s) - ${1:-$UI_T0} ))
  printf '%02d:%02d:%02d' $((s / 3600)) $((s % 3600 / 60)) $((s % 60))
}
_ui_cols() { local c; c=$(tput cols 2>/dev/null || echo 120); echo "${c:-120}"; }

ui_bar() {
  local cur=$1 tot=$2 text=$3 n pct fill line
  [ "$tot" -gt 0 ] 2>/dev/null || tot=1
  [ "$cur" -le "$tot" ] 2>/dev/null || cur=$tot
  n=$(( cur * UI_W / tot )); pct=$(( cur * 100 / tot ))
  fill="$(printf '%*s' "$n" '' | tr ' ' '#')$(printf '%*s' $(( UI_W - n )) '' | tr ' ' '.')"
  line=$(printf '[%s] %3d%% %s' "$fill" "$pct" "$text")
  if [ -t 2 ]; then
    printf '\r\033[K%s' "${line:0:$(( $(_ui_cols) - 1 ))}" >&2; _UI_LIVE=1
  else
    printf '%s\n' "$line" >&2
  fi
  _log_to_file "$line"
}
ui_bar_done() { if [ "$_UI_LIVE" = 1 ]; then printf '\n' >&2; _UI_LIVE=0; fi; }

ui_spin() {
  local t0=$1 text=$2 c='|/-+' line
  _UI_SPIN=$(( (_UI_SPIN + 1) % 4 ))
  line="[${c:$_UI_SPIN:1}] $(ui_elapsed "$t0") $text"
  if [ -t 2 ]; then printf '\r\033[K%s' "${line:0:$(( $(_ui_cols) - 1 ))}" >&2; _UI_LIVE=1
  else printf '%s\n' "$line" >&2; fi
  _log_to_file "$line"
}

ui_steps() { UI_STEPS=$1; UI_STEP=0; log_steps "$1"; }
ui_step() {
  ui_bar_done
  UI_STEP=$(( UI_STEP + 1 )); UI_STEP_T0=$(date +%s)
  printf '\n' >&2
  ui_bar $(( UI_STEP - 1 )) "$UI_STEPS" "totale: passo $UI_STEP di $UI_STEPS, trascorso $(ui_elapsed)"
  ui_bar_done
  log_step "$*"
}
ui_step_end() { ui_bar_done; log_ok "passo $UI_STEP/$UI_STEPS finito in $(ui_elapsed "$UI_STEP_T0")"; }

ui_run() { ui_bar_done; log_info "\$ $*"; "$@"; }
ui_kv() { printf '  %-30s %s\n' "$1" "$2" >&2; _log_to_file "  $1: $2"; }
ui_section() { ui_bar_done; printf '\n== %s\n' "$*" >&2; _log_to_file "== $*"; }
