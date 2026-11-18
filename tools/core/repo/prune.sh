#!/usr/bin/env bash
# desc: pulizia di un clone (anche su un'altra macchina): branch locali gia' su GitHub e worktree inutili
#
#   f1t repo prune [--repo DIR] [--apply] [--rescue DIR] [--keep b1,b2] [--no-fetch]
#
# Lavora sul clone DIR (default: quello della cartella corrente), non per forza su questo checkout.
# 1. git fetch --prune --tags origin.
# 2. Branch locali: si cancella (tag archive/<nome> se manca, poi branch -D) un branch che NON e' piu'
#    su GitHub (origin/<nome> assente) e i cui commit sono tutti gia' su GitHub (in un branch di origin o
#    in un tag archive/*). Restano sempre: main, campaign-final, il branch del checkout principale,
#    --keep, i branch ancora su GitHub e quelli con commit non pushati (elencati: pushali prima).
# 3. Worktree (escluso il checkout principale): si rimuove quello il cui branch si cancella o il cui
#    HEAD staccato e' gia' su GitHub, se non ha modifiche, file non tracciati, dati ignorati che non
#    siano build/cache, o modifiche nei submodule. Con --rescue DIR i file fuori da git (non tracciati e
#    ignorati non rigenerabili) vengono SPOSTATI in DIR/<nome worktree>/ (stesso percorso relativo) e le
#    modifiche salvate in uncommitted.patch; poi il worktree si rimuove.
# Senza --apply non cambia niente: stampa cosa farebbe. Uso su un'altra macchina (senza tools/ in main):
#   git -C ~/dev_ws/f1tenth_more fetch origin && mkdir -p /tmp/f1t \
#     && git -C ~/dev_ws/f1tenth_more archive origin/main tools | tar -x -C /tmp/f1t \
#     && /tmp/f1t/tools/f1t repo prune --repo ~/dev_ws/f1tenth_more
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"

REPO_ARG='' APPLY=0 RESCUE='' KEEP='' FETCH=1
while [ $# -gt 0 ]; do
  case "$1" in
    --repo) REPO_ARG=${2:?--repo DIR}; shift ;;
    --apply) APPLY=1 ;;
    --rescue) RESCUE=${2:?--rescue DIR}; shift ;;
    --keep) KEEP=${2:?--keep b1,b2}; shift ;;
    --no-fetch) FETCH=0 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $1 (f1t repo prune -h)" 2 ;;
  esac
  shift
done
cfg_load
REPO=$(git -C "${REPO_ARG:-.}" rev-parse --show-toplevel 2>/dev/null) || log_die "non e' un repo git: ${REPO_ARG:-$PWD}" 2
G=(git -C "$REPO")
MAINWT=$("${G[@]}" worktree list --porcelain | awk 'NR==1 {print substr($0, 10)}')
CUR=$(git -C "$MAINWT" symbolic-ref --short -q HEAD || echo '')
[ -z "$RESCUE" ] || { mkdir -p "$RESCUE"; RESCUE=$(cd "$RESCUE" && pwd); }
# shellcheck disable=SC2034  # stampate da cfg_print
F1T_PRUNE_REPO="$MAINWT (checkout principale su ${CUR:-HEAD staccato})" F1T_PRUNE_MODE=$( [ "$APPLY" = 1 ] && echo APPLY || echo "dry-run (--apply per agire)")
cfg_print F1T_PRUNE_REPO F1T_PRUNE_MODE RESCUE KEEP
"${G[@]}" remote get-url origin >/dev/null 2>&1 || log_die "il repo non ha il remoto origin" 1
REGEN='^(build|install|log|logs)/$|(^|/)__pycache__/$|\.pyc$|(^|/)\.pytest_cache/$|^\.cargo/$|(^|/)target/$|\.egg-info/$|(^|/)node_modules/$|^apps/pwa/dist/$|(^|/)test-results/$'
log_steps 3

log_step "git fetch --prune origin"
if [ "$FETCH" = 1 ]; then
  GIT_SSH_COMMAND=${GIT_SSH_COMMAND:-"ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4"} \
    timeout 600 "${G[@]}" fetch -q --prune --tags origin || log_die "fetch fallito: niente da decidere senza origin aggiornato" 1
  check_record OK "fetch origin" "$("${G[@]}" for-each-ref --format=x refs/remotes/origin | wc -l) ref remoti"
else
  check_skip "fetch origin" "--no-fetch"
fi

# 0 se tutti i commit di $1 sono su GitHub (branch di origin o tag archive/*).
on_github() { [ "$("${G[@]}" rev-list --count "$1" --not --remotes=origin --tags='archive/*')" = 0 ]; }
protected() { case ",main,campaign-final,$CUR,$KEEP," in *",$1,"*) return 0 ;; esac; return 1; }

log_step "Branch locali"
declare -A DEL=()
while IFS= read -r b; do
  if protected "$b"; then check_record OK "$b" "tenuto (protetto)"; continue; fi
  if "${G[@]}" rev-parse -q --verify "refs/remotes/origin/$b" >/dev/null; then
    n=$("${G[@]}" rev-list --count "origin/$b..$b")
    if [ "$n" = 0 ]; then check_record OK "$b" "tenuto: ancora su GitHub"
    else check_record WARN "$b" "tenuto: $n commit non pushati su origin/$b"; fi
    continue
  fi
  if on_github "$b"; then
    DEL[$b]=1
    check_record OK "$b" "da cancellare: non piu' su GitHub, commit tutti su GitHub"
  else
    check_record WARN "$b" "tenuto: $("${G[@]}" rev-list --count "$b" --not --remotes=origin --tags='archive/*') commit solo qui (pushali o archiviali)"
  fi
done < <("${G[@]}" for-each-ref --format='%(refname:short)' refs/heads)

log_step "Worktree"
rescue_wt() {  # sposta i file fuori da git di $1 in $RESCUE/<nome>; stampa quanti
  local w=$1 name out n=0 rel
  name=$(echo "${w#"$HOME"/}" | tr '/' '_'); out=$RESCUE/$name
  if [ "$(git -C "$w" status --porcelain --untracked-files=no --ignore-submodules=all | wc -l)" != 0 ]; then
    mkdir -p "$out"; git -C "$w" diff HEAD > "$out/uncommitted.patch"; git -C "$w" rev-parse HEAD > "$out/uncommitted.patch.base"
  fi
  while IFS= read -r rel; do
    rel=${rel%/}; [ -n "$rel" ] && [ -e "$w/$rel" ] || continue
    mkdir -p "$out/$(dirname "$rel")"; mv -n -- "$w/$rel" "$out/$rel" && n=$((n + 1))
  done < <({ git -C "$w" status --porcelain --untracked-files=all --ignore-submodules=all | sed -n 's/^?? //p'
             git -C "$w" status --porcelain --ignored --untracked-files=normal --ignore-submodules=all | sed -n 's/^!! //p' | grep -v -E "$REGEN"; } | sort -u)
  echo "$n"
}
while IFS=$'\t' read -r w head br; do
  [ "$w" != "$MAINWT" ] || continue
  if [ ! -d "$w" ]; then check_record OK "$w" "cartella assente: git worktree prune"; continue; fi
  if [ -n "$br" ]; then
    [ -n "${DEL[$br]:-}" ] || { check_record OK "$w" "tenuto: branch $br resta"; continue; }
  elif ! on_github "$head"; then
    check_record WARN "$w" "tenuto: HEAD staccato ${head:0:8} non su GitHub"; continue
  fi
  mod=$(git -C "$w" status --porcelain --untracked-files=no --ignore-submodules=all | wc -l)
  unt=$(git -C "$w" status --porcelain --ignore-submodules=all | grep -c '^??' || true)
  ign=$(git -C "$w" status --porcelain --ignored --untracked-files=normal --ignore-submodules=all | sed -n 's/^!! //p' | grep -c -v -E "$REGEN" || true)
  sub=$(git -C "$w" submodule foreach --quiet 'git status --porcelain' 2>/dev/null | grep -c . || true)
  what="${br:-HEAD ${head:0:8}}"
  if [ "$sub" != 0 ]; then check_record WARN "$w" "tenuto ($what): $sub modifiche nei submodule"; continue; fi
  if [ "$mod$unt$ign" != 000 ] && [ -z "$RESCUE" ]; then
    check_record WARN "$w" "tenuto ($what): $mod modificati, $unt non tracciati, $ign dati ignorati (--rescue DIR)"; continue
  fi
  if [ "$APPLY" = 0 ]; then
    check_record OK "$w" "da rimuovere ($what)$( [ "$mod$unt$ign" = 000 ] || echo ", prima salvo $mod/$unt/$ign in $RESCUE")"; continue
  fi
  saved=0; [ "$mod$unt$ign" = 000 ] || saved=$(rescue_wt "$w")
  if "${G[@]}" worktree remove --force "$w" 2>/dev/null; then check_record OK "$w" "rimosso ($what)$( [ "$saved" = 0 ] || echo ", $saved file in $RESCUE")"
  else check_record FAIL "$w" "git worktree remove fallito"; fi
done < <("${G[@]}" worktree list --porcelain | awk '
  /^worktree / {w = substr($0, 10); h = ""; b = ""}
  /^HEAD / {h = $2}
  /^branch / {b = $2; sub("refs/heads/", "", b)}
  /^$/ {if (w != "") print w "\t" h "\t" b; w = ""}
  END {if (w != "") print w "\t" h "\t" b}')
"${G[@]}" worktree prune

if [ "$APPLY" = 1 ]; then
  for b in "${!DEL[@]}"; do
    if "${G[@]}" worktree list --porcelain | grep -qx "branch refs/heads/$b"; then
      check_record WARN "$b" "non cancellato: ancora in un worktree tenuto"; continue
    fi
    t=archive/$b
    if ! "${G[@]}" rev-parse -q --verify "refs/tags/$t" >/dev/null; then
      "${G[@]}" tag -a "$t" "$b" -m "Archivio del branch $b (f1t repo prune $(date +%F))"
    elif [ "$("${G[@]}" rev-parse "$t^{commit}")" != "$("${G[@]}" rev-parse "$b")" ]; then
      t=archive/$b-$(date +%Y%m%d); "${G[@]}" rev-parse -q --verify "refs/tags/$t" >/dev/null || "${G[@]}" tag -a "$t" "$b" -m "Archivio del branch $b (f1t repo prune)"
    fi
    "${G[@]}" branch -D "$b" >/dev/null && check_record OK "$b" "cancellato (tag $t, locale)"
  done
fi
summary "f1t repo prune$( [ "$APPLY" = 1 ] || echo ' (dry-run)')"
