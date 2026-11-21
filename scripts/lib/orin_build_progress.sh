#!/usr/bin/env bash
# Gira SULL'ORIN (ssh bash -s < questo, da scripts/orin_deploy.sh): a che punto e' una build di f1t.
#   orin_build_progress.sh ROOT SHORT
# Stampa una riga: STATO|FASE|K|M|ETA|TESTO
#   STATO  running | ok | fail ... | none        (ROOT/SHORT.status scritto dal job di f1t build)
#   FASE   build | test | label | fine
#   K/M    passo del Dockerfile in corso ([stage K/M] di BuildKit), 0/0 fuori dalla build
#   ETA    secondi dall'ultima scrittura nei log della build (job e docker): una build viva scrive
set -euo pipefail
root=$1 short=$2
st=$(cat "$root/$short.status" 2>/dev/null || echo none)
job=$(tail -n 1 "$root/$short.log" 2>/dev/null | cut -c1-200 || true)
phase=build k=0 m=0 text=$job dlog=
case "$job" in
  *pytest*|"PKG "*|"    "*|*"test dei package"*|*"test di tutti"*|*"test: "*) phase="test" ;;
  *label*) phase=label ;;
esac
case "$st" in ok*|fail*) phase=fine ;; esac
if [ "$phase" = build ]; then
  # shellcheck disable=SC2012  # nomi build_<sha>_<data>.log, senza spazi
  dlog=$(ls -t "$root/$short"/output/orin_container/logs/build_*.log 2>/dev/null | head -1 || true)
  if [ -n "$dlog" ]; then
    hdr=$(grep -E '^#[0-9]+ \[[^]]* [0-9]+/[0-9]+\]' "$dlog" | tail -n 1 || true)
    if [[ $hdr =~ \[([^]]*\ )?([0-9]+)/([0-9]+)\] ]]; then k=${BASH_REMATCH[2]} m=${BASH_REMATCH[3]}; fi
    text=$(printf '%s' "${hdr:-$(tail -n 1 "$dlog")}" | tr -s ' ' | cut -c1-160)
  fi
fi
now=$(date +%s) last=0
for f in "$root/$short.log" ${dlog:+"$dlog"}; do
  t=$(stat -c %Y "$f" 2>/dev/null || echo 0); [ "$t" -le "$last" ] || last=$t
done
age=$(( last > 0 ? now - last : -1 ))
printf '%s|%s|%s|%s|%s|%s\n' "$st" "$phase" "$k" "$m" "$age" "$text"
