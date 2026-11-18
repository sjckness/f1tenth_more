#!/usr/bin/env bash
# f1t - entrypoint unico del tooling (tools/core). Uso: f1t <gruppo> <comando> [args]; f1t help.
#
# I comandi si scoprono da soli: ogni script eseguibile in tools/core/<gruppo>/ con una riga
# "# desc: ..." nelle prime righe e' il comando "f1t <gruppo> <nome>" (_ diventa -; se il nome e'
# uguale al gruppo basta "f1t <gruppo>"). Righe "# alias: parole" aggiungono altri nomi.
# Le estensioni di branch: f1t ext <nome> <comando> -> tools/ext/<nome>/<comando>.(sh|py).
set -euo pipefail
CORE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
TOOLS=$(dirname "$CORE")
EXT=$TOOLS/ext
F1T_CORE=$CORE
F1T_ROOT=$(dirname "$TOOLS")
export F1T_CORE F1T_ROOT
VERSION=$(cat "$CORE/VERSION" 2>/dev/null || echo '?')

# parole<TAB>file<TAB>descrizione<TAB>1 (nome) | 0 (alias)
index() {
  local f g n desc a
  for f in "$CORE"/*/*; do
    [ -f "$f" ] && [ -x "$f" ] || continue
    desc=$(sed -n '2,12s/^# desc: //p' "$f" | head -1)
    [ -n "$desc" ] || continue
    g=$(basename "$(dirname "$f")"); n=$(basename "$f"); n=${n%.*}; n=${n//_/-}
    if [ "$n" = "$g" ]; then printf '%s\t%s\t%s\t1\n' "$g" "$f" "$desc"
    else printf '%s %s\t%s\t%s\t1\n' "$g" "$n" "$f" "$desc"; fi
    while IFS= read -r a; do
      [ -z "$a" ] || printf '%s\t%s\t%s\t0\n' "$a" "$f" "$desc"
    done < <(sed -n '2,12s/^# alias: //p' "$f")
  done
}

ext_names() { local d; for d in "$EXT"/*/; do [ -d "$d" ] && basename "$d"; done; }

usage() {
  echo "f1t <comando> [args]   tooling f1tenth, tools/core v$VERSION (tools/core/README.md)"
  echo
  # una riga per script: il nome (gruppo comando) e gli alias, poi la descrizione
  index | awk -F'\t' '
    { if (!($2 in names)) { order[++n] = $2; desc[$2] = $3 }
      names[$2] = ($4 == "1") ? $1 (names[$2] == "" ? "" : " | " names[$2]) : (names[$2] == "" ? $1 : names[$2] " | " $1) }
    END { for (i = 1; i <= n; i++) printf "  %-30s %s\n", names[order[i]], desc[order[i]] }' | sort
  printf '  %-30s %s\n' "ext <nome> <cmd>" "comandi di branch in tools/ext/ ($(ext_names | tr '\n' ' '))"
  printf '  %-30s %s\n' "help" "questo elenco"
  echo
  echo "Ogni comando accetta -h. Log: logs/tools/. Config: tools/core/config.env (+ ~/.config/f1t/config.env)."
}

# Parole candidate dopo quelle date (completamento bash).
complete_words() {
  local prefix="$*" last=${*: -1}
  case "$last" in
    -disp) for f in "$CORE"/hosts/*.env; do basename "$f" .env; done; return ;;
    --scenario) for f in "$CORE"/hil/scenarios/*.env; do f=$(basename "$f" .env); [ "${f#_}" = "$f" ] && echo "$f"; done; return ;;
  esac
  if [ "${1:-}" = ext ]; then
    if [ $# -eq 1 ]; then ext_names; elif [ $# -eq 2 ]; then
      for f in "$EXT/$2"/*.sh "$EXT/$2"/*.py; do [ -x "$f" ] && { f=$(basename "$f"); echo "${f%.*}"; }; done
    fi
    return
  fi
  case "$prefix" in
    "hil run"*|"hil up"*) for f in "$CORE"/hil/scenarios/*.env; do f=$(basename "$f" .env); [ "${f#_}" = "$f" ] && echo "$f"; done; echo -disp; return ;;
    "hil precheck"*|"hil down"*) echo -disp; return ;;
  esac
  index | cut -f1 | awk -v p="$prefix" -v n=$# '
    { k = split($0, w, " ") }
    n == 0 { print w[1]; next }
    k > n && index($0, p " ") == 1 { print w[n + 1] }' | sort -u
  [ $# -gt 0 ] || { echo help; echo ext; }
}

run_ext() {
  local name=${1:-} cmd=${2:-} f
  if [ -z "$name" ]; then
    echo "estensioni in tools/ext/ (f1t ext <nome> <comando>):"
    for f in $(ext_names); do printf '  %-18s %s\n' "$f" "$(sed -n '1{s/^#* *//;p}' "$EXT/$f/README.md" 2>/dev/null)"; done
    return 0
  fi
  [ -d "$EXT/$name" ] || { echo "f1t: estensione sconosciuta '$name' ($(ext_names | tr '\n' ' '))" >&2; return 2; }
  for f in "$EXT/$name/$cmd" "$EXT/$name/${cmd//-/_}.sh" "$EXT/$name/${cmd//-/_}.py" "$EXT/$name/$cmd.sh" "$EXT/$name/$cmd.py"; do
    if [ -n "$cmd" ] && [ -f "$f" ] && [ -x "$f" ]; then
      shift 2
      F1T_SCRIPT="ext_${name}_$(basename "${f%.*}")" exec "$f" "$@"
    fi
  done
  echo "f1t: in tools/ext/$name non c'e' il comando '${cmd}'. Comandi:" >&2
  for f in "$EXT/$name"/*.sh "$EXT/$name"/*.py; do [ -x "$f" ] && echo "  $(basename "${f%.*}")" >&2; done
  return 2
}

main() {
  case "${1:-help}" in
    help|-h|--help) usage; return 0 ;;
    --version|version) echo "tools/core $VERSION"; return 0 ;;
    __complete) shift; complete_words "$@"; return 0 ;;
    __index) index; return 0 ;;
    ext) shift; run_ext "$@"; return $? ;;
  esac
  local n words line file
  for n in 3 2 1; do
    [ $# -ge "$n" ] || continue
    words="${*:1:$n}"
    line=$(index | awk -F'\t' -v w="$words" '$1 == w { print; exit }')
    if [ -n "$line" ]; then
      file=$(printf '%s\n' "$line" | cut -f2)
      shift "$n"
      F1T_SCRIPT=$(basename "$(dirname "$file")")_$(basename "${file%.*}")
      export F1T_SCRIPT
      exec "$file" "$@"
    fi
  done
  echo "f1t: comando sconosciuto '$*'" >&2
  echo "     f1t help elenca i comandi" >&2
  return 2
}

main "$@"
