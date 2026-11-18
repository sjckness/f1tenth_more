#!/usr/bin/env bash
# desc: rilascio di un modulo: bump SemVer, voce CHANGELOG (bozza da git log da confermare), commit e tag <pkg>/vX.Y.Z
#
#   f1t release <package> [patch|minor|major|X.Y.Z] [--dry-run] [--yes]      default: patch
#
# 1. versione attuale da package.xml, nuova versione (SemVer: MAJOR rompe topic/parametri/
#    interfacce, MINOR aggiunge, PATCH corregge);
# 2. bozza della voce: il contenuto di [Unreleased] del CHANGELOG.md piu' i commit che toccano il
#    package dall'ultimo tag <pkg>/v<attuale> (git log), da confermare: [s]i / [m]odifica in $EDITOR / [n]o;
# 3. scrive <version> in package.xml (e version= in setup.py, se c'e'), sposta la voce sotto
#    ## [X.Y.Z] - data, committa solo quei file ("release: <pkg> vX.Y.Z") e crea il tag annotato
#    <pkg>/vX.Y.Z. Niente push.
# Rifiuta: modifiche gia' in stage, tag gia' esistente, versione non maggiore dell'attuale.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"

PKG='' BUMP=patch DRY=0 YES=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --yes|-y) YES=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $1 (f1t release -h)" 2 ;;
    *) if [ -z "$PKG" ]; then PKG=$1; else BUMP=$1; fi ;;
  esac
  shift
done
cfg_load
[ -n "$PKG" ] || log_die "uso: f1t release <package> [patch|minor|major|X.Y.Z] [--dry-run] [--yes]" 2
line=$(F1T_ROOT=$F1T_ROOT python3 "$F1T_CORE/lib/modules.py" ref HEAD | awk -v p="$PKG" '$1 == p')
[ -n "$line" ] || log_die "modulo sconosciuto: $PKG (package.xml sotto src/, esclusi i submodule)" 2
read -r _ CUR DIR <<< "$line"
XML=$F1T_ROOT/$DIR/package.xml CL=$F1T_ROOT/$DIR/CHANGELOG.md SETUP=$F1T_ROOT/$DIR/setup.py
cur_wt=$(sed -n 's:.*<version>[[:space:]]*\([^<[:space:]]*\).*:\1:p' "$XML" | head -1)
[ "$cur_wt" = "$CUR" ] || log_die "package.xml modificato e non committato ($CUR in HEAD, $cur_wt nel file): committa prima" 1
NEW=$(python3 - "$CUR" "$BUMP" 2>&1 <<'EOF'
import re, sys
cur, bump = sys.argv[1:3]
m = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', cur)
if not m: sys.exit(f'versione attuale non SemVer: {cur}')
a, b, c = map(int, m.groups())
if bump == 'major': print(f'{a + 1}.0.0')
elif bump == 'minor': print(f'{a}.{b + 1}.0')
elif bump == 'patch': print(f'{a}.{b}.{c + 1}')
elif re.fullmatch(r'\d+\.\d+\.\d+', bump):
    if tuple(map(int, bump.split('.'))) <= (a, b, c): sys.exit(f'{bump} non e\' maggiore di {cur}')
    print(bump)
else: sys.exit(f'bump sconosciuto: {bump} (patch|minor|major|X.Y.Z)')
EOF
) || log_die "$NEW" 2
TAG=$PKG/v$NEW
# shellcheck disable=SC2034  # stampate da cfg_print (lettura indiretta ${!k})
F1T_RELEASE="$PKG $CUR -> $NEW ($DIR)"
# shellcheck disable=SC2034  # stampata da cfg_print (lettura indiretta)
F1T_RELEASE_BRANCH=$(git -C "$F1T_ROOT" rev-parse --abbrev-ref HEAD)
cfg_print F1T_RELEASE F1T_RELEASE_BRANCH
git -C "$F1T_ROOT" rev-parse -q --verify "refs/tags/$TAG" >/dev/null && log_die "il tag $TAG esiste gia'" 1
if ! git -C "$F1T_ROOT" diff --cached --quiet; then
  [ "$DRY" = 1 ] && log_warn "ci sono modifiche in stage: il rilascio vero rifiuterebbe" \
    || log_die "ci sono modifiche in stage: il commit di release deve contenere solo il rilascio" 1
fi
[ -f "$CL" ] || log_die "manca $DIR/CHANGELOG.md (formato: docs/modules/README.md)" 1

since=$PKG/v$CUR
if git -C "$F1T_ROOT" rev-parse -q --verify "refs/tags/$since" >/dev/null; then range="$since..HEAD"
else range=$(git -C "$F1T_ROOT" log -1 --format=%H -- "$CL")..HEAD; log_warn "tag $since assente: commit dall'ultima modifica del CHANGELOG"; fi
draft=$(mktemp); trap 'rm -f "$draft"' EXIT
python3 - "$CL" > "$draft" <<'EOF'
import re, sys
text = open(sys.argv[1], encoding='utf-8').read()
m = re.search(r'^## \[Unreleased\]\s*\n(.*?)(?=^## \[)', text, re.S | re.M)
body = (m.group(1).strip() if m else '')
print(body)
EOF
# shellcheck disable=SC2016  # i backtick sono testo del formato di git log
log_mail=$(git -C "$F1T_ROOT" log --no-merges --format='- %s (`%h`)' "$range" -- "$F1T_ROOT/$DIR" | grep -v -E '^- (chore: analisi spostate|release: )' || true)
has_body=0; grep -q . "$draft" && has_body=1
{
  [ "$has_body" = 1 ] || echo "### Changed"
  [ -z "$log_mail" ] || { echo; echo "<!-- bozza da git log $range: riscrivi in frasi, togli cio' che non conta -->"; printf '%s\n' "$log_mail"; }
} >> "$draft"
echo "---- voce proposta per ## [$NEW] - $(date +%Y-%m-%d) ($DIR/CHANGELOG.md)"
cat "$draft"
echo "----"
if [ "$DRY" = 1 ]; then log_warn "DRY-RUN: nessun file cambiato, nessun commit, nessun tag ($TAG)"; exit 0; fi
if [ "$YES" = 0 ]; then
  [ -t 0 ] || log_die "serve conferma: lancia da terminale o con --yes" 1
  while :; do
    read -r -p "Confermi la voce? [s]i / [m]odifica in \$EDITOR / [n]o: " ans
    case "$ans" in
      s|S|y|Y) break ;;
      m|M) "${EDITOR:-nano}" "$draft"; cat "$draft" ;;
      *) log_warn "annullato"; exit 1 ;;
    esac
  done
fi
python3 - "$CL" "$draft" "$NEW" "$(date +%Y-%m-%d)" <<'EOF'
import re, sys
cl, draft, new, date = sys.argv[1:5]
text = open(cl, encoding='utf-8').read()
body = open(draft, encoding='utf-8').read().strip()
body = re.sub(r'<!--.*?-->\n?', '', body, flags=re.S).strip()
section = f'## [Unreleased]\n\n## [{new}] - {date}\n\n{body}\n\n'
new_text, n = re.subn(r'^## \[Unreleased\]\s*\n.*?(?=^## \[)', section, text, count=1, flags=re.S | re.M)
if n == 0:
    sys.exit('nel CHANGELOG manca "## [Unreleased]" seguito da una versione')
open(cl, 'w', encoding='utf-8').write(new_text)
EOF
sed -i "0,/<version>[^<]*<\/version>/s//<version>$NEW<\/version>/" "$XML"
files=("$XML" "$CL")
if [ -f "$SETUP" ] && grep -q "version='[^']*'" "$SETUP"; then
  sed -i "0,/version='[^']*'/s//version='$NEW'/" "$SETUP"; files+=("$SETUP")
fi
git -C "$F1T_ROOT" add -- "${files[@]}"
git -C "$F1T_ROOT" commit -q -m "release: $PKG v$NEW" -- "${files[@]}"
git -C "$F1T_ROOT" tag -a "$TAG" -m "$PKG $NEW (vedi $DIR/CHANGELOG.md)"
log_ok "commit $(git -C "$F1T_ROOT" rev-parse --short HEAD) \"release: $PKG v$NEW\", tag $TAG (non pushati)"
log_info "push quando serve: git push origin HEAD && git push origin $TAG"
