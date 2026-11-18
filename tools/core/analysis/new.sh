#!/usr/bin/env bash
# desc: nuova analisi in docs/analysis/<area>/<data>_<slug>/ dal template, con le versioni deployate sull'Orin
# alias: exp new
#
#   f1t exp new <area> <slug> [--title "Titolo"] [--modules pkg1,pkg2] [--no-orin] [--hil-run DIR|last]
#
# Crea docs/analysis/<area>/YYYY-MM-DD_<slug>/report.md da docs/analysis/_template/report.md e compila
# id, data, area, titolo, branch e commit di questo checkout, e modules: con le versioni che girano
# sull'Orin (lette dal container, come f1t orin status; --modules ne tiene solo alcuni). Senza Orin
# (--no-orin o non raggiungibile) usa i package.xml del checkout e lo scrive nel report.
# --hil-run collega un run di f1t hil run (cartella in logs/hil/, o "last"): copia result.json e
# checks.json in data/ e lo cita nel Setup. Non committa: poi f1t exp index e il commit docs-only su main.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/checks.sh
source "$F1T_CORE/lib/checks.sh"

AREA='' SLUG='' TITLE='' MODS='' ORIN_READ=1 HILRUN=''
while [ $# -gt 0 ]; do
  case "$1" in
    --title) TITLE=${2:?}; shift ;;
    --modules) MODS=${2:?}; shift ;;
    --no-orin) ORIN_READ=0 ;;
    --hil-run) HILRUN=${2:?}; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $1 (f1t exp new -h)" 2 ;;
    *) if [ -z "$AREA" ]; then AREA=$1; elif [ -z "$SLUG" ]; then SLUG=$1; else log_die "troppi argomenti: $1" 2; fi ;;
  esac
  shift
done
cfg_load
DB=$F1T_ROOT/docs/analysis
areas=$(find "$DB" -mindepth 1 -maxdepth 1 -type d ! -name '_template' -printf '%f\n' | sort | tr '\n' ' ')
[ -n "$AREA" ] && [ -n "$SLUG" ] || log_die "uso: f1t exp new <area> <slug>   aree: $areas" 2
[ -d "$DB/$AREA" ] && [ "$AREA" != _template ] || log_die "area sconosciuta '$AREA'. Aree: $areas" 2
[[ "$SLUG" =~ ^[a-z0-9]+(-[a-z0-9]+)*$ ]] || log_die "slug '$SLUG': solo minuscole, cifre e trattini (es. wall-lock-replay)" 2
ID=$(date +%Y-%m-%d)_$SLUG
DIR=$DB/$AREA/$ID
[ ! -e "$DIR" ] || log_die "esiste gia': ${DIR#"$F1T_ROOT"/}" 1
BRANCH=$(git -C "$F1T_ROOT" rev-parse --abbrev-ref HEAD)
COMMIT=$(git -C "$F1T_ROOT" rev-parse --short=12 HEAD)
# shellcheck disable=SC2034  # stampate da cfg_print (lettura indiretta ${!k})
F1T_EXP_DIR=${DIR#"$F1T_ROOT"/} F1T_EXP_BRANCH="$BRANCH $COMMIT"
cfg_print F1T_EXP_DIR F1T_EXP_BRANCH F1T_ORIN_HOST

tmp=$(mktemp); trap 'rm -f "$tmp"' EXIT
SRC="checkout ($BRANCH $COMMIT)" IMAGE_NOTE=''
if [ "$ORIN_READ" = 1 ] && orin_facts && [ "${ORIN[ctr_state]:-}" = running ] \
   && orin_script "$F1T_CORE/orin/remote/modules.sh" --running "$F1T_ORIN_CONTAINER" "$F1T_MODULES_EXCLUDE_RE" > "$tmp" 2>/dev/null && [ -s "$tmp" ]; then
  SRC="Orin (${ORIN[ctr_image]}, ${ORIN[ctr_revision]:0:12})"
  IMAGE_NOTE="Immagine sull'Orin alla creazione: \`${ORIN[ctr_image]}\` (commit \`${ORIN[ctr_revision]:0:12}\`, branch \`${ORIN[ctr_branch]:-?}\`)."
  log_ok "versioni dei moduli lette dall'Orin: $SRC"
else
  [ "$ORIN_READ" = 0 ] || log_warn "Orin non letto (${ORIN[error]:-container non in esecuzione}): versioni dal checkout"
  F1T_ROOT=$F1T_ROOT python3 "$F1T_CORE/lib/modules.py" ref HEAD | awk '{print $1, $2}' > "$tmp"
  IMAGE_NOTE="Versioni dei moduli prese dal checkout ($BRANCH \`$COMMIT\`), non dall'Orin."
fi
modules=$(python3 - "$tmp" "$MODS" 2>&1 <<'EOF'
import sys
want = [m for m in sys.argv[2].split(',') if m]
mods = {}
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) >= 2 and (not want or p[0] in want):
        mods[p[0]] = p[1]
missing = [m for m in want if m not in mods]
if missing:
    sys.exit('moduli sconosciuti: ' + ', '.join(missing))
print('{' + ', '.join(f'{k}: "{v}"' for k, v in sorted(mods.items())) + '}')
EOF
) || log_die "$modules" 2

mkdir -p "$DIR"
python3 - "$DB/_template/report.md" "$DIR/report.md" "$ID" "${TITLE:-$SLUG}" "$AREA" "$(date +%Y-%m-%d)" \
  "$modules" "$BRANCH" "$COMMIT" "$IMAGE_NOTE" <<'EOF'
import json, re, sys
src, dst, id_, title, area, date, modules, branch, commit, note = sys.argv[1:11]
text = open(src, encoding='utf-8').read()
def setk(key, value):
    global text
    text = re.sub(rf'^{key}:.*$', lambda m: f'{key}: {value}', text, count=1, flags=re.M)
setk('id', id_)
setk('title', json.dumps(title, ensure_ascii=False))
setk('area', area)
setk('date', date)
setk('modules', modules)
setk('branch', json.dumps(branch))
setk('commit', json.dumps(commit))
setk('status', 'open')
text = re.sub(r'^# .*$', f'# {title}', text, count=1, flags=re.M)
text = text.replace('## Setup\n\n', f'## Setup\n\n{note}\n\n', 1)
open(dst, 'w', encoding='utf-8').write(text)
EOF
if [ -n "$HILRUN" ]; then
  [ "$HILRUN" != last ] || HILRUN=$(ls -1dt "$F1T_ROOT/$F1T_HIL_LOG_DIR"/*_*/ 2>/dev/null | head -1)
  HILRUN=${HILRUN%/}
  if [ -f "$HILRUN/result.json" ]; then
    mkdir -p "$DIR/data"
    cp "$HILRUN/result.json" "$DIR/data/hil_result.json"
    [ ! -f "$HILRUN/checks.json" ] || cp "$HILRUN/checks.json" "$DIR/data/hil_checks.json"
    python3 - "$DIR/report.md" "${HILRUN#"$F1T_ROOT"/}" <<'EOF'
import sys
p, run = sys.argv[1:3]
t = open(p, encoding='utf-8').read()
t = t.replace('## Metodo\n', f'Run HIL: `{run}` (log completi lì, fuori da git; esito in `data/hil_result.json`).\n\n## Metodo\n', 1)
open(p, 'w', encoding='utf-8').write(t)
EOF
    log_ok "collegato il run HIL ${HILRUN#"$F1T_ROOT"/}"
  else
    log_warn "run HIL non trovato: $HILRUN (nessun result.json)"
  fi
fi
log_ok "creato ${DIR#"$F1T_ROOT"/}/report.md (moduli: $SRC)"
echo "${DIR#"$F1T_ROOT"/}/report.md"
log_info "poi: compila ipotesi/risultato/decisione, f1t exp index --check, commit docs-only su main, tag exp/$ID sul commit testato"
