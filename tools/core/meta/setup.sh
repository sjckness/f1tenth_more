#!/usr/bin/env bash
# desc: installa hook pre-commit (tutti i worktree), comando f1t in ~/.local/bin e completamento bash
# alias: setup
#
#   f1t setup [--dry-run] [--bashrc]
#
# - hook: copia tools/core/meta/hooks/pre-commit in <git-common-dir>/hooks/pre-commit (vale per tutti i
#   worktree); un hook pre-commit diverso gia' presente diventa pre-commit.local e viene richiamato;
# - ~/.local/bin/f1t: lanciatore che usa tools/f1t del repo in cui ti trovi (ogni branch il suo
#   tooling), altrimenti quello di questo checkout;
# - completamento: --bashrc aggiunge a ~/.bashrc la riga di source di tools/core/completion.bash
#   (senza, la stampa). Idempotente.
set -euo pipefail
F1T_CORE=${F1T_CORE:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}
# shellcheck source=tools/core/lib/log.sh
source "$F1T_CORE/lib/log.sh"
DRY=0 BASHRC=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --bashrc) BASHRC=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) log_die "argomento sconosciuto: $a" 2 ;;
  esac
done
run() { if [ "$DRY" = 1 ]; then log_info "dry-run: $*"; else "$@"; fi; }

HOOKS=$(git -C "$F1T_ROOT" rev-parse --path-format=absolute --git-common-dir)/hooks
SRC=$F1T_CORE/meta/hooks/pre-commit
DST=$HOOKS/pre-commit
if [ -f "$DST" ] && ! grep -q 'f1t-pre-commit' "$DST"; then
  log_warn "c'e' gia' un hook pre-commit: diventa pre-commit.local (richiamato dal nuovo)"
  run mv "$DST" "$HOOKS/pre-commit.local"
fi
if [ -f "$DST" ] && cmp -s "$SRC" "$DST"; then log_ok "hook pre-commit gia' aggiornato ($DST)"
else run install -m 755 "$SRC" "$DST"; log_ok "hook pre-commit installato: $DST"; fi

BIN=$HOME/.local/bin/f1t
launcher=$(cat <<EOF
#!/usr/bin/env bash
# f1t-launcher (f1t setup): tools/f1t del repo corrente, altrimenti $F1T_ROOT
root=\$(git rev-parse --show-toplevel 2>/dev/null || true)
if [ -n "\$root" ] && [ -x "\$root/tools/f1t" ]; then exec "\$root/tools/f1t" "\$@"; fi
exec "$F1T_ROOT/tools/f1t" "\$@"
EOF
)
if [ -e "$BIN" ] && ! grep -q 'f1t-launcher' "$BIN"; then
  log_warn "$BIN esiste e non e' il lanciatore f1t: non lo tocco"
elif [ -f "$BIN" ] && [ "$(cat "$BIN")" = "$launcher" ]; then
  log_ok "lanciatore gia' presente: $BIN"
else
  if [ "$DRY" = 1 ]; then log_info "dry-run: scriverei $BIN"
  else mkdir -p "$(dirname "$BIN")"; printf '%s\n' "$launcher" > "$BIN"; chmod 755 "$BIN"; log_ok "lanciatore: $BIN"; fi
fi
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) log_warn "$HOME/.local/bin non e' nel PATH" ;; esac

LINE="[ -f \"$F1T_ROOT/tools/core/completion.bash\" ] && source \"$F1T_ROOT/tools/core/completion.bash\"  # f1t"
if grep -qF '# f1t' "$HOME/.bashrc" 2>/dev/null; then log_ok "completamento gia' in ~/.bashrc"
elif [ "$BASHRC" = 1 ]; then run bash -c "printf '%s\n' '$LINE' >> \"\$HOME/.bashrc\""; log_ok "completamento aggiunto a ~/.bashrc"
else log_info "completamento bash: aggiungi a ~/.bashrc (o f1t setup --bashrc):"; echo "  $LINE"; fi
