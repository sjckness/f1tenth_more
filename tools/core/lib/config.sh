# shellcheck shell=bash
# tools/core/lib/config.sh - configurazione unica del tooling.
#
#   cfg_load                    tools/core/config.env, poi ~/.config/f1t/config.env (se c'e'),
#                               poi le variabili F1T_* esportate dal chiamante (vincono sempre)
#   cfg_load_host NOME          tools/core/hosts/NOME.env (host della sim: linus | work)
#   cfg_load_scenario NOME      tools/core/hil/scenarios/NOME.env
#   cfg_require SCHEMA          controlla le chiavi obbligatorie di tools/core/schema/SCHEMA.keys
#                               (dice quale chiave manca e in quale file); ritorna 1 se ne manca una
#   cfg_print CHIAVE...         stampa la configurazione effettiva e i file letti
#
# I file .env sono bash (CHIAVE=valore). Nessun valore va scritto negli script: solo qui.
[ -n "${_F1T_CONFIG_SH:-}" ] && return 0
_F1T_CONFIG_SH=1
# shellcheck source=tools/core/lib/log.sh
source "$(dirname "${BASH_SOURCE[0]}")/log.sh"

F1T_CFG_FILES=()
_F1T_ENV_OVERRIDES=$(declare -px 2>/dev/null | sed -n 's/^declare -x \(F1T_[A-Z0-9_]*\)=.*/\1/p' \
  | grep -v -E '^F1T_(ROOT|CORE|SCRIPT|LOG_DIR|LOG_FILE|CFG_FILES)$' || true)
_F1T_ENV_SAVED=()
for _k in $_F1T_ENV_OVERRIDES; do _F1T_ENV_SAVED+=("$_k=${!_k}"); done
unset _k

_cfg_source() {
  local f=$1
  [ -f "$f" ] || return 1
  # shellcheck disable=SC1090
  source "$f"
  F1T_CFG_FILES+=("$f")
}
_cfg_reapply_env() {
  local kv
  for kv in "${_F1T_ENV_SAVED[@]}"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done
}

cfg_load() {
  _cfg_source "$F1T_CORE/config.env" || log_die "manca $F1T_CORE/config.env"
  _cfg_source "${F1T_USER_CONFIG:-$HOME/.config/f1t/config.env}" || true
  _cfg_reapply_env
}

# Nomi disponibili in una cartella di .env (senza i file che iniziano con _).
cfg_names() { local f; for f in "$1"/*.env; do [ -e "$f" ] || continue; f=$(basename "$f" .env); [ "${f#_}" = "$f" ] && echo "$f"; done; }

cfg_load_host() {
  local name=$1 f=$F1T_CORE/hosts/$1.env
  if [ -z "$name" ] || [ ! -f "$f" ]; then
    log_fail "host della sim sconosciuto: '${name}'. Valori validi: $(cfg_names "$F1T_CORE/hosts" | tr '\n' ' ')"
    return 2
  fi
  _cfg_source "$f"
  # shellcheck disable=SC2034  # letta da chi fa source
  F1T_DISP=$name
  _cfg_reapply_env
}

cfg_load_scenario() {
  local name=$1 f=$F1T_CORE/hil/scenarios/$1.env
  if [ -z "$name" ] || [ ! -f "$f" ]; then
    log_fail "scenario sconosciuto: '${name}'. Valori validi: $(cfg_names "$F1T_CORE/hil/scenarios" | tr '\n' ' ')"
    return 2
  fi
  _cfg_source "$f"
  # shellcheck disable=SC2034  # letta da chi fa source
  F1T_SCENARIO=$name
  _cfg_reapply_env
}

# Righe dello schema: "CHIAVE  # descrizione" (obbligatoria) o "?CHIAVE  # descrizione" (facoltativa).
cfg_require() {
  local schema=$F1T_CORE/schema/$1.keys line key desc missing=0 optional
  [ -f "$schema" ] || log_die "schema inesistente: $schema"
  while IFS= read -r line; do
    case "$line" in ''|\#*) continue ;; esac
    key=${line%%[[:space:]]*}; desc=${line#*#}; [ "$desc" != "$line" ] || desc=''
    optional=0; [ "${key#\?}" = "$key" ] || { optional=1; key=${key#\?}; }
    [ "$optional" = 0 ] || continue
    if [ -z "${!key:-}" ]; then
      log_fail "config: chiave obbligatoria $key mancante o vuota (${desc# }) - file letti: ${F1T_CFG_FILES[*]}"
      missing=1
    fi
  done < "$schema"
  return "$missing"
}

cfg_print() {
  local k
  _log_ensure_dir
  {
    echo "== configurazione effettiva ($F1T_SCRIPT)"
    for k in "$@"; do printf '  %-26s %s\n' "$k" "${!k:-}"; done
    echo "  file: ${F1T_CFG_FILES[*]}"
    [ -z "$_F1T_ENV_OVERRIDES" ] || echo "  dall'ambiente: $(echo "$_F1T_ENV_OVERRIDES" | tr '\n' ' ')"
  } | tee -a "$F1T_LOG_FILE" >&2 2>/dev/null || true
}
