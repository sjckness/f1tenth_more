# shellcheck shell=bash
# scripts/car/lib/orin_common.sh - funzioni comuni degli script di scripts/car/ (build, deploy, ready)
# (su linus, dopo tools/core/lib/config.sh e remote.sh).
#
#   image_commit REV                  l'ultimo commit fino a REV che cambia cio' che entra nell'immagine
#   same_image A B                    0 se A e B danno la stessa immagine (nessuna differenza in IMAGE_PATHS)
#   branch_of SHA                     il branch per l'alias: quello corrente se contiene SHA, altrimenti
#                                     il primo che lo contiene, altrimenti "detached"; ripulito per un tag
#                                     Docker (solo [A-Za-z0-9_.-], max 100 caratteri)
#   image_tag_of SHA                  f1tenth:jazzy-orin-<sha7>[-norust], come lo chiama f1t build
#   orin_reusable_image REV           "TAG SHA" di un'immagine gia' sull'Orin equivalente a REV: prima una
#                                     testata (la fissata se lo e'), poi la fissata, poi la piu' recente;
#                                     niente se non ce n'e'
#   orin_running_containers           "nome immagine stato" dei container in esecuzione sull'Orin
#   orin_drift_summary                riassunto del drift check (docker_orin drift) in una riga
#   img_label IMG LABEL / img_id IMG  label / ID di un'immagine sull'Orin
#   car_commit_gate SHA all|image     0 se SHA e' completo e committato (prima di toccare l'Orin)
#   car_tools_install SHA             script dell'host dell'Orin (CAR_TOOLS) di SHA -> ~/.local/bin
#   car_tools_check SHA               "nome OK|FAIL sha" per ognuno: installato == SHA
#   car_profiles_push                 profili del repo -> ~/.config/f1tenth_car/profiles sull'Orin
#   car_profiles_sums / car_profiles_remote_sums    "nome sha256" dei profili, linus / Orin
[ -n "${_ORIN_COMMON_SH:-}" ] && return 0
_ORIN_COMMON_SH=1

# Cio' che finisce nell'immagine: docker/orin/stage_sources.sh (git archive src scripts/env
# scripts/export_yolo_engine.sh) + il contesto di build docker/orin (Dockerfile, container/, car_sync.json).
# Esclusi i profili di lancio: sono anche nell'immagine, ma l'auto legge la copia in
# ~/.config/f1tenth_car/profiles (scripts/car/profiles.sh push), quindi un commit che cambia solo un
# profilo non chiede una nuova immagine.
IMAGE_PATHS=(src docker/orin scripts/env scripts/export_yolo_engine.sh
             ':(exclude)src/f1tenth_bringup/config/profiles')

image_commit() { git -C "$F1T_ROOT" log -1 --format=%H "$1" -- "${IMAGE_PATHS[@]}"; }

same_image() { git -C "$F1T_ROOT" diff --quiet "$1" "$2" -- "${IMAGE_PATHS[@]}" 2>/dev/null; }

branch_of() {
  local sha=$1 b
  b=$(git -C "$F1T_ROOT" branch --show-current)
  if [ -z "$b" ] || ! git -C "$F1T_ROOT" merge-base --is-ancestor "$sha" HEAD 2>/dev/null; then
    b=$(git -C "$F1T_ROOT" for-each-ref --contains "$sha" --format='%(refname:short)' refs/heads 2>/dev/null | head -1)
  fi
  b=${b:-detached}
  b=${b//[^A-Za-z0-9_.-]/-}
  echo "${b:0:100}"
}

image_tag_of() {
  local suffix=''
  [ "$F1T_IMAGE_RUST" = skip ] && suffix=-norust
  echo "$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX${1:0:7}$suffix"
}

orin_reusable_image() {
  local rev=$1 tag isha tested pin best='' fallback=''
  while read -r tag isha tested pin; do
    [ "$isha" != - ] || continue
    git -C "$F1T_ROOT" cat-file -e "$isha^{commit}" 2>/dev/null || continue
    same_image "$isha" "$rev" || continue
    if [ "$tested" = true ]; then
      if [ "$pin" = pin ] || [ -z "$best" ]; then best="$tag $isha"; fi
    elif [ "$pin" = pin ] || [ -z "$fallback" ]; then
      fallback="$tag $isha"
    fi
  done < <(orin_script "$F1T_ROOT/scripts/car/lib/orin_images.sh" "$F1T_IMAGE_REPO" "$F1T_IMAGE_PREFIX" 2>/dev/null || true)
  [ -n "$best$fallback" ] || return 1
  echo "${best:-$fallback}"
}

orin_running_containers() {
  orin_ssh "docker ps --format '{{.Names}} {{.Image}} {{.Status}}'" 2>/dev/null || true
}

orin_drift_summary() {
  { orin_ssh "$F1T_ORIN_DOCKER_ORIN drift 2>/dev/null" 2>/dev/null || true; } | python3 -c '
import json, sys
txt = sys.stdin.read()
try:
    d = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
except ValueError:
    print("non letto"); sys.exit()
if d.get("ok"):
    print("nessun drift"); sys.exit()
kinds = [x.get("kind") for x in d.get("drift", [])]
files = sum(len(x.get("files", [])) for x in d.get("drift", []))
ncommit = kinds.count("commit")
branch = d.get("native", {}).get("branch", "?")
print(f"{ncommit} commit e {files} file del checkout nativo non presenti nella immagine (branch nativo {branch})")
' 2>/dev/null || echo "non letto"
}

# ---------------------------------------------------------------- strumenti dell'Orin e profili
# Gli script che girano sull'host dell'Orin, installati da scripts/car/deploy.sh in ~/.local/bin dal
# commit deployato (git show SHA:percorso): "percorso nel repo:nome installato".
CAR_TOOLS=(scripts/car/orin/stack.sh:car-stack scripts/car/orin/container.sh:car-container
           scripts/orin/preflight.sh:car-preflight
           scripts/car/orin/hil.sh:car-hil scripts/car/orin/hil_agent.py:car-hil-agent)
# I profili di lancio: sorgente nel repo, copia sull'Orin (montata in sola lettura nel container,
# stesso percorso: compose.yaml monta ~/.config/f1tenth_car).
CAR_PROFILES_SRC=src/f1tenth_bringup/config/profiles
CAR_PROFILES_DIR='.config/f1tenth_car/profiles'      # relativo alla home dell'Orin

img_label() { orin_ssh "docker image inspect -f '{{index .Config.Labels \"$2\"}}' $1" 2>/dev/null || true; }
img_id() { orin_ssh "docker image inspect -f '{{.Id}}' $1" 2>/dev/null || true; }

# car_tool_sum SHA PERCORSO: sha256 del file com'e' nel commit SHA.
# Vuoto se il file non c'e' in SHA (es. uno script non ancora committato).
car_tool_sum() {
  git -C "$F1T_ROOT" cat-file -e "$1:$2" 2>/dev/null || return 0
  git -C "$F1T_ROOT" show "$1:$2" | sha256sum | cut -d' ' -f1
}

# car_commit_gate SHA all|image [--allow-dirty-profiles]: 0 se SHA si puo' portare sull'Orin cosi'
# com'e', altrimenti stampa perche' e ritorna 1. Va chiamata prima di toccare l'Orin: un deploy a meta'
# (immagine si', script no) e' peggio di nessun deploy.
#   all    gli script dell'host (CAR_TOOLS) esistono in SHA (il deploy li installa da li')
#   se SHA e' HEAD, nessuna modifica non committata in cio' che finisce sull'Orin (si porterebbe HEAD,
#   non quello che si vede): i file tracciati del contenuto dell'immagine (un sottomodulo conta solo
#   se ne cambia il commit, non per file sporchi al suo interno); per "all" anche
#   scripts/car, gli script dell'host e i profili, compresi i file nuovi mai aggiunti a git
car_commit_gate() {
  local sha=$1 what=$2 t missing=() dirty='' extra=()
  if [ "$what" = all ]; then
    for t in "${CAR_TOOLS[@]}"; do
      git -C "$F1T_ROOT" cat-file -e "$sha:${t%%:*}" 2>/dev/null || missing+=("${t%%:*}")
      extra+=("${t%%:*}")
    done
    extra+=(scripts/car)
    [ "${3:-}" = --allow-dirty-profiles ] || extra+=("$CAR_PROFILES_SRC")
  fi
  [ "${#missing[@]}" = 0 ] || echo "non sono nel commit ${sha:0:12}: ${missing[*]}"
  if [ "$sha" = "$(git -C "$F1T_ROOT" rev-parse HEAD)" ]; then
    dirty=$( { git -C "$F1T_ROOT" status --porcelain --untracked-files=no --ignore-submodules=dirty -- "${IMAGE_PATHS[@]}"
               [ "${#extra[@]}" = 0 ] || git -C "$F1T_ROOT" status --porcelain --untracked-files=all -- "${extra[@]}"
             } | sort -u | head -20)
    [ -z "$dirty" ] || printf "modifiche non committate (andrebbe sull'Orin HEAD, non questo):\n%s\n" "$dirty"
  fi
  [ "${#missing[@]}" = 0 ] && [ -z "$dirty" ]
}

# car_tools_install SHA: installa CAR_TOOLS di SHA in ~/.local/bin sull'Orin.
car_tools_install() {
  local sha=$1 t
  for t in "${CAR_TOOLS[@]}"; do
    git -C "$F1T_ROOT" show "$sha:${t%%:*}" | orin_ssh "install -D -m 755 /dev/stdin .local/bin/${t##*:}" || return 1
  done
}

# car_tools_check SHA: una riga per strumento "NOME OK|FAIL sha12": installato == commit.
car_tools_check() {
  local sha=$1 t want have names='' line
  for t in "${CAR_TOOLS[@]}"; do names+=" .local/bin/${t##*:}"; done
  # shellcheck disable=SC2086  # nomi senza spazi, apposta separati
  have=$(orin_ssh "sha256sum $names 2>/dev/null" || true)
  for t in "${CAR_TOOLS[@]}"; do
    want=$(car_tool_sum "$sha" "${t%%:*}")
    line=$(awk -v f=".local/bin/${t##*:}" '$2==f{print $1}' <<<"$have")
    if [ -n "$line" ] && [ "$line" = "$want" ]; then echo "${t##*:} OK ${line:0:12}"
    else echo "${t##*:} FAIL ${line:-assente}"; fi
  done
}

# car_profiles_sums: "nome sha256" dei profili nel repo di linus (albero di lavoro).
car_profiles_sums() {
  local f
  for f in "$F1T_ROOT/$CAR_PROFILES_SRC"/*.yaml; do
    [ -e "$f" ] || continue
    echo "$(basename "$f" .yaml) $(sha256sum "$f" | cut -d' ' -f1)"
  done
}

# car_profiles_push: copia i profili del repo sull'Orin (rsync --delete: l'Orin ha esattamente quelli).
car_profiles_push() {
  local -a o; mapfile -t o < <(_orin_opts)
  orin_ssh "mkdir -p $CAR_PROFILES_DIR" || return 1
  rsync -rlt --delete --checksum --include='*.yaml' --exclude='*' -e "ssh ${o[*]}" \
    "$F1T_ROOT/$CAR_PROFILES_SRC/" "$(orin_target):$CAR_PROFILES_DIR/"
}

# car_profiles_remote_sums: "nome sha256" dei profili sull'Orin.
car_profiles_remote_sums() {
  orin_ssh "cd $CAR_PROFILES_DIR 2>/dev/null && for f in *.yaml; do [ -e \"\$f\" ] && echo \"\${f%.yaml} \$(sha256sum \"\$f\" | cut -d' ' -f1)\"; done" 2>/dev/null || true
}
