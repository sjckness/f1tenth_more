# shellcheck shell=bash
# scripts/lib/orin_common.sh - funzioni comuni di scripts/orin_deploy.sh e scripts/orin_precheck.sh
# (su linus, dopo tools/core/lib/config.sh e remote.sh).
#
#   image_commit REV                  l'ultimo commit fino a REV che cambia cio' che entra nell'immagine
#   same_image A B                    0 se A e B danno la stessa immagine (nessuna differenza in IMAGE_PATHS)
#   branch_of SHA                     il branch per l'alias: quello corrente se contiene SHA, altrimenti
#                                     il primo che lo contiene, altrimenti "detached"
#   image_tag_of SHA                  f1tenth:jazzy-orin-<sha7>[-norust], come lo chiama f1t build
#   orin_reusable_image REV           "TAG SHA" di un'immagine gia' sull'Orin equivalente a REV (prima la
#                                     fissata, poi la piu' recente); niente se non ce n'e'
#   orin_running_containers           "nome immagine stato" dei container in esecuzione sull'Orin
#   orin_drift_summary                riassunto del drift check (docker_orin drift) in una riga
[ -n "${_ORIN_COMMON_SH:-}" ] && return 0
_ORIN_COMMON_SH=1

# Cio' che finisce nell'immagine: docker/orin/stage_sources.sh (git archive src scripts/env
# scripts/export_yolo_engine.sh) + il contesto di build docker/orin (Dockerfile, container/, car_sync.json).
IMAGE_PATHS=(src docker/orin scripts/env scripts/export_yolo_engine.sh)

image_commit() { git -C "$F1T_ROOT" log -1 --format=%H "$1" -- "${IMAGE_PATHS[@]}"; }

same_image() { git -C "$F1T_ROOT" diff --quiet "$1" "$2" -- "${IMAGE_PATHS[@]}" 2>/dev/null; }

branch_of() {
  local sha=$1 cur
  cur=$(git -C "$F1T_ROOT" branch --show-current)
  if [ -n "$cur" ] && git -C "$F1T_ROOT" merge-base --is-ancestor "$sha" HEAD 2>/dev/null; then
    echo "$cur"; return
  fi
  cur=$(git -C "$F1T_ROOT" branch --contains "$sha" --format='%(refname:short)' 2>/dev/null | head -1)
  echo "${cur:-detached}"
}

image_tag_of() {
  local suffix=''
  [ "$F1T_IMAGE_RUST" = skip ] && suffix=-norust
  echo "$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX${1:0:7}$suffix"
}

orin_reusable_image() {
  local rev=$1 pinned tags tag short sha
  pinned=$(orin_ssh "awk '\$1==\"tag\"{print \$2}' ~/.config/f1tenth/car_image" 2>/dev/null || true)
  tags=$(orin_ssh "docker images --format '{{.Repository}}:{{.Tag}}' $F1T_IMAGE_REPO" 2>/dev/null \
    | grep -E "^${F1T_IMAGE_REPO}:${F1T_IMAGE_PREFIX}[0-9a-f]{7}(-norust)?\$" || true)
  for tag in $pinned $tags; do
    grep -qxF "$tag" <<<"$tags" || continue
    short=${tag#"$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX"}; short=${short%-norust}
    [ "$tag" = "$(image_tag_of "$short")" ] || continue
    sha=$(git -C "$F1T_ROOT" rev-parse -q --verify "$short^{commit}" 2>/dev/null) || continue
    if same_image "$sha" "$rev"; then echo "$tag $sha"; return 0; fi
  done
  return 1
}

orin_running_containers() {
  orin_ssh "docker ps --format '{{.Names}} {{.Image}} {{.Status}}'" 2>/dev/null || true
}

orin_drift_summary() {
  orin_ssh "$F1T_ORIN_DOCKER_ORIN drift 2>/dev/null" 2>/dev/null | python3 -c '
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
