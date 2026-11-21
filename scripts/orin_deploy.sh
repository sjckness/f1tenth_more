#!/usr/bin/env bash
# orin_deploy.sh - da linus: immagine Jazzy dell'Orin per la campagna a 3 comandi, deploy, verifica.
#
#   scripts/orin_deploy.sh [--dry-run] [COMMIT]          default COMMIT = HEAD (committato)
#
# Riusa il tooling esistente, non lo duplica:
#   1 build   tools/f1t build COMMIT: bundle git -> build SULL'ORIN (il Dockerfile linka le librerie
#             L4T r36.5 dell'host, JetPack 6.2.x: su linus non si costruisce), pytest nell'immagine,
#             label f1tenth.tested=true. Tag f1tenth:jazzy-orin-<sha7>-norust. Nessun registry
#             (F1T_REGISTRY vuoto): l'immagine nasce gia' sull'Orin, niente docker save | load.
#   2 tag     alias f1tenth:<branch>-<sha7> sulla stessa immagine (stesso ID)
#   3 deploy  tools/f1t deploy --switch-mode: pin, restart della unit, health check, rollback automatico
#   4 script  scripts/orin_run_tests.sh dello stesso commit -> ~/.local/bin/orin_run_tests sull'Orin
#   5 verify  file di versione nell'immagine (/opt/f1tenth_ws/SOURCES: commit, branch, tree hash)
#             == commit di linus, pin == alias, script installato == commit: OK / FAIL
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_deploy
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
# shellcheck source=tools/core/lib/remote.sh
source "$F1T_CORE/lib/remote.sh"

DRY=() COMMIT=HEAD
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=(--dry-run) ;;
    -h|--help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) log_die "opzione sconosciuta: $a" 2 ;;
    *) COMMIT=$a ;;
  esac
done
cfg_load
SHA=$(git -C "$ROOT" rev-parse --verify "$COMMIT^{commit}") || log_die "commit sconosciuto: $COMMIT" 2
SHORT=${SHA:0:7}
BRANCH=$(git -C "$ROOT" branch --points-at "$SHA" --format='%(refname:short)' | head -1)
BRANCH=${BRANCH:-detached}
SUFFIX=; [ "$F1T_IMAGE_RUST" = skip ] && SUFFIX=-norust
IMAGE=$F1T_IMAGE_REPO:$F1T_IMAGE_PREFIX$SHORT$SUFFIX
TAG=$F1T_IMAGE_REPO:${BRANCH//\//-}-$SHORT
log_info "commit $SHA ($BRANCH)  immagine $IMAGE  alias $TAG  ${DRY[*]:-}"
log_steps 5

log_step "build sull'Orin (tools/f1t build)"
if [ ${#DRY[@]} = 0 ] && [ "$(orin_ssh "docker image inspect -f '{{index .Config.Labels \"f1tenth.tested\"}}' $IMAGE" 2>/dev/null)" = true ]; then
  log_ok "$IMAGE gia' sull'Orin e testata: build saltata"
else
  "$ROOT/tools/f1t" build "${DRY[@]}" "$SHA"
fi

log_step "alias $TAG"
if [ ${#DRY[@]} = 0 ]; then orin_ssh docker tag "$IMAGE" "$TAG"; else log_info "dry-run: docker tag $IMAGE $TAG"; fi

log_step "deploy (tools/f1t deploy --switch-mode)"
"$ROOT/tools/f1t" deploy "${DRY[@]}" --switch-mode "$IMAGE" \
  || { [ ${#DRY[@]} = 1 ] && log_warn "dry-run: deploy non pronto (atteso finche' l'immagine non esiste)"; } \
  || log_die "deploy fallito (f1t deploy fa gia' il rollback)"

log_step "orin_run_tests dal commit $SHORT"
if [ ${#DRY[@]} = 0 ]; then
  git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | orin_ssh 'install -D -m 755 /dev/stdin ~/.local/bin/orin_run_tests'
else
  log_info "dry-run: git show $SHORT:scripts/orin_run_tests.sh -> ~/.local/bin/orin_run_tests"
fi

log_step "verifica linus == Orin"
if [ ${#DRY[@]} = 1 ]; then log_info "dry-run: nessuna verifica"; exit 0; fi
pin_id=$(orin_ssh "awk '\$1==\"id\"{print \$2}' ~/.config/f1tenth/car_image")
tag_id=$(orin_ssh "docker image inspect -f '{{.Id}}' $TAG" 2>/dev/null || echo none)
img_sha=$(orin_ssh docker run --rm --network none --entrypoint awk "$TAG" "'\$1==\"commit\"{print \$2}'" /opt/f1tenth_ws/SOURCES 2>/dev/null || echo none)
want_sum=$(git -C "$ROOT" show "$SHA:scripts/orin_run_tests.sh" | sha256sum | cut -d' ' -f1)
have_sum=$(orin_ssh sha256sum .local/bin/orin_run_tests 2>/dev/null | cut -d' ' -f1 || echo none)
check_record "$([ "$pin_id" = "$tag_id" ] && echo OK || echo FAIL)" "pin = $TAG" "${tag_id:7:12}"
check_record "$([ "$img_sha" = "$SHA" ] && echo OK || echo FAIL)" "SOURCES nell'immagine" "${img_sha:0:12}"
check_record "$([ "$want_sum" = "$have_sum" ] && echo OK || echo FAIL)" "orin_run_tests" "${have_sum:0:12}"
if summary "orin_deploy $TAG"; then log_ok "linus $TAG == Orin $TAG: OK"; else log_die "linus $TAG != Orin: FAIL"; fi
