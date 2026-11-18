#!/bin/bash
# Build the Orin Jazzy image from ONE commit (Phase 5d D1):
#   docker/orin/build.sh [commit]          # default HEAD
# Everything comes from that commit: the workspace sources (stage_sources.sh)
# AND this directory (Dockerfile, package list, helper scripts), so the tag
# f1tenth:jazzy-orin-<short sha> means exactly that commit. Uncommitted edits
# are never in the image.
#
# - Refuses on a host that is not JetPack 6 / L4T r36.5 (F7: the build links
#   the host's L4T libraries, see the Dockerfile header).
# - Refuses if a pinned input is missing or its sha256 differs (fetch_inputs.sh).
# - Host networking (CLAUDE.md "Orin Docker").
# - Log: output/phase5d_logs/d1_build_<sha>_<time>.log. Long: run it in the
#   background.
# Env: ORIN_INPUTS (inputs dir), ORIN_STAGE (staging root), ORIN_BUILD_ARGS
# (extra docker build args).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SHA=$(git -C "$REPO" rev-parse --verify "${1:-HEAD}^{commit}")
SHORT=${SHA:0:7}
INPUTS=${ORIN_INPUTS:-/scratch/fabiocar/p5d_downloads}
STAGE=${ORIN_STAGE:-/scratch/fabiocar/p5d_build}/$SHORT
L4T=/usr/lib/aarch64-linux-gnu/nvidia
IMAGE=f1tenth:jazzy-orin-$SHORT
LOG=$REPO/output/phase5d_logs/d1_build_${SHORT}_$(date +%Y%m%d_%H%M%S).log

grep -q '^# R36 (release), REVISION: 5\.' /etc/nv_tegra_release 2>/dev/null \
  || { echo "refusing: not an L4T r36.5 host ($(head -1 /etc/nv_tegra_release 2>/dev/null))" >&2; exit 1; }
[ -e "$L4T/libnvbufsurface.so.1.0.0" ] || { echo "refusing: $L4T has no L4T multimedia libs" >&2; exit 1; }
"$HERE/fetch_inputs.sh" "$INPUTS" > /dev/null || { echo "refusing: pinned inputs missing or wrong (fetch_inputs.sh)" >&2; exit 1; }

if [ "$SHA" = "$(git -C "$REPO" rev-parse HEAD)" ] && [ -n "$(git -C "$REPO" status --porcelain -- src scripts/env docker/orin)" ]; then
  echo "note: uncommitted changes under src/, scripts/env/ or docker/orin/ are NOT in the image (built from $SHORT)"
fi

case "$STAGE/" in
  /scratch/fabiocar/*|"$HOME"/p5d_*) ;;
  *) echo "refusing: staging dir $STAGE is not under /scratch/fabiocar/ or ~/p5d_*" >&2; exit 1 ;;
esac
mkdir -p "$STAGE/ctx" "$(dirname "$LOG")"
"$HERE/stage_sources.sh" "$SHA" "$STAGE/sources" > /dev/null
rm -rf -- "${STAGE:?}/ctx/docker"
git -C "$REPO" archive "$SHA" docker/orin | tar -x -C "$STAGE/ctx"

{
  echo "image $IMAGE"
  echo "commit $SHA"
  echo "started $(date -Iseconds)"
  echo "host $(head -1 /etc/nv_tegra_release); nvpmodel $(nvpmodel -q 2>/dev/null | head -1)"
  cat "$STAGE/sources/SOURCES"
} > "$LOG"
echo "building $IMAGE, log: $LOG"
docker build --network=host --progress=plain \
  --build-context inputs="$INPUTS" \
  --build-context src="$STAGE/sources" \
  --build-context l4t="$L4T" \
  --build-arg GIT_SHA="$SHA" \
  ${ORIN_BUILD_ARGS:-} \
  -t "$IMAGE" -f "$STAGE/ctx/docker/orin/Dockerfile" "$STAGE/ctx/docker/orin" >> "$LOG" 2>&1
echo "finished $(date -Iseconds)" >> "$LOG"
echo "build OK: $IMAGE $(docker image inspect "$IMAGE" --format '{{.Id}} {{.Size}}')" | tee -a "$LOG"
