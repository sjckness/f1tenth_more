#!/bin/bash
# Dump an Orin Jazzy image's resolved versions and diff them against the
# manifest (Phase 5d D1, output/phase5d_image.md):
#   docker/orin/image_report.sh [image]   # default f1tenth:jazzy-orin-<HEAD short sha>
# Writes output/phase5d_logs/d1_image_versions_<tag>.txt and _diff.md.
# Starts one container (nvidia runtime, host network): the guard runs first.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
IMAGE=${1:-f1tenth:jazzy-orin-$(git -C "$REPO" rev-parse --short=7 HEAD)}
TAG=${IMAGE##*:}
OUT=$REPO/output/phase5d_logs
source "$HERE/lib_guard.sh"
orin_guard
mkdir -p "$OUT"
docker run --rm --runtime nvidia --network host \
  --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)" \
  -v "$HERE/image_versions.sh:/tmp/image_versions.sh:ro" \
  "$IMAGE" bash /tmp/image_versions.sh > "$OUT/d1_image_versions_${TAG}.txt" 2>&1
{
  echo "image $IMAGE"
  docker image inspect "$IMAGE" --format 'id {{.Id}}
size_bytes {{.Size}}
created {{.Created}}
revision {{index .Config.Labels "org.opencontainers.image.revision"}}'
} > "$OUT/d1_image_inspect_${TAG}.txt"
python3 "$HERE/diff_manifest.py" "$HERE/jazzy_versions.manifest" "$HERE/thor_capture/ros_jazzy_dpkg.txt" \
  "$OUT/d1_image_versions_${TAG}.txt" > "$OUT/d1_image_diff_${TAG}.md"
cat "$OUT/d1_image_inspect_${TAG}.txt"
echo "versions: $OUT/d1_image_versions_${TAG}.txt"
echo "diff:     $OUT/d1_image_diff_${TAG}.md"
