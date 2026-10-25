#!/bin/bash
# Dump an Orin Jazzy image's resolved versions and diff them against the
# manifest (plan P7; first written for Phase 5d D1):
#   docker/orin/image_report.sh [image]   # default f1tenth:jazzy-orin-<HEAD short sha>
# Writes output/orin_container/logs/image_{versions,inspect,diff}_<tag>.{txt,md}.
# Starts one short-lived container (nvidia runtime, host network, the host's
# video/render GIDs) that runs image_versions.sh; the guard runs first, since a
# second ROS-capable process tree must not share the Orin with a stack.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
IMAGE=${1:-f1tenth:jazzy-orin-$(git -C "$REPO" rev-parse --short=7 HEAD)}
TAG=${IMAGE##*:}
OUT=$REPO/output/orin_container/logs
# shellcheck source=docker/orin/lib_guard.sh
source "$HERE/lib_guard.sh"
orin_guard
mkdir -p "$OUT"
docker run --rm --runtime nvidia --network host \
  --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)" \
  -v "$HERE/image_versions.sh:/tmp/image_versions.sh:ro" \
  "$IMAGE" bash /tmp/image_versions.sh > "$OUT/image_versions_${TAG}.txt" 2>&1
{
  echo "image $IMAGE"
  docker image inspect "$IMAGE" --format 'id {{.Id}}
size_bytes {{.Size}}
created {{.Created}}
revision {{index .Config.Labels "org.opencontainers.image.revision"}}'
  echo "labels:"
  docker image inspect "$IMAGE" --format '{{range $k, $v := .Config.Labels}}  {{$k}}={{$v}}
{{end}}'
} > "$OUT/image_inspect_${TAG}.txt"
python3 "$HERE/diff_manifest.py" "$HERE/jazzy_versions.manifest" "$HERE/thor_capture/ros_jazzy_dpkg.txt" \
  "$OUT/image_versions_${TAG}.txt" > "$OUT/image_diff_${TAG}.md"
cat "$OUT/image_inspect_${TAG}.txt"
echo "versions: $OUT/image_versions_${TAG}.txt"
echo "diff:     $OUT/image_diff_${TAG}.md ($(tail -1 "$OUT/image_diff_${TAG}.md"))"
