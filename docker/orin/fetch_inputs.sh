#!/bin/bash
# Download the pinned binary inputs listed in inputs.sha256 into an inputs dir
# (default /scratch/fabiocar/p5d_downloads, about 4.2 GB) and verify every
# sha256. Files already present are only verified. Exit 1 on any mismatch.
#   docker/orin/fetch_inputs.sh [INPUTS_DIR]
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DIR=${1:-${ORIN_INPUTS:-/scratch/fabiocar/p5d_downloads}}
mkdir -p "$DIR"
bad=0
while read -r sum file url; do
  case "$sum" in ''|\#*) continue ;; esac
  if [ ! -f "$DIR/$file" ]; then
    echo "fetch $file"
    curl -sS -L --retry 5 -o "$DIR/$file.part" "$url" && mv "$DIR/$file.part" "$DIR/$file"
  fi
  if echo "$sum  $DIR/$file" | sha256sum -c --quiet - 2>/dev/null; then
    echo "ok    $file"
  else
    echo "BAD   $file (sha256 does not match inputs.sha256)" >&2
    bad=1
  fi
done < "$HERE/inputs.sha256"
exit $bad
