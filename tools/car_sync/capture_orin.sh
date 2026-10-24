#!/usr/bin/env bash
# Read-only capture of the car's native checkout: what the car runs beyond a base
# commit. Writes nothing in the checkout (GIT_OPTIONAL_LOCKS=0); the result is a
# tar stream on stdout.
#
#   ssh jetson-orin bash -s -- [WS] [BASE] < tools/car_sync/capture_orin.sh > capture.tar
#
# WS defaults to ~/dev_ws/f1tenth_more, BASE to fa5471f3 (campaign-final). The tar
# holds meta.txt, commits.txt (BASE..HEAD), src.diff (+ .sha256, uncommitted
# changes under src/), status.txt, untracked.tar + untracked.sha256 (untracked,
# not ignored, files under src/) and submodules.txt.
set -euo pipefail
export GIT_OPTIONAL_LOCKS=0
WS=${1:-$HOME/dev_ws/f1tenth_more}
BASE=${2:-fa5471f3}
tmp=$(mktemp -d)
trap 'rm -r -- "${tmp:?}"' EXIT
cd "$WS"
{
  echo "host=$(hostname)"
  echo "captured_at=$(date -Is)"
  echo "ws=$WS"
  echo "head=$(git rev-parse HEAD)"
  echo "branch=$(git rev-parse --abbrev-ref HEAD)"
  echo "base=$(git rev-parse "$BASE")"
} >"$tmp/meta.txt"
git log --format='%H %s' "$BASE..HEAD" >"$tmp/commits.txt"
git diff HEAD -- src >"$tmp/src.diff"
sha256sum <"$tmp/src.diff" | cut -d' ' -f1 >"$tmp/src.diff.sha256"
git status --porcelain -uall -- src >"$tmp/status.txt"
git ls-files --others --exclude-standard -z -- src >"$tmp/untracked.lst"
tar --null -T "$tmp/untracked.lst" -cf "$tmp/untracked.tar"
if [[ -s $tmp/untracked.lst ]]; then xargs -0 sha256sum <"$tmp/untracked.lst" >"$tmp/untracked.sha256"; else : >"$tmp/untracked.sha256"; fi
git submodule status >"$tmp/submodules.txt"
tar -C "$tmp" -cf - meta.txt commits.txt src.diff src.diff.sha256 status.txt untracked.tar untracked.sha256 submodules.txt
