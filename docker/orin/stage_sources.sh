#!/bin/bash
# Stage the workspace sources of ONE commit as the image's `src` build context:
#   <out>/src/...                     git archive <sha> src/ (+ every submodule at the commit <sha> pins)
#   <out>/scripts/env/                git archive <sha> scripts/env/ (the container sources jazzy.sh)
#   <out>/scripts/export_yolo_engine.sh  the TensorRT engine export (`docker_orin.sh engine`)
#   <out>/CAR_SYNC.json               docker/orin/car_sync.json at <sha>: the car's native tree
#                                     this commit reproduces (plan P1.4)
#   <out>/SOURCES                     the commit, its branch and describe, each submodule commit,
#                                     the car_sync hash, and a tree hash
# Only committed content is staged, so the image tag f1tenth:jazzy-orin-<sha>
# means exactly that commit. Submodule objects come from this repo's local
# module stores; a commit missing there is fetched from the .gitmodules URL
# into a cache. husarion_gz_worlds is sim-only and never built on the Orin.
#   docker/orin/stage_sources.sh <sha> <out_dir>
# Env: ORIN_GIT_BRANCH (the branch name recorded in SOURCES; default: the
# local or remote-tracking branch that points at <sha>, e.g. the Orin's
# refs/remotes/linus/main-jazzy-rust from the git bundle), STAGE_CACHE.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
SHA=$(git -C "$REPO" rev-parse --verify "${1:?commit}^{commit}")
OUT=${2:?out dir}
COMMON=$(git -C "$REPO" rev-parse --path-format=absolute --git-common-dir)
CACHE=${STAGE_CACHE:-$HOME/.cache/f1tenth_p5d/modules}
SKIP='src/husarion_gz_worlds'

# The branch: a ref that points at the commit (refs/heads/X -> X,
# refs/remotes/<remote>/X -> X), the first in ref order; none -> "detached".
branch_of() {
  local ref
  while read -r ref; do
    case "$ref" in
      refs/heads/*) echo "${ref#refs/heads/}"; return ;;
      refs/remotes/*/HEAD) ;;
      refs/remotes/*) ref=${ref#refs/remotes/}; echo "${ref#*/}"; return ;;
    esac
  done < <(git -C "$REPO" for-each-ref --points-at "$1" --format='%(refname)' refs/heads refs/remotes)
  echo detached
}

git -C "$REPO" cat-file -e "$SHA:docker/orin/car_sync.json" 2>/dev/null \
  || { echo "refusing: $SHA has no docker/orin/car_sync.json (plan P1.4)" >&2; exit 1; }

# CLAUDE.md "Harness gotchas": recursive deletes only under an expected prefix.
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
case "$OUT/" in
  /scratch/fabiocar/*|"$HOME"/p5d_*|"$REPO"/*) ;;
  *) echo "refusing: staging dir $OUT is not under /scratch/fabiocar/, ~/p5d_* or the repo" >&2; exit 1 ;;
esac
rm -rf -- "${OUT:?}/src" "${OUT:?}/scripts"
rm -f -- "${OUT:?}/SOURCES" "${OUT:?}/CAR_SYNC.json"
git -C "$REPO" archive "$SHA" src scripts/env scripts/export_yolo_engine.sh | tar -x -C "$OUT"
git -C "$REPO" show "$SHA:docker/orin/car_sync.json" > "$OUT/CAR_SYNC.json"
car_sync_head=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["native_head"])' "$OUT/CAR_SYNC.json")
{
  echo "commit $SHA"
  echo "subject $(git -C "$REPO" log -1 --format=%s "$SHA")"
  echo "branch ${ORIN_GIT_BRANCH:-$(branch_of "$SHA")}"
  echo "describe $(git -C "$REPO" describe --always --tags --abbrev=12 "$SHA")"
  echo "car_sync_sha256 $(sha256sum "$OUT/CAR_SYNC.json" | cut -d' ' -f1)"
  echo "car_sync_native_head $car_sync_head"
} > "$OUT/SOURCES"

git -C "$REPO" show "$SHA:.gitmodules" > "$OUT/.gitmodules.tmp"
git config -f "$OUT/.gitmodules.tmp" --get-regexp '^submodule\..*\.path$' | while read -r key path; do
  name=${key#submodule.}; name=${name%.path}
  case " $SKIP " in *" $path "*) echo "submodule $path skipped (sim only)" >> "$OUT/SOURCES"; continue ;; esac
  commit=$(git -C "$REPO" ls-tree "$SHA" "$path" | awk '{print $3}')
  url=$(git config -f "$OUT/.gitmodules.tmp" "submodule.$name.url")
  store="$COMMON/modules/$name"
  if ! git --git-dir="$store" cat-file -e "$commit^{commit}" 2>/dev/null; then
    store="$CACHE/$name.git"
    [ -d "$store" ] || git clone -q --bare --filter=blob:none "$url" "$store"
    git --git-dir="$store" cat-file -e "$commit^{commit}" 2>/dev/null \
      || git --git-dir="$store" fetch -q origin "$commit"
  fi
  mkdir -p "$OUT/$path"
  git --git-dir="$store" archive "$commit" | tar -x -C "$OUT/$path"
  echo "submodule $path $commit $url" >> "$OUT/SOURCES"
done
rm -f "$OUT/.gitmodules.tmp"
echo "tree_sha256 $(cd "$OUT" && find src scripts -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)" >> "$OUT/SOURCES"
cat "$OUT/SOURCES"
