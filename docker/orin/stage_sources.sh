#!/bin/bash
# Stage the workspace sources of ONE commit as the image's `src` build context:
#   <out>/src/...          git archive <sha> src/ (+ every submodule at the commit <sha> pins)
#   <out>/scripts/env/     git archive <sha> scripts/env/ (the container sources jazzy.sh)
#   <out>/SOURCES          the commit, each submodule commit, and a tree hash
# Only committed content is staged, so the image tag f1tenth:jazzy-orin-<sha>
# means exactly that commit. Submodule objects come from this repo's local
# module stores; a commit missing there is fetched from the .gitmodules URL
# into a cache. husarion_gz_worlds is sim-only and never built on the Orin.
#   docker/orin/stage_sources.sh <sha> <out_dir>
set -euo pipefail
SHA=$(git rev-parse --verify "${1:?commit}^{commit}")
OUT=${2:?out dir}
REPO=$(git rev-parse --show-toplevel)
COMMON=$(cd "$REPO" && git rev-parse --path-format=absolute --git-common-dir)
CACHE=${STAGE_CACHE:-$HOME/.cache/f1tenth_p5d/modules}
SKIP='src/husarion_gz_worlds'

# CLAUDE.md "Harness gotchas": recursive deletes only under an expected prefix.
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
case "$OUT/" in
  /scratch/fabiocar/*|"$HOME"/p5d_*|"$REPO"/*) ;;
  *) echo "refusing: staging dir $OUT is not under /scratch/fabiocar/, ~/p5d_* or the repo" >&2; exit 1 ;;
esac
rm -rf -- "${OUT:?}/src" "${OUT:?}/scripts" "${OUT:?}/SOURCES"
git -C "$REPO" archive "$SHA" src scripts/env | tar -x -C "$OUT"
{
  echo "commit $SHA"
  echo "subject $(git -C "$REPO" log -1 --format=%s "$SHA")"
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
