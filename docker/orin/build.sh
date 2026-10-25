#!/bin/bash
# Build the car's Orin Jazzy image from ONE commit (plan P5/P7):
#   docker/orin/build.sh [--dry-run] [commit]          # default HEAD
# Everything comes from that commit: the workspace sources, the engine export
# script and CAR_SYNC.json (stage_sources.sh) AND this directory (Dockerfile,
# package lists, helper scripts, container/), so the tag
# f1tenth:jazzy-orin-<short sha> (-norust for RUST=skip) means exactly that
# commit. Uncommitted edits are never in the image.
#
# Refuses (exit 1, says why) when:
#   - the host is not JetPack 6 / L4T r36.5 (F7: the build links the host's L4T
#     libraries, see the Dockerfile header);
#   - a stack runs: a native ROS process, a Discovery Server on UDP 11811, or an
#     f1tenth container (docker/orin/lib_guard.sh). The build loads every core
#     for ~1 h and must not share the Orin with a stack;
#   - a pinned input is missing or its sha256 differs (fetch_inputs.sh);
#   - the disk is short: < 5 GB free on /, < 40 GB free under Docker's root dir;
#   - a group the image must share with the host (video render dialout input
#     zed jtop) does not exist on the host;
#   - the Ubuntu archive snapshot (Dockerfile UBUNTU_SNAPSHOT) does not answer.
#     Documented fallback: ORIN_UBUNTU_SNAPSHOT=none builds from the live
#     archive (the image records every package version; label says none).
# After the build: the image's size against the budget (12 GB; over budget =
# exit 1, the image stays tagged for inspection), the largest layers, and the
# image ID appended to output/orin_container/logs/images.tsv.
#
# Host networking (CLAUDE.md "Orin Docker"). Log:
# output/orin_container/logs/build_<sha>[-norust]_<time>.log. Long (~50-70 min
# cold, ~10 min when only the sources changed): run it in the background.
# --dry-run: every check is run and reported, the sources are staged, the
# docker build command is printed; nothing is built.
# Env: ORIN_RUST (build | skip, default build), ORIN_UBUNTU_SNAPSHOT (a
# snapshot id or none; default the Dockerfile's), ORIN_INPUTS (inputs dir),
# ORIN_STAGE (staging root), ORIN_SIZE_BUDGET_GB (12), ORIN_GIT_BRANCH (see
# stage_sources.sh), ORIN_BUILD_ARGS (extra docker build args).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
DRY=0
if [ "${1:-}" = --dry-run ]; then DRY=1; shift; fi
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
SHA=$(git -C "$REPO" rev-parse --verify "${1:-HEAD}^{commit}")
SHORT=${SHA:0:7}
RUST=${ORIN_RUST:-build}
case "$RUST" in build) SUFFIX= ;; skip) SUFFIX=-norust ;; *) echo "ORIN_RUST must be build or skip, got '$RUST'" >&2; exit 2 ;; esac
INPUTS=${ORIN_INPUTS:-/scratch/fabiocar/p5d_downloads}
STAGE=${ORIN_STAGE:-/scratch/fabiocar/p5d_build}/$SHORT
L4T=/usr/lib/aarch64-linux-gnu/nvidia
IMAGE=f1tenth:jazzy-orin-$SHORT$SUFFIX
BUDGET_GB=${ORIN_SIZE_BUDGET_GB:-12}
LOGDIR=$REPO/output/orin_container/logs
LOG=$LOGDIR/build_${SHORT}${SUFFIX}_$(date +%Y%m%d_%H%M%S).log
refusals=0
refuse() {
  if [ "$DRY" = 1 ]; then echo "dry-run: WOULD REFUSE: $*" >&2; refusals=$((refusals + 1)); return 0; fi
  echo "refusing: $*" >&2
  exit 1
}

# ---- host
grep -q '^# R36 (release), REVISION: 5\.' /etc/nv_tegra_release 2>/dev/null \
  || refuse "not an L4T r36.5 host ($(head -1 /etc/nv_tegra_release 2>/dev/null || echo 'no /etc/nv_tegra_release'))"
[ -e "$L4T/libnvbufsurface.so.1.0.0" ] || refuse "$L4T has no L4T multimedia libs"

# ---- no stack runs (native ROS, Discovery Server, f1tenth container)
# shellcheck source=docker/orin/lib_guard.sh
source "$HERE/lib_guard.sh"
orin_guard || refuse "a stack runs (listed above); stop it first (docker/orin/host/stack_mode.sh status)"

# ---- pinned inputs
# (--dry-run downloads nothing: it only verifies what is there.)
fetch=("$HERE/fetch_inputs.sh")
[ "$DRY" = 0 ] || fetch+=(--verify-only)
"${fetch[@]}" "$INPUTS" > /dev/null || refuse "pinned inputs missing or wrong (fetch_inputs.sh $INPUTS)"

# ---- disk
free_gb() { df -P -BG "$1" 2>/dev/null | awk 'NR==2 {sub("G", "", $4); print $4}'; }
docker_root=$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)
root_free=$(free_gb /); docker_free=$(free_gb "$docker_root")
[ "${root_free:-0}" -ge 5 ] || refuse "/ has ${root_free:-?} GB free (< 5 GB)"
[ "${docker_free:-0}" -ge 40 ] || refuse "$docker_root has ${docker_free:-?} GB free (< 40 GB)"

# ---- the host's group IDs (the image's groups must equal them)
gid_args=()
for g in video render dialout input zed jtop; do
  gid=$(getent group "$g" | cut -d: -f3 || true)
  if [ -z "$gid" ]; then refuse "host group $g does not exist"; continue; fi
  gid_args+=(--build-arg "$(echo "$g" | tr '[:lower:]' '[:upper:]')_GID=$gid")
done

# ---- sources of the commit, and its docker/orin as the build context
if [ "$SHA" = "$(git -C "$REPO" rev-parse HEAD)" ] && [ -n "$(git -C "$REPO" status --porcelain -- src scripts docker/orin)" ]; then
  echo "note: uncommitted changes under src/, scripts/ or docker/orin/ are NOT in the image (built from $SHORT)"
fi
case "$STAGE/" in
  /scratch/fabiocar/*|"$HOME"/p5d_*) ;;
  *) echo "refusing: staging dir $STAGE is not under /scratch/fabiocar/ or ~/p5d_*" >&2; exit 1 ;;
esac
mkdir -p "$STAGE/ctx"
"$HERE/stage_sources.sh" "$SHA" "$STAGE/sources" > /dev/null
rm -rf -- "${STAGE:?}/ctx/docker"
git -C "$REPO" archive "$SHA" docker/orin | tar -x -C "$STAGE/ctx"
src_val() { awk -v k="$1" '$1 == k {print $2; exit}' "$STAGE/sources/SOURCES"; }

# ---- Ubuntu archive snapshot (L18)
SNAP=${ORIN_UBUNTU_SNAPSHOT:-$(sed -n 's/^ARG UBUNTU_SNAPSHOT=//p' "$STAGE/ctx/docker/orin/Dockerfile" | head -1)}
if [ "$SNAP" = none ]; then
  echo "note: ORIN_UBUNTU_SNAPSHOT=none, the documented fallback: live Ubuntu archive, versions recorded in the image"
elif ! [[ "$SNAP" =~ ^[0-9]{8}T[0-9]{6}Z$ ]]; then
  refuse "no valid Ubuntu snapshot id ('$SNAP'): the commit's Dockerfile has no ARG UBUNTU_SNAPSHOT=<YYYYMMDDTHHMMSSZ> and ORIN_UBUNTU_SNAPSHOT is not set"
elif ! curl -fsS -m 30 -o /dev/null "https://snapshot.ubuntu.com/ubuntu/$SNAP/dists/noble/InRelease"; then
  refuse "Ubuntu snapshot $SNAP does not answer (https://snapshot.ubuntu.com/ubuntu/$SNAP/). Retry, or build with the documented fallback ORIN_UBUNTU_SNAPSHOT=none and record it (output/orin_container/decisions.md)"
fi

build_cmd=(docker build --network=host --progress=plain
  --build-context inputs="$INPUTS"
  --build-context src="$STAGE/sources"
  --build-context l4t="$L4T"
  --build-arg RUST="$RUST"
  --build-arg UBUNTU_SNAPSHOT="$SNAP"
  "${gid_args[@]}"
  --build-arg GIT_SHA="$SHA"
  --build-arg GIT_BRANCH="$(src_val branch)"
  --build-arg GIT_DESCRIBE="$(src_val describe)"
  --build-arg SOURCES_TREE="$(src_val tree_sha256)"
  --build-arg CAR_SYNC_SHA256="$(src_val car_sync_sha256)"
  --build-arg CAR_SYNC_HEAD="$(src_val car_sync_native_head)"
  -t "$IMAGE" -f "$STAGE/ctx/docker/orin/Dockerfile" "$STAGE/ctx/docker/orin")
# shellcheck disable=SC2206  # ORIN_BUILD_ARGS is a word list on purpose
[ -z "${ORIN_BUILD_ARGS:-}" ] || build_cmd+=(${ORIN_BUILD_ARGS})

if [ "$DRY" = 1 ]; then
  echo "dry-run: image $IMAGE, staged sources in $STAGE/sources:"
  sed 's/^/  /' "$STAGE/sources/SOURCES" | grep -v '^  submodule'
  printf 'dry-run: would run:'; printf ' %q' "${build_cmd[@]}"; echo
  echo "dry-run: $refusals refusal(s); nothing built"
  exit 0
fi

mkdir -p "$LOGDIR"
{
  echo "image $IMAGE"
  echo "commit $SHA"
  echo "rust $RUST"
  echo "ubuntu_snapshot $SNAP"
  echo "started $(date -Iseconds)"
  echo "host $(head -1 /etc/nv_tegra_release); nvpmodel $(nvpmodel -q 2>/dev/null | head -1)"
  echo "gids ${gid_args[*]}"
  cat "$STAGE/sources/SOURCES"
} > "$LOG"
echo "building $IMAGE, log: $LOG"
"${build_cmd[@]}" >> "$LOG" 2>&1 || { echo "build FAILED: $IMAGE, see $LOG" | tee -a "$LOG" >&2; exit 1; }
echo "finished $(date -Iseconds)" >> "$LOG"

# ---- size, budget, record
id=$(docker image inspect "$IMAGE" --format '{{.Id}}')
size=$(docker image inspect "$IMAGE" --format '{{.Size}}')
{
  echo "build OK: $IMAGE $id"
  echo "size $size bytes ($(awk -v s="$size" 'BEGIN {printf "%.2f", s / 1e9}') GB), budget $BUDGET_GB GB"
  echo "largest layers:"
  docker history --no-trunc --format '{{.Size}}\t{{.CreatedBy}}' "$IMAGE" \
    | sort -h -r | head -12 | awk -F'\t' '{printf "  %-8s %s\n", $1, substr($2, 1, 140)}'
} | tee -a "$LOG"
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$(date -Iseconds)" "$IMAGE" "$id" "$SHA" "$RUST" "$SNAP" "$size" >> "$LOGDIR/images.tsv"
if [ "$size" -gt $((BUDGET_GB * 1000000000)) ]; then
  echo "OVER BUDGET: $IMAGE is $size bytes > $BUDGET_GB GB (the image stays tagged; see the layers above)" | tee -a "$LOG" >&2
  exit 1
fi
echo "next: docker/orin/image_report.sh $IMAGE; to run it: scripts/docker_orin.sh pin $IMAGE"
