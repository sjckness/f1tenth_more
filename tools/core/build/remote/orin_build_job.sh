#!/bin/bash
# Gira SULL'ORIN, staccato (setsid nohup), lanciato da tools/core/build/build.sh. Costruisce l'immagine
# di UN commit con il docker/orin/build.sh di quel commit, in un worktree temporaneo del repo di build
# (mai il checkout nativo, mai il worktree del repo di build), poi la testa e la etichetta.
#
#   orin_build_job.sh SHA RUST TEST BUILD_REPO BUILD_ROOT KEEP TEST_DOMAIN EXCLUDE_RE
#
# Passi: worktree add --detach -> ORIN_RUST=RUST docker/orin/build.sh SHA -> pytest dei package
# (TEST changed|all|none) in un container usa e getta dell'immagine (rete none, ROS_DOMAIN_ID
# TEST_DOMAIN) -> se i test passano, la stessa immagine riceve la label f1tenth.tested=true (un
# livello solo di metadati: FROM + LABEL) -> potatura dei worktree vecchi (tiene KEEP).
# Stato in BUILD_ROOT/<sha7>.status: "running", poi "ok <immagine> <id>" o "fail <passo> <rc>".
set -euo pipefail
SHA=$1 RUST=$2 TEST=$3 REPO=${4/#\~/$HOME} ROOT=$5 KEEP=$6 DOMAIN=$7 EXCL=$8
SHORT=${SHA:0:7}
SUFFIX=; [ "$RUST" = skip ] && SUFFIX=-norust
IMAGE=f1tenth:jazzy-orin-$SHORT$SUFFIX
WT=$ROOT/$SHORT
STATUS=$ROOT/$SHORT.status
mkdir -p "$ROOT"
case "$ROOT/" in /scratch/fabiocar/*|"$HOME"/p5d_*) ;; *) echo "refusing: BUILD_ROOT $ROOT fuori da /scratch/fabiocar o ~/p5d_*" >&2; exit 1 ;; esac
say() { echo "$(date +%H:%M:%S) [f1t build] $*"; }
fail() { echo "fail $1 $2" > "$STATUS"; say "FALLITO al passo $1 (exit $2)"; exit "$2"; }
echo running > "$STATUS"

say "commit $SHA, immagine $IMAGE, test $TEST"
if [ -d "$WT" ]; then
  [ "$(git -C "$WT" rev-parse HEAD)" = "$SHA" ] || fail worktree 1
  say "worktree $WT gia' presente"
else
  git -C "$REPO" worktree add -q --detach "$WT" "$SHA" || fail worktree $?
fi
git -C "$WT" submodule status >/dev/null 2>&1 || true

say "build: ORIN_RUST=$RUST $WT/docker/orin/build.sh $SHA"
rc=0; ORIN_RUST=$RUST "$WT/docker/orin/build.sh" "$SHA" || rc=$?
[ "$rc" = 0 ] || fail build "$rc"
docker image inspect "$IMAGE" >/dev/null 2>&1 || { say "immagine $IMAGE assente dopo la build"; fail build 1; }

# ---- test nell'immagine
pkgs=()
if [ "$TEST" != none ]; then
  base=
  if [ "$TEST" = changed ] && [ -f "$HOME/.config/f1tenth/car_image" ]; then
    pin=$(awk '$1=="tag"{print $2}' "$HOME/.config/f1tenth/car_image")
    base=$(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$pin" 2>/dev/null || true)
    git -C "$WT" cat-file -e "${base}^{commit}" 2>/dev/null || base=
  fi
  if [ -n "$base" ]; then
    say "test dei package cambiati tra ${base:0:8} (immagine fissata) e ${SHA:0:8}"
    files=$(git -C "$WT" diff --name-only "$base" "$SHA" -- src)
  else
    say "test di tutti i moduli"
    files=$(git -C "$WT" ls-files src)
  fi
  dirs=$(printf '%s\n' "$files" | while read -r f; do
    d=$(dirname "$f")
    while [ "$d" != . ] && [ "$d" != src ] && [ ! -f "$WT/$d/package.xml" ]; do d=$(dirname "$d"); done
    [ -f "$WT/$d/package.xml" ] && echo "$d"
  done | sort -u | grep -v -E "$EXCL" || true)
  for d in $dirs; do [ -d "$WT/$d/test" ] && pkgs+=("$d/test"); done
fi
tests="none"
if [ ${#pkgs[@]} -gt 0 ]; then
  say "pytest in $IMAGE (rete none, dominio $DOMAIN): ${pkgs[*]}"
  rc=0
  docker run --rm --network none -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --entrypoint /opt/f1tenth_ws/entrypoint.sh "$IMAGE" \
    bash -c 'cd /opt/f1tenth_ws && python3 -m pytest -q -p no:cacheprovider "$@" 2>&1 | tail -40; exit "${PIPESTATUS[0]}"' _ "${pkgs[@]}" \
    < /dev/null || rc=$?
  [ "$rc" = 0 ] || fail test "$rc"
  tests="${pkgs[*]}"
elif [ "$TEST" != none ]; then
  say "nessun package con test/ da provare"
fi

# ---- label tested (FROM + LABEL: solo metadati, stesso contenuto)
say "label f1tenth.tested=true su $IMAGE"
printf 'FROM %s\nLABEL f1tenth.tested="true" f1tenth.tested_at="%s" f1tenth.tests="%s"\n' \
  "$IMAGE" "$(date -Iseconds)" "$(printf '%s' "$tests" | tr '"' "'" | cut -c1-400)" \
  | docker build -q --network=host -t "$IMAGE" - >/dev/null || fail label $?
id=$(docker image inspect -f '{{.Id}}' "$IMAGE")

# ---- potatura dei worktree di build piu' vecchi (git worktree remove, mai rm -rf)
mapfile -t old < <(ls -1dt "$ROOT"/*/ 2>/dev/null | tail -n +$((KEEP + 1)))
for d in "${old[@]}"; do
  d=${d%/}
  case "$d/" in "$ROOT"/*) git -C "$REPO" worktree remove --force "$d" 2>/dev/null && say "rimosso worktree $d" ;; esac
done
git -C "$REPO" worktree prune
echo "ok $IMAGE $id" > "$STATUS"
say "OK $IMAGE $id"
