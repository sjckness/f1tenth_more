#!/usr/bin/env bash
# Archive tags for every branch the consolidation merged, found already in
# campaign-final, or judged obsolete (output/consolidation/branches.md).
# PREPARED, NOT RUN. Run on linus from ~/dev_ws/f1tenth_more, under bash:
# the default shell there is zsh, where "$x:..." is a history modifier.
#
#   bash output/consolidation/archive_tags.sh          # create the tags locally
#   PUSH=1 bash output/consolidation/archive_tags.sh   # ... and push them to GitHub
#
# It creates annotated tags only: no branch is moved or deleted. Each sha is
# the tip recorded on 2026-10-22. A branch that has moved since is reported
# and skipped, never tagged at its new tip.
#
# Already archived on GitHub, so not repeated here:
#   archive/orin-snapshot-20261013, archive/orin-verify-campaign-dirty,
#   archive/mpc-corridor-split, archive/test-campaign.
# Not archived:
#   campaign-final and main (kept), the Orin's stack-campaign-final-2026-10-22
#   (checked out on the car), the Jazzy branches (kept; a separate line).
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# Rows: name, sha on 2026-10-22, the ref that must still point at it, why.
while read -r name sha ref why; do
  [ -n "$name" ] || continue
  tag="archive/${name}"
  if git rev-parse -q --verify "refs/tags/${tag}" >/dev/null; then
    echo "exists, skipped: ${tag}"; continue
  fi
  now=$(git rev-parse -q --verify "${ref}^{commit}" || echo gone)
  if [ "${now}" != "${sha}" ]; then
    echo "MOVED, skipped: ${ref} is ${now}, expected ${sha}"; continue
  fi
  git tag -a "${tag}" "${sha}" -m "archive: ${name} @ ${sha:0:8} (${why}); consolidation into main-consolidation, 2026-10-22"
  echo "tagged ${tag} -> ${sha:0:8}"
done <<'EOF'
two-llm-arch                 175da113b2c3a83afed275715153a41f344d333a  refs/heads/two-llm-arch                  merged
campaign-final-yolo-gpu      5792e2bad18d0dad0febc8c3f8cadfd336f4aa27  refs/heads/campaign-final-yolo-gpu       merged
comms-base                   02b245e6539792b6f5e950c4376ee5dbf7a39969  refs/heads/comms-base                    merged
pwa-control                  87e6f69bea5b034477976158d87b51d88a037f42  refs/heads/pwa-control                   already-in
telegram-control             b131b9bf8747075b528e398792a21a3d725f01e9  refs/heads/telegram-control              already-in
mpc_corr_v2_reverse          4f53ab8fde943f83f8b483379aad6ef1d13ccd76  refs/remotes/origin/mpc_corr_v2_reverse  already-in
night-reverse                3b8b0b830e84403bce067eddce3980af55962136  refs/heads/night-reverse                 already-in
night-walls                  120d0f4da6f9483be8cd0e4a41bce57fafdb8b4e  refs/heads/night-walls                   already-in
night-reports                2ed52169513beb338b549d0c6bdb2220cc250777  refs/heads/night-reports                 already-in
door-fixes                   486a15773f5d1f0aeb3b328eef7ad93ff394a5ac  refs/heads/door-fixes                    already-in
door-detection               c873247400c81296342129a54a4fb5b76647276a  refs/heads/door-detection                already-in
obstacles-v2                 d0028f2efbc8d021b74d732875e3027159ddd72c  refs/heads/obstacles-v2                  already-in
cf-next                      3feae4783d6664f386d938c5867eb9d8f9d6db4c  refs/heads/cf-next                       already-in
cpu-cleanup                  e41512a112c6ec52b337a09ee6d4a6c54155dcf8  refs/heads/cpu-cleanup                   already-in-by-patch
tmp/door-fixes-core          9b73a63b86cbfdb4e1d8bffe18067fdcb959de27  refs/heads/tmp/door-fixes-core           obsolete-in-by-patch
tmp/door-fixes-node          dd5c565b2eca9715c216acbb0d1363d63849395d  refs/heads/tmp/door-fixes-node           obsolete-in-by-patch
tmp/door-fixes-replay        35b4d8e425912958e4b3f8a7c3f4f17b14622b9f  refs/heads/tmp/door-fixes-replay         obsolete-in-by-patch
tmp/door-fixes-replay2       d8ece076f919d44de2d4547d9e0e39af1b4649a7  refs/heads/tmp/door-fixes-replay2        obsolete-in-by-patch
tmp/door-fixes-tracker       ea4beb468036398e107d9a77bd8c6d6e7207f164  refs/heads/tmp/door-fixes-tracker        obsolete-in-by-patch
tmp/door-fixes-unknown       dee1687bd62a1a273e4e380daca51c146d8113d9  refs/heads/tmp/door-fixes-unknown        obsolete-in-by-patch
campaign-final-doors         6c693ac281f2b238b74f18a9f1bcb154e91048df  refs/remotes/orin/campaign-final-doors   already-in-orin-only-name
orin-wip-2026-10-22          02cd0f2084dae6335bb855b97aa03d3489fcc4ba  refs/remotes/orin/orin-wip-2026-10-22    obsolete-save-work-orin-only
docs/branch-audit            953c89707968f662848130b85ae5c8fd929d6291  refs/remotes/orin/docs/branch-audit      obsolete-superseded-orin-only
test-logging                 573f313b1ed502a34584ea9cf89b09baa17837e4  refs/remotes/orin/test-logging           already-in-orin-only-name
llm                          e47e64683fb61e63a9c393c3df91cf3df89cb190  refs/remotes/orin/llm                    obsolete-orin-only-name
scene-graph                  e47e64683fb61e63a9c393c3df91cf3df89cb190  refs/remotes/orin/scene-graph            obsolete-orin-only-name
sick                         e47e64683fb61e63a9c393c3df91cf3df89cb190  refs/remotes/orin/sick                   obsolete-orin-only-name
EOF

if [ "${PUSH:-0}" = 1 ]; then
  # archive/orin-wip-2026-10-22 uploads about 180 MB (271 MB of run data and
  # videos, largest blob 17.8 MB). Leave it out of the push to keep it on
  # linus and the Orin only:  git push origin $(git tag -l 'archive/*' | grep -v orin-wip | sed 's|^|refs/tags/|')
  git push origin 'refs/tags/archive/*:refs/tags/archive/*'
fi

# Submodule fork branches judged obsolete (branches.md §3). Optional, in the
# vesc repository, not run by this script:
#   git -C src/f1tenth_hardware/vesc fetch origin
#   git -C src/f1tenth_hardware/vesc tag -a archive/humble-jetson   a84869c4925a584125b7708264a831ffa5387c01 -m 'archive: humble-jetson (old vesc_tuning, superseded by f0cd7a9)'
#   git -C src/f1tenth_hardware/vesc tag -a archive/vesc-tuner-test 2324c85724c5fdb025b4ce36648bc224f877c411 -m 'archive: vesc-tuner-test (in humble-jetson)'
#   git -C src/f1tenth_hardware/vesc push origin archive/humble-jetson archive/vesc-tuner-test
