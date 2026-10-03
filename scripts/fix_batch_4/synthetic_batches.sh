#!/usr/bin/env bash
# Fix batch 4 (H1): the stack-free lifecycle reproduction, 30 rounds each,
# Jazzy (host) and Humble (f1tenth_more:jetson container, Fast DDS 2.6.11),
# with a REUSED server across rounds (1 s gap) and a FRESH server per round.
set -o pipefail
S=~/dev_ws/f1tenth_more/scripts/fix_batch_4
O=~/dev_ws/f1tenth_more/output/fix_batch_4/synthetic
mkdir -p "$O"
source /opt/ros/jazzy/setup.bash
cd "$O"
R=${ROUNDS:-30}
# jazzy_reuse.jsonl already holds the 8-round trial; top it up to R.
have=$(cat jazzy_reuse.jsonl 2>/dev/null | wc -l)
[ "$have" -lt "$R" ] && [ -z "$HUMBLE_ONLY" ] && REUSE_DS=1 GAP=1 ROS_DOMAIN_ID=94 python3 $S/synthetic_lifecycle.py jazzy_reuse.jsonl $((R - have)) 30 11899 > jazzy_reuse.out 2>&1
[ -z "$HUMBLE_ONLY" ] && ROS_DOMAIN_ID=94 python3 $S/synthetic_lifecycle.py jazzy_fresh.jsonl "$R" 30 11899 > jazzy_fresh.out 2>&1
for mode in reuse fresh; do
  extra=""; [ $mode = reuse ] && extra="-e REUSE_DS=1 -e GAP=1"
  docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=95 $extra \
    -v "$S":/s -v "$O":/o f1tenth_more:jetson bash -c \
    "source /opt/ros/humble/setup.bash && cd /o && python3 /s/synthetic_lifecycle.py humble_$mode.jsonl $R 30 11898" \
    > humble_$mode.out 2>&1
done
