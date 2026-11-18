#!/usr/bin/env bash
# Extra seeds for baseline / alpha / alpha_return, then the phase 3 loop from alpha.
set -uo pipefail
cd ~/dev_ws/f1tenth_isaac_tuning
python3 scripts/sim/isaac_tuning/evaluate.py --variants scripts/sim/isaac_tuning/tuning/variants_seeds.yaml \
  --families D --d-levels 0,1,2 --seeds 3,4,5 --out output/isaac_tuning/dodge/ab_seeds \
  > output/isaac_tuning/dodge/ab_seeds.log 2>&1
python3 scripts/sim/isaac_tuning/tuning/dodge_loop.py --start scripts/sim/isaac_tuning/tuning/dodge_start_alpha.yaml \
  --d-fail 1 --base-ab output/isaac_tuning/dodge/ab_phase2/alpha/summary.json --hours 8 \
  > output/isaac_tuning/dodge/loop.log 2>&1
