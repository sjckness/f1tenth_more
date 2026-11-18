#!/usr/bin/env bash
# Phase 2 A/B redo after the velocity gate fix (33c4119b): D levels 0-2 x seeds 0-5 for every variant,
# M1-M4 x 3 for the velocity variants, S1-S5 for alpha_return.
set -uo pipefail
cd ~/dev_ws/f1tenth_isaac_tuning
V=scripts/sim/isaac_tuning/tuning
OUT=output/isaac_tuning/dodge/ab_phase2c
python3 scripts/sim/isaac_tuning/evaluate.py --variants $V/variants_phase2c.yaml --only baseline \
  --families D --d-levels 0,1,2 --seeds 0,1,2,3,4,5 --out $OUT/D_only > $OUT.log 2>&1
python3 scripts/sim/isaac_tuning/evaluate.py --variants $V/variants_phase2c.yaml --only alpha,alpha_wake,informed,all \
  --families D,M --d-levels 0,1,2 --seeds 0,1,2,3,4,5 --out $OUT/D_M >> $OUT.log 2>&1
python3 scripts/sim/isaac_tuning/evaluate.py --variants $V/variants_phase2c.yaml --only alpha_return \
  --families D,M,S --d-levels 0,1,2 --seeds 0,1,2,3,4,5 --out $OUT/D_M_S >> $OUT.log 2>&1
