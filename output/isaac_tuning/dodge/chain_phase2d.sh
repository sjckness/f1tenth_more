#!/usr/bin/env bash
set -uo pipefail
cd ~/dev_ws/f1tenth_isaac_tuning
V=scripts/sim/isaac_tuning/tuning/variants_phase2d.yaml
OUT=output/isaac_tuning/dodge/ab_phase2c
E="python3 scripts/sim/isaac_tuning/evaluate.py --variants $V --d-levels 0,1,2 --seeds 0,1,2,3,4,5"
$E --only alpha_wake --families M --out $OUT/M_rest > $OUT.rest.log 2>&1
$E --only informed,all --families D,M --out $OUT/D_M_rest >> $OUT.rest.log 2>&1
$E --only alpha_wake_body --families D,M,S --out $OUT/body >> $OUT.rest.log 2>&1
