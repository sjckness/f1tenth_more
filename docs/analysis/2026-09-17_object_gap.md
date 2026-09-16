# go_to_object as a gap, with a controller-derived minimum (2026-09-17)

Work order Part 2. Nominal-model rig numbers from
`src/f1tenth_control/mpc_controller/test/test_object_gap_closed_loop.py`
(`python3 -m pytest test/test_object_gap_closed_loop.py -s -k table`).

## Geometry (f1tenth_params/object_geometry.py, one definition for everyone)

- Car front: `nose_reach = L/2 + 0.40` = 0.5525 m ahead of base_link, the
  solver's farthest front point (mpc_solver now reads its offsets
  0.10/0.25/0.40 from the same module; L = `mpc_wheelbase_m` 0.305).
- Target edge: the semantic track's fused footprint width / 2. Tracks now
  carry it (`semantic_layer.py` EMA over associated detections, footprint rule
  shared with obstacle_projector_node); a track without a width falls back to
  a class nominal (person 0.50 m, chair 0.45 m, others 0.30 m).
  **Used: the fused track width** (not the obstacles_2d fallback).
- Controller goal: centre distance = `gap_m + nose_reach + r_target`
  (ObjectGoal.standoff). object_reached's live r = actual gap − commanded gap.
- `gap_min(class) = car_radius + obstacle_safety_margin_m + class margin +
  object_gap_settle_buffer_m` = 0.20 + 0.12 + 0 + 0.10 = **0.42 m** for every
  class today (margin `{}`, no m\* in Part 1). Mission load rejects smaller
  gaps with that sum in the message. Default gap = gap_min rounded up to
  0.1 m = **0.5 m**.

## Verification (margin 0; m\* does not exist)

| class | commanded | gap m | gap_min | settled gap | min live r | object_reached |
|---|---|---|---|---|---|---|
| person | rest probe (gap 0) | 0.00 | 0.42 | 0.268 | +0.268 | no |
| person | gap_min | 0.42 | 0.42 | 0.427 | +0.009 | yes |
| person | default gap | 0.50 | 0.42 | 0.496 | −0.001 | yes |
| chair | rest probe (gap 0) | 0.00 | 0.42 | 0.268 | +0.268 | no |
| chair | gap_min | 0.42 | 0.42 | 0.428 | +0.010 | yes |
| chair | default gap | 0.50 | 0.42 | 0.497 | +0.000 | yes |

The rest probe is where w_obs actually holds the car's front: 0.268 m, 0.05
inside the analytic car_radius + avoidance_margin (0.32), because the soft
penalty lets it get closer before balancing. gap_min sits 0.152 m above it,
more than the 0.10 reach tolerance, so `settle_buffer` 0.10 needed no change.

## Default gap across scenarios

| class | scenario | settled gap | min live r | reached tick |
|---|---|---|---|---|
| person | ahead 4 m | 0.496 | −0.001 | 59 |
| person | 45° off, 4 m | 0.499 | +0.002 | 60 |
| person | 70° off, 2.5 m | 0.420 | −0.079 | 32 |
| person | jitter 5 cm @ 12.5 Hz | 0.496 | −0.064 | 55 |
| chair | ahead 4 m | 0.497 | +0.000 | 60 |
| chair | 45° off, 4 m | 0.498 | +0.000 | 60 |
| chair | 70° off, 2.5 m | 0.419 | −0.079 | 33 |
| chair | jitter 5 cm @ 12.5 Hz | 0.497 | −0.062 | 57 |

object_reached fires in all eight. The 70°-off approach overshoots to 0.42 m
(still at gap_min). In the real stack object_reached ends the move and the
hold stops the car there.

## Not measured

`object_reach_max_target_age_sec` stays 1.0, marked UNMEASURED: the bench run
that was to measure target_age did not happen.
