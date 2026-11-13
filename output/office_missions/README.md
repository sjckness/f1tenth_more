# office_missions — which LLM missions the HIL office world can actually run

Geometric and semantic analysis of the main HIL world (`sim/isaac/config/scenes/office.yaml`
→ `src/f1tenth_sim/worlds/husarion_office_2x.sdf`) for four campaign commands. Everything
is computed from the SDF, with no simulator involved, by
`sim/isaac/tools/office_missions.py`.

```bash
python3 sim/isaac/tools/office_missions.py --json output/office_missions \
    > output/office_missions/report.txt
```

Re-run it after any change to the SDF or to the stack numbers listed in that script's
header; it is the only thing that writes the files here.

| file | what |
|---|---|
| `office_hil.html` | the report: interactive floor plan, one tab per mission, plus the room / chair / door tables. Open it in a browser, it is self-contained. Also published at <https://claude.ai/artifact/WoHRuJqTysCDB2ehdM8KhY> |
| `report.txt` | the tool's own output: every primitive of every mission chain, step by step, with the numbers each verdict rests on |
| `map_geo.json` | the floor plan the HTML draws (wall and floor runs at 10 / 20 cm, object boxes, openings, rooms, the scene's sphere lights) |
| `results.json` | per mission: feasible yes/no, the waypoints, and the obstacle probes |

`office_hil.html` carries its own copy of `map_geo.json` inline, because a `file://` page
cannot fetch a sibling JSON. Regenerate the page after regenerating the JSON.

## The four commands, in one table

| command | verdict | start pose |
|---|---|---|
| go straight, enter the first door on the left and then approach the chair | feasible | `(9.33, −15.87) yaw 90°` |
| go straight at the wall, turn left and then approach the wall | feasible | `(16.83, −9.87) yaw 180°` |
| at the wall turn left, then go to the chair, then go straight 2 meters | 3 moves of 4 | `(16.83, −9.87) yaw 180°` |
| at the wall turn left, then go to the person, then enter the first door on the right | feasible, needs a person added | `(16.83, −9.87) yaw 180°` |

Three of the four share one start pose, which keeps the measurement frame identical across
the three `planner_controller_mode` runs of a `planner_compare` session.

## The three things that are not geometry

**"go straight 2 meters" after a chair cannot complete, anywhere.** `go_to chair` stops the
car 0.5 m from the chair's near edge *facing it* (`object_geometry.gap_min`), so the next
straight has an obstacle 0.40 m ahead: `distance_reached 2.0` never fires and the move
aborts on its 17 s timeout. This was checked against every pose in the world with a chair
visible after a left turn — none leaves 2 m clear. Either the 2 m goes *before* the chair,
or the abort is the measured outcome.

**An ordinal sends the command to `unsupported`.** `enter_door` with `side: left|right` is
supported end to end (`schemas/intent_v1.json`, `prompts/planner_prompt_porte.it.txt`,
`enter_door_enabled` default true), but that prompt routes ordinals — "the second door",
"the last door" — to `unsupported`, and *"the **first** door on the left"* reads as one. A
non-empty `unsupported` makes `llm_planner_node` ask for an interactive `[s/N]` and there is
no flag to skip it, so an unattended campaign stops there. "enter the door on the left"
avoids it and selects the same doorway: `side` already means the nearest one on that side.

**Light, not geometry, is the likely failure.** `office.yaml` has one dome at 400 and seven
`sphere_lights`, all on the x≈16.8 corridor and the (10, −19.4) door approach. The meeting
room, the NE kitchen and M-A's target chair sit 5.6–6.4 m from the nearest one; under
`RaytracedLighting`, with no indirect diffuse, YOLO has nothing to work with. `report.txt`
flags every waypoint over 5 m from a light. These are config lines, not code.

## Placing people as obstacles

The SDF has no people at all. They are added through scene config, either `person:` (one,
static) or `people.characters` (any number; a `path` of two identical points stands still,
because `people.py` treats `total <= 0` as a fixed pose). Both carry the `person` semantic
label, a capsule collider of r 0.175 m, and appear in `/sim/ground_truth/objects`.

To slip past one, the car's centre must stay 0.995 m from the person's centre
(0.175 + 0.5 `obstacle_class_margin_m` + 0.20 `car_radius` + 0.12 `obstacle_safety_margin_m`),
so with the 0.186 m half width a **1.18 m lateral gap** is needed. `report.txt` gives the
left and right gap at each probe point. The one that blocks — (11.63, −14.07) on M-B's
southbound leg, 0.45 m left and 0.80 m right — is the interesting one for the planner
comparison: the corridor's hard half-width is 0.64 m at the car and 1.13 m at the far end,
`p2p_geometric` stops at 0.5 m by design (`p2p_obstacle_stop_m`), and `corridor_rrt` only
routes around it if the detour fits inside that corridor.
