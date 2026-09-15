# llm

Natural-language mission planning: an Italian command in, a validated
`schema_version` 3.0 mission out.

```
comando (IT)
   -> LLM            -> intent JSON    schemas/intent_v1.json, schema-validated
   -> translator     -> mission v3.0   plan_translate.translate(), loader-validated
   -> stack          -> /mission/load_mission
```

Two rules govern the split, and everything below follows from them.

**The LLM authors intent only.** Speed, timeouts, abort policy, turn magnitude,
move ids and schema version are never emitted by the model. A model that can
author its own timeout can author a 600-second one.

**The translator invents nothing.** See the provenance rule.

## The four-source provenance rule

Every field of every emitted mission carries exactly one of four sources:

| Source | Meaning |
|---|---|
| `INTENT` | copied from the intent JSON |
| `CONFIG` | a named key of `TranslatorConfig` / `mission_translator_*` |
| `DERIVED` | a documented formula over INTENT and CONFIG |
| `CONST` | a literal fixed by the mission schema |

There is no fifth category, and in particular there is no "sensible default":
a value that cannot be traced to one of the four is not emitted at all and
`translate()` raises instead.

`translate(intent, explain=True)` returns a `provenance` dict mapping every
emitted path to its source, and for `DERIVED` fields the formula.
`test_intent_translate.py` asserts that **every leaf of every translated
mission has a provenance entry**, over the golden fixtures and over 400 random
in-range intents. That test is what stops invention creeping back in later.

### Why turn magnitude is not in the intent

The mission `llm_plan_1789054103` shipped a turn of `74.48451336700703`
degrees. That was not a hallucination. `llm_planner_node.SYSTEM_PROMPT` tells
the model outright to use `guard "turned" thresh 1.3`, `math.degrees(1.3)` is
`74.48451336700703`, and the legacy `phases_to_mission()` rendered it at full
float precision. Deleting turn magnitude from what the model may author deletes
that entire class of defect.

## Config

`mission_translator_*` in `f1tenth_params/config/stack_params.yaml`, mirrored
by `TranslatorConfig` so the module stays importable without an ament index.
`test_dataclass_defaults_match_stack_params` asserts the two agree, so they
cannot drift.

| Key | Default | Origin |
|---|---|---|
| `speed_straight` | 0.4 | real: `missions/wall_turn.json`'s run-up speed |
| `speed_turn` | 0.5 | real: `missions/wall_turn.json`'s turn speed |
| `turn_magnitude_deg` | 90.0 | choice, see D2 |
| `nominal_yaw_rate` | 0.22 | **measured**: 1.6091 rad in 7.3747 s, 2026-09-14 run |
| `open_guard_max_distance` | 10.0 | SYNTHETIC |
| `timeout_factor` | 3.0 | SYNTHETIC |
| `timeout_floor` | 5.0 | SYNTHETIC |
| `mission_id_prefix` | `"llm"` | choice |
| `post_turn_uses_straight` | true | see D1 |

`timeout_sec = clamp(ceil(expected_s * timeout_factor + timeout_floor), 1, 300)`,
where `expected_s` is `thresh / speed_straight` for the `distance` guard,
`open_guard_max_distance / speed_straight` for `wall` and `front_object`, and
`radians(turn_magnitude_deg) / nominal_yaw_rate` for a turn.
`open_guard_max_distance` exists because `wall` and `front_object` have no
bounded travel: if the wall is never seen the move must still end. The
generated missions before this work carried **no timeout at all**.

## Decisions D1-D4

**D1 - does `straight` work after a turn? Yes, and it always has.**
`post_turn_uses_straight` is `true`. The legacy translator already mapped a
post-turn continuation phase to `drive.mode "straight"`, because `MPC_corr`
re-anchors `psi_init_corridor` per move, so "straight" means "hold the heading
THIS move started with" -- which is what a post-turn phase wants. The old
prompt's rule "after a turn reuse wall_turn" was an artefact of
`f110_autonomy`'s never-re-anchored corridor and is gone from the v2 prompt.
`missions/wall_turn_then_straight.json` is the mission that exercises it and
**has not yet been run on hardware**; `false` restores the old workaround if it
turns out not to hold its heading.

**D2 - what happens with `turn_mag_deg` != 90? It is honoured. The premise that
it is not was wrong.** `condition_eval`'s `orientation_delta` branch ends the
move on `abs(turn_accum_deg) >= abs(value)` for whatever `value` it is given
(`condition_eval.py:379`), and `mission_config` requires that `value` to equal
`abs(drive.turn_mag_deg)`. Nothing clamps it to 90. `turn_magnitude_deg` is
therefore a genuine config choice and overrides are **not** rejected. It stays
at 90.0 because 90 is the only magnitude any of this has been run at, and the
MPC side of a larger turn is unverified -- not because anything forces it.

**D3 - `left` -> `+1.0`: documented by the stack, corroborated by odometry,
still unconfirmed physically.** `mission_config._parse_drive_spec` states the
convention outright: "drive.turn_sign of exactly -1.0 (right/CW) or +1.0
(left/CCW)". The 2026-09-14 run of `wall_turn.json` agrees: `turn_sign -1.0`
took `/odom` yaw from -0.014 rad to -1.62 rad, i.e. decreasing, i.e. clockwise.
What nobody has done is stand in the room and confirm that
clockwise-in-odom is what a person calls "right" -- an inverted yaw convention
upstream would satisfy all of the above and still turn the wrong way. That is
Stage 2 of `docs/bringup_checklist.md`.

**D4 - guard `wall` maps to `front_clearance`, and it does not touch the safety
path.** The full accepted enum is `mission_config.STOP_CONDITION_TYPES`:
`distance_reached`, `goal_reached`, `time_elapsed`, `object_seen`,
`object_cleared`, `obstacle_distance_below`, `front_clearance`,
`orientation_delta`, `manual`. There is no `front_wall_virtual`. The
`front_clearance` stop_condition reads **`/perception/front_distance`** -- the
object-excluded background distance -- not the identically named
`/perception/front_clearance`, which is `min(background, nearest obstacle)` and
is the one that belongs to the safety path. The type name is a misnomer kept
for schema compatibility. Guard `front_object` maps to
`obstacle_distance_below` with `forward_only: true`, because `front_clearance`
is object-blind by design and would never fire for a person or a chair.

## What the schema enforces, and what it does not

`schemas/intent_v1.json` is used for structured-output / constrained decoding
and as the first validation gate.

- **`multipleOf: 0.01` is deliberately absent.** It is the obvious way to say
  "at most 2 decimals" and it is wrong: JSON Schema evaluates it as float
  division, so `2.03 / 0.01 == 202.99999999999997` is not an integer and 2.03
  is rejected. 321 of the first 2000 valid 2-dp values fail that way under
  jsonschema 4.26.0. Decimal places are enforced in the translator by
  rounding, which cannot false-reject.
- **Per-guard `thresh` ranges** (`wall` 0.5-4.0, `front_object` 0.5-5.0,
  `distance` 0.2-20.0) are enforced in `GUARD_THRESH_RANGE`, because JSON
  Schema cannot condition `thresh` on `guard` without another `oneOf` layer.
  The schema carries only the blanket 0.2-20.0. One consequence worth knowing:
  `distance`'s per-guard range is *identical* to the blanket range, so a bad
  `distance` thresh is caught by the schema and raises `IntentSchemaError`,
  not `IntentRangeError`.
- Structured-output keyword support was **not** measured against llama-server
  in this pass; nothing here has been run against a live model. `minimum`,
  `maximum` and `enum` are all re-checked by the validator regardless, so a
  provider that drops them costs nothing but tokens.

## Failure behaviour

The translator never emits a partial or patched mission. It raises, and the
caller surfaces it:

| Condition | Exception |
|---|---|
| intent fails `intent_v1.json` | `IntentSchemaError` |
| `thresh` outside the per-guard range | `IntentRangeError(guard, value, range)` |
| empty `plan` | `EmptyPlanError` |
| output fails the loader | `TranslatorOutputError` (a translator bug; carries the document) |

`unsupported` non-empty is **not** a failure: the mission translates normally
and the result carries `requires_confirmation = True`. Such a mission **must
not auto-run** -- it is exactly the case where the operator believes they asked
for something else.

Output validation calls `mission_config.parse_mission()` rather than a separate
JSON Schema copy of v3.0. No such schema file exists in this repo, and writing
one would create a second spelling of the rules that drifts from the loader the
missions actually go through. The loader is the contract.

## Selecting a path

`planner_path` is a ROS parameter on `llm_planner_node`, `legacy` | `v2`,
**default `v2`**. It selects three things as one unit, declared once in
`PLANNER_PATH_SPEC`:

| | `legacy` | `v2` |
|---|---|---|
| prompt | `SYSTEM_PROMPT` (inline) | `prompts/planner_system_prompt.v2.it.txt` |
| validation | `validate_plan()` | `schemas/intent_v1.json` |
| translation | `phases_to_mission()` | `translate()` |

The halves cannot mix. A v2 response reaching the legacy path is refused by
name (`get_plan_from_llm` would otherwise have pulled `intent["plan"]` out of
the dict and built a plausible mission from the wrong semantics); a legacy
response reaching `translate()` fails the schema's `type: object`. The v2
prompt is read from the file at construction, before the llama-server wait, so
a packaging break stops startup rather than the first command.

The legacy path is kept as the fallback and is unchanged. **A v2 failure never
falls back to it** — a silent downgrade would produce a mission from a
different prompt with different semantics and the display would still look
reasonable.

## Failure handling

| Condition | Behaviour |
|---|---|
| `IntentSchemaError` / `IntentRangeError` | retry at most `MAX_INTENT_RETRIES` (2) more times, feeding the validator's error text back; every retry logged. Then give up, emit nothing. |
| `EmptyPlanError` | report the ambiguity to the operator, emit nothing. Never retried — ambiguity is the correct answer, not a model error. |
| `unsupported` non-empty | printed prominently; loading **requires an explicit confirmation** regardless of `--confirm`, and refuses outright with no terminal to ask on. |
| `TranslatorOutputError` | log the offending document in full. Our bug, not user error. |

## Generated plans on disk

`missions/llm_generated/` under `f1tenth_behavior`'s share directory, named by
`mission_id`, never `missions/` itself — that holds hand-written missions and
nothing in this repo writes there. The writer **refuses to overwrite a file
whose content differs**; identical content is reused, because the v2
`mission_id` is a hash of the intent, so re-issuing the same command hits the
same filename by design rather than by collision.
