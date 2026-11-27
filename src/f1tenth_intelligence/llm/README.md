# llm

> **Authoritative for topology, parameter routing and hardcoded values:** [`memory.md`](memory.md)
> (llama-server-only launch, the CLI-tool rule, `/mission/*` and `/test/plan_result` wires, the 5
> node params vs the `mission_translator_*` block, launch-unreachable constants); this file owns
> the four-source rule narrative, D1–D4, the schema-limit discussion and the analytics
> walkthrough — memory.md wins on conflict.

Natural-language mission planning: an Italian command in, a validated
`schema_version` 3.0 mission out (5.0 when the plan has a go_to phase).

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
| `speed_go_to` | 0.4 | `missions/go_to_person.json`'s speed |
| `go_to_acquire_timeout_sec` | 5.0 | `missions/go_to_person.json` |
| `go_to_lost_grace_sec` | 1.5 | `missions/go_to_person.json` |

`timeout_sec = clamp(ceil(expected_s * timeout_factor + timeout_floor), 1, 300)`,
where `expected_s` is `thresh / speed_straight` for the `distance` guard,
`open_guard_max_distance / speed_straight` for `wall` and `front_object`, and
`radians(turn_magnitude_deg) / nominal_yaw_rate` for a turn, and
`open_guard_max_distance / speed_go_to` for go_to.
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

## go_to and `go_to_enabled`

`go_to_enabled` in `stack_params.yaml` (default **true**, read once when
`llm_planner_node` starts; a ROS parameter of the same name overrides it for
one run) switches between two planners:

| | `true` | `false` |
|---|---|---|
| prompt | `prompts/planner_system_prompt.v2.it.txt` (teaches go_to) | `prompts/planner_system_prompt.v2.no_go_to.it.txt` (the prompt before go_to, byte for byte) |
| a go_to phase | a `go_to_object` move, stop `object_reached`, schema 5.0 | `UnsupportedIntentModeError`, fed back as retry text |
| refusal banner | adds the go_to example to the "Supportato" line | unchanged |

A go_to phase is `{"mode":"go_to","target":<COCO class>[,"thresh":<m>]}`.
`thresh` is the gap between the car's front and the object's near edge:

- absent: `default_gap(target)`, which is gap_min rounded up to 0.1 m;
- at least gap_min: used as given, rounded to 2 dp;
- below gap_min: raised to gap_min, rounded up to 1 cm, and an Italian `NOTA`
  printed with the requested and applied distances.

gap_min comes from `f1tenth_params.object_geometry.gap_limits`, the same
function the mission loader validates against. The mission_id hash covers the
resolved gaps too, so a margin change yields a new file rather than a refused
overwrite.

The model is told to refuse three kinds of request: classes not in the enum,
named individuals, and selections ("the second chair"). The live evals
(`docs/analysis/2026-09-17_go_to_live_eval.md`) show qwen2.5-3b does **not**
reliably refuse selections: "vai dalla persona a sinistra" was planned as
go_to the nearest person.

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
same filename by design rather than by collision. (For a go_to plan the hash
also covers the resolved gaps, which come from stack_params.)

## Analytics

`llm_analytics` is a recording proxy, not an instrumented node: it listens on a
second localhost port, the node is pointed at it with **one parameter**, and
every `/completion` exchange is written down on the way through.

```
llm_planner_node --> :8084 llm_analytics --> :8083 llama-server
```

Nothing in `llm_planner_node`, `plan_translate`, `llm.launch.py` or
`models.yaml` changes, which is why the numbers it reports are the numbers the
node really saw: the proxy records the payload it forwards (the exact prompt
string) and the body it receives (the exact model output, before
`plan_translate`'s airlock looks at it).

### Running it

```bash
# 1. the recorder runs the node itself -- dry run, nothing reaches the services
ros2 run llm llm_analytics --exec 'vai dritto al muro e gira a destra'

# 2. or watch a node started by hand; the banner prints this exact command
ros2 run llm llm_analytics
ros2 run llm llm_planner_node 'vai dritto' --dry-run \
    --ros-args -p llm_url:=http://127.0.0.1:8084/completion

# 3. re-analyse a capture with no server and no ROS at all
ros2 run llm llm_analytics --replay llm_analytics.jsonl

# 4. what does this llama-server build expose and return?
ros2 run llm llm_analytics --probe

# 5. sweep a file of commands: the node's own payload, node not started
ros2 run llm llm_analytics --exec-file commands.txt
```

`--exec` is a dry run unless `--live` is given, per-command timeout `--timeout`
(180 s), and it exits non-zero if the node timed out, failed, or any exchange
did. One JSON report (`--out`, default `llm_analytics.json`), its Markdown twin
written beside it (`report.json` -> `report.md`: metadata, the same summary
block, plus one table row per exchange), and one JSON line per exchange
(`--events`, default `llm_analytics.jsonl`) are written at the end of every run.

The run's own input is kept with the capture: before the first command goes out,
`llm_analytics` writes `commands.txt` beside `--events` -- `--exec-file` copied
byte for byte (comments and line endings included, so the copy can be handed
straight back to the same flag), `--exec` written one command per line. The
report names it with a count and a sha256 (its `- input:` line), so a run folder
can be re-run without guessing which version of the file produced it.
Nothing is written with `--no-events` (no folder) or by an idle recorder (no
input), and a failed copy is a warning, never an abort.

`--exec-file` is the cheap way to measure a lot of commands: one line per
command, blank lines and `#` comments skipped, and each line is wrapped exactly
as `llm_planner_node._completion()` wraps it (same system prompt file — the one
`plan_translate` picks for `go_to_enabled`, printed at the top with its sha256 —
same `n_predict`, temperature and `json_schema`, no `stop` list). The node is
never started, so it cannot move the robot, and no ROS is needed beyond the
package being built. Each exchange is recorded like any other, latency and
tokens per second included, and the console shows one line per command: HTTP
status, wall ms, tok/s, and the JSON verdict. A refused connection, a timeout, a
malformed or non-JSON answer is logged and the sweep continues; the exit code is
non-zero if any command went wrong.

`--exec-file` does not cover the legacy `SYSTEM_PROMPT` path: that prompt pairs
with `phases_to_mission()`, so the v2 airlock would reject everything it
produced. Use `--exec` for that path (it runs the real node).

`ros2 run llm llm_analytics` needs the package built, and this workspace is a
symlink install, so the installed package points back at the source tree: edit
`llm_analytics.py` and the next run picks it up, no rebuild. Build it with the
wrapper (a bare `colcon build` fails here — see `memory.md` drift 9):

```bash
scripts/colcon_build.sh build --packages-select llm
```

To run with no build, no install and no ROS at all, the identical `main()` is
reachable straight from the source tree, from the workspace root:

```bash
PYTHONPATH=src/f1tenth_intelligence/llm python3 -m llm.llm_analytics --replay llm_analytics.jsonl
```

### End-to-end trial: one prompt, step by step

On this machine the whole sequence below is also one command:
`scripts/analytics.sh up` once, then
`scripts/analytics.sh one "vai dritto e gira a sinistra alla parete"` -- it
sources the env, names the run folder, checks `/health` and prints the long form
it used (`scripts/analytics.sh help` lists every mode). The steps stay written
out because the wrapper hides nothing: same flags, same folders, same reports.
To compare two `config/models.yaml` entries instead, `scripts/analytics.sh
compare A B [FILE] --label NAME` runs them back to back -- one server at a time, the port
handed over between arms -- and writes
`analytics_runs/sequential/<date>-<label>/comparison.md` (read it with
`scripts/analytics.sh last --sequential`). Two entries ship by default,
`qwen25_3b_instruct` and `llama32_3b_instruct`, both on `:8083`; the Llama one carries
`-c 16384` in `extra_args` because its native 131072-token context would reserve ~15 GB of
KV cache and be killed by the OOM killer (2026-11-25), so the two arms legitimately differ
in `n_ctx` -- do not read that as a model difference.

The order below is the one that actually works, because the proxy only
*records*: it forwards `/completion` and writes down the answer. With no
upstream `llama-server` a run cannot produce a plan, only a refused
connection -- so the server comes first, the prompt second.

**1. Environment.** From the workspace root:

```bash
source scripts/env/jazzy.sh
```

`--exec-file` needs nothing beyond this (no node, no DDS, no stack). `--exec`
shells out to `ros2 run llm llm_planner_node`; for a standalone dry run,
`unset ROS_DISCOVERY_SERVER` is safe and avoids discovery warnings against a
Discovery Server that is not up -- a dry run needs no peers.

**2. An upstream server on `:8083`.** Already true when the stack is up (the
supervisor's `intelligence` component owns llama-server). Otherwise:

```bash
ros2 launch llm llm.launch.py      # server only, ~20-30 s GGUF load
curl -s http://127.0.0.1:8083/health   # {"status":"ok"} when ready
```

`llama_server_path:=` / `llama_server_cwd:=` are launch arguments and may point
at a llama.cpp tree elsewhere; the **model path is not** -- it comes from
`config/models.yaml` (`~/projects/llm/models/...`), so the GGUF must live there
or a `models.yaml` entry must point at it (then `model:=<name>`). A server on a
different port needs `--upstream http://127.0.0.1:<port>`.

**3. One prompt, no node** -- cheapest, one generation, and the mode to iterate
in:

```bash
cd /tmp && mkdir -p llm_trial && cd llm_trial
printf '%s\n' 'vai dritto e gira a sinistra alla parete' > commands.txt
ros2 run llm llm_analytics --exec-file commands.txt --airlock \
    --out report.json --events events.jsonl
```

`test/commands.sample.txt` is the same thing kept in the repo: 20 Italian commands
covering every plan shape the v2 prompt teaches, plus six that must be refused
(straight + wall turn, turn sequence, front object, reverse, go_to on COCO classes,
an unspeakable action, two ambiguous commands):

```bash
ros2 run llm llm_analytics --exec-file \
    src/f1tenth_intelligence/llm/test/commands.sample.txt --airlock
```

One line is one generation -- ~4 s warm, but ~50 s for the first one on a CPU-only
server (cold KV cache), so the whole file is 2-3 minutes and a single command stays
the cheap iteration loop. Either way the folder your `--events` points at ends up with
its own `commands.txt`: the repo sweep and an ad-hoc trial are reproduced the same way,
by the tool rather than by discipline.

`--airlock` is what turns the trial into a judgement: it runs the raw output
through `plan_translate.translate()` and prints accepted/rejected, so
"dritto + gira a sinistra alla parete" is either a `straight` + `wall_turn`
plan or a refusal you can read. It needs `f1tenth_behavior` importable -- a
sourced workspace is enough.

**4. The same prompt through the real node** (dry run by default, so nothing
reaches the mission services; you also get the node's own `RESULT` verdict and
its retry loop):

```bash
ros2 run llm llm_analytics --exec 'vai dritto e gira a sinistra alla parete' \
    --airlock --out report_exec.json
```

**5. Read it.** Console: the banner (upstream, `llm_url`, prompt file + sha256,
output paths), one line per command (`http 200 | 4200 ms | 38.4 tok/s |
JSON ok`), then the summary block, then the report path. The JSONL is the
per-exchange record; `--replay` re-analyses it with no server and no ROS.

Trial gotchas, cheapest to hit first:

- Outputs default to the **current directory** (`llm_analytics.json` /
  `.jsonl`) -- pass `--out`/`--events` so a trial does not litter the repo.
- **Exit code** is non-zero if the node timed out or failed, or if any exchange
  went wrong; `--exec-file` finishes the sweep and reports at the end.
- `--exec` **dry run still writes** the plan to
  `f1tenth_behavior/share/f1tenth_behavior/missions/llm_generated/<mission_id>.json`
  (for inspection only). `--live` is the flag that loads and starts it, i.e.
  moves the robot.
- **`--exec-file` is v2-only**; the legacy `SYSTEM_PROMPT` path pairs with
  `phases_to_mission()` and would be rejected by the v2 airlock -- use `--exec`.
- Timeouts stay ordered: node 90 s (hardcoded) < `--upstream-timeout` 120 <
  `--timeout` 180.
- Two runs are only comparable when the `prompts/...` name **and sha256**
  printed at the top match; `--no-go-to` swaps the prompt file.

### What it reports

| Section | Fields | Source |
|---|---|---|
| input | the prompt verbatim, split into system prompt / command / retry feedback; `n_predict`, `temperature`, `json_schema`, whether it is a retry | request body |
| thought | the raw model output verbatim, whether it parses as JSON (the node's own `raw_decode`), how much trailing text that discarded, moves/unsupported counts, `--airlock` verdict | response `content` |
| timings | prompt-eval and generation tokens/s and ms, `server_ms` (their sum), `overhead_ms` (proxy wall clock minus `server_ms`), prefill vs decode share | response `timings` + proxy clock |
| load | `n_predict` requested vs applied, `budget_used`, `hit_token_limit`, `stop_type`, tokens evaluated/predicted/cached | response + request |
| memory | KV occupancy (`/slots`, `/metrics`), server RSS/peak RSS/`delta` from `/proc/<pid>/status` | llama-server and `/proc` |

`overhead_ms` is the part of the call the server did not spend generating:
connection setup, queueing behind another slot, JSON encode/decode, the extra
loopback hop. On this machine that hop is a few hundred microseconds against a
generation that takes seconds, so the reported node-side latency stays honest.

Warm-up calls (`_wait_for_llama_server`, prompt `ciao`, `n_predict` 8) are
recorded and timed **separately** in the summary: they are the one call against
a cold server and would otherwise dominate the per-command averages.

### Caveats, stated rather than hidden

- **GPU utilisation is not measured.** No `nvidia-smi` dependency. KV occupancy
  and RSS are the proxy for how hard the box is working; `--ngl 999` means the
  model lives there, and `rss` alone will not show it.
- **`/metrics` requires llama-server `--metrics`**, which `llm.launch.py` does
  not pass. The poller tries once, disables itself permanently and says so; RSS
  and `/slots` still work and nothing is invented.
- **`--airlock` predicts one attempt**, with `go_to_enabled` as configured
  (`--no-go-to` to disable). The node's retry loop is still the node's business;
  the verdicts are there so a prompt change can be judged without reading logs.
- **An unknown KV occupancy is reported as unknown.** A `/slots` that gives no
  `n_past` prints `kv unknown of <n> tokens`, never `0%`; a slot that really
  reports `n_past: 0` prints `kv 0%`. Occupancy is read from both `n_past` at the
  slot's top level and `n_past` nested under `next_token`, since the shape moved
  between llama.cpp builds.
- The proxy is read-only and adds one localhost hop. If it dies the node simply
  cannot reach a server at `llm_url` — which is exactly what the node reports.