# llm — local memory

Contracts/invariants only; narrative (four-source rule, D1–D4, go_to): `README.md`.

## Identity
`ament_python`; the package is literally `llm` (no `f1tenth_` prefix), the only one in
`f1tenth_intelligence`. 2 executables (`setup.py:34-37`): `llm_planner_node`, `llm_analytics`.
`data_files` globs are **extension-tight** — `launch/*.launch.py`, `config/*.yaml`,
`schemas/*.json`, `prompts/*.txt`: only those install, so a non-`.yaml` file dropped in
`config/` silently never reaches the share dir. `extras_require={'test': ['pytest']}` is why
`colcon test` finds anything.
Manual sweep fixture `test/commands.sample.txt` (one command per line, `#` comments; **20
commands since 2026-11-18** — a benchmark sweep covering every phase shape the prompt
teaches plus six commands that must be refused, no longer a smoke test) — **not installed**
by design, run by source path. `load_command_file` skips blank and `#` lines only: the
interior of a line is sent verbatim, so a trailing comment becomes part of the command.
Since 2026-11-18 (D-F) `llm_analytics` also copies whatever `--exec-file`/`--exec` it is
given into the run folder as `commands.txt` before the first command is sent (byte for
byte, so it can be handed back), and names it with a count and a sha256 in `report.md`.
`README.md` "End-to-end trial" is the only place the prerequisite order (env → upstream
server → prompt → read) is written down. Since 2026-11-19 the operator entry point is
`scripts/analytics.sh` (D-G): `up/status/down/probe` own the server (both `llama_server_*`
overrides autodetected, since the `stack_params.yaml` default is the Jetson's),
`one/sweep/node/live/idle` own the five paths, `replay/runs/last` read the results. It
sources `scripts/env/jazzy.sh` in-process, creates `analytics_runs/YYYY-MM-DD-<label>/`
before the tool starts and always points `--out`/`--events` into it, so the tool's own
truncating defaults are never used; no flag was added to `llm_analytics` to make it work.
Since 2026-11-25 it also has `up --model NAME` (passes `model:=` to the launch file, adopts
that entry's port, records `_server/current_model`) and `compare A B`, which runs two models
strictly sequentially into `analytics_runs/sequential/<date>-A-vs-B/` and renders one
`comparison.md` via `scripts/compare_reports.py` (a stdlib reader: it checks the command
file, the prompt and every request `body_sha256` before comparing anything). Both are
verified live 2026-11-25 (`sequential/2026-11-25-smoke-two-model-3`), which also forced
`config/models.yaml` to grow a second entry: `qwen25_3b_instruct` (n_ctx 32768, the GGUF
default) and `llama32_3b_instruct` (both `:8083`; the Llama entry carries `-c 16384`
because its GGUF's `n_ctx_train` is 131072, ~15 GB of KV, which the OOM killer ended
mid-load on this 16 GiB host).

## Topology
`launch/llm.launch.py` starts **llama-server only** (one `ExecuteProcess`, no `Node`).
`interrogation` → `interrogations.yaml` → `default_model` → `models.yaml` (`model_path`, `port`,
`extra_args`); `model`'s default `''` is that auto-select's sentinel, explicit `model:=` wins.
Only caller: the supervisor's `intelligence` component, gated by `enable_intelligence` (default
**true**, CLI-overridable, *not* one of the four stack-wide values); ~20–30 s GGUF load (~58 s cold).
`llm_planner_node` is never launch-file-driven — a CLI tool (`ros2 run llm llm_planner_node
"<cmd>" [--dry-run|--confirm]`). `llm_analytics` is a stdlib-only proxy, node → `:8084` → `:8083`;
its `--exec-file FILE` mode replays one command per line as the node's own v2 payload (no node,
no ROS, **v2 only** — the legacy `SYSTEM_PROMPT` path would fail the airlock by design).
`--out X.json` always also writes `X.md` in the same folder (Markdown: metadata + the
console summary block + one table row per exchange, columns prompt/temp/JSON/airlock/
wall-ms/tokens/tok-s); `--replay "$RUN/events.jsonl" --out "$RUN/replayed.json"` writes
`replayed.json` + `replayed.md`, so a re-analysis never overwrites the live report.

## Wires
`/mission/load_mission` (`LoadMission`), `/mission/start_mission`, `/mission/abort_mission`
(`Trigger`); 2 s wait, 5 s call. Pub `/test/plan_result` (`std_msgs/String`, JSON, q10) *before*
abort/load, plus a `kind: delivery` follow-up on failure. Stdout: exactly one final
`RESULT {json}`, `status ∈ ok|llm_error|translator_error|refused|node_error` (`refused` =
`EmptyPlanError`); only an unreachable server exits non-zero. HTTP `/completion` takes a raw
prompt, never a chat structure.

## Params
Five node params (no launch file sets them): `llm_url`, `llm_timeout_sec`, `planner_path`,
`test_plan_result_topic`, `go_to_enabled`. `stack_params.yaml` owns `mission_translator_*` (14 keys,
1:1 with `TranslatorConfig`, literals for import-in-isolation pinned by
`test_dataclass_defaults_match_stack_params`), `go_to_enabled`, `enable_intelligence`,
`start_server`, `model`, `interrogation`, `llama_server_path` + `llama_server_cwd` (separate on
purpose: cwd is the llama.cpp root, not `dirname(binary)`).

## Invariants
- The model authors **intent only**; every field is INTENT/CONFIG/DERIVED/CONST with a provenance
  entry — no fifth source, no "sensible default".
- `planner_path` (`legacy|v2`, default **v2**) binds prompt + validation + translator as one unit;
  a v2 failure never falls back, mismatched halves are refused by name.
- The v2 prompt is read at construction: a packaging break stops startup, not the first command.
- Non-empty `unsupported` ⇒ `requires_confirmation`, asked even under `--confirm`; no terminal ⇒
  refuse.
- `mission_config.parse_mission()` validates output — the loader is the contract.
- Missions: `.../missions/llm_generated/<mission_id>.json`, `mission_id =
  f'{prefix}_{sha1(canonical_json(intent))[:12]}'`; identical reused, differing refused, never
  overwritten.
- `reverse` is first-class (`plan_translate.py:858`, schema, prompt): `drive.mode 'reverse'`,
  `speed = abs(speed_straight)`, guard `distance` only, schema thresh 0.2–5.0, **no**
  `GUARD_THRESH_RANGE` entry.

## Hardcoded, not retunable
`LLAMA_URL` `http://127.0.0.1:8083/completion`, `LLAMA_TIMEOUT` 90.0 — module globals
reassigned once from params before the warm-up wait.
`MAX_INTENT_RETRIES` 2, timeout clamp 1/300 s, `WALL_DEBOUNCE_TICKS` 3,
`APPROACH_D_SAFE_BACKOFF_M` 0.1, `GUARD_THRESH_RANGE` wall 0.5–4 / front_object 0.5–5 / distance
0.2–20; `multipleOf` deliberately absent from the schema (float division false-rejects); analytics
`8083/8084` with `--upstream-timeout` 120 > the node's 90 so the node times out first.

## Drift register — documented, not fixed
1. `pytest test -m "not llm"` → 6 failed / 306 passed / 1 skipped.
2. Three `test_llm_planner_request.py` tests assert the pre-`stop`/`json_schema` payload.
3. `test_the_prompt_ships_all_its_worked_examples` → `assert 17 == 14` (the `reverse` examples).
4. `test_flake8` / `test_pep257` debt, `llm_planner_node.py` dominant.
5. `package.xml`: a lost `<!--` leaves lines 44-51 as raw character data; `std_msgs` imported
   (`llm_planner_node.py:117`) while its header says removed, no `<depend>` exists;
   `f1tenth_behavior` (`plan_translate.py:928`) undeclared; `package.xml.bak` tracked.
6. `README.md` predates `reverse`; its table omits `mission_translator_wall_default_distance`
   and `..._before_turn`.
7. `llm_analytics` is referenced by nothing else in the workspace; its discoverability now rests
   on `README.md`'s "End-to-end trial" + `test/commands.sample.txt` (2026-11-04). Its
   `console_script` is installed (symlink install, 2026-11-04) and resolves through
   `build/llm/llm -> src/`, so source edits need no rebuild. **Corrected 2026-11-12:** the
   "no GGUF, no `llama-server` binary" clause was true when written and is now false — the
   binary and `qwen2.5-3b-instruct-q5_k_m.gguf` live in `~/projects/llm/`, and `--exec-file` +
   `--airlock` are verified live (5 commands, 5/5 JSON-valid, 2 accepted / 3 rejected on a
   missing-`guard` model deviation). `--exec` is still unexercised live; go_to still unreliable
   on hardware. Captures are kept untracked in `analytics_runs/<label>/` (decision D-B),
   currently `NEXT-02-first-capture/` and `2026-11-18-20_first_attempt/` (**20** commands:
   20/20 JSON-valid, 0 failures, 12 accepted / 8 rejected, but only 9/20 semantically right
   and 3 accepts silently wrong — the airlock is not an intent oracle). Since 2026-11-18
   (D-F) every run also writes `commands.txt` beside its `--events`, byte-identical for
   `--exec-file`, so a folder carries its own input; `2026-11-18-20_first_attempt/` was
   retrofitted by hand before that existed.
8. Standalone `pytest test/test_llm_analytics.py` needs `src/f1tenth_behavior` on `PYTHONPATH`.
   `translate()` validates through `mission_config`, so without it the golden-intent airlock test
   *fails* (`TranslatorOutputError: ... f1tenth_behavior is not importable`) rather than skipping:
   81/83 unsourced, 83/83 from a sourced workspace (2026-11-18; both failures are the
   golden-intent airlock verdict, which needs `f1tenth_behavior` importable).
9. **Toolchain trap, workspace-wide — worked around, not fixed.** setuptools 84.2.1 in
   `~/.local/lib/python3.12/site-packages` (a `pip --user` upgrade shadowing the distro's 68.1.2)
   dropped `develop --uninstall/--editable/--build-directory`, which
   `colcon_core/task/python/build.py:210-232` calls, so a bare `colcon build` fails for **all 19
   python packages** (`error: option --uninstall not recognized`, from `_undo_develop()`). Build
   with `scripts/colcon_build.sh` instead: a `--system-site-packages` venv plus
   `PYTHONNOUSERSITE=1` hands `setup.py` the distro setuptools, since colcon runs it with
   `sys.executable`. 84.2.1 is still installed, so anything outside the wrapper still sees it.
