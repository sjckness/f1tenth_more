# f1tenth_intelligence/llm

`llama-server` bringup plus an LLM-driven natural-language mission planner.
The only package under `f1tenth_intelligence/` (ROS package name is just
`llm`, not `f1tenth_llm` — an earlier, differently-named/-shaped pair of
packages was retired in favor of this one).

`llm_mpc_tuner_node` (a manual-mode, terminal-driven MPC parameter tuner —
the package's first interrogation) was removed outright, not deprioritized
— see git history if you need it back. `llm_planner_node` is the sole
interrogation now.

**`llm_planner_node` is never auto-launched, by anything** (component-
auto-start pass) — only `llama-server` itself can be. See "Component
supervisor / auto-start" below.

## Node

**`llm_planner_node`** — translates a natural-language command into a
`f1tenth_behavior` mission and loads/starts it through the real mission
service flow:

`LLM → normalize_plan → validate_plan → plan_translate.phases_to_mission()
→ write mission JSON → /mission/abort_mission (if one was already running)
→ /mission/load_mission → /mission/start_mission`

- `get_plan_from_llm()` talks to `llama-server`'s raw `/completion` HTTP
  endpoint (`llm_url` param, `python3-requests`) — a plain prompt string
  (`SYSTEM_PROMPT` + the command, in the same shape as `SYSTEM_PROMPT`'s own
  embedded few-shot examples), no chat/ChatML templating, no grammar/
  `json_schema` constraint, `temperature=0.0` for determinism.
- `SYSTEM_PROMPT`/`normalize_plan()`/`validate_plan()` define the LLM-facing
  phase vocabulary (`mode`/`guard`/`thresh`/`turn_sign`/`stop_at`/
  `stop_at_distance`) — see the module's own docstrings; not duplicated here.
- `plan_translate.phases_to_mission()` (same package, no ROS import, unit
  tested standalone) does the real translation into
  `f1tenth_behavior`'s mission schema (`moves`/`goal_distance`/`turn`/
  `stop_condition`) — keyed on each phase's `guard`, not `mode` (the real
  mission schema has no "mode" concept at all; see that module's own
  docstring for why the task that introduced it originally got this wrong).
- Command-line only, no interactive keyboard loop into a running mission:
  `--dry-run` (translate + write the mission file, skip the service calls),
  `--confirm` (prompt before calling `load_mission`/`start_mission`).
- `target_node`-style per-move amendment of an already-running mission
  (the earlier Ollama-backed version's `process_amendment`/`AMEND_PROMPT`)
  was removed, not gated: every command is a fresh mission — abort whatever
  was running, then translate/load/start the new one.

## Launch file

**`llm.launch.py`** — the only one. Starts `llama-server` ONLY (component-
auto-start pass: used to also launch the selected interrogation's node as a
persistent background `Node` action; removed outright, not just for the
supervisor path — see that file's own module docstring for why launching a
CLI tool this way was always broken, and the one consequence flagged there:
`llm_url` is no longer injected at launch time, so `llm_planner_node.py`'s
own hardcoded `LLAMA_URL` default is now the only source of truth for a bare
`ros2 run` invocation).
- Starts `llama-server` directly as an `ExecuteProcess` (no wrapper node),
  gated by `start_server` — defaults `true` (drop-the-start_server-override
  pass: this launch file's only remaining caller, the `intelligence`
  component, always wants it started, so there's no longer a real scenario
  wanting `false`). If a server is already running standalone elsewhere, set
  `start_server:=false` explicitly to attach to it instead.
- `interrogation` (launch arg) selects an entry from
  `config/interrogations.yaml` — resolves `default_model` (the only field
  this file still reads; `executable`/`params_file` are schema for a
  possible future node-auto-launch, not consumed today). Only one valid
  value today (`planner`); kept as a real launch arg rather than hardcoded
  away, see that file's own comment.
- `model` (launch arg) selects an entry from `config/models.yaml` — resolves
  `model_path` (expanded via `os.path.expanduser`), `port`, `extra_args`
  (appended verbatim to the `llama-server` CLI, e.g. `-ngl`). Defaults to
  `''`, a sentinel meaning "auto-select from the chosen interrogation's own
  `default_model` in `interrogations.yaml`" — an explicit `model:=<name>`
  always overrides this regardless of interrogation. Fixes a real gap:
  `model`/`interrogation` used to resolve fully independently, so picking a
  non-default interrogation with no explicit model override would silently
  serve it with whichever model happened to be `model`'s own hardcoded
  default — wrong output with no error at all.
- Uses an `OpaqueFunction` (not pure launch substitutions) because the
  model/interrogation selection requires reading YAML file contents at
  launch time, not just string-substituting launch arguments.

## Component supervisor / auto-start

`llm.launch.py` is registered as `f1tenth_bringup/config/components.yaml`'s
`intelligence` component, no `args` override needed (`start_server`'s own
default is now `'true'`, see above). Whether `intelligence` auto-starts with
the rest of `supervisor_bringup.launch.py` is gated on `enable_intelligence`
(default `true` as of the component-intelligence-autostart pass —
`intelligence` now auto-starts like every other component; pass
`enable_intelligence:=false` explicitly to skip the ~20-30s GGUF
load-into-GPU-memory cost when it isn't needed), a real `DeclareLaunchArgument`
threaded through the same way
`calibration` already is, NOT one of the 4 stack-wide branching values —
`enable_intelligence:=true`/`:=false` on the CLI actually reaches
`component_supervisor_node`. See `component_supervisor_node.py`'s own
`self.enable_intelligence`/`_CONDITIONAL_AUTO_START` and
`stack_params.yaml`'s `enable_intelligence` comment for the full reasoning.

`enable_llm` (the old stack-wide arg that used to gate `llm.launch.py` from
`stack_bringup.launch.py`, the other/older top-level bringup path) was
removed outright — `enable_intelligence` is now the only gate anywhere in
the workspace, and `stack_bringup.launch.py` no longer includes
`llm.launch.py` at all (see that file's own module docstring).

Only `llama-server` auto-starts this way — `llm_planner_node` never does,
regardless of `enable_intelligence`; run it by hand once `llama-server` is
confirmed live (`ros2 run llm llm_planner_node "<command>"`).

## Config

| File | Holds |
|---|---|
| `config/models.yaml` | Model name → `{model_path, port, extra_args}`. One entry today: `qwen25_3b_instruct` (stock Qwen2.5-3B-Instruct, q5_k_m, port 8083). |
| `config/interrogations.yaml` | Interrogation name → `{executable, params_file, default_model}`. One entry today: `planner` → `llm_planner_node`, `default_model: qwen25_3b_instruct`. `executable`/`params_file` unused by `llm.launch.py` since the component-auto-start pass (see above) — kept as schema, not live config. No `package` field yet — everything resolves inside this one package; would need adding if a second package ever joins `f1tenth_intelligence`. |
| `config/llm_planner_params.yaml` | Not currently loaded by anything (see its own top comment) — `llm_planner_node`'s ROS params (`llm_url`, `llm_timeout_sec`) would go here if node-auto-launch is ever deliberately reintroduced. |

## Consumed `stack_params.yaml` keys

`start_server`, `model`, `interrogation`, `llama_server_path`,
`llama_server_cwd` (by `llm.launch.py`), plus `enable_intelligence` (by
`component_supervisor_node.py`, via `supervisor_bringup.launch.py`) — see
each key's own `# Consumed by:` comment in `stack_params.yaml`.
(`mpc_node_name` was removed along with the node-auto-launch behavior above
— see git history.)

## Known limitations

- **Cold-start latency risk, not yet addressed**: the first `/completion`
  call against a freshly-started `llama-server` has been observed taking
  ~58s (GPU/kernel warm-up on top of the server's own already-documented
  ~20-30s startup warm-up) against `LLAMA_TIMEOUT`'s 60s default in
  `llm_planner_node.py` — uncomfortably close on the dev machine this was
  measured on, and plausibly *over* the timeout on the Jetson's weaker GPU.
  Neither the timeout nor a server-warm-up request has been changed to
  address this yet.
- Not yet soak-tested on the actual Jetson deployment target.
