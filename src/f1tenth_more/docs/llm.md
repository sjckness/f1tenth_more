# f1tenth_intelligence/llm

`llama-server` bringup plus an LLM-driven MPC parameter tuner. The only
package under `f1tenth_intelligence/` (ROS package name is just `llm`, not
`f1tenth_llm` — an earlier, differently-named/-shaped pair of packages was
retired in favor of this one). Opt-in via `enable_llm`.

## Node

**`llm_mpc_tuner_node`** — manual-mode, terminal-driven MPC tuner with an
odometry feedback loop:
- Single-key terminal input selects a mode: `e` → SLOW (0.4 m/s target),
  `s` → FAST (2.0 m/s target), `p` → STOP (publishes zero
  `AckermannDriveStamped` on `stop_topic`, suspends the loop).
- Subscribes `odom_topic` (`nav_msgs/Odometry`, default `/odom`).
- Publishes `stop_topic` (`AckermannDriveStamped`, default `/teleop` — the
  mux's `joystick` lane).
- Talks to `llama-server`'s `/completion` HTTP endpoint (`mpc_url`,
  `python3-requests`) to decide tuning adjustments.
- **Applies changes by shelling out to the `ros2` CLI**, not a native
  `SetParameters` service client: `ros2 param dump <target_node>` to read
  current values, `ros2 param set <target_node> <name> <value>` once per
  parameter to apply them (`subprocess.run`, one call per param — no atomic
  multi-param apply). Checks `ros2 node info <target_node>` first and logs
  clearly if the target isn't found.
- `target_node`'s own in-code default is the stale `/andre_mpc_controller`
  — but `llm.launch.py` always overrides it from `stack_params.yaml`'s
  `mpc_node_name` (currently `/mpc_corr`), so the in-code default only
  matters for a bare `ros2 run` invocation without launch-time overrides.

## Launch file

**`llm.launch.py`** — the only one:
- Starts `llama-server` directly as an `ExecuteProcess` (no wrapper node),
  gated by `start_server` — if a server is already running standalone
  elsewhere, set `start_server:=false` and the interrogation node still
  launches (unconditionally, regardless of `start_server`) and points at it.
- `model` (launch arg) selects an entry from `config/models.yaml` — resolves
  `model_path` (expanded via `os.path.expanduser`), `port`, `extra_args`
  (appended verbatim to the `llama-server` CLI, e.g. `-ngl`).
- `interrogation` (launch arg) selects an entry from
  `config/interrogations.yaml` — resolves which executable to run and which
  params file to load.
- Builds `mpc_url` from the resolved port and injects it (plus `target_node`
  from `mpc_node_name`) as parameter **overrides on top of** the
  interrogation's own `params_file` — `launch_ros` applies later
  dict entries over earlier ones, so these two always win regardless of what
  (if anything) the params file itself sets for them. Keeps the port and MPC
  node name each a single source of truth instead of duplicated into
  `config/mpc_tuner_params.yaml`.
- Uses an `OpaqueFunction` (not pure launch substitutions) because the model/
  interrogation selection requires reading YAML file contents at launch
  time, not just string-substituting launch arguments.

## Config

| File | Holds |
|---|---|
| `config/models.yaml` | Model name → `{model_path, port, extra_args}`. One entry today: `qwen_mpc_pruned` (port 8082). |
| `config/interrogations.yaml` | Interrogation name → `{executable, params_file}`. One entry today: `mpc_tuner` → `llm_mpc_tuner_node`. No `package` field yet — everything resolves inside this one package; would need adding if a second package ever joins `f1tenth_intelligence`. |
| `config/mpc_tuner_params.yaml` | `llm_mpc_tuner_node`'s own params (`update_frequency`, `llm_timeout_sec`, `slow_target_velocity`, `fast_target_velocity`, etc.) — deliberately does **not** set `mpc_url`/`target_node` itself, see above. |

## Consumed `stack_params.yaml` keys

`start_server`, `model`, `interrogation`, `llama_server_path`,
`llama_server_cwd`, `mpc_node_name` — see each key's own `# Consumed by:`
comment in `stack_params.yaml`.

## Known limitations

- Parameter application is entirely via CLI subprocess calls
  (`ros2 param set`, one per parameter) — not atomic, and each call is a
  separate process spawn; a partial failure partway through a multi-param
  update leaves the target node with a mix of old and new values.
- `target_node`'s in-code default (`/andre_mpc_controller`) is stale
  relative to the actually-deployed MPC node (`/mpc_corr`) — harmless today
  since `llm.launch.py` always overrides it, but would silently target the
  wrong (non-running) node if this package were ever invoked via bare
  `ros2 run` without that override.
