## Stream C: second LLM (two-llm-arch), commits 4612513..36e6a87

Branch `two-llm-arch` now has an LLM2 mock that tunes MPC parameters. It is off by default and sits in 6 commits on top of `campaign-final` (6002cb0): 30 files, +3067/−15.

In a 5-minute run against a real llama-server on linus, it gave 151/151 valid replies, all exact echoes. Median latency was 862 ms against the 2 s period. A real MPCController applied every update and ended with its weights unchanged.

Both independent reviews were addressed in 2 extra commits. Two majors were fixed:
- The LLM2 switch and whitelist can no longer be changed at runtime with no effect.
- With reverse on, LLM2 can no longer break the reverse planner's w_v invariant.

Three minors were fixed: `w_obs` is no longer tunable, two test gaps are closed, and stale comments are updated. The merged-seq case is documented.

Nothing was built or run on Humble or the Orin. The sim comparison was skipped, because the plan excludes the stack and the sim on linus. Nothing pushed; no sudo needed. Every server and node I started is stopped, and ports 8082 to 8084 are free.

### Commits
| sha | subject |
|---|---|
| 4612513 | two-llm: MpcParamUpdate message, LLM2 server config, enable_llm2 launch args |
| ea78a86 | two-llm: llm2_mpc_param_node, the LLM2 mock MPC-parameter interrogation |
| 731e090 | two-llm: MPC_corr subscription + whitelist for LLM2 parameter updates |
| 695f9f6 | two-llm: tests + docs for the LLM2 mock (C6, C7) |
| 3fcc033 | two-llm: review fixes - read-only LLM2 params, reverse w_v limit, w_obs never tunable |
| 36e6a87 | two-llm: review fixes - tests for non-echo replies and the enable_llm2 gate, stale comments |

The full guide is `output/two_llm/README.md`: how to enable it, the topics, the measurements and what blocks real tuning. The recon is `output/two_llm/recon.md`.

### How to enable it
There are two independent switches, both `false` by default:

| Switch | Where | What it does |
|---|---|---|
| `enable_llm2` | `stack_bringup` launch argument | Starts the second llama-server and `llm2_mpc_param_node`. Nothing else. |
| `llm2_params_enabled` | `mpc_corr` parameter; launch argument forwarded by `navigation.launch.py` | Lets mpc_corr subscribe and apply updates. When false there is no subscription at all. |

On the stack_bringup path:
```
ros2 launch f1tenth_bringup stack_bringup.launch.py enable_llm2:=true llm2_params_enabled:=true
```

On the supervisor path:
- Run `ros2 launch llm llm2.launch.py` by hand.
- Add `llm2_params_enabled: 'true'` under the `navigation` entry's args in `components.yaml`.

**`llm2_params_enabled` and the whitelist are read-only at runtime.** A `ros2 param set` on them is refused. To stop LLM2 changing the controller at runtime, stop the `llm2.launch.py` producer.

### Recon summary (C1)
- **LLM1 is `llm_planner_node`.** Its server is started by the `intelligence` component (`enable_intelligence`, default true) on **port 8083**: `models.yaml` entry `qwen25_3b_instruct`, and `llm_planner_node.py:165`.
  - That is why **LLM2 defaults to 8084**, not the plan's 8083.
  - `enable_llm` was removed from stack_bringup.
  - The old `llm_mpc_tuner` (port 8082) is gone.
- **MPC weights:**
  - Declared through `_w()` and read once into `self.weights`.
  - No parameter callback.
  - `scale_stage_weights` applies ×7/N = 0.35 to the stage terms on every solve.
  - Single-threaded `rclpy.spin`.
  - The tick boundary is the top of `control_loop`.
- **`vdes` is per-move state, not a parameter.**
- **The Qwen2.5-3B Q5_K_M model was already in `models.yaml` and on the Orin.**

### Model and server (C2, C3)
- **Model:** `Qwen/Qwen2.5-3B-Instruct-GGUF` Q5_K_M, **2438740384 B**, sha256 `2c63dde5f2c9ab1fd64d47dee2d34dade6ba9ff62442d1d20b5342310c982081`. Identical on Hugging Face, linus and the Orin.
- **New `models.yaml` entry `qwen2.5-3b-instruct-q5km`:** the same file as LLM1, with `-ngl 999 -np 1 -c 4096`.
- **llama.cpp on linus:** the official b9148 Vulkan prebuilt, the same build as the Orin's, running on the RTX 5050 GPU.
- **`llm/launch/llm2.launch.py`** starts the server and the node and passes the same port to both. `stack_bringup.launch.py` includes it only under `IfCondition(enable_llm2)`; this is now pinned by a test.
- **Message `f1tenth_messages/MpcParamUpdate.msg`:** `header`, `seq`, `source`, `names[]`, `values[]`.

### LLM2 node (C4)
`llm2_mpc_param_node`, with its logic in `llm2_protocol.py`.

**Each tick (2 s):**
- **If a request is still in flight,** the tick is skipped and counted, never queued.
- **Otherwise,** a worker thread sends `POST /completion` with:
  - temperature 0
  - a `json_schema` with exactly the expected keys and `additionalProperties: false`
  - `cache_prompt`, with a fixed ChatML prefix ahead of the JSON
- **Validation:** one JSON object, the same key set, finite numbers.
  - Valid replies go to `/llm2/mpc_params`, including a valid reply that is not an exact echo (now tested).
  - Invalid replies publish nothing.

**`/llm2/status`** is a JSON string, one per tick. Outcomes are ok / invalid / timeout / error / skipped. Each message also carries latency, prefill and generation times, echo_match and counters.

**Mock values (today's launch-path values):**

| Parameter | Value |
|---|---|
| w_term | 9.0 |
| w_psi | 4.5 |
| w_psi_stage | 4.7736 |
| w_corr | 3.5714 |
| w_v | 4.2857 |
| w_du_delta | 8.33 |

### mpc_corr side (C5)
- **Where the code is:** `mpc_controller/llm2_params.py`, plus one block in `MPC_corr.py`:
  - `_llm2_setup` at l.6887 and the read-only declarations at l.6906.
  - `_llm2_params_callback` at l.6942.
  - `_llm2_apply_pending` at l.6966.
  - The calls at l.2198 (in `__init__`) and l.4918 (top of `control_loop`).
- **Whitelist** (declared units; `stack_params.yaml:2905-2913`):

| Weight | Range |
|---|---|
| w_term | [2, 20] |
| w_psi | [1, 10] |
| w_psi_stage | [1, 12] |
| w_corr | [0.5, 9] |
| w_v | [1, 10] |
| w_du_delta | [2, 23.81] |

- **Only `mpc_w_*` weights can be whitelisted, and never `mpc_w_obs`.** Anything else stops mpc_corr at startup with a FATAL log.
- **An update is all or nothing.** One bad name, NaN/inf or out-of-range value rejects the whole message. Nothing is clamped.
- **With allow_reverse on,** any update with `mpc_w_v` above `w_rev/(2·vdes_default)` = 4.3 is also rejected whole (`llm2_params.py:158`).
- **Applying an update:** the callback only parks it. The next tick applies it through `set_parameters_atomically` and a new `self.weights` dict, then echoes it on `/mpc/applied_params` with the same seq.
- **`mission_logger` records all three topics.**

### Measured on linus
The RTX 5050 had 3.6 GB already used by other processes.

**5-minute run at 0.5 Hz**, 2026-10-22 00:41–00:47, domain 73, localhost only:

| | value |
|---|---|
| ticks in the 300 s after the first valid reply | 151: **151 ok**, 0 invalid / timeout / error / skipped |
| **echo_match** | **100 %** (151/151; 161/161 over the whole run) |
| latency, ms | min **854**, median **862**, p90 **870**, p99 **976**, max **1062** (whole run: 1187) |
| against the 2 s period | none over 2 s; the slowest used 53 % of the period |
| time split | prefill median 10.5 ms (214 tokens cached); generation 850 ms for 82 tokens (96.5 tok/s) |
| start-up | 9 ticks got HTTP 503 while the model loaded (~20 s) |
| end to end | 161 `/llm2/mpc_params` → 161 `/mpc/applied_params`; publish to application median 18.6 ms, max 122.5 ms; weights unchanged |
| GPU | 3655 MiB used before, max 6294 MiB during (+2.6 GB); llama-server RSS 0.4–1.1 GB; node 62 MB |

**First request after a server start:** 26.4 s in stage 1, because Vulkan builds its pipelines on first use. In later runs it took 79–145 ms. The node's 90 s warm-up timeout covers both.

**Both servers loaded:** LLM1's stand-in ran on the CPU only, because two models do not fit in the free VRAM.
- GPU: 6177 MiB used in total, of which LLM2 took 2587 MiB.
- RAM: the stand-in's RSS was 3644 MB.
- LLM2's own entry needs 144 MiB of KV cache, against 1152 MiB with LLM1's flags.
- **Orin estimate:** about 6.6 GB of 62.8 GB, not measured. The tegrastats commands for tomorrow are in README section 4.

### Review fixes (3fcc033, 36e6a87)
I checked every finding against the code first. All of them held up.

- **Major: runtime switch ignored.**
  - Reproduced: `set_parameters(llm2_params_enabled=False)` succeeded, yet updates kept being applied.
  - Fix: the 4 LLM2 parameters are now `ParameterDescriptor(read_only=True)` (Humble-compatible), so a set is refused with "read-only parameter". `rcl_interfaces` is declared in `package.xml`.
- **Major: reverse invariant.** Each offending update is rejected whole, as above.
  - I did not refuse the start instead: that would make the default whitelist unusable with reverse on.
  - The startup warning now says updates above 4.3 are rejected.
  - Pre-existing, not caused by LLM2: a move faster than 0.5 m/s already breaks the invariant with the shipped weights.
- **Minor: `w_obs`.**
  - Confirmed in `mpc_solver.py:1176-1186`: the obstacle term is a soft cost with no hard row behind it.
  - It is now in `NEVER_TUNABLE_WEIGHTS` (`llm2_params.py:51`). The docstring that claimed hard constraints bound every solution is corrected.
- **Minor: merged seq.**
  - Documented in README section 2.
  - The merge warning in mpc_corr's log now names the seq that will not appear on `/mpc/applied_params`.
  - The message format is unchanged.
- **Minor: test gaps.**
  - The non-echo reply test catches both reviewer mutations.
  - The new `f1tenth_bringup/test/test_stack_bringup_llm2_gate.py` checks the include's condition: false by default and with `:=false`, true with `:=true`.
  - One test was renamed after what it asserts.
- **Minor: stale comments** in `components.yaml:166` and `stack_params.yaml:2807`.

### Tests (compared against night-start/campaign-final)
- **Stream C tests: 123/123 pass:**
  - mpc_controller: 66
  - llm: 44
  - f1tenth_bringup: 4
  - f1tenth_control: 9
- **Mutation checks:** 27 deliberate bugs over the stream, each caught by at least one test (8/8 for the review fixes).

| Package | Result | Against baseline |
|---|---|---|
| mpc_controller | 1452 passed, 13 failed (344 s) | Same failures as before, the 13 known; flake8 1175 and pep257 1302 equal base |
| llm | 286 passed, 14 failed | Same 14 as base; flake8 165 and pep257 274 equal a HEAD-archive run |
| f1tenth_bringup | 52 passed, 3 failed | The 3 are lint tests with flake8/pep257 equal to base; copyright 20 → 21 files (the new test has no header, like every file in the package) |
| f1tenth_params | 80 passed | — |
| f1tenth_control | 64 passed | — |
| f1tenth_navigation | 12 passed | — |
| f1tenth_logger | 70 passed, 3 failed | The 3 are lint tests with counts equal to base |

- **Build after the fixes:** 4/4 in the worktree.
- **Stage 4:** a clean non-symlink build of all 8 packages: 8/8.
- **Python 3.10:** every changed file parses with the 3.10 grammar.

### What blocks real tuning (README section 5)
- **Weights are read once at startup.** There is no parameter callback, so a hand `ros2 param set` on `mpc_w_*` changes nothing live.
- **Declared and effective values differ** by ×0.35, and `rate_weight_factor` scales `w_du_delta` again.
- **`w_corr` does nothing on object approaches:** arc corridors override it.
- **No lateral-offset reference** exists to tune.
- **No speed knob:** `vdes` is not a parameter.
- **`w_v` with reverse on can only go down,** unless `w_rev` moves with it.
- **The prompt has no observation input.**
- **No step limit, and no revert to defaults** when LLM2 goes silent.
- **The mock reads its values from its params file,** not from the live controller.
- **Orin latency is unmeasured.**

### For the lead
- **Rebuild on the car:** f1tenth_messages, llm (new entry point), mpc_controller, f1tenth_control, f1tenth_navigation, f1tenth_params, f1tenth_logger, f1tenth_bringup.
- **Possible one-line merge conflicts with stream A:**
  - `MPC_corr.py` imports: `ParameterDescriptor` between numpy and rclpy, and `MpcParamUpdate` after the `f1tenth_messages.msg` tuple.
  - The ends of `mpc_corr.launch.py`'s parameter dict and return list.
  - The top of `_FORWARDED` in `navigation.launch.py`.
  - The logger topic list after `/mpc/solver_status`.
  - The `stack_params.yaml` LLM2 block after `llama_server_cwd`.
  - `mpc_controller/package.xml` after the `f1tenth_description` dependency.
  - `f1tenth_messages/CMakeLists.txt`, if another stream also adds a message.
- **Scratch:** everything is in `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_C/`:
  - `stage4/run5/` and `stage4/run5_analysis.txt`: the 5-minute run's raw data and analysis.
  - `fix/`: the review-fix runner, the mutation script and the full-suite output.