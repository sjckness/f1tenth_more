# two-llm-arch — C1 recon (2026-10-21, before code)

Base: `night-start/campaign-final` (6002cb0). Line numbers are for that tree
(MPC_corr.py and the MPC part of stack_params.yaml are unchanged by commit (1)).

## 1. How llama-server is launched today (LLM1)

| What | Where | Fact |
|---|---|---|
| Server launch | `llm/launch/llm.launch.py` | `ExecuteProcess([llama_server_path, -m <model_path>, --port <port>, *extra_args], cwd=llama_server_cwd)`, gated by `start_server` (default true). Launches **no node**. |
| Model registry | `llm/config/models.yaml` | One entry: `qwen25_3b_instruct` → `~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf`, **port 8083**, `extra_args: [-ngl, 999]`. |
| Interrogation registry | `llm/config/interrogations.yaml` | One entry: `planner` → `llm_planner_node`, `default_model: qwen25_3b_instruct`. `executable`/`params_file` are not read by llm.launch.py any more (documentation only). |
| Binary / cwd | `stack_params.yaml` `llama_server_path` / `llama_server_cwd` | `/scratch/fabiocar/llama.cpp/build/bin/llama-server`, cwd `/scratch/fabiocar/llama.cpp` (Orin: llama.cpp b9148, 42532afff, CUDA, built from source). |
| `enable_llm` in stack_bringup | — | **Removed.** stack_bringup.launch.py does not reach llm.launch.py at all (its docstring and section 7 say so). |
| Who starts LLM1 | `f1tenth_bringup/config/components.yaml` `intelligence` → `llm.launch.py` | Started by component_supervisor_node, gated by its `enable_intelligence` parameter (stack_params default **true**, i.e. LLM1's server auto-starts with every supervisor bringup). |

**Consumers and ports**

| Consumer | Role | Server | Model |
|---|---|---|---|
| `llm_planner_node` (CLI, `ros2 run llm llm_planner_node "<command>"`) | **LLM1** = mission translator: NL → intent JSON → `plan_translate` → f1tenth_behavior mission → `/mission/load_mission` + `start_mission`. Raw `/completion`, temperature 0, no grammar. | `http://127.0.0.1:8083/completion` (hardcoded `LLAMA_URL`, llm_planner_node.py:165; `llm_url` param) | qwen2.5-3b-instruct q5_k_m, 4 slots, n_ctx 32768 |
| `llm_mpc_tuner_node` | Old manual terminal MPC tuner | **Removed** (git history). Its model `v2_MPCLLM_structpruned_10-q5_k_m.gguf` was on port **8082**, still on the Orin's disk, unregistered. | — |

"PACEd-NAV" does not appear anywhere in the repo; LLM1 = `llm_planner_node`.

**Port finding that changes the plan.** The plan assumes LLM1 on 8082 and LLM2
on 8083. In the code LLM1 is on **8083** (models.yaml, LLAMA_URL, docs). 8082 was
the removed tuner's port. With `enable_intelligence` defaulting true, a second
server on 8083 could never bind on the car. → LLM2 defaults to **8084**
(`llm2_port`), LLM1 untouched. On linus tonight, stream C's own servers use 8083
(LLM2) and 8084 (LLM1 stand-in), per the lead's allocation — passed explicitly.

## 2. Models

| Machine | `~/projects/llm/models` |
|---|---|
| Orin | `decisionLLM_1-5B_structpruned_10-q5_k_m.gguf` (296632320 B), `qwen2.5-3b-instruct-q5_k_m.gguf` (2438740384 B), `v2_decisionLLM_structpruned_20-q5_k_m.gguf`, `v2_MPCLLM_structpruned_10-q5_k_m.gguf` |
| linus | nothing before tonight (no llama-server either) |

Qwen2.5-3B-Instruct Q5_K_M: **already on the Orin and already in models.yaml**
(LLM1 uses it). Downloaded to linus from `Qwen/Qwen2.5-3B-Instruct-GGUF` @
7dabda4d13d5 (official repo):

| Copy | Size (bytes) | sha256 |
|---|---|---|
| Hugging Face LFS oid | 2438740384 | 2c63dde5f2c9ab1fd64d47dee2d34dade6ba9ff62442d1d20b5342310c982081 |
| linus `~/projects/llm/models/` | 2438740384 | 2c63dde5f2c9ab1fd64d47dee2d34dade6ba9ff62442d1d20b5342310c982081 |
| Orin `~/projects/llm/models/` | 2438740384 | 2c63dde5f2c9ab1fd64d47dee2d34dade6ba9ff62442d1d20b5342310c982081 |

All three identical. LLM2 gets its own models.yaml entry
`qwen2.5-3b-instruct-q5km` pointing at the same file, with `-ngl 999 -np 1 -c
4096` (one request at a time, short prompt).

llama.cpp on linus: official prebuilt **b9148 Vulkan x64** (same build number
and commit as the Orin), `~/projects/llm/llama-b9148/` (tarball sha256 checked
against the GitHub release digest). No CUDA toolkit on linus and no Linux CUDA
prebuilt upstream; Vulkan sees the RTX 5050 (Blackwell) through driver 580 —
GPU, not CPU fallback.

## 3. MPC_corr cost / reference parameters

Control loop: `self.ts = 0.1` s (10 Hz timer, MPC_corr.py:2194), `self.N = 20`
(MPC_corr.py:850) → stage factor `STAGE_WEIGHT_REF_HORIZON / N = 7/20 = 0.35`.

| Name (ROS param) | Weights key | Declared? | Read | Scaled by scale_stage_weights? | Launch-path value (stack_params.yaml) | Effective |
|---|---|---|---|---|---|---|
| `mpc_w_term` | w_term | yes, `_w()` MPC_corr.py:1680-1694; launch arg in mpc_corr.launch.py | **once at startup** into `self.weights` | no (terminal) | 9.0 (l.2160) | 9.0 |
| `mpc_w_psi` | w_psi | yes | startup | no (terminal) | 4.5 (l.2163) | 4.5 |
| `mpc_w_psi_stage` | w_psi_stage | yes | startup | yes | 4.7736 (l.2166) | 1.6708 |
| `mpc_w_corr` | w_corr | yes | startup | yes; **overridden** on arc/pose_arc corridors by `_weights_for` (MPC_corr.py:6862-6868, from `mpc_arc_corr_sigma_m`) | 3.5714 (l.2169) | 1.25 |
| `mpc_w_line` | w_line | yes | startup | yes; **always overridden** where it applies (arc), inert elsewhere | 1.1662 (l.2172) | 0.408 |
| `mpc_w_v` | w_v | yes | startup | yes | 4.2857 (l.2175) | 1.50 |
| `mpc_w_obs` | w_obs | yes | startup | yes | 8.0 (l.2179) | 2.8 |
| `mpc_w_du_delta` | w_du_delta | yes | startup | yes | **8.33** (l.2202) | 2.9155 |
| `mpc_w_delta0` | w_delta0 | yes | startup | yes | 0.0 (l.2205) | 0 |
| `mpc_w_u_a` | w_u_a | yes | startup | yes | 0.0 (l.2208) | 0 |
| `mpc_w_du_a` | w_du_a | yes | startup | yes | 0.0 (l.2211) | 0 |
| `vdes` | (solve arg `vdes`) | **NO** — `self.vdes = 0.5` literal (MPC_corr.py:1705), baseline `self.vdes_default` | per tick, but **overwritten per move**: turn/drive goals set it from the move's `speed` (MPC_corr.py:3021, 3170), `_clear_drive_state` restores 0.5 (3054); per tick it is further capped by the stop ramp / object ramp / unknown-speed cap into a local (5301-5326) | n/a | none (not a param) | 0.5 bootstrap |
| `mpc_rear.w_*` | rear solver | yes (lazily, MPC_corr.py:4153/4182) | when the reverse solver is built | yes | copies of the forward set (l.1611-1641) | — |

Notes:
- **`mpc_w_du_delta` is 8.33 on the launch path, not 8.5714.** The yaml comment
  and MPC_corr's table (MPC_corr.py:1663) say 8.5714 (effective 3.0); the value
  changed in 68792db. Known failure #13 in output/mpc_known_failures.md. The mock
  must use **8.33** so the round trip changes nothing.
- All `mpc_w_*` are read **once** (`self.weights`, MPC_corr.py:1683). There is
  **no `add_on_set_parameters_callback`** in MPC_corr.py: `ros2 param set
  /mpc_corr mpc_w_v 5.0` today changes the parameter but **not** the controller.
- `self.weights` is passed **every solve** (`weights=self._weights_for(corridor)`,
  MPC_corr.py:5485), and `solve_mpc_step` re-scales on every call
  (mpc_solver.py:432). No cached or compiled copy of the weights exists, so a
  new `self.weights` dict takes full effect at the next solve.

## 4. What can change safely at runtime, and where

- **Executor**: `rclpy.spin(node)` (MPC_corr.py:7416), no callback groups, no
  threads → a subscription callback can never run in the middle of
  `control_loop`. A lock around the pending update is cheap and keeps it correct
  if a MultiThreadedExecutor is ever introduced.
- **Tick boundary**: the top of `control_loop` (MPC_corr.py:4901), before
  `_update_active_odom()`. Applying there means every use of the weights inside
  one tick — `_weights_for` → `_mpc_solve`, the CFG/model logs — sees one set.
  The solve itself is at MPC_corr.py:5479.
- **Safe to change at runtime** (pure cost shape, the hard constraints still
  bound the result): `mpc_w_term`, `mpc_w_psi`, `mpc_w_psi_stage`, `mpc_w_corr`,
  `mpc_w_v`, `mpc_w_du_delta`.
- **Not recommended for the default whitelist**:
  - `mpc_w_obs` — obstacle soft-deflection, a safety term.
  - `mpc_w_line` — overridden by the arc sigmas wherever it acts; a write does
    nothing.
  - `mpc_w_delta0`, `mpc_w_u_a`, `mpc_w_du_a` — deliberately 0 (w_delta0 would
    fight the ~2° steering bias).
  - `vdes` — not a declared param; per-move state the mission sets. Applying
    vdes=0.5 during a move commanded at 0.4 would **change the speed**, so a
    mock at "current values" is not a no-op unless vdes is left out or mapped
    to `vdes_default` only (the baseline used when a move gives no speed),
    which would need a new declared parameter. Recommendation: leave vdes out
    of the default whitelist and mock set; document it as the next step.
- **Never tunable** (hard limits / safety geometry): `car_radius` (871),
  `avoidance_margin` (883; `obstacle_safety_margin_m`), `delta_min`/`delta_max`
  (1513-1517), `dDeltaMin`/`dDeltaMax` (literals 1549-1550), `use_hard_boundary_constraints`
  (920), `boundary_hard` (942), `boundary_slack_weight`/`boundary_slack_max`
  (943-954), `boundary_source` (965), `corr_half_width_start_m` /
  `corr_width_scale_with_length` (1086-1099), `max_forward_speed_mps` /
  `max_reverse_speed` / `allow_reverse` (2032-2042), every `mpc_rear.*`.

Suggested whitelist ranges (declared, unscaled units; ~0.25×–2.5× of today,
the w_du_delta upper bound at its rho/sigma² derivation):

| name | today | min | max |
|---|---|---|---|
| mpc_w_term | 9.0 | 2.0 | 20.0 |
| mpc_w_psi | 4.5 | 1.0 | 10.0 |
| mpc_w_psi_stage | 4.7736 | 1.0 | 12.0 |
| mpc_w_corr | 3.5714 | 0.5 | 9.0 |
| mpc_w_v | 4.2857 | 1.0 | 10.0 |
| mpc_w_du_delta | 8.33 | 2.0 | 23.81 |

## 5. Blockers for real tuning later (for the README)

1. Weights are read once at startup; nothing re-reads them on a param change
   (LLM2's apply path has to write `self.weights` itself).
2. Two of the six cost knobs act through `scale_stage_weights` (×0.35 at N=20):
   an LLM reasoning in "effective" numbers is off by 2.86× on stage terms.
3. On arc corridors `w_corr`/`w_line` come from `mpc_arc_*_sigma_m`, not from the
   weights dict: LLM2's w_corr does nothing on object approaches.
4. `vdes` is per-move mission state, not a parameter.
5. No lateral-offset reference: the corridor centreline is the only lateral
   target; there is no parameter for "keep x m left of centre".
6. First request after server start: ~25 s on linus/Vulkan (pipeline build,
   prompt 133 tokens in 25.1 s); afterwards ~0.9 s. The node's timeout and its
   first tick have to allow for that.

## 6. Memory / GPU with both servers (C2), measured on linus 2026-10-21 23:46

Setup: llama.cpp b9148 Vulkan, RTX 5050 8151 MiB with 3557 MiB already used by
the desktop and other processes. LLM2 = exact models.yaml flags (`-ngl 999 -np 1
-c 4096`) on 8083. Two 2.3 GB models do not fit in the ~3.5 GB free, so the
**LLM1 stand-in** (same .gguf, LLM1's default slots/ctx: 4 slots, n_ctx 32768) ran
on **CPU only** (`--device none -ngl 0`) on 8084. Request = one LLM2 echo of the
six-weight JSON, `/completion`, temperature 0, `json_schema`, `cache_prompt`.

| State | GPU mem.used (total) | LLM2 server VRAM (nvidia-smi) | LLM2 RSS | LLM1 stand-in RSS |
|---|---|---|---|---|
| baseline | 3557 MiB | — | — | — |
| LLM2 alone, idle | 6185 MiB (+2628) | 2578 MiB | 632 MB (488 file, 144 anon) | — |
| LLM2 alone, during request | 6157–6194 MiB, util ≤ 91 % | — | — | — |
| LLM2 + LLM1 stand-in, idle | 6177 MiB | 2587 MiB | 560 MB | 3644 MB (2379 file, 1264 anon) |
| both, during one LLM2 request | 6177 MiB, util 90 % | 2587 MiB | 562 MB | 3644 MB |

Buffers from the server logs (same on any backend up to the compute buffer):

| | weights | KV cache | compute |
|---|---|---|---|
| LLM2 (`-np 1 -c 4096`) | 2116 MiB GPU + 204 MiB CPU-mapped | 144 MiB | 305 MiB + 16 MiB host |
| LLM1 (4 slots, n_ctx 32768) | 2320 MiB | 1152 MiB | 305 MiB |

Estimate for the Orin (unified memory, both on GPU): LLM1 ≈ 3.8 GB, LLM2 ≈ 2.8 GB,
together ≈ 6.6 GB of 62.8 GB (54.6 GB available at 23:36 with nothing running).
LLM2 with LLM1's default flags would cost ≈ 1 GB more (KV 1152 vs 144 MiB).

Latency (wall, LLM2, linus): first request after load **26.4 s** (prompt 133
tokens in 25.1 s: Vulkan pipeline build on first use; 82 tokens in 1.25 s);
then **0.90 s** alone and **0.98 s** with the stand-in loaded (whole prompt
cached, 1 prompt token; 82 tokens in 0.86–0.92 s, ~90 tok/s). echo_match true
on all three.

**Tomorrow on the Orin** (tegrastats; nothing was started there tonight):

```bash
# 1) LLM1 running as usual (supervisor bringup, enable_intelligence default true).
tegrastats --interval 500 --logfile ~/llm2_tegrastats_$(date +%Y%m%d_%H%M%S).log &
free -m
# 2) idle 30 s, then start LLM2 (server + node) and let it run ~60 s at 0.5 Hz:
ros2 launch llm llm2.launch.py            # or stack_bringup ... enable_llm2:=true
# 3) one request by hand while it runs (prompt-cached after the first):
curl -s http://127.0.0.1:8084/completion -d '{"prompt":"Return {\"a\":1} exactly.","n_predict":16,"temperature":0}' | head -c 300; echo
free -m
tegrastats --stop
# RAM used / GR3D_FREQ columns of the log: before vs after the LLM2 start.
```
