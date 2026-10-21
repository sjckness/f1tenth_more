# Two-LLM architecture: LLM2, the MPC-parameter tuner (mock)

Branch `two-llm-arch`, 2026-10-21/22. Built on `campaign-final` (6002cb0). The
recon behind every choice here, with line references, is in
[`recon.md`](recon.md).

LLM1 translates a mission (`llm_planner_node`, unchanged). LLM2 tunes the MPC
controller. **For now LLM2 is a mock**: every 2 s it sends its own llama-server
a fixed JSON of MPC weights and asks for it back verbatim. The node checks the
reply and publishes it. `mpc_corr`, if allowed to, applies it at the start of
its next control tick. The fixed set is today's launch-path values, so the
round trip changes nothing. It exists to prove the pipe end to end (server,
latency, validation, application, logging) before a real tuner goes in.

**Everything is OFF by default.** With the defaults, `stack_bringup` launches
the same 33 processes and nodes as `campaign-final` (checked with a dry walk of
the launch tree; the only difference is one extra parameter,
`llm2_params_enabled: false`, on mpc_corr), and `mpc_corr` does not even
subscribe. The gate is pinned by
`f1tenth_bringup/test/test_stack_bringup_llm2_gate.py`.

```
                 LLM1 (unchanged)                      LLM2 (new, off by default)
  llama-server :8083  <-  llm_planner_node    llama-server :8084  <-  llm2_mpc_param_node
  (intelligence component,  (CLI, by hand)     (llm2.launch.py)        0.5 Hz, mock echo
   enable_intelligence)                                                    |
                                                    /llm2/mpc_params  <----+   /llm2/status
                                                           |
                                         mpc_corr (llm2_params_enabled)
                                           whitelist + range check in the callback,
                                           applied at the top of control_loop
                                                           |
                                                  /mpc/applied_params
```

## 1. How to enable it

There are **two independent switches**. One starts the producer, the other
lets the consumer act on it.

| Switch | Where | Default | What it does |
|---|---|---|---|
| `enable_llm2` | `stack_bringup.launch.py` arg (stack_params.yaml) | `false` | Starts the second llama-server (`llm2_model` on `llm2_port`) and `llm2_mpc_param_node`. Nothing else. |
| `llm2_params_enabled` | `mpc_corr` parameter; launch arg of `mpc_corr.launch.py`, forwarded by `navigation.launch.py` and therefore by `stack_bringup` | `false` | `mpc_corr` subscribes to `/llm2/mpc_params` and applies valid updates. `false`: no subscription, no publisher, no whitelist parameters. **Read-only at run time**, like the whitelist: `ros2 param set /mpc_corr llm2_params_enabled false` is refused ("read-only parameter"); to switch it, restart mpc_corr (the `navigation` component). |

Either can run without the other: LLM2 alone publishes and logs but changes
nothing; `mpc_corr` alone waits for updates that never come. **To stop LLM2
from changing the controller at run time**, stop the producer (the
`llm2.launch.py` process): the values last applied stay in force until
mpc_corr restarts (section 5, item 8).

**stack_bringup path:**

```bash
ros2 launch f1tenth_bringup stack_bringup.launch.py enable_llm2:=true llm2_params_enabled:=true
# optional: llm2_port:=8084 llm2_model:=qwen2.5-3b-instruct-q5km
```

**supervisor path** (`supervisor_bringup.launch.py` has no `enable_llm2`):

```bash
# producer, next to the running stack:
ros2 launch llm llm2.launch.py              # [llm2_port:=8084] [llm2_model:=...]
# consumer: add to f1tenth_bringup/config/components.yaml (machine-local edit)
#   navigation:
#     - package: f1tenth_navigation
#       launch_file: navigation.launch.py
#       args:
#         llm2_params_enabled: 'true'
# then restart the navigation component.
```

Prerequisites on the car: the llama-server binary at `llama_server_path`
(stack_params.yaml, `/scratch/fabiocar/llama.cpp/build/bin/llama-server`) and
the model `~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf`. That is the
same file LLM1 already uses (2438740384 B, sha256 `2c63dde5…982081`, identical
on the Orin, on linus and on `Qwen/Qwen2.5-3B-Instruct-GGUF`), so nothing new
has to be downloaded.

**Ports.** LLM1's server is on **8083** (models.yaml `qwen25_3b_instruct`,
`llm_planner_node`'s `LLAMA_URL`) and starts by default with the
`intelligence` component. So LLM2 defaults to **8084**, not 8083 as the plan
said; 8082 was the removed `llm_mpc_tuner`'s port. `llm2.launch.py` passes the
same port to the node as `server_url`, and warns (does not refuse) if
`llm2_port` equals LLM1's.

**Check it is working:**

```bash
ros2 topic echo /llm2/status            # one JSON per tick: outcome ok, echo_match true
ros2 topic echo /mpc/applied_params     # same seq as /llm2/mpc_params, one per valid update
ros2 param get /mpc_corr mpc_w_v        # the live value (4.2857 with the mock)
```

`mpc_corr` logs `LLM2 | parameter updates ON ... Whitelist ...` at start, then
`LLM2 | applied seq=N (llm2_mock): no value changed (6 parameters)` at most
once every 30 s while the mock echoes. A changed value is logged
`name old -> new` every time.

## 2. Topics

All three are recorded by `mission_logger` (`_DEFAULT_TOPICS`).

| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| `/llm2/mpc_params` | `f1tenth_messages/MpcParamUpdate` | `llm2_mpc_param_node` | A **valid** LLM2 reply: one JSON object with exactly the requested keys, all finite numbers. `seq` = the node's tick number, `source` = `llm2_mock`, `names[i]`/`values[i]` = `mpc_w_*` parameter names and values in declared (unscaled) units. Invalid replies publish nothing. A valid reply that is not an exact echo **is** published (with a warning); mpc_corr's whitelist is the gate. |
| `/llm2/status` | `std_msgs/String` (JSON) | `llm2_mpc_param_node` | One message per tick, sent when that tick's outcome is known. Always the same keys: `seq`, `outcome` (`ok` / `invalid` / `timeout` / `error` / `skipped`), `reason`, `echo_match` (reply == sent), `latency_ms` (wall, request to reply), `timeout_s` (the timeout used), `in_flight_seq` (for a skipped tick), `tokens_cached`, `prompt_n`/`prompt_ms` (prefill), `predicted_n`/`predicted_ms` (generation), `source`, `counters` (cumulative ok / invalid / timeout / error / skipped / echo_match). `error` = no usable reply from the server (connection refused, HTTP 503 "Loading model", ...), as opposed to `invalid` = the model answered something unusable. |
| `/mpc/applied_params` | `f1tenth_messages/MpcParamUpdate` | `mpc_corr` | Published after every application, with the update's `seq` and `source`, the names and values applied, and the stamp of the tick that applied them. An exact echo is applied (and echoed) too, so a bag shows request → application one to one, with one exception: two valid updates that arrive before one control tick are merged (newer value per name) and applied under the **newer** seq, so the older seq never appears here although its other values were applied; mpc_corr logs `seq=N merged over seq=M`. At 10 Hz control against 0.5 Hz LLM2 that needs a stalled control loop; it did not happen in the 5-minute run. Rejected updates produce nothing here (only a warning in mpc_corr's log). |

`MpcParamUpdate.msg`: `std_msgs/Header header`, `uint32 seq`, `string source`,
`string[] names`, `float64[] values`.

## 3. What each side does

**`llm2_mpc_param_node`** (`llm/llm/llm2_mpc_param_node.py`, logic in
`llm/llm/llm2_protocol.py`, parameters in `llm/config/llm2_mpc_param_params.yaml`):

- `rate_hz` 0.5; `server_url` `http://127.0.0.1:8084` (the launch fills it in
  from `llm2_port`); `request_timeout_s` 10; `warmup_timeout_s` 90 for the
  first request and the one after any timeout or error (a fresh server builds
  its GPU pipelines on first use); `n_predict` 256.
- `mock_param_names` / `mock_param_values`: two parallel lists (a ROS parameter
  cannot be a map). Default = today's launch-path values: `mpc_w_term` 9.0,
  `mpc_w_psi` 4.5, `mpc_w_psi_stage` 4.7736, `mpc_w_corr` 3.5714, `mpc_w_v`
  4.2857, `mpc_w_du_delta` **8.33** (the launch-path value; the comments around
  it still quote 8.5714, see `output/mpc_known_failures.md` #13). A test holds
  the params file, `MOCK_PARAMS` and stack_params.yaml equal.
- Each tick: if a request is still in flight, the tick is **skipped and
  counted**, never queued. Otherwise one `POST /completion` on a worker thread
  (the executor never blocks on HTTP): temperature 0, a `json_schema` with
  exactly these keys and `additionalProperties: false` (llama-server turns it
  into a grammar), `cache_prompt` with a fixed ChatML prefix ahead of the JSON.
- Validation: one JSON object and nothing around it, no repeated keys, exactly
  the key set sent, int/float values that are finite (no bool, string, null,
  NaN, Infinity or overflow).

**`mpc_corr`** (`mpc_controller/llm2_params.py` holds the rules; the
`MPC_corr.py` block is `_llm2_setup` / `_llm2_params_callback` /
`_llm2_apply_pending`):

- Whitelist = three parallel list parameters with stack_params.yaml defaults,
  bounds in declared (unscaled) units, inclusive:

  | name | today | min | max |
  |---|---|---|---|
  | `mpc_w_term` | 9.0 | 2.0 | 20.0 |
  | `mpc_w_psi` | 4.5 | 1.0 | 10.0 |
  | `mpc_w_psi_stage` | 4.7736 | 1.0 | 12.0 |
  | `mpc_w_corr` | 3.5714 | 0.5 | 9.0 |
  | `mpc_w_v` | 4.2857 | 1.0 | 10.0 |
  | `mpc_w_du_delta` | 8.33 | 2.0 | 23.81 |

- **Only `mpc_w_*` cost weights can ever be whitelisted, and never
  `mpc_w_obs`.** Any other name (car_radius, avoidance_margin, steering
  limits, dDelta, boundary flags, corridor geometry, vdes, max speeds,
  allow_reverse, `mpc_rear.*`) refuses mpc_corr's start with a FATAL log, as
  does a malformed whitelist (unpaired lists, duplicates, non-finite bounds,
  negative minimum, min > max). `mpc_w_obs` is refused the same way: the
  obstacle cost has no hard obstacle row behind it in the RTI solve (the
  costmap boundary rows are soft by default), so a low `w_obs` would switch
  obstacle avoidance off. The other weights only reshape the objective; the
  input and speed boxes and the corridor's lateral bound stay hard rows.
- The four LLM2 parameters (`llm2_params_enabled`, `llm2_param_whitelist`,
  `_min`, `_max`) are declared **read-only**: they are read once at start,
  so a runtime `ros2 param set` is refused with a reason instead of being
  accepted, shown by `ros2 param get`, and ignored.
- An update is **all or nothing**: one non-whitelisted name, NaN/inf, or
  out-of-range value rejects the whole message, with a warning saying why.
  Nothing is clamped. With `allow_reverse` on, so is an update raising
  `mpc_w_v` above `w_rev / (2·vdes_default)` (4.3 with the defaults),
  whatever `llm2_param_max` says (section 5, item 6).
- The subscription callback only validates and parks the update (locked,
  one slot; a second valid update before the next tick is merged over the
  first, newer value wins, with a warning). `control_loop` applies it **first
  thing in the next tick**, never mid-solve:
  `set_parameters_atomically` on the declared `mpc_w_*` (so `ros2 param get`
  shows the live value) **and** a new `self.weights` dict (what every solve
  reads). If the parameter write is refused, nothing is applied.
- Startup warnings (start continues): a current value outside its own range
  (every echo would be rejected); `allow_reverse` on with the `mpc_w_v`
  maximum above `w_rev / (2·vdes)` = 4.3 (updates past 4.3 are then
  rejected, see section 5), or the current `mpc_w_v` already above it (every
  update carrying `mpc_w_v` is then rejected).

`vdes` is **not** in the whitelist and cannot be: it is not a declared
parameter but per-move mission state (each move sets it from its own speed),
so an "unchanged" 0.5 would change the speed of a move commanded at 0.4.

## 4. Measured

All on **linus**: RTX 5050 8 GB (3.6 GB already used by the desktop and other
processes), llama.cpp **b9148 Vulkan** prebuilt (same build number and commit
as the Orin's CUDA build), the same .gguf as the car. ROS_DOMAIN_ID 73,
localhost only. **Nothing was measured on the Orin**; see the end of this
section for tomorrow's commands.

### Latency: 5 minutes at 0.5 Hz against a real server (2026-10-22 00:41-00:47)

`ros2 launch llm llm2.launch.py llm2_port:=8083` (real llama-server with the
models.yaml flags `-ngl 999 -np 1 -c 4096`, plus the node with its shipped
parameters), a recorder of the three topics, and, as the consumer, the real
`MPCController` with `llm2_params_enabled:=true` spun on its own in one
process. That is one node with no odometry and no mission, not the stack;
its 10 Hz `control_loop` runs and returns at "no odom" every tick, after the
LLM2 tick boundary.

Start-up: the server took ~20 s to load the model. The first 9 ticks got
HTTP 503 "Loading model" (`error`, logged once thanks to the throttle). The
first valid reply was tick 10, 1062 ms, using the 90 s warm-up timeout.

Then, over the **300 s from the first valid reply**:

| | value |
|---|---|
| ticks | 151 (150 expected at 0.5 Hz); status spacing median 2.0 s, range 1.8-2.2 s |
| outcomes | **151 ok**, 0 invalid, 0 timeout, 0 error, 0 skipped |
| **echo_match** | **151 / 151 = 100 %** (all 161 ok replies of the whole run) |
| latency (wall), ms | **min 854, median 862, p90 870, p99 976, max 1062** (whole run after warm-up: max 1187) |
| vs the 2 s period | 0 replies over 2 s, 2 over 1 s; the slowest used 53 % of the period, so the in-flight skip never triggered (it is covered by the unit tests) |
| prefill (`prompt_ms`) | median 10.5 ms: 1 new prompt token, 214 cached. The first request: 133 tokens in 145 ms |
| generation (`predicted_ms`) | median 850 ms for 82 tokens (96.5 tok/s) |
| HTTP + JSON overhead | median 2.1 ms |

So the latency is almost all generation of the 82-token echo. A real tuner
that returns fewer tokens is faster; one with a changing state prompt adds
prefill.

**End to end, LLM2 → mpc_corr:** 161 `/llm2/mpc_params` → 161
`/mpc/applied_params`, with the same seqs in the same order and the same
names and values. From publish to application (header stamps): median 18.6
ms, p90 23.9, max 122.5 ms, i.e. within one 100 ms control tick plus DDS.
mpc_corr's `self.weights` and its declared `mpc_w_*` parameters were equal at
start and at end (the mock round trip changes nothing). The "no value
changed" log line appeared 11 times in 5.5 min (30 s throttle).

Earlier runs agree. Stage 1, 3 single requests: 0.90 s alone, 0.98 s with an
LLM1 stand-in loaded. Stage 2, a 90 s smoke run: 39/39 ok and echo, median
866 ms.

**Cold first request.** In stage 1 the first request after a server start
took **26.4 s** (133-token prompt in 25.1 s: Vulkan builds its pipelines on
first use). In the later runs the first prompt took 79-145 ms, presumably
because Vulkan's pipeline cache persisted on disk. The node's 90 s
`warmup_timeout_s` covers both. The Orin's CUDA cold start is unmeasured;
`docs/llm.md` reports ~58 s for LLM1's first request on a dev machine.

### Memory / GPU

LLM2 server, models.yaml flags, idle and during requests:

| State | GPU mem used (total) | LLM2 VRAM | LLM2 RSS |
|---|---|---|---|
| before | 3557-3655 MiB | — | — |
| LLM2 loaded, idle | 6185 MiB (+2628) | 2578 MiB | 632 MB |
| during the 5-minute run | max 6294 MiB (+2639); utilisation median 6 %, max 92 % during a request | — | 406-1082 MB (5 s samples; includes mapped model pages) |

`llm2_mpc_param_node` RSS: 62 MB.

**Both servers loaded** (stage 1). Two 2.3 GB models do not fit in the ~3.5
GB free on linus. So LLM2 ran fully on the GPU, and an LLM1 stand-in (same
.gguf, LLM1's default 4 slots × n_ctx 32768) ran CPU-only on 8084:

| State | GPU mem used | LLM2 VRAM | LLM2 RSS | LLM1 stand-in RSS |
|---|---|---|---|---|
| both idle | 6177 MiB | 2587 MiB | 560 MB | 3644 MB |
| both, one LLM2 request | 6177 MiB, util 90 % | 2587 MiB | 562 MB | 3644 MB |

Buffers from the server logs (backend-independent up to the compute buffer):
LLM2 weights 2116 MiB GPU + 204 MiB CPU-mapped, KV 144 MiB, compute 305 MiB.
LLM1 weights 2320 MiB, KV 1152 MiB, compute 305 MiB. LLM2's own models.yaml
entry (`-np 1 -c 4096`) saves ~1 GB of KV against LLM1's flags.

**Orin estimate** (unified memory, both on the GPU): LLM1 ≈ 3.8 GB + LLM2 ≈
2.8 GB ≈ 6.6 GB of 62.8 GB. Not measured.

### Tomorrow on the Orin

```bash
# 1) The stack as usual (LLM1's server comes with the intelligence component).
tegrastats --interval 500 --logfile ~/llm2_tegrastats_$(date +%Y%m%d_%H%M%S).log &
free -m
# 2) idle 30 s, then LLM2 (server + node) for ~60 s at 0.5 Hz:
ros2 launch llm llm2.launch.py            # or stack_bringup ... enable_llm2:=true
ros2 topic echo /llm2/status --once       # latency_ms, prompt_ms, predicted_ms
# 3) one request by hand while it runs:
curl -s http://127.0.0.1:8084/completion -d '{"prompt":"Return {\"a\":1} exactly.","n_predict":16,"temperature":0}' | head -c 300; echo
free -m
tegrastats --stop
# RAM used / GR3D_FREQ in the log: before vs after the LLM2 start. From
# /llm2/status: the first ok's latency (CUDA cold start) and the steady median.
```

## 5. What blocks real tuning later

1. **Weights are read once at startup.** `MPC_corr` reads every `mpc_w_*` into
   `self.weights` in `__init__` and has no `add_on_set_parameters_callback`.
   LLM2's apply path writes `self.weights` itself, but a hand `ros2 param set
   /mpc_corr mpc_w_v 5.0` still changes the parameter and **not** the
   controller. With LLM2 on, the two then disagree until the next LLM2
   application overwrites both. A real tuner (or a human) needs a parameter
   callback that goes through the same whitelist and tick boundary.
2. **Three different numbers for one name.** The declared value is what LLM2
   sends. `scale_stage_weights` multiplies the stage terms (`w_psi_stage`,
   `w_corr`, `w_v`, `w_du_delta`, and `w_line`/... if ever listed) by
   7/N = 0.35 at N = 20 on every solve; `w_term`/`w_psi` are terminal and not
   scaled. `w_du_delta` is further multiplied by `rate_weight_factor`
   ((control period / `mpc_prediction_dt_s`)², 1.0 at today's 0.1 s, 0.25 at
   0.2 s) on stages 1..N-1. A model reasoning about "effective" weights, or
   about values copied from the solver's debug log, is off by 2.86× (or 11.4×).
   The prompt of a real tuner must state which units it works in; the
   whitelist ranges are in declared units.
3. **`w_corr` does nothing on object approaches.** On `arc` / `pose_arc`
   corridors `_weights_for` replaces `w_corr` and `w_line` with values derived
   from `mpc_arc_corr_sigma_m` / `mpc_arc_line_sigma_m`. LLM2's `w_corr` only
   acts on plain corridors.
4. **No lateral-offset reference.** The corridor centreline is the only
   lateral target; there is no parameter for "keep x m left of centre", so
   "pass closer to the right wall" cannot be expressed as a parameter today.
   It would need a new reference input in the corridor / cost, not a weight.
5. **No speed knob.** `vdes` is per-move state (section 3). Tuning speed would
   need a new declared parameter (e.g. a `vdes_default` or a speed scale)
   re-read at the tick boundary, which changes controller behaviour and was
   out of scope for a mock that must change nothing.
6. **`w_v` and reverse.** `reverse_mpc` relies on `w_rev >= 2·w_v·vdes`
   ("reversing is never cheaper than standing"). With `allow_reverse` on,
   mpc_corr rejects (whole message) any update with `mpc_w_v` above
   `w_rev / (2·vdes_default)` = 4.3, so LLM2 can lower `w_v` but raise it by
   at most 0.3%. The check uses `vdes_default` (0.5), the speed the shipped
   `w_rev` was sized for: a move commanded faster than 0.5 m/s already breaks
   the invariant with the shipped weights (pre-existing, not LLM2's). A real
   tuner that needs a higher `w_v` with reverse on must move `w_rev` with it,
   which is a reverse-planner parameter and not tunable. LLM2 updates never
   touch the reverse manoeuvre's own `mpc_rear.*` weights.
7. **No observation input.** The mock prompt contains only the parameter
   JSON. A real tuner needs a compact state summary (speed error, cross-track
   error, solver status, corridor type, recent events) as a topic and in the
   prompt. That prompt changes every tick, so its suffix is not cached:
   expect prefill cost on top of the ~0.85 s generation measured here.
8. **No step limit, no fallback.** Each value only has to be inside its
   range: `mpc_w_v` may jump 1 → 10 between two ticks. And the last applied
   set stays in force if LLM2 stops publishing (no timeout back to the
   stack_params defaults) until mpc_corr restarts. A real tuner needs a max
   change per update and a "revert to defaults after X s of silence" rule.
9. **The mock's values come from its params file, not the live controller.**
   If mpc_corr was launched with a non-default `mpc_w_*` (launch override),
   an applied echo sets it back to the stack_params value. A real tuner
   should read the live values from `/mpc_corr`'s parameter service.
10. **Latency on the Orin is unmeasured.** linus (RTX 5050, Vulkan) answers in
    ~0.9 s; the Orin (CUDA) cold first request and steady latency are for
    tomorrow (section 4).

## 6. Tests

| File | What it pins |
|---|---|
| `llm/test/test_llm2_protocol.py` | JSON validation (exact echo, changed value, 15 invalid replies), refused mock sets, request body (greedy, `cache_prompt`, exact-key schema, stable prefix), skip-when-in-flight on the gate (8 concurrent ticks start exactly 1 request), fake HTTP server: exact JSON with timings, timeout, invalid content, HTTP 503, closed port |
| `llm/test/test_llm2_mpc_param_node.py` | The real node against the fake server, ticks by hand: publishes the echo (source, seq, names, values), publishes a valid non-echo as returned with `echo_match` false, skips while in flight, publishes nothing on an invalid reply, warm-up then short timeout, refuses unpaired lists; config couplings (params file == `MOCK_PARAMS` == stack_params.yaml; the interrogation entry) |
| `llm/test/test_llm2_launch.py` | `enable_llm2` defaults to false in stack_params.yaml; shipped `llm2_port` is not LLM1's; `llm2.launch.py` starts one server with `llm2_model`'s file and flags on `llm2_port` and gives the node the same port |
| `f1tenth_bringup/test/test_stack_bringup_llm2_gate.py` | `stack_bringup` includes `llm2.launch.py` exactly once, and its condition is false with the defaults and with `enable_llm2:=false`, true only with `enable_llm2:=true` |
| `llm/test/conftest.py` | The fake llama-server fixture |
| `mpc_controller/test/test_llm2_params.py` | Shipped whitelist and ranges (and the mock set inside them), non-weight names and `mpc_w_obs` refuse the whitelist, malformed whitelists, all-or-nothing rejection (14 cases), inclusive bounds, the reverse `w_v` limit, the pending slot (take once, merge, 6 threads × 300 puts), startup warnings |
| `mpc_controller/test/test_llm2_mpc_corr_node.py` | The real `MPCController`: off → no subscription/publisher; on → applied at the next tick, not by the callback, even when the update arrives mid-solve; rejected updates change and publish nothing and do not displace a parked one; with `allow_reverse` on, `mpc_w_v` 10 is rejected and 4.0 applied; a runtime `set_parameters` on the switch or the whitelist is refused (on and off); `car_radius` or `mpc_w_obs` in the whitelist refuses the start |
| `f1tenth_control/test/test_mpc_corr_launch_config.py` (2 added) | `llm2_params_enabled` reaches the node as bool False / True |

```bash
# from the package dir, in the workspace env:
python3 -m pytest test/test_llm2_protocol.py test/test_llm2_mpc_param_node.py test/test_llm2_launch.py   # llm
python3 -m pytest test/test_llm2_params.py test/test_llm2_mpc_corr_node.py                              # mpc_controller
```

Not done: the plan's sim comparison (feature on with the mock vs off, same
drive behaviour). Running the stack or the simulator on linus was excluded for
this session. On the car or the sim host: run the same short mission twice,
`enable_llm2:=true llm2_params_enabled:=true` and both false, and compare
`/drive` and `/mpc/status`; with the mock they must be identical up to timing.

## 7. Files

| File | Change |
|---|---|
| `f1tenth_messages/msg/MpcParamUpdate.msg`, `CMakeLists.txt` | New message |
| `llm/config/models.yaml` | Entry `qwen2.5-3b-instruct-q5km` (same .gguf, `-ngl 999 -np 1 -c 4096`) |
| `llm/config/interrogations.yaml` | Entry `llm2_mpc_params` |
| `llm/config/llm2_mpc_param_params.yaml` | Node parameters (mock set) |
| `llm/launch/llm2.launch.py` | Second llama-server + node |
| `llm/llm/llm2_protocol.py`, `llm/llm/llm2_mpc_param_node.py` | The node |
| `llm/setup.py`, `llm/package.xml` | Entry point; `std_msgs` dependency |
| `f1tenth_bringup/launch/stack_bringup.launch.py` | `enable_llm2`, `llm2_model`, `llm2_port` |
| `f1tenth_params/config/stack_params.yaml` | LLM2 block after `llama_server_cwd` (7 keys); no MPC weight touched |
| `mpc_controller/mpc_controller/llm2_params.py` | Whitelist / validation / pending slot |
| `mpc_controller/mpc_controller/MPC_corr.py` | One block of 3 methods after `_weights_for`, one call in `__init__`, one at the top of `control_loop`, imports |
| `mpc_controller/package.xml` | `rcl_interfaces` (the read-only `ParameterDescriptor`) |
| `f1tenth_bringup/config/components.yaml` | Comment only: two interrogations now |
| `f1tenth_control/launch/mpc_corr.launch.py`, `f1tenth_navigation/launch/navigation.launch.py` | `llm2_params_enabled` arg and its forwarding |
| `f1tenth_logger/f1tenth_logger/mission_logger_node.py` | Records the three topics |

Rebuild on the car (Humble): `f1tenth_messages`, `llm`, `mpc_controller`,
`f1tenth_control`, `f1tenth_navigation`, `f1tenth_params`, `f1tenth_logger`,
`f1tenth_bringup`.
