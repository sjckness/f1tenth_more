# planner.md — `llm_analytics`, the headless LLM measurement tool

> Personal working log and plan, kept by hand, grown indefinitely.
> **Exempt by decision (2026-11-12) from the memory-bank's 800-word limit and from the
> `progress.md` pruning rules. Its history is never truncated.** `memory-bank/` is the
> agent's context; this file is the human's intent, evidence and order of work.

**Subject:** `src/f1tenth_intelligence/llm/llm/llm_analytics.py` — a stdlib-only,
read-only recording proxy between `llm_planner_node` and `llama-server` — plus the
pipeline it exists to measure.

**Facts live elsewhere.** `src/f1tenth_intelligence/llm/memory.md` is authoritative for
topology, parameter routing and hardcoded values; `src/f1tenth_intelligence/llm/README.md`
owns the narrative (four-source rule, D1–D4, the analytics walkthrough). This file links
to them and does not restate or overrule them. If the two ever disagree, they win and this
file gets corrected in the same session.

## 0. How to use this file

**Status** — one per backlog row:

| Status | Means |
|---|---|
| `Idea` | written down, not agreed, no work started |
| `Agreed` | we decided to do it; scope is clear |
| `In progress` | someone is on it right now |
| `Done` | landed in a commit **and** a command proves it |
| `Blocked` | cannot proceed — the row says what it waits on |
| `Parked` | deliberately not now — the row says why |

**Evidence** — a second, independent axis, because "implemented" is not one state:

| Evidence | Means |
|---|---|
| `built` | the code exists and imports |
| `unit-tested` | offline tests cover it (no model, no server) |
| `verified-live` | it has been run against a real `llama-server` and the output was read |

A row may be `Done` at `unit-tested` (test-only work) or `Done` at `verified-live`.
It is never silently both. Anything marked `verified-live` names the date and the
artifact it was verified against.

**Rules of this file**

1. No row is `Done` without a commit sha **and** the command that proves it.
2. One row per item; discussion goes in the Log (§9), not into the row.
3. Sections §0–§3 are the stable part; §5 grows, §9 grows forever, §7 grows too
   (a fixed bug keeps its row and gains the fix sha).
4. Never delete a Log entry. Corrections are new entries that say what changed.

---

## 1. The tool — how it works

### 1.1 What it is, and what it is not

It is a **recording proxy and analyser**. It forwards `/completion` traffic and writes
down what went past, so the numbers it reports are the numbers the node really saw.

It is **not** the AI. It runs no model, authors no plan, and makes no decision. The
"AI" is `llama-server` (the GGUF weights) plus `llm_planner_node` (which authors intent);
`llm_analytics` is the instrument pointed at them. In `--exec-file` and `--replay` modes
it does not even start ROS.

Three properties make its measurements trustworthy:

- **Read-only by construction.** Nothing in `llm_planner_node.py`, `plan_translate.py`,
  `llm.launch.py` or `models.yaml` is modified for it to work. The node is redirected by
  one parameter, `llm_url`, and the recording happens on the extra localhost hop.
- **Byte-exact records.** It stores the request body it forwarded (with its sha256) and
  the response body it received, before `plan_translate`'s airlock looks at either.
- **Pure analyser.** `analyse_exchange()` is bytes-in/dict-out with no sockets and no
  clock, so the whole analysis path is testable against canned llama.cpp responses.

### 1.2 Architecture

```
        llm_planner_node                          (or --exec-file: no node at all,
        (v2 path: prompt file -> /completion)      just the request the node would send)
                  |
                  |  POST /completion
                  |  raw prompt, n_predict 512, temperature 0.0,
                  |  json_schema {plan,unsupported}, no `stop` list, no ChatML
                  v
     :8084  llm_analytics  <-- records request + response + wall clock
                  |              (one record per exchange, in order, "occurrence")
                  v
     :8083  llama-server   -m ~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf
                           --port 8083 -ngl 999

     side channel, one background thread (0.5 s):
        MemoryPoller --> GET /slots, GET /metrics, /proc/<llama-server pid>/status (RSS)
        finds the pid by matching the listening socket inode to /proc/<pid>/fd

     --replay:  JSONL on disk --> summary + report     (no proxy, no server, no ROS)
```

### 1.3 One exchange, field by field

Every `/completion` exchange becomes one record with these sections (this is what the
JSONL line and the report carry):

| Section | What it answers | Notes |
|---|---|---|
| `input` | what exactly was sent | system prompt, command, **retry feedback** (any text after `risposta:` on a retry), `n_predict`, temperature, whether `json_schema` was present, `is_retry`, `occurrence` (nth time this command was seen) |
| `thought` | what exactly came back | raw model output verbatim, `json_valid` (the node's own `raw_decode`), `trailing_chars` discarded after the JSON, move / unsupported counts, and the `--airlock` verdict when asked |
| `timings` | how fast | prompt-eval and generation tok/s and ms, `server_ms` (their sum), `overhead_ms` (proxy wall clock minus `server_ms`), prefill vs decode share |
| `load` | how hard the server worked | `n_predict` requested vs applied, `budget_used`, `hit_token_limit`, `stop_type`, tokens evaluated / predicted / cached |
| `memory` | KV and RSS | KV occupancy from `/slots` (`kv unknown of N` when the build reports no `n_past`, never a fake `0%`), server RSS / peak / delta from `/proc/<pid>/status` |

Plus, per record: `request.body_sha256` + `body_chars` (so two runs are provably the same
request), `response.http_status`, `response.error`, `response.model`, `response.stop_type`,
`response.truncated`, `response.slot_id`, `response.unknown_keys` (fields this build
returns that the analyser does not know yet), and `warnings[]`.

Two deliberate silences: **GPU utilisation is not measured** (no `nvidia-smi` dependency —
KV + RSS are the proxy for load), and **`/metrics` needs llama-server `--metrics`, which
`llm.launch.py` does not pass** — the poller tries once, disables itself for the run and
says so. `/slots` and RSS keep working.

Warm-up calls (`_wait_for_llama_server` sends prompt `ciao`, `n_predict 8`) are recorded
and timed **separately** in the summary: they hit a cold server and would otherwise
dominate the per-command averages.

### 1.4 Artifacts and exit codes

- `llm_analytics.json` — one report: `session` block (upstream, prompt file + sha256,
  input snapshot + sha256, paths, warnings, start/finish), `summary` (aggregates, warm-up
  separated), `server` block (props, KV, RSS), `runs[]` (`--exec` only), `exchanges[]`.
- `llm_analytics.md` — the human twin, written automatically beside `--out`
  (`report.json` -> `report.md`, same folder): metadata, the same summary block, then
  one row per exchange — `Command Prompt | Temp | JSON Valid | Airlock Verdict |
  Latency | Tokens | Tok/s` (`Tokens` = evaluated + predicted, `Tok/s` = decode rate).
- `llm_analytics.jsonl` — one JSON line per exchange, appendable across runs, the only
  thing `--replay` needs.
- `commands.txt` — this run's own input, written beside `--events` before the first
  command goes out: `--exec-file` byte for byte (comments and line endings included, so it
  can be handed straight back to the same flag), `--exec` one command per line. Named in
  `report.md` as `- input:`, with a count and a sha256; skipped with `--no-events` (no
  folder) and for an idle recorder (no input). Truncated like the rest: one run, one copy.
- Exit codes: `0` clean, `1` at least one exchange or node run went wrong (the sweep still
  finishes and reports), `2` usage/IO problem (cannot listen, unreadable command file,
  `--live` with `--exec-file`).

---

## 2. Workflow guide

`scripts/analytics.sh` **is** the workflow. It sources `scripts/env/jazzy.sh`, owns the
server, makes and names the run folder — `analytics_runs/YYYY-MM-DD-<label>/`, holding
`report.json`, `report.md`, `events.jsonl` and `commands.txt` — preflights `/health`, and
prints the equivalent `ros2 run llm llm_analytics ...` line before each run. Nothing lands
in the repo, and the tool itself is untouched: every flag the wrapper passes is one of the
tool's own 19. The proxy only records, so with no upstream server a run cannot produce a
plan — the wrapper exits `3` before the tool starts rather than collecting 502s.

Comparing two `models.yaml` entries is the one thing the wrapper restarts the server for:
`compare A B` runs one model at a time (`up A` → sweep → `down`, then the same for B),
because two llama-servers would compete for the same RAM and every latency in the report
would measure contention instead of the model. Both captures land under
`analytics_runs/sequential/<date>-A-vs-B/` (`A/` and `B/`, each with its own
`report.json`, `report.md`, `events.jsonl`, `commands.txt` and `llama-server.log`), and
`scripts/compare_reports.py` writes the one `comparison.md`. That script is a stdlib-only
*reader*: metadata + the input checks, one aggregate table, one row per command.

| Task | Command | What it does |
|---|---|---|
| server — start | `scripts/analytics.sh up [--wait S] [--model NAME]` | starts `llama-server`, waits for `/health` (120 s), logs to `analytics_runs/_server/`; `--model` launches `model:=NAME` from `config/models.yaml` and records it in `_server/current_model` |
| server — inspect | `scripts/analytics.sh status` | `/health`, active model, launcher pid, RSS, log path |
| server — stop | `scripts/analytics.sh down [--force]` | SIGINT → SIGTERM → `--force` SIGKILL; a server it did not start is left alone |
| server — probe | `scripts/analytics.sh probe` | what this build actually exposes (`/metrics` 501 is expected, NEXT-06) |
| one prompt, no node | `scripts/analytics.sh one "<command>"` | `--exec-file`: one generation, the node's v2 payload |
| a whole file, no node | `scripts/analytics.sh sweep [FILE]` | `--exec-file`, one generation per line (default `test/commands.sample.txt`) |
| two models, back to back | `scripts/analytics.sh compare A B [FILE]` | `up A` → sweep → `down`, `up B` → sweep → `down`; one `comparison.md` under `analytics_runs/sequential/` |
| real node, dry run | `scripts/analytics.sh node "<command>"` | `--exec`: the node's retries, verdict and `RESULT` line; no `/mission/*` calls |
| real node, mission loads | `scripts/analytics.sh live "<command>"` | `--exec --live`; asks for confirmation first |
| idle recorder | `scripts/analytics.sh idle` | prints the node command to start by hand; Ctrl-C on the recorder writes the report |
| replay a capture | `scripts/analytics.sh replay <label>` · `replay <path>` · `--last` | offline: no server, no ROS; writes that folder's `replayed.json` / `replayed.md` |
| list runs | `scripts/analytics.sh runs [--sequential]` | run folders, newest first (`--sequential`: the comparisons) |
| newest run | `scripts/analytics.sh last [--sequential]` | the newest folder plus its verdict table |

**Copy-pasteable — one model, then two models.** `up` is optional: `one`/`sweep`/`node`
call `ensure_server` and exit `3` when nothing answers on the port, so either start the
server once and reuse it, or start it per label (as below).

```bash
# one model: one server, one capture folder per --label
scripts/analytics.sh up --model qwen25_3b_instruct   # /health awaited, name recorded
scripts/analytics.sh sweep --label qwen-sweep        # FILE defaults to the package sample
scripts/analytics.sh down                            # SIGINT -> SIGTERM -> --force

# two models: one command file, one comparison.md, strictly sequential
scripts/analytics.sh compare qwen25_3b_instruct llama32_3b_instruct --label qwen-vs-llama

# read either history (sequential/ never pollutes the single-model one)
scripts/analytics.sh runs --sequential               # comparisons, newest first
scripts/analytics.sh last --sequential               # newest one: checks + tables
```

`up --model NAME` and `compare A B` resolve `NAME` through `config/models.yaml`
(`model_path`, `port`, `extra_args`) and adopt that entry's port; each model folder keeps
its own `report.json`, `report.md`, `events.jsonl`, `commands.txt` and `llama-server.log`
beside the shared `commands.txt` and the one `comparison.md`. The verified-live example is
`analytics_runs/sequential/2026-11-25-smoke-two-model-3/` (a 2-line input, kept as that
folder's `commands.txt`; §4, §9).

Options that change what is measured: `--label NAME` (the date is prepended; a same-day
rerun of the same name becomes `-2`, `-3` …), `--model NAME` (`up`/`compare`), `--no-airlock`,
`--no-go-to` (swaps the prompt file), `--timeout S`, `--port N`, `--simple-discovery`.
`--sequential` re-points `runs`/`last` at the comparisons. Exit codes: `0` ok, `1` an
exchange failed (the report still exists), `2` usage, `3` preflight — no server, no binary,
nothing to replay, or (for `compare`) a server on the port that this script did not start.

**`scripts/analytics.sh help` is the cheat sheet; `ros2 run llm llm_analytics --help` is the
source of truth for the raw flags.**

### 2.1 What to read & conceptual pitfalls

Read, in this order: the per-exchange console line (`http 200 | 4200 ms | 38.4 tok/s | JSON
ok`), the summary block (warm-up timed apart), then the run's `report.md` / `report.json` —
the markdown twin is written automatically beside `--out`. `events.jsonl` is the log of
record and replays with no server and no ROS. To diff two runs, compare
`request.body_sha256` per command: same hash ⇒ same request, so a latency difference is the
machine, not the prompt.

**Reading the output — metric cheat sheet** (labels verbatim from `format_summary()`)

| Line in the summary | What it counts | How to read it |
|---|---|---|
| `exchanges N (warmup N, completion N, other N)` | every recorded call, split by `classify_exchange` | `warmup 0` is **normal** in `sweep`/`one`: only the node's `ciao` probe (`n_predict 8`) is tagged warm-up, and `--exec-file` never sends it. `warmup wall ms` keeps that cold call out of the averages. |
| `failures` · `retries` | exchanges with HTTP ≠ 200 · completions carrying `is_retry` | `retries 0` in `sweep`/`one` — no node, so no retry loop; use `node` when retries are the subject. |
| `prompt parsed` | request prompt carried the v2 template header (`PROMPT_HEAD`) | below `exchanges` means some line was not a planner request (or the prompt file changed). |
| `json valid N (trailing text in M)` | node's own `raw_decode` found JSON first · completions with bytes after it | `M > 0` is not a failure — the JSON parsed and the model kept talking (console: `+N chars after the JSON`). |
| `airlock accepted A rejected R (…)` | `--airlock` verdicts, tallied by error class | `IntentSchemaError` = valid JSON failing the intent schema/**range** check; `EmptyPlanError` = no move survived (ambiguous/impossible command — the correct refusal). It checks schema and ranges, **not semantic intent**: a schema-valid plan can still be the wrong action. |
| `wall ms mean/median/p95/max (n=…)` | end-to-end latency per completion | **median is the steady-state warm latency**; `max` (and to a lesser degree `mean`) is skewed by the cold first call — compare medians across runs. |
| `prompt eval … t/s` (+ `prefill ms`, `prefill share`) | prompt processing rate / time | call 1 pays the full cold prefill (~55 s at 3B q5 on CPU); warm calls hit the prompt cache and drop to ~1 s. `prefill share` is its % of server time. |
| `generation … t/s` (+ `decode ms`) | token generation rate / time | the steady-state model speed (e.g. 6.8 t/s); prompt-cache hits do not change it, so compare this, not `wall ms`, across prompts. |
| `overhead ms` | proxy wall clock minus `server_ms` | cost of the extra localhost hop (~7 ms expected); a spike blames the proxy, not the model. |
| `tokens total evaluated/predicted/cached` | prompt tokens actually processed · generated · prompt-cache reuse | `cached` climbing across a sweep is the cache working; `evaluated` is what prefill really recomputed. |
| `budget used …%` · `budget limited N` | share of `n_predict 512` consumed · answers that hit it | read alongside per-exchange `stop=` / `load.stop_type`: `eos` finished on its own, `limit` hit the budget (the plan may be cut mid-JSON). |
| `rss MiB baseline/peak/last/delta` | `llama-server` host RAM from `/proc/<pid>/status` | **host RAM (`VmRSS`/`VmHWM`), not GPU VRAM**; a growing `delta` across a sweep is the leak signal. |
| `kv cache …` · `metrics delta` | KV occupancy from `/slots` · counter deltas from `/metrics` | expect `kv unknown of N tokens` and `metrics not served (llama-server without --metrics)` until NEXT-06 — a known limit of this build, not a broken run. |

The wrapper owns the folder, both flags and the `/health` check. These are the traps it
cannot do for you:

- `one` / `sweep` (`--exec-file`) **do not start the node and test no retries** — use `node`
  when the retry loop is the thing under test.
- A `node` dry run **still writes** the generated mission to
  `f1tenth_behavior/share/f1tenth_behavior/missions/llm_generated/` — expected; inspect or
  delete it.
- Two runs are comparable only when the prompt `sha256` **and** every `request.body_sha256`
  match. `--no-go-to` swaps the prompt **file**, so its numbers describe a different system.
- **Context length is a model property here, not a launch decision.** `llm.launch.py` passes
  no `-c`, so `llama-server` adopts the GGUF's own `n_ctx_train`: **32768** for
  `qwen25_3b_instruct`, but **131072** for `llama32_3b_instruct` — at ~115 KB of KV per token
  that is ~15 GB of cache, and on this 16 GiB host the kernel OOM-killed the server mid-load
  (2026-11-25, `journalctl -k`). `models.yaml` therefore pins `-c 16384` on the Llama entry
  (~1.9 GB KV, ~4.2 GB RSS). Consequence for a comparison: the two arms can differ in `n_ctx`
  by necessity — read `server … n_ctx` in each arm's `report.md` before attributing a latency
  difference to the model.
- A non-zero exit on a sweep means **at least one exchange failed** (the sweep still finishes
  and writes its report); read `session.warnings` for which line.
- `/metrics` returning **501** is expected on this build — `llm.launch.py` passes no
  `--metrics`, so the poller disables itself and says so (NEXT-06). RSS and `/slots` keep
  working.

*References to §2.1–§2.4 inside older §9 entries date from the pre-2026-11-25 layout and are
kept as written.*

---

## 3. The pipeline being measured (just enough to read a report)

```
command (IT)
  -> prompt file           prompts/planner_system_prompt.v2.it.txt
                           or ...v2.no_go_to.it.txt when go_to_enabled=false
  -> /completion           prompt = <system prompt> + '\ncomando: "<cmd>"\nrisposta:'
                           n_predict 512, temperature 0.0, json_schema {plan, unsupported}
                           no ChatML templating, no `stop` list
  -> raw_decode()          first JSON value only; trailing text logged and dropped
  -> schemas/intent_v1.json   straight | turn | reverse | go_to  (+ unsupported[])
  -> plan_translate.translate()   adds speed/timeouts/turn magnitude; every field
                                  INTENT / CONFIG / DERIVED / CONST, no fifth source
  -> mission schema 3.0   (5.0 when the plan has a go_to phase)
  -> missions/llm_generated/<mission_id>.json   id = prefix_sha1(canonical intent)[:12]
  -> /mission/abort_mission -> /mission/load_mission -> /mission/start_mission
```

`RESULT {json}` is the node's one stdout line: `status ∈ ok | llm_error | translator_error
| refused | node_error` (`refused` = `EmptyPlanError`, the *correct* answer to an ambiguous
or impossible command, never retried).

Two mapping facts that make reports readable: guard `wall` becomes the `front_clearance`
stop condition, which reads **`/perception/front_distance`** (object-blind) — not the
similarly named `/perception/front_clearance` used by the safety path; guard `front_object`
becomes `obstacle_distance_below` with `forward_only: true`. Full detail and the D1–D4
decisions: `src/f1tenth_intelligence/llm/README.md`.

---

## 4. Current state — built / unit-tested / verified-live

**The headline, stated plainly: the tool is fully built, offline-tested and — since
2026-11-12 — verified live against a real model.** `alex-desktop` now runs
`llama-server` (pid 95960, `:8083`) on
`/home/alex/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf`, so the refusal path
(5 × HTTP 502) is no longer the only evidence: NEXT-02 recorded **5 real generations,
5/5 JSON-valid, 0 transport failures** through the proxy. Artifacts are kept untouched
in `src/f1tenth_intelligence/llm/analytics_runs/NEXT-02-first-capture/` (gitignored,
decision D-B) and replay offline with `--replay events.jsonl`.

**Second capture, 2026-11-18 — the first one large enough to benchmark:** 20 Italian
commands, 139 s, **20/20 JSON-valid, 0 failures, 0 retries**, airlock 12 accepted /
8 rejected. Judged by hand against the prompt's own rules, **9/20 produced the intended
plan** and **3 of the 12 accepts were silently wrong** — the airlock validates schema and
range, not intent. `analytics_runs/2026-11-18-20_first_attempt/` (untracked, D-B) holds
`report.{json,md}`, `events.jsonl`, the replayed twin, the `NOTES.md` verdict table and
`commands.txt` — the input itself, copied beside the capture since D-F.
It also filed **BUG-02** (§7) and gave `reverse` its first live evidence.

The model under test is **Qwen2.5-3B-Instruct, q5_k_m** — 3 B parameters, 4 slots,
n_ctx 32768 — measured at **6.8 tok/s** decode. Its one systematic failure mode, visible
for the first time here, is dropping `guard`/`thresh` on a `straight` move, which the
schema rejects; the airlock caught it 2 of 5 times.

**Two models, 2026-11-25 — the sequential `compare` pipeline is verified live.**
`config/models.yaml` now holds **two** entries on the same port: `qwen25_3b_instruct`
(`qwen2.5-3b-instruct-q5_k_m.gguf`, :8083, GGUF default `n_ctx` 32768) and the newly
registered `llama32_3b_instruct` (`Llama-3.2-3B-Instruct-Q5_K_M.gguf`, :8083,
`extra_args: ["-ngl", "999", "-c", "16384"]` — the `-c` exists because this GGUF's
`n_ctx_train` is 131072 and the uncapped server was OOM-killed, §2.1 / §9). One command,
`scripts/analytics.sh compare qwen25_3b_instruct llama32_3b_instruct /tmp/smoke_2models.txt
--label smoke-two-model`, booted each server in turn, ran the same 2 Italian commands,
stopped it and rendered
`analytics_runs/sequential/2026-11-25-smoke-two-model-3/comparison.md` (gitignored, D-B;
the `/tmp` input was deleted afterwards, but its text — and a byte-identical copy in each
arm's folder — is that run's `commands.txt`, D-F).
Both arms booted, answered and stopped cleanly (`down :8083 no longer answers` twice, tool
exit 0 twice) and every check read `match` (command file `6c185c307322`, prompt
`84ab53aa1ea8`, 2/2 identical `body_sha256`). Result: both 2/2 JSON-valid, 0 failures,
0 retries; airlock qwen 1 accepted / 1 rejected (`IntentSchemaError` on
`{'mode':'straight'}`, BUG-01 again) vs Llama 2 accepted; median wall 36519 vs 38948 ms;
decode 6.1 vs 4.5 tok/s; peak RSS 3657 vs 4175 MiB. Two same-label attempts (`-` and `-2`)
remain on disk as partial captures — one killed mid-run, one by the OOM killer — and are
kept, not deleted: `-3` is the only folder with a `comparison.md`. Read it with
`scripts/analytics.sh last --sequential`.

| Component | Built | Unit-tested | Verified-live | Note |
|---|---|---|---|---|
| Recording proxy (`CompletionProxy`, `:8084`→`:8083`) | ✓ | ✓ | **✓ 2026-11-12** | 5 exchanges recorded live; refusal path (5 × 502) was the only earlier evidence |
| Exchange analyser (`analyse_exchange` + parse/summarise helpers) | ✓ | ✓ | **✓ 2026-11-12** | real `timings`, `overhead_ms`, budget, `stop_type` on live traffic |
| `--exec` (node-driven, dry run default) | ✓ | ✓ | ✗ | unchanged: no node was run this session; 5 × 502 from the earlier attempt |
| `--exec-file` (batch, node's v2 payload) | ✓ (committed) | ✓ (9 new tests) | **✓ 2026-11-12, ✓ 2026-11-18** | NEXT-02: 5 commands, 5/5 JSON-valid, 0 failures, 0 retries. 2026-11-18: 20 commands, 20/20 JSON-valid, 0 failures, 0 retries, airlock 12 accepted / 8 rejected, 139 s |
| `--replay` (offline summary) | ✓ | ✓ | n/a | offline by design, no server needed |
| `--probe` (`/health`, `/props`, `/slots`, `/metrics`, 8-token gen) | ✓ | ✓ | **✓ 2026-11-12** | every endpoint answered; `/metrics` **501** on this build |
| `--airlock` (`translate()` verdict) | ✓ | ✓ | **✓ 2026-11-12** | 2 accepted / 3 rejected, reason recorded per command |
| `MemoryPoller` (`/slots`, `/metrics`, `/proc` RSS) | ✓ | ✓ (canned fixtures) | **✓ RSS + `/slots`** / ✗ `/metrics` | 501 → poller disables itself and says so (NEXT-06) |
| `console_script` (`ros2 run llm llm_analytics`) | ✓ | — | ✓ 2026-11-04 | symlink install → source edits live |
| `llama-server` bringup (`llm.launch.py`) | ✓ | — | **✓ 2026-11-12 on `alex-desktop`** | Qwen2.5-3B q5_k_m, 4 slots, n_ctx 32768; ~20–30 s load |
| Sequential 2-model `compare` + `scripts/compare_reports.py` | ✓ | ✓ (8 tests) | **✓ 2026-11-25** | `sequential/2026-11-25-smoke-two-model-3`: both arms clean, 2/2 identical bodies, qwen 1 acc/1 rej vs Llama 2 acc, 36519 vs 38948 ms median, 6.1 vs 4.5 tok/s |
| Pipeline measured: v2 node + translator + prompts/schema | ✓ | ✓ | partially | translator verified via `--airlock`; node + `/mission/*` still unrun |

**Test surface** (test functions per file, `llm/test/`): `test_llm_analytics.py` 83 ·
`test_intent_go_to.py` 59 · `test_intent_translate.py` 37 · `test_plan_translate.py` 36 ·
`test_planner_path.py` 15 · `test_llm_planner_request.py` 10 · `test_intent_evals_live.py` 3
(marker `llm`, needs the real model) · copyright/flake8/pep257 1 each — plus
`scripts/test_compare_reports.py` 8 (outside `llm/test/`, run from the workspace root).

**Last measured suite state (2026-11-04, not re-verified since — that is NEXT-03):**
llm package **71/71 green** from a sourced workspace; workspace-wide
`pytest -m "not llm"` = **6 failed / 306 passed / 1 skipped**. The 6 are the stale-payload
and prompt-example drifts (NEXT-05).

**Baseline is committed (2026-11-12, `60c2ce46`) and the package tree is clean.** That
commit carries the `--exec-file` pass (`llm_analytics.py` +211, 9 tests,
`test/commands.sample.txt`, README runbook +107, `memory.md` +23) together with
`planner.md`, `memory-bank/`, `changelog.md` and `.clinerules` — 13 files, 1337 insertions.
`git status --short -- src/f1tenth_intelligence/llm/` is empty. Files belonging to *other*
packages are still modified in the tree and were deliberately left out of it (root
`README.md`, `output/`, `f1tenth_sim/`, `f1tenth_perception/`, `f1tenth_more/`). NEXT-01 and
NEXT-02 are both `Done`; the ✗ left in this table are the live `--exec` row and `/metrics`
(NEXT-06).

**Documented drifts that touch this tool** (full register in `llm/memory.md`):
drift 7 was **half retired on 2026-11-12** — `--exec-file` is now `verified-live`, and the
"no GGUF, no `llama-server` on `alex-desktop`" clause is simply false (both exist; see §9);
`--exec` is still unexercised live. Standing: standalone
`pytest test/test_llm_analytics.py` needs `src/f1tenth_behavior` on `PYTHONPATH` — 70/71
unsourced, 71/71 sourced (drift 8); the setuptools 84.2.1 build trap (drift 9);
`llm_analytics` is referenced by nothing else in the workspace.

---

## 5. Backlog

Ordered by "what unblocks the most next", not by size. Review at the start of a session;
move a row to `In progress` before touching it.

| ID | Item | Why | Done when | Status | Evidence |
|---|---|---|---|---|---|
| **NEXT-01** | Commit the pending `--exec-file` pass (code + 9 tests + README runbook + `memory.md`); decide whether `memory-bank/`, `changelog.md`, `.clinerules` are tracked | The tool's newest mode and this planner were untracked; nothing can be called `Done` on a dirty tree | **`60c2ce46`** — `git status --short -- src/f1tenth_intelligence/llm/` prints nothing; D-D answered by doing it (all four tracked) | Done | unit-tested |
| **NEXT-02** | **First live capture** on the machine that holds the GGUF: `--exec-file test/commands.sample.txt --airlock`, keep report + JSONL | Every ✗ in §4 waited on this. No live generation had ever been recorded | JSONL with 5 JSON-valid answers, tok/s, `overhead_ms`, KV/RSS; prompt name + sha256 copied into §9 | **Done** | `verified-live` 2026-11-12 — `llm/analytics_runs/NEXT-02-first-capture/{report.json,events.jsonl}`; prompt `planner_system_prompt.v2.it.txt` sha256 `84ab53aa1ea8`; 5/5 JSON, 0 failures, airlock 2 accepted / 3 rejected — the two schema rejections are **BUG-01** (§7) |
| **NEXT-03** | Re-baseline §4 here: `scripts/colcon_build.sh build/test --packages-select llm` | The 71/71 and 6F/306P figures date from 2026-11-04; the tree has moved | §4 carries real, dated numbers | Agreed | — |
| **NEXT-04** | Test isolation: `pytest test/test_llm_analytics.py` must work unsourced (import guard → clean skip, or declare the dependency) | 70/71 from a bare tree makes a real failure indistinguishable from an environment miss | 71/71 from the package dir with no `PYTHONPATH` help | Idea | — |
| **NEXT-05** | Green the rest of the package: 3 stale payload tests in `test_llm_planner_request.py`, prompt-example count `17 == 14` | `--airlock` verdicts and prompt edits are only trustworthy on a green suite | `colcon_build.sh test --packages-select llm` green | Idea | — |
| **NEXT-06** | Close the two stated blind spots: pass `--metrics` in `llm.launch.py`; add opt-in `nvidia-smi`/`tegrastats` sampling | KV-from-metrics self-disables today; GPU load is unmeasured by design | KV occupancy reported live; GPU sampler behind a flag, documented in §1.3 | Idea | — |
| **NEXT-07** | Use the tool for what it exists for: prove constrained decoding on this build (`budget_used`, `hit_token_limit`, `stop_type`, `trailing_chars`) | Prompt choices currently rest on an unmeasured assumption | One capture that settles it; conclusion written into §9 | Idea | — |
| **NEXT-08** | `go_to` numbers: sweep a go_to command set → `target_not_found` / `target_lost` rates, gap defaults per class | "go_to unreliable" is documented with zero data behind it | A JSONL plus one sentence with rates and gaps | Idea | — |
| **NEXT-09** | Docs sync: `llm/README.md` predates `reverse`; param table misses `..._wall_default_distance` / `..._before_turn`; drift register renumber | §8 pointers must stay honest | README matches code and `memory.md` | Idea | — |
| **NEXT-10** | Hardware (parked until car time): D1 `wall_turn_then_straight`, D3 left/right physical sense, `reverse` on the car, retune the 3 SYNTHETIC knobs (`open_guard_max_distance`, `timeout_factor`, `timeout_floor`) | Closes D1/D3 and turns the timeout formula from synthetic to measured | Run logs cited in §9; params updated with the measured basis | Parked | — |

---

## 6. Open decisions

| # | Decision | Options | Leaning | Blocks |
|---|---|---|---|---|
| D-A | One sweep file or one per go_to mode? | single file (sample mixes both) vs `--no-go-to` variant with its own file | two files, named after the prompt sha. **Evidence 2026-11-18:** the single file carried all 20 commands with `go_to` on (3 of the 12 accepts are `go_to`), so the positive arm needs one file only; a `--no-go-to` twin is for the refusal arm alone | NEXT-02, NEXT-08 |
| D-B | Where do captures live? | `llm/analytics/<date>-<prompt_sha7>/` committed vs `~/llm_runs/` untracked | **decided 2026-11-12:** in-package `llm/analytics_runs/<label>/`, untracked via the package `.gitignore` — keep the logs, commit only the conclusions (§9) | — |
| D-C | Retry behaviour: measure or assume? | `--exec` (node's real retries) vs `--exec-file` (1 generation) for the standing benchmark | `--exec` for the benchmark, `--exec-file` for prompt iteration | NEXT-02 |
| D-D | Track `memory-bank/` and `changelog.md` in git? | commit them vs `.gitignore` | commit — they are working state, and losing them costs more than the diff noise | NEXT-01 |
| D-E | GPU sampling dependency | parse `nvidia-smi` CSV vs use NVML via `pynvml` vs stay KV+RSS only | `nvidia-smi` CSV behind `--gpu`, no new import | NEXT-06 |
| D-F | Does a run folder keep its own input? | hand-copied by the operator vs written by the tool beside `--events` vs behind a `--keep-input` flag | **decided 2026-11-18:** written by the tool, always, no flag — `.clinerules` already requires the input beside its capture, and the step somebody must remember is the step that goes missing. Byte-identical for `--exec-file`, so the copy is re-runnable as-is | — |
| D-G | Who owns the run-folder ritual — the operator or a script? | keep §2 as prose (18-flag command lines) vs a wrapper that owns env + folder + label + preflight vs changing `llm_analytics`' own defaults | **decided 2026-11-19:** a wrapper, `scripts/analytics.sh`, and no change to the tool. Its 19-flag parser and 83 tests stay the contract; the wrapper passes only documented flags, always passes `--out`/`--events` into a folder it made, and turns `--label` into `YYYY-MM-DD-<label>` (`-2`, `-3` on a same-day rerun) so the append-only rule is enforced instead of remembered. It also owns `up/status/down` because `stack_params.yaml`'s `llama_server_path` default is the Jetson's and every session was retyping the two overrides | — |

---

## 7. Backlog & Known Bugs

§5 is the roadmap — NEXT-01…NEXT-10, ordered by what unblocks what. **This** section is
the defect register: one row per **defect**, with the evidence that proves it. Rows are
never deleted. A fixed bug keeps its row and gains the fix sha, because knowing that
something was once real is worth more than a tidy table.

| ID | Symptom | Status |
|---|---|---|
| **BUG-01** | A `straight` phase with no `guard` reaches the schema; the model can also invent an extra phase when the stop condition trails its clause | Open — documented 2026-11-13, fix **not** implemented |
| **BUG-02** | A `straight` phase with `guard` `front_object` and no `thresh` is rejected, although the prompt documents a default of 1.0 for it; no defaulter, no param, no schema allowance exists | Open — documented 2026-11-18, fix **not** implemented |

### BUG-01 — `{"mode": "straight"}` with no `guard` (the generic `oneOf` rejection)

**Observed** in NEXT-02 — `analytics_runs/NEXT-02-first-capture/` (`events.jsonl`,
`replayed.md`), **2 of 5** first attempts. The field is `thought.raw_output`:

| Command | Raw model output | Verdict |
|---|---|---|
| `vai dritto e gira a sinistra alla parete` | `[{"mode":"straight"},{"mode":"turn","dir":"left"},{"mode":"straight","guard":"wall"}]` | `IntentSchemaError` |
| `gira a sinistra, vai dritto, poi gira di nuovo a sinistra` | `[{"mode":"turn","dir":"left"},{"mode":"straight"},{"mode":"turn","dir":"left"}]` | `IntentSchemaError` |

Both airlocks read identically, and uselessly:

```
intent does not validate: {'mode': 'straight'} is not valid under any of the given schemas
```

**Trigger — human grammar.** Italian puts the stopping condition in a *trailing*
prepositional phrase ("… e gira a sinistra **alla parete**"). The model decodes the first
clause as a finished object, emits `}` after `"mode":"straight"`, and only then reads the
tail of the sentence. Decoding is autoregressive and left to right: the guard belonging
to "alla parete" did not exist yet when that object was closed, so it could not be put
inside it.

In the first row it is worse than a missing key. Re-parsing "gira a sinistra alla parete"
as *turn left, then drive to the wall*, the model emitted **three** phases for a
two-clause command — the guard got attached to an invented third motion. The operator
asked for two things and got a plan with three; the extra `straight` is a hallucinated
move, not a formatting slip.

**Why nothing rescues it.** `translate()` (`plan_translate.py` l.1014-1023) repairs
*before* validating:

```python
intent, defaulted_walls = _default_wall_thresh(intent, cfg)   # l.1014
jsonschema.validate(intent, load_intent_schema())             # l.1017
...
_check_ranges(plan)                                           # l.1023
```

and `_default_wall_thresh()` (l.951-981) is gated on the guard **already being present**:
`phase.get('mode') == 'straight' and phase.get('guard') == 'wall' and 'thresh' not in
phase`. It supplies a missing **thresh**, never a missing **guard**. A bare
`{"mode":"straight"}` therefore gets no default, matches none of the four `oneOf` branches
(`intent_v1.json` l.19-43: `additionalProperties: false`, `required: [mode, guard,
thresh]`), and aborts **before** `_check_ranges()` can say anything — which is why the
message is jsonschema's generic sentence naming the whole instance instead of "phase 0
has no guard".

**Proof the cause is the guard, not the thresh.** In the same capture
`vai dritto, al muro gira a destra e avanza 2 metri` produced
`{"mode":"straight","guard":"wall"}` **with no `thresh`** and was **accepted**: the
defaulter filled `wall_default_distance_before_turn = 3.0` because the next phase is a
turn (l.434-447). Omitting `thresh` is legal by design. Omitting `guard` is fatal.

**Why the request grammar cannot prevent it.** Both callers send llama-server an
**envelope-only** `json_schema` — `plan: {array of {type: object}}`, `unsupported: {array
of string}` (`llm_planner_node.py` l.347-365, copied verbatim by
`llm_analytics.INTENT_REQUEST_SCHEMA`). Any object is grammar-legal, so the server happily
closes the brace and stops, exactly as its own comment advertises; the per-branch
`required` / `additionalProperties: false` / `oneOf` structure is not expressible in this
build's JSON→grammar path. Guard presence is therefore a **prompt-compliance** property,
and the prompt states it only by example (the response template and the `vai dritto`
rule) — never as a rule that omission is a rejection.

**Mitigating fact — do not quote 2/5 as a mission-failure rate.** `--exec-file` judges one
generation per line (`airlock_verdict()`, `llm_analytics.py` l.404-430). The node retries:
`_plan_v2` catches `IntentSchemaError` / `IntentRangeError` and re-prompts **with the
message as feedback** (`llm_planner_node.py` l.1263-1272, `MAX_INTENT_RETRIES`). So this
is a 40 % *first-attempt* rejection rate. The hole is that the retry is fed the
uninformative `oneOf` sentence, which never mentions the guard, so the plan is re-rolled
rather than corrected.

**Proposed fix — NOT implemented** (deliberately, so the failure rate stays measurable
against this baseline):

- **A′ — reject explicitly, in Python, before the schema.** Walk `plan` and raise a
  specific `IntentSchemaError` such as `plan[0]: straight phase is missing guard
  (wall|front_object|distance)` whenever `mode == "straight"` and `guard` is absent.
  Validation stays strict, no intent is loosened, the 2/5 rate stays comparable — but the
  node's retry finally receives actionable text and the markdown table becomes readable.
  Verifiable offline: `--replay` reuses the recorded generations, so the new message can
  be diffed against `replayed.md` without a server.
- **B — harden the prompt.** One explicit rule ("ogni fase `straight` DEVE portare
  `guard`; una `straight` senza `guard` viene rifiutata") plus a **negative** example
  showing `{"mode":"straight"}` as wrong. This attacks the 40 % at the source, but it
  changes the prompt sha, so it needs a **fresh live capture** — `--replay` cannot
  validate it, since it replays generations the old prompt produced.
- Considered and **rejected**: defaulting `guard: "wall"` inside `_default_wall_thresh()`.
  It would make both rows pass, but it silently blesses non-compliance and destroys the
  only signal that measures prompt quality.
- **Done when:** `--replay` shows the specific message on both rows (5/5 JSON unchanged,
  airlock still 2 accepted / 3 rejected for A′); a fresh live capture shows the rate after
  B; and one `--exec` run reports how many retries recover.

**Noted while diagnosing, not filed separately:** `llm_planner_node.py` l.33 still claims
"NESSUN vincolo grammar/json_schema" although l.347 sends one (belongs to NEXT-09); the
last prompt examples in `planner_system_prompt.v2.it.txt` omit the `risposta:` label that
the other examples use.



---

### BUG-02 — `front_object` without `thresh`: the prompt documents a default that does not exist

**Observed** 2026-11-18 in `analytics_runs/2026-11-18-20_first_attempt/`, **2 of 20**
commands. Field: `thought.raw_output`.

| Command | Raw model output | Verdict |
|---|---|---|
| `Vai dritto ma fermati prima dell'ostacolo, poi gira a sinistra.` | `[{"mode":"straight","guard":"front_object"},{"mode":"turn","dir":"left"}]` | `IntentSchemaError` |
| `Vai dritto, oltrepassa la scatola e fermati al muro.` | `[{"mode":"straight","guard":"distance"},{"mode":"go_to","target":"box"},{"mode":"turn","dir":"right"},{"mode":"straight","guard":"wall"}]` | `IntentSchemaError` |

Both report the same generic jsonschema sentence, naming the phase but not the key:

```
intent does not validate: {'mode': 'straight', 'guard': 'front_object'} is not valid under any of the given schemas
```

**This one is a contradiction in the repo, not a model slip.** The Italian prompt tells
the model that omitting `thresh` on `front_object` is legal *and* what the number will be:

- l.30 — `- guard "front_object": 0.5 - 5.0    default 1.0`
- l.38-39 — *"Se una distanza non e' detta: con guard "wall" ometti thresh (la sceglie il
  sistema), con "front_object" usa il default."*

Three places would have to make that true, and none does:

| Layer | What it says | Consequence |
|---|---|---|
| `intent_v1.json` | `thresh` is in `required` on every `front_object` branch | omission is a validation error |
| `plan_translate.py` l.951-981 `_default_wall_thresh()` | gated on `phase.get('guard') == 'wall'` | `front_object` is never filled |
| `TranslatorConfig` l.595-596 | has `wall_default_distance` (0.5) and `wall_default_distance_before_turn` (3.0) **only** | there is no `front_object` number to fill in; `grep -rn front_object src/f1tenth_params/` is empty |

So a model that follows the prompt exactly is rejected, and the retry feedback it gets
back repeats the instruction that just failed — NEXT-02's `_plan_v2` loop cannot converge
on this command. Unlike BUG-01 there is no grammar excuse: the guard *was* emitted
correctly, the sentence was understood correctly (`front_object` is the right guard for
"fermati prima dell'ostacolo", l.49), and only the documented default is missing.

**`distance` is a different case, not part of this bug.** Prompt l.31 gives it
`nessun default` and l.39-40 calls a distance-less `distance` guard *ambiguous*, so the
second row's first phase should have been refused as ambiguous; the model invented
phases instead (a BUG-01-shaped failure on a different key). Keep the two apart when
fixing: `front_object` must gain a default, `distance` must keep failing.

**Proposed fix A′ (deliberately not implemented, same discipline as BUG-01):** in
`translate()` before `jsonschema.validate()`, fill
`thresh = front_object_default_thresh` when `mode == "straight" and guard == "front_object"
and "thresh" not in phase` — a **new** `TranslatorConfig` field sourced from
`stack_params.yaml` like `wall_default_distance`, defaulting to 1.0 so the prompt's own
number becomes true. Record `distance_source: default` the way the wall path already
does (l.1044-1046), so the report distinguishes a spoken distance from a filled one.
Do **not** relax the schema. Re-running this exact sweep then gives a before/after pair:
commands 5 and 8 flip to `Accepted`, command 6 must **not** change.

---

## 8. Pointers — where the facts actually live

| Need | File |
|---|---|
| The tool's own help | `ros2 run llm llm_analytics --help` (19 flags, defaults straight from `argparse`) |
| Narrative: four-source rule, D1–D4, analytics walkthrough, end-to-end trial runbook | `src/f1tenth_intelligence/llm/README.md` |
| **Authoritative** topology, parameter routing, hardcoded values, drift register | `src/f1tenth_intelligence/llm/memory.md` |
| Prompts and which one pairs with which `go_to` setting | `llm/prompts/planner_system_prompt.v2{,no_go_to}.it.txt`, loader in `plan_translate.intent_prompt_filename()` |
| The intent contract the model must satisfy | `llm/schemas/intent_v1.json` |
| Model list and GGUF path | `llm/config/models.yaml` |
| Node parameters and their defaults | `llm/config/llm_planner_params.yaml`, `src/f1tenth_params/config/stack_params.yaml` |
| Build/test wrapper (bare `colcon` is broken here) | `scripts/colcon_build.sh` |
| Example sweep input | `llm/test/commands.sample.txt` |
| Workspace router for agent context | `memory-bank/01-index.md` |

**Maintenance contract for this file:** add to §9 rather than rewriting history; when a
fact changes, fix the section *and* log why. §0–§3 are stable prose, §4 is re-measured
(NEXT-03), §5 is re-ordered, §6 shrinks as decisions land, §7 only ever grows (a bug is
never deleted), §8 is the pointer index, §9 is this log.

---

## 9. Log — append only, newest first, never pruned

*Entries older than the 2026-11-25 (session 2) entry below use the numbering of their day:
"§7" there is this log (now §9), "§8" is the bug register (now §7), "§9" is the set of
pointers (now §8). No entry text was rewritten when the log moved to the end of the file.*

### 2026-11-25 (session 4) — `llama32_3b_instruct` registered; `compare` verified live

- **Registered:** `config/models.yaml` gains `llama32_3b_instruct` →
  `~/projects/llm/models/Llama-3.2-3B-Instruct-Q5_K_M.gguf`, `port: 8083` (same port as
  qwen on purpose: `compare` hands it back between arms), same three-key schema as
  `qwen25_3b_instruct`.
- **One deliberate difference in `extra_args`:** `["-ngl", "999", "-c", "16384"]` instead
  of qwen's `["-ngl", "999"]`. `llm.launch.py` passes no `-c`, so llama-server adopts the
  GGUF's `n_ctx_train` — **131072** for Llama-3.2 — and at ~115 KB of KV per token
  (28 layers × 8 KV heads × 128 dim × 2 × 2 B) that is ~15 GB of cache. Measured, twice:
  the load stalls after `load_model: loading model '...'` and the kernel kills it
  (`journalctl -k`: `Out of memory: Killed process 200902 (llama-server) … total-vm:15567340kB,
  anon-rss:8904528kB`). At 16384 the KV cache is ~1.9 GB and the live run measured ~4.2 GB
  peak RSS. **Cost:** the arms differ in `n_ctx` (32768 vs 16384) — a comparison caveat
  recorded in §2.1, not a model difference.
- **Live smoke, one command:**
  `scripts/analytics.sh compare qwen25_3b_instruct llama32_3b_instruct /tmp/smoke_2models.txt
  --label smoke-two-model` → `analytics_runs/sequential/2026-11-25-smoke-two-model-3/`.
  Both arms `up` (`/health` ok) → 2 commands → `down` (`:8083 no longer answers`), tool exit
  0 twice, `llama32_3b_instruct` detected from `/props` and recorded. All four checks
  `match`: command file `6c185c307322`, prompt `84ab53aa1ea8`, 2/2 identical request
  `body_sha256`, 2/2 identical command text.
- **Result:** both arms 2/2 JSON-valid, 0 failures, 0 retries. Airlock: qwen 1 accepted /
  1 rejected (`IntentSchemaError` on `{'mode': 'straight'}`, BUG-01 reproduced) vs Llama 2
  accepted. Median wall 36519 vs 38948 ms — command 1 pays the ~65 s cold prefill in both,
  command 2 is 7.7 s vs 11.7 s. Decode 6.1 vs 4.5 tok/s. Peak RSS 3657 vs 4175 MiB. The
  page is `comparison.md`; `scripts/analytics.sh last --sequential` reprints checks +
  tables, `runs --sequential` lists it.
- **Append-only, so two partial attempts stay:** `2026-11-25-smoke-two-model` (killed while
  Llama reloaded) and `-2` (the OOM). Each holds a complete qwen arm and an empty Llama arm;
  `-3` is the only folder with a `comparison.md`. The input `commands.txt` is kept in all
  three (D-F); the `/tmp` input file was deleted after the run.
- **Untouched:** `llm_analytics.py` and its 83 tests (still 83/83 green).

### 2026-11-25 (session 3) — `compare A B`: two models, one page

- **Why:** the question the capture cannot answer is "is this model better than that one?",
  and answering it by hand means two sweeps, two prompt checks and a diff nobody writes.
  `llm_analytics` stays untouched (83/83 still pass): this is orchestration plus a reader.
- **Server selection:** `up --model NAME` validates `NAME` against `config/models.yaml`, passes
  `model:=NAME` to `llm.launch.py`, adopts that entry's port unless `--port` was explicit, and
  records the live name (from `/props` `model_path`, else the gguf basename) in
  `_server/current_model` — `status` prints it.
- **`compare A B [FILE]`:** strictly sequential (`up A` → sweep → `down` → `up B` → sweep →
  `down`), so RAM, page cache and thermals do not turn into a second variable. Both models read
  one copy of the command file, `analytics_runs/sequential/<date>-A-vs-B/commands.txt`.
- **Safety:** a server on `:8083` that this script did not start is never killed — `compare`
  exits `3` and says how to stop it; a server it did start is stopped and the port polled until
  it is really free. Each model keeps its own `llama-server.log` before `up` rotates the shared one.
- **`scripts/compare_reports.py`:** new stdlib-only reader with explicit flags (`--a/--b/--out/
  --name-a/--name-b/--commands/--exit-a/--exit-b`, no sidecar file). It checks the command file
  (session sha256, or each capture's own `commands.txt` for pre-2026-11-19 reports), the system
  prompt sha256 and every request `body_sha256`; a mismatch is printed with its position.
  Then: one aggregate table (completions, JSON valid, airlock accepted/rejected/by_error, median
  and mean wall ms, decode tok/s, peak RSS) and one row per command with `≠ airlock` / `≠ json`
  / `missing` flags. Missing `summary`/`server` blocks fall back to the exchanges instead of
  crashing. 8 tests in `scripts/test_compare_reports.py`.
- **Listing:** `runs`/`last` skip `sequential/`; `runs --sequential` / `last --sequential` list
  the comparisons, so the single-model history stays comparable run-for-run.
- **Limit:** `models.yaml` currently has one entry, so a live two-model comparison needs a second
  one before it can run on this host; everything up to `up` is exercised by fixtures and tests.

### 2026-11-25 (session 2) — the log moved to the end of the file

- **Why:** the Log is append-only history. It was §7, wedged between the decision table and
  the bug register, so a first read hit the longest section before the two short ones it
  cites. Rule 4 still holds — this file is *written to* the log, not read from it.
- **Moved:** §7 Log is now **§9**, the last section; §8 Backlog & Known Bugs is now **§7**
  and §9 Pointers is now **§8**, so the numbering stays monotonic (same rule as
  2026-11-13). The entries themselves are byte-identical: renumbered heading, nothing
  rewritten, nothing pruned.
- **References:** every live cross-reference was updated — §0 rules 2–3, §2.1's italic
  note, §3's BUG-02 pointer, §4's drift pointer, the NEXT-02/07/09/10 rows and D-B, and
  the maintenance contract. References *inside* older entries keep their original numbers;
  the note under the §9 heading says so, so "see §8" in a 2026-11-13 entry still resolves.
- **Docs only:** no section content changed except the numbers those references carry; no
  code and no other project file touched.

### 2026-11-25 — §1–§2 slimmed around `scripts/analytics.sh`

- **Why:** since D-G (2026-11-19) the wrapper is the daily path, so §2's manual walkthrough
  described the ritual the script exists to remove, and §1.4's 5-mode table + §1.5's 19-flag
  table restated §2 and `ros2 run llm llm_analytics --help`. Two references to learn from
  (`scripts/analytics.sh help`, §2's table) is enough; a third copy only drifts.
- **§1:** deleted §1.4 (five modes) and §1.5 (options), plus the CWD-relative defaults note
  under the §1.2 diagram (the wrapper always passes `--out`/`--events` into its own folder).
  Old §1.6 is now **§1.4 Artifacts and exit codes**; §1.1–§1.3 unchanged. The mode/flag facts
  survive in §2's command table, the wrapper's `help`, and `--help` (§9).
- **§2:** collapsed to one `scripts/analytics.sh` table — server lifecycle (`up`/`status`/
  `down`/`probe`), the five measurement modes (`one`/`sweep`/`node`/`live`/`idle`) and reading
  (`replay`/`runs`/`last`) — plus a single subsection, **§2.1 What to read & conceptual
  pitfalls**, keeping only the five traps the wrapper cannot fix: no node and no retries under
  `--exec-file`; a `node` dry run still writes `llm_generated/`; two runs need matching prompt
  `sha256` **and** `request.body_sha256` (`--no-go-to` swaps the prompt file); a non-zero sweep
  exit means at least one exchange failed (`session.warnings`); `/metrics` 501 is expected
  (NEXT-06). Gone: §2.1 setup, §2.2 manual server startup, the Path A/B/C bash walkthroughs
  and every `mkdir -p "$RUN"` block.
- **Also:** §4's stale "the next row that changes this table's ✗ columns is NEXT-02" replaced
  (NEXT-02 is `Done`; the ✗ left are the live `--exec` row and `/metrics`). Footer date bumped
  to 2026-11-25. §0, §3–§6, §8, §9 untouched; the older §7 references to §2.1–§2.4 are
  historical and were **not** rewritten (rule 4) — §2.1 carries a note saying so.

### 2026-11-19 — `scripts/analytics.sh`: the ritual becomes one command

- **The gap:** §2 was correct and unusable at 1 a.m. Four things had to be true before a
  measurement (`cd`, `source scripts/env/jazzy.sh`, `mkdir -p $RUN`, a unique
  `YYYY-MM-DD-<task-name>`) and §2.4 lists what each omission costs — `--events` into a
  missing folder aborts at startup, `--out`/`--events` default to the current directory and
  truncate on purpose, a rerun into the same names destroys the previous capture, and no
  server means forty 502s instead of an error. All memory work, and memory is what fails.
- **Decided (D-G):** a bash wrapper at `scripts/analytics.sh`, **no change to
  `llm_analytics.py`**. Modes: `up/status/down/probe` for the server, `one/sweep/node/live/idle`
  for the five documented paths, `replay/runs/last` for reading, `help` for the cheat sheet.
  Options: `--label --no-airlock --no-go-to --port --timeout --wait --force
  --simple-discovery --verbose`. Bash + `set -euo pipefail`, in the `colcon_build.sh` house
  style; nothing to build (`install/` is a symlink install).
- **What it enforces:** sources `scripts/env/jazzy.sh` in-process (with `set +u` around the
  ROS setup — `setup.bash` reads `$AMENT_TRACE_SETUP_FILES` unguarded), `mkdir -p` **before**
  the tool starts, always passes `--out`/`--events` into that folder, `/health` preflight →
  **exit 3** with the fix printed, and prints the equivalent `ros2 run llm llm_analytics ...`
  line before running it so the long form stays learnable.
- **Label rule:** `--label` is a name. Slugged, `YYYY-MM-DD-` prepended unless already dated,
  then the first free `-2`, `-3` … (`new_run_dir`). `replay` takes a bare label (newest
  match wins, and says which), an exact folder, a path, or `--last`.
- **Server:** `up` autodetects the binary (`$LLAMA_SERVER_BIN` → `stack_params.yaml` *if it
  exists here* → `~/projects/llm/llama.cpp/build/bin/llama-server`) and cwd (three levels up,
  never `dirname(binary)`), passes both launch args, `setsid`s the launcher, polls `/health`
  for 120 s, logs to `analytics_runs/_server/llama-server.log` (previous log rotated to
  `.prev`, pid file beside it). `down` signals the launcher it started (SIGINT → SIGTERM →
  `--force` SIGKILL), leaves a foreign server alone with a hint.
- **Verified live on `alex-desktop`** (server up/down through the wrapper, not by hand):
  `up` 45 s to `/health`; `status` (pid, 3547 MiB rss, cmdline); `probe` (`/metrics` 501 as
  expected); `one "vai dritto e gira a sinistra alla parete"` → `2026-11-19-test/`
  (51.0 s, 2635-token cold prefill, airlock rejected `{'mode': 'straight'}` → the
  `IntentSchemaError` of BUG-01, JSON valid); same label again → `2026-11-19-test-2/`;
  `replay` from label and from bare label; `last` prints the verdict table; a one-line
  `sweep` → `2026-11-19-sweep-smoke/` with `commands.txt` byte-identical to its input;
  `down` twice (second says nothing was running). `runs` sorts by mtime, so legacy
  `NEXT-02-first-capture/` no longer pretends to be newest.
- **Not exercised live:** `node`/`live` (need the full stack for `/test/plan_result` and the
  mission services; the `live` confirmation prompt was checked — a non-`yes` answer aborts
  before anything runs) and `idle`. `probe` with the server down exits **3**, by design.
- Unchanged: **83 passed** for `test/test_llm_analytics.py` from a sourced workspace (81/83
  unsourced — drift 8, not a regression). `scripts/analytics.sh` is the only new tracked file;
  the validation captures stay in the untracked `analytics_runs/` (D-B).

### 2026-11-18 (session 2) — the capture keeps its own input (`commands.txt`)

- **The gap:** `analytics_runs/` is untracked (D-B) and every artifact in it is truncated
  on purpose, so a folder holding `report.{json,md}` and `events.jsonl` but not the commands
  can be replayed forever and *re-run* never. §2.3 told the operator to keep `commands.txt`
  beside its capture, while the line below it said the repo sweep is "never copied into the
  run folder": the two rules contradicted each other, and neither was enforced by anything.
- **Decided (D-F):** the tool does it, with no flag. `llm_analytics` writes `commands.txt`
  next to `--events` **before the first command goes out** — `--exec-file` copied byte for
  byte (comments, blank lines and line endings included, so the file can be handed straight
  back to the same flag), `--exec` written one command per line, which is the format
  `load_command_file()` reads back. A failed copy is a warning, never an abort: the
  measurement is the point. Skipped with `--no-events` (no folder) or an idle recorder (no
  input).
- **Recorded, not just written:** the `session` block gains `commands_snapshot`,
  `commands_source` (`exec-file`/`exec`), `commands_count`, `commands_sha256`, and
  `report.md` prints them as one line beside the exec-file path and the prompt sha — the
  keys that make two runs comparable now name the **input** as well as the prompt.
- **The existing capture was retrofitted:** `2026-11-18-20_first_attempt/commands.txt` is a
  copy of `test/commands.sample.txt` whose 20 command lines were checked against every
  `input.command` in `events.jsonl` and match one for one (its comment header was reworded
  after the sweep, so the comments are the current text and the commands are the run's —
  stated in `NOTES.md`, not glossed over).
- 7 new tests (`TestInputSnapshot`, one end-to-end through `main()` on an ephemeral port) →
  **83 passed**.

### 2026-11-18 — 20-command Italian sweep (`2026-11-18-20_first_attempt`)

- Second live capture, and the first big enough to be a benchmark: the operator's 20
  Italian commands in `test/commands.sample.txt`. The file had been pasted in **fully
  commented out** — `load_command_file` skips `#`, so it would have run 0 commands; the
  20 lines were enabled and the header rewritten as a benchmark sweep rather than a
  smoke test. `--exec-file --airlock`, capture in
  `analytics_runs/2026-11-18-20_first_attempt/` (untracked, D-B) with `report.{json,md}`,
  `events.jsonl`, a `--replay` twin, and a **`NOTES.md`** carrying the per-command human
  verdict — the one column no report field can produce.
- Numbers: 20 exchanges in **139 s**, 0 failures, 0 retries, **20/20 JSON-valid**,
  airlock **12 accepted / 8 rejected** (IntentSchemaError 5, EmptyPlanError 3). Latency
  median 4.28 s / p95 7.96 s; the **52.9 s max is the first request only** (2644-token
  cold prefill at 54.6 t/s = 48.4 s), then 14–200 new tokens per command against 2658
  cached ones (53337 cached / 52767 evaluated over the run) → 0.7–1.7 s prefill. Decode
  8.2 tok/s steady, budget used 5 % median / 10 % max, `stop_type: eos` everywhere:
  nothing was truncated. `--replay` reproduces the summary offline with no server.
- **Host finding:** this `llama-server` build prints `no usable GPU found … compiled
  without GPU support`, so `-ngl 999` (`config/models.yaml`) is a **no-op on this
  machine** and 8.2 tok/s is 4-thread CPU. `stack_params.yaml`'s `llama_server_path`
  default (`/home/llm/...`) does not exist on `alex-desktop`: the launch needs both
  `llama_server_path:=` and `llama_server_cwd:=` overrides (recorded in `NOTES.md`).
- **The headline is not the rejection count.** Judged against the prompt's own rules,
  **9/20 produced the intended plan**; 3 of the 12 *accepted* plans were semantically
  wrong — command 6 emitted `distance 2.0` where the operator said "**if** you find an
  obstacle stop at 2 m", command 11 mapped `divano` to `dining table`, command 17
  mapped `automobile blu` to `car` and invented a `thresh 1.0` the operator never said.
  The airlock validates schema and range; it cannot see intent. That is the argument for
  an exact-plan layer (`test/golden/` + `test_intent_evals_live.py`) on top of it, and
  "accepted but wrong" is now the more expensive failure class.
- **BUG-02 filed (§8):** a `front_object`/`distance` phase without `thresh` is fatal,
  while the prompt explicitly promises `front_object` a default of 1.0 and only `wall`
  has a defaulter. Commands 5 and 8 both died on it. **BUG-01 recurred once** (command
  20: `{"mode":"straight"}` plus two invented values) — a third data point that the
  trailing-clause grammar is the trigger, not the particular command set.
- `go_to` class mapping measured **3/5**: tavolo, bicicletta, automobile right; divano
  wrong class, zaino falsely refused although `backpack` is in the enum → **NEXT-08**
  prompt work, not a tool defect. Command 13 (`20 cm`) exercised the clamp path: 0.2 is
  the schema floor and the translator reports both the spoken and the clamped number.

### 2026-11-13 — Markdown twin for `--out` + BUG-01 filed

- `llm_analytics` now writes `<stem>.md` beside every `--out` JSON, in both live and
  `--replay` mode: a metadata block (replay source, upstream/proxy, airlock + go_to,
  prompt sha256, warnings, the console summary verbatim) and a 7-column per-exchange
  table (`Command Prompt | Temp | JSON Valid | Airlock Verdict | Latency | Tokens |
  Tok/s`). One JSON Schema file, one human one — the JSON stays the machine contract.
  `write_report()` untouched, stdlib only, 5 new tests → **76 passed**. NEXT-02 was
  replayed into `analytics_runs/NEXT-02-first-capture/replayed.{json,md}`; the live
  `report.json` / `events.jsonl` were **not** rewritten (one run, one directory);
  `analytics_runs/` stays untracked (D-B), conclusions only.
- The replayed table is what exposed the actual cause of NEXT-02's 3 rejections, so
  **§8 is new**: BUG-01 is the **missing `guard`**, not the missing `thresh`. Two of five
  raw outputs carry a bare `{"mode":"straight"}`, and `_default_wall_thresh()` only fills
  a thresh when `guard == "wall"` is *already there* — so nothing rescues it and the
  schema answers with the generic `oneOf` sentence. One of those two rows also invents a
  third phase, moving the wall condition onto a move the operator never asked for.
  Proposed A′ (explicit Python rejection before `jsonschema.validate`) + B (prompt rule +
  negative example); **deliberately not implemented**, so the 2/5 first-attempt rate
  stays measurable and the fix can be shown against a baseline.
- §8 Pointers became **§9** to keep numbering monotonic (a second §8 would have been a
  defect of its own); the single cross-reference (NEXT-09) was updated.

### 2026-11-12 (session 3) — NEXT-02 closed: **first live capture**

- The blocker is dead: `llama-server` runs on `alex-desktop` (pid 95960, `:8083`) with
  `qwen2.5-3b-instruct-q5_k_m.gguf` (2.4 GB, `/home/alex/projects/llm/models/`). `/health`,
  `/props`, `/slots` → 200 (4 slots, n_ctx 32768); **`/metrics` → 501** on this build, so the
  poller disabled itself and logged a warning instead of inventing KV numbers → NEXT-06 is
  now confirmed live, not hypothetical.
- **Cold vs warm prefill is the whole latency story.** First generation of the session:
  **59.1 s wall**, of which prefill **54.7 s** (2635 tokens @ 48.1 t/s, 93 %) and decode 4.3 s
  (29 tokens, 6.5 tok/s). Every later call re-used that prompt cache: prefill fell to
  **~0.19–1.2 s** and the 5-command sweep averaged **4718 ms** wall (p95 7089 ms). Prompt
  sha256 `84ab53aa1ea8` (`planner_system_prompt.v2.it.txt`) is the comparability key — the
  cache only hits because the bytes were identical.
- Path A over `test/commands.sample.txt`: **5/5 JSON-valid, 0 failures, 0 retries**, 6.8 tok/s,
  proxy `overhead_ms` **7.4** (the hop is free), budget 5–8 % of 512, `stop=eos` every time,
  `trailing_chars` 0 (one answer was pretty-printed across 3 lines and still parsed).
  13177 tokens evaluated vs 13307 cached; RSS 3660 → 3661 MiB; `kv` unknown (no slot reports
  `n_past`).
- **The finding: the airlock rejected 3 of 5 — on a *model* deviation, not a prompt defect.**
  Commands 1 and 3 emitted `{"mode":"straight"}` with **no `guard`/`thresh`**, which
  `intent_v1.json` marks required; the prompt teaches both and every worked example carries
  them. `--exec-file` cannot retry by design — `--exec` would feed the validator error back
  (`MAX_INTENT_RETRIES` = 2). Command 5 `vai dalla porta` returned `plan: []` with
  `unsupported: ['vai dalla porta']` because "porta" is not in the `go_to` target enum
  (data for NEXT-08). Commands 2 and 4 translated cleanly (`straight/wall` + `turn/right` +
  `straight/distance 2.0`; `straight/front_object 1.0`).
- Determinism noted: the rejected command produced byte-identical output cold and warm —
  at `temperature 0.0` a re-run buys nothing, only a repaired prompt does.
- Artifacts persisted untracked, per D-B: `llm/analytics_runs/NEXT-02-first-capture/`
  (`report.json` 76 kB, `events.jsonl` 66 kB) with a package `.gitignore` entry, so the logs
  survive `/tmp` reaping and still replay with `--replay events.jsonl`.
- Drift 7 half retired (see §4): `--exec-file` is `verified-live`; `--exec` is still not.
  `memory.md` drift 7 carries the correction inline.

### 2026-11-12 (session 2) — §2 restructured; NEXT-01 committed

- §2 read as a mandatory sequence (Step 0 → 7) when it is really *two prerequisites and one
  choice*. Rewritten as four blocks: **2.1 Mandatory setup** (every new terminal), **2.2
  Server startup** (once per session), **2.3 Execution paths** (A `--exec-file`, B `--exec`,
  C idle recorder — stated as alternatives with a pick-one table), **2.4 Results & pitfalls**
  (read order + the trap table). Every command and trap survives; none is new.
- One stale pointer repaired rather than left dangling: the previous entry's "§2 Step 7" now
  means **§2.4**. No log text was deleted.
- NEXT-01: baseline committed as **`60c2ce46`** — the `--exec-file` pass (code, 9 tests,
  README runbook, `memory.md`), `test/commands.sample.txt`, `planner.md`, `memory-bank/`,
  `changelog.md`, `.clinerules`. Files outside the `llm` package that were already modified in
  the tree (root `README.md`, `output/`, `f1tenth_sim/`, `f1tenth_perception/`) were left
  untouched and unstaged — they belong to other work.

### 2026-11-12 — planner.md created
- Scope agreed: the whole `llm` package, **anchored on `llm_analytics`** — the instrument,
  not the AI. §3 keeps the measured pipeline to one screen and points at the package docs.
- Correction recorded for the record: `llm_analytics` runs no model and authors no plan;
  `llama-server` + `llm_planner_node` do the reasoning. Calling it "the AI" was wrong.
- Evidence axis introduced (`built` / `unit-tested` / `verified-live`) because the package
  is fully built and fully unit-tested while **no live generation has ever been recorded on
  this machine**.
- Traps catalogued in §2.4: CWD-relative outputs; `--exec-file` seeing no retries /
  `/mission/*`; `--exec-file` being v2-only; dry run still writing a mission JSON; timeout
  ordering 90 < 120 < 180; prompt sha256 as the only valid comparability key.
- `NEXT-03` deliberately kept as its own row (user's call) rather than folded into NEXT-02.
- This file is exempt from the 800-word cap and from `progress.md` pruning, by instruction:
  it grows forever.

*Last updated 2026-11-25 — maintained by hand, never pruned.*
