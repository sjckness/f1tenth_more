# Progress Log

- **2026-11-25 (session 4) — second model registered, 2-model `compare` verified live:**
  `llama32_3b_instruct` added to `config/models.yaml` (`:8083`, `-c 16384` to stop the OOM its
  GGUF's 131072-token default caused) and one command,
  `scripts/analytics.sh compare qwen25_3b_instruct llama32_3b_instruct --label smoke-two-model`,
  ran clean end to end — both arms boot/answer/shutdown, 2/2 identical request bodies, 2/2
  JSON-valid each, median 36519 vs 38948 ms, 6.1 vs 4.5 tok/s; capture
  `sequential/2026-11-25-smoke-two-model-3/` (untracked, D-B). `planner.md` §2/§2.1/§4/§9 and
  `llm/README.md` + `llm/memory.md` updated; `llm_analytics.py` untouched (**83/83**),
  `scripts/test_compare_reports.py` 8/8.

- **2026-11-25 (session 3) — two-model comparison (`up --model` + `compare A B`):**
  `scripts/analytics.sh` can now launch a named `config/models.yaml` entry (`model:=`, the
  entry's port adopted, live name in `_server/current_model`, shown by `status`) and run two
  of them strictly back to back in `analytics_runs/sequential/<date>-A-vs-B/`, one shared
  `commands.txt`, one `llama-server.log` per model, refusing (exit 3) to kill a server it did
  not start. New `scripts/compare_reports.py` (stdlib reader, explicit flags) writes the single
  `comparison.md`: metadata + command-file/prompt/`body_sha256` checks with positions, aggregate
  table (completions, JSON valid, airlock accept/reject/by_error, median/mean wall ms, tok/s,
  peak RSS) and one row per command with `≠ airlock` / `≠ json` / `missing`; stripped reports
  degrade to the exchanges. `runs`/`last` skip `sequential/` unless `--sequential`. `llm_analytics.py`
  untouched (**83/83** still pass); 8 new tests in `scripts/test_compare_reports.py` plus a
  fixture render from `2026-11-18-20_first_attempt` (mutated copy) exercised the diff paths.
  Live 2-model run still needs a second `models.yaml` entry.

- **2026-11-25 — `planner.md` §2.1 metric cheat sheet:** one 13-row table now decodes
  every line the `llm_analytics` summary and `report.md` header print (labels checked against
  `format_summary()`/`_server_lines()`), including `warmup 0`/`retries 0` in `sweep`/`one`,
  the two airlock error classes, median-vs-cold latency, prompt-cache tokens, and the RSS/KV/
  `/metrics` limits that stay until NEXT-06. Docs only; no source touched.

- **2026-11-25 — `planner.md` rebuilt around `scripts/analytics.sh`, Log moved last:** §1–§2 were
  trimmed and deduplicated (one wrapper command table + §2.1's five traps the script cannot fix),
  then the file was renumbered so it reads §0–§9 with the **Log as §9** (bugs §7, pointers §8),
  13 live cross-references updated plus a note mapping the old numbers for older entries, and every
  historical body verified byte-identical. Docs only — `planner.md` plus memory-bank bookkeeping.

- **2026-11-25 — `01-index.md` reconciled with the tree (15 discrepancies):** the router now points
  where things actually are — legacy Foxy container scripts at `src/scripts/` (not `scripts/`), four
  uninitialised submodules plus a dead `transport_drivers` gitlink called out, six packages' code dirs
  and `scripts/colcon_build.sh` routed, the false `corridor_perception` "empty description" and the
  VESC-node-ownership claims fixed, mux corrected to 200/100/50/10; 124 paths machine-verified,
  0 unresolved. Docs only — no source touched.

- **2026-11-19 — `scripts/analytics.sh`, the `llm_analytics` front door (D-G):** one command
  per intent replaces the four-step ritual — `up/status/down/probe` own the server (overrides
  autodetected, `stack_params.yaml`'s path is the Jetson's), `one/sweep/node/live/idle` the five
  documented paths, `replay/runs/last` the reading. It sources the env in-process, makes and
  names the dated run folder *before* the tool starts, always passes `--out`/`--events`,
  preflights `/health` (exit 3) and prints the equivalent `ros2 run` line. `llm_analytics.py`
  untouched (**83/83** tests from a sourced shell); validated live end to end
  (`up → status → probe → one → -2 → replay → last → sweep → down`) with 4 real defects fixed on
  the way; `node`/`live`/`idle` need the full stack and stay unexercised.

- **2026-11-18 (session 2) — every capture now carries its own input (`commands.txt`, D-F):**
  `llm_analytics` copies the run's commands beside `--events` before the first one goes out
  (`--exec-file` byte for byte, `--exec` one per line), names it in `report.md` (`- input:`,
  count + sha256) and the console, and treats a failed copy as a warning; 7 new tests →
  **83 passed**, `2026-11-18-20_first_attempt` retrofitted and verified against its
  `events.jsonl`, and the old "never copied into the run folder" rule purged from the docs.

- **2026-11-18 — 20-command Italian sweep (`2026-11-18-20_first_attempt`):** ran
  `llm_analytics --exec-file --airlock` over the operator's 20 commands (the file had been
  pasted in fully commented out — `load_command_file` skips `#`, so it would have run 0
  commands; enabled in place). 139 s, 0 failures / 0 retries, **20/20 JSON-valid**, airlock
  **12 accepted / 8 rejected**; latency median 4.28 s, p95 7.96 s, the 52.9 s max is the
  cold first request only; decode 8.2 tok/s (this build has **no GPU support**, so
  `-ngl 999` is a no-op and both `llama_server_path`/`_cwd` needed overriding —
  `/home/llm/...` does not exist on this host). Judged by hand against the prompt's own
  rules: **9/20 plans were right**, 3 of the 12 accepts were **silently wrong** (wrong
  guard, wrong class, dropped attribute + invented `thresh`) — the airlock is a safety net,
  not an intent oracle. Filed **BUG-02** (`front_object` without `thresh`: the prompt
  documents a default of 1.0 that exists in no layer) and one **BUG-01** recurrence;
  `reverse` got its first live evidence (2/2); `go_to` class mapping 3/5. Capture kept
  untracked (D-B) with a `NOTES.md` verdict table; `planner.md` §4/§6/§7/§8 updated.

- **2026-11-13 — Markdown twin + BUG-01 filed (`2d72f0d7`):** `llm_analytics --out` now also
  writes `<stem>.md` (metadata block + 7-column per-exchange table) in live and `--replay`
  mode; 5 new tests → **76 passed**; NEXT-02 replayed without touching the live capture. The
  replay exposed **BUG-01**: the schema rejections are a missing **`guard`**, not a missing
  `thresh` (one row even invents a third phase), documented in the new `planner.md` §8 with fix
  A′+B deliberately left unimplemented; old §8 Pointers → §9. Package `.gitignore` tracked for
  the first time; `analytics_runs/` stays untracked (D-B).

- **2026-11-12 — NEXT-02 closed (first live capture) + analytics runs made append-only:**
  `llm_analytics` ran against a real `llama-server` for the first time (Qwen2.5-3B q5_k_m,
  `:8083`) — 5/5 JSON-valid, 0 failures, 6.8 tok/s, proxy overhead 7.4 ms, cold prefill
  54.7 s vs ~1.2 s warm, airlock 2 accepted / 3 rejected (model dropped required
  `guard`/`thresh`; one unknown `go_to` target). Capture persisted **untracked** in
  `llm/analytics_runs/NEXT-02-first-capture/` (D-B decided), `planner.md` §4 → `verified-live`,
  §5 → Done, §7 new entry, `llm/memory.md` drift 7 corrected. New **`# ANALYTICS RUN PROTOCOL`**
  in `.clinerules` plus `planner.md` §2.3/§2.4 make every trial write into its own dated
  folder — motivated by `--events`/`--out` truncating by design. Next: NEXT-03.

**Open items:** populate `03-decision-log.md` once a real design decision is made.
