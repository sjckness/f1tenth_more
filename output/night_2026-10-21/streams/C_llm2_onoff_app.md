### Stream C, app addendum: LLM2 on/off from lamacchinina (two-llm-arch: 5f5ff56 gateway, c9a0859 PWA)

LLM2 (the MPC-parameter tuner) can now be paused and resumed from the phone. The app shows the state LLM2 reports, not the last tap.

**Stage 1: gateway (5f5ff56)**
A previous agent had been cut off and left uncommitted, untested work. I checked it against `output/two_llm/README.md` §2 and `llm2_mpc_param_node.py`, kept it, added fixes and tests, and committed it.

- **`llm2.py` (`Llm2State`, no rclpy):**
  - It merges `/llm2/status` and the `/llm2/set_active` answers (`view()` :82).
  - `not_running` means one of two things:
    - no status for `llm2.stale_s` (5 s, monotonic clock) and no answer either;
    - the service is not ready.
  - For `llm2.trust_s` (2 s, one period) after an answer, the answer wins over any status received in that time (the README rule for the two channels).
- **`ros_bridge.py`:** the `SetBool` client is created at start-up (:132) and `/llm2/status` is always subscribed (:134). `llm2_set_active` (:230) uses the `abort()` pattern: `call_async`, a timed poll from a pool thread, then cancel on timeout.
- **`app.py`:**
  - `status.llm2` is part of the existing 1 Hz status (:266).
  - `POST /api/llm2 {"active": bool}` (:828) uses the same auth as the other commands, runs in the thread pool and allows one call at a time (409 for a second one).
  - After every call, failures included, it pushes a status to every phone without awaiting it.
  - Neither the event loop, the E-STOP nor teleop ever waits on LLM2.
- **Error codes:** 401 no token, 409 not running or another call in flight, 422 bad body, 502 timeout (2 s) or unexpected answer.
- **Mock (`mock.py`):** LLM2 sends a status every 2 s with the node's keys. `/api/_mock/set` has four knobs: `llm2_running`, `llm2_active`, `llm2_fail`, `llm2_delay_s`.
- **Also:** config keys in `config.py:106-113`, `std_msgs` added to `package.xml`, and the README updated.

**Stage 2: PWA (c9a0859)**
- **`apps/pwa/src/screens/Cockpit.tsx`:** `Llm2Row` (:92) is placed below PROCESSI (:206). It is a "Parametri MPC" row using the app's `Switch` (`Attiva LLM2`, :131).
  - The state comes from `status.llm2.state`. `attivo` is green with the switch on; `in pausa` is grey with the switch off.
  - Next to the state: `latenza N ms`. Below it: `ok · invalid · timeout`. Only the state word is coloured; the latency and counters stay grey.
- **`setLlm2` (:79):** no confirmation and no retry.
  - While a call is in flight the switch is busy and stays at the real position. The text reads `pausa in corso…` or `ripresa in corso…` (amber) for this phone's call, or `comando in corso…` when `status.llm2.pending` comes from another phone.
  - On success the answer's `llm2` view is applied at once.
  - On failure a red toast shows `Pausa/Ripresa LLM2 fallita: <gateway detail>`, and the switch keeps the real state.
- **Not running:** with no status for 5 s or no service, the row turns grey and says **"LLM2 non attivo"** with the gateway's `why`, and the switch is disabled.
- **Older gateway:** `llm2Of` (:63) treats a missing `status.llm2` (campaign-final gateway) as not running, so the app does not crash. Offline, the row is greyed like the processes.
- **Other files:**
  - `store.ts:37`: `Llm2View`, plus an optional `Status.llm2`.
  - `styles.css`: `.llm2-sub`, at 10 px under 360 px wide.
  - `docs/pwa/SPEC.md:63` (lamacchinina section) and :145 (API table).

**How it was verified**
- **Gateway:** 110 passed on linus (baseline 105) and again at HEAD after stage 2. 110 passed on Humble / Python 3.10 (`tools/test_gateway_humble.sh`), including RosBridge against a fake `SetBool` service.
- **PWA build and e2e:**
  - `npm ci` then `npm run build` (tsc + vite) passes.
  - The e2e **webkit-portrait project: 17 passed**.
  - No new UI tests, as the spec asks.
- **WebKit checks (21/21 OK):** run against my own mock gateway on 127.0.0.1:8792, which I killed by PID afterwards; screenshots are in `.../scratchpad/stream_C_app_addendum2/shots/`. Covered:
  - at 390 px: active; pending (switch disabled, still on); paused, still paused after the next statuses;
  - a 2 s timeout: error toast, switch unchanged and enabled;
  - an external resume picked up from the status;
  - not running;
  - `status.llm2` stripped from WS and REST: no page error;
  - at 320 px: active and not running;
  - no horizontal overflow at either width.

**Not verified**
- Not run on the car or against the real node.
- `llm2.trust_s` assumes `rate_hz` 0.5.

**Car checklist**
- On two-llm-arch with `enable_llm2`, pause LLM2 from lamacchinina. Check that `/llm2/status` says `paused` and that nothing new arrives on `/llm2/mpc_params` (`ros2 topic hz /llm2/mpc_params`). Then resume.
- Also check that the row goes `attivo` → `in pausa` → `attivo`, with the `paused` counter rising in `/llm2/status` while `ok` stays flat.
- Check that with `enable_llm2:=false` the row is grey with "LLM2 non attivo" and the switch is disabled.

**Redeploy**
- Pull two-llm-arch on the Orin and restart the `f1tenth_gateway` systemd user unit. It runs from source, so there is no build.
- Run `npm run deploy` in `apps/pwa`.
**Review and fixes (b89a3bd)**

The workflow's review stage was killed at 09:41, when the lead session exited. A later session ran it again at ~10:55, with the same two read-only lenses.
- **Spec compliance and test reality:** every spec bullet is done. A mutation check on a scratch copy of the gateway ran 26 mutants against the LLM2 tests. 25 were killed, among them removing the service call, the status field, auth, the 5 s staleness, the trust window and the 409/422 paths. The one that survived ignored the answer's `success` flag: no test answered `success=false`. This was a minor finding (the node always answers `success=true`).
- **Correctness and safety:**
  - No blocking path. Measured on a mock during a 1.9 s LLM2 call: `GET /api/status` took 5 ms, a concurrent `POST /api/llm2` got 409 in ~7 ms, and `POST /api/estop` took 0.302 s against the 0.302 s baseline.
  - The trust window, the late answer after a timeout (Humble rclpy) and the frontend with no `status.llm2` are all fine.
  - One minor finding: `Llm2State.on_status` caught only `ValueError`. A deeply nested JSON on `/llm2/status` raises `RecursionError` instead, which escapes the callback. `MultiThreadedExecutor` re-raises it, and `RosBridge._spin` swallows it silently. So one bad message stopped the gateway's whole ROS side: teleop, every subscription, and the E-STOP service answer. A probe confirmed this on Jazzy and on Humble. Triggering it needs a buggy or hostile publisher on the domain.
- **Fixed in `b89a3bd`:**
  - `on_status` now also drops a payload that raises `RecursionError`. The new assertion errors on the old code.
  - A test pins a `success=false` answer as an error.
- **Left as is:** `RosBridge._spin` still swallows executor exceptions without a log line. This predates the toggle and the comment says "shutdown races". A dead executor stays invisible.
- **Verified after the fix:** gateway 110 passed on Jazzy (venv) and 110 on Humble/Python 3.10 (`tools/test_gateway_humble.sh`; `f1tenth_ctl` 98 passed, 15 skipped). The PWA is unchanged since c9a0859; both reviewers ran `tsc --noEmit` and a vite build into scratch, and both passed. On the Orin, the Humble build of `b89a3bd` finished 41/41 packages with 0 Python 3.10 syntax errors.
