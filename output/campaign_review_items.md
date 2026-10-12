# Part A review feedback and decisions (final state, 2026-10-12)

Branch `test-campaign`, as pushed. Everything was verified on linus only: unit tests, nominal-model rig, and the scenario harness on the real `mpc_corr`.

| Item | Commit | Result |
|---|---|---|
| 1, 2 | `959b40d` | Keep-out rows on 3 footprint circles; committed pass side in `pass_side.py`, published on `/mpc/pass_sides` (single source for B4) |
| 3 | `0b9e8ee` | `tools/prompt_regression.py`; prompt sha256 in every run's metadata. **Not run: no LLM on linus.** Run it on the Orin |
| 4 | `04facd9` | `MissionState.BLOCKED`, `mission_blocked` event, campaign outcome `blocked`. No-progress: < 0.15 m in 2.5 s at ≥ 0.4 m/s |
| 5 | `ce36ebb` | Recorder armed in advance, resumed at mission LOAD; one Recorder per process; all node params per run; `/mpc/config`; 10 s pre-roll |
| 6 | `ba58788` | `campaign_scenario` harness driving the real `mpc_corr` (`tools/run_scenarios.sh`) |
| 7 | `8eb4c29`, `472f036` | Stale, flaky and environment-dependent tests fixed; `mpc_model_check.py --demo` header bug fixed |
| Decisions | `58f9864` | `mpc_w_du_delta` 8.5714; `corridor_heading_return` false with the A/B reason; `go_to_person.json` and `bottle_then_person.json` restored; test: no `llm_` mission in `missions/` |
| Harness finding | `20544fa` | No BLOCKED cascade after a BLOCKED: cold warm start ramps steering to 0; published steering and `last_u` clipped to the bounds; stop ticks carry no acceleration |
| A1 dt review | `426eabf` | Per-step rate weights scaled by (0.1/step)²: `w_du_delta` effective 0.75 between 0.2 s steps, 3.0 on the 0.1 s first step. SLSQP first rate constraint now spans the control period. The other per-step terms needed no change (see the commit) |
| Orin crash | `babc885` | wall_turn with a stale or missing front distance: `float(None)` → NaN / "unknown", with a test |
| A4 wall stop at 0.5 m | see `git log` | The ±30° sector shows 2·d·tan 30° of a square-on wall: 0.57 m at the 0.5 m stop (LiDAR 0.49 m from the wall, laser at base_link x +0.12). The fixed 0.8 m lost the wall below a 0.71 m camera distance, so the stop never fired. Now a latch: the first confirmation still needs 0.8 m; frames whose line continues it need 0.7 × the visible length (0.40 m at the stop). A person or plant is not a continuation and still needs 0.8 m |

**Lookahead.** `corr_lookahead_frac` stays at 0.85. In the harness, 0.85 vs 0.5 gave the same outcomes except two cases, comparable clearances, passes about 0.5 s faster, and none of the 34 geometry warnings per run that 0.5 produces. Details are in `campaign_partA_review.md`.

## Final harness: min gap from the car outline to the true obstacle [m]

Each cell reads true-centre reporting / front-surface reporting. "BLOCKED" alone means both modes ended BLOCKED.

| Obstacle width | Centred, seen early | Centred, appears at 1.4 m | 0.3 m off-centre, early | 0.3 m off-centre, at 1.4 m |
|---|---|---|---|---|
| 0.1 m | passes +0.13 / passes +0.16 | passes +0.19 / passes +0.26 | passes +0.15 / passes +0.17 | passes +0.15 / passes +0.18 |
| 0.3 m | passes +0.14 / passes +0.23 | passes +0.27 / BLOCKED | passes +0.14 / passes +0.24 | passes +0.15 / passes +0.31 |
| 0.5 m | passes +0.11 / passes +0.18 | BLOCKED | passes +0.12 / passes +0.31 | passes +0.23 / BLOCKED |
| 1.0 m | BLOCKED | BLOCKED | passes +0.09 / BLOCKED | BLOCKED |

Across all 32 runs there was no command in (0, 0.4) m/s and no pass-side flip.

Known and accepted: 1.0 m objects pass only with true-centre reporting. That is Part B (B2), see `campaign_partB_backlog.md`.

## Open

- Run `tools/prompt_regression.py` on the Orin.
- Confirm that the armed recorder starts at mission LOAD on the Orin's rosbag2 version.
- Part B: B1–B8 in `campaign_partB_backlog.md`.
