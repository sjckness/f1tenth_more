# Part A review feedback: items 1–7 (2026-10-09)

Commits `0b9e8ee..472f036` on `test-campaign`, local and not pushed. Everything was verified on linus only.

| # | Commit | Result |
|---|---|---|
| 1, 2 | `959b40d` | Keep-out rows on 3 footprint circles; committed pass side in `pass_side.py`, published on `/mpc/pass_sides` |
| 3 | `0b9e8ee` | `tools/prompt_regression.py`; prompt sha256 recorded per run. **Not run: no LLM on linus** |
| 4 | `04facd9` | `MissionState.BLOCKED`, `mission_blocked` event, campaign outcome `blocked` |
| 5 | `ce36ebb` | Recorder armed in advance and resumed at mission LOAD; all node params dumped per run; `/mpc/config`; 10 s pre-roll |
| 6 | `ba58788` | `campaign_scenario` harness driving the real `mpc_corr` |
| 7 | `8eb4c29`, `472f036` | Triage; stale, flaky and environment-dependent tests fixed |

## Clearance from the car outline: real node, harness, 32 runs

| Obstacle | Centred, seen early | Centred, appears at 1.4 m | 0.3 m off-centre, early | 0.3 m off-centre, at 1.4 m |
|---|---|---|---|---|
| 0.1 m | passes, +0.18/+0.21 | passes, +0.28/+0.31 | passes, +0.17/+0.19 | passes, +0.16/+0.19 |
| 0.3 m | passes, +0.19/+0.27 | **BLOCKED** (stops 0.64–0.70 m away) | passes, +0.17/+0.30 | passes, +0.20/+0.32 |
| 0.5 m | passes, +0.21/+0.37 | BLOCKED | passes, +0.18/+0.35 | passes (true centre) / BLOCKED (front surface) |
| 1.0 m | BLOCKED | BLOCKED | passes (true centre) / BLOCKED (front surface) | BLOCKED |

- Gaps are in metres, given as true-centre / front-surface.
- No command fell in (0, 0.4) m/s and no pass side flipped, in any run.

## Left for your decision

- `mpc_w_du_delta`: 8.33 or 8.5714.
- `corridor_heading_return`: true or false.
- `missions/go_to_person.json` was overwritten by LLM output in `1d00853`. Restore it, or keep it and update the 3 tests that check it.
