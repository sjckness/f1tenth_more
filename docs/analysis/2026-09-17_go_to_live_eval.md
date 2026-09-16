# go_to from the LLM planner: live evals (2026-09-17)

The go_to prompt (`prompts/planner_system_prompt.v2.it.txt`) and the prompt
without go_to (`planner_system_prompt.v2.no_go_to.it.txt`, which
`go_to_enabled: false` loads) against the live model. Produced by
`docs/analysis/go_to_live_eval.py`, which imports the cases and the retry loop
from `src/f1tenth_intelligence/llm/test/test_intent_evals_live.py`.

## Setup

- Model: qwen2.5-3b-instruct-q5_k_m.gguf. llama-server was started as
  llm.launch.py starts it (`-m … --port 8083 -ngl 999`, 4 slots, n_ctx 32768
  each), on the Jetson, with nothing else running. It was stopped afterwards
  (port 8083 free).
- Request: `_completion()` unchanged: raw prompt + `comando: "…"\nrisposta:`,
  temperature 0.0, n_predict 512.
- Retries: as `LLMPlannerNode._plan_v2` does them. The translator's rejection
  text is fed back, up to MAX_INTENT_RETRIES = 2 extra attempts. `retries` is
  attempts − 1.
- Pass rule: the accepted intent's plan equals the expected plan with an
  empty `unsupported`. For a refusal, the plan is empty and `unsupported` is
  non-empty.

## Prompt size (llama-tokenize, same gguf)

| prompt | tokens |
|---|---|
| v2.it.txt before (HEAD edd0e6b; now also v2.no_go_to.it.txt, byte-identical) | 1710 |
| v2.it.txt after | 2378 |

+668 tokens (+39 %). 199 of them are the CLASSI list, which the in-enum /
not-in-enum rule needs. No length budget is written down anywhere in the repo.
The binding limit is the slot context (32768), which prompt + command + retry
feedback + n_predict 512 stays far below.


### go_to_enabled = true (planner_system_prompt.v2.it.txt)

| # | command | intent (accepted) | retries | result | s |
|---|---|---|---|---|---|
| 1 | vai dalla persona | `{"plan":[{"mode":"go_to","target":"person"}],"unsupported":[]}` | 0 | pass | 17.3 |
| 2 | raggiungi la sedia e fermati a un metro | `{"plan":[{"mode":"go_to","target":"chair","thresh":1.0}],"unsupported":[]}` | 0 | pass | 15.6 |
| 3 | vai verso la bottiglia e poi gira a destra | `{"plan":[{"mode":"go_to","target":"bottle"},{"mode":"turn","dir":"right"}],"unsupported":[]}` | 0 | pass | 15.6 |
| 4 | fermati a dieci centimetri dalla persona | `{"plan":[{"mode":"go_to","target":"person","thresh":0.1}],"unsupported":[]}` | 0 | pass | 15.6 |
| 5 | vai dalla porta | `{"plan":[],"unsupported":["vai dalla porta"]}` | 0 | pass | 15.7 |
| 6 | vai da Marco | `{"plan":[],"unsupported":["vai da Marco"]}` | 0 | pass | 15.7 |
| 7 | fermati prima dell'ostacolo | `{"plan":[{"mode":"straight","guard":"front_object","thresh":1.0}],"unsupported":[]}` | 0 | pass | 15.6 |
| 8 | avvicinati al divano | `{"plan":[{"mode":"go_to","target":"couch"}],"unsupported":[]}` | 0 | pass | 15.6 |
| 9 | vai verso lo zaino e fermati a mezzo metro | `{"plan":[{"mode":"go_to","target":"backpack","thresh":0.5}],"unsupported":[]}` | 0 | pass | 15.7 |
| 10 | vai al tavolo | `{"plan":[{"mode":"go_to","target":"dining table"}],"unsupported":[]}` | 0 | pass | 15.7 |
| 11 | vai dalla persona a sinistra | `{"plan":[{"mode":"go_to","target":"person","thresh":0.1}],"unsupported":[]}` | 0 | FAIL: expected a refusal, got plan [{'mode': 'go_to', 'target': 'person', 'thresh': 0.1}] | 15.7 |
| 12 | raggiungi la seconda sedia | `{"plan":[{"mode":"go_to","target":"chair","num":2}],"unsupported":[]}` | 2 | FAIL: retries exhausted: intent does not validate: {'mode': 'go_to', 'target': 'chair', 'num': 2} is not valid under any of the given schemas | 47.0 |
| 13 | vai verso la finestra | `{"plan":[],"unsupported":["vai verso la finestra"]}` | 0 | pass | 15.7 |
| 14 | vai dalla mia collega | `{"plan":[],"unsupported":["vai dalla mia collega"]}` | 0 | pass | 3.7 |
| 15 | vai verso il portatile | `{"plan":[{"mode":"go_to","target":"laptop"}],"unsupported":[]}` | 0 | pass | 15.7 |
| 16 | gira a sinistra e poi vai dalla sedia | `{"plan":[{"mode":"turn","dir":"left"},{"mode":"go_to","target":"chair"}],"unsupported":[]}` | 0 | pass | 15.6 |
| 17 | vai dritto e fermati a due metri dal muro | `{"plan":[{"mode":"straight","guard":"wall","thresh":2.0}],"unsupported":[]}` | 0 | pass | 15.6 |
| 18 | vai dritto, al muro gira a destra e avanza 2 metri | `{"plan":[{"mode":"straight","guard":"wall","thresh":3.0},{"mode":"turn","dir":"right"},{"mode":"straight","guard":"distance","thresh":2.0}],"unsupported":[]}` | 0 | pass | 15.7 |
| 19 | vai avanti e fermati prima di quello che trovi | `{"plan":[{"mode":"straight","guard":"front_object","thresh":1.0}],"unsupported":[]}` | 0 | pass | 15.6 |
| 20 | vai dalla bottiglia e fermati a due metri | `{"plan":[{"mode":"go_to","target":"bottle","thresh":2.0}],"unsupported":[]}` | 0 | pass | 15.6 |

18/20 passed

### go_to_enabled = false (planner_system_prompt.v2.no_go_to.it.txt)

| # | command | intent (accepted) | retries | result | s |
|---|---|---|---|---|---|
| 1 | vai dalla persona | `{"plan":[],"unsupported":["vai dalla persona"]}` | 0 | pass | 16.0 |
| 2 | vai verso la bottiglia e fermati a 1 metro | `{"plan":[],"unsupported":["vai verso la bottiglia e fermati a 1 metro"]}` | 0 | pass | 15.9 |
| 3 | vai avanti e fermati prima dell'ostacolo | `{"plan":[{"mode":"straight","guard":"front_object","thresh":1.0}],"unsupported":[]}` | 0 | pass | 4.6 |

3/3 passed

## Failures

- **#11 "vai dalla persona a sinistra": a selection planned as go_to.** The
  car would go to the NEAREST person, not the left one. The invented
  `thresh 0.1` would be clamped to 0.42 m with a note. This is the failure
  that matters: nothing downstream of the model can tell that the command
  selected an instance.
- **#12 "raggiungi la seconda sedia".** The model added a `num` field on all
  three attempts, the schema rejected it each time, and the node's exhaustion
  path fires: RICHIESTA NON SUPPORTATA, no mission. It was refused for the
  wrong reason, but no mission is emitted.

**The two runs did not agree on #12, despite temperature 0.** Straight
afterwards, `pytest -m llm test/test_intent_evals_live.py` on the same server
and prompt gave 22 passed, 2 failed (the same two commands). In that run #12
was accepted on the first attempt as
`{"plan":[{"mode":"go_to","target":"chair","thresh":0.0}],"unsupported":[]}`,
a silent go_to toward the nearest chair (the gap clamped to 0.42 m).
The cause of the difference was not investigated; llama-server's 4 slots
and its prompt cache are candidates. Treat any single run as a sample.

Selection was not solved by any prompt variant tried (below). Every variant
either planned it as go_to to the nearest instance, sometimes with an
invented thresh, or turned it into a schema error.

## Latency (observation)

Almost every command took about 15.6 s, including correct one-phase answers.
The model keeps generating (repeating the JSON) until n_predict 512;
`_completion` keeps the first JSON value. The prompt without go_to shows the
same 16 s, so this predates this change.

## How the shipped prompt was chosen

Full 20-case runs, same cases, temperature 0:

| variant | change | pass | failures |
|---|---|---|---|
| A | first rewrite | 16/20 | persona ×2 refused, "vai dalla bottiglia e fermati a due metri" refused, seconda sedia → go_to chair |
| B | "persona e' una classe" line, selection bullet reworded, + "sedia rossa" refusal example | 16/20 | persona ×2 refused, portatile refused, seconda sedia → partial plan + unsupported |
| C | B, examples reordered, go_to examples last | 16/20 | persona refused, a sinistra → go_to, seconda sedia → go_to, portatile refused |
| DE | examples in `comando:`/`risposta:` framing, go_to and refusal examples interleaved | 16/20 | "fermati prima dell'ostacolo" → go_to "front_object" (exhausted), a sinistra → go_to, seconda sedia → go_to, portatile refused |
| DG | DE + bare "fermati prima dell'ostacolo" example beside the clamped-person example | 17/20 | a sinistra → go_to, seconda sedia → go_to, portatile refused |
| **shipped** | DG with a duplicated class-mapping block removed | **18/20** | table above |

Order sensitivity is high. In a 6-command ablation on B, moving only the
"vai dalla persona" example to the end fixed both person commands and broke
the selection one. In another, the go_to examples last (DC) made
"vai da Marco" go to the nearest person.

## bottle_then_person.json in git history (the working-tree file untouched)

- First version, 37be92e (2026-08-10): one move, `search_person`:
  goal_distance 7.0, vdes 0.8, stop on `object_seen` class person, timeout 60,
  abort. No bottle, and nothing is approached. It drives until a person is
  seen.
- 96c6bbc: goal_distance 6.0, vdes 0.5, stop on distance_reached 6.0.
- 72c2e03: a 5 s settle move, then 5.0 m at 0.5.
- HEAD, ace2446: a 2 s settle, then 3.0 m at 0.4, stop on distance_reached.
