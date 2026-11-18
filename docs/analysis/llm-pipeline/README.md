# Area: llm-pipeline (Pipeline LLM)

Planner LLM (intent, traduttore, missione JSON), valutazioni dei prompt, LLM2 tuner dei parametri MPC.

## 1. Stato attuale

> Planner: 18/20 live sul set go_to, 17/50 nella campagna dry-run; LLM2 solo come mock.

- Prompt `go_to` (qwen2.5-3b q5_k_m): 18/20 comandi giusti live a temperatura 0; la selezione di istanza ("la seconda sedia") non è risolta; circa 15.6 s per comando.
- Campagna dry-run di 50 prompt (2026-09-18): 17 ok, 24 rifiutati, 9 errori; `goal_door_opening` 0/10.
- LLM2 (tuner MPC) esiste solo come mock: 151/151 risposte valide, latenza mediana 862 ms; tutto spento di default; i pesi MPC si leggono una volta all'avvio.
- Il planner end to end in Isaac sta in [simulation](../simulation/README.md) (`sim_next`).

## 2. Decisioni prese

- Prompt spedito: variante DG senza blocco duplicato (18/20); niente selezione nelle prove a terra ([2026-09-17_go-to-live-eval](2026-09-17_go-to-live-eval/report.md))
- LLM2: due interruttori indipendenti, default false; whitelist di parametri con range check ([2026-10-21_two-llm-arch](2026-10-21_two-llm-arch/report.md))

## 3. Domande aperte / prossimi esperimenti

- Latenza di 15.6 s per comando e non determinismo a temperatura 0.
- Cause dei rifiuti `ambiguous_after_rejection` della campagna dry-run: n/d.
- Tuner LLM2 reale: serve un parameter callback in `mpc_corr`.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-10-21 | [Architettura a due LLM: LLM2 tuner parametri MPC (mock)](2026-10-21_two-llm-arch/report.md) | open | Su linus, 300 s a 0.5 Hz: 151/151 ok, echo_match 100%, latenza mediana 862 ms (max 1062); 161 pubblicazioni = 161 applicazioni, mediana 18.6 ms; VRAM LLM2 2578 MiB. Nulla misurato sull'Orin. | llm 0.0.1, mpc_controller 0.0.0, f1tenth_bringup 0.0.1, f1tenth_messages 0.0.1, f1tenth_logger 0.0.1 |
| 2026-09-18 | [Campagna dry-run del planner LLM (50 prompt)](2026-09-18_llm-dry-campaign/report.md) | done | Run 20260918_160413 su 50 prompt: 17 ok, 24 refused, 5 translator_error, 4 llm_error; goal_door_opening 10/10 refused; run piccoli di prova su timeout e URL irraggiungibile. | llm 0.0.1 |
| 2026-09-17 | [Eval live del go_to del planner LLM (qwen2.5-3b) con e senza go_to](2026-09-17_go-to-live-eval/report.md) | done | 18/20 passati con go_to, 3/3 senza; falliti 'vai dalla persona a sinistra' (selezione pianificata come go_to) e 'la seconda sedia'; prompt +668 token (1710 -> 2378) | llm 0.0.1, f1tenth_behavior 0.0.1 |
<!-- AUTO:END -->
