# Area: _unsorted (Da assegnare)

Materiale trovato sui branch che non è chiaramente un esperimento o non ha un'area chiara. Ogni voce ha una nota: decidere e spostare (o eliminare).

## 1. Stato attuale

> 3 voci da decidere: bench senza report, log di processo git, documento Hydra.

- `2026-11-09_bench-archive`: risultati di bench (JSON) senza report, più un piano di pulizia dell'archivio non eseguito. Area possibile: llm-pipeline o hardware-comms.
- `2026-10-16_git-integration-orin`: log di un'operazione git sull'Orin, non un esperimento. Probabilmente da togliere dal database.
- `2026-11-10_hydra-hil`: guida di integrazione di Hydra nell'HIL, senza misure; forse va tenuta in `docs/` come documentazione.

## 2. Decisioni prese

- n/d

## 3. Domande aperte / prossimi esperimenti

- Per ognuna: spostarla in un'area (aggiornando `area` nel front matter) o toglierla dal database.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-11-10 | [Hydra scene graph nel setup HIL Isaac](2026-11-10_hydra-hil/report.md) | open | Scritto e controllato su linus (RTX 5050) senza Hydra reale: smoke test, viewer, saver del logger e hydra_cost eseguiti solo in parte; Hydra reale mai eseguito. | f1tenth_sim 0.0.0, f1tenth_perception 0.0.1 |
| 2026-11-09 | [Bench avvio missione planner e piano pulizia archivio](2026-11-09_bench-archive/report.md) | open | Bench del 2026-11-09 hanno lasciato 342 voci in archivio (209 MB, 98 llm + 244 ctl); ne le soglie di latenza ne gli esiti sono commentati nel documento. | llm 0.0.1, f1tenth_logger 0.0.1, f1tenth_ctl 0.0.1 |
| 2026-10-16 | [Log integrazione git sull'Orin (campaign-final)](2026-10-16_git-integration-orin/report.md) | done | Divergenza: 3 commit solo remoti (da linus) e 64 solo locali (3ffe245..9c46add); 1 file toccato da entrambi (TEST_CAMPAIGN.md); merge senza collisioni, e67aded locale non pushato. | f1tenth_logger 0.0.1 |
<!-- AUTO:END -->
