# Database analisi

Ogni prova, analisi o esperimento ha una cartella `docs/analysis/<area>/<id>/` con un `report.md`
(front matter YAML + testo), le figure in `figures/` e i dati riassuntivi in `data/`. I dati grezzi
(bag, video, log per run) **non** vanno qui: il report li cita nel campo `raw_data`.

- Un esperimento sta in **una** area; le altre lo citano con `tags`.
- Ogni area ha un `README.md` con stato attuale, decisioni, domande aperte e la tabella degli esperimenti.
- `INDEX.csv` elenca tutti gli esperimenti; `docs/modules/README.md` le versioni dei moduli.
- Tabelle e indici si rigenerano con `python3 tools/analysis_index.py` (aggiunge `--check` per uscire
  con 1 se un report ha campi mancanti o cita un modulo o una versione inesistente).

## Aree

<!-- AUTO:START -->
| area | esperimenti | open / done / abandoned | ultimo | stato in una riga |
|---|---|---|---|---|
| [field-tests](field-tests/README.md) | 10 | 5 / 5 / 0 | 2026-10-21 | Prima campagna analizzata (stop spiegati, bag spesso incompleti); dopo il 2026-10-22 nessun nuovo run analizzato qui. |
| [hardware-comms](hardware-comms/README.md) | 5 | 4 / 1 / 0 | 2026-10-21 | Telegram, PWA e hotspot funzionano su linus, nessuno provato sull'auto; il recorder perdeva l'inizio delle missioni. |
| [llm-pipeline](llm-pipeline/README.md) | 4 | 2 / 2 / 0 | 2026-11-13 | Planner: 18/20 live sul set go_to, 17/50 nella campagna dry-run; LLM2 solo come mock. |
| [localization-frames](localization-frames/README.md) | 3 | 2 / 1 / 0 | 2026-10-22 | Tracker muri corretto sul muro lontano (P0), foot da depth non superato (P1); wall lock solo su replay. |
| [mpc-control](mpc-control/README.md) | 13 | 5 / 8 / 0 | 2026-11-18 | MPC ok sugli statici; ostacoli in moto migliorati solo in Isaac (dodge 12-13/18); nulla del 2026-11 è stato provato sull'auto. |
| [perception](perception/README.md) | 8 | 7 / 1 / 0 | 2026-10-28 | Semantic layer e ostacoli unknown validati su replay; porte, vetro e parte stereo mai validati su dati reali. |
| [platform](platform/README.md) | 28 | 14 / 14 / 0 | 2026-11-11 | Jazzy in parità con Humble su Thor e linus; container Orin pronto ma P9-P12 in attesa; CPU dominata da glue Python. |
| [simulation](simulation/README.md) | 7 | 5 / 2 / 0 | 2026-11-13 | Isaac è il simulatore di riferimento (e2e 9/9); l'HIL gira sulla work con Isaac nativo su Windows, l'Orin nel giro resta senza risultati. |
| [_unsorted](_unsorted/README.md) | 4 | 3 / 1 / 0 | 2026-11-13 | 4 voci da decidere: bench senza report, log di processo git, documento Hydra, campagna office HIL senza risultati. |

Totale: 82 esperimenti. Elenco completo: [INDEX.csv](INDEX.csv).
<!-- AUTO:END -->

## Aggiungere un esperimento

1. `cp -r docs/analysis/_template/report.md docs/analysis/<area>/<AAAA-MM-GG_slug>/report.md` e compila il
   front matter: `id` (= nome cartella), `title`, `area`, `date`, `tags`, `modules` (versioni testate,
   da `package.xml`), `branch`, `commit`, `status` (`open` | `done` | `abandoned`), `hypothesis`,
   `result` (una riga), `decision`, `raw_data`.
2. Corpo: Setup, Metodo, Risultati (figure), Conclusioni, Prossimi passi. Figure sopra 1 MB: fuori da git,
   puntatore in `raw_data`.
3. `python3 tools/analysis_index.py`, poi aggiorna "Stato attuale" e "Decisioni prese" nel README dell'area.
4. Commit solo-docs su `main`. Un risultato non sta mai solo su un feature branch.
5. Tag `exp/<id>` sul commit testato (tiene raggiungibili codice e dati grezzi anche se il branch sparisce).

## Migrazione del 2026-11-18

I report esistenti sono stati raccolti da tutti i branch, i remote e i tag e spostati qui. Il corpo dei
report migrati è quello originale (solo i link alle figure sono stati aggiornati); il front matter è stato
aggiunto in migrazione, ricavato dal testo, con "n/d" dove il testo non lo dice. Per i report che non
avevano un testo proprio, `report.md` è stato scritto in migrazione e lo dice in testa.

- [`_inventory.md`](_inventory.md): ogni file trovato, branch e commit di origine, destinazione, stato.
  Serve anche per ritrovare un vecchio percorso (`output/...`, `docs/analysis/*.md`, `first_test_campaign/analysis/`).
- I dati grezzi rimasti su git sono raggiungibili dal tag `exp/<id>` indicato in `raw_data`.
- I moduli citati in `modules` dei report migrati hanno la versione di allora (`0.0.x`, mai incrementata):
  il commit esatto è nel campo `commit`.
