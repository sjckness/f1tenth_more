# office_batch: il batch HIL sul mondo office

Il batch prende tutte le pose di partenza fattibili dell'office e le fa girare con tutti i
planner, per N ripetizioni ciascuna, in un'unica corsa senza assistenza. Alla fine calcola i
risultati. Le pose sono 35, tutte quelle di `find_door_runup.py --run 5 --side left`
([docs/analysis/simulation/2026-11-13_office-feasible-tests](../../../docs/analysis/simulation/2026-11-13_office-feasible-tests/README.md)).
I modi sono `corridor_mpc`, `corridor_rrt` e `p2p_geometric`, con 3 ripetizioni: in totale
**315 run**.

- **Isaac** gira sulla work, nativo su Windows, con un lancio per posa: la posa di partenza
  si fissa al lancio di Isaac.
- **Lo stack** gira sull'Orin, nel container `f1tenth_hil` creato dall'immagine fissata
  dell'auto. Per ogni modo parte da zero, perche' `planner_controller_mode` si sceglie al
  lancio.
- **La work comanda l'Orin** via ssh, attraverso `car-hil`. Sull'Orin non si modifica nulla:
  `car-hil` e `car-hil-agent` ci arrivano da linus con `scripts/car/deploy.sh --tools-only`.

## La missione

`missions/straight_5m_then_left_door.json`, fissa:

1. rettilineo di 5 m a 0,4 m/s;
2. `enter_door` sulla porta a sinistra (la stessa seconda mossa di
   `straight_2m_then_left_door.json`).

Non passa dall'LLM, perche' l'`llm` di questa linea non ha l'intent `enter_door`. Con una
missione fissa i tre modi eseguono esattamente le stesse mosse.

## Cosa si misura (`car-hil-agent analyze`)

Ogni run viene giudicato in due modi.

- **Missione**: lo stato finale (`COMPLETE` / `ABORTED`) e il `/mission/move_outcome` di ogni
  mossa, per esempio `door_not_found` o `timeout:abort`.
- **Ground truth**: dove passa e dove si ferma l'auto rispetto al varco vero della posa.
  - *attraversato*: il centro dell'auto taglia la linea del varco dentro l'apertura;
  - *nel varco*: a fine run l'auto sta nel varco, con |along| <= 0,30 m,
    |lateral| <= w/2 - 0,186 m e yaw entro 20 gradi.

**Successo** = missione COMPLETE **e** (attraversato **o** nel varco).

Per ogni modo l'analisi riporta il tasso di successo con l'IC di Wilson al 95 %, i fallimenti
per tipo, la durata mediana e la CPU del controllore (ms di CPU per secondo simulato, da
`/proc`).

## Sulla work, una volta sola

La work non sviluppa: si fa solo il pull.

```bat
git clone https://github.com/sjckness/f1tenth_more F:\Users\Andreas Steffens\dev_ws\f1tenth_batch
cd /d F:\Users\Andreas Steffens\dev_ws\f1tenth_batch
git checkout hil-office-batch
```

Poi serve l'USD dell'office, che non e' in git (`sim/isaac/usd/generated/office/office.usd`).
Ci sono due modi per averlo:

- estrarlo da `~/f1tenth_salvataggi/work-20261119.tar.gz` (WSL), nella cartella
  `sim/isaac/usd/generated/office/` di questo checkout;
- rigenerarlo con `scripts\sim\win\hil_world_convert.bat`, che usa la cache Fuel `~/.gz/fuel`.

Il resto e' gia' pronto:

- `sim/isaac/config/hil.work.yaml` contiene gli indirizzi della work (192.168.200.2 verso
  192.168.200.11, dominio 44, Discovery Server :11812). Lo seleziona `hil_win_env.bat`.
- Il link, il firewall e la chiave ssh verso `fabiocar@192.168.200.11` sono quelli dell'HIL
  del 13/11 (WORK_FINDINGS.md B.4).

## Sull'Orin, una volta per avvio

```bash
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
```

Senza questo comando il preflight da' solo un WARN: le immagini della camera si perdono a
raffiche, mentre il lidar, che serve alla porta, passa lo stesso. Inoltre lo stack dell'auto
deve essere fermo (`car-stack stop`), e il preflight lo controlla.

## Lanciare

Dalla cartella del checkout, sulla work:

```bat
scripts\sim\win\hil_office_batch.bat --dry-run   :: controlli e piano, niente avviato
scripts\sim\win\hil_office_batch.bat --smoke     :: 1 posa, 1 modo, 1 ripetizione: la catena intera
scripts\sim\win\hil_office_batch.bat             :: il batch intero
```

Prima di partire il driver controlla tre cose, e si ferma se una fallisce:

- che `car-hil` e `car-hil-agent` sull'Orin siano identici a quelli del checkout. Se non lo
  sono, va fatto `scripts/car/deploy.sh --tools-only <commit>` da linus;
- `car-hil preflight`;
- che l'USD dell'office esista.

Si possono anche scegliere pose, modi e ripetizioni:

```bat
scripts\sim\win\hil_office_batch.bat --poses 2,16 --modes corridor_rrt --reps 1
```

Per riprendere una sessione interrotta (Ctrl+C, crash, reboot) si rilancia con lo stesso id.
I run che hanno gia' un risultato vengono saltati:

```bat
scripts\sim\win\hil_office_batch.bat --session 20261123_220000_office_door_left_5m
```

Per rifare solo l'analisi:

```bat
scripts\sim\win\hil_office_batch.bat --analyze-only 20261123_220000_office_door_left_5m
```

**Durata stimata**: circa 15 minuti per posa, quindi circa 9 ore per 35 pose. Il calcolo e'
1 lancio di Isaac, poi per ciascuno dei 3 modi uno stack da circa 1 minuto e 3 run da circa
1-2 minuti.

## Dove finiscono i risultati

- **Sull'Orin**, in `~/f1tenth_hil_batch/<sessione>/`:
  - `session.json`, `mission.json`, `versions.json` (le versioni dei moduli dell'immagine);
  - `logs/` (stack, Discovery Server, run);
  - `runs/<pNN_modo_rK>/` (`gt.csv`, `odom.csv`, `drive.csv`, `mission.csv`, `cpu.csv`,
    `outcomes.jsonl`, `doors.jsonl`, `paths.jsonl`, `result.json`);
  - `analysis/` (`report.md` con il front matter di docs/analysis, `runs.csv`,
    `summary_by_mode.csv`, `by_pose.csv`, `figures/`).
- **Sulla work**, in `scripts\sim\win\batch\<sessione>\`: `batch.log`, i log di Isaac per posa
  e una copia di `report.md`.

Per portarli in `docs/analysis`, da linus:

```bash
rsync -a jetson-orin:f1tenth_hil_batch/<sessione>/analysis/ docs/analysis/simulation/<data>_office-door-batch/
```

I CSV per run restano sull'Orin, come `raw_data`.

## Come e' fatto

| file | dove gira | cosa fa |
|---|---|---|
| `scripts/sim/win/hil_office_batch.py` (`.bat`) | work | piano, Isaac per posa, ssh verso car-hil, ripresa, report |
| `scripts/sim/win/hil_win.py` | work | lancia Isaac (`sim --net link --layout door_left --set scene.car.start.*`) |
| `scripts/car/orin/hil.sh` = `car-hil` | host dell'Orin | container, Discovery Server, stack up/down, run, mark, analyze |
| `scripts/car/orin/hil_agent.py` = `car-hil-agent` | nel container | profilo DDS, attese, ready, un run (reset, load, start, registrazione), analisi |
| `scripts/sim/office_batch/batch.yaml` | work | pose, layout, missione, modi, ripetizioni, timeout, argomenti dello stack |
| `scripts/sim/office_batch/fake_sim.py` | container (solo test) | sostituto cinematico di Isaac per provare la catena senza la work |

Prova senza la work, da linus: `fake_sim.py` gira nel container HIL dell'Orin al posto di Isaac.
Il comando completo e' nel messaggio del commit che ha introdotto questa cartella.
