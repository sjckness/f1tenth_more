# office-feasible-tests — le pose di partenza verificate fattibili nel mondo `husarion_office_2x`

Recuperato dalla macchina **work** (DESKTOP-Q3F81S6) il 2026-11-23. Tutto quello che c'è qui è
stato **calcolato dall'SDF**, senza simulatore: `src/f1tenth_sim/worlds/husarion_office_2x.sdf`
proiettato su una griglia di occupazione a 5 cm al piano del lidar (base_link + 0,19 m, il piano
dell'UST-10LX), come fanno `sim/isaac/tools/pick_spawn_poses.py` e
`sim/isaac/tools/office_missions.py`.

Le coordinate sono **base_link in `map_sim`**, cioè gli stessi metri di Gazebo e dell'USD
generato. `z = 0.02` per ogni posa.

## Da dove viene, e perché non era in git

Questi file erano negli **scratchpad** delle sessioni Claude Code del 2026-11-13 su `work`:

| file qui | origine |
|---|---|
| `data/door_runup_left_5m.json` | `…/Temp/claude/56e189eecf0f421f/e41e0b8e-…/scratchpad/door_runup.json`, 2026-11-13 14:57 |
| `data/*.json` (gli altri) e `scripts/mapan/*.py` | `…/Temp/claude/56e189eecf0f421f/b579ad0e-…/scratchpad/mapan/`, 2026-11-13 14:35-15:17 |

Sessione `e41e0b8e` = il lavoro `planner_compare` + porte; sessione `b579ad0e` = l'analisi
missioni office. Del primo insieme in git è finito **solo il candidato migliore**, come preset
`door_left` in `sim/isaac/config/scenes/office.yaml` (commit `c72de207`); del secondo è finito il
report a 4 comandi (`6b38cea8` → `docs/analysis/llm-pipeline/2026-11-13_office-missions/`). Le
liste complete non erano mai state salvate da nessuna parte.

Il commit che faceva girare Isaac su `work` è **`47ac7b8b`** (branch `planner-modes-hil`, mai su
`origin/main`): oggi è nei tag `archive/work-planner-modes-hil-20261119` e
`exp/2026-11-13_office-hil-campaign`. ⚠️ `main` **non contiene** `sim/`, quindi per rieseguire
questi strumenti serve `git checkout exp/2026-11-13_office-hil-campaign`.

## Il pezzo principale: 35 pose di partenza, non 36

`door_runup_left_5m.txt` / `data/door_runup_left_5m.json` —
`find_door_runup.py --run 5 --side left`:

> **130 varchi nel mondo, 35 rincorse tenute.**

Una riga = un test: una posa di partenza da cui l'auto può percorrere **5 m in rettilineo** e
arrivare con un **varco esattamente 2 m alla propria sinistra** — la geometria che serve a
`"go straight 5m then enter the door on the left"`. Un candidato è tenuto solo se la posa di
partenza sta su pavimento con più di un raggio di footprint di franco, la corsa è libera per
almeno 5 m e il franco non scende mai sotto il raggio di footprint lungo tutto il tragitto.

Campi per candidato: `start {x, y, z, yaw_deg}`, `door {x, y, width_m, passage_axis}`,
`free_ahead_m`, `free_beyond_arrival_m`, `min_lateral_clearance_m`, `start_clearance_m`,
`floor_z_m`.

**Sono 35, non 36.** Se cercavi 36 e questi non sono, l'unica altra lista di quell'ordine di
grandezza su questa macchina è `data/turn_left.json` filtrata (26 pose con
`straight2m_after_chair_free: true`). Nessun file, blob git, sessione o tar su `work` contiene una
lista di 36 — vedi `WORK_FINDINGS.md` nella radice per l'elenco completo dei posti controllati.

`free_beyond_arrival_m` va letto prima di scegliere: `corridor_rrt` costruisce il corridoio
`rrt_corridor_length_drive_m` (3,0 m) **oltre** l'auto e dice `no path` quando quell'estremo è
occupato, quindi un muro subito dietro la porta blocca la fase di guida prima che inizi quella
della porta. Per questo il preset `door_left` ha preso la riga con **12,7 m liberi oltre
l'arrivo** e non una geometricamente più stretta.

La griglia **sovrastima** l'occupazione (box allineati agli assi per triangolo): una posa che lo
strumento dichiara libera è libera; una che rifiuta può non esserlo.

## Gli altri insiemi

| file | voci | cos'è |
|---|---|---|
| `data/turn_left.json` | 565 | pose di partenza per "muro → svolta a **sinistra** → …": `wall1`, `leg1`, `after_turn`, `wall2`, le sedie visibili dopo la svolta (`chairs`, 168 pose ne hanno almeno una), `go_to_stop`, e `straight2m_after_chair_free` (**26 true**: le sole pose del mondo dove un "go straight 2 m" dopo una sedia avrebbe spazio) |
| `data/turn_right.json` | 552 | lo stesso a **destra** |
| `data/mB_LEFT.json` | 189 | pose equivalenti per M-B (`muro → sinistra → muro`), con `leg1`/`leg2` e uno `score`. Sono le "189 pose equivalenti" citate nel report a 4 comandi |
| `data/mB_RIGHT.json` | 241 | lo stesso specchiato |
| `data/door_missions.json` | 23 | una riga per varco **attraversabile**: `start`+`yaw`, larghezza, asse del passaggio, stanze ai due lati, `side`, `ahead`, `lateral`, `room_beyond`, `earlier_same_side` (se un ordinale sarebbe ambiguo) e `chairs_from_door` |
| `data/mD.json` | 5 | le 5 varianti di M-D (`svolta → persona → porta`) con la posa della persona e il franco al muro |
| `data/missions.json` | 5 | i 5 verdetti finali del report a 4 comandi: M-A `ok`, M-A2 `ok`, M-B `ok`, M-C `partial`, M-D `ok-with-setup` |
| `data/doorways.json` | 164 | ogni varco trovato sulla griglia, con i due punti di approccio |
| `data/openings2.json` | 23 | i varchi reali, con `detectable` (**11**: dentro la banda 0,6-1,5 m del rilevatore), `nominal` (banda 0,7-1,4 m) e `drivable` |
| `data/openings.json` | 26 | il primo passaggio, prima del filtro |
| `data/objects2.json` | 93 | i 93 modelli di arredo con bounding box, classe semantica e stanza |
| `data/map_geo.json` | — | la pianta che disegna `office_hil.html` (tratti di muro e pavimento a 10/20 cm, box oggetti, varchi, stanze, le 7 sphere light) |

## Rigenerare

Serve un checkout con `sim/` e le mesh Fuel (su `work` sono in `~/.gz/fuel`). Niente Isaac,
niente GPU.

```bash
git checkout exp/2026-11-13_office-hil-campaign
python3 sim/isaac/tools/find_door_runup.py --run 5 --side left  --json output/door_runup_left.json
python3 sim/isaac/tools/find_door_runup.py --run 5 --side right --json output/door_runup_right.json   # mai eseguito
python3 sim/isaac/tools/pick_spawn_poses.py --json output/spawn_poses.json
python3 sim/isaac/tools/office_missions.py --json output/office_missions > output/office_missions/report.txt
```

`scripts/find_door_runup.py` è la versione committata (`47ac7b8b`), che importa
`pick_spawn_poses` (`occupancy`, `clearance_map`, `doorways`, `free_run`, `world_xy`) e
`sdf_to_usd` (`parse_world`, `mesh_info`) dalla stessa cartella `tools/`.
`scripts/find_door_runup.scratch.py` è la versione usata in sessione, che prendeva la cartella
`tools/` con `--tools` e ha prodotto **questo** JSON.
`scripts/mapan/*.py` sono gli script di analisi one-shot che hanno prodotto gli altri `data/*.json`
(`an1`-`an4` griglia e muri, `openings`/`prim` i varchi, `turn_obj` le svolte, `doors_missions` i
varchi attraversabili, `mission`/`verify`/`final` le catene di move, `mD`/`obst` la persona,
`export`/`gen` l'HTML). Hanno percorsi assoluti della work dentro: sono qui come traccia del
calcolo, non come strumenti riusabili — gli strumenti sono quelli in `sim/isaac/tools/`.

## Usarli per un batch HIL

La posa di partenza **è fissata al lancio di Isaac** (`scene.car.start`, dal preset in
`office.yaml`): `/sim/reset` riporta l'auto sempre a quella posa e non esiste un servizio per
spostarla a caldo. Quindi **una posa = una sessione Isaac**:

```bat
scripts\sim\win\hil_sim_link.bat --layout door_left ^
   --set scene.car.start.x=16.464 --set scene.car.start.y=-12.604 ^
   --set scene.car.start.yaw_deg=180.0 --set scene.car.start.z=0.02
```

Gli `--set` vincono sul preset (ordine in `sim/isaac/sim/config.py:load_config`: `scene.layout`,
poi il merge del preset, poi tutti gli altri override). ⚠️ `_merge` **sostituisce le liste per
intero**: non elencare solo alcune `sphere_lights`, o le altre si spengono in silenzio.

Il resto — riavvii automatici fra pose, reset dello stack, ssh all'Orin, versioni — è in
`WORK_FINDINGS.md` nella radice del branch.
