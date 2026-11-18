---
id: 2026-11-17_isaac-mpc-tuning
title: "Harness Isaac per il tuning MPC 'ostacolo davanti' e MPC con target sulla centerline"
area: mpc-control
date: 2026-11-17
tags: [isaac, obstacle-avoidance, w-obs, side-rule, night-loop, centerline-target]
modules: {mpc_controller: "0.0.0", f1tenth_params: "0.0.1"}
branch: "isaac-mpc-tuning"
commit: "fbf9c4730ccf"
status: done
hypothesis: "Con il target sempre sulla centerline, w_obs 8 -> 200 e la regola di lato, l'MPC aggira in Isaac un ostacolo statico davanti senza collisioni"
result: "Iterazione 1 (default): S1 3/3, S2 3/3, S3 2/3, S4 0/3 (e-stop), S5 1/1; M1 mobile 0/1 (collisione); scarti laterali 1.3-1.6 m per un Ø0.5"
decision: "Target mai deflesso, mpc_w_obs 8.0 -> 200.0, regola di lato (tie_band 0.05 m, lato left); night loop fermato dopo l'iterazione 1 per passare agli ostacoli mobili (seguito: 2026-11-18_mpc-dodge)"
raw_data: "git exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/moving_M1.log; exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/moving_M1/ (15 file .json/.jsonl/.log/.yaml); exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/night.log; exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/night2.log; exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/night_20261118_0021/ (113 file .json/.jsonl/.log/.yaml); exp/2026-11-17_isaac-mpc-tuning:output/isaac_tuning/night_20261118_0039/ (113 file .json/.jsonl/.log/.yaml); +15 gruppi (vedi _inventory.md). Esterni: nessuna bag: metriche per run in run.json (output/isaac_tuning/*/ su git); Isaac in ~/isaac_ws branch mpc-tuning"
migrated_from: "isaac-mpc-tuning:scripts/sim/isaac_tuning/HANDOFF.md"
migrated_on: 2026-11-18
---
# Handoff: MPC "target sulla centerline" + harness Isaac (stato al 2026-11-18, 00:55)

Da incollare all'inizio di una nuova chat. Tutto sta su **linus**. Il lavoro è committato e pushato su GitHub.

## 1. Dove sta il codice

| Cosa | Branch | Worktree | Ultimo commit |
|---|---|---|---|
| MPC nuovo (solo MPC, per l'Orin) | `mpc-centerline-target-linus` | `~/dev_ws/f1tenth_mpc` | `943f98c6` |
| Harness Isaac + MPC mergiato | `isaac-mpc-tuning` | `~/dev_ws/f1tenth_isaac_tuning` (già buildato, `--symlink-install`) | vedi `git log` |
| Lato Isaac (cilindri, layout `tuning`, `/sim/scenario`, ostacoli mobili, camera spettatore) | `mpc-tuning` | `~/isaac_ws` (repo a parte) | `0268464` |

- **Base comune:** `be9fa250` (`orin/mpc-centerline-target`, la linea dell'auto). Non è `hil-isaac`, che non contiene i 95 commit dell'auto.
- **`origin/mpc-centerline-target`** è un'implementazione **diversa**, pushata dall'Orin durante la sessione: `399e8854`, con `w_obs` 193. Non l'ho toccata; la mia sta su `-linus`.
- **Il checkout principale `~/dev_ws/f1tenth_more` non è coinvolto.** È sul vecchio `jazzy` e ha submodule sporchi, `.sjck/` e `docs/pwa/`; non l'ho toccato.

## 2. L'MPC nuovo (`mpc-centerline-target-linus`, 4 commit)

1. **Target sempre sulla centerline.** Da `compute_local_target` ho tolto la deflessione verso l'ostacolo (`obstacle_target_shift_m`), il coast di 5 tick e lo smoothing esponenziale. L'evitamento ora lo fa solo il costo `w_obs` del QP.
2. **`mpc_w_obs` da 8.0 a 200.0** (effettivo da 2.8 a 70). `w_term` resta 9.0 e `w_corr` resta 3.5714: abbassare `w_corr` non cambiava nulla. Ho provato anche ad attenuare `w_term` in base alla penetrazione dell'ostacolo e l'ho scartato: non migliorava la distanza e, con il minimo a 0.2, l'auto si fermava davanti all'ostacolo.
3. **Test closed-loop** in `test_centerline_target_closed_loop.py` (rig vero: corridoio, target, solver RTI, modello). Gli xfail sono strict, con i numeri misurati.
4. **Regola di lato** (richiesta tua). Un ostacolo entro `mpc_obstacle_tie_band_m` (0.05 m) dalla linea dell'auto passa al solver spostato sul lato opposto a `mpc_obstacle_tie_side` (`left`), con il raggio aumentato dello spostamento. Così il disco spostato contiene quello vero e non si perde clearance. File: `mpc_controller/side_rule.py`, testato in `test_side_rule.py`.

**Parametri cambiati in `stack_params.yaml`:**
- `obstacle_target_shift_m`: rimosso;
- `mpc_w_obs`: da 8.0 a 200.0;
- `mpc_obstacle_tie_band_m: 0.05` e `mpc_obstacle_tie_side: left`: nuovi.

**Test.** La suite di `mpc_controller` ha gli stessi 35 fallimenti e 7 errori di `be9fa250`, tutti preesistenti su linus: nessuna regressione. pep257 e flake8 non peggiorano. Nel closed-loop restano 5 xfail:
- R_safe nei due casi sulla linea: −4 e −7 mm. Il costo ostacolo guarda solo punti davanti al muso, e quando l'auto sterza il centro resta indietro;
- 3 cambi di segno dello sterzo nel caso perfettamente centrato: c'è un tremolio quando controsterza, ma non ho aggiunto altri bias;
- rientro sotto 0.10 m dopo l'ostacolo, sia nel caso centrato sia in quello spostato di 5 mm: non avviene perché il corridoio si ri-ancora sull'auto ogni secondo, per scelta di progetto ("THE PRICE" in `MPC_corr`).

**Sull'Orin** (immagine Jazzy, circa 10 minuti se cambiano solo i sorgenti):
```bash
cd ~/dev_ws/f1tenth_car                     # oggi su mpc-centerline-target (quello dell'Orin), pulito
git fetch linus mpc-centerline-target-linus   # oppure: git fetch origin mpc-centerline-target-linus
git switch -c mpc-centerline-target-linus FETCH_HEAD
ORIN_RUST=skip docker/orin/build.sh         # -> f1tenth:jazzy-orin-943f98c-norust
scripts/docker_orin.sh pin f1tenth:jazzy-orin-943f98c-norust && scripts/docker_orin.sh restart
```
**Attenzione:** l'immagine pinnata oggi è `33d7045` (`planners-on-car`). Questo branch **non contiene i planner modes** (`planner_controller_mode`); va deciso se servono.

## 3. Harness Isaac (`scripts/sim/isaac_tuning/`, vedi README.md e tuning/PLAN.md)

```bash
cd ~/dev_ws/f1tenth_isaac_tuning
systemctl --user stop freeparty-llm.service      # OBBLIGATORIO: il suo llama-server (5.4 GB) fa crashare Isaac (VRAM)
python3 scripts/sim/isaac_tuning/run_scenario.py --scenario S1 --params scripts/sim/isaac_tuning/tuning/baseline.yaml --repeat 3 --spectator
python3 scripts/sim/isaac_tuning/run_set.py --params scripts/sim/isaac_tuning/tuning/next_params.yaml   # S1-S5, una riga in history.csv
setsid nohup python3 scripts/sim/isaac_tuning/tuning/night_loop.py --max-iters 10 --spectator > output/isaac_tuning/night.log 2>&1 < /dev/null &
python3 scripts/sim/isaac_tuning/plots.py --night output/isaac_tuning/night_<data>      # figure
systemctl --user start freeparty-llm.service     # alla fine (il night loop lo fa da solo)
```
- **Isaac** gira headless nel container `f1t_isaac`; l'immagine è stata ricostruita stanotte (23 GB). Lo stack è lo stesso dell'auto (`sim:=isaac`). Uso dominio ROS 78 e Discovery Server sulla 11878.
- **Parametri:** passano tramite overlay generati, mai da `stack_params`, e vengono riletti dai nodi live prima di ogni run.
- **Ostacoli:** `unknown_objects_mode: active`, perché i cilindri non sono classi YOLO; sull'auto è `off`.
- **Per run:** posa con jitter tramite `/sim/scenario`, poi `/reset_all`, la missione dritta di 4 m a 0.5 m/s e la registrazione dei topic live, senza bag. Le metriche finiscono in `run.json`.
- **Chiusura:** a fine sessione controllo che non restino processi orfani.
- **Tempi:** Isaac si avvia in circa 20 s, lo stack in circa 15 s. Un run con lo stack riusato dura 20–35 s; un set (13 run) circa 7 minuti.
- **Night loop:** proposta di parametri con `claude -p` (fallback casuale), screening ×3, conferma ×10, stop quando tutti i run sono buoni (successo e distanza ≥ R_safe − 3 cm). Rispetta lo spazio dichiarato in `tuning/space.yaml`.
- **Scenari statici:** S1 Ø0.5 a 1.5 m, S2 Ø1.0, S3 ±0.15 m, S4 a 1.0 m, S5 senza ostacolo.
- **Scenari mobili:** M1 attraversa da destra a 0.4 m/s, M2 frontale a 0.25 m/s, M3 attraversa da sinistra a 0.5 m/s, M4 entra e si ferma sulla linea.

## 4. Risultati (iterazione 1, MPC nuovo + regola di lato, parametri di default)

| scenario | run buoni / run | esito | distanza minima − R_safe | scarto laterale massimo |
|---|---|---|---|---|
| S1 Ø0.5 a 1.5 m | 3/3 | successo | +0.12 m | 1.6 m |
| S2 Ø1.0 a 1.5 m | 3/3 | successo | 0.00 m | 1.45 m |
| S3 ±0.15 m | 2/3 | successo | −0.07 m (r2) | 1.3 m |
| S4 a 1.0 m | 0/3 | **e-stop** (BT IsProximityTooClose / hard stop) | −0.02 m | 0.07 m |
| S5 senza ostacolo | 1/1 | successo | – | 0.07 m |
| M1 (mobile, attraversa) | 0/1 | **collisione** | −0.02 m | 0.8 m |

- **Prima e dopo la regola di lato:** in S2, senza la regola, 1 run su 2 validi finiva in collisione; con la regola è 3/3.
- **Figure** in `output/isaac_tuning/figures/`:
  - `mappa_scenari_statici.png`, `mappa_prima_dopo_regola_di_lato.png`, `margini_per_run.png`, `mappa_ostacolo_mobile_M1.png`;
  - `screen_S1..S5.jpg` e `screen_M1.jpg`: griglie di 6 frame dalla camera spettatore.
- **Dati:**
  - `output/isaac_tuning/night_20261118_0039/it01_*`: iterazione 1;
  - `output/isaac_tuning/night_20261118_0021/`: prima della regola di lato, run validi solo in parte;
  - `output/isaac_tuning/moving_M1/`;
  - `tuning/history.csv`.

## 5. Problemi aperti

1. **Ostacoli mobili: l'MPC non prevede il moto** (come mostra M1). Vede ogni tick come una fotografia di dischi fermi. In M1 l'ostacolo arriva da destra, l'auto lo evita andando a sinistra, ma l'ostacolo continua ad andare a sinistra e lei gli finisce addosso. Serve la velocità dell'ostacolo, dai track del semantic layer (da verificare se la stimano), e poi dischi predetti per stadio nel QP, cioè la posizione al tempo k·ts. È il prossimo passo naturale.
2. **S4 (ostacolo a 1.0 m):** sempre e-stop. Da fermo l'auto parte a 0.4–0.5 m/s mentre le ruote stanno ancora girando (rate 0.5 rad/s) e scatta la prossimità del BT a 0.15 m. Nel rig nominale invece l'MPC aspetta circa 2 s prima di partire. Da capire chi impone la velocità: speed channel o floor.
3. **Gli scarti laterali sono grandi:** 1.3–1.6 m per un Ø0.5. L'auto si allarga molto e non rientra (manca l'homing). È un obiettivo del tuning: `w_obs` più basso o `w_corr`.
4. `/reset_all` ogni tanto fallisce sul passo `ekf_global` (`/ekf_global/set_pose` non disponibile). Ora il runner riavvia lo stack e ritenta il run.
5. Del night loop è stata fatta **solo l'iterazione 1**: l'abbiamo fermato per passare agli ostacoli mobili. Il proponente Claude è stato provato una volta, a mano, e ha funzionato.
6. Il sim usa `unknown_objects_mode: active`, l'auto `off`: i pesi trovati qui valgono sull'auto solo se arriva un disco simile.

## 6. Idee di Andreas (2026-11-18, fine sessione): da fare nella prossima chat

### 6.1 Oggetti veri, riconosciuti dalla perception: niente posizioni "regalate"
Oggi i cilindri arrivano all'MPC dal LiDAR come oggetti `unknown` (`unknown_objects_mode: active`), e il sim conosce la loro posizione esatta. È troppo facile. Serve **una persona che si muove**, riconosciuta da YOLO sulle immagini della ZED renderizzate da Isaac, proiettata in 3D, tracciata e messa sulla mappa dal semantic layer (`/costmap/semantic_objects` e `/costmap/semantic_obstacles`). Così la pipeline è la stessa dell'auto e si può tornare a `unknown_objects_mode: off`, come sull'auto.

**Cosa c'è già:**
- **Persona statica:** già in `~/isaac_ws`, base_room (`M_Medical_01` in posa `arms_down`, collider a capsula, ground truth `/sim/ground_truth/person_pose` a 50 Hz). Nel layout `tuning` oggi è spenta.
- **Persone che camminano:** esiste `sim/isaac/sim/people.py` sulla linea hil-isaac (commit `c6e5feb6`; si legge con `git show orin/logger-planner-mode:sim/isaac/sim/people.py`). Ogni persona segue dei waypoint a velocità costante. È un rigid body kinematic con collider a capsula, quindi il LiDAR la vede, ha una mesh che YOLO riconosce, e pubblica la ground truth negli objects. Limite: scivola, non c'è un ciclo di camminata. Va portato in `~/isaac_ws` `mpc-tuning`, a fianco dei movers di `/sim/scenario`.

**Cosa fare:**
1. In `/sim/scenario` aggiungere una o più `people: [{waypoints, speed, start_on_go}]`, avviate da `/sim/scenario_go` come i movers.
2. Pubblicare la ground truth di ogni persona su `/sim/ground_truth/movers`, così le metriche funzionano già.
3. Scenari P1–P4, con `unknown_objects_mode: off` (quindi senza `fixed_node_params`):
   - P1: persona che attraversa da destra a 1 m/s;
   - P2: persona che cammina verso l'auto;
   - P3: persona che entra e si ferma sulla linea;
   - P4: persona ferma sulla linea, il caso della regola di lato.
4. Nelle metriche: raggio della persona circa 0.29 (impronta 0.58 × 0.42); R_safe usa il margine di classe `person` 0.5 di `obstacle_class_margin_m`, come sull'auto.
5. Controllare la VRAM: Isaac, YOLO dello stack e camera spettatore sugli 8 GB. Durante i run di stanotte la GPU era intorno ai 3.7 GB.

Vantaggi: si misurano latenza e campo visivo della camera (la persona esce dall'inquadratura mentre la si supera: c'è già `test_person_pass_closed_loop.py`), il tracker e la memoria. I pesi trovati qui diventano trasferibili all'auto.

### 6.2 Regola di lato "informata": passare dal lato da cui l'ostacolo si sta allontanando
Oggi la regola di lato sceglie un lato **fisso** (`left`) quando l'ostacolo è entro 5 cm dalla linea. L'idea è usare le **osservazioni precedenti**. Se l'ostacolo si sta spostando lateralmente, il lato da cui proviene si sta liberando ed è più probabile che resti libero; quindi si passa **dietro** di lui, sul lato opposto alla sua velocità laterale. È anche la risposta giusta a M1, dove l'auto ha schivato nel verso in cui andava il cilindro e lo ha colpito.

**Dati disponibili:** `SemanticObject` (su `/costmap/semantic_objects`, già sottoscritto da `mpc_corr` per la memoria) ha **`id` di track stabile**, `x`, `y` in map, `radius_m`, `state` (TRACKED o MEMORY) e `time_since_seen_s`, ma **non la velocità**. `Obstacle2D` (`/costmap/semantic_obstacles`, la lista che usa il solver) ha solo `x`, `y`, `r`, senza `id`.

**Proposta** (da implementare in `side_rule.py` e `MPC_corr`, testabile nel rig closed-loop):
1. Stimare per ogni `id` la velocità da posizioni successive, con un filtro passa-basso o una regressione su una finestra di circa 1 s. Si possono anche aggiungere `vx`, `vy` a `SemanticObject` nel semantic layer, dove il tracker esiste già.
2. Associare gli ostacoli del solver ai track: stessa posizione entro pochi cm, oppure far portare l'`id` a `Obstacle2D`.
3. Regola per un ostacolo nella banda:
   - se `|v_lat| > v_min` (circa 0.1 m/s), il lato scelto è opposto al segno di `v_lat` (si passa alle sue spalle);
   - se è fermo, il lato dal quale è arrivato, se il track lo ricorda (lato appena liberato);
   - altrimenti il lato con più spazio libero nel costmap;
   - solo come ultima risorsa, il lato fisso.
4. **Latch per track e per mossa:** una volta scelto, il lato non cambia finché quel track è nella banda, con isteresi, per evitare oscillazioni. Oggi la regola non ha memoria.
5. Più in generale, oltre la banda: **dischi predetti per stadio** nel costo `w_obs`. La posizione dell'ostacolo allo stadio k diventa `p + v·k·ts`, così l'MPC "vede" dove sarà la persona. Con questo M1 si risolve senza bisogno della regola di lato.
6. Test: rig closed-loop con un ostacolo che si muove lateralmente (attraversa da destra e da sinistra, si ferma sulla linea) e asserzioni su lato scelto, nessun contatto e numero di cambi di segno. Poi in Isaac M1–M4 e P1–P4.

## 7. Prossimi passi suggeriti

- Girare M1–M4 ×3: `run_scenario.py --scenario M2 ...`; tutti funzionano già con `--spectator`.
- Nell'MPC: ostacoli con velocità, cioè posizione predetta per stadio nel costo `w_obs`. Usare `side_rule` per il caso frontale.
- Rilanciare il night loop sugli statici dopo aver deciso cosa fare di S4.
- Deploy sull'Orin con i comandi qui sopra.
