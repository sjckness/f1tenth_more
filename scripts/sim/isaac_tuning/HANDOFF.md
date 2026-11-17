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

## 6. Prossimi passi suggeriti

- Girare M1–M4 ×3: `run_scenario.py --scenario M2 ...`; tutti funzionano già con `--spectator`.
- Nell'MPC: ostacoli con velocità, cioè posizione predetta per stadio nel costo `w_obs`. Usare `side_rule` per il caso frontale.
- Rilanciare il night loop sugli statici dopo aver deciso cosa fare di S4.
- Deploy sull'Orin con i comandi qui sopra.
