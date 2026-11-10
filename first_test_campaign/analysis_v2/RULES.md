# Regole di analisi v2: first test campaign

Stato: **confermato il 2026-11-02**.

Output previsto: `first_test_campaign/analysis_v2/`.

## 1. Principi

1. **I dati grezzi sono in sola lettura.** Non si toccano le cartelle delle run, né `analysis/`, né `campaign_results.csv`.
2. **Assente non vuol dire zero.** Un valore non misurato è `null` con un motivo in `<colonna>_why`, mai `0` o una stima.
3. **Ogni numero è tracciabile:** ID completo della run, commit dei dati, commit dello script, parametri e checksum dei file letti.
4. **Unità nel nome della colonna:** `jerk_rms_mps3`, `min_clear_any_m`.
5. **Automatico e manuale restano separati.** Le colonne calcolate non si modificano a mano. I giudizi manuali stanno in `manual_verdicts.yaml`, ciascuno con il motivo e la prova (video, nota, timestamp).
6. **Nessuna metrica fa due lavori.** Se un valore può significare due cose, diventa due colonne.
7. **Criteri prima dei risultati.** Soglie e criteri si fissano qui prima del ricalcolo. Se cambiano dopo, si registra il cambio e si ricalcola tutto.

## 2. Selezione e tentativi

- `selection.yaml` è l'unica fonte delle run incluse: nome della cartella di output e, per ogni run, ostacolo sì/no e output da generare. Sostituisce `run_selection.yaml` per la v2.
- I gruppi di analisi sono **missione (`M01`, `M02`…) + ostacolo sì/no**, con l'ostacolo preso dal file. Prompt e obiettivo atteso si leggono dalla run (`plan.json`, eventi, `meta.json`).
- `attempts.csv` elenca **tutti** i tentativi della missione nelle stesse sessioni, inclusi quelli non selezionati, con lo stato:
  - `selected`
  - `not_started`: l'auto non si è mossa e la missione non è partita davvero
  - `log_invalid`: dati insufficienti
  - `setup_error`: errore di preparazione, con nota
  - `not_selected`: altra run scartata, con motivo obbligatorio
- **Un fallimento dell'auto non è un motivo di esclusione.** Contatto, estop o abort possono uscire dalla selezione solo se dichiarati in `attempts.csv`.
- Ogni tabella riporta entrambi: `k/N selezionate` e `k/N tentativi validi`.

## 3. Finestra di analisi

| Finestra | Definizione |
|---|---|
| `drive` | da `mission_started` al primo tra `mission_finished` e `mission_aborted` |
| `moving` | campioni di `drive` con velocità > 0.05 m/s |
| `after_start` | campioni di `drive` dopo i primi 0.30 m percorsi |

Per ogni metrica è indicata la finestra usata. Se la posa non è valida, `moving` e `after_start` sono `null`.

## 4. Controlli di qualità

Ogni run ha flag calcolati prima delle metriche. Un flag fallito annulla solo le metriche che dipendono da quel dato.

| Flag | Fallisce se | Annulla |
|---|---|---|
| `pose_valid` | percorso = 0 con comandi attivi, salto tra campioni oltre `v_max·dt` + tolleranza, oppure percorso > `v_max·durata` | distanze, deviazione laterale, sterzo/m, metriche `moving` |
| `clearance_coverage_pct` | sotto il 50% del tempo di `drive` | tutte le clearance; tra 50% e 90% resta il valore con flag `partial` |
| `clearance_source` | sempre riportato: `live` oppure `backfill` | backfill = sempre `partial`, il vero minimo può essere più basso |
| `imu_valid` | meno di 2 s di dati o frequenza sotto 40 Hz | jerk |
| `solver_valid` | nessuno stato MPC in `drive` | metriche solver |
| `corridor_schema` | diverso da v2 | metriche di corridoio |

Casi già noti da verificare con questi flag:
- P004-R014 e R015: posa ferma a zero.
- P004-R015: nessuna clearance.
- P004-R017: percorso impossibile (8.01 m in 13.5 s, con 0.38 m/s massimi).
- P001-R001…R005: clearance solo in backfill, copertura 35–43%.

Il 50% è volutamente stretto: con la copertura al 35–43%, la clearance delle P001-R001…R005 risulta `null`, con il valore di backfill in una colonna a parte.

## 5. Esito: tre livelli separati

Il vecchio `Success` unico sparisce, perché mescolava cause diverse.

| Colonna | Domanda | Regola |
|---|---|---|
| `plan_ok` | L'LLM ha prodotto il piano giusto? | sequenza e parametri confrontati con l'obiettivo atteso; per run, non per comando |
| `exec_ok` | L'auto ha eseguito il piano? | `mission_finished` raggiunto e valore del piano rispettato; un abort o un timeout è fallimento |
| `safe` | È stata sicura? | nessun `estop`, nessun `contact`, nessun contatto segnalato a mano |
| `task_ok` | Il compito del prompt è riuscito? | criterio della sezione 6 misurato sui dati |
| `success` | **Riuscita complessiva** | `plan_ok AND exec_ok AND safe AND task_ok` |

- **Contatto manuale.** Il LiDAR è un piano solo e non vede ciò che sta sotto, come un piede. Le note "mi è arrivato addosso" e "passato sopra il piede" diventano `contact_manual: true`, quindi `safe = false`.
- **Override.** `success_manual` può contraddire il calcolo solo con un motivo scritto, e compare accanto al valore calcolato, senza sostituirlo.
- **Terminologia.** Le distanze vengono dall'odometria, quindi sono stime e non verità a terra. La colonna si chiama `*_odom_m`.

## 6. Criteri per gruppo

Le sigle G indicano i comandi del vecchio foglio. Il raggruppamento effettivo è quello della sezione 2; P006 e P008 restano quindi separate.

| Gruppo | Prompt | Criterio `task_ok` | Stato |
|---|---|---|---|
| G1 | go ahead 3 meters | spostamento 3.0 ± 0.15 m, errore laterale < 0.20 m | ok |
| G2 | approach the wall | **da decidere** (D1) | aperto |
| G3 | go to the potted plant | fermo entro 0.5 m dalla pianta, orientato verso di essa | serve una misura della distanza dal target (D5) |
| G4 | go to the person | fermo a 1.0 ± 0.3 m dalla persona | ok |
| G5 | turn left at the wall | svolta iniziata alla distanza richiesta, circa 90° ± 15° | **distanza da decidere** (D2) |
| G6 | go ahead 4 meters + ostacolo | spostamento 4.0 ± 0.15 m, errore laterale < 0.20 m misurato a fine manovra, ostacolo superato | ok |
| G7 | approach the wall + ostacolo | come G2 | dipende da D1 |
| G9 | go to the person + ostacolo | come G4 | ok |
| G10 | turn left at the wall + ostacolo | come G5 | dipende da D2 |
| G11 | turn at the wall, then go to a person | svolta corretta, poi come G4 | sinistra e destra: D4 |
| G12 | go to the chair, then go straight 4m | fermo vicino alla sedia, poi 4.0 ± 0.15 m oltre | la fermata alla sedia prevista dal piano va chiarita (nota "0.9") |

Il vecchio cmd 8 (potted plant + ostacolo, tutto abortito) non ha run nel tuo elenco: compare solo in `attempts.csv`, se i tentativi esistono sulla Orin.

## 7. Metriche

### Precisione
- `final_displacement_odom_m`, `goal_error_m`: spostamento finale ed errore rispetto al piano e al criterio.
- `lateral_error_max_m`, `lateral_error_rms_m`: rispetto alla retta fissata a inizio move, solo per i move rettilinei.
- `heading_change_deg`: variazione di yaw durante il move, per le svolte.

### Corridoio
Il vecchio `viol_rate` non poteva quasi mai scattare sui rettilinei, perché il corridoio riparte dall'auto a ogni ricostruzione.

- `viol_fixed_pct`: % di tempo con la sagoma fuori dal **primo corridoio del move, tenuto fisso**. Vale solo per i rettilinei; per gli altri move è `null`.
- `viol_rebuilt_pct`: la vecchia metrica, tenuta come diagnostica e non usata per giudicare il controllo.
- `horizon_err_mean_m`, `horizon_err_max_m`: distanza tra predizione MPC e traiettoria reale.

### Clearance
Il dato salvato è un solo valore: la distanza minima dalla sagoma a qualsiasi ritorno LiDAR. Non sa cosa sia il ritorno e non vede gli oggetti sotto il piano di scansione.

- `clear_start_m`, `clear_end_m`: al primo e all'ultimo campione.
- `min_clear_any_m`, `t_min_clear_s`: minimo su tutto `drive` e quando avviene.
- `min_clear_after_start_m`: minimo dopo i primi 0.30 m, per non scambiare ciò che era già vicino alla partenza con un avvicinamento.
- `front_clear_end_m`: distanza frontale dal paraurti alla fine. Solo dove c'è la scansione nella bag, altrimenti `null`.
- `min_obstacle_clear_m`: distanza dall'ostacolo vero. Solo se ricostruibile da scansione più tracce, altrimenti `null`.
- `contact_lidar`: il valore è ≤ 0. Il valore grezzo resta tale, senza forzarlo a 0.

### Solver
La vecchia `Feas.` contava `solved_inaccurate` come infattibile.

- `solver_solved_pct`, `solver_inaccurate_pct`, `solver_maxiter_pct`, `solver_infeasible_pct`: la somma fa 100.
- `solver_usable_pct`: `solved` + `solved_inaccurate`.
- `unusable_streak_max_s`: serie più lunga di solve non utilizzabili.
- `solve_time_p95_ms`.

Prima del ricalcolo va verificato nel codice cosa invia il controller davvero su `max_iter`, `infeasible` e `solved_inaccurate`. Se `solved_inaccurate` non venisse applicato, la definizione di `usable` cambia.

### Fluidità
- L'IMU dà le accelerazioni in g: si convertono con 9.80665 prima di derivare.
- `jerk_rms_mps3`: finestra `moving`, solo orizzontale, filtro Butterworth zero-phase di ordine 2 a 2 Hz.
- `jerk_rms_1hz_mps3`, `jerk_rms_5hz_mps3`: stesso calcolo a 1 e 5 Hz, per mostrare la sensibilità al filtro.
- `moving_pct`: quota di tempo in movimento, così una run ferma a lungo non sembra fluida.

Motivo del filtro: su P001-R001 il vecchio valore passa da 0.75 a 0.23 e 0.08 g/s con cutoff 5, 2 e 1 Hz, quindi a 5 Hz domina la vibrazione. Il cutoff va confermato con uno spettro su 3–4 run (D3).

### Sterzo
- `steer_direction_rev_per_m`: inversioni di direzione del comando filtrato, con isteresi di 0.02 rad tra un estremo e l'altro, diviso per il percorso in `drive`. Vede anche le oscillazioni attorno a un angolo diverso da zero.
- `steer_sign_rev_per_m`: la vecchia metrica, per confronto.
- `steer_rate_rms_radps`, `steer_abs_max_rad`.

Se il percorso non è valido, le colonne per metro sono `null`.

### Contesto
`llm_latency_ms`, `drive_duration_s`, `path_odom_m`, `v_max_mps`, `auto_outcome`, `abort_reason`, `timeout_hit`, `git_commit`, `odom_topic`, `robot_radius_m`.

## 8. Aggregazione

- Successi: `k/N` con intervallo di Wilson al 95%. Con 5/5 l'intervallo è circa 57–100%: non va scritto "100% affidabile".
- Continue: mediana, minimo e massimo, con N valido. Niente medie su 5 run, niente media su colonne con molti `null` senza dirlo.
- Il gruppo con ostacolo non si mischia a quello senza. Le PNG selezionate sovrappongono solo run della stessa missione con lo stesso valore di ostacolo.
- Run con commit o parametri diversi nello stesso gruppo vengono segnalate, perché il confronto non è a parità di codice.

## 9. Output

```text
first_test_campaign/analysis_v2/
  RULES.md             questo documento
  selection.yaml       unico file da modificare
  run_analysis.py      unico script da lanciare
  scripts/             metriche, grafici e video
  <output_folder>/
    results.csv        tabella stile f1tenth_testing
    runs_full.csv      tutte le metriche
    groups.csv         aggregati per missione + ostacolo
    data_quality.csv   flag e motivi di ogni null
    attempts.csv       tutti i tentativi con stato
    old_vs_new.csv     vecchio valore, nuovo valore, differenza spiegata
    manual_verdicts.yaml
    build_info.json    commit, parametri, checksum
    <Mxx_con_ostacolo|Mxx_no_ostacolo>/
      png_singole/  png_selezionate/  video_maschera/  video_dall_alto/
```

Il foglio Google si aggiorna solo dopo, incollando `runs.csv` e `groups.csv` in nuove tab. Le tab attuali restano come storico.

## 10. Verifica prima di fidarsi dei numeri

Pochi controlli mirati, senza campagne di test:

1. **Conversione IMU:** `az` da fermo ≈ 9.81 m/s² dopo la conversione.
2. **Coerenza vecchio/nuovo:** dove la definizione non cambia, il valore coincide con `campaign_results.csv`.
3. **Casi noti:** P004-R015 passa da ~45% a ~99% usabile, P003-R009 resta critica, P004-R014/R015 hanno `pose_valid = false`, P001-R008 ha jerk in movimento molto più alto del vecchio 0.315.
4. **Ricalcolo identico:** due esecuzioni danno lo stesso `runs.csv`.

## 11. Run da ricalcolare

| Gruppo | Run | Stato |
|---|---|---|
| G1 | P001-R001-20260921T145508, P001-R002-20260921T145711, P001-R003-20260921T145854, P001-R004-20260921T150023, P001-R005-20260921T150157 | dati nel repo |
| G2 | P002-R001-20260922T142524, P002-R002-20260922T142712, P002-R003-20260922T142859, P002-R004-20260922T143058, P002-R005-20260922T143244 | dati nel repo |
| G3 | P003-R004-20260928T165409, P003-R005-20260928T165545, P003-R006-20260928T165817, P003-R009-20260928T170138, P003-R010-20260928T170337 | dati nel repo |
| G4 | P004-R007-20260924T134658, P004-R008-20260924T134911, P004-R009-20260924T135136, P004-R010-20260924T135341, P004-R011-20260924T135544 | dati nel repo |
| G5 | P005-R003-20260924T140430, P005-R005-20260924T141303, P005-R006-20260924T141529, P005-R007-20260924T141924, P005-R008-20260924T142157 | dati nel repo |
| G6 | P001-R007-20260930T173000, P001-R008-20260930T173125, P001-R009-20260930T173424, P001-R010-20260930T173632, P001-R011-20260930T173814 | dati nel repo |
| G7 | P002-R023-20261020T185108, P002-R024-20261020T185329, P002-R025-20261020T185553, P002-R026-20261020T185812, P002-R027-20261020T190051 | **nome trovato, cartella da cercare sulla Orin** |
| G9 | P004-R014-20261001T112202, P004-R015-20261001T112447, P004-R016-20261001T112933, P004-R017-20261001T113120, P004-R018-20261001T113259 | dati nel repo |
| G10 | **da identificare** tra P005-R010…R021 (22–23/10) | D6 |
| G11 | P006-R002-20261027T141233, P006-R003-20261027T141517, P006-R004-20261027T141736, P008-R001-20261027T133550, P008-R002-20261027T133837 | cartelle da cercare sulla Orin |
| G12 | P007-R001-20261027T165917, P007-R002-20261027T170135, P007-R003-20261027T170317, P007-R004-20261027T171737, P007-R005-20261028T131845 | cartelle da cercare sulla Orin |

Note:
- P002-R023…R027: i nomi vengono da `campaign_results.csv`, dove però le righe non hanno metriche. Il foglio riporta valori per questi "23–26" (con il 26 ripetuto, probabilmente 27), ma non sono ancora verificabili.
- `run_selection.yaml` usa per M02 R001–R005 la sessione dell'8/10. Il tuo elenco usa quella del 22/09: per la v2 vale il tuo elenco.

## 12. Decisioni prese

- **D1, approach the wall:** `task_ok` a 0.5 ± 0.2 m; `exec_ok` rispetto al valore del piano; il piano a 2.0 m è un errore di traduzione.
- **D2, turn at the wall:** `exec_ok` rispetto al piano; `task_ok` a 1.0 ± 0.2 m.
- **D3, jerk:** cutoff a 2 Hz, verificato con lo spettro su 3–4 run e segnalato se lo spettro lo smentisce.
- **D4, P006 e P008:** separate come M06 e M08.
- **D5, distanza dal target:** da `tracks.jsonl`; se manca, `null` e giudizio manuale con prova.
- **D6, P005 con ostacolo:** gli ID e il flag si segnano in `selection.yaml`.
- **D7, abort al timeout:** `exec_ok = false`; `task_ok` calcolato a parte dalla svolta misurata.
