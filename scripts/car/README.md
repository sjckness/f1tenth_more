# scripts/car: build, deploy e avvio dello stack sull'Orin

Tre comandi da linus, due sull'Orin. Ogni script ha `-h`.

```
linus                                        Orin (shell dell'host, non nel container)
-----                                        -----------------------------------------
scripts/car/build.sh   [COMMIT]              car-container start | stop | status
scripts/car/deploy.sh  [COMMIT]              car-stack start [PROFILO] | stop | restart [PROFILO]
scripts/car/ready.sh   [--profile P]                   | status | profiles
scripts/car/profiles.sh push | diff | list
```

## Flusso

1. **`build.sh`** prepara l'immagine per COMMIT (default HEAD) senza toccare l'auto. Se sull'Orin c'è
   già un'immagine con lo stesso contenuto, la riusa; altrimenti `tools/f1t build` la costruisce
   sull'Orin, con i test e la label `tested`. Facoltativo: `deploy.sh` lo fa da sé.
2. **`deploy.sh`** porta COMMIT sull'Orin, in quest'ordine:
   1. l'immagine, fissata con `tools/f1t deploy` (health check e rollback automatico)
   2. `car-stack`, `car-container` e `car-preflight` in `~/.local/bin`, dallo stesso commit
   3. i profili

   Alla fine verifica che l'Orin coincida con linus. Lo stack non parte.
3. **`ready.sh [--profile P]`** risponde READY o NOT READY: READY se e solo se nessun controllo è
   FAIL. Controlla:
   - il container: sano, con l'immagine giusta
   - gli script dell'Orin, uguali a quelli di COMMIT
   - che non ci sia uno stack già acceso
   - i sensori (ZED su USB3, lidar, VESC)
   - `stack_args` e il profilo: uguale a linus e valido nel container
   - disco e temperatura

   Fa solo letture. Quello che si misura solo con lo stack acceso (frequenze, TF, e-stop, batteria)
   lo verifica `car-stack start`.
4. **Sull'Orin:** `car-stack start test_orin_rrt` avvia lo stack con il profilo e aspetta il READY
   di f1tenth_ctl. `car-stack stop` lo ferma. Il container resta su.

## Profili

Un profilo è `src/f1tenth_bringup/config/profiles/<nome>.yaml`. Sul comando si scrive il nome senza
`.yaml`.

```yaml
launch_arguments:          # argomenti di supervisor_bringup.launch.py (default: stack_params.yaml)
  planner_controller_mode: corridor_rrt
  enable_mission_logger: 'true'
component_args:            # {componente di components.yaml: {argomento del suo launch file: valore}}
  perception: {yolo_backend: engine}
```

- **Regole:** `src/f1tenth_bringup/f1tenth_bringup/launch_profile.py`.
  - Ogni nome che il profilo usa deve esistere, altrimenti lo stack non parte. Prima veniva
    ignorato in silenzio.
  - Gli argomenti che il supervisor imposta da sé (`calibration`, `enable_mission_logger`,
    `planner_controller_mode`, gli `*_impl`) vanno in `launch_arguments`.
  - I valori sono stringhe, mai `on`/`off`/`yes`/`no`.
- **Dove li legge l'Orin:** da `~/.config/f1tenth_car/profiles/`, non dall'immagine. Quella
  cartella è montata in sola lettura nel container e viene riletta a ogni avvio dello stack. Per
  cambiare un profilo:
  1. modificalo su linus e committalo
  2. `scripts/car/profiles.sh push`
  3. sull'Orin, `car-stack restart <nome>`

  Non servono rebuild né riavvio del container. Un commit che cambia solo profili non chiede una
  nuova immagine.
- **Tracciabilità:** lo stack.log registra nome, percorso e sha256 del profilo usato.
  `car-stack status` lo mostra.
- **Come arriva allo stack:** `car-stack start P` scrive in `~/.config/f1tenth_car/stack_args` i
  token `profile:=P profiles_dir:=...` e lascia com'erano gli altri token. Un token in
  `stack_args` vince sul profilo, e `ready.sh` lo segnala come WARN.

## Test

`scripts/car/tests/car_scripts_test.sh`: niente rete e niente ssh. Copre:

- shellcheck
- contenuto dell'immagine
- riscrittura di `stack_args`
- controlli di ready su fatti finti
- argomenti

I profili sono testati da `colcon test --packages-select f1tenth_bringup`
(`test_launch_profile.py`).

Gli script sostituiti stanno in `scripts/old/`.
