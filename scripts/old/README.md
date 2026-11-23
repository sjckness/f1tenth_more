# scripts/old: script sostituiti da scripts/car/ (2026-11-23)

Questi script sono tenuti solo come riferimento: non vengono più usati né testati. Molti cercano le
loro librerie in `scripts/lib/`, che è stata spostata, quindi possono non partire più.

| Vecchio | Sostituito da |
|---|---|
| `orin_deploy.sh` | `scripts/car/build.sh` (immagine) + `scripts/car/deploy.sh` |
| `orin_precheck.sh` | `scripts/car/ready.sh --profile campaign3_rrt` (solo letture), poi `car-stack start campaign3_rrt` |
| `orin_run_tests.sh` (`~/.local/bin/orin_run_tests`) | `car-stack start campaign3_rrt` / `car-stack stop` / `car-stack status` |
| `orin/container_up.sh` | `car-container start` |
| `orin/container_down.sh` | `car-container stop` |
| `orin/stack_live.sh` | `car-stack start [PROFILO]`. `--trt` diventa un profilo con `component_args: {perception: {yolo_backend: engine}}`; `--rust` diventa gli `*_impl` in `launch_arguments` |
| `orin/stack_stop.sh` | `car-stack stop` |
| `lib/orin_summary.sh` | i controlli di `scripts/car/lib/car_ready.sh` |

`~/.local/bin/orin_run_tests` resta sull'Orin finché qualcuno non lo cancella. Il nuovo deploy non lo
aggiorna più.
