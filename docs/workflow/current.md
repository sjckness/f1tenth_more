# Flussi di lavoro prima di `f1t` (rilievo 2026-11-18, sola lettura)

**Build dell'immagine.** `docker/orin/build.sh [--dry-run] <commit>` gira SOLO sull'Orin (L4T r36.5: il
Dockerfile linka le librerie L4T dell'host, su linus non esistono; buildx/QEMU non usati). Sorgenti di un
commit (`stage_sources.sh`, submodule inclusi), input binari verificati (`fetch_inputs.sh`, 4.2 GB in
`/scratch/fabiocar/p5d_downloads`), tag `f1tenth:jazzy-orin-<sha7>[-norust]`, ~10 min solo sorgenti,
50-70 min a freddo. Manuale: `git bundle` su linus -> scp in `/scratch/fabiocar/xfer` -> fetch nel repo di
build `~/dev_ws/f1tenth_car` (il remote `orin` e' read-only da linus) -> checkout lì -> build staccata.
Fragile: build.sh rifiuta se uno stack gira (la build carica tutti i core ~1 h); main non ha la versione
nuova di build.sh/compose (`scripts/docker_orin.sh` esiste solo sulla linea dell'auto, `orin/mpc-dodge-car`);
nessun test dopo la build oltre a `build_checks.sh` nel Dockerfile; nessun registry.

**Deploy.** Sull'Orin, a mano: `docker_orin pin <img>` (scrive `~/.config/f1tenth/car_image`) poi
`stack_mode container` o `systemctl --user restart f1tenth-car-container.service` (unit oneshot che fa
`docker_orin up`: controlli gid/dispositivi/drift, attesa healthy 150 s, healthcheck = DS 11811 + gateway
8770). Rollback = pin dell'immagine precedente (storia in `/scratch/fabiocar/f1tenth_container/history.log`).
Fragile: nessun cancello "testata", nessun diff dei parametri, nessun `deployed.json`; i tag `pre-*` e
`prev` sono fatti a mano. Oggi gira `f1tenth:jazzy-orin-c0c1397-norust` (branch mpc-dodge-car, 150
commit non in main, 29 moduli con versione diversa da main).

**Drift sull'Orin.** Checkout nativo `~/dev_ws/f1tenth_more` (Humble, `stack-campaign-final-2026-10-22`)
con 6 file modificati; override a caldo in `~/.config/f1tenth_car/stack_args` (montato ro nel container)
con 3 backup `stack_args.bak_*` fatti a mano. `scripts/orin/drift_check.sh` confronta nativo e CAR_SYNC.

**HIL.** Setup B (branch hil-isaac, hil-hydra-integration, planner-modes): `scripts/sim/sim_isaac_hil` su
linus + `scripts/sim/hil_orin` sull'Orin, config unica `sim/isaac/config/hil.yaml` (dominio 44, DS
10.42.60.2:11812, cavo 10.42.60.0/24). **Mai eseguito**: link bloccato (10.42.60.1 non risponde ad ARP),
mondo office mai generato, `f1tenth_isaac:5.1.0` non c'e' su linus, `hil_orin` userebbe il container
dell'auto. Altri setup Isaac non HIL: A `sim_isaac.sh` linus+Thor (dominio 42, DS 10.42.0.2:11811), C
`isaac_tuning` (tutto su linus, dominio 78), D `planner_compare`, E `rust_parity/isaac`. Macchina "work":
n/d (nessun hostname/IP/percorso nel repo). Fragile: VRAM (freeparty-llm tiene 5.4 GB su 8), 20-70 s al
primo /clock, `ros2 launch` in background ignora SIGINT, `docker exec` nei script ssh vuole `</dev/null`,
`pkill -f` si uccide da solo, sysctl e IP dell'Orin si perdono al riavvio.

**Versioni e analisi.** `tools/analysis_index.py` (ora `f1t exp index`), CHANGELOG per package, tag
`<pkg>/vX.Y.Z` (non pushati), report in `docs/analysis/<area>/<id>/`. Manuale: bump, voce, tag, front
matter `modules:` scritto a mano (le versioni dell'Orin non si leggevano da nessuna parte).
