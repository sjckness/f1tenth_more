# Macchine: stato dei cloni di f1tenth_more

Rilievo e pulizia per macchina. Una sezione per host.

## tpad

Data: 2026-11-18. Eseguito in locale sul tpad (hostname `TPad`).

### Rilievo (prima della pulizia)

- Repo: `/home/andreas/dev_ws/f1tenth_more`, remoto `origin` = `git@github.com:sjckness/f1tenth_more.git`
- Branch corrente: `jazzy` (70dc09fa, 85 commit dietro `origin/jazzy`); `main` locale assente
- Branch locali: solo `jazzy`, 0 commit fuori da `origin/*` e dai tag
- Stash: nessuno. Worktree: solo quello principale
- Altri cloni di f1tenth_more in `~` (profondità 4): nessuno
- Processi nella cartella: VS Code, la sessione Claude Code, gitstatus (cartella in uso, non cancellata, solo riallineata)
- Dimensione `.git`: 285 MiB (pack 257.6 MiB)

| elemento | classe | cosa c'è di unico | azione |
|---|---|---|---|
| branch `jazzy` | pulito | niente | `git branch -D` |
| `M .claude/settings.json` | unico | `"enabledPlugins": {}` aggiunto | salvato nel tar, poi reset |
| `M .gitignore` | unico | riga `.sjck/` aggiunta | salvato nel tar, poi reset |
| `?? pacman/` | unico | 17 file, 132K (gioco JS con test) | salvato nel tar, rimosso dal repo |
| `.sjck/` (ignorato su jazzy) | unico | stato sjck-graph, 2.5M | salvato nel tar, lasciato su disco |
| cartella repo | in uso | aperta da VS Code e Claude Code | non cancellata |

### Azioni fatte

1. Nessun tag `archive/tpad-*`: nessun commit unico da salvare, nessun push.
2. Tar: `~/f1tenth_salvataggi/tpad-20261118.tar.gz` (sul tpad, 789 kB). Contiene anche `tpad-jazzy-worktree.diff` con il diff delle due modifiche.
3. `git fetch --all --prune` (potati `origin/llm`, `origin/scene-graph`, `origin/sick`, `origin/test-logging`), `main` creato da `origin/main` (d8093a68), `git reset --hard origin/main`.
4. `git branch -D jazzy`, `git worktree prune`. Submodule `ackermann_mux` e `vesc` riportati ai commit fissati da `main` (`git submodule update`); erano puliti e senza commit unici.
5. `git gc --prune=now`: `.git` da 291716 KiB a 288488 KiB, liberati circa 3.2 MiB.

Rimasto: `.sjck/` non tracciato nella cartella del repo (su `main` non è in `.gitignore`).
Da fare: `ackermann_mux` è un pacchetto CMake e la sua sorgente è cambiata; ricostruirlo prima di usarlo.

### Contenuto del tar

```
tpad-jazzy-worktree.diff
.claude/settings.json
.gitignore
pacman/test/t03.test.js
pacman/test/t06.test.js
pacman/test/t04.test.js
pacman/test/smoke.test.js
pacman/test/harness.js
pacman/test/t02.test.js
pacman/index.html
pacman/js/scenes.js
pacman/js/input.js
pacman/js/render.js
pacman/js/audio.js
pacman/js/cutscenes.js
pacman/js/attract.js
pacman/js/main.js
pacman/js/game.js
pacman/js/killscreen.js
pacman/js/core.js
.sjck/findings.jsonl
.sjck/out/T05.val.json
.sjck/out/T03.val.json
.sjck/out/T03.json
.sjck/out/T04.val.json
.sjck/out/T06.val.json
.sjck/out/T04.json
.sjck/out/T05.json
.sjck/out/T02.val.json
.sjck/out/T02.json
.sjck/out/REVIEW.json
.sjck/out/T01.val.json
.sjck/out/T01.json
.sjck/out/T06.json
.sjck/state.json
.sjck/watchdog_state.json
.sjck/watchdog.jsonl
.sjck/console/runs/2026-10-26T21-37-13-236Z.jsonl
.sjck/console/runs/2026-10-26T21-13-31-324Z.jsonl
.sjck/console/runs/2026-10-26T21-15-30-478Z.jsonl
.sjck/view.html
.sjck/backlog.md
.sjck/decisions/D-1793051769534-074d3c.json
.sjck/archive/pacman-baseline/index.html
.sjck/cards/T01.md
.sjck/cards/T05.md
.sjck/cards/T06.md
.sjck/cards/T04.md
.sjck/cards/T03.md
.sjck/cards/T02.md
.sjck/findings.lock
.sjck/interfaces/pacman.md
```

## work

Data: 2026-11-19. Eseguito in locale sulla work (hostname `DESKTOP-Q3F81S6`, Windows 10 Pro
19045; lo stack ROS gira nella WSL `Ubuntu-24.04`, stesso hostname, utente `andreas`).
Nessun host `work` in `~/.ssh/config` (ci sono solo `orin` e `thor`): la macchina e' questa.

### Rilievo (prima della pulizia)

Tre checkout di `f1tenth_more`, non uno:

- **Windows** `F:\Users\Andreas Steffens\dev_ws\f1tenth_hil` - `origin` =
  `https://github.com/sjckness/f1tenth_more`, piu' un secondo remoto `orin` =
  `fabiocar@192.168.200.11:dev_ws/f1tenth_hil` (non raggiungibile: ping e ssh in timeout).
  Branch `planner-modes-hil` @ `47ac7b8b`. `.git` 312 MiB.
- **WSL A** `/home/andreas/dev_ws/f1tenth_hil` - `origin` = *il checkout Windows*
  (`/mnt/f/.../dev_ws/f1tenth_hil`), non GitHub. Branch `planner-modes-hil` @ `3b39917f`
  (3 commit dietro il Windows). 648 MiB, `.git` 178 MiB. Workspace colcon costruito
  (build 79 MiB, install 15 MiB, 24 package).
- **WSL B** `/home/andreas/f1tenth_hil` - `origin` = GitHub. Branch `hil-hydra-integration`
  @ `f686d815` (gia' su `origin/planner-modes`). 709 MiB, `.git` 332 MiB. Submodule
  `third_party/hydra_ros` (11) inizializzati.

Commit fuori da `origin` e dai tag (il ref `orin/campaign-office` *non* conta: e' un
remote-tracking locale verso una macchina spenta), tutti sul checkout Windows:

| branch | commit unici | note |
|---|---|---|
| `planner-modes-hil` | 11 (`47ac7b8b`..`bd12e552`) | include `3af7d70c` corridor_rrt rrt_target_retreat |
| `hil-hydra-integration` | 1 (`bd12e552`) | e' anche dentro `planner-modes-hil` |
| `planner-modes` | 0 | `ad1ac455`, su `origin/planner-modes` |

Stash: nessuno in nessuno dei tre. Worktree: solo il principale in ciascuno.

Altre cartelle:

- `~/dev_ws/campaign.bundle` (182 MiB): bundle con un solo head,
  `refs/heads/planner-modes-hil` @ `8842c1f1`, gia' antenato di `47ac7b8b`. Niente di unico.
- `~/dev_ws/f1tenth_hil;F`: cartella vuota, artefatto di un path mangled.
- `./git` nel repo Windows: file da 0 byte, artefatto di un comando sbagliato.
- `~/work/isaac_hil/underwater_ws`, `~/rov_work`: altri progetti, non `f1tenth_more`.
- WSL `~/hydra_ws` (590 MiB): solo build/install/log colcon.
- WSL `~/projects/llm` (2.7 GiB): clone di llama.cpp, altro progetto.
- WSL `~/f1tenth_sim_archive` (7.0 GiB): solo bag (`.mcap`/`.db3`), nessun `.md`/`summary`/`.html`/`.pdf`.
- WSL `~/hydra_runs/20261112_144157` (2.5 MiB): output Hydra grezzo (dsg.json, mesh.ply, timing csv), senza report.
- WSL `~/*.py` + `f1t_apt.sh` + `build_hydra.log`: 6 script il cui blob non sta in nessun ref e
  il cui path non e' mai esistito nella storia del repo.

HIL: Isaac Sim 5.1.0 installato come venv pip in `F:\envs\isaac` (uv, CPython 3.11); GPU
`NVIDIA GeForce RTX 4090`. Nessun processo Isaac/ROS attivo sul lato Windows al momento del
rilievo; nella WSL girano `fast-discovery-server` + `python3` + `sh` con cwd in **WSL A**
(PID 472/495/497, su da 5 giorni). Docker Desktop fermo, nessun container, nessun volume.
`lsof +D`: WSL A occupata da quei tre processi; il repo Windows e' aperto da VS Code e da
questa sessione Claude Code.

File Isaac fuori da git dentro il repo Windows (ignorati, ma reali):
`sim/isaac/usd/generated/` (24 MiB, scene USD generate), `sim/isaac/config/hil.local.yaml`
(5 KiB, config locale della macchina), `scripts/sim/win/night/` (788 KiB, 15 log della
campagna notturna), `scripts/sim/win/run.log` + `run_tcp.log` (52/53 KiB),
`scripts/sim/win/run_prev.log` (51 MiB), `output/runs/.../what_hydra_saw.mp4` (4.9 MiB),
i CSV per-run e i 7 `summary.csv` di `output/planner_compare/` (gitignorati da `*.csv`).

### Classificazione

| elemento | classe | cosa c'è di unico | dim. | azione |
|---|---|---|---|---|
| checkout Windows `F:\Users\Andreas Steffens\dev_ws\f1tenth_hil` | in uso + unico | 11 commit fuori da `origin` | `.git` 312 MiB | taggato, poi `main` creato da `origin/main`; non cancellato (VS Code + sessione Claude Code) |
| branch `planner-modes-hil` (`47ac7b8b`) | unico | 11 commit, incl. `3af7d70c` RRT retreat | — | tag `archive/work-planner-modes-hil-20261119`, pushato, poi `-D` |
| branch `hil-hydra-integration` (`bd12e552`) | unico | 1 commit, già dentro il precedente | — | coperto dallo stesso tag, poi `-D` |
| branch `planner-modes` (`ad1ac455`) | pulito | niente (su `origin/planner-modes`) | — | `-D` |
| **WSL A** `/home/andreas/dev_ws/f1tenth_hil` | in uso + unico | 6 cartelle `output/planner_compare/` non tracciate (incl. le 2 prove del retreat) | 648 MiB | untracked salvato; tenuta come **unico checkout di build** della work (origin ripuntato su GitHub, remoto `work` rimosso, allineata a `main`) |
| **WSL B** `/home/andreas/f1tenth_hil` | pulito | **niente**: 7 su 8 file non tracciati byte-identici al committato, `hydra_run.sh` più vecchio, `workstation_full.yaml` identico | 709 MiB | **da cancellare** |
| WSL `~/{rrt_analyse,pc_pdf,scan_probe,kill_stack,fail_detail}.py`, `f1t_apt.sh`, `build_hydra.log` | unico | blob in nessun ref, path mai esistito | 38 KiB | nel tar |
| WSL `~/hydra_runs/20261112_144157` | unico | output Hydra grezzo, senza report | 2.5 MiB | nel tar |
| File Isaac fuori da git nel repo Windows | unico | `sim/isaac/usd/generated/`, `hil.local.yaml`, `scripts/sim/win/night/`, `run.log`, `run_tcp.log`, `what_hydra_saw.mp4`, CSV per-run e 7 `summary.csv` | ~35 MiB | nel tar (escluso `run_prev.log`, 51 MiB di stdout Isaac) |
| WSL `~/f1tenth_sim_archive` | unico | 7.0 GiB di sole bag, **nessun report dentro** | 7.0 GiB | archiviato a parte (vedi sotto), **da spostare su linus** |
| `~/dev_ws/campaign.bundle` | pulito | unico head `8842c1f1`, già antenato di `47ac7b8b` | 178 MiB | **cancellato** |
| `~/dev_ws/f1tenth_hil;F` + `./git` | pulito | cartella vuota + file da 0 byte | 0 | **cancellati** |
| WSL `~/hydra_ws` | pulito | solo build/install/log colcon | 590 MiB | **da cancellare** |
| WSL `~/projects/llm`, `~/work/isaac_hil`, `~/rov_work` | — | altri progetti | 2.7 GiB + | non toccati |

### Azioni fatte

1. Tag d'archivio su GitHub, dopo un grep segreti risultato pulito (solo prosa su una chiave ssh
   senza password e righe di log `llama-server` con la parola "token"; nessun `.env`, nessuna chiave):
   `archive/work-planner-modes-hil-20261119` (`47ac7b8b`) e `archive/work-rrt-p2p-20261119`
   (`3af7d70c`). Verificati con `git ls-remote --tags origin`.
2. Report copiati fuori dal repo **prima** di qualsiasi reset, in
   `~/f1tenth_salvataggi/work-report-20261119/` (38 MB, 736 file), poi migrati in `docs/analysis/`:
   5 esperimenti nuovi (82 totali nel database, erano 77). Registro in
   [`_inventory.md`](_inventory.md), sezione "Recupero dalla macchina work".
3. Tar degli elementi unici: `~/f1tenth_salvataggi/work-20261119.tar.gz`, **16.5 MiB, 521 voci**.
4. Bag: `zip` non è installato nella WSL, quindi `~/f1tenth_salvataggi/work-sim-archive-20261119.tar.gz`,
   **2.4 GiB** da 7.0 GiB. Da spostare su linus; l'originale in `~/f1tenth_sim_archive` è ancora lì.
5. RRT/P2P su `main`: `3af7d70c` portato con cherry-pick su `integrate/rrt-p2p` (nessun conflitto),
   `f1tenth_planners` 0.0.1 -> 0.1.0 (con `CHANGELOG.md` nuovo) e `f1tenth_params` 0.1.0 -> 0.2.0,
   tag `f1tenth_planners/v0.1.0` e `f1tenth_params/v0.2.0`, merge `--no-ff` in `main`.
   Verifica nella WSL: `colcon build --packages-above` 14 pacchetti 0 falliti (3 saltati, `vesc_driver`
   non compila senza `ros-jazzy-serial-driver`); `colcon test` 71 + 84 test, 0 fallimenti;
   `f1t selftest` 10 ok / 16 falliti, **tutti exit 141 (SIGPIPE) e identici su `main` senza il
   cherry-pick**, quindi non una regressione — da indagare a parte.
6. `git branch -D` dei 4 branch locali diversi da `main`, `git worktree prune` (nessun worktree extra).
7. `git submodule update --init --recursive`: 5 su 5 inizializzati e puliti. Nella WSL `zed_msgs`
   compila (rc=0); `zed_components` no, manca l'SDK ZED (`find_package(ZED)`): la work non ha la camera.
8. `git gc --prune=now`: `.git` da 348537 a 345984 KiB, **circa 2.5 MiB liberati**. Spazio liberato in
   totale finora: 178 MiB (il bundle) + 2.5 MiB.

### Rimasto da fare

Il classificatore della sessione ha rifiutato le cancellazioni di cartelle. Restano, tutte già
salvate (nel tar e nel tag), da cancellare a mano:

```bash
# nel checkout Windows (residui non tracciati del branch, nel tar e nel tag)
cd "F:/Users/Andreas Steffens/dev_ws/f1tenth_hil" && git clean -fdx -- sim scripts/sim/win   # 25 + 51 MiB
rm -rf "F:/Users/Andreas Steffens/dev_ws/f1tenth_hil/src/husarion_gz_worlds"                 # non è un submodule su main
# nella WSL
rm -rf /home/andreas/f1tenth_hil      # WSL B, 709 MiB, niente di unico
rm -rf /home/andreas/hydra_ws         # 590 MiB, solo build/install/log
cd /home/andreas/dev_ws/f1tenth_hil && git clean -fdx -- output/planner_compare   # ~6 MiB, migrato
```

Totale recuperabile: circa **1.4 GiB**, più i 7.0 GiB di `~/f1tenth_sim_archive` una volta che
l'archivio è su linus.

### `tools/core/hosts/work.env`: differenze dalla macchina reale

Segnalate, non corrette (il file è tutto `n/d`):

| chiave | nel file | sulla macchina |
|---|---|---|
| `HOST_HOSTNAME` | vuota | `DESKTOP-Q3F81S6` |
| `HOST_GPU_NAME` | vuota | `NVIDIA GeForce RTX 4090` |
| `HOST_USER` | vuota | `Andreas Steffens` su Windows, `andreas` nella WSL |
| `HOST_HIL_WORKTREE` | vuota | `F:\Users\Andreas Steffens\dev_ws\f1tenth_hil` (Isaac, Windows); il checkout di build è `/home/andreas/dev_ws/f1tenth_hil` nella WSL |
| `HOST_ISAAC_IMAGE` | `f1tenth_isaac:5.1.0` | **nessuna immagine**: Isaac gira nativo dal venv pip `F:\envs\isaac` (in Docker su Windows non vede la GPU) |
| `HOST_ISAAC_VERSION` | `5.1.0` | `5.1.0` (coincide) |
| `HOST_LINK_IP` / `HOST_ORIN_LINK_IP` | `10.42.60.1` / `10.42.60.2` | `192.168.200.2` / `192.168.200.11`, su `Ethernet 2` (Intel X710) |
