### Stream B4 + B5 on `pwa-control` (`~/dev_ws/f1tenth_more_pwa`, head `87e6f69`)

**Commits** (nothing pushed; `output/pwa_screens/browser_disconnected.png` is still untracked):
- `306782f apps/pwa: rename the Cockpit view to lamacchinina` (B4)
- `3accf86 f1tenth_gateway: CPU per core, GPU % and pinned processes in the status message` (B5)
- `2368a2d apps/pwa: GPU tile and a per-core CPU view with the pinned processes` (B5)
- `cf9e5ab f1tenth_gateway: review fixes - nvidia-smi hang, exiting processes, interpreter options`
- `87e6f69 apps/pwa: review fixes - CPU view colours, chip names, 320 px tiles and tab`

#### B4: "Cockpit" is now "lamacchinina"
- **Renamed:** the visible name in:
  - the tab (`apps/pwa/src/components/ui.tsx:54`);
  - the h1 (`Cockpit.tsx:62`);
  - Guida's back link and its aria-label (`Guida.tsx:102,124`);
  - the SPEC (`docs/pwa/SPEC.md:52-57,81`);
  - the design mockups;
  - the gateway README's troubleshooting table.
- **E2E:** the heading checks use `exact: true` (`apps/pwa/e2e/app.spec.ts:16,22`).
- **Kept on purpose:** route `#/cockpit`, `start_url`, `Cockpit.tsx`, and the test and screenshot names.
- **Review fix:** at 320 px the tab label ran past the screen edge. Below 360 px the first tab is now 1.15 columns wide and the labels are 10 px: "lamacchinina" is 64 px in a 70 px tab (`styles.css:122`). 390 px is unchanged (70 px in 76 px).

#### B5, gateway
**`src/f1tenth_gateway/f1tenth_gateway/sysstats.py`** reads only `/proc` and sysfs: no jtop, no tegrastats, no sudo. It is Python 3.10 compatible.

**CPU**
- Total and per-core % from `/proc/stat` deltas (`parse_proc_stat` :95). iowait counts as idle; guest is not counted twice.
- Cores come from `cpu/present`, online/offline from `cpu/online`. An offline core has `percent: null`.

**GPU**
- The load file is per-mille and converted to %. Fallback order in `GPU_LOAD_PATHS` (:47) and `_find_gpu` (:316).
- **Paths on the Orin, read-only over ssh (L4T R36.5, 5.15.185-tegra):**
  - load: `/sys/devices/platform/bus@0/17000000.gpu/load`. The same file is also at `/sys/devices/platform/17000000.gpu/load` and `/sys/devices/platform/gpu.0/load`. `/sys/devices/gpu.0/load` does not exist there.
  - frequency: `/sys/class/devfreq/17000000.gpu/cur_freq`, 306 MHz when idle. `max_freq` is 816 MHz, the power mode's cap; the available steps go up to 1300.5 MHz.
- **On linus:** `nvidia-smi` runs in a pool thread, at most once per sample, with a 1 s timeout. A timeout is retried after 30 s.
  - **Review fix:** `_nvidia_smi` (:363) now uses Popen. It kills the child at the timeout and waits at most 0.5 s. Before, `subprocess.run` waited forever, so an nvidia-smi stuck in the driver hung the sample and a pool thread that the e-stop also uses. A child that does not die is abandoned, and nvidia-smi stays off for the rest of the run (:381).

**Pinned processes** (`scan_procs` :206), every 5 s
- A process counts as pinned when its `Cpus_allowed_list` leaves out an online core.
- Each one carries a pid, a name (`friendly_name` :156), its cores (`"10-11"`) and its own CPU % (100 = one core).
- Every other process only counts toward `unpinned`.
- Kernel threads are counted apart (:228). This departs from the spec: the idle Orin has 221 kernel threads, 123 of them pinned per CPU.
- **Review fixes:**
  - zombies, and processes exiting with their command line already gone, are skipped (:225, :240);
  - `python3 -W ignore x.py` is now named `x.py`, and `sh -c '<cmd>'` takes the command's first word instead of the whole string (:58-59).

**Wiring and mock**
- `Gateway.sysstats_loop` (`app.py:504`) samples at 1 Hz in the pool, only while a phone is connected.
- The sample goes out as `status.system` (`app.py:262`); `cpu_percent` is the gateway's own reading (:261). There is no new endpoint.
- `MockSysStats` (`mock.py:569`) reports 12 cores, a GPU and the stack's pins (`MOCK_PINS` :560).

#### B5, PWA
- **lamacchinina:** the number row has four tiles, VEL · VESC · **CPU ›** · **GPU** (`Cockpit.tsx:123`).
- **Telemetria:** CPU and GPU tiles at the top (`Telemetria.tsx:235`).
- **GPU tile:** the % with the MHz under it; `n/a` when the gateway reads no GPU, `—` before the first sample.
- **Review fix (320 px):** the 4-tile row keeps "1300 MHz" inside the GPU tile, and no tile label wraps.

**CPU view** (`apps/pwa/src/screens/Cpu.tsx`, route `#/cpu`)
- **Bars:** one per core, at most 4 per row (:104). Grey below 60 %, amber 60-85 %, red from 85 % (`loadColour` :62). Offline cores are dashed.
- **Chips:** above each core, one chip per process pinned to it. A process on several cores appears on each.
- **Legend:** each process with its colour (the same as its chips), its cores and its CPU %, then the line "N processi non vincolati" (:151).
- **Back** (:69): returns to the view the tile was tapped in. Opened by URL, it goes to lamacchinina.
- **Review fixes:**
  - **Palette** (:14): six hues, without the yellow and red that looked like the load states. They pass the dataviz validator on `#13161B`. Processes 7-12 get a dashed border, 13-18 dotted.
  - **Slots** (`assignSlots` :24): freed when a process is gone, so live processes never share a style. Before, slots were never freed, and restarts used up the palette.
  - **Chip names** (`chipLabel` :49): shortened at an underscore (`ekf_global`, `slam`, `yolo`). At 390 px none is cut.
  - **Note:** the kernel-thread count is shown under the legend (:153).
- **Docs:** `docs/pwa/SPEC.md:90` describes the view. The `Main.dc.html` mockup now shows the 4-tile row.

#### How it was verified
- **Gateway, Jazzy venv:** 102 passed, 3 skipped, including flake8 and pep257. The skips are `test_ros_bridge`, which needs `f1tenth_messages`.
- **Gateway, Humble / Python 3.10 (docker `f1tenth_more:cpu`):** 105 passed.
- **Unit tests:** `test/test_sysstats.py` has 7 tests on real Orin fixture files. They cover:
  - the cpu-list format (`0-3,6`);
  - `/proc/stat`, including iowait and guest;
  - offline cores;
  - the GPU load file and the path order;
  - processes that vanish, are zombies or are exiting;
  - names, including interpreter options;
  - nvidia-smi: read, timed out, wedged.
- **Wiring test:** `test_app_mock.py:208`.
- **Mutations:** 10 of 10 caught on a scratch copy.
- **On the Orin, read-only:** the new module was piped over ssh. Results: GPU source = the bus@0 path, 12 cores, 0 pinned (stack down), 198 unpinned, 221 kernel threads.
- **PWA:**
  - `npm run build` passed;
  - Playwright webkit-portrait + webkit-landscape: **27 passed**;
  - WebKit probe on my own mock gateway (port 8793) with 12 pins injected: 12 distinct styles; when one pin was replaced, the newcomer took the freed slot and nothing else changed; no overflow at 390 or 320 px.
  - Screenshots: `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_B45/fix/fix_*.png`

#### Not verified / open
- **Not run on the car with the stack up.** Check that the CPU view lists ekf 0-1, slam 2, foxglove 3, behavior 4, detection/obstacle 6-7, yolo 8-9 and mpc_corr 10-11, and that the GPU % moves off 0.
- **Not run:** the `screens` project (the committed screenshots are stale: "Cockpit", 3 tiles) and chromium-multitouch.
- **Mock gateway on :8770** (PID 3095461) runs yesterday's code. It sends no `system`, so its GPU tile shows "—" until it is restarted.
- **At 320 px** chip names are still cut with an ellipsis; the legend has them in full.
- **Docs:** there is no mockup for the CPU view.

#### Redeploy on the car
- **`f1tenth_gateway`:** restart it on the new code. No new packages are needed.
- **`apps/pwa`:** build and deploy the dist.