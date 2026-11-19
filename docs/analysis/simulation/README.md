# Area: simulation (Simulazione)

Gazebo Harmonic, Isaac Sim, sim a due macchine, HIL con l'Orin, sim2real.

## 1. Stato attuale

> Isaac è il simulatore di riferimento (e2e 9/9); l'HIL gira sulla work con Isaac nativo su Windows, l'Orin nel giro resta senza risultati.

- Gazebo Harmonic portato su Jazzy: RTF 1.00 headless, drift odometria 2.9 % su 9.70 m.
- Sim a due macchine: `/clock` throttle a 250 Hz e immagini compresse (RGB 21.4 KB a circa 26 Hz) dopo i problemi di fan-out e banda.
- Isaac: e2e 9/9 PASS sullo stack jazzy-campaign (vedi [platform](../platform/README.md)); `sim_next` porta RTF GUI da 0.57 a 1.00 e il planner LLM end to end (3 comandi su 6 ok nella base room).
- HIL Isaac + Orin: configurazione scritta, link diretto bloccato, task 3-7 non iniziati (stato al 2026-11-10).
- HIL sulla work (2026-11-13): Isaac Sim 5.1 gira **nativo** su Windows, perche' in Docker Desktop/WSL2 non vede la GPU (viene esposto `/dev/dxg`, non `/dev/nvidia0`); link 192.168.200.0/24 su Ethernet 2, domain 44, Discovery Server dell'Orin su :11812.
- Hydra in WSL2 contro Isaac su Windows funziona dal 2026-11-12: serviva una sola regola firewall su `172.16.0.0/12` (l'IP della VM WSL cambia a ogni riavvio). Giro buono 120 s / 15.32 m, 638 nodi, zero errori GVD.
- Hydra sulla scena `office`: 24.90 m in 92 s, grafo finale 1036 nodi, PLACES 74 -> 932, ROOMS 1 -> 5. Il secondo giro sullo stesso terreno aggiunge 34 place contro i 561 del primo: riconosce invece di duplicare. `OBJECTS` resta 0 perche' `office` etichetta solo gli id 0/1/2 (i mobili sono `unlabeled`).
- Costo di Hydra misurato: `hydra_ros_node` 633 % CPU medio (max 867) e 8.5 GB RSS su 166 campioni; GPU 38 % media.
- Un percorso ad anello aperto non regge il ritorno: 0.44 m di deriva su 10 m contro 0.25 m di gioco alla strozzatura.
- Fedeltà Isaac: auto parcheggiata che scivola a 6.5 mm/s, `/scan` senza intensità (il detector del vetro non è esercitabile).

## 2. Decisioni prese

- Sim headless, sterzo clampato al range reale, IMU `frame_id` vuoto come l'auto ([2026-10-04_gazebo-harmonic-port](2026-10-04_gazebo-harmonic-port/report.md))
- `/clock` throttle a 250 Hz, passo fisico 4 ms ([2026-10-06_sim-clock-fanout](2026-10-06_sim-clock-fanout/report.md))
- Compressione delle immagini su linus, decompressione sul Thor ([2026-10-07_sim-camera](2026-10-07_sim-camera/report.md))
- HIL su `jazzy-doors` 698faa14, Discovery Server 11812 ([2026-11-10_hil-isaac-orin](2026-11-10_hil-isaac-orin/report.md))
- Isaac nativo su Windows, non in Docker; `flow_controller_descriptors` fuori dal profilo DDS dell'HIL (invalido in Jazzy, faceva ignorare tutto il profilo e portava l'RTF a 0.44) ([2026-11-13_hil-windows-wsl](2026-11-13_hil-windows-wsl/report.md))
- Per esercitare il ramo oggetti di Hydra serve la scena `base_room`, la sola che etichetta `potted plant` e `box` ([2026-11-12_office-hydra-run](2026-11-12_office-hydra-run/report.md))

## 3. Domande aperte / prossimi esperimenti

- Link diretto linus-Orin per l'HIL; mondo office mai generato.
- RTF Isaac 0.72-0.77 con lo stack del Thor connesso.
- Passo del veicolo 0.325 contro 0.305 m tra URDF e odometria.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-11-13 | [HIL sulla work: Isaac Sim nativo su Windows, stack dell'auto sull'Orin, Hydra in WSL2](2026-11-13_hil-windows-wsl/report.md) | open | Eseguito. Isaac 5.1 nativo da F:/envs/isaac; link 192.168.200.0/24 su Ethernet 2 (Intel X710, non la NDIS del BMC), domain 44, Discovery Server dell'Orin su :11812. Hydra in WSL2 funziona dal 2026-11-12 con una sola reg… | f1tenth_sim 0.1.0, f1tenth_bringup 0.1.0, f1tenth_params 0.1.0 |
| 2026-11-12 | [Hydra sulla scena office: prima run registrata e validata (24,9 m, 1036 nodi)](2026-11-12_office-hydra-run/report.md) | open | 24,90 m in 92 s, 8 segmenti, nessuna guardia scattata; grafo finale 1036 nodi / 1002 archi interlayer, PLACES 74 -> 932, ROOMS 1 -> 5, AGENTS 0 -> 70. Primo giro +561 place, secondo +34 (16 volte meno): Hydra riconosce.… | f1tenth_sim 0.1.0, f1tenth_bringup 0.1.0, f1tenth_params 0.1.0 |
| 2026-11-10 | [HIL: Isaac Sim sulla workstation, stack dell'auto sull'Orin](2026-11-10_hil-isaac-orin/report.md) | open | Non ancora eseguito: handoff 2026-11-11 con survey fatto, link bloccato (10.42.60.1 non risponde ad ARP) e task 3-7 non iniziati; mondo office mai generato. | f1tenth_sim 0.0.0, f1tenth_bringup 0.0.1, f1tenth_params 0.0.1, f1tenth_perception 0.0.1 |
| 2026-10-16 | [sim_next: fix render Isaac, stanza campagna, planner LLM end to end, office](2026-10-16_sim-next-isaac/report.md) | open | RTF GUI 0.57 -> 1.00; persona 0.577 x 0.420 x 1.788 m; NaN 1543 -> 0 con limite coppia 2 N m; planner 3 comandi su 6 ok nella base room (go 3 m, go to person gap 0.84/0.82 m, 2 m poi persona) | f1tenth_sim 0.0.0, llm 0.0.1, f1tenth_perception 0.0.1, f1tenth_bringup 0.0.1 |
| 2026-10-07 | [ZED simulata su due macchine: compressione dei frame](2026-10-07_sim-camera/report.md) | done | Il Thor riceveva 3 frame in 60 s (0.05 Hz) su /camera/image_raw; con compressione RGB 675 KB -> 21.4 KB (~31x, ~26 Hz), depth 921 KB -> 20.1 KB (~45x, ~25 Hz). | f1tenth_sim 0.0.0, f1tenth_perception 0.0.1 |
| 2026-10-06 | [Clock simulato instabile nel sim a due macchine: fan-out /clock, throttle e RTF](2026-10-06_sim-clock-fanout/report.md) | open | Causa: fan-out /clock 1000 Hz x ~40 reader; con passo 4 ms (250 Hz) 1250 msg/5 s, max_gap <= 35 ms. Resta RTF instabile (min 0.05, max 1.40) per render ZED su linus | f1tenth_sim 0.0.0, f1tenth_bringup 0.0.1, f1tenth_perception 0.0.1 |
| 2026-10-04 | [Port f1tenth_sim da Ignition Fortress a Gazebo Harmonic (ROS 2 Jazzy)](2026-10-04_gazebo-harmonic-port/report.md) | done | Step 0-5 completati; verifica TPad 12/12: /scan 40 Hz, imu/odom 50 Hz, ranges[540] 4.830 m, test 37 su 37 ok; RTF 1.00 headless, 0.74 GUI su Iris Xe. | f1tenth_sim 0.0.0, f1tenth_description 0.0.0, f1tenth_hardware 0.0.1 |
<!-- AUTO:END -->
