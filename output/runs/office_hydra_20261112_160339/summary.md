# Hydra sull'ufficio: run registrata del 2026-11-12

Prima run documentata di Hydra sui dati veri di Isaac: percorso scritto a mano e **validato contro
la geometria del mondo prima di guidare**, maschera semantica e grafo registrati dall'inizio alla
fine. Isaac Sim nativo su Windows, Hydra in WSL2, `seg_source:=gt`, `pose_source:=isaac_gt`.

## Risultato

| | |
|---|---|
| Percorso | **24,90 m** in 92 s di guida, 8 segmenti, **nessuna guardia scattata** |
| Grafo finale | **1036 nodi**, 1002 archi interlayer |
| PLACES | 74 -> **932** (3102 archi) |
| AGENTS | 0 -> **70** (69 archi) |
| ROOMS | 1 -> **5** |
| MESH_PLACES | 8 -> 28 |
| OBJECTS | **0**, per progetto (vedi sotto) |
| Bag | 2,4 GiB mcap, **33.420 messaggi**, 154 s, 10 topic tutti non vuoti |
| Maschere | 106 PNG, id presenti: **0, 1, 2** |
| Snapshot grafo | 12 (`scene_graph/dsg_t*.sparkdsg` + `dsg_final.sparkdsg`) |
| PhysX | 0 `Invalid PhysX transform`: la macchina e' rimasta integra |

`graph.png` mostra la traiettoria in rosso: la discesa dritta e poi **due cerchi sovrapposti**, i
due giri da 360°. Il grafo e la guida concordano, il che e' una verifica indipendente.

## Il video: `what_hydra_saw.mp4`

**154 s a 10 fps, cioe' tempo reale, 1280x360, H.264, 4,9 MB.** A sinistra la telecamera, a destra
la maschera semantica GT che `seg_source:=gt` da' in pasto a Hydra, con il tempo e la legenda degli
id presenti. I due flussi sono appaiati per timestamp piu' vicino (errore peggiore 100 ms, cioe' un
fotogramma: a 0,3 m/s sono 3 cm).

Generato da `scripts/hydra/bag_to_video.py <run_dir>`, che legge il bag: i PNG in `mask/` sono uno
al secondo, abbastanza per un'occhiata e troppo pochi da guardare. Non servono ne' il sim ne'
Hydra, solo il bag su disco.

Guardarlo e' il modo piu' rapido di capire la sezione qui sotto: scrivanie e sedie si vedono
benissimo a sinistra, e a destra sono **nere**.

## La domanda a cui la run risponde: Hydra rivede o duplica?

Il secondo giro percorre **lo stesso terreno** del primo, con lo stesso movimento. Se Hydra
duplicasse lo spazio gia' mappato, PLACES crescerebbe allo stesso ritmo.

| snapshot | percorso | PLACES | |
|---|---|---|---|
| t00034 | 9,7 m | 337 | discesa |
| t00045 | 12,7 m | 487 | |
| t00055 | 15,8 m | 753 | primo giro |
| **t00065** | **18,5 m** | **898** | **fine primo giro** |
| t00075 | 21,2 m | 919 | secondo giro |
| t00086 | 24,4 m | 893 | |
| finale | 24,9 m | 932 | |

**Primo giro: +561 place. Secondo giro: +34.** Sedici volte meno, e il conteggio oscilla
(919 -> 893 -> 932) invece di salire. Hydra **riconosce** lo spazio gia' mappato. ROOMS e' passato
da 1 a 5 proprio durante il primo giro: e' la rotazione sul posto che costruisce le stanze, non i
metri percorsi.

## Perche' OBJECTS e' 0, in una immagine

`mask/mask_t00030.png` lo spiega meglio di qualsiasi paragrafo: arancione il muro (id 2), blu il
pavimento (id 1), e **le sagome nere sono scrivanie, sedie, monitor e una pianta**. I mobili ci
sono, Hydra li vede e li integra nella mesh, ma sono tutti `unlabeled` (id 0), che
`src/f1tenth_hydra/config/labels/isaac_to_ade20k_mp3d.yaml` manda a misc (38), commentato
"integrated, **never an object**".

Non e' un difetto di Hydra ne' della pipeline: e' la scena `office` che non etichetta i suoi
oggetti. Il suo blocco `semantics:` conosce solo `{unlabeled, floor, wall, person, car, glass,
door}`, e l'istogramma delle maschere conferma che in tutta la run sono arrivati **solo gli id
0, 1 e 2**. Per esercitare il ramo oggetti serve la scena `base_room`, la sola che etichetta
`potted plant` e `box`.

## Il percorso, e perche' non ha un ritorno

`sim/isaac/config/routes/office_corridor_loop.json`, validato da
`sim/isaac/tools/check_drive_path.py` (margine peggiore 0,54 m, che e' la posa di spawn della
scena: niente sul percorso e' piu' stretto di dove la macchina nasce).

Una versione precedente risaliva il corridoio in retromarcia. **Ha divergito**: il validatore
prevedeva x = 17,03 a fine discesa, la macchina reale era a **x = 17,47**, cioe' **0,44 m di deriva
ad anello aperto su 10 m**. A y = -8 la corsia libera finisce a 17,76, quindi con un'impronta di
0,30 m stava strisciando il muro, e un'auto che spinge un muro va in NaN
(`car_spec.yaml:68`: "unbounded wheel torque until PhysX goes NaN").

La deriva (0,44 m) e' **piu' grande del gioco che la strozzatura lascia** (0,25 m). Un elenco di
segmenti senza retroazione puo' fare la discesa ma non il ritorno: servirebbe un controllo sulla
posizione laterale, non durate piu' precise. Al suo posto, il secondo giro da' la rivisitazione.

## Come rifarla

```bat
rem Windows
scripts\sim\win\hil_sim_wsl.bat
python sim\isaac\tools\check_drive_path.py --json output\hil\path_check_office_corridor_loop.json
```
```bash
# WSL
scripts/hydra/hydra_run.sh --path-check /mnt/f/.../path_check_office_corridor_loop.json
```

La validazione gira su Windows perche' le mesh del mondo stanno li' (il submodule
`src/husarion_gz_worlds` piu' la cache Fuel); in WSL la griglia verrebbe vuota, e lo strumento in
quel caso **esce con errore** invece di emettere un verdetto senza valore.

## Dove sono i dati

Pesanti in WSL: `~/f1tenth_sim_archive/office_hydra_20261112_160339/` (2,5 GB: bag, 106 maschere,
12 snapshot, i log di Hydra). **Mai in `/tmp`**, che questa distro ripulisce, e mai su `/mnt/f`,
che e' un mount 9p.

Qui nel repo 5 MB: il video (4,9 MB) piu' `graph.png`, `graph_summary.txt`, `run.json`,
`path_check.json`, `bag_info.txt`, `hydra_cost_summary.md` e quattro maschere. Il video e' l'unico
file pesante: se non lo si vuole in git, si rigenera in 14 s dal bag con `bag_to_video.py`.

Aprire il grafo: `scripts/hydra/open_graph.sh ~/f1tenth_sim_archive/office_hydra_20261112_160339`

## Note per la prossima volta

- **Il bag non si chiude da solo.** Il recorder non reagisce al SIGINT entro 45 s e viene ucciso:
  l'mcap resta **completo e leggibile**, ma senza `metadata.yaml`, e `ros2 bag info` sulla cartella
  dice "Could not find metadata", che sembra una registrazione persa e non lo e'. `hydra_run.sh`
  ora fa `ros2 bag reindex` da solo. Resta da capire perche' il segnale non arrivi.
- `--compression-mode file` e' stato tolto: comprimere 2 GB alla chiusura supera qualsiasi attesa
  ragionevole, e il primo bag ne e' uscito illeggibile.
- Hydra scrive il grafo **solo su SIGINT pulito**: vanno lasciati ~30 s.
