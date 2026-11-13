# Hydra sul PC (WSL2): stato e piano

Aggiornato 2026-11-11. Obiettivo: far girare Hydra in WSL2 (Ubuntu 24.04) contro Isaac Sim nativo su
Windows, senza Orin (`seg_source:=gt`, `pose_source:=isaac_gt`). Guide di riferimento:
[run_windows.md](run_windows.md) e [../../docs/hydra_hil.md](../../docs/hydra_hil.md).

---

## FATTO: Hydra gira sui dati di Isaac

La regola firewall e' stata messa il 2026-11-12 ed **era l'unica cosa che mancava**. La catena
completa funziona: Isaac Sim nativo su Windows -> Fast DDS attraverso la NAT di WSL2 -> Hydra in
Ubuntu 24.04, con `seg_source:=gt` e `pose_source:=isaac_gt`. **RViz si apre via WSLg.**

La regola in uso (amministratore, una volta; `netsh ... show rule name="..."` per verificarla):

```
netsh advfirewall firewall add rule name="HIL Hydra WSL in ingresso (UDP)" dir=in action=allow protocol=UDP localip=172.16.0.0/12 remoteip=172.16.0.0/12 profile=any
```

Usa `172.16.0.0/12` e non un indirizzo fisso perche' l'IP di WSL cambia a ogni riavvio della VM;
i profili DDS sono rigenerati a ogni avvio, quindi non c'e' nulla da aggiornare a mano. Verificato:
dopo un riavvio di WSL, `~/hydra_ws`, il fix GVD e i buffer socket a 16 MB sono sopravvissuti senza
toccare niente, e gli indirizzi sono rimasti 172.26.129.85 / 172.26.128.1.

### Il giro buono: 120 s, 15,3 m, nessun crash

Macchina guidata a 0,3 m/s dritta, a scatti di 4 s con pause, con un guardiano che interrompe se
la posa non e' piu' finita (non e' mai scattato).

| Layer | Inizio | Fine |
|---|---|---|
| PLACES | 67 / 169 | **584 / 1898** |
| AGENTS | 0 | **43 / 42** |
| ROOMS | 1 | **2** |
| BUILDINGS | 1 | 1 |
| MESH_PLACES | 8 | 8 |
| Nodi totali | | **638**, 622 archi interlayer |

173 messaggi DSG, percorso 15,32 m da (16,81, 1,11) a (17,62, -13,93), zero errori GVD. Verso i
70 s la macchina ha raggiunto il fondo del corridoio e si e' fermata contro il muro: a 0,3 m/s la
fisica ha tenuto, PLACES si e' assestato e AGENTS e' rimasto a 43.

**A macchina ferma** (giro precedente, 90 s): 117 messaggi, PLACES 70/211, ROOMS 1, AGENTS **0**, e
il grafo congelato dopo 55 s. Non e' un guasto: parcheggiata non esplora area nuova e non accumula
keyframe di pose graph. Il movimento e' il discrimine.

### OBJECTS resta 0, e **e' per progetto**

Misurato, non supposto. L'istogramma di `/sim/ground_truth/left/semantic` contiene **solo tre id**:
0 unlabeled (48,8 %), 1 floor (17,9 %), 2 wall (33,3 %). E la mappa
`src/f1tenth_hydra/config/labels/isaac_to_ade20k_mp3d.yaml` e' esplicita:

- `unlabeled: 38` -> misc, "integrated, **never an object**": i mobili dell'ufficio non hanno label;
- `person: 0` e `car: 0` -> **void**, mascherati dall'integrazione di proposito (dinamici / il
  nostro stesso corpo);
- gli oggetti di questo spazio label sono `plant` (14) e `box` (37), piu' chair/table/door/window
  "not labelled by Isaac yet".

Quindi nel mondo `office` **nessun oggetto puo' comparire**, in nessun layout: girare con il layout
`people` NON produce oggetti, perche' la persona e' void. Per esercitare il ramo oggetti serve la
scena **`base_room`**, la sola che etichetta `potted plant` e `box` (ed e' quella che il feed
sintetico dello smoke test imita, con gli id 5 e 6).

### Due trappole trovate sbagliando

**1. `/tmp` di questa distro viene ripulito.** I grafi del primo giro buono sono spariti pochi
minuti dopo: tutte le voci di `/tmp` portano lo stesso timestamp di reinizializzazione, directory
di systemd incluse. Il `log_dir:=/tmp/...` che avevo passato era l'errore: **il default di Hydra,
`~/.hydra/f1tenth/...`, sta sul disco ext4 e persiste.** Rifatto con `log_dir` sotto `$HOME`, gli
artefatti ci sono e sono utilizzabili:

```
log/backend/dsg.json            213 KB   (riletto con spark_dsg: 125 nodi, 456 archi)
log/frontend/dsg_with_mesh.json 546 KB
log/isaac_label_remap.yaml      240 B    (generato al lancio, NON mancava: era /tmp)
log/timing_stats.csv, log/{frontend,backend}/layer_*_statistics.csv, log/backend/trajectory.csv
```

**2. I grafi si salvano solo su SIGINT pulito.** Nel primo giro Hydra e' stato escalato a SIGTERM
dopo 5 s e non ha scritto nulla. Dargli circa 25 s per uscire.

### E una cosa che ho rotto

Il primo tentativo di guida (**0,8 m/s per 150 s con sterzata sinusoidale** fino a 0,18 rad, layout
`corridor`) ha fatto prendere un muro alla macchina e la sospensione articolata di PhysX e' esplosa
dopo circa 9 s:

```
[Warning] [omni.physx.plugin] Invalid PhysX transform detected for .../Wheel_Rear_Left  (e ogni altro corpo rigido)
[Error]   [omni.physx.plugin] PhysX error: Illegal BroadPhaseUpdateData
[Error]   [omni.physx.plugin] PhysX error: NpSceneQueries::raycast pose is not valid.
```

`/sim/ground_truth/odom` e' andato in `nan`, Hydra ha smesso di ricevere input e il sim e' stato
riavviato. Per confronto, il drive test del repo (`scripts/sim/hil_check` step 6, riga 339) usa
**0,3 m/s per 2,0 s, sterzata 0**, QoS RELIABLE depth 1: tre volte piu' lento e settantacinque
volte piu' breve.

**In questo setup non esiste una guida validata per esplorare.** Il drive test e' una spinta di due
secondi per vedere se la macchina si muove; a guidare dovrebbe essere il planner sull'Orin, qui
fuori dal percorso. Per un giro lungo: piano, dritto, e un guardiano sulla posa.

Altre due note operative: **la macchina non torna al punto di partenza da sola** (il giro dopo e'
partito col muso contro il muro, 0,8 m percorsi in 70 s) -- usare `/sim/reset`
(`hil_reset.bat`) tra un giro e l'altro; e `ros-jazzy-ackermann-msgs` **non era installato** in WSL,
serve per pubblicare su `/ackermann_drive`.

---

## La macchina WSL

| Cosa | Valore |
|---|---|
| WSL / kernel | 2.6.0.0 / 6.6.87.2. Niente "mirrored" (Windows 10 build 19045) |
| Distro | Ubuntu 24.04.5 LTS, `systemd=true`, utente `andreas` |
| Risorse viste da WSL | 96 CPU, 251 GB RAM, 955 GB liberi su `/` |
| IP Ubuntu (`eth0`) | **172.26.129.85/20** |
| IP Windows (`vEthernet (WSL)`) | **172.26.128.1/20** (e' anche il nameserver) |
| IPv4 verso packages.ros.org / archive.ubuntu.com / raw.githubusercontent.com | funziona (200) |
| IPv6 | **non funziona**, nessuna route di default. Per questo `getent hosts packages.ros.org` da' solo AAAA e apt perderebbe tempo in timeout |
| Firewall lato Linux | **nessuno**: niente `ufw`, il binario `iptables` non esiste. L'unico filtro in gioco e' quello di Windows |
| WSLg | `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, `/mnt/wslg` montato, socket `X0` presente. RViz nativo e' plausibile, non ancora provato con una finestra |
| `net.core.rmem_max` / `wmem_max` | partiva da **212992 (208 kB)**, 80 volte meno dei 16 MB che i profili chiedono. Alzato in `/etc/sysctl.d/99-f1tenth-dds.conf`, che con `systemd=true` sopravvive al riavvio |
| Firewall di Windows verso la rete WSL | scartava l'UDP in ingresso (misurato: 0 datagrammi su 12); **risolto** con la regola in cima |

Gli IP 172.26.x **cambiano a ogni riavvio della VM WSL**: un profilo DDS che li contiene va rigenerato
a ogni avvio, non scritto a mano una volta.

## Stato

| Cosa | Stato |
|---|---|
| Ubuntu-24.04 installato e funzionante | verificato |
| `Acquire::ForceIPv4` in `/etc/apt/apt.conf.d/99force-ipv4` | fatto |
| locale `en_US.UTF-8`, componente `universe`, repo ROS 2 + chiave | fatto |
| `ros-jazzy-desktop` 0.11.0, `rviz2` 14.1.24, `rosdep` 0.27.0, `colcon` 0.3.0 | verificato, nessun errore |
| Clone in `~/f1tenth_hil` (filesystem WSL), branch `hil-hydra-integration` | verificato, 707 MB |
| 11 submodule `third_party/hydra_ros` ai SHA attesi, `check_hydra_pins.py` | **verificato, passa** |
| `rosdep install` per le dipendenze Hydra | **verificato**: `ros-jazzy-gtsam` installato, "All system dependencies have been satisfied" |
| `build_hydra_ws.sh --jobs 12` -> `~/hydra_ws` | **verificato**: 22 pacchetti in 3 min 54 s, un solo stderr (l'avviso TensorRT atteso) |
| `spark_dsg` python bindings | verificato (serviva `python3-pip`, vedi sotto) |
| `colcon build f1tenth_hydra f1tenth_params` | verificato |
| **Smoke test Hydra** | **verificato: 2 passed**, dopo una correzione di config (vedi sotto) |
| **Hydra gira e costruisce un grafo completo** | **verificato a mano, 60 s: vedi "La prova vera"** |
| Profili DDS Isaac <-> WSL | **scritti e verificati**: `wsl.xml.in` + `hil_win.py --net wsl` + `wsl_dds_profile.sh` |
| `net.core.rmem_max` alzato a 16 MB, persistente | fatto (`/etc/sysctl.d/99-f1tenth-dds.conf`) |
| Isaac accetta il profilo e si lega agli indirizzi giusti | **verificato** (vedi sotto) |
| WSL vede i topic di Isaac | **SI**, da quando c'e' la regola firewall |
| Regola firewall | **messa** il 2026-11-12 (amministratore). Era l'unica cosa che mancava |
| Alternativa TCP (Isaac client, Hydra server) | **a meta'** e non piu' necessaria: la connessione si stabilisce, la discovery non completa. I file restano come riferimento |
| RViz in WSLg | **verificato**: `hydra.launch.py` apre la finestra RViz dalla distro |
| `hydra_dsg_viewer` | non provato |
| **Hydra contro il feed di Isaac** | **verificato**: 584 place, 43 agent, 2 room su 15,3 m guidati |
| Grafi salvati e rileggibili da disco | **verificato** con `spark_dsg` (ma NON in `/tmp`, vedi sopra) |
| OBJECTS > 0 | non ottenibile nel mondo `office`: nessuna classe oggetto etichettata. Serve `base_room` |

## Passo 5: DDS Isaac (Windows) <-> Hydra (WSL)

### Quello che funziona

- `hil_win.py sim --net wsl` rileva gli indirizzi dalla distro in esecuzione a ogni avvio
  (windows side 172.26.128.1, distro 172.26.129.85) e rende `wsl.xml.in`.
- `scripts/hydra/wsl_dds_profile.sh` rende lo stesso template specchiato sul lato WSL.
- Isaac parte sano: `RTF 1.000 (sim 10.0 s / wall 10.0 s)` e **nessun `XMLPARSER`**, quindi Fast DDS
  ha accettato il profilo invece di scartarlo.
- **Il profilo fa esattamente quello che deve.** Il processo Isaac si e' legato a:
  `172.26.128.1:18400` (discovery del dominio 44), `172.26.128.1:18410` e `:18411`
  (partecipanti unicast), piu' i corrispondenti su `127.0.0.1`, e **nient'altro**: la
  `interfaceWhiteList` e' rispettata alla lettera.

### Quello che non funziona, e la misura che lo dimostra

Da WSL, `ros2 topic list` mostra solo i propri `/parameter_events` e `/rosout`: nessun topic di
Isaac, nessun `/clock`. Nemmeno `ros2 node list`, e **nemmeno fermando il daemon** di ros2 (che
tiene in cache il grafo: era il primo sospetto, ed era sbagliato).

Due strade sbagliate, per memoria:

- La prima diagnosi elencava sette indizi circostanziali (firewall attivo, nessuna regola per
  172.26.x, `vEthernet (WSL)` senza categoria di rete...). Erano **compatibili** con il blocco, ma
  nessuno lo dimostrava.
- Poi una cattura `tcpdump` su `eth0` in WSL ha mostrato traffico in entrambe le direzioni (275
  pacchetti in uscita, 170 in entrata) e sembrava **escludere** il firewall. Anche questo era
  sbagliato: la cattura e' **dentro WSL**, quindi prova che Windows -> WSL *arriva*, ma per
  WSL -> Windows prova solo che i pacchetti **partono**. Inviato non vuol dire consegnato.

La misura decisiva: un ricevitore e un mittente UDP scritti a mano, eseguiti con **lo stesso
interprete** che usa Isaac, cosi' che il firewall veda lo stesso programma.

| Prova | Risultato |
|---|---|
| WSL -> Windows, non sollecitato, 12 datagrammi verso una porta in ascolto su Windows | **0 consegnati** |
| Windows -> WSL, poi risposta sullo stesso socket | **5/5 risposte ricevute** |

Quindi: **l'ingresso verso un programma Windows senza regola e' scartato**, mentre tutto cio' che
**Windows inizia** riceve le risposte attraverso il filtro stateful. La discovery UDP non puo'
vivere con questo, perche' il lato WSL deve per primo spedire non sollecitato alla porta
ben nota di Windows, e quel pacchetto muore.

Altro contesto utile, verificato: le regole esistenti sono per coppia di indirizzi
(192.168.200.2 con .10 e .11; 10.42.60.1 con .2) e nessuna copre 172.26.x; non esiste alcuna
regola di **blocco** che si frapponga (le `salome_session_server_*` sono regole di *allow*); e la
porta 53 funziona (e' come la distro risolve il DNS), quindi il filtro e' selettivo, non totale.

### Il dettaglio che avrebbe fatto sbagliare la regola

Il processo che apre le porte DDS **non** e' `F:\envs\isaac\Scripts\python.exe`, ma l'interprete
su cui quel venv e' costruito:

```
F:\Users\Andreas Steffens\AppData\Roaming\uv\python\cpython-3.11-windows-x86_64-none\python.exe
```

Windows filtra sull'immagine reale, quindi una regola con `program=` sul path del venv **non
aggancia nulla**. Per questo la regola proposta non usa `program=` affatto, come del resto nessuna
delle regole HIL esistenti.

### La regola (amministratore, una volta)

Segue lo stile delle altre regole HIL (coppia di indirizzi, UDP, qualsiasi porta, nessun
programma), ma usa l'intervallo privato da cui WSL pesca invece di un indirizzo fisso, perche'
**l'indirizzo cambia a ogni riavvio di WSL** e una regola su 172.26.x/32 si romperebbe. Quel
intervallo non e' raggiungibile dall'esterno: lo usano solo le VM WSL di questo PC.

```
netsh advfirewall firewall add rule name="HIL Hydra WSL in ingresso (UDP)" dir=in action=allow protocol=UDP localip=172.16.0.0/12 remoteip=172.16.0.0/12 profile=any
```

Per rimuoverla: `netsh advfirewall firewall delete rule name="HIL Hydra WSL in ingresso (UDP)"`.

Variante piu' stretta, se si preferisce limitare alle porte del dominio 44 invece che a qualsiasi
porta (si rompe se `ros.domain_id` cambia, perche' le porte sono
`7400 + 250*domain`): aggiungere `localport=18400-18500`.

### Alternativa provata senza la regola: Fast DDS su TCP (parziale)

Visto che tutto cio' che **Windows inizia** riceve risposta, l'idea e' far fare a Isaac il **client
TCP** e a Hydra il **server**: una sola connessione in uscita da Windows porta discovery e dati in
entrambi i versi, e nessuna regola di ingresso serve. Implementato in
`config/sim/dds/wsl_tcp.xml.in`, con `hil_win.py sim --net wsltcp` (client) e
`scripts/hydra/wsl_dds_profile.sh --tcp` (server).

Risultato: **il canale si apre, la discovery no.**

- il server WSL ascolta: `LISTEN 0.0.0.0:42100`;
- Isaac parte sano (`RTF 1.000`, nessun `XMLPARSER`: il profilo e' accettato);
- la connessione **si stabilisce**: `ss` mostra `ESTAB 172.26.129.85:42100 <- 172.26.128.1:55155`,
  quindi il firewall permette la connessione in uscita come previsto;
- ma dopo 180 s nessuno dei due vede i topic dell'altro: zero topic estranei.

La documentazione di eProsima e' stata letta per escludere l'errore piu' probabile, la confusione
tra porta fisica e porta logica nei locator TCP: **la configurazione corrisponde al loro esempio**
(`physical_port` uguale a `port` e' esattamente il pattern dei loro snippet; il client non deve
avere porte in ascolto; il server non ha bisogno di `initialPeersList`). Quindi il blocco e' piu'
sottile, probabilmente nell'interazione tra `rmw_fastrtps_cpp` e il trasporto TCP, e andare avanti
a tentativi sarebbe indovinare. I file restano nel repo: il canale funziona, manca il passo finale.

### Terza alternativa, non ancora provata: Discovery Server in WSL

Stessa leva (solo Windows inizia): un Fast DDS Discovery Server **in WSL**, con Isaac come client.
Isaac manda al server (Windows -> WSL, permesso), il server risponde sul flusso esistente
(stateful, permesso), e per i dati e' sempre Isaac a scrivere per primo, quindi gli ACK di Hydra
viaggiano su una mappatura gia' aperta. Riusa il macchinario che il repo ha gia'
(`ROS_DISCOVERY_SERVER`, `ros.discovery_server_*`). Serve il CLI `fastdds`, che **non e'
installato** nella distro (`command -v fastdds` non trova nulla): un `apt install` in piu'.

### Quarta alternativa, certa ma non live: registra su Windows, riproduci in WSL

Nessuna rete di mezzo. Isaac pubblica in loopback su Windows, un piccolo registratore in rclpy
serializza i topic che servono a Hydra su un file, e in WSL un lettore li ripubblica. Valida Hydra
sui **dati veri di Isaac**, che e' l'obiettivo, al prezzo di non essere interattiva e di circa
1-2 GB al minuto su disco. `rosbag2` non c'e' nel bridge di Isaac, quindi il registratore va
scritto (poche decine di righe, `rclpy.serialization`).

### Ancora da verificare, dopo la regola

Che i topic compaiano, che Hydra costruisca il grafo **dal feed Isaac** (non da quello sintetico),
e il costo sul wire delle immagini a 10 Hz. Finche' la regola non c'e', il passo 5 resta **non
verificato**.

## La prova vera (non solo il test verde)

Il test passa in 2,3 s, che e' sospetto: si interrompe al **primo** DSG che ha PLACES > 0. Per non
fidarmi di un verde cosi' rapido, Hydra e il feed sintetico sono stati lanciati a mano per 60 s su un
dominio isolato, osservando `/hydra/backend/dsg`. Risultato:

- **88 messaggi DSG in 60 s** (circa 1,5 Hz, coerente col tetto di 2 Hz del profilo `workstation_full`);
- **PLACES** cresce fino a 90-98 nodi e 310-340 archi;
- **ROOMS: 2** -- il layer stanze funziona, e dipende dalle place GVD: la pipeline
  mesh -> places -> rooms e' completa;
- **AGENTS** cresce da 26 a 40: la traiettoria si accumula, quindi `pose_source:=isaac_gt` funziona;
- `mesh_vertices: 0` in `layer_contents` e' **normale**: la mesh va sul topic separato
  `/hydra/backend/dsg_mesh`, come dice la documentazione;
- zero occorrenze dell'errore GVD;
- grafi salvati su disco: `frontend/dsg.json`, `backend/dsg.json`, `dsg_with_mesh.json`.

### Il bug che bloccava tutto: `workstation_full.yaml`

Al primo tentativo il test e' fallito con PLACES **esattamente 0**, mentre OBJECTS, MESH_PLACES e
AGENTS crescevano. Nel log, a ogni frame:

```
gvd_place_extractor.cpp:121] GVD integrator min distance must be less than
truncation distance (currently 0.2 vs. truncation distance 0.15)
```

In `hydra/src/frontend/gvd_place_extractor.cpp:120` il controllo e' su
`config.gvd.min_distance_m >= map.config.truncation_distance`, e `detect()` **ritorna subito**: per
questo PLACES era 0 e non "poco popolato". In `config/hydra/workstation_full.yaml` l'autore aveva
dimezzato `voxel_size` (0.1 -> 0.05) e `truncation_distance` (0.3 -> 0.15) rispetto a habitat, ma
**non** le distanze GVD, rimaste ai valori di habitat. In `jetson_min.yaml` sono infatti coerenti
(0.2 < 0.3), quindi il bug riguardava solo questo profilo.

Correzione: `freespace_places.gvd.min_distance_m` 0.2 -> **0.1**, lo stesso rapporto di 2/3 della
truncation, coerente con i commenti "0.3 at 0.1" e "0.25 at 0.1 m voxels" del resto del file.
`voronoi_config.min_distance_m: 0.30` **non e' stato toccato**: e' un parametro diverso, non entra in
quel controllo, e non c'erano prove che fosse sbagliato.

Era esattamente il rischio dichiarato in `docs/hydra_hil.md` ("Not verified yet"): i valori erano
copiati dai config upstream e verificati contro il sorgente, non contro un parser in esecuzione.

### Due inciampi minori, risolti

- **`python3-pip` mancava.** Le "sudo steps" di `docs/hydra_hil.md` lo elencano; nella fase A ne ho
  installati tre su quattro. `build_hydra_ws.sh` compila tutti i 22 pacchetti e poi fallisce
  all'ultima riga con `No module named pip`.
- **`docs/hydra_hil.md:91` non funziona su un clone pulito.** Dice
  `colcon build --packages-select f1tenth_hydra f1tenth_logger f1tenth_params`, ma `f1tenth_logger`
  dipende da `corridor_perception`, `f1tenth_messages`, `f1tenth_description` e `f1tenth_perception`,
  che non sono compilati. Lo smoke test non ne ha bisogno: `_hydra_available()` chiede solo
  `f1tenth_hydra`, `hydra_msgs` e `spark_dsg`.
- **Rumore innocuo:** alla chiusura `odom_to_tf` stampa un traceback
  `rclpy.executors.ExternalShutdownException`. Verificato che arriva **dopo** il SIGINT (il processo
  ha girato tutti i 60 s): e' la solita verruca di `rclpy.spin`, non un guasto.

## Piano (riordinato)

Il repo contiene `src/f1tenth_hydra/f1tenth_hydra/synthetic_rgbd.py`, che pubblica un feed ZED 2i + GT
sintetico (stanza ray-cast, scatole, pianta, percorso ad anello). Quindi **Hydra si valida interamente
dentro WSL**, senza Isaac e senza attraversare il confine DDS. Questo toglie il firewall dal percorso
critico del primo traguardo, che e' "Hydra gira e produce un grafo non vuoto".

1. **ROS Jazzy in WSL** -- fatto.
2. **rosdep** -- fatto: `rosdep init`, `rosdep update`, poi
   `rosdep install --from-paths third_party/hydra_ros --ignore-src -r -y` (root). Porta GTSAM, PCL,
   glog/gflags, zmq, nanoflann. TensorRT/nvcc servono solo a `seg_source:=mobilenet`: si salta.
3. **Build** -- fatto: `scripts/hydra/build_hydra_ws.sh` -> `~/hydra_ws`, poi
   `colcon build --packages-select f1tenth_hydra f1tenth_params` (**non** `f1tenth_logger`).
4. **Smoke test, tutto in WSL** -- fatto e verificato a mano: vedi "La prova vera".
5. **DDS Isaac (Windows) <-> Hydra (WSL)** -- fatto: vedi "Passo 5" per la diagnosi e "FATTO" in
   cima per il risultato.
6. **Sim su Windows con le etichette GT** -- fatto: `scripts/sim/win/hil_sim_wsl.bat` le accende da
   solo (`--set sensors.stereo.semantic.enabled=true`), perche' `seg_source:=gt` le richiede.
7. **Hydra contro Isaac** -- **fatto e verificato**: vedi "FATTO" in cima.
8. **Visualizzazione** -- RViz dalla distro **verificato** (WSLg 1.0.66). `hydra_dsg_viewer` non
   provato; il grafo su disco e' pronto da aprire.

## Come avviare i due lati

**Windows**, in un cmd:

```bat
cd /d "F:\Users\Andreas Steffens\dev_ws\f1tenth_hil\scripts\sim\win"
hil_sim_wsl.bat
```

**WSL**, in un'altra finestra:

```bash
cd ~/f1tenth_hil && source scripts/hydra/hydra_env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE="$(scripts/hydra/wsl_dds_profile.sh)"
export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch f1tenth_hydra hydra.launch.py hydra_profile:=workstation_full \
  seg_source:=gt pose_source:=isaac_gt use_sim_time:=true
```

## Note operative

- I comandi in WSL vanno dati dal tool Bash, non da PowerShell: PowerShell rovina le virgolette
  annidate (`%{http_code}` di curl e' arrivato a destinazione spezzato).
- Niente `sudo` con password: `wsl -d Ubuntu-24.04 -u root` per apt/rosdep, `-u andreas` per il
  resto, cosi' i file del repo e di `~/hydra_ws` restano dell'utente.
- Alla prima invocazione dopo l'installazione, `wsl` resta appeso finche' la configurazione OOBE
  (creazione utente) non e' finita, senza stampare nulla di utile.
- Le modifiche fatte nel checkout Windows **non arrivano** al clone WSL, che viene da GitHub: vanno
  copiate a mano (lo fanno gli script in `scratchpad`) finche' non sono committate e si puo'
  `git pull` in WSL.
- Isaac va fermato per PID (`taskkill /PID <pid> /T /F`), non per pattern sul nome: il processo che
  tiene le porte DDS e' un figlio, e il path del venv non e' quello dell'immagine reale.

## Cosa resta davvero da fare

| | |
|---|---|
| `/sim/reset` tra i giri | la macchina resta dove l'hai lasciata; `hil_reset.bat` non e' mai stato provato qui |
| Ramo oggetti | serve la scena `base_room` (la sola che etichetta `plant` e `box`), oppure aggiungere label a `office.yaml` |
| `hydra_dsg_viewer` | mai aperto; il grafo su disco e' pronto (`~/hydra_runs/<stamp>/log/backend/dsg.json`) |
| Guida per esplorare | non esiste di validata: oggi va fatta piano, dritta e con un guardiano sulla posa. Il lavoro vero spetta al planner sull'Orin |
| Costo sul filo | mai misurato: stimato ~369 Mbit/s per le immagini a 10 Hz |
| `seg_source:=mobilenet` | fuori dalla strada scelta: `semantic_inference` e' compilato **senza** TensorRT (nessun `libnvinfer-dev`, nessun `nvcc`). Solo `gt` funziona |
| `pose_source:=stack_tf` | non toccato: serve l'Orin |
| `jetson_min` su aarch64 | non toccato |
| Trasporto TCP | a meta' strada, e non piu' necessario ora che c'e' la regola |

## Modifiche non committate nel repo

Niente e' stato committato. Lo stato al 2026-11-11 (`git status`):

**Modificati**

| File | Perche' |
|---|---|
| `src/f1tenth_hydra/config/hydra/workstation_full.yaml` | **il fix che sblocca PLACES** (vedi "Il bug che bloccava tutto") |
| `docs/hydra_hil.md` | "Not verified yet" dichiarava il falso: Hydra ora ha girato |
| `scripts/sim/win/hil_win.py` (dentro `scripts/sim/win/`, non tracciato) | modi `--net wsl` e `--net wsltcp` |
| `sim/isaac/config/hil.yaml` | sezione `wsl:` (`distro`, `tcp_port`), piu' `orin.models_dir` di prima |
| `scripts/sim/hil_orin`, `sim/isaac/sim/app.py`, `main.py`, `tools/sdf_to_usd.py`, `src/f1tenth_bringup/config/sim/dds/hil.xml.in` | **invariati da prima di oggi** |

**Nuovi**

| File | Cosa |
|---|---|
| `src/f1tenth_bringup/config/sim/dds/wsl.xml.in` | profilo UDP, un template per entrambi i lati |
| `src/f1tenth_bringup/config/sim/dds/wsl_tcp.xml.in` | profilo TCP, l'alternativa a meta' strada |
| `scripts/hydra/wsl_dds_profile.sh` | render lato WSL (`--tcp` per la variante) |
| `scripts/sim/win/hil_sim_wsl.bat` | il launcher del sim per il percorso WSL, etichette GT accese |
| `scripts/sim/win/` | i launcher Windows, incluso il nuovo `hil_sim_wsl.bat` |
| `output/hil/run_windows.md`, `output/hil/hydra_wsl_status.md` | la guida e questo file |

Fuori dal repo, sulla macchina: `sim/isaac/config/hil.local.yaml` (git-ignored) ha la sezione
`wsl:`; in WSL ci sono `/etc/apt/apt.conf.d/99force-ipv4` e
`/etc/sysctl.d/99-f1tenth-dds.conf`, piu' il clone `~/f1tenth_hil` e il workspace `~/hydra_ws`.
