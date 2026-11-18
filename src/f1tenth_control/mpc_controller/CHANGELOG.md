# Changelog: `mpc_controller`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_control/mpc_controller`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` al commit `e376731f`, nessuna modifica al codice
  rispetto a 0.0.0. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.0] - 2026-10-24

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-24. Voci ricostruite da `git log main -- src/f1tenth_control/mpc_controller` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nodo MPC_corr (solver RTI/OSQP) come controllore dispiegato, con vincoli duri sui bordi del corridoio e correzione dell'infattibilità OSQP; rimosse le varianti morte. (`0f24e660`, `7f893d47`; 2026-08-13)
- Corridoio dell'MPC: visualizzazione, ricostruzione a 10 Hz, blend a S dell'intestazione nel corridoio rettilineo, riferimento rettilineo congelato all'inizio della mossa, ancoraggio map->odom con guardia contro trasformate stantie e opt-out da launch argument per frequenza di ricostruzione e vincoli duri. (`f3fa1967`, `c849120b`, `431aed8a`, `c5123234`, `27010781`, `d6bd8cda`; 2026-08-21..2026-09-07)
- Corridoio schema v2 con definizione di funzione ed evaluator condiviso; gli snapshot del corridoio non vengono più distrutti a ogni avvio. (`90c51d6b`, `b68f360e`; 2026-09-23)
- Mosse DriveCommand a durata aperta (`straight`, `wall_turn`, schema 3.0), con svolta nel verso comandato e non per la via più breve. (`e164f80d`, `32587b0d`; 2026-09-10)
- Pass di controllo wall_turn: tracker, pianificatore di svolta, `d_wall` e log del modello; in seguito rampa di velocità e freno sul lidar grezzo indipendente dal pianificatore, distanza dal muro dai track semantici dietro flag `corr_wall_turn_distance_source` (default `legacy`), niente più blocco quando l'auto è dentro il margine. Evidence: docs/analysis/mpc-control/2026-09-14_wall-turn-investigation/report.md (`4c35975a`, `4262b74f`, `6137165f`, `0ca071fa`; 2026-09-14..2026-10-24)
- Corridoio d'oggetto (direzione mantenuta, bloccato sul target), con arco che porta l'auto al target dietro parametro, lunghezza massima, fine sul target e tolleranza per curva; flag per tick `inside_turn_radius` e `target_behind`. (`03286d64`, `f2f18f46`, `51c6154e`, `79663383`, `fa9f99e4`, `bc3b0490`; 2026-09-16..2026-09-23)
- Mossa di missione `go_to_object`: la distanza è un gap con minimo derivato dal controllore, `go_to_enabled` (default true), eco di `track_id` e gap vivo nello stato oggetto, avvicinamento alla velocità di mossa con pavimento 0.4 e stop su trigger latched. Evidence: docs/analysis/mpc-control/2026-09-17_object-gap/report.md (`22ff0315`, `edd0e6bc`, `d28a86d7`, `e5f47311`, `6260f761`; 2026-09-16..2026-09-17)
- Righe-muro come semipiani dal tracker multi-muro, `boundary_source` con i muri come sorgente di default. Evidence: docs/analysis/mpc-control/2026-10-19_avoidance-wall-boundaries/report.md (`91fe0b0e`, `d3671b17`; 2026-10-19)
- Percorso ostacoli unico verso l'MPC con `unknown_objects_mode` e cap di velocità; margine per classe `obstacle_class_margin_m = {person: 0.5}`. Evidence: docs/analysis/mpc-control/2026-09-17_person-pass-margin/report.md (`79562b18`, `4e0391d3`, `bfeb5038`; 2026-09-16..2026-10-20)
- Retromarcia: scelta propria dell'MPC (candidati di direzione, flag `allow_reverse`, cost-to-go terminale consapevole dei muri) e manovra di retromarcia con arbitraggio automatico (bloccato, pronto, consentito), memoria semantica degli oggetti con decadimento legato alla visibilità, `reverse_status` su entrambi i meccanismi. Evidence: docs/analysis/mpc-control/2026-10-19_reverse-mpc/report.md, docs/analysis/mpc-control/2026-10-20_mpc-v2-reverse-memory/report.md (`780fd09c`, `9f780042`, `353bc07c`, `b5f46ea4`, `c33f969c`, `fc4c2728`, `a8a03ea4`; 2026-10-19..2026-10-20)
- Rampa di decelerazione che arriva a un punto di arresto invece di superarlo, e che punta al muro frontale bloccato, con log di ogni transizione di sorgente. Evidence: docs/analysis/mpc-control/2026-10-20_stop-distance/report.md (`6c693ac2`, `00635090`, `480fb6c4`; 2026-10-20..2026-10-22)
- Comando di velocità senza offset (modello di lag + osservatore di disturbo), `speed_command_mode` `offset_free` come nuovo default (il legacy resta bit-identico). Evidence: docs/analysis/mpc-control/2026-10-21_speed-gap/report.md (`4701caa0`; 2026-10-21)
- Sottoscrizione MPC_corr agli aggiornamenti parametri di LLM2, con whitelist dei parametri tunabili (`w_obs` mai tunabile, parametri LLM2 in sola lettura). (`4e93fdc0`, `384621fc`; 2026-10-22)
- Fonti dati della campagna di test: `/mpc/status`, `/corridor`, `/safety/event`, `/obstacle_clearance`. (`3e50039b`; 2026-09-21)
### Changed
- Pesi normalizzati e unica fonte via `stack_params`, bilanciamento stadio/terminale invariante all'orizzonte, righe di bordo morbide di default con numero di slot configurabile, costo di intestazione per stadio (inerte), limiti di sterzo reali derivati dalla calibrazione del servo, spostamento del target ostacolo 0.36 -> 0.30. (`ea66819f`, `62e9777f`, `69aa44a0`, `7eb56ab3`, `863304aa`, `6dc5a37d`; 2026-09-08..2026-09-09)
- Geometria e frequenza di ricostruzione del corridoio fissate per misura; default `corridor_update_period` 1.0 e vincoli duri disattivi. (`ee9fb1db`, `f000ae0f`, `8107f72c`, `01240dd2`; 2026-09-04..2026-09-08)
- Campagna finale: corridoio trapezoidale con rapporto fisso fine/inizio 2.3/1.3 e semi-larghezza iniziale 0.64 m (`corr_width_scale_with_length` ripristina il vecchio scaling), passo di predizione 0.2 s con N = 20 (orizzonte 4 s, ciclo 10 Hz), calibrazione guadagno erpm, guadagni servo e limiti di sterzo. (`a6dd23e2`, `30b0580e`, `9b8f1b86`; 2026-10-13)
- Frame `base_front`: tutte le distanze frontali misurate dal paraurti (soglie invariate, quindi gli stop avvengono prima), passo 0.305 nel URDF e posizioni di paraurti, LiDAR e ZED dalla misura a nastro. (`10d3eb34`, `bdc7c93b`, `d7b2770e`; 2026-10-13)
- Consumatori di muro (stop e wall_turn) leggono l'unica verità sul muro; CLEAR guida, NO_DATA tiene fermo, e 50 m non arriva mai al solver. (`ca8e15bc`, `796f3693`; 2026-10-13..2026-10-15)
- Ogni velocità su `/drive` passa da un solo clamp (niente retromarcia, max 1.0 m/s), pavimenti 2026-09-18 (`max_forward_speed_mps` 0.5, go_to 0.4), ogni abort di missione attiva `/mpc/hold`; la retromarcia si sblocca solo da manovra o scelta MPC, mai da un sottopasso in avanti. (`e00cd8d7`, `04152e23`, `07023d86`, `a8a03ea4`; 2026-09-16..2026-10-20)
- Il conteggio di retromarcia usa l'odometria, non l'ancora della mappa, e termina sul maggiore tra i conteggi odom e mappa; la riga CTRL riporta ciò che è stato pubblicato a ogni tick. (`dd129b97`, `65368775`, `3a13b1db`; 2026-10-20..2026-10-21)
### Fixed
- Warm-start dell'RTI che in realtà non partiva, e riferimento rettilineo dopo curva con spin errato nell'`orientation_delta` di 180 gradi. (`84ddc90e`, `2bd64204`; 2026-08-21..2026-09-10)
- Nessun margine persona sopravvive al dropout del campo visivo (margine resta `{}`). (`56ec0265`; 2026-09-17)
- wall_turn senza distanza dal muro non solleva più eccezioni. (`75bf2469`; 2026-10-14)
- MPC_corr rifiuta di partire con entrambi i meccanismi di retromarcia attivi; il bersaglio della mossa non è mai un ostacolo. (`f8844004`, `6f48d197`; 2026-10-20)
