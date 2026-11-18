# Changelog: `f1tenth_logger`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_logger`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` al commit `e376731f`, nessuna modifica al codice
  rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-22

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-09-03 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_logger` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nuovo pacchetto: recorder di missione e renderer di replay spostati qui, indice `runs.db` sui manifest con CLI `runs`, renderer diviso in metà ROS e ROS-free, consolidamento dei bag in una cartella per run (`903a0f1`, `a9598d4`, `e18fa7b`, `97873f1`, `6279171`; 2026-09-03).
- `mission_logger`: archivio di run a due stati, cache di scrittura con coda profonda per `/tf`, dati `go_to_object` e clamp di `/drive` nel bag e nel riepilogo, registrazione dei muri (`/perception/walls` e marker), della profondità ZED a 5 Hz (`mission_logger_record_depth`) e dell'intero anello di velocità (`6ecda96`, `61c22e0`, `8ce08fa`, `ebbf595`, `fa79f59`, `17c54cc`; 2026-10-21).
- Logger della campagna di test affiancato al mission logger: provenienza del piano, configurazione logger, backfill, corridoio schema v2 con valutatore condiviso, plot per test e corridoio ad arco verso l'oggetto con lunghezza massima (`8e0d48c`, `6de6ac3`, `90c51d6`, `0e52526`, `51c6154`, `7966338`, `bc3b049`; 2026-09-23).
- Bring-up della campagna: registrazione da missione LOADED con pre-roll di 15 s, relay di visualizzazione, whitelist Foxglove, profilo di registrazione, replay muri v2 (`407ff6d`, `3f3cfba`, `0abfd92`; 2026-10-16).
- `mission_overlay`: stills e video del rapporto di run con maschera di scena e LiDAR (`e9d18c2`, `1cc0a8a`, `2ed5216`; 2026-10-22).
- Registrazione dei dati di reverse, memoria semantica degli oggetti e sottoscrizione MPC_corr con whitelist per gli aggiornamenti parametri di LLM2 (`780fd09`, `b5f46ea`, `c33f969`, `4e93fdc`; 2026-10-22).
### Changed
- La registrazione è opt-in e il toggle ora raggiunge davvero il logger; l'immagine annotata non è registrata di default (`774b2e0`, `14f449b`; 2026-10-16).
- Il conto alla rovescia di avvio missione vale solo se il mission logger registra; default 0 e definito in un solo punto in `stack_params` (`a14ec2f`, `3ac5865`; 2026-10-21).
- Il corridoio della campagna misura la distanza dai muri e non dal tappo finale; il replay disegna il padding di confine proprio di ciascun run (`220dba3`, `bb66a54`; 2026-09-23).
- Consumatori di arresto sul muro e `wall_turn` leggono l'unica verità sul muro (`ca8e15b`; 2026-10-13).
### Fixed
- `mission_replay_video` richiede `--out-dir` senza radice del workspace; dichiarata la dipendenza `pyarrow` e rimosso il doppio trattino dal commento di `package.xml` (`7fa35b8`, `97873f1`, `fe2c845`; 2026-09-03).
