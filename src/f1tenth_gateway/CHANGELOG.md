# Changelog: `f1tenth_gateway`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_gateway`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` dopo il merge della linea Jazzy
  (`origin/main` 74c08897), nessuna modifica al codice rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-23

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-10-20 al 2026-10-23. Voci ricostruite da `git log main -- src/f1tenth_gateway` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Gateway della PWA: REST + WebSocket + rclpy + `f1tenth_ctl`, con mock e test; `RosBridge` su rclpy reale; endpoint `POST /api/_mock/reset`; README di installazione (Orin, token, systemd, tailscale, iPhone). Evidence: docs/analysis/hardware-comms/2026-10-20_pwa-lab-control/report.md (`a1a28631`, `c95b4131`, `20b8c687`, `4586b99c`; 2026-10-20)
- Accoppiamento via QR: token nel frammento del link e QR sull'Orin; WebSocket non autenticato spiega il motivo; `/api/status` indica se l'E-STOP è raggiungibile. (`7f3be61d`, `8626c194`, `1ea9ff68`; 2026-10-21)
- `/comms/link_status` nel payload di stato (solo report). (`0b096329`; 2026-10-21)
- Retromarcia nella guida manuale, passando per zero a ogni cambio di direzione. (`4d5ff3f3`; 2026-10-21)
- CPU per core, GPU % e processi fissati nel messaggio di stato. (`3accf86d`, `cf9e5ab5`; 2026-10-22)
- Stato di LLM2 nello status ed endpoint `/llm2/set_active`. Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`53fa6f1d`, `175da113`; 2026-10-22)
- Badge MOCK permanente nella PWA quando il gateway simula l'auto; vista Cockpit rinominata `lamacchinina`; messaggio chiaro con voce disattivata. (`9c5371d4`, `306782f7`, `fc273b75`; 2026-10-22)

### Changed
- Su Jazzy il gateway è super client del Discovery Server (`3ae4befa`; 2026-10-23).
- READY segue subito il latch dell'e-stop. (`c58cdda1`; 2026-10-20)

### Fixed
- Il parsing dello stato LLM2 non può più fermare l'executor ROS; gestiti hang di `nvidia-smi` e processi in uscita. (`175da113`, `cf9e5ab5`; 2026-10-22)
