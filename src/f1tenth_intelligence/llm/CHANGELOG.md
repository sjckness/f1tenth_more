# Changelog: `llm`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_intelligence/llm`).

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

## [0.0.1] - 2026-10-22

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-08-10 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_intelligence/llm` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Strumenti sim_next: warm-up di llama-server e scorer dei run del planner (`5ef58d17`; 2026-10-16). Evidence: docs/analysis/simulation/2026-10-16_sim-next-isaac/report.md
- Pacchetto `llm`: bringup di llama-server e tuner LLM dei parametri MPC. (`f3250a88`; 2026-08-10)
- `llm_planner_node` con consegna via servizio di missione e backend llama-server. (`580a2afb`; 2026-08-31)
- Percorso v2 a intento minimo: il modello scrive un intento breve e un traduttore che non inventa nulla produce il piano (schema 3.0 senza distanze inventate), selezionato da `planner_path`; "a N metri dal muro" è una soglia, non una distanza da percorrere. (`4f1f0458`, `9c7aa0fc`, `312165fa`, `ece482ce`; 2026-09-14..2026-09-15)
- `go_to` del planner diventa mossa `go_to_object` (`go_to_enabled`, default true); avvicinamento alla velocità di mossa con pavimento 0.4. Evidence: docs/analysis/llm-pipeline/2026-09-17_go-to-live-eval/report.md (`93565ddf`, `d28a86d7`, `6260f761`; 2026-09-16..2026-09-17)
- Output `RESULT` JSON del planner (`_emit_result`), `plan_result` prima del load, provenienza del piano, registrazione per test con successo manuale e conto alla rovescia, logger della campagna; chiavi blackboard di preflight registrate da un'unica lista. Evidence: docs/analysis/llm-pipeline/2026-09-18_llm-dry-campaign/report.md (`bc4631ad`, `dc158d24`, `8e0d48c5`, `6de6ac34`; 2026-09-21)
- Architettura a due LLM: `llm2_mpc_param_node` (mock che interroga i parametri MPC a 0.5 Hz, pubblica `MpcParamUpdate` su `/llm2/mpc_params` e stato su `/llm2/status`), config del server LLM2 (stesso gguf di LLM1), launch argument `enable_llm2`, pausa/ripresa via `/llm2/set_active` e `start_active`. Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`bc8eebdf`, `4a1b9edb`, `b2acbe82`, `87395195`, `8fe9db45`, `c3d8c92b`; 2026-10-22)

### Changed
- Guardia muro LLM di default della campagna finale; distanze dei piani misurate dal paraurti (`base_front`, posizioni da misura a nastro, passo 0.305). (`32bf7697`, `10d3eb34`, `bdc7c93b`, `d7b2770e`; 2026-10-13)
- Margine di classe `obstacle_class_margin_m = {person: 0.5}`. (`bfeb5038`; 2026-10-20)
- `llm_planner_node` avvia la missione alla conferma del caricamento; il timeout è solo un tetto. (`9c240241`; 2026-10-20)
