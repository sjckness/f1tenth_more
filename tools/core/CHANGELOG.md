# Changelog di tools/core

Formato [Keep a Changelog](https://keepachangelog.com/it-IT/1.1.0/), versioni SemVer: MAJOR cambia o
toglie un comando o un argomento (il vecchio resta una versione con un WARN "deprecato, usa ..."),
MINOR aggiunge, PATCH corregge. Ogni modifica a tools/core porta un bump di `VERSION` e una voce qui,
nello stesso commit, su main.

## [Unreleased]

## [1.0.2] - 2026-11-18

### Fixed
- `f1t selftest`: i test di doctor/core sync e dell'hook calcolano la versione successiva invece di
  scrivere 1.0.1 (fallivano appena VERSION arrivava a 1.0.1).

## [1.0.1] - 2026-11-18

### Fixed
- `f1t deploy`: rifiuta prima di toccare l'Orin un'immagine senza label `f1tenth.car_sync.sha256`
  (`docker_orin pin` la rifiuterebbe: oggi e' il caso delle immagini costruite dal docker/orin di main).
- `f1t deploy`: se lo switch fallisce prima del restart e il container di prima e' ancora sano, rimette
  il pin e si ferma, invece di un rollback con restart inutile.

## [1.0.0] - 2026-11-18

### Added
- `tools/f1t` con scoperta automatica dei comandi (`# desc:` / `# alias:`), `f1t help`, completamento bash.
- `f1t build`: build remota sull'Orin da un commit (bundle git, worktree temporaneo, build.sh del
  commit), pytest nell'immagine e label `f1tenth.tested=true`.
- `f1t deploy`: cancelli (tree pulito, immagine testata, `--after-hil`, stack fermo, modalita'
  container), diff dei parametri, pin + restart della unit, health check, rollback automatico,
  `deployed.json`; `--rollback`.
- `f1t orin status`, `f1t diff-params`.
- `f1t hil precheck|run|down -disp <host>`, scenari `smoke` e `office-corridor` (setup B di hil-isaac).
- `f1t whatis`, `f1t exp new`, `f1t release`.
- `f1t doctor`, `f1t core sync`, `f1t setup` (hook pre-commit, lanciatore), `f1t selftest`, `f1t ci ...`.

### Changed
- `tools/analysis_index.py` diventa `tools/core/analysis/index.py` (`f1t exp index`); il nome vecchio
  resta per tools/core 1.x con un avviso.
