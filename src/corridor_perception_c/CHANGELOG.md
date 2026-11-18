# Changelog: `corridor_perception_c`

Tutte le modifiche rilevanti di questo modulo (`src/corridor_perception_c`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.1] - 2026-11-18

### Fixed
- Correzioni di build e test per Jazzy, entrate su `main` con la linea Jazzy dopo la 0.1.0
  senza bump di versione (`2893d419`; 2026-10-19).

## [0.1.0] - 2026-10-16

Versione dichiarata alla creazione del pacchetto e mai cambiata. Voci ricostruite da
`git log main -- src/corridor_perception_c`.

### Added
- Nuovo pacchetto con l'implementazione in C di `extract_lines` per `corridor_perception` (`c421578`; 2026-10-16). Evidence: docs/analysis/platform/2026-10-16_cpu-h1-h5/report.md
