# Moduli

Un modulo è un pacchetto ROS 2 del workspace (`src/`, esclusi i submodule esterni). Ogni modulo ha:

- una sola versione, in `package.xml`, in [SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia
  usata da altri (topic, servizio, messaggio, parametro), MINOR aggiunge una funzione o cambia un
  comportamento, PATCH corregge;
- un `CHANGELOG.md` accanto a `package.xml` ([Keep a Changelog](https://keepachangelog.com/it-IT/1.1.0/)):
  sezione `[Unreleased]`, poi `[x.y.z] - data` con Added / Changed / Fixed / Removed. Una voce può citare
  `Evidence: docs/analysis/<area>/<id>/report.md`;
- un tag git `<pacchetto>/vX.Y.Z` sul commit di rilascio.

Regola: un bump di versione = una voce nel CHANGELOG, nello stesso commit, sul branch della feature.
Nessun merge senza. I report di analisi citano in `modules:` le versioni testate.

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| modulo | versione | ultimo cambio (git) | ultima release (CHANGELOG) | aree analisi collegate | changelog |
|---|---|---|---|---|---|
| `corridor_perception` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [localization-frames](../analysis/localization-frames/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/corridor_perception/CHANGELOG.md) |
| `corridor_perception_c` | 0.1.1 | 2026-11-18 | 0.1.1 (2026-11-18) | [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/corridor_perception_c/CHANGELOG.md) |
| `f1tenth_behavior` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [localization-frames](../analysis/localization-frames/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_behavior/CHANGELOG.md) |
| `f1tenth_bringup` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [localization-frames](../analysis/localization-frames/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_bringup/CHANGELOG.md) |
| `f1tenth_comms` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [hardware-comms](../analysis/hardware-comms/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_comms/CHANGELOG.md) |
| `f1tenth_control` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_control/f1tenth_control/CHANGELOG.md) |
| `f1tenth_costmap` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_costmap/CHANGELOG.md) |
| `f1tenth_costmap_boundary_rs` | 0.1.0 | 2026-10-25 | — | — | — |
| `f1tenth_ctl` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_ctl/CHANGELOG.md) |
| `f1tenth_description` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_description/CHANGELOG.md) |
| `f1tenth_diagnostics` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_diagnostics/CHANGELOG.md) |
| `f1tenth_gateway` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_gateway/CHANGELOG.md) |
| `f1tenth_hard_stop_rs` | 0.1.0 | 2026-10-25 | — | — | — |
| `f1tenth_hardware` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [mpc-control](../analysis/mpc-control/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_hardware/f1tenth_hardware/CHANGELOG.md) |
| `f1tenth_localization` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_localization/CHANGELOG.md) |
| `f1tenth_logger` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_logger/CHANGELOG.md) |
| `f1tenth_messages` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_messages/CHANGELOG.md) |
| `f1tenth_more` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | — | [CHANGELOG](../../src/f1tenth_more/CHANGELOG.md) |
| `f1tenth_mpc_core` | 0.1.0 | 2026-10-24 | — | — | — |
| `f1tenth_navigation` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [mpc-control](../analysis/mpc-control/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_navigation/CHANGELOG.md) |
| `f1tenth_params` | 0.2.0 | 2026-11-19 | 0.2.0 (2026-11-19) | [field-tests](../analysis/field-tests/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [localization-frames](../analysis/localization-frames/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_params/CHANGELOG.md) |
| `f1tenth_perception` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [hardware-comms](../analysis/hardware-comms/README.md), [localization-frames](../analysis/localization-frames/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_perception/CHANGELOG.md) |
| `f1tenth_planners` | 0.1.0 | 2026-11-19 | 0.1.0 (2026-11-19) | [mpc-control](../analysis/mpc-control/README.md) | [CHANGELOG](../../src/f1tenth_control/f1tenth_planners/CHANGELOG.md) |
| `f1tenth_proximity_guard` | 0.1.0 | 2026-10-25 | — | — | — |
| `f1tenth_sim` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_sim/CHANGELOG.md) |
| `f1tenth_telegram` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [hardware-comms](../analysis/hardware-comms/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_telegram/CHANGELOG.md) |
| `f1tenth_wall_tracker_core` | 0.1.0 | 2026-10-24 | — | — | — |
| `husarion_gz_worlds` | 0.1.0 | ? | — | — | — |
| `llm` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [mpc-control](../analysis/mpc-control/README.md), [platform](../analysis/platform/README.md), [simulation](../analysis/simulation/README.md) | [CHANGELOG](../../src/f1tenth_intelligence/llm/CHANGELOG.md) |
| `mpc_controller` | 0.1.0 | 2026-11-18 | 0.1.0 (2026-11-18) | [_unsorted](../analysis/_unsorted/README.md), [field-tests](../analysis/field-tests/README.md), [llm-pipeline](../analysis/llm-pipeline/README.md), [localization-frames](../analysis/localization-frames/README.md), [mpc-control](../analysis/mpc-control/README.md), [perception](../analysis/perception/README.md), [platform](../analysis/platform/README.md) | [CHANGELOG](../../src/f1tenth_control/mpc_controller/CHANGELOG.md) |
<!-- AUTO:END -->
