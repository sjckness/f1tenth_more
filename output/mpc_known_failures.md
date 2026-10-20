# mpc_controller: i 13 test che falliscono già

2026-10-20, branch `cf-next` (bfeb503). Eseguiti su linus (Jazzy, Python
3.12) con `python3 -m pytest test` nell'ambiente di install completo:
1058 passati, 13 falliti, 1 saltato, 2 xfail.

Sono **gli stessi 13** su `campaign-final` (cd180d8) e su
`mpc_corr_v2_reverse` (4f53ab8), ciascuno compilato e testato da solo. Nessuno
è causato dai merge né dai commit di oggi. Quasi tutti sono test che fissano un
valore di configurazione cambiato senza aggiornare il test: la trappola
descritta in CLAUDE.md, "A test that never runs will contradict the code".

| # | Test | Causa (una riga) |
|---|---|---|
| 1 | `test_corridor_direction_recovery.py::TestCarTrackingCorridorOnAFrozenHeading::test_the_default_geometry_freezes_both_ends` | Fissa `corridor_heading_return` a false, ma `stack_params.yaml` ora vale true. |
| 2 | `test_corridor_sizing.py::test_corridor_holds_the_whole_car_passing_a_person_one_metre_ahead_of_the_bumper` | Il corridoio di default è largo 0.86 m (semi-larghezza) a 1.4 m, il test ne chiede 1.37: i parametri di larghezza del corridoio sono cambiati dopo il test. |
| 3 | `test_drive_limits.py::TestDefaultsChangeNoRequestedSpeed::test_the_go_to_approach_speed_is_the_operating_floor` | Si aspetta `mission_translator_speed_go_to` = 0.4 (il minimo), ma lo YAML ha 0.5 dal 2026-09-17 (d28a86d). |
| 4 | `test_d_wall_correction.py::TestTheMissionThatExercisesIt::test_every_other_wall_turn_mission_is_still_terminal_on_the_turn` | La missione `drive_stop_2m_from_wall.json` ora termina con un move `straight`, il test la tratta ancora come wall_turn. |
| 5 | `test_flake8.py::test_flake8` | 1177 violazioni di stile già presenti (import order, docstring, indentazione); oggi nessuna nuova sui file toccati. |
| 6 | `test_model_log.py::test_header_is_the_one_the_analysis_script_parses` | L'header del log del modello è cambiato (`t,x,y,psi,v,steer_cmd,accel_cmd`) e lo script di analisi atteso dal test no. |
| 7 | `test_pep257.py::test_pep257` | Docstring non conformi già presenti, come per flake8. |
| 8 | `test_prediction_step.py::test_the_lookahead_floor_stays_inside_a_full_length_corridor` | Il pavimento del lookahead risulta 1.25 m invece di 2.5: l'orizzonte di predizione è passato da 0.2 s a 0.1 s per passo (vedi #9). |
| 9 | `test_prediction_step.py::test_the_shipped_horizon_is_twenty_steps_of_0p2_s_on_a_10_hz_loop` | Il passo di predizione spedito è 0.1 s, il test fissa 0.2 s (20 × 0.2 = 4 s). |
| 10 | `test_wall_turn_increment.py::TestCorridorWiring::test_psi_end_psi_ref_turn_and_the_centreline_share_one_increment` | Con questo setup l'incremento di virata `psiRefTurn` vale 0: la logica del trigger di virata è cambiata dopo il test. |
| 11 | `test_wall_turn_increment.py::TestEachDirectionUsesItsOwnSteeringBound::test_the_asymmetry_changes_when_the_turn_commits` | Stessa origine di #10 e #12: l'asimmetria per direzione non scatta più dove il test la aspetta. |
| 12 | `test_wall_turn_increment.py::TestEachDirectionUsesItsOwnSteeringBound::test_the_trigger_distances_are_the_work_orders_table` | La distanza di trigger calcolata è 2.30 m, la tabella dell'ordine di lavoro dice 3.71 m: i limiti di sterzo o la geometria sono cambiati. |
| 13 | `test_weight_set.py::TestTheEffectiveValuesAreWhatTheNoteQuotes::test_effective_value[w_du_delta-3.0]` | Il peso effettivo `w_du_delta` è 2.92, la nota citata dal test dice 3.0 ± 0.05: il peso è uscito dalla tabella il 2026-09-09 e il test no. |

Per rieseguirli:

```bash
source /opt/ros/jazzy/setup.bash   # sull'Orin: humble
source <workspace>/install/setup.bash
cd src/f1tenth_control/mpc_controller
python3 -m pytest -q test -rfE
```

Note:
- Le cause dei #1, #3, #5, #6, #7, #9, #13 si leggono direttamente nel
  messaggio di asserzione.
- Le cause dei #2, #4, #8, #10, #11, #12 sono dedotte dal messaggio e dal
  nome del test, senza aver risalito il commit che ha cambiato il valore.
- Gli altri fallimenti già presenti su linus, sempre uguali ai genitori, li
  lascio solo come conteggio: llm 14 (flake8, pep257, `test_intent_go_to` ×7,
  `test_planner_path` ×5), bringup 3 (copyright, flake8, pep257), logger 11 (3 lint, 2 in
  `test_robot_logger` sulla ricerca del workspace, 6 errori in
  `test_object_run_summary`; misurati con un `pyarrow<16` temporaneo, perché
  su linus manca).
