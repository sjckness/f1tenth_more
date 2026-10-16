| id | command | translation_ok | final_state | success | stop_error_m | key_metric | min_clear_person_m | min_clear_plant_m | min_clear_walls_m | command_to_end_s | running_s | planner_s | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| approach_wall | approach the wall | True | COMPLETE | False | -0.452 | {"along_m": 0.992, "lateral_m": 0.003, "final_wall_gap_m": 1.548, "heading_change_deg": -0.4} | 0.457 | None | None | 35.0 | 4.3 | 10.4 | stop {'type': 'front_clearance', 'distance': 2.0, 'debounce_ticks': 3} |
| go_to_person | go to the person | True | COMPLETE | True | 0.243 | {"along_m": 0.504, "lateral_m": 0.134, "final_gap_m": 0.843, "bearing_to_target_deg": -0.9, "heading_change_deg": 27.5} | 0.84 | None | None | 33.7 | 3.1 | 10.5 | go_to person gap 0.6 |
