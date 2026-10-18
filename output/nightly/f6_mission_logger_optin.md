# F6 — `enable_mission_logger` default false

## What changed

| file | change |
|---|---|
| `src/f1tenth_params/config/stack_params.yaml` | `enable_mission_logger: true -> false`, with the measurement that justifies it in the description |
| `src/f1tenth_logger/launch/mission_logger.launch.py` | docstring only: it said "defaults true". **No logic touched** -- both the node and the 5 Hz depth throttle were already gated on this argument |
| `src/f1tenth_bringup/launch/supervisor_bringup.launch.py` | declares `enable_mission_logger` (default from stack_params) and passes it to the supervisor node, exactly as `calibration` and `enable_intelligence` are |
| `src/f1tenth_bringup/f1tenth_bringup/component_supervisor_node.py` | `_apply_mission_logger_override()`, mirroring `_apply_calibration_override()`: writes the live value into the args of whichever component entry launches `mission_logger.launch.py`, found by launch_file rather than by position |
| `src/f1tenth_logger/TEST_CAMPAIGN.md` | the campaign's own stack command now carries `enable_mission_logger:=true`, with why and two commands to check it took |
| `docs/procedures/2026-09-18_go_to_floor.md`, `docs/procedures/2026-09-17_floor_person_pass.md` | the same on the two live procedures that bring the stack up for a recorded run |
| `README.md` | the Run row notes the argument |

**The supervisor change is load-bearing.** The supervisor launches each
component with the args `components.yaml` gives it and forwards none of its
own CLI arguments, so without it
`supervisor_bringup.launch.py enable_mission_logger:=true` would have been
silently ignored and the component would still have run the launch file with
its own (now false) default. Verified offline:

```
enable_mission_logger=False -> diagnostics: ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=false
enable_mission_logger=True  -> diagnostics: ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=true
```

## Evidence: the launch description, default args vs `:=true`

Built offline (no graph, no processes): every action of
`mission_logger.launch.py` whose condition evaluates true.

```
--- DEFAULT ARGS (enable_mission_logger=false)
    (nothing)
--- enable_mission_logger:=true (enable_mission_logger=true)
    Node topic_tools/throttle            <- the 5 Hz depth copy
    Node f1tenth_logger/mission_logger_node
```

So with false the node does not exist, and neither does the throttle that only
exists to feed its bags; with true the description is exactly what it was. The
diff is the whole launch description.
