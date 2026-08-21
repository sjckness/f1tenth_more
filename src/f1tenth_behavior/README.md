# f1tenth_behavior

py_trees-based reactive safety-stop (emergency + obstacle-corridor lanes), Nav2
goal-pose navigation, and a scripted **mission subtree** for the F1TENTH stack.
This README documents the mission subtree's config format -- the single source
of truth for it, alongside `f1tenth_behavior/mission/mission_config.py` (the
parser/validator that actually enforces this table) and `missions/*.json`
(worked examples).

If this table and `mission_config.py` ever disagree, `mission_config.py` is
correct at runtime and this table is stale -- fix the table.

## Loading a mission

- **`mission_file_name`** (string parameter, also in `f1tenth_params/config/stack_params.yaml`
  and forwarded by `behavior_bringup.launch.py`): bare filename (not a full path) of a
  mission JSON under this package's installed `missions/` dir, read once at
  `behavior_executor_node` startup and resolved there (`mission/loader.py`). Leave unset
  (`''`, the default) to start with no mission loaded (`mission.state == IDLE`).
Loading and running are two separate steps: a load only installs a mission
(`mission.state -> LOADED`) and does not move the car; `/mission/start_mission`
is what actually begins it (`LOADED -> RUNNING`). This lets a mission be loaded
well ahead of time (pre-flight checks, etc.) without driving off the instant the
file parses.

- **`/mission/load_path`** (`std_msgs/String`): publish a path to (re)load a
  mission at runtime. A successful load replaces whatever mission was loaded/
  running and installs the new one at move 0, state `LOADED` (call
  `/mission/start_mission` to actually begin it). A failed load (bad path,
  malformed JSON, or any schema violation below) is rejected in full -- logged
  clearly, and whatever mission was already loaded/running is left untouched.
  Missions are never partially loaded.

  (Chose a plain `String` topic over a custom service so this didn't require
  adding a new `.srv` to `f1tenth_messages` -- see the design note in
  `mission/loader.py` if synchronous load-result reporting turns out to be
  needed later.)

- **`/mission/load_mission`** (`f1tenth_messages/srv/LoadMission`): same load
  path as `/mission/load_path` above (same validation, same rejection
  behavior, same `LOADED`-not-running result), but synchronous: the response
  reports whether the load succeeded.

  ```
  # request:  string path
  # response: bool success, string message

  ros2 service call /mission/load_mission f1tenth_messages/srv/LoadMission \
    "{path: '/opt/ros/f1tenth/share/f1tenth_behavior/missions/dock_approach_01.json'}"

  # success: message like "loaded 'dock_approach_01', 4 moves"
  # failure: message is the same validation-error text load_path/mission_config.py
  #          already produce for that failure -- not a different wording
  ```

- **`/mission/start_mission`** (`std_srvs/Trigger`): `LOADED -> RUNNING`. Fails
  (`success=false`) unless state is currently `LOADED` -- in particular it does
  NOT re-arm an `ABORTED`/`COMPLETE` mission; load it again first.

  ```
  ros2 service call /mission/start_mission std_srvs/srv/Trigger {}

  # success: message = "started 'dock_approach_01'"
  # failure: message = "cannot start: state is <X>, not LOADED (load a mission
  #          via /mission/load_mission first)"
  ```

- **`/mission/abort_mission`** (`std_srvs/Trigger`): abort whatever mission is
  currently loaded-or-running (`LOADED`, `RUNNING`, or `HOLDING`) -- no
  request payload, no mission_id to get right; there's only ever one mission
  slot at a time in this system, so "abort the current one" is unambiguous.
  (Used to require a `mission_id`, guarded via `f1tenth_messages/srv/
  AbortMission` -- dropped per a later simplification request; that .srv was
  removed from `f1tenth_messages` since this was its only consumer.) Performs
  the exact same transition as an `abort_mission` `on_object` action
  (`mission.state -> ABORTED`, `/mpc/hold` published `true`) -- see the
  `on_object` table below.

  ```
  ros2 service call /mission/abort_mission std_srvs/srv/Trigger {}

  # success: message = "aborted 'dock_approach_01'"
  # failure: message = "no mission to abort (state=<X>)" (state is
  #          IDLE/COMPLETE/ABORTED already)
  ```

- **`/mission/emergency_stop`** (`std_srvs/Trigger`): latches an internal flag
  for the rest of this process's lifetime -- no reset service exists on
  purpose, restart `behavior_executor_node` (or the whole BT process) to clear
  it. Deliberately separate from mission control above: it feeds the BT's
  *emergency* lane (`IsEmergencyStopTriggered`, alongside `IsBatteryLow`/
  `IsSystemOverheated`), stopping the car via the `safety_stop` ackermann_mux
  lane (priority 200) regardless of mission/MPC state, rather than going
  through `/mpc/hold` like `abort_mission` does.

  ```
  ros2 service call /mission/emergency_stop std_srvs/srv/Trigger {}
  ```

- **`/mission/status`** (`f1tenth_messages/msg/MissionStatus`, transient-local/
  latched QoS -- a subscriber started after this node still gets the current
  value immediately): `state`, `json_path`, and `emergency_stop_active`,
  republished on every load/start/abort/emergency_stop. Mainly for external
  introspection (`ros2 topic echo /mission/status`, dashboards); the BT's own
  mission subtree reads mission state directly off the blackboard instead
  (same process, no need to round-trip through this topic).

## Top-level mission JSON

```json
{
  "mission_id": "dock_approach_01",
  "schema_version": "2.0",
  "moves": [ /* array of Move objects, executed in order */ ]
}
```

`mission_id` is for logging only. `moves` must be non-empty; every move's `id`
must be unique within the mission. `schema_version` is optional -- omitted
entirely (every mission written before it existed) means `"1.0"`, and nothing
below is actually gated on this value: it's informational bookkeeping, not an
enforced feature flag. `"2.0"` is just the first version aware of `turn` /
`orientation_delta` / optional `timeout_sec`; a `"1.0"`-labeled mission that
happened to use any of them would still parse and run identically.

## Move fields

| Field | Type | Required | Notes |
|---|---|---|---|
| `id` | string | yes | Unique within the mission; used by `skip_to_move` and in logs |
| `goal_distance` | float | one of this/`goal_pose`/`turn` | Straight-line target, passed to `/mpc/goal_distance` |
| `goal_pose` | `{x, y, yaw}` (floats) | one of this/`goal_distance`/`turn` | Published by `PublishMoveGoal` straight to mpc_corr's own `/mpc/goal_pose` input -- drives the robot directly through mpc_corr (not Nav2/`NavigateThroughPoses`; see "Safety interactions" below for why). Position-only, same scope limit as mpc_corr.py's own pose-goal handling: `yaw` is sent and stored but not consumed for arrival. |
| `turn` | object (schema_version 2.0) | one of this/`goal_distance`/`goal_pose` | `{heading_delta_deg, speed, steering, reference}` -- see its own section below. Published as `f1tenth_messages/TurnGoal` to mpc_corr's `/mpc/goal_turn`. |
| `vdes` | float | no | Overrides mpc_corr's `vdes` for this move. **No such override mechanism exists yet for non-`turn` moves** -- logged once per move as a TODO stub, otherwise ignored. (A `turn` step's own `speed` field is a separate, real, already-working override -- see below.) |
| `stop_condition` | object | yes | One of the types below |
| `on_object` | array of objects | no | Each entry: `{class, action, ...action-specific fields}` -- see below |
| `timeout_sec` | float | no (schema_version 2.0 -- was required pre-2.0) | Hard escape hatch from move start; must be > 0 if present. Omitted means no per-move timeout cap is enforced at all. Every pre-2.0 mission always set this explicitly, so relaxing it to optional doesn't change how any of them behave. |
| `on_timeout` | `"abort"` \| `"skip"` \| `"stop"` (schema_version 2.0 added `"stop"`) | no, default `"abort"` | What happens if `timeout_sec` elapses before `stop_condition` fires -- see below. **Rejected at load time if present without `timeout_sec`** (nothing for it to apply to). |

Exactly one of `goal_distance`/`goal_pose`/`turn` is required -- providing
more than one, or none, is rejected at load time.

`on_timeout` behaviors: `"abort"` aborts the whole mission
(`mission.state -> ABORTED`); `"skip"` advances to the next move as if
`stop_condition` had fired; `"stop"` (new) does neither -- it holds the car
(`/mpc/hold` true) and leaves the mission sitting on the same, now-timed-out
move indefinitely, until something else intervenes (`/mission/abort_mission`,
a `skip_to_move`, etc.).

## `turn` fields (schema_version 2.0)

| Field | Type | Required | Notes |
|---|---|---|---|
| `heading_delta_deg` | float | yes | Signed: `+` = left/CCW, `-` = right/CW (standard ROS yaw right-hand rule). |
| `speed` | float | yes, must be > 0 | Linear speed while turning [m/s] -- sets mpc_corr's `vdes` for the duration of the turn. Unlike the general `vdes` field above, this one is real/working, scoped to this move only. |
| `steering` | `"full_lock"` \| `"partial:<deg>"` | yes | Shapes how tightly mpc_corr curves toward the turn (via the standard Ackermann turn-radius relationship, not a literal open-loop steering command -- the MPC solver still computes the actual steering output every tick). `"partial:15"` means a 15° request. |
| `reference` | string | no, default/only value `"odometry_orientation"` | Kept as a real field (not hardcoded) for a future alternate heading source; only one value is accepted today. |

A `turn` step's own `stop_condition` (see below) must be `orientation_delta`
with a `value` matching `abs(heading_delta_deg)` -- checked at load time, not
trusted separately at runtime, so the two can never silently disagree.

`turn` drives through the exact same actuation path every other move does
(mpc_corr -> the mux's `navigation` lane) -- it does **not** bypass
mpc_corr or publish drive commands directly. mpc_corr resolves a turn
request into a synthetic `goal_pose` target along the requested final
heading and reuses its own already-existing corridor-following/arrival
machinery; see `MPC_corr.py`'s `goal_turn_callback` docstring for the exact
mechanism. mpc_corr's own arrival at that synthetic target is **not** the
authoritative "is the turn done" signal for the mission, though -- that's
this move's own `orientation_delta` stop_condition, tracked independently
against live `/odom` yaw by `CheckStopCondition`.

## `stop_condition.type` values

| `type` | Extra fields | Meaning |
|---|---|---|
| `distance_reached` | `distance` (float, optional -- defaults to the move's own `goal_distance`) | Straight-line distance traveled from move start ≥ target |
| `goal_reached` | `tolerance` (float) -- **currently unread, see note below** | `True` once mpc_corr reports its current goal reached. Shares mpc_corr's single `/mpc/goal_reached` topic between distance-mode and pose-mode arrival (mode-agnostic on the wire), so this works for either kind of move, not just `goal_pose` ones. Latched per-move by `CheckStopCondition` (resets when the current move changes) so a stale `True` from an earlier move can't misfire the next one. **The per-condition `tolerance` field above is not consulted** -- actual pose-arrival tolerance is mpc_corr's own `pose_goal_tolerance` ROS parameter (default 0.15 m), set once for the whole node, not per move. |
| `time_elapsed` | `duration_sec` (float, required) | Wall-clock time since move start ≥ duration |
| `object_seen` | `class` (string, required), `min_confidence` (float, optional) | `True` once `class` is present in `detected_classes` at/above confidence, within the freshness window (1.0s default -- see "Assumptions" below) |
| `object_cleared` | `class` (string, required), `debounce_sec` (float, default 1.0) | `True` once a previously-seen class has been absent for ≥ `debounce_sec` |
| `obstacle_distance_below` | `distance` (float, required) | `True` when `/mpc/min_obstacle_distance` < threshold |
| `front_clearance` | `distance` (float, required) | `True` when `/perception/front_clearance` < threshold. Published by `f1tenth_perception`'s `wall_detector_node` (RANSAC plane segmentation, nearest front-facing wall/planar surface) -- a separate sensing path from `obstacle_distance_below`'s YOLO/`obstacle_projector_node` discrete-object distance. `None` (not yet satisfiable) until the first message arrives; `wall_detector_node` publishes `+inf`, not silence, whenever no front-facing wall is currently in view, so a never-arriving message and "no wall detected" are distinguishable. |
| `orientation_delta` (schema_version 2.0) | `value` (float, required, degrees) | `True` when `abs(current_yaw - turn_start_yaw)` (atan2-wrapped, correctly handles the ±180° seam) ≥ `value`. `turn`-exclusive -- rejected at load time on any other step type. `turn_start_yaw` is this move's own `/odom` yaw at the moment it was first observed (lazily captured the same way `distance_reached`'s `move_start_xy` already is), not the mission's or the car's all-time starting heading. |
| `manual` | -- | Never auto-completes. **Out of scope for this pass** -- always behaves as still-waiting; only an external trigger (not yet implemented) could advance it. |

`distance_reached` requires either its own `distance` field or the move's
`goal_distance` to be set -- a `distance_reached` move with a `goal_pose` and
no explicit `distance` is rejected at load time (nothing to measure against).

Stub types (`goal_reached`, `manual`) never crash and never silently succeed --
they behave as permanently unsatisfied, so a move using one only ever completes
via `timeout_sec`/`on_timeout`. A warning is logged once per move (not once per
tick) when a stub type is actually in use.

## `on_object[].action` values

Each `on_object` entry is `{class, action, ...}`; a move can list several,
watching different classes with different reactions at once.

| `action` | Extra fields | Effect |
|---|---|---|
| `stop_and_hold` | `resume_condition` (optional, one of the `stop_condition` objects above; if omitted, resumes when the triggering class clears, `debounce_sec=1.0`) | Freezes mission progress: `mission.state -> HOLDING`, `/mpc/hold` published `true`. `resume_condition` (or the default) is evaluated relative to when the **hold began**, not the move. Resumes automatically once satisfied: `mission.state -> RUNNING`, `/mpc/hold` published `false`, move resumes exactly where it left off. |
| `reduce_speed` | `factor` (float, 0-1) | Scale `vdes` for the rest of the current move. **Stub** -- no `vdes`-override mechanism exists yet; logged once per move, otherwise ignored. |
| `reduce_speed_for` | `factor` (float), `duration_sec` (float) | Same, auto-restoring after `duration_sec`. **Stub**, same as above. |
| `abort_mission` | `reason` (string, required) | `mission.state -> ABORTED`, reason logged, `/mpc/hold` published `true` so mpc_corr actually stops. |
| `skip_to_move` | `move_id` (string, required) | Jumps `mission.current_index` to the named move. Target must be a real move id in the same mission -- checked at load time. |
| `log_only` | `message` (string, required) | No control effect -- just a structured log line. |

`stop_and_hold`/`abort_mission`/`skip_to_move` actively change mission flow and
take priority over normal move progression on the tick they fire.
`log_only` and the two stub actions (`reduce_speed`/`reduce_speed_for`) have no
control effect and deliberately do *not* block normal progression -- an
in-view object tagged only with `log_only` won't stall the mission.

## Safety interactions

- **`goal_pose` moves drive through mpc_corr, not Nav2.** `NavigateThroughPoses`
  (the `enable_nav2:=true` navigation lane -- see `behavior_executor_node`'s
  module docstring) is a separate, alternative navigation backend against
  Nav2's own `bt_navigator`/`planner_server`/`controller_server`, gated
  structurally on `enable_nav2`; it never touches `/mpc/goal_pose` or
  `mpc_corr.py` at all, and Nav2's own controller has no concept of
  `/mpc/hold` or mpc_corr's obstacle corridor. `goal_pose` mission moves
  publish straight to mpc_corr's `/mpc/goal_pose` instead, so they get exactly
  the same `/mpc/hold` and obstacle-corridor behavior `goal_distance` moves
  already have -- this was confirmed deliberately, not assumed (see chat log).
- **Emergency stays strictly highest priority.** The mission subtree sits
  below `emergency` and `handle_obstacle` in the root Selector -- both are
  ticked (and can preempt) before mission is ever reached, unchanged from
  before this feature existed.
- **Mission sits above `navigation`.** Both lanes ultimately drive mpc_corr
  through `/mpc/goal_distance`; positioning mission above navigation means
  whenever a mission is actively `RUNNING`/`HOLDING` it wins the root Selector
  outright, and `navigation`'s `HasMpcGoal` condition (a permanent latch: SUCCESS
  once *any* goal has ever arrived) never gets ticked that cycle -- so there's
  never a tick where both lanes are trying to be "the" driver at once.
- **`ABORTED`/`COMPLETE` always stop mpc_corr.** Both transitions publish
  `/mpc/hold(true)` rather than leaving mpc_corr mid-move chasing a goal the
  mission no longer intends to reach.
- **The hold mechanism (`/mpc/hold`, `std_msgs/Bool`) never touches
  `/mpc/goal_distance`.** Republishing `goal_distance` resets mpc_corr's own
  progress-tracking (`goal_start_xy`) on every message -- see
  `MPC_corr.py`'s `goal_distance_callback` -- so a hold implemented that way
  would silently reset the current move's progress every time it engaged.
  `/mpc/hold` instead short-circuits `mpc_corr`'s `control_loop` to publish
  zero speed/steering without touching any of that state, so releasing the
  hold resumes exactly where the move left off.

## Detected object classes

`detected_classes` (`class name -> last-seen timestamp + confidence`) is
written by `mission/detected_classes_bridge.py`, which subscribes to
**`/camera/detections`** (`vision_msgs/Detection2DArray`, published by
`f1tenth_perception`'s `yolo_detector_node`). This was confirmed by inspection,
not assumed: `/perception/obstacles_2d` (the other candidate) is geometry-only
(`x, y, r`) -- `f1tenth_messages/msg/Obstacle2D.msg` has no class field at all.

## Assumptions made while implementing this (flagged, not silent)

- **`object_seen`/`ObjectSeen` freshness window**: the schema has no
  configurable field for how long a sighting counts as "current" for
  `object_seen` (stop_condition) or the `ObjectSeen` BT condition (only
  `object_cleared`'s `debounce_sec` is configurable). Defaulted to **1.0s**
  (`condition_eval.DEFAULT_SEEN_FRESHNESS_SEC`), matching `object_cleared`'s
  own default.
- **Load trigger is a `std_msgs/String` topic, not a custom service** -- see
  "Loading a mission" above.
- **`/mpc/hold` (`std_msgs/Bool`) is a new topic**, added to `mpc_corr.py`
  specifically for this feature, confirmed with the requester before
  implementing (see git history / chat log for that exchange).
- **`CheckStopCondition` owns a single, simplified `/odom` subscription** (just
  `x, y`), not mpc_corr's full hw/sim dual-source selection logic -- adequate
  for distance-traveled bookkeeping, shared with `HandleObjectAction` via the
  blackboard so there's only one subscription, not a second copy of
  odom-fusion logic.
- **Tree ordering deviates from the original task's diagram in one place**:
  `PublishMoveGoal` is ticked *before* `CheckStopCondition`/`AdvanceMove`
  within the progress sequence, not after. The literal last-child ordering
  has no path to ever publish move 0's goal at mission start (`PublishMoveGoal`
  would only be reachable after a move-advance that hasn't happened yet). See
  that behaviour's own docstring for the full reasoning.
- **`MissionActive` succeeds for `HOLDING` too, not `RUNNING` only** (as
  literally specified) -- otherwise a hold can never be checked for resume at
  all, since the whole subtree (including `HandleObjectAction`, the only thing
  that can end a hold) would stop being ticked the instant `HOLDING` began. See
  `mission_active.py`'s docstring.
- **`ObjectSeen` takes no fixed `class_name`** -- a move's `on_object` is a
  list, so it checks the *current move's* list dynamically each tick instead
  of one hardcoded class, and records which entry matched for
  `HandleObjectAction` to read. See `object_seen.py`'s docstring.
- **`goal_pose`/`goal_reached` wiring** (added once mpc_corr gained a
  `/mpc/goal_pose` input): `PublishMoveGoal` publishes straight to mpc_corr,
  not through Nav2/`NavigateThroughPoses` -- those are separate,
  `enable_nav2`-gated backends that never interact (see "Safety interactions"
  above). `CheckStopCondition` tracks mpc_corr's shared `/mpc/goal_reached`
  topic with a per-move latch (see its own docstring) rather than
  re-implementing distance-to-goal itself. `HandleObjectAction`'s
  `resume_condition` does NOT get this live signal -- a `resume_condition` of
  type `goal_reached` still behaves as a stub there (always `RUNNING`, logged
  once) since a hold's resume was never wired to mpc_corr's reached-signal in
  this pass.
- **`MissionLoader`'s new `threading.Lock`** (added alongside
  `/mission/load_mission`/`/mission/abort_mission`) is not fixing an active
  race: `behavior_executor_node.main()` drives the BT tick timer, the
  `/mission/load_path` subscription, and both new services through one
  `rclpy.spin(node)` call on rclpy's default `SingleThreadedExecutor`, so none
  of them can ever actually run concurrently. The lock documents that
  invariant explicitly for the three entry points `loader.py` owns and is a
  safety net if the executor/callback-group setup ever changes; it does not
  reach into `HandleObjectAction`/`AdvanceMove`'s own mutations of the same
  `MissionRuntimeState` (out of scope for this pass), which stay safe today
  for the same single-threaded-executor reason, not because they share this
  lock.
