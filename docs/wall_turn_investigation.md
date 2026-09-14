# wall_turn investigation

Read-only reconnaissance, 2026-09-14, branch `scene-graph`. No source, config or
launch file was modified; nothing was launched; the LiDAR was not powered.

## Verdict

**Yes — the turn phase is externally observable, on `/mpc/wall_track`
(`f1tenth_messages/WallTrack`), with no lag and no proxy.** `MPC_corr._wall_track_tick`
publishes exactly one message per control tick (10 Hz, `self.ts = 0.1`) while and only
while `self.drive_cmd['mode'] == 'wall_turn'`, and returns without publishing otherwise
(`MPC_corr.py:2507-2523`). The message's own contract states this explicitly:
"Published by MPC_corr on /mpc/wall_track once per control tick while a wall_turn drive
command is active, valid or not, so silence means 'no wall_turn active' (or a dead node),
never 'no wall'" (`WallTrack.msg:1-5`). Better still, it carries the *commit* sub-state
too: `psi_commit` is the heading frozen at commit and is `NaN` before commit
(`WallTrack.msg:25`, `MPC_corr.py:2540`), so a subscriber can distinguish
*wall_turn issued but still driving straight* from *wall_turn committed and bending*
without any inference.

**Four caveats, all of which the new node must handle, and none of which are
hypothetical:**

1. The publisher only exists when `wall_track_enable` is true
   (`MPC_corr.py:1456-1458`; default `true`, `stack_params.yaml:725-727`). With it false
   there is no publisher at all and the topic is silent through an entire turn.
2. `control_loop` returns **before** `_wall_track_tick()` on `/mpc/hold`
   (`MPC_corr.py:2620-2624` vs `2641`) and on missing odometry (`2615-2619`). So silence
   also means "held" or "no odom", not only "no wall_turn".
3. `self.drive_cmd` is cleared only by the arrival of the *next* goal on one of the four
   goal topics (`_clear_drive_state`, `MPC_corr.py:1912`, called at `1632`, `1785`,
   `1892`, `2014`) — **never** by `/mpc/hold`. So on a terminal wall_turn move the phase
   never formally exits; the topic simply goes quiet because the hold branch short-circuits.
   Exit is therefore observable as *silence*, not as a final message.
4. A mission `"turn"` step (`TurnGoal` on `/mpc/goal_turn`) is **not** a wall_turn. It is
   dispatched through a synthetic `goal_pose` (`goal_turn_callback`, `MPC_corr.py:1845-1911`)
   and produces no `/mpc/wall_track` traffic at all. Only `DriveCommand` with
   `mode == "wall_turn"` on `/mpc/goal_drive` enters the phase.

Nothing subscribes to `/mpc/wall_track` today (`grep` over `src/` and `tools/`: only
`tools/wall_track_replay.py`, an offline replay harness that imports `wall_tracker`
directly). The new node would be its first consumer.

**For `psi_end`: `psi_end` as a name does not exist in the shipping code** (only as a
local variable in three tests). The corridor's terminal heading is `psiEnd`, a local in
`build_straight_corridor`, published into the corridor dict as `corridor["psiRef"]`
(`MPC_corr.py:3668`). There is no node, topic or parameter through which an external node
can apply a correction to it. See "Risks" below — this is the single largest structural
obstacle to the design in the Context section.

---

## Answers

| # | Question | Answer | Evidence |
|---|----------|--------|----------|
| 1.1 | Where is `wall_turn` implemented? | Split in two. The **rule** is a pure, rclpy-free module: `plan_wall_turn_step()`, a module-level function (not a class) returning a `WallTurnStep` NamedTuple. The **driver** is the `wall_turn` branch of `MPC_corr.build_straight_corridor`. Node executable: `mpc_corr` (node name `mpc_corr`). | `mpc_controller/wall_turn.py:143` `def plan_wall_turn_step(...)`; `MPC_corr.py:3239` `if drive_cmd['mode'] == 'wall_turn':`; `MPC_corr.py:3296` `wall_step = plan_wall_turn_step(...)`; `mpc_controller/setup.py:28` `'mpc_corr = mpc_controller.MPC_corr:main'`; `f1tenth_control/launch/mpc_corr.launch.py:213-214` `executable='mpc_corr', name='mpc_corr'` |
| 1.2 | Formal state machine, conditionals, or BT/plugin? | **Implicit conditionals over four plain node attributes.** No enum, no transition table, no py_trees construct. The attributes are `self.drive_cmd` (a dict, or `None`), `self.turn_progress_rad`, `self.wall_turn_committed`, `self.wall_turn_commanded_rot`. | `MPC_corr.py:866-873`: `self.turn_progress_rad = 0.0` / `self._turn_progress_last_yaw: Optional[float] = None` / `self.wall_turn_committed = False` / `self.wall_turn_commanded_rot: Optional[float] = None` |
| 1.3 | Is there a state enumeration to quote? | The closest thing is the **mode string set**, validated in two places. Not a state enum — two driving modes, not phases. | `mission_config.py:47` `DRIVE_MODES = {'straight', 'wall_turn'}`; `MPC_corr.py:1999` `if mode not in ('straight', 'wall_turn'):` |
| 1.4 | What triggers **entry**? | Two nested events. (a) *Phase entry*: a `DriveCommand` with `mode=="wall_turn"` arrives on `/mpc/goal_drive`; `goal_drive_callback` stores it and zeroes the progress counters. (b) *Commit* (the turn actually starts bending): `d_avail <= k_safety * R_min * abs(dpsi_rem)`, latched thereafter. | (a) `MPC_corr.py:2035-2047` `self.drive_cmd = {...}` then `self._reset_turn_progress()`; `MPC_corr.py:2028-2031` rejects `mode "wall_turn" senza turn_sign`. (b) `wall_turn.py:180` `committed = committed or d_avail <= k_safety * r_min * abs(dpsi_rem)`; the latch is documented `wall_turn.py:22-32` ("stays committed for the rest of the move") |
| 1.5 | What triggers **exit**? Geometry, time, or planned-path completion? | **Neither geometry nor time nor path completion inside the controller.** `mpc_corr` never self-terminates a drive command. Exit is externally decided: the behaviour tree's `stop_condition` fires and the tree publishes either the next move's goal or `/mpc/hold`. Only a new goal actually clears `drive_cmd`. For `wall_turn.json` / `drive_stop_2m_from_wall.json` that condition is `orientation_delta` 90.0 deg, evaluated on the BT's own **unwrapped accumulated** yaw. | `MPC_corr.py:1955-1966` (docstring) "never sets goal_reached, NEVER publishes /mpc/goal_reached and NEVER self-terminates. The behaviour tree's stop_condition is the sole authority on when the move ends"; `MPC_corr.py:2643-2660` the drive branch "with no termination check of its own"; `_clear_drive_state` `MPC_corr.py:1912`, called only at `1632/1785/1892/2014` (the four goal callbacks) — **not** from `hold_callback` (`MPC_corr.py:1573-1576`); `missions/wall_turn.json:8` `"stop_condition": {"type": "orientation_delta", "value": 90.0}`; `condition_eval.py:362-379` `return abs(ctx.turn_accum_deg) >= abs(float(p['value']))` |
| 1.6 | **Is the phase published / served / parameterised / in diagnostics?** | **YES — published.** Topic `/mpc/wall_track`, type `f1tenth_messages/WallTrack`, **10 Hz** (one per control tick; `self.ts = 0.1`), QoS **depth 10, default RELIABLE, VOLATILE — not transient_local / not latched**. The field that carries the phase is *the existence of the message itself* (publish is gated on the mode); the field that carries the **commit** sub-state is `psi_commit` (NaN until commit). `valid`/`provenance` carry whether a wall is being tracked. | `MPC_corr.py:1456-1458` `self.wall_track_pub = self.create_publisher(WallTrack, '/mpc/wall_track', 10)` (plain int depth ⇒ `rclpy` default reliable/volatile); `MPC_corr.py:2507-2523` the mode gate: `if drive_cmd is None or drive_cmd.get('mode') != 'wall_turn': return`; `MPC_corr.py:2641` called once per `control_loop`; `MPC_corr.py:468` `self.ts = 0.1`; `MPC_corr.py:1464` `self.timer = self.create_timer(self.ts, self.control_loop)`; `WallTrack.msg:1-5`, `:15-28`; `MPC_corr.py:2540` `msg.psi_commit = float(tracker.psi_commit) if tracker.armed else math.nan` |
| 1.7 | Is the phase exposed any *other* way? | `NOT FOUND` as a service, action, or parameter. Partially, as **log lines only**: `CORR/wall_turn`, `WALL/commit`, `WALL/select`, `WALL/track`, `DRIVE`, `DRIVE/clear`. These are not machine-readable off a topic. `/mpc/solver_status` (`MpcSolverStatus`) carries solver outcome, not mode. `/behavior/tree_status` (`BehaviorTreeStatus`) carries which BT *lane* won a tick, not which move or drive mode. `/mission/status` (`MissionStatus`, **transient_local/latched**, depth 1) carries only `state` (IDLE/LOADED/RUNNING/…), `json_path` and `emergency_stop_active` — no move id, no drive mode. | `MPC_corr.py:3333-3344` (`CORR/wall_turn` log); `MPC_corr.py:3323-3326` (`WALL/commit`); `BehaviorTreeStatus.msg:16-40`; `MissionStatus.msg:14-26`; `loader.py:135-140` `MISSION_STATUS_QOS = QoSProfile(depth=1, RELIABLE, TRANSIENT_LOCAL, KEEP_LAST)` |
| 1.8 | If not published, what externally visible signals change at entry/exit? | Not applicable (it *is* published), but for redundancy the ranked alternatives are listed. **`/mpc/goal_drive`** (`DriveCommand`, depth 10, reliable, volatile, **not latched**) changes *before* the transition — it is the command that causes it; but it is sent **once per move entry**, so a node that starts late or drops the message never sees it. **Commanded steering on `/drive`** changes *after* commit (it is the solve's output) and is indistinguishable from any other corridor bend. **`/mpc/corridor_markers`** changes shape at the rebuild after commit — *after* the transition, and at the 1 Hz rebuild cadence, so up to 1 s late. | `publish_move_goal.py:108-109` `self.goal_drive_pub = self.node.create_publisher(DriveCommand, self._goal_drive_topic, 10)` and `:159` published once in `update()`; `DriveCommand.msg:1-2` "published once per mission 'drive' step entry"; `MPC_corr.py:1444-1448` corridor markers publisher; `stack_params.yaml:622-624` `corridor_update_period: default: 1.0` |
| 1.9 | Existing pattern for one node observing another's state? | **Yes, two, and they agree.** (a) `check_stop_condition` (f1tenth_behavior) subscribes to `mpc_corr`'s own diagnostic topics `/mpc/min_obstacle_distance` and `/mpc/goal_reached` and shares them onto the BT blackboard. (b) `/mpc/wall_track` and `/mpc/solver_status` were both added specifically as "a diagnostic feed nothing controls off" with default reliable QoS. The new node should match (b): plain `create_publisher(Type, topic, 10)`, one message per tick including the empty ones, never withheld. | `check_stop_condition.py:17-18` "Also owns /mpc/min_obstacle_distance (published by mpc_corr itself)"; `:67-70` "/mpc/goal_reached (std_msgs/Bool, published by mpc_corr …)"; `MPC_corr.py:1450-1455` "A diagnostic feed like /mpc/solver_status: nothing controls off the topic, but the trace is the evidence …, so default reliable QoS" |
| 2.1 | Where is `psi_end` defined / computed / consumed? | The identifier `psi_end` **does not exist in shipping code** — only as a local in `test_warm_start.py:79`, `test_corridor_turn_shape.py:97` and a test name. The real quantity is `psiEnd`, a **local variable** in `build_straight_corridor`. Defined/computed in five mutually exclusive branches; consumed (i) as the corridor's terminal-heading normal/tangent for the wall Béziers, (ii) exported as `corridor["psiRef"]`, (iii) read by `mpc_solver`'s terminal yaw cost. | Computed: `MPC_corr.py:3310` (wall_turn) `psiEnd = psi0 + wall_step.dpsi_this`; `:3361` (straight) `psiEnd = psi_base`; `:3379` (goal_pose) `psiEnd = math.atan2(gy - Y0, gx - X0)`; `:3498` (goal_distance, map anchor) `psiEnd = float(anchor_pose[2])`; `:3535` (bootstrap) `psiEnd = float(psi_ref) if psi_ref is not None else psi0`. Consumed: `:3604` `n1 = np.array([-math.sin(psiEnd), math.cos(psiEnd)])`, `:3612` `e1 = ...`, `:3668` `"psiRef": float(psiEnd)`; `mpc_solver.py:678-687` `terminal_heading_target()` |
| 2.2 | What does it represent — absolute heading in which frame, or an offset? | **An absolute heading in the odom frame.** Every branch assigns either the live yaw plus an increment, a frozen absolute anchor heading, or an absolute bearing. Corridor points are absolute odom coordinates. The *offset* is carried separately as `corridor["psiRefTurn"]` = `dpsi_this`, the signed rotation measured from `psiStart`. | `MPC_corr.py:3310` `psiEnd = psi0 + wall_step.dpsi_this` where `psi0 = float(x[2])` (`:3193`) is the state yaw; `MPC_corr.py:2538` `msg.header.frame_id = self.odom_frame`; `MPC_corr.py:321` `self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)`; `_corridor_line_marker`: "Points are already absolute odom-frame coordinates"; `MPC_corr.py:3669-3678` `"psiRefTurn": (None if turn_remaining is None else float(turn_remaining))` |
| 2.3 | Is any correction / bias / feedback already applied to it? | **Three, and all three must be respected by any fourth.** (a) **The wall_turn increment itself** — `psiEnd` is *not* the commanded end heading, it is live yaw plus only the slice the distance and horizon can carry (`min(abs(dpsi_rem), dpsi_by_dist, dpsi_by_horizon)`). (b) **The ratchet** — the end heading, measured as rotation from move start, may never retreat toward the start heading within a move; it sets `WallTurnStep.held`. (c) **The horizon clip** — `abs(dpsi_this) <= dpsi_by_horizon` unconditionally, applied *after* the ratchet, and "nothing overrides it". There is also a **map→odom anchor correction** on the goal_distance branch (`_refresh_goal_anchor`), but it does not touch the wall_turn branch. `lat_off` at `:3521` is **diagnostic only** and explicitly "not acted on any more". | `wall_turn.py:187-189` the `min()`; `wall_turn.py:197-205` the ratchet incl. `held = True`; `wall_turn.py:206-207` `if abs(dpsi_this) > dpsi_by_horizon: dpsi_this = math.copysign(dpsi_by_horizon, dpsi_this)`; `wall_turn.py:61-65` "THE JUMP BOUND is the horizon cap, and nothing overrides it"; `MPC_corr.py:3513-3521` "Diagnostic only: … Deliberately not acted on any more" |
| 2.4 | How would a second correction combine with those? | Sequentially, and **it would be clipped last**. Any additive `psi_end` correction applied inside `build_straight_corridor` before line 3310 flows through the ratchet and then through the horizon clip, so a correction larger than `dpsi_by_horizon` (= `N*ts*v_ref/R_min` = `20*0.1*0.5/1.069` ≈ **0.935 rad** at defaults, but ≈ 0.749 rad at the `speed: 0.4` the wall_turn missions command) is silently truncated. A correction applied *after* line 3310 (i.e. to `psiEnd` only) would **desynchronise** `psiEnd` from `psiRefTurn` and from the S-curve `dpsi`, which the code explicitly forbids. | `MPC_corr.py:3272-3277` "dpsi_this is the ONE source of truth for this corridor: it is psiEnd - psiStart (terminal heading cost), psiRefTurn (the solver's signed branch) AND the centreline's S-curve dpsi below. Feeding the increment to one of them while another still swung the full angle would set them against each other."; `wall_turn.py:174-175` `horizon_len = max(n_steps*ts*v_ref, 0.0)`; `missions/wall_turn.json:7` `"speed": 0.4` |
| 2.5 | How is a corridor represented? | A **plain Python dict** (no message type, no class) holding parallel numpy arrays of length `corr_N = 120`: a centreline (`xc`,`yc`), two boundary polylines (`xL`,`yL`,`xR`,`yR`) built as cubic Béziers, unit tangent/normal (`tx`,`ty`,`nx`,`ny`), a per-sample `halfWidth`, plus scalars `psiRef`, `psiRefTurn`, `L`, `psiStart`, `t`, `Pend`, `dFront`, `dpsi`. All in odom coordinates. It is **published for visualisation only**, as three `visualization_msgs/MarkerArray` LINE_STRIPs on `/mpc/corridor_markers`. | `MPC_corr.py:3661-3687` the dict literal; `:3598-3641` the Bézier construction; `:614` `self.corr_N = 120`; `MPC_corr.py:1444-1448` markers publisher |
| 2.6 | **Is corridor width known a priori or estimated?** | **A priori, and hardcoded as two bare literals in `MPC_corr.py` — they are not ROS parameters and not in `stack_params.yaml`.** The corridor is *synthetic*: it is drawn around the reference centreline at a fixed half-width that widens linearly (via the Bézier) from `corr_wmin` at the car to `corr_wmax` at the far end. Nothing measures it. | `MPC_corr.py:615-616` `self.corr_wmin = 0.4333` / `self.corr_wmax = 0.7667`; `:602` "corr_wmin/corr_wmax are HALF-widths, not full widths"; `:585-586` "Narrowed to 1/3 width on request (2026-09-07): corr_wmin 1.3 -> 0.4333, corr_wmax 2.3 -> 0.7667"; `stack_params.yaml:1140-1141` confirms "both bare literals in MPC_corr.py"; `MPC_corr.py:3600-3609` `w0 = self.corr_wmin` … `P_L1 = C1 + w1 * n1` |
| 2.7 | Are both corridor walls available, or typically only one? | **Both, always — but neither is sensed.** `xL/yL` and `xR/yR` are always computed and are symmetric offsets of the centreline. The only **sensed** wall representations in the stack are single-wall: `/mpc/wall_track` (one line, the front wall), `/perception/lidar_front_wall{,_virtual}` (one line, the forward sector), and `/costmap/boundaries` (up to 3 half-planes). | `MPC_corr.py:3605-3609`; `WallTrack.msg:23-24`; `WallLineFit.msg:23-31`; `stack_params.yaml` MPC BOUNDARY block; `mpc_solver.py:176-179` "THREE" slots |
| 3.1 | Footprint: length, width, `base_link` position — URDF | `base.xacro` places `base_link` **at the rear axle, on the ground plane**, laterally centred. Wheelbase (rear→front axle) **0.325 m**, track 0.2 m, wheel radius 0.05, ride height 0.05. The chassis geometry is an STL mesh with a scaled/offset visual origin — no axis-aligned box is declared in the URDF itself. | `base.xacro:5-9` "base_link is the kinematic root: it sits on the ground plane, at the rear axle, centred laterally"; `robot.urdf.xacro:39-43` `wheelbase 0.325` / `track 0.2` / `wheel_radius 0.05` / `wheel_width 0.045` / `ride_height 0.05`; `base.xacro:38-40` the mesh origin/scale |
| 3.2 | Footprint — separate planner/controller parameter, and does it disagree? | **Yes, and there are three disagreeing sets.** (a) `swept_clearance_*` (perception) gives the only explicit rectangle: front `+0.443`, rear `-0.082`, half-width `0.136` (⇒ 0.525 m × 0.272 m), rear axle at `x=0`. (b) `mpc_corr` carries **no rectangle at all** — only a scalar `car_radius = 0.20`. (c) Two **documented contradictions the repo itself flags**: the wheelbase is 0.305 (MPC/swept) vs 0.325 (URDF) vs 0.3302 (F1TENTH spec); and `base_link` is "at the rear axle" (`base.xacro`) vs "centered between the axles, 0.07 m above the ground" (`description.launch.py`, `sensors.xacro`, `camera.launch.py`). | `stack_params.yaml:510-517` `swept_clearance_body_front_x_m: 0.443` / `_rear_x_m: -0.082` / `_half_width_m: 0.136`; `:528-530` `swept_clearance_rear_axle_x_m: 0.0` — "0 per the URDF (base.xacro: base_link at the rear axle). Some docstrings say base_link is between the axles (-0.1525 here); change only on a measurement."; `swept_corridor.py:16-31`; `MPC_corr.py:1147-1157` `self.car_radius = float(self.declare_parameter('car_radius', 0.20).value)`; `stack_params.yaml:841-843`; `description.launch.py:35-36` |
| 3.3 | Wheelbase and maximum steering angle | Wheelbase used by the MPC and by `R_min`: **0.305 m** (`mpc_wheelbase_m`). Steering bounds are **asymmetric**: `delta_max = +0.278` rad (+15.93°), `delta_min = -0.283` rad (−16.22°). Both are *derived by inverting the servo clip through the calibration*, not measured; `test_steering_limits.py` recomputes them from `steering_calibration.yaml` and fails on drift. **A competing pair (−0.264/+0.314) exists and is NOT used**, in `steering_offset_calibration_node.py`'s declared defaults; the repo documents why it is wrong (+0.314 maps to servo 0.1064, below `servo_min` 0.15, i.e. not commandable). | `stack_params.yaml:841-843` `mpc_wheelbase_m: default: 0.305`; `:844-849` `mpc_steering_angle_min_rad: -0.283` / `mpc_steering_angle_max_rad: 0.278`; `:806-818` the derivation from `servo_min 0.15 / servo_max 0.8318 / offset 0.4874 / gain −1.2135`; `:819-833` the rejected pair |
| 3.4 | Minimum turning radius — stated or derivable? | **Both.** Derived at runtime by `min_turn_radius(wheelbase, bound) = L / tan(abs(delta))`; the resulting numbers are also **stated in prose in two places**: **1.069 m turning left, 1.049 m turning right**. | `wall_turn.py:108-110` `return float(wheelbase) / math.tan(abs(float(steering_bound)))`; `wall_turn.py:39-42` "R_min is 1.069 m turning left and 1.049 m turning right"; `stack_params.yaml:676-678` "R_min = mpc_wheelbase_m / tan(steering bound for the direction): 1.069 m left (0.278), 1.049 m right (-0.283)" |
| 3.5 | Sign convention: positive `psi` / positive yaw | **Standard ROS right-hand rule: positive = left = CCW.** Stated identically in the message definitions, the mission schema and the perception geometry. Steering shares the sense: positive commanded steering angle = left. | `DriveCommand.msg:23-25` "+1.0 = left/CCW, -1.0 = right/CW"; `TurnGoal.msg:6` "+ = left/CCW, - = right/CW (right-hand rule, standard ROS yaw convention)"; `mission_config.py:136` and `:169`; `MPC_corr.py:915` "convention (+ = left, same sense as psi)"; `swept_corridor.py:39` "kappa = tan(delta) / wheelbase, + = left"; `steering_calibration_node.py:372` "positive commanded angle = left" |
| 3.6 | Sign convention: lateral distance — is left positive? | **Left is positive, consistently, in every lateral quantity found.** The corridor's left wall is `C + w*n` with `n = (−sin ψ, cos ψ)`; the diagnostic lateral offset uses the same projection; wall bearings are documented "0 ahead, + left". `WallTrack.d_wall` is the one exception by design: it is an **unsigned magnitude** (`abs`), with direction carried separately in `normal_yaw`. | `MPC_corr.py:3604-3608` `n1 = np.array([-math.sin(psiEnd), math.cos(psiEnd)])` then `P_L1 = C1 + w1 * n1`; `MPC_corr.py:3521-3522` `lat_off = (-(X0 - anchor[0]) * math.sin(psiEnd) + (Y0 - anchor[1]) * math.cos(psiEnd))`; `WallLineFit.msg:26` "normal_angle … base_link [rad], 0 ahead, + left"; `WallDetection.msg:4`; `lidar_front_wall.py:17`; `wall_tracker.py` docstring `d_wall = abs(c - n . p_bumper)` |
| 3.7 | Frames and frame tree | `base_link` (root, per `base.xacro` on the ground at the rear axle) → `base_footprint` (identity) → `chassis`/wheels; plus three sensor edges published by **static_transform_publisher nodes on real hardware, not by the URDF**: `base_link→laser`, `base_link→zed2_camera_link` (camera.launch.py), `base_link→imu` (vesc.launch.py, identity). In sim, `sensors.xacro` supplies them instead and the static publishers are suppressed. | `base.xacro:26-33`; `description.launch.py:22-27`; `sensors.xacro:10-13` |
| 3.8 | Laser frame name and its static transform to `base_link` | Frame name **`laser`** (also `laser_frame_id: "laser"` in the driver config). Real hardware: `x=0.12, y=0.0, z=0.20, yaw=0.0, pitch=0.0, roll=0.0` (positional args are **x y z YAW PITCH ROLL**). **These are explicitly ruler-measured placeholders, not a calibration**, and roll=pitch=0 is flagged as an unverified assumption. Note the **sim URDF disagrees**: `sensors.xacro` still has the pre-remount rear-facing `(-0.12, 0, 0.15), yaw=pi`. | `description.launch.py:186-193` `arguments=['0.12','0.0','0.20','0.0','0.0','0.0','base_link','laser']`; `:42-58` "PHYSICAL REMOUNT (rear-facing -> front-facing) … NEW values are approximate, ruler-measured placeholders, NOT a real calibration"; `sensors.yaml:9` `laser_frame_id: "laser"`; `sensors.xacro:11` |
| 3.9 | Is `odom` published by the EKF or by something else? | **By the local `robot_localization` EKF**, with `world_frame: odom` and `publish_tf: true`, i.e. it owns the `odom → base_link` edge; `vesc_to_odom_node`'s own `publish_tf` is left false. `mpc_corr` reads **`/odometry/filtered`** (local EKF, 50 Hz) because `localization_source` defaults to `ekf`; with `raw_odom` it would read `/odom`. A separate **global** EKF instance publishes `map → odom`. | `ekf.launch.py:4` "vesc_to_odom_node's own publish_tf is left false"; `localization.launch.py:28-29` "ekf.yaml's own world_frame reverted from map to odom, publish_tf stays true"; `stack_params.yaml:48-50` `localization_source: default: ekf`; `MPC_corr.py:1237-1245` `get_odom_topic()`; `MPC_corr.py:910-911` "/odometry/filtered (local EKF, 50 Hz)" |
| 4.1 | Does the MPC accept obstacle/boundary constraints? | **Yes, three distinct mechanisms.** (a) **Hard linear half-plane rows** in the RTI/OSQP path — up to `boundary_max_sources` (default 3) `(normal_x, normal_y, offset)` triples per stage, padded to exactly 3 with an inert sentinel, optionally softened by a slack block. (b) **Soft point-obstacle cost** `w_obs` over `/perception/obstacles_2d`. (c) **Corridor half-width** as a per-stage soft cost, not a constraint. | `mpc_solver.py:40-45`, `:57-69`, `:202-232`, `:1023-1055`; `mpc_solver.py:325-327` "boundaries: up to 3 (normal_x, normal_y, offset) hard boundary constraints"; `mpc_solver.py:602` `half_w = float(corridor["halfWidth"][idx])` |
| 4.2 | Where is that interface defined? | Message: `f1tenth_messages/BoundaryConstraint` + `BoundaryConstraintArray`. Wire topic into the MPC: **`/costmap/boundaries`**. Producers today: `f1tenth_costmap`'s `costmap_boundary_node`, plus `f1tenth_perception`'s `wall_detector_node` (`/perception/front_wall_boundary`) and `lidar_boundary_node` (`/perception/lidar_boundaries`). | `MPC_corr.py:1304-1308` `self.sub_costmap_boundaries = self.create_subscription(BoundaryConstraintArray, '/costmap/boundaries', self.costmap_boundaries_callback, 10)`; `BoundaryConstraint.msg:1-11`, `:31-32` `float64[2] normal` / `float64 offset`; `BoundaryConstraintArray.msg:9-10` |
| 4.3 | Existing hard-boundary representation to compute clearance against? | **`f1tenth_messages/BoundaryConstraint`** — `normal · (x,y) <= offset`. Two things to get right: its `normal` points **toward** the wall (free side is where `normal·p` is small), which is the **opposite** of `WallDetection.normal`; and `offset` is the **raw** geometric boundary, deliberately not pre-shrunk by any margin — consumers subtract `car_radius + obstacle_safety_margin_m` themselves. It carries **no header**; the array does, and both current producers emit **base_link**, so an odom-frame consumer must transform. And it is **live**: `use_hard_boundary_constraints` has been `true` on the vehicle all along. | `BoundaryConstraint.msg:1-2`, `:13-22`, `:24-29`; `BoundaryConstraintArray.msg:1-7`; `stack_params.yaml:762-772` "TRUE IS NOW A CHOSEN VALUE, NOT AN INHERITED ONE (2026-09-09) … the vehicle has been solving WITH costmap_boundary_node's wall rows since the key was added, and nobody chose that" |
| 4.4 | Existing clearance / margin / safety-distance computation to extend? | **Yes — five, and the two most relevant are already written and unused.** (i) **`swept_corridor.clearance()`** — *this is the swept-footprint clearance the Context section describes*: rear-axle arc length before the **body** (a real rectangle, margin-widened sideways only) touches a point, computed along the steering arc, LiDAR + ZED fused, published on `/perception/swept_clearance` by `swept_clearance_node`. It has a launch file and an entry point but **is registered in no supervisor component and is included by no other launch file — it never runs on the car today**. (ii) **`lidar_front_wall_node`** — perpendicular base_link-to-wall distance with measured/dead-reckoned/seeded provenance on `/perception/lidar_front_wall_virtual`; runs (its own component) but **nothing consumes it**. (iii) `MPC_corr.compute_predicted_clearance` over the predicted rollout → `/mpc/predicted_min_clearance`. (iv) `compute_robot_obstacle_distance` / `compute_forward_obstacle_distance` → `/mpc/min_obstacle_distance{,_forward}`, consumed by the BT. (v) `front_clearance_node` → `/perception/front_distance` (wall-only, objects masked out) and `/perception/front_clearance` (min with obstacles). | `swept_corridor.py:1-6`, `:56-60`, `:206`; `swept_clearance_node.py:2-24` "NOTHING SUBSCRIBES TO THIS YET"; `swept_clearance.launch.py:83-92`; `setup.py:50` entry point; **absent** from `components.yaml` and from `perception.launch.py` (verified by grep); `lidar_front_wall_node.py:1-20` "Nothing consumes either topic yet, deliberately"; `MPC_corr.py:3173` / `:1398`; `MPC_corr.py:3119`, `:3132`; `front_clearance_node.py:14-34` |
| 4.5 | Controller update rate, and expected input rates | Control loop **10 Hz** (`ts = 0.1`, hardcoded, **not** a ROS parameter). Horizon `N = 20` (hardcoded) ⇒ 2.0 s. Corridor rebuild **1 Hz** (`corridor_update_period = 1.0`, a real parameter). Inputs: `/odometry/filtered` 50 Hz; `/scan` 40 Hz; `/perception/front_distance` ~17 Hz (one per ZED depth frame). Staleness is enforced only for `front_distance` (0.5 s ⇒ treated as unknown). | `MPC_corr.py:468-469` `self.ts = 0.1` / `self.N = 20` (no `declare_parameter` for either — verified by grep); `MPC_corr.py:1464`; `stack_params.yaml:622-624`; `MPC_corr.py:910-911` (50 Hz); `sensors.yaml:26` "at 40 Hz"; `stack_params.yaml:692-694` `corr_wall_turn_front_distance_max_age_sec: 0.5`, "front_clearance_node publishes once per ZED depth frame (~17 Hz)" |
| 5.1 | Supervisor / lifecycle architecture; how a component declares itself | `component_supervisor_node` spawns and independently restarts named "components". A component **declares itself as one entry in `f1tenth_bringup/config/components.yaml`**, a list of `{package, launch_file, args}` — each run as `ros2 launch <package> <launch_file> [args...]` in its **own session** (`start_new_session=True`, so pgid == pid). Whether it auto-starts is decided **in the node, not the YAML**, by membership in `_ALWAYS_AUTO_START` / `_NEVER_AUTO_START` / `_CONDITIONAL_AUTO_START`. A watchdog polls each subprocess with a per-process restart budget (`max_auto_restarts` 3 in `restart_budget_window_sec` 60). Services: `RestartComponent`, `ComponentControl` (SHUTDOWN/START/RESTART). | `component_supervisor_node.py:1-20`; `components.yaml:1-4`; `component_supervisor_node.py:948-960` `_start_component`; `:308-319` `_ALWAYS_AUTO_START` / `_NEVER_AUTO_START`; `:483-493`; `:665-666` |
| 5.2 | **Exact restart mechanism, and its scope** | `_stop_component(name)` iterates **every tracked process registered under that name** and `os.killpg(pgid, SIGINT)`s each, escalating to `SIGKILL` after `restart_timeout_sec` (10 s). **The restart unit is the component, and every launch file under it dies together.** | `component_supervisor_node.py:1092-1096` (docstring) "SIGINT (then SIGKILL if needed) every tracked process for `name`"; `:1103-1108` `for proc in self._processes.get(name, []): … pgid = proc.popen.pid … os.killpg(pgid, signal.SIGINT)`; `:1121-1126` the SIGKILL escalation |
| 5.3 | **Can a new `/scan` subscriber avoid being able to trigger a `urg_node` restart?** | **Yes — and the stack already has the exact precedent, with tests enforcing it.** `urg_node` lives in `lidar.launch.py` under the `perception` component. `lidar_front_wall.launch.py` was deliberately made **its own component** (`lidar_front_wall`) for precisely this reason, and two tests fail if that placement changes. The new node must follow the same shape: its own launch file, its own component entry, added to `_ALWAYS_AUTO_START`. | `components.yaml:74-76` "lidar.launch.py runs urg_node, the only /scan publisher, which the e-stop (IsProximityTooClose) reads. Everything in this component restarts together, so do not add /scan consumers here"; `components.yaml:82-91`; `lidar_front_wall.launch.py:3-19` "THIS FILE IS ITS OWN SUPERVISOR COMPONENT ON PURPOSE… Folding this into lidar.launch.py looks like a tidy-up. It is the incident this comment exists to prevent."; `component_supervisor_node.py:308-313`; `test_lidar_front_wall_component.py:34-54` |
| 5.4 | Where node parameters live, and the naming convention | Single source of truth: **`src/f1tenth_params/config/stack_params.yaml`**, one top-level key per parameter as `{default, description}`, resolved through `f1tenth_params.param_defaults` (`get_default` / `get_value` / `get_path_default`), which reads the **installed share copy**. Convention: keys are prefixed with the node's name, and the launch file **strips the prefix** when forwarding, so the node declares the bare name. Two sub-conventions coexist: launch-arg-backed (swept_clearance, lidar_front_wall) and **no-launch-arg** (`corr_wall_turn_*`, `wall_*` — `mpc_corr` reads these straight into `declare_parameter` defaults; "No launch arg: edit here and relaunch"). | `param_defaults.py:1-4`, `:36-54`; representative block quoted below; `swept_clearance.launch.py:66-72` `default, description = get_default('swept_clearance_' + name)` … `params[name] = ParameterValue(LaunchConfiguration('swept_clearance_' + name), value_type=value_type)`; `stack_params.yaml:672-674` and `:701-703` "No launch arg: edit here and relaunch" |
| 5.5 | Is there an e-stop path, and what are its inputs? | **Yes.** `f1tenth_behavior`'s BT `emergency` lane, condition `IsProximityTooClose`: **raw `/scan` only**, read as two disjoint windows — a ±45° front cone tripping below `proximity_front_threshold_m` (0.15 m) and the remaining ~270° tripping below `proximity_side_threshold_m` (0.15 m), on the **raw scan frame** (angle 0 = the laser's own forward axis, which is genuinely forward post-remount). It previously used ZED depth for the front and no longer does. It publishes on the `safety_stop` mux lane (priority 200), which outranks the MPC's `navigation` lane (priority 10). | `is_proximity_too_close.py:1-20`, `:21-33`, `:34-40`; `:209-210` `self.node.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)`; `stack_params.yaml:1178-1182`; `stack_params.yaml:1159-1161` (the `safety_stop` mux lane, priority 200); `MPC_corr.py:1354-1356` (the MPC's own `navigation` lane, priority 10) |
| 5.6 | Does adding another `/scan` subscriber affect the e-stop? | **No, provided the new node uses best-effort sensor QoS and stays out of the `perception` component.** `mpc_corr` already added exactly such a subscriber and documented why it is harmless: best-effort sends `urg_node` no acknowledgements and so cannot back-pressure it or the e-stop. The real hazard is not the subscription but **co-locating in a restartable unit with `urg_node`** (5.3) and **CPU contention with the e-stop's core** — the stack pins nodes off `behavior_executor_node`'s core 4 for that reason. | `MPC_corr.py:1322-1334` "Best-effort sensor QoS: compatible with urg_node's reliable publisher, sends it no acknowledgements, and cannot back-pressure it or the e-stop (f1tenth_behavior's IsProximityTooClose) that reads the same topic"; `MPC_corr.py:1340-1341` `qos_profile_sensor_data`; `lidar_front_wall.launch.py:34-38` "Core 5 is the one core no pinned node reserves … and in particular not the behavior executor's core 4, which hosts the e-stop"; `swept_clearance.launch.py:77-78` |
| 5.7 | Existing test layout, and how tests are invoked | `test/` beside the package source, plain pytest modules (`test_*.py`), plus the ament boilerplate (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`). Invoked by `colcon test`. **`setup.py` MUST declare `extras_require={'test': ['pytest']}` or colcon runs nothing and reports green** — a workspace-wide defect fixed in `aaae377`; `tests_require` does *not* work. Both `mpc_controller` and `f1tenth_perception` declare it correctly. Tests are pure logic: no rclpy spin-up, no hardware. Cross-package tests are allowed and used (`f1tenth_bringup/test/test_lidar_front_wall_component.py` imports the supervisor's constant directly). | `CLAUDE.md:5-45`; `mpc_controller/setup.py:20-24`; `f1tenth_perception/setup.py:32-41`; `ls src/f1tenth_control/mpc_controller/test/`; `test_lidar_front_wall_component.py:8-19` |

### Representative existing parameter block (5.4)

From `src/f1tenth_params/config/stack_params.yaml:725-741` — the block a new
clearance node's should be modelled on (prefix, `default` + `description`, and a
description that records *where the number came from* and *what to tune against*):

```yaml
wall_track_enable:
  default: true
  description: "Run the wall tracker for wall_turn moves. false: no /scan subscription in mpc_corr, no /mpc/wall_track, and the increment runs on dFront alone as before."
wall_normal_tol_rad:
  default: 0.39
  description: "Largest angle [rad] between a candidate line's normal (folded to [0, pi/2], a fit gives the sign only up to a flip) and the heading frozen at commit for it to count as the FRONT wall. 0.39 (22.5 deg) is wide enough for an oblique approach and narrow enough that a side wall (90 deg off) can never qualify. If real walls are being rejected, this is the first thing to tune, and mpc_corr's WALL/select rejection log (with each candidate's angle_err) is what to tune it against."
wall_min_span_m:
  default: 0.5
  ...
wall_assoc_dist_m:
  default: 0.3
  ...
```

---

## Conflicts and ambiguities

1. **Wheelbase: three values, all live.** `mpc_wheelbase_m: 0.305` and
   `swept_clearance_wheelbase_m: 0.305` (`stack_params.yaml:841`, `:507`) vs URDF
   `wheelbase 0.325` (`robot.urdf.xacro:39`) vs the F1TENTH spec 0.3302.
   **0.305 wins at runtime** for anything MPC- or clearance-related: `MPC_corr`'s
   `params['L']` and therefore `R_min` read the parameter, never the URDF
   (`MPC_corr.py:575-576`, `:3300`). The URDF's 0.325 only shapes the TF tree /
   visualisation. The repo itself calls 0.305 "unmeasured on this car".

2. **`base_link` position: rear axle or between the axles?** `base.xacro:5-9` says
   rear axle, on the ground. `description.launch.py:35-36`, `sensors.xacro:10-13`
   and `camera.launch.py` say "centered between the axles, 0.07 m above the ground".
   **The URDF joints win** — `swept_clearance_rear_axle_x_m: 0.0` is chosen on that
   basis and explicitly says the alternative would be −0.1525
   (`stack_params.yaml:528-530`). This is a **0.15 m unresolved ambiguity in the
   longitudinal origin**, which propagates directly into any bumper-referenced
   distance. `WallTrack.d_wall` already uses `bumper_x = 0.443` from `base_link`
   (`MPC_corr.py:743-745`), i.e. it takes the rear-axle reading.

3. **Laser static transform: sim and real disagree, and the real one is a
   placeholder.** Real: `(0.12, 0, 0.20), yaw 0` (`description.launch.py:190-191`).
   Sim URDF: `(−0.12, 0, 0.15), yaw π` (`sensors.xacro:11`, `:42`). The real values
   are self-described as "approximate, ruler-measured placeholders, NOT a real
   calibration", with `roll=pitch=0` an unverified assumption. **The real static
   publisher wins on hardware** (the URDF's sensor links are gated off by
   `enable_sensors`, `sensors.xacro:27`).

4. **Steering limits: two pairs in the repo.** `±0.278/−0.283` (`stack_params.yaml`,
   used) vs `−0.264/+0.314` (`steering_offset_calibration_node.py` declared defaults,
   unused). The shipping pair wins for `mpc_corr`; the file documents at length why the
   other is wrong. `test_steering_limits.py` pins the shipping pair to
   `steering_calibration.yaml`.

5. **`psi_end` does not exist.** The Context section's name has no referent. The
   nearest thing is a **local variable** in a 500-line method, exported read-only as
   `corridor["psiRef"]`. There is no seam through which an external node can apply a
   proportional correction to it. This is an ambiguity in the *task*, not in the repo.

6. **Three unrelated signals are called "front clearance".**
   `/perception/front_clearance` (camera, min with obstacles),
   `/perception/front_distance` (camera, **wall only**, objects masked out — this is
   what the mission `front_clearance` stop_condition and the wall_turn commit decision
   actually read), and `/costmap/front_clearance` (SLAM-derived). A fourth,
   `/perception/swept_clearance`, is the swept-footprint one. Getting this wrong is a
   documented recurring error (`front_clearance_node.py:36-56`,
   `check_stop_condition.py:25-47`).

7. **`/perception/front_distance`: the prose disagrees about the publisher, but the
   launch file settles it.** `is_proximity_too_close.py:49-53` says
   `front_depth_monitor_node` "ITSELF is untouched and still runs".
   **That is stale.** `detection.launch.py:13-18` states plainly:
   "front_clearance_node REPLACES front_depth_monitor_node, which used to be launched
   here and is no longer launched at all. Both publish /perception/front_distance, so
   running the two together would put two publishers on one topic". The live publisher
   is **`front_clearance_node`**, unconditionally — no branch restores the other
   (`detection.launch.py:386-391`). `check_stop_condition.py:19-22`,
   `front_clearance_node.py:22` and `wall_turn.py:76-78` are the correct ones. Note
   `detection.launch.py` is modified on this branch, so re-check if it changes again.

8. **`corr_wmin`/`corr_wmax` are bare literals, not parameters.** `stack_params.yaml:1140`
   acknowledges this. A node deriving `d_ref` from corridor width would have to
   duplicate two numbers that have no single source of truth.

---

## Risks for the clearance node

1. **There is no seam for the `psi_end` correction.** `psiEnd` is a local inside
   `build_straight_corridor`, and the code states that `dpsi_this` is "the ONE source of
   truth for this corridor: it is psiEnd − psiStart (terminal heading cost), psiRefTurn
   (the solver's signed branch) AND the centreline's S-curve dpsi"
   (`MPC_corr.py:3272-3277`). A correction applied to `psiEnd` alone breaks that
   invariant three ways. **The new node can publish `d_perp` but cannot itself apply the
   correction** — the correction has to be an edit inside `MPC_corr`'s wall_turn branch,
   which is out of scope for a new node and must be planned as a change to `mpc_corr`.

2. **Any correction is silently clipped by the horizon bound, and may be erased by the
   ratchet.** `abs(dpsi_this) <= dpsi_by_horizon` is applied last and unconditionally
   (`wall_turn.py:206-207`), and the ratchet forbids the end heading retreating toward
   the move's start heading (`wall_turn.py:197-205`). A *proportional* correction whose
   sign points back toward the start heading — which is exactly what "the car
   over-rotated, bring `psi_end` back" means — **will be suppressed by the ratchet**,
   not applied. The design in the Context section is, in the post-turn direction, the
   thing the ratchet exists to prevent. This needs resolving before implementation.

3. **The corridor rebuilds at 1 Hz, not 10 Hz.** `corridor_update_period = 1.0`
   (`stack_params.yaml:622-624`), and the measurement recorded there says rebuilding at
   0.1 s measured *worse* on the heading return. So a correction fed into the corridor
   lands at most once per second, and the node's publish rate above ~1 Hz buys nothing
   for that half of the design (it still matters for the advisory clearance).

4. **`wall_tracker.py` has no test file.** `src/f1tenth_control/mpc_controller/test/`
   contains `test_wall_turn_increment.py` but **no `test_wall_tracker.py`** (directory
   listing; `git status` shows `wall_tracker.py` itself as untracked). Its only
   exercise is the offline `tools/wall_track_replay.py`. Building a second consumer on
   top of an untested 504-line geometry module is a real exposure.

5. **`swept_clearance_node` — the thing that already computes swept-footprint
   clearance — is not launched anywhere.** It has an entry point
   (`f1tenth_perception/setup.py:50`) and a launch file, but appears in **no**
   `components.yaml` entry and in **no** other launch file. Either the new node
   duplicates `swept_corridor.clearance()`, or the first step is registering the
   existing node — the latter is what "extend rather than add a parallel one" implies,
   and its geometry is already unit-tested (`test_swept_corridor.py`).

6. **Silence on `/mpc/wall_track` is overloaded.** It means *no wall_turn* **or**
   *`/mpc/hold` engaged* (`MPC_corr.py:2620-2624` returns before `:2641`) **or** *no
   odometry yet* (`:2615-2619`) **or** *`wall_track_enable` false* (`:1456`) **or** *node
   dead*. A consumer that treats silence as "turn finished" will fire on a
   safety hold mid-turn. Since a terminal wall_turn move ends with `/mpc/hold` and
   `drive_cmd` is never cleared by hold (`_clear_drive_state` is not called from
   `hold_callback`), "turn finished" and "turn held" are **indistinguishable on this
   topic**. If the node needs a true exit edge it must also subscribe to `/mpc/hold`
   and/or `/mpc/goal_drive`.

7. **`/mpc/goal_drive` is not latched.** `create_publisher(DriveCommand, topic, 10)`
   with default volatile durability (`publish_move_goal.py:108-109`), published exactly
   once per move entry (`DriveCommand.msg:1-2`). A node that starts, restarts or has a
   discovery hiccup between move entry and move end **never learns the turn was
   commanded**. `/mpc/wall_track`'s per-tick republication is the only recoverable
   signal, which is another reason to prefer it.

8. **Odometry under-reports distance ~17-20%, and the wall tracker inherits it.**
   Dead-reckoned stretches of `d_wall` carry the bias uncorrected, deliberately
   (`wall_tracker.py` FALLBACK section; `lidar_front_wall_node.py` records the figure).
   A perpendicular distance used for a *proportional* correction will therefore have a
   systematic gain error whenever the refit is degraded — and `provenance` on
   `/mpc/wall_track` is what tells you when.

9. **`d_wall` is a magnitude, not a signed lateral offset.** `WallTrack.d_wall` is
   `abs(c − n·p_bumper)` with direction carried separately in `normal_yaw`
   (`WallTrack.msg:23-24`). A proportional correction needs a **signed** error; the sign
   must be reconstructed from `normal_yaw` against the car's yaw, and the stack's
   convention (left positive, 3.6) must be applied deliberately rather than inherited.

10. **`d_ref` cannot be derived from corridor width.** The corridor is synthetic with
    hardcoded half-widths (2.6), and only one wall is ever sensed (2.7). `d_ref` must be
    a **fixed single-wall offset**, a parameter, not a derived quantity.

11. **Adding a `/scan` consumer to the `perception` component would take the e-stop's
    sensor down on every restart of the new node.** The constraint is explicit, has
    two enforcing tests, and the comment calls folding it in "the incident this comment
    exists to prevent" (`lidar_front_wall.launch.py:3-19`). The new node needs its own
    launch file, its own `components.yaml` entry, membership in `_ALWAYS_AUTO_START`
    (`component_supervisor_node.py:308-313` — an uncategorised component silently never
    starts), a `taskset` core that is **not** core 4, and its own copies of the two
    placement tests.

12. **Declare the `test` extra in any new package's `setup.py`.** Without
    `extras_require={'test': ['pytest']}` colcon reports a green result having executed
    nothing (`CLAUDE.md:5-45`). Not an issue if the node lands in `f1tenth_perception`,
    which already declares it.

---

## If the phase were not observable — options (recorded for completeness)

It **is** observable, so no decision is needed on this axis. The three fallbacks, had
it not been, in the order they would rank:

| Option | Cost | Lag |
|---|---|---|
| Subscribe `/mpc/goal_drive` and mirror the mode locally | Trivial; no change to `mpc_corr` | 0 at entry, but **misses the message entirely** on a late start or restart, and has no exit edge at all (nothing republishes) |
| Add a `mode`/`committed` field to an existing per-tick topic (e.g. `MpcSolverStatus`) | One `.msg` change + one publisher line in `mpc_corr`; rebuild of `f1tenth_messages` | 0; robust to late start |
| Infer from `/mpc/corridor_markers` shape change | No change to `mpc_corr`, but fragile geometric inference | up to **1.0 s** (`corridor_update_period`), and *after* the transition |

**The open decision is not observability — it is item 1 and item 2 under Risks:** the
`psi_end` correction has no external seam, and in the direction the design needs it the
ratchet is specifically built to suppress it. That needs a decision before any node is
written.

---

# Phase 0 findings — corridor seam and sign conventions

Read-only, 2026-09-14, branch `scene-graph`. Nothing launched, LiDAR not powered,
no source or config modified. Line numbers are as of this commit.

## 0.1 Where does the corridor correction belong?

### There is exactly one corridor build, and its branch is chosen by `drive_cmd`

`build_straight_corridor` (`MPC_corr.py:3191`) has **one** call site:
`control_loop`'s `if need_update:` block (`MPC_corr.py:2818-2828`), gated on
`corridor_update_period` (1.0 s). It is not called from anywhere else — no second
"post-turn" call exists. Which corridor *shape* comes out is decided inside, by
the dispatch at `MPC_corr.py:3216`: `drive_cmd is not None` selects the drive
branch, and within it `drive_cmd['mode'] == 'wall_turn'` (`:3239`) selects the
turn geometry against `:3361`'s `psiEnd = psi_base` for `"straight"`.

So "is a separate post-exit corridor built?" reduces to "what is `drive_cmd` after
the turn ends, and does `control_loop` reach line 2828 at all?"

### Exit path 1 — terminal move: no post-exit corridor is ever built

Every `wall_turn` mission in the repo ends on the turn:

| mission | moves | wall_turn `terminal` |
|---|---|---|
| `wall_turn.json` | 1 | `true` |
| `turn_90_left.json` | 1 | `true` |
| `drive_turn_180.json` | 1 | `true` |
| `drive_stop_2m_from_wall.json` | 2 (straight, then turn) | `true` |

On a terminal move `AdvanceMove` calls `state.complete()` and publishes
`/mpc/hold(True)` (`advance_move.py:87-88`). `control_loop` then returns at
`MPC_corr.py:2620` — **before** `_wall_track_tick()` (`:2641`) and before the
corridor rebuild (`:2828`). `drive_cmd` is never cleared: `_clear_drive_state`
(`:1912`) is called only from the four goal callbacks (`:1632`, `:1785`, `:1892`,
`:2014`) and *not* from `hold_callback`.

**Consequence: today there is no post-exit corridor to correct.** The turn is the
last thing the car does; the wheels stop. Option B has no referent in any shipping
mission.

### Exit path 2 — a following move: the corridor is genuinely separate

If a move followed the turn, `PublishMoveGoal` would publish the next goal, which
calls `_clear_drive_state()` (`:2014` for another `DriveCommand`) and
`_invalidate_move_state()` (`:2190`) — the latter clears `last_corridor_time`, so
the rebuild is forced on the **very next tick**, not up to a period later. The
resulting corridor is separate in every respect that matters:

- **Its own `psiEnd`.** `"straight"` takes `psiEnd = psi_base = psi_init_corridor`
  (`:3361`), re-anchored to `self.yaw` by `goal_drive_callback` (`:2045`) at the
  instant the new command arrives.
- **No `dpsi_this` inherited.** `turn_remaining` is reset to `None` at the top of
  every call (`:3206`) and assigned only inside the wall_turn branch, so the
  corridor dict carries `"psiRefTurn": None` (`:3677`) and the solver's terminal
  yaw cost falls back to its ordinary shortest-branch unwrap.
- **Outside the ratchet entirely.** The ratchet lives in `plan_wall_turn_step`
  (`wall_turn.py:197-205`), which the `"straight"` branch never calls. Its state
  is also per-move regardless: `_reset_turn_progress` (`MPC_corr.py:2149-2168`)
  zeroes `wall_turn_committed` and `wall_turn_commanded_rot` and calls
  `tracker.reset()` on every new drive command.
- **Outside the horizon clip.** `dpsi_by_horizon` is likewise only applied inside
  `plan_wall_turn_step` (`wall_turn.py:206-207`).

**So Option B's premise is structurally true but operationally absent.** Making it
do anything requires authoring a mission with a move after the turn — a JSON
change, not a controller change.

### The finding that reframes all three options: the MPC does not own the exit heading

The `wall_turn` move ends when **the behaviour tree** decides it has, on
`orientation_delta` (`condition_eval.py:362-379`):

```python
return abs(ctx.turn_accum_deg) >= abs(float(p['value']))
```

`turn_accum_deg` is the BT's own unwrapped accumulated yaw
(`check_stop_condition.py:187`, `:435`). Every wall_turn mission sets
`value: 90.0` (or `180.0`). The move therefore ends when the car has **physically
rotated 90 degrees**, whatever the corridor asked for.

Two consequences, and they are the crux of the decision:

1. **The exit heading is not a controllable quantity.** It is
   `heading_at_move_start + 90 deg`, fixed by the BT's own integrator. No edit
   inside `MPC_corr` can change it — an edit can only change the *path* the car
   takes to accumulate those 90 degrees, and therefore *where* it ends up.
2. **Option A's authority collapses to zero exactly where the correction is
   wanted.** `dpsi_this = copysign(min(|dpsi_rem|, dpsi_by_dist, dpsi_by_horizon),
   owed_sign)` (`wall_turn.py:187-189`) is bounded by `|dpsi_rem|`, which goes to
   zero as the turn completes. At the moment the BT is about to stop the move,
   `dpsi_rem ≈ 0` and so is any correction injected there. On top of that the
   ratchet (`wall_turn.py:197-205`) suppresses any correction whose sign points
   back toward the start heading, which is the direction "the car over-rotated"
   needs. Option A is squeezed from both sides.

The quantity that *is* controllable, and that `d_wall` actually measures, is the
car's **lateral position** at the moment it finishes rotating.

### Option C — is there a lateral seam?

**As a parameter or a topic: no.** `grep` over `mpc_controller/` for
`lat_off|lateral|w_lat|cross_track|centre_offset|center_offset` finds no offset
input. The one `lat_off` (`MPC_corr.py:3521`) is on the `goal_distance` branch and
is labelled "Diagnostic only: … Deliberately not acted on any more". The
centreline is anchored on the car by construction:

```python
xc = X0 + np.cumsum(np.cos(theta) * ds)      # MPC_corr.py:3594-3595
yc = Y0 + np.cumsum(np.sin(theta) * ds)
```

**As a two-line code edit: yes, and it is the cleanest of the three.** Displacing
that anchor along the corridor's left normal —
`X0 + off*(-sin psi0)`, `Y0 + off*(cos psi0)` — shifts the whole corridor
sideways without touching `dpsi_this`, `psiEnd`, `psiRefTurn` or the S-curve, so
it cannot violate the "ONE source of truth" invariant at `MPC_corr.py:3272-3277`,
and it is not subject to the ratchet or the horizon clip. Two live mechanisms
would then consume it:

- `w_corr`, effective **1.25** and genuinely wired as a stagewise squared
  lateral-offset cost since this pass (`MPC_corr.py:1055-1058`,
  `mpc_solver.py:974`, `stack_params.yaml:901`). It was `0.0` before.
- the per-stage corridor half-width soft cost (`mpc_solver.py:602`). Note
  `corr_wmin = 0.4333` (`MPC_corr.py:615`) is the half-width **at the car**, so a
  usable offset must stay well inside that or the car starts outside its own
  corridor at `u=0`.

**But this is a deliberate behaviour reversal, not a gap.** `MPC_corr.py:3400-3425`
records that the corridor origin was moved *off* the frozen line and *onto* the
car on purpose:

> "deliberate behaviour change, confirmed with Andreas — NOT a regression to be
> cautiously reverted … The previous pass put the origin at the perpendicular FOOT
> of the live pose on the frozen line, which made the corridor home laterally back
> onto the exact original line after any deflection. That worked, but it fed two
> moving inputs into one geometry … and the result was a visibly jittery corridor."
>
> "THE PRICE, stated plainly so nobody has to rediscover it: there is NO lateral
> homing any more … Lateral offset from the ORIGINAL line is once again zero by
> construction here, so the corridor half-width bound and w_corr have nothing to
> act on."

So Option C re-introduces lateral homing, and `w_corr = 1.25` has never acted on a
non-zero offset on this geometry. The jitter failure mode is on record. The
mitigations the design already specifies — deadband from `fit_rms`, rate limit,
saturation — are aimed at exactly that, and the input is a LiDAR-tracked wall
rather than a SLAM-stepped frozen line, but this needs Andreas's sign-off as a
reversal, not just a technical go-ahead.

**`/costmap/boundaries` is not a seam.** It is documented single-source
(`MPC_corr.py:411`, `:2302-2309`) and `costmap_boundaries_callback` replaces
`self.costmap_boundaries_world` wholesale (`:2321`), so a second publisher would
race and clobber `costmap_boundary_node`. A half-plane is also one-sided: it can
say "no closer than `d_ref`" but cannot pull a car that is too far back in. Useful
for the Phase 2 *gate*, useless as a centring law.

### 0.1 verdict

| | viable? | touches | authority at turn exit |
|---|---|---|---|
| **A** — inject at `dpsi_this` | mechanically yes | `wall_turn.py` + `MPC_corr.py` wall_turn branch | **~zero** — bounded by `dpsi_rem → 0`, and the ratchet suppresses the needed sign |
| **B** — correct the post-exit corridor | structurally yes, **operationally absent** | `MPC_corr.py:3361` straight branch + a mission JSON | full, but only *after* the turn, and only once a mission has a following move |
| **C** — lateral centreline offset | yes, 2-line edit, no invariant broken | `MPC_corr.py:3594-3595` | full, and available *during* the turn as well as after |

## 0.2 Sign conventions

### Footprint and `base_link`

| quantity | value | source |
|---|---|---|
| `base_link` position | ground plane, **at the rear axle**, laterally centred | `base.xacro:5-9`, and the joints at `:26-33` |
| footprint rectangle | `x ∈ [-0.082, +0.443]`, `|y| ≤ 0.136` ⇒ **0.525 m × 0.272 m** | `swept_corridor.py:116-118`; `stack_params.yaml:510-517` |
| rear axle within it | `x = 0.0` | `stack_params.yaml:528-530` |
| URDF box | **none declared** — chassis is an STL with a scaled/offset visual origin | `base.xacro:38-40` |

**The two disagree and the repo knows it.** `description.launch.py:35-36`,
`sensors.xacro:10-13` and `camera.launch.py` all say `base_link` is "centered
between the axles, 0.07 m above the ground". The URDF joints say rear axle, on the
ground. `swept_clearance_rear_axle_x_m` is set to `0.0` on that basis and states
the alternative would be `-0.1525` ("change only on a measurement"). **This is an
unresolved 0.1525 m ambiguity in the longitudinal origin** and it propagates into
every bumper-referenced distance, including `WallTrack.d_wall`, which takes
`bumper_x = 0.443` from `base_link` (`MPC_corr.py:743-745`) i.e. the rear-axle
reading. It does **not** affect a *lateral* distance, which is why `d_wall` as a
signed perpendicular offset is the safer quantity of the two.

### Steering and radii

| quantity | value | source |
|---|---|---|
| wheelbase (runtime) | **0.305 m** | `stack_params.yaml:841`, `MPC_corr.py:575-576` |
| wheelbase (URDF, TF only) | 0.325 m | `robot.urdf.xacro:39` |
| `delta_max` | **+0.278 rad** (+15.93°) | `stack_params.yaml:844-849` |
| `delta_min` | **−0.283 rad** (−16.22°) | `stack_params.yaml:844-849` |
| `R_min` left | **1.069 m** | `wall_turn.py:39-42`, `108-110` |
| `R_min` right | **1.049 m** | same |
| track | 0.2 m | `robot.urdf.xacro:40` |

Bounds are **asymmetric** and are derived by inverting the servo clip through
`steering_calibration.yaml`, not measured. `min_turn_radius()` picks the bound by
the sign of what is still owed, so a correction reversing direction changes `R_min`.

### Yaw and `psi`

**Standard ROS right-hand rule: positive = left = CCW.** Unanimous across
`DriveCommand.msg:23-25`, `TurnGoal.msg:6`, `mission_config.py:136`, `:169`,
`MPC_corr.py:915`, `swept_corridor.py:39`, `steering_calibration_node.py:372`.
Positive commanded steering angle = left, same sense as `psi`.

### Every lateral quantity in the stack

**Left is positive in every quantity that carries a sign.** The traps are the
quantities that are *unsigned*, and the two `Line` classes.

| quantity | signed? | convention | source |
|---|---|---|---|
| corridor normal `n = (−sin ψ, cos ψ)`; left wall `C + w·n` | signed | **+ = left** | `MPC_corr.py:3604-3608` |
| `corridor_lateral_coordinates` → `d_lat = n·(p − pc)`, `n = (−ty, tx)` | signed | **+ = car is left of centreline** | `mpc_solver.py:1408-1420` |
| `lat_off` (diagnostic only) | signed | **+ = left** | `MPC_corr.py:3521-3522` |
| `swept_corridor` frame, `kappa = tan δ / L` | signed | **y left, + kappa = left** | `swept_corridor.py:16-17`, `:39` |
| `WallLineFit.normal_angle` | signed | **0 ahead, + = left**, car → wall | `WallLineFit.msg:26`, `lidar_front_wall.py:16-18` |
| `WallEstimate.normal_angle` | signed | same | `WallEstimate.msg:23-24` |
| `WallDetection.bearing` | signed | **+x forward, + = left** | `WallDetection.msg:4` |
| `WallDetection.normal` | direction | oriented **back toward the robot** | `WallDetection.msg:5` |
| `BoundaryConstraint.normal` | direction | **toward the wall** — the *opposite* of `WallDetection.normal`; free side is where `n·p` is small | `BoundaryConstraint.msg:13-22` |
| `BoundaryConstraint.offset` | — | raw geometry, **not** pre-shrunk by any margin | `BoundaryConstraint.msg:24-29` |
| `WallLineFit.distance` | **unsigned** | `≥ 0`, base_link origin ⊥ to line | `WallLineFit.msg:25`, `lidar_front_wall.py:14-15` |
| `WallTrack.d_wall` | **unsigned** | `abs(...)`; direction carried separately in `normal_yaw` | `WallTrack.msg:23-24`, `wall_tracker.py:362-369` |

#### The trap: two `Line` classes with opposite signed-distance conventions

```python
# mpc_controller/wall_tracker.py:201-203   -- "positive before it"
def offset(self, px, py):  return self.c - (self.nx * px + self.ny * py)

# f1tenth_perception/glass_detect.py:443-444
def signed(self, px, py):  return self.nx * px + self.ny * py - self.c
```

They are **exact negations of each other**. Phase 1 reads `glass_detect.Line`
(via `detect()` / `GlassTracker`) while the existing `d_wall` comes from
`wall_tracker.Line`. Mixing them silently flips the sign of the error, which is
precisely the steer-into-the-pane bug. Note also that `wall_tracker._gate`
(`wall_tracker.py:377-379`) **re-orients `n` from the car toward the line** before
anything reads it; `glass_detect` does no such normalisation — `tls_line` returns
whatever sign the SVD produced.

### Frames

| | |
|---|---|
| laser frame name | **`laser`** (`sensors.yaml:9` `laser_frame_id: "laser"`) |
| `base_link → laser`, real hardware | `x=0.12, y=0.0, z=0.20`, `yaw=pitch=roll=0` (`description.launch.py:186-193`; positional args are **x y z YAW PITCH ROLL**, verified empirically) |
| status of those numbers | "approximate, ruler-measured placeholders, **NOT a real calibration**"; `roll=pitch=0` is an unverified assumption (`description.launch.py:42-58`) |
| sim URDF (disagrees, loses on hardware) | `(−0.12, 0, 0.15)`, `yaw = π` — pre-remount, rear-facing (`sensors.xacro:11`, `:42`) |
| tree | `base_link → base_footprint` (identity) → `chassis`/wheels; sensor edges from `static_transform_publisher` on hardware, from `sensors.xacro` in sim |
| `odom → base_link` | owned by the **local `robot_localization` EKF** (`publish_tf: true`); `vesc_to_odom_node`'s own `publish_tf` is false |
| `mpc_corr`'s pose source | `/odometry/filtered` (local EKF, 50 Hz), since `localization_source: ekf` |

### Recommended `d_wall` sign convention for the new message

Reuse the stack's convention rather than inventing one: **`d_wall` is the signed
perpendicular offset of the tracked wall from the car, left positive** — i.e. a
wall on the car's left gives `d_wall > 0`, a wall on the right gives `d_wall < 0`,
and `|d_wall|` is the perpendicular distance. That matches the corridor normal
(`MPC_corr.py:3604`), `d_lat` (`mpc_solver.py:1417`), `swept_corridor`'s frame and
every `+ = left` bearing field. It deliberately **differs from
`WallTrack.d_wall`**, which is an unsigned magnitude — that difference must be
stated in the `.msg` because both topics will be live at once.

With that convention and left-positive `Δψ`, the control law's sign is:
`e = d_wall − d_ref` where `d_ref` carries the sign of the side the wall is on,
and `Δψ = −k·e` steers **away** from the wall when the car is too close
(`|d_wall| < |d_ref|`). Both signs must be pinned by test 1 on both sides, and
Stage 2 of the powered session is the only thing that can confirm the convention
itself is not backwards.

---

# Bag inventory (AMENDMENT: LiDAR down for the duration)

Searched `~/f1tenth_archive/{active,complete,mission_row}/*/bag/`,
`~/.ros/mission_bags/`, `dev_ws/f1tenth_more/bags/`, plus a `find ~` for every
`*.db3`/`*.mcap`. ~120 archived runs; **14** are `wall_turn` or `drive_turn_180`.

## `/mpc/wall_track` is in ZERO bags

Every bag predates the wall tracker (`wall_tracker.py` is still untracked in
`git status`). `/perception/front_distance` and `/mpc/goal_drive` are **also**
absent from every bag, which is what `tools/wall_track_replay.py:6-18` already
records and works around.

**So no bag can validate the phase machine directly.** What a bag *can* do is
drive the phase machine off a **regenerated** `/mpc/wall_track`: run
`wall_tracker` + `plan_wall_turn_step` over the bag's real `/scan` + `/odometry/filtered`
at the real tick timing, exactly as `tools/wall_track_replay.py` already does, and
feed the synthesised per-tick messages in. That reproduces real scan jitter, real
odometry, real 10 Hz cadence and the real turn geometry — but **not** the real
silence pattern at turn exit, real dropped messages, or real `/mpc/hold` timing,
because those depend on topics the bag does not carry. Treat a replay pass as
"not falsified against real geometry", not as validation of the silence logic.

## Best assets, ranked

| run | tier | dur | `/scan` | `/odometry/filtered` | `/tf` | `/drive` | `/mpc/corridor_markers` |
|---|---|---|---|---|---|---|---|
| `2026-09-11T13-49-54_mission-wall_turn` | complete | **55.9 s** | 2228 | 2684 | 5807 | 556 | 53 |
| `2026-09-10T15-35-24_mission-straight_then_wall_turn_3m` | complete | 20.7 s | 824 | 1010 | 2242 | 204 | 19 |
| `2026-09-10T15-33-46_mission-wall_turn` | complete | 18.1 s | 717 | 865 | 1912 | 179 | 16 |
| `2026-09-10T15-56-51_mission-wall_turn` | complete | 17.8 s | 707 | 835 | 1738 | 177 | 17 |
| `2026-09-10T13-39-04_mission-wall_turn` | complete | 7.8 s | 310 | 379 | 838 | 77 | 7 |
| `2026-09-10T13-34-09_mission-wall_turn` | complete | 7.9 s | 281 | 339 | 701 | 80 | 7 |
| `2026-09-10T13-07-04_mission-drive_turn_180` | complete | 5.4 s | 200 | 254 | 542 | 51 | 5 |

Every one of them also carries `/tf_static` (5 msgs — so the real
`base_link → laser` transform is recoverable per-bag rather than assumed),
`/mission/status`, `/behavior/tree_status`, `/mpc/hold`, `/mpc/solver_status`,
`/costmap/boundaries`, `/slam/pose` and `/ekf_global/odometry/filtered`.
Two of the 14 are zero-length (aborted before recording), and
`2026-09-11T12-48-03` has 5 scans and no odometry at all.

`2026-09-10T15-35-24_mission-straight_then_wall_turn_3m` is worth noting: the
mission JSON for it **is not in the repo** (no
`straight_then_wall_turn_3m.json` under `src/f1tenth_behavior/missions/`), so it
was authored, run, and later removed or renamed. Its archived
`*.params.yaml` and `manifest.json` are the record of what it actually ran.

Each run directory also has `<run>.extract.parquet` (~1 MB), `<run>.params.yaml`
(~107 kB, the launched parameter set) and `snapshots/*.jpg`.

## `capture a wall_turn bag with /mpc/wall_track` → powered-session list

A bag recorded *after* `wall_track_enable` went live, containing `/mpc/wall_track`
alongside `/scan`, `/odometry/filtered`, `/tf` and `/mpc/hold`, is the one asset
that would make the phase machine's silence logic offline-testable. It does not
exist. Stage 4 of `docs/bringup_checklist.md` is where it gets captured.

---

# Contradictions against the `d_wall` implementation prompt

Four, all verified by `find`/`grep`, all affecting scope rather than design.

### 1. `glass_detect.py` has no ROS node, so there is no "segment output" to subscribe to

The prompt's Phase 1 says tracking is "fed by `glass_detect.py`'s segment output".
`glass_detect.py:4-5` itself says "glass_detector_node.py is the ROS glue".

**`glass_detector_node.py` does not exist.** No such file under
`src/f1tenth_perception/f1tenth_perception/`, no `console_scripts` entry
(`setup.py:43-51` lists seven nodes, none of them glass), no launch file, no
`components.yaml` entry. `glass_detect.py` is 955 lines of pure logic with 37
passing unit tests and **no publisher anywhere**.

Consequence: `wall_distance_node` must call `detect()` / `GlassTracker.update()`
**in-process** off its own `/scan` subscription. It becomes `glass_detect`'s first
and only runtime consumer. That is a larger surface than "subscribe to a topic",
and it means the node owns the glass detector's CPU cost as well as its own.

### 2. There is no `glass_*` parameter block

`DetectorConfig`'s docstring (`glass_detect.py:539`) says "Every gate, defaulted
to `stack_params.yaml`'s `glass_*` values". `grep -c '^glass_'
src/f1tenth_params/config/stack_params.yaml` returns **0**. Every gate is
currently a Python default in the dataclass. Standing rule 4 ("no magic numbers,
everything tunable goes in `stack_params.yaml`") therefore pulls ~25 detector
gates into scope alongside the ~15 `wall_distance_node` keys the prompt lists.

### 3. `GlassTracker` already implements most of Phase 1's tracking — but has no `track_id`

`glass_detect.py:826-941` already has: odom-frame tracks, re-association by
`match_endpoint_tol` **and** `match_angle_tol_deg` (the prompt's two parameter
names, already spelled that way), and — critically — the exact
"carry the track through when no beams land on it" rule the prompt asks for,
implemented as `_expected_return()` and documented "NEVER DECAY ON ABSENCE ALONE
… which is exactly the clearing bug this whole design exists to prevent,
reintroduced one layer up".

What it lacks is a **stable published identity**: tracks are distinguished by
Python `id(track)` (`:920`, `:930`). Phase 1's `track_id` is an extension to
`Track`, not a new tracker. It also has no coasting cap — `drop_stale()` defaults
to "never forget", the opposite of `max_coast_distance`/`max_coast_yaw`, so the
cap is genuinely new behaviour and must not be confused with `drop_stale`.

### 4. `swept_corridor.clearance()` is points-only; the segment API does not exist

`clearance(points_xy, delta, *, wheelbase, half_width, margin, max_range,
absolute_min_clearance, front_x, rear_x)` (`swept_corridor.py:204`) takes an
`(N,2)` point array. There is no segment overload.

But `sample_segment(p0, p1, spacing)` **already exists**
(`glass_detect.py:943-957`) and `DetectorConfig.point_spacing = 0.025` is already
the parameter name the prompt uses. So the prompt's first option ("sample the
fitted line into synthetic points at `point_spacing`") is a one-line call to
existing, tested code, while the second ("extend the API to accept a segment")
means new geometry inside the one module the stack's swept-clearance correctness
rests on. **Sampling is the smaller and safer change** — recommended, subject to
the 0.025 m spacing being fine enough for a 5 m arc (a 5 m segment sampled at
0.025 m is 201 points, which is cheap next to the ~1081-beam scan the same
function already takes).
