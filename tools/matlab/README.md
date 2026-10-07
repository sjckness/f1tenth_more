# f1db: the campaign runs in MATLAB

`+f1db` reads the database that `export_matlab` writes. MATLAB only reads it:
the `runs/*.mat` files are written by the exporter and by nothing else.
`f1db.build` writes `index/index.mat` and nothing more.

```matlab
addpath('~/dev_ws/f1tenth_more/tools/matlab')   % the folder that holds +f1db
db = f1db.open();                                % ~/matlab_data (or $F1TENTH_MATLAB_DATA)
```

## Filling the database

```bash
ros2 run f1tenth_logger export_matlab                  # every campaign run, incremental
ros2 run f1tenth_logger export_matlab --mission M04 --runs "14-15-16"
ros2 run f1tenth_logger export_matlab --force          # re-export everything
```

The database root is resolved in this order, and printed at start:

1. `--db-root` (alias `--out`)
2. `$F1TENTH_MATLAB_DATA`
3. `matlab_export.db_root` in `src/f1tenth_logger/config/test_campaign_logger.yaml`
4. `~/matlab_data`

The exporter needs `rosbags` (`pip install --user rosbags`, already installed on the
Jetson), numpy and scipy, but not rclpy. A run whose `.mat` was written by the same
exporter version is skipped. `--runs` takes `"1-3-4-67-89"`
(R001 R003 R004 R067 R089), short IDs (`P004-R016`) or full test IDs, mixed.

## Layout

```
~/matlab_data/
  runs/<test_id>.mat     one per campaign run, MATLAB v5, compressed
  index/runs.csv         one row per run (written by export_matlab)
  index/cmds.csv         the test sheet, exported by hand (see below)
  index/index.mat        runs + cmds tables (written by f1db.build)
```

### Inside a run file

Each topic is a struct with the same layout:

| field | meaning |
|---|---|
| `source` | `"campaign"` (the test folder) or `"bag"` (the archive bag) |
| `time_source` | `"header"`, `"receive"`, or `"header+receive"` (mixed) |
| `topic` | the file or ROS topic it came from |
| `t_ros` | seconds, ROS time |
| `t_rel` | seconds from the drive start (`mission_started`; else the first command) |
| one column per field | n x 1 numbers, n x k matrices, or n x 1 cells for variable-length data |
| `info` | per-topic constants (bag topics: frame_id, scan angles, ...) |

The campaign topics are always present and cover the whole test, including
2 s before the mission starts and 2 s after it ends:

| struct | columns |
|---|---|
| `kin` | `x y yaw yaw_rate vx vy speed ax ay acc corridor_clearance obstacle_clearance`, from /odometry/filtered |
| `imu` | `ax ay az gx gy gz imu_yaw sensor_stamp` |
| `cmd` | `speed steer yaw_rate throttle brake cmd_source` |
| `mpc` | `status solve_time_ms cost iterations` |
| `horizon` | `i corridor_id ts n`, plus cells `x y yaw v steer accel` (20-point MPC horizon per solve) |
| `corridor` | `id corridor_source`, cell `polygon` (n x 2), `meta_*` (definition, object fields, ref steps) |
| `corridor_dbg` | `robot_x/y/yaw/v`, cells `corridor_xc yc xL yL xR yR Pend`, `target_x/y`, `obstacles_world` |
| `tracks` | `stamp frame_id n`, cell `tracks` (struct of columns per message) |
| `events` | `event`, `fields_json` |
| `llm` | `prompt response llm_raw latency_ms ttft_ms ok ...`, `translated_plan_json` |
| `plan` | `tag file plan_id plan_hash text` (the plan JSON) |

The bag topics cover only the part of the drive the bag recorded. See
`meta.bag_coverage_pct`, `meta.bag_t_start_rel`, `meta.bag_t_end_rel` and the
`has_bag` index column.

| struct | columns |
|---|---|
| `odom`, `ekf_local`, `ekf_global` | `x y yaw vx vy wz` |
| `slam_pose`, `slam_pose_cal` | `x y yaw cov_xx cov_yy cov_yawyaw` |
| `drive_mpc` (/drive), `drive_out` (/ackermann_drive), `safety_stop` | `speed steering_angle steering_angle_velocity acceleration jerk` |
| `scan` | `ranges` (n x beams, single), `intensities`; angles in `info` |
| `solver`, `clamp`, `bt`, `mission_status`, `move_outcome`, `goal_object`, `object_status`, `front_clear`, `hold`, ... | every message field, flattened |
| `boundaries`, `obstacles2d`, `det2d`, `det3d`, `sem_tracks` | cell per message, each a struct of columns |
| `map` | the last `/slam/map` of the bag: `grid` (int8, height x width, row 1 = lowest y), `resolution origin_x origin_y origin_yaw` |

`meta` holds `run_id`, `mission`, the bag match (`bag_status`,
`archive_run_id`, `bag_match_dt_s`), the topic list with types, counts and
sources (`meta.topics`), `topic_errors`, `git_commit`, `exporter_version`, and
the test's own `meta.json` (`campaign_meta_json`).

**Frames:** `kin`, `ekf_local`, `horizon` and `corridor` use the odom frame.
`map`, `slam_pose` and `ekf_global` use the map frame. To plot over the map,
use `ekf_global` or `slam_pose` (bag stretch only), not `kin`.

### index/runs.csv

These are the `campaign_results.csv` columns, recomputed by the campaign exporter:
plain CSV, `.` decimals, an empty cell is NaN, never 0. The manual columns
(`success`, `transl_ok`, `notes`) are copied from the campaign's
`campaign_results.csv`. The exporter also adds `has_bag`, `bag_coverage_pct`,
`bag_t_start_rel`, `bag_t_end_rel`, `archive_run_id`, `bag_status`
(`attached`, `empty_bag`, `no_match`, `ambiguous`, `bag_unreadable`),
`bag_match_dt_s` and `mat_file`.

### index/cmds.csv (by hand)

Export the **"first test campaing"** tab of the *F1tenth testing* sheet
(File > Download > CSV) and save it as `<db_root>/index/cmds.csv`, UTF-8,
comma-separated, with this header row:

```
Cmd #,Command text,file,obstacle,LLM OK,Translator OK,Rep,Success
```

- `Cmd #`, `Command text` and `obstacle` may be written once per block.
  `f1db.open` fills `Cmd #` down, and `Command text` and `obstacle` down within
  each `Cmd #` block.
- `file` names the run, as a full test ID or the `P003-R009` short form.
  That is the join key onto `db.runs`.
- The columns come out as `cmd_no command_text file obstacle llm_ok translator_ok
  rep success_sheet`. `Success` is renamed so it does not clash with the index's
  manual `success` column.

Run `f1db.build` after replacing the file.

## Examples

Single run: speed against command.

```matlab
db = f1db.open();
r = f1db.run(db, "P004-R016", {'kin', 'cmd'});
plot(r.kin.t_rel, r.kin.speed); hold on
stairs(r.cmd.t_rel, r.cmd.speed); xline(seconds(0), '--', 'drive start')
legend('speed', 'command'); xlabel('t from drive start')
```

Group filter and statistics:

```matlab
g = db.runs(db.runs.mission == "M04_person" & db.runs.date == "2026-10-01", :);
groupsummary(db.runs, "mission", ["median" "max"], ["min_clear_m" "feas_pct" "viol_rate_pct"])
groupsummary(db.runs(db.runs.has_bag == 1, :), "mission", "mean", "bag_coverage_pct")
```

Overlay of the trajectories of a group:

```matlab
T = f1db.runs(db, g, {'kin'});
figure; hold on; axis equal; grid on
for i = 1:height(T)
    k = T.kin{i};
    plot(k.x, k.y, 'DisplayName', T.run_id(i));
end
legend('Interpreter', 'none'); xlabel('x [m]'); ylabel('y [m]')
```

Over the map (map frame, bag stretch only):

```matlab
r = f1db.run(db, "P004-R016", {'map', 'ekf_global'});
m = r.map;
x = m.origin_x + (0:m.width - 1) * m.resolution;
y = m.origin_y + (0:m.height - 1) * m.resolution;
imagesc(x, y, double(m.grid)); set(gca, 'YDir', 'normal'); colormap(flipud(gray)); hold on
plot(r.ekf_global.x, r.ekf_global.y, 'r', 'LineWidth', 1.5); axis equal
```

LaserScan at one instant:

```matlab
r = f1db.run(db, "P002-R008", {'scan'});
i = find(r.scan.t_rel >= seconds(20), 1);
a = r.scan.Properties.UserData.info;
th = a.angle_min + (0:a.n_beams - 1) * a.angle_increment;
polarplot(th, r.scan.ranges(i, :), '.')
```
