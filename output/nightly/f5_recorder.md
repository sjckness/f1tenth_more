# F5 — the second recorder: who, what, what it costs, and one proposal

**Report and proposal only. Nothing in this file is applied** (see the verdict
on the bringup question below, which is what the point made conditional).

## Who launched it

**Nobody in the repo. It was typed by hand.** The evidence:

- No launch file, component or script in the workspace runs `ros2 bag record`
  against the live stack. The only three occurrences are
  `tools/record_raw.sh` (8 topics into `~/raw_data/<date>_<clip>/bag`, which is
  neither this bag's name nor its topic set),
  `scripts/replay_bag_through_mpc.sh` (offline) and two documented MANUAL
  operator commands (`docs/bringup_checklist.md:30`,
  `docs/procedures/2026-09-17_floor_person_pass.md:86`), both of which write to
  `~/f1tenth_archive/manual/`.
- `components.yaml` has no recorder entry, and the supervisor's own log
  directory holds no recorder component -- only `mission_logger.launch.py`.
- The bag itself says it was `ros2 bag record -a`: 148 topics, **21 of them with
  zero messages**, 16 `/viz/*` relays that `mission_logger` is specifically
  coded to drop (`_NEVER_RECORDED_PREFIXES`), and `/events/write_split`,
  `/parameter_events` and `/rosout` -- `/events/write_split` is a topic the
  RECORDER ITSELF publishes, so only `-a` subscribes to it.
- `~/.bash_history` has no `bag record` line at all, so the shell it was typed
  in did not flush its history (a VS Code terminal, or a session killed).

**So the bringup question is settled: it is NOT part of the bringup, and there
is nothing to toggle to false.** F6 already made the recorder that IS part of
the bringup opt-in.

## What each one recorded, and what it cost

| | manual `ros2 bag record -a` | `mission_logger` (archive run) |
|---|---|---|
| file | 548.4 MB | 71.3 MB |
| span | 98.7 s | 13.9 s |
| **payload rate** | **5.51 MB/s** | **5.09 MB/s** |
| messages | 46 973 | 5 368 |
| topics | 148 (21 empty) | 55 |
| storage | sqlite3, no compression | sqlite3, no compression (mcap requested, plugin absent) |
| write cache | 100 MiB (`ros2 bag record` default) | 100 MiB (explicit) |
| QoS overrides | none | best-effort for 5 topics, deep RELIABLE for `/tf` |
| `/viz/*` | 16 topics, 18.3 MB (3 %) | excluded by code |
| Image/grid/cloud | 507.9 MB (**93 %**) | 62.2 MB (**88 %**) |

They ran **at the same time**, so the box was writing **~10.6 MB/s** while all
12 cores sat at 95-97 %.

Top topics by bandwidth:

| topic | manual bag | mission bag |
|---|---|---|
| `/zed2/zed_node/depth/depth_registered_5hz` | 1.45 MB/s | **3.32 MB/s (65 %)** |
| `/perception/scene_labels/image` (overlay) | 0.97 MB/s | not recorded |
| `/costmap/visualization` | 0.53 MB/s | not recorded |
| `/perception/wall_points` | 0.39 MB/s | 0.64 MB/s |
| `/camera/detection_masks` | 0.33 MB/s | 0.27 MB/s |
| `/costmap/wall_cell_ids` | 0.32 MB/s | — |
| `/zed2/zed_node/depth/depth_registered` (full rate!) | 0.29 MB/s | not recorded |
| `/costmap/wall_cells` | 0.08 MB/s | 0.22 MB/s |
| `/scan` | 0.20 MB/s | 0.32 MB/s |

CPU cost cannot be attributed retrospectively -- `/diagnostics/system_status`
reports the whole box (95-97 % on 12 cores), not per process. What IS measured
is the consequence: the manual bag lost **24 % of `/scan`** (30.4 Hz recorded
against the 39.95 Hz urg_node reports) and **48 % of `/odometry/filtered`**
(26 Hz against the EKF's own-reported 50.000 Hz), while the mission bag over
the same seconds holds `/scan` **complete at 40.0 Hz**. Two recorders on a
saturated box cost the weaker one a quarter of the LiDAR.

## Proposal: one recorder, and what it should be configured as

Not applied. Five changes, in order of what they buy:

1. **Install the mcap plugin and let the existing default take effect.**
   `sudo apt install ros-humble-rosbag2-storage-mcap`. `mission_bag_storage_id`
   already defaults to `mcap` and `_resolve_storage_id()` already falls back to
   sqlite3 with a loud warning, so this is an apt command and no code change.
   sqlite3 is also what Foxglove reads worst.
2. **zstd, fast, per file.** `record_options.compression_mode = 'file'` and
   `compression_format = 'zstd'` in `mission_logger_node._start_recording`
   (it sets neither today). File mode compresses on close, off the hot path, so
   the recording itself does not pay for it. On this payload -- float32 depth
   and OccupancyGrids -- 3-5x is the usual range, i.e. ~5 MB/s down to ~1-1.5 MB/s
   at rest. Message mode would compress inline and is the wrong trade on a box
   this loaded.
3. **Raw images out of the default list.** The depth throttle is 65 % of the
   mission bag on its own. `mission_logger_record_depth` already exists and
   already turns the throttle and all three depth topics off together -- flip
   its default to **false**, like `enable_mission_logger`, and pass
   `mission_logger_record_depth:=true` on the runs that are specifically
   validating the stereo half. Same for `/perception/wall_points` (0.64 MB/s,
   inspection only, nothing in the control path reads it) and
   `/camera/detection_masks`.
4. **`/viz/*` never, which is already true** for `mission_logger`
   (`_NEVER_RECORDED_PREFIXES`, enforced by `test_viz_topics_not_recorded.py`).
   It is only the manual `-a` recorder that picks them up, which is reason
   enough not to use `-a`.
5. **Cache 512 MiB**, from 100. `max_cache_size=512*1024*1024`. The measured
   failure is a recorder that falls behind for hundreds of milliseconds
   (`/scan` latency median 288-521 ms, p95 to 1.3 s) against subscription
   queues of depth 10, and a bigger write cache is the one lever that absorbs
   a burst without touching QoS. Costs 0.5 GB of RAM while recording.

And the operational rule, which costs nothing: **never run a second
`ros2 bag record` beside the mission logger.** For the same seconds the
supervised recorder's bag was complete and the manual one had lost a quarter of
the scans. If a raw clip is genuinely needed, `tools/record_raw.sh` exists and
records 8 topics, not 148.
