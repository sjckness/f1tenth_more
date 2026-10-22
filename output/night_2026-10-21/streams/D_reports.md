## Stream D: run reports with segmentation masks and LiDAR projection
Branch `night-reports`, worktree `~/dev_ws/f1tenth_more_reports`.

Report images and the per-run video now show the camera frame with light fills for wall, floor, inferred floor and objects, plus the LiDAR points projected on it. Everything is drawn after the run, from the bag. The only addition on the car is one small JPEG at 5 Hz, published only while the logger records.

No run on linus had the full default set of data, so that path still needs one check on the car.

Two independent reviews followed the first version. All of their findings are fixed, except one refactor I declined (see Decisions).

**Commits** (nothing pushed):
- `0887eeb` scene_mask: the shared renderer
- `838f4ca` yolo_detector_node: the clean frame
- `e9d18c2` mission_overlay: stills, video, logger, params
- `1cc0a8a` review fixes: stamp search, still order, stack_params fallback
- `fb14a79` review fixes: mcap warning, descriptions
- `2ed5216` review fixes: zero-stamped messages

### Recon
- **Where the Run view gets its reports.** `apps/pwa/src/screens/Run.tsx` calls `/api/runs`. On pwa-control, `f1tenth_gateway/runs.py` builds the page itself:
  - stills: `<run_dir>/snapshots/*.jpg`, sorted by name;
  - video: `<run_dir>/*.mp4`, else `<ws>/mission_videos/<run_id>*.mp4`.
- **What generates them.** The stills come from `mission_logger_node._write_snapshots` at finalize (`mission_logger_node.py:1156`). The video is made by hand with `mission_debug_video`. These are the two things I changed.
- **Image topics recorded today** (default profile):
  - `/camera/debug_image/compressed`: YOLO's 5 Hz debug JPEG, already drawn on;
  - `/camera/detection_masks`;
  - `/perception/scene_labels`;
  - the 5 Hz depth and both camera_infos.

  The campaign profile adds `/perception/scene_labels/classes` and `/camera/image_raw_5hz`. **The default profile had no camera frame without drawings on it.**
- **Masks and LiDAR projection.** The masks are published only when something subscribes, and recorded only in the campaign profile. The LiDAR-to-image projection existed only inside `front_clearance_node`'s live `/perception/scene_labels/image`, never recorded.

### Option taken: offline, with the mask rebuilt from depth when it was not recorded
- **Renderer reused, no second implementation.** `scene_mask.overlay()` (`scene_mask.py:285`) got keyword options; its defaults draw exactly what the live node drew before.
  - New helpers there: `draw_legend` (`:376`) and `scan_to_base` (`:237`, moved out of `front_clearance_node`).
  - Stream A edits neither file.
- **New `f1tenth_logger/mission_overlay.py`**, needs no ROS (reads sqlite3 and decodes CDR by hand):
  - camera: `image_raw_5hz`, else the new clean JPEG, else `image_annotated`;
  - mask: the recorded classes, else `RebuiltMask` (`:555`), built from depth, the detection masks and the run's own `params.yaml` (keys it lacks come from today's stack_params);
  - pairing by header stamp (`Topic.nearest`, `:430`);
  - stills: `pick_stills` (`:848`) and `write_snapshots` (`:872`); there is also a `mission_overlay` command.
- **`mission_logger_node`** records the clean topic and writes the overlay stills at finalize; any failure falls back to the old extraction.
- **`mission_debug_video`** uses the overlay when the run has it. New flags: `--source auto|overlay|recorded`, `--alpha`, `--point-px`, `--no-legend`.
- **Output names are unchanged, so the app needs no change.**
- **Fallback.** A run without a camera frame, camera_info, camera pose or mask keeps exactly its old stills and video, with no error.

**Why not the recorded-overlay option:** depth is recorded by default, so the masks can be rebuilt offline. Rendering the overlay topic on the car would cost tens of ms per frame.

### Extra CPU on the car
- **During a run:** `yolo_detector_node._debug_publish_clean` (`yolo_detector_node.py:1204`) JPEG-encodes the already-downscaled 480x270 tick frame, only while subscribed (`:1214`).
  - 0.43 ms per frame on linus; my Orin estimate is about 1-1.5 ms per 5 Hz tick (not measured).
  - About 12 KiB per frame, roughly 4 MB/min of bag.
  - Nothing at all when the logger is off (the default).
- **After the run (finalize):** 10 stills take 0.19-0.23 s warm on linus (0.42 s before the review fixes); not measured on the Orin.

### New params
| Param | Default | Where |
|---|---|---|
| `mission_report_overlay_alpha` | 0.3 | stack_params block at `:2698-2714` |
| `mission_report_overlay_point_px` | 2 | same block |
| `mission_report_overlay_legend` | true | same block |
| `debug_clean_image_topic` | `/camera/debug_image/clean/compressed` | yolo node param, not in stack_params |

- The report params are read once per process. The logger's finalize stills pick up a yaml edit at the next stack start; `mission_overlay` and `mission_debug_video` on their next run. They are not live.
- My stack_params hunks (`:350`, `:2676`, `:2698-2714`) are away from Stream A's (`:780`, `:3325+`).

### Review fixes
1. **Lookup by stamp (major).** `Topic.nearest` searched a fixed ±0.35 s window around the topic's median receive delay. When the recorder fell behind, it missed messages: 448 of 3223 `/scan` messages of corridor10m_20261019T131815 (delay -0.002 to 0.679 s) were not found by their own stamp.
   - It now bisects on the header stamps, starting from the same guess, and a zero (unstamped) message no longer ends the search.
   - Result: 0 of 3223 missed, and 10 of 10 stills with a sim clock at half rate (was 2 of 10).
2. **Stills in time order.** On short runs, neighbouring stills could pick each other's frames; the real example had come out as frames 3, 4, 2, 0, 1. `pick_stills` sorts them now.
3. **Params fallback.** A key missing from the run's snapshot now comes from the installed stack_params, not from a copy of the node's code defaults (the copy is gone). Rebuilt vs the car's own mask on the 10-19 obstacles bag, which has no snapshot: 90.6 % agreement before, 99.9 % on all 3 frames after.
4. **Faster params read.** The params snapshot is parsed with libyaml: 0.23 s down to 0.015 s.
5. **Finalize composition tested.** A new test runs the logger's own finalize step: without a mask the stills are the recorded JPEGs byte for byte; with a mask they are overlay stills.
6. **Docs.**
   - The storage warning now says the overlay reads sqlite3 only.
   - The stack_params descriptions for the snapshot source and the clean frame's JPEG quality are updated.
   - The "live" wording is corrected (see New params).

### Tests
- **`test_mission_overlay.py`: 13 passed.** This includes the 5 new ones. The 3 that cover review findings 1-3 fail on the code before the fixes.
- **Also new or updated:** the yolo clean-frame test, and the logger topic-list test.
- **Suites, compared with night-start/campaign-final:**

  | Suite | Result | Notes |
  |---|---|---|
  | logger | 325 passed (312 at baseline) | same 5 failed + 6 errors as baseline, all env-only on linus; flake8 stays at 3518 |
  | perception | 1232 passed | only the 2 known `TestPlacement` failures |
  | params | 80 passed | |

- **Lint:** no new messages in the changed files. All of them parse as Python 3.10.

### Example images
- **Real recorded data** (car's own mask + `/scan` + TF on `/camera/image_annotated`):
  `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_D/fix/cli_obst/corridor_obstacles_20261019T135307.004.jpg`
- **Mask rebuilt from depth**, on a scratch copy of 2026-10-21 with a clean topic synthesized from the debug JPEG (so YOLO's drawings show):
  - still: `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_D/fix/stills_new/2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission.004.jpg`
  - video: `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_D/fix/video_1021/2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission.debug.mp4` (89 frames)
- The recorded data was only read. The bags' sha256 still match after all reads.

### Packages to rebuild on the car (Humble)
`f1tenth_params`, `f1tenth_perception`, `f1tenth_logger` (new module and new command).

**Integration note:** f1tenth_logger now depends on f1tenth_perception. On linus, build it with `--packages-ignore-regex` for the zed packages and corridor_perception_c; `--packages-skip-regex` makes perception fail.

### Car checklist
1. Run a mission with `enable_mission_logger:=true` (default profile, sqlite3 storage). Check:
   - `ros2 bag info` shows `/camera/debug_image/clean/compressed` at about 5 Hz;
   - the finalize log says "N overlay snapshot(s)".
2. **Open the new run's report in the app (Run → Report) and check the overlay:**
   - light fills with thin outlines;
   - small LiDAR dots;
   - the small legend;
   - the camera image still readable;
   - stills in time order.
3. Optional: `ros2 run f1tenth_logger mission_debug_video <run_dir>`, then Run → Video. `--source recorded` gives the old video.
4. If the fills look too strong or too weak: edit `mission_report_overlay_*`, then restart the stack (or re-render with `--alpha` / `--point-px` / `--no-legend`).
5. Do not install the mcap storage plugin while the overlay is wanted.

### Not verified
- The clean JPEG plus rebuilt mask on a real run.
- Orin timings.
- A rebuilt mask is not the car's own: it can lag the image by up to 0.1 s.

### Decisions taken
- Offline rendering, with the one clean JPEG on the car, for the reasons above.
- `f1tenth_logger` now exec_depends on `f1tenth_perception` and `python3-opencv`.
- pyarrow went into the scratch dir with `pip --target`, only for the baseline extract tests.
- **Declined:** moving front_clearance_node's mask-tick glue into a shared function. That is the car's live mask path, and the spec only asks for the renderer to be shared. Instead, the defaults copy is gone and a test ties the rebuild's keys to stack_params.
- **Not done:** reading mcap bags; no code change for the wider build closure (integration note only).