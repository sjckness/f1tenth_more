# Mission recorder: late start and empty bags

*Analysis and proposed fix for the next campaign. Nothing is implemented yet.*
*Data: the 58 runs of `first_test_campaing`, matched to `~/f1tenth_archive` by
`export_matlab` (2026-10-01).*

## What the first campaign shows

| | |
|---|---|
| Campaign runs | 58 |
| Matched to an archive run (start within 2 s, unambiguous) | 58 |
| Bag with data | 41 |
| **Empty bag** (valid metadata, 0 messages) | **17**: all 10 of M03, 6 of M04, P002-R006 |
| First bag message after `mission_started` | median **8.2 s** (4.9 to 13.7 s) |
| Share of the drive covered by the bag (bags with data) | median **42 %** (5 to 76 %) |
| Bag end vs. drive end | median +0.03 s: the bag stops exactly at the terminal state, with no post-roll |

Empty bags are not limited to short missions. P004-R017 drove 13.5 s and
recorded nothing.

One run, P005-R001 (a 208 s drive), shows a different problem. Its bag
stopped 187 s before the drive ended (5 % coverage), so the recorder ended
early on a state transition. That needs a separate look at its
`/mission/status` sequence.

## Why

`mission_logger_node._start_recording` creates a new `rosbag2_py.Recorder` on
the transition **to RUNNING**, with `is_discovery_disabled = False` and a
100 ms polling interval. The recorder then has to discover all 37 topics and
create a subscription for each before it writes anything.

The node's own docstring measured this at 2.4 to 3.4 s. With the full stack
under the Discovery Server it now takes 5 to 14 s, which is longer than most
of this campaign's drives (median 9 to 13 s).

Stopping on the terminal state is immediate, so nothing after the end is kept
either.

The test-campaign logger has none of these problems: it is a long-lived node,
subscribed before the test opens, with a 2 s ring for pre-roll and 2 s of
post-roll. That is why its test folders are complete while the bags are not.

## Proposed fix: a pre-armed recorder

The idea is to do the discovery while nothing is happening. The local install
is rosbag2 0.15.16 (Humble), and its `rosbag2_py.RecordOptions` has
`start_paused`; `StorageOptions` has `snapshot_mode`.

1. **Arm at startup and after every run.** Create the Recorder with the fixed
   topic list (`record_options.topics`, as today) and
   `record_options.start_paused = True`, writing to a fresh
   `active/<pending>/bag`. Discovery and subscription happen while it sits
   paused, usually minutes before the next mission. Use `node_prefix` to give
   the recorder node a known name.
2. **Resume at mission LOAD.** On the transition to LOADED, call the
   recorder's resume service and rename the pending run to its real run_id.
   The bag then holds the 3 s countdown, which acts as a free pre-roll, and the
   whole drive from its first tick.
3. **Post-roll, then stop.** On COMPLETE / ABORTED / EMERGENCY_STOP, keep
   recording for `postroll_s` (2 s, the same as the campaign logger), then
   `cancel()` to write metadata.yaml. Re-arm the next recorder straight away.
   Fill in the `preroll_s` and `postroll_s` that the manifest schema already
   has (always empty today).
4. **Flag it honestly.** Write `first_message_rel_s` (first bag stamp relative
   to RUNNING) into the manifest at stop. A run whose bag starts after RUNNING
   is marked `late_start` instead of looking complete.

Cost: one idle set of subscriptions between missions. The recorder takes
serialized messages and never deserializes them, so the cost is mostly DDS
traffic for the image topics (about 5 MB/s at 7 Hz). Measure the CPU on the
Jetson while it sits paused before committing to it.

To verify: the exact service names (`~/resume`, `~/pause`) of the 0.15.16
recorder node, by `ros2 service list` with a paused recorder up.

### Alternative: always-on ring (`snapshot_mode`)

Run one recorder for the whole session with `snapshot_mode = True` and
`max_cache_size` sized for about 60 s, and trigger `~/snapshot` at mission end.

This captures the pre-roll by construction. It costs RAM, though: about
350 MB for 60 s with the camera images, or under 60 MB without them. Humble
appends successive snapshots to one bag, so per-mission bags would need a
split step afterwards. Choose this only if step 1 turns out not to remove the
discovery delay.

## Also for the next campaign

- Add `/sensors/imu/raw` and the VESC state (`/sensors/core`) to the recorder's
  topic list. Neither is recorded today, so the MATLAB export has no VESC data
  and no IMU quaternion.
- Acceptance check after the change: run `export_matlab` on the first new
  missions. It should report every bag `attached`, with `bag_t_start_rel <= 0`
  and coverage near 100 %.
