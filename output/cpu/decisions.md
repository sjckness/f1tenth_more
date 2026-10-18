# Decisions taken unattended (H1..H5)

## H1 — the C refuses a non-uniform bearing grid, and the dispatcher sends it to the Python

The specified API takes `angle_min` and `angle_increment`, so the C can only
express a uniform grid -- which is exactly what a `LaserScan` is. But
`extract_segments` is also called with arbitrary bearings: `test_extraction.py
::test_merge_keeps_parallel_offset_surfaces_apart` synthesises points and sorts
their `arctan2`, and `corridor_perception.replay` re-derives bearings from a
cloud. Rebuilding a uniform grid from `a[0]` and a mean increment there is a
silently RESAMPLED scan, and it changed that test's answer from 2 segments to 3.

Decision: `extraction_c.uniform_angles()` checks the array against the
progression to 1e-12 rad; the dispatcher runs the reference when it fails, and
the C itself raises rather than guessing. Two tests pin both halves.

- Alternative A: pass the angle array to the C as well. Rejected: the point
  fixes the API, and two sources of truth for the same numbers is how the
  bit-exactness argument gets lost.
- Alternative B: let the C resample. Rejected outright -- it is wrong, and it
  was wrong silently.
- Closest to current behaviour: the reference's answer, which is what this is.

## H1 — both backends are compared on the same float32 values

The C's API takes float32 ranges. To measure the ALGORITHM rather than float32
rounding of the input, `extract_backends_equiv.py` and the backend tests feed
the Python the same float32 values widened back to float64. In production the C
sees the float32 of a float64 range, a ~1e-7 m difference on a range, four
orders of magnitude under the 1e-4 tolerance.
## H2 — the cycle-count thresholds KEEP their counts; the report states the time

The point allows either. Counts kept, for the reason the rules give: it is what
the car does today and what the tests pin, and a hits-in-window rule expressed
in seconds changes meaning when the cycle rate varies (which is exactly what a
CPU-bound node does).

And the arithmetic favours it. Today the node is not running at 40 Hz: the
nightly audit measured **4.2 observations a second** because it was CPU-bound.
Decimation 4 at the nominal 40 Hz scan rate is **10 Hz**, so every count-based
threshold becomes FASTER in wall-clock than it is today, not slower. The table
is in the report.

- Alternative: convert confirm_hits / stereo_confirm_hits /
  negative_miss_to_kill / rect_reject_cycles to seconds. Rejected for now: four
  new parameters, four changed semantics, and no measurement yet of the rate
  they would be set against. Revisit once H5 has measured the real cycle rate
  on the car with H1 in.

## H2 — a skipped scan HOLDS the tracker's last verdict

The node publishes its contract on every scan (its own docstring makes that a
hard contract). On a scan the tracker skips, `_front_d_ray`,
`_tracker_has_data` and `_tracked_wall` are left exactly as the last processed
scan left them, so the contract repeats a verdict at most 75 ms old.

- Alternative: publish NO_DATA on skipped scans. Rejected: it would flap the
  contract three scans in four, and NO_DATA means "the sensors are not telling
  us", which is false here.
- The debug payload carries `tracker_skipped` so a consumer can tell a held
  verdict from a fresh one.

## H2 — every replay forces decimation 1

A bag holds one recorded tracker CYCLE per `/perception/virtual_wall/debug`
message (~3.2 a second on this recording), not the 40 scans a second the car
sees. Decimating that again would compare a different set of scans against the
recording, and would leave three cycles in four reporting the previous cycle's
`tracker_ms`. `WALL_TRACKER_DECIMATION` overrides it for a run that wants to
see the decimation itself.
## H3 — 'other' takes no slot, so it never reaches the semantic grid

The point is explicit, and this is the consequence worth writing down: since
H3 an `other` track is not on `/perception/walls` at all, and
`semantic_layer_node` only rasterises what is published. So the `ROLE_VALUE
['other'] = 50` cell added last night (F4) is now unreachable through the
tracker path -- correct for an unknown role arriving from anywhere else, dead
for this one.

- Not changed: F4's value is still the right answer for a role this module has
  never heard of, and removing it would put `other` back on `back`.
- The `other` tracks are still visible: `/perception/virtual_wall/debug`'s
  `tracks` list carries every live track with its role when
  `wall_debug_tracks` is on, and the top-down video draws them.
- If the costmap should carry them as occupied-but-unlabelled surfaces, that is
  a decision about the costmap, not about the cap, and it needs its own point.

## H3 — FRONT takes every slot it needs, sides get one each

The order is: all FRONT tracks (nearest along the heading first), then the
nearest LEFT, then the nearest RIGHT, then BACK, then any remaining side
walls so the budget is never left unspent. "FRONT always (if present)" read as
"front is never the thing that drops off the end", which is the defect: a
front wall at rank 4 of 7 was being hidden on 36 of 206 cycles.
## H5 — pidstat is not installed, so the probe reads /proc

`pidstat -u -t -d 1` is named in the point and is not available on this box (no
`sysstat` package), and installing one needs sudo, which the point forbids.
`tools/cpu_test_probe.py procstat` reads the same numbers without root:
per-thread `utime + stime` from `/proc/<pid>/task/<tid>/stat`, per-process
`read_bytes`/`write_bytes` from `/proc/<pid>/io`, the CPU a thread last ran on
and its affinity mask. `cpu_test.sh` uses `pidstat` when it IS present and the
probe otherwise, and `summary.md` says which ran.

## H5 — `jetson_clocks --show` needs root and is not given it

It refuses without sudo. The script records the refusal verbatim and reads
`/sys/devices/system/cpu/*/cpufreq` (current, min, max, governor) and the
thermal zones instead, which need nothing. `nvpmodel -q` and `tegrastats` both
work unprivileged on this JetPack and are used directly.

## H5 — the dry run plays a bag into lidar_front_wall_node only

The point asks for "the stack's perception nodes only". The dry run launches
`lidar_front_wall.launch.py` and plays the recording's `/scan`,
`/odometry/filtered`, `/tf`, `/tf_static`, `/camera/detections_3d`,
`/perception/stereo_columns`, `/perception/wall_points`,
`/costmap/front_clearance` and the depth `camera_info` into it. Not
`front_clearance_node`: its mask tick needs a depth IMAGE on its own topic,
and the only depth in this bag is the 5 Hz throttled copy on a different topic
name, so it would idle and measure nothing. The rectangle verification is
therefore also absent from the dry run -- `/perception/scene_labels/classes`
did not exist when the bag was recorded. The dry-run numbers measure the
decimated tracker and the recorder, and nothing else; the report says so.

## H5 — phase b publishes one /mission/status to start the recorder

`mission_logger_node` starts recording on a `/mission/status` transition and
on nothing else. In a dry run no behaviour tree is running, so the script
publishes one RUNNING message and one COMPLETE at the end. It is not a sensor
driver and not a command path. On the car this is unnecessary: a real mission
provides it.
## H5 — `ros2 topic pub -1` blocks, so every publish is rate-limited under a timeout

The first dry run of `cpu_test.sh` HUNG in phase b. `ros2 topic pub -1` exits
only once it believes the message is away, and blocks indefinitely when no
subscriber has matched yet -- and `mission_logger_node` needs 5-14 s of
discovery on this box before it is subscribed at all, so a single message would
have been lost even if the pub had returned. Both publishes are now
`timeout N ros2 topic pub -r 2 ...`, which also means the recorder gets the
transition whenever it finishes discovering.

Found by running it, which is what the dry run is for. The cleanup trap did its
job: the two processes the script owned were killed and nothing else was
touched.
## H5 — the dry run unsets ROS_DISCOVERY_SERVER; the car run does not

`ROS_DISCOVERY_SERVER=127.0.0.1:11811` is set in this box's environment, and
`supervisor_bringup.launch.py` is what starts that server. With the stack down,
every participant points at a server that is not there and nothing discovers
anything: the second dry run subscribed to all eleven topics and counted zero
messages on every one of them, with the bag demonstrably playing.

Decision: `--dry-run` unsets `ROS_DISCOVERY_SERVER` for its own children, which
puts them on plain multicast discovery. A side effect worth having: they then
cannot see, or be seen by, a live stack. On the car the script does NOT touch
the variable -- there the real stack and the probes must share the server.
## H5 — the cleanup kills process GROUPS, after a dry run left two orphans

The first dry runs left two `depth_throttle_5hz` nodes behind. `ros2 launch`
starts them, and killing the launch pid does not always give it time to forward
the signal. Each `ros2 launch` child is now started with `setsid`, so it leads
its own process group, and `cleanup()` signals the group.

- Alternative: `pkill -f depth_throttle_5hz` and friends. **Rejected outright**:
  on the car that would reach the live stack's own nodes, which is exactly what
  this script must never do. Process groups mean it can only ever signal what
  it started.
