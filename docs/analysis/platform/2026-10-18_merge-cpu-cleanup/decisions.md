# Decisions — merging cpu-cleanup into campaign-final, 2026-10-18

## D0. The merge needed no decision

`git merge --no-ff gh/cpu-cleanup` produced **no conflicts**. The three files both
branches touched (`stack_params.yaml`, `lidar_front_wall_node.py`,
`test_lidar_front_wall_node.py`) merged by disjoint hunk, and both sides' whole
contribution is present in the result (hunk counts in `log.md` section 1). None of the
three collisions the work order anticipated — front role/cap, contract publishing, wall
layer rate vs `/viz/walls` lifetime — turned into a textual or semantic clash:

* **front role / cap.** H1-H5 (cap by role, the C backend, decimation) sit in the
  merge-base `9c46add`, so cpu-cleanup is already built on them; R2 block 3 changes
  which observations may become a FRONT, not how many tracks are published.
* **contract publishing.** V1-V4 only add `/viz/walls` and a per-marker lifetime on
  `/perception/walls/markers`. They never touch `_publish_virtual`, which is the single
  writer of the WALL / CLEAR / NO_DATA contract. Proved empirically, not just read:
  the replay through the merged tree and through `82c0afa` alone are identical on every
  headline metric (`log.md` section 5, result 1).
* **wall layer rate vs lifetime.** `semantic_layer_rate_hz` 5.0 (launch) ==
  `wall_layer_rate_hz` 5.0 (node timer); `wall_marker_lifetime_s` 0.6 s ==
  3 x the 0.2 s period. The two were already sized against each other, by V2's own
  comment and by `test_no_marker_outlives_the_tracker_by_more_than_its_lifetime`,
  which asserts `lifetime >= 3 * (4/40)`. Nothing to reconcile.

## D1. A no-transform scan now publishes its NO_DATA (commit `0c1cd417`)

**This is a defect inside cpu-cleanup, not a merge artefact, and it is the one place
where two intents did collide.** It is a separate commit on top of the merge so you can
drop it alone.

The clash is between two blocks of the same branch:

* **R2 block 3** (`d578963`): "No transform: the scan cannot be placed, so the contract
  is NO_DATA — not whatever the last placed scan said", with
  `test_a_scan_with_no_laser_transform_is_NO_DATA` to prove it.
* **R2 block 2** (`9786bfc`): `/perception/virtual_wall` is published only on a NEW
  estimate — `virtual_wall_due(tracker_on, tracker_ran, ...)` returns False whenever the
  tracker is on and did not step this scan.

The no-transform branch of `_scan_cb` cannot step the tracker: it returns before the
tracker runs, leaving `self._tracker_ran` False from the top of the callback. So with
the tracker on, block 3's verdict is computed and then dropped on the floor, and the
last placed scan's verdict stands on the wire for as long as `/tf_static` is missing —
exactly the failure block 3 names.

Block 3's own test was therefore **already red on `gh/cpu-cleanup`**. Verified rather
than assumed: `git archive 82c0afa src/f1tenth_perception` into a scratch directory and
run the file there — `1 failed, 5 passed`, same `TypeError: 'NoneType' object is not
subscriptable` from `_last(n.virtual_debug_pub)`, i.e. nothing was published at all.

**Resolution, keeping both intents.** A no-transform scan carries a new verdict, so it
is offered to the gate; the gate still applies `virtual_wall_rate_hz`.

    # lidar_front_wall_node.py, __init__
    + self._no_transform_verdict = False

    # _scan_cb, per scan
      self._tracker_ran = False
    + self._no_transform_verdict = False

    # _scan_cb, the laser_pose is None branch
      self._tracker_has_data = False
      self._front_d_ray = math.nan
    + self._no_transform_verdict = True

    # _virtual_due
    - due = virtual_wall_due(self.wall_tracker is not None, self._tracker_ran, now,
    + due = virtual_wall_due(self.wall_tracker is not None,
    +                        self._tracker_ran or self._no_transform_verdict, now,
                             self._last_virtual_pub, self.virtual_wall_rate_hz)

Block 3 gets its NO_DATA on the wire; block 2 keeps its cap, so a missing `/tf_static`
publishes NO_DATA at `virtual_wall_rate_hz` (10 Hz) and not at the 40 Hz scan rate. A
new flag rather than setting `_tracker_ran = True`, because the tracker did not run and
that name is the gate's only other input — lying to it would be a trap for the next
reader.

After: `test_lidar_front_wall_node.py`, `test_wall_markers.py`,
`test_multi_wall_tracker.py`, `test_virtual_wall_rate.py` — **212 passed**, including
the five rate-cap tests that would have caught an uncapped publish.

## D2. Not decided here — cpu-cleanup's contract shift

`log.md` section 5, result 2. The merged pipeline reports CLEAR for 160 of this bag's
163 s where the pre-merge one reported WALL for 124 s, and agreement with the recorded
car run falls from 93 % to 12 %. It removes the 147-150 s phantom and all 23 near-field
junk tracks, which is what R2 block 3 was for. It is **identical on `82c0afa` without
this merge**, so it is a property of the branch you are merging, not of the merge, and
no choice taken here changes it. Flagged for you rather than resolved: this bag never
comes within 8.6 m of anything, so it cannot settle whether losing the far wall on the
approach is acceptable.
