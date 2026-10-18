# Walls in Foxglove — diagnosis and decisions

Orin, `campaign-final`, 2026-10-16. Stack was DOWN, so V1 is from code plus the
newest bag, `~/bags/corridor10m_20261016T170931` (read-only sqlite URI).

## Premise correction

The work order is headed "after the cpu-cleanup integration". **That integration did
not happen**: `cpu-cleanup` does not exist on GitHub (`git ls-remote --heads` →
campaign-final, jazzy, jazzy-campaign, main only). HEAD is `e67aded`, the merge of
linus's three campaign-final commits into the local H1-H5 series. So "cpu-cleanup:
wall layer 5 Hz" is NOT in this tree and cannot be a cause. H3 and ROLE_VALUE are.

## V1. Cause — five lines

1. `/perception/walls/markers` is published and healthy: 1845 msgs in the bag,
   `visualization_msgs/MarkerArray`, frame `odom`, ~10 Hz
   (`lidar_front_wall_node.py:646`, `_publish_walls` at :1395, fed from the scan
   callback at :1316 through the 4x tracker decimation at :554).
2. **It is not on the bridge.** `foxglove_topic_whitelist`
   (`stack_params.yaml:2435-2451`) is an *allowlist* of 15 anchored regexes —
   `^/viz/.*`, tf, latched, `^/costmap/semantic_markers$` — and
   `/perception/walls/markers` full-matches none of them, so foxglove_bridge never
   advertises the channel and the 3D panel cannot offer it. This alone is sufficient.
3. It is also not relayed: `viz_relays.yaml` has 16 relays and no wall entry, so no
   `/viz/...` copy exists either.
4. Even once visible, almost nothing would be drawn: `_publish_walls` is fed
   `wall_tracker.published(car)` (:1297), and `published()`
   (`multi_wall_tracker.py:1153-1207`) returns only `ever_confirmed` tracks whose role
   is front/left/right/back, capped at `max_published=4`. Measured in the bag: 1-2
   tracks per message, roles only `left` (400), `front` (72), `right` (6) — **zero
   `other`, zero tentative**. That is H3's "'OTHER' NEVER TAKES A SLOT" (:1174).
5. And the markers are built to vanish: every message starts with a
   **`DELETEALL`** (`:1399-1402`; 400 of 400 messages in the bag) and **no marker
   ever sets `lifetime`** (measured `(0,0)` on all 1356 markers), so the array is
   torn down and rebuilt every cycle with nothing to survive a dropped message.

Bag evidence, verbatim:

    markers/msg histogram: {3: 322, 5: 78}
    actions (0=ADD 2=DELETE 3=DELETEALL): {3: 400, 0: 956}
    ns: {'': 400, 'walls': 956}  frame_id: {'odom': 1356}
    lifetime (sec,nsec): {(0, 0): 1356}
    types (1=CUBE 9=TEXT): {0: 400, 1: 478, 9: 478}
    tracks/msg histogram: {1: 322, 2: 78}
    roles: {'left': 400, 'right': 6, 'front': 72}

Note the docstrings at `multi_wall_tracker.py:1174-1179` ("still tracked, **still
drawn**") and `lidar_front_wall_node.py:349-351` ("'other' ... so it is drawn --
dimly") are both **wrong**: `other` never reaches the marker builder, because the
builder is fed `published()`. The comments describe an intent H3 removed.

## Decisions

**D1. V2 does add `/perception/walls/markers` to the bridge allowlist**, as the order
asks, even though it breaks the repo's "only `/viz/*` copies cross the bridge" rule
(`viz_relays.yaml` header, `foxglove_bridge.launch.py:38-42`). It is cheap — 1-2
tracks, ~5 small markers, 10 Hz — and it means V2 stands on its own without V3. If
you later want the rule back, drop that one line and keep `/viz/walls`.

**D2. V3 needs no whitelist edit.** `^/viz/.*` (`stack_params.yaml:2437`) already
full-matches `/viz/walls`. Adding `^/viz/walls$` would be dead weight. The
`test_foxglove_whitelists.py` `shown` list is extended instead, so the intent is
locked by a test rather than by a redundant regex.

**D3. `/viz/walls` is published by the node, not by a `topic_tools` relay.** Every
other `/viz/*` topic is a throttled copy. A relay cannot work here: the order
requires ALL tracks including `other` and tentative, and those never exist on
`/perception/walls/markers` to be copied. The lazy-subscription property a relay
would have provided is reproduced directly with `get_subscription_count() > 0`.

**D4. Colour conflict, flagged.** The order specifies left **blue**, right
**yellow**; the existing `_ROLE_COLOUR` (`lidar_front_wall_node.py:347-351`) is left
**green**, right **blue**. I implemented the order as written for `/viz/walls` and
left `/perception/walls/markers` alone, so the two topics now disagree about what
blue means. Looking at both at once will mislead. Say the word and I will align them
either way; I did not pick for you.

**D5. One lifetime param for both topics**, `wall_marker_lifetime_s`, default 0.6 s.
There is no nominal-scan-rate param in the workspace to derive a period from. 0.6 s
is >= 3x the tracker's 0.1 s period (40 Hz / decimation 4) and exactly 3x the 0.2 s
`/viz/walls` period at its 5 Hz cap, so one number satisfies both. `sec` and `nanosec`
are split properly — packing a whole lifetime into `nanosec` is the uint32 assertion
that once crash-looped mpc_corr (`MPC_corr.py:5355-5368`).

**D6. Prism height.** The order says "0-2 m". Tracks carry a measured `z_min/z_max`
when the stereo saw the height. The prism uses the measured span when
`height_measured`, and 0-2 m otherwise, so a merely-assumed wall is still visibly the
default height.

---

## V2. What was changed on the existing path

`lidar_front_wall_node._publish_walls`, commit `a94f2cb`:

- the leading `DELETEALL` is gone;
- every marker carries `wall_marker_lifetime_s` (node param, default 0.6 s) through
  one `_set_lifetime` helper that splits sec/nanosec properly;
- a track that dies, or merely drops out of `published()`'s four slots, gets an
  explicit `DELETE` on both of its marker ids;
- `^/perception/walls/markers$` is added to `foxglove_topic_whitelist` (D1).

`test_every_array_starts_by_clearing_so_gone_tracks_disappear` asserted the
DELETEALL, so it is replaced by three tests that check what now carries its
guarantee. Its name argued for the behaviour that caused the bug, which is the
failure mode CLAUDE.md's "Name a test after what it asserts" section describes.

## V3/V4. `/viz/walls`

`wall_markers.py` (new, pure, no rclpy) + the node wiring, commits `30db6e2` and
`bf891be`. 44 builder tests, 8 node-level gate tests, 73 passing in
`test_lidar_front_wall_node.py`.

### Live verification — stack DOWN, so a real-DDS check instead

The order's `ros2 topic echo --once /viz/walls` needs a running stack. Instead: the
real node, a real subscriber in a second node, real DDS, no mocks
(`ROS_SUPER_CLIENT=TRUE`). Measured:

    topic: /viz/walls   viz_walls_enabled: True   max_hz: 5.0

    --- no subscriber ---
    subscription_count = 0
    builder drawn_ids (must be empty): set()

    --- subscriber matched ---
    subscription_count = 1
    arrays received over DDS: 10          <- 40 cycles x 0.05 s = 2.0 s at 5 Hz
    markers in the last array: 4
    actions (0=ADD 2=DELETE 3=DELETEALL): {0: 3, 2: 1}
    DELETEALL present: False
    ns: {'viz_walls'}   frame_id: {'odom'}
    lifetimes: {0.6, 0.0}
    ids: [4, 5, 6, 7]                     <- 4*1 + {0,1,2,3}
    label: '1 front unverified 1.60m'
    tracker holds ids: [1]

    --- subscriber gone ---
    subscription_count = 0
    new arrays after it left: 0
    builder drawn_ids (forgotten): set()

So all three gates hold on the wire, the id scheme is the one documented, and the
rate cap is exact: 2.0 s of scan stamps at a 5 Hz cap produced exactly 10 arrays.
The one `0.0` lifetime is the `DELETE` of the unused glass slot — a DELETE needs no
lifetime. The one ADD-track case is all the synthetic single-wall scan can produce;
the multi-track and 'other'/tentative cases are covered by the builder tests, which
construct real `mwt.Track` objects directly.

### Lint debt, measured before and after

| check | `f1tenth_perception` before (`e18e90d`) | after |
|---|---|---|
| `ament_flake8` | 18 | 18 |
| `ament_pep257` | 1307 | 1321 |

No new flake8 error: all 18 pre-exist (E128/F811/E306 in the node, E501/F401 in its
test, same five at shifted line numbers). The 14 added pep257 errors are all D213,
"summary should start at the second line", which this entire workspace violates by
house style — `wall_markers.py` writes first-line summaries like every file around
it. `f1tenth_perception`'s `test_pep257` was already failing on 1307.

## How to look at it in Foxglove

Add a **3D** panel, set **Fixed frame = `odom`**, and enable the topic:

- **`/viz/walls`** — every track: `other`, tentative, coasting and rejected
  included. Colour = role (front red, left blue, right yellow, back grey, other dark
  grey), opacity = state (verified 0.9, unverified 0.5, tentative/rejected 0.2),
  cyan outline = glass. 5 Hz, and only while the panel is open.
- **`/perception/walls/markers`** — the four walls the node asserts to the stack,
  ~10 Hz. Different colour convention (D4).

`map` also works as the fixed frame as long as SLAM is publishing `map -> odom`;
`odom` is the frame the markers are actually in, so it is the one that cannot be
wrong.
