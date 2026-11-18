# The runs, and which carry the data this needs

305 recordings found under `~/bags`, `~/f1tenth_archive/*` and the campaign folders. **2 usable.**

A run is usable when it has all of `/scan`, `/perception/virtual_wall/debug` (the car's own tracker steps -- without it a replay cannot ask the same question on the same cycles), `/odometry/filtered` and `/tf_static`, at least 20 recorded steps, and a camera input: recorded `/perception/stereo_columns` (replayed as pass `rec`) or recorded depth (pass `full`, mask and columns recomputed).

## Usable

| run | dur [s] | steps | camera pass | video mask | depth | cols | classes | path [m] | moving | camera pitch [rad] |
|---|---|---|---|---|---|---|---|---|---|---|
| `corridor10m_20261016T144537` | 164.5 | 2830 | rec | recorded | 0 | 625 | 625 | 14.93 | 22 % | -0.03635 |
| `2026-10-15T15-18-27_mission-llm_86cff518e42d` | 13.9 | 53 | rec | replay | 50 | 41 | 0 | 4.88 | 100 % | -0.03635 |

A pitch of `+0.00000` is a bag recorded before the camera pitch was measured (2026-10-15): its own `/tf_static` says the camera looks straight ahead, so every camera projection in that replay is off by 2.1 deg. The replay uses the bag's TF, which is what the car had -- so those runs are honest about the car and weak evidence about the geometry. Q0 (`c3eec321`) measured this.

## Dropped, but close (they have the debug topic)

| run | steps | why |
|---|---|---|
| `corridor10m_20261016T170931` | 7175 | no camera input: neither recorded stereo columns nor depth, so no camera observation ever reaches the tracker and the camera gates cannot differ between variants |
| `2026-10-14T12-05-34_mission-llm_2b356aac445f` | 444 | its /perception/virtual_wall/debug payload has no `state`: recorded before the WALL / CLEAR / NO_DATA contract existed, so no recorded cycle can be placed against a replayed one |
| `2026-10-14T12-56-47_mission-llm_2b356aac445f` | 80 | its /perception/virtual_wall/debug payload has no `state`: recorded before the WALL / CLEAR / NO_DATA contract existed, so no recorded cycle can be placed against a replayed one |
| `2026-10-15T11-18-01_mission-llm_86cff518e42d` | 153 | its /perception/virtual_wall/debug payload has no `state`: recorded before the WALL / CLEAR / NO_DATA contract existed, so no recorded cycle can be placed against a replayed one |
| `2026-10-14T13-11-30_mission-llm_be55eb2392c4` | 61 | its /perception/virtual_wall/debug payload has no `state`: recorded before the WALL / CLEAR / NO_DATA contract existed, so no recorded cycle can be placed against a replayed one |
| `2026-10-14T13-12-37_mission-llm_2b356aac445f` | 27 | its /perception/virtual_wall/debug payload has no `state`: recorded before the WALL / CLEAR / NO_DATA contract existed, so no recorded cycle can be placed against a replayed one |
| `2026-10-14T13-15-20_mission-llm_2b356aac445f` | 1 | only 1 recorded tracker step(s) (fewer than 20) |

The remaining 296 recordings have no `/perception/virtual_wall/debug` at all (or no messages whatever) and are not listed: the topic is newer than they are.
