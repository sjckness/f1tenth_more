# F4 — `other` in the semantic grid, and who reads that grid

`wall_raster.ROLE_VALUE` gains `'other': 50`, and the fallback for an unknown
role becomes `'other'` instead of `'back'`.

| role | cell value |
|---|---|
| front | 10 |
| left | 20 |
| right | 30 |
| back | 40 |
| **other** | **50** (new) |

Multiples of ten, no value reused, all inside the OccupancyGrid convention's
0..100 so the grid still reads on its own in any viewer. Before this, every
`other` wall -- which since E3 is most of the tracks in a corridor -- was filed
by `ROLE_VALUE.get(role, ROLE_VALUE['back'])` as the wall BEHIND the car. The
cells were right (a real surface, occupied); the label was wrong.

## Every consumer of the two topics the semantic layer publishes

`semantic_layer_node` publishes `/costmap/wall_cells` (OccupancyGrid, the role
values) and `/costmap/wall_cell_ids` (Int32MultiArray, the track id per cell).
Searched the whole workspace for both topic names and for `ROLE_VALUE` /
`VALUE_ROLE`:

| consumer | reads | breaks on 50? |
|---|---|---|
| `mission_logger_node` (`_DEFAULT_TOPICS`) | records `/costmap/wall_cells` into the bag | no. It copies bytes; it never interprets a cell value. |
| `wall_raster.VALUE_ROLE` | the inverse map, used only by `test_wall_raster.py` | no, and it now resolves 50 -> `other` instead of raising a KeyError |
| `costmap_renderer_node` | `/map` (OccupancyGrid) + `/costmap/semantic_markers` (MarkerArray of OBJECTS) | not a consumer of this grid at all |
| `costmap_boundary_node` | `/map`, the pose, `/costmap/safe_corridor_report` | not a consumer |
| MPC (`MPC_corr.py`) | `/costmap/boundaries`, `/costmap/front_clearance`, obstacles | not a consumer |
| behaviour tree / `behavior_executor_node` | `/perception/*`, `/costmap/front_clearance` | not a consumer |
| nav2 / slam | `/map`, `/slam/map` | not a consumer |
| foxglove relays (`viz_relays.yaml`) | `/costmap/semantic_markers`, not the wall grid | not a consumer |

**Nothing in the control path subscribes to `/costmap/wall_cells`.** Its only
runtime consumer is the recorder, and the only code that maps a value back to a
role is a test. So the new value cannot be misread by anything that exists
today, and a viewer that colours by value now shows `other` walls as their own
shade instead of as `back`.

Two regression tests added (`test_wall_raster.py`): `other` gets its own value
and never lands on `back`, and an unknown role falls back to `other`.
