# tracks/

CSV waypoint files for `track_mpc_opt_node` (node `track_mpc_controller`).

Expected columns: `x, y`   (metric coordinates, one row per waypoint)

Additional columns are ignored. A leading `#` comment line (header) is skipped.

The node closes the path into a loop automatically (it appends the first
waypoint if the last point is more than 0.5 m from it), so files may be either
open or already-closed loops.

Select a file with the `track_file` ROS 2 parameter (path relative to the
package share directory), e.g.:

```bash
ros2 run mpc_controller track_mpc_controller --ros-args -p track_file:=tracks/random.csv
```
