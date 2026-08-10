# f1tenth_localization

Owns `map → odom` (via one of two mutually-exclusive sources, selected by
`localization_source`) plus the static `odom → base_link` identity TF the
EKF's correction composes against.

## Nodes

| Node | Package | Subscribes | Publishes | Role |
|---|---|---|---|---|
| `ekf_filter_node` | `robot_localization` (external) | `/odom` (x,y,yaw), `/sensors/imu/raw` (yaw rate + linear accel) | `map → odom` TF, `/odometry/filtered` | Only when `localization_source=ekf`. Config: `f1tenth_bringup/config/ekf.yaml` (`world_frame: map`, so it publishes `map → odom`, not `odom → base_link` — that edge is a separate fixed static TF, see below). Replaces the VESC odometry node's own in-node Kalman filter (that node's `publish_tf` stays `false`). |
| `raw_odom_map_tf_node` | this package | `/odom` | `map → odom` TF (direct mirror, no filtering) | Only when `localization_source=raw_odom` (**the current default**) — used while EKF tuning is in progress. |

## Launch files

| File | Purpose |
|---|---|
| `localization.launch.py` | Top-level entry: resolves `localization_source` (plain `stack_params.yaml` read, not a `DeclareLaunchArgument`) and includes `ekf.launch.py` or `raw_odom.launch.py` accordingly. Also publishes the static **`odom → base_link`** identity TF unconditionally, and includes `f1tenth_description/description.launch.py` (robot_state_publisher + static sensor TFs) as part of the same restartable group. |
| `ekf.launch.py` | Just the `ekf_filter_node`, config sourced from `ekf_config` (`f1tenth_bringup/config/ekf.yaml`). |
| `raw_odom.launch.py` | Just `raw_odom_map_tf_node`, no config file — trivial passthrough. |

The `odom → base_link` static TF being published *here* (rather than
`f1tenth_navigation`, where it used to live) matters: previously it was only
a side effect of `nav2.launch.py` (gated on `enable_nav2`), so
`enable_nav2:=false` runs published **no** `odom → base_link` TF at all — a
real, since-fixed bug, found while making the BT drive without Nav2. It's
needed regardless of `localization_source` (even `raw_odom`'s direct
`map → odom` mirror doesn't need it directly, but everything downstream that
looks up `base_link`'s pose in `map` needs the full `map → odom → base_link`
chain) and regardless of `enable_nav2`.

## Config

`f1tenth_bringup/config/ekf.yaml` (not owned by this package, but the only
config file this package's launch files consume) — `robot_localization` EKF
tuning: `world_frame: map`, fuses `/odom` (x, y, yaw) + `/sensors/imu/raw`
(yaw rate only — accel axes are declared in the fusion mask but not
currently trusted, see that file's own comments), `two_d_mode: true`.

## Consumed `stack_params.yaml` keys

`localization_source`, `ekf_config` — see each key's own `# Consumed by:`
comment in `stack_params.yaml`.

## Known limitations

- `raw_odom_map_tf_node` is a direct, unfiltered mirror — no smoothing, no
  IMU fusion, no covariance estimate. It's an explicitly temporary fallback
  ("while EKF tuning is in progress" per its own launch file docstring), and
  it's still the current default (`localization_source: raw_odom`) — the EKF
  path exists and works but isn't the deployed default as of this writing.
- `ekf.launch.py`'s module docstring still refers to "6 stack-wide branching
  args" (should be 5, `enable_safety_stop` was removed) — a stale comment
  count with no functional effect, not fixed as part of this documentation
  pass.
