  # Simulated ZED 2i images not visible — diagnosis

  Date: 2026-10-06
  Host run on: **linus** (sim host). Read-only diagnosis — nothing was edited or
  restarted. The two offline steps run (`xacro` expand + `gz sdf -p`) operate on a
  throwaway `/tmp/r.urdf` and do not touch the repo or any running process.

  ## Verdict (stop at first failing layer)

  **L1 fails: there is no Gazebo server running on linus.** `gz topic -l` is empty,
  `/clock` has 0 publishers and is not ticking, and `/scan` / `/camera/*` do not
  exist in the ROS graph. The only ROS nodes alive are orphaned leftovers
  (`sim_robot_state_publisher` + two `f1tenth_sim_drive_bridge`), ~21 h old. The
  `gz sim` server, `ros_gz_bridge`, the camera bridge, and the `create`/spawner
  steps are all absent.

  Because the sim server is down, no sensor is rendered, so no `/sim/zed2/*` gz
  topics exist for the camera bridge to forward — and the camera bridge isn't
  running either. **No image can reach the Thor because nothing is producing one
  on linus.** The camera model itself is correct (see L1b): the sensors are not
  the problem; the missing simulator is.

  L2/L3/L4 were **not reached** — their inputs (`/camera/image_raw`,
  `/camera/camera_info`, `/scan`) do not exist while the sim is down. L3/L4 are
  Thor-side and this session is on linus.

  ### Fix
  Relaunch the sim on linus:
  `ros2 launch f1tenth_sim sim_bringup.launch.py` (camera:=true is the default).
  Then re-run this trace from L1.

  ---

  ## L1 — Gazebo (linus)

  ### gz topics
  ```
  $ gz topic -l            # (sourced /opt/ros/jazzy + install/setup.bash)
                          # <no output>
  exit: 0
  $ gz topic -l | grep -i zed2
                          # <no output>, exit 1 (grep: no match)
  ```
  Expected `/sim/zed2/rgb/image`, `/sim/zed2/rgb/camera_info`,
  `/sim/zed2/depth/image`, `/sim/zed2/depth/camera_info` — **none present. No gz
  topics of any kind → no gz server.**

  ### Process inventory
  `ps -eo pid,etimes,args` (filtered) — no `gz sim` / `ruby` server, no
  `parameter_bridge`, no `ros_gz_sim create`/`spawner`. Only:
  ```
  1174153   75627  …/f1tenth_sim/lib/f1tenth_sim/drive_bridge  (node f1tenth_sim_drive_bridge)
  1175121   75588  …/f1tenth_sim/lib/f1tenth_sim/drive_bridge  (node f1tenth_sim_drive_bridge)
  1175117   75588  robot_state_publisher  (node sim_robot_state_publisher, /sim/* remaps)
  ```
  etimes ≈ 75 600 s ≈ 21 h: these predate today's session; they are orphans from a
  sim launch whose gz server has since exited.

  ### ROS graph
  ```
  $ ros2 node list
  /f1tenth_sim_drive_bridge
  /f1tenth_sim_drive_bridge
  /sim_robot_state_publisher

  $ ros2 topic list
  /ackermann_drive
  /ackermann_steering_controller/odometry
  /ackermann_steering_controller/reference
  /clock
  /odom
  /parameter_events
  /rosout
  /sensors/imu/raw
  /sim/imu_raw
  /sim/joint_states
  /sim/robot_description
  /sim/tf
  /sim/tf_static
  ```
  No `/scan`, no `/camera/image_raw`, no `/camera/camera_info`, no
  `/zed2/zed_node/depth/*`.

  ### Publisher / liveness checks
  ```
  $ ros2 topic info /clock   -> Publisher count: 0   Subscription count: 3
  $ ros2 topic info /odom    -> Publisher count: 2   Subscription count: 0
  $ ros2 topic info /scan             -> Unknown topic '/scan'
  $ ros2 topic info /camera/image_raw -> Unknown topic '/camera/image_raw'
  $ ros2 topic info /camera/camera_info -> Unknown topic '/camera/camera_info'

  $ ros2 topic hz /clock   (5 s)  -> <no output; clock not ticking>
  ```
  `/clock` with 0 publishers confirms the gz→ROS clock bridge is gone; the drive
  bridges are the two stale `/odom` publishers.

  ## L1b — Camera survives URDF→SDF (offline, read-only)

  Even though the sim is down, the model definition was verified so the camera is
  ruled out as a cause for when the sim is relaunched.

  ```
  $ xacro .../urdf/roboracer.urdf.xacro use_sim:=true enable_sensors:=true \
          enable_camera_mock:=true pkg_share:=<desc_share> \
          control_config:=<f1tenth_sim>/config/controllers.yaml > /tmp/r.urdf
  xacro exit=0 ; urdf bytes=20713
  ```
  URDF declares both sensors on `zed2_left_camera_frame` with the expected topics:
  ```
  399:  <sensor name="zed2_rgb" type="camera">
  400:      <topic>/sim/zed2/rgb/image</topic>
  406:      <camera_info_topic>/sim/zed2/rgb/camera_info</camera_info_topic>
  419:  <sensor name="zed2_depth" type="depth_camera">
  420:      <topic>/sim/zed2/depth/image</topic>
  426:      <camera_info_topic>/sim/zed2/depth/camera_info</camera_info_topic>
  ```
  `gz sdf -p /tmp/r.urdf` — both sensors survive conversion, lumped onto
  `base_link`:
  ```
  124:      <sensor name='zed2_rgb' type='camera'>
  125:        <topic>/sim/zed2/rgb/image</topic>
  130:          <camera_info_topic>/sim/zed2/rgb/camera_info</camera_info_topic>
  143:        <gz_frame_id>zed2_left_camera_frame_optical</gz_frame_id>
  145:      <sensor name='zed2_depth' type='depth_camera'>
  146:        <topic>/sim/zed2/depth/image</topic>
  151:          <camera_info_topic>/sim/zed2/depth/camera_info</camera_info_topic>
  164:        <gz_frame_id>zed2_left_camera_frame_optical</gz_frame_id>
  ```
  Only stderr is the benign gz extension passthrough (not an error), for every
  sensor incl. imu/hokuyo:
  ```
  Warning [Utils.cc:132] XML Element[gz_frame_id], child of element[sensor], not
  defined in SDF. Copying[gz_frame_id] as children of [sensor].
  ```
  No OGRE/render/camera errors (there is no running server to emit any; a live
  `gz sim -v 4` render-error check was intentionally **not** run — it would start a
  new server, which is out of scope for a read-only pass while the stack expects
  the sim already up).

  ## Bridge mapping (reference, from config — not a live check)

  `src/f1tenth_sim/config/ros_gz_bridge_camera.yaml` (bridged only with
  camera:=true) maps GZ→ROS:

  | gz topic | ROS topic |
  |---|---|
  | /sim/zed2/rgb/image | /camera/image_raw |
  | /sim/zed2/rgb/camera_info | /camera/camera_info |
  | /sim/zed2/depth/image | /zed2/zed_node/depth/depth_registered |
  | /sim/zed2/depth/camera_info | /zed2/zed_node/depth/camera_info |

  This config is correct; it was never instantiated because the camera bridge
  process is not running.

  ## Not reached

  - **L2 (ROS bridge, linus):** `/camera/image_raw`, `/camera/camera_info` do not
    exist — nothing to measure.
  - **L3 (across machines, Thor):** requires running on the Thor under the stack's
    discovery env; also moot while linus produces nothing.
  - **L4 (consumers, Thor):** Foxglove panel / `yolo_detector_node` / tf_echo —
    Thor-side, not reachable from this linus session.
