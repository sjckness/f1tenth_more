GOAL
Drive the real ZED 2i pan motor from ROS 2 on the car. Two nodes:
  1. camera_pan_controller  - computes where the camera should look (direction the car is GOING, on the arc ahead)
                              and publishes the command angle
  2. camera_pan_driver      - talks USB serial to the Pico that drives the stepper, publishes the MEASURED angle
                              and the TF of the pan joint, so perception projects detections correctly
Pan range ±15° about z, + = LEFT (CCW, ROS z-up). A future "scan" mode will sweep the camera: reserve the mode, don't build it.

HARDWARE (already built and tested on the bench)
- Raspberry Pi Pico 2 W (RP2350, arduino-pico core), USB to the Jetson (native USB CDC, does NOT reset when the port opens)
- ULN2003 board on GP2..GP5 + 28BYJ-48 geared stepper (half-step, ~11.32 steps/deg, ~1-3° gearbox backlash, ~60°/s max)
- No home switch yet, no encoder yet. Zero = camera centred by hand at power-up.
- Firmware: firmware/camera_pan/camera_pan.ino (Andreas will have copied it into the repo - read it, it is the source of truth
  for the protocol below; do not change the protocol without telling me)

SERIAL PROTOCOL (ASCII, one line per message, "\n")
  Jetson -> Pico:  T <deg>   target angle, clamped to ±15 by the firmware. Ignored until homed.
                   Z         current position = 0, mark homed
                   H         homing (only with a home switch, not now)
                   E <0|1>   motor off/on (off = position lost, homed cleared)
                   W <0|1>   firmware watchdog off/on (default ON: no T for 200 ms -> returns to 0)
                   ?         print config ("I ..." lines)
  Pico -> Jetson:  S <pico_millis> <position_deg> <target_deg> <flags>   at 50 Hz
                   flags: 1=homed 2=moving 4=fault 8=watchdog_tripped 16=motor_enabled 32=coils_on
                   I <text>  info,   E <text>  error
  position_deg is the STEP COUNTER, not an encoder: true angle only if no steps were lost, and backlash is unmodelled.

READ FIRST, BEFORE WRITING ANYTHING (report findings)
- Branch `jazzy`, dirty working tree with changes that are not yours. Stage/commit ONLY files you create or edit. Don't push.
- Check whether the sim-side camera pan work already exists (grep camera_pan, pan joint in f1tenth_description,
  sim_camera_tf.launch.py, stack_params.yaml). If an aim-law module / controller node / TF publisher already exists,
  REUSE it - the hardware driver must plug into the same topics, not duplicate them.
- How the car publishes the camera TF today: camera.launch.py static base_link -> zed2_camera_link (0.12, 0, 0.15),
  ZED wrapper's robot_state_publisher below zed2_camera_link. Confirm nothing else publishes zed2_camera_link.
- Every perception TF lookup involving camera frames must use the IMAGE HEADER STAMP, never rclpy.time.Time() (latest).
  With a moving joint, latest-available is silently wrong. List each lookup and fix the ones that use latest.
- Where wheelbase, steering angle (applied / measured) and local EKF odometry (/odometry/filtered) come from.
- Conventions in CLAUDE.md: `test` extra in setup.py, no "--" in package.xml comments, taskset CPU pinning in launch,
  supervisor_bringup.launch.py is the canonical launch, Fast-DDS discovery server profile.

NODE 1 - camera_pan_controller (f1tenth_perception unless the sim work already put it elsewhere)
- Aim law as a pure, unit-tested module:
  - curvature κ: measured κ = ω_z / v from the local EKF when |v| > v_min, else κ = tan(δ)/L from the steering angle;
    blend smoothly between them, no hard switch
  - look-ahead arc length s = clamp(v * t_lookahead, s_min, s_max)
  - point on the arc in base_link (rear axle, bicycle model): P = (sin(κs)/κ, (1 - cos(κs))/κ), P -> (s, 0) as κ -> 0
    (series form near 0, no divide-by-zero)
  - pan = atan2(P_y - pivot_y, P_x - pivot_x), clamp ±15°, then low-pass + rate limit + small deadband
  - reversing or stopped: return to 0 (param)
- param `mode`: track_heading | fixed (hold fixed_angle_deg, for testing) | scan (reserved: log "not implemented", hold 0)
- publishes std_msgs/Float64 /camera_pan/command in RADIANS at 50 Hz
- all params in stack_params.yaml with descriptions that match the defaults

NODE 2 - camera_pan_driver (f1tenth_hardware)
- pyserial, port param default /dev/camera_pan. Add a udev rule (Pico VID 2e8a) creating that symlink, with install
  instructions; check how existing udev rules (VESC, lidar) are handled in the repo and follow the same pattern.
- On connect: send `?`, read the banner/state. If the S flags say homed -> keep the zero. If NOT homed and param
  zero_on_connect=true -> send Z and log a loud WARN ("assuming camera centred by hand"). Never re-zero a homed board.
- Send `T <deg>` from the latest /camera_pan/command at 50 Hz (this also feeds the firmware watchdog). If no command
  has arrived for > 0.5 s, keep sending T 0. Convert rad -> deg here and nowhere else.
- Parse S lines robustly (bad/partial lines counted and dropped, never crash). Reconnect loop if the port disappears.
- Time: map pico_millis to ROS time with an offset estimated as min(ros_receive_time - pico_ms) over a sliding window
  (min filter rejects USB latency jitter); handle the millis wrap. Stamp everything with the mapped time.
- Publishes:
  - sensor_msgs/JointState /camera_pan/joint_state  (name camera_pan_joint, position rad, MEASURED = position_deg)
  - TF: base_link -> camera_pan_base (static, the pivot; params pan_pivot_xyz) and
        camera_pan_base -> zed2_camera_link (dynamic, yaw = measured angle, stamped with the mapped Pico time, every S line)
  - diagnostic_msgs on /diagnostics: connected, homed, fault, watchdog_tripped, stale-data age, parse error count
- FAIL SAFE: if not connected, not homed, or fault -> STOP publishing the dynamic transform (do not publish 0).
  Stamp-exact lookups then fail and perception drops detections instead of placing them in the wrong place. Log ERROR.
- camera.launch.py: behind param camera_pan_enabled (default FALSE so today's car is unchanged) replace the static
  base_link -> zed2_camera_link with the driver's TF. With pan_pivot_xyz + zero angle the resulting base_link ->
  zed2_camera_link must equal today's (0.12, 0, 0.15) exactly - test this. Andreas will measure the real pivot later.

TESTS
- Aim law: straight -> 0, left turn -> positive, right -> negative, clamp, κ -> 0 continuity, reverse -> 0.
- Driver: protocol parser (good, malformed, partial lines), rad/deg conversion and sign, time-offset estimator incl.
  millis wrap, fail-safe (no TF when not homed/fault).
- A fake-Pico simulator (python, over a pty) that implements the protocol incl. watchdog and 60°/s motion; an
  integration test runs the driver against it and checks joint_state + TF follow the command.
- TF identity test at angle 0 vs today's static transform.

BENCH PROCEDURE (write it into the package README, I will run it on linus with the real Pico)
1. udev rule installed, /dev/camera_pan exists
2. camera_pan_driver alone, camera centred by hand -> zeroes, ros2 topic echo /camera_pan/joint_state
3. ros2 topic pub /camera_pan/command std_msgs/Float64 "data: 0.2" -> camera turns LEFT ~11.5°, joint_state follows
4. kill the publisher -> camera returns to 0 within ~0.5 s
5. unplug USB -> ERROR, TF stops; replug -> reconnects without re-zeroing
6. controller in fixed mode, then track_heading while pushing the car by hand / with wheels off the ground

DELIVERABLE
What you changed, the TF-lookup audit, test output, open questions (pivot position, encoder, home switch).
Commit only your files on `jazzy` with a clear message. Don't push.
