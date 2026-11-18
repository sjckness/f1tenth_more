# Single 2D obstacle, ground-plane disk approximation used by mpc_controller's
# MPC_corr.py. x/y are in the frame carried by Obstacle2DArray's own header.

float32 x
float32 y
float32 r

# --- motion of the track this disk belongs to (mpc-dodge, 2026-11-18) ---------
# Filled by semantic_layer_node (/costmap/semantic_obstacles) when its
# semantic_track_velocity_enabled is true; obstacle_projector_node and older
# publishers leave the defaults (id -1, v_valid false). An elongated unknown
# object is several disks with the same id. The velocity is ground-fixed, in the
# LOCAL odom frame (ego motion compensated: f1tenth_costmap/track_velocity.py),
# so a consumer working in odom (mpc_corr) uses it as is.
int32 id -1                 # track id (the same as SemanticObject.id), -1 = none
float32 vx                  # [m/s] odom
float32 vy                  # [m/s] odom
float32 speed               # [m/s] |v|
float32 heading             # [rad] odom direction of v
float32 v_sigma             # [m/s] sqrt of the largest eigenvalue of the velocity covariance
float32 vx_car              # [m/s] the same ground velocity in the car's axes (debug / Foxglove only)
float32 vy_car              # [m/s]
bool v_valid                # track old enough, |v| above the floor, covariance below the ceiling
