# mpc_corr's speed law (mpc_controller/speed_channel.py), on every /drive
# publish, on /mpc/speed_channel.
#
# The model: c the command in force, v the real speed,
#   c+ = c + ts * a,   v+ = v + alpha * (c + d - v),   alpha = 1 - exp(-ts / tau)
# d is in m/s OF COMMAND: the car behaves as if it were sent c + d (a car
# returning 76 % of 0.5 m/s has d = -0.12). In mode "legacy" the estimator
# still runs and d is reported, but nothing acts on it.
std_msgs/Header header

string mode                    # "offset_free" or "legacy" (speed_command_mode)

float32 v_meas                 # measured forward speed this tick [m/s] (the solver's x0 v)
float32 v_hat                  # observer speed estimate [m/s]
float32 d_hat                  # disturbance estimate [m/s of command]
float32 innovation             # v_meas minus the observer's one-step prediction [m/s]

float32 v_cmd                  # the command published this tick [m/s]
float32 v_cmd_in_force         # the command the solve started from (x0's c, after a start-up lift) [m/s]
float32 v_cmd_applied          # the mux output /ackermann_drive, last value received (NaN: none yet) [m/s]
bool override                  # the mux output differs from v_cmd: another lane is driving
float32 v_cmd_ceiling          # the forward command limit (max_forward_speed_mps) [m/s]

bool frozen                    # d was NOT integrated this tick
string freeze_reasons          # "+"-joined: hold, estop, mission_inactive, stopping, reverse,
                               # odom_stale, override, low_command, low_speed, stall,
                               # breakaway, anti_windup
bool stalled                   # command >= stall threshold with the car not moving
float32 stall_cap              # command cap while stalled [m/s] (NaN: none)
bool breakaway_active          # the start-up lower bound (v_breakaway_mps) is on this solve
float32 speed_slack            # how far over the soft real-speed cap the plan went [m/s]

string last_reset              # goal_change, hold, hold_release, estop, estop_release, reverse_switch
uint32 reset_count

# The constants the above were computed with.
float32 tau_s                  # speed_lag_tau_s
float32 alpha                  # 1 - exp(-control period / tau)
float32 gain_l_v               # observer gain on v
float32 gain_l_d               # observer gain on d
