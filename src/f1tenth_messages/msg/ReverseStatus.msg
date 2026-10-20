# mpc_corr's reverse manoeuvre (mpc_controller/reverse_manoeuvre.py), every
# control tick on /mpc/reverse_status.
#
# mode: "forward" (the forward MPC drives), "stopping" (gear change: zero
# speed until |v| < 0.05 m/s for gear_change_dwell_s, either direction),
# "reverse" (the rear solver drives along the frozen rear corridor) or
# "stuck" (reverse_max_attempts used up without forward progress: held
# stopped until the next move or mission).
std_msgs/Header header

string mode
bool reverse_active            # mode is "stopping" toward reverse, or "reverse"

# The stuck detector (any of its conditions true on every tick of the window).
bool stuck
string stuck_reason            # "forward_infeasible", "blocked_front", "no_progress", "+"-joined; "" if none
float32 stuck_timer_s
bool intentional_stop          # a stop the mission asked for: the timer is held at 0

# The rear side.
bool rear_ready                # rear solver feasible on the latest tick AND pre-checks OK
string rear_reason             # why not: "rear_unknown", "rear_occupied", "rear_object", ...
bool rear_solver_running       # lazy mode: only while the stuck timer runs
float32 rear_solve_ms
float32 rear_clearance_m       # free distance behind the rear bumper along the rear corridor

# The behaviour tree's permission (/bt/allow_reverse).
bool bt_allow_reverse
bool bt_allow_fresh

uint8 attempts                 # consecutive reverses without forward progress
float64 last_switch_time_s     # node clock, 0 = never
string last_termination_reason
float32 requested_m
float32 travelled_m
