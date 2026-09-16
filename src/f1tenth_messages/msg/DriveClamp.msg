# mpc_corr clamped a /drive speed command, on /mpc/drive_clamp.
#
# Published only on the ticks where the clamp in MPC_corr._publish_drive
# changed the command (drive_limits.clamp_drive_speed), so its message count
# is the clamp's event count. See max_forward_speed_mps / max_reverse_speed_mps
# in stack_params.yaml.
std_msgs/Header header       # stamp: when the command was published
float32 requested_speed      # what the caller asked for [m/s]; NaN if non-finite
float32 applied_speed        # what went out on /drive [m/s]
float32 max_forward_speed    # the limits in force [m/s], both magnitudes
float32 max_reverse_speed
