# Published continuously by f1tenth_diagnostics' diagnostics_server_node on
# /diagnostics/battery_status, independent of enable_sys_obs -- battery safety
# monitoring is never gateable behind the sys_obs observability toggle (see that
# node's own docstring). Consumed by f1tenth_behavior's IsBatteryLow BT condition
# (the emergency lane) and by RunDiagnostics' response.

std_msgs/Header header

# False until at least one VescStateStamped sample has been received on
# diagnostics_server_node's state_topic -- voltage/ok are both meaningless (0.0/false)
# while this is false. Prevents a cold-start BT tick (or an early run_diagnostics call)
# from misreading "no data yet" as "battery critically low."
bool has_data

float64 voltage       # mean of samples received since the last publish tick, volts
float64 min_voltage   # threshold voltage was compared against (min_battery_voltage)
bool ok                # has_data && voltage >= min_voltage
