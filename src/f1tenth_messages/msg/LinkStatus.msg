# /comms/link_status: link_manager's view of the link from the car to the
# base (f1tenth_comms). A report only: nothing on the car stops, aborts or
# changes a mission on this message. The app's "no client for 2 s -> abort"
# lives in f1tenth_gateway and does not read it.
#
# One heartbeat ROUND is one UDP packet sent to every path with the same
# sequence number. A round is answered when any path replies within the
# timeout, lost when none does.

uint8 STATE_OK=0
uint8 STATE_DEGRADED=1
uint8 STATE_LOST=2

std_msgs/Header header

uint8 state
string state_name              # "OK", "DEGRADED" or "LOST"
string path                    # active path: "tailscale", "local", or "" when none answers
string address                 # host:port of the active path, "" when none answers

float32 rtt_ms                 # last round trip on the active path [ms]; NaN when none
float32 rtt_mean_ms            # mean round trip on the active path over window_s [ms]; NaN when none
float32 loss_pct               # rounds with no reply on any path over window_s [%]; NaN before the first is decided
float32 since_last_reply_s     # since the last reply on any path [s]; +inf before the first
float32 window_s

bool wifi_associated
float32 rssi_dbm               # Wi-Fi signal [dBm]; NaN when not associated or unreadable
string wifi_interface

# Cumulative since the node started. rounds_sent - rounds_answered -
# rounds_lost are still in flight.
uint32 rounds_sent
uint32 rounds_answered
uint32 rounds_lost

# One entry per configured path, in priority order (tailscale first).
string[] path_names
string[] path_addresses
bool[] path_up
float32[] path_rtt_ms          # mean over window_s; NaN when none
float32[] path_loss_pct        # over window_s; NaN before the first is decided

# Reconnect attempts (nmcli connection up <hotspot>), only while LOST.
uint32 reconnect_attempts
bool reconnect_in_flight
string last_reconnect_trigger  # "time", "distance", "new_cell", or ""
string last_reconnect_result   # "ok", "dry-run: ...", "skipped: ...", "failed: ...", or ""
float32 since_last_reconnect_s # since the last attempt started [s]; +inf before the first
