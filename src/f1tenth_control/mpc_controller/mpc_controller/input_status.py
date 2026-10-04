"""mpc_corr's input-freshness heartbeat, /mpc/input_status (fix batch 5, H1).

Why it exists: without a goal, everything mpc_corr publishes is published
whether or not it receives odometry -- /drive every tick (a hold when the
odometry is fresh, a hard zero when it is not), /mpc/solver_status and
/mpc/status only after a solve. Fix batch 3's isolated mpc_corr, whose
odometry subscription never matched, looked exactly like a healthy idle one.
The supervisor's topic-liveness watchdog (component_supervisor_node,
components.yaml health: navigation) reads this status instead:

  level OK     the active odometry source is fresher than
               odom_stale_timeout_sec (the same test _update_active_odom()
               uses to pick it);
  level ERROR  neither source is: mpc_corr has no state and holds the car.

Published from the control loop itself, at most every period_sec, so its
arrival also proves the loop runs. Ages are in the node's own clock (sim time
under use_sim_time), the same clock the staleness test uses.
"""

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

STATUS_NAME = 'mpc_corr: odometry input'


def _age(value):
    return 'never' if math.isinf(value) else f'{value:.3f}'


def odom_input_status(source, hw_age, sim_age, timeout):
    """Build the DiagnosticStatus for one tick.

    source: 'hardware', 'sim' or None (what _update_active_odom() chose).
    hw_age, sim_age: seconds since the last message on each source (inf if
    none yet).
    """
    st = DiagnosticStatus()
    st.name = STATUS_NAME
    st.hardware_id = 'mpc_corr'
    if source is None:
        st.level = DiagnosticStatus.ERROR
        st.message = f'odometry stale (no source fresher than {timeout:g} s)'
    else:
        st.level = DiagnosticStatus.OK
        st.message = f'odometry fresh ({source})'
    st.values = [
        KeyValue(key='source', value=source or 'none'),
        KeyValue(key='hw_odom_age_s', value=_age(hw_age)),
        KeyValue(key='sim_odom_age_s', value=_age(sim_age)),
        KeyValue(key='odom_stale_timeout_s', value=f'{timeout:g}'),
    ]
    return st
