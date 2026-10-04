"""/mpc/input_status: mpc_corr's odometry-freshness heartbeat (fix batch 5, H1).

The supervisor's topic-liveness watchdog restarts 'navigation' when this
status says ERROR while /odometry/filtered is fresh at the supervisor -- the
signature of fix batch 3's isolated mpc_corr. Pinned here:

1. the level follows the same source choice _update_active_odom() makes;
2. control_loop's publish helper rate-limits to input_status_period_sec, and
   publishes again after the clock jumps backwards (a looped bag in sim).

Same duck-typed stand-in shape as test_campaign_status.py: the method under
test is MPCController's own, bound to an object carrying only what it reads.

Run standalone: python3 -m pytest test/test_input_status.py -v
"""
import math

from diagnostic_msgs.msg import DiagnosticStatus
from mpc_controller.input_status import odom_input_status
from mpc_controller.MPC_corr import MPCController


def _values(st):
    return {kv.key: kv.value for kv in st.values}


def test_fresh_source_is_ok():
    st = odom_input_status('hardware', 0.02, math.inf, 0.5)
    assert st.level == DiagnosticStatus.OK
    assert _values(st) == {'source': 'hardware', 'hw_odom_age_s': '0.020',
                           'sim_odom_age_s': 'never', 'odom_stale_timeout_s': '0.5'}


def test_no_source_is_error():
    st = odom_input_status(None, 0.7, math.inf, 0.5)
    assert st.level == DiagnosticStatus.ERROR
    assert _values(st)['source'] == 'none'
    assert 'stale' in st.message


def test_never_received_is_error():
    st = odom_input_status(None, math.inf, math.inf, 0.5)
    assert st.level == DiagnosticStatus.ERROR
    assert _values(st)['hw_odom_age_s'] == 'never'


class _Publisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeController:
    _publish_input_status = MPCController._publish_input_status

    def __init__(self):
        self.input_status_pub = _Publisher()
        self.input_status_period_sec = 0.5
        self._input_status_last_sec = None
        self._odom_ages = (0.01, math.inf)
        self.active_odom_source = 'hardware'
        self.odom_stale_timeout_sec = 0.5


def test_publishes_at_most_every_period():
    c = _FakeController()
    for i in range(30):                 # 3 s of 10 Hz ticks
        c._publish_input_status(100.0 + 0.1 * i)
    assert len(c.input_status_pub.msgs) == 6


def test_publishes_again_after_the_clock_jumps_back():
    c = _FakeController()
    c._publish_input_status(100.0)
    c._publish_input_status(5.0)        # bag looped: sim time restarted
    assert len(c.input_status_pub.msgs) == 2


def test_reports_what_the_tick_chose():
    c = _FakeController()
    c.active_odom_source = None
    c._odom_ages = (0.9, math.inf)
    c._publish_input_status(1.0)
    assert c.input_status_pub.msgs[-1].level == DiagnosticStatus.ERROR
