"""
f1tenth_ctl, used in-process.

Every f1tenth_ctl command is cli.cmd_<name>(settings, args) and returns the
same dict the CLI prints as JSON, so the gateway calls them directly: no
Python start-up per call, one code path with the Telegram bot. They block
(a stop takes up to 15 s, estop-reset up to a minute), so the app runs them
in a thread pool. f1tenth_ctl's own ROS work (the f1t-monitor unit, the
rosprobe subprocesses) stays as it is.

The mock (mock.py) implements the same methods.
"""

from argparse import Namespace
import threading
import time
from typing import Dict, Optional


class CtlPort:
    def __init__(self, env_file: Optional[str] = None):
        from f1tenth_ctl import cli, config, missions, rosprobe
        self._cli, self._missions, self._rosprobe = cli, missions, rosprobe
        self.settings = config.load(env_file)
        # One operation per managed process at a time (start/stop), and one
        # plan at a time across this gateway.
        self.proc_locks = {name: threading.Lock() for name in cli.MANAGED}
        self.plan_lock = threading.Lock()

    # ---- status and processes ---------------------------------------------------------

    def status(self) -> Dict:
        return self._cli.cmd_status(self.settings, Namespace(), time.monotonic())

    def start(self, name: str) -> Dict:
        return self._cli.cmd_start(self.settings, Namespace(target=name))

    def stop(self, name: str) -> Dict:
        return self._cli.cmd_stop(self.settings, Namespace(target=name))

    def estop(self) -> Dict:
        """f1tenth_ctl estop as the CLI runs it: the gateway's fallback if its own fails."""
        return self._cli.cmd_estop(self.settings, Namespace())

    def estop_reset(self) -> Dict:
        return self._cli.cmd_estop_reset(self.settings, Namespace(force=False))

    def estop_spec(self) -> Dict:
        """Return what f1tenth_ctl estop would use: same topic, services, timeouts."""
        s = self.settings
        return {'topic': s.estop_topic, 'service': s.estop_service,
                'service_timeout_s': s.estop_service_timeout_s,
                'abort_service': s.abort_service, 'abort_timeout_s': s.estop_abort_timeout_s,
                'publish_s': s.estop_publish_s, 'publish_hz': s.estop_publish_hz}

    # ---- planner and missions ------------------------------------------------------------

    def planner_log(self, since: Optional[int] = None) -> Dict:
        return self._cli.cmd_planner_log(self.settings,
                                         Namespace(since=since, max_bytes=65536))

    def send_planner(self, text: str) -> Dict:
        return self._cli.cmd_send(self.settings, Namespace(target='planner', text=[text]))

    def mission_load(self, mid: str) -> Dict:
        return self._cli.cmd_mission_load(self.settings, Namespace(id=mid, allow_old=False))

    def mission_start(self, mid: str) -> Dict:
        return self._cli.cmd_mission_start(self.settings, Namespace(id=mid, allow_old=False),
                                           time.monotonic())

    def mission_record(self, mid: str) -> Optional[Dict]:
        return self._missions.get(self.settings, mid)

    def mission_path(self, mid: str) -> str:
        return self._missions.mission_path(self.settings, mid)

    def summarize(self, mission: Dict) -> Dict:
        return self._missions.summarize(mission)

    def validate(self, path: str) -> Dict:
        """f1tenth_behavior's own loader on the file (the stack's validator)."""
        return self._rosprobe.validate_mission_file(path)

    def describe_state(self, ros_state, json_path, wall_now: float) -> Dict:
        return self._missions.describe_state(self.settings, ros_state, json_path, now=wall_now,
                                             countdown_s=self.settings.mission_countdown_s)

    @property
    def plan_max_age_s(self) -> float:
        return float(self.settings.mission_max_age_s)
