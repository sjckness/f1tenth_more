"""Topic-liveness watchdog: the pure logic (no rclpy).

Used by component_supervisor_node. Fix batch 5, backlog H1.

Why: a component can be alive as a process and still be cut off from the
graph -- a subscription that never matched its publisher after a bringup
(discovery isolation, H1), a lifecycle transition that never arrived, a node
frozen by the kernel. The process watchdog cannot see any of that. This one
looks at what each component is FOR: its output topics, received by the
supervisor, measured in wall time.

Configuration (components.yaml, top-level `health:` and `health_topic_types:`,
next to `components:` so a component cannot be added without stating its
health -- test_component_health_config.py enforces it):

  health_topic_types:            # every topic named below -> its message type
    /scan: sensor_msgs/msg/LaserScan
  health:
    swept_clearance:
      action: restart            # restart | alert   (default restart)
      grace_sec: 15              # after (re)start, before checks apply
      fail_for_sec: 3            # continuous failure before acting
      enabled_if: {use_lidar: true}   # stack_params.yaml values; else not watched
      checks:
        - topic: /perception/swept_clearance/lidar
          max_age_sec: 0.5
          launch_file: swept_clearance.launch.py   # who is restarted (optional
                                                   # when the component has one)
          when_fresh: {/scan: 0.5}  # only judged while these inputs are fresh
          min_rate_hz: 30           # receipts per second over the last
          rate_window_sec: 2.0      # rate_window_sec must reach this (node
                                    # clock Hz: scaled by the real-time
                                    # factor in sim). Default: no rate check.
          once: false               # instead of an age: at least one message
                                    # since the launch file (re)started
                                    # (max_age_sec is then ignored; .inf)
          settle_sec: 15            # once checks only (default
                                    # ONCE_SETTLE_SEC): no verdict until the
                                    # when_fresh inputs have been fresh this
                                    # long (an input that appears late)
          expect: {level: [0]}      # field -> allowed values of the LAST
                                    # message (default: none)
  unwatched:
    dev_tools: why it has no checks

Verdict per check, every evaluation:
  skip   the owning launch file is not running, or in its grace period, or a
         when_fresh input is stale (an upstream outage is not this
         component's fault -- this is what separates isolation from it);
  grace  a `once` check whose inputs have been fresh for less than
         settle_sec: the node has not had the time to answer yet;
  fail   the topic is older than max_age_sec (never received = infinitely
         old) -- or, for a `once` check, nothing arrived since the launch
         file started -- or its last message does not match `expect`, or
         it arrives slower than min_rate_hz;
  ok     otherwise.

A launch file whose checks have failed continuously for fail_for_sec is acted
on: action 'alert' logs and reports it; action 'restart' asks the supervisor
to restart that launch file, through the supervisor's existing restart path
and restart budget. A component restarted max_consecutive_restarts times by
the watchdog without being healthy (every check ok) for healthy_for_sec in
between is given up on: FAILED, no more restarts, until a manual START or
RESTART. That limit exists because the existing budget (3 restarts within
60 s) cannot stop a watchdog loop on its own: one watchdog cycle (grace +
fail_for + the restart itself) takes longer than 20 s, so a component that
fails after every restart would never use up 3 restarts inside 60 s.

Paused simulation (sim mode only): the supervisor passes paused=True while
/clock is not advancing. Nothing is judged then, failure timers are cleared,
and once the clock moves again every component gets resume_grace_sec before
it is judged -- nodes on use_sim_time stop publishing when the clock stops,
and their inputs need a moment to flow again after it restarts. On the car
there is no /clock and the supervisor never passes paused=True.
"""

import collections
import math

ACTIONS = ('restart', 'alert')

# DiagnosticStatus levels, used for the per-component health status.
OK, WARN, ERROR, STALE = 0, 1, 2, 3

DEFAULTS = {
    'action': 'restart',
    'grace_sec': 20.0,
    'fail_for_sec': 3.0,
    'healthy_for_sec': 10.0,
    'max_consecutive_restarts': 3,
}

# A `once` check's default settle_sec. An age check gets its max_age_sec after
# its inputs return (check_verdict); a once check has no age, so without this
# it failed the moment its input first appeared after the grace period -- live,
# a /scan fed 60 s after bringup restarted a healthy slam 6 s later (fix batch
# 5 report, B6). slam_toolbox's first map comes one map_update_interval (5 s
# of ITS clock) after its first processed scan: up to 5 s / RTF in wall time
# in sim (worst measured RTF 0.74: 6.8 s), plus the grid rebuild. Measured
# from the first /scan to the first /slam/map: see the report's B6 table.
ONCE_SETTLE_SEC = 15.0


class HealthConfigError(ValueError):
    pass


class Check:
    def __init__(self, topic, max_age_sec, launch_file, when_fresh=None,
                 expect=None, once=False, min_rate_hz=None, rate_window_sec=2.0,
                 settle_sec=None):
        self.topic = topic
        self.once = bool(once)
        self.settle_sec = float(ONCE_SETTLE_SEC if settle_sec is None else settle_sec)
        self.min_rate_hz = None if min_rate_hz is None else float(min_rate_hz)
        self.rate_window_sec = float(rate_window_sec)
        self.max_age_sec = float(max_age_sec)
        self.launch_file = launch_file
        self.when_fresh = {t: float(a) for t, a in (when_fresh or {}).items()}
        self.expect = {k: list(v) if isinstance(v, (list, tuple)) else [v]
                       for k, v in (expect or {}).items()}

    def label(self):
        return f'{self.topic}' + (f' {self.expect}' if self.expect else '')


class ComponentHealthConfig:
    def __init__(self, name, checks, action, grace_sec, fail_for_sec,
                 healthy_for_sec, max_consecutive_restarts, enabled_if=None):
        self.name = name
        self.checks = checks
        self.action = action
        self.grace_sec = float(grace_sec)
        self.fail_for_sec = float(fail_for_sec)
        self.healthy_for_sec = float(healthy_for_sec)
        self.max_consecutive_restarts = int(max_consecutive_restarts)
        self.enabled_if = dict(enabled_if or {})


def parse_health_config(doc, registry):
    """Parse components.yaml's health section.

    Returns ({component: ComponentHealthConfig}, {topic: type},
    {component: reason}) from the loaded document. Raises HealthConfigError on
    anything inconsistent, so a broken health section fails at startup, not
    silently.

    registry: {component: [{'launch_file': ...}, ...]} as components.yaml
    declares it (before sim mode or feature flags filter launch files out).
    """
    types = dict(doc.get('health_topic_types') or {})
    unwatched = dict(doc.get('unwatched') or {})
    health = {}
    for name, raw in (doc.get('health') or {}).items():
        if name not in registry:
            raise HealthConfigError(f"health: '{name}' is not a component")
        if name in unwatched:
            raise HealthConfigError(f"'{name}' is both in health and unwatched")
        opts = dict(DEFAULTS)
        opts.update({k: v for k, v in raw.items() if k in DEFAULTS})
        unknown = set(raw) - set(DEFAULTS) - {'checks', 'enabled_if'}
        if unknown:
            raise HealthConfigError(f'health.{name}: unknown keys {sorted(unknown)}')
        if opts['action'] not in ACTIONS:
            raise HealthConfigError(f'health.{name}.action must be one of {ACTIONS}')
        launch_files = [e['launch_file'] for e in registry[name]]
        checks = []
        for i, c in enumerate(raw.get('checks') or []):
            where = f'health.{name}.checks[{i}]'
            lf = c.get('launch_file')
            if lf is None:
                if len(launch_files) != 1:
                    raise HealthConfigError(
                        f'{where}: launch_file is required ({name} runs {launch_files})')
                lf = launch_files[0]
            elif lf not in launch_files:
                raise HealthConfigError(f"{where}: {lf} is not one of {name}'s {launch_files}")
            unknown = set(c) - {'topic', 'max_age_sec', 'launch_file', 'when_fresh', 'expect',
                                'once', 'min_rate_hz', 'rate_window_sec', 'settle_sec'}
            if unknown:
                raise HealthConfigError(f'{where}: unknown keys {sorted(unknown)}')
            check = Check(c['topic'], c.get('max_age_sec', math.inf), lf, c.get('when_fresh'),
                          c.get('expect'), c.get('once', False), c.get('min_rate_hz'),
                          c.get('rate_window_sec', 2.0), c.get('settle_sec'))
            if not check.once and math.isinf(check.max_age_sec):
                raise HealthConfigError(f'{where}: max_age_sec is required unless once: true')
            if 'settle_sec' in c and not check.once:
                raise HealthConfigError(f'{where}: settle_sec is only for once checks '
                                        '(an age check settles for its max_age_sec)')
            if check.settle_sec < 0:
                raise HealthConfigError(f'{where}: settle_sec must be >= 0')
            for t in [check.topic, *check.when_fresh]:
                if t not in types:
                    raise HealthConfigError(f'{where}: {t} has no entry in health_topic_types')
            checks.append(check)
        if not checks:
            raise HealthConfigError(f'health.{name}: no checks (use unwatched: instead)')
        health[name] = ComponentHealthConfig(
            name, checks, opts['action'], opts['grace_sec'], opts['fail_for_sec'],
            opts['healthy_for_sec'], opts['max_consecutive_restarts'],
            raw.get('enabled_if'))
    for name in unwatched:
        if name not in registry:
            raise HealthConfigError(f"unwatched: '{name}' is not a component")
    return health, types, unwatched


def topic_states(health):
    """{topic: TopicState} for every topic the checks of `health` read."""
    thresholds, history = {}, {}
    for cfg in health.values():
        for c in cfg.checks:
            thresholds.setdefault(c.topic, set())
            if c.min_rate_hz is not None:
                history[c.topic] = max(history.get(c.topic, 0.0), c.rate_window_sec)
            for t, max_age in c.when_fresh.items():
                thresholds.setdefault(t, set()).add(max_age)
    return {t: TopicState(th, history.get(t, 0.0)) for t, th in thresholds.items()}


def needs_message(topic, health):
    """Tell whether some check reads the message itself (`expect`).

    Otherwise the supervisor only needs the receipt time and can subscribe raw
    (no deserialization).
    """
    return any(c.topic == topic and c.expect
               for cfg in health.values() for c in cfg.checks)


class TopicState:
    """What the supervisor has received on one topic.

    Times are monotonic seconds (wall time, not ROS time).
    """

    def __init__(self, fresh_thresholds=(), history_sec=0.0):
        self.last_rx = None
        self.last_msg = None
        # Receipt times of the last history_sec, for rate checks only.
        self.history_sec = float(history_sec)
        self.rx_times = collections.deque()
        self.count = 0
        # max_age -> start of the current run of receipts no more than max_age
        # apart, for every max_age a when_fresh uses on this topic.
        self._fresh_since = {float(a): None for a in fresh_thresholds}

    def on_message(self, now, msg=None):
        self.count += 1
        for max_age, since in self._fresh_since.items():
            if since is None or now - self.last_rx > max_age:
                self._fresh_since[max_age] = now
        self.last_rx = now
        if msg is not None:
            self.last_msg = msg
        if self.history_sec > 0:
            self.rx_times.append(now)
            while self.rx_times[0] < now - self.history_sec:
                self.rx_times.popleft()

    def age(self, now):
        return math.inf if self.last_rx is None else now - self.last_rx

    def rate(self, now, window):
        """Receipts per second over the last `window` seconds."""
        return sum(1 for t in self.rx_times if t > now - window) / window

    def fresh_since(self, max_age):
        """Start of the current run of receipts no more than max_age apart.

        None if nothing was ever received. A check gated on this input judges
        its output only from here on: after an upstream outage the output gets
        its full max age to reappear. max_age must be one of the thresholds the
        state was built with (topic_states() does that from the config).
        """
        return self._fresh_since[float(max_age)]


def _field(msg, path):
    v = msg
    for part in path.split('.'):
        v = getattr(v, part)
    if isinstance(v, (bytes, bytearray)) and len(v) == 1:
        v = v[0]  # a `byte` field (DiagnosticStatus.level) arrives as bytes
    return v


def check_verdict(check, topics, now, started_at=None, rate_scale=1.0):
    """Judge one check against the received topics.

    Returns ('ok' | 'fail' | 'skip' | 'grace', reason); 'grace' only from a
    `once` check still settling. The component's grace and running state are
    the monitor's business, not this function's; started_at (the owning launch
    file's start, monotonic) is read by `once` and rate checks. rate_scale
    multiplies min_rate_hz: the real-time factor in sim, 1 on the car.
    """
    inputs_fresh_since = None
    for t, max_age in check.when_fresh.items():
        age = topics[t].age(now)
        # Never received is stale even against an infinite max age (.inf =
        # "received at least once").
        if topics[t].last_rx is None or age > max_age:
            return 'skip', f'input {t} stale ({_fmt_age(age)})'
        since = topics[t].fresh_since(max_age)
        if inputs_fresh_since is None or since > inputs_fresh_since:
            inputs_fresh_since = since
    st = topics[check.topic]
    if check.once:
        if st.last_rx is None or (started_at is not None and st.last_rx < started_at):
            # The node needs a moment to answer an input that just appeared:
            # settle_sec from when its inputs became fresh. The node's own
            # startup is the grace period's business; an input that has been
            # there since before the grace ended has long settled.
            if (inputs_fresh_since is not None
                    and now - inputs_fresh_since < check.settle_sec):
                return 'grace', (f'{check.topic}: settling, inputs fresh for '
                                 f'{now - inputs_fresh_since:.0f} of {check.settle_sec:g} s')
            since = 'never' if st.last_rx is None else 'not since its launch file started'
            return 'fail', f'{check.topic}: no message {since} (must publish at least once)'
        return 'ok', ''
    age = st.age(now)
    if inputs_fresh_since is not None:
        # Silence from before the inputs came back is not the node's fault.
        age = min(age, now - inputs_fresh_since)
    inputs = ', '.join(f'{t} {_fmt_age(topics[t].age(now))}' for t in check.when_fresh)
    while_fresh = f' while its input is fresh ({inputs})' if inputs else ''
    if age > check.max_age_sec:
        return 'fail', (f'{check.topic}: no message for {_fmt_age(age)} '
                        f'(max {check.max_age_sec:g} s){while_fresh}')
    # The node itself needs a moment to see an input that just came back
    # (mpc_corr's level was ERROR for 0.5 s at feed start, live).
    settling = inputs_fresh_since is not None and now - inputs_fresh_since <= check.max_age_sec
    if check.min_rate_hz is not None:
        window = check.rate_window_sec
        # A full window of the node's output since its inputs came back and
        # since it started, or the rate is not yet meaningful.
        starts = [t for t in (inputs_fresh_since, started_at) if t is not None]
        if not starts or now - max(starts) >= window + check.max_age_sec:
            rate, need = st.rate(now, window), check.min_rate_hz * rate_scale
            if rate < need:
                scale = f' x real-time factor {rate_scale:.2f}' if rate_scale != 1.0 else ''
                return 'fail', (f'{check.topic}: {rate:.1f} Hz over {window:g} s, below '
                                f'{check.min_rate_hz:g} Hz{scale}{while_fresh}')
    if check.expect and settling:
        return 'ok', ''
    for path, allowed in check.expect.items():
        if st.last_msg is None:
            return 'fail', f'{check.topic}: no message to read {path} from'
        v = _field(st.last_msg, path)
        if v not in allowed:
            return 'fail', f'{check.topic}: {path} = {v!r}, expected one of {allowed}'
    return 'ok', ''


def _fmt_age(age):
    return 'never received' if math.isinf(age) else f'{age:.1f} s'


class ComponentMonitor:
    """State of one watched component.

    evaluate() is called every watchdog period and returns the launch files to
    restart now (empty for most calls); the caller performs the restart and
    reports back through restart_refused() if its restart budget said no.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.status = 'NOT_RUNNING'
        self.level = STALE
        self.message = ''
        self.failing_since = {}          # launch_file -> monotonic time
        self.consecutive_restarts = 0
        self.healthy_since = None
        self.gave_up = False
        self.alerted = False
        self.failures = 0                # failure episodes acted on (alert or restart)
        self.restarts = 0                # restarts the watchdog requested
        self.resume_until = None
        self.events = []                 # (level, text) for the caller to log

    # -- external events -----------------------------------------------------

    def reset(self):
        """Forget failures and any give-up: a manual START/RESTART happened."""
        self.failing_since.clear()
        self.consecutive_restarts = 0
        self.healthy_since = None
        self.gave_up = False
        self.alerted = False

    def resume(self, now, resume_grace_sec):
        self.failing_since.clear()
        self.healthy_since = None
        self.resume_until = now + resume_grace_sec

    def restart_refused(self, why):
        self.gave_up = True
        self.status, self.level = 'FAILED', ERROR
        self.message = why
        self.events.append((ERROR, f"'{self.cfg.name}' FAILED: {why}. Not restarting it "
                                   'again; ~/control_component START or RESTART resets this.'))

    # -- evaluation ----------------------------------------------------------

    def evaluate(self, now, topics, procs, paused=False, stalled=False, rate_scale=1.0):
        """Judge every check once; return the launch files to restart now.

        procs: {launch_file: (running: bool, started_at: monotonic or None)}.
        paused: sim clock not advancing. stalled: the supervisor itself did not
        run for a while (blocked in a restart), so receipt ages are not
        trustworthy this once. rate_scale: see check_verdict.
        """
        cfg = self.cfg
        if paused or stalled:
            self.failing_since.clear()
            self.healthy_since = None
            if paused:
                self.status, self.level, self.message = 'PAUSED', OK, 'sim clock paused'
            return []

        verdicts = []
        for c in cfg.checks:
            running, started_at = procs.get(c.launch_file, (False, None))
            if not running:
                verdicts.append((c, 'skip', f'{c.launch_file} not running'))
            elif started_at is not None and now - started_at < cfg.grace_sec:
                left = cfg.grace_sec - (now - started_at)
                verdicts.append((c, 'grace', f'grace {left:.0f} s left'))
            elif self.resume_until is not None and now < self.resume_until:
                verdicts.append((c, 'grace', 'sim clock resumed'))
            else:
                v, why = check_verdict(c, topics, now, started_at, rate_scale)
                verdicts.append((c, v, why))

        failing = {}
        for c, v, why in verdicts:
            if v == 'fail':
                failing.setdefault(c.launch_file, []).append(why)
        for lf in list(self.failing_since):
            if lf not in failing:
                del self.failing_since[lf]
        for lf in failing:
            self.failing_since.setdefault(lf, now)

        if all(v == 'ok' for _, v, _ in verdicts):
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since >= cfg.healthy_for_sec and self.consecutive_restarts:
                self.events.append((OK, f"'{cfg.name}' healthy again for "
                                        f'{cfg.healthy_for_sec:g} s after '
                                        f'{self.consecutive_restarts} watchdog restart(s)'))
                self.consecutive_restarts = 0
            if not failing:
                self.alerted = False
        else:
            self.healthy_since = None

        restart = []
        if self.gave_up:
            self.status, self.level = 'FAILED', ERROR
            if failing:
                self.message = '; '.join(w for ws in failing.values() for w in ws)
            return []

        due = [lf for lf, t0 in self.failing_since.items() if now - t0 >= cfg.fail_for_sec]
        if due:
            reasons = '; '.join(w for lf in due for w in failing[lf])
            if cfg.action == 'alert':
                if not self.alerted:
                    self.failures += 1
                    self.events.append((ERROR, f"'{cfg.name}' LIVENESS FAILURE (alert only, "
                                               f'not restarted): {reasons}'))
                    self.alerted = True
                self.status, self.level, self.message = 'ALERT', ERROR, reasons
                return []
            self.failures += 1
            if self.consecutive_restarts >= cfg.max_consecutive_restarts:
                self.restart_refused(
                    f'still failing after {self.consecutive_restarts} watchdog restarts '
                    f'without being healthy for {cfg.healthy_for_sec:g} s in between: {reasons}')
                return []
            self.consecutive_restarts += 1
            self.restarts += 1
            count = f'{self.consecutive_restarts}/{cfg.max_consecutive_restarts}'
            self.events.append((ERROR, f"'{cfg.name}' LIVENESS FAILURE: {reasons} -- restarting "
                                       f"{', '.join(due)} (watchdog restart {count})"))
            for lf in due:
                del self.failing_since[lf]
                restart.append(lf)
            self.healthy_since = None
            self.status, self.level, self.message = 'RESTARTING', ERROR, reasons
            return restart

        if failing:
            self.status, self.level = 'FAILING', WARN
            self.message = '; '.join(w for ws in failing.values() for w in ws)
        elif any(v == 'grace' for _, v, _ in verdicts):
            self.status, self.level = 'STARTING', OK
            self.message = next(w for _, v, w in verdicts if v == 'grace')
        elif all(v == 'skip' and 'not running' in w for _, v, w in verdicts):
            self.status, self.level, self.message = 'NOT_RUNNING', STALE, ''
        elif not any(v == 'ok' for _, v, _ in verdicts):
            # Nothing could be judged: every input is out.
            self.status, self.level = 'UPSTREAM_STALE', WARN
            self.message = '; '.join(sorted({w for _, v, w in verdicts if v == 'skip'}))
        else:
            # Everything judged passed. Checks on an absent input (no VESC in
            # sim: no /sensors/core) are noted, not a warning.
            self.status, self.level = 'OK', OK
            skipped = sorted({w for _, v, w in verdicts if v == 'skip'})
            self.message = f'not judged: {"; ".join(skipped)}' if skipped else ''
        return restart
