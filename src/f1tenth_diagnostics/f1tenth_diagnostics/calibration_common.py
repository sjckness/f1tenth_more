"""Shared plumbing for the stationary calibration nodes (gyro_bias_
calibration_node, sensor_covariance_calibration_node's 'stationary' mode,
slam_pose_covariance_calibration_node) -- introduced when the first two were
wired into automatic, boot-time calibration (f1tenth_hardware/launch/
vesc.launch.py) alongside the existing manual calibration.launch.py entry
point. Pulled out to a shared module instead of duplicated in each node file,
since all now need the exact same things: a distinguishable exit-code scheme,
a backup-then-patch YAML writer, source-tree path resolution, the Welford
online variance accumulator, and the pre-sampling "confirm the car is
actually stationary" gate.

light_motion mode (sensor_covariance_calibration_node's other calibration_mode)
does not use the StationaryGate here -- it inherently drives the car on
purpose, so a stationary check would be actively wrong for it. It does reuse
write_vesc_yaml() (already did, before this module existed).

write_vesc_yaml()/resolve_source_vesc_yaml_path() are now thin wrappers
around the more general write_yaml_config()/resolve_source_config_path()
(added for slam_pose_covariance_calibration_node.py, which writes into a
DIFFERENT file -- f1tenth_bringup/config/ekf_global.yaml, not vesc.yaml, and
a single fixed section rather than vesc.yaml's own per-key section routing)
-- both original functions keep their exact original signature/behavior
unchanged, so gyro_bias_calibration_node.py/sensor_covariance_calibration_
node.py needed no changes at all for this.
"""

import difflib
import io
import os
import re
import shutil
import time
from datetime import datetime
from pathlib import Path

try:
    from ruamel.yaml import YAML
except ImportError:
    YAML = None

# ---------------------------------------------------------------------------
# Exit codes -- distinguishable failure reasons, so callers (today: vesc.launch.py's
# calibration_exit_handler; potentially any future automation) can log WHY a
# calibration run failed instead of only knowing that it did. Every calibration
# node's main() ends in sys.exit() with one of these.
#
# IMPORTANT: vesc.launch.py does not import this module (launch files here treat
# launched nodes as opaque subprocesses communicating only via exit code/params/
# topics, never via a Python import of node internals) -- it keeps its own local
# copy of this mapping for logging. Keep that copy in sync with this one if it
# ever changes.
# ---------------------------------------------------------------------------
EXIT_SUCCESS = 0
EXIT_INSUFFICIENT_SAMPLES = 1
EXIT_NOT_STATIONARY = 2
EXIT_MISSING_DEPENDENCY = 3
# Added by the calibration-safety-gates pass (Phase A) -- see this pass's own
# report (investigation into the 2026-08-19 11:06:58 bad gyro_bias_z write)
# for the full reasoning behind both. Currently only raised by
# gyro_bias_calibration_node.py; sensor_covariance_calibration_node.py is
# unchanged (different keys, different write path -- explicitly out of scope
# for this pass, see gyro_bias_calibration_node.py's own module docstring).
EXIT_SANITY_VIOLATION = 4
EXIT_MOTION_DURING_SAMPLING = 5

EXIT_REASONS = {
    EXIT_SUCCESS: 'success',
    EXIT_INSUFFICIENT_SAMPLES: (
        'insufficient samples (no/too little data on the sampled '
        'topic(s) -- is the sensor publishing?)'),
    EXIT_NOT_STATIONARY: (
        'stationary check failed or timed out (car never confirmed '
        'still before the timeout, or the state topic never published)'),
    EXIT_MISSING_DEPENDENCY: (
        'ruamel.yaml not installed -- computed values were logged '
        'but vesc.yaml was NOT written'),
    EXIT_SANITY_VIOLATION: (
        'measured value failed a post-sampling sanity bound (implausible '
        'magnitude or implausible delta from the value already in effect) '
        '-- see the ERROR log line above for both values. vesc.yaml was NOT '
        'written.'),
    EXIT_MOTION_DURING_SAMPLING: (
        'motion or vibration detected partway through the sampling window '
        '(ERPM speed or accelerometer deviation) -- see the ERROR log line '
        'above for when. vesc.yaml was NOT written.'),
}


def resolve_source_config_path(*relative_parts: str) -> str:
    """Reliably resolve a real SOURCE-TREE config file (e.g. 'f1tenth_bringup',
    'config', 'ekf_global.yaml'), regardless of whether this workspace was
    built with --symlink-install. General form of resolve_source_vesc_yaml_
    path's own logic (now a thin wrapper around this) -- see that function's
    own docstring, unchanged below, for the full "why walk up looking for
    'install'" reasoning; only the joined path at the end is now a parameter
    instead of a hardcoded ('f1tenth_bringup', 'config', 'vesc.yaml').
    """
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == 'install':
            ws_root = parent.parent
            return str(ws_root.joinpath('src', *relative_parts))

    # No 'install' ancestor -- this file's own resolved location IS inside
    # src/ already (editable install via --symlink-install).
    # parents[0] = f1tenth_diagnostics/f1tenth_diagnostics (this file's dir)
    # parents[1] = f1tenth_diagnostics (the package, containing setup.py)
    # parents[2] = src (the actual workspace src/ root)
    src_dir = this_file.parents[2]
    return str(src_dir.joinpath(*relative_parts))


def resolve_source_vesc_yaml_path() -> str:
    """Reliably resolve the real SOURCE-TREE vesc.yaml, regardless of whether
    this workspace was built with --symlink-install.

    ament_python's own .py modules are only editable-installed (egg-link back
    to src/) when the workspace was built WITH --symlink-install. Without it
    (confirmed to be this workspace's actual state), this file's own __file__
    resolves to a PLAIN COPY under
    install/f1tenth_diagnostics/lib/python3.10/site-packages/..., same as any
    other install-space file -- so anchoring off __file__ alone is not
    reliable either. What IS reliable regardless of --symlink-install is
    colcon's own workspace layout convention: <ws_root>/install/<pkg>/... and
    <ws_root>/src/<pkg>/... are always siblings. So: walk up this file's
    resolved path looking for an 'install' directory; if found, its parent IS
    the workspace root, and 'src' sits right next to it. If no 'install'
    ancestor is found at all, this file must BE the source file already
    (--symlink-install case), so fall back to anchoring directly off it.

    Used both by calibration.launch.py (to compute an explicit vesc_yaml_path
    override for both nodes) and available for anyone invoking a node
    directly via `ros2 run` in a --symlink-install workspace. Now a thin
    wrapper around the more general resolve_source_config_path() (added for
    slam_pose_covariance_calibration_node.py's own, different target file) --
    unchanged behavior/signature, existing callers need no changes.
    """
    return resolve_source_config_path('f1tenth_bringup', 'config', 'vesc.yaml')


# Which ros__parameters block each writable key lives under in vesc.yaml.
# Anything not listed here falls back to the shared '/**' block (covers all 6
# gyro/accel variance keys plus gyro_bias_z -- all genuinely shared params).
_KEY_SECTIONS = {
    'vx_variance': 'vesc_to_odom_node',
}
_SHARED_SECTION = '/**'
_DEFAULT_MAX_BACKUPS = 5


def read_vesc_yaml_value(vesc_yaml_path, key):
    """Read `key`'s CURRENT value out of vesc_yaml_path (same section-routing
    as write_vesc_yaml, same ruamel.yaml round-trip loader) -- for a
    calibration node that needs to know what value was already in effect
    BEFORE it patches a new one in. Added alongside gyro_bias_calibration_
    node's own residual-vs-absolute fix (see that node's own comment on its
    call site): gyro_bias_z specifically is subtracted from the raw gyro
    reading by vesc_driver_node BEFORE the calibration node ever samples it
    (vesc_driver.cpp: `angular_velocity.z = imuData->gyr_z() - gyro_bias_z_`),
    so the node's own measured mean is a RESIDUAL relative to whatever
    gyro_bias_z the driver it's sampling from was actually started with --
    not the raw hardware bias directly. Reading that value back here is what
    lets the caller add the two together instead of overwriting the correct
    total with just the latest residual.

    Raises RuntimeError if ruamel.yaml isn't importable -- same contract as
    write_vesc_yaml, same reasoning (caller checks `YAML is None` first).
    """
    if YAML is None:
        raise RuntimeError(
            'ruamel.yaml is not installed (pip install ruamel.yaml or '
            'apt install python3-ruamel.yaml) -- cannot read vesc.yaml.')
    section = _KEY_SECTIONS.get(key, _SHARED_SECTION)
    yaml = YAML()
    yaml.preserve_quotes = True
    with open(vesc_yaml_path, 'r') as f:
        data = yaml.load(f)
    return float(data[section]['ros__parameters'][key])


def write_vesc_yaml(vesc_yaml_path, results, logger, max_backups=_DEFAULT_MAX_BACKUPS):
    """Back up vesc_yaml_path (timestamped copy, same directory) then patch,
    in place via ruamel.yaml's round-trip mode, exactly the keys present in
    `results` (preserves comments/formatting/ordering). Shared by both
    calibration nodes/modes -- each key is routed to the YAML section it
    actually lives in via _KEY_SECTIONS (default: the shared '/**' block).

    After writing, prunes old `<vesc_yaml_path>.bak.*` files down to the
    newest `max_backups` (default 5, matching the count of stale pre-existing
    backups cleaned up in an earlier pass of this repo) -- without this,
    running calibration at every boot accumulates one new backup file per
    boot forever. Chosen policy: keep the last N rather than a single rolling
    backup, so a bad calibration doesn't immediately erase the only recent
    good backup if noticed a few boots later.

    Raises RuntimeError if ruamel.yaml isn't importable -- callers are
    expected to check that themselves first (via `YAML is None`, re-exported
    from this module) so they can set EXIT_MISSING_DEPENDENCY and log before
    ever calling this, but this guard exists too in case that's skipped.
    """
    if YAML is None:
        raise RuntimeError(
            'ruamel.yaml is not installed (pip install ruamel.yaml or '
            'apt install python3-ruamel.yaml) -- cannot write vesc.yaml.')

    timestamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    backup_path = f'{vesc_yaml_path}.bak.{timestamp}'
    shutil.copy2(vesc_yaml_path, backup_path)
    logger.info(f'Backed up {vesc_yaml_path} -> {backup_path}')

    yaml = YAML()
    yaml.preserve_quotes = True
    with open(vesc_yaml_path, 'r') as f:
        data = yaml.load(f)

    for key, value in results.items():
        section = _KEY_SECTIONS.get(key, _SHARED_SECTION)
        data[section]['ros__parameters'][key] = value

    with open(vesc_yaml_path, 'w') as f:
        yaml.dump(data, f)

    logger.info(f'calibration complete, updated {list(results.keys())} in: '
                f'[{vesc_yaml_path}]')

    _prune_old_backups(vesc_yaml_path, max_backups, logger)


def write_yaml_config(yaml_path, section, results, logger, max_backups=_DEFAULT_MAX_BACKUPS):
    """General single-section sibling of write_vesc_yaml -- for a calibration
    node whose target file only ever needs ONE fixed destination section
    (slam_pose_covariance_calibration_node.py's own ekf_global.yaml/
    slam_pose_relay_node case), rather than vesc.yaml's own per-key section
    routing (_KEY_SECTIONS) that write_vesc_yaml exists for. Deliberately a
    SEPARATE function, not a generalization of write_vesc_yaml itself (which
    is left completely unchanged by this addition, zero risk to its own two
    existing, boot-time-wired callers) -- shares the same backup/prune
    mechanics (_prune_old_backups is already a generic, path-only utility
    despite its own vesc-flavored name) and the same ruamel.yaml round-trip
    approach (preserves comments/formatting/ordering), just with a single,
    caller-supplied section name instead of a lookup table.

    Raises RuntimeError if ruamel.yaml isn't importable -- same contract as
    write_vesc_yaml.
    """
    if YAML is None:
        raise RuntimeError(
            'ruamel.yaml is not installed (pip install ruamel.yaml or '
            f'apt install python3-ruamel.yaml) -- cannot write {yaml_path}.')

    timestamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    backup_path = f'{yaml_path}.bak.{timestamp}'
    shutil.copy2(yaml_path, backup_path)
    logger.info(f'Backed up {yaml_path} -> {backup_path}')

    yaml = YAML()
    yaml.preserve_quotes = True
    with open(yaml_path, 'r') as f:
        data = yaml.load(f)

    for key, value in results.items():
        data[section]['ros__parameters'][key] = value

    with open(yaml_path, 'w') as f:
        yaml.dump(data, f)

    logger.info(f'calibration complete, updated {list(results.keys())} in '
                f'[{yaml_path}] section "{section}"')

    _prune_old_backups(yaml_path, max_backups, logger)


def _prune_old_backups(vesc_yaml_path, max_backups, logger):
    """Delete all but the newest `max_backups` `<vesc_yaml_path>.bak.*` files.
    The timestamp format (%Y%m%dT%H%M%S) sorts lexicographically in
    chronological order, so a plain name sort is enough -- no need to parse
    the timestamps back out. max_backups <= 0 disables pruning entirely."""
    if max_backups <= 0:
        return
    directory = os.path.dirname(vesc_yaml_path) or '.'
    base_name = os.path.basename(vesc_yaml_path)
    prefix = f'{base_name}.bak.'
    try:
        candidates = sorted(
            name for name in os.listdir(directory) if name.startswith(prefix))
    except OSError as exc:
        logger.warning(f'could not list {directory} to prune old backups: {exc}')
        return

    excess = candidates[:-max_backups] if len(candidates) > max_backups else []
    for name in excess:
        path = os.path.join(directory, name)
        try:
            os.remove(path)
            logger.info(f'pruned old calibration backup: {path}')
        except OSError as exc:
            logger.warning(f'failed to prune old backup {path}: {exc}')


def check_sanity_bound(key, new_value, old_value, absolute_bound, delta_bound, logger):
    """calibration-safety-gates pass (Phase A) -- refuses a calibration write
    that looks physically implausible, closing the actual gap that let the
    2026-08-19 11:06:58 bad gyro_bias_z write take effect silently (see this
    pass's own report). Two independent bounds, either one failing refuses
    the write:
      - absolute: |new_value| must stay under absolute_bound.
      - delta: |new_value - old_value| must stay under delta_bound (catches an
        implausible SWING even if the absolute value alone still looks
        plausible) -- but ONLY when old_value itself already passes
        absolute_bound. See "delta-bound recovery trap" paragraph below for
        why that condition exists.
    That actual bad write would have tripped BOTH at their current defaults:
    |new|=0.0590 >= absolute_bound=0.05, AND |delta|=0.0523 >= delta_bound=0.02.

    delta-bound recovery trap (found live, same day as the bad write this
    pass otherwise closes -- a real gap in the FIRST version of this
    function, not hypothetical): once a bad value like -0.0590 is already
    sitting in vesc.yaml, correcting it back to a genuinely good value
    (e.g. -0.008) is ITSELF a large delta (~0.051) -- the unconditional delta
    bound above would refuse that correction too, permanently trapping
    vesc.yaml in the bad state (a healthy re-measurement can never land,
    since ANY value close enough to the true bias to pass its OWN absolute
    bound is, by construction, far from the bad old value). Fixed by making
    the delta bound conditional on the OLD value's own sanity: if old_value
    already fails absolute_bound, this function trusts it less than a fresh
    measurement and skips the delta bound entirely for this call, enforcing
    only absolute_bound on new_value -- recovery to any plausible value is
    then always reachable in one calibration run, not permanently blocked.
    In the normal (old_value already sane) case, both bounds apply exactly
    as before -- this only relaxes the specific "digging out of a hole"
    case, never loosens protection against a first bad write while old_value
    was already fine.

    Deliberately a plain function taking old/new/bounds/logger, not a class or
    anything reading vesc.yaml itself -- callers own reading the old value and
    deciding what to do with a False return (today: gyro_bias_calibration_
    node.py's _finish(), before ever calling write_vesc_yaml -- see that call
    site). write_vesc_yaml() itself stays completely value-agnostic and
    unchanged, so sensor_covariance_calibration_node.py (out of scope for this
    pass, different keys) is untouched by this addition.

    Returns True if `new_value` passes the bound(s) actually in force for this
    call (see above). Logs an INFO line naming which bounds are being
    enforced and why (routine, every call -- visible in the calibration log
    regardless of outcome), then on failure an ERROR (both values, which
    bound(s) failed, by how much) and returns False -- callers MUST NOT write
    on a False return.
    """
    delta = new_value - old_value
    old_is_sane = abs(old_value) < absolute_bound
    if old_is_sane:
        logger.info(
            f'Sanity check for {key}: old value {old_value:+.6f} is within '
            f'absolute_bound={absolute_bound:.6f} -- enforcing both absolute_bound '
            'and delta_bound on the new value.')
    else:
        logger.info(
            f'Sanity check for {key}: old value {old_value:+.6f} already fails '
            f'absolute_bound={absolute_bound:.6f} -- recovery from out-of-bound '
            'stored value, enforcing ONLY absolute_bound on the new value '
            '(delta_bound skipped, so a genuine correction is never permanently '
            'blocked by its own distance from the bad old value).')

    violations = []
    if abs(new_value) >= absolute_bound:
        violations.append(
            f'|{key}|={abs(new_value):.6f} >= absolute_bound={absolute_bound:.6f}')
    if old_is_sane and abs(delta) >= delta_bound:
        violations.append(
            f'|delta {key}|={abs(delta):.6f} >= delta_bound={delta_bound:.6f}')
    if not violations:
        return True
    logger.error(
        f'Sanity check FAILED for {key}: ' + '; '.join(violations) +
        f' -- old={old_value:+.6f} new={new_value:+.6f} delta={delta:+.6f}. '
        'Refusing to write vesc.yaml.')
    return False


def exceeds_motion_threshold(speed, threshold):
    """True if |speed| (VescStateStamped.state.speed, raw ERPM) indicates real
    motion. calibration-safety-gates pass (Phase A): used both by the existing
    pre-sampling StationaryGate below (via on_speed_sample, unchanged) and, new
    in this pass, by gyro_bias_calibration_node.py's continuous during-
    sampling check -- see that node's own _state_callback for why the
    pre-sampling gate's own one-shot confirmation isn't enough on its own
    (nothing used to watch the car for the ~30s sampling window itself)."""
    return abs(speed) > threshold


def exceeds_vibration_threshold(accel_x, accel_y, accel_z, threshold, gravity=1.0):
    """True if the summed deviation |accel_x|+|accel_y|+|accel_z-gravity|
    exceeds `threshold` -- a motion/vibration signature on the accelerometer,
    independent of (and a real gap not covered by) the ERPM-based check above:
    catches disturbances ERPM alone can't see (something touching/bumping the
    car, an external vibration source) during gyro_bias_calibration_node's
    sampling window.

    gravity defaults to 1.0, NOT 9.81 -- confirmed live, not assumed: this
    stack's vendored VESC IMU driver (f1tenth_hardware/vesc/vesc_driver/src/
    vesc_packet.cpp, VescPacketImu::acc_x/y/z()'s own "g/s" comment) publishes
    linear_acceleration in units of standard gravity (measured az ~0.996 at
    rest over a 20s/965-sample stationary probe, this pass's own
    verification), not m/s^2 as sensor_msgs/Imu's own REP-103 convention
    calls for -- a pre-existing vendored-driver mismatch, out of scope to fix
    here, but this check has to match what's ACTUALLY published or it would
    trip on every single sample even at rest (|az - 9.81| alone would be
    ~8.8 at rest with this driver)."""
    return abs(accel_x) + abs(accel_y) + abs(accel_z - gravity) > threshold


class Welford:
    """Online (flat-memory) mean/variance accumulator -- no sample buffer.
    Moved here from sensor_covariance_calibration_node.py's own private
    _Welford (that file now imports this instead of keeping its own copy) so
    slam_pose_covariance_calibration_node.py can share the exact same,
    already-live-used implementation rather than a fresh reimplementation of
    the same handful of lines."""

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x):
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.m2 += delta * delta2

    def variance(self):
        # sample variance (n-1 denominator); undefined for n < 2
        return self.m2 / (self.n - 1)


class StationaryGate:
    """Confirms the car is stationary, via raw ERPM telemetry
    (VescStateStamped.state.speed on state_topic, default /sensors/core --
    vesc_driver_node already zeroes anything below vesc.yaml's erpm_deadband
    before publishing, so a small nonzero threshold here is comparing against
    an already largely-denoised signal), for a continuous confirm_sec window
    before sampling is allowed to start. A single near-zero reading is not
    treated as proof of stillness -- the window must hold continuously; any
    reading above threshold resets it.

    Holds no rclpy state of its own (no subscription, no timer) so it's
    reusable across both calibration nodes' different callback wiring --
    callers feed it samples via on_speed_sample() from their own
    subscription callback, and poll `confirmed` / call timed_out() from
    either that same callback or a periodic watchdog timer (a watchdog is
    needed regardless of message traffic, since if state_topic never
    publishes at all -- e.g. hardware disconnected -- on_speed_sample() is
    never called and only a timer will ever notice the timeout).
    """

    def __init__(self, threshold, confirm_sec, timeout_sec, now_fn=time.monotonic):
        self._threshold = threshold
        self._confirm_sec = confirm_sec
        self._timeout_sec = timeout_sec
        self._now_fn = now_fn
        self._start_time = now_fn()
        self._stationary_since = None
        self.confirmed = False

    def on_speed_sample(self, speed):
        if self.confirmed:
            return
        now = self._now_fn()
        if abs(speed) <= self._threshold:
            if self._stationary_since is None:
                self._stationary_since = now
            elif now - self._stationary_since >= self._confirm_sec:
                self.confirmed = True
        else:
            self._stationary_since = None  # motion detected -- reset the window

    def timed_out(self):
        if self.confirmed:
            return False
        return (self._now_fn() - self._start_time) >= self._timeout_sec


# Sentinel lines bracketing an auto-written provenance block. A block is
# removed on rewrite by deleting BEGIN..END inclusive -- see
# strip_provenance_block() for why bracketing beats trying to recognise the
# block by its content.
PROVENANCE_BEGIN = '>>> BEGIN {tool} provenance -- auto-written, edits below are overwritten'
PROVENANCE_END = '<<< END {tool} provenance'


def _provenance_bounds(tool):
    return PROVENANCE_BEGIN.format(tool=tool), PROVENANCE_END.format(tool=tool)


def strip_provenance_block(text, tool):
    """Delete every BEGIN..END provenance block belonging to `tool` from a run
    of comment lines, leaving all other lines untouched. Pure string surgery,
    unit-testable without ruamel.

    Line-level, and bracketed by explicit sentinels, for two reasons found by
    testing this against the real steering_calibration.yaml rather than
    assuming:

      - ruamel does NOT round-trip a "before key" comment back to where it
        wrote it. yaml_set_comment_before_after_key puts the block in slot 1
        of the target key, but on the next load that same text comes back in
        slot 2 (the after-value comment) of the PRECEDING key. Clearing only
        the slot we wrote to therefore misses the block entirely on every
        re-run, and provenance blocks stack up one per calibration run --
        observed, three runs gave three stacked blocks.
      - ruamel merges a contiguous run of comment lines into ONE CommentToken.
        So a token can hold a human-written comment AND an auto-written block
        together, and dropping whole tokens destroys human documentation.
        (Confirmed live: steering_calibration.yaml's "Placeholder until
        retuned" note about gain_left/_right is parsed into the same slot as
        the key above it.)

    Bracketing sidesteps both: find the sentinels wherever they turn up, and
    delete only the lines between them.
    """
    begin, end = _provenance_bounds(tool)
    out = []
    skipping = False
    for line in text.split('\n'):
        if not skipping and begin in line:
            skipping = True
            continue
        if skipping:
            if end in line:
                skipping = False
            continue
        out.append(line)
    # An unterminated block (file hand-edited mid-block) would otherwise eat
    # the rest of the comment run; putting the lines back is the safe failure.
    return '\n'.join(out) if not skipping else text


def _strip_provenance_from_comments(block, tool):
    """Apply strip_provenance_block to every comment token attached anywhere in
    `block`, regardless of which key/slot ruamel filed it under (see that
    function's docstring for why the location is not predictable)."""
    ca = getattr(block, 'ca', None)
    if ca is None:
        return
    for entry in ca.items.values():
        for index, slot in enumerate(entry):
            if slot is None:
                continue
            tokens = slot if isinstance(slot, list) else [slot]
            for token in tokens:
                value = getattr(token, 'value', None)
                if value is None:
                    continue
                token.value = strip_provenance_block(value, tool)
            if not isinstance(slot, list):
                # A slot whose comment is now empty must become None, or
                # ruamel emits a stray blank comment line on dump.
                if not entry[index].value.strip():
                    entry[index] = None


_KEY_LINE_RE = re.compile(r'^(\s*)([A-Za-z_][A-Za-z0-9_]*):(\s.*)$')


def restore_unintended_value_changes(before_text, after_text, intended_keys, logger):
    """Undo any `key: value` line the YAML round-trip changed that we did not
    ask it to change.

    ruamel round-trip mode preserves formatting and comments, but it does NOT
    guarantee byte-identical re-emission of every scalar. Observed live on the
    real vesc.yaml: writing `wheelbase` also rewrote an untouched neighbour,

        -    accel_variance_y: 7.457561226365246e-06
        +    accel_variance_y: 7.457561226365245e-06

    one unit in the last place, from re-formatting a float ruamel could not
    round-trip exactly. Numerically irrelevant, and precisely the kind of
    silent, unrequested edit to a separately-calibrated value that this
    codebase keeps getting bitten by -- it also makes the printed diff lie
    about what the calibration actually did.

    So: after the dump, any changed line whose key is not one we intended to
    write is restored from the original text, and the fact is logged. Keys are
    matched by name, which is enough here because these config files do not
    reuse a key name across sections with different values; a key that IS
    intended is left exactly as ruamel wrote it.
    """
    intended = set(intended_keys)
    before_by_key = {}
    for line in before_text.split('\n'):
        match = _KEY_LINE_RE.match(line)
        if match and match.group(2) not in intended:
            before_by_key.setdefault(match.group(2), []).append(line)

    restored = []
    out = []
    seen = {}
    for line in after_text.split('\n'):
        match = _KEY_LINE_RE.match(line)
        if match:
            key = match.group(2)
            index = seen.get(key, 0)
            seen[key] = index + 1
            originals = before_by_key.get(key)
            if originals is not None and index < len(originals):
                original = originals[index]
                if original != line:
                    restored.append((line.strip(), original.strip()))
                    out.append(original)
                    continue
        out.append(line)

    if restored:
        logger.warning(
            f'the YAML round-trip altered {len(restored)} line(s) this calibration '
            'did not intend to touch; restoring the original value(s): ' +
            '; '.join(f'{new!r} -> {old!r}' for new, old in restored))
    return '\n'.join(out)


def write_yaml_config_with_provenance(yaml_path, updates, provenance_lines, logger,
                                      tool='calibration', max_backups=_DEFAULT_MAX_BACKUPS):
    """Backup-then-patch like write_vesc_yaml/write_yaml_config, with three
    things steering_offset_calibration_node needs that neither existing writer
    has: MULTI-SECTION updates in one pass, a PROVENANCE COMMENT BLOCK written
    into the file above each patched key, and replacement (not stacking) of a
    previous run's provenance block.

    A third separate function rather than a generalization of the other two,
    following this module's own established convention (see write_yaml_config's
    docstring): both existing writers have live, boot-time-wired callers
    (gyro_bias_calibration_node, sensor_covariance_calibration_node) and are
    left byte-for-byte unchanged by this addition.

    updates: {section_name: {key: value}}, e.g.
        {'/**': {'steering_angle_to_servo_offset': 0.4494}}
    provenance_lines: plain strings WITHOUT a leading '#' -- the comment marker
        and indentation are added here. Written into the file itself, not just
        the log, because the log is gone by the next boot while the yaml is
        what somebody reads six months later wondering where the number came
        from.
    tool: names the writing tool in the block's BEGIN/END sentinels, and is
        what identifies a previous block for removal.

    Returns (backup_path, unified_diff_text). Deliberately does not print the
    diff itself -- the caller decides whether it goes to the console,
    /diagnostics, or both.
    """
    if YAML is None:
        raise RuntimeError(
            'ruamel.yaml is not installed (pip install ruamel.yaml or '
            f'apt install python3-ruamel.yaml) -- cannot write {yaml_path}.')

    with open(yaml_path, 'r') as f:
        before_text = f.read()

    timestamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    backup_path = f'{yaml_path}.bak.{timestamp}'
    shutil.copy2(yaml_path, backup_path)
    logger.info(f'Backed up {yaml_path} -> {backup_path}')

    yaml = YAML()
    yaml.preserve_quotes = True
    with open(yaml_path, 'r') as f:
        data = yaml.load(f)

    begin, end = _provenance_bounds(tool)
    for section, results in updates.items():
        block = data[section]['ros__parameters']
        _strip_provenance_from_comments(block, tool)
        first_key = None
        for key, value in results.items():
            block[key] = value
            if first_key is None:
                first_key = key
        if first_key is not None and provenance_lines:
            lines = [begin] + list(provenance_lines) + [end]
            comment = '\n'.join(line.rstrip() for line in lines)
            block.yaml_set_comment_before_after_key(
                first_key, before='\n' + comment, indent=4)

    buffer = io.StringIO()
    yaml.dump(data, buffer)
    # Dump to a buffer first so unrequested round-trip changes can be reverted
    # BEFORE anything reaches disk -- see restore_unintended_value_changes.
    after_text = restore_unintended_value_changes(
        before_text, buffer.getvalue(),
        [key for results in updates.values() for key in results], logger)
    with open(yaml_path, 'w') as f:
        f.write(after_text)

    diff_text = ''.join(difflib.unified_diff(
        before_text.splitlines(keepends=True),
        after_text.splitlines(keepends=True),
        fromfile=f'{os.path.basename(yaml_path)} (before)',
        tofile=f'{os.path.basename(yaml_path)} (after)'))

    written = {sec: list(res) for sec, res in updates.items()}
    logger.info(f'calibration complete, updated {written} in: [{yaml_path}]')
    _prune_old_backups(yaml_path, max_backups, logger)
    return backup_path, diff_text
