"""Static/logic tests for the 'localization' deferred-start fix (calibration-
restart gap -- see component_supervisor_node.py's own module docstring,
'localization deferred-start' paragraph, and this pass's own report).

No rclpy.init() / live ROS graph needed for these -- all pieces under test are
plain-Python guard logic:
  - _apply_calibration_override(): reads self.calibration (the live declared
    parameter -- calibration-single-source-of-truth pass) and writes it into
    self._registry['hardware']'s own args dict.
  - _hardware_will_calibrate(): reads only self._registry, a plain dict built
    from components.yaml at parse time (plus whatever
    _apply_calibration_override() already wrote into it).
  - _resolve_localization_deferral(): reads/writes only
    self._localization_deferred_started/_localization_calib_sub/
    _localization_calib_timeout_timer -- called here against a lightweight
    stand-in object (duck-typed, not a real Node) rather than a constructed
    ComponentSupervisorNode, so these tests don't need rclpy at all.

What this does NOT cover (flagged, not silently skipped): the actual
/calibration/in_progress subscription wiring and the True->False detection
over a real topic -- that needs a live rclpy context and is out of scope for
a no-hardware verification pass. See this pass's own report for what to
re-check live once hardware is reconnected.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'f1tenth_bringup'))

from component_supervisor_node import ComponentSupervisorNode  # noqa: E402


class _FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _FakeSupervisor:
    """Duck-typed stand-in for ComponentSupervisorNode -- exposes only the
    attributes the methods under test actually touch, plus a
    destroy_subscription() stub matching rclpy.node.Node's own signature."""

    def __init__(self, registry=None, deferred_started=True, calibration=True):
        self._registry = registry or {}
        self._localization_deferred_started = deferred_started
        self._localization_calib_sub = 'sub-sentinel'
        self._localization_calib_timeout_timer = _FakeTimer()
        self.destroyed_subs = []
        # calibration-single-source-of-truth pass: the live declared parameter
        # _apply_calibration_override() reads -- see that method's own docstring.
        self.calibration = calibration

    def destroy_subscription(self, sub):
        self.destroyed_subs.append(sub)


# -- _hardware_will_calibrate ------------------------------------------------

@pytest.mark.parametrize('args,expected', [
    ({'calibration': 'true'}, True),
    ({'calibration': 'True'}, True),   # YAML bool-as-string, case-insensitive
    ({'calibration': ' true '}, True),  # tolerate incidental whitespace
    ({'calibration': 'false'}, False),
    ({}, False),                        # key absent entirely
    ({'calibration': ''}, False),
])
def test_hardware_will_calibrate_single_entry(args, expected):
    fake = _FakeSupervisor(registry={'hardware': [{'args': args}]})
    assert ComponentSupervisorNode._hardware_will_calibrate(fake) is expected


def test_hardware_will_calibrate_no_hardware_component():
    # Registry missing 'hardware' entirely (customized deployment) -- must not
    # raise, must resolve to False (nothing to defer against).
    fake = _FakeSupervisor(registry={})
    assert ComponentSupervisorNode._hardware_will_calibrate(fake) is False


def test_hardware_will_calibrate_multi_entry_any_true():
    # components.yaml's own registry shape: a component can have >1 launch
    # entry. True if ANY entry requests calibration, matching the real
    # 'hardware' entry (one launch file today, but not assumed fixed at one).
    fake = _FakeSupervisor(registry={'hardware': [
        {'args': {'calibration': 'false'}},
        {'args': {'calibration': 'true'}},
    ]})
    assert ComponentSupervisorNode._hardware_will_calibrate(fake) is True


def test_hardware_will_calibrate_matches_real_components_yaml():
    """calibration-single-source-of-truth pass: reads the actual, real
    components.yaml this workspace ships -- not a synthetic fixture.
    'hardware's own entry no longer carries a 'calibration' key at all (see
    that file's own header comment) -- stack_params.yaml's `calibration` key,
    applied at runtime via _apply_calibration_override(), is now the only
    thing that decides this. Confirms the structural absence first (a stale
    literal reappearing here would silently defeat the override -- it would
    still get overwritten, but that's exactly the kind of confusion this pass
    removed), then runs the real override + gate for both live values."""
    import yaml
    components_yaml = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'components.yaml')
    with open(components_yaml) as f:
        registry_raw = yaml.safe_load(f)['components']

    for entry in registry_raw['hardware']:
        assert 'calibration' not in entry.get('args', {}), (
            "components.yaml's 'hardware' entry has a 'calibration' key again -- "
            'stack_params.yaml is supposed to be the only declaration of this '
            "value now (see that file's own `calibration` comment).")

    for calibration, expected in [(True, True), (False, False)]:
        registry = {
            name: [dict(entry, args=entry.get('args', {})) for entry in entries]
            for name, entries in registry_raw.items()
        }
        fake = _FakeSupervisor(registry=registry, calibration=calibration)
        ComponentSupervisorNode._apply_calibration_override(fake)
        assert ComponentSupervisorNode._hardware_will_calibrate(fake) is expected


# -- _apply_calibration_override ----------------------------------------------
# calibration-single-source-of-truth pass: the live 'calibration' parameter
# (wired from supervisor_bringup.launch.py's own launch argument, default
# sourced from stack_params.yaml) is now the only input -- components.yaml's
# `hardware` entry carries no 'calibration' key of its own at all.

@pytest.mark.parametrize('calibration,expected_arg', [
    (True, 'true'),
    (False, 'false'),
])
def test_apply_calibration_override_writes_live_value(calibration, expected_arg):
    fake = _FakeSupervisor(
        registry={'hardware': [{'args': {}}]}, calibration=calibration)
    ComponentSupervisorNode._apply_calibration_override(fake)
    assert fake._registry['hardware'][0]['args']['calibration'] == expected_arg


def test_apply_calibration_override_overwrites_any_stale_literal():
    """Even if a customized registry still had a 'calibration' key of its own,
    the live parameter must win -- there is exactly one source of truth now,
    not a components.yaml default with a param override only sometimes
    applying."""
    fake = _FakeSupervisor(
        registry={'hardware': [{'args': {'calibration': 'true'}}]},
        calibration=False)
    ComponentSupervisorNode._apply_calibration_override(fake)
    assert fake._registry['hardware'][0]['args']['calibration'] == 'false'


def test_apply_calibration_override_no_hardware_component():
    # Must not raise on a customized registry missing 'hardware' entirely.
    fake = _FakeSupervisor(registry={}, calibration=True)
    ComponentSupervisorNode._apply_calibration_override(fake)  # no exception


def test_apply_calibration_override_leaves_other_args_untouched():
    fake = _FakeSupervisor(
        registry={'hardware': [{'args': {'release_downstream': 'false'}}]},
        calibration=True)
    ComponentSupervisorNode._apply_calibration_override(fake)
    assert fake._registry['hardware'][0]['args'] == {
        'release_downstream': 'false', 'calibration': 'true'}


# -- live param -> registry -> defer gate, end to end -------------------------
# The actual bug this pass fixes: previously components.yaml hardcoded
# 'hardware's calibration arg to 'true' with no live override path at all, so
# _hardware_will_calibrate() (and therefore __init__'s own defer_localization
# gate) could never see 'false' short of hand-editing that file. These cover
# the gate branching correctly via the real live-parameter path -- not just
# the components.yaml literal the old version of this test file only checked.

@pytest.mark.parametrize('calibration,expect_defer', [
    (False, False),  # calibration=false -> skip the wait, start immediately
    (True, True),     # calibration=true -> defer + wait, as before
])
def test_calibration_param_drives_defer_gate_end_to_end(calibration, expect_defer):
    fake = _FakeSupervisor(
        registry={'hardware': [{'args': {'release_downstream': 'false'}}]},
        calibration=calibration)
    ComponentSupervisorNode._apply_calibration_override(fake)
    assert ComponentSupervisorNode._hardware_will_calibrate(fake) is expect_defer


# -- _resolve_localization_deferral ------------------------------------------

def test_resolve_localization_deferral_first_call_acts_and_cleans_up():
    fake = _FakeSupervisor(deferred_started=False)
    sub_sentinel = fake._localization_calib_sub
    timer = fake._localization_calib_timeout_timer

    result = ComponentSupervisorNode._resolve_localization_deferral(fake)

    assert result is True
    assert fake._localization_deferred_started is True
    assert fake._localization_calib_sub is None
    assert fake.destroyed_subs == [sub_sentinel]
    assert timer.cancelled is True
    assert fake._localization_calib_timeout_timer is None


def test_resolve_localization_deferral_idempotent_on_repeat_calls():
    """The whole point of this guard: whichever of the three triggers
    (completion callback, timeout callback, manual service request) reaches
    it FIRST wins; every later one must be a safe no-op, not a double
    _start_component('localization') call."""
    fake = _FakeSupervisor(deferred_started=False)

    first = ComponentSupervisorNode._resolve_localization_deferral(fake)
    second = ComponentSupervisorNode._resolve_localization_deferral(fake)
    third = ComponentSupervisorNode._resolve_localization_deferral(fake)

    assert (first, second, third) == (True, False, False)
    # Only the first call's cleanup actually happened -- not repeated.
    assert len(fake.destroyed_subs) == 1


def test_resolve_localization_deferral_noop_when_nothing_was_pending():
    """The default state (deferral never applied at all -- e.g.
    calibration:=false, or a customized registry) must also be a safe no-op:
    every call site in this file calls this unconditionally, without first
    checking whether deferral ever applied."""
    fake = _FakeSupervisor(deferred_started=True)  # the __init__ default
    result = ComponentSupervisorNode._resolve_localization_deferral(fake)
    assert result is False
    assert fake.destroyed_subs == []  # nothing to clean up, nothing touched
