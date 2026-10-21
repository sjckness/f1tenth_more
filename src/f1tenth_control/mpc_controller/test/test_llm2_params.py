"""
LLM2's MPC-parameter updates, the pure half: mpc_controller/llm2_params.py.

The whitelist (only cost weights, sane bounds), the validation of one update
(whole message or nothing, no clamping), the one-slot pending store the
control loop drains, and the startup warnings. No ROS. The node half -- no
subscription when disabled, application only at the tick boundary -- is
test_llm2_mpc_corr_node.py.

Run standalone: python3 -m pytest test/test_llm2_params.py -v
"""

from array import array
import threading

from f1tenth_params.param_defaults import get_value
from mpc_controller.llm2_params import (
    build_whitelist, PendingUpdate, startup_warnings, tunable_weights, Update,
    validate_update)
import pytest

# MPC_corr's self.weights keys (the _w() block in MPC_corr.__init__).
_WEIGHT_KEYS = ('w_term', 'w_v', 'w_psi', 'w_u_a', 'w_du_delta', 'w_du_a', 'w_delta0',
                'w_obs', 'w_corr', 'w_line', 'w_psi_stage')
_TUNABLE = tunable_weights({key: 0.0 for key in _WEIGHT_KEYS})
_WL = {'mpc_w_v': (1.0, 10.0), 'mpc_w_term': (2.0, 20.0)}


def _shipped_whitelist():
    return build_whitelist(get_value('llm2_param_whitelist'), get_value('llm2_param_min'),
                           get_value('llm2_param_max'), _TUNABLE)


class TestTheShippedWhitelist:

    def test_it_is_the_six_cost_shape_weights(self):
        assert set(_shipped_whitelist()) == {
            'mpc_w_term', 'mpc_w_psi', 'mpc_w_psi_stage', 'mpc_w_corr', 'mpc_w_v',
            'mpc_w_du_delta'}

    def test_todays_value_of_every_listed_weight_is_inside_its_range(self):
        # The mock echoes today's values: one outside its range and every
        # mock update is rejected whole.
        for name, (lo, hi) in _shipped_whitelist().items():
            assert lo <= float(get_value(name)) <= hi, name

    def test_the_llm2_mock_set_passes_validation(self):
        # Couples stream C's two halves: the node's mock set (llm package)
        # against this side's whitelist. Skipped where llm is not installed.
        try:
            from llm.llm2_protocol import MOCK_PARAMS
        except ImportError:
            pytest.skip('llm package not on the path')
        names, values = zip(*dict(MOCK_PARAMS).items())
        assert validate_update(names, values, _shipped_whitelist()) == dict(MOCK_PARAMS)


class TestOnlyCostWeightsCanBeWhitelisted:

    @pytest.mark.parametrize('name', [
        'car_radius', 'avoidance_margin', 'delta_min', 'delta_max', 'vdes',
        'use_hard_boundary_constraints', 'boundary_hard', 'boundary_slack_max',
        'corr_half_width_start_m', 'max_forward_speed_mps', 'allow_reverse',
        'mpc_rear.w_v', 'w_v'])
    def test_a_non_weight_name_refuses_the_whitelist(self, name):
        with pytest.raises(ValueError, match='not an MPC cost weight'):
            build_whitelist([name], [0.0], [1.0], _TUNABLE)

    def test_every_weight_can_be_listed(self):
        names = sorted(_TUNABLE)
        wl = build_whitelist(names, [0.0] * len(names), [1.0] * len(names), _TUNABLE)
        assert set(wl) == {'mpc_' + key for key in _WEIGHT_KEYS}

    @pytest.mark.parametrize('names, mins, maxs, match', [
        (['mpc_w_v'], [1.0, 2.0], [3.0], 'not paired'),
        ([], [], [], 'empty'),
        (['mpc_w_v', 'mpc_w_v'], [1.0, 1.0], [2.0, 2.0], 'twice'),
        (['mpc_w_v'], [float('nan')], [2.0], 'not finite'),
        (['mpc_w_v'], [1.0], [float('inf')], 'not finite'),
        (['mpc_w_v'], [-0.5], [2.0], 'negative'),
        (['mpc_w_v'], [3.0], [2.0], 'above maximum'),
    ])
    def test_a_malformed_whitelist_is_refused(self, names, mins, maxs, match):
        with pytest.raises(ValueError, match=match):
            build_whitelist(names, mins, maxs, _TUNABLE)


class TestAnUpdateIsAllOrNothing:

    def test_a_whitelisted_subset_in_range_passes_as_floats(self):
        out = validate_update(['mpc_w_v'], [5], _WL)
        assert out == {'mpc_w_v': 5.0} and isinstance(out['mpc_w_v'], float)

    def test_the_bounds_are_inclusive(self):
        assert validate_update(['mpc_w_v', 'mpc_w_term'], [1.0, 20.0], _WL) == {
            'mpc_w_v': 1.0, 'mpc_w_term': 20.0}

    def test_rclpys_float64_array_is_accepted(self):
        # MpcParamUpdate.values arrives as array('d') in rclpy.
        assert validate_update(['mpc_w_v'], array('d', [4.5]), _WL) == {'mpc_w_v': 4.5}

    @pytest.mark.parametrize('names, values, match', [
        (['mpc_w_v', 'car_radius'], [5.0, 0.2], "'car_radius' is not in"),
        (['mpc_w_v', 'mpc_w_obs'], [5.0, 8.0], "'mpc_w_obs' is not in"),
        (['mpc_w_v'], [float('nan')], 'not finite'),
        (['mpc_w_v'], [float('inf')], 'not finite'),
        (['mpc_w_v'], [float('-inf')], 'not finite'),
        (['mpc_w_v'], [0.99], 'outside'),
        (['mpc_w_v'], [10.01], 'outside'),
        (['mpc_w_v', 'mpc_w_term'], [5.0, 25.0], 'outside'),
        (['mpc_w_v'], [True], 'not a number'),
        (['mpc_w_v'], ['5.0'], 'not a number'),
        (['mpc_w_v'], [None], 'not a number'),
        (['mpc_w_v', 'mpc_w_term'], [5.0], 'names but'),
        ([], [], 'no names'),
        (['mpc_w_v', 'mpc_w_v'], [5.0, 6.0], 'twice'),
    ])
    def test_one_bad_entry_rejects_the_whole_update(self, names, values, match):
        # Valid entries in the same message do not survive: nothing is
        # returned to apply, and nothing is clamped into range.
        with pytest.raises(ValueError, match=match):
            validate_update(names, values, _WL)


class TestThePendingSlot:

    def test_take_returns_the_update_once(self):
        slot = PendingUpdate()
        assert slot.take() is None
        update = Update(3, 'llm2_mock', {'mpc_w_v': 5.0})
        assert slot.put(update) is None
        assert slot.take() == update
        assert slot.take() is None

    def test_a_second_put_merges_over_the_unapplied_first(self):
        slot = PendingUpdate()
        slot.put(Update(1, 'a', {'mpc_w_v': 5.0, 'mpc_w_term': 10.0}))
        assert slot.put(Update(2, 'b', {'mpc_w_v': 6.0})) == 1
        assert slot.take() == Update(2, 'b', {'mpc_w_v': 6.0, 'mpc_w_term': 10.0})

    def test_concurrent_puts_and_takes_lose_no_update(self):
        # Every seq put is accounted for exactly once: either taken, or
        # reported by the put that merged over it.
        slot = PendingUpdate()
        merged, taken, done = [], [], threading.Event()

        def producer(base):
            for i in range(300):
                prev = slot.put(Update(base + i, 'p', {'mpc_w_v': 5.0}))
                if prev is not None:
                    merged.append(prev)

        def consumer():
            while not done.is_set():
                update = slot.take()
                if update is not None:
                    taken.append(update.seq)

        threads = [threading.Thread(target=producer, args=(k * 1000,)) for k in range(6)]
        reader = threading.Thread(target=consumer)
        reader.start()
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        done.set()
        reader.join()
        last = slot.take()
        if last is not None:
            taken.append(last.seq)
        assert sorted(merged + taken) == sorted(k * 1000 + i for k in range(6) for i in range(300))


class TestStartupWarnings:

    def test_a_current_value_outside_its_range_is_reported(self):
        warnings = startup_warnings(_WL, {'mpc_w_v': 12.0, 'mpc_w_term': 9.0})
        assert len(warnings) == 1 and 'mpc_w_v' in warnings[0]

    def test_the_shipped_set_today_gives_no_warning_without_reverse(self):
        current = {name: float(get_value(name)) for name in _shipped_whitelist()}
        assert startup_warnings(_shipped_whitelist(), current) == []

    def test_a_w_v_maximum_past_the_reverse_speed_invariant_is_reported(self):
        # reverse_mpc: w_rev >= 2 * w_v * vdes. 4.3 / (2 * 0.5) = 4.3.
        current = {'mpc_w_v': 4.2857, 'mpc_w_term': 9.0}
        assert len(startup_warnings(_WL, current, w_rev=4.3, vdes=0.5)) == 1
        tight = {'mpc_w_v': (1.0, 4.3), 'mpc_w_term': (2.0, 20.0)}
        assert startup_warnings(tight, current, w_rev=4.3, vdes=0.5) == []
