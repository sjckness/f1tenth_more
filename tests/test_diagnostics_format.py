"""Change 1: floats must survive the diagnostics wire format exactly.

If they do not, ``replay`` is approximately identical rather than
bit-identical, and ``compare``'s zero-tolerance check reports serialization
artifacts as node bugs.
"""

import math
import re
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

from go_to_object.diagnostics_format import (
    NONE,
    format_float,
    format_optional_float,
    parse_float,
    parse_optional_float,
)

FULL_MANTISSA = 0.12345678901234567
ROS_STAMP = 1789459839.1234567

ADVERSARIAL = [
    pytest.param(0.1, id='0.1'),
    pytest.param(1.0 / 3.0, id='one-third'),
    pytest.param(sys.float_info.min, id='smallest-normal'),
    pytest.param(sys.float_info.max, id='largest-finite'),
    pytest.param(5e-324, id='smallest-subnormal'),
    pytest.param(sys.float_info.epsilon, id='epsilon'),
    pytest.param(FULL_MANTISSA, id='full-17-digit-mantissa'),
    pytest.param(-0.0, id='negative-zero'),
    pytest.param(0.0, id='zero'),
    pytest.param(ROS_STAMP, id='ros-timestamp'),
    pytest.param(-ROS_STAMP, id='negative-ros-timestamp'),
    pytest.param(math.pi, id='pi'),
    pytest.param(-1.7976931348623157e308, id='most-negative-finite'),
]


def _bits(value: float) -> bytes:
    """The actual 64 bits, so 'equal' means equal and not merely close."""
    return struct.pack('<d', value)


@pytest.mark.parametrize('value', ADVERSARIAL)
def test_every_adversarial_value_round_trips_bit_for_bit(value):
    assert _bits(parse_float(format_float(value))) == _bits(value)


def test_negative_zero_keeps_its_sign_bit():
    """``-0.0 == 0.0``, so equality alone would not catch losing the sign."""
    recovered = parse_float(format_float(-0.0))

    assert math.copysign(1.0, recovered) == -1.0
    assert _bits(recovered) == _bits(-0.0)


def test_a_realistic_timestamp_sequence_keeps_its_low_bits():
    """The case most likely to lose precision: large magnitude, small steps.

    A ROS stamp near 1.79e9 has about 2e-7 s of resolution left in a double,
    so a 10 ms tick is only ~5 significant digits above the noise floor.
    Anything short of full precision collapses neighbouring ticks together.
    """
    stamps = [ROS_STAMP + i * 0.01 for i in range(200)]
    recovered = [parse_float(format_float(s)) for s in stamps]

    assert [_bits(r) for r in recovered] == [_bits(s) for s in stamps]
    assert len(set(recovered)) == len(stamps), 'no two ticks collapsed together'
    assert all(b > a for a, b in zip(recovered, recovered[1:]))


def test_the_shortest_repr_is_still_exact():
    """``repr`` is shorter than ``%.17g`` and no less exact."""
    for value in (0.1, ROS_STAMP, FULL_MANTISSA):
        assert len(format_float(value)) <= len('%.17g' % value)
        assert _bits(parse_float('%.17g' % value)) == _bits(parse_float(format_float(value)))


@pytest.mark.parametrize('spec,value,lossy', [
    ('%.4f', ROS_STAMP, True), ('%.4f', 2 / 3, True),
    ('%.6f', ROS_STAMP, True), ('%.6f', 2 / 3, True),
    # The trap: a spec that is exact at one magnitude and lossy at another.
    ('%.7f', ROS_STAMP, False), ('%.7f', 2 / 3, True),
    ('%.16g', ROS_STAMP, True), ('%.16g', 2 / 3, False),
    ('%.17g', ROS_STAMP, False), ('%.17g', 2 / 3, False),
])
def test_no_fixed_format_spec_is_safe_across_both_magnitudes(spec, value, lossy):
    """Evidence the requirement is real, and why it cannot be satisfied by
    "enough" decimal places.

    Losslessness of ``%.Nf`` depends on the magnitude of the value, and this
    system carries two that are nine orders apart: ROS timestamps near
    1.79e9 and curvatures near 1. ``%.7f`` round-trips the timestamp exactly
    and loses the curvature; ``%.16g`` does the reverse. Only ``%.17g`` --
    and ``repr``, which is shorter -- is exact for both, which is why the
    formatter takes no format spec at all.
    """
    assert (_bits(float(spec % value)) != _bits(value)) is lossy


@pytest.mark.parametrize('value', [np.float64(0.1), np.float32(0.25),
                                   np.float64(ROS_STAMP)])
def test_numpy_scalars_are_coerced_before_serialization(value):
    """numpy 2.x reprs scalars as ``np.float64(0.1)``, which will not parse."""
    text = format_float(value)

    assert not text.startswith('np.')
    assert _bits(parse_float(text)) == _bits(float(value))


def test_optional_values_round_trip_including_the_sentinel():
    assert format_optional_float(None) == NONE
    assert parse_optional_float(NONE) is None
    assert parse_optional_float('') is None
    assert _bits(parse_optional_float(format_optional_float(ROS_STAMP))) == _bits(ROS_STAMP)


def test_nan_is_reproduced_faithfully_even_though_equality_fails():
    """Stated rather than rounded away: this one cannot be compared by ``==``.

    ``repr(nan)`` -> ``'nan'`` -> ``float('nan')`` reproduces a NaN, so no
    precision is lost. But IEEE-754 says ``nan != nan``, so ``compare`` will
    always flag a NaN tick. That is a property of NaN, not of this format,
    and a NaN reaching diagnostics is itself the bug worth chasing.
    """
    recovered = parse_float(format_float(float('nan')))

    assert math.isnan(recovered)
    assert recovered != recovered


@pytest.mark.parametrize('value', [float('inf'), float('-inf')])
def test_infinities_round_trip(value):
    assert parse_float(format_float(value)) == value


def test_no_format_spec_has_crept_into_the_diagnostics_values():
    """Source-level guard against the regression the requirement names.

    The risk is someone adding ``:.3f`` to a diagnostics value to make
    ``ros2 topic echo`` readable. Reading the node as text keeps this test
    free of an rclpy import.
    """
    source = (Path(__file__).resolve().parents[1]
              / 'go_to_object' / 'mission_node.py').read_text()
    block = source[source.index("values = {"):source.index("status = DiagnosticStatus()")]

    assert 'format_float' in block
    assert not re.search(r':\s*\.\d+[efg]', block), 'format spec in a diagnostics value'
    assert not re.search(r'%\.\d+[efg]', block)
    assert 'repr(' not in block, 'go through format_float so numpy is coerced'
