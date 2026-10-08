"""Camera-pan TF math -- pure, no ROS.

The key guarantee: pan=0 is bit-identical to today's static
base_link -> zed2_camera_link, and a non-zero pan rotates the camera about the
pivot (lens swings the right way).
"""
import math

from f1tenth_camera_pan.frames import (
    base_to_pan_base,
    compose_base_to_camera,
    pan_base_to_camera,
    rotate_point_to_base,
    yaw_to_quat,
)
import pytest

# The sim mount (sim_sensor_mounts.yaml) and the +-15 deg joint limit.
SIM_PIVOT = (0.36, 0.0, 0.25)
MAX_PAN = 0.2618


def test_yaw_quat_identity_and_known_values():
    assert yaw_to_quat(0.0) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = yaw_to_quat(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sin(math.pi / 4))
    assert w == pytest.approx(math.cos(math.pi / 4))


def test_pan_zero_is_bit_identical_to_the_old_static_transform():
    # Old static TF: translation = pivot, rotation = identity.
    trans, quat = compose_base_to_camera(SIM_PIVOT, 0.0)
    assert trans == SIM_PIVOT
    assert quat == (0.0, 0.0, 0.0, 1.0)
    # And the dynamic half alone is identity at pan=0.
    dyn = pan_base_to_camera(0.0)
    assert dyn['translation'] == (0.0, 0.0, 0.0)
    assert dyn['rotation'] == (0.0, 0.0, 0.0, 1.0)
    # Static half is exactly the pivot, identity rotation.
    st = base_to_pan_base(SIM_PIVOT)
    assert st['translation'] == SIM_PIVOT
    assert st['rotation'] == (0.0, 0.0, 0.0, 1.0)


def test_camera_origin_stays_at_the_pivot_for_any_pan():
    # zero translation on the revolute joint => the camera link origin is AT the
    # pivot regardless of angle; only the orientation changes.
    for yaw in (-MAX_PAN, -0.1, 0.0, 0.1, MAX_PAN):
        trans, quat = compose_base_to_camera(SIM_PIVOT, yaw)
        assert trans == SIM_PIVOT
        assert quat == pytest.approx(yaw_to_quat(yaw))


def test_positive_pan_swings_a_forward_point_left():
    # a point 1 m in front of the lens (camera +x) maps left (+y in base_link)
    # for a positive (left) pan, right for a negative pan.
    left = rotate_point_to_base(SIM_PIVOT, +MAX_PAN, (1.0, 0.0, 0.0))
    right = rotate_point_to_base(SIM_PIVOT, -MAX_PAN, (1.0, 0.0, 0.0))
    assert left[1] > SIM_PIVOT[1]      # +y
    assert right[1] < SIM_PIVOT[1]     # -y
    # symmetric about the centreline
    assert left[1] == pytest.approx(-(right[1] - SIM_PIVOT[1]) + SIM_PIVOT[1])


def test_max_pan_rotation_magnitude():
    _, quat = compose_base_to_camera(SIM_PIVOT, MAX_PAN)
    # yaw recovered from quaternion z,w
    yaw = 2.0 * math.atan2(quat[2], quat[3])
    assert yaw == pytest.approx(MAX_PAN)
