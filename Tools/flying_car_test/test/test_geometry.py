import math

import pytest

from flying_car_test.geometry import body_offset_to_enu, enu_to_longitudinal_lateral, quaternion_to_rpy


def test_body_offset_respects_spawn_yaw():
    east, north = body_offset_to_enu(5.0, 0.0, -math.pi / 2)
    assert east == pytest.approx(0.0, abs=1e-6)
    assert north == pytest.approx(-5.0, abs=1e-6)


def test_quaternion_to_rpy_returns_yaw_for_z_axis_rotation():
    roll, pitch, yaw = quaternion_to_rpy(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    assert roll == pytest.approx(0.0, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_enu_projection_returns_forward_and_left_offsets():
    longitudinal, lateral = enu_to_longitudinal_lateral(0.0, -5.0, -math.pi / 2)
    assert longitudinal == pytest.approx(5.0, abs=1e-6)
    assert lateral == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("values", [
    (math.nan, 0.0, 0.0),
    (0.0, math.inf, 0.0),
    (0.0, 0.0, -math.inf),
])
def test_body_offset_rejects_non_finite_inputs(values):
    with pytest.raises(ValueError):
        body_offset_to_enu(*values)
