from dataclasses import FrozenInstanceError, fields

import pytest

from flying_car_test.config import load_config
from flying_car_test.models import Action, Phase, Snapshot


def test_confirmed_trajectory_values(config_path):
    cfg = load_config(str(config_path))

    assert cfg.trajectory.ground_forward_m == 5.0
    assert cfg.trajectory.takeoff_agl_m == 4.0
    assert cfg.trajectory.air_forward_m == 5.0
    assert cfg.trajectory.hover_s == 3.0
    assert cfg.trajectory.final_forward_m == 3.0
    assert cfg.trajectory.final_left_m == 3.0


@pytest.mark.parametrize("old,new,key", [
    ("ground_forward_m: 5.0", "ground_forward_m: -1.0", "trajectory.ground_forward_m"),
    ("publish_rate_hz: 30.0", "publish_rate_hz: 19.0", "publish_rate_hz"),
    ("hard_tilt_deg: 45.0", "hard_tilt_deg: 30.0", "safety.hard_tilt_deg"),
])
def test_rejects_invalid_numeric_contract(config_path, tmp_path, old, new, key):
    invalid_config = tmp_path / "experiment.yaml"
    invalid_config.write_text(config_path.read_text().replace(old, new), encoding="utf-8")

    with pytest.raises(ValueError, match=key):
        load_config(str(invalid_config))


def test_rejects_missing_timeout(config_path, tmp_path):
    invalid_config = tmp_path / "experiment.yaml"
    invalid_config.write_text(
        config_path.read_text().replace("  landing_s: 45.0\n", ""), encoding="utf-8"
    )

    with pytest.raises(ValueError, match="timeouts.landing_s"):
        load_config(str(invalid_config))


def test_rejects_non_finite_values(config_path, tmp_path):
    invalid_config = tmp_path / "experiment.yaml"
    invalid_config.write_text(
        config_path.read_text().replace("maximum_agl_m: 12.0", "maximum_agl_m: .nan"),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="safety.maximum_agl_m"):
        load_config(str(invalid_config))


def test_rejects_oversized_integer_with_key_name(config_path, tmp_path):
    invalid_config = tmp_path / "experiment.yaml"
    huge_integer = "9" * 10000
    invalid_config.write_text(
        config_path.read_text().replace("maximum_agl_m: 12.0", "maximum_agl_m: " + huge_integer),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="safety.maximum_agl_m"):
        load_config(str(invalid_config))


def test_rejects_float_overflow_with_key_name():
    class OverflowingNumber:
        def __float__(self):
            raise OverflowError("int too large to convert to float")

    from flying_car_test.config import _number

    with pytest.raises(ValueError, match="safety.maximum_agl_m"):
        _number(
            {"maximum_agl_m": OverflowingNumber()}, "maximum_agl_m", "safety"
        )


def test_phase_members_and_values_match_controller_contract():
    expected = [
        "PRECHECK",
        "GROUND_CONFIGURE",
        "GROUND_FORWARD",
        "GROUND_STOP",
        "FLIGHT_CONFIGURE",
        "OFFBOARD_PRIME",
        "TAKEOFF",
        "AIR_FORWARD",
        "HOVER",
        "MANUAL_AIR",
        "REACQUIRE_OFFBOARD",
        "LAND",
        "GROUND_RECONFIGURE",
        "GROUND_TURN",
        "GROUND_DIAGONAL",
        "COMPLETE",
        "ABORT",
    ]

    assert [member.name for member in Phase] == expected
    assert [member.value for member in Phase] == expected


def test_snapshot_supports_exact_fields_and_is_frozen():
    snapshot = Snapshot(
        stamp=1.0,
        connected=True,
        armed=False,
        mode="OFFBOARD",
        landed=False,
        mav_type=2,
        x=3.0,
        y=4.0,
        z=5.0,
        vx=0.1,
        vy=0.2,
        vz=0.3,
        roll=0.01,
        pitch=0.02,
        yaw=0.03,
        clock_ok=True,
        processes_ok=True,
    )

    assert [field.name for field in fields(Snapshot)] == [
        "stamp",
        "connected",
        "armed",
        "mode",
        "landed",
        "mav_type",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "roll",
        "pitch",
        "yaw",
        "clock_ok",
        "processes_ok",
    ]
    with pytest.raises(FrozenInstanceError):
        snapshot.x = 9.0


def test_action_supports_exact_fields_and_is_frozen():
    action = Action(kind="set_mode", values={"mode": "OFFBOARD"})

    assert [field.name for field in fields(Action)] == ["kind", "values"]
    assert action.kind == "set_mode"
    assert action.values == {"mode": "OFFBOARD"}
    with pytest.raises(FrozenInstanceError):
        action.kind = "arm"
