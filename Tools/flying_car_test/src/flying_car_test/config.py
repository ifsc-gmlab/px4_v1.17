"""Load and validate the flying-car experiment contract."""

import math
from pathlib import Path
from typing import Any, Dict, Type

import yaml

from .models import CommandsConfig, ExperimentConfig, ManualControl, SafetyConfig, TolerancesConfig, TimeoutsConfig, TrajectoryConfig


def _invalid(key: str) -> ValueError:
    return ValueError("invalid configuration key: {}".format(key))


def _mapping(value: Any, key: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise _invalid(key)
    return value


def _number(values: Dict[str, Any], key: str, prefix: str, nonnegative: bool = False) -> float:
    full_key = "{}.{}".format(prefix, key) if prefix else key
    if key not in values or isinstance(values[key], bool):
        raise _invalid(full_key)
    try:
        value = float(values[key])
    except (TypeError, ValueError, OverflowError):
        raise _invalid(full_key)
    if not math.isfinite(value) or (nonnegative and value < 0.0):
        raise _invalid(full_key)
    return value


def _numeric_section(raw: Any, key: str, section_type: Type[Any]) -> Any:
    values = _mapping(raw, key)
    return section_type(**{
        name: _number(values, name, key, nonnegative=True)
        for name in section_type.__dataclass_fields__
    })


def _manual_control(raw: Any, key: str) -> ManualControl:
    values = _mapping(raw, key)
    return ManualControl(**{
        name: _number(values, name, key) for name in ManualControl.__dataclass_fields__
    })


def _commands(raw: Any) -> CommandsConfig:
    values = _mapping(raw, "commands")
    return CommandsConfig(
        manual_neutral=_manual_control(values.get("manual_neutral"), "commands.manual_neutral"),
        ground_neutral=_manual_control(values.get("ground_neutral"), "commands.ground_neutral"),
        ground_forward=_manual_control(values.get("ground_forward"), "commands.ground_forward"),
        ground_left_turn=_manual_control(values.get("ground_left_turn"), "commands.ground_left_turn"),
        air_manual_forward=_manual_control(values.get("air_manual_forward"), "commands.air_manual_forward"),
        air_manual_duration_s=_number(values, "air_manual_duration_s", "commands", nonnegative=True),
        air_manual_brake_s=_number(values, "air_manual_brake_s", "commands", nonnegative=True),
    )


def load_config(path: str) -> ExperimentConfig:
    """Return the immutable configuration loaded from *path*."""
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError("invalid configuration key: config") from error

    root = _mapping(raw, "config")
    publish_rate_hz = _number(root, "publish_rate_hz", "", nonnegative=True)
    if publish_rate_hz < 20.0:
        raise _invalid("publish_rate_hz")
    model_name = root.get("model_name")
    if not isinstance(model_name, str) or not model_name:
        raise _invalid("model_name")
    safety = _numeric_section(root.get("safety"), "safety", SafetyConfig)
    if safety.hard_tilt_deg <= safety.soft_tilt_deg:
        raise _invalid("safety.hard_tilt_deg")
    return ExperimentConfig(
        publish_rate_hz=publish_rate_hz,
        model_name=model_name,
        trajectory=_numeric_section(root.get("trajectory"), "trajectory", TrajectoryConfig),
        tolerances=_numeric_section(root.get("tolerances"), "tolerances", TolerancesConfig),
        timeouts=_numeric_section(root.get("timeouts"), "timeouts", TimeoutsConfig),
        commands=_commands(root.get("commands")),
        safety=safety,
    )
