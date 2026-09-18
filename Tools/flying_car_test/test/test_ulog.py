"""Tests for the ULog loader used by validator evidence extraction.

The production loader accepts a real ``pyulog.ULog`` instance, but the
unit tests inject a fake ``ULog`` object whose ``data`` attribute maps
message names to objects exposing a ``data`` field with the required
numpy arrays. The fake keeps every code path exercised without depending
on the optional pyulog binary fixture.
"""

import math

import numpy as np
import pytest

from flying_car_test.ulog import (
    ActuatorLoadError,
    load_actuator_outputs,
)


class FakeMsg:
    def __init__(self, data):
        self.data = data


class FakeULog:
    def __init__(self, messages):
        self.data = messages


def _make_actuator_msg(samples):
    arrays = {"timestamp": np.arange(samples, dtype=np.float64) * 0.1}
    for index in range(12):
        arrays["output[{}]".format(index)] = np.zeros(samples, dtype=np.float64)
    return FakeMsg(arrays)


def test_load_actuator_outputs_returns_twelve_arrays_and_timestamps():
    ulog = FakeULog({"actuator_outputs": _make_actuator_msg(5)})
    timestamps, outputs = load_actuator_outputs(ulog)
    assert isinstance(timestamps, np.ndarray)
    assert timestamps.shape == (5,)
    assert len(outputs) == 12
    for index, array in enumerate(outputs):
        assert isinstance(array, np.ndarray)
        assert array.shape == (5,)
        assert math.isfinite(float(array[0]))


def test_load_actuator_outputs_missing_message_raises_clear_error():
    ulog = FakeULog({})
    with pytest.raises(ActuatorLoadError, match="actuator_outputs"):
        load_actuator_outputs(ulog)


def test_load_actuator_outputs_missing_output_field_raises_clear_error():
    partial = {"timestamp": np.zeros(3, dtype=np.float64)}
    for index in range(11):
        partial["output[{}]".format(index)] = np.zeros(3, dtype=np.float64)
    ulog = FakeULog({"actuator_outputs": FakeMsg(partial)})
    with pytest.raises(ActuatorLoadError, match=r"output\[11\]"):
        load_actuator_outputs(ulog)


def test_load_actuator_outputs_missing_timestamp_raises_clear_error():
    data = {}
    for index in range(12):
        data["output[{}]".format(index)] = np.zeros(3, dtype=np.float64)
    ulog = FakeULog({"actuator_outputs": FakeMsg(data)})
    with pytest.raises(ActuatorLoadError, match="timestamp"):
        load_actuator_outputs(ulog)


def test_load_actuator_outputs_non_finite_timestamp_raises_value_error():
    data = {"timestamp": np.array([0.0, math.inf, 2.0], dtype=np.float64)}
    for index in range(12):
        data["output[{}]".format(index)] = np.zeros(3, dtype=np.float64)
    ulog = FakeULog({"actuator_outputs": FakeMsg(data)})
    with pytest.raises(ValueError, match="finite"):
        load_actuator_outputs(ulog)


def test_load_actuator_outputs_non_finite_output_raises_value_error():
    data = {"timestamp": np.zeros(3, dtype=np.float64)}
    for index in range(11):
        data["output[{}]".format(index)] = np.zeros(3, dtype=np.float64)
    data["output[11]"] = np.array([0.0, math.nan, 0.0], dtype=np.float64)
    ulog = FakeULog({"actuator_outputs": FakeMsg(data)})
    with pytest.raises(ValueError, match="finite"):
        load_actuator_outputs(ulog)