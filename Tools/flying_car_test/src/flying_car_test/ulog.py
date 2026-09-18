"""Real ULog loader used by the validator.

The loader accepts a real ``pyulog.ULog`` instance. Unit tests inject a
fake ``ULog`` whose ``data`` attribute maps message names to objects with a
``data`` field containing the required numpy arrays so the production code
path is exercised without requiring the optional pyulog binary fixture.

The loader is intentionally minimal: it validates the actuator_outputs
message contract and returns ``(timestamps, [output[0]..output[11]])``.
"""

import math

import numpy as np


ACTUATOR_OUTPUT_KEY = "actuator_outputs"
OUTPUT_FIELD_PREFIX = "output["
OUTPUT_FIELD_SUFFIX = "]"
OUTPUT_FIELD_COUNT = 12
TIMESTAMP_FIELD = "timestamp"


class ActuatorLoadError(RuntimeError):
    """Raised when the actuator_outputs message is missing or malformed."""


def _ensure_finite(name: str, values: np.ndarray) -> None:
    if not isinstance(values, np.ndarray):
        raise ValueError(
            "{!s} field is not a numpy array".format(name)
        )
    if values.dtype.kind == "f" and not np.isfinite(values).all():
        raise ValueError(
            "{!s} contains a non-finite value".format(name)
        )
    if values.dtype.kind != "f":
        converted = values.astype(np.float64)
        if not np.isfinite(converted).all():
            raise ValueError(
                "{!s} contains a non-finite value".format(name)
            )


def load_actuator_outputs(ulog):
    """Return ``(timestamps, [12 outputs])`` from the actuator_outputs message.

    ``ulog`` is duck-typed: it must expose ``data``, a mapping of message
    names to objects exposing a ``data`` field. The ``data`` field must be a
    mapping that includes ``timestamp`` and ``output[0]``..``output[11]``.

    Raises
    ------
    ActuatorLoadError
        When the message is missing or any required field is absent.
    ValueError
        When a value is non-finite.
    """
    if ulog is None or not hasattr(ulog, "data"):
        raise ActuatorLoadError("ULog has no message catalog")
    messages = ulog.data
    if ACTUATOR_OUTPUT_KEY not in messages:
        raise ActuatorLoadError(
            "missing required message: {}".format(ACTUATOR_OUTPUT_KEY)
        )
    message = messages[ACTUATOR_OUTPUT_KEY]
    data = getattr(message, "data", None)
    if not isinstance(data, dict):
        raise ActuatorLoadError(
            "actuator_outputs message exposes no data field"
        )
    if TIMESTAMP_FIELD not in data:
        raise ActuatorLoadError(
            "missing required field: {}".format(TIMESTAMP_FIELD)
        )
    expected_outputs = [
        "{}{}{}".format(OUTPUT_FIELD_PREFIX, index, OUTPUT_FIELD_SUFFIX)
        for index in range(OUTPUT_FIELD_COUNT)
    ]
    for field in expected_outputs:
        if field not in data:
            raise ActuatorLoadError(
                "missing required field: {}".format(field)
            )
    timestamps = np.asarray(data[TIMESTAMP_FIELD])
    outputs = []
    for field in expected_outputs:
        array = np.asarray(data[field])
        outputs.append(array)
    _ensure_finite(TIMESTAMP_FIELD, timestamps)
    for field, array in zip(expected_outputs, outputs):
        _ensure_finite(field, array)
    if timestamps.size and not math.isfinite(float(timestamps[0])):
        raise ValueError("timestamp contains a non-finite value")
    return timestamps, outputs