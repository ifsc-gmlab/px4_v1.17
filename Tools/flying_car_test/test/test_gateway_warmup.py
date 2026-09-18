"""Unit tests for Px4Gateway.warm_up_params() and heartbeat MAV_TYPE."""

import sys
import threading
import types
from types import SimpleNamespace

import pytest


def _install_fake_mavros_msgs():
    """Provide mavros_msgs.srv without requiring a sourced ROS env."""
    pkg = types.ModuleType("mavros_msgs")
    srv = types.ModuleType("mavros_msgs.srv")
    srv.ParamPull = type("ParamPull", (), {})
    pkg.srv = srv
    sys.modules["mavros_msgs"] = pkg
    sys.modules["mavros_msgs.srv"] = srv


_install_fake_mavros_msgs()

from flying_car_test.px4_gateway import Px4Gateway  # noqa: E402


class _FakeRos:
    def __init__(self):
        self.sleep_calls = 0
        self.pull_calls = 0
        self.Time = SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 123.0)
        )

    def wait_for_service(self, *_args, **_kwargs):
        return None

    def ServiceProxy(self, *_args, **_kwargs):
        def pull(**_kw):
            self.pull_calls += 1
            return SimpleNamespace(success=True)

        return pull

    def sleep(self, _seconds):
        self.sleep_calls += 1


class _ParamGetResponse:
    success = True
    value = SimpleNamespace(integer=14, real=0.0)


def _gateway(fake_ros, connected=True):
    gateway = Px4Gateway.__new__(Px4Gateway)
    gateway._ros = fake_ros
    fake_ros.gateway = gateway
    gateway._parameter_lock = threading.RLock()
    gateway._operation_responses = {}
    gateway.last_response_fields = {}
    gateway._mav_type = None
    gateway._state = (
        SimpleNamespace(connected=True) if connected else None
    )
    gateway._mavlink_framing_ok = 1
    gateway._mavlink_heartbeat_msgid = 0
    gateway._mavlink_heartbeat_min_length = 9
    gateway._system_id = 1
    gateway._autopilot_component_id = 1
    gateway._feedback_stamps = {}
    gateway._param_get = lambda **_kw: _ParamGetResponse()
    gateway._bindings = SimpleNamespace(
        ParamValue=lambda **_kw: SimpleNamespace(
            **{"integer": 0, "real": 0.0}
        )
    )
    return gateway


def test_warm_up_params_returns_mav_type():
    fake = _FakeRos()
    gateway = _gateway(fake)
    assert gateway.warm_up_params(timeout_s=5.0) == 14
    assert fake.pull_calls >= 1


def test_warm_up_params_times_out_without_connection():
    fake = _FakeRos()
    gateway = _gateway(fake, connected=False)
    with pytest.raises(RuntimeError, match="MAVROS link not established"):
        gateway.warm_up_params(timeout_s=0.5)


def test_warm_up_params_keeps_pulling_until_success():
    fake = _FakeRos()
    gateway = _gateway(fake)
    original = gateway.get_mav_type

    calls = {"n": 0}

    def flaky():
        calls["n"] += 1
        if calls["n"] < 3:
            raise RuntimeError("MAV_TYPE ParamGet was rejected")
        return original()

    gateway.get_mav_type = flaky
    assert gateway.warm_up_params(timeout_s=5.0) == 14
    assert calls["n"] >= 3
    assert fake.sleep_calls >= 2


def test_on_mavlink_parses_heartbeat_type():
    fake = _FakeRos()
    gateway = _gateway(fake)
    message = SimpleNamespace(
        framing_status=1,
        msgid=0,
        len=9,
        sysid=1,
        compid=1,
        payload64=[(14 << 32) | 12345] + [0] * 7,
        header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda: 1.0)
        ),
    )
    gateway._on_mavlink(message)
    assert gateway._mav_type == 14
    assert "mav_type" in gateway._feedback_stamps


def test_on_mavlink_ignores_non_heartbeat_or_wrong_system():
    fake = _FakeRos()
    gateway = _gateway(fake)
    base = dict(
        framing_status=1,
        msgid=0,
        len=9,
        sysid=2,
        compid=1,
        payload64=[(14 << 32) | 12345] + [0] * 7,
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 1.0)),
    )
    gateway._on_mavlink(SimpleNamespace(**base))
    assert gateway._mav_type is None
    gateway._on_mavlink(
        SimpleNamespace(**{**base, "sysid": 1, "msgid": 30})
    )
    assert gateway._mav_type is None


if __name__ == "__main__":
    import sys as _sys

    _sys.exit(pytest.main([__file__, "-q"]))
