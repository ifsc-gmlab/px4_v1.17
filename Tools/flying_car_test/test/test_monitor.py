import json
import threading
from types import SimpleNamespace

import pytest

import flying_car_test.monitor as monitor_module
from flying_car_test.monitor import GAZEBO_MODEL_NAME, PassiveMonitor
from flying_car_test.recording import EvidenceRecorder


def point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def pose(x, y, z):
    return SimpleNamespace(
        position=point(x, y, z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def mavlink_heartbeat(
    mav_type,
    stamp,
    framing_status=1,
    length=9,
    msgid=0,
    sysid=1,
    compid=1,
    payload=True,
):
    # HEARTBEAT: custom_mode:u32, type:u8, autopilot:u8,
    # base_mode:u8, system_status:u8, mavlink_version:u8.
    word = (
        (int(mav_type) << 32)
        | (12 << 40)
        | (81 << 48)
        | (4 << 56)
    )
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda: float(stamp))
        ),
        framing_status=framing_status,
        len=length,
        sysid=sysid,
        compid=compid,
        msgid=msgid,
        payload64=[word, 3] if payload else [],
    )


def test_gazebo_lookup_uses_exact_model_name(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_model_states(SimpleNamespace(
        name=["prefix-" + GAZEBO_MODEL_NAME, GAZEBO_MODEL_NAME],
        pose=[pose(1.0, 2.0, 3.0), pose(7.0, 8.0, 9.0)],
    ))

    assert monitor.sample(12.0)["gazebo"] == {
        "x": 7.0, "y": 8.0, "z": 9.0
    }
    recorder.close(clean=False, abort_reason="test")


def test_clock_backward_movement_latches_clock_not_ok(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_clock(SimpleNamespace(clock=SimpleNamespace(to_sec=lambda: 10.0)))
    assert monitor.sample(10.0)["clock_ok"] is True
    monitor.on_clock(SimpleNamespace(clock=SimpleNamespace(to_sec=lambda: 9.5)))
    assert monitor.sample(10.1)["clock_ok"] is False
    monitor.on_clock(SimpleNamespace(clock=SimpleNamespace(to_sec=lambda: 11.0)))
    assert monitor.sample(10.2)["clock_ok"] is False
    recorder.close(clean=False, abort_reason="test")


def test_sample_aggregates_state_pose_velocity_attitude_and_truth(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_state(SimpleNamespace(connected=True, armed=False, mode="MANUAL"))
    monitor.on_extended_state(SimpleNamespace(landed_state=1))
    monitor.on_local_pose(SimpleNamespace(pose=pose(1.0, 2.0, 3.0)))
    monitor.on_local_velocity(SimpleNamespace(
        twist=SimpleNamespace(linear=point(4.0, 5.0, 6.0))
    ))
    monitor.on_model_states(SimpleNamespace(
        name=[GAZEBO_MODEL_NAME],
        pose=[pose(7.0, 8.0, 9.0)],
    ))
    monitor.on_clock(SimpleNamespace(clock=SimpleNamespace(to_sec=lambda: 4.5)))

    assert monitor.sample(5.0) == {
        "stamp": 5.0,
        "clock": 4.5,
        "connected": True,
        "armed": False,
        "mode": "MANUAL",
        "landed": True,
        "mav_type": None,
        "mav_type_fresh": False,
        "local": {"x": 1.0, "y": 2.0, "z": 3.0},
        "velocity": {"x": 4.0, "y": 5.0, "z": 6.0},
        "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "gazebo": {"x": 7.0, "y": 8.0, "z": 9.0},
        "clock_ok": True,
    }
    recorder.close(clean=False, abort_reason="test")


def test_phase_command_and_abort_are_recorded_passively(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_phase(SimpleNamespace(data="PRECHECK"))
    monitor.on_command(SimpleNamespace(
        data='{"kind":"manual","stamp":1.0,"values":{"x":0.0}}'
    ))
    monitor.on_abort(SimpleNamespace(
        data='{"reason":"first","stamp":2.0}'
    ))
    monitor.on_abort(SimpleNamespace(
        data='{"reason":"second","stamp":3.0}'
    ))
    monitor.close()

    assert recorder.abort_reason == "first"
    assert (tmp_path / "run" / "events.jsonl.part").is_file()
    assert (tmp_path / "run" / "commands.jsonl.part").is_file()


def test_complete_phase_closes_cleanly_and_late_callbacks_are_ignored(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_phase(SimpleNamespace(data="COMPLETE"))
    monitor.close()

    monitor.on_command(SimpleNamespace(data='{"stamp":3.0}'))
    monitor.on_phase(SimpleNamespace(data="COMPLETE"))
    monitor.record_sample(4.0)
    assert (tmp_path / "run" / "samples.jsonl").is_file()
    assert not list((tmp_path / "run").glob("*.part"))


def test_heartbeat_tracks_real_mav_type_transitions_and_freshness(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    assert monitor.sample(0.0)["mav_type"] is None
    assert monitor.sample(0.0)["mav_type_fresh"] is False

    for stamp, mav_type in ((1.0, 10), (2.0, 14), (3.0, 10)):
        monitor.on_mavlink(mavlink_heartbeat(mav_type, stamp))
        captured = monitor.sample(stamp + 0.1)
        assert captured["mav_type"] == mav_type
        assert captured["mav_type_fresh"] is True

    assert monitor.sample(6.0)["mav_type"] == 10
    assert monitor.sample(6.0)["mav_type_fresh"] is False
    recorder.close(clean=False, abort_reason="test")


@pytest.mark.parametrize(
    "changes",
    [
        {"framing_status": 2},
        {"length": 8},
        {"msgid": 1},
        {"sysid": 2},
        {"compid": 2},
        {"payload": False},
    ],
)
def test_malformed_or_unrelated_mavlink_does_not_pollute_type(
    tmp_path, changes
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_mavlink(mavlink_heartbeat(10, 1.0))
    monitor.on_mavlink(mavlink_heartbeat(14, 2.0, **changes))

    captured = monitor.sample(2.1)
    assert captured["mav_type"] == 10
    assert captured["mav_type_fresh"] is True
    recorder.close(clean=False, abort_reason="test")


class FailingSampleRecorder:
    closed = False
    abort_reason = None

    def append_sample(self, _sample):
        raise OSError("disk full")

    def abort(self, reason):
        self.abort_reason = str(reason)
        return self.abort_reason


def test_timer_append_failure_latches_abort_and_requests_shutdown():
    recorder = FailingSampleRecorder()
    monitor = PassiveMonitor(recorder)
    shutdown_reasons = []

    assert monitor.record_sample_safely(
        1.0, shutdown_reasons.append
    ) is False
    assert "sample recording failed" in recorder.abort_reason
    assert shutdown_reasons == [recorder.abort_reason]


def test_abort_already_in_callback_cannot_race_clean_close(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    monitor = PassiveMonitor(recorder)
    monitor.on_phase(SimpleNamespace(data="COMPLETE"))
    entered = threading.Event()
    release = threading.Event()
    real_decode = monitor_module._decoded_object

    def blocked_decode(message, fallback_key="raw"):
        entered.set()
        release.wait(timeout=2.0)
        return real_decode(message, fallback_key)

    monkeypatch.setattr(monitor_module, "_decoded_object", blocked_decode)
    abort_thread = threading.Thread(
        target=monitor.on_abort,
        args=(SimpleNamespace(data='{"reason":"late abort"}'),),
    )
    abort_thread.start()
    assert entered.wait(timeout=1.0)
    close_thread = threading.Thread(target=monitor.close)
    close_thread.start()
    release.set()
    abort_thread.join(timeout=2.0)
    close_thread.join(timeout=2.0)

    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is False
    assert complete["abort_reason"] == "late abort"
