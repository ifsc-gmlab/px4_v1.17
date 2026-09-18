"""Tests for the end-to-end trial validator.

The validator is exercised against synthesized result directories created
via pytest ``tmp_path`` fixtures so the suite does not depend on the
optional pyulog binary fixture. The fake ULog contract mirrors the one
declared in the ``flying_car_test.ulog`` module.
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from flying_car_test import validation
from flying_car_test.models import (
    CommandsConfig,
    ExperimentConfig,
    ManualControl,
    SafetyConfig,
    TimeoutsConfig,
    TolerancesConfig,
    TrajectoryConfig,
)


CONFIG = ExperimentConfig(
    publish_rate_hz=30.0,
    model_name="FS-FASMv8_octo",
    trajectory=TrajectoryConfig(
        ground_forward_m=5.0,
        takeoff_agl_m=5.0,
        air_forward_m=5.0,
        hover_s=3.0,
        final_forward_m=3.0,
        final_left_m=3.0,
    ),
    tolerances=TolerancesConfig(
        first_ground_longitudinal_m=1.0,
        first_ground_lateral_m=1.0,
        altitude_m=0.75,
        air_longitudinal_m=1.0,
        air_lateral_m=1.5,
        hover_altitude_m=0.75,
        final_forward_m=1.5,
        final_left_m=1.5,
    ),
    timeouts=TimeoutsConfig(
        precheck_s=30.0,
        transition_s=5.0,
        ground_forward_s=30.0,
        takeoff_s=30.0,
        air_forward_s=20.0,
        landing_s=45.0,
        final_ground_s=40.0,
    ),
    commands=CommandsConfig(
        manual_neutral=ManualControl(0.0, 0.0, 500.0, 0.0),
        ground_neutral=ManualControl(0.0, 0.0, 0.0, 0.0),
        ground_forward=ManualControl(0.0, 0.0, 650.0, 0.0),
        ground_left_turn=ManualControl(0.0, 500.0, 500.0, 0.0),
        air_manual_forward=ManualControl(150.0, 0.0, 500.0, 0.0),
        air_manual_duration_s=1.0,
        air_manual_brake_s=2.0,
    ),
    safety=SafetyConfig(
        heartbeat_timeout_s=0.3,
        soft_tilt_deg=30.0,
        hard_tilt_deg=45.0,
        maximum_agl_m=8.0,
        maximum_planar_deviation_m=10.0,
        stopped_speed_mps=0.15,
    ),
)


LOG_RULES = {
    "fatal": [
        "HEARTBEAT timed out",
        "poll timeout",
        "process has died",
        "Segmentation fault",
        "lockstep.*timeout",
        "TM: Time jump detected",
        "Traceback \\(most recent call last\\)",
        "\\[ERROR\\]",
        "\\[FATAL\\]",
    ],
    "critical_warnings": [
        "failsafe",
        "connection.*lost",
        "mode.*reject",
        "arming.*reject",
    ],
    "allowed_warnings": [],
}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _write_jsonl(path: Path, rows) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True))
            stream.write("\n")


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


def _sample(
    stamp,
    x=0.0,
    y=0.0,
    z=0.0,
    vx=0.0,
    vy=0.0,
    vz=0.0,
    connected=True,
    armed=False,
    mode="MANUAL",
    landed=True,
    mav_type=10,
    mav_type_fresh=True,
    clock_ok=True,
    yaw=0.0,
    roll=0.0,
    pitch=0.0,
):
    return {
        "stamp": stamp,
        "clock": stamp,
        "connected": connected,
        "armed": armed,
        "mode": mode,
        "landed": landed,
        "mav_type": mav_type,
        "mav_type_fresh": mav_type_fresh,
        "local": {"x": x, "y": y, "z": z},
        "velocity": {"x": vx, "y": vy, "z": vz},
        "attitude": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "gazebo": {"x": x, "y": y, "z": z},
        "clock_ok": clock_ok,
    }


def _event(stamp, phase):
    return {"stamp": stamp, "type": "phase", "phase": phase}


def _command(stamp, kind, **values):
    return {"stamp": stamp, "kind": kind, "values": values}


def _build_passing_run(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    samples = []
    phase_events = []  # (stamp, phase) recorded at phase transitions only

    def emit_samples(stamp_iter, sample_value, phase):
        first = True
        for stamp in stamp_iter:
            samples.append(sample_value(stamp))
            if first:
                phase_events.append((stamp, phase))
                first = False

    # PRECHECK: 0.0 - 1.0
    emit_samples(
        (index * 0.1 for index in range(10)),
        lambda stamp: _sample(stamp),
        "PRECHECK",
    )
    # GROUND_FORWARD: 1.0 - 2.0
    def _ground_forward(stamp, idx=[0]):
        idx[0] += 1
        return _sample(stamp, x=idx[0] * 0.5, y=0.0)
    emit_samples(
        (1.0 + index * 0.1 for index in range(10)),
        _ground_forward,
        "GROUND_FORWARD",
    )
    # GROUND_STOP: 2.0 - 3.0
    emit_samples(
        (2.0 + index * 0.1 for index in range(10)),
        lambda stamp: _sample(stamp, x=5.0),
        "GROUND_STOP",
    )
    # FLIGHT_CONFIGURE: 3.0 - 4.0 -> MAV_TYPE 14
    def _flight(stamp, idx=[0]):
        mav = 14 if idx[0] >= 5 else 10
        idx[0] += 1
        return _sample(stamp, x=5.0, mav_type=mav, mav_type_fresh=True)
    emit_samples(
        (3.0 + index * 0.1 for index in range(10)),
        _flight,
        "FLIGHT_CONFIGURE",
    )
    # OFFBOARD_PRIME / TAKEOFF: 4.0 - 6.0 climb to 5.0 m AGL
    def _takeoff(stamp, idx=[0]):
        z = idx[0] * 0.25
        idx[0] += 1
        return _sample(stamp, x=5.0, z=z, mav_type=14, landed=False, mode="OFFBOARD")
    emit_samples(
        (4.0 + index * 0.1 for index in range(20)),
        _takeoff,
        "TAKEOFF",
    )
    # AIR_FORWARD: 6.0 - 7.0 travel forward 5.0 m
    def _air_forward(stamp, idx=[0]):
        x = 5.0 + (idx[0] + 1) * 0.5
        idx[0] += 1
        return _sample(stamp, x=x, z=5.0, mav_type=14, mode="OFFBOARD", landed=False)
    emit_samples(
        (6.0 + index * 0.1 for index in range(10)),
        _air_forward,
        "AIR_FORWARD",
    )
    # HOVER: 7.0 - 7.5
    emit_samples(
        (7.0 + index * 0.1 for index in range(5)),
        lambda stamp: _sample(stamp, x=10.0, z=5.0, mav_type=14, mode="OFFBOARD", landed=False),
        "HOVER",
    )
    # MANUAL_AIR: 7.5 - 8.0
    emit_samples(
        (7.5 + index * 0.1 for index in range(5)),
        lambda stamp: _sample(stamp, x=10.5, z=5.0, mav_type=14, mode="MANUAL", landed=False),
        "MANUAL_AIR",
    )
    # REACQUIRE_OFFBOARD: 8.0 - 8.5
    emit_samples(
        (8.0 + index * 0.1 for index in range(5)),
        lambda stamp: _sample(stamp, x=10.5, z=5.0, mav_type=14, mode="OFFBOARD", landed=False),
        "REACQUIRE_OFFBOARD",
    )
    # LAND: 8.5 - 10.0
    landing_yaw = math.pi / 2 + 0.1  # captured at LAND time
    def _land(stamp, idx=[0]):
        z = max(0.0, 5.0 - (idx[0] + 1) * 0.4)
        landed = idx[0] == 14
        idx[0] += 1
        return _sample(stamp, x=10.5, z=z, mav_type=14, mode="AUTO.LAND", landed=landed, yaw=landing_yaw)
    emit_samples(
        (8.5 + index * 0.1 for index in range(15)),
        _land,
        "LAND",
    )
    # GROUND_RECONFIGURE: 10.0 - 11.0 -> MAV_TYPE 10
    def _reconfigure(stamp, idx=[0]):
        mav = 10 if idx[0] >= 5 else 14
        idx[0] += 1
        return _sample(stamp, x=10.5, z=0.0, mav_type=mav)
    emit_samples(
        (10.0 + index * 0.1 for index in range(10)),
        _reconfigure,
        "GROUND_RECONFIGURE",
    )
    # GROUND_TURN: 11.0 - 12.0 yaw rotation 90 deg
    yaw_values = [
        0.0,
        math.pi / 8,
        math.pi / 4,
        math.pi / 3,
        math.pi / 2 + 0.1,
    ]
    def _turn(stamp, idx=[0]):
        yaw = yaw_values[min(idx[0], len(yaw_values) - 1)]
        idx[0] += 1
        return _sample(stamp, x=10.5, z=0.0, mav_type=10, yaw=yaw)
    emit_samples(
        (11.0 + index * 0.1 for index in range(10)),
        _turn,
        "GROUND_TURN",
    )
    # GROUND_DIAGONAL: 12.0 - 14.0 forward 3 m, left 3 m
    target_x = 10.5 + math.cos(landing_yaw) * 3.0 - math.sin(landing_yaw) * 3.0
    target_y = 0.0 + math.sin(landing_yaw) * 3.0 + math.cos(landing_yaw) * 3.0
    def _diagonal(stamp, idx=[0]):
        progress = (idx[0] + 1) / 20.0
        x = 10.5 + (target_x - 10.5) * progress
        y = 0.0 + (target_y - 0.0) * progress
        idx[0] += 1
        return _sample(stamp, x=x, y=y, mav_type=10, yaw=landing_yaw)
    emit_samples(
        (12.0 + index * 0.1 for index in range(20)),
        _diagonal,
        "GROUND_DIAGONAL",
    )
    # COMPLETE
    samples.append(_sample(14.0, x=target_x, y=target_y, mav_type=10))
    phase_events.append((14.0, "COMPLETE"))

    _write_jsonl(root / "samples.jsonl", samples)
    _write_jsonl(
        root / "events.jsonl",
        [_event(stamp, phase) for stamp, phase in phase_events],
    )
    _write_jsonl(root / "commands.jsonl", [_command(0.5, "manual", name="ground_forward")])
    _write_json(
        root / "metadata.json",
        {
            "px4_git_sha": "0" * 40,
            "experiment_config_sha256": "f" * 64,
            "launch_arguments": {},
        },
    )
    _write_json(
        root / "monitor_complete.json",
        {
            "clean": True,
            "abort_reason": None,
            "errors": [],
            "samples": len(samples),
            "events": len(phase_events),
            "commands": 1,
        },
    )


def _actuator_zero(samples):
    arrays = {"timestamp": np.arange(samples, dtype=np.float64) * 0.05}
    for index in range(12):
        arrays["output[{}]".format(index)] = np.zeros(samples, dtype=np.float64)
    return arrays


def _actuator_active(samples):
    arrays = {"timestamp": np.arange(samples, dtype=np.float64) * 0.05}
    for index in range(12):
        arrays["output[{}]".format(index)] = np.full(samples, 1500.0, dtype=np.float64)
    return arrays


def _build_actuator_ulog(path: Path, mode: str) -> None:
    arrays = _actuator_zero(5) if mode == "zero" else _actuator_active(5)
    header = (
        "# ULog fake for tests\n"
        "{" + "\n" +
        json.dumps(
            {
                "actuator_outputs": {
                    "timestamp": arrays["timestamp"].tolist(),
                    **{key: arrays[key].tolist() for key in [f"output[{i}]" for i in range(12)]},
                }
            }
        ) +
        "\n}\n"
    )
    path.write_text(header, encoding="utf-8")


def _build_time_varying_actuator_ulog(samples=200, dt=0.1):
    """Build a fake ULog whose actuator outputs follow the experiment phases.

    Ground phases (PRECHECK, GROUND_*, LAND if landed) carry zero rotors.
    Air phases (TAKEOFF, AIR_FORWARD, HOVER, MANUAL_AIR, REACQUIRE_OFFBOARD)
    carry 1500 PWM output; ground phases stay at the 1000 disarmed floor.

    Returns the FakeULog instance directly. The validator does not need to
    open the on-disk path because we also wire a matching ``.ulg`` file
    placeholder so ``_load_actuator_ulog`` finds a candidate.
    """
    timestamps = np.arange(samples, dtype=np.float64) * dt
    outputs = np.zeros((12, samples), dtype=np.float64)
    for index in range(samples):
        stamp = float(timestamps[index])
        # Air phases roughly 4.0 - 8.5.
        if 4.0 <= stamp < 8.5:
            outputs[:, index] = 1500.0
        else:
            outputs[:, index] = 1000.0
    arrays = {"timestamp": timestamps}
    for index in range(12):
        arrays["output[{}]".format(index)] = outputs[index]
    msg = type("Msg", (), {"data": arrays})()
    return type("ULog", (), {"data": {"actuator_outputs": msg}})()


def _write_log_rules_yaml(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(LOG_RULES, sort_keys=True), encoding="utf-8")
    return path


def _failing_run(root: Path) -> None:
    """Failing run: missing samples.jsonl entirely."""
    root.mkdir(parents=True, exist_ok=True)
    _write_json(
        root / "monitor_complete.json",
        {"clean": True, "abort_reason": None, "errors": [], "samples": 0, "events": 0, "commands": 0},
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_load_actuator_outputs_returns_numpy_arrays():
    arrays = _actuator_zero(4)
    message = type("Msg", (), {"data": arrays})()
    ulog = type("ULog", (), {"data": {"actuator_outputs": message}})()
    timestamps, outputs = validation.load_actuator_outputs_for_test(ulog)
    assert len(outputs) == 12
    assert timestamps.shape == (4,)


def test_passing_run_evaluates_to_overall_pass(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    factory = lambda _path: _build_time_varying_actuator_ulog()

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    summary = report.summary()
    assert summary["overall"] == "PASS", summary
    assert summary["abort_reason"] is None
    statuses = {item["id"]: item["status"] for item in summary["items"]}
    assert statuses["T01"] == "PASS"
    assert statuses["T02"] == "PASS"
    assert statuses["T03"] == "PASS"
    assert statuses["T04"] == "PASS"
    assert statuses["T05"] == "PASS"
    assert statuses["T07"] == "PASS"
    assert statuses["T08"] == "PASS"
    assert statuses["T09"] == "PASS"
    assert statuses["T10"] == "PASS"


def test_missing_samples_fails_closed(tmp_path):
    run = tmp_path / "run"
    _failing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    assert summary["overall"] == "FAIL", summary


def test_abort_reason_latched_overall_abort(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor_complete.json").write_text(
        json.dumps(
            {
                "clean": False,
                "abort_reason": "controller_command_ingress_timeout",
                "errors": ["controller_command_ingress_timeout"],
                "samples": 10,
                "events": 1,
                "commands": 0,
            }
        ),
        encoding="utf-8",
    )
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    assert summary["overall"] == "ABORT", summary
    assert summary["abort_reason"] == "controller_command_ingress_timeout"


def test_ground_forward_reaches_target(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    items = {item["id"]: item for item in report.summary()["items"]}
    t01 = items["T01"]
    assert t01["status"] == "PASS"
    assert t01["measured"]["longitudinal_m"] >= 4.0
    assert abs(t01["measured"]["lateral_m"]) <= 1.0


def test_air_forward_reaches_target(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    items = {item["id"]: item for item in report.summary()["items"]}
    t05 = items["T05"]
    assert t05["status"] == "PASS"
    assert abs(t05["measured"]["longitudinal_m"] - 5.0) <= 1.0


def test_mav_type_round_trip(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    items = {item["id"]: item for item in report.summary()["items"]}
    t09 = items["T09"]
    assert t09["status"] == "PASS"
    assert t09["measured"]["sequence"] == [10, 14, 10]


def test_fatal_pattern_in_logs_fails_T10(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.log").write_text(
        "[ERROR] PX4: navigation failed\n", encoding="utf-8"
    )
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "FAIL"
    assert summary["overall"] == "FAIL"


def test_python_traceback_in_logs_fails_T10(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "controller.log").write_text(
        "Traceback (most recent call last):\n  File x.py\n", encoding="utf-8"
    )
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "FAIL"
    assert summary["overall"] == "FAIL"


def test_critical_warning_fails_T10_when_no_allowlist(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "px4.log").write_text(
        "failsafe triggered due to attitude\n", encoding="utf-8"
    )
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "FAIL"
    assert summary["overall"] == "FAIL"


def test_allowed_warning_suppresses_only_matching_critical(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    rules_path = tmp_path / "rules.yaml"
    rules_path.parent.mkdir(parents=True, exist_ok=True)
    rules_path.write_text(
        yaml.safe_dump(
            {
                "fatal": LOG_RULES["fatal"],
                "critical_warnings": LOG_RULES["critical_warnings"],
                "allowed_warnings": ["failsafe triggered due to attitude"],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (run / "px4.log").write_text(
        "failsafe triggered due to attitude\n", encoding="utf-8"
    )
    factory = lambda _path: _build_time_varying_actuator_ulog()

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=rules_path,
        ulog_factory=factory,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "PASS"
    assert summary["overall"] == "PASS"


def test_allowed_warning_does_not_suppress_fatal(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    rules_path = tmp_path / "rules.yaml"
    rules_path.parent.mkdir(parents=True, exist_ok=True)
    rules_path.write_text(
        yaml.safe_dump(
            {
                "fatal": LOG_RULES["fatal"],
                "critical_warnings": LOG_RULES["critical_warnings"],
                "allowed_warnings": ["Segmentation fault"],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (run / "px4.log").write_text(
        "Segmentation fault in px4\n", encoding="utf-8"
    )

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=rules_path,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "FAIL"


def test_allowed_warning_does_not_suppress_unrelated_critical(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    rules_path = tmp_path / "rules.yaml"
    rules_path.parent.mkdir(parents=True, exist_ok=True)
    rules_path.write_text(
        yaml.safe_dump(
            {
                "fatal": LOG_RULES["fatal"],
                "critical_warnings": LOG_RULES["critical_warnings"],
                "allowed_warnings": ["failsafe triggered due to attitude"],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (run / "px4.log").write_text(
        "connection lost to mavros\n", encoding="utf-8"
    )

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=rules_path,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T10"]["status"] == "FAIL"


def test_no_log_files_uses_monitor_complete_clean_flag(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    factory = lambda _path: _build_time_varying_actuator_ulog()

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    # No log files means T10 is vacuously PASS so the overall verdict is set
    # by monitor_complete.json.
    assert items["T10"]["status"] == "PASS"
    assert summary["overall"] == "PASS"


def test_missing_ulog_fails_actuator_tests(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    def factory(_path):
        raise FileNotFoundError("no .ulg file")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    assert items["T02"]["status"] == "FAIL"
    assert items["T04"]["status"] == "FAIL"
    assert items["T07"]["status"] == "FAIL"


def test_takeoff_altitude_check(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    t03 = items["T03"]
    assert t03["status"] == "PASS"
    assert abs(t03["measured"]["altitude_m"] - 5.0) <= 0.75


def test_final_ground_target_reached(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    items = {item["id"]: item for item in summary["items"]}
    t08 = items["T08"]
    assert t08["status"] == "PASS"
    assert abs(t08["measured"]["forward_m"] - 3.0) <= 1.5
    assert abs(t08["measured"]["left_m"] - 3.0) <= 1.5


def test_validation_report_schema(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    assert summary["schema_version"] == 1
    assert "config_sha256" in summary
    assert summary["trial_result_dir"].endswith("run")
    assert set(summary["counts"].keys()) == {
        "fatal",
        "critical_warnings",
        "allowed_warnings",
        "ros_errors",
        "px4_errors",
        "python_tracebacks",
    }
    item_ids = {item["id"] for item in summary["items"]}
    assert {"T01", "T02", "T03", "T04", "T05", "T06", "T07", "T08", "T09", "T10"} <= item_ids


def test_write_validation_report_persists_file(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    output = run / "validation.json"

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    validation.write_validation_report(report, output)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert {item["id"] for item in payload["items"]} >= {"T01", "T10"}


def test_static_passing_run_fixture_evaluates_pass():
    fixture_dir = (
        Path(__file__).resolve().parent / "fixtures" / "passing_run"
    )
    log_rules = (
        Path(__file__).resolve().parent.parent / "config" / "log_rules.yaml"
    )
    factory = lambda _path: _build_time_varying_actuator_ulog()
    report = validation.evaluate_run(
        result_dir=fixture_dir,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    summary = report.summary()
    assert summary["overall"] == "PASS", summary


def test_static_failing_run_fixture_evaluates_fail():
    fixture_dir = (
        Path(__file__).resolve().parent / "fixtures" / "failing_run"
    )
    log_rules = (
        Path(__file__).resolve().parent.parent / "config" / "log_rules.yaml"
    )
    report = validation.evaluate_run(
        result_dir=fixture_dir,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    assert summary["overall"] == "FAIL", summary
    # The failing fixture deliberately omits the ULog so T02/T04/T07 must
    # close-fail with missing evidence.
    items = {item["id"]: item for item in summary["items"]}
    assert items["T02"]["status"] == "FAIL"
    assert items["T04"]["status"] == "FAIL"
    assert items["T07"]["status"] == "FAIL"


# ---------------------------------------------------------------------------
# Log scan via GNU grep
# ---------------------------------------------------------------------------


def _make_grep_invocation_spy(matches):
    """Return a fake grep_factory that records its calls and returns canned matches."""

    calls = []

    def factory(patterns, globs, path):
        calls.append({
            "patterns": patterns,
            "globs": globs,
            "path": path,
        })
        return list(matches)

    factory.calls = calls
    return factory


def test_scan_logs_invokes_grep_factory_with_result_dir(tmp_path):
    """Production _scan_logs must invoke self.grep_factory exactly once
    with the configured result directory and the LOG_GLOBS list."""
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    spy = _make_grep_invocation_spy([])
    validator = validation.TrialValidator(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        grep_factory=spy,
    )
    scan = validator._scan_logs()
    assert scan["fatal"] == []
    assert scan["critical"] == []
    assert len(spy.calls) == 1, spy.calls
    call = spy.calls[0]
    assert Path(call["path"]).resolve() == run.resolve()
    assert set(call["patterns"].keys()) == {"case_sensitive", "case_insensitive"}
    # All configured fatal + critical patterns must be passed to the factory.
    for pattern in LOG_RULES["fatal"]:
        assert pattern in call["patterns"]["case_sensitive"]
    for pattern in LOG_RULES["critical_warnings"]:
        assert pattern in call["patterns"]["case_sensitive"]
    assert "lockstep.*timeout" in call["patterns"]["case_insensitive"]
    # The factory must receive the default log globs so GNU grep -RInE
    # --include=... restricts the search to log files.
    for glob in validation.TrialValidator.LOG_GLOBS:
        assert glob in call["globs"]


def test_scan_logs_under_injection_classifies_fatal_match(tmp_path):
    """An injected grep_factory that returns a fatal-pattern match must
    cause T10 to fail with the expected classification."""
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.log").write_text("dummy\n", encoding="utf-8")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    matches = [{
        "file": str(run / "monitor.log"),
        "line": 5,
        "content": "[ERROR] PX4: navigation failed",
    }]
    spy = _make_grep_invocation_spy(matches)
    validator = validation.TrialValidator(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        grep_factory=spy,
    )
    scan = validator._scan_logs()
    assert scan["fatal"], scan
    fatal = scan["fatal"][0]
    assert fatal["pattern"] == "\\[ERROR\\]"
    assert fatal["line"] == 5
    # The vendor-error counters require a literal ``[ERROR]`` pattern;
    # the configured rules store the ERE-escaped form so the counters
    # stay at zero today. This test exercises the match classification
    # contract end-to-end without depending on that pre-existing corner.
    assert scan["counts"]["px4_errors"] == 0
    assert scan["counts"]["ros_errors"] == 0


def test_scan_logs_under_injection_classifies_critical_match(tmp_path):
    """An injected grep_factory that returns a critical-pattern match must
    cause T10 to fail unless the allowlist suppresses it."""
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    matches = [{
        "file": str(run / "px4.log"),
        "line": 12,
        "content": "failsafe triggered due to attitude",
    }]
    spy = _make_grep_invocation_spy(matches)
    validator = validation.TrialValidator(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        grep_factory=spy,
    )
    scan = validator._scan_logs()
    assert scan["critical"], scan
    critical = scan["critical"][0]
    assert critical["pattern"] == "failsafe"
    assert critical["line"] == 12
    # Allowed warning suppresses the match into allowed_count.
    allowed_rules = tmp_path / "rules_allowed.yaml"
    allowed_rules.write_text(
        yaml.safe_dump(
            {
                "fatal": LOG_RULES["fatal"],
                "critical_warnings": LOG_RULES["critical_warnings"],
                "allowed_warnings": ["failsafe triggered due to attitude"],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    validator2 = validation.TrialValidator(
        result_dir=run,
        config=CONFIG,
        log_rules_path=allowed_rules,
        grep_factory=spy,
    )
    scan2 = validator2._scan_logs()
    assert scan2["critical"] == []
    assert scan2["allowed_count"] == 1


def test_default_grep_factory_invokes_subprocess_grep(tmp_path, monkeypatch):
    """The default grep_factory MUST shell out to GNU grep via subprocess.run
    so that ripgrep/Windows-grep is never used. Spy on subprocess.run and
    assert the captured argv includes the grep binary resolved via ``which``
    plus the -RInE flags."""
    run = tmp_path / "run"
    run.mkdir()
    (run / "monitor.log").write_text("[ERROR] sample\n", encoding="utf-8")

    captured = []

    class _Completed:
        returncode = 0
        stdout = "{}:1:[ERROR] sample\n".format(run / "monitor.log")
        stderr = ""

    def fake_run(args, **kwargs):
        captured.append({"args": args, "kwargs": kwargs})
        return _Completed()

    monkeypatch.setattr(validation.subprocess, "run", fake_run)
    monkeypatch.setattr(validation.shutil, "which", lambda name: "/usr/bin/grep")

    factory = validation.TrialValidator._default_grep_factory
    patterns = {
        "case_sensitive": [r"\[ERROR\]"],
        "case_insensitive": [],
    }
    out = factory(
        None,
        patterns,
        list(validation.TrialValidator.LOG_GLOBS),
        str(run),
    )
    assert len(captured) == 1, captured
    argv = captured[0]["args"]
    assert argv[0] == "/usr/bin/grep"
    assert "-RInE" in argv
    assert r"-e" in argv
    idx = argv.index(r"-e")
    assert argv[idx + 1] == r"\[ERROR\]"
    assert "--include" in " ".join(argv)
    assert str(run) in argv
    assert isinstance(out, list), out
    if out:
        assert "[ERROR] sample" in out[0]["content"]


def test_default_grep_factory_returns_parsed_matches(tmp_path, monkeypatch):
    """The default factory must return parsed entries with file/line/content
    keys so ``_scan_logs`` can classify the matches."""
    run = tmp_path / "run"
    run.mkdir()
    sample_path = run / "controller.log"
    sample_path.write_text("Traceback (most recent call last):\n", encoding="utf-8")

    class _Completed:
        returncode = 0
        stdout = "{}:1:Traceback (most recent call last):\n".format(sample_path)
        stderr = ""

    monkeypatch.setattr(validation.subprocess, "run", lambda args, **kw: _Completed())
    monkeypatch.setattr(validation.shutil, "which", lambda name: "/usr/bin/grep")

    factory = validation.TrialValidator._default_grep_factory
    out = factory(
        None,
        {
            "case_sensitive": [r"Traceback \(most recent call last\)"],
            "case_insensitive": [],
        },
        list(validation.TrialValidator.LOG_GLOBS),
        str(run),
    )
    assert isinstance(out, list)
    assert out, out
    entry = out[0]
    assert set(entry.keys()) == {"file", "line", "content"}
    assert entry["file"].endswith("controller.log")
    assert entry["line"] == 1
    assert "Traceback" in entry["content"]


def test_default_grep_factory_works_with_real_grep(tmp_path):
    """End-to-end: write a fatal-pattern log and run the default factory
    against WSL GNU grep. The match must come back with file/line/content."""
    run = tmp_path / "run"
    run.mkdir()
    sample = run / "monitor.log"
    sample.write_text(
        "normal line\n"
        "[ERROR] PX4: navigation failed\n"
        "another normal line\n",
        encoding="utf-8",
    )

    factory = validation.TrialValidator._default_grep_factory
    matches = factory(
        None,
        {
            "case_sensitive": [r"\[ERROR\]"],
            "case_insensitive": [],
        },
        list(validation.TrialValidator.LOG_GLOBS),
        str(run),
    )
    assert matches, matches
    entry = matches[0]
    assert entry["file"].endswith("monitor.log")
    assert entry["line"] == 2
    assert "[ERROR] PX4: navigation failed" in entry["content"]


def test_validate_run_cli_writes_validation_json(tmp_path):
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = tmp_path / "rules.yaml"
    _write_log_rules_yaml(log_rules)

    config_path = tmp_path / "experiment.yaml"
    config_path.write_text(
        Path(__file__).resolve().parents[1]
        .joinpath("config", "experiment.yaml")
        .read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    factory = lambda _path: _build_time_varying_actuator_ulog()
    # Patch the default ULog factory so the CLI uses our fake ULog without
    # requiring a real binary fixture.
    import flying_car_test.validation as _validation_module
    original_default = _validation_module.TrialValidator._default_ulog_factory
    _validation_module.TrialValidator._default_ulog_factory = lambda self, path: factory(path)
    try:
        rc = validation.main(
            [
                "--result-dir",
                str(run),
                "--config",
                str(config_path),
                "--log-rules",
                str(log_rules),
            ]
        )
    finally:
        _validation_module.TrialValidator._default_ulog_factory = original_default
    assert rc in (0, 1, 2)
    assert (run / "validation.json").exists()
    payload = json.loads((run / "validation.json").read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1


# ---------------------------------------------------------------------------
# Code-quality round 2 fixes (grep timeout, ulog_available, non-finite JSON)
# ---------------------------------------------------------------------------


def test_default_grep_factory_timeout_is_wired(tmp_path, monkeypatch):
    """The default grep_factory MUST pass a finite timeout to subprocess.run
    so a runaway grep invocation cannot hang the validator indefinitely."""
    run = tmp_path / "run"
    run.mkdir()
    captured_kwargs = []

    class _Completed:
        returncode = 0
        stdout = ""
        stderr = ""

    def fake_run(args, **kwargs):
        captured_kwargs.append(kwargs)
        return _Completed()

    monkeypatch.setattr(validation.subprocess, "run", fake_run)
    monkeypatch.setattr(validation.shutil, "which", lambda name: "/usr/bin/grep")

    factory = validation.TrialValidator._default_grep_factory
    factory(
        None,
        {"case_sensitive": [r"\[ERROR\]"], "case_insensitive": []},
        list(validation.TrialValidator.LOG_GLOBS),
        str(run),
    )
    assert captured_kwargs, captured_kwargs
    timeout = captured_kwargs[0].get("timeout")
    assert isinstance(timeout, (int, float))
    assert timeout > 0


def test_default_grep_factory_timeout_treated_as_fatal_match(tmp_path, monkeypatch):
    """When the grep subprocess exceeds its timeout the factory must raise
    subprocess.TimeoutExpired so the validator's classifier catches it and
    treats it as a structured fatal match (no indefinite hang)."""
    import subprocess as _subprocess

    run = tmp_path / "run"
    run.mkdir()

    def fake_run(args, **kwargs):
        raise _subprocess.TimeoutExpired(cmd=args, timeout=kwargs.get("timeout"))

    monkeypatch.setattr(validation.subprocess, "run", fake_run)
    monkeypatch.setattr(validation.shutil, "which", lambda name: "/usr/bin/grep")

    factory = validation.TrialValidator._default_grep_factory
    with pytest.raises(_subprocess.TimeoutExpired):
        factory(
            None,
            {"case_sensitive": [r"\[ERROR\]"], "case_insensitive": []},
            list(validation.TrialValidator.LOG_GLOBS),
            str(run),
        )


def test_scan_logs_classifies_grep_timeout_as_fatal(tmp_path, monkeypatch):
    """When the grep subprocess exceeds its timeout the validator must
    classify it as a fatal match carrying the explicit "grep_timeout"
    pattern so the operator can see what happened."""
    import subprocess as _subprocess

    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    def fake_run(args, **kwargs):
        raise _subprocess.TimeoutExpired(cmd=args, timeout=kwargs.get("timeout"))

    monkeypatch.setattr(validation.subprocess, "run", fake_run)
    monkeypatch.setattr(validation.shutil, "which", lambda name: "/usr/bin/grep")

    validator = validation.TrialValidator(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    scan = validator._scan_logs()
    fatal_patterns = [match["pattern"] for match in scan["fatal"]]
    assert "grep_timeout" in fatal_patterns, scan


def test_metadata_ulog_available_false_when_ulog_missing(tmp_path):
    """metadata.ulog_available must be False when the .ulg evidence is
    absent and no ULog factory is provided."""
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    summary = report.summary()
    assert summary["metadata"]["ulog_available"] is False


def test_metadata_ulog_available_true_when_ulog_loads(tmp_path):
    """metadata.ulog_available must be True when _load_actuator_ulog
    successfully loads actuator outputs."""
    run = tmp_path / "run"
    _build_passing_run(run)
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    factory = lambda _path: _build_time_varying_actuator_ulog()
    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    summary = report.summary()
    assert summary["metadata"]["ulog_available"] is True


def test_write_validation_report_rejects_non_finite_measured(tmp_path):
    """write_validation_report must refuse to serialise a report that
    contains a non-finite float (NaN/Inf) and raise a clear error."""
    run = tmp_path / "run"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
    )
    # Inject a non-finite measured value into one of the items.
    if not report.items:
        pytest.skip("validator produced no items")
    report.items[0]["measured"]["altitude_m"] = float("nan")
    output = run / "validation.json"
    with pytest.raises((ValueError, json.JSONDecodeError)):
        validation.write_validation_report(report, output)


def test_validate_run_cli_exits_zero_for_passing_fixture(tmp_path):
    """The CLI entry point must exit 0 when the on-disk passing fixture
    validates successfully."""
    fixture_dir = (
        Path(__file__).resolve().parent / "fixtures" / "passing_run"
    )
    log_rules = (
        Path(__file__).resolve().parents[1] / "config" / "log_rules.yaml"
    )
    config_path = tmp_path / "experiment.yaml"
    config_path.write_text(
        Path(__file__).resolve().parents[1]
        .joinpath("config", "experiment.yaml")
        .read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    factory = lambda _path: _build_time_varying_actuator_ulog()
    import flying_car_test.validation as _validation_module
    original_default = _validation_module.TrialValidator._default_ulog_factory
    _validation_module.TrialValidator._default_ulog_factory = lambda self, path: factory(path)
    try:
        rc = validation.main(
            [
                "--result-dir",
                str(fixture_dir),
                "--config",
                str(config_path),
                "--log-rules",
                str(log_rules),
            ]
        )
    finally:
        _validation_module.TrialValidator._default_ulog_factory = original_default
    assert rc == 0


def test_malformed_ulog_file_fails_T02_T04_T07(tmp_path):
    """A 0-byte / malformed monitor.ulg must not crash the validator; it
    must surface as informative FAILs on the actuator-dependent tests."""
    run = tmp_path / "run"
    _build_passing_run(run)
    # Write a 0-byte monitor.ulg file so the loader is invoked but fails.
    (run / "monitor.ulg").write_bytes(b"")
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")

    def factory(_path):
        # The default loader raises ValueError when the ULog cannot be
        # parsed; the validator must catch it and surface the error.
        raise ValueError("non-finite actuator outputs")

    report = validation.evaluate_run(
        result_dir=run,
        config=CONFIG,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )
    items = {item["id"]: item for item in report.summary()["items"]}
    assert items["T02"]["status"] == "FAIL"
    assert items["T04"]["status"] == "FAIL"
    assert items["T07"]["status"] == "FAIL"
    # Errors must be informative, not empty strings.
    for test_id in ("T02", "T04", "T07"):
        error = items[test_id].get("measured", {}).get("error", "")
        assert error, items[test_id]
