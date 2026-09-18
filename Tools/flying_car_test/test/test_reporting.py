"""Tests for the flying-car experiment report generator (Task 9).

The reporting layer consumes a single trial's result directory plus the
on-disk ``validation.json`` and emits a human-readable ``report.md``,
deterministic PNG plots, and a machine-readable ``summary.json``.

The single-run ``write_single_run_report`` writes:
    report.md
    trajectory.png
    altitude.png
    attitude.png
    modes.png
    actuators.png
    summary.json

The aggregate ``write_aggregate_report`` consumes multiple trial
directories and writes:
    aggregate_report.md
    aggregate_summary.json

Both writers are exercised end-to-end against an in-tmp_path trial
fixture that mirrors the on-disk passing fixture (samples.jsonl,
events.jsonl, commands.jsonl, metadata.json, monitor_complete.json and
a ULog placeholder).
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from flying_car_test import reporting, validation


def _load_config(config_path: Path):
    from flying_car_test.config import load_config
    return load_config(str(config_path))


def _event(stamp, phase):
    return {"stamp": stamp, "type": "phase", "phase": phase}


def _command(stamp, kind, **values):
    return {"stamp": stamp, "kind": kind, "values": values}


def _sample(stamp, **overrides):
    record = {
        "stamp": stamp,
        "clock": stamp,
        "connected": True,
        "armed": False,
        "mode": "MANUAL",
        "landed": True,
        "mav_type": 10,
        "mav_type_fresh": True,
        "local": {"x": 0.0, "y": 0.0, "z": 0.0},
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "gazebo": {"x": 0.0, "y": 0.0, "z": 0.0},
        "clock_ok": True,
    }
    for key, value in overrides.items():
        if key in ("x", "y", "z"):
            record["local"][key] = value
        elif key in ("vx", "vy", "vz"):
            record["velocity"][key] = value
        elif key in ("roll", "pitch", "yaw"):
            record["attitude"][key] = value
        else:
            record[key] = value
    return record


def _write_jsonl(path: Path, rows):
    path.write_text(
        "\n".join(json.dumps(dict(row)) for row in rows) + "\n",
        encoding="utf-8",
    )


def _write_json(path: Path, value):
    path.write_text(json.dumps(dict(value)), encoding="utf-8")


def _build_passing_run(root: Path) -> None:
    """Build a tiny deterministic passing run with one of each phase.

    The fixture writes:
        samples.jsonl, events.jsonl, commands.jsonl, metadata.json,
        monitor_complete.json and a (zero-byte) monitor.ulg placeholder.
    """
    root.mkdir(parents=True, exist_ok=True)
    samples = []
    phase_events = []

    def emit(stamp_iter, factory, phase):
        first = True
        for stamp in stamp_iter:
            samples.append(factory(stamp))
            if first:
                phase_events.append((stamp, phase))
                first = False

    emit(
        (i * 0.1 for i in range(10)),
        lambda s: _sample(s),
        "PRECHECK",
    )

    def _ground(stamp, idx=[0]):
        idx[0] += 1
        return _sample(stamp, x=idx[0] * 0.5)
    emit((1.0 + i * 0.1 for i in range(10)), _ground, "GROUND_FORWARD")
    emit(
        (2.0 + i * 0.1 for i in range(10)),
        lambda s: _sample(s, x=5.0),
        "GROUND_STOP",
    )

    def _flight(stamp, idx=[0]):
        idx[0] += 1
        mav = 14 if idx[0] > 5 else 10
        return _sample(stamp, x=5.0, mav_type=mav, mav_type_fresh=True)
    emit((3.0 + i * 0.1 for i in range(10)), _flight, "FLIGHT_CONFIGURE")

    def _takeoff(stamp, idx=[0]):
        idx[0] += 1
        z = min(4.0, idx[0] * 0.25)
        return _sample(stamp, x=5.0, z=z, mav_type=14, landed=False, mode="OFFBOARD")
    emit((4.0 + i * 0.1 for i in range(20)), _takeoff, "TAKEOFF")

    def _air_forward(stamp, idx=[0]):
        idx[0] += 1
        x = 5.0 + idx[0] * 0.5
        return _sample(stamp, x=x, z=4.0, mav_type=14, mode="OFFBOARD", landed=False)
    emit((6.0 + i * 0.1 for i in range(10)), _air_forward, "AIR_FORWARD")

    emit(
        (7.0 + i * 0.1 for i in range(5)),
        lambda s: _sample(s, x=10.0, z=4.0, mav_type=14, mode="OFFBOARD", landed=False),
        "HOVER",
    )
    emit(
        (7.5 + i * 0.1 for i in range(5)),
        lambda s: _sample(s, x=10.5, z=4.0, mav_type=14, mode="MANUAL", landed=False),
        "MANUAL_AIR",
    )
    emit(
        (8.0 + i * 0.1 for i in range(5)),
        lambda s: _sample(s, x=10.5, z=4.0, mav_type=14, mode="OFFBOARD", landed=False),
        "REACQUIRE_OFFBOARD",
    )
    landing_yaw = math.pi / 2 + 0.1

    def _land(stamp, idx=[0]):
        idx[0] += 1
        z = max(0.0, 4.0 - idx[0] * 0.4)
        return _sample(
            stamp,
            x=10.5,
            z=z,
            mav_type=14,
            mode="AUTO.LAND",
            landed=(idx[0] >= 14),
            yaw=landing_yaw,
        )
    emit((8.5 + i * 0.1 for i in range(15)), _land, "LAND")

    def _reconfigure(stamp, idx=[0]):
        idx[0] += 1
        mav = 10 if idx[0] > 5 else 14
        return _sample(stamp, x=10.5, z=0.0, mav_type=mav)
    emit((10.0 + i * 0.1 for i in range(10)), _reconfigure, "GROUND_RECONFIGURE")

    yaw_values = [0.0, math.pi / 8, math.pi / 4, math.pi / 3, math.pi / 2 + 0.1]

    def _turn(stamp, idx=[0]):
        yaw = yaw_values[min(idx[0], len(yaw_values) - 1)]
        idx[0] += 1
        return _sample(stamp, x=10.5, z=0.0, mav_type=10, yaw=yaw)
    emit((11.0 + i * 0.1 for i in range(10)), _turn, "GROUND_TURN")

    target_x = 10.5 + math.cos(landing_yaw) * 3.0 - math.sin(landing_yaw) * 3.0
    target_y = math.sin(landing_yaw) * 3.0 + math.cos(landing_yaw) * 3.0

    def _diagonal(stamp, idx=[0]):
        progress = (idx[0] + 1) / 20.0
        x = 10.5 + (target_x - 10.5) * progress
        y = 0.0 + (target_y - 0.0) * progress
        idx[0] += 1
        return _sample(stamp, x=x, y=y, mav_type=10, yaw=landing_yaw)
    emit((12.0 + i * 0.1 for i in range(20)), _diagonal, "GROUND_DIAGONAL")

    samples.append(_sample(14.0, x=target_x, y=target_y, mav_type=10))
    phase_events.append((14.0, "COMPLETE"))

    _write_jsonl(root / "samples.jsonl", samples)
    _write_jsonl(
        root / "events.jsonl",
        [_event(s, p) for s, p in phase_events],
    )
    _write_jsonl(root / "commands.jsonl", [_command(0.5, "manual", name="ground_forward")])
    _write_json(
        root / "metadata.json",
        {
            "px4_git_sha": "0" * 40,
            "px4_dirty": False,
            "experiment_config_sha256": "f" * 64,
            "model_sdf_sha256": "a" * 64,
            "airframe_sha256": "b" * 64,
            "launch_arguments": {"vehicle": "fs_fasmv8"},
            "ros_distribution": "melodic",
            "mavros_version": "1.0",
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
    # Write a ULog placeholder so the validator finds one. The fixture is
    # replaced below by the time-varying fake factory at evaluation time.
    (root / "monitor.ulg").write_bytes(b"")


def _time_varying_actuator_ulog(samples=200, dt=0.1):
    timestamps = np.arange(samples, dtype=np.float64) * dt
    outputs = np.zeros((12, samples), dtype=np.float64)
    for index in range(samples):
        stamp = float(timestamps[index])
        if 4.0 <= stamp < 8.5:
            outputs[:, index] = 1500.0
        else:
            outputs[:, index] = 1000.0
    arrays = {"timestamp": timestamps}
    for index in range(12):
        arrays["output[{}]".format(index)] = outputs[index]
    msg = type("Msg", (), {"data": arrays})()
    return type("ULog", (), {"data": {"actuator_outputs": msg}})()


def _write_log_rules_yaml(path: Path):
    rules = {
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
    path.write_text(yaml.safe_dump(rules, sort_keys=True), encoding="utf-8")
    return path


def _evaluate_trial(run: Path, log_rules: Path, config) -> "validation.ValidationReport":
    factory = lambda _path: _time_varying_actuator_ulog()
    return validation.evaluate_run(
        result_dir=run,
        config=config,
        log_rules_path=log_rules,
        ulog_factory=factory,
    )


def _write_validation_json(run: Path, report) -> None:
    (run / "validation.json").write_text(
        json.dumps(report.summary(), indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )


def _make_passing_trial(root: Path, idx: int, log_rules, config) -> Path:
    """Build a passing trial directory with a freshly written validation.json."""
    run = root / ("trial_{}".format(idx))
    _build_passing_run(run)
    report = _evaluate_trial(run, log_rules, config)
    assert report.overall == "PASS"
    _write_validation_json(run, report)
    return run


@pytest.fixture
def passing_run(tmp_path):
    run = tmp_path / "trial"
    _build_passing_run(run)
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    config = _load_config(
        Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    )
    report = _evaluate_trial(run, log_rules, config)
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    return {
        "run": run,
        "config": config,
        "config_path": Path(__file__).resolve().parents[1] / "config" / "experiment.yaml",
        "log_rules": log_rules,
        "report": report,
        "metadata": metadata,
    }


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_module_exposes_required_symbols():
    """Reporting module exports the writers and plot helpers we depend on."""
    expected = [
        "write_single_run_report",
        "write_aggregate_report",
        "plot_trajectory",
        "plot_altitude",
        "plot_attitude",
        "plot_modes",
        "plot_actuators",
    ]
    for name in expected:
        assert hasattr(reporting, name), "reporting missing {}".format(name)


def test_single_run_report_emits_all_artifacts(passing_run, tmp_path):
    output_dir = tmp_path / "out"
    output_dir.mkdir()
    md_path = output_dir / "report.md"
    plot_dir = output_dir / "plots"
    summary_path = output_dir / "summary.json"
    reporting.write_single_run_report(
        result_dir=passing_run["run"],
        validation_report=passing_run["report"],
        metadata=passing_run["metadata"],
        config_path=passing_run["config_path"],
        output_md_path=md_path,
        output_plot_dir=plot_dir,
        output_summary_path=summary_path,
    )
    for name in ("trajectory.png", "altitude.png", "attitude.png", "modes.png", "actuators.png"):
        path = plot_dir / name
        assert path.exists(), "missing {}".format(name)
        assert path.stat().st_size > 0, "empty {}".format(name)
    assert md_path.exists()
    assert summary_path.exists()
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["overall"] == "PASS"


def test_single_run_report_markdown_contents(passing_run, tmp_path):
    output_dir = tmp_path / "out"
    output_dir.mkdir()
    md_path = output_dir / "report.md"
    plot_dir = output_dir / "plots"
    summary_path = output_dir / "summary.json"
    reporting.write_single_run_report(
        result_dir=passing_run["run"],
        validation_report=passing_run["report"],
        metadata=passing_run["metadata"],
        config_path=passing_run["config_path"],
        output_md_path=md_path,
        output_plot_dir=plot_dir,
        output_summary_path=summary_path,
    )
    text = md_path.read_text(encoding="utf-8")
    # Objective and environment headers.
    assert "Objective" in text
    assert "Environment" in text
    # Git SHA and dirty state.
    assert "0000000000000000000000000000000000000000" in text
    assert "dirty" in text.lower()
    # Hashes
    assert "f" * 64 in text
    assert "a" * 64 in text
    assert "b" * 64 in text
    # Reproduction commands
    assert "Reproduction" in text or "reproduce" in text.lower()
    assert "validate_run" in text
    # T-table for T01..T10
    for tid in ("T01", "T02", "T03", "T04", "T05", "T06", "T07", "T08", "T09", "T10"):
        assert tid in text
    # Mention of T11 placeholder for the QGC/manual run (Task 11 will
    # populate it; this report only fills T01..T10).
    assert "T11" in text
    # Expected and measured values
    assert "Expected" in text
    assert "Measured" in text
    # Counts
    assert "Errors" in text or "Fatal" in text
    assert "Critical" in text or "critical" in text
    assert "Allowed" in text or "allowed" in text
    # QGC procedure reference (README may not exist yet -> TODO marker)
    assert "QGC" in text
    assert "README" in text
    # Future aggregate placeholder
    assert "aggregate" in text.lower()
    # Links to artifacts
    assert "trajectory.png" in text
    assert "altitude.png" in text
    assert "attitude.png" in text
    assert "modes.png" in text
    assert "actuators.png" in text


def test_aggregate_report_passes_only_when_all_trials_pass(tmp_path):
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    config = _load_config(
        Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    )
    trial_dirs = []
    for index in range(3):
        trial_dirs.append(_make_passing_trial(tmp_path, index, log_rules, config))
    config_path = Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    out_dir = tmp_path / "agg"
    out_dir.mkdir()
    reporting.write_aggregate_report(
        trial_dirs=trial_dirs,
        config_path=config_path,
        output_md_path=out_dir / "aggregate_report.md",
        output_plot_dir=out_dir / "plots",
        output_summary_path=out_dir / "aggregate_summary.json",
    )
    assert (out_dir / "aggregate_report.md").exists()
    payload = json.loads((out_dir / "aggregate_summary.json").read_text(encoding="utf-8"))
    assert payload["overall"] == "PASS"
    stats = payload["statistics"]
    # Required statistics keys: scalar metrics carry mean/std/min/max
    # directly; phase_duration_s is per-phase.
    for key in ("total_distance_m", "peak_altitude_m", "peak_tilt_deg"):
        assert key in stats
        for stat in ("mean", "std", "min", "max"):
            assert stat in stats[key]
            assert isinstance(stats[key][stat], (int, float))
    # phase_duration_s is keyed by phase name with mean/std/min/max per phase.
    assert "phase_duration_s" in stats
    assert isinstance(stats["phase_duration_s"], dict)
    assert stats["phase_duration_s"], "phase_duration_s should be non-empty"
    for phase_block in stats["phase_duration_s"].values():
        for stat in ("mean", "std", "min", "max"):
            assert stat in phase_block
            assert isinstance(phase_block[stat], (int, float))


def test_aggregate_report_fails_if_one_trial_fails(tmp_path):
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    config = _load_config(
        Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    )
    trial_dirs = []
    for index in range(3):
        trial_dirs.append(_make_passing_trial(tmp_path, index, log_rules, config))
    # Force the first trial's validation.json to report FAIL so the
    # aggregator must demote overall to FAIL.
    failure_report = {
        "schema_version": 1,
        "config_sha256": "f" * 64,
        "trial_result_dir": str(trial_dirs[0]),
        "overall": "FAIL",
        "abort_reason": "synthetic trial failure",
        "counts": {"fatal": 0, "critical_warnings": 0, "allowed_warnings": 0,
                   "ros_errors": 0, "px4_errors": 0, "python_tracebacks": 0},
        "items": [],
        "metadata": {"trial_utc": 0.0, "git_sha": "0" * 40, "ulog_available": True, "duration_s": 0.0},
    }
    (trial_dirs[0] / "validation.json").write_text(
        json.dumps(failure_report), encoding="utf-8"
    )
    config_path = Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    out_dir = tmp_path / "agg"
    out_dir.mkdir()
    reporting.write_aggregate_report(
        trial_dirs=trial_dirs,
        config_path=config_path,
        output_md_path=out_dir / "aggregate_report.md",
        output_plot_dir=out_dir / "plots",
        output_summary_path=out_dir / "aggregate_summary.json",
    )
    payload = json.loads((out_dir / "aggregate_summary.json").read_text(encoding="utf-8"))
    assert payload["overall"] == "FAIL"


def test_aggregate_report_tolerates_missing_ulog(tmp_path):
    """Aggregator must still produce a PASS report when ``validation.json``
    is missing for every trial, as long as the underlying trial evidence
    passes.

    Locks in the missing-validation regeneration path: the aggregator
    must re-evaluate each trial against the canonical
    ``log_rules.yaml`` and propagate the regenerated verdict into the
    aggregate payload. A previous bug caused ``_default_log_rules_path``
    to return ``None`` and silently turn ``validation`` into ``None``
    inside ``aggregate_summary.json``, forcing ``overall`` to FAIL even
    when every trial passed on the ground truth.

    The original loose form of this test only asserted that the
    aggregate files were emitted. The tightened form additionally
    asserts ``overall == "PASS"`` and that each per-trial ``validation``
    is regenerated (not ``None``) and PASS.
    """
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    config = _load_config(
        Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    )
    trial_dirs = []
    for index in range(2):
        run = _make_passing_trial(tmp_path, index, log_rules, config)
        # Drop the validation.json so the aggregator must regenerate
        # the verdict from the on-disk trial evidence (samples.jsonl,
        # events.jsonl, commands.jsonl, monitor_complete.json,
        # monitor.ulg). The ULog is kept so T02/T04/T07 (rotor checks)
        # can still PASS.
        validation_path = run / "validation.json"
        if validation_path.exists():
            validation_path.unlink()
        trial_dirs.append(run)
    config_path = Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    out_dir = tmp_path / "agg"
    out_dir.mkdir()
    reporting.write_aggregate_report(
        trial_dirs=trial_dirs,
        config_path=config_path,
        output_md_path=out_dir / "aggregate_report.md",
        output_plot_dir=out_dir / "plots",
        output_summary_path=out_dir / "aggregate_summary.json",
    )
    assert (out_dir / "aggregate_report.md").exists()
    payload = json.loads((out_dir / "aggregate_summary.json").read_text(encoding="utf-8"))
    assert payload["overall"] == "PASS"
    # Per-trial validation must be regenerated (not None) and PASS.
    for entry in payload["trials"]:
        assert entry["validation"] is not None
        assert entry["validation"].get("overall") == "PASS"


def test_default_log_rules_path_resolves_to_repo_config():
    """``_default_log_rules_path`` must point at the on-disk
    ``Tools/flying_car_test/config/log_rules.yaml``.

    The reporter package lives at
    ``Tools/flying_car_test/src/flying_car_test/reporting.py``, so the
    helper must walk two levels up (to ``Tools/flying_car_test/``) and
    then join ``config/log_rules.yaml``. Walking only one level up
    silently resolves to a non-existent ``config/log_rules.yaml``
    inside the ``src`` tree and breaks the missing-validation
    regeneration path.
    """
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "log_rules.yaml"
    )
    assert config_path.exists(), (
        "fixture missing: expected on-disk log_rules.yaml at "
        + str(config_path)
    )
    resolved = reporting._default_log_rules_path(config_path)
    assert resolved == config_path
    assert resolved.exists()


def test_default_log_rules_path_returns_existing_file():
    """Sanity: the resolved path must point at an existing file even
    when the caller passes an unrelated config path (the helper
    deliberately ignores the argument)."""
    resolved = reporting._default_log_rules_path(Path("/tmp/nonexistent.yaml"))
    assert resolved is not None
    assert resolved.exists()
    assert resolved.name == "log_rules.yaml"


def test_reporting_import_works_without_matplotlib(monkeypatch):
    """``flying_car_test.reporting`` must import cleanly when matplotlib
    is not installed. The plotting writers may raise later, but the
    top-level ``import reporting`` must not fail.
    """
    import importlib
    import sys

    monkeypatch.setitem(sys.modules, "matplotlib", None)
    monkeypatch.setitem(sys.modules, "matplotlib.pyplot", None)
    monkeypatch.setitem(sys.modules, "matplotlib.backends", None)

    import flying_car_test.reporting as reporting_fresh
    importlib.reload(reporting_fresh)

    # The reload above replaces the module in ``sys.modules`` already.
    assert hasattr(reporting_fresh, "write_single_run_report")
    assert hasattr(reporting_fresh, "write_aggregate_report")
    # The flag exists so the writers can raise a clear error later.
    assert hasattr(reporting_fresh, "_HAS_MPL")


def test_plot_helper_raises_without_matplotlib(tmp_path, monkeypatch):
    """Plot writers must raise a clear error when matplotlib is absent."""
    import importlib
    import sys

    monkeypatch.setitem(sys.modules, "matplotlib", None)
    monkeypatch.setitem(sys.modules, "matplotlib.pyplot", None)
    # Reload reporting so it picks up the missing-import sentinel.
    import flying_car_test.reporting as reporting_fresh

    importlib.reload(reporting_fresh)

    samples = [_sample(0.0)]
    with pytest.raises(ImportError):
        reporting_fresh.plot_trajectory(samples, tmp_path / "t.png")

    # Restore matplotlib so other tests in the session continue to work.
    monkeypatch.undo()
    importlib.reload(reporting_fresh)


def test_report_writes_are_atomic(tmp_path):
    """``write_single_run_report`` must write report.md and summary.json
    atomically via a temporary file plus rename.

    The spec required atomic writes so a crash mid-write never leaves a
    half-written report on disk. We verify the helper by patching
    ``Path.replace`` so we can observe the rename call without actually
    clobbering files.
    """
    # Build a passing run.
    log_rules = _write_log_rules_yaml(tmp_path / "rules.yaml")
    config = _load_config(
        Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
    )
    run = _make_passing_trial(tmp_path, 0, log_rules, config)
    report = _evaluate_trial(run, log_rules, config)

    out_dir = tmp_path / "out"
    out_dir.mkdir()
    md_path = out_dir / "report.md"
    summary_path = out_dir / "summary.json"
    plot_dir = out_dir / "plots"

    renamed = []

    import flying_car_test.reporting as reporting_module

    real_replace = Path.replace

    def spy_replace(self, target):
        renamed.append((str(self), str(target)))
        return real_replace(self, target)

    # Install the spy directly; restore on exit so other tests are
    # unaffected. We avoid ``pytest.MonkeyPatch`` because the test
    # environment ships pytest 4.x, which does not expose
    # ``MonkeyPatch`` as a class attribute.
    Path.replace = spy_replace
    try:
        reporting_module.write_single_run_report(
            result_dir=run,
            validation_report=report,
            metadata=json.loads((run / "metadata.json").read_text(encoding="utf-8")),
            config_path=Path(__file__).resolve().parents[1] / "config" / "experiment.yaml",
            output_md_path=md_path,
            output_plot_dir=plot_dir,
            output_summary_path=summary_path,
        )
    finally:
        Path.replace = real_replace

    # Both the markdown report and the summary must have been written
    # via a temp + rename dance. We accept any number of renames (plots,
    # etc.) but require at least the two that produced report.md /
    # summary.json.
    rename_targets = {target for _, target in renamed}
    assert str(md_path) in rename_targets
    assert str(summary_path) in rename_targets


def test_plot_helpers_write_non_empty_pngs(tmp_path, passing_run):
    plot_dir = tmp_path / "plots"
    plot_dir.mkdir()
    samples = _read_jsonl_samples(passing_run["run"] / "samples.jsonl")
    events = _read_jsonl(passing_run["run"] / "events.jsonl")
    cfg = passing_run["config"]
    reporting.plot_trajectory(samples, plot_dir / "trajectory.png")
    reporting.plot_altitude(samples, cfg.trajectory.takeoff_agl_m, cfg.tolerances.altitude_m, plot_dir / "altitude.png")
    reporting.plot_attitude(samples, cfg.safety.soft_tilt_deg, cfg.safety.hard_tilt_deg, plot_dir / "attitude.png")
    reporting.plot_modes(events, plot_dir / "modes.png")
    # Actuators use a small fake time-varying array.
    timestamps = np.arange(50, dtype=np.float64) * 0.1
    outputs = [np.zeros(50, dtype=np.float64) for _ in range(12)]
    reporting.plot_actuators(timestamps, outputs, plot_dir / "actuators.png")
    for name in ("trajectory.png", "altitude.png", "attitude.png", "modes.png", "actuators.png"):
        path = plot_dir / name
        assert path.exists() and path.stat().st_size > 0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _read_jsonl(path: Path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _read_jsonl_samples(path: Path):
    return _read_jsonl(path)
