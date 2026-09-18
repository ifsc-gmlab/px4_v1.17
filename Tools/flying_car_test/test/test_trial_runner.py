"""Contract tests for Task 10: roslaunch orchestration (run_trials.py).

These tests verify, without spawning any real processes, that the trial
runner:

* produces UTC-timestamped, ``trial-NN`` directories under ``--out-root`` and
  refuses to clobber an already-existing result directory;
* gives each child process a unique ``ROS_LOG_DIR``;
* refuses to launch while the PX4/MAVROS/Gazebo UDP ports are occupied;
* never invokes broad destructive operations (pkill, killall, rosclean,
  ``rm -rf``); and
* enforces the launch contract (full / monitor-only launches and the rosbag
  topic list).
"""

from __future__ import annotations

import datetime
import importlib.util
import os
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET
from argparse import Namespace
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
FLYING_CAR_DIR = REPO_ROOT / "Tools" / "flying_car_test"
LAUNCH_DIR = FLYING_CAR_DIR / "launch"
RUNNER_SCRIPT = FLYING_CAR_DIR / "scripts" / "run_trials.py"
FULL_LAUNCH = LAUNCH_DIR / "full_experiment.launch"
MONITOR_LAUNCH = LAUNCH_DIR / "monitor_only.launch"

# Required TCP/UDP ports for PX4 sitl, MAVROS, Gazebo, and ros master.
DEFAULT_REQUIRED_PORTS = (
    ("udp", "127.0.0.1", 14540),
    ("udp", "127.0.0.1", 14550),
    ("udp", "127.0.0.1", 14580),
    ("tcp", "127.0.0.1", 11345),
    ("tcp", "127.0.0.1", 11311),
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_runner_script():
    module_name = "flying_car_test.run_trials"
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, RUNNER_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    package = __import__("flying_car_test")
    setattr(package, "run_trials", module)
    return module


RUNNER_MODULE = _load_runner_script()


def _load_launch(path):
    return ET.parse(path).getroot()


def _node(root, name=None, ty=None, pkg=None):
    for node in root.iter("node"):
        if name is not None and node.attrib.get("name") != name:
            continue
        if ty is not None and node.attrib.get("type") != ty:
            continue
        if pkg is not None and node.attrib.get("pkg") != pkg:
            continue
        return node
    return None


def _remaps(node):
    return {
        (child.attrib.get("from"), child.attrib.get("to"))
        for child in node.findall("remap")
    }


def _includes(root, attr="file"):
    return [child.attrib.get(attr) for child in root.findall("include")]


def _fixed_now():
    return datetime.datetime(
        2026, 7, 30, 12, 34, 56, tzinfo=datetime.timezone.utc
    )


class _StaticProbe:
    """Deterministic port prober used by the preflight-only tests."""

    def __init__(self, occupied=()):
        self.occupied = set(occupied)

    def is_occupied(self, host, port, protocol="udp"):
        return (protocol, host, int(port)) in self.occupied


class _CapturingLaunchRunner:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = False

    def start(self):
        self.started = True

    def poll_completion_marker(self, timeout_s):
        return self.kwargs["result_dir"] / "monitor_complete.json"

    def request_shutdown(self):
        return None

    def shutdown(self, **kwargs):
        return 0

    def wait(self, timeout=None):
        return 0

    def poll(self):
        return 0

    def terminate(self):
        return True

    def kill(self):
        return True

    def locate_ulog(self):
        return None


# ---------------------------------------------------------------------------
# Module-level / import smoke
# ---------------------------------------------------------------------------


def test_run_trials_module_is_importable():
    """The authorized runner script directly exposes the orchestrator API."""
    runner = RUNNER_MODULE

    assert callable(getattr(runner, "main", None))
    assert callable(getattr(runner, "run_trials", None))
    assert Path(runner.__file__).resolve() == RUNNER_SCRIPT.resolve()


def test_runner_script_is_self_contained_without_src_module():
    """Task 10 implementation lives only in the authorized script."""
    runner = RUNNER_MODULE

    assert callable(runner.build_trial_directory)


# ---------------------------------------------------------------------------
# Trial directory naming + collision protection
# ---------------------------------------------------------------------------


def test_build_trial_directory_uses_utc_timestamp_and_index(tmp_path):
    from flying_car_test.run_trials import build_trial_directory

    path = build_trial_directory(tmp_path, trial_number=1, now=_fixed_now())
    assert path == tmp_path / "2026-07-30T123456Z" / "trial-01"


def test_build_trial_directory_pads_trial_index(tmp_path):
    from flying_car_test.run_trials import build_trial_directory

    parent = tmp_path / "2026-01-01T000000Z"
    (parent / "trial-09").mkdir(parents=True)
    path = build_trial_directory(
        tmp_path,
        trial_number=10,
        now=datetime.datetime(
            2026, 1, 1, 0, 0, 0, tzinfo=datetime.timezone.utc
        ),
    )
    assert path == parent / "trial-10"


def test_check_no_existing_result_rejects_collision(tmp_path):
    from flying_car_test.run_trials import (
        ResultDirectoryExistsError,
        build_trial_directory,
        check_no_existing,
    )

    fixed_now = _fixed_now()
    candidate = build_trial_directory(tmp_path, trial_number=1, now=fixed_now)
    candidate.mkdir(parents=True)

    with pytest.raises(ResultDirectoryExistsError):
        check_no_existing(tmp_path, trial_number=1, now=fixed_now)


def test_check_no_existing_passes_on_fresh_directory(tmp_path):
    from flying_car_test.run_trials import check_no_existing

    candidate = check_no_existing(tmp_path, trial_number=1, now=_fixed_now())
    assert not candidate.exists()


# ---------------------------------------------------------------------------
# ROS_LOG_DIR isolation
# ---------------------------------------------------------------------------


def test_allocate_ros_log_dir_is_per_trial(tmp_path):
    from flying_car_test.run_trials import allocate_ros_log_dir

    a = allocate_ros_log_dir(tmp_path / "trial-01")
    b = allocate_ros_log_dir(tmp_path / "trial-02")
    assert a.exists() and b.exists()
    assert str(a).startswith(str(tmp_path / "trial-01"))
    assert str(b).startswith(str(tmp_path / "trial-02"))


def test_unique_ros_log_env_is_per_trial(tmp_path):
    from flying_car_test.run_trials import unique_ros_log_env

    env_a = unique_ros_log_env(tmp_path / "trial-01")
    env_b = unique_ros_log_env(tmp_path / "trial-02")
    assert env_a["ROS_LOG_DIR"] != env_b["ROS_LOG_DIR"]
    assert env_a["ROS_LOG_DIR"].startswith(str(tmp_path / "trial-01"))
    assert env_b["ROS_LOG_DIR"].startswith(str(tmp_path / "trial-02"))


def test_single_trial_passes_isolated_launch_arguments(tmp_path):
    captured = {}

    def factory(**kwargs):
        runner = _CapturingLaunchRunner(**kwargs)
        captured["runner"] = runner
        return runner

    config = tmp_path / "experiment.yaml"
    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=config,
        gui=True,
        model_name="test-model",
    )

    outcome = RUNNER_MODULE._run_single_trial(
        tmp_path,
        2,
        args=args,
        env_overrides={},
        port_prober=_StaticProbe(),
        launch_runner_factory=factory,
        now=_fixed_now(),
    )

    command = captured["runner"].kwargs["command"]
    assert "result_dir:={}".format(outcome.trial_dir) in command
    assert "config:={}".format(config) in command
    assert "rosbag_name:=trial-02" in command
    assert "gui:=true" in command
    assert "model_name:=test-model" in command
    assert captured["runner"].kwargs["stdout_log"] == (
        outcome.trial_dir / "roslaunch.log"
    )
    assert captured["runner"].kwargs["stderr_log"] == (
        outcome.trial_dir / "roslaunch.log"
    )


def test_single_trial_env_inherits_parent_and_applies_overrides(
    monkeypatch, tmp_path
):
    captured = {}

    def factory(**kwargs):
        captured.update(kwargs)
        return _CapturingLaunchRunner(**kwargs)

    monkeypatch.setenv("TRIAL_RUNNER_BASELINE", "inherited")
    monkeypatch.setenv("TRIAL_RUNNER_OVERRIDE", "baseline")
    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=tmp_path / "experiment.yaml",
        gui=False,
        model_name="test-model",
    )

    RUNNER_MODULE._run_single_trial(
        tmp_path,
        1,
        args=args,
        env_overrides={"TRIAL_RUNNER_OVERRIDE": "trial"},
        port_prober=_StaticProbe(),
        launch_runner_factory=factory,
        now=_fixed_now(),
    )

    child_env = captured["env"]
    assert child_env["TRIAL_RUNNER_BASELINE"] == "inherited"
    assert child_env["TRIAL_RUNNER_OVERRIDE"] == "trial"
    assert child_env["ROS_LOG_DIR"] == str(
        tmp_path / "2026-07-30T123456Z" / "trial-01" / "ros_log"
    )


# ---------------------------------------------------------------------------
# Preflight / port probing
# ---------------------------------------------------------------------------


def test_preflight_blocks_when_occupied_ports(tmp_path):
    from flying_car_test.run_trials import preflight

    probe = _StaticProbe(occupied=DEFAULT_REQUIRED_PORTS)
    with pytest.raises(RuntimeError) as excinfo:
        preflight(
            out_root=tmp_path,
            port_prober=probe,
            required_ports=DEFAULT_REQUIRED_PORTS,
            disk_free_bytes=10 ** 12,
            minimum_free_bytes=10 ** 9,
            required_commands=(),
            required_paths=(),
        )
    assert "port" in str(excinfo.value).lower()


def test_preflight_passes_when_clean(tmp_path):
    from flying_car_test.run_trials import preflight

    probe = _StaticProbe()
    outcome = preflight(
        out_root=tmp_path,
        port_prober=probe,
        required_ports=DEFAULT_REQUIRED_PORTS,
        disk_free_bytes=10 ** 12,
        minimum_free_bytes=10 ** 9,
        required_commands=(),
        required_paths=(),
    )
    assert outcome.ok
    assert not outcome.occupied_ports
    assert outcome.missing_commands == ()


def test_preflight_blocks_on_missing_command(tmp_path):
    from flying_car_test.run_trials import preflight

    with pytest.raises(RuntimeError) as excinfo:
        preflight(
            out_root=tmp_path,
            port_prober=_StaticProbe(),
            required_ports=(),
            disk_free_bytes=10 ** 12,
            minimum_free_bytes=10 ** 9,
            required_commands=("definitely-not-a-real-command-xyz",),
            required_paths=(),
        )
    assert "command" in str(excinfo.value).lower()


def test_preflight_blocks_on_low_disk(tmp_path):
    from flying_car_test.run_trials import preflight

    with pytest.raises(RuntimeError) as excinfo:
        preflight(
            out_root=tmp_path,
            port_prober=_StaticProbe(),
            required_ports=(),
            disk_free_bytes=10 ** 6,
            minimum_free_bytes=10 ** 12,
            required_commands=(),
            required_paths=(),
        )
    assert "disk" in str(excinfo.value).lower()


def test_preflight_blocks_on_existing_output(tmp_path):
    from flying_car_test.run_trials import preflight

    (tmp_path / "occupied").mkdir()
    with pytest.raises(RuntimeError) as excinfo:
        preflight(
            out_root=tmp_path,
            port_prober=_StaticProbe(),
            required_ports=(),
            disk_free_bytes=10 ** 12,
            minimum_free_bytes=10 ** 9,
            required_commands=(),
            required_paths=(),
        )
    assert "output" in str(excinfo.value).lower()


def test_preflight_fails_closed_when_px4_package_is_unresolvable(tmp_path):
    def resolver(package_name):
        if package_name == "px4":
            raise RUNNER_MODULE.TrialRunnerError("not found")
        return tmp_path / package_name

    with pytest.raises(RUNNER_MODULE.TrialRunnerError) as excinfo:
        RUNNER_MODULE.preflight(
            out_root=tmp_path / "results",
            port_prober=_StaticProbe(),
            required_ports=(),
            disk_free_bytes=10 ** 12,
            minimum_free_bytes=1,
            required_commands=(),
            required_paths=(),
            required_packages=("px4", "flying_car_test"),
            package_resolver=resolver,
        )

    message = str(excinfo.value)
    assert "px4" in message
    assert "ROS_PACKAGE_PATH" in message


# ---------------------------------------------------------------------------
# Process-group isolation
# ---------------------------------------------------------------------------


def test_launch_runner_spawns_with_new_session(monkeypatch, tmp_path):
    from flying_car_test import run_trials

    captured = {}

    class FakePopen:
        def __init__(self, command, **kwargs):
            captured["start_new_session"] = kwargs.get("start_new_session")
            captured["shared_log"] = kwargs["stdout"] is kwargs["stderr"]
            self.pid = 4242

    monkeypatch.setattr(run_trials.subprocess, "Popen", FakePopen)

    runner = run_trials.LaunchRunner(
        command=["roslaunch", "pkg", "f.launch"],
        env={"PATH": "/bin"},
        result_dir=tmp_path,
        stdout_log=tmp_path / "roslaunch.log",
        stderr_log=tmp_path / "roslaunch.log",
        ros_log_dir=tmp_path / "log",
        poll_interval_s=0.01,
    )
    runner._spawn()
    assert captured["start_new_session"] is True
    assert captured["shared_log"] is True


def test_request_shutdown_signals_only_trial_roslaunch(monkeypatch):
    from flying_car_test.run_trials import LaunchRunner

    class _FakePopen:
        pid = 123

        def __init__(self):
            self.sent = []

        def send_signal(self, signal_value):
            self.sent.append(signal_value)

    proc = _FakePopen()
    runner = LaunchRunner.__new__(LaunchRunner)
    runner._proc = proc
    runner.pid = proc.pid
    monkeypatch.setattr(
        RUNNER_MODULE.subprocess,
        "run",
        lambda *args, **kwargs: pytest.fail(
            "normal shutdown must not invoke a global ROS command"
        ),
    )

    runner.request_shutdown()

    assert proc.sent == [signal.SIGINT]


def test_signal_group_targets_only_one_trial(monkeypatch):
    """The runner must scope signal delivery to its own process group."""
    from flying_car_test.run_trials import LaunchRunner

    class _FakePopen:
        pid = 123

        def send_signal(self, signal_value):
            raise AssertionError("fallback must signal the process group")

    proc = _FakePopen()
    runner = LaunchRunner.__new__(LaunchRunner)
    runner._proc = proc
    runner.pid = proc.pid
    sent = []
    monkeypatch.setattr(
        RUNNER_MODULE.os,
        "killpg",
        lambda pgid, signal_value: sent.append((pgid, signal_value)),
    )

    assert runner.signal_group(signal.SIGTERM) is True
    assert sent == [(123, signal.SIGTERM)]


def test_shutdown_escalates_after_bounded_wait_timeouts(monkeypatch):
    from flying_car_test.run_trials import LaunchRunner

    class _FakePopen:
        pid = 456

        def __init__(self):
            self.signals = []
            self.wait_count = 0

        def send_signal(self, signal_value):
            self.signals.append(signal_value)

        def wait(self, timeout=None):
            self.wait_count += 1
            if self.wait_count < 3:
                raise subprocess.TimeoutExpired("roslaunch", timeout)
            return -signal.SIGKILL

    proc = _FakePopen()
    runner = LaunchRunner.__new__(LaunchRunner)
    runner._proc = proc
    runner.pid = proc.pid
    group_signals = []
    monkeypatch.setattr(
        RUNNER_MODULE.os,
        "killpg",
        lambda pgid, signal_value: group_signals.append(
            (pgid, signal_value)
        ),
    )

    returncode = runner.shutdown(
        graceful_timeout_s=0.01,
        term_timeout_s=0.01,
        kill_timeout_s=0.01,
    )

    assert proc.signals == [signal.SIGINT]
    assert group_signals == [
        (456, signal.SIGTERM),
        (456, signal.SIGKILL),
    ]
    assert returncode == -signal.SIGKILL


def test_completion_marker_wait_honors_call_timeout(monkeypatch, tmp_path):
    class _StillRunning:
        def poll(self):
            return None

    runner = RUNNER_MODULE.LaunchRunner(
        command=["roslaunch"],
        env={},
        result_dir=tmp_path,
        stdout_log=tmp_path / "out.log",
        stderr_log=tmp_path / "err.log",
        ros_log_dir=tmp_path / "ros_log",
        timeout_s=None,
    )
    runner._proc = _StillRunning()
    times = iter((0.0, 1.0))
    monkeypatch.setattr(
        RUNNER_MODULE.time, "monotonic", lambda: next(times)
    )
    monkeypatch.setattr(
        RUNNER_MODULE.time,
        "sleep",
        lambda _seconds: pytest.fail("timeout must stop marker polling"),
    )

    assert runner.poll_completion_marker(timeout_s=0.5) is None


def test_run_trials_fails_closed_before_next_trial_if_ports_stay_bound(
    tmp_path
):
    required_ports = (
        ("udp", "127.0.0.1", 14540),
        ("tcp", "127.0.0.1", 11345),
    )

    class _PortsBindAfterPreflight:
        def __init__(self):
            self.calls = 0

        def is_occupied(self, host, port, protocol="udp"):
            self.calls += 1
            return self.calls > len(required_ports)

    launches = []

    def factory(**kwargs):
        launches.append(kwargs["result_dir"])
        return _CapturingLaunchRunner(**kwargs)

    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=tmp_path / "experiment.yaml",
        gui=False,
        model_name="test-model",
        port_release_timeout_s=0.0,
    )

    with pytest.raises(RUNNER_MODULE.TrialRunnerError, match="ports"):
        RUNNER_MODULE.run_trials(
            tmp_path,
            trial_count=2,
            args=args,
            env_overrides={},
            port_prober=_PortsBindAfterPreflight(),
            disk_free_bytes=10 ** 12,
            launch_runner_factory=factory,
            required_ports=required_ports,
            required_commands=(),
            minimum_free_bytes=1,
            now=_fixed_now(),
        )

    assert launches == [
        tmp_path / "2026-07-30T123456Z" / "trial-01"
    ]


# ---------------------------------------------------------------------------
# ULog collection, validation, and reporting
# ---------------------------------------------------------------------------


def test_locate_trial_ulog_excludes_logs_from_before_launch(tmp_path):
    ulog_root = tmp_path / "rootfs" / "log"
    old_ulog = ulog_root / "2026-07-30" / "old.ulg"
    new_ulog = ulog_root / "2026-07-30" / "new.ulg"
    old_ulog.parent.mkdir(parents=True)
    old_ulog.write_bytes(b"old")
    new_ulog.write_bytes(b"new")
    os.utime(old_ulog, ns=(1_000_000_000, 1_000_000_000))
    os.utime(new_ulog, ns=(3_000_000_000, 3_000_000_000))

    found = RUNNER_MODULE.locate_trial_ulog(
        ulog_root,
        launched_at_ns=2_000_000_000,
        paths_existing_at_launch={old_ulog.resolve()},
    )

    assert found == new_ulog


def test_process_trial_artifacts_copies_ulog_and_propagates_verdicts(
    tmp_path
):
    trial_dir = tmp_path / "trial-01"
    trial_dir.mkdir()
    source_ulog = tmp_path / "source" / "flight.ulg"
    source_ulog.parent.mkdir()
    source_ulog.write_bytes(b"ulog-evidence")
    config = tmp_path / "experiment.yaml"
    log_rules = tmp_path / "log_rules.yaml"
    config.write_text("model_name: test\n", encoding="utf-8")
    log_rules.write_text("rules: []\n", encoding="utf-8")

    def validation_entrypoint(argv):
        output = Path(argv[argv.index("--output") + 1])
        output.write_text('{"overall":"FAIL"}', encoding="utf-8")
        return 7

    def reporting_entrypoint(argv):
        output_dir = Path(argv[argv.index("--output-dir") + 1])
        (output_dir / "report.md").write_text(
            "trial report", encoding="utf-8"
        )
        return 0

    result = RUNNER_MODULE.process_trial_artifacts(
        trial_dir=trial_dir,
        ulog_source=source_ulog,
        config_path=config,
        log_rules_path=log_rules,
        validation_entrypoint=validation_entrypoint,
        reporting_entrypoint=reporting_entrypoint,
    )

    assert result.ulog == trial_dir / "flight.ulg"
    assert result.ulog.read_bytes() == b"ulog-evidence"
    assert result.validation_exit == 7
    assert result.reporting_exit == 0
    assert (trial_dir / "validation.json").is_file()
    assert (trial_dir / "report.md").is_file()


def test_process_trial_artifacts_missing_ulog_cannot_report_success(tmp_path):
    trial_dir = tmp_path / "trial-01"
    trial_dir.mkdir()

    result = RUNNER_MODULE.process_trial_artifacts(
        trial_dir=trial_dir,
        ulog_source=None,
        config_path=tmp_path / "experiment.yaml",
        log_rules_path=tmp_path / "log_rules.yaml",
    )

    assert result.ulog is None
    assert result.validation_exit != 0
    assert result.reporting_exit != 0
    assert (trial_dir / "artifact_error.txt").is_file()
    assert not (trial_dir / "validation.json").exists()


def test_single_trial_collects_post_launch_ulog_and_propagates_validation(
    monkeypatch, tmp_path
):
    ulog_root = tmp_path / "rootfs" / "log"
    ulog_root.mkdir(parents=True)
    old_ulog = ulog_root / "old.ulg"
    old_ulog.write_bytes(b"old")
    os.utime(old_ulog, ns=(1_000_000_000, 1_000_000_000))

    class _UlogProducingRunner(_CapturingLaunchRunner):
        def start(self):
            super().start()
            new_ulog = ulog_root / "new.ulg"
            new_ulog.write_bytes(b"current")
            os.utime(new_ulog, ns=(3_000_000_000, 3_000_000_000))

    def factory(**kwargs):
        return _UlogProducingRunner(**kwargs)

    def validation_entrypoint(argv):
        output = Path(argv[argv.index("--output") + 1])
        output.write_text('{"overall":"FAIL"}', encoding="utf-8")
        return 5

    def reporting_entrypoint(argv):
        output_dir = Path(argv[argv.index("--output-dir") + 1])
        (output_dir / "report.md").write_text("report", encoding="utf-8")
        return 0

    monkeypatch.setattr(RUNNER_MODULE.time, "time_ns", lambda: 2_000_000_000)
    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=tmp_path / "experiment.yaml",
        log_rules=tmp_path / "log_rules.yaml",
        gui=False,
        model_name="test-model",
        ulog_root=ulog_root,
    )

    outcome = RUNNER_MODULE._run_single_trial(
        tmp_path,
        1,
        args=args,
        env_overrides={},
        port_prober=_StaticProbe(),
        launch_runner_factory=factory,
        validation_entrypoint=validation_entrypoint,
        reporting_entrypoint=reporting_entrypoint,
        now=_fixed_now(),
    )

    assert outcome.ulog == outcome.trial_dir / "new.ulg"
    assert outcome.ulog.read_bytes() == b"current"
    assert outcome.validation_exit == 5
    assert outcome.report == str(outcome.trial_dir / "report.md")


def test_generate_aggregate_report_uses_trial_parent_and_propagates_exit(
    tmp_path
):
    run_dir = tmp_path / "2026-07-30T123456Z"
    trial_dirs = [run_dir / "trial-01", run_dir / "trial-02"]
    for trial_dir in trial_dirs:
        trial_dir.mkdir(parents=True)

    def reporting_entrypoint(argv):
        assert argv.count("--result-dir") == 2
        output_dir = Path(argv[argv.index("--output-dir") + 1])
        (output_dir / "aggregate_report.md").write_text(
            "aggregate", encoding="utf-8"
        )
        return 9

    exit_code = RUNNER_MODULE.generate_aggregate_report(
        trial_dirs=trial_dirs,
        config_path=tmp_path / "experiment.yaml",
        log_rules_path=tmp_path / "log_rules.yaml",
        reporting_entrypoint=reporting_entrypoint,
    )

    assert exit_code == 9
    assert (run_dir / "aggregate_report.md").read_text(
        encoding="utf-8"
    ) == "aggregate"


def test_generate_aggregate_report_exists_for_single_trial(tmp_path):
    run_dir = tmp_path / "2026-07-30T123456Z"
    trial_dir = run_dir / "trial-01"
    trial_dir.mkdir(parents=True)
    (trial_dir / "validation.json").write_text(
        '{"overall":"PASS","items":[]}',
        encoding="utf-8",
    )

    exit_code = RUNNER_MODULE.generate_aggregate_report(
        trial_dirs=[trial_dir],
        config_path=tmp_path / "experiment.yaml",
        log_rules_path=tmp_path / "log_rules.yaml",
    )

    assert exit_code == 0
    assert (run_dir / "aggregate_report.md").is_file()
    assert (run_dir / "aggregate_summary.json").is_file()


def test_run_trials_generates_aggregate_report(monkeypatch, tmp_path):
    output_root = tmp_path / "results"
    ulog_root = tmp_path / "rootfs" / "log"
    ulog_root.mkdir(parents=True)
    launches = []

    class _ProducingRunner(_CapturingLaunchRunner):
        def start(self):
            super().start()
            index = len(launches)
            ulog = ulog_root / "flight-{:02d}.ulg".format(index)
            ulog.write_bytes("ulog-{}".format(index).encode("ascii"))
            os.utime(ulog, ns=(3_000_000_000, 3_000_000_000))

    def factory(**kwargs):
        launches.append(kwargs["result_dir"])
        return _ProducingRunner(**kwargs)

    def validation_entrypoint(argv):
        output = Path(argv[argv.index("--output") + 1])
        output.write_text('{"overall":"PASS"}', encoding="utf-8")
        return 0

    def reporting_entrypoint(argv):
        result_count = argv.count("--result-dir")
        output_dir = Path(argv[argv.index("--output-dir") + 1])
        if result_count == 1:
            (output_dir / "report.md").write_text(
                "trial", encoding="utf-8"
            )
        else:
            (output_dir / "aggregate_report.md").write_text(
                "aggregate", encoding="utf-8"
            )
        return 0

    monkeypatch.setattr(RUNNER_MODULE.time, "time_ns", lambda: 2_000_000_000)
    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=tmp_path / "experiment.yaml",
        log_rules=tmp_path / "log_rules.yaml",
        gui=False,
        model_name="test-model",
        ulog_root=ulog_root,
        port_release_timeout_s=0.0,
    )

    outcomes = RUNNER_MODULE.run_trials(
        output_root,
        trial_count=2,
        args=args,
        env_overrides={},
        port_prober=_StaticProbe(),
        disk_free_bytes=10 ** 12,
        launch_runner_factory=factory,
        required_ports=(),
        required_commands=(),
        minimum_free_bytes=1,
        validation_entrypoint=validation_entrypoint,
        reporting_entrypoint=reporting_entrypoint,
        now=_fixed_now(),
    )

    assert len(outcomes) == 2
    assert (
        output_root
        / "2026-07-30T123456Z"
        / "aggregate_report.md"
    ).is_file()


def test_run_trials_snapshots_utc_timestamp_once_for_all_trials(
    monkeypatch, tmp_path
):
    instants = iter(
        (
            datetime.datetime(
                2026, 7, 30, 12, 34, 59, tzinfo=datetime.timezone.utc
            ),
            datetime.datetime(
                2026, 7, 30, 12, 35, 0, tzinfo=datetime.timezone.utc
            ),
        )
    )
    utc_calls = []

    def changing_utc_now():
        value = next(instants)
        utc_calls.append(value)
        return value

    monkeypatch.setattr(RUNNER_MODULE, "_utc_now", changing_utc_now)
    args = Namespace(
        launch_argv=["flying_car_test", "full_experiment.launch"],
        timeout_s=30.0,
        config=tmp_path / "experiment.yaml",
        log_rules=tmp_path / "log_rules.yaml",
        gui=False,
        model_name="test-model",
        ulog_root=tmp_path / "rootfs" / "log",
        port_release_timeout_s=0.0,
    )

    outcomes = RUNNER_MODULE.run_trials(
        tmp_path / "results",
        trial_count=2,
        args=args,
        env_overrides={},
        port_prober=_StaticProbe(),
        disk_free_bytes=10 ** 12,
        launch_runner_factory=lambda **kwargs: _CapturingLaunchRunner(
            **kwargs
        ),
        required_ports=(),
        required_commands=(),
        minimum_free_bytes=1,
        reporting_entrypoint=lambda _argv: 0,
    )

    expected_parent = (
        tmp_path / "results" / "2026-07-30T123459Z"
    )
    assert [outcome.trial_dir.parent for outcome in outcomes] == [
        expected_parent,
        expected_parent,
    ]
    assert utc_calls == [
        datetime.datetime(
            2026, 7, 30, 12, 34, 59, tzinfo=datetime.timezone.utc
        )
    ]


# ---------------------------------------------------------------------------
# Launch file contracts (full / monitor-only)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def full_root():
    if not FULL_LAUNCH.is_file():
        pytest.skip("full_experiment.launch not yet created")
    return _load_launch(FULL_LAUNCH)


@pytest.fixture(scope="module")
def monitor_root():
    if not MONITOR_LAUNCH.is_file():
        pytest.skip("monitor_only.launch not yet created")
    return _load_launch(MONITOR_LAUNCH)


def test_full_experiment_launch_includes_flying_car_simulation(full_root):
    files = [
        f for f in (_includes(full_root)) if f and "/launch/FS_flying_car.launch" in f
    ]
    assert files, "full_experiment.launch must include FS_flying_car.launch"


def test_full_experiment_launch_passes_no_timesync_mavros(full_root):
    nested = full_root.find("include")
    assert nested is not None
    args = {
        child.attrib["name"]: child.attrib["value"]
        for child in nested.findall("arg")
        if "name" in child.attrib and "value" in child.attrib
    }
    assert (
        args.get("mavros_launch")
        == "$(find flying_car_test)/launch/mavros_sitl_no_timesync.launch"
    )


def test_full_experiment_starts_monitor_before_controller(full_root):
    monitor = _node(full_root, name="flying_car_experiment_monitor")
    controller = _node(full_root, name="flying_car_experiment_controller")
    assert monitor is not None and controller is not None
    children = list(full_root)
    assert children.index(monitor) < children.index(controller)


def test_full_experiment_starts_safety_guard_before_controller(full_root):
    guard = _node(full_root, name="flying_car_safety_guard")
    controller = _node(full_root, name="flying_car_experiment_controller")
    assert guard is not None and controller is not None
    children = list(full_root)
    assert children.index(guard) < children.index(controller)


def test_full_experiment_controller_remaps(full_root):
    controller = _node(full_root, name="flying_car_experiment_controller")
    remaps = _remaps(controller)
    assert (
        "/mavros/manual_control/send",
        "/flying_car_safety_guard/controller/manual",
    ) in remaps
    assert (
        "/mavros/setpoint_position/local",
        "/flying_car_safety_guard/controller/pose",
    ) in remaps


def test_full_experiment_safety_guard_keeps_mavros_publishers(full_root):
    guard = _node(full_root, name="flying_car_safety_guard")
    assert guard is not None
    forbidden = {
        "/mavros/manual_control/send",
        "/mavros/setpoint_position/local",
    }
    for src, _dst in _remaps(guard):
        assert src not in forbidden, (
            "safety_guard node must not remap %s" % src
        )


def test_full_experiment_marks_required_nodes(full_root):
    for name in (
        "flying_car_experiment_controller",
        "flying_car_experiment_monitor",
        "flying_car_safety_guard",
    ):
        node = _node(full_root, name=name)
        assert node is not None, "missing required node %s" % name
        assert node.attrib.get("required", "false").lower() == "true"


def test_full_experiment_rosbag_topics(full_root):
    bag = _node(full_root, ty="record")
    assert bag is not None
    topics = set(bag.attrib["args"].split())
    required = {
        "/mavros/state",
        "/mavros/local_position/pose",
        "/mavros/local_position/velocity_local",
        "/flying_car_experiment_controller/command",
        "/flying_car_experiment_controller/phase",
        "/clock",
        "/gazebo/model_states",
    }
    assert required <= topics
    assert "/flying_car_safety_guard/controller/command" not in topics
    assert "/flying_car_experiment/phase" not in topics


def test_full_experiment_controller_param_passthrough(full_root):
    controllers = {
        "flying_car_experiment_controller",
        "flying_car_experiment_monitor",
        "flying_car_safety_guard",
    }
    for name in controllers:
        node = _node(full_root, name=name)
        assert node is not None, "missing %s" % name
        params = {p.attrib["name"]: p.attrib["value"] for p in node.findall("param")}
        assert "config_file" in params
        assert "result_dir" in params
        assert "model_name" in params


def test_monitor_only_launch_omits_controller(monitor_root):
    assert _node(monitor_root, name="flying_car_experiment_controller") is None
    assert _node(monitor_root, name="flying_car_experiment_monitor") is not None
    assert _node(monitor_root, name="flying_car_safety_guard") is not None


def test_monitor_only_launch_disables_guard_heartbeat(monitor_root):
    guard = _node(monitor_root, name="flying_car_safety_guard")
    assert guard is not None
    seen = False
    for child in guard.findall("param"):
        if child.attrib.get("name") == "heartbeat_required":
            assert child.attrib["value"].lower() in {"false", "0"}
            seen = True
            break
    assert seen, "monitor_only launch must set heartbeat_required=false"


def test_monitor_only_launch_rosbag_topics(monitor_root):
    bag = _node(monitor_root, ty="record")
    assert bag is not None
    topics = set(bag.attrib["args"].split())
    required = {
        "/mavros/state",
        "/mavros/local_position/pose",
        "/mavros/local_position/velocity_local",
        "/clock",
        "/gazebo/model_states",
        "/mavros/manual_control/control",
        "/mavros/rc/in",
        "/flying_car_test/stage_marker",
    }
    assert required <= topics
    assert "/flying_car_experiment_controller/command" not in topics
    assert "/flying_car_experiment_controller/phase" not in topics
    assert "/flying_car_safety_guard/controller/command" not in topics
    assert "/flying_car_experiment/phase" not in topics


def test_monitor_only_does_not_start_marker_cli_without_an_argument(
    monitor_root,
):
    assert _node(monitor_root, ty="stage_marker.py") is None


def test_monitor_only_launch_includes_flying_car(monitor_root):
    files = [
        f for f in (_includes(monitor_root)) if f and "/launch/FS_flying_car.launch" in f
    ]
    assert files, "monitor_only.launch must include FS_flying_car.launch"


# ---------------------------------------------------------------------------
# CLI smoke -- preflight-only mode does not spawn anything
# ---------------------------------------------------------------------------


def test_preflight_only_cli_returns_zero(monkeypatch, tmp_path, capsys):
    """--preflight-only should not invoke the orchestrator body."""
    from flying_car_test import run_trials

    spawned = {"flag": False}

    def _orchestrator(*args, **kwargs):
        spawned["flag"] = True
        raise AssertionError("preflight-only must not run the orchestrator")

    monkeypatch.setattr(run_trials, "default_port_prober", lambda: _StaticProbe())
    monkeypatch.setattr(
        run_trials,
        "default_disk_free_bytes",
        lambda path: 10 ** 12,
        raising=False,
    )
    monkeypatch.setattr(run_trials, "run_trials", _orchestrator)
    package_roots = {
        "px4": REPO_ROOT,
        "flying_car_test": FLYING_CAR_DIR,
    }
    monkeypatch.setattr(
        run_trials,
        "resolve_ros_package",
        lambda package_name: package_roots[package_name],
        raising=False,
    )

    rc = run_trials.main(
        argv=[
            "--output-root",
            str(tmp_path),
            "--preflight-only",
            "--trials",
            "1",
        ],
    )
    captured = capsys.readouterr()
    assert rc == 0
    assert spawned["flag"] is False
    assert "preflight" in (captured.out or captured.err).lower()


def test_output_root_and_out_root_are_cli_aliases(tmp_path):
    parser = RUNNER_MODULE._build_arg_parser()

    canonical = parser.parse_args(
        ["--output-root", str(tmp_path / "canonical")]
    )
    compatibility = parser.parse_args(
        ["--out-root", str(tmp_path / "compatibility")]
    )

    assert canonical.output_root == tmp_path / "canonical"
    assert compatibility.output_root == tmp_path / "compatibility"


def test_preflight_only_cli_rejects_missing_custom_mavros_launch(
    monkeypatch, tmp_path, capsys
):
    px4_root = tmp_path / "px4"
    flying_car_root = tmp_path / "flying_car_test"
    (px4_root / "launch").mkdir(parents=True)
    (px4_root / "launch" / "FS_flying_car.launch").write_text(
        "<launch/>", encoding="utf-8"
    )
    (flying_car_root / "launch").mkdir(parents=True)
    (flying_car_root / "config").mkdir()
    config = flying_car_root / "config" / "experiment.yaml"
    config.write_text("model_name: test\n", encoding="utf-8")
    (flying_car_root / "config" / "log_rules.yaml").write_text(
        "rules: []\n", encoding="utf-8"
    )
    roots = {"px4": px4_root, "flying_car_test": flying_car_root}
    monkeypatch.setattr(
        RUNNER_MODULE,
        "resolve_ros_package",
        lambda package_name: roots[package_name],
        raising=False,
    )
    monkeypatch.setattr(
        RUNNER_MODULE, "default_port_prober", lambda: _StaticProbe()
    )
    monkeypatch.setattr(
        RUNNER_MODULE,
        "default_disk_free_bytes",
        lambda _path: 10 ** 12,
    )

    rc = RUNNER_MODULE.main(
        [
            "--output-root",
            str(tmp_path / "results"),
            "--config",
            str(config),
            "--preflight-only",
        ]
    )

    captured = capsys.readouterr()
    assert rc != 0
    assert "mavros_sitl_no_timesync.launch" in captured.err
