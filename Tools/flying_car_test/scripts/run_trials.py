#!/usr/bin/env python3
"""Orchestrator for isolated flying-car experiment trials.

The :func:`run_trials` function (and the ``scripts/run_trials.py`` wrapper
around it) drives a deterministic sequence of trial runs:

1. ``preflight`` verifies required commands, packages, ports, free disk, and
   a fresh output directory;
2. For each trial the orchestrator builds a unique ``trial-NN`` directory
   under a UTC-timestamped parent, copies MAVROS / PX4 / Gazebo state with a
   dedicated ``ROS_LOG_DIR``, spawns ``roslaunch`` in its own process group
   (``subprocess.Popen(..., start_new_session=True)``), and writes
   ``roslaunch.stdout.log`` / ``roslaunch.stderr.log`` next to it;
3. After the running monitor publishes ``monitor_complete.json`` (or a
   configurable timeout elapses) the orchestrator triggers the normal ROS
   shutdown and only escalates to ``SIGTERM``/``SIGKILL`` of the trial's own
   process group if shutdown stalls;
4. The orchestrator copies the PX4 ``.ulg`` evidence file (if found), runs
   :mod:`flying_car_test.validation` and :mod:`flying_car_test.reporting`,
   and finally waits until every required PX4/MAVROS/Gazebo port is free
   before starting the next trial.

The module is intentionally pure Python and free of broad destructive
operations. Normal teardown signals only the trial's roslaunch parent;
bounded fallback escalation targets only the new process group created for
that trial.
"""

from __future__ import annotations

import argparse
import json
import datetime
import errno
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)


# ---------------------------------------------------------------------------
# Errors / dataclasses
# ---------------------------------------------------------------------------


class TrialRunnerError(RuntimeError):
    """Raised on preflight, port, or directory violations."""


class ResultDirectoryExistsError(TrialRunnerError):
    """Raised when the trial directory already exists."""


@dataclass(frozen=True)
class PreflightResult:
    """Return value of :func:`preflight` for the happy path."""

    ok: bool
    missing_commands: Tuple[str, ...] = ()
    occupied_ports: Tuple[Tuple[str, str, int], ...] = ()
    failed_check: Optional[str] = None
    detail: str = ""


@dataclass(frozen=True)
class TrialOutcome:
    """Per-trial verdict surfaced from :func:`run_trials`."""

    trial_dir: Path
    ulog: Optional[Path]
    returncode: int
    report: str
    validation_exit: int


@dataclass(frozen=True)
class ArtifactResult:
    """Evidence collection and post-processing result for one trial."""

    ulog: Optional[Path]
    validation_exit: int
    reporting_exit: int
    report_path: Optional[Path]


@dataclass
class _LaunchHandles:
    proc: Any
    stdout_log: Path
    stderr_log: Path
    ros_log_dir: Path
    pid: Optional[int] = None


# ---------------------------------------------------------------------------
# Trial directory + ROS_LOG_DIR helpers
# ---------------------------------------------------------------------------


def _utc_now() -> datetime.datetime:
    return datetime.datetime.now(tz=datetime.timezone.utc)


def build_trial_directory(
    out_root: Path,
    trial_number: int,
    now: Optional[datetime.datetime] = None,
) -> Path:
    """Return ``out_root / <UTC-stamp> / trial-NN``.

    ``trial_number`` is 1-based and zero-padded to two digits ("trial-01").
    """
    moment = now or _utc_now()
    if moment.tzinfo is None:
        moment = moment.replace(tzinfo=datetime.timezone.utc)
    stamp = moment.astimezone(datetime.timezone.utc).strftime("%Y-%m-%dT%H%M%SZ")
    return Path(out_root) / stamp / f"trial-{int(trial_number):02d}"


def check_no_existing(
    out_root: Path,
    trial_number: int,
    now: Optional[datetime.datetime] = None,
    *,
    fs: Optional[Callable[[Path], bool]] = None,
) -> Path:
    """Verify that the trial directory is fresh.

    The default ``fs`` callable uses ``Path.exists``.
    """
    candidate = build_trial_directory(out_root, trial_number, now=now)
    if fs is None:
        exists = candidate.exists()
    else:
        exists = bool(fs(candidate))
    if exists:
        raise ResultDirectoryExistsError(
            "trial directory already exists: {}".format(candidate)
        )
    return candidate


def allocate_ros_log_dir(trial_dir: Path) -> Path:
    """Create (and return) the per-trial ``ROS_LOG_DIR``."""
    path = Path(trial_dir) / "ros_log"
    path.mkdir(parents=True, exist_ok=True)
    return path


def unique_ros_log_env(trial_dir: Path) -> Dict[str, str]:
    """Return an env override mapping containing only ``ROS_LOG_DIR``."""
    ros_log_dir = allocate_ros_log_dir(Path(trial_dir))
    return {"ROS_LOG_DIR": str(ros_log_dir)}


# ---------------------------------------------------------------------------
# Port probing / disk usage
# ---------------------------------------------------------------------------


def default_port_prober() -> "PortProber":
    return PortProber()


def default_disk_free_bytes(path: Path) -> int:
    """Return free bytes available to the filesystem containing *path*."""
    probe = Path(path)
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    usage = shutil.disk_usage(str(probe))
    return int(usage.free)


class PortProber:
    """Determine whether a UDP/TCP endpoint is bound on the local host."""

    def __init__(
        self,
        socket_factory: Optional[Callable[[int, int], socket.socket]] = None,
        timeout_s: float = 0.25,
    ):
        self._factory = socket_factory or socket.socket
        self._timeout_s = float(timeout_s)

    def is_occupied(self, host: str, port: int, protocol: str = "udp") -> bool:
        proto = (protocol or "udp").lower()
        sock_type = socket.SOCK_DGRAM if proto == "udp" else socket.SOCK_STREAM
        sock = self._factory(socket.AF_INET, sock_type)
        try:
            # Without SO_REUSEADDR a TCP port lingering in TIME_WAIT after a
            # previous trial still fails bind() with EADDRINUSE even though a
            # new listener can bind immediately; treat that as free.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError:
            pass
        try:
            sock.settimeout(self._timeout_s)
        except OSError:
            pass
        try:
            try:
                sock.bind((host, int(port)))
            except OSError as error:
                if error.errno in (
                    errno.EADDRINUSE,
                    errno.EACCES,
                ):
                    return True
                # ``EADDRNOTAVAIL`` / ``ENOTSUP`` count as free for our purposes.
                return False
            if proto == "tcp":
                try:
                    sock.listen(1)
                except OSError as error:
                    if error.errno in (errno.EADDRINUSE, errno.EACCES):
                        return True
                    return False
            return False
        finally:
            try:
                sock.close()
            except OSError:
                pass


def _command_exists(name: str) -> bool:
    return shutil.which(str(name)) is not None


def resolve_ros_package(package_name: str) -> Path:
    """Resolve a ROS package with rospack or raise a preflight error."""
    try:
        completed = subprocess.run(
            ["rospack", "find", str(package_name)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5.0,
            env=os.environ.copy(),
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise TrialRunnerError(
            "cannot resolve ROS package '{}': {}".format(package_name, error)
        ) from error
    path_text = (completed.stdout or "").strip()
    if completed.returncode != 0 or not path_text:
        detail = (completed.stderr or "").strip()
        raise TrialRunnerError(
            "ROS package '{}' is not resolvable: {}".format(
                package_name, detail or "rospack find failed"
            )
        )
    return Path(path_text)


def preflight(
    out_root: Path,
    *,
    port_prober: PortProber,
    required_ports: Iterable[Tuple[str, str, int]] = (),
    disk_free_bytes: Optional[int] = None,
    minimum_free_bytes: int = 10 ** 10,
    required_commands: Iterable[str] = (),
    required_paths: Iterable[Path] = (),
    required_packages: Iterable[str] = (),
    required_package_files: Iterable[Tuple[str, Path]] = (),
    package_resolver: Optional[Callable[[str], Path]] = None,
    now: Optional[datetime.datetime] = None,
) -> PreflightResult:
    """Check that the host is ready for a new round of trials.

    Raises :class:`TrialRunnerError` with a precise reason; returns a
    :class:`PreflightResult` summarizing the check on success.
    """
    del now  # retained in signature for symmetry / future use

    out_root_path = Path(out_root)
    if out_root_path.exists():
        if not out_root_path.is_dir():
            raise TrialRunnerError(
                "output path exists and is not a directory: {}".format(out_root_path)
            )
        if any(out_root_path.iterdir()):
            raise TrialRunnerError(
                "output directory already contains files: {}".format(out_root_path)
            )

    missing_commands = tuple(
        sorted(name for name in required_commands if not _command_exists(name))
    )
    if missing_commands:
        raise TrialRunnerError(
            "required command(s) missing on PATH: {}".format(
                ", ".join(missing_commands)
            )
        )

    missing_paths = [
        str(Path(path))
        for path in required_paths
        if not Path(path).exists()
    ]
    if missing_paths:
        raise TrialRunnerError(
            "required path(s) missing: {}".format(", ".join(missing_paths))
        )

    resolver = package_resolver or resolve_ros_package
    package_roots: Dict[str, Path] = {}
    for package_name in required_packages:
        try:
            package_root = Path(resolver(str(package_name)))
        except Exception as error:
            raise TrialRunnerError(
                "required ROS package '{}' is not resolvable; source the "
                "workspace setup and ensure the PX4 repository is present "
                "in ROS_PACKAGE_PATH: {}".format(package_name, error)
            ) from error
        if not package_root.is_dir():
            raise TrialRunnerError(
                "resolved ROS package '{}' path does not exist: {}".format(
                    package_name, package_root
                )
            )
        package_roots[str(package_name)] = package_root

    missing_package_files = []
    for package_name, relative_path in required_package_files:
        package_key = str(package_name)
        if package_key not in package_roots:
            try:
                package_roots[package_key] = Path(resolver(package_key))
            except Exception as error:
                raise TrialRunnerError(
                    "required ROS package '{}' is not resolvable; source the "
                    "workspace setup and ensure the PX4 repository is present "
                    "in ROS_PACKAGE_PATH: {}".format(package_key, error)
                ) from error
        candidate = package_roots[package_key] / Path(relative_path)
        if not candidate.is_file():
            missing_package_files.append(str(candidate))
    if missing_package_files:
        raise TrialRunnerError(
            "required ROS package file(s) missing: {}".format(
                ", ".join(missing_package_files)
            )
        )

    occupied_ports: List[Tuple[str, str, int]] = []
    for protocol, host, port in required_ports:
        if port_prober.is_occupied(host, int(port), protocol=protocol):
            occupied_ports.append((str(protocol), str(host), int(port)))
    if occupied_ports:
        formatted = ", ".join(
            "{0}://{1}:{2}".format(*entry) for entry in occupied_ports
        )
        raise TrialRunnerError(
            "required PX4/MAVROS/Gazebo ports already bound: {}".format(formatted)
        )

    free_bytes = (
        int(disk_free_bytes)
        if disk_free_bytes is not None
        else default_disk_free_bytes(out_root_path)
    )
    if int(free_bytes) < int(minimum_free_bytes):
        raise TrialRunnerError(
            "insufficient free disk: {} < {}".format(free_bytes, minimum_free_bytes)
        )

    return PreflightResult(ok=True)


# ---------------------------------------------------------------------------
# Launch runner (process-group isolated)
# ---------------------------------------------------------------------------


@dataclass
class LaunchRunner:
    """Encapsulates a single roslaunch invocation and its teardown."""

    command: Sequence[str]
    env: Mapping[str, str]
    result_dir: Path
    stdout_log: Path
    stderr_log: Path
    ros_log_dir: Path
    poll_interval_s: float = 0.5
    timeout_s: Optional[float] = None
    completion_marker: str = "monitor_complete.json"
    _proc: Any = field(default=None, init=False, repr=False)
    pid: Optional[int] = field(default=None, init=False, repr=False)

    def _spawn(self) -> None:
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.stdout_log.parent.mkdir(parents=True, exist_ok=True)
        self.stderr_log.parent.mkdir(parents=True, exist_ok=True)
        stdout_handle = self.stdout_log.open("ab")
        stderr_handle = (
            stdout_handle
            if self.stderr_log == self.stdout_log
            else self.stderr_log.open("ab")
        )
        try:
            self._proc = subprocess.Popen(
                list(self.command),
                env=dict(self.env),
                cwd=str(self.result_dir),
                stdout=stdout_handle,
                stderr=stderr_handle,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        finally:
            try:
                stdout_handle.close()
            except Exception:
                pass
            if stderr_handle is not stdout_handle:
                try:
                    stderr_handle.close()
                except Exception:
                    pass
        self.pid = int(self._proc.pid) if self._proc is not None else None

    def start(self) -> None:
        if self._proc is None:
            self._spawn()

    def wait(self, timeout: Optional[float] = None) -> int:
        if self._proc is None:
            raise TrialRunnerError("launch runner was never started")
        return int(self._proc.wait(timeout=timeout))

    def poll(self) -> Optional[int]:
        if self._proc is None:
            return None
        return self._proc.poll()

    def poll_completion_marker(self, timeout_s: float) -> Optional[Path]:
        """Return the path to ``monitor_complete.json`` once it appears."""
        deadline = time.monotonic() + float(timeout_s)
        marker = self.result_dir / self.completion_marker
        while True:
            if marker.is_file():
                return marker
            if time.monotonic() > deadline:
                return None
            if self._proc is not None and self._proc.poll() is not None:
                # Process died -- still wait briefly for the marker to land.
                end_time = time.monotonic() + 1.0
                while time.monotonic() < end_time:
                    if marker.is_file():
                        return marker
                    time.sleep(self.poll_interval_s)
                return None
            time.sleep(self.poll_interval_s)

    def request_shutdown(self) -> None:
        """Ask this trial's roslaunch parent to shut down its own nodes."""
        if self._proc is None:
            return
        try:
            self._proc.send_signal(int(signal.SIGINT))
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def signal_group(self, signal_value: int) -> bool:
        """Send *signal_value* only to *this* run's process group."""
        if self._proc is None or self.pid is None:
            return False
        try:
            os.killpg(int(self.pid), int(signal_value))
            return True
        except (ProcessLookupError, PermissionError, OSError):
            return False

    def terminate(self) -> bool:
        return self.signal_group(int(signal.SIGTERM))

    def kill(self) -> bool:
        return self.signal_group(int(signal.SIGKILL))

    def shutdown(
        self,
        *,
        graceful_timeout_s: float = 20.0,
        term_timeout_s: float = 5.0,
        kill_timeout_s: float = 5.0,
    ) -> int:
        """Perform bounded roslaunch shutdown and process-group escalation."""
        self.request_shutdown()
        try:
            return self.wait(timeout=graceful_timeout_s)
        except subprocess.TimeoutExpired:
            self.terminate()
        try:
            return self.wait(timeout=term_timeout_s)
        except subprocess.TimeoutExpired:
            self.kill()
        try:
            return self.wait(timeout=kill_timeout_s)
        except subprocess.TimeoutExpired as error:
            raise TrialRunnerError(
                "trial process group did not exit after SIGKILL"
            ) from error

def wait_for_ports_free(
    port_prober: PortProber,
    required_ports: Iterable[Tuple[str, str, int]],
    *,
    timeout_s: float,
    poll_interval_s: float = 0.5,
    monotonic: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> None:
    """Wait until every guarded port is free, or fail closed."""
    ports = tuple(required_ports)
    deadline = monotonic() + max(0.0, float(timeout_s))
    while True:
        occupied = [
            (protocol, host, int(port))
            for protocol, host, port in ports
            if port_prober.is_occupied(host, int(port), protocol=protocol)
        ]
        if not occupied:
            return
        remaining = deadline - monotonic()
        if remaining <= 0:
            formatted = ", ".join(
                "{0}://{1}:{2}".format(*entry) for entry in occupied
            )
            raise TrialRunnerError(
                "required ports were not released after trial cleanup: "
                + formatted
            )
        sleeper(min(float(poll_interval_s), remaining))


def default_ulog_root() -> Path:
    """Return PX4 SITL's logger output root for this repository.

    The SITL launch runs ``px4`` from the ROS home directory, so the
    logger's ``./log`` output lands under ``~/.ros/log`` rather than the
    build tree's ``rootfs/log``.
    """
    ros_home_log = Path.home() / ".ros" / "log"
    if ros_home_log.is_dir():
        return ros_home_log
    repository_root = Path(__file__).resolve().parents[3]
    return repository_root / "build" / "px4_sitl_default" / "rootfs" / "log"


def default_package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_config_path() -> Path:
    return default_package_root() / "config" / "experiment.yaml"


def default_log_rules_path() -> Path:
    return default_package_root() / "config" / "log_rules.yaml"


def snapshot_ulog_paths(ulog_root: Path) -> Tuple[Path, ...]:
    """Capture ULog paths that already exist before a trial launches."""
    root = Path(ulog_root)
    if not root.is_dir():
        return ()
    return tuple(path.resolve() for path in root.glob("**/*.ulg"))


def locate_trial_ulog(
    ulog_root: Path,
    *,
    launched_at_ns: int,
    paths_existing_at_launch: Iterable[Path] = (),
) -> Optional[Path]:
    """Locate a ULog created after this trial's launch boundary."""
    candidates = locate_trial_ulogs(
        ulog_root,
        launched_at_ns=launched_at_ns,
        paths_existing_at_launch=paths_existing_at_launch,
    )
    if not candidates:
        return None
    return candidates[-1]


def locate_trial_ulogs(
    ulog_root: Path,
    *,
    launched_at_ns: int,
    paths_existing_at_launch: Iterable[Path] = (),
) -> List[Path]:
    """Locate every ULog created after this trial's launch boundary.

    PX4's SITL logger starts a fresh segment on each arming cycle, so a
    single trial can produce several ``.ulg`` files that cover disjoint
    phase windows; all of them are needed as evidence.
    """
    root = Path(ulog_root)
    if not root.is_dir():
        return []
    existing = {Path(path).resolve() for path in paths_existing_at_launch}
    candidates = []
    for path in root.glob("**/*.ulg"):
        resolved = path.resolve()
        if resolved in existing:
            continue
        try:
            if path.stat().st_mtime_ns < int(launched_at_ns):
                continue
        except OSError:
            continue
        candidates.append(path)
    candidates.sort(key=lambda path: path.stat().st_mtime_ns)
    return candidates


def _write_artifact_error(trial_dir: Path, message: str) -> None:
    (Path(trial_dir) / "artifact_error.txt").write_text(
        str(message).rstrip() + "\n",
        encoding="utf-8",
    )


def process_trial_artifacts(
    *,
    trial_dir: Path,
    ulog_source: Optional[Path] = None,
    ulog_sources: Optional[Sequence[Path]] = None,
    config_path: Path,
    log_rules_path: Path,
    validation_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
    reporting_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
) -> ArtifactResult:
    """Copy ULog evidence, then invoke the existing validate/report APIs."""
    trial_dir = Path(trial_dir)
    if ulog_sources is None:
        sources = (
            [Path(ulog_source)]
            if ulog_source is not None
            else []
        )
    else:
        sources = [Path(source) for source in ulog_sources]
    sources = [source for source in sources if source.is_file()]
    if not sources:
        _write_artifact_error(
            trial_dir,
            "no PX4 ULog was created after this trial's launch boundary",
        )
        return ArtifactResult(
            ulog=None,
            validation_exit=2,
            reporting_exit=2,
            report_path=None,
        )

    try:
        copied = []
        for source in sources:
            copied_ulog = trial_dir / source.name
            shutil.copy2(str(source), str(copied_ulog))
            copied.append(copied_ulog)
    except OSError as error:
        _write_artifact_error(trial_dir, "failed to copy PX4 ULog: {}".format(error))
        return ArtifactResult(
            ulog=None,
            validation_exit=2,
            reporting_exit=2,
            report_path=None,
        )

    if validation_entrypoint is None:
        from flying_car_test.validation import main as validation_entrypoint
    if reporting_entrypoint is None:
        from flying_car_test.reporting import main as reporting_entrypoint

    validation_argv = [
        "--result-dir",
        str(trial_dir),
        "--config",
        str(config_path),
        "--log-rules",
        str(log_rules_path),
        "--output",
        str(trial_dir / "validation.json"),
    ]
    reporting_argv = [
        "--result-dir",
        str(trial_dir),
        "--config",
        str(config_path),
        "--log-rules",
        str(log_rules_path),
        "--output-dir",
        str(trial_dir),
    ]
    try:
        validation_exit = int(validation_entrypoint(validation_argv))
    except Exception as error:
        validation_exit = 2
        _write_artifact_error(
            trial_dir, "validation failed to execute: {}".format(error)
        )
    try:
        reporting_exit = int(reporting_entrypoint(reporting_argv))
    except Exception as error:
        reporting_exit = 2
        _write_artifact_error(
            trial_dir, "report generation failed to execute: {}".format(error)
        )
    report_path = trial_dir / "report.md"
    return ArtifactResult(
        ulog=copied[0],
        validation_exit=validation_exit,
        reporting_exit=reporting_exit,
        report_path=report_path if report_path.is_file() else None,
    )


def generate_aggregate_report(
    *,
    trial_dirs: Sequence[Path],
    config_path: Path,
    log_rules_path: Path,
    reporting_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
) -> int:
    """Invoke the existing reporting CLI contract for all trial directories."""
    if not trial_dirs:
        return 2
    output_dir = Path(trial_dirs[0]).parent
    if reporting_entrypoint is None:
        from flying_car_test.reporting import write_aggregate_report
        try:
            payload = write_aggregate_report(
                trial_dirs=trial_dirs,
                config_path=config_path,
                output_md_path=output_dir / "aggregate_report.md",
                output_plot_dir=output_dir / "plots",
                output_summary_path=output_dir / "aggregate_summary.json",
            )
            return 0 if payload.get("overall") == "PASS" else 1
        except Exception as error:
            (output_dir / "aggregate_error.txt").write_text(
                "aggregate report generation failed: {}\n".format(error),
                encoding="utf-8",
            )
            return 2

    argv: List[str] = []
    for trial_dir in trial_dirs:
        argv.extend(["--result-dir", str(trial_dir)])
    argv.extend(
        [
            "--config",
            str(config_path),
            "--log-rules",
            str(log_rules_path),
            "--output-dir",
            str(output_dir),
        ]
    )
    try:
        return int(reporting_entrypoint(argv))
    except Exception as error:
        (output_dir / "aggregate_error.txt").write_text(
            "aggregate report generation failed: {}\n".format(error),
            encoding="utf-8",
        )
        return 2


# Default UDP/TCP ports the orchestrator guards before each trial.
DEFAULT_REQUIRED_PORTS: Tuple[Tuple[str, str, int], ...] = (
    ("udp", "127.0.0.1", 14540),
    ("udp", "127.0.0.1", 14550),
    ("udp", "127.0.0.1", 14580),
    ("tcp", "127.0.0.1", 11345),
    ("tcp", "127.0.0.1", 11311),
)


def _default_launch_runner_factory(
    command: Sequence[str],
    env: Mapping[str, str],
    result_dir: Path,
    stdout_log: Path,
    stderr_log: Path,
    ros_log_dir: Path,
    poll_interval_s: float,
    timeout_s: Optional[float],
) -> LaunchRunner:
    return LaunchRunner(
        command=command,
        env=env,
        result_dir=result_dir,
        stdout_log=stdout_log,
        stderr_log=stderr_log,
        ros_log_dir=ros_log_dir,
        poll_interval_s=poll_interval_s,
        timeout_s=timeout_s,
    )


def _run_single_trial(
    out_root: Path,
    trial_number: int,
    *,
    args: argparse.Namespace,
    env_overrides: Mapping[str, str],
    port_prober: PortProber,
    launch_runner_factory: Callable[..., LaunchRunner],
    validation_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
    reporting_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
    now: Optional[datetime.datetime] = None,
) -> TrialOutcome:
    """Spawn, wait for, and validate exactly one trial run."""
    candidate = check_no_existing(out_root, trial_number, now=now)
    candidate.mkdir(parents=True)
    ros_log_dir = allocate_ros_log_dir(candidate)
    stdout_log = candidate / "roslaunch.log"
    stderr_log = stdout_log
    env = os.environ.copy()
    env.update(
        {
            str(key): str(value)
            for key, value in env_overrides.items()
            if not str(key).startswith("__")
        }
    )
    env.update(unique_ros_log_env(candidate))
    launch_argv: List[str] = list(getattr(args, "launch_argv", []))
    launch_arguments = [
        "result_dir:={}".format(candidate),
        "config:={}".format(getattr(args, "config")),
        "rosbag_name:=trial-{0:02d}".format(int(trial_number)),
        "gui:={}".format(
            "true" if bool(getattr(args, "gui", False)) else "false"
        ),
        "model_name:={}".format(
            getattr(args, "model_name", "FS-FASMv8_octo")
        ),
    ]
    command = ["roslaunch", "--screen", *launch_argv, *launch_arguments]
    timeout_s = float(getattr(args, "timeout_s", 0.0) or 0.0) or None
    runner = launch_runner_factory(
        command=command,
        env=env,
        result_dir=candidate,
        stdout_log=stdout_log,
        stderr_log=stderr_log,
        ros_log_dir=ros_log_dir,
        poll_interval_s=0.5,
        timeout_s=timeout_s,
    )
    ulog_root = Path(
        getattr(args, "ulog_root", None) or default_ulog_root()
    )
    preexisting_ulogs = snapshot_ulog_paths(ulog_root)
    launched_at_ns = time.time_ns()
    runner.start()
    marker = runner.poll_completion_marker(timeout_s=timeout_s or 3600.0)
    returncode = runner.shutdown()
    source_ulogs = locate_trial_ulogs(
        ulog_root,
        launched_at_ns=launched_at_ns,
        paths_existing_at_launch=preexisting_ulogs,
    )
    artifacts = process_trial_artifacts(
        trial_dir=candidate,
        ulog_sources=source_ulogs,
        config_path=Path(
            getattr(args, "config", None) or default_config_path()
        ),
        log_rules_path=Path(
            getattr(args, "log_rules", None) or default_log_rules_path()
        ),
        validation_entrypoint=validation_entrypoint,
        reporting_entrypoint=reporting_entrypoint,
    )
    validation_exit = (
        artifacts.validation_exit
        if artifacts.validation_exit != 0
        else artifacts.reporting_exit
    )
    if (marker is None or returncode != 0) and validation_exit == 0:
        # A deliberate timeout kill returns non-zero and the completion
        # marker is only written while tearing down; that is normal for a
        # run that already reached COMPLETE.  Only distrust a PASS when the
        # monitor never finalized cleanly.
        monitor_path = Path(candidate) / "monitor_complete.json"
        try:
            monitor_clean = bool(
                json.loads(
                    monitor_path.read_text(encoding="utf-8")
                ).get("clean")
            )
        except (OSError, ValueError):
            monitor_clean = False
        if not monitor_clean:
            validation_exit = 2
    return TrialOutcome(
        trial_dir=candidate,
        ulog=artifacts.ulog,
        returncode=returncode,
        report=str(artifacts.report_path) if artifacts.report_path else "",
        validation_exit=validation_exit,
    )


def run_trials(
    out_root: Path,
    *,
    trial_count: int,
    args: argparse.Namespace,
    env_overrides: Optional[Mapping[str, str]] = None,
    port_prober: Optional[PortProber] = None,
    disk_free_bytes: Optional[int] = None,
    launch_runner_factory: Optional[Callable[..., LaunchRunner]] = None,
    required_ports: Iterable[Tuple[str, str, int]] = DEFAULT_REQUIRED_PORTS,
    required_commands: Iterable[str] = ("roslaunch",),
    minimum_free_bytes: int = 10 ** 10,
    validation_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
    reporting_entrypoint: Optional[Callable[[Sequence[str]], int]] = None,
    now: Optional[datetime.datetime] = None,
) -> List[TrialOutcome]:
    """Drive ``trial_count`` independent trials and return their outcomes."""
    run_timestamp = now if now is not None else _utc_now()
    env_overrides = dict(env_overrides or {})
    port_prober = port_prober or default_port_prober()
    launch_runner_factory = (
        launch_runner_factory or _default_launch_runner_factory
    )

    preflight(
        out_root=out_root,
        port_prober=port_prober,
        required_ports=required_ports,
        disk_free_bytes=disk_free_bytes,
        minimum_free_bytes=minimum_free_bytes,
        required_commands=required_commands,
        required_paths=tuple(env_overrides.get("__required_paths__", []) or ()),
    )

    outcomes: List[TrialOutcome] = []
    for index in range(1, int(trial_count) + 1):
        outcomes.append(
            _run_single_trial(
                out_root,
                index,
                args=args,
                env_overrides=env_overrides,
                port_prober=port_prober,
                launch_runner_factory=launch_runner_factory,
                validation_entrypoint=validation_entrypoint,
                reporting_entrypoint=reporting_entrypoint,
                now=run_timestamp,
            )
        )
        wait_for_ports_free(
            port_prober,
            required_ports,
            timeout_s=float(
                getattr(args, "port_release_timeout_s", 30.0)
            ),
        )
    if outcomes:
        aggregate_exit = generate_aggregate_report(
            trial_dirs=[outcome.trial_dir for outcome in outcomes],
            config_path=Path(
                getattr(args, "config", None) or default_config_path()
            ),
            log_rules_path=Path(
                getattr(args, "log_rules", None) or default_log_rules_path()
            ),
            reporting_entrypoint=reporting_entrypoint,
        )
        if aggregate_exit != 0 and all(
            outcome.validation_exit == 0 for outcome in outcomes
        ):
            last = outcomes[-1]
            outcomes[-1] = TrialOutcome(
                trial_dir=last.trial_dir,
                ulog=last.ulog,
                returncode=last.returncode,
                report=last.report,
                validation_exit=aggregate_exit,
            )
    return outcomes


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_trials",
        description="Isolated flying-car experiment trial orchestrator",
    )
    parser.add_argument(
        "--output-root",
        "--out-root",
        dest="output_root",
        required=True,
        type=Path,
        help="Directory under which trial-NN subdirectories will be created",
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=1,
        help="Number of trials to run sequentially",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Run preflight checks and exit without spawning anything",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="Experiment YAML config",
    )
    parser.add_argument(
        "--log-rules",
        type=Path,
        default=default_log_rules_path(),
        help="Validation log-rules YAML",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Enable the Gazebo GUI for each trial",
    )
    parser.add_argument(
        "--model-name",
        default="fs_fasmv8_flying_car",
        help="Gazebo model name passed to experiment nodes",
    )
    parser.add_argument(
        "--ulog-root",
        type=Path,
        default=default_ulog_root(),
        help="PX4 SITL ULog source root",
    )
    parser.add_argument(
        "--port-release-timeout-s",
        type=float,
        default=30.0,
        help="Maximum cleanup wait for guarded ports",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=None,
        help="Per-trial timeout in seconds (0 = no timeout)",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    try:
        preflight(
            out_root=args.output_root,
            port_prober=default_port_prober(),
            required_ports=DEFAULT_REQUIRED_PORTS,
            required_commands=("roslaunch", "rospack"),
            required_paths=(args.config, args.log_rules),
            required_packages=("px4", "flying_car_test"),
            required_package_files=(
                ("px4", Path("launch") / "FS_flying_car.launch"),
                (
                    "flying_car_test",
                    Path("launch") / "mavros_sitl_no_timesync.launch",
                ),
                (
                    "flying_car_test",
                    Path("launch") / "full_experiment.launch",
                ),
            ),
            package_resolver=resolve_ros_package,
        )
    except TrialRunnerError as error:
        sys.stderr.write("preflight failed: {}\n".format(error))
        return 2

    if getattr(args, "preflight_only", False):
        sys.stdout.write(
            "preflight OK for out_root={} trials={}\n".format(
                args.output_root, args.trials
            )
        )
        return 0

    args.launch_argv = ["flying_car_test", "full_experiment.launch"]

    outcomes = run_trials(
        out_root=args.output_root,
        trial_count=int(args.trials),
        args=args,
        env_overrides={},
        port_prober=default_port_prober(),
    )
    failing = [outcome for outcome in outcomes if outcome.validation_exit != 0]
    sys.stdout.write(
        "completed {0} trial(s); {1} validation failure(s)\n".format(
            len(outcomes), len(failing)
        )
    )
    return 0 if not failing else 1


if __name__ == "__main__":
    raise SystemExit(main())
