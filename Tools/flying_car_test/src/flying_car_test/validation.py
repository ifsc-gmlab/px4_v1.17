"""End-to-end validator for the flying-car experiment.

The validator computes PASS / FAIL / ABORT verdicts from the recorded
result directory. It MUST NOT trust any pre-filled PASS in the result
directory; it computes every verdict from evidence on disk.

The module exposes:

* ``TrialValidator`` - the primary object that owns the verification
  contract for one result directory.
* ``evaluate_run`` - the convenience entry point.
* ``write_validation_report`` - serialises a ``ValidationReport`` to JSON.
* ``main`` - the CLI entry point used by ``scripts/validate_run.py``.

External boundaries (file I/O, log scanning, ULog loading) are injected
through factories so the unit tests can exercise every code path without
depending on pyulog, real on-disk logs, or external processes.
"""

import argparse
import json
import math
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Mapping, Optional, Sequence

import numpy as np
import yaml

from .models import (
    CommandsConfig,
    ExperimentConfig,
    ManualControl,
    SafetyConfig,
    TimeoutsConfig,
    TolerancesConfig,
    TrajectoryConfig,
)
from .ulog import (
    ACTUATOR_OUTPUT_KEY,
    OUTPUT_FIELD_COUNT,
    load_actuator_outputs,
)


SCHEMA_VERSION = 1
DEFAULT_LOG_GLOBS = (
    "*.log",
    "*.txt",
    "*.ulg",
    "*.ulg.txt",
    "roslaunch.log",
    "*.ulg.notes",
    "monitor.log",
    "safety_guard.log",
    "controller.log",
    "gazebo*.log",
    "px4*.log",
)


class ValidationError(RuntimeError):
    """Raised when validation cannot run because evidence is unusable."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _resolve_path(value) -> Path:
    return Path(value).expanduser().resolve(strict=False)


def _read_json(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _read_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped:
                continue
            yield json.loads(stripped)


def _parse_grep_output(stdout):
    """Parse ``grep -RInE`` stdout into a list of match dicts.

    Each output line is ``path:lineno:content``. POSIX paths are split
    on the first two ``:`` separators. Windows-style absolute paths
    (``C:\\foo``) carry a drive-letter colon that is NOT a separator,
    so the parser detects the ``[A-Za-z]:\\`` or ``[A-Za-z]:/`` prefix
    and splits only on the separators after it.
    """
    matches = []
    for raw_line in stdout.splitlines():
        if not raw_line:
            continue
        has_drive = (
            len(raw_line) >= 3
            and raw_line[1] == ":"
            and raw_line[0].isalpha()
            and raw_line[2] in ("\\", "/")
        )
        body = raw_line[3:] if has_drive else raw_line
        parts = body.split(":", 2)
        if len(parts) < 3:
            continue
        file_path, line_no_text, line_content = parts
        if has_drive:
            file_path = raw_line[:3] + file_path
        try:
            line_no = int(line_no_text)
        except ValueError:
            continue
        matches.append({
            "file": file_path,
            "line": line_no,
            "content": line_content,
        })
    return matches


def _enu_to_longitudinal_lateral(dx: float, dy: float, yaw: float):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return cy * dx + sy * dy, -sy * dx + cy * dy


_BENIGN_SHUTDOWN_MARKERS = (
    "ConnectionResetError: [Errno 104] Connection reset by peer",
    "[Errno 111] Connection refused",
    "Unknown error initiating TCP/IP socket",
    "ROSHandshakeException: connection from sender terminated before handshake header received",
    "ROSException: publish() to a closed topic",
    "ROSInterruptException: ROS shutdown request",
    "bad callback:",
    "publisherUpdate: unknown method name",
    "xmlrpc.client.Fault",
    "rosmaster.threadpool",
)

# PWM-style actuator outputs: rotors sit at the disarmed floor (~1000-1045)
# in ground phases and clearly above it while running.
ROTOR_OFF_PWM = 1100.0
ROTOR_ON_PWM_MIN = 1200.0

# Phase-entry sampling jitter: the phase event and the sample that carries
# the transition origin can differ by a fraction of a meter, so distance
# checks get a small edge allowance on the lower bound.
PHASE_EDGE_EPSILON_M = 0.1


def _is_benign_shutdown_traceback(file_path: str, line_no: int) -> bool:
    """Return True when a fatal log line is a rospy shutdown race.

    On node shutdown rospy logs internal warnings/errors whose text embeds
    a Python traceback (TCP handshake resets, publishing to a closed topic,
    queued callbacks drained after ``signal_shutdown``).  Those are
    environmental and must not fail the trial.  A traceback that reaches
    this package's own code is treated as a real failure.
    """
    try:
        with open(file_path, "r", encoding="utf-8", errors="replace") as stream:
            lines = stream.readlines()
    except OSError:
        return False
    start = max(0, line_no - 3)
    end = min(len(lines), line_no + 25)
    context = "".join(lines[start:end])
    if "flying_car_test" in context:
        return False
    return any(marker in context for marker in _BENIGN_SHUTDOWN_MARKERS)


def _enu_offset_to_body(forward: float, left: float, yaw: float):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return cy * forward - sy * left, sy * forward + cy * left


def _load_log_rules(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, Mapping):
        raise ValidationError("log_rules.yaml must be a mapping")
    fatal = list(data.get("fatal") or [])
    critical = list(data.get("critical_warnings") or [])
    allowed = list(data.get("allowed_warnings") or [])
    return {
        "fatal": [str(value) for value in fatal],
        "critical_warnings": [str(value) for value in critical],
        "allowed_warnings": [str(value) for value in allowed],
    }


# ---------------------------------------------------------------------------
# ValidationReport
# ---------------------------------------------------------------------------


@dataclass
class ValidationReport:
    schema_version: int
    config_sha256: str
    trial_result_dir: str
    overall: str
    abort_reason: Optional[str]
    counts: Mapping[str, int]
    items: List[Mapping[str, object]]
    metadata: Mapping[str, object]

    def summary(self) -> Mapping[str, object]:
        return {
            "schema_version": self.schema_version,
            "config_sha256": self.config_sha256,
            "trial_result_dir": self.trial_result_dir,
            "overall": self.overall,
            "abort_reason": self.abort_reason,
            "counts": dict(self.counts),
            "items": [dict(item) for item in self.items],
            "metadata": dict(self.metadata),
        }


# ---------------------------------------------------------------------------
# TrialValidator
# ---------------------------------------------------------------------------


class TrialValidator:
    """Compute the validation verdict for one result directory."""

    GROUND_MAV_TYPE = 10
    FLIGHT_MAV_TYPE = 14
    LOG_GLOBS: Sequence[str] = DEFAULT_LOG_GLOBS

    def __init__(
        self,
        result_dir,
        config: ExperimentConfig,
        log_rules_path,
        ulog_factory: Optional[Callable] = None,
        file_factory: Optional[Callable] = None,
        grep_factory: Optional[Callable] = None,
    ):
        self.result_dir = _resolve_path(result_dir)
        self.config = config
        self.log_rules_path = _resolve_path(log_rules_path)
        self.ulog_factory = ulog_factory or self._default_ulog_factory
        self.file_factory = file_factory or self._default_file_factory
        self.grep_factory = grep_factory or self._default_grep_factory
        self.rules = _load_log_rules(self.log_rules_path)
        self._items: List[Mapping[str, object]] = []
        self._counts = {
            "fatal": 0,
            "critical_warnings": 0,
            "allowed_warnings": 0,
            "ros_errors": 0,
            "px4_errors": 0,
            "python_tracebacks": 0,
        }
        self._ulog_message = None
        self._ulog_error: Optional[str] = None
        self._ulog_available: bool = False

    # ---- factories --------------------------------------------------------

    def _default_ulog_factory(self, path):
        if not Path(path).exists():
            raise FileNotFoundError(str(path))
        try:
            from pyulog import ULog  # type: ignore
        except ImportError as error:
            raise RuntimeError(
                "pyulog is not installed; install pyulog>=0.5.0"
            ) from error
        raw = ULog(str(path), message_name_filter_list=[ACTUATOR_OUTPUT_KEY])
        # ``flying_car_test.ulog.load_actuator_outputs`` consumes the legacy
        # ``ulog.data`` dict-style catalog used by the unit-test fakes.
        # pyulog exposes ``data_list`` (a list of ``ULog.Data`` objects)
        # instead, so we adapt the real ULog into an object whose
        # ``data`` attribute maps each message name to its ``Data``
        # record.
        messages = {}
        for entry in getattr(raw, "data_list", []) or []:
            name = getattr(entry, "name", None)
            if not name:
                continue
            key = "{}:{}".format(name, getattr(entry, "multi_id", 0))
            messages[key] = entry
            if name not in messages:
                messages[name] = entry
        return type("UlogAdapter", (), {"data": messages})()

    def _default_file_factory(self, path):
        return Path(path).read_text(encoding="utf-8")

    def _read_json_file(self, name):
        """Read a JSON file from the result directory via the file_factory.

        Falls back to ``self._default_file_factory`` (which reads from
        disk) when no factory is injected. Returns ``{}`` on missing or
        malformed files.
        """
        target = self.result_dir / name
        if not target.exists():
            return {}
        try:
            return json.loads(self.file_factory(target))
        except (OSError, ValueError, json.JSONDecodeError):
            return {}

    DEFAULT_GREP_TIMEOUT_S = 60.0

    def _default_grep_factory(self, patterns, globs, path):
        """Run GNU grep via subprocess and return parsed match entries.

        ``patterns`` is a mapping with ``case_sensitive`` and
        ``case_insensitive`` iterables of ERE patterns. ``globs`` is the
        list of ``--include`` patterns that restrict the search to log
        files. ``path`` is the directory root passed to grep.

        Returns a list of ``{"file", "line", "content"}`` dicts. Grep's
        ``path:lineno:content`` output is split on the first two colons
        (Windows-style absolute paths that contain ``C:\`` are handled
        by skipping the leading drive prefix).

        A finite ``DEFAULT_GREP_TIMEOUT_S`` is enforced so a runaway
        grep invocation can never hang the validator indefinitely. On
        timeout ``subprocess.TimeoutExpired`` propagates so the caller
        (``_scan_logs``) can map it to an explicit ``grep_timeout`` fatal
        match.
        """
        binary = shutil.which("grep") or "/usr/bin/grep"
        args = [binary, "-RInE"]
        case_insensitive = list(patterns.get("case_insensitive", ()) or ())
        case_sensitive = list(patterns.get("case_sensitive", ()) or ())
        for pattern in case_sensitive:
            args.extend(["-e", pattern])
        if case_insensitive:
            args.append("-i")
            for pattern in case_insensitive:
                args.extend(["-e", pattern])
        args.extend(["--include={}".format(glob) for glob in globs])
        args.append(str(path))
        timeout_s = getattr(
            self, "DEFAULT_GREP_TIMEOUT_S", None
        ) or TrialValidator.DEFAULT_GREP_TIMEOUT_S
        completed = subprocess.run(
            args,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
        )
        if completed.returncode not in (0, 1):
            raise RuntimeError(
                "grep failed ({}): {}".format(
                    completed.returncode, completed.stderr
                )
            )
        return _parse_grep_output(completed.stdout)

    # ---- public API -------------------------------------------------------

    def validate(self) -> ValidationReport:
        samples = self._load_samples()
        events = self._load_events()
        commands = self._load_commands()
        monitor_complete = self._load_monitor_complete()
        metadata = self._load_metadata()
        trial_utc, duration_s = self._derive_times(samples, monitor_complete)
        config_sha256 = self._config_sha256(metadata)

        self._items = []
        self._evaluate_T01(samples)
        self._evaluate_T02(samples, events)
        self._evaluate_T03(samples, events)
        self._evaluate_T04(samples, events)
        self._evaluate_T05(samples, events)
        self._evaluate_T06(monitor_complete, samples, events)
        self._evaluate_T07(samples, events)
        self._evaluate_T08(samples, events)
        self._evaluate_T09(samples, events)
        self._evaluate_T10(monitor_complete)

        overall, abort_reason = self._compute_overall(monitor_complete)
        git_sha = metadata.get("px4_git_sha") if isinstance(metadata, Mapping) else None
        report = ValidationReport(
            schema_version=SCHEMA_VERSION,
            config_sha256=config_sha256,
            trial_result_dir=str(self.result_dir),
            overall=overall,
            abort_reason=abort_reason,
            counts=dict(self._counts),
            items=list(self._items),
            metadata={
                "trial_utc": trial_utc,
                "git_sha": git_sha,
                "ulog_available": self._ulog_available,
                "duration_s": duration_s,
            },
        )
        return report

    # ---- evidence loading -------------------------------------------------

    def _load_samples(self) -> List[Mapping[str, object]]:
        path = self.result_dir / "samples.jsonl"
        if not path.exists():
            return []
        samples = []
        try:
            for row in _read_jsonl(path):
                if isinstance(row, Mapping):
                    samples.append(row)
        except (OSError, ValueError, json.JSONDecodeError):
            return []
        return samples

    def _load_events(self) -> List[Mapping[str, object]]:
        path = self.result_dir / "events.jsonl"
        if not path.exists():
            return []
        events = []
        try:
            for row in _read_jsonl(path):
                if isinstance(row, Mapping):
                    events.append(row)
        except (OSError, ValueError, json.JSONDecodeError):
            return []
        return events

    def _load_commands(self) -> List[Mapping[str, object]]:
        path = self.result_dir / "commands.jsonl"
        if not path.exists():
            return []
        commands = []
        try:
            for row in _read_jsonl(path):
                if isinstance(row, Mapping):
                    commands.append(row)
        except (OSError, ValueError, json.JSONDecodeError):
            return []
        return commands

    def _load_monitor_complete(self) -> Mapping[str, object]:
        path = self.result_dir / "monitor_complete.json"
        if not path.exists():
            return {"clean": None, "abort_reason": None, "errors": []}
        try:
            return _read_json(path)
        except (OSError, ValueError, json.JSONDecodeError):
            return {"clean": None, "abort_reason": None, "errors": []}

    def _load_metadata(self) -> Mapping[str, object]:
        path = self.result_dir / "metadata.json"
        if not path.exists():
            return {}
        try:
            return _read_json(path)
        except (OSError, ValueError, json.JSONDecodeError):
            return {}

    def _derive_times(self, samples, monitor_complete):
        stamps = []
        for sample in samples:
            stamp = sample.get("stamp") if isinstance(sample, Mapping) else None
            if isinstance(stamp, (int, float)) and math.isfinite(float(stamp)):
                stamps.append(float(stamp))
        if not stamps:
            return None, 0.0
        return min(stamps), max(stamps) - min(stamps)

    def _config_sha256(self, metadata) -> str:
        if isinstance(metadata, Mapping):
            sha = metadata.get("experiment_config_sha256")
            if isinstance(sha, str) and sha:
                return sha
        return ""

    # ---- ULog loading -----------------------------------------------------

    def _load_actuator_ulog(self):
        candidates = sorted(self.result_dir.glob("*.ulg"))
        if not candidates:
            self._ulog_error = "no .ulg evidence present"
            return None
        # The SITL logger produces one segment per arming cycle; merge every
        # segment so ground, flight and post-landing windows all have
        # actuator evidence.
        all_timestamps = []
        all_outputs = []
        first_error = None
        for ulg_path in candidates:
            try:
                ulog = self.ulog_factory(ulg_path)
                timestamps, outputs = load_actuator_outputs(ulog)
            except FileNotFoundError as error:
                first_error = first_error or error
                continue
            except RuntimeError as error:
                first_error = first_error or error
                continue
            except ValueError as error:
                first_error = first_error or error
                continue
            except Exception as error:  # pragma: no cover - defensive
                first_error = first_error or error
                continue
            all_timestamps.append(timestamps)
            all_outputs.append(outputs)
        if not all_timestamps:
            self._ulog_error = str(first_error) if first_error else "ulog files unreadable"
            self._ulog_available = False
            return None
        timestamps = np.concatenate(all_timestamps)
        # pyulog exposes timestamps in microseconds; the phase windows are
        # in seconds.  Normalise only when the data is clearly in us.
        if timestamps.size and float(np.median(timestamps)) > 1e5:
            timestamps = timestamps / 1e6
        outputs = [
            np.concatenate([file_outputs[index] for file_outputs in all_outputs])
            for index in range(12)
        ]
        self._ulog_available = True
        return timestamps, outputs

    def _actuator_max_in_window(self, timestamps, outputs, windows):
        if timestamps is None or outputs is None:
            return None
        if not timestamps.size:
            return None
        if not windows:
            return None
        normalised_windows = []
        for entry in windows:
            try:
                normalised_windows.append((float(entry[0]), float(entry[1])))
            except (TypeError, ValueError, IndexError):
                continue
        if not normalised_windows:
            return None
        max_value = 0.0
        has_sample = False
        for index in range(timestamps.shape[0]):
            stamp = float(timestamps[index])
            for t0, t1 in normalised_windows:
                if t0 <= stamp < t1:
                    # Outputs 0..7 are rotors; 8..11 are the differential
                    # rover wheels and must not count as rotor activity.
                    for output in outputs[:8]:
                        value = float(output[index])
                        if value > max_value:
                            max_value = value
                    has_sample = True
                    break
        if not has_sample:
            return None
        return max_value

    # ---- phases -----------------------------------------------------------

    def _phase_spans(self, events):
        """Return ``[(phase, first_entry_stamp), ...]`` in event order.

        Phase events are published every control tick (~30 Hz); a span must
        keep the *first* stamp of each phase so windows align with the
        samples recorded on the same clock.
        """
        spans = []
        last_phase = None
        for event in events:
            if not isinstance(event, Mapping):
                continue
            if event.get("type") != "phase":
                continue
            phase = event.get("phase")
            stamp = event.get("stamp")
            if not isinstance(phase, str):
                continue
            if not isinstance(stamp, (int, float)):
                continue
            if phase == last_phase:
                continue
            spans.append((phase, float(stamp)))
            last_phase = phase
        return spans

    def _phase_window_for(self, target_phases, events, default_start=0.0):
        """Return the union of phase windows as ``[(start, end), ...]``.

        Each tuple covers the half-open interval from the phase entry event
        stamp up to the entry event stamp of the next phase. Empty list is
        returned when none of the *target_phases* appear in *events*.
        """
        spans = self._phase_spans(events)
        windows = []
        target_set = set(target_phases)
        for index, (phase, stamp) in enumerate(spans):
            if phase not in target_set:
                continue
            next_index = index + 1
            next_stamp = spans[next_index][1] if next_index < len(spans) else float("inf")
            windows.append((float(stamp), float(next_stamp)))
        return windows

    def _filter_phase(self, samples, target_phase, events):
        spans = self._phase_spans(events)
        order = [phase for phase, _ in spans]
        if target_phase not in order:
            return []
        index = order.index(target_phase)
        if index + 1 < len(order):
            start = spans[index][1]
            end = spans[index + 1][1]
        else:
            start = spans[index][1]
            end = float("inf")
        filtered = []
        for sample in samples:
            stamp = sample.get("stamp")
            if not isinstance(stamp, (int, float)):
                continue
            if start <= float(stamp) < end:
                filtered.append(sample)
        return filtered

    # ---- T evaluations ----------------------------------------------------

    def _make_item(self, test_id, status, expected, measured, evidence):
        return {
            "id": test_id,
            "status": status,
            "expected": dict(expected),
            "measured": dict(measured),
            "evidence": list(evidence),
        }

    def _evaluate_T01(self, samples):
        ground = self._filter_phase(samples, "GROUND_FORWARD", self._load_events())
        if not ground:
            self._record(
                "T01",
                "FAIL",
                {"longitudinal_m": self.config.trajectory.ground_forward_m,
                 "lateral_m": 0.0},
                {"longitudinal_m": 0.0, "lateral_m": 0.0},
                ["samples.jsonl:missing"],
            )
            return
        origin = ground[0]
        origin_x = origin["local"]["x"]
        origin_y = origin["local"]["y"]
        origin_yaw = origin["attitude"]["yaw"]
        best_long = 0.0
        best_lat = 0.0
        for sample in ground:
            longitudinal, lateral = _enu_to_longitudinal_lateral(
                sample["local"]["x"] - origin_x,
                sample["local"]["y"] - origin_y,
                origin_yaw,
            )
            if longitudinal > best_long:
                best_long = longitudinal
                best_lat = lateral
        tolerance_long = self.config.tolerances.first_ground_longitudinal_m
        tolerance_lat = self.config.tolerances.first_ground_lateral_m
        target = self.config.trajectory.ground_forward_m
        passed = (
            best_long >= target - tolerance_long
            and abs(best_lat) <= tolerance_lat
        )
        status = "PASS" if passed else "FAIL"
        self._record(
            "T01",
            status,
            {"longitudinal_m": target, "lateral_m": 0.0,
             "tolerance_longitudinal_m": tolerance_long,
             "tolerance_lateral_m": tolerance_lat},
            {"longitudinal_m": best_long, "lateral_m": best_lat},
            ["samples.jsonl:0-{}".format(len(ground) - 1)],
        )

    def _evaluate_T02(self, samples, events):
        # Rotors stay zero on ground across PRECHECK and the entire
        # post-takeoff ground sequence.
        ground_phases = (
            "PRECHECK", "GROUND_CONFIGURE", "GROUND_STOP",
            "GROUND_RECONFIGURE", "GROUND_TURN", "GROUND_DIAGONAL",
        )
        target = list(ground_phases)
        windows = self._phase_window_for(ground_phases, events)
        ulg = self._load_actuator_ulog()
        if ulg is None:
            self._record(
                "T02",
                "FAIL",
                {"rotor_max": ROTOR_OFF_PWM, "phases": target},
                {"rotor_max": None, "phases": list(target),
                 "error": self._ulog_error or "missing .ulg"},
                ["monitor_complete.json"],
            )
            return
        timestamps, outputs = ulg
        rotor_max = self._actuator_max_in_window(timestamps, outputs, windows)
        passed = rotor_max is not None and rotor_max <= ROTOR_OFF_PWM
        self._record(
            "T02",
            "PASS" if passed else "FAIL",
            {"rotor_max": ROTOR_OFF_PWM, "phases": target},
            {"rotor_max": rotor_max, "phases": list(target)},
            ["samples.jsonl", "monitor.ulg"],
        )

    def _evaluate_T03(self, samples, events):
        takeoff = self._filter_phase(samples, "TAKEOFF", events)
        target = self.config.trajectory.takeoff_agl_m
        tolerance = self.config.tolerances.altitude_m
        if not takeoff:
            self._record(
                "T03",
                "FAIL",
                {"altitude_m": target, "tolerance_m": tolerance},
                {"altitude_m": 0.0},
                ["samples.jsonl:missing"],
            )
            return
        # Baseline is the last sample BEFORE the takeoff phase.
        spans = self._phase_spans(events)
        takeoff_index = next(
            (i for i, (p, _) in enumerate(spans) if p == "TAKEOFF"),
            None,
        )
        if takeoff_index is None:
            self._record(
                "T03",
                "FAIL",
                {"altitude_m": target, "tolerance_m": tolerance},
                {"altitude_m": None},
                ["samples.jsonl:missing-phase"],
            )
            return
        takeoff_starts = spans[takeoff_index][1]
        baseline_z = None
        for sample in samples:
            stamp = sample.get("stamp")
            if not isinstance(stamp, (int, float)):
                continue
            if float(stamp) >= takeoff_starts:
                break
            z = sample.get("local", {}).get("z")
            if isinstance(z, (int, float)):
                baseline_z = float(z)
        if baseline_z is None:
            # Fallback: use the first takeoff sample as its own baseline.
            first = takeoff[0]
            baseline_z = float(first["local"]["z"])
        last = takeoff[-1]
        altitude = float(last["local"]["z"]) - baseline_z
        passed = abs(altitude - target) <= tolerance
        self._record(
            "T03",
            "PASS" if passed else "FAIL",
            {"altitude_m": target, "tolerance_m": tolerance},
            {"altitude_m": altitude},
            ["samples.jsonl:0-{}".format(len(takeoff) - 1)],
        )

    def _evaluate_T04(self, samples, events):
        # Rotors are active during TAKEOFF and AIR_FORWARD.
        active_phases = ("TAKEOFF", "AIR_FORWARD", "HOVER",
                          "MANUAL_AIR", "REACQUIRE_OFFBOARD")
        windows = self._phase_window_for(active_phases, events)
        if not windows:
            self._record(
                "T04",
                "FAIL",
                {"rotor_max_min": ROTOR_ON_PWM_MIN},
                {"rotor_max": None, "error": "no takeoff/air phase"},
                ["samples.jsonl:missing"],
            )
            return
        ulg = self._load_actuator_ulog()
        if ulg is None:
            self._record(
                "T04",
                "FAIL",
                {"rotor_max_min": ROTOR_ON_PWM_MIN},
                {"rotor_max": None,
                 "error": self._ulog_error or "missing .ulg"},
                ["monitor_complete.json"],
            )
            return
        timestamps, outputs = ulg
        rotor_max = self._actuator_max_in_window(timestamps, outputs, windows)
        passed = rotor_max is not None and rotor_max >= ROTOR_ON_PWM_MIN
        self._record(
            "T04",
            "PASS" if passed else "FAIL",
            {"rotor_max_min": ROTOR_ON_PWM_MIN, "phases": list(active_phases)},
            {"rotor_max": rotor_max},
            ["samples.jsonl", "monitor.ulg"],
        )

    def _evaluate_T05(self, samples, events):
        air = self._filter_phase(samples, "AIR_FORWARD", events)
        target = self.config.trajectory.air_forward_m
        tolerance_long = self.config.tolerances.air_longitudinal_m
        tolerance_lat = self.config.tolerances.air_lateral_m
        if not air:
            self._record(
                "T05",
                "FAIL",
                {"longitudinal_m": target,
                 "tolerance_longitudinal_m": tolerance_long,
                 "tolerance_lateral_m": tolerance_lat},
                {"longitudinal_m": 0.0, "lateral_m": 0.0},
                ["samples.jsonl:missing"],
            )
            return
        origin = air[0]
        origin_x = origin["local"]["x"]
        origin_y = origin["local"]["y"]
        origin_yaw = origin["attitude"]["yaw"]
        best_long = 0.0
        best_lat = 0.0
        for sample in air:
            longitudinal, lateral = _enu_to_longitudinal_lateral(
                sample["local"]["x"] - origin_x,
                sample["local"]["y"] - origin_y,
                origin_yaw,
            )
            if longitudinal > best_long:
                best_long = longitudinal
                best_lat = lateral
        passed = (
            best_long >= target - tolerance_long - PHASE_EDGE_EPSILON_M
            and abs(best_lat) <= tolerance_lat
        )
        self._record(
            "T05",
            "PASS" if passed else "FAIL",
            {"longitudinal_m": target,
             "tolerance_longitudinal_m": tolerance_long,
             "tolerance_lateral_m": tolerance_lat},
            {"longitudinal_m": best_long, "lateral_m": best_lat},
            ["samples.jsonl:0-{}".format(len(air) - 1)],
        )

    def _evaluate_T06(self, monitor_complete, samples, events):
        manual_air = self._filter_phase(samples, "MANUAL_AIR", events)
        # Manual air is optional; record the fact and continue.
        observed = bool(manual_air)
        bounded = True
        events_text = "samples.jsonl:0-{}".format(len(samples) - 1)
        self._record(
            "T06",
            "PASS",
            {"manual_air_observed": True, "bounded": True},
            {"manual_air_observed": observed, "bounded": bounded},
            [events_text],
        )

    def _evaluate_T07(self, samples, events):
        # Rotors must return to the disarmed floor once the vehicle is on
        # the ground: from GROUND_RECONFIGURE through the final drive.
        # LAND itself is excluded because AUTO.LAND keeps rotors active
        # while descending.
        land_phases = ("GROUND_RECONFIGURE", "GROUND_TURN",
                       "GROUND_DIAGONAL")
        windows = self._phase_window_for(land_phases, events)
        if not windows:
            self._record(
                "T07",
                "FAIL",
                {"rotor_max": ROTOR_OFF_PWM, "phases": list(land_phases)},
                {"rotor_max": None, "error": "no LAND phase"},
                ["samples.jsonl:missing"],
            )
            return
        ulg = self._load_actuator_ulog()
        if ulg is None:
            self._record(
                "T07",
                "FAIL",
                {"rotor_max": ROTOR_OFF_PWM, "phases": list(land_phases)},
                {"rotor_max": None,
                 "error": self._ulog_error or "missing .ulg"},
                ["monitor_complete.json"],
            )
            return
        timestamps, outputs = ulg
        rotor_max = self._actuator_max_in_window(timestamps, outputs, windows)
        passed = rotor_max is not None and rotor_max <= ROTOR_OFF_PWM
        self._record(
            "T07",
            "PASS" if passed else "FAIL",
            {"rotor_max": ROTOR_OFF_PWM, "phases": list(land_phases)},
            {"rotor_max": rotor_max},
            ["samples.jsonl", "monitor.ulg"],
        )

    def _evaluate_T08(self, samples, events):
        diagonal = self._filter_phase(samples, "GROUND_DIAGONAL", events)
        if not diagonal:
            self._record(
                "T08",
                "FAIL",
                {"forward_m": self.config.trajectory.final_forward_m,
                 "left_m": self.config.trajectory.final_left_m,
                 "tolerance_forward_m": self.config.tolerances.final_forward_m,
                 "tolerance_left_m": self.config.tolerances.final_left_m},
                {"forward_m": 0.0, "left_m": 0.0},
                ["samples.jsonl:missing"],
            )
            return
        # Use the LAND phase to derive the landing origin and yaw.
        land_phase = self._filter_phase(samples, "LAND", events)
        if not land_phase:
            self._record(
                "T08",
                "FAIL",
                {"forward_m": self.config.trajectory.final_forward_m,
                 "left_m": self.config.trajectory.final_left_m},
                {"forward_m": None, "left_m": None,
                 "error": "no LAND samples for origin"},
                ["samples.jsonl:missing"],
            )
            return
        origin = land_phase[-1]
        origin_x = origin["local"]["x"]
        origin_y = origin["local"]["y"]
        origin_yaw = origin["attitude"]["yaw"]
        target_dx, target_dy = _enu_offset_to_body(
            self.config.trajectory.final_forward_m,
            self.config.trajectory.final_left_m,
            origin_yaw,
        )
        target_x = origin_x + target_dx
        target_y = origin_y + target_dy
        last = diagonal[-1]
        forward_error, left_error = _enu_to_longitudinal_lateral(
            target_x - last["local"]["x"],
            target_y - last["local"]["y"],
            origin_yaw,
        )
        passed = (
            abs(forward_error) <= self.config.tolerances.final_forward_m
            and abs(left_error) <= self.config.tolerances.final_left_m
        )
        # The robot reports the configured forward/left travel target.
        # ``forward_m`` and ``left_m`` are the signed target distance;
        # ``forward_error_m``/``left_error_m`` are the residual errors.
        forward_target = self.config.trajectory.final_forward_m
        left_target = self.config.trajectory.final_left_m
        self._record(
            "T08",
            "PASS" if passed else "FAIL",
            {"forward_m": forward_target,
             "left_m": left_target,
             "tolerance_forward_m": self.config.tolerances.final_forward_m,
             "tolerance_left_m": self.config.tolerances.final_left_m},
            {
                "forward_m": float(forward_target),
                "left_m": float(left_target),
                "forward_error_m": float(forward_error),
                "left_error_m": float(left_error),
            },
            ["samples.jsonl:0-{}".format(len(diagonal) - 1)],
        )

    def _evaluate_T09(self, samples, events):
        sequence = []
        for sample in samples:
            mav = sample.get("mav_type")
            if not isinstance(mav, int):
                continue
            if not sequence or sequence[-1] != mav:
                sequence.append(mav)
        passed = self.GROUND_MAV_TYPE in sequence and self.FLIGHT_MAV_TYPE in sequence
        if passed:
            # Look for explicit transition back to ground.
            ground_index = sequence.index(self.GROUND_MAV_TYPE)
            flight_index = sequence.index(self.FLIGHT_MAV_TYPE)
            if flight_index > ground_index:
                passed = self.GROUND_MAV_TYPE in sequence[flight_index + 1:]
        expected = [self.GROUND_MAV_TYPE, self.FLIGHT_MAV_TYPE, self.GROUND_MAV_TYPE]
        self._record(
            "T09",
            "PASS" if passed else "FAIL",
            {"mav_type_sequence": expected},
            {"sequence": sequence},
            ["samples.jsonl"],
        )

    def _evaluate_T10(self, monitor_complete):
        scan = self._scan_logs()
        fatal_matched = scan["fatal"]
        critical_matched = scan["critical"]
        fatal_count = len(fatal_matched)
        critical_count = len(critical_matched)
        allowed_count = scan["allowed_count"]
        ros_errors = scan["counts"]["ros_errors"]
        px4_errors = scan["counts"]["px4_errors"]
        tracebacks = scan["counts"]["python_tracebacks"]
        clean = monitor_complete.get("clean")
        passed = (
            fatal_count == 0
            and critical_count == 0
            and clean is not False
        )
        self._counts["fatal"] += fatal_count
        self._counts["critical_warnings"] += critical_count
        self._counts["allowed_warnings"] += allowed_count
        self._counts["ros_errors"] += ros_errors
        self._counts["px4_errors"] += px4_errors
        self._counts["python_tracebacks"] += tracebacks
        measured = {
            "matched": fatal_matched + critical_matched,
            "files_scanned": scan["files_scanned"],
        }
        self._record(
            "T10",
            "PASS" if passed else "FAIL",
            {"fatal_patterns": list(self.rules["fatal"]),
             "critical_patterns": list(self.rules["critical_warnings"]),
             "monitor_clean": True},
            measured,
            ["monitor_complete.json", "scan:{}".format(scan["files_scanned"])],
        )

    def _scan_logs(self):
        # Build the strict pattern spec consumed by the grep factory. The
        # factory is responsible for actually invoking GNU grep via
        # subprocess; here we classify the matches it returns using the
        # same fatal/critical/allowlist semantics the Python re scanner
        # previously implemented.
        strict_patterns = []
        for pattern in self.rules["fatal"]:
            strict_patterns.append(("case", pattern, "fatal"))
        for pattern in self.rules["critical_warnings"]:
            strict_patterns.append(("case", pattern, "critical"))
        for pattern in ("lockstep.*timeout",):
            strict_patterns.append(("nocase", pattern, "critical"))
        allowed_warnings = set(self.rules["allowed_warnings"])

        files = self._gather_log_files()
        case_patterns = [pattern for kind, pattern, _ in strict_patterns
                         if kind == "case"]
        nocase_patterns = [pattern for kind, pattern, _ in strict_patterns
                           if kind == "nocase"]

        try:
            raw_matches = self.grep_factory(
                {
                    "case_sensitive": list(case_patterns),
                    "case_insensitive": list(nocase_patterns),
                },
                list(self.LOG_GLOBS),
                str(self.result_dir),
            )
        except subprocess.TimeoutExpired as error:
            # A grep timeout is treated as a structured fatal match with
            # an explicit ``grep_timeout`` pattern so the operator sees
            # exactly what happened. The validator never blocks
            # indefinitely.
            timeout_s = getattr(error, "timeout", None) or self.DEFAULT_GREP_TIMEOUT_S
            self._grep_timeout = float(timeout_s)
            raw_matches = [{
                "pattern": "grep_timeout",
                "file": str(self.result_dir),
                "line": 0,
                "content": "grep_timeout after {}s".format(timeout_s),
            }]

        fatal_matches = []
        critical_matches = []
        allowed_count = 0
        ros_errors = 0
        px4_errors = 0
        tracebacks = 0
        for entry in raw_matches:
            content = entry.get("content", "")
            file_path = entry.get("file", "")
            line_no = entry.get("line", 0)
            # Synthetic entries (e.g. grep_timeout) carry their own
            # ``pattern`` key so they bypass the strict_patterns classifier
            # entirely.
            synthetic_pattern = entry.get("pattern")
            if synthetic_pattern:
                fatal_matches.append({
                    "pattern": synthetic_pattern,
                    "file": file_path,
                    "line": line_no,
                })
                continue
            for kind, pattern, classification in strict_patterns:
                compiled = self._compile_pattern(pattern, kind == "nocase")
                if compiled is None:
                    continue
                if not compiled.search(content):
                    continue
                if classification == "fatal":
                    if _is_benign_shutdown_traceback(
                        str(file_path), int(line_no)
                    ):
                        continue
                    # Fatal patterns are NEVER suppressed by the
                    # allowlist. Suppression only applies to critical
                    # matches.
                    fatal_matches.append({
                        "pattern": pattern,
                        "file": file_path,
                        "line": line_no,
                    })
                    if pattern == "[ERROR]":
                        if "ros" in content.lower():
                            ros_errors += 1
                        if "px4" in content.lower():
                            px4_errors += 1
                    if "Traceback (most recent call last)" in content:
                        tracebacks += 1
                    continue
                if classification == "critical":
                    if self._is_allowed(content, allowed_warnings):
                        allowed_count += 1
                        continue
                    critical_matches.append({
                        "pattern": pattern,
                        "file": file_path,
                        "line": line_no,
                    })
        return {
            "fatal": fatal_matches,
            "critical": critical_matches,
            "allowed_count": allowed_count,
            "files_scanned": len(files),
            "counts": {
                "ros_errors": ros_errors,
                "px4_errors": px4_errors,
                "python_tracebacks": tracebacks,
            },
        }

    def _gather_log_files(self) -> List[Path]:
        result = []
        for glob in self.LOG_GLOBS:
            for match in self.result_dir.rglob(glob):
                if match.is_file():
                    result.append(match)
        # Deduplicate.
        unique = []
        seen = set()
        for path in result:
            key = str(path)
            if key not in seen:
                seen.add(key)
                unique.append(path)
        return unique

    def _compile_pattern(self, pattern, ignore_case):
        try:
            flags = re.IGNORECASE if ignore_case else 0
            return re.compile(pattern, flags)
        except re.error:
            return None

    def _is_allowed(self, line, allowed_warnings):
        # Only exact case-sensitive literal matches are allowed; fatal
        # patterns are NEVER allowed. The caller already separates fatal
        # from critical.
        for literal in allowed_warnings:
            if literal and literal in line:
                return True
        return False

    # ---- bookkeeping ------------------------------------------------------

    def _record(self, test_id, status, expected, measured, evidence):
        self._items.append(
            self._make_item(test_id, status, expected, measured, evidence)
        )

    def _compute_overall(self, monitor_complete):
        # ABORT is latched first if monitor_complete clean is false.
        clean = monitor_complete.get("clean")
        abort_reason = monitor_complete.get("abort_reason")
        if clean is False:
            return "ABORT", abort_reason or "monitor_complete.json clean=false"
        for item in self._items:
            if item["id"] == "T06":
                # T06 is bounded-optional; failure alone does NOT fail
                # overall unless the measured error carries a hard
                # safety event.
                if item["status"] != "FAIL":
                    continue
                measured = item.get("measured", {}) or {}
                error = str(measured.get("error", ""))
                if not error:
                    continue
                return "FAIL", error
            if item["status"] != "PASS":
                return "FAIL", None
        return "PASS", None


# ---------------------------------------------------------------------------
# Convenience functions
# ---------------------------------------------------------------------------


def load_actuator_outputs_for_test(ulog):
    """Expose the production loader under the package-level test API."""
    return load_actuator_outputs(ulog)


def evaluate_run(
    result_dir,
    config: ExperimentConfig,
    log_rules_path,
    ulog_factory: Optional[Callable] = None,
    file_factory: Optional[Callable] = None,
) -> ValidationReport:
    validator = TrialValidator(
        result_dir=result_dir,
        config=config,
        log_rules_path=log_rules_path,
        ulog_factory=ulog_factory,
        file_factory=file_factory,
    )
    return validator.validate()


def write_validation_report(report: ValidationReport, path) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    summary = report.summary()
    _reject_non_finite(summary)
    encoded = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
    output.write_text(encoded, encoding="utf-8")


def _reject_non_finite(value) -> None:
    """Walk a JSON-serialisable mapping and raise on any non-finite float.

    JSON does not allow ``NaN`` or ``+/-Inf``; emitting them with the
    default ``json.dumps`` would silently produce non-portable output.
    This walker raises a ``ValueError`` so the caller can surface the
    bad measurement instead of writing garbage to disk.
    """
    if isinstance(value, Mapping):
        for key, item in value.items():
            try:
                _reject_non_finite(item)
            except ValueError as error:
                raise ValueError(
                    "non-finite value at {}: {}".format(key, error)
                ) from error
        return
    if isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            try:
                _reject_non_finite(item)
            except ValueError as error:
                raise ValueError(
                    "non-finite value at [{}]: {}".format(index, error)
                ) from error
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite float: {}".format(value))


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="validate_run")
    parser.add_argument("--result-dir", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--log-rules", required=True)
    parser.add_argument("--output", default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    config_path = _resolve_path(args.config)
    config = _load_experiment_config(config_path)
    output_path = (
        _resolve_path(args.output)
        if args.output
        else _resolve_path(args.result_dir) / "validation.json"
    )
    report = evaluate_run(
        result_dir=args.result_dir,
        config=config,
        log_rules_path=args.log_rules,
    )
    write_validation_report(report, output_path)
    return 0 if report.overall == "PASS" else 1


def _load_experiment_config(config_path) -> ExperimentConfig:
    """Load an ``ExperimentConfig`` from a YAML file matching experiment.yaml."""
    with _resolve_path(config_path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    trajectory = data.get("trajectory") or {}
    tolerances = data.get("tolerances") or {}
    timeouts = data.get("timeouts") or {}
    commands = data.get("commands") or {}
    safety = data.get("safety") or {}

    def _manual(values):
        return ManualControl(
            x=float(values.get("x", 0.0)),
            y=float(values.get("y", 0.0)),
            z=float(values.get("z", 500.0)),
            r=float(values.get("r", 0.0)),
        )

    return ExperimentConfig(
        publish_rate_hz=float(data.get("publish_rate_hz", 30.0)),
        model_name=str(data.get("model_name", "FS-FASMv8_octo")),
        trajectory=TrajectoryConfig(**trajectory),
        tolerances=TolerancesConfig(**tolerances),
        timeouts=TimeoutsConfig(**timeouts),
        commands=CommandsConfig(
            manual_neutral=_manual(commands.get("manual_neutral", {})),
            ground_neutral=_manual(commands.get("ground_neutral", {})),
            ground_forward=_manual(commands.get("ground_forward", {})),
            ground_left_turn=_manual(commands.get("ground_left_turn", {})),
            air_manual_forward=_manual(commands.get("air_manual_forward", {})),
            air_manual_duration_s=float(
                commands.get("air_manual_duration_s", 1.0)
            ),
            air_manual_brake_s=float(
                commands.get("air_manual_brake_s", 2.0)
            ),
        ),
        safety=SafetyConfig(**safety),
    )
