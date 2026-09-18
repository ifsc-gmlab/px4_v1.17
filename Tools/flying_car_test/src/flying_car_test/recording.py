"""Crash-resilient evidence recording for flying-car experiments."""

import csv
import hashlib
import json
import math
import os
import re
import subprocess
import threading
from pathlib import Path
from typing import Callable, Mapping, Optional, Sequence


JSONL_NAMES = ("samples", "events", "commands")
SAMPLE_COLUMNS = (
    "stamp",
    "clock",
    "connected",
    "armed",
    "mode",
    "landed",
    "mav_type",
    "mav_type_fresh",
    "local_x",
    "local_y",
    "local_z",
    "velocity_x",
    "velocity_y",
    "velocity_z",
    "attitude_roll",
    "attitude_pitch",
    "attitude_yaw",
    "gazebo_x",
    "gazebo_y",
    "gazebo_z",
    "clock_ok",
)
_GIT_SHA = re.compile(r"^[0-9a-fA-F]{40}$")


class MetadataError(RuntimeError):
    """Required reproducibility metadata could not be collected."""


class RecordingError(RuntimeError):
    """The recorder could not establish an explicit terminal marker."""


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _forbidden_directories(repository_root: Path):
    return {
        Path("/").resolve(),
        Path("/home").resolve(),
        Path("/home/px4").resolve(),
        repository_root,
    }


def validate_result_dir(
    result_dir, repository_root: Optional[Path] = None
) -> Path:
    """Resolve *result_dir* and reject destructive high-level targets."""
    try:
        resolved = Path(result_dir).expanduser().resolve(strict=False)
        repository = Path(
            repository_root if repository_root is not None else _repository_root()
        ).expanduser().resolve(strict=False)
    except (OSError, RuntimeError) as error:
        raise ValueError("invalid result directory") from error
    if resolved in _forbidden_directories(repository):
        raise ValueError(
            "refusing unsafe result directory: {}".format(resolved)
        )
    return resolved


def _validate_json(value, location="record") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("{} contains a non-finite value".format(location))
    if isinstance(value, Mapping):
        for key, child in value.items():
            _validate_json(child, "{}.{}".format(location, key))
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _validate_json(child, "{}[{}]".format(location, index))


def _json_object(record: Mapping[str, object]) -> str:
    if not isinstance(record, Mapping):
        raise ValueError("record must be a JSON object")
    _validate_json(record)
    try:
        return json.dumps(
            dict(record),
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("record is not strict JSON: {}".format(error)) from error


def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _run_command(args: Sequence[str], cwd=None):
    return subprocess.run(
        list(args),
        cwd=None if cwd is None else str(cwd),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )


def _required_command(
    label: str,
    args: Sequence[str],
    cwd,
    command_runner: Callable,
    allow_empty: bool = False,
) -> str:
    try:
        completed = command_runner(list(args), cwd)
        returncode = int(completed.returncode)
        stdout = str(completed.stdout).strip()
        stderr = str(getattr(completed, "stderr", "")).strip()
    except (AttributeError, OSError, TypeError, ValueError) as error:
        raise MetadataError(
            "required metadata {} unavailable: {}".format(label, error)
        ) from error
    if returncode != 0:
        raise MetadataError(
            "required metadata {} command failed ({}): {}".format(
                label, returncode, stderr
            )
        )
    if not allow_empty and not stdout:
        raise MetadataError(
            "required metadata {} was empty".format(label)
        )
    return stdout


def build_metadata(
    repository_root,
    model_sdf,
    airframe,
    experiment_config,
    launch_arguments,
    command_runner: Callable = _run_command,
    environment=None,
) -> Mapping[str, object]:
    """Collect immutable inputs needed to reproduce an experiment."""
    repository = Path(repository_root).resolve()
    env = os.environ if environment is None else environment
    distribution = str(env.get("ROS_DISTRO", "")).strip()
    if not distribution:
        raise MetadataError("required metadata ROS_DISTRO was empty")
    git_sha = _required_command(
        "PX4 Git SHA",
        ["git", "rev-parse", "HEAD"],
        repository,
        command_runner,
    )
    if not _GIT_SHA.match(git_sha):
        raise MetadataError("required metadata PX4 Git SHA was invalid")
    dirty_output = _required_command(
        "PX4 dirty status",
        ["git", "status", "--porcelain"],
        repository,
        command_runner,
        allow_empty=True,
    )
    mavros_version = _required_command(
        "MAVROS version",
        ["rosversion", "mavros"],
        None,
        command_runner,
    )
    try:
        hashes = {
            "model_sdf_sha256": sha256_file(model_sdf),
            "airframe_sha256": sha256_file(airframe),
            "experiment_config_sha256": sha256_file(experiment_config),
        }
    except OSError as error:
        raise MetadataError(
            "required metadata input hash unavailable: {}".format(error)
        ) from error
    return {
        "px4_git_sha": git_sha,
        "px4_dirty": bool(dirty_output),
        "ros_distribution": distribution,
        "mavros_version": mavros_version,
        "model_sdf_path": str(Path(model_sdf).resolve()),
        "model_sdf_sha256": hashes["model_sdf_sha256"],
        "airframe_path": str(Path(airframe).resolve()),
        "airframe_sha256": hashes["airframe_sha256"],
        "experiment_config_path": str(Path(experiment_config).resolve()),
        "experiment_config_sha256": hashes[
            "experiment_config_sha256"
        ],
        "launch_arguments": dict(launch_arguments),
    }


def _nested(record, section, field):
    value = record.get(section, {})
    return value.get(field, "") if isinstance(value, Mapping) else ""


def flatten_sample(record: Mapping[str, object]) -> Mapping[str, object]:
    return {
        "stamp": record.get("stamp", ""),
        "clock": record.get("clock", ""),
        "connected": record.get("connected", ""),
        "armed": record.get("armed", ""),
        "mode": record.get("mode", ""),
        "landed": record.get("landed", ""),
        "mav_type": record.get("mav_type", ""),
        "mav_type_fresh": record.get("mav_type_fresh", ""),
        "local_x": _nested(record, "local", "x"),
        "local_y": _nested(record, "local", "y"),
        "local_z": _nested(record, "local", "z"),
        "velocity_x": _nested(record, "velocity", "x"),
        "velocity_y": _nested(record, "velocity", "y"),
        "velocity_z": _nested(record, "velocity", "z"),
        "attitude_roll": _nested(record, "attitude", "roll"),
        "attitude_pitch": _nested(record, "attitude", "pitch"),
        "attitude_yaw": _nested(record, "attitude", "yaw"),
        "gazebo_x": _nested(record, "gazebo", "x"),
        "gazebo_y": _nested(record, "gazebo", "y"),
        "gazebo_z": _nested(record, "gazebo", "z"),
        "clock_ok": record.get("clock_ok", ""),
    }


class EvidenceRecorder:
    """Own append-only streams and publish finals only after a clean close."""

    _OPEN = "open"
    _CLOSING = "closing"
    _CLOSED = "closed"
    _FAILED = "failed"
    _MARKER_ABSENT_DURABLE = "absent_durable"
    _MARKER_ABSENT_UNSYNCED = "absent_unsynced"
    _MARKER_INDETERMINATE = "indeterminate"

    def __init__(
        self,
        result_dir,
        metadata: Mapping[str, object],
        repository_root: Optional[Path] = None,
    ):
        self._lock = threading.RLock()
        self._state = self._OPEN
        self._terminal_clean = False
        self._abort_reason: Optional[str] = None
        self._errors = []
        self._sample_count = 0
        self._event_count = 0
        self._command_count = 0
        self._streams = {}
        self._streams_closed = False
        self._data_finalized = False
        self._clean_marker_visible = False
        self._clean_marker_durability = self._MARKER_ABSENT_DURABLE
        self._directory_validated = False
        self._dir_fd = None
        self.result_dir = validate_result_dir(result_dir, repository_root)
        repository = Path(
            repository_root if repository_root is not None else _repository_root()
        ).resolve(strict=False)
        try:
            self._dir_fd = self._open_directory_chain(self.result_dir)
            self._validate_open_directory(repository)
            self._directory_validated = True
            self._write_json_atomic("metadata.json", metadata)
            for name in JSONL_NAMES:
                self._streams[name] = self._open_text_exclusive(
                    name + ".jsonl.part"
                )
        except BaseException as error:
            self._abort_reason = "initialization failed: {}".format(error)
            self._best_effort_close_streams(sync=False)
            self._best_effort_completion()
            self._close_directory()
            self._state = self._FAILED
            raise

    @staticmethod
    def _open_directory_chain(path: Path) -> int:
        """Open/create each canonical component relative to a pinned fd."""
        flags = os.O_RDONLY | os.O_DIRECTORY
        flags |= getattr(os, "O_NOFOLLOW", 0)
        current = os.open("/", flags)
        try:
            for component in path.parts[1:]:
                try:
                    os.mkdir(component, 0o700, dir_fd=current)
                except FileExistsError:
                    pass
                following = os.open(component, flags, dir_fd=current)
                os.close(current)
                current = following
            return current
        except BaseException:
            os.close(current)
            raise

    def _validate_open_directory(self, repository: Path) -> None:
        opened = os.fstat(self._dir_fd)
        current = os.stat(str(self.result_dir), follow_symlinks=False)
        if (opened.st_dev, opened.st_ino) != (
            current.st_dev,
            current.st_ino,
        ):
            raise ValueError("result directory changed during open")
        for forbidden in _forbidden_directories(repository):
            try:
                value = os.stat(str(forbidden))
            except OSError:
                continue
            if (opened.st_dev, opened.st_ino) == (
                value.st_dev,
                value.st_ino,
            ):
                raise ValueError(
                    "refusing unsafe opened result directory"
                )

    def _open_text_exclusive(self, name, newline="\n"):
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(name, flags, 0o600, dir_fd=self._dir_fd)
        try:
            return os.fdopen(fd, "w", encoding="utf-8", newline=newline)
        except BaseException:
            os.close(fd)
            raise

    def _open_text_read(self, name):
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(name, flags, dir_fd=self._dir_fd)
        try:
            return os.fdopen(fd, "r", encoding="utf-8", newline="")
        except BaseException:
            os.close(fd)
            raise

    def _directory_fsync(self) -> None:
        os.fsync(self._dir_fd)

    def _write_json_atomic(self, name, value) -> None:
        encoded = _json_object(value)
        temporary = name + ".part"
        stream = self._open_text_exclusive(temporary)
        try:
            stream.write(encoded)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
        except BaseException:
            try:
                if not stream.closed:
                    stream.close()
            except BaseException:
                pass
            try:
                os.unlink(temporary, dir_fd=self._dir_fd)
            except OSError:
                pass
            raise
        os.replace(
            temporary,
            name,
            src_dir_fd=self._dir_fd,
            dst_dir_fd=self._dir_fd,
        )
        self._directory_fsync()

    @property
    def abort_reason(self) -> Optional[str]:
        with self._lock:
            return self._abort_reason

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._state == self._CLOSED

    def _latch_abort_locked(self, reason) -> str:
        captured = str(reason).strip() or "unspecified abort"
        if self._abort_reason is None:
            self._abort_reason = captured
        return self._abort_reason

    def _append_locked(
        self,
        stream_name: str,
        record: Mapping[str, object],
        count_name: str,
    ) -> None:
        if self._state != self._OPEN:
            raise RuntimeError("recorder is not open")
        try:
            encoded = _json_object(record)
            stream = self._streams[stream_name]
            stream.write(encoded)
            stream.write("\n")
            stream.flush()
        except BaseException as error:
            self._latch_abort_locked(
                "append {} failed: {}".format(stream_name, error)
            )
            raise
        setattr(self, count_name, getattr(self, count_name) + 1)

    def _append(
        self,
        stream_name: str,
        record: Mapping[str, object],
        count_name: str,
    ) -> None:
        with self._lock:
            self._append_locked(stream_name, record, count_name)

    def append_sample(self, record: Mapping[str, object]) -> None:
        self._append("samples", record, "_sample_count")

    def append_event(self, record: Mapping[str, object]) -> None:
        self._append("events", record, "_event_count")

    def append_command(self, record: Mapping[str, object]) -> None:
        self._append("commands", record, "_command_count")

    def abort(self, reason) -> str:
        """Latch and return the first non-empty abort reason."""
        with self._lock:
            return self._latch_abort_locked(reason)

    def record_abort(
        self, event: Mapping[str, object], reason
    ) -> str:
        """Atomically latch an abort and append its event."""
        with self._lock:
            captured = self._latch_abort_locked(reason)
            if self._state == self._OPEN:
                self._append_locked(
                    "events", event, "_event_count"
                )
            return captured

    def _best_effort_close_streams(self, sync: bool) -> None:
        if self._streams_closed:
            return
        for name, stream in list(self._streams.items()):
            try:
                descriptor = stream.fileno()
            except (OSError, ValueError):
                descriptor = None
            try:
                if sync:
                    stream.flush()
                    os.fsync(descriptor)
            except BaseException as error:
                self._errors.append(
                    "{} fsync failed: {}".format(name, error)
                )
                self._latch_abort_locked(self._errors[-1])
            finally:
                try:
                    stream.close()
                except BaseException as error:
                    self._errors.append(
                        "{} close failed: {}".format(name, error)
                    )
                    self._latch_abort_locked(self._errors[-1])
                    if descriptor is not None:
                        try:
                            os.close(descriptor)
                        except OSError:
                            pass
        self._streams_closed = True

    def _read_verified_samples(self):
        samples = []
        stream = self._open_text_read("samples.jsonl.part")
        try:
            for line_number, line in enumerate(stream, 1):
                try:
                    value = json.loads(
                        line,
                        parse_constant=lambda token: (_ for _ in ()).throw(
                            ValueError(token)
                        ),
                    )
                    if not isinstance(value, Mapping):
                        raise ValueError("line is not an object")
                    _validate_json(value)
                except (TypeError, ValueError) as error:
                    raise RecordingError(
                        "invalid samples JSONL line {}: {}".format(
                            line_number, error
                        )
                    ) from error
                samples.append(dict(value))
        finally:
            stream.close()
        if len(samples) != self._sample_count:
            raise RecordingError(
                "samples JSONL count {} does not match committed count {}".format(
                    len(samples), self._sample_count
                )
            )
        return samples

    def _write_csv_part(self) -> None:
        samples = self._read_verified_samples()
        stream = self._open_text_exclusive("samples.csv.part", newline="")
        try:
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_COLUMNS)
            writer.writeheader()
            for sample in samples:
                writer.writerow(flatten_sample(sample))
            stream.flush()
            os.fsync(stream.fileno())
        finally:
            stream.close()

    def _rename_data_to_final(self) -> None:
        renamed = []
        try:
            for source, target in [
                (name + ".jsonl.part", name + ".jsonl")
                for name in JSONL_NAMES
            ] + [("samples.csv.part", "samples.csv")]:
                os.replace(
                    source,
                    target,
                    src_dir_fd=self._dir_fd,
                    dst_dir_fd=self._dir_fd,
                )
                renamed.append((source, target))
            self._directory_fsync()
            self._data_finalized = True
        except BaseException:
            for source, target in reversed(renamed):
                try:
                    os.replace(
                        target,
                        source,
                        src_dir_fd=self._dir_fd,
                        dst_dir_fd=self._dir_fd,
                    )
                except OSError:
                    pass
            try:
                self._directory_fsync()
            except OSError:
                pass
            raise

    def _rollback_data_to_parts(self) -> None:
        if not self._data_finalized:
            return
        errors = []
        for source, target in [
            (name + ".jsonl.part", name + ".jsonl")
            for name in JSONL_NAMES
        ] + [("samples.csv.part", "samples.csv")]:
            try:
                os.replace(
                    target,
                    source,
                    src_dir_fd=self._dir_fd,
                    dst_dir_fd=self._dir_fd,
                )
            except OSError as error:
                errors.append(
                    "rollback {} failed: {}".format(target, error)
                )
        try:
            self._directory_fsync()
        except OSError as error:
            errors.append("rollback directory fsync failed: {}".format(error))
        self._data_finalized = False
        if errors:
            self._errors.extend(errors)
            self._latch_abort_locked(errors[0])

    def _completion(self, clean: bool):
        return {
            "clean": bool(clean),
            "abort_reason": None if clean else self._abort_reason,
            "errors": list(self._errors),
            "samples": self._sample_count,
            "events": self._event_count,
            "commands": self._command_count,
        }

    def _best_effort_completion(self) -> bool:
        if self._dir_fd is None or not self._directory_validated:
            return False
        try:
            self._write_json_atomic(
                "monitor_complete.json", self._completion(False)
            )
            return True
        except BaseException:
            return False

    def _completion_marker_exists(self) -> Optional[bool]:
        try:
            os.stat(
                "monitor_complete.json",
                dir_fd=self._dir_fd,
                follow_symlinks=False,
            )
            return True
        except FileNotFoundError:
            return False
        except OSError as error:
            self._errors.append(
                "completion marker stat failed: {}".format(error)
            )
            return None

    def _sync_completion_marker_absence(self) -> bool:
        try:
            self._directory_fsync()
        except OSError as error:
            self._errors.append(
                "completion marker cleanup fsync failed: {}".format(error)
            )
            return False
        self._clean_marker_durability = self._MARKER_ABSENT_DURABLE
        return True

    def _remove_completion_marker(
        self, sync_uncertain_first: bool = False
    ) -> bool:
        """Remove a published marker before data finals may be rolled back."""
        namespace_synced = False
        if (
            sync_uncertain_first
            and self._clean_marker_durability
            != self._MARKER_ABSENT_DURABLE
        ):
            try:
                self._directory_fsync()
            except OSError as error:
                self._errors.append(
                    "completion marker cleanup precondition fsync failed: "
                    "{}".format(error)
                )
                return False
            namespace_synced = True
            if (
                self._clean_marker_durability
                == self._MARKER_ABSENT_UNSYNCED
            ):
                self._clean_marker_durability = (
                    self._MARKER_ABSENT_DURABLE
                )
                return True

        marker_exists = self._completion_marker_exists()
        if marker_exists is None:
            self._clean_marker_durability = self._MARKER_INDETERMINATE
            return False
        if not marker_exists:
            if (
                namespace_synced
                or self._clean_marker_durability
                == self._MARKER_ABSENT_DURABLE
            ):
                self._clean_marker_durability = (
                    self._MARKER_ABSENT_DURABLE
                )
                return True
            self._clean_marker_durability = (
                self._MARKER_ABSENT_UNSYNCED
            )
            return self._sync_completion_marker_absence()
        try:
            os.unlink("monitor_complete.json", dir_fd=self._dir_fd)
        except OSError as error:
            self._clean_marker_durability = self._MARKER_INDETERMINATE
            self._errors.append(
                "completion marker cleanup failed: {}".format(error)
            )
            return False
        self._clean_marker_durability = self._MARKER_ABSENT_UNSYNCED
        return self._sync_completion_marker_absence()

    def _close_directory(self) -> None:
        if self._dir_fd is not None:
            try:
                os.close(self._dir_fd)
            finally:
                self._dir_fd = None

    def close(
        self, clean: bool, abort_reason: Optional[str] = None
    ) -> bool:
        """Finalize once, retaining JSONL parts on every abnormal path."""
        with self._lock:
            if self._state == self._CLOSED:
                return self._terminal_clean
            if self._state == self._CLOSING:
                return False
            if self._dir_fd is None:
                raise RecordingError("recorder has no open result directory")
            if self._state == self._FAILED and self._clean_marker_visible:
                retry_clean = (
                    bool(clean)
                    and abort_reason is None
                    and self._abort_reason is None
                )
                if retry_clean:
                    marker_exists = self._completion_marker_exists()
                    if marker_exists is None:
                        raise RecordingError("; ".join(self._errors))
                    self._clean_marker_durability = (
                        self._MARKER_INDETERMINATE
                    )
                    try:
                        if marker_exists:
                            self._directory_fsync()
                        else:
                            self._write_json_atomic(
                                "monitor_complete.json",
                                self._completion(True),
                            )
                    except BaseException as error:
                        self._errors.append(
                            "completion retry failed: {}".format(error)
                        )
                        raise RecordingError(
                            "; ".join(self._errors)
                        ) from error
                    self._terminal_clean = True
                    self._state = self._CLOSED
                    self._close_directory()
                    return True
                if abort_reason is not None:
                    self._latch_abort_locked(abort_reason)
                elif not clean and self._abort_reason is None:
                    self._latch_abort_locked(
                        "monitor closed before experiment completion"
                    )
                if not self._remove_completion_marker(
                    sync_uncertain_first=True
                ):
                    raise RecordingError("; ".join(self._errors))
                self._clean_marker_visible = False
                self._rollback_data_to_parts()
            self._state = self._CLOSING
            if abort_reason is not None:
                self._latch_abort_locked(abort_reason)
            if not clean and self._abort_reason is None:
                self._latch_abort_locked(
                    "monitor closed before experiment completion"
                )

            self._best_effort_close_streams(sync=True)
            actual_clean = bool(clean) and self._abort_reason is None
            if actual_clean:
                try:
                    self._write_csv_part()
                    self._rename_data_to_final()
                except BaseException as error:
                    self._errors.append(
                        "clean finalization failed: {}".format(error)
                    )
                    self._latch_abort_locked(self._errors[-1])
                    actual_clean = False

            try:
                self._write_json_atomic(
                    "monitor_complete.json",
                    self._completion(actual_clean),
                )
            except BaseException as error:
                failure = "completion marker failed: {}".format(error)
                self._errors.append(failure)
                self._state = self._FAILED
                if actual_clean:
                    self._clean_marker_durability = (
                        self._MARKER_INDETERMINATE
                    )
                    try:
                        marker_removed = self._remove_completion_marker()
                    except BaseException as cleanup_error:
                        self._errors.append(
                            "completion marker cleanup failed: {}".format(
                                cleanup_error
                            )
                        )
                        marker_removed = False
                    if marker_removed:
                        self._latch_abort_locked(failure)
                        self._rollback_data_to_parts()
                    else:
                        # Preserve final data until a clean publication can be
                        # durably confirmed or safely replaced.
                        self._clean_marker_visible = True
                else:
                    self._latch_abort_locked(failure)
                raise RecordingError(
                    "; ".join(self._errors)
                ) from error
            self._terminal_clean = actual_clean
            self._state = self._CLOSED
            self._close_directory()
            return actual_clean

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, _traceback):
        if exception is None:
            self.close(clean=True)
        else:
            self.close(
                clean=False,
                abort_reason="{}: {}".format(
                    exception_type.__name__, exception
                ),
            )
        return False
