import csv
import json
import math
import os
import stat
from pathlib import Path
from types import SimpleNamespace

import pytest

import flying_car_test.recording as recording
from flying_car_test.recording import (
    SAMPLE_COLUMNS,
    EvidenceRecorder,
    MetadataError,
    RecordingError,
    build_metadata,
    validate_result_dir,
)


def sample(stamp=1.0):
    return {
        "stamp": stamp,
        "clock": stamp,
        "connected": True,
        "armed": False,
        "mode": "MANUAL",
        "landed": True,
        "mav_type": 10,
        "mav_type_fresh": True,
        "local": {"x": 1.0, "y": 2.0, "z": 3.0},
        "velocity": {"x": 4.0, "y": 5.0, "z": 6.0},
        "attitude": {"roll": 0.1, "pitch": 0.2, "yaw": 0.3},
        "gazebo": {"x": 7.0, "y": 8.0, "z": 9.0},
        "clock_ok": True,
    }


def strict_json_lines(path):
    with path.open(encoding="utf-8") as stream:
        return [
            json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(value)
            ))
            for line in stream
        ]


def test_append_writes_one_strict_json_object_per_line(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={"trial": 1})
    recorder.append_sample(sample(1.0))
    recorder.append_sample(sample(2.0))
    recorder.append_event({"stamp": 1.0, "phase": "PRECHECK"})
    recorder.append_command({"stamp": 1.0, "kind": "manual"})
    recorder.close(clean=True)

    assert [row["stamp"] for row in strict_json_lines(
        tmp_path / "run" / "samples.jsonl"
    )] == [1.0, 2.0]
    assert len(strict_json_lines(tmp_path / "run" / "events.jsonl")) == 1
    assert len(strict_json_lines(tmp_path / "run" / "commands.jsonl")) == 1


def test_clean_close_creates_stable_csv_matching_jsonl(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample(1.0))
    recorder.append_sample(sample(2.0))
    recorder.close(clean=True)

    with (tmp_path / "run" / "samples.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == list(SAMPLE_COLUMNS)
    assert len(rows) - 1 == len(strict_json_lines(
        tmp_path / "run" / "samples.jsonl"
    ))
    assert not list((tmp_path / "run").glob("*.part"))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_values_are_rejected_without_partial_line(tmp_path, value):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    invalid = sample()
    invalid["gazebo"]["z"] = value
    with pytest.raises(ValueError, match="finite"):
        recorder.append_sample(invalid)
    assert "append samples failed" in recorder.abort_reason
    recorder.close(clean=False, abort_reason="invalid telemetry")
    assert (tmp_path / "run" / "samples.jsonl.part").read_text(
        encoding="utf-8"
    ) == ""


@pytest.mark.parametrize(
    "forbidden",
    [Path("/"), Path("/home"), Path("/home/px4")],
)
def test_dangerous_result_directories_are_rejected(forbidden):
    with pytest.raises(ValueError, match="result directory"):
        validate_result_dir(forbidden)


def test_repository_and_symlink_alias_to_it_are_rejected(tmp_path):
    repository = tmp_path / "PX4-Autopilot"
    repository.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(repository, target_is_directory=True)
    with pytest.raises(ValueError, match="result directory"):
        validate_result_dir(repository, repository_root=repository)
    with pytest.raises(ValueError, match="result directory"):
        validate_result_dir(alias, repository_root=repository)


def test_abnormal_close_retains_parts_and_preserves_first_abort_reason(tmp_path):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    recorder.abort("first failure")
    recorder.abort("later failure")
    recorder.close(clean=False, abort_reason="shutdown")

    assert (tmp_path / "run" / "samples.jsonl.part").is_file()
    assert not (tmp_path / "run" / "samples.jsonl").exists()
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is False
    assert complete["abort_reason"] == "first failure"


def test_close_fsyncs_every_open_stream(monkeypatch, tmp_path):
    fsynced = []
    monkeypatch.setattr("flying_car_test.recording.os.fsync", fsynced.append)
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    recorder.close(clean=True)

    # metadata, three JSONL streams, CSV, and completion marker
    assert len(fsynced) >= 6


def test_metadata_contains_reproducibility_fields_and_sha256(tmp_path):
    repository = tmp_path / "repo"
    repository.mkdir()
    model = tmp_path / "model.sdf"
    airframe = tmp_path / "airframe"
    config = tmp_path / "experiment.yaml"
    model.write_bytes(b"model")
    airframe.write_bytes(b"airframe")
    config.write_bytes(b"config")

    responses = {
        ("git", "rev-parse", "HEAD"): (0, "a" * 40 + "\n"),
        ("git", "status", "--porcelain"): (0, ""),
        ("rosversion", "mavros"): (0, "1.17.0\n"),
    }

    def run(args, _cwd=None):
        return SimpleNamespace(
            returncode=responses[tuple(args)][0],
            stdout=responses[tuple(args)][1],
            stderr="",
        )

    metadata = build_metadata(
        repository_root=repository,
        model_sdf=model,
        airframe=airframe,
        experiment_config=config,
        launch_arguments={"gui": False},
        command_runner=run,
        environment={"ROS_DISTRO": "noetic"},
    )

    assert metadata["px4_git_sha"] == "a" * 40
    assert metadata["px4_dirty"] is False
    assert metadata["ros_distribution"] == "noetic"
    assert metadata["mavros_version"] == "1.17.0"
    assert metadata["model_sdf_sha256"] == (
        "9372c470eeadd5ecd9c3c74c2b3cb633"
        "f8e2f2fad799250a0f70d652b6b825e4"
    )
    assert metadata["airframe_sha256"] == (
        "f07f3efaa1da8968837714df0dedd297"
        "b780f6c37acfcc1dfcc3b86e11139ac6"
    )
    assert metadata["experiment_config_sha256"] == (
        "b79606fb3afea5bd1609ed40b622142f"
        "1c98125abcfe89a76a661b0e8e343910"
    )
    assert metadata["launch_arguments"] == {"gui": False}


@pytest.mark.parametrize(
    "failed_command",
    [
        ("git", "rev-parse", "HEAD"),
        ("git", "status", "--porcelain"),
        ("rosversion", "mavros"),
    ],
)
def test_metadata_rejects_failed_required_commands(tmp_path, failed_command):
    files = []
    for name in ("model.sdf", "airframe", "experiment.yaml"):
        path = tmp_path / name
        path.write_text(name, encoding="utf-8")
        files.append(path)

    def run(args, _cwd=None):
        failed = tuple(args) == failed_command
        output = {
            ("git", "rev-parse", "HEAD"): "b" * 40,
            ("git", "status", "--porcelain"): "",
            ("rosversion", "mavros"): "1.17.0",
        }[tuple(args)]
        return SimpleNamespace(
            returncode=1 if failed else 0,
            stdout="" if failed else output,
            stderr="failure" if failed else "",
        )

    with pytest.raises(MetadataError, match="required metadata"):
        build_metadata(
            repository_root=tmp_path,
            model_sdf=files[0],
            airframe=files[1],
            experiment_config=files[2],
            launch_arguments={},
            command_runner=run,
            environment={"ROS_DISTRO": "noetic"},
        )


def test_metadata_rejects_missing_ros_distribution(tmp_path):
    files = []
    for name in ("model.sdf", "airframe", "experiment.yaml"):
        path = tmp_path / name
        path.write_text(name, encoding="utf-8")
        files.append(path)

    def run(args, _cwd=None):
        output = (
            "c" * 40
            if tuple(args) == ("git", "rev-parse", "HEAD")
            else "1.17.0"
        )
        if tuple(args) == ("git", "status", "--porcelain"):
            output = ""
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    with pytest.raises(MetadataError, match="ROS_DISTRO"):
        build_metadata(
            repository_root=tmp_path,
            model_sdf=files[0],
            airframe=files[1],
            experiment_config=files[2],
            launch_arguments={},
            command_runner=run,
            environment={},
        )


@pytest.mark.parametrize(
    "empty_command",
    [
        ("git", "rev-parse", "HEAD"),
        ("rosversion", "mavros"),
    ],
)
def test_metadata_rejects_empty_required_command_output(
    tmp_path, empty_command
):
    files = []
    for name in ("model.sdf", "airframe", "experiment.yaml"):
        path = tmp_path / name
        path.write_text(name, encoding="utf-8")
        files.append(path)

    def run(args, _cwd=None):
        output = {
            ("git", "rev-parse", "HEAD"): "d" * 40,
            ("git", "status", "--porcelain"): "",
            ("rosversion", "mavros"): "1.17.0",
        }[tuple(args)]
        if tuple(args) == empty_command:
            output = ""
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    with pytest.raises(MetadataError, match="required metadata"):
        build_metadata(
            repository_root=tmp_path,
            model_sdf=files[0],
            airframe=files[1],
            experiment_config=files[2],
            launch_arguments={},
            command_runner=run,
            environment={"ROS_DISTRO": "noetic"},
        )


def test_metadata_rejects_missing_command_executable(tmp_path):
    files = []
    for name in ("model.sdf", "airframe", "experiment.yaml"):
        path = tmp_path / name
        path.write_text(name, encoding="utf-8")
        files.append(path)

    def missing(_args, _cwd=None):
        raise FileNotFoundError("command unavailable")

    with pytest.raises(MetadataError, match="unavailable"):
        build_metadata(
            repository_root=tmp_path,
            model_sdf=files[0],
            airframe=files[1],
            experiment_config=files[2],
            launch_arguments={},
            command_runner=missing,
            environment={"ROS_DISTRO": "noetic"},
        )


def test_metadata_records_dirty_true_separately_from_clean(tmp_path):
    files = []
    for name in ("model.sdf", "airframe", "experiment.yaml"):
        path = tmp_path / name
        path.write_text(name, encoding="utf-8")
        files.append(path)

    def run(args, _cwd=None):
        output = {
            ("git", "rev-parse", "HEAD"): "e" * 40,
            ("git", "status", "--porcelain"): " M tracked-file",
            ("rosversion", "mavros"): "1.17.0",
        }[tuple(args)]
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    metadata = build_metadata(
        repository_root=tmp_path,
        model_sdf=files[0],
        airframe=files[1],
        experiment_config=files[2],
        launch_arguments={},
        command_runner=run,
        environment={"ROS_DISTRO": "noetic"},
    )
    assert metadata["px4_dirty"] is True


class FailFlushOnce:
    def __init__(self, stream):
        self.stream = stream
        self.failed = False

    def __getattr__(self, name):
        return getattr(self.stream, name)

    def write(self, value):
        return self.stream.write(value)

    def flush(self):
        if not self.failed:
            self.failed = True
            raise OSError("injected append flush failure")
        return self.stream.flush()


def test_append_io_failure_forces_abnormal_and_prevents_csv_divergence(
    tmp_path,
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder._streams["samples"] = FailFlushOnce(
        recorder._streams["samples"]
    )
    with pytest.raises(OSError, match="injected"):
        recorder.append_sample(sample())

    recorder.close(clean=True)
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is False
    assert "append samples failed" in complete["abort_reason"]
    assert (tmp_path / "run" / "samples.jsonl.part").is_file()
    assert not (tmp_path / "run" / "samples.csv").exists()


def _open_targets():
    targets = set()
    for entry in Path("/proc/self/fd").iterdir():
        try:
            targets.add(os.readlink(str(entry)))
        except OSError:
            pass
    return targets


def test_constructor_stream_failure_closes_handles_and_marks_abnormal(tmp_path):
    result = tmp_path / "run"
    result.mkdir()
    (result / "events.jsonl.part").write_text("", encoding="utf-8")

    with pytest.raises(FileExistsError):
        EvidenceRecorder(result, metadata={})

    assert str(result / "samples.jsonl.part") not in _open_targets()
    complete = json.loads(
        (result / "monitor_complete.json").read_text(encoding="utf-8")
    )
    assert complete["clean"] is False
    assert "initialization failed" in complete["abort_reason"]


def test_fsync_failure_closes_all_streams_and_writes_abnormal_marker(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    streams = list(recorder._streams.values())
    real_fsync = recording.os.fsync
    failed = [False]

    def fail_samples_once(fd):
        target = os.readlink("/proc/self/fd/{}".format(fd))
        if target.endswith("samples.jsonl.part") and not failed[0]:
            failed[0] = True
            raise OSError("injected final fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(recording.os, "fsync", fail_samples_once)
    assert recorder.close(clean=True) is False

    assert all(stream.closed for stream in streams)
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is False
    assert "fsync" in complete["abort_reason"]


def test_directory_swap_cannot_redirect_final_artifacts_into_repository(
    tmp_path,
):
    repository = tmp_path / "PX4-Autopilot"
    repository.mkdir()
    result = tmp_path / "run"
    recorder = EvidenceRecorder(
        result, metadata={}, repository_root=repository
    )
    recorder.append_sample(sample())
    original_inode = tmp_path / "original-run"
    result.rename(original_inode)
    result.symlink_to(repository, target_is_directory=True)

    recorder.close(clean=True)

    assert (original_inode / "monitor_complete.json").is_file()
    assert (original_inode / "samples.csv").is_file()
    assert not (repository / "monitor_complete.json").exists()
    assert not (repository / "samples.csv").exists()


def test_clean_close_fsyncs_directory_after_data_and_completion_renames(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    directory_syncs = []

    def observe(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs.append(fd)
        return real_fsync(fd)

    monkeypatch.setattr(recording.os, "fsync", observe)
    recorder.close(clean=True)

    assert len(directory_syncs) >= 2


def test_completion_failure_remains_retryable_and_ends_abnormal(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    failed = [False]

    def fail_completion_once(fd):
        target = os.readlink("/proc/self/fd/{}".format(fd))
        if target.endswith("monitor_complete.json.part") and not failed[0]:
            failed[0] = True
            raise OSError("injected completion fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(recording.os, "fsync", fail_completion_once)
    with pytest.raises(RecordingError, match="completion marker"):
        recorder.close(clean=True)
    assert recorder.closed is False

    monkeypatch.setattr(recording.os, "fsync", real_fsync)
    assert recorder.close(clean=False) is False
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is False
    assert "completion marker failed" in complete["abort_reason"]
    assert (tmp_path / "run" / "samples.jsonl.part").is_file()
    assert not (tmp_path / "run" / "samples.csv").exists()


def test_completion_directory_fsync_failure_removes_clean_marker_before_rollback(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    directory_syncs = [0]

    def fail_second_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs[0] += 1
            if directory_syncs[0] == 2:
                raise OSError("injected completion directory fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(recording.os, "fsync", fail_second_directory_sync)
    with pytest.raises(RecordingError, match="completion marker"):
        recorder.close(clean=True)
    marker_path = tmp_path / "run" / "monitor_complete.json"
    marker_was_clean = (
        marker_path.exists()
        and json.loads(marker_path.read_text(encoding="utf-8"))["clean"]
    )
    data_was_part = (tmp_path / "run" / "samples.jsonl.part").exists()

    monkeypatch.setattr(recording.os, "fsync", real_fsync)
    assert recorder.close(clean=False) is False
    assert marker_was_clean is False
    assert data_was_part is True


def test_failed_clean_marker_cleanup_does_not_roll_back_final_data(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    real_unlink = recording.os.unlink
    directory_syncs = [0]

    def fail_second_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs[0] += 1
            if directory_syncs[0] == 2:
                raise OSError("injected completion directory fsync failure")
        return real_fsync(fd)

    def refuse_clean_marker_removal(path, *args, **kwargs):
        if path == "monitor_complete.json":
            raise OSError("injected marker cleanup failure")
        return real_unlink(path, *args, **kwargs)

    monkeypatch.setattr(recording.os, "fsync", fail_second_directory_sync)
    monkeypatch.setattr(recording.os, "unlink", refuse_clean_marker_removal)
    with pytest.raises(RecordingError, match="completion marker"):
        recorder.close(clean=True)
    marker_was_clean = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )["clean"]
    finals_existed = all(
        (tmp_path / "run" / name).exists()
        for name in (
            "samples.jsonl",
            "events.jsonl",
            "commands.jsonl",
            "samples.csv",
        )
    )
    parts_existed = any(
        (tmp_path / "run" / name).exists()
        for name in (
            "samples.jsonl.part",
            "events.jsonl.part",
            "commands.jsonl.part",
        )
    )

    monkeypatch.setattr(recording.os, "fsync", real_fsync)
    monkeypatch.setattr(recording.os, "unlink", real_unlink)
    retry_clean = recorder.close(clean=True)
    assert marker_was_clean is True
    assert finals_existed is True
    assert parts_existed is False
    assert retry_clean is True


def test_cleanup_directory_fsync_failure_preserves_finals_for_retry(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    directory_syncs = [0]

    def fail_publish_and_cleanup_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs[0] += 1
            if directory_syncs[0] in (2, 3):
                raise OSError(
                    "injected directory fsync {}".format(
                        directory_syncs[0]
                    )
                )
        return real_fsync(fd)

    monkeypatch.setattr(
        recording.os, "fsync", fail_publish_and_cleanup_directory_sync
    )
    with pytest.raises(RecordingError, match="completion marker"):
        recorder.close(clean=True)
    finals_existed = all(
        (tmp_path / "run" / name).exists()
        for name in (
            "samples.jsonl",
            "events.jsonl",
            "commands.jsonl",
            "samples.csv",
        )
    )
    parts_existed = any(
        (tmp_path / "run" / name).exists()
        for name in (
            "samples.jsonl.part",
            "events.jsonl.part",
            "commands.jsonl.part",
        )
    )

    monkeypatch.setattr(recording.os, "fsync", real_fsync)
    assert recorder.close(clean=True) is True
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert finals_existed is True
    assert parts_existed is False
    assert complete["clean"] is True
    assert recorder.closed is True


def test_abnormal_retry_syncs_uncertain_marker_deletion_before_rollback(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    real_replace = recording.os.replace
    directory_syncs = [0]
    phase = ["initial"]
    operations = []

    def fail_publish_and_cleanup_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs[0] += 1
            if directory_syncs[0] in (2, 3):
                raise OSError(
                    "injected directory fsync {}".format(
                        directory_syncs[0]
                    )
                )
        result = real_fsync(fd)
        if phase[0] == "retry" and stat.S_ISDIR(os.fstat(fd).st_mode):
            operations.append(("directory_fsync",))
        return result

    def observe_replace(source, target, *args, **kwargs):
        result = real_replace(source, target, *args, **kwargs)
        if phase[0] == "retry":
            operations.append(("replace", source, target))
        return result

    monkeypatch.setattr(
        recording.os, "fsync", fail_publish_and_cleanup_directory_sync
    )
    monkeypatch.setattr(recording.os, "replace", observe_replace)
    with pytest.raises(RecordingError, match="completion marker"):
        recorder.close(clean=True)

    assert not (tmp_path / "run" / "monitor_complete.json").exists()
    assert all(
        (tmp_path / "run" / name).exists()
        for name in (
            "samples.jsonl",
            "events.jsonl",
            "commands.jsonl",
            "samples.csv",
        )
    )

    phase[0] = "retry"
    assert recorder.close(clean=False, abort_reason="first abort") is False

    first_directory_sync = operations.index(("directory_fsync",))
    first_rollback = operations.index(
        ("replace", "samples.jsonl", "samples.jsonl.part")
    )
    assert first_directory_sync < first_rollback
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["abort_reason"] == "first abort"


def test_marker_stat_error_exits_closing_and_clean_retry_closes_directory(
    monkeypatch, tmp_path
):
    recorder = EvidenceRecorder(tmp_path / "run", metadata={})
    recorder.append_sample(sample())
    real_fsync = recording.os.fsync
    real_stat = recording.os.stat
    directory_syncs = [0]
    failed_stat = [False]

    def fail_completion_directory_sync(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_syncs[0] += 1
            if directory_syncs[0] == 2:
                raise OSError("injected completion directory fsync failure")
        return real_fsync(fd)

    def fail_marker_stat_once(path, *args, **kwargs):
        if (
            path == "monitor_complete.json"
            and kwargs.get("dir_fd") is not None
            and not failed_stat[0]
        ):
            failed_stat[0] = True
            raise OSError("injected marker stat EIO")
        return real_stat(path, *args, **kwargs)

    monkeypatch.setattr(recording.os, "fsync", fail_completion_directory_sync)
    monkeypatch.setattr(recording.os, "stat", fail_marker_stat_once)
    with pytest.raises(RecordingError, match="marker stat EIO"):
        recorder.close(clean=True)

    monkeypatch.setattr(recording.os, "fsync", real_fsync)
    monkeypatch.setattr(recording.os, "stat", real_stat)
    assert recorder.close(clean=True) is True
    assert recorder.closed is True
    assert recorder._dir_fd is None
    complete = json.loads(
        (tmp_path / "run" / "monitor_complete.json").read_text(
            encoding="utf-8"
        )
    )
    assert complete["clean"] is True
