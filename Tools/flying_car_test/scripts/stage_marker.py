"""Flying-car experiment stage marker CLI.

Self-contained entry point that satisfies the Task 11 contract
end-to-end.  Implementation history:

* Initial version (commit 367aca9a8): minimal validator + helper.
* Fix Round 1 (commit d59115b3b5): moved implementation into this
  self-contained script (deleting the out-of-scope ``src`` module),
  introduced on-disk state with ``fcntl`` cross-process lock, ROS
  ``Time.now()`` stamp, and the COMPLETE phase bridge.
* Fix Round 2 (this version): implements the full external-review
  contract documented in
  ``.superpowers/sdd/2026-07-30-flying-car-experiment-implementation/review-d59115b3b5..HEAD.diff``:

  - **Critical #1** — both ``STAGE_MARKER_TOPIC`` and ``PHASE_TOPIC``
    are now ``latch=True``; after publish we call
    ``ros.sleep(POST_PUBLISH_FLUSH_S``) so the Linger buffer drains
    before the script exits.
  - **Critical #1+#2** — the publish path waits independently for the
    stage and phase consumers, writes a durable pending intent, publishes
    only missing topic deliveries, then promotes the sequence to
    committed.  A process interruption therefore leaves a resumable
    write-ahead record instead of a false committed marker.
  - **Important #4** — ``init_node`` is called with
    ``anonymous=True`` so concurrent CLI invocations get unique node
    names.
  - **Important #5** — ``--reset`` (or ``STAGE_MARKER_RESET=1`` env
    var) clears the persisted and pending state for a fresh trial.
  - The on-disk state file is the canonical truth; the public
    ``/flying_car_test/stage_sequence`` ROS parameter mirrors pending
    and committed state while the ROS master is running.

``main(['--help'])`` exits with status 0.  An empty argv prints the
same usage text but exits with status 2 so a missing positional
argument does not silently pass.

The file has no third-party dependencies beyond the Python standard
library.  ROS (``rospy`` + ``std_msgs.msg.String``) is imported
lazily inside :func:`_resolve_rospy` so that the module can be
loaded in environments where ``rospy`` is unavailable (eg. the
catkin build host, or a CI container without ``/opt/ros``).
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Mapping, Optional


#: Sentinel module attribute that tests use to inject a fake ROS
#: (``monkeypatch.setattr(module, "rospy", fake)``).  Set to ``None`` so
#: ``_resolve_rospy`` falls back to the real ``import rospy`` at first
#: use; never imported at module load.
rospy: Any = None

#: Sentinel module attribute that tests use to inject a fake
#: ``std_msgs.msg.String`` subclass (``monkeypatch.setattr(module,
#: "std_msgs", _Stub)``).  Same lifecycle as :data:`rospy`.
std_msgs: Any = None


# ---------------------------------------------------------------------------
# Canonical constants (kept stable across releases).
# ---------------------------------------------------------------------------

#: The seven stage markers in canonical order.  Each maps to its
#: 1-based sequence number - the values must remain a dense 1..N range
#: so the strict "previous + 1 == next" check holds.
ALLOWED_MARKERS: Mapping[str, int] = {
    "ground_forward_start": 1,
    "flight_switch_start": 2,
    "takeoff_start": 3,
    "air_forward_start": 4,
    "landing_start": 5,
    "final_ground_start": 6,
    "experiment_end": 7,
}

#: Marker -> phase string published to ``PHASE_TOPIC``.  The values
#: mirror the names documented in ``src/flying_car_test/state_machine.py``
#: so the passive ``experiment_monitor`` sees a familiar phase timeline.
MARKER_PHASE: Mapping[str, str] = {
    "ground_forward_start": "GROUND_FORWARD",
    "flight_switch_start": "FLIGHT_CONFIGURE",
    "takeoff_start": "TAKEOFF",
    "air_forward_start": "AIR_FORWARD",
    "landing_start": "LAND",
    "final_ground_start": "GROUND_DIAGONAL",
    "experiment_end": "COMPLETE",
}

#: Stage-marker latched topic - recorded by the rosbag in both
#: ``full_experiment.launch`` and ``monitor_only.launch``.
STAGE_MARKER_TOPIC = "/flying_car_test/stage_marker"

#: Controller phase topic the passive ``experiment_monitor`` subscribes
#: to.  We publish pure-string phase messages here so the manual
#: monitor-only trial can reach ``phase == COMPLETE`` without starting
#: the ``experiment_controller`` node.
PHASE_TOPIC = "/flying_car_experiment_controller/phase"

#: Public, stable parameter name that mirrors the committed sequence
#: to anyone who calls ``ros.get_param`` within the lifetime of one
#: CLI invocation.  The leading slash mirrors the stage-marker topic
#: convention so the value is unambiguous (``stage_marker`` already
#: implies the ``stage_sequence`` value).
#:
#: NOTE: The ROS parameter survives independent ``rosrun`` calls while
#: the same ROS master remains alive, but it disappears with that
#: master's lifecycle.  It is therefore an advisory mirror; canonical
#: persistence lives in the on-disk state file.  Tests pin this string
#: to ``/flying_car_test/stage_sequence`` so the contract is stable
#: across releases.
PARAMETER_NAME = "/flying_car_test/stage_sequence"

#: Highest valid sequence number (also serves as the "experiment
#: already completed" sentinel for the validator).
MAX_SEQUENCE = len(ALLOWED_MARKERS)

#: How long (seconds) the publish path waits for at least one
#: subscriber to attach before giving up.  Tunable via ``monkeypatch``
#: from the test suite.
SUBSCRIBER_WAIT_TIMEOUT_S = 5.0

#: How long (seconds) the script sleeps after publishing both topics
#: to let the Linger queue drain.  Tunable via ``monkeypatch`` from
#: the test suite; default 0.1 s is short enough to keep the CLI
#: responsive but long enough to flush a non-latched edge case.
POST_PUBLISH_FLUSH_S = 0.1


# ---------------------------------------------------------------------------
# Errors and validation.
# ---------------------------------------------------------------------------


class StageMarkerError(ValueError):
    """Raised when a marker fails validation.

    The CLI translates this exception into a non-zero POSIX exit
    status and does NOT publish anything on the ROS topics.
    """


class SubscriberUnavailable(RuntimeError):
    """Raised by :func:`_wait_for_subscriber` if no subscriber attaches
    within :data:`SUBSCRIBER_WAIT_TIMEOUT_S`.

    Distinct from :class:`StageMarkerError` so the CLI can publish a
    diagnostic that distinguishes "no one is listening" from "the
    marker you asked for is invalid".
    """


def _coerce_previous(previous_sequence: object) -> Optional[int]:
    if previous_sequence is None:
        return None
    if isinstance(previous_sequence, bool):
        raise StageMarkerError(
            "previous_sequence must be None or a non-negative int, "
            f"got bool {previous_sequence!r}"
        )
    if isinstance(previous_sequence, int):
        if previous_sequence < 0:
            raise StageMarkerError(
                "previous_sequence must be None or a non-negative int, "
                f"got {previous_sequence!r}"
            )
        return previous_sequence
    raise StageMarkerError(
        "previous_sequence must be None or a non-negative int, "
        f"got {type(previous_sequence).__name__} {previous_sequence!r}"
    )


def _validate_marker_name(marker: object) -> str:
    if not isinstance(marker, str):
        raise StageMarkerError(
            "marker must be a string, got "
            f"{type(marker).__name__} {marker!r}"
        )
    if marker == "":
        raise StageMarkerError("marker must be a non-empty string")
    if marker not in ALLOWED_MARKERS:
        allowed = ", ".join(repr(name) for name in ALLOWED_MARKERS)
        raise StageMarkerError(
            f"unknown marker {marker!r}; allowed markers are: {allowed}"
        )
    return marker


def validate_marker(marker: str, previous_sequence: Optional[int]) -> int:
    """Validate ``marker`` against ``previous_sequence`` and return the
    new 1-based sequence.

    Rules:

    * ``marker`` must be one of :data:`ALLOWED_MARKERS`.
    * The new sequence must equal ``previous_sequence + 1`` when a
      previous sequence exists, or ``1`` when ``None``.
    * Duplicates raise with ``"duplicate"`` in the message; out-of-order
      markers raise with ``"out-of-order"`` in the message.
    * Sequences ``>= MAX_SEQUENCE+1`` reject further markers with an
      "experiment already completed" message.
    """

    name = _validate_marker_name(marker)
    previous = _coerce_previous(previous_sequence)
    expected = ALLOWED_MARKERS[name]

    if previous is None:
        if expected != 1:
            raise StageMarkerError(
                f"out-of-order marker {name!r}: expected first marker "
                f"to be one of {ALLOWED_MARKERS!r}"
            )
        return expected

    if previous >= MAX_SEQUENCE:
        raise StageMarkerError(
            f"experiment already completed at sequence {previous}; "
            f"no further markers accepted"
        )

    next_expected = previous + 1
    if expected == previous:
        raise StageMarkerError(
            f"duplicate marker {name!r} at sequence {previous}"
        )
    if expected < next_expected:
        raise StageMarkerError(
            f"out-of-order marker {name!r}: expected sequence "
            f"{next_expected} (the next marker after sequence "
            f"{previous}), got sequence {expected}"
        )
    if expected > next_expected:
        raise StageMarkerError(
            f"out-of-order marker {name!r}: expected sequence "
            f"{next_expected} (the next marker after sequence "
            f"{previous}), got sequence {expected}"
        )
    return expected


# ---------------------------------------------------------------------------
# JSON event builder.
# ---------------------------------------------------------------------------


def build_event(marker: str, sequence: int, ros_time: float) -> dict:
    """Build the JSON payload published to :data:`STAGE_MARKER_TOPIC`.

    ``ros_time`` MUST be a finite float because it identifies when the
    operator declared the stage in the recorded rosbag evidence.
    """

    if not isinstance(ros_time, (int, float)) or not (
        ros_time == ros_time and ros_time not in (float("inf"), float("-inf"))
    ):
        raise StageMarkerError(
            "ros_time must be a finite float, got " + repr(ros_time)
        )
    if isinstance(ros_time, bool):
        raise StageMarkerError("ros_time must be a finite float, got bool")
    sequence = _coerce_previous(sequence)
    if sequence is None:
        raise StageMarkerError("sequence must be a positive int")
    name = _validate_marker_name(marker)
    if sequence != ALLOWED_MARKERS[name]:
        raise StageMarkerError(
            f"sequence/marker mismatch: marker {name!r} maps to "
            f"sequence {ALLOWED_MARKERS[name]}, got {sequence}"
        )
    return {
        "marker": name,
        "sequence": sequence,
        "stamp": float(ros_time),
        "phase": MARKER_PHASE[name],
    }


# ---------------------------------------------------------------------------
# On-disk state file + cross-process lock.
# ---------------------------------------------------------------------------

#: Per-user cache directory so multiple WSL users on the same host do
#: not share state.
_DEFAULT_STATE_DIR_ENV = "STAGE_MARKER_STATE_DIR"


def _default_state_dir() -> str:
    explicit = os.environ.get(_DEFAULT_STATE_DIR_ENV)
    if explicit:
        return os.path.abspath(explicit)
    cache_root = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser(
        "~/.cache"
    )
    return os.path.join(cache_root, "flying_car_test", "stage_marker")


def _state_path() -> str:
    return os.path.join(_default_state_dir(), "stage_sequence.json")


def _sequence_lock_path() -> str:
    return os.path.join(_default_state_dir(), "stage_sequence.lock")


def _ensure_state_dir() -> None:
    directory = _default_state_dir()
    os.makedirs(directory, exist_ok=True)


def _load_state():
    """Return ``(committed, pending)`` after strict schema validation."""
    path = _state_path()
    if not os.path.exists(path):
        return None, None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            payload = json.load(fh)
    except (json.JSONDecodeError, OSError) as error:
        raise StageMarkerError(
            "cannot read stage-marker state {!r}: {}".format(path, error)
        ) from error
    if not isinstance(payload, Mapping):
        raise StageMarkerError("stage-marker state must be a JSON object")

    # Fix Round 1 used the legacy ``sequence``/``saved_at`` shape.
    # Distinguish it explicitly from the complete current schema:
    # permissive ``dict.get`` fallback would turn a damaged but
    # JSON-valid file into a fresh trial.
    legacy_keys = {"sequence", "saved_at"}
    current_keys = {
        "committed",
        "pending_marker",
        "pending_sequence",
        "pending_stamp",
        "stage_published",
        "phase_published",
    }
    if "sequence" in payload:
        if "committed" in payload or not set(payload).issubset(legacy_keys):
            raise StageMarkerError(
                "invalid mixed legacy/current stage-marker state"
            )
        committed = payload["sequence"]
        if (
            not isinstance(committed, int)
            or isinstance(committed, bool)
            or not 0 <= committed <= MAX_SEQUENCE
        ):
            raise StageMarkerError(
                "invalid legacy sequence in stage-marker state: {!r}".format(
                    committed
                )
            )
        if "saved_at" in payload:
            saved_at = payload["saved_at"]
            if (
                not isinstance(saved_at, (int, float))
                or isinstance(saved_at, bool)
                or not float("-inf") < float(saved_at) < float("inf")
            ):
                raise StageMarkerError(
                    "invalid legacy saved_at in stage-marker state: {!r}".format(
                        saved_at
                    )
                )
    else:
        if set(payload) != current_keys:
            raise StageMarkerError(
                "invalid current stage-marker state schema"
            )
        committed = payload["committed"]
    if committed is not None and not (
        isinstance(committed, int)
        and not isinstance(committed, bool)
        and 0 <= committed <= MAX_SEQUENCE
    ):
        raise StageMarkerError(
            "invalid committed sequence in stage-marker state: {!r}".format(
                committed
            )
        )

    marker = payload.get("pending_marker")
    sequence = payload.get("pending_sequence")
    stamp = payload.get("pending_stamp")
    stage_done = payload.get("stage_published", False)
    phase_done = payload.get("phase_published", False)
    has_pending = any(
        value is not None for value in (marker, sequence, stamp)
    ) or stage_done is not False or phase_done is not False
    if not has_pending:
        return committed, None

    if marker not in ALLOWED_MARKERS:
        raise StageMarkerError("invalid pending marker: {!r}".format(marker))
    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or sequence != ALLOWED_MARKERS[marker]
    ):
        raise StageMarkerError(
            "pending marker/sequence mismatch: {!r}/{!r}".format(
                marker, sequence
            )
        )
    expected = 1 if committed in (None, 0) else committed + 1
    if sequence != expected:
        raise StageMarkerError(
            "pending sequence {!r} does not follow committed {!r}".format(
                sequence, committed
            )
        )
    if (
        not isinstance(stamp, (int, float))
        or isinstance(stamp, bool)
        or not (float("-inf") < float(stamp) < float("inf"))
    ):
        raise StageMarkerError("invalid pending ROS stamp: {!r}".format(stamp))
    if not isinstance(stage_done, bool) or not isinstance(phase_done, bool):
        raise StageMarkerError("pending delivery flags must be booleans")
    if phase_done and not stage_done:
        raise StageMarkerError("phase cannot be delivered before stage")
    return committed, {
        "marker": marker,
        "sequence": sequence,
        "stamp": float(stamp),
        "stage_published": stage_done,
        "phase_published": phase_done,
    }


def _load_sequence() -> Optional[int]:
    return _load_state()[0]


def _load_pending() -> Optional[dict]:
    return _load_state()[1]


def _save_state(
    *,
    committed: Optional[int],
    pending_marker: Optional[str],
    pending_sequence: Optional[int],
    pending_stamp: Optional[float] = None,
    stage_published: bool = False,
    phase_published: bool = False,
) -> None:
    """Atomically write the state file.

    Uses ``NamedTemporaryFile`` + ``fsync`` + ``os.replace`` + a final
    ``fsync`` on the parent directory so a crash never leaves a
    partially-written file on disk.
    """

    payload = {
        "committed": committed,
        "pending_marker": pending_marker,
        "pending_sequence": pending_sequence,
        "pending_stamp": pending_stamp,
        "stage_published": bool(stage_published),
        "phase_published": bool(phase_published),
    }
    _ensure_state_dir()
    path = _state_path()
    directory = os.path.dirname(path)
    fd, tmp_path = tempfile.mkstemp(prefix=".stage_sequence.", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, sort_keys=True))
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_path, path)
        dir_fd = os.open(directory, os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def _save_sequence(sequence: int) -> None:
    _save_state(committed=sequence, pending_marker=None, pending_sequence=None)


def _save_pending(
    marker: str,
    sequence: int,
    *,
    committed: Optional[int],
    stamp: float,
    stage_published: bool,
    phase_published: bool,
) -> None:
    _save_state(
        committed=committed,
        pending_marker=marker,
        pending_sequence=sequence,
        pending_stamp=stamp,
        stage_published=stage_published,
        phase_published=phase_published,
    )


def _commit_pending(marker: str, sequence: int) -> None:
    """Promote the pending marker to committed; clears any older
    pending record.
    """

    _save_state(
        committed=sequence,
        pending_marker=None,
        pending_sequence=None,
    )


def reset_state() -> None:
    """Drop both the committed and pending state.  Used by the
    ``--reset`` CLI flag (and the ``STAGE_MARKER_RESET=1`` env var) to
    restart a trial from marker 1.
    """

    _save_state(
        committed=None, pending_marker=None, pending_sequence=None
    )


class _exclusive_sequence_lock:
    """Context manager that takes an exclusive ``fcntl.flock`` on the
    sidecar lock file.  Each CLI invocation MUST acquire this lock
    around the validate -> save -> publish window so two simultaneous
    callers cannot both advance the sequence.
    """

    def __init__(self) -> None:
        self._fh: Optional[Any] = None

    def __enter__(self) -> "_exclusive_sequence_lock":
        _ensure_state_dir()
        self._fh = open(_sequence_lock_path(), "w", encoding="utf-8")
        fcntl.flock(self._fh.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fh is None:
            return
        try:
            fcntl.flock(self._fh.fileno(), fcntl.LOCK_UN)
        finally:
            self._fh.close()


# ---------------------------------------------------------------------------
# ROS resolution (lazy import so the script can be loaded in
# catkin-build environments without roscore or rospy installed).
# ---------------------------------------------------------------------------


def _resolve_rospy():
    """Return the ``rospy`` module (or whatever the test fixture
    substituted via ``module.rospy``).  Raises :class:`RuntimeError`
    if ``rospy`` cannot be imported.
    """

    rospy = getattr(sys.modules[__name__], "rospy", None)
    if rospy is None:
        try:
            import rospy  # type: ignore[import-not-found]
        except ImportError as error:
            raise RuntimeError(
                "stage_marker.py requires a ROS installation: " + str(error)
            ) from error
        sys.modules[__name__].rospy = rospy
    return rospy


def _resolve_string_class():
    """Return ``std_msgs.msg.String`` (or a test stub substituted via
    ``module.std_msgs``).  The stub path mirrors the test's
    ``monkeypatch.setattr(module, "std_msgs", _Stub)``.
    """

    stub = getattr(sys.modules[__name__], "std_msgs", None)
    if stub is not None:
        return stub.msg.String
    rospy = _resolve_rospy()
    import std_msgs.msg as _std_msgs_msg  # type: ignore[import-not-found]
    return _std_msgs_msg.String


# ---------------------------------------------------------------------------
# Publisher lifecycle.
# ---------------------------------------------------------------------------


def _make_publisher(ros, topic, string_cls, *, latch):
    """Wrap ``ros.Publisher`` so a missing subclass can be injected in
    tests.
    """

    return ros.Publisher(topic, string_cls, latch=latch, queue_size=1)


def _wait_for_subscriber(ros, publisher, timeout_s: float) -> None:
    """Block until ``publisher`` has at least one subscriber or the
    timeout expires.  Raises :class:`SubscriberUnavailable` on
    timeout.

    ``publisher.get_num_connections()`` must return a positive count.
    Waiting for an incoming message would test the opposite direction
    and therefore is deliberately not used as a fallback.
    """

    timeout_s = max(0.0, float(timeout_s))
    get_num = getattr(publisher, "get_num_connections", None)
    deadline = time.monotonic() + timeout_s

    if get_num is None:
        raise SubscriberUnavailable(
            "publisher does not expose get_num_connections(); "
            "cannot verify evidence subscriber"
        )

    while True:
        try:
            count = int(get_num())
        except Exception:
            count = 0
        if count > 0:
            return
        if time.monotonic() >= deadline:
            raise SubscriberUnavailable(
                f"no subscriber attached to publisher within "
                f"{timeout_s:.2f}s (get_num_connections={count})"
            )
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))


# ---------------------------------------------------------------------------
# Publish path.
# ---------------------------------------------------------------------------


def _publish_marker(marker: str, ros=None, string_cls=None) -> int:
    """Publish a single stage marker.

    Returns the 1-based sequence number on success.  Raises
    :class:`StageMarkerError` if validation fails (and nothing is
    persisted or published); raises :class:`SubscriberUnavailable`
    if no subscriber attaches within :data:`SUBSCRIBER_WAIT_TIMEOUT_S`;
    on a publish failure, records the pending state and re-raises
    the underlying error so the CLI exits non-zero and the operator
    can re-invoke the same marker to retry.
    """

    if ros is None:
        ros = _resolve_rospy()
    if string_cls is None:
        string_cls = _resolve_string_class()

    # Anonymous True ensures concurrent CLI invocations get unique
    # node names so the roscore does not shutdown the older node.
    ros.init_node("flying_car_stage_marker", anonymous=True)

    # Note: do NOT delete_param here.  The on-disk state file is the
    # canonical truth, but the ROS parameter is also written so a
    # subscriber that joins during the lifetime of one CLI invocation
    # can read the latest committed sequence.  Deleting would race
    # with that subscriber's read.

    with _exclusive_sequence_lock():
        previous, pending = _load_state()

        if pending is not None and pending["marker"] != marker:
            raise StageMarkerError(
                "pending marker {!r} must be completed before {!r}".format(
                    pending["marker"], marker
                )
            )

        # Pending-state retry resumes only the topic deliveries that were
        # not durably recorded as completed.
        if pending is not None:
            sequence = pending["sequence"]
            stage_published = pending["stage_published"]
            phase_published = pending["phase_published"]
            stamp = pending["stamp"]
        else:
            sequence = validate_marker(marker, previous_sequence=previous)
            stage_published = False
            phase_published = False
            stamp = None

        # A retry reuses the original event timestamp, keeping repeated
        # at-least-once transport attempts byte-identical for downstream
        # sequence/stamp de-duplication.
        if stamp is None:
            stamp = ros.Time.now().to_sec()
        event = build_event(marker=marker, sequence=sequence, ros_time=stamp)
        payload = json.dumps(event, sort_keys=True)
        phase_payload = MARKER_PHASE[marker]

        stage_publisher = ros.Publisher(
            STAGE_MARKER_TOPIC, string_cls, latch=True, queue_size=1
        )
        phase_publisher = ros.Publisher(
            PHASE_TOPIC, string_cls, latch=True, queue_size=1
        )

        # Each missing delivery must have a live consumer before the
        # write-ahead record is created.
        if not stage_published:
            _wait_for_subscriber(ros, stage_publisher, SUBSCRIBER_WAIT_TIMEOUT_S)
        if not phase_published:
            _wait_for_subscriber(ros, phase_publisher, SUBSCRIBER_WAIT_TIMEOUT_S)

        # Durable write-ahead intent comes before either transport call.
        # BaseException-level process interruptions therefore leave a
        # retryable pending marker instead of a false committed sequence.
        _save_pending(
            marker,
            sequence,
            committed=previous,
            stamp=stamp,
            stage_published=stage_published,
            phase_published=phase_published,
        )
        ros.set_param(
            PARAMETER_NAME,
            {
                "committed": int(previous) if previous is not None else 0,
                "pending_marker": marker,
                "pending_sequence": sequence,
            },
        )

        if not stage_published:
            stage_publisher.publish(string_cls(data=payload))
            # ``publish`` only enqueues transport data.  Do not mark this
            # side delivered until the queue has had its flush window.
            ros.sleep(POST_PUBLISH_FLUSH_S)
            stage_published = True
            _save_pending(
                marker,
                sequence,
                committed=previous,
                stamp=stamp,
                stage_published=True,
                phase_published=phase_published,
            )

        if not phase_published:
            phase_publisher.publish(string_cls(data=phase_payload))
            ros.sleep(POST_PUBLISH_FLUSH_S)
            phase_published = True
            _save_pending(
                marker,
                sequence,
                committed=previous,
                stamp=stamp,
                stage_published=stage_published,
                phase_published=True,
            )

        _commit_pending(marker=marker, sequence=sequence)
        try:
            ros.set_param(PARAMETER_NAME, sequence)
        except Exception:
            # Disk is the canonical commit.  Remove the now-stale pending
            # mirror rather than reporting a false failed marker after both
            # evidence deliveries completed.
            try:
                ros.delete_param(PARAMETER_NAME)
            except Exception:
                pass

    return sequence


# ---------------------------------------------------------------------------
# CLI parser and entry point.
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="stage_marker.py",
        description=(
            "Publish a validated flying-car experiment stage marker. Each "
            "invocation advances the persistent sequence by one; "
            "out-of-order, duplicate, or unknown markers are rejected with "
            "a non-zero exit status and no published message."
        ),
    )
    parser.add_argument(
        "marker",
        nargs="?",
        choices=list(ALLOWED_MARKERS),
        help=(
            "Stage marker to publish. Must be the next one in the canonical "
            "sequence; previous sequence is read from the on-disk state file "
            "(the ROS parameter is advisory only)."
        ),
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help=(
            "Drop the persisted and pending state so a fresh trial can "
            "start from marker 1.  Equivalent to STAGE_MARKER_RESET=1 in "
            "the environment."
        ),
    )
    return parser


def main(argv=None) -> int:
    parser = _build_parser()
    if argv is None:
        argv = sys.argv[1:]

    env_reset = os.environ.get("STAGE_MARKER_RESET", "").lower() in (
        "1", "true", "yes"
    )

    if not argv and not env_reset:
        parser.print_help()
        return 2

    try:
        args = parser.parse_args(argv)
    except SystemExit as exit_event:
        # argparse prints usage + error on bad input and exits with
        # code 2; we mirror that exit status.
        code = exit_event.code
        return 2 if code is None else int(code)

    reset_requested = bool(args.reset or env_reset)
    if reset_requested and args.marker is not None:
        print(
            "stage_marker.py: reset cannot be combined with a marker",
            flush=True,
        )
        return 2

    if reset_requested:
        try:
            ros = _resolve_rospy()
            ros.init_node("flying_car_stage_marker_reset", anonymous=True)
            with _exclusive_sequence_lock():
                try:
                    ros.delete_param(PARAMETER_NAME)
                except KeyError:
                    pass
                reset_state()
        except Exception as error:
            print(f"stage_marker.py: reset failed: {error}", flush=True)
            return 4
        print(
            "stage_marker.py: persisted state cleared; the next "
            "marker call will start from sequence 1",
            flush=True,
        )
        return 0

    if args.marker is None:
        parser.print_help()
        return 2

    try:
        ros = _resolve_rospy()
    except RuntimeError as error:
        print(str(error), flush=True)
        return 2

    string_cls = _resolve_string_class()

    try:
        sequence = _publish_marker(
            args.marker, ros=ros, string_cls=string_cls
        )
    except StageMarkerError as error:
        print(f"stage_marker.py: {error}", flush=True)
        return 1
    except SubscriberUnavailable as error:
        print(f"stage_marker.py: {error}", flush=True)
        return 3
    except Exception as error:
        print(f"stage_marker.py: publish failed: {error}", flush=True)
        return 4

    print(
        f"published stage marker {args.marker!r} "
        f"(sequence {sequence}) to {STAGE_MARKER_TOPIC}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
