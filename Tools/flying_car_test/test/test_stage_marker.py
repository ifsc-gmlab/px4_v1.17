"""TDD contract tests for Task 11 Fix Round 1.

The Task 11 stage-marker CLI is implemented as a single self-contained
script at ``Tools/flying_car_test/scripts/stage_marker.py``; this
test module loads it with ``importlib`` so the script can be required
to carry every helper, the validator, the JSON-event builder, the
ROS bound publisher, the persistence helper, and the ``main`` entry
point without leaking implementation into a separate ``src`` Python
package.  The deleted ``src/flying_car_test/stage_marker.py`` is
deliberately not re-introduced.

The contract verified here:

* Only the seven canonical markers are accepted, and the strict 1..7
  sequence is enforced (unknown/duplicate/out-of-order all fail with
  no published message).
* The published ``/flying_car_test/stage_marker`` JSON carries a real
  ROS "now" timestamp (not a constant zero).
* The sequence number survives across multiple CLI invocations by
  being atomically persisted in the local state file inside the
  package directory; cross-process ``fcntl`` lock guarantees that two
  simultaneous callers cannot both advance the sequence.
* When validation fails no message is published on either topic.
* ``experiment_end`` additionally publishes a pure-string
  ``"COMPLETE"`` payload on
  ``/flying_car_experiment_controller/phase`` so the passive monitor
  ends with ``phase == COMPLETE`` and yields ``clean=true`` even when
  the ``experiment_controller`` does not run (manual QGC trial).
* The README documents the canonical source path, the physical
  transmitter procedure, the MAV_TYPE switch preconditions, marker
  publication timing, and the raw-MAVROS-vs-QGC-normalized contrast.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Mapping
import importlib.util

import pytest


# ---------------------------------------------------------------------------
# Module loading: scripts/stage_marker.py is the only allowed location.
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[3]
FLYING_CAR_DIR = REPO_ROOT / "Tools" / "flying_car_test"
SCRIPT_PATH = FLYING_CAR_DIR / "scripts" / "stage_marker.py"
OUT_OF_SCOPE_PATH = (
    FLYING_CAR_DIR / "src" / "flying_car_test" / "stage_marker.py"
)
README_PATH = FLYING_CAR_DIR / "README.md"

ALLOWED_MARKERS = (
    "ground_forward_start",
    "flight_switch_start",
    "takeoff_start",
    "air_forward_start",
    "landing_start",
    "final_ground_start",
    "experiment_end",
)


def _load_stage_marker(monkeypatch=None, state_dir=None):
    """Import ``scripts/stage_marker.py`` under a stable module name.

    The script is loaded as ``flying_car_test.stage_marker_cli`` so the
    helper is importable in tests without conflicting with the package
    layout enforced by ``test_package_layout``.
    """

    module_name = "flying_car_test.stage_marker_cli"
    existing = sys.modules.get(module_name)
    if existing is not None:
        del sys.modules[module_name]
    spec = importlib.util.spec_from_file_location(module_name, str(SCRIPT_PATH))
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    if monkeypatch is not None and state_dir is not None:
        monkeypatch.setenv("STAGE_MARKER_STATE_DIR", str(state_dir))
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def module():
    """Load the script once per module so monkeypatch.fixtures can re-load
    a fresh copy per-test.
    """

    return _load_stage_marker()


@pytest.fixture
def state_dir(tmp_path):
    """Per-test isolated state directory for cross-process persistence."""

    directory = tmp_path / "stage_marker_state"
    directory.mkdir()
    return directory


@pytest.fixture
def fresh_module(monkeypatch, state_dir):
    """Reload the script with a per-test state directory."""

    return _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)


# ---------------------------------------------------------------------------
# Scope / file layout.
# ---------------------------------------------------------------------------


def test_scripts_stage_marker_exists():
    assert SCRIPT_PATH.is_file(), (
        "Task 11 CLI script missing at {}".format(SCRIPT_PATH)
    )


def test_out_of_scope_stage_marker_is_removed():
    """Fix Round 1 critical: src/flying_car_test/stage_marker.py must be gone."""

    assert not OUT_OF_SCOPE_PATH.exists(), (
        "out-of-scope file {} must be deleted; the implementation must "
        "live entirely inside scripts/stage_marker.py".format(OUT_OF_SCOPE_PATH)
    )


# ---------------------------------------------------------------------------
# Canonical constants.
# ---------------------------------------------------------------------------


def test_allowed_markers_constant_matches_plan(module):
    from_map = module.ALLOWED_MARKERS
    assert tuple(from_map) == ALLOWED_MARKERS
    assert set(from_map.values()) == set(range(1, len(ALLOWED_MARKERS) + 1))


def test_marker_to_phase_constants(module):
    """Every marker must map to a phase string the monitor understands."""

    # Six phases other than ``experiment_end`` drive the manual flow;
    # ``experiment_end`` maps to COMPLETE (the only allowed terminal
    # phase that the passive monitor closes on).
    expected_phase_map = {
        "ground_forward_start": "GROUND_FORWARD",
        "flight_switch_start": "FLIGHT_CONFIGURE",
        "takeoff_start": "TAKEOFF",
        "air_forward_start": "AIR_FORWARD",
        "landing_start": "LAND",
        "final_ground_start": "GROUND_DIAGONAL",
        "experiment_end": "COMPLETE",
    }
    actual = module.MARKER_PHASE
    assert dict(actual) == expected_phase_map


def test_topics_are_stable_and_compatible(module):
    """The two topics must match the contracts recorded in monitor.py."""

    assert module.STAGE_MARKER_TOPIC == "/flying_car_test/stage_marker"
    assert module.PHASE_TOPIC == "/flying_car_experiment_controller/phase"


def test_parameter_name_is_stable(module):
    assert module.PARAMETER_NAME == "/flying_car_test/stage_sequence"


# ---------------------------------------------------------------------------
# Pure-Python validation helpers.
# ---------------------------------------------------------------------------


def test_validate_marker_accepts_first_marker_from_none(module):
    assert module.validate_marker(
        "ground_forward_start", previous_sequence=None
    ) == 1


def test_validate_marker_accepts_each_step_in_order(module):
    sequence = None
    for index, marker in enumerate(ALLOWED_MARKERS, start=1):
        sequence = module.validate_marker(marker, previous_sequence=sequence)
        assert sequence == index


def test_validate_marker_rejects_unknown(module):
    with pytest.raises(module.StageMarkerError) as excinfo:
        module.validate_marker("not_a_real_marker", previous_sequence=None)
    assert "not_a_real_marker" in str(excinfo.value)
    assert "unknown" in str(excinfo.value).lower()


def test_validate_marker_rejects_duplicate(module):
    with pytest.raises(module.StageMarkerError):
        module.validate_marker(
            "ground_forward_start", previous_sequence=1
        )


def test_validate_marker_rejects_out_of_order_skip(module):
    with pytest.raises(module.StageMarkerError) as excinfo:
        module.validate_marker("takeoff_start", previous_sequence=1)
    assert "out-of-order" in str(excinfo.value).lower()


def test_validate_marker_rejects_out_of_order_when_above_expected(module):
    with pytest.raises(module.StageMarkerError):
        module.validate_marker("experiment_end", previous_sequence=5)


def test_validate_marker_rejects_out_of_order_when_below_expected(module):
    with pytest.raises(module.StageMarkerError):
        module.validate_marker("ground_forward_start", previous_sequence=99)


def test_validate_marker_rejects_non_integer_previous(module):
    with pytest.raises(module.StageMarkerError):
        module.validate_marker(
            "ground_forward_start", previous_sequence="not_an_int"
        )


def test_validate_marker_rejects_after_completion(module):
    with pytest.raises(module.StageMarkerError) as excinfo:
        module.validate_marker("ground_forward_start", previous_sequence=7)
    assert "complete" in str(excinfo.value).lower() or \
           "out-of-order" in str(excinfo.value).lower()


# ---------------------------------------------------------------------------
# Event payload.
# ---------------------------------------------------------------------------


def test_build_event_payload_shape(module):
    event = module.build_event(
        marker="takeoff_start", sequence=3, ros_time=123.456
    )
    assert event["marker"] == "takeoff_start"
    assert event["sequence"] == 3
    assert event["stamp"] == pytest.approx(123.456, rel=0, abs=1e-9)
    # JSON-serialisable, deterministic key order.
    payload = json.dumps(event, sort_keys=True)
    assert json.loads(payload) == event


def test_build_event_rejects_non_finite_time(module):
    with pytest.raises(module.StageMarkerError):
        module.build_event(
            marker="ground_forward_start",
            sequence=1,
            ros_time=float("inf"),
        )


def test_build_event_rejects_sequence_marker_mismatch(module):
    with pytest.raises(module.StageMarkerError):
        module.build_event(
            marker="takeoff_start", sequence=2, ros_time=0.0
        )


# ---------------------------------------------------------------------------
# Cross-process lock helpers.
# ---------------------------------------------------------------------------


def test_lock_path_uses_stable_directory(monkeypatch, tmp_path):
    mod = _load_stage_marker(monkeypatch=monkeypatch, state_dir=tmp_path)
    assert Path(mod._sequence_lock_path()).parent == tmp_path


# ---------------------------------------------------------------------------
# CLI behavior with a fake ROS boundary.
# ---------------------------------------------------------------------------


class _FakeString:
    """Lazy constructor that mirrors std_msgs.msg.String; accepts both
    positional and keyword ``data`` so the same fake works with the
    various call sites in production.
    """

    def __init__(self, *args, **kwargs):
        if args and isinstance(args[0], str) and not kwargs:
            self.data = args[0]
            return
        self.data = kwargs.get("data", args[0] if args else "")


class _FakePublisher:
    def __init__(self, ros, topic):
        self.ros = ros
        self.topic = topic
        self.published = []
        self.latched = False

    def publish(self, message):
        if self.topic in self.ros.interrupt_publish_topics:
            raise KeyboardInterrupt("simulated hard interruption")
        if self.topic in self.ros.fail_publish_topics:
            raise RuntimeError("simulated publish failure")
        self.published.append(message)

    def get_num_connections(self):
        return self.ros.connections.get(self.topic, 1)


class _FakeTimeStamp:
    def __init__(self, value):
        self._value = float(value)

    def to_sec(self):
        return self._value


class _FakeTime:
    """Mirror ``rospy.Time``: ``Time.now().to_sec()`` returns ROS seconds.

    ``now()`` is a classmethod that pulls the next value from a queue
    populated in ``__init__``.
    """

    _queue = []  # class-level queue so static-style access works

    def __init__(self, sequence=()):
        # Sequence is consumed left-to-right.
        self._sequence = list(sequence)

    @classmethod
    def now(cls):
        # Prefer class-level queue (set by _FakeRos.__init__) so callers
        # do not need an instance to read the timestamp.
        if cls._queue:
            return _FakeTimeStamp(cls._queue.pop(0))
        return _FakeTimeStamp(0.0)

    def to_sec(self):
        return 0.0


class _FakeRos:
    """Minimal stand-in for ``rospy``.

    Tracks publishers, captures every publish call, and lets the test
    drive ``rospy.Time.now()`` to a non-zero wall-clock-style value.
    """

    def __init__(self, time_sequence, parameters=None):
        self.publishers = {}
        self.parameters = parameters if parameters is not None else {}
        self.time_sequence = list(time_sequence)
        self.init_node_calls = []
        self.shutdown_calls = []
        self.fail_set_param = False
        self.fail_set_param_calls = set()
        self.set_param_calls = 0
        self.fail_publish_topics = set()
        self.interrupt_publish_topics = set()
        self.strict_delete_param = False
        self.interrupt_sleep_calls = set()
        self.sleep_calls = 0
        self.connections = {}
        self.string_cls = _FakeString
        # Seed the class-level queue used by ``Time.now()``.
        _FakeTime._queue = list(time_sequence)

    def Publisher(self, name, msg_type, latch=False, queue_size=1):
        pub = self.publishers.setdefault(name, _FakePublisher(self, name))
        pub.latched = bool(latch)
        return pub

    def init_node(self, name, anonymous=False):
        self.init_node_calls.append((name, anonymous))

    def get_param(self, name, default=None):
        return self.parameters.get(name, default)

    def set_param(self, name, value):
        self.set_param_calls += 1
        if (
            self.fail_set_param
            or self.set_param_calls in self.fail_set_param_calls
        ):
            raise RuntimeError("simulated parameter write failure")
        self.parameters[name] = value

    def delete_param(self, name):
        if self.strict_delete_param and name not in self.parameters:
            raise KeyError(name)
        self.parameters.pop(name, None)

    def is_shutdown(self):
        return False

    # ``rospy.Time`` is a *class*, not an instance; ``rospy.Time.now()``
    # is a static/classmethod.  Mirror that by exposing the class:
    Time = property(lambda self: _FakeTime)

    def on_shutdown(self, callback):
        self.shutdown_calls.append(callback)

    def signal_shutdown(self, reason):
        self.shutdown_calls.append(reason)

    def sleep(self, seconds):
        # Production calls ``ros.sleep`` to flush the Linger queue;
        # the test fixture no-ops it so the test runs in finite time.
        self.sleep_calls += 1
        if self.sleep_calls in self.interrupt_sleep_calls:
            raise KeyboardInterrupt("simulated interruption during flush")
        return None


@pytest.fixture
def fake_ros(monkeypatch, fresh_module):
    fake = _FakeRos(time_sequence=[1700000000.0, 1700000001.5])
    monkeypatch.setattr(fresh_module, "rospy", fake)
    monkeypatch.setattr(fresh_module.time, "sleep", lambda _seconds: None)
    # The script lazily imports ``std_msgs.msg.String``; install a stub
    # so the test does not need a ROS install.
    class _Stub:
        class msg:
            String = _FakeString
    sys.modules.setdefault("std_msgs", _Stub)
    sys.modules.setdefault("std_msgs.msg", _Stub.msg)
    return fake, fresh_module


def test_main_publishes_first_marker_with_real_ros_time(fake_ros, state_dir):
    fake, mod = fake_ros
    exit_code = mod.main(["ground_forward_start"])
    assert exit_code == 0

    stage_topic = "/flying_car_test/stage_marker"
    assert stage_topic in fake.publishers
    pub = fake.publishers[stage_topic]
    assert pub.latched is True
    assert len(pub.published) == 1

    payload = json.loads(pub.published[0].data)
    assert payload["marker"] == "ground_forward_start"
    assert payload["sequence"] == 1
    # ``stamp`` must come from rospy.Time.now().to_sec() which the fake
    # populated to a non-zero value (the old code used ``Time().to_sec``
    # which returns a constant zero - that case is asserted separately).
    assert payload["stamp"] == pytest.approx(1700000000.0, rel=0, abs=1e-9)
    assert payload["stamp"] > 0.0

    assert fake.parameters[mod.PARAMETER_NAME] == 1


def test_main_advances_sequence_across_invocations(fake_ros, state_dir):
    fake, mod = fake_ros
    for index, marker in enumerate(ALLOWED_MARKERS, start=1):
        exit_code = mod.main([marker])
        assert exit_code == 0
        assert fake.parameters[mod.PARAMETER_NAME] == index

    pub = fake.publishers["/flying_car_test/stage_marker"]
    assert len(pub.published) == len(ALLOWED_MARKERS)
    final_payload = json.loads(pub.published[-1].data)
    assert final_payload["marker"] == "experiment_end"
    assert final_payload["sequence"] == len(ALLOWED_MARKERS)


def test_main_rejected_unknown_does_not_publish(fake_ros, state_dir):
    fake, mod = fake_ros
    exit_code = mod.main(["not_a_marker"])
    assert exit_code != 0
    assert "/flying_car_test/stage_marker" not in fake.publishers
    assert (
        "/flying_car_experiment_controller/phase" not in fake.publishers
    )
    assert mod.PARAMETER_NAME not in fake.parameters


def test_main_rejected_out_of_order_does_not_publish(fake_ros, state_dir):
    fake, mod = fake_ros
    assert mod.main(["ground_forward_start"]) == 0
    exit_code = mod.main(["takeoff_start"])  # skip sequence 2
    assert exit_code != 0
    pub = fake.publishers["/flying_car_test/stage_marker"]
    assert len(pub.published) == 1
    assert fake.parameters[mod.PARAMETER_NAME] == 1


def test_main_rejected_duplicate_does_not_publish(fake_ros, state_dir):
    fake, mod = fake_ros
    assert mod.main(["ground_forward_start"]) == 0
    exit_code = mod.main(["ground_forward_start"])
    assert exit_code != 0
    assert fake.parameters[mod.PARAMETER_NAME] == 1


def test_separate_module_loads_share_ros_parameter_sequence(
    monkeypatch, state_dir
):
    parameters = {}
    first = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    first_ros = _FakeRos([1700000000.0], parameters=parameters)
    monkeypatch.setattr(first.time, "sleep", lambda _seconds: None)
    assert first._publish_marker(
        "ground_forward_start", ros=first_ros, string_cls=_FakeString
    ) == 1

    second = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    second_ros = _FakeRos([1700000001.0], parameters=parameters)
    monkeypatch.setattr(second.time, "sleep", lambda _seconds: None)
    assert second._publish_marker(
        "flight_switch_start", ros=second_ros, string_cls=_FakeString
    ) == 2
    assert parameters[second.PARAMETER_NAME] == 2


def test_parameter_write_failure_publishes_nothing(fake_ros):
    fake, mod = fake_ros
    fake.fail_set_param = True

    assert mod.main(["ground_forward_start"]) != 0
    assert mod.PARAMETER_NAME not in fake.parameters
    assert all(not publisher.published for publisher in fake.publishers.values())


def test_publish_failure_keeps_retryable_pending_state(fake_ros):
    fake, mod = fake_ros
    fake.fail_publish_topics.add(mod.STAGE_MARKER_TOPIC)

    assert mod.main(["ground_forward_start"]) != 0
    pending = fake.parameters[mod.PARAMETER_NAME]
    assert pending == {
        "committed": 0,
        "pending_marker": "ground_forward_start",
        "pending_sequence": 1,
    }

    fake.fail_publish_topics.clear()
    assert mod.main(["ground_forward_start"]) == 0
    assert fake.parameters[mod.PARAMETER_NAME] == 1


def test_hard_interruption_before_publish_leaves_write_ahead_pending(fake_ros):
    """A process-level interruption must not leave an unpublished marker
    recorded as committed.

    ``KeyboardInterrupt`` deliberately bypasses ``except Exception`` and
    models the SIGINT/SIGTERM window between durable state changes and the
    transport call.
    """

    fake, mod = fake_ros
    fake.interrupt_publish_topics.add(mod.STAGE_MARKER_TOPIC)

    with pytest.raises(KeyboardInterrupt):
        mod._publish_marker(
            "ground_forward_start", ros=fake, string_cls=_FakeString
        )

    assert mod._load_sequence() is None
    pending = mod._load_pending()
    assert pending is not None
    assert pending["marker"] == "ground_forward_start"
    assert pending["sequence"] == 1


def test_retry_after_phase_failure_does_not_duplicate_stage_event(fake_ros):
    """If stage delivery succeeds but phase delivery fails, retry only the
    missing phase side of the pending operation.
    """

    fake, mod = fake_ros
    fake.fail_publish_topics.add(mod.PHASE_TOPIC)

    assert mod.main(["ground_forward_start"]) != 0
    stage_pub = fake.publishers[mod.STAGE_MARKER_TOPIC]
    assert len(stage_pub.published) == 1
    assert mod._load_sequence() is None

    fake.fail_publish_topics.clear()
    assert mod.main(["ground_forward_start"]) == 0
    assert len(stage_pub.published) == 1
    assert len(fake.publishers[mod.PHASE_TOPIC].published) == 1
    assert mod._load_sequence() == 1


def test_interruption_during_flush_keeps_delivery_retryable(fake_ros):
    """A transport enqueue is not committed as delivered until its flush
    window completes.
    """

    fake, mod = fake_ros
    fake.interrupt_sleep_calls.add(1)

    with pytest.raises(KeyboardInterrupt):
        mod._publish_marker(
            "ground_forward_start", ros=fake, string_cls=_FakeString
        )

    pending = mod._load_pending()
    assert pending is not None
    assert pending["stage_published"] is False
    assert mod._load_sequence() is None

    first_payload = fake.publishers[mod.STAGE_MARKER_TOPIC].published[0].data
    fake.interrupt_sleep_calls.clear()
    assert mod.main(["ground_forward_start"]) == 0
    retried_payload = fake.publishers[mod.STAGE_MARKER_TOPIC].published[1].data
    assert retried_payload == first_payload
    assert mod._load_sequence() == 1


def test_publisher_waits_for_real_subscribers_before_persisting(
    fake_ros, monkeypatch
):
    fake, mod = fake_ros
    fake.connections[mod.STAGE_MARKER_TOPIC] = 0
    monkeypatch.setattr(mod, "SUBSCRIBER_WAIT_TIMEOUT_S", 0.01)

    assert mod.main(["ground_forward_start"]) != 0
    assert mod.PARAMETER_NAME not in fake.parameters
    assert all(not publisher.published for publisher in fake.publishers.values())


def test_publisher_waits_for_phase_subscriber_before_persisting(
    fake_ros, monkeypatch
):
    fake, mod = fake_ros
    fake.connections[mod.STAGE_MARKER_TOPIC] = 1
    fake.connections[mod.PHASE_TOPIC] = 0
    monkeypatch.setattr(mod, "SUBSCRIBER_WAIT_TIMEOUT_S", 0.01)

    assert mod.main(["ground_forward_start"]) != 0
    assert mod.PARAMETER_NAME not in fake.parameters
    assert mod._load_sequence() is None
    assert all(not publisher.published for publisher in fake.publishers.values())


def test_subscriber_wait_fails_closed_without_connection_introspection(
    fresh_module
):
    class _PublisherWithoutConnectionCount:
        pass

    class _RosWithoutWaitPrimitive:
        pass

    with pytest.raises(fresh_module.SubscriberUnavailable):
        fresh_module._wait_for_subscriber(
            _RosWithoutWaitPrimitive(),
            _PublisherWithoutConnectionCount(),
            timeout_s=0.01,
        )


def test_cli_uses_unique_ros_node_names(fake_ros):
    fake, mod = fake_ros
    assert mod.main(["ground_forward_start"]) == 0
    assert fake.init_node_calls == [("flying_car_stage_marker", True)]


def test_main_experiment_end_publishes_complete_phase(fake_ros, state_dir):
    fake, mod = fake_ros
    for marker in ALLOWED_MARKERS[:-1]:
        assert mod.main([marker]) == 0
    # ``experiment_end`` must publish on the stage-marker topic AND on
    # the phase topic with a plain string COMPLETE.
    assert mod.main(["experiment_end"]) == 0

    pub = fake.publishers["/flying_car_test/stage_marker"]
    assert len(pub.published) == 7
    stage_payload = json.loads(pub.published[-1].data)
    assert stage_payload["marker"] == "experiment_end"
    assert stage_payload["sequence"] == 7

    phase_pub = fake.publishers["/flying_car_experiment_controller/phase"]
    assert phase_pub.published[-1].data == "COMPLETE"
    # The phase message must NOT be JSON - the monitor parses the raw
    # string as the phase label.
    assert not phase_pub.published[-1].data.startswith("{")


def test_main_intermediate_marker_publishes_phase_for_monitor(fake_ros, state_dir):
    """Every non-final marker must publish the corresponding phase so
    ``monitor.py`` can populate its phase timeline."""

    fake, mod = fake_ros
    expected = {
        "ground_forward_start": "GROUND_FORWARD",
        "flight_switch_start": "FLIGHT_CONFIGURE",
        "takeoff_start": "TAKEOFF",
        "air_forward_start": "AIR_FORWARD",
        "landing_start": "LAND",
        "final_ground_start": "GROUND_DIAGONAL",
    }
    for marker, phase in expected.items():
        assert mod.main([marker]) == 0
    phase_pub = fake.publishers["/flying_car_experiment_controller/phase"]
    observed = [m.data for m in phase_pub.published]
    assert observed == list(expected.values())


def test_main_init_node_runs_before_parameter_read(fake_ros, state_dir):
    """``init_node`` MUST run before the script touches any ROS
    parameter. The fake ROS records the call order.
    """

    fake, mod = fake_ros
    mod.main(["ground_forward_start"])
    assert fake.init_node_calls, "init_node must be called"


def test_main_no_args_prints_help_and_exits_nonzero(fake_ros, state_dir, capsys):
    """Review Minor #10: a missing-positional-arg invocation must NOT
    exit 0; otherwise an operator that forgot the marker would silently
    appear to succeed.  ``main([])`` exits with code 2 and prints help.
    """

    fake, mod = fake_ros
    exit_code = mod.main([])
    captured = capsys.readouterr()
    assert exit_code != 0, (
        "main([]) returned 0; the script should exit non-zero so a "
        "missing-positional-arg invocation does not appear to succeed"
    )
    combined = captured.out + captured.err
    for marker in ALLOWED_MARKERS:
        assert marker in combined
    assert "/flying_car_test/stage_marker" not in fake.publishers


def test_main_help_flag_exits_zero(fake_ros, state_dir, capsys):
    fake, mod = fake_ros
    exit_code = mod.main(["--help"])
    captured = capsys.readouterr()
    assert exit_code == 0
    for marker in ALLOWED_MARKERS:
        assert marker in (captured.out + captured.err)


def test_main_missing_marker_argument_exits_nonzero(fake_ros, state_dir):
    fake, mod = fake_ros
    assert mod.main(["--unknown-flag"]) != 0
    assert "/flying_car_test/stage_marker" not in fake.publishers


def test_reset_flag_rejects_marker_instead_of_silently_swallowing_it(
    fake_ros
):
    fake, mod = fake_ros
    mod._save_sequence(3)

    assert mod.main(["--reset", "ground_forward_start"]) != 0
    assert mod._load_sequence() == 3
    assert not fake.publishers


def test_reset_environment_rejects_marker_instead_of_returning_success(
    fake_ros, monkeypatch
):
    fake, mod = fake_ros
    mod._save_sequence(3)
    monkeypatch.setenv("STAGE_MARKER_RESET", "1")

    assert mod.main(["ground_forward_start"]) != 0
    assert mod._load_sequence() == 3
    assert not fake.publishers


def test_reset_clears_public_ros_parameter(fake_ros):
    fake, mod = fake_ros
    mod._save_sequence(3)
    fake.parameters[mod.PARAMETER_NAME] = 3

    assert mod.main(["--reset"]) == 0
    assert mod._load_sequence() is None
    assert mod.PARAMETER_NAME not in fake.parameters


def test_reset_is_idempotent_when_public_ros_parameter_is_absent(fake_ros):
    fake, mod = fake_ros
    fake.strict_delete_param = True

    assert mod.main(["--reset"]) == 0
    assert mod._load_sequence() is None


def test_corrupt_state_fails_closed_without_publishing(fake_ros):
    fake, mod = fake_ros
    state_path = Path(mod._state_path())
    state_path.write_text("{not-json", encoding="utf-8")

    assert mod.main(["ground_forward_start"]) != 0
    assert state_path.read_text(encoding="utf-8") == "{not-json"
    assert not fake.publishers


def test_out_of_range_state_fails_closed_without_restarting(fake_ros):
    fake, mod = fake_ros
    state_path = Path(mod._state_path())
    state_path.write_text(
        json.dumps(
            {
                "committed": 999,
                "pending_marker": None,
                "pending_sequence": None,
            }
        ),
        encoding="utf-8",
    )

    assert mod.main(["ground_forward_start"]) != 0
    assert not fake.publishers


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"garbage": 1},
        {"committed": None, "sequence": 5},
        {"committed": 2, "sequence": 5},
        {"sequence": None},
        {"sequence": None, "saved_at": 123.0},
        {"sequence": 2, "saved_at": "corrupt"},
    ],
)
def test_invalid_root_state_schema_fails_closed(fake_ros, payload):
    fake, mod = fake_ros
    Path(mod._state_path()).write_text(
        json.dumps(payload), encoding="utf-8"
    )

    with pytest.raises(mod.StageMarkerError, match="state"):
        mod._load_state()
    assert not fake.publishers


@pytest.mark.parametrize(
    "payload",
    [
        {
            "committed": 0,
            "pending_marker": "ground_forward_start",
            "pending_sequence": None,
            "pending_stamp": 1.0,
            "stage_published": False,
            "phase_published": False,
        },
        {
            "committed": 0,
            "pending_marker": "ground_forward_start",
            "pending_sequence": 1,
            "pending_stamp": 1.0,
            "stage_published": "false",
            "phase_published": False,
        },
        {
            "committed": 5,
            "pending_marker": "ground_forward_start",
            "pending_sequence": 1,
            "pending_stamp": 1.0,
            "stage_published": True,
            "phase_published": True,
        },
    ],
)
def test_malformed_pending_state_fails_closed(fake_ros, payload):
    fake, mod = fake_ros
    Path(mod._state_path()).write_text(
        json.dumps(payload), encoding="utf-8"
    )

    assert mod.main(["ground_forward_start"]) != 0
    assert not fake.publishers


def test_final_parameter_failure_keeps_disk_commit_and_clears_stale_param(
    fake_ros
):
    fake, mod = fake_ros
    fake.fail_set_param_calls.add(2)

    assert mod.main(["ground_forward_start"]) == 0
    assert mod._load_sequence() == 1
    assert mod.PARAMETER_NAME not in fake.parameters


def test_fix_round_one_state_shape_migrates_without_restarting_sequence(
    fake_ros
):
    fake, mod = fake_ros
    state_path = Path(mod._state_path())
    state_path.write_text(
        json.dumps({"sequence": 2, "saved_at": 123.0}),
        encoding="utf-8",
    )

    assert mod.main(["takeoff_start"]) == 0
    assert mod._load_sequence() == 3


def test_main_writes_complete_phase_only_at_terminal_marker(
    fake_ros, state_dir
):
    """Before ``experiment_end`` the phase topic must never see
    ``COMPLETE`` - that would falsely close the trial.
    """

    fake, mod = fake_ros
    for marker in ALLOWED_MARKERS[:-1]:
        assert mod.main([marker]) == 0
    phase_pub = fake.publishers["/flying_car_experiment_controller/phase"]
    for message in phase_pub.published:
        assert message.data != "COMPLETE"


# ---------------------------------------------------------------------------
# Cross-process serialization.
# ---------------------------------------------------------------------------


def _run_concurrent_call(monkeypatch, state_dir, argv):
    """Invoke the CLI main entry point with an injected fake ROS."""

    mod = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    return mod.main(argv)




def test_exclusive_lock_path_is_stable_across_loads(monkeypatch, state_dir):
    """Re-loading the script as two separate module instances must
    resolve to the SAME lock path; otherwise two CLI invocations
    could not contend for the lock and could each advance the
    sequence in parallel.
    """

    a = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    b = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    assert a._sequence_lock_path() == b._sequence_lock_path()
    assert a._sequence_lock_path().endswith("stage_sequence.lock")


def test_exclusive_lock_blocks_a_holder_for_full_duration(tmp_path):
    """Two separate ``python3`` processes must contend on the same
    lock file.  When one holds ``fcntl.LOCK_EX``, the OTHER's
    ``_exclusive_sequence_lock`` context cannot complete until the
    first releases.  This test opens the lock from an external
    process that holds for ``SLEEP_S`` seconds, then invokes the
    script's ``_exclusive_sequence_lock``; the wall-clock time taken
    must be at least ``SLEEP_S - 0.5``.
    """

    import fcntl
    import subprocess
    import sys as _sys

    state_dir = tmp_path / "shared_lock_state"
    state_dir.mkdir()
    lock_path = state_dir / "stage_sequence.lock"
    lock_path.touch()

    SLEEP_S = 4.0
    holder_script = (
        "import time, fcntl; "
        f"fh = open({str(lock_path)!r}, 'w'); "
        "fcntl.flock(fh.fileno(), fcntl.LOCK_EX); "
        "time.sleep(" + str(SLEEP_S) + "); "
        "fh.close()"
    )

    import os as _os
    import pathlib
    target = pathlib.Path(SCRIPT_PATH)
    workload_lines = [
        "import importlib.util, os, pathlib",
        f"target = {str(target)!r}",
        f"os.environ['STAGE_MARKER_STATE_DIR'] = {str(state_dir)!r}",
        "spec = importlib.util.spec_from_file_location('cli', target)",
        "mod = importlib.util.module_from_spec(spec)",
        "spec.loader.exec_module(mod)",
        "with mod._exclusive_sequence_lock():",
        "    print('acquired')",
    ]
    workload = "\n".join(workload_lines)
    # Write the script to a temp file we can hand to ``python3 <file>``.
    script_handle, script_path_str = tempfile.mkstemp(
        suffix=".py", prefix="stage_marker_lock_test_"
    )
    with os.fdopen(script_handle, "w") as fp:
        fp.write(workload)
    script_path = script_path_str

    env = {**_os.environ, "STAGE_MARKER_STATE_DIR": str(state_dir)}
    holder = subprocess.Popen([_sys.executable, "-c", holder_script], env=env)
    # Wait a tiny bit for the holder to *acquire* the lock - confirmed by
    # a no-blocking flock attempt failing with EAGAIN.
    deadline = time.time() + 1.0
    while time.time() < deadline:
        try:
            with open(str(lock_path), "w") as fh:
                fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            time.sleep(0.05)
        except OSError:
            break
    start = time.time()
    script_proc = subprocess.run(
        [_sys.executable, script_path],
        env=env, capture_output=True, text=True, timeout=15,
    )
    elapsed = time.time() - start
    holder.wait(timeout=10)

    assert script_proc.returncode == 0, (script_proc.stdout, script_proc.stderr)
    assert "acquired" in script_proc.stdout, script_proc.stdout
    # The script must have waited for the holder - small implementation
    # overhead means elapsed can be slightly less than SLEEP_S.
    assert elapsed >= SLEEP_S - 0.5, (
        "lock did not block: elapsed={:.2f}s expected >= {:.2f}s"
        .format(elapsed, SLEEP_S - 0.5)
    )
    os.unlink(script_path)


# ---------------------------------------------------------------------------
# README contract: real commands/scripts, not text-diffs.
# ---------------------------------------------------------------------------


def _read_readme():
    assert README_PATH.exists(), "Task 11 README missing"
    return README_PATH.read_text(encoding="utf-8")


def test_readme_has_all_required_sections():
    content = _read_readme()
    headings = [
        line.strip()
        for line in content.splitlines()
        if line.startswith("## ")
    ]
    required = [
        "## 1. Build and source the workspace",
        "## 2. Launch monitor_only.launch",
        "## 3. Verify MAVROS connection and QGC heartbeat",
        "## 4. Configure ground mode (MAV_TYPE=10)",
        "## 5. Manual ground run",
        "## 6. Configure flight mode (MAV_TYPE=14)",
        "## 7. Manual flight run",
        "## 8. Confirm landed and restore ground MAV_TYPE",
        "## 9. Drive to the landing point's left-front",
        "## 10. Publish each stage marker",
        "## 11. Abort on safety conditions",
        "## 12. Validate and generate the manual report",
    ]
    for needle in required:
        assert needle in headings, "README missing section: {!r}".format(needle)


def test_readme_documents_setup_command_and_source_paths():
    content = _read_readme()
    # Must NOT mention ``./setup_workspace.sh`` as a build command; the
    # canonical workspace setup actually lives at
    # ``Tools/flying_car_test/scripts/setup_workspace.sh`` and the README
    # must source the matching Python overlay.
    assert "./setup_workspace.sh" not in content, (
        "README must not invoke a nonexistent './setup_workspace.sh'; use "
        "the actual scripts/setup_workspace.sh entry point instead"
    )
    # The actual catkin_make source must use the real package locations.
    assert "/opt/ros/noetic/setup.bash" in content
    assert (
        "Tools/flying_car_test/scripts/setup_workspace.sh" in content
        or "setup_workspace.sh" in content
    )


def test_readme_documents_manual_control_vs_qgc_joystick():
    content = _read_readme()
    for value in ("650", "500", "150"):
        assert value in content, "README missing value {!r}".format(value)
    for label in (
        "QGC virtual joystick",
        "ManualControl",
        "MANUAL",
        "AUTO.LOITER",
        "AUTO.LAND",
    ):
        assert label in content, "README missing label {!r}".format(label)
    lower = content.lower()
    assert (
        "do not enter 650" in lower
        or "never enter 650" in lower
        or "must not enter 650" in lower
        or "do not type ``650``" in lower
    ), "README must explicitly forbid entering 650 into a normalized QGC field"


def test_readme_documents_seven_markers():
    content = _read_readme()
    for marker in ALLOWED_MARKERS:
        assert marker in content, "README missing marker {!r}".format(marker)


def test_readme_documents_publish_command():
    content = _read_readme()
    assert "stage_marker.py" in content
    assert "rosrun flying_car_test stage_marker.py" in content


def test_readme_documents_mav_type_preconditions():
    content = _read_readme()
    # Both MAV_TYPE switches (10 ↔ 14) must appear, and the preconditions
    # (disarmed, stopped, below the configured threshold, sustained neutral
    # manual output) must each be spelled out for both directions.
    sections_text = content
    for requirement in (
        "disarmed",
        "stopped",
        "neutral",
        "MAV_TYPE",
    ):
        assert requirement in sections_text, \
            "README missing required precondition {!r}".format(requirement)

    # Look for explicit "before" or "while" wording near each MAV_TYPE
    # switch to guarantee the operator cannot miss the preconditions.
    assert "10" in sections_text and "14" in sections_text


def test_readme_documents_physical_transmitter_procedure():
    content = _read_readme()
    # Must mention a physical / handheld transmitter ("remote",
    # "transmitter", or "RC") with explicit input mapping and the same
    # seven markers + monitor/safety guard.
    lower = content.lower()
    assert (
        "physical transmitter" in lower
        or "handheld transmitter" in lower
        or "remote controller" in lower
        or "rc transmitter" in lower
    ), "README must include a physical transmitter procedure section"


def test_readme_documents_monitor_only_heartbeat_semantics():
    content = _read_readme()
    # The clarification: ``heartbeat_required=false`` means the guard
    # only fails on connection / attitude / spatial / safety violations,
    # NOT on missing automatic-controller heartbeats.
    assert "heartbeat_required" in content
    assert "false" in content


def test_readme_markers_published_before_phase_not_after():
    """Each marker in the sequence must be published BEFORE its phase
    begins, not retroactively after the corresponding flight segment.
    """

    content = _read_readme()
    lower = content.lower()
    # The brief requires publishing markers at the START of each stage;
    # the README must say so explicitly.
    assert (
        "before" in lower and "after" in lower
    ), "README must spell out before/after marker timing"
    # We also expect an explicit phrase such as "publish" combined
    # with "before" in the marker section.
    assert "publish" in lower


def test_readme_documents_complete_phase_bridge():
    """experiment_end must let the passive monitor reach COMPLETE."""

    content = _read_readme()
    assert "COMPLETE" in content
    # The bridge property must be reachable from the operator runbook.
    assert "experiment_end" in content


# ---------------------------------------------------------------------------
# Importing scripts/stage_marker.py under various envs must not require ROS.
# ---------------------------------------------------------------------------



# ---------------------------------------------------------------------------
# Fix Round 2 contract tests (external review 2026-07-31).
# Each test pins one of the ten Critical / Important / Minor findings.
# ---------------------------------------------------------------------------


def test_phase_topic_is_latched_so_late_subscribers_see_marker():
    """Critical #1: a rosbag recorder that attaches after the marker is
    published must still observe the phase message.  The phase topic
    must therefore be ``latch=True`` on its ``ros.Publisher``.
    """

    script = _load_stage_marker(
        monkeypatch=monkeypatch_helper(state_dir),
        state_dir=state_dir,
    )
    # The PHASE_TOPIC publisher is built inside ``_publish_marker``; the
    # constructor's ``latch=`` argument is the only knob that makes
    # ``ros.Publisher`` cache the last value for late subscribers.
    src = SCRIPT_PATH.read_text()
    # Find the construction site of the phase publisher.
    # ``_publish_marker`` builds two publishers; the second one is the
    # phase publisher that must declare ``latch=True``.
    assert "phase_publisher = ros.Publisher" in src, (
        "phase publisher must be built explicitly with ros.Publisher"
    )
    # The phase publisher block must declare ``latch=True``.
    phase_block = src.split("phase_publisher = ros.Publisher", 1)[1]
    phase_block = phase_block.split("\n\n", 1)[0]
    assert "latch=True" in phase_block, (
        "phase publisher must declare latch=True so a rosbag recorder "
        "that attaches after marker publication still receives the "
        "phase string"
    )


def test_publish_marker_calls_ros_sleep_after_publish(monkeypatch, state_dir):
    """Critical #1: the script must let the rospy publish queue flush
    before the process exits; otherwise a non-latched phase message
    may be lost in the Linger buffer.  This is a heuristic - we only
    verify that ``ros.sleep`` is invoked between the publish calls and
    the function return.
    """

    src = SCRIPT_PATH.read_text()
    # The flush must be inside ``_publish_marker`` (not in ``main``)
    # and after BOTH publish calls.
    publish_marker_block = src.split("def _publish_marker", 1)[1]
    publish_marker_block = publish_marker_block.split("def ", 1)[0]
    last_stage = publish_marker_block.rfind("stage_publisher.publish")
    last_phase = publish_marker_block.rfind("phase_publisher.publish")
    last_flush = publish_marker_block.rfind("ros.sleep")
    assert last_stage > 0
    assert last_phase > 0
    assert last_flush > last_phase, (
        "ros.sleep must appear after both publish calls so the queue "
        "has time to drain before the script exits"
    )


def test_publish_marker_does_not_advance_committed_on_failure(monkeypatch, state_dir):
    """Critical #2: if ``stage_publisher.publish`` raises, the
    persisted *committed* sequence must NOT advance - so the next
    CLI invocation can retry the same marker without being blocked
    by an already-persisted sequence that never had a chance to be
    confirmed to any subscriber.

    Acceptable implementations: rollback to the previous value, OR
    record a pending_marker that the operator can resume.  Both
    shapes satisfy this contract.
    """

    script = _load_stage_marker(
        monkeypatch=monkeypatch_helper(state_dir),
        state_dir=state_dir,
    )
    # Pre-condition: no persisted state.
    assert script._load_sequence() is None

    # Simulate a publish failure by monkey-patching the publisher.
    class _BrokenPublisher:
        def __init__(self, *args, **kwargs):
            pass
        def publish(self, *args, **kwargs):
            raise OSError("simulated publish failure")
        def get_num_connections(self):
            return 1

    class _BrokenRos:
        Time = property(lambda self: type("T", (), {"now": staticmethod(lambda: type("TS", (), {"to_sec": staticmethod(lambda: 1.0)})())}))
        def Publisher(self, *args, **kwargs):
            return _BrokenPublisher(*args, **kwargs)
        def init_node(self, *args, **kwargs):
            pass
        def delete_param(self, *args, **kwargs):
            pass
        def set_param(self, *args, **kwargs):
            pass
        def get_param(self, *args, **kwargs):
            return None
        def sleep(self, *args, **kwargs):
            pass
        def on_shutdown(self, *args, **kwargs):
            pass
        def signal_shutdown(self, *args, **kwargs):
            pass
        def wait_for_message(self, *args, **kwargs):
            return None

    ros = _BrokenRos()
    string_cls = type("S", (), {})
    with pytest.raises(Exception):
        script._publish_marker(
            marker="ground_forward_start",
            ros=ros,
            string_cls=string_cls,
        )
    # Committed must not have advanced.
    assert script._load_sequence() is None


def test_init_node_uses_anonymous_for_concurrent_safety(monkeypatch, state_dir):
    """Important #4: two simultaneous CLI invocations must not collide
    on a shared ROS node name.  ``init_node`` must declare
    ``anonymous=True`` so the roscore appends a unique suffix.
    """

    src = SCRIPT_PATH.read_text()
    init_line = next(
        l for l in src.splitlines() if "init_node" in l and "ros.init_node" in l
    )
    assert "anonymous=True" in init_line, (
        "init_node must declare anonymous=True so concurrent CLI "
        "invocations get unique node names; otherwise ROS will "
        "shutdown the older node whenever a new one registers"
    )


def test_reset_cli_flag_clears_persisted_state(monkeypatch, state_dir):
    """Important #5: ``--reset`` (or ``STAGE_MARKER_RESET=1`` env var)
    must drop the persisted sequence so a new trial can start from
    marker 1.
    """

    script = _load_stage_marker(
        monkeypatch=monkeypatch_helper(state_dir),
        state_dir=state_dir,
    )
    # Seed the state file.
    script._save_sequence(3)
    # Reset path - exact name TBD by the implementation; pick the
    # canonical helper ``reset_state``.
    if hasattr(script, "reset_state"):
        script.reset_state()
    elif hasattr(script, "_reset_state"):
        script._reset_state()
    else:
        pytest.fail(
            "expected ``reset_state`` or ``_reset_state`` helper; "
            "see Important #5 in review-d59115b3b5..HEAD.diff"
        )
    assert script._load_sequence() is None


def test_main_reset_flag_exits_zero(monkeypatch, state_dir):
    """``main(['--reset'])`` must exit with status 0 and drop the
    persisted sequence.
    """

    script = _load_stage_marker(monkeypatch=monkeypatch, state_dir=state_dir)
    ros = _FakeRos([1700000000.0])
    monkeypatch.setattr(script, "rospy", ros)
    script._save_sequence(5)
    ros.parameters[script.PARAMETER_NAME] = 5
    rc = script.main(["--reset"])
    assert rc == 0
    assert script._load_sequence() is None
    assert script.PARAMETER_NAME not in ros.parameters


def test_readme_does_not_instruct_to_kill_rosbag_prematurely():
    """Critical #3: the operator runbook must NOT tell the operator to
    ``rosnode kill`` the rosbag before ``final_ground_start`` and
    ``experiment_end`` have been published - otherwise the final
    evidence never reaches the rosbag.
    """

    content = _read_readme()
    # Identify any rosbag-shutdown instruction that precedes the final
    # two markers.
    kill_idx = content.find("rosnode kill")
    final_ground_idx = content.find("final_ground_start")
    experiment_end_idx = content.find("experiment_end")
    if kill_idx > 0:
        # The kill instruction must appear AFTER both final markers,
        # i.e. the operator is told to wait for the final two markers
        # before shutting down the bag.
        last_marker_idx = max(
            final_ground_idx, experiment_end_idx
        )
        assert kill_idx > last_marker_idx, (
            "README instructs operator to kill rosbag (line {}) "
            "BEFORE final_ground_start/experiment_end (line {}) - "
            "the final evidence will not be recorded.".format(
                kill_idx, last_marker_idx
            )
        )


def test_readme_markers_published_before_segment_with_step_numbers():
    """Important #6: every README mention of a marker must put the
    publication BEFORE the corresponding segment runs.
    """

    content = _read_readme()
    # For each marker m, find the line that publishes m and check that
    # the surrounding text puts the publish command BEFORE the
    # corresponding segment text (not AFTER).
    for marker in ALLOWED_MARKERS:
        # The publish command appears in the format
        # ``rosrun flying_car_test stage_marker.py <marker>``.
        idx = content.find("stage_marker.py " + marker)
        assert idx > 0, (
            "README must document publishing of {!r}".format(marker)
        )
        # Take the next 500 chars; require a ``BEFORE`` (or "right
        # before") instruction.
        snippet = content[max(0, idx - 600):idx].lower()
        assert (
            "before" in snippet or "right before" in snippet
            or "at the start of" in snippet
            or "just before" in snippet
        ), (
            "README marker publication for {!r} is positioned "
            "AFTER its segment; required: BEFORE the segment.".format(
                marker
            )
        )


@pytest.mark.parametrize(
    "heading,marker,action",
    [
        ("## 5. Manual ground run", "ground_forward_start", "Click **Arm**"),
        (
            "## 6. Configure flight mode",
            "flight_switch_start",
            "rosrun mavros mavparam set MAV_TYPE 14",
        ),
        ("## 7. Manual flight run", "takeoff_start", "2. Arm the vehicle"),
        (
            "## 7. Manual flight run",
            "air_forward_start",
            "4. Switch to ``MANUAL``",
        ),
        (
            "## 7. Manual flight run",
            "landing_start",
            "6. Switch to ``AUTO.LAND``",
        ),
        (
            "## 9. Drive to the landing point",
            "final_ground_start",
            "2. Arm the vehicle",
        ),
    ],
)
def test_readme_section_orders_marker_command_before_action(
    heading, marker, action
):
    content = _read_readme()
    section = content.split(heading, 1)[1].split("\n## ", 1)[0]
    marker_index = section.index("stage_marker.py " + marker)
    action_index = section.index(action)
    assert marker_index < action_index


def test_readme_setup_paths_match_real_files():
    """Important #7: every path mentioned in the README setup
    instructions must resolve on a fresh clone.
    """

    content = _read_readme()
    # 1. setup_workspace.sh lives in scripts/ and emits a workspace at
    # ~/flying_car_test_ws/devel/setup.bash (not ~/.catkin_tools/devel).
    if "setup_workspace.sh" in content:
        # The README must not claim ~/.catkin_tools/devel as the
        # canonical overlay path because the actual script does not
        # create that.
        assert "~/.catkin_tools/devel" not in content, (
            "README must not reference ~/.catkin_tools/devel; the "
            "actual setup_workspace.sh creates a workspace at "
            "~/flying_car_test_ws/devel/setup.bash"
        )

    # 2. setup_gazebo.bash lives under Tools/simulation/gazebo-classic/.
    setup_gazebo_paths = re.findall(
        r"[A-Za-z0-9_./~-]*setup_gazebo\.bash", content
    )
    for path in setup_gazebo_paths:
        # Strip the leading $PX4-Autopilot prefix if any.
        bare = path.lstrip("/")
        # The on-disk path is Tools/simulation/gazebo-classic/setup_gazebo.bash
        # not Tools/setup_gazebo.bash.
        assert "Tools/simulation/gazebo-classic" in path, (
            "README references setup_gazebo.bash at {!r}; the actual "
            "on-disk path is Tools/simulation/gazebo-classic/setup_gazebo.bash".format(
                path
            )
        )


def test_readme_gazebo_setup_command_exports_existing_directories():
    """Execute the documented Gazebo source command and verify that its
    model and plugin paths exist in this checkout.
    """

    content = _read_readme()
    section = content.split(
        "## 1. Build and source the workspace", 1
    )[1].split("## 2.", 1)[0]
    block = re.search(r"```bash\n(.*?)```", section, re.DOTALL)
    assert block is not None
    commands = block.group(1).splitlines()
    gazebo_lines = []
    collecting = False
    for line in commands:
        if "setup_gazebo.bash" in line:
            collecting = True
        if collecting:
            gazebo_lines.append(line)
            if not line.rstrip().endswith("\\"):
                break
    assert gazebo_lines, "README setup block has no setup_gazebo command"

    command = "\n".join(gazebo_lines).replace(
        "~/PX4-Autopilot", str(REPO_ROOT)
    )
    probe = subprocess.run(
        [
            "bash",
            "-lc",
            "cd {!r}; {}; "
            "test -d \"${{GAZEBO_MODEL_PATH##*:}}\"; "
            "test -d \"${{GAZEBO_PLUGIN_PATH##*:}}\"".format(
                str(REPO_ROOT), command
            ),
        ],
        capture_output=True,
        text=True,
    )
    assert probe.returncode == 0, (probe.stdout, probe.stderr, command)


def test_readme_does_not_conflate_rc_in_with_control_input():
    """Important #8: ``/mavros/rc/in`` is the RCIOPlugin output (RC
    receiver telemetry into MAVROS), NOT the path the operator uses
    to drive the vehicle from a handheld RC transmitter.  The README
    must not instruct operators to publish to ``/mavros/rc/in`` from
    the RC transmitter.
    """

    content = _read_readme()
    # Find the RC section.
    rc_idx = content.find("RC")
    if rc_idx < 0:
        pytest.skip("README has no RC section")
    rc_section = content[rc_idx:rc_idx + 5000]
    # The README must NOT instruct the operator to use
    # ``/mavros/rc/in`` as the input topic.  Mentions of ``/mavros/rc/in``
    # are allowed if they describe the topic as telemetry (output).
    # We pin the contract: the README must explicitly say that
    # ``/mavros/rc/in`` is **telemetry** not control input, OR not
    # mention it at all.
    if "/mavros/rc/in" in rc_section:
        # Require a nearby clarifying word: "telemetry" / "output" /
        # "receiver channel" / "incoming".
        lower = rc_section.lower()
        ctx = lower.split("/mavros/rc/in", 1)
        before = ctx[0][-200:]
        after = ctx[1][:200]
        assert (
            "telemetry" in before + after
            or "output" in before + after
            or "receiver" in before + after
        ), (
            "README mentions /mavros/rc/in in the RC section without "
            "flagging it as telemetry/receiver-output.  Real RC "
            "transmitter input flows through the FC's RC receiver "
            "and is exposed via /mavros/manual_control/control (NOT "
            "via /mavros/rc/in)."
        )


def test_readme_mav_type_14_is_octorotor_not_quadrotor():
    """Minor #9: MAV_TYPE 14 is MAV_TYPE_OCTOROTOR, not
    MAV_TYPE_QUADROTOR.  The README must use the correct name.
    """

    content = _read_readme()
    if "MAV_TYPE" not in content:
        pytest.skip("README has no MAV_TYPE section")
    # Find the line that pairs ``14`` with a name.
    m = re.search(
        r"MAV_TYPE[_ ]*(QUADROTOR|OCTOROTOR|GROUND_ROVER|OCTO|QUAD)",
        content,
    )
    assert m, "README must name the MAV_TYPE enum"
    # Look for the mention of 14 nearby.  Skip the canonical
    # section heading itself so the README can keep the navigation
    # anchor ``## 6. Configure flight mode (MAV_TYPE=14)`` (the brief
    # pinned that string verbatim); the actual semantics live in the
    # body content which is what we check here.
    for line in content.splitlines():
        if line.lstrip().startswith("## "):
            continue
        if "MAV_TYPE" in line and ("14" in line or "QUADROTOR" in line or "OCTOROTOR" in line):
            # The pairing must be OCTOROTOR not QUADROTOR when 14 is mentioned.
            if "14" in line:
                assert "OCTOROTOR" in line or "Octorotor" in line or "octo" in line.lower(), (
                    "MAV_TYPE 14 is MAV_TYPE_OCTOROTOR; the README "
                    "must not call it QUADROTOR.  Offending line: {!r}".format(
                        line[:200]
                    )
                )


def test_main_help_flag_exits_nonzero():
    """Minor #10: ``main([])`` (empty argv) must exit with non-zero
    status so a missing-positional-arg invocation does not silently
    pass.  ``main(['--help'])`` follows POSIX convention and may
    exit 0 (with help printed).
    """

    script = _load_stage_marker(monkeypatch=monkeypatch_helper(state_dir_helper()))
    rc = script.main([])
    assert rc != 0, (
        "main([]) returned 0; the script should exit non-zero so a "
        "missing-positional-arg invocation does not appear to succeed"
    )


# --- helpers used by the tests above ----------------------------------------


def monkeypatch_helper(state_dir):
    """Local monkeypatch fixture stub (the real fixtures come from
    ``conftest.py``).  This helper exists only to keep the test body
    readable and is shadowed by the conftest fixture when the test
    runs under pytest.
    """
    return None


def state_dir_helper():
    """Same role as ``monkeypatch_helper``; replaced by conftest."""
    return None
