import math
import threading
import time

import pytest

from flying_car_test.models import Action, ManualControl, Phase
from flying_car_test.safety import (
    ExpectedNodeMonitor,
    SafetyActionExecutor,
    SafetyCommandArbiter,
    SafetyGuard,
    SafetyRuntime,
    missing_expected_nodes,
)


def make_guard(**overrides):
    values = {
        "heartbeat_timeout_s": 0.3,
        "hard_tilt_deg": 45.0,
        "maximum_agl_m": 12.0,
        "maximum_planar_deviation_m": 10.0,
    }
    values.update(overrides)
    return SafetyGuard(**values)


def test_ground_heartbeat_loss_neutralizes_before_disarm():
    guard = make_guard()

    actions = guard.tick(
        now=1.0,
        phase=Phase.GROUND_FORWARD,
        last_heartbeat=0.0,
        heartbeat_seen=True,
        armed=True,
    )

    assert [action.kind for action in actions] == [
        "manual_neutral",
        "abort",
    ]


def test_ground_intervention_waits_for_three_neutral_ticks_then_disarms_once():
    guard = make_guard()

    first = guard.tick(
        now=1.0,
        phase=Phase.GROUND_FORWARD,
        last_heartbeat=0.0,
        heartbeat_seen=True,
        armed=True,
    )
    second = guard.tick(now=1.1, phase=Phase.GROUND_FORWARD, armed=True)
    third = guard.tick(now=1.2, phase=Phase.GROUND_FORWARD, armed=True)
    fourth = guard.tick(now=1.3, phase=Phase.GROUND_FORWARD, armed=True)
    stopped = guard.tick(now=1.4, phase=Phase.GROUND_FORWARD, armed=False)

    assert [action.kind for action in first] == [
        "manual_neutral",
        "abort",
    ]
    assert [action.kind for action in second] == ["manual_neutral"]
    assert [action.kind for action in third] == [
        "manual_neutral",
        "disarm",
    ]
    assert [action.kind for action in fourth] == ["manual_neutral"]
    assert stopped == []


def test_air_intervention_holds_requests_land_once_and_streams_until_landed():
    guard = make_guard()
    inputs = {
        "phase": Phase.AIR_FORWARD,
        "x": 4.0,
        "y": -2.0,
        "z": 5.0,
        "yaw": 0.25,
    }

    first = guard.tick(
        now=1.0,
        last_heartbeat=0.0,
        heartbeat_seen=True,
        landed=False,
        **inputs
    )
    second = guard.tick(now=1.1, landed=False, **inputs)
    stopped = guard.tick(now=1.2, landed=True, **inputs)

    assert [action.kind for action in first] == [
        "position_hold",
        "auto_land",
        "abort",
    ]
    assert first[0].values == {
        "x": 4.0,
        "y": -2.0,
        "z": 5.0,
        "yaw": 0.25,
    }
    assert [action.kind for action in second] == ["position_hold"]
    assert stopped == []


def test_air_intervention_latches_first_hold_despite_feedback_drift():
    guard = make_guard(heartbeat_required=False)

    first = guard.tick(
        now=1.0,
        phase=Phase.AIR_FORWARD,
        connected=False,
        landed=False,
        x=1.0,
        y=2.0,
        z=3.0,
        yaw=0.4,
    )
    second = guard.tick(
        now=1.1,
        phase=Phase.AIR_FORWARD,
        landed=False,
        x=8.0,
        y=9.0,
        z=10.0,
        yaw=1.1,
    )

    assert first[0].values == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "yaw": 0.4,
    }
    assert second[0].values == first[0].values


def test_phase_sets_partition_every_phase():
    classified = (
        SafetyGuard.GROUND_PHASES
        | SafetyGuard.AIR_PHASES
        | SafetyGuard.INACTIVE_PHASES
    )

    assert classified == set(Phase)
    assert not (SafetyGuard.GROUND_PHASES & SafetyGuard.AIR_PHASES)
    assert not (SafetyGuard.GROUND_PHASES & SafetyGuard.INACTIVE_PHASES)
    assert not (SafetyGuard.AIR_PHASES & SafetyGuard.INACTIVE_PHASES)
    assert Phase.OFFBOARD_PRIME in SafetyGuard.GROUND_PHASES
    assert Phase.LAND in SafetyGuard.AIR_PHASES


def test_inactive_failure_neutralizes_without_vehicle_service_action():
    guard = make_guard()

    actions = guard.tick(
        now=1.0,
        phase=Phase.PRECHECK,
        connected=False,
        armed=True,
    )

    assert [action.kind for action in actions] == [
        "manual_neutral",
        "abort",
    ]


@pytest.mark.parametrize(
    "phase",
    [Phase.PRECHECK, Phase.ABORT, Phase.COMPLETE],
)
def test_trusted_airborne_state_overrides_inactive_phase(phase):
    guard = make_guard(heartbeat_required=False)

    actions = guard.tick(
        now=1.0,
        phase=phase,
        connected=False,
        state_seen=True,
        landed_seen=True,
        armed=True,
        landed=False,
        x=1.0,
        y=2.0,
        z=3.0,
        yaw=0.4,
    )

    assert [action.kind for action in actions] == [
        "position_hold",
        "auto_land",
        "abort",
    ]


def test_trusted_ground_state_uses_neutral_then_disarm_in_terminal_phase():
    guard = make_guard(heartbeat_required=False)
    inputs = {
        "phase": Phase.COMPLETE,
        "state_seen": True,
        "landed_seen": True,
        "armed": True,
        "landed": True,
    }

    first = guard.tick(now=1.0, connected=False, **inputs)
    second = guard.tick(now=1.1, **inputs)
    third = guard.tick(now=1.2, **inputs)

    assert [action.kind for action in first] == [
        "manual_neutral",
        "abort",
    ]
    assert [action.kind for action in second] == ["manual_neutral"]
    assert [action.kind for action in third] == [
        "manual_neutral",
        "disarm",
    ]


@pytest.mark.parametrize(
    "phase, state",
    [
        (Phase.GROUND_FORWARD, {"armed": False}),
        (Phase.AIR_FORWARD, {"landed": True}),
    ],
)
def test_abort_telemetry_is_emitted_even_if_vehicle_is_already_safe(
    phase, state
):
    guard = make_guard(heartbeat_required=False)

    actions = guard.tick(
        now=1.0,
        phase=phase,
        connected=False,
        **state
    )

    assert [action.kind for action in actions] == ["abort"]
    assert actions[0].values["reason"] == "connection_loss"


def test_automatic_watchdog_does_not_intervene_before_first_heartbeat():
    guard = make_guard()

    actions = guard.tick(
        now=100.0,
        phase=Phase.GROUND_FORWARD,
        last_heartbeat=None,
        heartbeat_seen=False,
    )

    assert actions == []
    assert guard.abort_reason is None


@pytest.mark.parametrize(
    "inputs, expected_reason",
    [
        ({"roll": math.radians(45.1)}, "hard_tilt"),
        ({"pitch": math.radians(-45.1)}, "hard_tilt"),
        ({"agl_m": 12.01}, "maximum_agl"),
        ({"planar_deviation_m": 10.01}, "maximum_planar_deviation"),
        ({"connected": False}, "connection_loss"),
        (
            {"processes_ok": False, "missing_processes": ("/mavros",)},
            "expected_ros_node_loss:/mavros",
        ),
    ],
)
def test_hard_failure_paths_latch_expected_reason(inputs, expected_reason):
    guard = make_guard(heartbeat_required=False)

    actions = guard.tick(
        now=1.0,
        phase=Phase.AIR_FORWARD,
        **inputs
    )

    assert guard.abort_reason == expected_reason
    assert actions[-1].kind == "abort"
    assert actions[-1].values["reason"] == expected_reason


@pytest.mark.parametrize(
    "inputs",
    [
        {"roll": math.radians(45.0)},
        {"pitch": math.radians(-45.0)},
        {"agl_m": 8.0},
        {"planar_deviation_m": 10.0},
    ],
)
def test_hard_limits_trigger_only_when_exceeded(inputs):
    guard = make_guard(heartbeat_required=False)

    actions = guard.tick(
        now=1.0,
        phase=Phase.AIR_FORWARD,
        **inputs
    )

    assert actions == []


def test_manual_mode_ignores_heartbeat_but_keeps_other_checks_active():
    guard = make_guard(heartbeat_required=False)

    ignored = guard.tick(
        now=100.0,
        phase=Phase.AIR_FORWARD,
        last_heartbeat=0.0,
        heartbeat_seen=True,
    )
    failed = guard.tick(
        now=100.1,
        phase=Phase.AIR_FORWARD,
        roll=math.radians(46.0),
    )

    assert ignored == []
    assert guard.abort_reason == "hard_tilt"
    assert failed[-1].values["reason"] == "hard_tilt"


def test_first_reason_is_latched_and_abort_is_not_published_twice():
    guard = make_guard(heartbeat_required=False)

    first = guard.tick(
        now=1.0,
        phase=Phase.GROUND_FORWARD,
        connected=False,
    )
    second = guard.tick(
        now=1.1,
        phase=Phase.GROUND_FORWARD,
        roll=math.radians(60.0),
        agl_m=20.0,
    )

    assert guard.abort_reason == "connection_loss"
    assert [action.kind for action in first].count("abort") == 1
    assert [action.kind for action in second].count("abort") == 0


def test_backward_clock_is_a_hard_failure():
    guard = make_guard(heartbeat_required=False)

    assert guard.tick(
        now=1.0,
        phase=Phase.GROUND_FORWARD,
        clock_stamp=10.0,
    ) == []
    actions = guard.tick(
        now=1.1,
        phase=Phase.GROUND_FORWARD,
        clock_stamp=9.9,
    )

    assert guard.abort_reason == "clock_backward"
    assert actions[-1].values["reason"] == "clock_backward"


def test_stopped_clock_fails_after_timeout_not_on_one_duplicate_sample():
    guard = make_guard(heartbeat_required=False)

    assert guard.tick(
        now=1.0,
        phase=Phase.GROUND_FORWARD,
        clock_stamp=10.0,
    ) == []
    assert guard.tick(
        now=1.2,
        phase=Phase.GROUND_FORWARD,
        clock_stamp=10.0,
    ) == []
    actions = guard.tick(
        now=1.31,
        phase=Phase.GROUND_FORWARD,
        clock_stamp=10.0,
    )

    assert guard.abort_reason == "clock_stopped"
    assert actions[-1].values["reason"] == "clock_stopped"


class FakeGateway:
    def __init__(self):
        self.manual = []
        self.holds = []
        self.services = []
        self.events = []
        self.mode_result = True
        self.arm_result = True
        self.mode_exception = None
        self.mode_started = threading.Event()
        self.mode_release = threading.Event()
        self.block_mode = False

    def publish_manual(self, x, y, z, r):
        self.manual.append((x, y, z, r))
        self.events.append(("manual", x, y, z, r))

    def publish_local_pose(self, x, y, z, yaw):
        self.holds.append((x, y, z, yaw))
        self.events.append(("pose", x, y, z, yaw))

    def set_mode(self, mode):
        self.services.append(("set_mode", mode))
        self.events.append(("set_mode", mode))
        self.mode_started.set()
        if self.block_mode:
            self.mode_release.wait()
        if self.mode_exception is not None:
            raise self.mode_exception
        return self.mode_result

    def set_armed(self, armed):
        self.services.append(("set_armed", armed))
        self.events.append(("set_armed", armed))
        return self.arm_result


def make_executor(gateway, aborts=None, statuses=None):
    return SafetyActionExecutor(
        gateway=gateway,
        manual_neutral=ManualControl(0.0, 0.0, 500.0, 0.0),
        publish_abort=(aborts if aborts is not None else []).append,
        publish_status=(statuses if statuses is not None else []).append,
    )


def test_command_arbiter_relays_controller_commands_and_keepalive_normally():
    gateway = FakeGateway()
    arbiter = SafetyCommandArbiter(
        gateway, ManualControl(0.0, 0.0, 500.0, 0.0)
    )

    assert arbiter.relay_manual(1.0, 2.0, 600.0, 3.0)
    assert arbiter.relay_pose(4.0, 5.0, 6.0, 0.7)
    assert arbiter.forward_latest()

    assert arbiter.wiring_ready is True
    assert gateway.manual == [(1.0, 2.0, 600.0, 3.0)]
    assert gateway.holds == [
        (4.0, 5.0, 6.0, 0.7),
        (4.0, 5.0, 6.0, 0.7),
    ]


def test_fresh_controller_motion_clears_neutral_takeover():
    gateway = FakeGateway()
    arbiter = SafetyCommandArbiter(
        gateway, ManualControl(0.0, 0.0, 500.0, 0.0)
    )
    assert arbiter.relay_manual(10.0, 20.0, 700.0, 30.0)

    assert arbiter.publish_neutral()
    assert arbiter.relay_manual(40.0, 50.0, 800.0, 60.0)
    assert arbiter.relay_pose(7.0, 8.0, 9.0, 1.0)
    assert arbiter.forward_latest()
    assert arbiter.publish_neutral()

    assert gateway.events == [
        ("manual", 10.0, 20.0, 700.0, 30.0),
        ("manual", 0.0, 0.0, 500.0, 0.0),
        ("manual", 40.0, 50.0, 800.0, 60.0),
        ("pose", 7.0, 8.0, 9.0, 1.0),
        ("pose", 7.0, 8.0, 9.0, 1.0),
        ("manual", 0.0, 0.0, 500.0, 0.0),
    ]


def test_disarm_is_queued_only_after_three_actual_outbound_neutral_ticks():
    gateway = FakeGateway()
    executor = make_executor(gateway)
    try:
        executor.execute([Action("manual_neutral", {})])
        executor.execute([
            Action("manual_neutral", {}),
            Action("disarm", {}),
        ])
        assert executor.wait(timeout=0.5)
        assert gateway.services == []

        executor.execute([
            Action("manual_neutral", {}),
            Action("disarm", {}),
        ])
        assert executor.wait(timeout=0.5)

        assert gateway.events == [
            ("manual", 0.0, 0.0, 500.0, 0.0),
            ("manual", 0.0, 0.0, 500.0, 0.0),
            ("manual", 0.0, 0.0, 500.0, 0.0),
            ("set_armed", False),
        ]
    finally:
        assert executor.close(timeout=0.5)


def test_fresh_controller_pose_clears_hold_takeover():
    gateway = FakeGateway()
    arbiter = SafetyCommandArbiter(
        gateway, ManualControl(0.0, 0.0, 500.0, 0.0)
    )
    first_hold = {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.4}

    assert arbiter.publish_hold(first_hold)
    assert arbiter.relay_pose(9.0, 9.0, 9.0, 0.9)
    assert arbiter.publish_hold(
        {"x": 7.0, "y": 8.0, "z": 9.0, "yaw": 1.0}
    )

    assert gateway.holds == [
        (1.0, 2.0, 3.0, 0.4),
        (9.0, 9.0, 9.0, 0.9),
        (7.0, 8.0, 9.0, 1.0),
    ]


def test_blocked_service_does_not_block_safe_output_or_abort_telemetry():
    gateway = FakeGateway()
    gateway.block_mode = True
    aborts = []
    executor = make_executor(gateway, aborts=aborts)
    actions = [
        Action("position_hold", {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.4}),
        Action("auto_land", {}),
        Action("abort", {"reason": "hard_tilt"}),
    ]
    try:
        executor.execute(actions)
        assert gateway.mode_started.wait(timeout=0.5)

        started = time.monotonic()
        executor.execute([
            Action("position_hold", {
                "x": 1.0,
                "y": 2.0,
                "z": 3.0,
                "yaw": 0.4,
            }),
        ])
        elapsed = time.monotonic() - started

        assert elapsed < 0.1
        assert gateway.holds == [
            (1.0, 2.0, 3.0, 0.4),
            (1.0, 2.0, 3.0, 0.4),
        ]
        assert aborts == ["hard_tilt"]
    finally:
        gateway.mode_release.set()
        assert executor.close(timeout=0.5)


@pytest.mark.parametrize("failure", [False, RuntimeError("service down")])
def test_service_failure_is_reported_once_without_retry_storm(failure):
    gateway = FakeGateway()
    statuses = []
    if isinstance(failure, Exception):
        gateway.mode_exception = failure
    else:
        gateway.mode_result = failure
    executor = make_executor(gateway, statuses=statuses)
    try:
        action = Action("auto_land", {})
        executor.execute([action])
        executor.execute([action])
        assert executor.wait(timeout=0.5)
        executor.execute([action])
        assert executor.wait(timeout=0.5)

        assert gateway.services == [("set_mode", "AUTO.LAND")]
        assert len(executor.service_failures) == 1
        assert statuses == [executor.service_failures[0]]
        assert executor.service_failures[0]["ok"] is False
    finally:
        assert executor.close(timeout=0.5)


def test_service_actions_are_serial_and_idempotent():
    gateway = FakeGateway()
    executor = make_executor(gateway)
    try:
        executor.execute([
            Action("manual_neutral", {}),
            Action("manual_neutral", {}),
            Action("manual_neutral", {}),
        ])
        executor.execute([
            Action("auto_land", {}),
            Action("disarm", {}),
            Action("auto_land", {}),
            Action("disarm", {}),
        ])
        assert executor.wait(timeout=0.5)

        assert gateway.services == [
            ("set_mode", "AUTO.LAND"),
            ("set_armed", False),
        ]
    finally:
        assert executor.close(timeout=0.5)


def test_expected_node_loss_uses_normalized_exact_ros_names():
    missing = missing_expected_nodes(
        ["gazebo", "/mavros", "/flying_car_experiment_monitor"],
        ["/gazebo", "/mavros", "/unrelated"],
    )

    assert missing == ("/flying_car_experiment_monitor",)


def test_expected_node_monitor_arms_after_startup_then_latches_loss():
    samples = iter([
        ["/gazebo"],
        ["/gazebo", "/mavros"],
        ["/gazebo"],
    ])
    monitor = ExpectedNodeMonitor(
        expected=["/gazebo", "/mavros"],
        list_nodes=lambda: next(samples),
    )

    monitor.sample_once()
    assert monitor.missing == ()
    monitor.sample_once()
    assert monitor.missing == ()
    monitor.sample_once()
    assert monitor.missing == ("/mavros",)


def test_expected_node_monitor_fails_missing_startup_node_after_grace():
    now = [0.0]
    monitor = ExpectedNodeMonitor(
        expected=["/gazebo", "/mavros"],
        list_nodes=lambda: ["/gazebo"],
        startup_grace_s=0.3,
        clock=lambda: now[0],
    )

    monitor.sample_once()
    assert monitor.missing == ()
    now[0] = 0.31
    monitor.sample_once()

    assert monitor.missing == ("/mavros",)


def test_runtime_maps_feedback_to_guard_and_streams_selected_safe_output():
    gateway = FakeGateway()
    aborts = []
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway, aborts=aborts)
    health = ExpectedNodeMonitor([], list_nodes=lambda: [])
    runtime = SafetyRuntime(guard, executor, health)
    try:
        runtime.update_phase("PRECHECK")
        runtime.update_vehicle(connected=True, armed=True, landed=True)
        runtime.update_pose(1.0, 2.0, 1.0, 0.0, 0.0, 0.25)
        assert runtime.step(now=1.0) == []

        runtime.update_phase("AIR_FORWARD")
        runtime.update_landed(False)
        runtime.update_pose(1.0, 2.0, 13.1, 0.0, 0.0, 0.25)
        actions = runtime.step(now=1.1)

        assert guard.abort_reason == "maximum_agl"
        assert [action.kind for action in actions] == [
            "position_hold",
            "auto_land",
            "abort",
        ]
        assert gateway.holds == [(1.0, 2.0, 13.1, 0.25)]
        assert aborts == ["maximum_agl"]
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_captures_agl_origin_only_from_trusted_landed_feedback():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )
    try:
        runtime.update_state(connected=True, armed=True)
        runtime.update_landed(False)
        runtime.update_pose(0.0, 0.0, 5.0, 0.0, 0.0, 0.0)
        runtime.update_landed(True)
        runtime.update_pose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        runtime.update_landed(False)
        runtime.update_pose(0.0, 0.0, 13.1, 0.0, 0.0, 0.0)

        actions = runtime.step(now=1.0)

        assert guard.abort_reason == "maximum_agl"
        assert actions[0].kind == "position_hold"
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_captures_plan_origin_only_from_trusted_landed_feedback():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )
    try:
        runtime.update_state(connected=True, armed=True)
        runtime.update_landed(False)
        runtime.update_pose(5.0, 0.0, 2.0, 0.0, 0.0, 0.0)
        runtime.update_landed(True)
        runtime.update_pose(1.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        runtime.update_landed(False)
        runtime.update_pose(11.1, 0.0, 2.0, 0.0, 0.0, 0.0)

        actions = runtime.step(now=1.0)

        assert guard.abort_reason == "maximum_planar_deviation"
        assert actions[0].kind == "position_hold"
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_detects_clock_that_never_starts_after_grace():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )
    try:
        runtime.update_phase("PRECHECK")
        assert runtime.step(now=1.0) == []
        actions = runtime.step(now=1.31)

        assert guard.abort_reason == "clock_failure"
        assert actions[-1].values["reason"] == "clock_failure"
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_allows_missing_state_only_during_wall_clock_startup_grace():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
    )
    try:
        runtime.update_clock(10.0)
        assert runtime.step(now=1.0) == []
        runtime.update_clock(10.1)
        actions = runtime.step(now=1.31)

        assert guard.abort_reason == "state_feedback_missing"
        assert actions[-1].values["reason"] == "state_feedback_missing"
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_relays_ingress_and_reports_wiring_ready():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
    )
    try:
        runtime.update_state(connected=True, armed=True)
        runtime.update_landed(True)
        runtime.update_clock(10.0)

        assert runtime.relay_manual(
            1.0, 2.0, 600.0, 3.0, received_at=1.0
        )
        assert runtime.wiring_ready is True
        assert runtime.step(now=1.0) == []

        assert gateway.manual == [
            (1.0, 2.0, 600.0, 3.0),
            (1.0, 2.0, 600.0, 3.0),
        ]
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_fails_closed_when_controller_ingress_is_not_remapped():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
    )
    try:
        runtime.update_state(connected=True, armed=True)
        runtime.update_landed(True)
        runtime.update_clock(10.0)
        assert runtime.step(now=1.0) == []
        runtime.update_clock(10.1)
        actions = runtime.step(now=1.31)

        assert runtime.wiring_ready is False
        assert guard.abort_reason == "controller_command_ingress_missing"
        assert actions[-1].values["reason"] == (
            "controller_command_ingress_missing"
        )
    finally:
        assert executor.close(timeout=0.5)


@pytest.mark.parametrize(
    ("landed", "phase", "safe_output"),
    [
        (True, Phase.GROUND_FORWARD, "manual_neutral"),
        (False, Phase.AIR_FORWARD, "position_hold"),
    ],
)
def test_runtime_rejects_stale_manual_ingress_after_strict_timeout(
    landed, phase, safe_output
):
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
        command_ingress_timeout_s=0.3,
    )
    moving_command = (1.0, 2.0, 600.0, 3.0)
    try:
        runtime.update_phase(phase)
        runtime.update_vehicle(
            connected=True, armed=True, landed=landed
        )
        runtime.update_pose(4.0, 5.0, 6.0, 0.0, 0.0, 0.7)
        runtime.update_clock(10.0)
        runtime.update_controller_command("manual", now=0.0)
        assert runtime.relay_manual(
            *moving_command, received_at=0.0
        )

        assert runtime.step(now=0.3) == []
        assert gateway.manual.count(moving_command) == 2

        runtime.update_clock(10.1)
        actions = runtime.step(now=0.300001)

        assert guard.abort_reason == (
            "controller_command_ingress_timeout"
        )
        assert actions[0].kind == safe_output
        assert gateway.manual.count(moving_command) == 2
    finally:
        assert executor.close(timeout=0.5)


@pytest.mark.parametrize(
    ("initial_kind", "next_kind"),
    [
        ("manual", "position"),
        ("position", "manual"),
    ],
)
def test_runtime_requires_fresh_ingress_matching_command_kind_after_switch(
    initial_kind, next_kind
):
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
        command_ingress_timeout_s=0.3,
    )

    def send_ingress(kind, received_at):
        if kind == "manual":
            return runtime.relay_manual(
                1.0, 2.0, 600.0, 3.0,
                received_at=received_at,
            )
        return runtime.relay_pose(
            4.0, 5.0, 6.0, 0.7,
            received_at=received_at,
        )

    try:
        runtime.update_phase(Phase.GROUND_FORWARD)
        runtime.update_vehicle(
            connected=True, armed=True, landed=True
        )
        runtime.update_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        runtime.update_clock(10.0)
        runtime.update_controller_command(initial_kind, now=0.0)
        assert send_ingress(initial_kind, received_at=0.0)
        assert runtime.step(now=0.0) == []

        runtime.update_controller_command(next_kind, now=0.1)
        output_count = len(gateway.events)
        runtime.update_clock(10.1)
        assert runtime.step(now=0.39) == []
        assert len(gateway.events) == output_count

        runtime.update_clock(10.2)
        actions = runtime.step(now=0.400001)

        assert guard.abort_reason == (
            "controller_command_ingress_timeout"
        )
        assert actions[0].kind == "manual_neutral"
        assert len(gateway.events) == output_count + 1
    finally:
        assert executor.close(timeout=0.5)


@pytest.mark.parametrize(
    ("initial_kind", "next_kind", "late_kind"),
    [
        ("manual", "position", "manual"),
        ("position", "manual", "pose"),
    ],
)
def test_runtime_rejects_late_ingress_after_active_kind_switch(
    initial_kind, next_kind, late_kind
):
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )

    def send_ingress(kind, received_at):
        if kind == "manual":
            return runtime.relay_manual(
                1.0, 2.0, 600.0, 3.0,
                received_at=received_at,
            )
        return runtime.relay_pose(
            4.0, 5.0, 6.0, 0.7,
            received_at=received_at,
        )

    try:
        runtime.update_controller_command(initial_kind, now=0.0)
        assert send_ingress(initial_kind, received_at=0.0)
        output_count = len(gateway.events)

        runtime.update_controller_command(next_kind, now=0.1)

        assert not send_ingress(late_kind, received_at=0.11)
        assert len(gateway.events) == output_count
    finally:
        assert executor.close(timeout=0.5)


@pytest.mark.parametrize("late_kind", ["manual", "pose"])
def test_runtime_rejects_all_controller_ingress_for_unknown_kind(
    late_kind
):
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )

    def send_ingress(kind):
        if kind == "manual":
            return runtime.relay_manual(
                1.0, 2.0, 600.0, 3.0, received_at=0.11
            )
        return runtime.relay_pose(
            4.0, 5.0, 6.0, 0.7, received_at=0.11
        )

    try:
        runtime.update_controller_command("unsupported", now=0.1)
        output_count = len(gateway.events)

        assert not send_ingress(late_kind)
        assert len(gateway.events) == output_count
    finally:
        assert executor.close(timeout=0.5)


def test_unknown_command_kind_expires_grace_and_does_not_forward_old_ingress():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        command_ingress_timeout_s=0.3,
    )
    try:
        runtime.update_phase(Phase.GROUND_FORWARD)
        runtime.update_vehicle(
            connected=True, armed=True, landed=True
        )
        runtime.update_clock(10.0)
        runtime.update_controller_command("manual", now=0.0)
        assert runtime.relay_manual(
            1.0, 2.0, 600.0, 3.0, received_at=0.0
        )
        runtime.update_controller_command("unsupported", now=0.1)
        output_count = len(gateway.events)

        assert runtime.step(now=0.39) == []
        assert len(gateway.events) == output_count

        runtime.update_clock(10.1)
        actions = runtime.step(now=0.400001)
        assert guard.abort_reason == (
            "controller_command_ingress_timeout"
        )
        assert actions[0].kind == "manual_neutral"
    finally:
        assert executor.close(timeout=0.5)


def test_wiring_optional_does_not_require_controller_ingress():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
        command_ingress_timeout_s=0.3,
    )
    try:
        runtime.update_vehicle(
            connected=True, armed=True, landed=True
        )
        runtime.update_clock(10.0)
        assert runtime.step(now=0.0) == []

        runtime.update_clock(10.1)
        assert runtime.step(now=1.0) == []
        assert guard.abort_reason is None
    finally:
        assert executor.close(timeout=0.5)


def test_task10_remap_contract_uses_private_controller_ingress_topics():
    assert SafetyRuntime.MANUAL_INGRESS_TOPIC == "~controller/manual"
    assert SafetyRuntime.POSE_INGRESS_TOPIC == "~controller/pose"


def test_first_heartbeat_does_not_exempt_missing_state_after_grace():
    gateway = FakeGateway()
    guard = make_guard(heartbeat_required=True)
    executor = make_executor(gateway)
    runtime = SafetyRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
        startup_grace_s=0.3,
    )
    try:
        runtime.update_clock(10.0)
        runtime.update_heartbeat(1.3)
        assert runtime.step(now=1.0) == []
        runtime.update_clock(10.1)
        actions = runtime.step(now=1.31)

        assert guard.abort_reason == "state_feedback_missing"
        assert actions[-1].values["reason"] == "state_feedback_missing"
    finally:
        assert executor.close(timeout=0.5)


class ExplodingRuntime(SafetyRuntime):
    def step(self, now):
        raise RuntimeError("boom")


def test_runtime_exception_latches_fixed_air_hold_and_one_auto_land():
    gateway = FakeGateway()
    aborts = []
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway, aborts=aborts)
    runtime = ExplodingRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )
    try:
        runtime.update_phase("ABORT")
        runtime.update_vehicle(connected=True, armed=True, landed=False)
        runtime.update_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.4)
        runtime.run_once(now=1.0)
        runtime.update_pose(8.0, 9.0, 10.0, 0.0, 0.0, 1.1)
        runtime.run_once(now=1.1)
        assert executor.wait(timeout=0.5)

        assert guard.abort_reason == (
            "safety_runtime_exception:RuntimeError:boom"
        )
        assert gateway.holds == [
            (1.0, 2.0, 3.0, 0.4),
            (1.0, 2.0, 3.0, 0.4),
        ]
        assert gateway.services == [("set_mode", "AUTO.LAND")]
        assert aborts == [
            "safety_runtime_exception:RuntimeError:boom"
        ]
    finally:
        assert executor.close(timeout=0.5)


def test_runtime_exception_ground_path_neutralizes_three_times_then_disarms():
    gateway = FakeGateway()
    aborts = []
    guard = make_guard(heartbeat_required=False)
    executor = make_executor(gateway, aborts=aborts)
    runtime = ExplodingRuntime(
        guard,
        executor,
        ExpectedNodeMonitor([], list_nodes=lambda: []),
    )
    try:
        runtime.update_phase("COMPLETE")
        runtime.update_vehicle(connected=True, armed=True, landed=True)
        runtime.update_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        runtime.run_once(now=1.0)
        runtime.run_once(now=1.1)
        runtime.run_once(now=1.2)
        runtime.run_once(now=1.3)
        assert executor.wait(timeout=0.5)

        assert gateway.manual == [
            (0.0, 0.0, 500.0, 0.0),
            (0.0, 0.0, 500.0, 0.0),
            (0.0, 0.0, 500.0, 0.0),
            (0.0, 0.0, 500.0, 0.0),
        ]
        assert gateway.services == [("set_armed", False)]
        assert aborts == [
            "safety_runtime_exception:RuntimeError:boom"
        ]
    finally:
        assert executor.close(timeout=0.5)
