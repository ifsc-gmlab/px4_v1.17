import json
import math
import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest

from flying_car_test.controller import ExperimentController
from flying_car_test.models import (
    Action,
    CommandsConfig,
    ExperimentConfig,
    ManualControl,
    Phase,
    SafetyConfig,
    Snapshot,
    TimeoutsConfig,
    TolerancesConfig,
    TrajectoryConfig,
)
from flying_car_test.px4_gateway import Px4Gateway


class Header:
    def __init__(self):
        self.stamp = None


class ManualControlMessage:
    def __init__(self):
        self.header = Header()
        self.x = self.y = self.z = self.r = 0.0
        self.buttons = 0


class PoseStampedMessage:
    def __init__(self):
        self.header = Header()
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class ParamValueMessage:
    def __init__(self, integer=0, real=0.0):
        self.integer = integer
        self.real = real


class ParamGet:
    pass


class ParamSet:
    pass


class SetMode:
    pass


class CommandBool:
    pass


class StateMessage:
    pass


class ExtendedStateMessage:
    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2


class LocalPoseMessage:
    pass


class LocalVelocityMessage:
    pass


class ModelStatesMessage:
    pass


class FakeBindings:
    Mavlink = object
    ManualControl = ManualControlMessage
    PoseStamped = PoseStampedMessage
    ParamValue = ParamValueMessage
    ParamGet = ParamGet
    ParamSet = ParamSet
    SetMode = SetMode
    CommandBool = CommandBool
    State = StateMessage
    ExtendedState = ExtendedStateMessage
    LocalPose = LocalPoseMessage
    LocalVelocity = LocalVelocityMessage
    ModelStates = ModelStatesMessage


class FakeGateway:
    def __init__(self):
        self.calls = []
        self.responses = {
            "get_mav_type": 10,
            "set_mav_type": True,
            "set_mode": True,
            "set_armed": True,
        }
        self.last_response_fields = {}

    def get_mav_type(self):
        self.calls.append(("get_mav_type",))
        return self.responses["get_mav_type"]

    def set_mav_type(self, value):
        self.calls.append(("set_mav_type", value))
        return self.responses["set_mav_type"]

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        return self.responses["set_mode"]

    def set_armed(self, armed):
        self.calls.append(("set_armed", armed))
        return self.responses["set_armed"]

    def publish_manual(self, x, y, z, r):
        self.calls.append(("publish_manual", x, y, z, r))

    def publish_local_pose(self, x, y, z, yaw):
        self.calls.append(("publish_local_pose", x, y, z, yaw))


class RejectingParamGateway(FakeGateway):
    def get_mav_type(self):
        self.calls.append(("get_mav_type",))
        self.last_response_fields = {"success": False}
        raise RuntimeError("ParamGet rejected")


class OperationResponseGateway(FakeGateway):
    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        self.last_response_fields = {"success": True, "integer": 14}
        return False

    def response_fields(self, operation):
        assert operation == "set_mode"
        return {"mode_sent": False}


class ConcurrentRejectingParamGateway(RejectingParamGateway):
    def __init__(self):
        super().__init__()
        self.get_started = threading.Event()

    def get_mav_type(self):
        self.get_started.set()
        return super().get_mav_type()


class BlockingParamGateway(FakeGateway):
    def __init__(self):
        super().__init__()
        self.get_started = threading.Event()
        self.release_get = threading.Event()

    def get_mav_type(self):
        self.get_started.set()
        self.release_get.wait(timeout=1.0)
        self.calls.append(("get_mav_type",))
        return 14


class BlockingRejectingParamGateway(RejectingParamGateway):
    def __init__(self):
        super().__init__()
        self.get_started = threading.Event()
        self.release_get = threading.Event()

    def get_mav_type(self):
        self.get_started.set()
        self.release_get.wait(timeout=1.0)
        return super().get_mav_type()


class BlockingSafetyLandGateway(RejectingParamGateway):
    def __init__(self):
        super().__init__()
        self.land_started = threading.Event()
        self.release_land = threading.Event()

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        self.land_started.set()
        self.release_land.wait(timeout=1.0)
        self.last_response_fields = {"mode_sent": True}
        return True


class BlockingServicesGateway(FakeGateway):
    def __init__(self):
        super().__init__()
        self.mode_started = threading.Event()
        self.release_mode = threading.Event()

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        self.mode_started.set()
        self.release_mode.wait(timeout=1.0)
        return self.responses["set_mode"]


class BlockingLandingResultGateway(BlockingServicesGateway):
    def __init__(self, raises):
        super().__init__()
        self.raises = raises

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        self.mode_started.set()
        self.release_mode.wait(timeout=1.0)
        if self.raises:
            raise RuntimeError("landing transport failed")
        self.last_response_fields = {"mode_sent": False}
        return False


class RaisingShutdownGateway(FakeGateway):
    def set_armed(self, armed):
        self.calls.append(("set_armed", armed))
        raise RuntimeError("arming transport failed")


class RaisingModeGateway(FakeGateway):
    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        raise RuntimeError("mode transport failed")


class RejectingSafetyLandGateway(RejectingParamGateway):
    def __init__(self, raises):
        super().__init__()
        self.raises = raises

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        if self.raises:
            raise RuntimeError("landing transport failed")
        self.last_response_fields = {"mode_sent": False}
        return False


class FakeStamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def __eq__(self, other):
        return isinstance(other, FakeStamp) and self.seconds == other.seconds


class FakePublisher:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeRos:
    def __init__(self):
        self.now = FakeStamp(12.5)
        self.publishers = {}
        self.service_types = {}
        self.service_calls = []
        self.subscribers = {}
        self.mav_type = 14
        self.param_get_success = True
        self.Time = SimpleNamespace(now=lambda: self.now)

    def Publisher(self, topic, message_type, queue_size):
        publisher = FakePublisher(topic, message_type)
        self.publishers[topic] = publisher
        return publisher

    def ServiceProxy(self, name, service_type):
        self.service_types[name] = service_type

        def call(**kwargs):
            self.service_calls.append((name, kwargs))
            if name == "/mavros/param/get":
                return SimpleNamespace(
                    success=self.param_get_success,
                    value=ParamValueMessage(integer=self.mav_type, real=0.0),
                )
            if name == "/mavros/param/set":
                self.mav_type = kwargs["value"].integer
                return SimpleNamespace(success=True, value=kwargs["value"])
            if name == "/mavros/set_mode":
                return SimpleNamespace(mode_sent=True)
            return SimpleNamespace(success=True, result=0)

        return call

    def Subscriber(self, topic, message_type, callback, queue_size):
        self.subscribers[topic] = (message_type, callback)
        return SimpleNamespace()

    def emit(self, topic, message):
        self.subscribers[topic][1](message)


class ActionMachine:
    def __init__(self, actions=(), phase=Phase.PRECHECK):
        self.actions = list(actions)
        self.phase = phase
        self.advance_calls = []

    def advance(self, snapshot, now):
        self.advance_calls.append((snapshot, now))
        return list(self.actions)

    def enter(self, phase, now):
        self.phase = phase


class BlockingActionMachine(ActionMachine):
    def __init__(self):
        super().__init__(phase=Phase.GROUND_CONFIGURE)
        self.advance_started = threading.Event()
        self.release_advance = threading.Event()

    def advance(self, snapshot, now):
        self.advance_started.set()
        self.release_advance.wait(timeout=1.0)
        self.phase = Phase.GROUND_FORWARD
        return [Action("arm", {})]


@pytest.fixture
def config():
    neutral = ManualControl(0.0, 0.0, 500.0, 0.0)
    return ExperimentConfig(
        publish_rate_hz=30.0,
        model_name="test_vehicle",
        trajectory=TrajectoryConfig(5.0, 5.0, 5.0, 3.0, 3.0, 3.0),
        tolerances=TolerancesConfig(1.0, 1.0, 0.75, 1.0, 1.5, 0.75, 1.5, 1.5),
        timeouts=TimeoutsConfig(30.0, 5.0, 30.0, 30.0, 20.0, 45.0, 40.0),
        commands=CommandsConfig(
            manual_neutral=neutral,
            ground_neutral=ManualControl(0.0, 0.0, 0.0, 0.0),
            ground_forward=ManualControl(0.0, 0.0, 650.0, 0.0),
            ground_left_turn=ManualControl(0.0, 500.0, 500.0, 0.0),
            air_manual_forward=ManualControl(150.0, 0.0, 500.0, 0.0),
            air_manual_duration_s=1.0,
            air_manual_brake_s=2.0,
        ),
        safety=SafetyConfig(0.3, 30.0, 45.0, 8.0, 10.0, 0.15),
    )


@pytest.fixture
def snapshot():
    return Snapshot(
        stamp=0.0,
        connected=True,
        armed=False,
        mode="MANUAL",
        landed=True,
        mav_type=10,
        x=10.0,
        y=20.0,
        z=2.0,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        clock_ok=True,
        processes_ok=True,
    )


def make_controller(config, gateway, machine):
    telemetry = {"heartbeat": [], "phase": [], "command": [], "abort": []}
    controller = ExperimentController(
        config,
        machine,
        gateway,
        publish_heartbeat=telemetry["heartbeat"].append,
        publish_phase=telemetry["phase"].append,
        publish_command=telemetry["command"].append,
        publish_abort=telemetry["abort"].append,
    )
    return controller, telemetry


def test_streams_neutral_before_state_machine_selects_manual_command(
    config, snapshot
):
    gateway = FakeGateway()
    controller, _ = make_controller(config, gateway, ActionMachine())

    controller.tick(snapshot, now=0.0)

    assert gateway.calls[0] == ("publish_manual", 0.0, 0.0, 500.0, 0.0)


def test_missing_feedback_still_streams_neutral_and_tick_telemetry(config):
    gateway = FakeGateway()
    machine = ActionMachine()
    controller, telemetry = make_controller(config, gateway, machine)

    controller.tick(None, now=0.0)

    assert gateway.calls == [
        ("publish_manual", 0.0, 0.0, 500.0, 0.0)
    ]
    assert machine.advance_calls == []
    assert telemetry["heartbeat"] == [0.0]
    assert telemetry["phase"] == ["PRECHECK"]
    assert len(telemetry["command"]) == 1


def test_offboard_pose_streams_for_one_second_before_mode_request(
    config, snapshot
):
    actions = [
        Action("position", {"x": 1.0, "y": 2.0, "z": 7.0, "yaw": 0.25}),
        Action("set_mode", {"mode": "OFFBOARD"}),
    ]
    gateway = FakeGateway()
    controller, _ = make_controller(
        config, gateway, ActionMachine(actions, Phase.OFFBOARD_PRIME)
    )

    for now in (0.0, 0.5, 1.49):
        controller.tick(replace(snapshot, stamp=now), now=now)
    assert ("set_mode", "OFFBOARD") not in gateway.calls

    controller.tick(replace(snapshot, stamp=1.5), now=1.5)
    assert controller.wait_for_services(timeout=1.0)

    pose_calls = [call for call in gateway.calls if call[0] == "publish_local_pose"]
    assert pose_calls == [
        ("publish_local_pose", 1.0, 2.0, 7.0, 0.25),
        ("publish_local_pose", 1.0, 2.0, 7.0, 0.25),
        ("publish_local_pose", 1.0, 2.0, 7.0, 0.25),
    ]
    assert gateway.calls[-1] == ("set_mode", "OFFBOARD")


@pytest.mark.parametrize(
    ("action", "response_key", "service_name"),
    [
        (Action("set_mode", {"mode": "MANUAL"}), "set_mode", "/mavros/set_mode"),
        (
            Action("set_mav_type", {"mav_type": 14}),
            "set_mav_type",
            "/mavros/param/set",
        ),
        (Action("arm", {}), "set_armed", "/mavros/cmd/arming"),
    ],
)
def test_rejected_service_aborts_once_without_retry_storm(
    config, snapshot, action, response_key, service_name
):
    gateway = FakeGateway()
    gateway.responses[response_key] = False
    gateway.last_response_fields = {"success": False, "result": 4}
    machine = ActionMachine([action], Phase.GROUND_CONFIGURE)
    controller, telemetry = make_controller(config, gateway, machine)

    for now in (0.0, 0.1, 0.2):
        controller.tick(replace(snapshot, stamp=now), now=now)
    assert controller.wait_for_services(timeout=1.0)

    service_calls = [
        call
        for call in gateway.calls
        if call[0] in ("set_mode", "set_mav_type", "set_armed")
    ]
    event = json.loads(telemetry["abort"][0])
    assert len(service_calls) == 1
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert event["service"] == service_name
    assert event["response"] == {"success": False, "result": 4}


def test_rejected_param_get_aborts_once_without_periodic_retries(
    config, snapshot
):
    gateway = RejectingParamGateway()
    machine = ActionMachine(phase=Phase.PRECHECK)
    controller, telemetry = make_controller(config, gateway, machine)

    controller.refresh_mav_type(now=1.0)
    controller.refresh_mav_type(now=2.0)
    assert controller.wait_for_services(timeout=1.0)

    assert gateway.calls == [("get_mav_type",)]
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert json.loads(telemetry["abort"][0]) == {
        "stamp": 1.0,
        "reason": "service_exception",
        "service": "/mavros/param/get",
        "operation": "get_mav_type",
        "response": {
            "success": False,
            "exception": "ParamGet rejected",
        },
    }


def test_abort_uses_response_fields_bound_to_rejected_operation(
    config, snapshot
):
    gateway = OperationResponseGateway()
    machine = ActionMachine(
        [Action("set_mode", {"mode": "MANUAL"})],
        Phase.GROUND_CONFIGURE,
    )
    controller, telemetry = make_controller(config, gateway, machine)

    controller.tick(snapshot, now=0.0)
    assert controller.wait_for_services(timeout=1.0)

    event = json.loads(telemetry["abort"][0])
    assert event["response"] == {"mode_sent": False}


def test_parameter_refresh_queues_abort_for_next_state_machine_tick(
    config, snapshot
):
    gateway = ConcurrentRejectingParamGateway()
    machine = BlockingActionMachine()
    controller, telemetry = make_controller(config, gateway, machine)
    tick_thread = threading.Thread(
        target=controller.tick, args=(snapshot, 0.0)
    )
    refresh_thread = threading.Thread(
        target=controller.refresh_mav_type, args=(0.1,)
    )

    tick_thread.start()
    assert machine.advance_started.wait(timeout=1.0)
    refresh_thread.start()
    refresh_ran_during_advance = gateway.get_started.wait(timeout=0.05)
    machine.release_advance.set()
    tick_thread.join(timeout=1.0)
    refresh_thread.join(timeout=1.0)

    assert refresh_ran_during_advance
    assert not tick_thread.is_alive()
    assert not refresh_thread.is_alive()
    assert machine.phase == Phase.ABORT
    assert [
        call for call in gateway.calls if call[0] == "get_mav_type"
    ] == [("get_mav_type",)]
    assert ("set_armed", True) not in gateway.calls
    assert len(telemetry["abort"]) == 1


def test_blocked_parameter_refresh_does_not_block_control_tick(
    config, snapshot
):
    gateway = BlockingParamGateway()
    controller, _ = make_controller(config, gateway, ActionMachine())
    refresh_thread = threading.Thread(
        target=controller.refresh_mav_type, args=(0.0,)
    )
    tick_thread = threading.Thread(
        target=controller.tick, args=(snapshot, 0.1)
    )

    refresh_thread.start()
    assert gateway.get_started.wait(timeout=1.0)
    tick_thread.start()
    tick_thread.join(timeout=0.1)
    tick_completed_while_param_get_blocked = not tick_thread.is_alive()
    gateway.release_get.set()
    refresh_thread.join(timeout=1.0)
    tick_thread.join(timeout=1.0)

    assert tick_completed_while_param_get_blocked
    assert not refresh_thread.is_alive()


def test_blocking_service_does_not_interrupt_30hz_stream(config, snapshot):
    gateway = BlockingServicesGateway()
    machine = ActionMachine(
        [Action("set_mode", {"mode": "MANUAL"})],
        Phase.GROUND_CONFIGURE,
    )
    controller, telemetry = make_controller(config, gateway, machine)

    controller.control_tick(snapshot, now=0.0)
    assert gateway.mode_started.wait(timeout=1.0)
    for index in range(6):
        controller.stream_tick(now=index / 30.0)

    manual_calls = [
        call for call in gateway.calls if call[0] == "publish_manual"
    ]
    assert len(manual_calls) == 6
    assert len(telemetry["heartbeat"]) == 6
    assert len(telemetry["command"]) == 6
    gateway.release_mode.set()
    assert controller.wait_for_services(timeout=1.0)


@pytest.mark.parametrize(
    ("actions", "stream_call"),
    [
        ([Action("set_mode", {"mode": "MANUAL"})], "publish_manual"),
        (
            [
                Action(
                    "position",
                    {"x": 1.0, "y": 2.0, "z": 7.0, "yaw": 0.25},
                ),
                Action("set_mode", {"mode": "MANUAL"}),
            ],
            "publish_local_pose",
        ),
    ],
)
def test_real_stream_loop_stays_above_20hz_while_service_is_blocked(
    config, snapshot, actions, stream_call
):
    gateway = BlockingServicesGateway()
    machine = ActionMachine(actions, Phase.GROUND_CONFIGURE)
    controller, telemetry = make_controller(config, gateway, machine)

    controller.start_streaming()
    controller.control_tick(snapshot, now=0.0)
    assert gateway.mode_started.wait(timeout=1.0)
    measure_started = time.monotonic()
    threading.Event().wait(0.3)
    measure_ended = time.monotonic()
    streamed_while_blocked = [
        stamp
        for stamp in telemetry["heartbeat"]
        if measure_started <= stamp <= measure_ended
    ]

    assert not gateway.release_mode.is_set()
    gateway.release_mode.set()
    assert controller.wait_for_services(timeout=1.0)
    assert controller.stop_streaming(timeout=1.0)
    elapsed = measure_ended - measure_started
    assert len(streamed_while_blocked) / elapsed >= 20.0
    assert max(
        later - earlier
        for earlier, later in zip(
            streamed_while_blocked,
            streamed_while_blocked[1:],
        )
    ) <= 0.075
    assert len(
        [call for call in gateway.calls if call[0] == stream_call]
    ) >= 6
    assert len(telemetry["phase"]) == len(telemetry["heartbeat"])
    assert len(telemetry["command"]) == len(telemetry["heartbeat"])
    assert controller.close(timeout=1.0)


def test_param_get_and_state_machine_services_share_one_serial_worker(
    config, snapshot
):
    gateway = BlockingParamGateway()
    machine = ActionMachine(
        [Action("set_mode", {"mode": "MANUAL"})],
        Phase.GROUND_CONFIGURE,
    )
    controller, _ = make_controller(config, gateway, machine)

    assert controller.refresh_mav_type(now=0.0)
    assert gateway.get_started.wait(timeout=1.0)
    controller.control_tick(snapshot, now=0.1)

    assert ("set_mode", "MANUAL") not in gateway.calls
    gateway.release_get.set()
    assert controller.wait_for_services(timeout=1.0)
    assert gateway.calls == [
        ("get_mav_type",),
        ("set_mode", "MANUAL"),
    ]
    assert controller.close(timeout=1.0)


def test_shutdown_streams_above_20hz_while_queued_service_is_blocked(
    config, snapshot
):
    gateway = BlockingRejectingParamGateway()
    machine = ActionMachine(phase=Phase.GROUND_FORWARD)
    controller, telemetry = make_controller(config, gateway, machine)
    controller.start_streaming()
    assert controller.refresh_mav_type(now=0.0)
    assert gateway.get_started.wait(timeout=1.0)
    shutdown_thread = threading.Thread(
        target=controller.shutdown,
        args=(replace(snapshot, armed=True),),
    )

    shutdown_thread.start()
    measure_started = time.monotonic()
    threading.Event().wait(0.3)
    measure_ended = time.monotonic()
    streamed_while_blocked = [
        stamp
        for stamp in telemetry["heartbeat"]
        if measure_started <= stamp <= measure_ended
    ]
    assert not gateway.release_get.is_set()
    gateway.release_get.set()
    shutdown_thread.join(timeout=1.0)

    assert not shutdown_thread.is_alive()
    assert (
        len(streamed_while_blocked) / (measure_ended - measure_started)
        >= 20.0
    )
    assert ("set_armed", False) not in gateway.calls
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert controller.close(timeout=1.0)
    assert controller.stop_streaming(timeout=1.0)


def test_service_actions_are_serialized(config, snapshot):
    gateway = BlockingServicesGateway()
    machine = ActionMachine(
        [
            Action("set_mode", {"mode": "MANUAL"}),
            Action("arm", {}),
        ],
        Phase.GROUND_CONFIGURE,
    )
    controller, _ = make_controller(config, gateway, machine)

    controller.control_tick(snapshot, now=0.0)
    assert gateway.mode_started.wait(timeout=1.0)
    assert ("set_armed", True) not in gateway.calls
    gateway.release_mode.set()
    assert controller.wait_for_services(timeout=1.0)

    service_calls = [
        call
        for call in gateway.calls
        if call[0] in ("set_mode", "set_armed")
    ]
    assert service_calls == [
        ("set_mode", "MANUAL"),
        ("set_armed", True),
    ]


def test_first_service_failure_skips_remaining_same_tick_actions(
    config, snapshot
):
    gateway = FakeGateway()
    gateway.responses["set_mode"] = False
    gateway.last_response_fields = {"mode_sent": False}
    machine = ActionMachine(
        [
            Action("set_mode", {"mode": "MANUAL"}),
            Action("arm", {}),
        ],
        Phase.GROUND_CONFIGURE,
    )
    controller, telemetry = make_controller(config, gateway, machine)

    controller.control_tick(snapshot, now=0.0)
    assert controller.wait_for_services(timeout=1.0)

    assert ("set_mode", "MANUAL") in gateway.calls
    assert ("set_armed", True) not in gateway.calls
    assert len(telemetry["abort"]) == 1


@pytest.mark.parametrize("raises", [False, True])
def test_rejected_ground_shutdown_aborts_once_and_stays_neutral(
    config, snapshot, raises
):
    gateway = RaisingShutdownGateway() if raises else FakeGateway()
    if not raises:
        gateway.responses["set_armed"] = False
        gateway.last_response_fields = {"success": False, "result": 4}
    machine = ActionMachine(phase=Phase.GROUND_FORWARD)
    controller, telemetry = make_controller(config, gateway, machine)

    accepted = controller.shutdown(replace(snapshot, armed=True))
    controller.shutdown(replace(snapshot, armed=True))
    controller.stream_tick(now=1.0)

    assert accepted is False
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert len(
        [call for call in gateway.calls if call == ("set_armed", False)]
    ) == 1
    assert gateway.calls[-1] == (
        "publish_manual",
        0.0,
        0.0,
        500.0,
        0.0,
    )
    event = json.loads(telemetry["abort"][0])
    assert event["service"] == "/mavros/cmd/arming"
    assert event["operation"] == "set_armed"
    assert event["response"] == (
        {"exception": "arming transport failed"}
        if raises
        else {"success": False, "result": 4}
    )


@pytest.mark.parametrize("raises", [False, True])
def test_rejected_air_shutdown_aborts_once_and_streams_position_hold(
    config, snapshot, raises
):
    gateway = RaisingModeGateway() if raises else FakeGateway()
    if not raises:
        gateway.responses["set_mode"] = False
        gateway.last_response_fields = {"mode_sent": False}
    airborne = replace(
        snapshot,
        landed=False,
        armed=True,
        x=4.0,
        y=5.0,
        z=7.0,
        yaw=0.4,
    )
    machine = ActionMachine(phase=Phase.LAND)
    controller, telemetry = make_controller(config, gateway, machine)

    accepted = controller.shutdown(airborne)
    controller.stream_tick(now=1.0)

    assert accepted is False
    assert gateway.calls[0] == (
        "publish_local_pose",
        4.0,
        5.0,
        7.0,
        0.4,
    )
    assert gateway.calls[1] == ("set_mode", "AUTO.LAND")
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert gateway.calls[-1] == (
        "publish_local_pose",
        4.0,
        5.0,
        7.0,
        0.4,
    )
    event = json.loads(telemetry["abort"][0])
    assert event["service"] == "/mavros/set_mode"
    assert event["operation"] == "set_mode"
    assert event["response"] == (
        {"exception": "mode transport failed"}
        if raises
        else {"mode_sent": False}
    )


def test_param_get_failure_immediately_stops_ground_motion_before_next_advance(
    config, snapshot
):
    gateway = RejectingParamGateway()
    machine = ActionMachine(
        [
            Action(
                "manual",
                {
                    "name": "ground_forward",
                    "command": config.commands.ground_forward,
                },
            )
        ],
        Phase.GROUND_FORWARD,
    )
    controller, telemetry = make_controller(config, gateway, machine)
    moving = replace(snapshot, armed=True)
    controller.control_tick(moving, now=0.0)
    controller.stream_tick(now=0.1)
    assert gateway.calls[-1][3] == 650.0
    advance_count = len(machine.advance_calls)

    controller.refresh_mav_type(now=0.2)
    assert controller.wait_for_services(timeout=1.0)
    controller.stream_tick(now=0.3)
    controller.control_tick(moving, now=0.3)

    assert machine.phase == Phase.ABORT
    assert len(machine.advance_calls) == advance_count
    assert len(telemetry["abort"]) == 1
    assert gateway.calls[-1] == (
        "publish_manual",
        0.0,
        0.0,
        500.0,
        0.0,
    )


def test_async_failure_replaces_air_motion_with_position_hold(config, snapshot):
    gateway = RejectingParamGateway()
    airborne = replace(
        snapshot,
        landed=False,
        armed=True,
        mode="MANUAL",
        x=8.0,
        y=9.0,
        z=7.0,
        yaw=0.6,
    )
    machine = ActionMachine(
        [
            Action(
                "manual",
                {
                    "name": "air_forward",
                    "command": config.commands.air_manual_forward,
                },
            )
        ],
        Phase.MANUAL_AIR,
    )
    controller, _ = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)
    controller.stream_tick(now=0.1)
    assert gateway.calls[-1][1] == 150.0

    controller.refresh_mav_type(now=0.2)
    assert controller.wait_for_services(timeout=1.0)
    controller.stream_tick(now=0.3)

    assert machine.phase == Phase.ABORT
    assert gateway.calls[-1] == (
        "publish_local_pose",
        8.0,
        9.0,
        7.0,
        0.6,
    )
    assert ("set_mode", "AUTO.LAND") in gateway.calls


def test_airborne_state_machine_abort_holds_and_requests_auto_land(
    config, snapshot
):
    gateway = FakeGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(
        [Action("abort", {"reason": "hard_tilt"})],
        Phase.MANUAL_AIR,
    )
    controller, telemetry = make_controller(config, gateway, machine)

    controller.control_tick(airborne, now=0.0)
    assert controller.wait_for_services(timeout=1.0)
    controller.stream_tick(now=0.1)

    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    assert ("set_mode", "AUTO.LAND") in gateway.calls
    assert gateway.calls[-1] == (
        "publish_local_pose",
        10.0,
        20.0,
        7.0,
        0.0,
    )


def test_airborne_abort_reuses_in_flight_auto_land(config, snapshot):
    gateway = BlockingServicesGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(
        [Action("set_mode", {"mode": "AUTO.LAND"})],
        Phase.LAND,
    )
    controller, telemetry = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)
    assert gateway.mode_started.wait(timeout=1.0)
    machine.actions = [Action("abort", {"reason": "hard_tilt"})]

    controller.control_tick(airborne, now=0.1)
    gateway.release_mode.set()
    assert controller.wait_for_services(timeout=1.0)

    assert [
        call for call in gateway.calls if call == ("set_mode", "AUTO.LAND")
    ] == [("set_mode", "AUTO.LAND")]
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1


def test_airborne_abort_reuses_pending_auto_land(config, snapshot):
    gateway = BlockingParamGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(
        [Action("set_mode", {"mode": "AUTO.LAND"})],
        Phase.LAND,
    )
    controller, telemetry = make_controller(config, gateway, machine)
    assert controller.refresh_mav_type(now=0.0)
    assert gateway.get_started.wait(timeout=1.0)
    controller.control_tick(airborne, now=0.1)
    machine.actions = [Action("abort", {"reason": "hard_tilt"})]

    controller.control_tick(airborne, now=0.2)
    gateway.release_get.set()
    assert controller.wait_for_services(timeout=1.0)

    assert [
        call for call in gateway.calls if call == ("set_mode", "AUTO.LAND")
    ] == [("set_mode", "AUTO.LAND")]
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1


def test_airborne_abort_requests_land_again_after_confirmed_mode_is_lost(
    config, snapshot
):
    gateway = FakeGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(
        [Action("set_mode", {"mode": "AUTO.LAND"})],
        Phase.LAND,
    )
    controller, _ = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)
    assert controller.wait_for_services(timeout=1.0)
    machine.actions = []
    controller.control_tick(
        replace(airborne, stamp=0.1, mode="AUTO.LAND"),
        now=0.1,
    )
    machine.actions = [Action("abort", {"reason": "mode_loss"})]

    controller.control_tick(
        replace(airborne, stamp=0.2, mode="MANUAL"),
        now=0.2,
    )
    assert controller.wait_for_services(timeout=1.0)

    assert [
        call for call in gateway.calls if call == ("set_mode", "AUTO.LAND")
    ] == [
        ("set_mode", "AUTO.LAND"),
        ("set_mode", "AUTO.LAND"),
    ]


@pytest.mark.parametrize("raises", [False, True])
def test_in_flight_auto_land_records_result_after_abort(
    config, snapshot, raises
):
    gateway = BlockingLandingResultGateway(raises)
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(
        [Action("set_mode", {"mode": "AUTO.LAND"})],
        Phase.LAND,
    )
    controller, _ = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)
    assert gateway.mode_started.wait(timeout=1.0)
    machine.actions = [Action("abort", {"reason": "hard_tilt"})]

    controller.control_tick(airborne, now=0.1)
    gateway.release_mode.set()
    assert controller.wait_for_services(timeout=1.0)

    assert controller._safety_land_result == {
        "accepted": False,
        "service": "/mavros/set_mode",
        "operation": "set_mode",
        "response": (
            {"exception": "landing transport failed"}
            if raises
            else {"mode_sent": False}
        ),
    }
    assert [
        call for call in gateway.calls if call == ("set_mode", "AUTO.LAND")
    ] == [("set_mode", "AUTO.LAND")]


def test_service_started_on_ground_uses_latest_airborne_snapshot_for_land(
    config, snapshot
):
    gateway = BlockingRejectingParamGateway()
    machine = ActionMachine(phase=Phase.GROUND_FORWARD)
    controller, _ = make_controller(config, gateway, machine)
    controller.control_tick(snapshot, now=0.0)
    assert controller.refresh_mav_type(now=0.0)
    assert gateway.get_started.wait(timeout=1.0)
    airborne = replace(
        snapshot,
        stamp=0.1,
        landed=False,
        armed=True,
        x=8.0,
        y=9.0,
        z=7.0,
        yaw=0.6,
    )

    controller.control_tick(airborne, now=0.1)
    gateway.release_get.set()
    assert controller.wait_for_services(timeout=1.0)
    controller.stream_tick(now=0.2)

    assert ("set_mode", "AUTO.LAND") in gateway.calls
    assert gateway.calls[-1] == (
        "publish_local_pose",
        8.0,
        9.0,
        7.0,
        0.6,
    )


def test_abort_is_visible_while_safety_auto_land_is_blocked(config, snapshot):
    gateway = BlockingSafetyLandGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(phase=Phase.MANUAL_AIR)
    controller, telemetry = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)
    assert controller.refresh_mav_type(now=0.1)
    assert gateway.land_started.wait(timeout=1.0)

    controller.control_tick(airborne, now=0.2)

    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    gateway.release_land.set()
    assert controller.wait_for_services(timeout=1.0)


def test_blocked_air_failure_holds_latest_position(config, snapshot):
    gateway = BlockingRejectingParamGateway()
    machine = ActionMachine(phase=Phase.MANUAL_AIR)
    controller, _ = make_controller(config, gateway, machine)
    initial = replace(
        snapshot,
        landed=False,
        armed=True,
        x=1.0,
        y=2.0,
        z=7.0,
        yaw=0.1,
    )
    latest = replace(
        initial,
        stamp=0.1,
        x=8.0,
        y=9.0,
        yaw=0.6,
    )
    controller.control_tick(initial, now=0.0)
    assert controller.refresh_mav_type(now=0.0)
    assert gateway.get_started.wait(timeout=1.0)

    controller.control_tick(latest, now=0.1)
    gateway.release_get.set()
    assert controller.wait_for_services(timeout=1.0)
    controller.stream_tick(now=0.2)

    assert gateway.calls[-1] == (
        "publish_local_pose",
        8.0,
        9.0,
        7.0,
        0.6,
    )


@pytest.mark.parametrize("raises", [False, True])
def test_async_air_failure_checks_auto_land_result_once(
    config, snapshot, raises
):
    gateway = RejectingSafetyLandGateway(raises)
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(phase=Phase.MANUAL_AIR)
    controller, telemetry = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)

    assert controller.refresh_mav_type(now=0.1)
    assert controller.wait_for_services(timeout=1.0)
    controller.control_tick(airborne, now=0.2)
    controller.control_tick(airborne, now=0.3)

    assert [
        call
        for call in gateway.calls
        if call[0] in ("get_mav_type", "set_mode")
    ] == [
        ("get_mav_type",),
        ("set_mode", "AUTO.LAND"),
    ]
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1
    event = json.loads(telemetry["abort"][0])
    assert event["safety_operation"] == "set_mode"
    assert event["safety_service"] == "/mavros/set_mode"
    assert controller.close(timeout=1.0)


def test_failed_param_get_blocks_next_mode_and_arm_actions(config, snapshot):
    gateway = RejectingParamGateway()
    machine = ActionMachine(phase=Phase.GROUND_CONFIGURE)
    controller, _ = make_controller(config, gateway, machine)
    controller.control_tick(snapshot, now=0.0)
    advance_count = len(machine.advance_calls)
    machine.actions = [
        Action("set_mode", {"mode": "MANUAL"}),
        Action("arm", {}),
    ]

    assert controller.refresh_mav_type(now=0.1)
    assert controller.wait_for_services(timeout=1.0)
    controller.control_tick(snapshot, now=0.2)

    assert len(machine.advance_calls) == advance_count
    assert ("set_mode", "MANUAL") not in gateway.calls
    assert ("set_armed", True) not in gateway.calls


def test_service_worker_has_idempotent_termination_protocol(config):
    controller, _ = make_controller(config, FakeGateway(), ActionMachine())

    assert controller.close(timeout=1.0)
    assert controller.close(timeout=1.0)
    assert not controller.refresh_mav_type(now=1.0)


def test_close_consumes_completed_air_failure_without_retry(
    config, snapshot
):
    gateway = BlockingSafetyLandGateway()
    airborne = replace(snapshot, landed=False, armed=True, z=7.0)
    machine = ActionMachine(phase=Phase.MANUAL_AIR)
    controller, telemetry = make_controller(config, gateway, machine)
    controller.control_tick(airborne, now=0.0)

    assert controller.refresh_mav_type(now=0.1)
    assert gateway.land_started.wait(timeout=1.0)
    assert controller.shutdown(airborne) is False
    gateway.release_land.set()
    assert controller.close(timeout=1.0)

    assert [
        call
        for call in gateway.calls
        if call[0] in ("get_mav_type", "set_mode")
    ] == [
        ("get_mav_type",),
        ("set_mode", "AUTO.LAND"),
    ]
    assert machine.phase == Phase.ABORT
    assert len(telemetry["abort"]) == 1


def test_heartbeat_phase_and_command_telemetry_publish_every_tick(
    config, snapshot
):
    gateway = FakeGateway()
    machine = ActionMachine(
        [
            Action(
                "manual",
                {"name": "ground_forward", "command": config.commands.ground_forward},
            )
        ],
        Phase.GROUND_FORWARD,
    )
    controller, telemetry = make_controller(config, gateway, machine)

    controller.tick(snapshot, now=0.0)
    controller.tick(replace(snapshot, stamp=0.1), now=0.1)

    assert telemetry["heartbeat"] == [0.0, 0.1]
    assert telemetry["phase"] == ["GROUND_FORWARD", "GROUND_FORWARD"]
    assert len(telemetry["command"]) == 2
    assert json.loads(telemetry["command"][0]) == {
        "stamp": 0.0,
        "phase": "GROUND_FORWARD",
        "kind": "manual",
        "values": {"x": 0.0, "y": 0.0, "z": 500.0, "r": 0.0},
    }
    assert json.loads(telemetry["command"][1])["values"]["z"] == 650.0


def test_shutdown_on_ground_neutralizes_then_disarms(config, snapshot):
    gateway = FakeGateway()
    controller, _ = make_controller(config, gateway, ActionMachine())

    controller.shutdown(snapshot)

    assert gateway.calls == [
        ("publish_manual", 0.0, 0.0, 500.0, 0.0),
        ("set_armed", False),
    ]


def test_shutdown_without_feedback_still_sends_neutral(config):
    gateway = FakeGateway()
    controller, _ = make_controller(config, gateway, ActionMachine())

    controller.shutdown(None)

    assert gateway.calls == [
        ("publish_manual", 0.0, 0.0, 500.0, 0.0)
    ]


def test_shutdown_in_flight_requests_auto_land(config, snapshot):
    gateway = FakeGateway()
    controller, _ = make_controller(config, gateway, ActionMachine())

    controller.shutdown(replace(snapshot, landed=False, z=7.0))

    assert gateway.calls == [
        ("publish_local_pose", 10.0, 20.0, 7.0, 0.0),
        ("set_mode", "AUTO.LAND"),
    ]


@pytest.mark.parametrize(
    ("shutdown_snapshot", "service_call"),
    [
        (None, ("set_armed", False)),
        ("airborne", ("set_mode", "AUTO.LAND")),
    ],
)
def test_successful_shutdown_reentry_does_not_repeat_service(
    config, snapshot, shutdown_snapshot, service_call
):
    gateway = FakeGateway()
    controller, telemetry = make_controller(
        config,
        gateway,
        ActionMachine(),
    )
    selected = (
        replace(snapshot, armed=True)
        if shutdown_snapshot is None
        else replace(snapshot, landed=False, armed=True, z=7.0)
    )

    assert controller.shutdown(selected)
    assert not controller.shutdown(selected)

    assert [call for call in gateway.calls if call == service_call] == [
        service_call
    ]
    assert telemetry["abort"] == []


def test_gateway_uses_exact_mavros_bindings_and_reads_back_mav_type():
    ros = FakeRos()
    gateway = Px4Gateway("test_vehicle", ros=ros, bindings=FakeBindings)

    assert set(ros.service_types) == {
        "/mavros/param/get",
        "/mavros/param/set",
        "/mavros/set_mode",
        "/mavros/cmd/arming",
    }
    assert ros.service_types["/mavros/param/get"].__name__ == "ParamGet"
    assert ros.service_types["/mavros/param/set"].__name__ == "ParamSet"
    assert ros.service_types["/mavros/set_mode"].__name__ == "SetMode"
    assert ros.service_types["/mavros/cmd/arming"].__name__ == "CommandBool"
    assert gateway.set_mav_type(10)
    assert [call[0] for call in ros.service_calls] == [
        "/mavros/param/set",
        "/mavros/param/get",
    ]
    assert ros.service_calls[0][1]["param_id"] == "MAV_TYPE"
    assert ros.service_calls[0][1]["value"].integer == 10
    assert gateway.last_response_fields == {
        "success": True,
        "requested": 10,
        "readback": 10,
    }


def test_rejected_real_param_get_preserves_mavros_response_in_abort(
    config
):
    ros = FakeRos()
    ros.param_get_success = False
    gateway = Px4Gateway("test_vehicle", ros=ros, bindings=FakeBindings)
    machine = ActionMachine(phase=Phase.PRECHECK)
    controller, telemetry = make_controller(config, gateway, machine)

    assert controller.refresh_mav_type(now=1.0)
    assert controller.wait_for_services(timeout=1.0)

    event = json.loads(telemetry["abort"][0])
    assert event["operation"] == "get_mav_type"
    assert event["response"] == {
        "success": False,
        "integer": 14,
        "real": 0.0,
        "exception": "MAV_TYPE ParamGet was rejected",
    }


def test_gateway_stamps_manual_and_pose_commands(config):
    ros = FakeRos()
    gateway = Px4Gateway("test_vehicle", ros=ros, bindings=FakeBindings)

    gateway.publish_manual(1.0, 2.0, 3.0, 4.0)
    gateway.publish_local_pose(5.0, 6.0, 7.0, math.pi / 2.0)

    manual = ros.publishers["/mavros/manual_control/send"].messages[0]
    pose = ros.publishers["/mavros/setpoint_position/local"].messages[0]
    assert manual.header.stamp == ros.now
    assert (manual.x, manual.y, manual.z, manual.r, manual.buttons) == (
        1.0,
        2.0,
        3.0,
        4.0,
        0,
    )
    assert pose.header.stamp == ros.now
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (
        5.0,
        6.0,
        7.0,
    )
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_gateway_assembles_snapshot_from_mavros_feedback():
    ros = FakeRos()
    gateway = Px4Gateway("test_vehicle", ros=ros, bindings=FakeBindings)
    assert gateway.get_mav_type() == 14
    ros.emit(
        "/mavlink/from",
        SimpleNamespace(
            framing_status=1,
            msgid=0,
            len=9,
            sysid=1,
            compid=1,
            payload64=[(14 << 32) | 0] + [0] * 7,
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 20.0)
            ),
        ),
    )
    ros.service_calls.clear()
    quaternion = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sqrt(0.5),
        w=math.sqrt(0.5),
    )
    ros.emit(
        "/mavros/state",
        SimpleNamespace(connected=True, armed=True, mode="OFFBOARD"),
    )
    ros.emit(
        "/mavros/extended_state",
        SimpleNamespace(
            landed_state=ExtendedStateMessage.LANDED_STATE_IN_AIR
        ),
    )
    ros.emit(
        "/mavros/local_position/pose",
        SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=quaternion,
            )
        ),
    )
    ros.emit(
        "/mavros/local_position/velocity_local",
        SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.1, y=0.2, z=0.3)
            )
        ),
    )
    ros.emit(
        "/gazebo/model_states",
        SimpleNamespace(
            name=["other", "test_vehicle"],
            pose=[
                SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
                SimpleNamespace(
                    position=SimpleNamespace(x=10.0, y=20.0, z=7.0)
                ),
            ],
        ),
    )

    result = gateway.snapshot(12.5, clock_ok=True, processes_ok=True)

    assert result == Snapshot(
        stamp=12.5,
        connected=True,
        armed=True,
        mode="OFFBOARD",
        landed=False,
        mav_type=14,
        x=1.0,
        y=2.0,
        z=3.0,
        vx=0.1,
        vy=0.2,
        vz=0.3,
        roll=0.0,
        pitch=0.0,
        yaw=pytest.approx(math.pi / 2.0),
        clock_ok=True,
        processes_ok=True,
    )
    assert ros.service_calls == []


def test_snapshot_uses_oldest_required_feedback_stamp():
    ros = FakeRos()
    gateway = Px4Gateway("test_vehicle", ros=ros, bindings=FakeBindings)
    gateway.get_mav_type()
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    messages = [
        (
            "/mavros/state",
            SimpleNamespace(connected=True, armed=False, mode="MANUAL"),
        ),
        (
            "/mavros/extended_state",
            SimpleNamespace(
                landed_state=ExtendedStateMessage.LANDED_STATE_ON_GROUND
            ),
        ),
        (
            "/mavros/local_position/pose",
            SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                    orientation=quaternion,
                )
            ),
        ),
        (
            "/mavros/local_position/velocity_local",
            SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            ),
        ),
        (
            "/gazebo/model_states",
            SimpleNamespace(
                name=["test_vehicle"],
                pose=[
                    SimpleNamespace(
                        position=SimpleNamespace(x=1.0, y=2.0, z=3.0)
                    )
                ],
            ),
        ),
    ]
    for stamp, (topic, message) in enumerate(messages, start=1):
        ros.now = FakeStamp(float(stamp))
        ros.emit(topic, message)
    ros.now = FakeStamp(20.0)
    ros.emit(
        "/mavros/state",
        SimpleNamespace(connected=True, armed=False, mode="MANUAL"),
    )
    ros.emit(
        "/mavlink/from",
        SimpleNamespace(
            framing_status=1,
            msgid=0,
            len=9,
            sysid=1,
            compid=1,
            payload64=[(14 << 32) | 0] + [0] * 7,
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 20.0)
            ),
        ),
    )

    result = gateway.snapshot(20.0, clock_ok=True, processes_ok=True)

    assert result.stamp == 2.0
