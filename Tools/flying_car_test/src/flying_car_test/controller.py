"""Automatic control loop for the flying-car experiment."""

import json
import logging
import queue
import threading
import time
from typing import Callable, Mapping, Optional, Protocol

from .models import Action, ExperimentConfig, ManualControl, Phase, Snapshot
from .state_machine import ExperimentStateMachine


logger = logging.getLogger(__name__)


class Px4GatewayProtocol(Protocol):
    """Commands required by :class:`ExperimentController`."""

    def get_mav_type(self) -> int:
        ...

    def set_mav_type(self, value: int) -> bool:
        ...

    def set_mode(self, mode: str) -> bool:
        ...

    def set_armed(self, armed: bool) -> bool:
        ...

    def publish_manual(self, x: float, y: float, z: float, r: float) -> None:
        ...

    def publish_local_pose(
        self, x: float, y: float, z: float, yaw: float
    ) -> None:
        ...


Publisher = Callable[[object], None]


def _discard(_message: object) -> None:
    pass


class ExperimentController:
    """Dispatch actions asynchronously while maintaining command streams."""

    OFFBOARD_PRIME_SECONDS = 1.0
    SHUTDOWN_SERVICE_TIMEOUT_SECONDS = 2.0
    _SERVICE_STOP = object()
    SERVICE_NAMES = {
        "get_mav_type": "/mavros/param/get",
        "set_mav_type": "/mavros/param/set",
        "set_mode": "/mavros/set_mode",
        "set_armed": "/mavros/cmd/arming",
    }

    def __init__(
        self,
        config: ExperimentConfig,
        state_machine: ExperimentStateMachine,
        gateway: Px4GatewayProtocol,
        publish_heartbeat: Publisher = _discard,
        publish_phase: Publisher = _discard,
        publish_command: Publisher = _discard,
        publish_abort: Publisher = _discard,
    ):
        self.config = config
        self.state_machine = state_machine
        self.gateway = gateway
        self.publish_heartbeat = publish_heartbeat
        self.publish_phase = publish_phase
        self.publish_command = publish_command
        self.publish_abort = publish_abort

        self._output_lock = threading.Lock()
        self._control_lock = threading.RLock()
        self._service_lock = threading.RLock()
        self._snapshot_lock = threading.Lock()
        self._active_kind = "manual"
        self._active_values = self._manual_values(config.commands.manual_neutral)
        self._pose_stream_started_at: Optional[float] = None
        self._phase_value = self.state_machine.phase.value
        self._latest_snapshot: Optional[Snapshot] = None
        self._aborted = False
        self._abort_requested = threading.Event()
        self._abort_event: Optional[Mapping[str, object]] = None
        self._service_failure_latched = False
        self._safety_land_result = None
        self._service_keys = set()
        self._waiting_feedback = set()
        self._service_queue = queue.Queue()
        self._services_closed = False
        self._service_worker = threading.Thread(
            target=self._run_service_worker,
            name="flying-car-mavros-services",
            daemon=True,
        )
        self._service_worker.start()
        self._stream_lock = threading.Lock()
        self._stream_stop = threading.Event()
        self._stream_thread = None
        self._stream_clock: Callable[[], float] = time.monotonic

    def tick(self, snapshot: Optional[Snapshot], now: float) -> None:
        """Compatibility tick: stream first, then advance control."""
        self.stream_tick(now)
        self.control_tick(snapshot, now)

    def stream_tick(self, now: float) -> None:
        """Publish command and telemetry without waiting for service work."""
        now = float(now)
        with self._output_lock:
            kind = self._active_kind
            values = dict(self._active_values)
            phase = self._phase_value
            if kind == "position":
                self.gateway.publish_local_pose(**values)
                if self._pose_stream_started_at is None:
                    self._pose_stream_started_at = now
            else:
                self.gateway.publish_manual(**values)
        self.publish_heartbeat(now)
        self.publish_phase(phase)
        self.publish_command(
            json.dumps(
                {
                    "stamp": now,
                    "phase": phase,
                    "kind": kind,
                    "values": values,
                },
                sort_keys=True,
            )
        )

    def start_streaming(
        self, clock: Optional[Callable[[], float]] = None
    ) -> bool:
        """Start the controller-owned fixed-rate publication thread."""
        with self._stream_lock:
            if self._stream_thread is not None and self._stream_thread.is_alive():
                return True
            self._stream_clock = clock or time.monotonic
            self._stream_stop.clear()
            self._stream_thread = threading.Thread(
                target=self._run_stream,
                name="flying-car-control-stream",
                daemon=True,
            )
            self._stream_thread.start()
        return True

    def stop_streaming(self, timeout: float = 1.0) -> bool:
        """Stop the fixed-rate publication thread, idempotently."""
        with self._stream_lock:
            thread = self._stream_thread
            self._stream_stop.set()
        if thread is None:
            return True
        if thread is not threading.current_thread():
            thread.join(timeout=float(timeout))
        return not thread.is_alive()

    def _run_stream(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        deadline = time.monotonic()
        while not self._stream_stop.is_set():
            self.stream_tick(self._stream_clock())
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                deadline = time.monotonic()
                continue
            self._stream_stop.wait(remaining)

    def control_tick(
        self, snapshot: Optional[Snapshot], now: float
    ) -> None:
        """Advance the state machine and enqueue service actions."""
        now = float(now)
        with self._control_lock:
            if snapshot is not None:
                self._remember_snapshot(snapshot)
                self._release_confirmed_services(snapshot)
            self._consume_requested_abort()
            if (
                snapshot is None
                or self._aborted
                or self._abort_requested.is_set()
            ):
                self._set_phase_value(self.state_machine.phase.value)
                return
            actions = self.state_machine.advance(snapshot, now)
            if self._abort_requested.is_set():
                self._consume_requested_abort()
                return
            for action in actions:
                if self._abort_requested.is_set():
                    self._consume_requested_abort()
                    break
                if not self._dispatch(action, now, snapshot):
                    break
            self._consume_requested_abort()
            self._set_phase_value(self.state_machine.phase.value)

    def refresh_mav_type(self, now: float) -> bool:
        """Queue one serialized MAV_TYPE read without blocking the caller."""
        return self._submit_service(
            "get_mav_type",
            self.gateway.get_mav_type,
            None,
            float(now),
            self._latest_feedback_snapshot(),
        )

    def shutdown(self, snapshot: Optional[Snapshot]) -> bool:
        """Neutralize/hold, then validate the required shutdown service."""
        if snapshot is not None:
            self._remember_snapshot(snapshot)
        self._activate_safe_output(snapshot)
        if snapshot is None:
            self.gateway.publish_manual(
                **self._manual_values(self.config.commands.manual_neutral)
            )
            self._consume_requested_abort()
            return not self._abort_requested.is_set()
        if snapshot.landed:
            self.gateway.publish_manual(
                **self._manual_values(self.config.commands.manual_neutral)
            )
            operation = "set_armed"
            function = self.gateway.set_armed
            argument = False
        else:
            self.gateway.publish_local_pose(
                snapshot.x,
                snapshot.y,
                snapshot.z,
                snapshot.yaw,
            )
            operation = "set_mode"
            function = self.gateway.set_mode
            argument = "AUTO.LAND"
        self._consume_requested_abort()
        if self._abort_requested.is_set() or self._aborted:
            return False
        completion = threading.Event()
        result = []
        submitted = self._submit_service(
            operation,
            function,
            argument,
            float(snapshot.stamp),
            snapshot,
            completion=completion,
            result=result,
        )
        if not submitted:
            return False
        if not completion.wait(
            timeout=self.SHUTDOWN_SERVICE_TIMEOUT_SECONDS
        ):
            self._request_abort(
                {
                    "stamp": float(snapshot.stamp),
                    "reason": "service_timeout",
                    "service": self.SERVICE_NAMES[operation],
                    "operation": operation,
                    "response": {
                        "timeout_s": self.SHUTDOWN_SERVICE_TIMEOUT_SECONDS
                    },
                },
                snapshot,
            )
            self._consume_requested_abort()
            return False
        self._consume_requested_abort()
        return bool(result and result[0])

    def wait_for_services(self, timeout: float = 1.0) -> bool:
        """Wait for queued work in tests and orderly shutdown paths."""
        deadline = time.monotonic() + float(timeout)
        while self._service_queue.unfinished_tasks:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.001)
        self._consume_requested_abort()
        return True

    def close(self, timeout: float = 1.0) -> bool:
        """Stop accepting service work and terminate the worker."""
        with self._service_lock:
            if not self._services_closed:
                self._services_closed = True
                self._service_queue.put(self._SERVICE_STOP)
            worker = self._service_worker
        if worker is not threading.current_thread():
            worker.join(timeout=float(timeout))
        stopped = not worker.is_alive()
        if stopped:
            self._consume_requested_abort()
        return stopped

    def _dispatch(
        self, action: Action, now: float, snapshot: Snapshot
    ) -> bool:
        with self._service_lock:
            if self._service_failure_latched:
                return False
            if action.kind == "manual":
                command = action.values["command"]
                with self._output_lock:
                    self._active_kind = "manual"
                    self._active_values = self._manual_values(command)
                    self._pose_stream_started_at = None
                    self._phase_value = self.state_machine.phase.value
                return True
            if action.kind == "position":
                with self._output_lock:
                    self._active_kind = "position"
                    self._active_values = {
                        name: float(action.values[name])
                        for name in ("x", "y", "z", "yaw")
                    }
                    self._phase_value = self.state_machine.phase.value
                return True
            if action.kind == "set_mode":
                mode = str(action.values["mode"])
                if (
                    mode == "OFFBOARD"
                    and not self._offboard_stream_is_primed(now)
                ):
                    return True
                return self._submit_service_locked(
                    "set_mode",
                    self.gateway.set_mode,
                    mode,
                    now,
                    snapshot,
                )
            if action.kind == "set_mav_type":
                return self._submit_service_locked(
                    "set_mav_type",
                    self.gateway.set_mav_type,
                    int(action.values["mav_type"]),
                    now,
                    snapshot,
                )
            if action.kind == "arm":
                return self._submit_service_locked(
                    "set_armed",
                    self.gateway.set_armed,
                    True,
                    now,
                    snapshot,
                )
            if action.kind == "disarm":
                return self._submit_service_locked(
                    "set_armed",
                    self.gateway.set_armed,
                    False,
                    now,
                    snapshot,
                )
        if action.kind == "abort":
            event = {
                "stamp": now,
                "reason": str(action.values["reason"]),
            }
            self._enter_abort(now, event, snapshot)
            self._submit_safety_land(snapshot, now)
            return False
        event = {
            "stamp": now,
            "reason": "unknown_action",
            "action": action.kind,
        }
        self._enter_abort(now, event, snapshot)
        self._submit_safety_land(snapshot, now)
        return False

    def _offboard_stream_is_primed(self, now: float) -> bool:
        with self._output_lock:
            return (
                self._active_kind == "position"
                and self._pose_stream_started_at is not None
                and now - self._pose_stream_started_at
                >= self.OFFBOARD_PRIME_SECONDS
            )

    def _submit_service(
        self,
        operation: str,
        function: Callable,
        argument: object,
        now: float,
        snapshot: Optional[Snapshot],
        completion: Optional[threading.Event] = None,
        result: Optional[list] = None,
    ) -> bool:
        with self._service_lock:
            return self._submit_service_locked(
                operation,
                function,
                argument,
                now,
                snapshot,
                completion,
                result,
            )

    def _submit_service_locked(
        self,
        operation: str,
        function: Callable,
        argument: object,
        now: float,
        snapshot: Optional[Snapshot],
        completion: Optional[threading.Event] = None,
        result: Optional[list] = None,
    ) -> bool:
        key = (operation, argument)
        if (
            self._services_closed
            or self._service_failure_latched
            or self._aborted
            or key in self._service_keys
            or key in self._waiting_feedback
        ):
            if result is not None:
                result.append(False)
            if completion is not None:
                completion.set()
            return False
        task = {
            "operation": operation,
            "function": function,
            "argument": argument,
            "now": float(now),
            "snapshot": snapshot,
            "key": key,
            "completion": completion,
            "result": result,
            "safety": False,
        }
        self._service_keys.add(key)
        self._service_queue.put(task)
        return True

    def _submit_safety_land(
        self, snapshot: Optional[Snapshot], now: float
    ) -> bool:
        if snapshot is None or snapshot.landed:
            return False
        with self._service_lock:
            if self._services_closed:
                return False
            key = ("set_mode", "AUTO.LAND")
            if (
                key in self._service_keys
                or key in self._waiting_feedback
                or snapshot.mode == "AUTO.LAND"
            ):
                return True
            task = {
                "operation": "set_mode",
                "function": self.gateway.set_mode,
                "argument": "AUTO.LAND",
                "now": float(now),
                "snapshot": snapshot,
                "key": key,
                "completion": None,
                "result": None,
                "safety": True,
            }
            self._service_keys.add(key)
            self._service_queue.put(task)
        return True

    def _run_service_worker(self) -> None:
        while True:
            task = self._service_queue.get()
            if task is self._SERVICE_STOP:
                self._service_queue.task_done()
                return
            accepted = False
            try:
                landing = (
                    task["operation"] == "set_mode"
                    and task["argument"] == "AUTO.LAND"
                )
                with self._service_lock:
                    safety = bool(task["safety"])
                    skip = (
                        self._service_failure_latched or self._aborted
                    ) and not safety and not landing
                if not skip:
                    accepted, response = self._invoke_service(
                        task["operation"],
                        task["function"],
                        task["argument"],
                    )
                    if landing:
                        with self._service_lock:
                            self._safety_land_result = {
                                "accepted": accepted,
                                "service": self.SERVICE_NAMES["set_mode"],
                                "operation": "set_mode",
                                "response": dict(response),
                            }
                    if safety:
                        pass
                    elif not accepted:
                        self._handle_service_failure(
                            task,
                            response,
                        )
                    elif task["operation"] != "get_mav_type":
                        with self._service_lock:
                            self._waiting_feedback.add(task["key"])
            finally:
                with self._service_lock:
                    self._service_keys.discard(task["key"])
            if task["result"] is not None:
                task["result"].append(accepted)
            if task["completion"] is not None:
                task["completion"].set()
            self._service_queue.task_done()

    def _invoke_service(
        self,
        operation: str,
        function: Callable,
        argument: object,
    ):
        try:
            if argument is None:
                function()
                accepted = True
            else:
                accepted = bool(function(argument))
            response = self._response_fields(operation)
        except Exception as error:
            accepted = False
            try:
                response = dict(self._response_fields(operation))
            except Exception:
                response = {}
            response["exception"] = str(error)
        return accepted, response

    def _handle_service_failure(
        self,
        task: Mapping[str, object],
        response: Mapping[str, object],
    ) -> None:
        if response.get("result") == 1:
            # MAV_RESULT_TEMPORARILY_REJECTED: the autopilot is busy (e.g.
            # arming while the mode/estimator is still settling).  The state
            # machine re-issues the same service on the next tick, so treat
            # this as retryable instead of aborting the whole trial.
            logger.warning(
                "service temporarily rejected, retrying: "
                "{} {}".format(
                    self.SERVICE_NAMES[task["operation"]],
                    task.get("argument"),
                )
            )
            return
        snapshot = self._latest_feedback_snapshot() or task["snapshot"]
        needs_safety_land = (
            snapshot is not None
            and not snapshot.landed
            and not (
                task["operation"] == "set_mode"
                and task["argument"] == "AUTO.LAND"
            )
        )
        event = {
            "stamp": task["now"],
            "reason": (
                "service_exception"
                if "exception" in response
                else "service_rejected"
            ),
            "service": self.SERVICE_NAMES[task["operation"]],
            "operation": task["operation"],
            "response": dict(response),
        }
        if needs_safety_land:
            event.update(
                {
                    "safety_service": self.SERVICE_NAMES["set_mode"],
                    "safety_operation": "set_mode",
                }
            )
        with self._service_lock:
            if self._service_failure_latched or self._aborted:
                return
            self._service_failure_latched = True
            self._abort_event = event
            self._abort_requested.set()
        self._activate_safe_output(snapshot)
        if needs_safety_land:
            self._submit_safety_land(snapshot, float(task["now"]))

    def _request_abort(
        self, event: Mapping[str, object], snapshot: Optional[Snapshot]
    ) -> None:
        with self._service_lock:
            if self._service_failure_latched or self._aborted:
                return
            self._service_failure_latched = True
            self._abort_event = dict(event)
            self._abort_requested.set()
        self._activate_safe_output(
            snapshot or self._latest_feedback_snapshot()
        )

    def _consume_requested_abort(self) -> None:
        if not self._abort_requested.is_set():
            return
        with self._service_lock:
            event = self._abort_event
        if event is not None:
            self._enter_abort(
                float(event["stamp"]),
                event,
                self._latest_feedback_snapshot(),
            )

    def _enter_abort(
        self,
        now: float,
        event: Mapping[str, object],
        snapshot: Optional[Snapshot],
    ) -> None:
        with self._control_lock:
            if self._aborted:
                return
            self._aborted = True
            self.state_machine.enter(Phase.ABORT, now)
            self._activate_safe_output(
                snapshot or self._latest_feedback_snapshot(),
                phase=Phase.ABORT.value,
            )
            self.publish_abort(json.dumps(dict(event), sort_keys=True))

    def _activate_safe_output(
        self,
        snapshot: Optional[Snapshot],
        phase: Optional[str] = None,
    ) -> None:
        with self._output_lock:
            if snapshot is not None and not snapshot.landed:
                self._active_kind = "position"
                self._active_values = {
                    "x": float(snapshot.x),
                    "y": float(snapshot.y),
                    "z": float(snapshot.z),
                    "yaw": float(snapshot.yaw),
                }
            else:
                self._active_kind = "manual"
                self._active_values = self._manual_values(
                    self.config.commands.manual_neutral
                )
            self._pose_stream_started_at = None
            if phase is not None:
                self._phase_value = phase

    def _remember_snapshot(self, snapshot: Snapshot) -> None:
        with self._snapshot_lock:
            self._latest_snapshot = snapshot

    def _latest_feedback_snapshot(self) -> Optional[Snapshot]:
        with self._snapshot_lock:
            return self._latest_snapshot

    def _set_phase_value(self, phase: str) -> None:
        with self._output_lock:
            self._phase_value = phase

    def _release_confirmed_services(self, snapshot: Snapshot) -> None:
        confirmed = set()
        with self._service_lock:
            for operation, argument in self._waiting_feedback:
                if operation == "set_mode" and snapshot.mode == argument:
                    confirmed.add((operation, argument))
                elif (
                    operation == "set_mav_type"
                    and snapshot.mav_type == argument
                ):
                    confirmed.add((operation, argument))
                elif operation == "set_armed" and snapshot.armed == argument:
                    confirmed.add((operation, argument))
            self._waiting_feedback.difference_update(confirmed)

    def _response_fields(self, operation: str) -> Mapping[str, object]:
        response_reader = getattr(self.gateway, "response_fields", None)
        if response_reader is not None:
            fields = response_reader(operation)
            if isinstance(fields, Mapping):
                return dict(fields)
        fields = getattr(self.gateway, "last_response_fields", {})
        if isinstance(fields, Mapping):
            return dict(fields)
        return {"value": fields}

    @staticmethod
    def _manual_values(command: ManualControl):
        return {
            "x": float(command.x),
            "y": float(command.y),
            "z": float(command.z),
            "r": float(command.r),
        }


def main() -> None:
    """Run the ROS controller node."""
    from pathlib import Path

    import rospy
    from std_msgs.msg import Empty, String

    from .config import load_config
    from .px4_gateway import Px4Gateway

    rospy.init_node("flying_car_experiment_controller")
    default_config = (
        Path(__file__).resolve().parents[2] / "config" / "experiment.yaml"
    )
    config = load_config(rospy.get_param("~config", str(default_config)))
    gateway = Px4Gateway(config.model_name)
    gateway.warm_up_params()
    heartbeat = rospy.Publisher("~heartbeat", Empty, queue_size=10)
    phase = rospy.Publisher("~phase", String, queue_size=10)
    command = rospy.Publisher("~command", String, queue_size=10)
    abort = rospy.Publisher("~abort", String, queue_size=10, latch=True)
    controller = ExperimentController(
        config,
        ExperimentStateMachine(
            config,
            spawn_yaw=float(rospy.get_param("~spawn_yaw", -1.5708)),
        ),
        gateway,
        publish_heartbeat=lambda _stamp: heartbeat.publish(Empty()),
        publish_phase=lambda value: phase.publish(String(data=value)),
        publish_command=lambda value: command.publish(String(data=value)),
        publish_abort=lambda value: abort.publish(String(data=value)),
    )
    controller.start_streaming(
        lambda: rospy.Time.now().to_sec(),
    )
    mav_type_refresh = rospy.Timer(
        rospy.Duration(1.0),
        lambda _event: controller.refresh_mav_type(
            rospy.Time.now().to_sec()
        ),
    )
    rate = rospy.Rate(config.publish_rate_hz)
    latest_snapshot = [None]

    def shutdown() -> None:
        mav_type_refresh.shutdown()
        controller.shutdown(latest_snapshot[0])
        controller.close(timeout=2.0)
        controller.stop_streaming(timeout=1.0)

    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        try:
            latest_snapshot[0] = gateway.snapshot(
                now,
                clock_ok=now > 0.0,
                processes_ok=bool(rospy.get_param("~processes_ok", True)),
            )
        except RuntimeError as error:
            rospy.logwarn_throttle(1.0, str(error))
            controller.control_tick(None, now)
            rate.sleep()
            continue
        controller.control_tick(latest_snapshot[0], now)
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            # Normal ROS shutdown path; exit the loop cleanly instead of
            # surfacing a traceback that log scanners classify as fatal.
            break
