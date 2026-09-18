"""Independent safety intervention for the flying-car experiment."""

import math
import queue
import threading
import time
from typing import Callable, List, Mapping, Optional, Sequence

from .models import Action, Phase


def _discard(_message: object) -> None:
    pass


def missing_expected_nodes(expected, actual):
    """Return normalized expected ROS node names absent from *actual*."""
    normalize = lambda name: "/" + str(name).lstrip("/")
    expected_names = {normalize(name) for name in expected}
    actual_names = {normalize(name) for name in actual}
    return tuple(sorted(expected_names - actual_names))


class ExpectedNodeMonitor:
    """Poll ROS master off the safety-output path and detect node loss."""

    def __init__(
        self,
        expected,
        list_nodes,
        interval_s: float = 0.25,
        startup_grace_s: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.expected = tuple(
            sorted("/" + str(name).lstrip("/") for name in expected)
        )
        self.list_nodes = list_nodes
        self.interval_s = float(interval_s)
        self.startup_grace_s = float(startup_grace_s)
        self._clock = clock
        self._started_at = float(clock())
        self._lock = threading.Lock()
        self._missing = ()
        self._armed = not self.expected
        self._stop = threading.Event()
        self._thread = None

    @property
    def missing(self):
        with self._lock:
            return self._missing

    def sample_once(self) -> None:
        try:
            missing = missing_expected_nodes(
                self.expected, self.list_nodes()
            )
        except Exception:
            missing = self.expected
        with self._lock:
            if not missing:
                self._armed = True
            elif (
                float(self._clock()) - self._started_at
                > self.startup_grace_s
            ):
                self._armed = True
            self._missing = missing if self._armed else ()

    def start(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="flying-car-node-health",
            daemon=True,
        )
        self._thread.start()
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            self.sample_once()
            self._stop.wait(self.interval_s)

    def close(self, timeout: float = 1.0) -> bool:
        self._stop.set()
        thread = self._thread
        if thread is None:
            return True
        if thread is not threading.current_thread():
            thread.join(timeout=float(timeout))
        return not thread.is_alive()


class SafetyCommandArbiter:
    """Make this node the sole relay and permanent safety-takeover owner."""

    def __init__(self, gateway, manual_neutral):
        self.gateway = gateway
        self.manual_neutral = manual_neutral
        self._lock = threading.RLock()
        self._takeover_kind = None
        self._fixed_hold = None
        self._latest = None
        self._last_ingress_at = {
            "manual": None,
            "pose": None,
        }
        self._neutral_output_ticks = 0

    @property
    def wiring_ready(self) -> bool:
        with self._lock:
            return any(
                received_at is not None
                for received_at in self._last_ingress_at.values()
            )

    @property
    def latest_kind(self):
        with self._lock:
            return None if self._latest is None else self._latest[0]

    @property
    def neutral_output_ticks(self) -> int:
        with self._lock:
            return self._neutral_output_ticks

    def ingress_fresh(
        self,
        kind: str,
        now: float,
        timeout_s: float,
        not_before: Optional[float] = None,
    ) -> bool:
        with self._lock:
            received_at = self._last_ingress_at.get(str(kind))
            if received_at is None:
                return False
            if (
                not_before is not None
                and received_at < float(not_before)
            ):
                return False
            age = float(now) - received_at
            return 0.0 <= age <= float(timeout_s)

    def relay_manual(
        self, x, y, z, r, received_at: Optional[float] = None
    ) -> bool:
        values = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "r": float(r),
        }
        arrival = (
            time.monotonic()
            if received_at is None
            else float(received_at)
        )
        with self._lock:
            self._last_ingress_at["manual"] = arrival
            if self._takeover_kind not in (None, "neutral", "hold"):
                return False
            # A fresh controller command clears a stale neutral/hold takeover
            # that was latched during startup or a transient intervention.
            self._takeover_kind = None
            self._latest = ("manual", values)
            self.gateway.publish_manual(**values)
            return True

    def relay_pose(
        self, x, y, z, yaw, received_at: Optional[float] = None
    ) -> bool:
        values = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "yaw": float(yaw),
        }
        arrival = (
            time.monotonic()
            if received_at is None
            else float(received_at)
        )
        with self._lock:
            self._last_ingress_at["pose"] = arrival
            if self._takeover_kind not in (None, "neutral", "hold"):
                return False
            self._takeover_kind = None
            self._latest = ("pose", values)
            self.gateway.publish_local_pose(**values)
            return True

    def forward_latest(
        self,
        now: Optional[float] = None,
        expected_kind: Optional[str] = None,
        timeout_s: Optional[float] = None,
        not_before: Optional[float] = None,
    ) -> bool:
        with self._lock:
            if self._takeover_kind is not None or self._latest is None:
                return False
            kind, values = self._latest
            if expected_kind is not None and kind != expected_kind:
                return False
            if timeout_s is not None:
                current = (
                    time.monotonic() if now is None else float(now)
                )
                if not self.ingress_fresh(
                    kind,
                    current,
                    float(timeout_s),
                    not_before=not_before,
                ):
                    return False
            if kind == "manual":
                self.gateway.publish_manual(**values)
            else:
                self.gateway.publish_local_pose(**values)
            return True

    def publish_neutral(self) -> bool:
        values = {
            "x": float(self.manual_neutral.x),
            "y": float(self.manual_neutral.y),
            "z": float(self.manual_neutral.z),
            "r": float(self.manual_neutral.r),
        }
        with self._lock:
            if self._takeover_kind is None:
                self._takeover_kind = "neutral"
            if self._takeover_kind != "neutral":
                return False
            self.gateway.publish_manual(**values)
            self._neutral_output_ticks += 1
            return True

    def publish_hold(self, values) -> bool:
        requested = {
            name: float(values[name])
            for name in ("x", "y", "z", "yaw")
        }
        with self._lock:
            if self._takeover_kind is None:
                self._takeover_kind = "hold"
                self._fixed_hold = requested
            if self._takeover_kind != "hold":
                return False
            self.gateway.publish_local_pose(**self._fixed_hold)
            return True


class SafetyActionExecutor:
    """Keep safe output synchronous and MAVROS services serialized."""

    _STOP = object()

    def __init__(
        self,
        gateway,
        manual_neutral,
        publish_abort: Callable[[object], None] = _discard,
        publish_status: Callable[[object], None] = _discard,
        arbiter=None,
    ):
        self.gateway = gateway
        self.manual_neutral = manual_neutral
        self.arbiter = arbiter or SafetyCommandArbiter(
            gateway, manual_neutral
        )
        self.publish_abort = publish_abort
        self.publish_status = publish_status
        self.service_failures: List[Mapping[str, object]] = []
        self._output_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._service_keys = set()
        self._abort_sent = False
        self._closed = False
        self._queue = queue.Queue()
        self._worker = threading.Thread(
            target=self._run_services,
            name="flying-car-safety-services",
            daemon=True,
        )
        self._worker.start()

    def execute(self, actions: Sequence[Action]) -> None:
        """Execute outputs immediately and enqueue each service at most once."""
        for action in actions:
            if action.kind == "manual_neutral":
                with self._output_lock:
                    self.arbiter.publish_neutral()
            elif action.kind == "position_hold":
                with self._output_lock:
                    self.arbiter.publish_hold(action.values)
            elif action.kind == "disarm":
                if self.arbiter.neutral_output_ticks >= 3:
                    self._submit_service(action.kind)
            elif action.kind == "auto_land":
                self._submit_service(action.kind)
            elif action.kind == "abort":
                with self._state_lock:
                    if self._abort_sent:
                        continue
                    self._abort_sent = True
                self.publish_abort(str(action.values["reason"]))

    def relay_manual(
        self, x, y, z, r, received_at: Optional[float] = None
    ) -> bool:
        return self.arbiter.relay_manual(
            x, y, z, r, received_at=received_at
        )

    def relay_pose(
        self, x, y, z, yaw, received_at: Optional[float] = None
    ) -> bool:
        return self.arbiter.relay_pose(
            x, y, z, yaw, received_at=received_at
        )

    def relay_keepalive(
        self,
        now: Optional[float] = None,
        expected_kind: Optional[str] = None,
        timeout_s: Optional[float] = None,
        not_before: Optional[float] = None,
    ) -> bool:
        return self.arbiter.forward_latest(
            now=now,
            expected_kind=expected_kind,
            timeout_s=timeout_s,
            not_before=not_before,
        )

    def _submit_service(self, kind: str) -> bool:
        with self._state_lock:
            if self._closed or kind in self._service_keys:
                return False
            self._service_keys.add(kind)
            self._queue.put(kind)
            return True

    def _run_services(self) -> None:
        while True:
            kind = self._queue.get()
            try:
                if kind is self._STOP:
                    return
                self._run_service(str(kind))
            finally:
                self._queue.task_done()

    def _run_service(self, kind: str) -> None:
        try:
            if kind == "auto_land":
                result = self.gateway.set_mode("AUTO.LAND")
            else:
                result = self.gateway.set_armed(False)
            if bool(result):
                return
            failure = {
                "action": kind,
                "ok": False,
                "error": "service rejected request",
            }
        except Exception as error:
            failure = {
                "action": kind,
                "ok": False,
                "error": "{}: {}".format(type(error).__name__, error),
            }
        with self._state_lock:
            self.service_failures.append(failure)
        self.publish_status(failure)

    def wait(self, timeout: float = 1.0) -> bool:
        """Wait for already queued service work without accepting retries."""
        deadline = time.monotonic() + float(timeout)
        while self._queue.unfinished_tasks:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.001)
        return True

    def close(self, timeout: float = 1.0) -> bool:
        """Stop accepting work and join the sole service worker."""
        with self._state_lock:
            if not self._closed:
                self._closed = True
                self._queue.put(self._STOP)
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=float(timeout))
        return not self._worker.is_alive()


class SafetyRuntime:
    """Thread-safe feedback adapter around the deterministic guard."""

    MANUAL_INGRESS_TOPIC = "~controller/manual"
    POSE_INGRESS_TOPIC = "~controller/pose"

    def __init__(
        self,
        guard,
        executor,
        node_health,
        startup_grace_s: float = 5.0,
        wiring_required=None,
        command_ingress_timeout_s=None,
    ):
        self.guard = guard
        self.executor = executor
        self.node_health = node_health
        self.startup_grace_s = float(startup_grace_s)
        self.wiring_required = (
            bool(guard.heartbeat_required)
            if wiring_required is None
            else bool(wiring_required)
        )
        self.command_ingress_timeout_s = float(
            guard.heartbeat_timeout_s
            if command_ingress_timeout_s is None
            else command_ingress_timeout_s
        )
        if self.command_ingress_timeout_s <= 0.0:
            raise ValueError(
                "command_ingress_timeout_s must be positive"
            )
        self._lock = threading.Lock()
        self._phase = Phase.PRECHECK
        self._connected = None
        self._armed = False
        self._landed = False
        self._state_seen = False
        self._landed_seen = False
        self._position = (0.0, 0.0, 0.0)
        self._attitude = (0.0, 0.0, 0.0)
        self._ground_z = None
        self._plan_xy = None
        self._clock_stamp = None
        self._last_heartbeat = None
        self._heartbeat_seen = False
        self._command_telemetry_seen = False
        self._expected_command_kind = None
        self._expected_command_since = None
        self._last_command_telemetry_at = None
        self._first_step_at = None
        self._stop = threading.Event()
        self._thread = None

    def update_phase(self, phase) -> None:
        value = phase if isinstance(phase, Phase) else Phase(str(phase))
        with self._lock:
            self._phase = value

    def update_vehicle(
        self, connected: bool, armed: bool, landed: bool
    ) -> None:
        with self._lock:
            self._connected = bool(connected)
            self._armed = bool(armed)
            self._landed = bool(landed)
            self._state_seen = True
            self._landed_seen = True

    def update_state(self, connected: bool, armed: bool) -> None:
        with self._lock:
            self._connected = bool(connected)
            self._armed = bool(armed)
            self._state_seen = True

    def update_landed(self, landed: bool) -> None:
        with self._lock:
            self._landed = bool(landed)
            self._landed_seen = True

    def update_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        with self._lock:
            self._position = (float(x), float(y), float(z))
            self._attitude = (
                float(roll),
                float(pitch),
                float(yaw),
            )
            if (
                self._ground_z is None
                and self._landed_seen
                and self._landed
            ):
                self._ground_z = float(z)
            if (
                self._plan_xy is None
                and self._landed_seen
                and self._landed
            ):
                self._plan_xy = (float(x), float(y))

    def update_plan(self, x: float, y: float) -> None:
        with self._lock:
            self._plan_xy = (float(x), float(y))

    def update_clock(self, stamp: float) -> None:
        with self._lock:
            self._clock_stamp = float(stamp)

    def update_heartbeat(self, now: float) -> None:
        with self._lock:
            self._last_heartbeat = float(now)
            self._heartbeat_seen = True

    def update_controller_command(self, kind, now: float) -> None:
        command_kind = str(kind)
        normalized = {
            "manual": "manual",
            "position": "pose",
            "pose": "pose",
        }.get(command_kind, "unknown")
        received_at = float(now)
        with self._lock:
            if (
                not self._command_telemetry_seen
                or normalized != self._expected_command_kind
            ):
                self._expected_command_kind = normalized
                self._expected_command_since = received_at
            self._command_telemetry_seen = True
            self._last_command_telemetry_at = received_at

    @property
    def wiring_ready(self) -> bool:
        return self.executor.arbiter.wiring_ready

    def relay_manual(
        self, x, y, z, r, received_at: Optional[float] = None
    ) -> bool:
        with self._lock:
            if (
                self._command_telemetry_seen
                and self._expected_command_kind != "manual"
            ):
                return False
            return self.executor.relay_manual(
                x, y, z, r, received_at=received_at
            )

    def relay_pose(
        self, x, y, z, yaw, received_at: Optional[float] = None
    ) -> bool:
        with self._lock:
            if (
                self._command_telemetry_seen
                and self._expected_command_kind != "pose"
            ):
                return False
            self._plan_xy = (float(x), float(y))
            return self.executor.relay_pose(
                x, y, z, yaw, received_at=received_at
            )

    def _guard_inputs(self, now: float):
        with self._lock:
            if self._first_step_at is None:
                self._first_step_at = float(now)
            x, y, z = self._position
            roll, pitch, yaw = self._attitude
            ground_z = z if self._ground_z is None else self._ground_z
            plan_x, plan_y = (
                (x, y) if self._plan_xy is None else self._plan_xy
            )
            expected_kind = self._expected_command_kind
            expected_since = self._expected_command_since
            command_telemetry_seen = self._command_telemetry_seen
            startup_elapsed = float(now) - self._first_step_at

            relay_kind = None
            relay_not_before = None
            command_ingress_ok = True
            command_ingress_reason = None
            if self.wiring_required:
                if command_telemetry_seen:
                    relay_kind = expected_kind
                    relay_not_before = expected_since
                    matching_ingress = (
                        expected_kind in ("manual", "pose")
                        and self.executor.arbiter.ingress_fresh(
                            expected_kind,
                            now=float(now),
                            timeout_s=self.command_ingress_timeout_s,
                            not_before=expected_since,
                        )
                    )
                    switch_age = (
                        None
                        if expected_since is None
                        else float(now) - expected_since
                    )
                    switch_grace = (
                        switch_age is not None
                        and 0.0
                        <= switch_age
                        <= self.command_ingress_timeout_s
                    )
                    command_ingress_ok = (
                        matching_ingress or switch_grace
                    )
                    if not command_ingress_ok:
                        command_ingress_reason = (
                            "controller_command_ingress_timeout"
                        )
                else:
                    relay_kind = self.executor.arbiter.latest_kind
                    fresh_ingress = (
                        relay_kind is not None
                        and self.executor.arbiter.ingress_fresh(
                            relay_kind,
                            now=float(now),
                            timeout_s=self.command_ingress_timeout_s,
                        )
                    )
                    command_ingress_ok = (
                        fresh_ingress
                        or startup_elapsed <= self.startup_grace_s
                    )
                    if not command_ingress_ok:
                        command_ingress_reason = (
                            "controller_command_ingress_timeout"
                            if self.executor.arbiter.wiring_ready
                            else "controller_command_ingress_missing"
                        )
            inputs = {
                "phase": self._phase,
                "connected": (
                    True
                    if self._connected is None
                    else self._connected
                ),
                "armed": self._armed,
                "landed": self._landed,
                "state_seen": self._state_seen,
                "landed_seen": self._landed_seen,
                "state_feedback_ok": (
                    self._state_seen
                    or float(now) - self._first_step_at
                    <= self.startup_grace_s
                ),
                "command_ingress_ok": command_ingress_ok,
                "command_ingress_reason": command_ingress_reason,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "roll": roll,
                "pitch": pitch,
                "agl_m": max(0.0, z - ground_z),
                "planar_deviation_m": math.hypot(
                    x - plan_x, y - plan_y
                ),
                "clock_stamp": self._clock_stamp,
                "clock_ok": (
                    self._clock_stamp is not None
                    or float(now) - self._first_step_at
                    <= self.guard.heartbeat_timeout_s
                ),
                "last_heartbeat": self._last_heartbeat,
                "heartbeat_seen": self._heartbeat_seen,
                "_relay_kind": relay_kind,
                "_relay_not_before": relay_not_before,
            }
        missing = self.node_health.missing
        inputs["processes_ok"] = not missing
        inputs["missing_processes"] = missing
        return inputs

    def step(self, now: float):
        inputs = self._guard_inputs(now)
        relay_kind = inputs.pop("_relay_kind")
        relay_not_before = inputs.pop("_relay_not_before")
        actions = self.guard.tick(now=float(now), **inputs)
        self.executor.execute(actions)
        if not actions:
            self.executor.relay_keepalive(
                now=float(now),
                expected_kind=relay_kind,
                timeout_s=self.command_ingress_timeout_s,
                not_before=relay_not_before,
            )
        return actions

    def run_once(self, now: float):
        """Run one iteration and convert internal exceptions to takeover."""
        try:
            return self.step(now)
        except Exception as error:
            reason = "safety_runtime_exception:{}:{}".format(
                type(error).__name__, error
            )
            inputs = self._guard_inputs(now)
            actions = self.guard.fail_safe(
                reason=reason,
                now=float(now),
                **inputs
            )
            self.executor.execute(actions)
            return actions

    def start(
        self,
        rate_hz: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(float(rate_hz), clock),
            name="flying-car-safety-output",
            daemon=True,
        )
        self._thread.start()
        return True

    def _run(self, rate_hz: float, clock: Callable[[], float]) -> None:
        period = 1.0 / rate_hz
        while not self._stop.is_set():
            self.run_once(clock())
            self._stop.wait(period)

    def close(self, timeout: float = 1.0) -> bool:
        self._stop.set()
        thread = self._thread
        if thread is None:
            return True
        if thread is not threading.current_thread():
            thread.join(timeout=float(timeout))
        return not thread.is_alive()


class SafetyGuard:
    """Classify hard failures and select a phase-safe intervention."""

    GROUND_PHASES = frozenset({
        Phase.GROUND_CONFIGURE,
        Phase.GROUND_FORWARD,
        Phase.GROUND_STOP,
        Phase.FLIGHT_CONFIGURE,
        Phase.OFFBOARD_PRIME,
        Phase.GROUND_RECONFIGURE,
        Phase.GROUND_TURN,
        Phase.GROUND_DIAGONAL,
    })
    AIR_PHASES = frozenset({
        Phase.TAKEOFF,
        Phase.AIR_FORWARD,
        Phase.HOVER,
        Phase.MANUAL_AIR,
        Phase.REACQUIRE_OFFBOARD,
        Phase.LAND,
    })
    INACTIVE_PHASES = frozenset({
        Phase.PRECHECK,
        Phase.COMPLETE,
        Phase.ABORT,
    })

    def __init__(
        self,
        heartbeat_timeout_s: float,
        hard_tilt_deg: float,
        maximum_agl_m: float,
        maximum_planar_deviation_m: float,
        heartbeat_required: bool = True,
    ):
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.hard_tilt_deg = float(hard_tilt_deg)
        self.maximum_agl_m = float(maximum_agl_m)
        self.maximum_planar_deviation_m = float(
            maximum_planar_deviation_m
        )
        self.heartbeat_required = bool(heartbeat_required)
        self.abort_reason: Optional[str] = None
        self._abort_published = False
        self._intervention: Optional[str] = None
        self._neutral_ticks = 0
        self._vehicle_action_requested = False
        self._fixed_hold = None
        self._last_clock_stamp: Optional[float] = None
        self._clock_advanced_at: Optional[float] = None

    def tick(
        self,
        now: float,
        phase: Phase,
        last_heartbeat: Optional[float] = None,
        heartbeat_seen: bool = False,
        armed: bool = True,
        landed: bool = False,
        connected: bool = True,
        state_feedback_ok: bool = True,
        command_ingress_ok: bool = True,
        command_ingress_reason: Optional[str] = None,
        state_seen: bool = False,
        landed_seen: bool = False,
        processes_ok: bool = True,
        missing_processes=(),
        clock_ok: bool = True,
        clock_stamp: Optional[float] = None,
        roll: float = 0.0,
        pitch: float = 0.0,
        agl_m: float = 0.0,
        planar_deviation_m: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0,
        forced_reason: Optional[str] = None,
        **_inputs: object
    ) -> List[Action]:
        """Return deterministic actions for one guard evaluation."""
        reason = forced_reason
        clock_reason = None
        if clock_stamp is not None:
            current_clock = float(clock_stamp)
            if (
                self._last_clock_stamp is not None
                and current_clock < self._last_clock_stamp
            ):
                clock_reason = "clock_backward"
            elif (
                self._last_clock_stamp is None
                or current_clock > self._last_clock_stamp
            ):
                self._last_clock_stamp = current_clock
                self._clock_advanced_at = float(now)
            elif (
                self._clock_advanced_at is not None
                and float(now) - self._clock_advanced_at
                > self.heartbeat_timeout_s
            ):
                clock_reason = "clock_stopped"
        if reason is not None:
            pass
        elif not state_feedback_ok:
            reason = "state_feedback_missing"
        elif not command_ingress_ok:
            reason = (
                command_ingress_reason
                or "controller_command_ingress_missing"
            )
        elif not connected:
            reason = "connection_loss"
        elif not processes_ok:
            missing = sorted(str(name) for name in missing_processes)
            reason = "expected_ros_node_loss"
            if missing:
                reason += ":" + ",".join(missing)
        elif not clock_ok:
            reason = "clock_failure"
        elif clock_reason is not None:
            reason = clock_reason
        elif math.degrees(max(abs(float(roll)), abs(float(pitch)))) > (
            self.hard_tilt_deg
        ):
            reason = "hard_tilt"
        elif float(agl_m) > self.maximum_agl_m:
            reason = "maximum_agl"
        elif (
            float(planar_deviation_m)
            > self.maximum_planar_deviation_m
        ):
            reason = "maximum_planar_deviation"
        elif (
            self.heartbeat_required
            and heartbeat_seen
            and last_heartbeat is not None
            and float(now) - float(last_heartbeat)
            > self.heartbeat_timeout_s
        ):
            reason = "heartbeat_timeout"
        if self.abort_reason is None and reason is not None:
            self.abort_reason = reason
            if state_seen and landed_seen:
                self._intervention = (
                    "ground" if landed else "air"
                )
            elif phase in self.GROUND_PHASES:
                self._intervention = "ground"
            elif phase in self.AIR_PHASES:
                self._intervention = "air"
            else:
                self._intervention = "inactive"
            if self._intervention == "air":
                self._fixed_hold = {
                    "x": float(x),
                    "y": float(y),
                    "z": float(z),
                    "yaw": float(yaw),
                }

        if self.abort_reason is None:
            return []

        actions: List[Action] = []
        if self._intervention == "ground":
            if armed:
                self._neutral_ticks += 1
                actions.append(Action("manual_neutral", {}))
                if (
                    self._neutral_ticks >= 3
                    and not self._vehicle_action_requested
                ):
                    actions.append(Action("disarm", {}))
                    self._vehicle_action_requested = True
        elif self._intervention == "air":
            if not landed:
                actions.append(Action(
                    "position_hold", dict(self._fixed_hold)
                ))
                if not self._vehicle_action_requested:
                    actions.append(Action("auto_land", {}))
                    self._vehicle_action_requested = True
        else:
            actions.append(Action("manual_neutral", {}))

        if not self._abort_published:
            actions.append(Action("abort", {"reason": self.abort_reason}))
            self._abort_published = True
        return actions

    def fail_safe(self, reason: str, now: float, **inputs):
        """Latch an internal failure through the normal intervention path."""
        inputs["forced_reason"] = str(reason)
        return self.tick(now=float(now), **inputs)


def main() -> None:
    """Run the independent wall-clock safety guard ROS node."""
    import json
    from pathlib import Path

    import rosnode
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import (
        ExtendedState,
        ManualControl as ManualControlMessage,
        State,
    )
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Empty, String

    from .config import load_config
    from .geometry import quaternion_to_rpy
    from .px4_gateway import Px4Gateway

    rospy.init_node("flying_car_safety_guard")
    default_config = (
        Path(__file__).resolve().parents[2] / "config" / "experiment.yaml"
    )
    config = load_config(rospy.get_param("~config", str(default_config)))
    heartbeat_required = bool(
        rospy.get_param("~heartbeat_required", True)
    )
    startup_grace_s = float(
        rospy.get_param("~startup_grace_s", 5.0)
    )
    command_ingress_timeout_s = float(
        rospy.get_param(
            "~command_ingress_timeout_s",
            config.safety.heartbeat_timeout_s,
        )
    )
    if command_ingress_timeout_s <= 0.0:
        raise ValueError(
            "~command_ingress_timeout_s must be positive"
        )
    default_nodes = [
        "/gazebo",
        "/mavros",
        "/flying_car_experiment_controller",
        "/flying_car_experiment_monitor",
    ]
    expected_nodes = rospy.get_param(
        "~expected_ros_nodes", default_nodes
    )
    if not isinstance(expected_nodes, (list, tuple)):
        raise ValueError("~expected_ros_nodes must be a list")

    abort_publisher = rospy.Publisher(
        "~abort", String, queue_size=10, latch=True
    )
    status_publisher = rospy.Publisher(
        "~action_status", String, queue_size=10, latch=True
    )
    gateway = Px4Gateway(config.model_name)
    executor = SafetyActionExecutor(
        gateway,
        config.commands.manual_neutral,
        publish_abort=lambda reason: abort_publisher.publish(
            String(data=str(reason))
        ),
        publish_status=lambda status: status_publisher.publish(
            String(data=json.dumps(status, sort_keys=True))
        ),
    )
    guard = SafetyGuard(
        heartbeat_timeout_s=config.safety.heartbeat_timeout_s,
        hard_tilt_deg=config.safety.hard_tilt_deg,
        maximum_agl_m=config.safety.maximum_agl_m,
        maximum_planar_deviation_m=(
            config.safety.maximum_planar_deviation_m
        ),
        heartbeat_required=heartbeat_required,
    )
    node_health = ExpectedNodeMonitor(
        expected_nodes, rosnode.get_node_names
    )
    runtime = SafetyRuntime(
        guard,
        executor,
        node_health,
        startup_grace_s=startup_grace_s,
        command_ingress_timeout_s=command_ingress_timeout_s,
    )

    def on_pose(message) -> None:
        position = message.pose.position
        orientation = message.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        runtime.update_pose(
            position.x,
            position.y,
            position.z,
            roll,
            pitch,
            yaw,
        )

    def on_command(message) -> None:
        try:
            command = json.loads(message.data)
            kind = command["kind"]
            runtime.update_controller_command(
                kind, now=time.monotonic()
            )
            if kind == "position":
                values = command["values"]
                runtime.update_plan(values["x"], values["y"])
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(
                1.0, "invalid flying-car controller command telemetry"
            )

    def _relay_manual_safe(message) -> None:
        try:
            runtime.relay_manual(
                message.x,
                message.y,
                message.z,
                message.r,
                received_at=time.monotonic(),
            )
        except Exception:
            # ROS shutdown can deliver a queued ingress message after the
            # outbound topics are closed; that is a normal exit race and
            # must not surface as a "bad callback" traceback.
            rospy.logwarn_throttle(
                1.0, "relay_manual dropped during shutdown race"
            )

    def _relay_pose_safe(message) -> None:
        try:
            runtime.relay_pose(
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
                quaternion_to_rpy(
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                )[2],
                received_at=time.monotonic(),
            )
        except Exception:
            rospy.logwarn_throttle(
                1.0, "relay_pose dropped during shutdown race"
            )

    subscriptions = (
        rospy.Subscriber(
            "/mavros/state",
            State,
            lambda message: runtime.update_state(
                message.connected, message.armed
            ),
            queue_size=10,
        ),
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            lambda message: runtime.update_landed(
                message.landed_state
                == ExtendedState.LANDED_STATE_ON_GROUND
            ),
            queue_size=10,
        ),
        rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            on_pose,
            queue_size=10,
        ),
        rospy.Subscriber(
            "/clock",
            Clock,
            lambda message: runtime.update_clock(
                message.clock.to_sec()
            ),
            queue_size=10,
        ),
        rospy.Subscriber(
            "/flying_car_experiment_controller/heartbeat",
            Empty,
            lambda _message: runtime.update_heartbeat(time.monotonic()),
            queue_size=10,
        ),
        rospy.Subscriber(
            "/flying_car_experiment_controller/phase",
            String,
            lambda message: runtime.update_phase(message.data),
            queue_size=10,
        ),
        rospy.Subscriber(
            "/flying_car_experiment_controller/command",
            String,
            on_command,
            queue_size=10,
        ),
        rospy.Subscriber(
            SafetyRuntime.MANUAL_INGRESS_TOPIC,
            ManualControlMessage,
            _relay_manual_safe,
            queue_size=10,
        ),
        rospy.Subscriber(
            SafetyRuntime.POSE_INGRESS_TOPIC,
            PoseStamped,
            _relay_pose_safe,
            queue_size=10,
        ),
    )

    shutdown_lock = threading.Lock()
    closed = [False]

    def shutdown() -> None:
        with shutdown_lock:
            if closed[0]:
                return
            closed[0] = True
        runtime_stopped = runtime.close(timeout=1.0)
        health_stopped = node_health.close(timeout=1.0)
        services_stopped = executor.close(timeout=2.0)
        if not (runtime_stopped and health_stopped and services_stopped):
            rospy.logerr(
                "safety guard worker shutdown timed out: "
                "runtime=%s health=%s services=%s",
                runtime_stopped,
                health_stopped,
                services_stopped,
            )

    rospy.on_shutdown(shutdown)
    node_health.start()
    runtime.start(config.publish_rate_hz, time.monotonic)
    try:
        rospy.spin()
    finally:
        shutdown()
        _ = subscriptions
