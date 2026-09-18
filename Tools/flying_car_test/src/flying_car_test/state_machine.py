"""Pure feedback-driven state machine for the flying-car experiment."""

import math
from typing import Callable, Dict, List, Optional, Tuple

from .geometry import body_offset_to_enu, enu_to_longitudinal_lateral
from .models import Action, ExperimentConfig, Phase, Snapshot


class ExperimentStateMachine:
    """Advance the experiment using snapshots and emit declarative actions only."""

    GROUND_MAV_TYPE = 10
    FLIGHT_MAV_TYPE = 14

    def __init__(
        self,
        config: ExperimentConfig,
        spawn_yaw: float = 0.0,
        spawn_yaw_tolerance_deg: float = 20.0,
        switch_boot_min_s: float = 2.5,
    ):
        self.config = config
        self._switch_boot_min_s = float(switch_boot_min_s)
        self._spawn_yaw = float(spawn_yaw)
        self._spawn_yaw_tolerance_rad = math.radians(
            float(spawn_yaw_tolerance_deg)
        )
        self.phase = Phase.PRECHECK
        self.phase_started_at: Optional[float] = None
        self.phase_origin: Optional[Snapshot] = None
        self.first_ground_origin: Optional[Tuple[float, float, float]] = None
        self.takeoff_ground_height: Optional[float] = None
        self.air_forward_origin: Optional[Tuple[float, float, float, float]] = None
        self.landing_origin: Optional[Tuple[float, float, float]] = None
        self.final_ground_target: Optional[Tuple[float, float]] = None
        self._offboard_hold: Optional[Tuple[float, float, float, float]] = None
        self._phase_safety_origin: Optional[Tuple[float, float]] = None
        self._hover_stable_since: Optional[float] = None
        self._manual_air_started_at: Optional[float] = None
        self._manual_air_braking = False
        self._manual_air_brake_started_at: Optional[float] = None
        self._handlers: Dict[Phase, Callable[[Snapshot, float], List[Action]]] = {
            Phase.PRECHECK: self._precheck,
            Phase.GROUND_CONFIGURE: self._ground_configure,
            Phase.GROUND_FORWARD: self._ground_forward,
            Phase.GROUND_STOP: self._ground_stop,
            Phase.FLIGHT_CONFIGURE: self._flight_configure,
            Phase.OFFBOARD_PRIME: self._offboard_prime,
            Phase.TAKEOFF: self._takeoff,
            Phase.AIR_FORWARD: self._air_forward,
            Phase.HOVER: self._hover,
            Phase.MANUAL_AIR: self._manual_air,
            Phase.REACQUIRE_OFFBOARD: self._reacquire_offboard,
            Phase.LAND: self._land,
            Phase.GROUND_RECONFIGURE: self._ground_reconfigure,
            Phase.GROUND_TURN: self._ground_turn,
            Phase.GROUND_DIAGONAL: self._ground_diagonal,
        }

    def enter(self, phase: Phase, now: float) -> None:
        """Enter *phase* and reset phase-local timing and origin state."""
        self.phase = phase
        self.phase_started_at = float(now)
        self.phase_origin = None
        self._phase_safety_origin = None
        if phase == Phase.HOVER:
            self._hover_stable_since = None
        if phase == Phase.MANUAL_AIR:
            self._manual_air_started_at = None
            self._manual_air_braking = False
            self._manual_air_brake_started_at = None

    def set_phase_origin(self, snapshot: Snapshot) -> None:
        """Capture an immutable origin appropriate for the current phase."""
        self.phase_origin = snapshot
        if self._phase_safety_origin is None:
            self._phase_safety_origin = (snapshot.x, snapshot.y)
        if self.phase == Phase.GROUND_FORWARD and self.first_ground_origin is None:
            self.first_ground_origin = (snapshot.x, snapshot.y, snapshot.yaw)
        elif self.phase == Phase.TAKEOFF and self.takeoff_ground_height is None:
            self.takeoff_ground_height = snapshot.z
        elif self.phase == Phase.AIR_FORWARD and self.air_forward_origin is None:
            self.air_forward_origin = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
        elif self.phase in (Phase.HOVER, Phase.REACQUIRE_OFFBOARD):
            self._offboard_hold = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
        elif self.phase == Phase.GROUND_DIAGONAL and self.landing_origin is not None:
            self._calculate_final_ground_target()

    def record_landing_origin(self, snapshot: Snapshot) -> None:
        """Capture the landing pose and its immutable final-ground target."""
        if self.landing_origin is not None:
            return
        self.landing_origin = (snapshot.x, snapshot.y, snapshot.yaw)
        self._calculate_final_ground_target()

    def advance(self, snapshot: Snapshot, now: float) -> List[Action]:
        """Advance by one feedback sample and return ordered declarative actions."""
        if self.phase in (Phase.COMPLETE, Phase.ABORT):
            return []
        if self.phase_started_at is None:
            self.phase_started_at = float(now)
        if self._phase_safety_origin is None:
            self._phase_safety_origin = (snapshot.x, snapshot.y)

        failure = self._hard_safety_failure(snapshot, now)
        if failure is not None:
            return self._abort(failure, now)
        if float(now) - self.phase_started_at > self._timeout_for(self.phase):
            return self._abort("phase_timeout", now)
        return self._handlers[self.phase](snapshot, float(now))

    def _hard_safety_failure(self, snapshot: Snapshot, now: float) -> Optional[str]:
        if not math.isfinite(float(now)) or float(now) < self.phase_started_at:
            return "clock_failure"
        if not snapshot.clock_ok:
            return "clock_failure"
        if not snapshot.processes_ok:
            return "process_failure"
        if not snapshot.connected:
            return "connection_loss"
        if float(now) - snapshot.stamp > self.config.safety.heartbeat_timeout_s:
            return "heartbeat_timeout"
        hard_tilt = math.radians(self.config.safety.hard_tilt_deg)
        if abs(snapshot.roll) > hard_tilt or abs(snapshot.pitch) > hard_tilt:
            return "hard_tilt"
        if self.phase == Phase.PRECHECK and self.takeoff_ground_height is None:
            self.takeoff_ground_height = snapshot.z
        if (
            self.takeoff_ground_height is not None
            and snapshot.z - self.takeoff_ground_height
            > self.config.safety.maximum_agl_m
        ):
            return "maximum_agl"
        if self._phase_safety_origin is not None:
            distance = math.hypot(
                snapshot.x - self._phase_safety_origin[0],
                snapshot.y - self._phase_safety_origin[1],
            )
            if distance > self.config.safety.maximum_planar_deviation_m:
                return "maximum_planar_deviation"
        return None

    def _timeout_for(self, phase: Phase) -> float:
        timeouts = self.config.timeouts
        return {
            Phase.PRECHECK: timeouts.precheck_s,
            Phase.GROUND_CONFIGURE: timeouts.transition_s,
            Phase.GROUND_FORWARD: timeouts.ground_forward_s,
            Phase.GROUND_STOP: timeouts.transition_s,
            Phase.FLIGHT_CONFIGURE: timeouts.transition_s,
            Phase.OFFBOARD_PRIME: timeouts.transition_s,
            Phase.TAKEOFF: timeouts.takeoff_s,
            Phase.AIR_FORWARD: timeouts.air_forward_s,
            Phase.HOVER: timeouts.transition_s,
            Phase.MANUAL_AIR: timeouts.transition_s,
            Phase.REACQUIRE_OFFBOARD: timeouts.transition_s,
            Phase.LAND: timeouts.landing_s,
            Phase.GROUND_RECONFIGURE: timeouts.transition_s,
            Phase.GROUND_TURN: timeouts.transition_s,
            Phase.GROUND_DIAGONAL: timeouts.final_ground_s,
        }[phase]

    def _abort(self, reason: str, now: float) -> List[Action]:
        self.enter(Phase.ABORT, now)
        return [Action("abort", {"reason": reason})]

    def _transition(self, phase: Phase, now: float, snapshot: Snapshot) -> None:
        self.enter(phase, now)
        self._phase_safety_origin = (snapshot.x, snapshot.y)

    @staticmethod
    def _action(kind: str, **values: object) -> Action:
        return Action(kind, values)

    def _manual(self, name: str) -> Action:
        commands = {
            "neutral": self.config.commands.manual_neutral,
            "ground_neutral": self.config.commands.ground_neutral,
            "ground_forward": self.config.commands.ground_forward,
            "ground_left_turn": self.config.commands.ground_left_turn,
            "air_forward": self.config.commands.air_manual_forward,
        }
        return self._action("manual", name=name, command=commands[name])

    @staticmethod
    def _planar_speed(snapshot: Snapshot) -> float:
        return math.hypot(snapshot.vx, snapshot.vy)

    def _position(self, point: Tuple[float, float, float, float]) -> Action:
        x, y, z, yaw = point
        return self._action("position", x=x, y=y, z=z, yaw=yaw)

    def _configure_ground_for(
        self,
        snapshot: Snapshot,
        now: float,
        next_phase: Phase,
    ) -> List[Action]:
        actions = [self._manual("ground_neutral")]
        # The flying-car switch block in Commander only runs 2 s after boot;
        # setting MAV_TYPE before that makes the heartbeat flip early and
        # creates an armed/switch race. Hold until the window has passed.
        if float(now) < self._switch_boot_min_s:
            return actions
        if snapshot.mav_type != self.GROUND_MAV_TYPE:
            if snapshot.armed:
                actions.append(self._action("disarm"))
            elif self._planar_speed(snapshot) < self.config.safety.stopped_speed_mps:
                actions.append(
                    self._action("set_mav_type", mav_type=self.GROUND_MAV_TYPE)
                )
            return actions
        if snapshot.armed and snapshot.mode != "MANUAL":
            actions.append(self._action("disarm"))
            return actions
        if snapshot.mode != "MANUAL":
            actions.append(self._action("set_mode", mode="MANUAL"))
            return actions
        if not snapshot.armed:
            actions.append(self._action("arm"))
            return actions
        # SITL EKF2 yaw alignment converges over ~10-20 s after boot; hold
        # (armed, wheels stopped) until the reported heading matches the
        # spawn orientation so ground odometry is in the right frame.
        if (
            abs(self._signed_angle(snapshot.yaw - self._spawn_yaw))
            > self._spawn_yaw_tolerance_rad
        ):
            return actions
        self._transition(next_phase, now, snapshot)
        self.set_phase_origin(snapshot)
        return []

    def _ground_motion_blocked(self, snapshot: Snapshot) -> Optional[List[Action]]:
        if snapshot.mode != "MANUAL":
            return [
                self._manual("ground_neutral"),
                self._action("set_mode", mode="MANUAL"),
            ]
        if not snapshot.armed:
            return [
                self._manual("ground_neutral"),
                self._action("arm"),
            ]
        return None

    def _precheck(self, snapshot: Snapshot, now: float) -> List[Action]:
        self._transition(Phase.GROUND_CONFIGURE, now, snapshot)
        return []

    def _ground_configure(self, snapshot: Snapshot, now: float) -> List[Action]:
        return self._configure_ground_for(
            snapshot,
            now,
            Phase.GROUND_FORWARD,
        )

    def _ground_forward(self, snapshot: Snapshot, now: float) -> List[Action]:
        blocked = self._ground_motion_blocked(snapshot)
        if blocked is not None:
            return blocked
        if self.first_ground_origin is None:
            self.set_phase_origin(snapshot)
        origin_x, origin_y, origin_yaw = self.first_ground_origin
        longitudinal, lateral = enu_to_longitudinal_lateral(
            snapshot.x - origin_x,
            snapshot.y - origin_y,
            origin_yaw,
        )
        if (
            longitudinal >= self.config.trajectory.ground_forward_m
            - self.config.tolerances.first_ground_longitudinal_m
            and abs(lateral) <= self.config.tolerances.first_ground_lateral_m
        ):
            self._transition(Phase.GROUND_STOP, now, snapshot)
            return [self._manual("ground_neutral")]
        return [self._manual("ground_forward")]

    def _ground_stop(self, snapshot: Snapshot, now: float) -> List[Action]:
        actions = [self._manual("ground_neutral")]
        if snapshot.armed:
            actions.append(self._action("disarm"))
            return actions
        if self._planar_speed(snapshot) >= self.config.safety.stopped_speed_mps:
            return actions
        self._transition(Phase.FLIGHT_CONFIGURE, now, snapshot)
        return []

    def _flight_configure(self, snapshot: Snapshot, now: float) -> List[Action]:
        if snapshot.mav_type == self.FLIGHT_MAV_TYPE:
            self._transition(Phase.OFFBOARD_PRIME, now, snapshot)
            self._offboard_hold = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
            return []
        if snapshot.armed:
            return [self._manual("neutral"), self._action("disarm")]
        if self._planar_speed(snapshot) >= self.config.safety.stopped_speed_mps:
            return [self._manual("neutral")]
        return [self._action("set_mav_type", mav_type=self.FLIGHT_MAV_TYPE)]

    def _offboard_prime(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self._offboard_hold is None:
            self._offboard_hold = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
        if snapshot.mode == "OFFBOARD":
            if self.takeoff_ground_height is None:
                self.takeoff_ground_height = self._offboard_hold[2]
            self._transition(Phase.TAKEOFF, now, snapshot)
            self._phase_safety_origin = (
                self._offboard_hold[0],
                self._offboard_hold[1],
            )
            return []
        return [
            self._position(self._offboard_hold),
            self._action("set_mode", mode="OFFBOARD"),
        ]

    def _takeoff(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self.takeoff_ground_height is None:
            self.takeoff_ground_height = snapshot.z
        target_z = self.takeoff_ground_height + self.config.trajectory.takeoff_agl_m
        if self._offboard_hold is None:
            self._offboard_hold = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
        target = (
            self._offboard_hold[0],
            self._offboard_hold[1],
            target_z,
            self._offboard_hold[3],
        )
        if snapshot.z >= target_z - self.config.tolerances.altitude_m:
            self._transition(Phase.AIR_FORWARD, now, snapshot)
            self.set_phase_origin(snapshot)
            return []
        actions = [self._position(target)]
        if not snapshot.armed:
            actions.append(self._action("arm"))
        return actions

    def _air_forward(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self.air_forward_origin is None:
            self.set_phase_origin(snapshot)
        origin_x, origin_y, origin_z, origin_yaw = self.air_forward_origin
        longitudinal, lateral = enu_to_longitudinal_lateral(
            snapshot.x - origin_x,
            snapshot.y - origin_y,
            origin_yaw,
        )
        if (
            abs(longitudinal - self.config.trajectory.air_forward_m)
            <= self.config.tolerances.air_longitudinal_m
            and abs(lateral) <= self.config.tolerances.air_lateral_m
        ):
            self._transition(Phase.HOVER, now, snapshot)
            self.set_phase_origin(snapshot)
            self._hover_stable_since = now
            return []
        dx, dy = body_offset_to_enu(
            self.config.trajectory.air_forward_m,
            0.0,
            origin_yaw,
        )
        return [self._position((origin_x + dx, origin_y + dy, origin_z, origin_yaw))]

    def _hover(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self._offboard_hold is None:
            self.set_phase_origin(snapshot)
        altitude_ok = abs(snapshot.z - self._offboard_hold[2]) <= self.config.tolerances.hover_altitude_m
        if not altitude_ok:
            self._hover_stable_since = None
        else:
            if self._hover_stable_since is None:
                self._hover_stable_since = now
            if now - self._hover_stable_since >= self.config.trajectory.hover_s:
                self._transition(Phase.MANUAL_AIR, now, snapshot)
                return []
        return [self._position(self._offboard_hold)]

    def _manual_air(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self._manual_air_braking:
            if snapshot.mode != "MANUAL":
                return [
                    self._manual("neutral"),
                    self._action("set_mode", mode="MANUAL"),
                ]
            if self._manual_air_brake_started_at is None:
                self._manual_air_brake_started_at = now
            brake_elapsed = now - self._manual_air_brake_started_at
            if (
                self._planar_speed(snapshot)
                < self.config.safety.stopped_speed_mps
                or brake_elapsed
                >= self.config.commands.air_manual_brake_s
            ):
                self._transition(Phase.REACQUIRE_OFFBOARD, now, snapshot)
                self.set_phase_origin(snapshot)
            return [self._manual("neutral")]
        if snapshot.mode != "MANUAL":
            self._manual_air_started_at = None
            return [
                self._manual("neutral"),
                self._action("set_mode", mode="MANUAL"),
            ]
        if self._manual_air_started_at is None:
            self._manual_air_started_at = now
            return [self._manual("air_forward")]
        if (
            now - self._manual_air_started_at
            >= self.config.commands.air_manual_duration_s
        ):
            self._manual_air_braking = True
            return [self._manual("neutral")]
        return [self._manual("air_forward")]

    def _reacquire_offboard(self, snapshot: Snapshot, now: float) -> List[Action]:
        if self._offboard_hold is None:
            self.set_phase_origin(snapshot)
        if snapshot.mode == "OFFBOARD":
            self._transition(Phase.LAND, now, snapshot)
            return []
        return [
            self._position(self._offboard_hold),
            self._action("set_mode", mode="OFFBOARD"),
        ]

    def _land(self, snapshot: Snapshot, now: float) -> List[Action]:
        if snapshot.landed:
            self.record_landing_origin(snapshot)
            self._transition(Phase.GROUND_RECONFIGURE, now, snapshot)
            return []
        return [self._action("set_mode", mode="AUTO.LAND")]

    def _ground_reconfigure(self, snapshot: Snapshot, now: float) -> List[Action]:
        return self._configure_ground_for(snapshot, now, Phase.GROUND_TURN)

    def _ground_turn(self, snapshot: Snapshot, now: float) -> List[Action]:
        blocked = self._ground_motion_blocked(snapshot)
        if blocked is not None:
            return blocked
        if self.phase_origin is None:
            self.set_phase_origin(snapshot)
        yaw_change = self._signed_angle(snapshot.yaw - self.phase_origin.yaw)
        if yaw_change >= math.radians(45.0):
            self._transition(Phase.GROUND_DIAGONAL, now, snapshot)
            self.set_phase_origin(snapshot)
            return [self._manual("ground_neutral")]
        return [self._manual("ground_left_turn")]

    def _ground_diagonal(self, snapshot: Snapshot, now: float) -> List[Action]:
        blocked = self._ground_motion_blocked(snapshot)
        if blocked is not None:
            return blocked
        if self.final_ground_target is None:
            self.record_landing_origin(snapshot)
        target_x, target_y = self.final_ground_target
        landing_yaw = self.landing_origin[2]
        forward_error, left_error = enu_to_longitudinal_lateral(
            target_x - snapshot.x,
            target_y - snapshot.y,
            landing_yaw,
        )
        if (
            abs(forward_error) <= self.config.tolerances.final_forward_m
            and abs(left_error) <= self.config.tolerances.final_left_m
        ):
            self._transition(Phase.COMPLETE, now, snapshot)
            return [self._manual("ground_neutral")]
        return [self._manual("ground_forward")]

    def _calculate_final_ground_target(self) -> None:
        if self.landing_origin is None:
            return
        landing_x, landing_y, landing_yaw = self.landing_origin
        dx, dy = body_offset_to_enu(
            self.config.trajectory.final_forward_m,
            self.config.trajectory.final_left_m,
            landing_yaw,
        )
        self.final_ground_target = (landing_x + dx, landing_y + dy)

    @staticmethod
    def _signed_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
