"""Immutable domain records for the flying-car experiment."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Optional, Tuple


@dataclass(frozen=True)
class TrajectoryConfig:
    ground_forward_m: float
    takeoff_agl_m: float
    air_forward_m: float
    hover_s: float
    final_forward_m: float
    final_left_m: float


@dataclass(frozen=True)
class TolerancesConfig:
    first_ground_longitudinal_m: float
    first_ground_lateral_m: float
    altitude_m: float
    air_longitudinal_m: float
    air_lateral_m: float
    hover_altitude_m: float
    final_forward_m: float
    final_left_m: float


@dataclass(frozen=True)
class TimeoutsConfig:
    precheck_s: float
    transition_s: float
    ground_forward_s: float
    takeoff_s: float
    air_forward_s: float
    landing_s: float
    final_ground_s: float


@dataclass(frozen=True)
class ManualControl:
    x: float
    y: float
    z: float
    r: float


@dataclass(frozen=True)
class CommandsConfig:
    manual_neutral: ManualControl
    ground_neutral: ManualControl
    ground_forward: ManualControl
    ground_left_turn: ManualControl
    air_manual_forward: ManualControl
    air_manual_duration_s: float
    air_manual_brake_s: float


@dataclass(frozen=True)
class SafetyConfig:
    heartbeat_timeout_s: float
    soft_tilt_deg: float
    hard_tilt_deg: float
    maximum_agl_m: float
    maximum_planar_deviation_m: float
    stopped_speed_mps: float


@dataclass(frozen=True)
class ExperimentConfig:
    publish_rate_hz: float
    model_name: str
    trajectory: TrajectoryConfig
    tolerances: TolerancesConfig
    timeouts: TimeoutsConfig
    commands: CommandsConfig
    safety: SafetyConfig


class Phase(str, Enum):
    PRECHECK = "PRECHECK"
    GROUND_CONFIGURE = "GROUND_CONFIGURE"
    GROUND_FORWARD = "GROUND_FORWARD"
    GROUND_STOP = "GROUND_STOP"
    FLIGHT_CONFIGURE = "FLIGHT_CONFIGURE"
    OFFBOARD_PRIME = "OFFBOARD_PRIME"
    TAKEOFF = "TAKEOFF"
    AIR_FORWARD = "AIR_FORWARD"
    HOVER = "HOVER"
    MANUAL_AIR = "MANUAL_AIR"
    REACQUIRE_OFFBOARD = "REACQUIRE_OFFBOARD"
    LAND = "LAND"
    GROUND_RECONFIGURE = "GROUND_RECONFIGURE"
    GROUND_TURN = "GROUND_TURN"
    GROUND_DIAGONAL = "GROUND_DIAGONAL"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class Snapshot:
    stamp: float
    connected: bool
    armed: bool
    mode: str
    landed: bool
    mav_type: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    clock_ok: bool
    processes_ok: bool


@dataclass(frozen=True)
class Action:
    kind: str
    values: Mapping[str, object]


@dataclass(frozen=True)
class PhaseResult:
    phase: Phase
    passed: bool
    elapsed_s: float
    detail: str = ""
    final_snapshot: Optional[Snapshot] = None


@dataclass(frozen=True)
class RunResult:
    passed: bool
    phase_results: Tuple[PhaseResult, ...]
    reason: str = ""
