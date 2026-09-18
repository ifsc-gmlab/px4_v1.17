import math
from dataclasses import replace

import pytest

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
from flying_car_test.state_machine import ExperimentStateMachine


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


@pytest.fixture
def machine(config):
    return ExperimentStateMachine(config)


def kinds(actions):
    return [action.kind for action in actions]


def test_precheck_requires_feedback_then_enters_ground_configure(machine, snapshot):
    actions = machine.advance(snapshot, now=0.0)

    assert actions == []
    assert machine.phase == Phase.GROUND_CONFIGURE


def test_ground_configure_repeats_request_until_feedback(machine, snapshot):
    machine.enter(Phase.GROUND_CONFIGURE, now=0.0)

    actions = machine.advance(
        replace(snapshot, stamp=3.0, mav_type=14), now=3.0
    )

    assert kinds(actions) == ["manual", "set_mav_type"]
    assert actions[1].values["mav_type"] == machine.GROUND_MAV_TYPE
    assert machine.phase == Phase.GROUND_CONFIGURE
    mode_actions = machine.advance(
        replace(snapshot, stamp=3.1, mode="OFFBOARD"),
        now=3.1,
    )
    arm_actions = machine.advance(replace(snapshot, stamp=3.2), now=3.2)
    assert kinds(mode_actions) == ["manual", "set_mode"]
    assert mode_actions[1].values["mode"] == "MANUAL"
    assert kinds(arm_actions) == ["manual", "arm"]
    assert machine.phase == Phase.GROUND_CONFIGURE
    machine.advance(replace(snapshot, stamp=3.3, armed=True), now=3.3)
    assert machine.phase == Phase.GROUND_FORWARD


def test_never_changes_mav_type_while_armed(machine, snapshot):
    machine.enter(Phase.FLIGHT_CONFIGURE, now=0.0)

    actions = machine.advance(replace(snapshot, armed=True), now=1.0)

    assert all(action.kind != "set_mav_type" for action in actions)


def test_never_changes_mav_type_while_vehicle_is_moving(machine, snapshot):
    machine.enter(Phase.GROUND_RECONFIGURE, now=0.0)

    actions = machine.advance(replace(snapshot, mav_type=14, vx=0.2), now=1.0)

    assert all(action.kind != "set_mav_type" for action in actions)


def test_ground_distance_causes_neutral_then_ground_stop(machine, snapshot):
    snapshot = replace(snapshot, armed=True)
    machine.enter(Phase.GROUND_FORWARD, now=0.0)
    machine.set_phase_origin(snapshot)
    reached = replace(snapshot, x=snapshot.x + 5.0, stamp=5.0)

    actions = machine.advance(reached, now=5.0)

    assert actions[0].kind == "manual"
    assert actions[0].values["name"] == "ground_neutral"
    assert machine.phase == Phase.GROUND_STOP


def test_ground_forward_uses_start_heading_and_lateral_tolerance(machine, snapshot):
    start = replace(snapshot, armed=True, yaw=math.pi / 2.0)
    machine.enter(Phase.GROUND_FORWARD, now=0.0)
    machine.set_phase_origin(start)

    actions = machine.advance(replace(start, x=8.5, y=25.0, stamp=1.0), now=1.0)

    assert actions[0].kind == "manual"
    assert actions[0].values["name"] == "ground_forward"
    assert machine.phase == Phase.GROUND_FORWARD


def test_ground_stop_disarms_then_waits_for_stopped_feedback(machine, snapshot):
    machine.enter(Phase.GROUND_STOP, now=0.0)

    armed_actions = machine.advance(replace(snapshot, armed=True), now=0.1)
    moving_actions = machine.advance(replace(snapshot, vx=0.2), now=0.2)
    machine.advance(snapshot, now=0.3)

    assert "disarm" in kinds(armed_actions)
    assert kinds(moving_actions) == ["manual"]
    assert machine.phase == Phase.FLIGHT_CONFIGURE


def test_flight_configure_waits_for_disarmed_stopped_feedback(machine, snapshot):
    machine.enter(Phase.FLIGHT_CONFIGURE, now=0.0)

    actions = machine.advance(snapshot, now=0.1)

    assert kinds(actions) == ["set_mav_type"]
    assert actions[0].values["mav_type"] == machine.FLIGHT_MAV_TYPE
    assert machine.phase == Phase.FLIGHT_CONFIGURE
    machine.advance(replace(snapshot, mav_type=machine.FLIGHT_MAV_TYPE), now=0.2)
    assert machine.phase == Phase.OFFBOARD_PRIME


def test_offboard_priming_publishes_setpoint_before_mode_request(machine, snapshot):
    machine.enter(Phase.OFFBOARD_PRIME, now=0.0)

    actions = machine.advance(replace(snapshot, mav_type=14), now=0.1)

    assert kinds(actions) == ["position", "set_mode"]
    assert actions[1].values["mode"] == "OFFBOARD"
    machine.advance(replace(snapshot, mav_type=14, mode="OFFBOARD"), now=0.2)
    assert machine.phase == Phase.TAKEOFF


def test_takeoff_uses_captured_ground_height_and_feedback_altitude(machine, snapshot):
    machine.enter(Phase.OFFBOARD_PRIME, now=0.0)
    machine.advance(replace(snapshot, mav_type=14, mode="OFFBOARD"), now=0.1)

    actions = machine.advance(replace(snapshot, mav_type=14, mode="OFFBOARD", armed=True), now=0.2)
    reached = replace(
        snapshot,
        stamp=1.0,
        mav_type=14,
        mode="OFFBOARD",
        armed=True,
        z=6.3,
        landed=False,
    )
    machine.advance(reached, now=1.0)

    assert actions[0].kind == "position"
    assert actions[0].values["z"] == pytest.approx(7.0)
    assert machine.phase == Phase.AIR_FORWARD


def test_air_forward_completion_uses_start_heading_projection(machine, snapshot):
    start = replace(snapshot, x=2.0, y=3.0, z=7.0, yaw=math.pi / 2.0, landed=False, armed=True)
    machine.enter(Phase.AIR_FORWARD, now=0.0)
    machine.set_phase_origin(start)

    not_reached = replace(start, x=4.0, y=8.0, stamp=1.0)
    reached = replace(start, x=2.5, y=7.2, stamp=2.0)
    actions = machine.advance(not_reached, now=1.0)
    machine.advance(reached, now=2.0)

    assert actions[0].kind == "position"
    assert machine.phase == Phase.HOVER


def test_air_forward_origin_is_immutable(machine, snapshot):
    first = replace(snapshot, x=2.0, y=3.0, z=7.0, yaw=0.4)
    second = replace(snapshot, x=20.0, y=30.0, z=6.0, yaw=1.4)
    machine.enter(Phase.AIR_FORWARD, now=0.0)

    machine.set_phase_origin(first)
    machine.set_phase_origin(second)

    assert machine.air_forward_origin == pytest.approx((2.0, 3.0, 7.0, 0.4))


def test_hover_resets_timer_and_requires_three_continuous_stable_seconds(machine, snapshot):
    machine.config = replace(
        machine.config,
        timeouts=replace(machine.config.timeouts, transition_s=10.0),
    )
    airborne = replace(snapshot, z=7.0, landed=False, armed=True, mode="OFFBOARD")
    machine.enter(Phase.HOVER, now=0.0)
    machine.set_phase_origin(airborne)

    machine.advance(airborne, now=0.0)
    machine.advance(replace(airborne, stamp=0.2, z=8.0), now=0.2)
    machine.advance(replace(airborne, stamp=0.5), now=0.5)
    machine.advance(replace(airborne, stamp=3.49), now=3.49)
    assert machine.phase == Phase.HOVER
    machine.advance(replace(airborne, stamp=3.5), now=3.5)

    assert machine.phase == Phase.MANUAL_AIR


def test_manual_air_waits_for_mode_then_neutralizes_and_waits_for_stop(machine, snapshot):
    airborne = replace(snapshot, z=7.0, landed=False, armed=True)
    machine.enter(Phase.MANUAL_AIR, now=0.0)

    waiting = machine.advance(
        replace(airborne, stamp=0.1, mode="OFFBOARD"),
        now=0.1,
    )
    moving = machine.advance(replace(airborne, stamp=0.2, mode="MANUAL"), now=0.2)
    neutralizing = machine.advance(
        replace(airborne, stamp=1.2, mode="MANUAL", vx=1.0),
        now=1.2,
    )
    still_braking = machine.advance(
        replace(airborne, stamp=1.3, mode="MANUAL", vx=0.2),
        now=1.3,
    )

    assert kinds(waiting) == ["manual", "set_mode"]
    assert waiting[0].values["name"] == "neutral"
    assert moving[0].values["name"] == "air_forward"
    assert neutralizing[0].values["name"] == "neutral"
    assert still_braking[0].values["name"] == "neutral"
    assert machine.phase == Phase.MANUAL_AIR
    machine.advance(
        replace(airborne, stamp=1.4, mode="MANUAL", vx=0.14),
        now=1.4,
    )
    assert machine.phase == Phase.REACQUIRE_OFFBOARD


def test_manual_air_mode_never_confirmed_cannot_send_motion(machine, snapshot):
    airborne = replace(snapshot, z=7.0, landed=False, armed=True, mode="OFFBOARD")
    machine.enter(Phase.MANUAL_AIR, now=0.0)

    first = machine.advance(airborne, now=0.0)
    second = machine.advance(replace(airborne, stamp=1.0), now=1.0)

    assert all(
        action.values.get("name") != "air_forward" for action in first + second
    )
    assert machine.phase == Phase.MANUAL_AIR


def test_manual_air_braking_latches_across_temporary_mode_loss(machine, snapshot):
    airborne = replace(
        snapshot,
        armed=True,
        landed=False,
        mode="MANUAL",
        z=7.0,
        vx=1.0,
    )
    machine.enter(Phase.MANUAL_AIR, now=0.0)

    moving = machine.advance(airborne, now=0.0)
    braking = machine.advance(replace(airborne, stamp=1.0), now=1.0)
    mode_lost = machine.advance(
        replace(airborne, stamp=1.1, mode="OFFBOARD"),
        now=1.1,
    )
    mode_restored = machine.advance(
        replace(airborne, stamp=1.2, mode="MANUAL"),
        now=1.2,
    )

    assert moving[0].values["name"] == "air_forward"
    assert braking[0].values["name"] == "neutral"
    assert kinds(mode_lost) == ["manual", "set_mode"]
    assert mode_lost[0].values["name"] == "neutral"
    assert mode_restored[0].values["name"] == "neutral"
    assert all(
        action.values.get("name") != "air_forward"
        for action in braking + mode_lost + mode_restored
    )
    assert machine.phase == Phase.MANUAL_AIR

    stopped = machine.advance(
        replace(airborne, stamp=1.3, mode="MANUAL", vx=0.14),
        now=1.3,
    )
    assert stopped[0].values["name"] == "neutral"
    assert machine.phase == Phase.REACQUIRE_OFFBOARD


def test_reacquire_offboard_primes_before_mode_request(machine, snapshot):
    airborne = replace(snapshot, z=7.0, landed=False, armed=True, mode="MANUAL")
    machine.enter(Phase.REACQUIRE_OFFBOARD, now=0.0)
    machine.set_phase_origin(airborne)

    actions = machine.advance(airborne, now=0.1)

    assert kinds(actions) == ["position", "set_mode"]
    machine.advance(replace(airborne, mode="OFFBOARD"), now=0.2)
    assert machine.phase == Phase.LAND


def test_landing_requires_landed_confirmation(machine, snapshot):
    airborne = replace(snapshot, z=7.0, landed=False, armed=True, mode="OFFBOARD")
    machine.enter(Phase.LAND, now=0.0)

    actions = machine.advance(airborne, now=0.1)
    machine.advance(
        replace(snapshot, stamp=1.0, armed=False, landed=True, x=12.0, y=21.0, yaw=0.4),
        now=1.0,
    )

    assert kinds(actions) == ["set_mode"]
    assert actions[0].values["mode"] == "AUTO.LAND"
    assert machine.phase == Phase.GROUND_RECONFIGURE
    assert machine.landing_origin == pytest.approx((12.0, 21.0, 0.4))


def test_ground_reconfigure_disarms_before_changing_type(machine, snapshot):
    machine.enter(Phase.GROUND_RECONFIGURE, now=0.0)
    flight = replace(snapshot, stamp=3.0, mav_type=14, armed=True)

    armed_actions = machine.advance(flight, now=3.0)
    disarmed_actions = machine.advance(
        replace(flight, stamp=3.1, armed=False), now=3.1
    )
    mode_actions = machine.advance(
        replace(snapshot, stamp=3.2, mode="OFFBOARD"),
        now=3.2,
    )
    arm_actions = machine.advance(replace(snapshot, stamp=3.3), now=3.3)
    machine.advance(replace(snapshot, stamp=3.4, armed=True), now=3.4)

    assert "set_mav_type" not in kinds(armed_actions)
    assert "disarm" in kinds(armed_actions)
    assert kinds(disarmed_actions) == ["manual", "set_mav_type"]
    assert kinds(mode_actions) == ["manual", "set_mode"]
    assert kinds(arm_actions) == ["manual", "arm"]
    assert machine.phase == Phase.GROUND_TURN


def test_ground_turn_completes_after_45_degree_left_yaw(machine, snapshot):
    start = replace(snapshot, armed=True, yaw=0.2)
    machine.enter(Phase.GROUND_TURN, now=0.0)
    machine.set_phase_origin(start)

    actions = machine.advance(
        replace(start, stamp=0.5, yaw=0.2 + math.radians(44.0)),
        now=0.5,
    )
    final_actions = machine.advance(
        replace(start, stamp=1.0, yaw=0.2 + math.radians(45.0)),
        now=1.0,
    )

    assert actions[0].values["name"] == "ground_left_turn"
    assert final_actions[0].values["name"] == "ground_neutral"
    assert machine.phase == Phase.GROUND_DIAGONAL


def test_ground_turn_does_not_treat_small_right_turn_as_left_turn(machine, snapshot):
    start = replace(snapshot, armed=True, yaw=0.2)
    machine.enter(Phase.GROUND_TURN, now=0.0)
    machine.set_phase_origin(start)

    actions = machine.advance(
        replace(start, stamp=0.5, yaw=0.2 - math.radians(1.0)),
        now=0.5,
    )

    assert actions[0].values["name"] == "ground_left_turn"
    assert machine.phase == Phase.GROUND_TURN


def test_final_target_is_immutable_from_landing_pose(machine, snapshot):
    landing = replace(snapshot, x=10.0, y=20.0, yaw=math.pi / 2.0)
    machine.record_landing_origin(landing)
    machine.record_landing_origin(replace(landing, x=99.0, y=99.0, yaw=0.0))
    machine.enter(Phase.GROUND_DIAGONAL, now=0.0)
    machine.set_phase_origin(replace(landing, x=99.0, y=99.0, yaw=2.0))

    assert machine.final_ground_target == pytest.approx((7.0, 23.0))


def test_final_forward_left_target_requires_both_projected_errors(machine, snapshot):
    landing = replace(snapshot, armed=True, x=10.0, y=20.0, yaw=math.pi / 2.0)
    machine.record_landing_origin(landing)
    machine.enter(Phase.GROUND_DIAGONAL, now=0.0)

    outside_left = replace(landing, stamp=0.5, x=8.6, y=23.0)
    actions = machine.advance(outside_left, now=0.5)
    machine.advance(replace(landing, stamp=1.0, x=7.5, y=22.5), now=1.0)

    assert actions[0].values["name"] == "ground_forward"
    assert machine.phase == Phase.COMPLETE


@pytest.mark.parametrize(
    ("phase", "timeout"),
    [
        (Phase.PRECHECK, 30.0),
        (Phase.GROUND_CONFIGURE, 5.0),
        (Phase.GROUND_FORWARD, 30.0),
        (Phase.GROUND_STOP, 5.0),
        (Phase.FLIGHT_CONFIGURE, 5.0),
        (Phase.OFFBOARD_PRIME, 5.0),
        (Phase.TAKEOFF, 30.0),
        (Phase.AIR_FORWARD, 20.0),
        (Phase.HOVER, 5.0),
        (Phase.MANUAL_AIR, 5.0),
        (Phase.REACQUIRE_OFFBOARD, 5.0),
        (Phase.LAND, 45.0),
        (Phase.GROUND_RECONFIGURE, 5.0),
        (Phase.GROUND_TURN, 5.0),
        (Phase.GROUND_DIAGONAL, 40.0),
    ],
)
def test_any_active_phase_timeout_aborts(machine, snapshot, phase, timeout):
    machine.enter(phase, now=0.0)

    actions = machine.advance(replace(snapshot, stamp=timeout + 0.01), now=timeout + 0.01)

    assert machine.phase == Phase.ABORT
    assert actions[-1].kind == "abort"
    assert actions[-1].values["reason"] == "phase_timeout"


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"roll": math.radians(45.1)}, "hard_tilt"),
        ({"pitch": -math.radians(45.1)}, "hard_tilt"),
        ({"clock_ok": False}, "clock_failure"),
        ({"processes_ok": False}, "process_failure"),
        ({"connected": False}, "connection_loss"),
        ({"stamp": -0.31}, "heartbeat_timeout"),
        ({"x": 20.01}, "maximum_planar_deviation"),
    ],
)
def test_hard_safety_conditions_abort_before_phase_actions(machine, snapshot, changes, reason):
    machine.enter(Phase.GROUND_FORWARD, now=0.0)
    machine.set_phase_origin(snapshot)

    actions = machine.advance(replace(snapshot, **changes), now=0.0)

    assert machine.phase == Phase.ABORT
    assert kinds(actions) == ["abort"]
    assert actions[0].values["reason"] == reason


def test_monotonic_clock_regression_aborts(machine, snapshot):
    machine.enter(Phase.TAKEOFF, now=2.0)

    actions = machine.advance(snapshot, now=1.0)

    assert machine.phase == Phase.ABORT
    assert actions[-1].values["reason"] == "clock_failure"


def test_maximum_agl_aborts_relative_to_captured_ground(machine, snapshot):
    machine.takeoff_ground_height = snapshot.z
    machine.enter(Phase.TAKEOFF, now=0.0)

    actions = machine.advance(replace(snapshot, z=10.01), now=0.1)

    assert kinds(actions) == ["abort"]
    assert actions[0].values["reason"] == "maximum_agl"


@pytest.mark.parametrize(
    "phase",
    [Phase.GROUND_FORWARD, Phase.GROUND_TURN, Phase.GROUND_DIAGONAL],
)
def test_ground_motion_never_moves_before_armed_feedback(machine, snapshot, phase):
    machine.enter(phase, now=0.0)
    machine.set_phase_origin(snapshot)
    if phase == Phase.GROUND_DIAGONAL:
        machine.record_landing_origin(snapshot)

    actions = machine.advance(snapshot, now=0.1)

    assert kinds(actions) == ["manual", "arm"]
    assert actions[0].values["name"] == "ground_neutral"
    assert machine.phase == phase


def test_ground_motion_restores_manual_mode_before_requesting_arm(machine, snapshot):
    machine.enter(Phase.GROUND_FORWARD, now=0.0)

    actions = machine.advance(replace(snapshot, mode="OFFBOARD"), now=0.1)

    assert kinds(actions) == ["manual", "set_mode"]
    assert actions[0].values["name"] == "ground_neutral"
    assert actions[1].values["mode"] == "MANUAL"


def test_takeoff_keeps_fixed_horizontal_target_and_enforces_deviation(machine, snapshot):
    flight = replace(snapshot, mav_type=14, mode="OFFBOARD")
    machine.enter(Phase.OFFBOARD_PRIME, now=0.0)
    machine.advance(flight, now=0.1)

    setpoint = machine.advance(
        replace(flight, stamp=0.2, armed=True, x=11.0, y=21.0),
        now=0.2,
    )[0]
    abort = machine.advance(
        replace(flight, stamp=0.3, armed=True, x=20.01),
        now=0.3,
    )

    assert setpoint.kind == "position"
    assert setpoint.values["x"] == pytest.approx(10.0)
    assert setpoint.values["y"] == pytest.approx(20.0)
    assert kinds(abort) == ["abort"]
    assert abort[0].values["reason"] == "maximum_planar_deviation"


@pytest.mark.parametrize("phase", [Phase.MANUAL_AIR, Phase.LAND])
def test_air_motion_phases_always_enforce_planar_deviation(machine, snapshot, phase):
    airborne = replace(snapshot, armed=True, landed=False, mode="MANUAL", z=7.0)
    machine.enter(phase, now=0.0)
    machine.advance(airborne, now=0.0)

    actions = machine.advance(
        replace(airborne, stamp=0.1, x=20.01),
        now=0.1,
    )

    assert kinds(actions) == ["abort"]
    assert actions[0].values["reason"] == "maximum_planar_deviation"


def test_precheck_captures_ground_height_before_agl_limit(machine, snapshot):
    elevated_ground = replace(snapshot, z=100.0)

    actions = machine.advance(elevated_ground, now=0.0)

    assert actions == []
    assert machine.takeoff_ground_height == pytest.approx(100.0)
    assert machine.phase == Phase.GROUND_CONFIGURE
    machine.enter(Phase.TAKEOFF, now=0.1)
    abort = machine.advance(
        replace(elevated_ground, stamp=0.2, z=108.01),
        now=0.2,
    )
    assert abort[0].values["reason"] == "maximum_agl"


def test_maximum_agl_is_relative_to_captured_takeoff_ground(machine, snapshot):
    machine.enter(Phase.TAKEOFF, now=0.0)
    machine.set_phase_origin(snapshot)

    actions = machine.advance(replace(snapshot, z=9.0), now=0.1)

    assert "abort" not in kinds(actions)
    assert machine.phase == Phase.AIR_FORWARD


def test_complete_and_abort_are_terminal(machine, snapshot):
    machine.enter(Phase.COMPLETE, now=0.0)
    assert machine.advance(snapshot, now=100.0) == []
    machine.enter(Phase.ABORT, now=0.0)
    assert machine.advance(snapshot, now=100.0) == []


def test_actions_are_domain_records(machine, snapshot):
    machine.enter(Phase.GROUND_FORWARD, now=0.0)

    actions = machine.advance(snapshot, now=0.1)

    assert actions
    assert all(isinstance(action, Action) for action in actions)
