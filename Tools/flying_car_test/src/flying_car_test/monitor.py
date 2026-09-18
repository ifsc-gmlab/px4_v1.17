"""Passive ROS telemetry aggregation for flying-car experiments."""

import json
import math
import threading
from pathlib import Path
from typing import Mapping

from .geometry import quaternion_to_rpy
from .recording import EvidenceRecorder, build_metadata


GAZEBO_MODEL_NAME = "fs_fasmv8_flying_car"
LANDED_STATE_ON_GROUND = 1
SAMPLE_RATE_HZ = 30.0
MAVLINK_FRAMING_OK = 1
MAVLINK_HEARTBEAT_MSGID = 0
MAVLINK_HEARTBEAT_MIN_LENGTH = 9
MAV_TYPE_FRESH_SECONDS = 2.0
DEFAULT_SYSTEM_ID = 1
DEFAULT_AUTOPILOT_COMPONENT_ID = 1


def _message_seconds(message) -> float:
    value = getattr(message, "clock", message)
    to_sec = getattr(value, "to_sec", None)
    return float(to_sec() if to_sec is not None else value)


def _string_payload(message):
    return getattr(message, "data", message)


def _decoded_object(message, fallback_key="raw") -> Mapping[str, object]:
    payload = _string_payload(message)
    if isinstance(payload, Mapping):
        return dict(payload)
    try:
        decoded = json.loads(str(payload))
    except (TypeError, ValueError):
        return {fallback_key: str(payload)}
    if isinstance(decoded, Mapping):
        return dict(decoded)
    return {fallback_key: decoded}


class PassiveMonitor:
    """Aggregate subscribed values without publishing commands or services."""

    def __init__(
        self,
        recorder: EvidenceRecorder,
        system_id: int = DEFAULT_SYSTEM_ID,
        autopilot_component_id: int = DEFAULT_AUTOPILOT_COMPONENT_ID,
    ):
        self.recorder = recorder
        self._lock = threading.RLock()
        self._callback_condition = threading.Condition()
        self._active_abort_callbacks = 0
        self._producer_stopped = False
        self._system_id = int(system_id)
        self._autopilot_component_id = int(autopilot_component_id)
        self._connected = False
        self._armed = False
        self._mode = ""
        self._landed = False
        self._mav_type = None
        self._mav_type_stamp = None
        self._local = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._gazebo = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._clock = 0.0
        self._clock_seen = False
        self._clock_ok = True
        self._phase = ""

    @property
    def phase(self) -> str:
        with self._lock:
            return self._phase

    def on_state(self, message) -> None:
        with self._lock:
            self._connected = bool(message.connected)
            self._armed = bool(message.armed)
            self._mode = str(message.mode)

    def on_extended_state(self, message) -> None:
        with self._lock:
            self._landed = (
                int(message.landed_state) == LANDED_STATE_ON_GROUND
            )

    def on_local_pose(self, message) -> None:
        pose = message.pose
        position = pose.position
        orientation = pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        with self._lock:
            self._local = {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
            }
            self._attitude = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }

    def on_local_velocity(self, message) -> None:
        velocity = message.twist.linear
        with self._lock:
            self._velocity = {
                "x": float(velocity.x),
                "y": float(velocity.y),
                "z": float(velocity.z),
            }

    def on_model_states(self, message) -> None:
        try:
            index = list(message.name).index(GAZEBO_MODEL_NAME)
            position = message.pose[index].position
        except (ValueError, IndexError, AttributeError):
            return
        with self._lock:
            self._gazebo = {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
            }

    def on_clock(self, message) -> None:
        try:
            value = _message_seconds(message)
        except (TypeError, ValueError, OverflowError):
            with self._lock:
                self._clock_ok = False
            return
        with self._lock:
            if not math.isfinite(value):
                self._clock_ok = False
                return
            if self._clock_seen and value < self._clock:
                self._clock_ok = False
            self._clock = value
            self._clock_seen = True

    def on_mavlink(self, message) -> None:
        """Capture MAV_TYPE from a valid autopilot HEARTBEAT frame."""
        try:
            if int(message.framing_status) != MAVLINK_FRAMING_OK:
                return
            if int(message.msgid) != MAVLINK_HEARTBEAT_MSGID:
                return
            length = int(message.len)
            if length < MAVLINK_HEARTBEAT_MIN_LENGTH:
                return
            if int(message.sysid) != self._system_id:
                return
            if int(message.compid) != self._autopilot_component_id:
                return
            payload = list(message.payload64)
            required_words = (length + 7) // 8
            if len(payload) < required_words:
                return
            stamp = _message_seconds(message.header.stamp)
            first_word = int(payload[0])
            mav_type = (first_word >> 32) & 0xFF
            if not math.isfinite(stamp):
                return
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self._lock:
            if (
                self._mav_type_stamp is not None
                and stamp < self._mav_type_stamp
            ):
                return
            self._mav_type = mav_type
            self._mav_type_stamp = stamp

    def on_phase(self, message) -> None:
        phase = str(_string_payload(message))
        with self._lock:
            self._phase = phase
            stamp = self._clock
        self._append_if_open(self.recorder.append_event, {
            "stamp": stamp,
            "type": "phase",
            "phase": phase,
        })

    def on_command(self, message) -> None:
        command = _decoded_object(message)
        with self._lock:
            command.setdefault("observed_clock", self._clock)
        self._append_if_open(self.recorder.append_command, command)

    def on_abort(self, message) -> None:
        with self._callback_condition:
            if self._producer_stopped:
                return
            self._active_abort_callbacks += 1
        try:
            event = _decoded_object(message)
            reason = str(event.get("reason", "controller abort"))
            event["type"] = "abort"
            with self._lock:
                event.setdefault("observed_clock", self._clock)
            self.recorder.record_abort(event, reason)
        finally:
            with self._callback_condition:
                self._active_abort_callbacks -= 1
                self._callback_condition.notify_all()

    def sample(self, stamp: float) -> Mapping[str, object]:
        with self._lock:
            stamp = float(stamp)
            mav_type_fresh = (
                self._mav_type_stamp is not None
                and 0.0 <= stamp - self._mav_type_stamp
                <= MAV_TYPE_FRESH_SECONDS
            )
            return {
                "stamp": stamp,
                "clock": self._clock,
                "connected": self._connected,
                "armed": self._armed,
                "mode": self._mode,
                "landed": self._landed,
                "mav_type": self._mav_type,
                "mav_type_fresh": mav_type_fresh,
                "local": dict(self._local),
                "velocity": dict(self._velocity),
                "attitude": dict(self._attitude),
                "gazebo": dict(self._gazebo),
                "clock_ok": self._clock_ok,
            }

    def record_sample(self, stamp: float) -> None:
        self._append_if_open(
            self.recorder.append_sample, self.sample(stamp)
        )

    def record_sample_safely(self, stamp: float, signal_shutdown) -> bool:
        """Turn a timer I/O error into a latched abnormal shutdown."""
        try:
            self.record_sample(stamp)
            return True
        except Exception as error:
            reason = self.recorder.abort(
                "sample recording failed: {}".format(error)
            )
            signal_shutdown(reason)
            return False

    def _append_if_open(self, append, record) -> None:
        try:
            append(record)
        except RuntimeError:
            # A callback may already be queued when ROS shutdown closes files.
            if not self.recorder.closed:
                raise

    def close(self) -> bool:
        with self._callback_condition:
            self._producer_stopped = True
            while self._active_abort_callbacks:
                self._callback_condition.wait()
        clean = self.phase == "COMPLETE" and self.recorder.abort_reason is None
        return self.recorder.close(
            clean=clean,
            abort_reason=None if clean else "ROS shutdown before COMPLETE",
        )


def main() -> None:
    """Run the passive ROS monitoring node."""
    import rospy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import ExtendedState, Mavlink, State
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    rospy.init_node("flying_car_experiment_monitor")
    repository = Path(__file__).resolve().parents[4]
    config_path = Path(rospy.get_param(
        "~config",
        str(repository / "Tools/flying_car_test/config/experiment.yaml"),
    ))
    model_sdf = Path(rospy.get_param(
        "~model_sdf",
        str(
            repository
            / "Tools/simulation/gazebo-classic/sitl_gazebo-classic"
            / "models/FS-FASMv8_octo/FS-FASMv8_octo.sdf"
        ),
    ))
    airframe = Path(rospy.get_param(
        "~airframe",
        str(
            repository
            / "ROMFS/px4fmu_common/init.d-posix/airframes"
            / "80004_gazebo-classic_fs_fasmv8_flying_car"
        ),
    ))
    launch_arguments = rospy.get_param("~launch_arguments", {})
    if not isinstance(launch_arguments, Mapping):
        raise ValueError("~launch_arguments must be a mapping")
    metadata = build_metadata(
        repository_root=repository,
        model_sdf=model_sdf,
        airframe=airframe,
        experiment_config=config_path,
        launch_arguments=launch_arguments,
    )
    recorder = EvidenceRecorder(
        rospy.get_param("~result_dir"),
        metadata=metadata,
        repository_root=repository,
    )
    monitor = PassiveMonitor(
        recorder,
        system_id=int(
            rospy.get_param("~target_system_id", DEFAULT_SYSTEM_ID)
        ),
        autopilot_component_id=int(
            rospy.get_param(
                "~target_component_id",
                DEFAULT_AUTOPILOT_COMPONENT_ID,
            )
        ),
    )
    closed = [False]
    timer = [None]
    subscriptions = []
    shutdown_lock = threading.Lock()

    def shutdown() -> None:
        with shutdown_lock:
            if closed[0]:
                return
            if (
                timer[0] is not None
                and timer[0] is not threading.current_thread()
            ):
                timer[0].shutdown()
            for subscription in subscriptions:
                try:
                    subscription.unregister()
                except Exception:
                    pass
            monitor.close()
            closed[0] = recorder.closed

    rospy.on_shutdown(shutdown)
    try:
        subscriptions.extend((
            rospy.Subscriber(
                "/mavros/state", State, monitor.on_state, queue_size=10
            ),
            rospy.Subscriber(
                "/mavros/extended_state",
                ExtendedState,
                monitor.on_extended_state,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/mavros/local_position/pose",
                PoseStamped,
                monitor.on_local_pose,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/mavros/local_position/velocity_local",
                TwistStamped,
                monitor.on_local_velocity,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/gazebo/model_states",
                ModelStates,
                monitor.on_model_states,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/clock", Clock, monitor.on_clock, queue_size=10
            ),
            rospy.Subscriber(
                "/mavlink/from",
                Mavlink,
                monitor.on_mavlink,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/flying_car_experiment_controller/phase",
                String,
                monitor.on_phase,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/flying_car_experiment_controller/command",
                String,
                monitor.on_command,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/flying_car_experiment_controller/abort",
                String,
                monitor.on_abort,
                queue_size=10,
            ),
        ))
        timer[0] = rospy.Timer(
            rospy.Duration(1.0 / SAMPLE_RATE_HZ),
            lambda _event: monitor.record_sample_safely(
                rospy.Time.now().to_sec(),
                rospy.signal_shutdown,
            ),
        )
        rospy.spin()
    except BaseException as error:
        recorder.abort(
            "monitor exception {}: {}".format(type(error).__name__, error)
        )
        raise
    finally:
        shutdown()
        # Keep subscriptions strongly referenced for the complete lifecycle.
        _ = subscriptions
