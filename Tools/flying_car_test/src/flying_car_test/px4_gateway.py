"""MAVROS and Gazebo bindings for the flying-car controller."""

import math
import threading
from types import SimpleNamespace
from typing import Any, Mapping, Optional

from .geometry import quaternion_to_rpy
from .models import Snapshot


def _load_ros_bindings():
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import (
        ExtendedState,
        Mavlink,
        ManualControl,
        ParamValue,
        State,
    )
    from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode

    return SimpleNamespace(
        CommandBool=CommandBool,
        ExtendedState=ExtendedState,
        Mavlink=Mavlink,
        LocalPose=PoseStamped,
        LocalVelocity=TwistStamped,
        ManualControl=ManualControl,
        ModelStates=ModelStates,
        ParamGet=ParamGet,
        ParamSet=ParamSet,
        ParamValue=ParamValue,
        PoseStamped=PoseStamped,
        SetMode=SetMode,
        State=State,
    )


class Px4Gateway:
    """Translate controller commands and feedback to MAVROS ROS APIs."""

    def __init__(self, model_name: str, ros=None, bindings=None):
        if ros is None:
            import rospy

            ros = rospy
        self._ros = ros
        self._bindings = bindings or _load_ros_bindings()
        self._model_name = model_name
        self._mavlink_framing_ok = 1
        self._mavlink_heartbeat_msgid = 0
        self._mavlink_heartbeat_min_length = 9
        self._system_id = 1
        self._autopilot_component_id = 1
        self._parameter_lock = threading.RLock()
        self.last_response_fields: Mapping[str, object] = {}
        self._operation_responses = {}
        self._mav_type: Optional[int] = None
        self._state = None
        self._extended_state = None
        self._local_pose = None
        self._local_velocity = None
        self._truth_position = None
        self._feedback_stamps = {}

        self._manual_publisher = ros.Publisher(
            "/mavros/manual_control/send",
            self._bindings.ManualControl,
            queue_size=10,
        )
        self._pose_publisher = ros.Publisher(
            "/mavros/setpoint_position/local",
            self._bindings.PoseStamped,
            queue_size=10,
        )
        self._param_get = ros.ServiceProxy(
            "/mavros/param/get", self._bindings.ParamGet
        )
        self._param_set = ros.ServiceProxy(
            "/mavros/param/set", self._bindings.ParamSet
        )
        self._set_mode = ros.ServiceProxy(
            "/mavros/set_mode", self._bindings.SetMode
        )
        self._set_armed = ros.ServiceProxy(
            "/mavros/cmd/arming", self._bindings.CommandBool
        )
        self._subscriptions = (
            ros.Subscriber(
                "/mavros/state",
                self._bindings.State,
                self._on_state,
                queue_size=10,
            ),
            ros.Subscriber(
                "/mavros/extended_state",
                self._bindings.ExtendedState,
                self._on_extended_state,
                queue_size=10,
            ),
            ros.Subscriber(
                "/mavros/local_position/pose",
                self._bindings.LocalPose,
                self._on_local_pose,
                queue_size=10,
            ),
            ros.Subscriber(
                "/mavros/local_position/velocity_local",
                self._bindings.LocalVelocity,
                self._on_local_velocity,
                queue_size=10,
            ),
            ros.Subscriber(
                "/gazebo/model_states",
                self._bindings.ModelStates,
                self._on_model_states,
                queue_size=10,
            ),
            ros.Subscriber(
                "/mavlink/from",
                self._bindings.Mavlink,
                self._on_mavlink,
                queue_size=100,
            ),
        )

    def get_mav_type(self) -> int:
        """Read ``MAV_TYPE`` through ``ParamGet``."""
        with self._parameter_lock:
            try:
                response = self._param_get(param_id="MAV_TYPE")
            except Exception as error:
                self._record_response(
                    "get_mav_type", {"exception": str(error)}
                )
                raise RuntimeError(
                    "failed to read MAV_TYPE: {}".format(error)
                )
            self._record_response("get_mav_type", {
                "success": bool(response.success),
                "integer": int(response.value.integer),
                "real": float(response.value.real),
            })
            if not response.success:
                raise RuntimeError("MAV_TYPE ParamGet was rejected")
            return int(response.value.integer)

    def warm_up_params(self, timeout_s: float = 90.0) -> int:
        """Force MAVROS to populate its parameter cache before get/set use.

        MAVROS 1.20 delays its first automatic parameter pull until ~10 s
        after the FCU link is established, and ``/mavros/param/get`` rejects
        requests until that pull completes.  This method blocks until
        ``MAV_TYPE`` is readable so the controller never races the cache.
        """
        import time as _time

        from mavros_msgs.srv import ParamPull

        deadline = _time.monotonic() + float(timeout_s)
        self._ros.wait_for_service(
            "/mavros/param/pull", timeout=min(30.0, float(timeout_s))
        )
        pull = self._ros.ServiceProxy("/mavros/param/pull", ParamPull)

        while self._state is None or not bool(self._state.connected):
            if _time.monotonic() >= deadline:
                raise RuntimeError(
                    "warm_up_params: MAVROS link not established within timeout"
                )
            self._ros.sleep(0.5)

        last_error = None
        while _time.monotonic() < deadline:
            try:
                response = pull(force_pull=True)
                if bool(response.success):
                    try:
                        return self.get_mav_type()
                    except RuntimeError as error:
                        last_error = error
            except Exception as error:
                last_error = error
            self._ros.sleep(1.0)
        raise RuntimeError(
            "warm_up_params: MAV_TYPE still unreadable after pull: {}".format(
                last_error
            )
        )

    def set_mav_type(self, value: int) -> bool:
        """Set ``MAV_TYPE`` and confirm it with an independent readback."""
        with self._parameter_lock:
            requested = int(value)
            parameter = self._bindings.ParamValue(integer=requested, real=0.0)
            try:
                response = self._param_set(
                    param_id="MAV_TYPE", value=parameter
                )
            except Exception as error:
                self._record_response(
                    "set_mav_type", {"exception": str(error)}
                )
                return False
            if not response.success:
                self._record_response("set_mav_type", {
                    "success": False,
                    "requested": requested,
                    "returned_integer": int(response.value.integer),
                    "returned_real": float(response.value.real),
                })
                return False
            try:
                readback = self.get_mav_type()
            except RuntimeError as error:
                self._record_response("set_mav_type", {
                    "success": False,
                    "requested": requested,
                    "readback_error": str(error),
                })
                return False
            accepted = readback == requested
            self._record_response("set_mav_type", {
                "success": accepted,
                "requested": requested,
                "readback": readback,
            })
            return accepted

    def set_mode(self, mode: str) -> bool:
        """Request a PX4 custom mode through MAVROS."""
        try:
            response = self._set_mode(base_mode=0, custom_mode=str(mode))
        except Exception as error:
            self._record_response("set_mode", {"exception": str(error)})
            return False
        self._record_response(
            "set_mode", {"mode_sent": bool(response.mode_sent)}
        )
        return bool(response.mode_sent)

    def set_armed(self, armed: bool) -> bool:
        """Request arm or disarm through MAVROS."""
        try:
            response = self._set_armed(value=bool(armed))
        except Exception as error:
            self._record_response("set_armed", {"exception": str(error)})
            return False
        self._record_response("set_armed", {
            "success": bool(response.success),
            "result": int(response.result),
        })
        return bool(response.success)

    def response_fields(self, operation: str) -> Mapping[str, object]:
        """Return the response captured for one operation without races."""
        with self._parameter_lock:
            return dict(self._operation_responses.get(operation, {}))

    def _record_response(
        self, operation: str, fields: Mapping[str, object]
    ) -> None:
        captured = dict(fields)
        with self._parameter_lock:
            self._operation_responses[operation] = captured
            self.last_response_fields = captured

    def publish_manual(
        self, x: float, y: float, z: float, r: float
    ) -> None:
        """Publish one stamped manual-control command."""
        message = self._bindings.ManualControl()
        message.header.stamp = self._ros.Time.now()
        message.x = float(x)
        message.y = float(y)
        message.z = float(z)
        message.r = float(r)
        message.buttons = 0
        self._manual_publisher.publish(message)

    def publish_local_pose(
        self, x: float, y: float, z: float, yaw: float
    ) -> None:
        """Publish one stamped ENU local-position setpoint."""
        message = self._bindings.PoseStamped()
        message.header.stamp = self._ros.Time.now()
        message.header.frame_id = "map"
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = float(z)
        half_yaw = float(yaw) / 2.0
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        self._pose_publisher.publish(message)

    def snapshot(
        self, now: float, clock_ok: bool, processes_ok: bool
    ) -> Snapshot:
        """Build one immutable snapshot from the latest subscribed feedback."""
        missing = [
            name
            for name, value in (
                ("state", self._state),
                ("extended_state", self._extended_state),
                ("local_pose", self._local_pose),
                ("local_velocity", self._local_velocity),
                ("gazebo_model_state", self._truth_position),
                ("mav_type", self._mav_type),
            )
            if value is None
        ]
        if missing:
            raise RuntimeError(
                "waiting for feedback: {}".format(", ".join(missing))
            )
        orientation = self._local_pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        velocity = self._local_velocity.twist.linear
        position = self._local_pose.pose.position
        return Snapshot(
            stamp=min(self._feedback_stamps.values()),
            connected=bool(self._state.connected),
            armed=bool(self._state.armed),
            mode=str(self._state.mode),
            landed=(
                self._extended_state.landed_state
                == self._bindings.ExtendedState.LANDED_STATE_ON_GROUND
            ),
            mav_type=self._mav_type,
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            vx=float(velocity.x),
            vy=float(velocity.y),
            vz=float(velocity.z),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            clock_ok=bool(clock_ok),
            processes_ok=bool(processes_ok),
        )

    def _on_mavlink(self, message: Any) -> None:
        """Capture MAV_TYPE from a valid autopilot HEARTBEAT frame.

        The heartbeat is the authoritative confirmation that PX4 actually
        switched vehicle type (``system_type``); the MAVROS parameter cache
        updates before the switch completes, which previously let the
        controller arm before PX4 entered ROVER mode.
        """
        try:
            if int(message.framing_status) != self._mavlink_framing_ok:
                return
            if int(message.msgid) != self._mavlink_heartbeat_msgid:
                return
            length = int(message.len)
            if length < self._mavlink_heartbeat_min_length:
                return
            if int(message.sysid) != self._system_id:
                return
            if int(message.compid) != self._autopilot_component_id:
                return
            payload = list(message.payload64)
            required_words = (length + 7) // 8
            if len(payload) < required_words:
                return
            first_word = int(payload[0])
            mav_type = (first_word >> 32) & 0xFF
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self._parameter_lock:
            self._mav_type = mav_type
            self._feedback_received("mav_type")

    def _feedback_received(self, name: str) -> None:
        self._feedback_stamps[name] = self._ros.Time.now().to_sec()

    def _on_state(self, message: Any) -> None:
        self._state = message
        self._feedback_received("state")

    def _on_extended_state(self, message: Any) -> None:
        self._extended_state = message
        self._feedback_received("extended_state")

    def _on_local_pose(self, message: Any) -> None:
        self._local_pose = message
        self._feedback_received("local_pose")

    def _on_local_velocity(self, message: Any) -> None:
        self._local_velocity = message
        self._feedback_received("local_velocity")

    def _on_model_states(self, message: Any) -> None:
        try:
            index = message.name.index(self._model_name)
        except ValueError:
            return
        self._truth_position = message.pose[index].position
        self._feedback_received("gazebo_model_state")
