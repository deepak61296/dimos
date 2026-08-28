# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ArduPilot MAVLink client on pymavlink: a single reader thread dispatches all
inbound messages, and every COMMAND_LONG is correlated to its own COMMAND_ACK by
command id, so an ACK from one command can never be attributed to another. Mode
ids come from the autopilot's own mode table and mode changes are confirmed
against HEARTBEAT, never trusted from the ACK alone."""

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import queue
import threading
import time
from typing import Any

from pymavlink import mavutil

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

COMMAND_ACK_TIMEOUT = 3.0
MODE_CONFIRM_TIMEOUT = 5.0
STATE_POLL_INTERVAL = 0.05
_READER_RECV_TIMEOUT = 0.5
_STATUSTEXT_KEEP = 20
_TAKEOFF_ALTITUDE_FRACTION = 0.95

_STREAM_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10.0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 4.0,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2.0,
}

_VELOCITY_YAW_RATE_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)

_POSITION_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


@dataclass(frozen=True)
class LocalOdom:
    """Local position/attitude sample in ArduPilot's NED frame."""

    ts: float
    north: float
    east: float
    down: float
    vn: float
    ve: float
    vd: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class VehicleStatus:
    """Vehicle state distilled from HEARTBEAT / SYS_STATUS / GLOBAL_POSITION_INT."""

    ts: float
    armed: bool
    mode: str
    prearm_ok: bool
    relative_alt: float
    lat: float
    lon: float
    battery_voltage: float


class ModeChangeError(RuntimeError):
    """The autopilot did not accept or reach the requested flight mode."""


class CommandTimeoutError(TimeoutError):
    """No COMMAND_ACK for our command id arrived within the deadline."""


class ArduPilotClient:
    """Threaded MAVLink client for one ArduPilot vehicle."""

    def __init__(
        self,
        connection_string: str,
        on_odom: Callable[[LocalOdom], None] | None = None,
        on_status: Callable[[VehicleStatus], None] | None = None,
    ) -> None:
        self.connection_string = connection_string
        self.on_odom = on_odom
        self.on_status = on_status

        self._mavlink: mavutil.MavlinkConnection | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()

        self._state_lock = threading.RLock()
        self._mode_map: dict[str, int] = {}
        self._armed = False
        self._custom_mode: int | None = None
        self._prearm_ok = False
        self._local: dict[str, float] = {}
        self._attitude: dict[str, float] = {}
        self._global: dict[str, float] = {}
        self._ekf_flags = 0
        self._battery_voltage = 0.0
        self._statustexts: deque[str] = deque(maxlen=_STATUSTEXT_KEEP)

        self._ack_lock = threading.Lock()
        self._ack_waiters: dict[int, queue.Queue[Any]] = {}

    # -- lifecycle -----------------------------------------------------------

    def connect(self, timeout: float = 30.0) -> None:
        """Connect, load the autopilot's mode table, and start the reader."""
        self._mavlink = mavutil.mavlink_connection(self.connection_string)
        heartbeat = self._mavlink.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            raise TimeoutError(f"no HEARTBEAT from {self.connection_string} within {timeout:g}s")
        mode_map = self._mavlink.mode_mapping()
        if not mode_map:
            raise RuntimeError("autopilot reported no mode mapping")
        with self._state_lock:
            self._mode_map = dict(mode_map)

        self._stop.clear()
        self._reader = threading.Thread(
            target=self._reader_loop, name="ardupilot-mavlink-reader", daemon=True
        )
        self._reader.start()

        for msg_id, rate_hz in _STREAM_RATES_HZ.items():
            result = self.run_command(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                float(msg_id),
                1e6 / rate_hz,
            )
            if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.warning("stream request rejected: msg_id=%s result=%s", msg_id, result)

    def close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=2.0)
            self._reader = None
        if self._mavlink is not None:
            self._mavlink.close()
            self._mavlink = None

    # -- reader --------------------------------------------------------------

    def _reader_loop(self) -> None:
        assert self._mavlink is not None
        while not self._stop.is_set():
            msg = self._mavlink.recv_match(blocking=True, timeout=_READER_RECV_TIMEOUT)
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: Any) -> None:
        msg_type = msg.get_type()
        if msg_type == "COMMAND_ACK":
            with self._ack_lock:
                waiter = self._ack_waiters.get(msg.command)
            if waiter is not None:
                waiter.put(msg)
            else:
                logger.debug("unclaimed COMMAND_ACK: command=%s result=%s", msg.command, msg.result)
        elif msg_type == "HEARTBEAT":
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            with self._state_lock:
                self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._custom_mode = msg.custom_mode
            self._emit_status()
        elif msg_type == "LOCAL_POSITION_NED":
            with self._state_lock:
                self._local = {
                    "north": msg.x,
                    "east": msg.y,
                    "down": msg.z,
                    "vn": msg.vx,
                    "ve": msg.vy,
                    "vd": msg.vz,
                }
            self._emit_odom()
        elif msg_type == "ATTITUDE":
            with self._state_lock:
                self._attitude = {"roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw}
        elif msg_type == "SYS_STATUS":
            prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            with self._state_lock:
                self._prearm_ok = bool(
                    msg.onboard_control_sensors_enabled
                    & msg.onboard_control_sensors_health
                    & prearm_bit
                )
                self._battery_voltage = msg.voltage_battery / 1000.0
        elif msg_type == "GLOBAL_POSITION_INT":
            with self._state_lock:
                self._global = {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "relative_alt": msg.relative_alt / 1000.0,
                }
        elif msg_type == "EKF_STATUS_REPORT":
            with self._state_lock:
                self._ekf_flags = msg.flags
        elif msg_type == "STATUSTEXT":
            self._statustexts.append(msg.text)
            if msg.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                logger.warning("autopilot: %s", msg.text)

    def _emit_odom(self) -> None:
        if self.on_odom is None:
            return
        with self._state_lock:
            if not self._local or not self._attitude:
                return
            sample = LocalOdom(ts=time.time(), **self._local, **self._attitude)
        self.on_odom(sample)

    def _emit_status(self) -> None:
        if self.on_status is None:
            return
        self.on_status(self.status())

    # -- state accessors -----------------------------------------------------

    @property
    def armed(self) -> bool:
        with self._state_lock:
            return self._armed

    @property
    def prearm_ok(self) -> bool:
        with self._state_lock:
            return self._prearm_ok

    @property
    def has_position_estimate(self) -> bool:
        """EKF reports an absolute horizontal position (required to arm GUIDED)."""
        with self._state_lock:
            return bool(self._ekf_flags & mavutil.mavlink.EKF_POS_HORIZ_ABS) and not bool(
                self._ekf_flags & mavutil.mavlink.EKF_CONST_POS_MODE
            )

    @property
    def mode(self) -> str:
        with self._state_lock:
            for name, mode_id in self._mode_map.items():
                if mode_id == self._custom_mode:
                    return name
            return f"UNKNOWN({self._custom_mode})"

    @property
    def position_ned(self) -> tuple[float, float, float] | None:
        with self._state_lock:
            if not self._local:
                return None
            return (self._local["north"], self._local["east"], self._local["down"])

    @property
    def relative_alt(self) -> float | None:
        with self._state_lock:
            if not self._local:
                return None
            return -self._local["down"]

    def status(self) -> VehicleStatus:
        with self._state_lock:
            return VehicleStatus(
                ts=time.time(),
                armed=self._armed,
                mode=self.mode,
                prearm_ok=self._prearm_ok,
                relative_alt=self._global.get("relative_alt", 0.0),
                lat=self._global.get("lat", 0.0),
                lon=self._global.get("lon", 0.0),
                battery_voltage=self._battery_voltage,
            )

    def recent_statustexts(self) -> list[str]:
        return list(self._statustexts)

    # -- commands ------------------------------------------------------------

    def run_command(
        self,
        command: int,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
        timeout: float = COMMAND_ACK_TIMEOUT,
    ) -> int:
        """Send COMMAND_LONG and return the MAV_RESULT from its own ACK."""
        assert self._mavlink is not None, "not connected"
        waiter: queue.Queue[Any] = queue.Queue(maxsize=1)
        with self._ack_lock:
            if command in self._ack_waiters:
                raise RuntimeError(f"command {command} already in flight")
            self._ack_waiters[command] = waiter
        try:
            self._mavlink.mav.command_long_send(
                self._mavlink.target_system,
                self._mavlink.target_component,
                command,
                0,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
            )
            try:
                ack = waiter.get(timeout=timeout)
            except queue.Empty:
                raise CommandTimeoutError(
                    f"no COMMAND_ACK for command {command} within {timeout:g}s"
                ) from None
            return int(ack.result)
        finally:
            with self._ack_lock:
                self._ack_waiters.pop(command, None)

    def set_mode(self, mode: str, timeout: float = MODE_CONFIRM_TIMEOUT) -> None:
        """Switch flight mode and confirm the change against HEARTBEAT."""
        name = mode.upper()
        with self._state_lock:
            mode_id = self._mode_map.get(name)
        if mode_id is None:
            raise ModeChangeError(
                f"unknown mode {name!r}; autopilot supports {sorted(self._mode_map)}"
            )
        result = self.run_command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(mode_id),
        )
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise ModeChangeError(f"mode {name} rejected: {self._describe_result(result)}")
        if not self._wait_for(lambda: self._current_mode_id() == mode_id, timeout):
            raise ModeChangeError(f"mode {name} accepted but HEARTBEAT never confirmed it")

    def wait_until_armable(self, timeout: float = 90.0) -> bool:
        """Wait for pre-arm checks AND an EKF position estimate.

        The SYS_STATUS pre-arm bit alone goes healthy before the EKF converges,
        and arming in GUIDED is rejected with "Need Position Estimate" until
        EKF_STATUS_REPORT shows absolute horizontal position.
        """
        return self._wait_for(lambda: self.prearm_ok and self.has_position_estimate, timeout)

    def arm(self, timeout: float = 10.0) -> None:
        result = self.run_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(
                f"arming rejected: {self._describe_result(result)}; "
                f"recent autopilot messages: {self.recent_statustexts()[-5:]}"
            )
        if not self._wait_for(lambda: self.armed, timeout):
            raise RuntimeError("arm accepted but vehicle never reported armed")

    def disarm(self, timeout: float = 10.0) -> None:
        result = self.run_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"disarm rejected: {self._describe_result(result)}")
        if not self._wait_for(lambda: not self.armed, timeout):
            raise RuntimeError("disarm accepted but vehicle still reports armed")

    def takeoff(self, altitude: float, timeout: float | None = None) -> None:
        """GUIDED takeoff to ``altitude`` meters above home; blocks until reached."""
        if altitude <= 0.0:
            raise ValueError("takeoff altitude must be positive")
        self.set_mode("GUIDED")
        if not self.armed:
            self.arm()
        result = self.run_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=float(altitude))
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"takeoff rejected: {self._describe_result(result)}")
        deadline = timeout if timeout is not None else altitude * 2.0 + 15.0
        target = altitude * _TAKEOFF_ALTITUDE_FRACTION
        if not self._wait_for(lambda: (self.relative_alt or 0.0) >= target, deadline):
            raise RuntimeError(
                f"takeoff did not reach {altitude:g}m within {deadline:g}s (at {self.relative_alt})"
            )

    def send_body_velocity(
        self, forward: float, right: float, down: float, yaw_rate: float = 0.0
    ) -> None:
        """Stream one body-frame velocity setpoint (GUIDED mode)."""
        assert self._mavlink is not None, "not connected"
        self._mavlink.mav.set_position_target_local_ned_send(
            0,
            self._mavlink.target_system,
            self._mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            _VELOCITY_YAW_RATE_TYPE_MASK,
            0.0,
            0.0,
            0.0,
            forward,
            right,
            down,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate,
        )

    def goto_ned(self, north: float, east: float, down: float) -> None:
        """Send one local-NED position target (GUIDED mode)."""
        assert self._mavlink is not None, "not connected"
        self._mavlink.mav.set_position_target_local_ned_send(
            0,
            self._mavlink.target_system,
            self._mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _POSITION_TYPE_MASK,
            north,
            east,
            down,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def land(self, timeout: float = 60.0) -> None:
        """Switch to LAND and block until the vehicle disarms."""
        self.set_mode("LAND")
        if not self._wait_for(lambda: not self.armed, timeout):
            raise RuntimeError(f"vehicle did not disarm within {timeout:g}s of LAND")

    def rtl(self) -> None:
        self.set_mode("RTL")

    # -- helpers -------------------------------------------------------------

    def _current_mode_id(self) -> int | None:
        with self._state_lock:
            return self._custom_mode

    def _wait_for(self, predicate: Callable[[], bool], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            if self._stop.wait(STATE_POLL_INTERVAL):
                return False
        return predicate()

    @staticmethod
    def _describe_result(result: int) -> str:
        names = {
            value: name
            for name, value in vars(mavutil.mavlink).items()
            if name.startswith("MAV_RESULT_")
        }
        return names.get(result, str(result))
