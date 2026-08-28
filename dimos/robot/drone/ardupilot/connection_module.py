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

"""DimOS module for ArduPilot vehicles over MAVLink, with a managed SITL mode."""

from dataclasses import asdict
import json
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT, STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.drone.ardupilot.mavlink_client import (
    ArduPilotClient,
    CommandTimeoutError,
    LocalOdom,
    ModeChangeError,
    VehicleStatus,
)
from dimos.robot.drone.ardupilot.sitl import ArduPilotSitlProcess
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

VELOCITY_WATCHDOG_TIMEOUT_S = 0.5
_WATCHDOG_POLL_S = 0.1


class Config(ModuleConfig):
    connection_string: str = "udp:0.0.0.0:14550"
    sitl: bool = False
    sitl_binary: str = "arducopter"
    sitl_speedup: float = 1.0
    sitl_home: str = "-35.363261,149.165230,584,353"


class ArdupilotConnectionModule(Module):
    """Fly an ArduPilot vehicle (real or SITL) and stream its odometry."""

    dedicated_worker = True

    config: Config

    cmd_vel: In[Twist]

    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    status: Out[Any]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.client: ArduPilotClient | None = None
        self.sitl: ArduPilotSitlProcess | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_stop = threading.Event()
        self._cmd_lock = threading.Lock()
        self._last_cmd_ts = 0.0
        self._cmd_in_flight = False

    @rpc
    def start(self) -> None:
        connection_string = self.config.connection_string
        if self.config.sitl:
            self.sitl = ArduPilotSitlProcess(
                binary=self.config.sitl_binary,
                workdir=STATE_DIR / "ardupilot-sitl",
                home=self.config.sitl_home,
                speedup=self.config.sitl_speedup,
            )
            self.sitl.start()
            connection_string = self.sitl.connection_string

        self.client = ArduPilotClient(
            connection_string,
            on_odom=self._publish_odom,
            on_status=self._publish_status,
        )
        self.client.connect()

        if self.cmd_vel.transport:
            self.register_disposable(self.cmd_vel.pure_observable().subscribe(self._on_cmd_vel))

        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._velocity_watchdog, name="ardupilot-cmd-watchdog", daemon=True
        )
        self._watchdog_thread.start()
        super().start()

    @rpc
    def stop(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._watchdog_thread = None
        if self.client is not None:
            self.client.close()
            self.client = None
        if self.sitl is not None:
            self.sitl.stop()
            self.sitl = None
        super().stop()

    # -- streams -------------------------------------------------------------

    def _publish_odom(self, sample: LocalOdom) -> None:
        # ArduPilot reports NED; dimos world frame is ENU (z-up).
        position = Vector3(sample.east, sample.north, -sample.down)
        orientation = Quaternion.from_euler(
            Vector3(sample.roll, -sample.pitch, _wrap_pi(math.pi / 2.0 - sample.yaw))
        )
        pose = PoseStamped(
            position=position, orientation=orientation, frame_id="world", ts=sample.ts
        )
        self.odom.publish(pose)
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=position,
                    rotation=orientation,
                    frame_id="world",
                    child_frame_id="base_link",
                    ts=sample.ts,
                )
            )
        )

    def _publish_status(self, vehicle_status: VehicleStatus) -> None:
        self.status.publish(String(json.dumps(asdict(vehicle_status))))

    # -- velocity control ----------------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self.client is None:
            return
        # ROS body convention (x fwd, y left, z up, CCW yaw) -> NED body frame.
        self.client.send_body_velocity(
            forward=twist.linear.x,
            right=-twist.linear.y,
            down=-twist.linear.z,
            yaw_rate=-twist.angular.z,
        )
        moving = any((twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z))
        with self._cmd_lock:
            self._last_cmd_ts = time.monotonic()
            self._cmd_in_flight = moving

    def _velocity_watchdog(self) -> None:
        """Zero the velocity target when the cmd_vel stream stops mid-motion.

        Without this, a dropped teleop link leaves the vehicle flying its last
        commanded velocity until the GUIDED timeout.
        """
        while not self._watchdog_stop.wait(_WATCHDOG_POLL_S):
            with self._cmd_lock:
                stale = (
                    self._cmd_in_flight
                    and time.monotonic() - self._last_cmd_ts > VELOCITY_WATCHDOG_TIMEOUT_S
                )
                if stale:
                    self._cmd_in_flight = False
            if stale and self.client is not None:
                logger.warning("cmd_vel stream stale, zeroing velocity target")
                self.client.send_body_velocity(0.0, 0.0, 0.0, 0.0)

    # -- skills --------------------------------------------------------------

    @skill
    def arm(self) -> str:
        """Arm the vehicle motors after pre-arm checks pass."""
        assert self.client is not None
        if not self.client.wait_until_armable(timeout=30.0):
            return (
                "arm failed: pre-arm checks not passing; recent autopilot messages: "
                f"{self.client.recent_statustexts()[-3:]}"
            )
        try:
            self.client.arm()
        except (RuntimeError, CommandTimeoutError) as error:
            return f"arm failed: {error}"
        return "armed"

    @skill
    def disarm(self) -> str:
        """Disarm the vehicle motors."""
        assert self.client is not None
        try:
            self.client.disarm()
        except (RuntimeError, CommandTimeoutError) as error:
            return f"disarm failed: {error}"
        return "disarmed"

    @skill
    def takeoff(self, altitude: float = 3.0) -> str:
        """Take off in GUIDED mode and climb to the target altitude.

        Args:
            altitude: Target altitude above home in meters.
        """
        assert self.client is not None
        if not self.client.wait_until_armable(timeout=60.0):
            return (
                "takeoff failed: pre-arm checks not passing; recent autopilot messages: "
                f"{self.client.recent_statustexts()[-3:]}"
            )
        try:
            self.client.takeoff(altitude)
        except (ValueError, RuntimeError, CommandTimeoutError) as error:
            return f"takeoff failed: {error}"
        return f"reached {altitude:g}m"

    @skill
    def land(self) -> str:
        """Land at the current position and wait for disarm."""
        assert self.client is not None
        try:
            self.client.land()
        except (RuntimeError, CommandTimeoutError) as error:
            return f"land failed: {error}"
        return "landed and disarmed"

    @skill
    def rtl(self) -> str:
        """Return to the launch point and land."""
        assert self.client is not None
        try:
            self.client.rtl()
        except (ModeChangeError, CommandTimeoutError) as error:
            return f"rtl failed: {error}"
        return "returning to launch"

    @skill
    def set_flight_mode(self, mode: str) -> str:
        """Switch flight mode (GUIDED, LOITER, RTL, LAND, ...).

        Args:
            mode: ArduPilot flight mode name.
        """
        assert self.client is not None
        try:
            self.client.set_mode(mode)
        except (ModeChangeError, CommandTimeoutError) as error:
            return f"mode change failed: {error}"
        return f"mode is now {mode.upper()}"

    @skill
    def move(
        self, forward: float = 0.0, left: float = 0.0, up: float = 0.0, yaw_rate: float = 0.0
    ) -> str:
        """Set a body-frame velocity; the watchdog zeroes it if not refreshed.

        Args:
            forward: Forward velocity in m/s.
            left: Left velocity in m/s.
            up: Up velocity in m/s.
            yaw_rate: Counter-clockwise yaw rate in rad/s.
        """
        assert self.client is not None
        if self.client.mode != "GUIDED":
            return "move failed: not in GUIDED mode; call set_flight_mode('GUIDED') first"
        self._on_cmd_vel(
            Twist(linear=Vector3(forward, left, up), angular=Vector3(0.0, 0.0, yaw_rate))
        )
        return "velocity set"

    @skill
    def goto_ned(self, north: float, east: float, down: float) -> str:
        """Fly to a local-NED position target relative to the EKF origin.

        Args:
            north: North offset in meters.
            east: East offset in meters.
            down: Down offset in meters (negative for altitude above origin).
        """
        assert self.client is not None
        if self.client.mode != "GUIDED":
            return "goto failed: not in GUIDED mode; call set_flight_mode('GUIDED') first"
        self.client.goto_ned(north, east, down)
        return f"flying to NED ({north:g}, {east:g}, {down:g})"

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Latest vehicle status snapshot."""
        assert self.client is not None
        return asdict(self.client.status())


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
