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

"""Tests for the ArduPilot MAVLink client and SITL manager."""

import math
import shutil
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from pymavlink import mavutil
import pytest

from dimos.robot.drone.ardupilot.mavlink_client import (
    ArduPilotClient,
    CommandTimeoutError,
    ModeChangeError,
)
from dimos.robot.drone.ardupilot.sitl import ArduPilotSitlProcess

SITL_HOME = "-35.363261,149.165230,584,353"
SITL_SPEEDUP = 5.0


class FakeAck:
    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result

    def get_type(self) -> str:
        return "COMMAND_ACK"


def _mocked_client() -> ArduPilotClient:
    client = ArduPilotClient("udp:0.0.0.0:14550")
    client._mavlink = MagicMock()
    client._mavlink.target_system = 1
    client._mavlink.target_component = 1
    return client


def _wait_for(predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class TestAckCorrelation(unittest.TestCase):
    """A command's result must come from its own COMMAND_ACK, never another's."""

    def test_stray_ack_is_not_misattributed(self) -> None:
        client = _mocked_client()
        results: dict[str, int] = {}

        def run() -> None:
            results["result"] = client.run_command(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1.0, 4.0, timeout=2.0
            )

        thread = threading.Thread(target=run)
        thread.start()
        self.assertTrue(
            _wait_for(lambda: mavutil.mavlink.MAV_CMD_DO_SET_MODE in client._ack_waiters, 1.0)
        )
        # A stale ACK from an unrelated command arrives first: it must be ignored.
        client._handle_message(
            FakeAck(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )
        )
        client._handle_message(
            FakeAck(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_RESULT_FAILED)
        )
        thread.join(timeout=3.0)
        self.assertEqual(results["result"], mavutil.mavlink.MAV_RESULT_FAILED)

    def test_missing_ack_times_out(self) -> None:
        client = _mocked_client()
        with self.assertRaises(CommandTimeoutError):
            client.run_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=0.1)

    def test_ack_after_timeout_is_unclaimed(self) -> None:
        client = _mocked_client()
        with self.assertRaises(CommandTimeoutError):
            client.run_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=0.05)
        # The late ACK must not blow up or poison a later command.
        client._handle_message(
            FakeAck(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_RESULT_ACCEPTED)
        )
        self.assertEqual(client._ack_waiters, {})


class TestSetMode(unittest.TestCase):
    """Mode changes are confirmed against HEARTBEAT, not trusted from the ACK."""

    def _client_with_modes(self) -> ArduPilotClient:
        client = _mocked_client()
        client._mode_map = {"STABILIZE": 0, "GUIDED": 4}
        return client

    def test_accepted_but_unconfirmed_mode_raises(self) -> None:
        client = self._client_with_modes()
        with patch.object(client, "run_command", return_value=mavutil.mavlink.MAV_RESULT_ACCEPTED):
            with self.assertRaises(ModeChangeError):
                client.set_mode("GUIDED", timeout=0.2)

    def test_confirmed_mode_succeeds(self) -> None:
        client = self._client_with_modes()

        def accept_and_change(*args: float, **kwargs: float) -> int:
            client._custom_mode = 4
            return mavutil.mavlink.MAV_RESULT_ACCEPTED

        with patch.object(client, "run_command", side_effect=accept_and_change):
            client.set_mode("GUIDED", timeout=0.5)
        self.assertEqual(client.mode, "GUIDED")

    def test_rejected_mode_raises(self) -> None:
        client = self._client_with_modes()
        with patch.object(client, "run_command", return_value=mavutil.mavlink.MAV_RESULT_FAILED):
            with self.assertRaises(ModeChangeError):
                client.set_mode("GUIDED", timeout=0.2)

    def test_unknown_mode_raises_with_alternatives(self) -> None:
        client = self._client_with_modes()
        with self.assertRaisesRegex(ModeChangeError, "GUIDED"):
            client.set_mode("WARPDRIVE")


requires_sitl = pytest.mark.skipif(
    shutil.which("arducopter") is None,
    reason="ArduPilot SITL binary 'arducopter' not on PATH",
)


@requires_sitl
class TestSitlFlight:
    """Full mission against real ArduPilot firmware in SITL."""

    def test_takeoff_goto_land(self, tmp_path) -> None:
        sitl = ArduPilotSitlProcess(workdir=tmp_path, home=SITL_HOME, speedup=SITL_SPEEDUP)
        sitl.start()
        client = ArduPilotClient(sitl.connection_string)
        try:
            client.connect(timeout=30.0)
            assert client.wait_until_armable(timeout=120.0), client.recent_statustexts()

            client.takeoff(5.0)
            assert client.relative_alt is not None
            assert 4.0 <= client.relative_alt <= 6.5
            assert client.mode == "GUIDED"
            assert client.armed

            client.goto_ned(5.0, 0.0, -5.0)
            assert _wait_for(
                lambda: client.position_ned is not None
                and math.hypot(client.position_ned[0] - 5.0, client.position_ned[1]) < 1.0,
                30.0,
            ), f"never reached waypoint, at {client.position_ned}"

            client.land(timeout=120.0)
            assert not client.armed
        finally:
            client.close()
            sitl.stop()
