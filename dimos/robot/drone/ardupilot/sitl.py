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

"""Managed ArduPilot SITL process. Spawns a prebuilt ``arducopter`` binary,
waits for its MAVLink TCP endpoint to accept connections, and guarantees
teardown. Real autopilot firmware in the loop — no Gazebo required."""

from collections import deque
from pathlib import Path
import shutil
import subprocess
import threading
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_SITL_BASE_PORT = 5760
_SITL_PORT_STRIDE = 10
_STOP_GRACE_S = 5.0
_LOG_TAIL_KEEP = 40


def default_param_file(binary: str) -> Path | None:
    """Locate copter.parm in the ArduPilot tree the binary was built in."""
    resolved = shutil.which(binary)
    if resolved is None:
        return None
    candidate = (
        Path(resolved).resolve().parents[3]
        / "Tools"
        / "autotest"
        / "default_params"
        / "copter.parm"
    )
    return candidate if candidate.is_file() else None


class SitlStartupError(RuntimeError):
    """The SITL process exited or never opened its MAVLink port."""


class ArduPilotSitlProcess:
    """Own one ArduPilot SITL instance: spawn in start(), kill in stop()."""

    def __init__(
        self,
        binary: str = "arducopter",
        model: str = "quad",
        workdir: Path | str | None = None,
        home: str | None = None,
        speedup: float = 1.0,
        instance: int = 0,
        param_file: Path | str | None = None,
    ) -> None:
        self.binary = binary
        self.model = model
        self.workdir = Path(workdir) if workdir is not None else None
        self.home = home
        self.speedup = speedup
        self.instance = instance
        self.param_file = Path(param_file) if param_file is not None else None
        self.process: subprocess.Popen[bytes] | None = None
        self._log_tail: deque[str] = deque(maxlen=_LOG_TAIL_KEEP)
        self._log_thread: threading.Thread | None = None
        self._listening = threading.Event()

    @property
    def connection_string(self) -> str:
        return f"tcp:127.0.0.1:{_SITL_BASE_PORT + _SITL_PORT_STRIDE * self.instance}"

    def start(self, timeout: float = 30.0) -> None:
        binary = shutil.which(self.binary)
        if binary is None:
            raise SitlStartupError(
                f"ArduPilot SITL binary {self.binary!r} not found; build it with "
                "Tools/environment_install then './waf configure --board sitl && ./waf copter'"
            )
        workdir = self.workdir if self.workdir is not None else Path.cwd()
        workdir.mkdir(parents=True, exist_ok=True)

        cmd = [
            binary,
            "--model",
            self.model,
            "--speedup",
            str(self.speedup),
            "--instance",
            str(self.instance),
        ]
        param_file = (
            self.param_file if self.param_file is not None else default_param_file(self.binary)
        )
        if param_file is not None:
            cmd += ["--defaults", str(param_file)]
        if self.home is not None:
            cmd += ["--home", self.home]

        self.process = subprocess.Popen(
            cmd,
            cwd=workdir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self._log_thread = threading.Thread(
            target=self._drain_output, name="ardupilot-sitl-log", daemon=True
        )
        self._log_thread.start()
        self._wait_for_port(timeout)

    def stop(self) -> None:
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=_STOP_GRACE_S)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process = None
        if self._log_thread is not None:
            self._log_thread.join(timeout=2.0)
            self._log_thread = None

    def log_tail(self) -> list[str]:
        return list(self._log_tail)

    def _drain_output(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for raw in self.process.stdout:
            line = raw.decode(errors="replace").rstrip()
            self._log_tail.append(line)
            # SITL prints this once SERIAL0 is bound and accepting a client.
            if "Waiting for connection" in line:
                self._listening.set()

    def _wait_for_port(self, timeout: float) -> None:
        assert self.process is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise SitlStartupError(
                    f"SITL exited with code {self.process.returncode}; "
                    f"last output: {self.log_tail()[-5:]}"
                )
            if self._listening.wait(timeout=0.2):
                return
        self.stop()
        raise SitlStartupError(f"SITL did not open {self.connection_string} within {timeout:g}s")
