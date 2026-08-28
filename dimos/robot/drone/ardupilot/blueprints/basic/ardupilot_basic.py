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

"""ArduPilot drone blueprint: real vehicle over MAVLink, or managed SITL with
``dimos --simulation run ardupilot-basic``."""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.drone.ardupilot.connection_module import ArdupilotConnectionModule
from dimos.visualization.vis_module import vis_module


def _static_drone_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.25, 0.1],
            colors=[(255, 100, 0)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


_vis = vis_module(
    global_config.viewer,
    rerun_config={"static": {"world/tf/base_link": _static_drone_body}},
)

ardupilot_basic = autoconnect(
    _vis,
    ArdupilotConnectionModule.blueprint(sitl=bool(global_config.simulation)),
)
