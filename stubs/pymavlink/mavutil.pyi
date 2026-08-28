from typing import Any

class _MavlinkMessage:
    base_mode: int
    custom_mode: int
    command: int
    result: int
    type: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    lat: int
    lon: int
    relative_alt: int
    onboard_control_sensors_enabled: int
    onboard_control_sensors_health: int
    voltage_battery: int
    flags: int
    severity: int
    text: str
    def get_type(self) -> str: ...

class _MavSender:
    def command_long_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_mode_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_position_target_local_ned_send(self, *args: Any, **kwargs: Any) -> None: ...

class MavlinkConnection:
    target_system: int
    target_component: int
    mav: _MavSender
    def wait_heartbeat(self, timeout: float | None = ...) -> _MavlinkMessage: ...
    def recv_match(
        self,
        type: str | list[str] | None = ...,
        blocking: bool = ...,
        timeout: float | None = ...,
    ) -> _MavlinkMessage | None: ...
    def mode_mapping(self) -> dict[str, int] | None: ...
    def close(self) -> None: ...

def mavlink_connection(
    device: str,
    baud: int = ...,
    source_system: int = ...,
    source_component: int = ...,
    **kwargs: Any,
) -> MavlinkConnection: ...

class _MavlinkConstants:
    MAV_CMD_COMPONENT_ARM_DISARM: int
    MAV_CMD_DO_SET_MODE: int
    MAV_CMD_NAV_LAND: int
    MAV_CMD_NAV_TAKEOFF: int
    MAV_CMD_SET_MESSAGE_INTERVAL: int
    MAV_FRAME_BODY_NED: int
    MAV_FRAME_BODY_OFFSET_NED: int
    MAV_FRAME_LOCAL_NED: int
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: int
    MAV_MODE_FLAG_SAFETY_ARMED: int
    MAV_RESULT_ACCEPTED: int
    MAV_RESULT_FAILED: int
    MAV_SEVERITY_WARNING: int
    MAV_SYS_STATUS_PREARM_CHECK: int
    MAV_TYPE_GCS: int
    MAVLINK_MSG_ID_ATTITUDE: int
    MAVLINK_MSG_ID_EKF_STATUS_REPORT: int
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT: int
    MAVLINK_MSG_ID_LOCAL_POSITION_NED: int
    MAVLINK_MSG_ID_SYS_STATUS: int
    EKF_CONST_POS_MODE: int
    EKF_POS_HORIZ_ABS: int
    POSITION_TARGET_TYPEMASK_X_IGNORE: int
    POSITION_TARGET_TYPEMASK_Y_IGNORE: int
    POSITION_TARGET_TYPEMASK_Z_IGNORE: int
    POSITION_TARGET_TYPEMASK_VX_IGNORE: int
    POSITION_TARGET_TYPEMASK_VY_IGNORE: int
    POSITION_TARGET_TYPEMASK_VZ_IGNORE: int
    POSITION_TARGET_TYPEMASK_AX_IGNORE: int
    POSITION_TARGET_TYPEMASK_AY_IGNORE: int
    POSITION_TARGET_TYPEMASK_AZ_IGNORE: int
    POSITION_TARGET_TYPEMASK_YAW_IGNORE: int
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE: int

mavlink: _MavlinkConstants
