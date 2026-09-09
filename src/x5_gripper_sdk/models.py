"""SDK 公共数据类型与配置。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class GripperError(RuntimeError):
    """夹爪通信或状态错误。"""


class GripperSafetyError(GripperError):
    """安全条件不满足，动作被拒绝或已紧急失能。"""


@dataclass(frozen=True)
class GripperConfig:
    interface: str
    motor_can_id: int
    kp: float
    kd: float
    feedback_can_id: int | None = None
    refresh_hz: float = 50.0
    feedback_timeout_s: float = 0.50
    feedback_watchdog_s: float = 0.10
    claim_listen_s: float = 0.20
    stable_velocity_rad_s: float = 0.05
    max_hold_velocity_rad_s: float = 2.0
    max_nudge_velocity_rad_s: float = 0.50
    max_torque_velocity_rad_s: float = 2.0
    max_hold_excursion_rad: float = 0.50
    max_tracking_error_rad: float = 0.12
    max_wrong_way_rad: float = 0.05
    max_overshoot_rad: float = 0.08
    max_close_nudge_rad: float = 0.20
    max_open_nudge_rad: float = 0.50
    max_nudge_speed_rad_s: float = 0.25
    max_ff_torque_nm: float = 0.50
    max_position_kp_abs_rad: float = 6.0
    allow_residual_low_kp_command: bool = False
    calibration_path: Path | None = None
    device_serial: str = "X5-2023-001"
    acknowledge_unverified_hardware: bool = False

    def validate(self) -> None:
        if not self.interface or any(c in self.interface for c in "^$*+?{}[]|()\\/"):
            raise ValueError("interface 必须是精确的 SocketCAN 名称，例如 can2。")
        if not 1 <= int(self.motor_can_id) <= 15:
            raise ValueError("motor_can_id 必须在 [1, 15]。")
        if self.feedback_can_id is not None and not 0 <= int(self.feedback_can_id) <= 0x7FF:
            raise ValueError("feedback_can_id 必须是 11 位 CAN ID。")
        if not 0.0 <= float(self.kp) <= 5.0:
            raise ValueError("kp 必须在 [0, 5]。")
        if not 0.0 <= float(self.kd) <= 0.5:
            raise ValueError("kd 必须在 [0, 0.5]。")
        if not 10.0 <= float(self.refresh_hz) <= 100.0:
            raise ValueError("refresh_hz 必须在 [10, 100] Hz。")
        numeric = (
            self.feedback_timeout_s,
            self.feedback_watchdog_s,
            self.claim_listen_s,
            self.stable_velocity_rad_s,
        )
        if not all(math.isfinite(float(value)) and float(value) > 0 for value in numeric):
            raise ValueError("超时和速度阈值必须为有限正数。")


@dataclass(frozen=True)
class GripperState:
    can_id: int
    motor_id: int
    status: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_temp_c: int
    rotor_temp_c: int
    opening: float | None = None


@dataclass(frozen=True)
class MotionResult:
    operation: str
    start_position_rad: float
    target_position_rad: float
    final_state: GripperState
    max_velocity_rad_s: float
    max_excursion_rad: float
    max_tracking_error_rad: float
    sample_count: int
    target_reached: bool
    stopped_by_request: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "operation": self.operation,
            "start_position_rad": self.start_position_rad,
            "target_position_rad": self.target_position_rad,
            "final_state": self.final_state.__dict__.copy(),
            "max_velocity_rad_s": self.max_velocity_rad_s,
            "max_excursion_rad": self.max_excursion_rad,
            "max_tracking_error_rad": self.max_tracking_error_rad,
            "sample_count": self.sample_count,
            "target_reached": self.target_reached,
            "stopped_by_request": self.stopped_by_request,
        }
