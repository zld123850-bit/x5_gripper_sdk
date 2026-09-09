"""夹爪双端点软件标定，不写电机内部零点。"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path


CALIBRATION_VERSION = 1


@dataclass(frozen=True)
class GripperCalibration:
    device_serial: str
    interface: str
    motor_can_id: int
    feedback_can_id: int | None
    closed_position_rad: float
    open_position_rad: float
    calibrated_at: str
    schema_version: int = CALIBRATION_VERSION
    robot_model: str = "X5-2023"
    motor_profile: str = "dm_j4310"

    @property
    def travel_rad(self) -> float:
        return abs(self.open_position_rad - self.closed_position_rad)

    @property
    def open_direction_sign(self) -> float:
        return math.copysign(1.0, self.open_position_rad - self.closed_position_rad)

    def validate(
        self,
        *,
        device_serial: str | None = None,
        interface: str | None = None,
        motor_can_id: int | None = None,
    ) -> None:
        if self.schema_version != CALIBRATION_VERSION:
            raise ValueError(f"不支持标定版本 {self.schema_version}。")
        if self.robot_model != "X5-2023" or self.motor_profile != "dm_j4310":
            raise ValueError("标定文件的机器人或电机型号不匹配。")
        if not all(math.isfinite(v) and -12.5 <= v <= 12.5 for v in (self.closed_position_rad, self.open_position_rad)):
            raise ValueError("标定端点超出 DM-J4310 编码范围。")
        if self.travel_rad <= 1e-6:
            raise ValueError("闭合点和张开点不能相同。")
        if device_serial is not None and self.device_serial != device_serial:
            raise ValueError("标定文件的设备序列号不匹配。")
        if interface is not None and self.interface != interface:
            raise ValueError("标定文件的 CAN 接口不匹配。")
        if motor_can_id is not None and self.motor_can_id != int(motor_can_id):
            raise ValueError("标定文件的电机 CAN ID 不匹配。")

    def normalized_opening(self, position_rad: float, *, clamp: bool = True) -> float:
        value = (float(position_rad) - self.closed_position_rad) * self.open_direction_sign / self.travel_rad
        return min(1.0, max(0.0, value)) if clamp else value

    def position_for_opening(self, opening: float) -> float:
        value = float(opening)
        if not 0.0 <= value <= 1.0:
            raise ValueError("opening 必须在 [0, 1]。")
        return self.closed_position_rad + self.open_direction_sign * self.travel_rad * value

    def contains(self, position_rad: float, *, margin_rad: float = 0.0) -> bool:
        lower, upper = sorted((self.closed_position_rad, self.open_position_rad))
        return lower - margin_rad <= float(position_rad) <= upper + margin_rad


def save_calibration(path: str | Path, calibration: GripperCalibration) -> Path:
    calibration.validate()
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(
            json.dumps(asdict(calibration), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return destination


def load_calibration(
    path: str | Path,
    *,
    device_serial: str | None = None,
    interface: str | None = None,
    motor_can_id: int | None = None,
) -> GripperCalibration:
    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        calibration = GripperCalibration(**payload)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取标定文件 {source}: {exc}") from exc
    calibration.validate(
        device_serial=device_serial,
        interface=interface,
        motor_can_id=motor_can_id,
    )
    return calibration

