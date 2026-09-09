"""DaMiao DM-J4310 MIT 协议编解码；本模块不打开 CAN 设备。"""

from __future__ import annotations

from dataclasses import dataclass


ENABLE_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC))
DISABLE_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD))
SET_ZERO_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE))
CLEAR_ERROR_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB))
SYSTEM_COMMAND_PREFIX = bytes((0xFF,) * 7)
KP_LIMIT = 500.0
KD_LIMIT = 5.0
VALID_FEEDBACK_STATUS = frozenset({0x0, 0x1, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE})
MAX_PLAUSIBLE_TEMP_C = 125


@dataclass(frozen=True)
class MotorLimits:
    name: str
    p_max: float
    v_max: float
    t_max: float


DM_J4310_LIMITS = MotorLimits("dm_j4310", 12.5, 30.0, 10.0)


@dataclass(frozen=True)
class MitFeedback:
    can_id: int
    motor_id: int
    status: int
    position: float
    velocity: float
    torque: float
    mos_temp_c: int
    rotor_temp_c: int


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    if not 1 <= bits <= 16:
        raise ValueError("bits 必须在 [1, 16]。")
    if value_max <= value_min:
        raise ValueError("value_max 必须大于 value_min。")
    clipped = min(max(float(value), value_min), value_max)
    return int(round((clipped - value_min) * ((1 << bits) - 1) / (value_max - value_min)))


def uint_to_float(value: int, value_min: float, value_max: float, bits: int) -> float:
    if not 1 <= bits <= 16:
        raise ValueError("bits 必须在 [1, 16]。")
    if value_max <= value_min:
        raise ValueError("value_max 必须大于 value_min。")
    clipped = min(max(int(value), 0), (1 << bits) - 1)
    return value_min + clipped * (value_max - value_min) / ((1 << bits) - 1)


def pack_mit_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    limits: MotorLimits = DM_J4310_LIMITS,
) -> bytes:
    p = float_to_uint(position, -limits.p_max, limits.p_max, 16)
    v = float_to_uint(velocity, -limits.v_max, limits.v_max, 12)
    kp_u = float_to_uint(kp, 0.0, KP_LIMIT, 12)
    kd_u = float_to_uint(kd, 0.0, KD_LIMIT, 12)
    t = float_to_uint(torque, -limits.t_max, limits.t_max, 12)
    return bytes(
        (
            p >> 8,
            p & 0xFF,
            v >> 4,
            ((v & 0xF) << 4) | (kp_u >> 8),
            kp_u & 0xFF,
            kd_u >> 4,
            ((kd_u & 0xF) << 4) | (t >> 8),
            t & 0xFF,
        )
    )


def unpack_mit_command(data: bytes, limits: MotorLimits = DM_J4310_LIMITS) -> dict[str, float]:
    payload = bytes(data)
    if len(payload) != 8:
        raise ValueError("MIT 命令必须为 8 字节。")
    p = (payload[0] << 8) | payload[1]
    v = (payload[2] << 4) | (payload[3] >> 4)
    kp = ((payload[3] & 0xF) << 8) | payload[4]
    kd = (payload[5] << 4) | (payload[6] >> 4)
    t = ((payload[6] & 0xF) << 8) | payload[7]
    return {
        "position": uint_to_float(p, -limits.p_max, limits.p_max, 16),
        "velocity": uint_to_float(v, -limits.v_max, limits.v_max, 12),
        "kp": uint_to_float(kp, 0.0, KP_LIMIT, 12),
        "kd": uint_to_float(kd, 0.0, KD_LIMIT, 12),
        "torque": uint_to_float(t, -limits.t_max, limits.t_max, 12),
    }


def is_system_command(data: bytes) -> bool:
    payload = bytes(data)
    return len(payload) == 8 and payload[:7] == SYSTEM_COMMAND_PREFIX


def looks_like_feedback(data: bytes) -> bool:
    payload = bytes(data)
    if len(payload) != 8 or is_system_command(payload):
        return False
    status = (payload[0] >> 4) & 0xF
    return (
        status in VALID_FEEDBACK_STATUS
        and payload[6] <= MAX_PLAUSIBLE_TEMP_C
        and payload[7] <= MAX_PLAUSIBLE_TEMP_C
    )


def unpack_mit_feedback(
    can_id: int,
    data: bytes,
    limits: MotorLimits = DM_J4310_LIMITS,
) -> MitFeedback:
    payload = bytes(data)
    if len(payload) != 8:
        raise ValueError("MIT 反馈必须为 8 字节。")
    return MitFeedback(
        can_id=int(can_id) & 0x7FF,
        motor_id=payload[0] & 0xF,
        status=(payload[0] >> 4) & 0xF,
        position=uint_to_float((payload[1] << 8) | payload[2], -limits.p_max, limits.p_max, 16),
        velocity=uint_to_float((payload[3] << 4) | (payload[4] >> 4), -limits.v_max, limits.v_max, 12),
        torque=uint_to_float(((payload[4] & 0xF) << 8) | payload[5], -limits.t_max, limits.t_max, 12),
        mos_temp_c=int(payload[6]),
        rotor_temp_c=int(payload[7]),
    )

