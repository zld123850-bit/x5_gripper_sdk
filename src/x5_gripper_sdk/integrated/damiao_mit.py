"""Public DaMiao MIT CAN packing used by X5-class joint motors.

The byte layout, enable/disable opcodes, and DM-J4310 P/V/T ranges come from
the published DaMiao MIT protocol, not from the closed ARX ``.so``.  This
module does not open a bus and does not send frames.
"""

from __future__ import annotations

from dataclasses import dataclass

ENABLE_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC))
DISABLE_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD))
SET_ZERO_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE))
CLEAR_ERROR_COMMAND = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB))

SYSTEM_COMMAND_PREFIX = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF))

KP_LIMIT = 500.0
KD_LIMIT = 5.0

STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1
VALID_FEEDBACK_STATUS = frozenset({0x0, 0x1, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE})
MAX_PLAUSIBLE_TEMP_C = 125


@dataclass(frozen=True)
class DamiaoMitLimits:
    """Encoder packing ranges for one Damiao motor type."""

    name: str
    p_max: float
    v_max: float
    t_max: float


# Published DM-J4310 MIT defaults.  ARX5 open-source configs use this type
# for the gripper; this 2023 external gripper is not yet confirmed on-bus.
DM_J4310_LIMITS = DamiaoMitLimits(
    name="dm_j4310",
    p_max=12.5,
    v_max=30.0,
    t_max=10.0,
)

MOTOR_PROFILES = {
    DM_J4310_LIMITS.name: DM_J4310_LIMITS,
}


@dataclass(frozen=True)
class DamiaoMitFeedback:
    can_id: int
    motor_id: int
    status: int
    position: float
    velocity: float
    torque: float
    mos_temp_c: int
    rotor_temp_c: int


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    if bits <= 0 or bits > 16:
        raise ValueError("bits must be in 1..16.")
    span = value_max - value_min
    if span <= 0.0:
        raise ValueError("value_max must be greater than value_min.")
    clipped = min(max(float(value), value_min), value_max)
    scale = (1 << bits) - 1
    return int(round((clipped - value_min) * scale / span))


def uint_to_float(value: int, value_min: float, value_max: float, bits: int) -> float:
    if bits <= 0 or bits > 16:
        raise ValueError("bits must be in 1..16.")
    span = value_max - value_min
    if span <= 0.0:
        raise ValueError("value_max must be greater than value_min.")
    scale = (1 << bits) - 1
    clipped = min(max(int(value), 0), scale)
    return value_min + clipped * span / scale


def pack_mit_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    limits: DamiaoMitLimits,
) -> bytes:
    """Pack one 8-byte MIT command using the published DaMiao bit layout."""
    p_u = float_to_uint(position, -limits.p_max, limits.p_max, 16)
    v_u = float_to_uint(velocity, -limits.v_max, limits.v_max, 12)
    kp_u = float_to_uint(kp, 0.0, KP_LIMIT, 12)
    kd_u = float_to_uint(kd, 0.0, KD_LIMIT, 12)
    t_u = float_to_uint(torque, -limits.t_max, limits.t_max, 12)
    return bytes(
        (
            (p_u >> 8) & 0xFF,
            p_u & 0xFF,
            (v_u >> 4) & 0xFF,
            ((v_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0xF) << 4) | ((t_u >> 8) & 0xF),
            t_u & 0xFF,
        )
    )


def unpack_mit_command(
    data: bytes,
    limits: DamiaoMitLimits,
) -> dict[str, float]:
    """Decode one 8-byte MIT command. Used to label sniff traffic, not to send."""
    payload = bytes(data)
    if len(payload) != 8:
        raise ValueError("DaMiao MIT command is exactly 8 bytes.")
    p_u = (payload[0] << 8) | payload[1]
    v_u = (payload[2] << 4) | (payload[3] >> 4)
    kp_u = ((payload[3] & 0xF) << 8) | payload[4]
    kd_u = (payload[5] << 4) | (payload[6] >> 4)
    t_u = ((payload[6] & 0xF) << 8) | payload[7]
    return {
        "position": uint_to_float(p_u, -limits.p_max, limits.p_max, 16),
        "velocity": uint_to_float(v_u, -limits.v_max, limits.v_max, 12),
        "kp": uint_to_float(kp_u, 0.0, KP_LIMIT, 12),
        "kd": uint_to_float(kd_u, 0.0, KD_LIMIT, 12),
        "torque": uint_to_float(t_u, -limits.t_max, limits.t_max, 12),
    }


def unpack_mit_feedback(
    can_id: int,
    data: bytes,
    limits: DamiaoMitLimits,
) -> DamiaoMitFeedback:
    """Decode one 8-byte DaMiao MIT feedback frame."""
    if len(data) != 8:
        raise ValueError("DaMiao MIT feedback is exactly 8 bytes.")
    payload = bytes(data)
    status = (payload[0] >> 4) & 0xF
    motor_id = payload[0] & 0xF
    p_u = (payload[1] << 8) | payload[2]
    v_u = (payload[3] << 4) | (payload[4] >> 4)
    t_u = ((payload[4] & 0xF) << 8) | payload[5]
    return DamiaoMitFeedback(
        can_id=int(can_id) & 0x7FF,
        motor_id=motor_id,
        status=status,
        position=uint_to_float(p_u, -limits.p_max, limits.p_max, 16),
        velocity=uint_to_float(v_u, -limits.v_max, limits.v_max, 12),
        torque=uint_to_float(t_u, -limits.t_max, limits.t_max, 12),
        mos_temp_c=int(payload[6]),
        rotor_temp_c=int(payload[7]),
    )


def is_system_command(data: bytes) -> bool:
    payload = bytes(data)
    return len(payload) == 8 and payload[:7] == SYSTEM_COMMAND_PREFIX


def system_command_name(data: bytes) -> str | None:
    if not is_system_command(data):
        return None
    last = bytes(data)[7]
    names = {
        ENABLE_COMMAND[7]: "enable",
        DISABLE_COMMAND[7]: "disable",
        SET_ZERO_COMMAND[7]: "set_zero",
        CLEAR_ERROR_COMMAND[7]: "clear_error",
    }
    return names.get(last, f"system_0x{last:02x}")


def looks_like_damiao_feedback(data: bytes) -> bool:
    """Reject MIT command frames that happen to be 8 bytes.

    DaMiao feedback carries a documented status nibble and two temperature
    bytes.  Host MIT commands use the same length but typically fail both
    checks, so sniff must not decode them as position/velocity.
    """
    payload = bytes(data)
    if len(payload) != 8 or is_system_command(payload):
        return False
    status = (payload[0] >> 4) & 0xF
    if status not in VALID_FEEDBACK_STATUS:
        return False
    return payload[6] <= MAX_PLAUSIBLE_TEMP_C and payload[7] <= MAX_PLAUSIBLE_TEMP_C


def feedback_motor_id(data: bytes) -> int | None:
    if not looks_like_damiao_feedback(data):
        return None
    return bytes(data)[0] & 0xF
