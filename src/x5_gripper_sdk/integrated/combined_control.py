"""Keyboard Catch-position jog for the X5 external gripper and Cartesian arm.

When a two-point calibration exists, startup first uses a bounded direct-CAN
velocity servo to move the gripper from any point in its calibrated travel to a
known reference endpoint. It then sends DISABLE to ESC 8 while independently
checking DaMiao feedback so mandatory vendor Init cannot move the jaw unnoticed.
It maps small Catch targets to decoded MIT torque commands, fits and verifies the
zero-torque target, then enables ESC 8 under independent command-torque,
velocity, and excursion guards. After a stable handoff, vendor Catch owns
runtime gripper jog.
"""

from __future__ import annotations

import argparse
from collections import deque
import gc
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Any, Callable, Sequence, TextIO

import numpy as np

from . import arm_backend as backend
from .damiao_mit import (
    DISABLE_COMMAND,
    DM_J4310_LIMITS,
    ENABLE_COMMAND,
    SET_ZERO_COMMAND,
    is_system_command,
    pack_mit_command,
    unpack_mit_command,
)
from .can_hold import matching_feedback, open_socketcan
from .session_log import resolve_session_log_path
from .keyboard_jog import (
    KEY_DIRECTIONS,
    MAX_MEASURED_VELOCITY_RAD_S,
    TARGET_TOLERANCE_M,
    JogSafetyError,
    _diagnostic,
    _distance,
    _enter_and_warm_up_position_mode,
    _execute_interpolated_joint_move,
    _execute_joint_limit_normalization,
    _filtered_vendor_output,
    _fk_pose,
    _joint_positions,
    _print_state,
    _warm_up_feedback,
    build_cartesian_target,
    build_return_target,
    validate_ik_solution,
)


ALLOWED_INTERFACE = "can2"
GRIPPER_MOTOR_CAN_ID = 8
GRIPPER_FEEDBACK_CAN_ID = 0
STARTUP_DISABLE_PERIOD_S = 0.01
VENDOR_INIT_SUPPRESS_S = 5.0
DEFAULT_REFRESH_HZ = 50.0
DEFAULT_OPEN_RATE_RAW_S = 1.0
DEFAULT_CLOSE_RATE_RAW_S = 0.5
# Live CAN2 Catch feedback confirms that increasing the vendor target opens
# this installed external gripper. These defaults apply before a two-point
# calibration is available; calibrated endpoints override them per device.
DEFAULT_OPEN_TARGET_RAW = 5.0
DEFAULT_CLOSE_TARGET_RAW = 0.0
DEFAULT_JOG_SECONDS = 120.0
MAX_JOG_SECONDS = 180.0
DEFAULT_MOTION_TIMEOUT_S = 15.0
MAX_CATCH_RATE_RAW_S = 1.0
MAX_GRIPPER_VELOCITY_RAW_S = 2.0
GRIPPER_OVERSPEED_SAMPLES = 3
RUNTIME_SAMPLE_HZ = 20.0
RUNTIME_SAMPLE_PERIOD_S = 1.0 / RUNTIME_SAMPLE_HZ
INIT_SETTLE_VELOCITY_RAW_S = 0.20
INIT_SETTLE_STABLE_SAMPLES = 3
INIT_SETTLE_MIN_S = 0.5
INIT_SETTLE_TIMEOUT_S = 8.0
STARTUP_MIN_MIT_FEEDBACK_SAMPLES = 3
STARTUP_MAX_MIT_SEGMENT_DRIFT_RAD = 0.10
STARTUP_MAX_MIT_VELOCITY_RAD_S = 0.50
STARTUP_MAX_FINAL_MIT_VELOCITY_RAD_S = 0.10
PREFLIGHT_MIT_COMMAND_HISTORY = 256
PREFLIGHT_MIT_SDK_COORDINATE_TOLERANCE = 0.10
DISABLED_CATCH_PROBE_TARGETS_RAW = (0.0, 0.025, 0.05, 0.075, 0.10, 0.15, 0.20)
DISABLED_CATCH_PROBE_SAMPLES = 10
DISABLED_CATCH_PROBE_TIMEOUT_S = 0.25
DISABLED_CATCH_PROBE_POLL_S = 0.002
DISABLED_MAP_MIN_SLOPE_NM_PER_RAW = 6.0
DISABLED_MAP_MAX_SLOPE_NM_PER_RAW = 10.0
DISABLED_MAP_MAX_RESIDUAL_NM = 0.05
DISABLED_MAP_MIN_ZERO_RAW = 0.05
DISABLED_MAP_MAX_ZERO_RAW = 0.15
DISABLED_MAP_MAX_BALANCE_TORQUE_NM = 0.05
DISABLED_MAP_MAX_ABS_POSITION_COMMAND_RAD = 0.01
DISABLED_MAP_MAX_ABS_VELOCITY_COMMAND_RAD_S = 0.05
DISABLED_MAP_MAX_KP = 0.01
DISABLED_MAP_MIN_KD = 0.15
DISABLED_MAP_MAX_KD = 0.25
HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM = 0.40
RUNTIME_TARGET_COMMAND_TORQUE_NM = 0.30
HANDOFF_DIRECT_MAX_VELOCITY_RAD_S = 0.50
HANDOFF_DIRECT_MAX_EXCURSION_RAD = 0.05
VENDOR_CATCH_ENABLE_LOCKED = False
HANDOFF_VERIFY_S = 0.50
HANDOFF_SAMPLE_PERIOD_S = 0.02
HANDOFF_MAX_TRACKING_ERROR_RAW = 0.20
HANDOFF_MAX_VELOCITY_RAW_S = 0.25
HANDOFF_VELOCITY_CONSECUTIVE_SAMPLES = 3
HANDOFF_EMERGENCY_VELOCITY_RAW_S = 2.0
HANDOFF_MAX_EXCURSION_RAW = 0.10
HANDOFF_FINAL_MAX_VELOCITY_RAW_S = 0.05
HANDOFF_FINAL_MAX_TRACKING_ERROR_RAW = 0.15
STALL_MAX_VELOCITY_RAW_S = 0.05
STALL_MIN_TRACKING_ERROR_RAW = 0.30
STALL_MIN_EFFORT0 = 1.80
STALL_CONSECUTIVE_SAMPLES = 15
GRIPPER_RAW_MIN = 0.0
GRIPPER_RAW_MAX = 5.0
GRIPPER_CALIBRATION_VERSION = 1
GRIPPER_CALIBRATION_CAPTURE_MAX_VELOCITY_RAW_S = 0.05
CALIBRATED_STARTUP_FEEDBACK_TIMEOUT_S = 1.0
CALIBRATED_STARTUP_PERIOD_S = 0.01
CALIBRATED_STARTUP_TIMEOUT_S = 45.0
CALIBRATED_STARTUP_KD = 0.50
CALIBRATED_STARTUP_MAX_COMMAND_VELOCITY_RAD_S = 0.25
CALIBRATED_STARTUP_COMMAND_VELOCITY_LIMIT_RAD_S = 0.50
CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S = 0.03
CALIBRATED_STARTUP_VELOCITY_GAIN = 1.0
CALIBRATED_STARTUP_VELOCITY_CONTROL_GAIN = 2.25
CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM = 0.30
CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM = 0.40
CALIBRATED_STARTUP_MOVING_BIAS_MIN_TORQUE_NM = 0.02
CALIBRATED_STARTUP_MOVING_BIAS_MAX_TORQUE_NM = 0.06
CALIBRATED_STARTUP_TORQUE_RAMP_NM_S = 0.30
CALIBRATED_STARTUP_CONTROL_VELOCITY_DEADBAND_RAD_S = 0.05
CALIBRATED_STARTUP_MAX_MEASURED_VELOCITY_RAD_S = 0.60
CALIBRATED_STARTUP_EMERGENCY_VELOCITY_RAD_S = 0.80
CALIBRATED_STARTUP_OVERSPEED_SAMPLES = 3
CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD = 0.07
CALIBRATED_STARTUP_FINAL_VELOCITY_RAD_S = 0.05
CALIBRATED_STARTUP_STABLE_S = 0.15
CALIBRATED_STARTUP_STALL_ERROR_RAD = (
    CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD
)
CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD = 0.005
CALIBRATED_STARTUP_STALL_S = 1.5
CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM = (
    0.90 * CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM
)
CALIBRATED_STARTUP_LONG_STALL_S = 5.0
CALIBRATED_STARTUP_FEEDBACK_WATCHDOG_S = 0.20
CALIBRATED_STARTUP_CALIBRATION_MARGIN_RAD = 0.10
CALIBRATED_STARTUP_WRONG_WAY_MARGIN_RAD = 0.08
VENDOR_PRIME_QUIET_S = 0.20
VENDOR_PRIME_QUIET_TIMEOUT_S = 2.0
KEYPRESS_JOG_MAX_ELAPSED_S = 0.02
# The vendor FK/IK chain terminates at the link6 frame.  The current X5-2023
# description places the flange face 105 mm along link6 +X, and the installed
# gripper TCP was measured 200 mm beyond that flange face.  The gripper is
# mounted without an additional rotation, so its TCP lies on the same local +X
# axis.  Cartesian jog poses in this command are expressed at that TCP.
LINK6_TO_FLANGE_OFFSET_M = 0.105
FLANGE_TO_GRIPPER_TCP_OFFSET_M = 0.200
LINK6_TO_GRIPPER_TCP_OFFSET_M = (
    LINK6_TO_FLANGE_OFFSET_M + FLANGE_TO_GRIPPER_TCP_OFFSET_M
)
OPEN_KEYS = frozenset({"c", "C", "[", ",", "-"})
CLOSE_KEYS = frozenset({"o", "O", "]", ".", "="})
STOP_KEYS = frozenset({" "})
QUIT_KEYS = frozenset({"q", "Q", "\x1b"})
MOTION_CONFIRMATION = "1"
GRIPPER_KEY_LAYOUTS = frozenset({"c-open", "c-close"})


def map_gripper_key(key: str, layout: str) -> str:
    """Map C/O letters while leaving arm, stop, and micro-jog keys unchanged."""
    if layout not in GRIPPER_KEY_LAYOUTS:
        raise ValueError(f"Unsupported gripper key layout: {layout}.")
    if layout == "c-open":
        return key
    swaps = {"c": "o", "C": "O", "o": "c", "O": "C"}
    return swaps.get(key, key)


def _write_log(stream: TextIO, event: str, **fields: Any) -> None:
    record = {
        "time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "event": event,
        **fields,
    }
    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    stream.flush()


def _user_print(*args: Any, **kwargs: Any) -> None:
    print(*args, **kwargs)


@dataclass
class ArmJogContext:
    controllers: list[Any]
    controller: Any
    fk_solver: Callable[[np.ndarray], Sequence[float]]
    ik_solver: Callable[[np.ndarray], Sequence[float]]
    startup_pose: list[float]
    motion_timeout: float
    max_velocity: float


def _offset_pose_along_local_x(
    pose: Sequence[float], offset_m: float
) -> list[float]:
    """Translate an XYZ/RPY pose along its local +X axis."""
    values = [float(value) for value in pose]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise JogSafetyError("Tool pose must contain six finite XYZ/RPY values.")
    pitch = values[4]
    yaw = values[5]
    cos_pitch = math.cos(pitch)
    values[0] += float(offset_m) * math.cos(yaw) * cos_pitch
    values[1] += float(offset_m) * math.sin(yaw) * cos_pitch
    values[2] -= float(offset_m) * math.sin(pitch)
    return values


@dataclass(frozen=True)
class GripperTcpKinematics:
    """Adapt the vendor link6 FK/IK pair to the installed gripper TCP."""

    vendor_fk_solver: Callable[[np.ndarray], Sequence[float]]
    vendor_ik_solver: Callable[[np.ndarray], Sequence[float]]
    link6_to_tcp_offset_m: float = LINK6_TO_GRIPPER_TCP_OFFSET_M

    def forward_kinematics(self, joints: np.ndarray) -> list[float]:
        link6_pose = self.vendor_fk_solver(joints)
        return _offset_pose_along_local_x(
            link6_pose, self.link6_to_tcp_offset_m
        )

    def inverse_kinematics(self, tcp_pose: np.ndarray) -> Sequence[float]:
        link6_pose = _offset_pose_along_local_x(
            tcp_pose, -self.link6_to_tcp_offset_m
        )
        return self.vendor_ik_solver(np.asarray(link6_pose, dtype=float))


@dataclass(frozen=True)
class GripperTravelCalibration:
    """Persistent absolute-motor-coordinate endpoints for one gripper."""

    interface: str
    closed_zero_physical_raw: float
    open_max_physical_raw: float
    calibrated_at: str

    @property
    def max_travel_raw(self) -> float:
        return abs(self.open_max_physical_raw - self.closed_zero_physical_raw)

    @property
    def open_direction_sign(self) -> float:
        return math.copysign(
            1.0,
            self.open_max_physical_raw - self.closed_zero_physical_raw,
        )

    def normalized_position_raw(self, physical_position_raw: float) -> float:
        return (
            float(physical_position_raw) - self.closed_zero_physical_raw
        ) * self.open_direction_sign

    def as_dict(self) -> dict[str, Any]:
        return {
            "version": GRIPPER_CALIBRATION_VERSION,
            "robot_model": "X5-2023",
            "interface": self.interface,
            "motor_can_id": GRIPPER_MOTOR_CAN_ID,
            "feedback_can_id": GRIPPER_FEEDBACK_CAN_ID,
            "semantics": "absolute_motor_coordinate_two_point_calibration",
            "closed_zero_physical_raw": self.closed_zero_physical_raw,
            "open_max_physical_raw": self.open_max_physical_raw,
            "normalized_closed_raw": 0.0,
            "normalized_open_max_raw": self.max_travel_raw,
            "close_increases_physical_raw": (
                self.closed_zero_physical_raw > self.open_max_physical_raw
            ),
            "calibrated_at": self.calibrated_at,
        }


def _default_gripper_calibration_path(interface: str) -> Path:
    return (
        Path(__file__).resolve().parents[3]
        / "calibration"
        / "X5-2023-001"
        / f"gripper_can_jog_{interface}.json"
    )


def _validate_gripper_calibration(
    calibration: GripperTravelCalibration,
    *,
    interface: str,
) -> None:
    if calibration.interface != interface:
        raise ValueError(
            "Gripper calibration interface does not match "
            f"{interface}: {calibration.interface}."
        )
    endpoints = (
        calibration.open_max_physical_raw,
        calibration.closed_zero_physical_raw,
    )
    if not all(math.isfinite(value) for value in endpoints):
        raise ValueError("Gripper calibration endpoints must be finite.")
    if not all(
        -DM_J4310_LIMITS.p_max <= value <= DM_J4310_LIMITS.p_max
        for value in endpoints
    ):
        raise ValueError(
            "Gripper calibration endpoints are outside the motor encoder "
            f"protocol range [-{DM_J4310_LIMITS.p_max:.1f}, "
            f"{DM_J4310_LIMITS.p_max:.1f}]."
        )
    if math.isclose(
        calibration.open_max_physical_raw,
        calibration.closed_zero_physical_raw,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "Gripper closed-zero and maximum-open points must be different."
        )


def load_gripper_calibration(
    path: Path,
    *,
    interface: str,
) -> GripperTravelCalibration | None:
    """Load a two-point calibration, returning None when it does not exist."""
    source = Path(path)
    if not source.exists():
        return None
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
        if int(payload["version"]) != GRIPPER_CALIBRATION_VERSION:
            raise ValueError(
                "Unsupported gripper calibration version "
                f"{payload['version']}."
            )
        if payload.get("robot_model") != "X5-2023":
            raise ValueError("Gripper calibration robot_model must be X5-2023.")
        if int(payload.get("motor_can_id")) != GRIPPER_MOTOR_CAN_ID:
            raise ValueError("Gripper calibration motor CAN ID must be 8.")
        calibration = GripperTravelCalibration(
            interface=str(payload["interface"]),
            closed_zero_physical_raw=float(
                payload["closed_zero_physical_raw"]
            ),
            open_max_physical_raw=float(payload["open_max_physical_raw"]),
            calibrated_at=str(payload["calibrated_at"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"Invalid gripper calibration file {source}: {exc}") from exc
    _validate_gripper_calibration(calibration, interface=interface)
    return calibration


def save_gripper_calibration(
    path: Path,
    calibration: GripperTravelCalibration,
) -> Path:
    """Atomically persist a validated gripper calibration."""
    _validate_gripper_calibration(
        calibration,
        interface=calibration.interface,
    )
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.name}.{os.getpid()}.tmp"
    )
    try:
        temporary.write_text(
            json.dumps(
                calibration.as_dict(),
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return destination


def calibrated_catch_targets(
    calibration: GripperTravelCalibration,
    *,
    startup_physical_raw: float,
    command_feedback_offset_raw: float,
) -> tuple[float, float, dict[str, float]]:
    """Translate persistent absolute endpoints into this Init's Catch frame."""
    origin = float(startup_physical_raw)
    offset = float(command_feedback_offset_raw)
    if not all(math.isfinite(value) for value in (origin, offset)):
        raise ValueError("Calibration coordinate transform must be finite.")
    requested_open = calibration.open_max_physical_raw - origin + offset
    requested_close = calibration.closed_zero_physical_raw - origin + offset
    if not (
        GRIPPER_RAW_MIN <= requested_open <= GRIPPER_RAW_MAX
        and GRIPPER_RAW_MIN <= requested_close <= GRIPPER_RAW_MAX
    ):
        raise JogSafetyError(
            "The calibrated startup reference did not place both endpoints "
            "inside vendor Catch [0, 5]. "
            f"Requested open/close targets were {requested_open:.3f}/"
            f"{requested_close:.3f} raw."
        )
    if math.isclose(requested_open, requested_close, abs_tol=1e-9):
        raise JogSafetyError(
            "Calibrated Catch targets have no usable span."
        )
    lower, upper = sorted((requested_open, requested_close))
    if not lower <= offset <= upper:
        raise JogSafetyError(
            "The startup gripper position is outside the calibrated travel."
        )
    return requested_open, requested_close, {
        "requested_open_target_raw": requested_open,
        "requested_close_target_raw": requested_close,
        "open_target_raw": requested_open,
        "close_target_raw": requested_close,
    }


def calibrated_startup_reference_raw(
    calibration: GripperTravelCalibration,
) -> float:
    """Return the endpoint that makes all calibrated Catch offsets nonnegative."""
    return min(
        calibration.closed_zero_physical_raw,
        calibration.open_max_physical_raw,
    )


def _preposition_feedback(received: Any) -> Any | None:
    if received is None:
        return None
    can_id, payload = int(received[0]), bytes(received[1])
    if can_id == GRIPPER_MOTOR_CAN_ID:
        raise JogSafetyError(
            "CAN ID 8 received a command before the vendor controller was "
            "created; another commander is using the gripper."
        )
    return matching_feedback(
        can_id,
        payload,
        motor_can_id=GRIPPER_MOTOR_CAN_ID,
        feedback_can_id=GRIPPER_FEEDBACK_CAN_ID,
        limits=DM_J4310_LIMITS,
    )


def _calibrated_startup_feedforward_torque(
    desired_velocity: float,
    measured_velocity: float,
) -> tuple[float, float]:
    """Return travel-only feedforward torque and deadbanded velocity.

    The ESC already applies ``kd`` damping around zero velocity. Applying an
    additional full-scale feedforward torque opposite the requested travel can
    over-brake a static-friction release and drive the mechanism into a rebound.
    Remove travel feedforward immediately when it would reverse direction and
    let the ESC damping slow the motor instead.
    """
    control_velocity = (
        0.0
        if abs(measured_velocity)
        <= CALIBRATED_STARTUP_CONTROL_VELOCITY_DEADBAND_RAD_S
        else measured_velocity
    )
    if desired_velocity == 0.0:
        requested_torque = 0.0
    elif control_velocity == 0.0:
        # Keep enough travel torque to overcome the measured static friction
        # even when the position-dependent target velocity tapers near the
        # endpoint. The existing ramp still applies before this is sent.
        requested_torque = math.copysign(
            CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM,
            desired_velocity,
        )
    else:
        requested_torque = min(
            CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM,
            max(
                -CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM,
                CALIBRATED_STARTUP_VELOCITY_CONTROL_GAIN
                * (desired_velocity - control_velocity),
            ),
        )
        # The ESC's kd term supplies most of the braking. Retain a small
        # direction-following bias instead of dropping travel torque to zero;
        # this prevents the measured elastic rebound from restarting the
        # standstill ramp on every stick-slip release.
        moving_bias = min(
            CALIBRATED_STARTUP_MOVING_BIAS_MAX_TORQUE_NM,
            max(
                CALIBRATED_STARTUP_MOVING_BIAS_MIN_TORQUE_NM,
                CALIBRATED_STARTUP_KD * abs(desired_velocity),
            ),
        )
        if (
            requested_torque == 0.0
            or math.copysign(1.0, requested_torque)
            != math.copysign(1.0, desired_velocity)
            or abs(requested_torque) < moving_bias
        ):
            requested_torque = math.copysign(moving_bias, desired_velocity)
    return requested_torque, control_velocity


def preposition_calibrated_gripper(
    bus: Any,
    calibration: GripperTravelCalibration,
    *,
    log_stream: TextIO,
    max_command_velocity_rad_s: float = (
        CALIBRATED_STARTUP_MAX_COMMAND_VELOCITY_RAD_S
    ),
    timeout_s: float = CALIBRATED_STARTUP_TIMEOUT_S,
    monotonic: Callable[[], float] = time.monotonic,
) -> dict[str, Any]:
    """Move to the calibrated low-coordinate reference before vendor Init.

    Position gain is deliberately zero. A software velocity loop uses the
    independently decoded feedback and stays inside the already exercised
    direct-CAN envelope (kd <= 0.5 and feedforward torque <= 0.5 Nm). Applied
    torque ramps up gently, but braking torque may be applied immediately.
    """
    _validate_gripper_calibration(calibration, interface=calibration.interface)
    max_command_velocity_rad_s = float(max_command_velocity_rad_s)
    timeout_s = float(timeout_s)
    if not (
        math.isfinite(max_command_velocity_rad_s)
        and CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S
        <= max_command_velocity_rad_s
        <= CALIBRATED_STARTUP_COMMAND_VELOCITY_LIMIT_RAD_S
    ):
        raise ValueError(
            "startup speed must be between "
            f"{CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S:.2f} and "
            f"{CALIBRATED_STARTUP_COMMAND_VELOCITY_LIMIT_RAD_S:.2f} rad/s."
        )
    if not math.isfinite(timeout_s) or not 5.0 <= timeout_s <= 180.0:
        raise ValueError("startup timeout must be between 5 and 180 seconds.")
    reference = calibrated_startup_reference_raw(calibration)
    calibrated_low, calibrated_high = sorted(
        (
            calibration.closed_zero_physical_raw,
            calibration.open_max_physical_raw,
        )
    )
    initial_samples: list[Any] = []
    enabled = False
    neutral_command = pack_mit_command(
        0.0,
        0.0,
        0.0,
        CALIBRATED_STARTUP_KD,
        0.0,
        DM_J4310_LIMITS,
    )
    last_command: bytes | None = None
    disabled_probe_count = 0
    enabled_probe_count = 0
    bus.send(GRIPPER_MOTOR_CAN_ID, DISABLE_COMMAND)
    try:
        feedback_deadline = monotonic() + min(
            0.20,
            CALIBRATED_STARTUP_FEEDBACK_TIMEOUT_S,
        )
        while monotonic() < feedback_deadline and len(initial_samples) < 3:
            # Some ESC states reply to MIT queries while disabled, so try the
            # least invasive path first. Other states remain completely silent.
            bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
            last_command = neutral_command
            disabled_probe_count += 1
            parsed = _preposition_feedback(
                bus.recv(
                    min(
                        CALIBRATED_STARTUP_PERIOD_S,
                        max(0.0, feedback_deadline - monotonic()),
                    )
                )
            )
            if parsed is not None:
                initial_samples.append(parsed)

        if len(initial_samples) < 3:
            # A fully disabled ESC 8 is silent on this hardware. Preload the
            # neutral kp=0/kd-only frame, then enable and keep sending neutral
            # frames until stationary position feedback is established. No
            # travel torque is added in this phase.
            initial_samples.clear()
            bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
            bus.send(GRIPPER_MOTOR_CAN_ID, ENABLE_COMMAND)
            enabled = True
            bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
            last_command = neutral_command
            _write_log(
                log_stream,
                "calibrated_startup_feedback_wake_enabled",
                disabled_probe_count=disabled_probe_count,
                neutral_command_hex=neutral_command.hex(),
                kp=0.0,
                kd=CALIBRATED_STARTUP_KD,
                velocity=0.0,
                torque=0.0,
            )
            feedback_deadline = (
                monotonic() + CALIBRATED_STARTUP_FEEDBACK_TIMEOUT_S
            )
            while monotonic() < feedback_deadline and len(initial_samples) < 3:
                bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
                last_command = neutral_command
                enabled_probe_count += 1
                parsed = _preposition_feedback(
                    bus.recv(
                        min(
                            CALIBRATED_STARTUP_PERIOD_S,
                            max(0.0, feedback_deadline - monotonic()),
                        )
                    )
                )
                if parsed is not None:
                    if (
                        abs(float(parsed.velocity))
                        > CALIBRATED_STARTUP_EMERGENCY_VELOCITY_RAD_S
                    ):
                        raise JogSafetyError(
                            "Gripper moved too fast during neutral feedback "
                            "wake: "
                            f"{float(parsed.velocity):.3f} rad/s."
                        )
                    initial_samples.append(parsed)
        if len(initial_samples) < 3:
            raise JogSafetyError(
                "Too few independent gripper feedback samples before "
                "calibrated startup motion "
                f"({len(initial_samples)}/3 after {disabled_probe_count} "
                f"disabled and {enabled_probe_count} enabled neutral probes)."
            )
        initial = initial_samples[-1]
        initial_position = float(initial.position)
        initial_velocity = float(initial.velocity)
        initial_drift = max(
            abs(float(sample.position) - initial_position)
            for sample in initial_samples
        )
        if initial_drift > STARTUP_MAX_MIT_SEGMENT_DRIFT_RAD:
            raise JogSafetyError(
                "Gripper was moving before calibrated startup motion: "
                f"feedback drift {initial_drift:.3f} rad."
            )
        if abs(initial_velocity) > STARTUP_MAX_FINAL_MIT_VELOCITY_RAD_S:
            raise JogSafetyError(
                "Gripper was not stationary before calibrated startup motion: "
                f"velocity {initial_velocity:.3f} rad/s."
            )
        if not (
            calibrated_low - CALIBRATED_STARTUP_CALIBRATION_MARGIN_RAD
            <= initial_position
            <= calibrated_high + CALIBRATED_STARTUP_CALIBRATION_MARGIN_RAD
        ):
            raise JogSafetyError(
                "Current gripper position is outside the saved calibrated "
                "travel; refusing automatic reference motion. "
                f"position={initial_position:.3f}, calibrated="
                f"[{calibrated_low:.3f}, {calibrated_high:.3f}] rad."
            )

        initial_error = reference - initial_position
        _write_log(
            log_stream,
            "calibrated_startup_preposition_checked",
            initial_position_rad=initial_position,
            initial_velocity_rad_s=initial_velocity,
            reference_position_rad=reference,
            initial_error_rad=initial_error,
            calibrated_low_rad=calibrated_low,
            calibrated_high_rad=calibrated_high,
            neutral_disabled_probe_count=disabled_probe_count,
            neutral_enabled_probe_count=enabled_probe_count,
            feedback_wake_required_enable=enabled,
        )
        if abs(initial_error) <= CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD:
            result = {
                "moved": False,
                "initial_position_rad": initial_position,
                "final_position_rad": initial_position,
                "reference_position_rad": reference,
                "elapsed_s": 0.0,
                "max_abs_velocity_rad_s": abs(initial_velocity),
                "command_count": 0,
            }
            _write_log(
                log_stream,
                "calibrated_startup_preposition_complete",
                **result,
            )
            return result

        if not enabled:
            bus.send(GRIPPER_MOTOR_CAN_ID, ENABLE_COMMAND)
            enabled = True
            bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
            last_command = neutral_command
        started = monotonic()
        last_feedback_at = started
        last_log_at = started - 0.05
        latest = initial
        command_count = 0
        sample_count = 0
        overspeed_samples = 0
        stable_since: float | None = None
        target_captured = False
        stall_window_started: float | None = None
        stall_window_best_abs_error = abs(initial_error)
        long_stall_window_started = started
        long_stall_window_best_abs_error = abs(initial_error)
        best_abs_error = abs(initial_error)
        max_abs_velocity = abs(initial_velocity)
        command_torque = 0.0
        last_control_at = started
        _write_log(
            log_stream,
            "calibrated_startup_preposition_started",
            initial_position_rad=initial_position,
            reference_position_rad=reference,
            max_command_velocity_rad_s=(
                max_command_velocity_rad_s
            ),
            timeout_s=timeout_s,
            max_measured_velocity_rad_s=(
                CALIBRATED_STARTUP_MAX_MEASURED_VELOCITY_RAD_S
            ),
            kd=CALIBRATED_STARTUP_KD,
            maximum_feedforward_torque_nm=(
                CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM
            ),
            standstill_torque_nm=CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM,
            moving_bias_min_torque_nm=(
                CALIBRATED_STARTUP_MOVING_BIAS_MIN_TORQUE_NM
            ),
            moving_bias_max_torque_nm=(
                CALIBRATED_STARTUP_MOVING_BIAS_MAX_TORQUE_NM
            ),
            torque_ramp_nm_s=CALIBRATED_STARTUP_TORQUE_RAMP_NM_S,
            stall_arm_torque_nm=CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM,
            long_stall_window_s=CALIBRATED_STARTUP_LONG_STALL_S,
        )
        while True:
            now = monotonic()
            elapsed = now - started
            if elapsed > timeout_s:
                raise JogSafetyError(
                    "Calibrated startup reference motion timed out after "
                    f"{timeout_s:.1f} s."
                )
            position = float(latest.position)
            error = reference - position
            if target_captured and abs(error) <= (
                2.0 * CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD
            ):
                desired_velocity = 0.0
            elif abs(error) <= CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD:
                target_captured = True
                desired_velocity = 0.0
            else:
                target_captured = False
                desired_speed = min(
                    max_command_velocity_rad_s,
                    max(
                        CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S,
                        abs(error) * CALIBRATED_STARTUP_VELOCITY_GAIN,
                    ),
                )
                desired_velocity = math.copysign(desired_speed, error)

            measured_velocity = float(latest.velocity)
            requested_torque, control_velocity = (
                _calibrated_startup_feedforward_torque(
                    desired_velocity,
                    measured_velocity,
                )
            )
            control_dt = max(0.0, now - last_control_at)
            last_control_at = now
            # Limit torque buildup in the travel direction. When measured speed
            # is too high, feedforward drops to zero at once and ESC kd provides
            # braking without an active reverse-torque kick.
            increasing_same_direction = (
                desired_velocity != 0.0
                and math.copysign(1.0, requested_torque)
                == math.copysign(1.0, desired_velocity)
                and abs(requested_torque) > abs(command_torque)
                and (
                    command_torque == 0.0
                    or math.copysign(1.0, command_torque)
                    == math.copysign(1.0, requested_torque)
                )
            )
            if increasing_same_direction:
                torque_step = CALIBRATED_STARTUP_TORQUE_RAMP_NM_S * control_dt
                command_torque = math.copysign(
                    min(
                        abs(requested_torque),
                        abs(command_torque) + torque_step,
                    ),
                    requested_torque,
                )
            else:
                command_torque = requested_torque
            last_command = pack_mit_command(
                0.0,
                0.0,
                0.0,
                CALIBRATED_STARTUP_KD,
                command_torque,
                DM_J4310_LIMITS,
            )
            bus.send(GRIPPER_MOTOR_CAN_ID, last_command)
            command_count += 1

            parsed = _preposition_feedback(
                bus.recv(CALIBRATED_STARTUP_PERIOD_S)
            )
            now = monotonic()
            if parsed is None:
                if now - last_feedback_at > (
                    CALIBRATED_STARTUP_FEEDBACK_WATCHDOG_S
                ):
                    raise JogSafetyError(
                        "Lost independent gripper feedback during calibrated "
                        "startup motion."
                    )
                continue
            latest = parsed
            last_feedback_at = now
            sample_count += 1
            position = float(parsed.position)
            velocity = float(parsed.velocity)
            error = reference - position
            abs_error = abs(error)
            abs_velocity = abs(velocity)
            max_abs_velocity = max(max_abs_velocity, abs_velocity)
            if not all(
                math.isfinite(value)
                for value in (position, velocity, float(parsed.torque))
            ):
                raise JogSafetyError(
                    "Non-finite independent gripper feedback during calibrated "
                    "startup motion."
                )
            if abs_velocity > CALIBRATED_STARTUP_EMERGENCY_VELOCITY_RAD_S:
                _write_log(
                    log_stream,
                    "calibrated_startup_preposition_emergency_velocity",
                    elapsed_s=elapsed,
                    position_rad=position,
                    reference_position_rad=reference,
                    position_error_rad=error,
                    velocity_rad_s=velocity,
                    measured_torque_nm=float(parsed.torque),
                    desired_velocity_rad_s=desired_velocity,
                    command_torque_nm=command_torque,
                    command_hex=last_command.hex(),
                )
                raise JogSafetyError(
                    "Calibrated startup motion emergency velocity exceeded "
                    f"{CALIBRATED_STARTUP_EMERGENCY_VELOCITY_RAD_S:.2f} rad/s "
                    f"(measured {velocity:.3f} rad/s, command torque "
                    f"{command_torque:.3f} Nm)."
                )
            overspeed_samples = (
                overspeed_samples + 1
                if abs_velocity
                > CALIBRATED_STARTUP_MAX_MEASURED_VELOCITY_RAD_S
                else 0
            )
            if overspeed_samples >= CALIBRATED_STARTUP_OVERSPEED_SAMPLES:
                _write_log(
                    log_stream,
                    "calibrated_startup_preposition_sustained_overspeed",
                    elapsed_s=elapsed,
                    position_rad=position,
                    reference_position_rad=reference,
                    position_error_rad=error,
                    velocity_rad_s=velocity,
                    measured_torque_nm=float(parsed.torque),
                    desired_velocity_rad_s=desired_velocity,
                    command_torque_nm=command_torque,
                    overspeed_samples=overspeed_samples,
                )
                raise JogSafetyError(
                    "Calibrated startup measured velocity remained above "
                    f"{CALIBRATED_STARTUP_MAX_MEASURED_VELOCITY_RAD_S:.2f} "
                    "rad/s."
                )
            if abs_error > (
                best_abs_error + CALIBRATED_STARTUP_WRONG_WAY_MARGIN_RAD
            ):
                raise JogSafetyError(
                    "Calibrated startup motion moved away from the reference."
                )
            best_abs_error = min(best_abs_error, abs_error)

            if abs_error > CALIBRATED_STARTUP_STALL_ERROR_RAD:
                long_progress = long_stall_window_best_abs_error - abs_error
                if long_progress >= CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD:
                    long_stall_window_started = now
                    long_stall_window_best_abs_error = abs_error
                elif (
                    now - long_stall_window_started
                    >= CALIBRATED_STARTUP_LONG_STALL_S
                ):
                    _write_log(
                        log_stream,
                        "calibrated_startup_preposition_no_net_progress",
                        elapsed_s=elapsed,
                        position_rad=position,
                        reference_position_rad=reference,
                        position_error_rad=error,
                        velocity_rad_s=velocity,
                        command_torque_nm=command_torque,
                        progress_window_s=now - long_stall_window_started,
                        progress_rad=long_progress,
                        minimum_progress_rad=(
                            CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD
                        ),
                    )
                    raise JogSafetyError(
                        "Calibrated startup motion made less than "
                        f"{CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD:.3f} rad "
                        "net progress for "
                        f"{CALIBRATED_STARTUP_LONG_STALL_S:.1f} s "
                        f"(error={error:.3f} rad)."
                    )
                if stall_window_started is None:
                    stall_window_best_abs_error = abs_error
                    torque_drives_toward_reference = (
                        command_torque != 0.0
                        and math.copysign(1.0, command_torque)
                        == math.copysign(1.0, error)
                    )
                    if (
                        torque_drives_toward_reference
                        and abs(command_torque)
                        >= CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM
                    ):
                        stall_window_started = now
                        _write_log(
                            log_stream,
                            "calibrated_startup_preposition_stall_monitor_armed",
                            elapsed_s=elapsed,
                            position_rad=position,
                            reference_position_rad=reference,
                            position_error_rad=error,
                            command_torque_nm=command_torque,
                            arm_torque_nm=(
                                CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM
                            ),
                        )
                else:
                    progress = stall_window_best_abs_error - abs_error
                    if progress >= CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD:
                        stall_window_started = None
                        stall_window_best_abs_error = abs_error
                    elif (
                        now - stall_window_started
                        >= CALIBRATED_STARTUP_STALL_S
                    ):
                        _write_log(
                            log_stream,
                            "calibrated_startup_preposition_stalled",
                            elapsed_s=elapsed,
                            position_rad=position,
                            reference_position_rad=reference,
                            position_error_rad=error,
                            velocity_rad_s=velocity,
                            control_velocity_rad_s=control_velocity,
                            command_torque_nm=command_torque,
                            stall_window_s=now - stall_window_started,
                            stall_progress_rad=progress,
                            minimum_progress_rad=(
                                CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD
                            ),
                            stall_arm_torque_nm=(
                                CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM
                            ),
                        )
                        raise JogSafetyError(
                            "Calibrated startup motion made less than "
                            f"{CALIBRATED_STARTUP_STALL_MIN_PROGRESS_RAD:.3f} "
                            "rad progress after the command torque reached "
                            f"{CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM:.2f} Nm "
                            f"for {CALIBRATED_STARTUP_STALL_S:.1f} s "
                            f"(error={error:.3f} rad)."
                        )
            else:
                stall_window_started = None
                stall_window_best_abs_error = abs_error
                long_stall_window_started = now
                long_stall_window_best_abs_error = abs_error

            if (
                target_captured
                and abs_error
                <= 2.0 * CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD
                and abs_velocity <= CALIBRATED_STARTUP_FINAL_VELOCITY_RAD_S
            ):
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= CALIBRATED_STARTUP_STABLE_S:
                    result = {
                        "moved": True,
                        "initial_position_rad": initial_position,
                        "final_position_rad": position,
                        "reference_position_rad": reference,
                        "final_error_rad": error,
                        "elapsed_s": elapsed,
                        "max_abs_velocity_rad_s": max_abs_velocity,
                        "command_count": command_count,
                        "sample_count": sample_count,
                    }
                    _write_log(
                        log_stream,
                        "calibrated_startup_preposition_complete",
                        **result,
                    )
                    return result
            else:
                stable_since = None

            if now - last_log_at >= 0.05:
                _write_log(
                    log_stream,
                    "calibrated_startup_preposition_sample",
                    elapsed_s=elapsed,
                    position_rad=position,
                    reference_position_rad=reference,
                    position_error_rad=error,
                    velocity_rad_s=velocity,
                    control_velocity_rad_s=control_velocity,
                    measured_torque_nm=float(parsed.torque),
                    desired_velocity_rad_s=desired_velocity,
                    command_torque_nm=command_torque,
                    overspeed_samples=overspeed_samples,
                )
                last_log_at = now
    finally:
        if enabled:
            # Remove the velocity request before disabling. The following
            # DISABLE remains the final frame even when an exception occurs.
            bus.send(GRIPPER_MOTOR_CAN_ID, neutral_command)
        bus.send(GRIPPER_MOTOR_CAN_ID, DISABLE_COMMAND)
        _write_log(
            log_stream,
            "calibrated_startup_preposition_disabled",
            enabled=enabled,
            last_command_hex=(
                None if last_command is None else last_command.hex()
            ),
        )


class StartupEscDisableGuard:
    """Keep ESC 8 disabled only while the vendor's mandatory Init runs."""

    def __init__(
        self,
        bus: Any,
        *,
        log_stream: TextIO,
        period_s: float = STARTUP_DISABLE_PERIOD_S,
    ) -> None:
        self.bus = bus
        self.log_stream = log_stream
        self.period_s = float(period_s)
        self.started_at = 0.0
        self.disable_count = 0
        self.immediate_disable_count = 0
        self.vendor_id8_frame_count = 0
        self.vendor_enable_count = 0
        self.vendor_disable_count = 0
        self.vendor_mit_count = 0
        self.vendor_other_system_count = 0
        self.vendor_set_zero_count = 0
        self.vendor_malformed_count = 0
        self.last_vendor_id8_payload_hex: str | None = None
        self.latest_vendor_mit_command: dict[str, Any] | None = None
        self._vendor_mit_commands: deque[dict[str, Any]] = deque(
            maxlen=PREFLIGHT_MIT_COMMAND_HISTORY
        )
        self._vendor_mit_lock = threading.Lock()
        self.mit_feedback_count = 0
        self.mit_feedback_can_ids: set[int] = set()
        self.first_mit_position_rad: float | None = None
        self.latest_mit_position_rad: float | None = None
        self.latest_mit_velocity_rad_s: float | None = None
        self.latest_mit_torque_nm: float | None = None
        self.latest_mit_status: int | None = None
        self.max_mit_segment_drift_rad = 0.0
        self.max_abs_mit_velocity_rad_s = 0.0
        self.mit_position_rebase_count = 0
        self._mit_segment_reference_rad: float | None = None
        self._mit_rebase_pending = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._sender_stopped = False
        self._suppression_active = True
        self._handoff_enabled = False
        self._handoff_reference_position_rad: float | None = None
        self._handed_off = False
        self._closed = False

    def _send_disable(self) -> None:
        self.bus.send(GRIPPER_MOTOR_CAN_ID, DISABLE_COMMAND)
        self.disable_count += 1

    def _record_mit_feedback(self, can_id: int, feedback: Any) -> None:
        position = float(feedback.position)
        velocity = float(feedback.velocity)
        self.mit_feedback_count += 1
        self.mit_feedback_can_ids.add(int(can_id))
        if self.first_mit_position_rad is None:
            self.first_mit_position_rad = position
        if self._mit_segment_reference_rad is None or self._mit_rebase_pending:
            if self._mit_rebase_pending:
                self.mit_position_rebase_count += 1
            self._mit_segment_reference_rad = position
            self._mit_rebase_pending = False
        else:
            self.max_mit_segment_drift_rad = max(
                self.max_mit_segment_drift_rad,
                abs(position - self._mit_segment_reference_rad),
            )
        self.max_abs_mit_velocity_rad_s = max(
            self.max_abs_mit_velocity_rad_s,
            abs(velocity),
        )
        self.latest_mit_position_rad = position
        self.latest_mit_velocity_rad_s = velocity
        self.latest_mit_torque_nm = float(feedback.torque)
        self.latest_mit_status = int(feedback.status)

    def _mit_feedback_summary(self) -> dict[str, Any]:
        return {
            "sample_count": self.mit_feedback_count,
            "feedback_can_ids": sorted(self.mit_feedback_can_ids),
            "first_position_rad": self.first_mit_position_rad,
            "latest_position_rad": self.latest_mit_position_rad,
            "latest_velocity_rad_s": self.latest_mit_velocity_rad_s,
            "latest_torque_nm": self.latest_mit_torque_nm,
            "latest_status": self.latest_mit_status,
            "max_segment_drift_rad": self.max_mit_segment_drift_rad,
            "max_abs_velocity_rad_s": self.max_abs_mit_velocity_rad_s,
            "position_rebase_count": self.mit_position_rebase_count,
        }

    def require_stationary_mit_feedback(self, *, phase: str) -> None:
        """Require independent motor feedback to remain stationary under guard."""
        self.check()
        summary = self._mit_feedback_summary()
        _write_log(
            self.log_stream,
            "startup_mit_feedback_checked",
            phase=phase,
            **summary,
        )
        if self.mit_feedback_count < STARTUP_MIN_MIT_FEEDBACK_SAMPLES:
            raise JogSafetyError(
                "Too few independent gripper MIT feedback samples during "
                f"{phase}: {self.mit_feedback_count}."
            )
        if self.max_mit_segment_drift_rad > STARTUP_MAX_MIT_SEGMENT_DRIFT_RAD:
            raise JogSafetyError(
                "Independent gripper MIT position drift exceeded "
                f"{STARTUP_MAX_MIT_SEGMENT_DRIFT_RAD:.2f} rad during {phase}."
            )
        if self.max_abs_mit_velocity_rad_s > STARTUP_MAX_MIT_VELOCITY_RAD_S:
            raise JogSafetyError(
                "Independent gripper MIT velocity exceeded "
                f"{STARTUP_MAX_MIT_VELOCITY_RAD_S:.2f} rad/s during {phase}."
            )
        if (
            self.latest_mit_velocity_rad_s is None
            or abs(self.latest_mit_velocity_rad_s)
            > STARTUP_MAX_FINAL_MIT_VELOCITY_RAD_S
        ):
            raise JogSafetyError(
                "Independent gripper MIT feedback was not stationary at the end "
                f"of {phase}."
            )

    def _worker(self) -> None:
        try:
            next_periodic = time.monotonic() + self.period_s
            while not self._stop.is_set():
                now = time.monotonic()
                timeout = (
                    max(0.0, min(self.period_s, next_periodic - now))
                    if self._suppression_active
                    else self.period_s
                )
                received = self.bus.recv(timeout)
                now = time.monotonic()
                if (
                    received is not None
                    and int(received[0]) == GRIPPER_FEEDBACK_CAN_ID
                ):
                    parsed = matching_feedback(
                        int(received[0]),
                        bytes(received[1]),
                        motor_can_id=GRIPPER_MOTOR_CAN_ID,
                        feedback_can_id=GRIPPER_FEEDBACK_CAN_ID,
                        limits=DM_J4310_LIMITS,
                    )
                    if parsed is not None:
                        self._record_mit_feedback(int(received[0]), parsed)
                        if self._handoff_enabled and not self._handed_off:
                            if (
                                abs(float(parsed.velocity))
                                > HANDOFF_DIRECT_MAX_VELOCITY_RAD_S
                            ):
                                raise JogSafetyError(
                                    "Independent MIT handoff velocity exceeded "
                                    f"{HANDOFF_DIRECT_MAX_VELOCITY_RAD_S:.2f} "
                                    "rad/s."
                                )
                            reference = self._handoff_reference_position_rad
                            if (
                                reference is not None
                                and abs(float(parsed.position) - reference)
                                > HANDOFF_DIRECT_MAX_EXCURSION_RAD
                            ):
                                raise JogSafetyError(
                                    "Independent MIT handoff excursion exceeded "
                                    f"{HANDOFF_DIRECT_MAX_EXCURSION_RAD:.2f} rad."
                                )
                if received is not None and int(received[0]) == GRIPPER_MOTOR_CAN_ID:
                    payload = bytes(received[1])
                    self.vendor_id8_frame_count += 1
                    self.last_vendor_id8_payload_hex = payload.hex()
                    if payload == ENABLE_COMMAND:
                        self.vendor_enable_count += 1
                    elif payload == DISABLE_COMMAND:
                        self.vendor_disable_count += 1
                    elif is_system_command(payload):
                        self.vendor_other_system_count += 1
                        if payload == SET_ZERO_COMMAND:
                            self.vendor_set_zero_count += 1
                            # A SET_ZERO changes encoder coordinates without
                            # physical motion. Start a fresh drift segment at
                            # the next independent feedback sample.
                            self._mit_rebase_pending = True
                    elif len(payload) == 8:
                        self.vendor_mit_count += 1
                        decoded = unpack_mit_command(payload, DM_J4310_LIMITS)
                        command = {
                            "sequence": self.vendor_mit_count,
                            "payload_hex": payload.hex(),
                            **decoded,
                        }
                        with self._vendor_mit_lock:
                            self.latest_vendor_mit_command = command
                            self._vendor_mit_commands.append(command)
                        if (
                            self._handoff_enabled
                            and abs(float(command["torque"]))
                            > HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM
                        ):
                            _write_log(
                                self.log_stream,
                                "vendor_mit_torque_limit_exceeded",
                                command=command,
                                limit_nm=HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM,
                                handed_off=self._handed_off,
                            )
                            raise JogSafetyError(
                                "Vendor MIT command torque exceeded "
                                f"{HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM:.2f} Nm."
                            )
                    else:
                        self.vendor_malformed_count += 1

                    if self._suppression_active:
                        # This socket has CAN_RAW_RECV_OWN_MSGS disabled. An
                        # observed ID-8 frame came from the vendor SDK, so
                        # revoke it immediately until monitored handoff starts.
                        self._send_disable()
                        self.immediate_disable_count += 1
                        next_periodic = now + self.period_s
                elif self._suppression_active and now >= next_periodic:
                    self._send_disable()
                    next_periodic = now + self.period_s
        except BaseException as exc:
            if self._handoff_enabled:
                try:
                    self._send_disable()
                    _write_log(
                        self.log_stream,
                        (
                            "runtime_esc_torque_emergency_disabled"
                            if self._handed_off
                            else "startup_esc_handoff_emergency_disabled"
                        ),
                        error=repr(exc),
                    )
                except BaseException as disable_exc:
                    _write_log(
                        self.log_stream,
                        "startup_esc_handoff_emergency_disable_failed",
                        error=repr(disable_exc),
                    )
            self._error = exc
            self._stop.set()

    def start(self) -> None:
        self.started_at = time.monotonic()
        self._send_disable()
        self._thread = threading.Thread(
            target=self._worker,
            name="x5-gripper-init-disable",
            daemon=True,
        )
        self._thread.start()
        _write_log(
            self.log_stream,
            "startup_esc_disable_started",
            motor_can_id=GRIPPER_MOTOR_CAN_ID,
            period_s=self.period_s,
        )

    def check(self) -> None:
        if self._error is not None:
            raise JogSafetyError(
                f"Independent ESC monitor failed: {self._error}"
            ) from self._error

    def wait_for_vendor_init(self) -> None:
        deadline = self.started_at + VENDOR_INIT_SUPPRESS_S
        while time.monotonic() < deadline:
            self.check()
            time.sleep(min(0.02, deadline - time.monotonic()))
        self.check()
        _write_log(
            self.log_stream,
            "vendor_init_suppression_complete",
            elapsed_s=time.monotonic() - self.started_at,
            disable_count=self.disable_count,
            immediate_disable_count=self.immediate_disable_count,
            vendor_id8_frame_count=self.vendor_id8_frame_count,
            vendor_enable_count=self.vendor_enable_count,
            vendor_disable_count=self.vendor_disable_count,
            vendor_mit_count=self.vendor_mit_count,
            vendor_other_system_count=self.vendor_other_system_count,
            vendor_set_zero_count=self.vendor_set_zero_count,
            vendor_malformed_count=self.vendor_malformed_count,
            last_vendor_id8_payload_hex=self.last_vendor_id8_payload_hex,
            mit_feedback=self._mit_feedback_summary(),
        )

    def capture_vendor_mit_commands(
        self,
        *,
        after_sequence: int,
        sample_count: int = DISABLED_CATCH_PROBE_SAMPLES,
        timeout_s: float = DISABLED_CATCH_PROBE_TIMEOUT_S,
        poll_s: float = DISABLED_CATCH_PROBE_POLL_S,
    ) -> list[dict[str, Any]]:
        """Capture decoded vendor frames while ESC 8 remains disabled."""
        if self._sender_stopped or self._handoff_enabled:
            raise JogSafetyError(
                "Vendor MIT capture requires the ESC disable guard."
            )
        required = int(sample_count)
        if required <= 0:
            raise ValueError("Vendor MIT capture sample count must be positive.")
        deadline = time.monotonic() + float(timeout_s)
        last_sequence = int(after_sequence)
        captured: list[dict[str, Any]] = []
        while time.monotonic() < deadline:
            self.check()
            with self._vendor_mit_lock:
                commands = [
                    dict(command)
                    for command in self._vendor_mit_commands
                    if int(command["sequence"]) > last_sequence
                ]
            for command in commands:
                last_sequence = int(command["sequence"])
                captured.append(command)
                if len(captured) >= required:
                    return captured
            time.sleep(float(poll_s))
        raise JogSafetyError(
            f"Captured only {len(captured)}/{required} vendor MIT commands "
            f"within {float(timeout_s):.2f} s while ESC 8 was disabled."
        )

    def _stop_sender(self) -> None:
        if self._sender_stopped:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.period_s * 5.0))
            if self._thread.is_alive():
                raise JogSafetyError("Startup ESC disable sender did not stop.")
        self.check()
        self._sender_stopped = True

    def enable_for_handoff(self) -> None:
        """Enable ESC 8 while the receiver keeps emergency DISABLE control."""
        if VENDOR_CATCH_ENABLE_LOCKED:
            raise JogSafetyError(
                "Vendor Catch ENABLE is locked pending disabled torque-map "
                "review."
            )
        self.check()
        if self.latest_mit_position_rad is None:
            raise JogSafetyError("No independent MIT position before handoff.")
        self._handoff_reference_position_rad = self.latest_mit_position_rad
        self._handoff_enabled = True
        self._suppression_active = False
        self.bus.send(GRIPPER_MOTOR_CAN_ID, ENABLE_COMMAND)
        _write_log(
            self.log_stream,
            "startup_esc_handoff_enabled",
            motor_can_id=GRIPPER_MOTOR_CAN_ID,
            enable_count=1,
            zero_error_verification_pending=True,
            direct_command_torque_limit_nm=(
                HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM
            ),
            direct_velocity_limit_rad_s=HANDOFF_DIRECT_MAX_VELOCITY_RAD_S,
            direct_excursion_limit_rad=HANDOFF_DIRECT_MAX_EXCURSION_RAD,
            direct_reference_position_rad=(
                self._handoff_reference_position_rad
            ),
        )

    def complete_handoff(self) -> None:
        """Keep direct CAN read-only after zero-error verification passes."""
        if not self._handoff_enabled:
            raise JogSafetyError("ESC 8 handoff was not enabled before completion.")
        self._handed_off = True
        _write_log(
            self.log_stream,
            "startup_esc_handed_to_vendor",
            motor_can_id=GRIPPER_MOTOR_CAN_ID,
            disable_count=self.disable_count,
            enable_count=1,
            immediate_disable_count=self.immediate_disable_count,
            vendor_id8_frame_count=self.vendor_id8_frame_count,
            vendor_enable_count=self.vendor_enable_count,
            vendor_mit_count=self.vendor_mit_count,
            runtime_torque_monitoring=True,
            runtime_command_torque_limit_nm=(
                HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM
            ),
            mit_feedback=self._mit_feedback_summary(),
        )

    def hand_off_to_vendor(self) -> None:
        """Compatibility helper for tests that do not need staged verification."""
        self.enable_for_handoff()
        self.complete_handoff()

    def _close_bus(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.bus.close()
        except OSError:
            pass

    def close_disabled(self) -> None:
        """On startup failure, stop the sender and leave ESC 8 disabled."""
        if self._closed:
            return
        try:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=max(1.0, self.period_s * 5.0))
            if not self._handed_off:
                try:
                    self._send_disable()
                except BaseException as exc:
                    _write_log(
                        self.log_stream,
                        "startup_esc_final_disable_failed",
                        error=repr(exc),
                    )
        finally:
            _write_log(
                self.log_stream,
                "startup_esc_closed",
                handed_off=self._handed_off,
                disable_count=self.disable_count,
                immediate_disable_count=self.immediate_disable_count,
                vendor_id8_frame_count=self.vendor_id8_frame_count,
                vendor_enable_count=self.vendor_enable_count,
                vendor_mit_count=self.vendor_mit_count,
                handoff_enabled=self._handoff_enabled,
                mit_feedback=self._mit_feedback_summary(),
            )
            self._close_bus()


def _wait_for_vendor_id8_quiet(
    startup_guard: StartupEscDisableGuard,
    *,
    log_stream: TextIO,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Require the temporary vendor sender to stop before direct CAN owns ID 8."""
    deadline = monotonic() + VENDOR_PRIME_QUIET_TIMEOUT_S
    last_count = startup_guard.vendor_id8_frame_count
    quiet_since = monotonic()
    while monotonic() < deadline:
        startup_guard.check()
        sleep(0.02)
        current_count = startup_guard.vendor_id8_frame_count
        if current_count != last_count:
            last_count = current_count
            quiet_since = monotonic()
        elif monotonic() - quiet_since >= VENDOR_PRIME_QUIET_S:
            _write_log(
                log_stream,
                "vendor_gripper_prime_sender_quiet",
                vendor_id8_frame_count=current_count,
                quiet_s=monotonic() - quiet_since,
            )
            return
    raise JogSafetyError(
        "Temporary vendor SDK continued sending CAN ID 8 after close; "
        "refusing direct calibrated startup motion."
    )


def _prime_gripper_esc_with_vendor_sdk(
    args: argparse.Namespace,
    startup_guard: StartupEscDisableGuard,
    *,
    log_stream: TextIO,
) -> dict[str, Any]:
    """Configure the gripper ESC under DISABLE, then fully stop the temporary SDK."""
    controllers: list[Any] = []
    prime_result: dict[str, Any] | None = None
    try:
        controllers = backend.create_controllers(
            args.interface,
            [],
            [],
            [],
            [],
            [],
            0,
            1,
            sdk_path=args.sdk_path,
        )
        if len(controllers) != 1:
            raise JogSafetyError(
                "Expected exactly one temporary vendor controller, got "
                f"{len(controllers)}."
            )
        backend.disable_motors(controllers)
        backend.initialize(controllers)
        startup_guard.wait_for_vendor_init()
        startup_guard.require_stationary_mit_feedback(
            phase="vendor_gripper_prime"
        )
        prime_result = {
            "physical_position_rad": startup_guard.latest_mit_position_rad,
            "physical_velocity_rad_s": startup_guard.latest_mit_velocity_rad_s,
            "feedback_sample_count": startup_guard.mit_feedback_count,
            "vendor_id8_frame_count": startup_guard.vendor_id8_frame_count,
        }
        _write_log(
            log_stream,
            "vendor_gripper_prime_complete",
            **prime_result,
        )
    finally:
        if controllers:
            try:
                backend.close(controllers)
            finally:
                controllers = []
        # InterfacesPy stops its native CAN sender in its destructor. Force the
        # wrapper destruction now, then prove ID 8 is quiet before direct CAN.
        gc.collect()
        _wait_for_vendor_id8_quiet(
            startup_guard,
            log_stream=log_stream,
        )
    if prime_result is None:
        raise JogSafetyError("Temporary vendor gripper initialization failed.")
    return prime_result


def bounded_catch_target(
    current: float,
    *,
    direction: int,
    elapsed_s: float,
    open_rate_raw_s: float,
    close_rate_raw_s: float,
    open_target_raw: float,
    close_target_raw: float,
) -> float:
    """Advance a Catch target without exceeding the configured raw rate."""
    if direction not in (-1, 0, 1):
        raise ValueError("Catch direction must be -1, 0, or 1.")
    values = (
        current,
        elapsed_s,
        open_rate_raw_s,
        close_rate_raw_s,
        open_target_raw,
        close_target_raw,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Catch trajectory values must be finite.")
    if elapsed_s < 0.0:
        raise ValueError("Catch elapsed time must be non-negative.")
    if open_rate_raw_s <= 0.0 or close_rate_raw_s <= 0.0:
        raise ValueError("Catch rates must be positive.")
    opened = float(open_target_raw)
    closed = float(close_target_raw)
    lower = min(opened, closed)
    upper = max(opened, closed)
    target = min(upper, max(lower, float(current)))
    if direction == 0:
        return target
    endpoint = opened if direction < 0 else closed
    rate = open_rate_raw_s if direction < 0 else close_rate_raw_s
    step = rate * elapsed_s
    remaining = endpoint - target
    if abs(remaining) <= step:
        return endpoint
    return target + math.copysign(step, remaining)


def _gripper_feedback_value(state: dict[str, Any], name: str) -> float | None:
    value = state.get("additional_sdk_feedback", {}).get(name)
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


class VendorCatchGripperSession:
    """Rate-limit Catch targets while status 5 owns all seven actuators."""

    def __init__(
        self,
        controller: Any,
        *,
        initial_target_raw: float,
        open_target_raw: float,
        close_target_raw: float,
        open_rate_raw_s: float,
        close_rate_raw_s: float,
        refresh_hz: float,
        max_arm_velocity: float,
        log_stream: TextIO,
        physical_origin_raw: float | None = None,
        travel_calibration: GripperTravelCalibration | None = None,
        command_feedback_offset_raw: float | None = None,
        runtime_torque_slope_nm_per_raw: float | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.controller = controller
        self.open_target_raw = float(open_target_raw)
        self.close_target_raw = float(close_target_raw)
        self.open_rate_raw_s = float(open_rate_raw_s)
        self.close_rate_raw_s = float(close_rate_raw_s)
        self.period = 1.0 / float(refresh_hz)
        self.max_arm_velocity = float(max_arm_velocity)
        self.log_stream = log_stream
        self.physical_origin_raw = (
            None if physical_origin_raw is None else float(physical_origin_raw)
        )
        self.travel_calibration = travel_calibration
        self.command_feedback_offset_raw = (
            None
            if command_feedback_offset_raw is None
            else float(command_feedback_offset_raw)
        )
        self.runtime_torque_slope_nm_per_raw = (
            None
            if runtime_torque_slope_nm_per_raw is None
            else float(runtime_torque_slope_nm_per_raw)
        )
        if (
            self.runtime_torque_slope_nm_per_raw is not None
            and (
                not math.isfinite(self.runtime_torque_slope_nm_per_raw)
                or self.runtime_torque_slope_nm_per_raw <= 0.0
            )
        ):
            raise ValueError("Runtime Catch torque slope must be positive.")
        self._monotonic = monotonic
        self._sleep = sleep
        self.started_at = monotonic()
        endpoint_min = min(self.open_target_raw, self.close_target_raw)
        endpoint_max = max(self.open_target_raw, self.close_target_raw)
        self.target_raw = min(
            endpoint_max,
            max(endpoint_min, float(initial_target_raw)),
        )
        self.start_target_raw = self.target_raw
        self._last_update = self.started_at
        self._active_direction = 0
        self._last_direction_key_at: float | None = None
        self._last_direction_key = 0
        self._overspeed_samples = 0
        self._stall_samples = 0
        self.command_count = 0
        self.runtime_sample_count = 0
        self._last_runtime_sample_at = self.started_at - RUNTIME_SAMPLE_PERIOD_S
        self.latest_state: dict[str, Any] | None = None

    def _limit_target_by_live_torque_map(
        self,
        candidate: float,
        *,
        direction: str,
    ) -> float:
        slope = self.runtime_torque_slope_nm_per_raw
        offset = self.command_feedback_offset_raw
        if slope is None or offset is None:
            return float(candidate)
        if self.latest_state is None:
            self.observe(force_log=True)
        assert self.latest_state is not None
        position = _gripper_feedback_value(self.latest_state, "position_raw")
        if position is None:
            raise JogSafetyError(
                "Catch feedback is unavailable for runtime torque limiting."
            )
        neutral_target = position + offset
        max_lead_raw = RUNTIME_TARGET_COMMAND_TORQUE_NM / slope
        limited = min(
            neutral_target + max_lead_raw,
            max(neutral_target - max_lead_raw, float(candidate)),
        )
        endpoint_min = min(self.open_target_raw, self.close_target_raw)
        endpoint_max = max(self.open_target_raw, self.close_target_raw)
        limited = min(endpoint_max, max(endpoint_min, limited))
        if not math.isclose(limited, float(candidate), abs_tol=1e-12):
            _write_log(
                self.log_stream,
                "catch_target_torque_limited",
                direction=direction,
                requested_target_raw=float(candidate),
                limited_target_raw=limited,
                feedback_position_raw=position,
                neutral_target_raw=neutral_target,
                max_lead_raw=max_lead_raw,
                predicted_torque_limit_nm=RUNTIME_TARGET_COMMAND_TORQUE_NM,
                fitted_slope_nm_per_raw=slope,
            )
        return limited

    def apply_key(self, key: str | None, now: float | None = None) -> bool:
        timestamp = self._monotonic() if now is None else float(now)
        if key in QUIT_KEYS:
            self._active_direction = 0
            self._last_direction_key_at = None
            self._last_direction_key = 0
            self._last_update = timestamp
            return True
        if key in STOP_KEYS:
            self._active_direction = 0
            self._last_direction_key_at = None
            self._last_direction_key = 0
            self._last_update = timestamp
            return False
        requested_direction = (
            -1 if key in OPEN_KEYS else 1 if key in CLOSE_KEYS else 0
        )
        if requested_direction:
            endpoint = (
                self.open_target_raw
                if requested_direction < 0
                else self.close_target_raw
            )
            if math.isclose(self.target_raw, endpoint, abs_tol=1e-12):
                return False
            elapsed = min(self.period, KEYPRESS_JOG_MAX_ELAPSED_S)
            if (
                self._last_direction_key_at is not None
                and requested_direction == self._last_direction_key
            ):
                elapsed = min(
                    KEYPRESS_JOG_MAX_ELAPSED_S,
                    max(self.period, timestamp - self._last_direction_key_at),
                )
            previous = self.target_raw
            candidate = bounded_catch_target(
                self.target_raw,
                direction=requested_direction,
                elapsed_s=elapsed,
                open_rate_raw_s=self.open_rate_raw_s,
                close_rate_raw_s=self.close_rate_raw_s,
                open_target_raw=self.open_target_raw,
                close_target_raw=self.close_target_raw,
            )
            direction_name = "open" if requested_direction < 0 else "close"
            self.target_raw = self._limit_target_by_live_torque_map(
                candidate,
                direction=direction_name,
            )
            if not math.isclose(self.target_raw, previous, abs_tol=1e-12):
                self.controller.set_gripper_raw_position(self.target_raw)
                self.command_count += 1
                _write_log(
                    self.log_stream,
                    "catch_target_updated",
                    direction=direction_name,
                    trigger="key_press",
                    previous_target_raw=previous,
                    target_raw=self.target_raw,
                    elapsed_s=elapsed,
                )
            self._active_direction = 0
            self._last_direction_key_at = timestamp
            self._last_direction_key = requested_direction
            self._last_update = timestamp
            if math.isclose(self.target_raw, endpoint, abs_tol=1e-12):
                _write_log(
                    self.log_stream,
                    "catch_target_endpoint_reached",
                    direction=direction_name,
                    trigger="key_press",
                    target_raw=self.target_raw,
                )
        return False

    def _advance(self, now: float) -> None:
        timestamp = max(float(now), self._last_update)
        self._active_direction = 0
        self._last_update = timestamp

    def _hold_measured_and_stop(self, state: dict[str, Any], reason: str) -> None:
        measured = _gripper_feedback_value(state, "position_raw")
        if measured is not None:
            endpoint_min = min(self.open_target_raw, self.close_target_raw)
            endpoint_max = max(self.open_target_raw, self.close_target_raw)
            self.target_raw = min(
                endpoint_max,
                max(endpoint_min, measured),
            )
            self.controller.set_gripper_raw_position(self.target_raw)
        self._active_direction = 0
        self._stall_samples = 0
        _write_log(
            self.log_stream,
            "catch_safety_hold",
            reason=reason,
            hold_target_raw=self.target_raw,
            state=state,
        )

    def _log_runtime_sample(
        self,
        state: dict[str, Any],
        *,
        now: float,
        force: bool,
    ) -> None:
        if not force and now - self._last_runtime_sample_at < RUNTIME_SAMPLE_PERIOD_S:
            return
        position = _gripper_feedback_value(state, "position_raw")
        velocity = _gripper_feedback_value(state, "velocity_raw")
        effort = _gripper_feedback_value(state, "effort0")
        arm_velocities = [
            float(value) for value in state["arm"]["velocity_rad_s"]
        ]
        direction = (
            "open"
            if self._active_direction < 0
            else "close" if self._active_direction > 0 else "hold"
        )
        self.runtime_sample_count += 1
        self._last_runtime_sample_at = now
        physical_position = (
            None
            if position is None or self.physical_origin_raw is None
            else self.physical_origin_raw + position
        )
        normalized_position = (
            None
            if physical_position is None or self.travel_calibration is None
            else self.travel_calibration.normalized_position_raw(
                physical_position
            )
        )
        torque_control_error = (
            None
            if position is None or self.command_feedback_offset_raw is None
            else self.target_raw
            - (position + self.command_feedback_offset_raw)
        )
        predicted_command_torque = (
            None
            if torque_control_error is None
            or self.runtime_torque_slope_nm_per_raw is None
            else torque_control_error * self.runtime_torque_slope_nm_per_raw
        )
        _write_log(
            self.log_stream,
            "catch_runtime_sample",
            sample_index=self.runtime_sample_count,
            elapsed_s=now - self.started_at,
            final=bool(force),
            direction=direction,
            target_raw=self.target_raw,
            position_raw=position,
            physical_position_raw=physical_position,
            calibrated_position_raw=normalized_position,
            velocity_raw_s=velocity,
            effort0=effort,
            tracking_error_raw=(
                None if position is None else self.target_raw - position
            ),
            torque_control_error_raw=torque_control_error,
            predicted_command_torque_nm=predicted_command_torque,
            catch_status=state.get("gripper", {}).get("catch_status"),
            at_open_target=math.isclose(
                self.target_raw, self.open_target_raw, abs_tol=1e-9
            ),
            at_close_target=math.isclose(
                self.target_raw, self.close_target_raw, abs_tol=1e-9
            ),
            arm_velocity_rad_s=arm_velocities,
            arm_max_velocity_rad_s=max(abs(value) for value in arm_velocities),
            gripper_overspeed_samples=self._overspeed_samples,
            gripper_stall_samples=self._stall_samples,
        )

    def observe(self, *, force_log: bool = False) -> dict[str, Any]:
        state = _diagnostic(self.controller)
        self.latest_state = state
        now = self._monotonic()
        arm_velocity = max(
            abs(float(value)) for value in state["arm"]["velocity_rad_s"]
        )
        gripper_velocity = _gripper_feedback_value(state, "velocity_raw")
        gripper_position = _gripper_feedback_value(state, "position_raw")
        gripper_effort = _gripper_feedback_value(state, "effort0")
        if (
            gripper_velocity is not None
            and abs(gripper_velocity) > MAX_GRIPPER_VELOCITY_RAW_S
        ):
            self._overspeed_samples += 1
        else:
            self._overspeed_samples = 0
        tracking_error = (
            None
            if gripper_position is None
            else self.target_raw - gripper_position
        )
        stalled = (
            gripper_velocity is not None
            and gripper_effort is not None
            and tracking_error is not None
            and abs(gripper_velocity) <= STALL_MAX_VELOCITY_RAW_S
            and abs(gripper_effort) >= STALL_MIN_EFFORT0
            and abs(tracking_error) >= STALL_MIN_TRACKING_ERROR_RAW
        )
        self._stall_samples = self._stall_samples + 1 if stalled else 0
        self._log_runtime_sample(
            state,
            now=now,
            force=force_log,
        )
        if arm_velocity > self.max_arm_velocity:
            self._hold_measured_and_stop(state, "arm_velocity")
            raise JogSafetyError(
                f"Measured arm velocity exceeded {self.max_arm_velocity:.3f} rad/s."
            )
        if self._overspeed_samples >= GRIPPER_OVERSPEED_SAMPLES:
            self._hold_measured_and_stop(state, "gripper_velocity")
            raise JogSafetyError(
                "Measured gripper velocity exceeded "
                f"{MAX_GRIPPER_VELOCITY_RAW_S:.2f} raw/s for "
                f"{GRIPPER_OVERSPEED_SAMPLES} consecutive samples."
            )
        if self._stall_samples >= STALL_CONSECUTIVE_SAMPLES:
            stalled_samples = self._stall_samples
            self._hold_measured_and_stop(state, "gripper_stall")
            raise JogSafetyError(
                "Gripper stalled with low velocity, high effort0, and "
                "large tracking error for "
                f"{stalled_samples} consecutive samples."
            )
        return state

    def pump(self, now: float | None = None, *, wait: bool = False) -> None:
        timestamp = self._monotonic() if now is None else float(now)
        self._advance(timestamp)
        self.observe()
        if wait:
            self._sleep(self.period)

    def idle_pump(self) -> None:
        self.pump(self._monotonic(), wait=False)

    def result(self) -> dict[str, Any]:
        state = self.latest_state or self.observe()
        relative_position = _gripper_feedback_value(state, "position_raw")
        physical_position = (
            None
            if relative_position is None or self.physical_origin_raw is None
            else self.physical_origin_raw + relative_position
        )
        return {
            "passed": True,
            "control_mode": "vendor_catch_position",
            "runtime_direct_can_commands": 0,
            "start_target_raw": self.start_target_raw,
            "final_target_raw": self.target_raw,
            "final_position_raw": relative_position,
            "final_physical_position_raw": physical_position,
            "final_calibrated_position_raw": (
                None
                if physical_position is None or self.travel_calibration is None
                else self.travel_calibration.normalized_position_raw(
                    physical_position
                )
            ),
            "catch_command_count": self.command_count,
            "runtime_sample_count": self.runtime_sample_count,
        }


class GripperCalibrationRecorder:
    """Capture closed/open points from stable runtime feedback and persist them."""

    def __init__(
        self,
        *,
        path: Path,
        interface: str,
        physical_origin_raw: float,
        command_feedback_offset_raw: float,
        log_stream: TextIO,
    ) -> None:
        self.path = Path(path)
        self.interface = interface
        self.physical_origin_raw = float(physical_origin_raw)
        self.command_feedback_offset_raw = float(command_feedback_offset_raw)
        self.log_stream = log_stream
        self.pending_closed_zero_physical_raw: float | None = None
        self.saved_calibration: GripperTravelCalibration | None = None

    def _stable_position(
        self,
        session: VendorCatchGripperSession,
        *,
        point: str,
    ) -> tuple[float, float, dict[str, Any]]:
        session.apply_key(" ")
        state = session.observe(force_log=True)
        relative = _gripper_feedback_value(state, "position_raw")
        velocity = _gripper_feedback_value(state, "velocity_raw")
        if relative is None or velocity is None:
            raise ValueError("夹爪位置或速度反馈不可用，不能保存标定点。")
        if abs(velocity) > GRIPPER_CALIBRATION_CAPTURE_MAX_VELOCITY_RAW_S:
            _write_log(
                self.log_stream,
                "gripper_calibration_capture_rejected",
                point=point,
                position_raw=relative,
                velocity_raw_s=velocity,
                reason="moving",
            )
            raise ValueError(
                "夹爪仍在运动；未保存该点，请等待静止后再次按键。"
            )
        return relative, self.physical_origin_raw + relative, state

    def capture_closed_zero(
        self,
        session: VendorCatchGripperSession,
    ) -> float:
        relative, physical, state = self._stable_position(
            session,
            point="closed_zero",
        )
        self.pending_closed_zero_physical_raw = physical
        _write_log(
            self.log_stream,
            "gripper_calibration_closed_zero_captured",
            sdk_relative_position_raw=relative,
            physical_position_raw=physical,
            target_raw=session.target_raw,
            state=state,
        )
        return physical

    def capture_open_max_and_save(
        self,
        session: VendorCatchGripperSession,
    ) -> GripperTravelCalibration:
        if self.pending_closed_zero_physical_raw is None:
            raise ValueError("请先将夹爪闭合到零点并按 Z，再保存最大张开位置。")
        relative, physical, state = self._stable_position(
            session,
            point="open_max",
        )
        calibration = GripperTravelCalibration(
            interface=self.interface,
            closed_zero_physical_raw=self.pending_closed_zero_physical_raw,
            open_max_physical_raw=physical,
            calibrated_at=datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
        )
        _validate_gripper_calibration(calibration, interface=self.interface)
        saved_path = save_gripper_calibration(self.path, calibration)
        self.saved_calibration = calibration
        session.travel_calibration = calibration
        _write_log(
            self.log_stream,
            "gripper_calibration_saved",
            calibration_path=str(saved_path),
            sdk_relative_position_raw=relative,
            physical_position_raw=physical,
            target_raw=session.target_raw,
            command_feedback_offset_raw=self.command_feedback_offset_raw,
            calibration=calibration.as_dict(),
            state=state,
        )
        return calibration


def _wait_for_gripper_settle(
    controller: Any,
    *,
    log_stream: TextIO,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[dict[str, Any], float]:
    """Wait for vendor Init to expose a stable Catch-scale seventh axis."""
    started = monotonic()
    stable = 0
    while monotonic() - started < INIT_SETTLE_TIMEOUT_S:
        latest = _diagnostic(controller)
        position = _gripper_feedback_value(latest, "position_raw")
        velocity = _gripper_feedback_value(latest, "velocity_raw")
        valid = (
            monotonic() - started >= INIT_SETTLE_MIN_S
            and position is not None
            and velocity is not None
            and -1.0 <= position <= GRIPPER_RAW_MAX + 0.25
            and abs(velocity) <= INIT_SETTLE_VELOCITY_RAW_S
        )
        stable = stable + 1 if valid else 0
        if stable >= INIT_SETTLE_STABLE_SAMPLES:
            target = min(GRIPPER_RAW_MAX, max(GRIPPER_RAW_MIN, position))
            _write_log(
                log_stream,
                "catch_init_settled",
                position_raw=position,
                velocity_raw_s=velocity,
                settled_clamped_position_raw=target,
                stable_samples=stable,
            )
            return latest, target
        sleep(1.0 / DEFAULT_REFRESH_HZ)
    raise JogSafetyError(
        "Vendor gripper Init did not provide stable Catch feedback within "
        f"{INIT_SETTLE_TIMEOUT_S:.0f} s."
    )


def _validate_physical_catch_coordinate(
    initial_state: dict[str, Any],
    startup_guard: StartupEscDisableGuard,
) -> tuple[float, float, float]:
    """Prove early SDK feedback and independent MIT use one coordinate."""
    sdk_position = _gripper_feedback_value(initial_state, "position_raw")
    mit_position = startup_guard.latest_mit_position_rad
    if sdk_position is None or mit_position is None:
        raise JogSafetyError(
            "Cannot establish the gripper handoff coordinate from SDK and MIT "
            "feedback."
        )
    target = float(mit_position)
    coordinate_error = target - float(sdk_position)
    if not math.isfinite(target) or not math.isfinite(coordinate_error):
        raise JogSafetyError("Physical gripper handoff target is not finite.")
    if not -DM_J4310_LIMITS.p_max <= target <= DM_J4310_LIMITS.p_max:
        raise JogSafetyError(
            "Independent MIT gripper position is outside the allowed Catch "
            f"motor range [-{DM_J4310_LIMITS.p_max:.1f}, "
            f"{DM_J4310_LIMITS.p_max:.1f}]."
        )
    if abs(coordinate_error) > PREFLIGHT_MIT_SDK_COORDINATE_TOLERANCE:
        raise JogSafetyError(
            "Independent MIT and early SDK gripper coordinates differ by "
            f"more than {PREFLIGHT_MIT_SDK_COORDINATE_TOLERANCE:.2f}."
        )
    return target, float(sdk_position), coordinate_error


def _summarize_vendor_mit_commands(
    commands: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    if not commands:
        raise ValueError("Cannot summarize an empty vendor MIT command set.")
    fields = ("position", "velocity", "kp", "kd", "torque")
    summary: dict[str, Any] = {
        "sample_count": len(commands),
        "first_sequence": int(commands[0]["sequence"]),
        "last_sequence": int(commands[-1]["sequence"]),
        "unique_payload_hex": sorted(
            {str(command["payload_hex"]) for command in commands}
        ),
    }
    for field in fields:
        values = [float(command[field]) for command in commands]
        summary[f"{field}_median"] = median(values)
        summary[f"{field}_min"] = min(values)
        summary[f"{field}_max"] = max(values)
    return summary


def _fit_disabled_catch_torque_map(
    points: Sequence[dict[str, Any]],
) -> dict[str, float]:
    """Fit torque = slope * Catch + intercept and validate the live map."""
    if len(points) < 3:
        raise JogSafetyError("Too few Catch torque-map points for handoff.")
    for point in points:
        if abs(float(point["position_median"])) > (
            DISABLED_MAP_MAX_ABS_POSITION_COMMAND_RAD
        ):
            raise JogSafetyError("Vendor Catch map unexpectedly used MIT position.")
        if abs(float(point["velocity_median"])) > (
            DISABLED_MAP_MAX_ABS_VELOCITY_COMMAND_RAD_S
        ):
            raise JogSafetyError("Vendor Catch map requested unsafe MIT velocity.")
        if float(point["kp_median"]) > DISABLED_MAP_MAX_KP:
            raise JogSafetyError("Vendor Catch map unexpectedly used MIT kp.")
        if not DISABLED_MAP_MIN_KD <= float(point["kd_median"]) <= (
            DISABLED_MAP_MAX_KD
        ):
            raise JogSafetyError("Vendor Catch map MIT kd is outside review bounds.")
    xs = [float(point["target_raw"]) for point in points]
    ys = [float(point["torque_median"]) for point in points]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    if denominator <= 0.0:
        raise JogSafetyError("Catch torque-map targets have no span.")
    slope = sum(
        (target - mean_x) * (torque - mean_y)
        for target, torque in zip(xs, ys)
    ) / denominator
    intercept = mean_y - slope * mean_x
    residuals = [
        torque - (slope * target + intercept)
        for target, torque in zip(xs, ys)
    ]
    max_residual = max(abs(value) for value in residuals)
    if not DISABLED_MAP_MIN_SLOPE_NM_PER_RAW <= slope <= (
        DISABLED_MAP_MAX_SLOPE_NM_PER_RAW
    ):
        raise JogSafetyError(
            "Disabled Catch torque-map slope is outside "
            f"[{DISABLED_MAP_MIN_SLOPE_NM_PER_RAW:.1f}, "
            f"{DISABLED_MAP_MAX_SLOPE_NM_PER_RAW:.1f}] Nm/raw."
        )
    if max_residual > DISABLED_MAP_MAX_RESIDUAL_NM:
        raise JogSafetyError(
            "Disabled Catch torque-map residual exceeded "
            f"{DISABLED_MAP_MAX_RESIDUAL_NM:.2f} Nm."
        )
    zero_target = -intercept / slope
    if not DISABLED_MAP_MIN_ZERO_RAW <= zero_target <= DISABLED_MAP_MAX_ZERO_RAW:
        raise JogSafetyError(
            "Disabled Catch zero-torque target is outside "
            f"[{DISABLED_MAP_MIN_ZERO_RAW:.2f}, "
            f"{DISABLED_MAP_MAX_ZERO_RAW:.2f}] raw."
        )
    return {
        "slope_nm_per_raw": slope,
        "intercept_nm": intercept,
        "zero_torque_target_raw": zero_target,
        "max_abs_residual_nm": max_residual,
    }


def _run_disabled_catch_torque_probe(
    controller: Any,
    startup_guard: StartupEscDisableGuard,
    *,
    log_stream: TextIO,
) -> tuple[list[dict[str, Any]], dict[str, float]]:
    """Map small Catch targets to vendor MIT torque without enabling ESC 8."""
    results: list[dict[str, Any]] = []
    _write_log(
        log_stream,
        "catch_disabled_torque_probe_started",
        targets_raw=list(DISABLED_CATCH_PROBE_TARGETS_RAW),
        samples_per_target=DISABLED_CATCH_PROBE_SAMPLES,
        esc_enable_locked=VENDOR_CATCH_ENABLE_LOCKED,
    )
    for target in DISABLED_CATCH_PROBE_TARGETS_RAW:
        controller.set_gripper_raw_position(target)
        after_sequence = startup_guard.vendor_mit_count
        commands = startup_guard.capture_vendor_mit_commands(
            after_sequence=after_sequence
        )
        point = {
            "target_raw": target,
            **_summarize_vendor_mit_commands(commands),
        }
        results.append(point)
        _write_log(log_stream, "catch_disabled_torque_probe_point", **point)

    fit = _fit_disabled_catch_torque_map(results)
    balance_target = fit["zero_torque_target_raw"]
    controller.set_gripper_raw_position(balance_target)
    restore_after_sequence = startup_guard.vendor_mit_count
    restore_commands = startup_guard.capture_vendor_mit_commands(
        after_sequence=restore_after_sequence
    )
    restore = {
        "target_raw": balance_target,
        **_summarize_vendor_mit_commands(restore_commands),
    }
    if abs(float(restore["torque_median"])) > (
        DISABLED_MAP_MAX_BALANCE_TORQUE_NM
    ):
        raise JogSafetyError(
            "Fitted Catch balance target still requested more than "
            f"{DISABLED_MAP_MAX_BALANCE_TORQUE_NM:.2f} Nm."
        )
    startup_guard.require_stationary_mit_feedback(
        phase="disabled_catch_torque_probe"
    )
    _write_log(
        log_stream,
        "catch_disabled_torque_probe_complete",
        points=results,
        fit=fit,
        balanced=restore,
        esc_enable_locked=VENDOR_CATCH_ENABLE_LOCKED,
        physical_mit_feedback=startup_guard._mit_feedback_summary(),
    )
    return results, fit


def _verify_zero_error_handoff(
    controller: Any,
    *,
    target_raw: float,
    log_stream: TextIO,
    startup_guard: StartupEscDisableGuard | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Verify a stationary Catch handoff before direct CAN is closed."""
    started = monotonic()
    sample_count = 0
    overspeed_samples = 0
    start_position: float | None = None
    latest: dict[str, Any] | None = None
    controller.set_gripper_raw_position(float(target_raw))
    while monotonic() - started < HANDOFF_VERIFY_S:
        if startup_guard is not None:
            startup_guard.check()
        latest = _diagnostic(controller)
        position = _gripper_feedback_value(latest, "position_raw")
        velocity = _gripper_feedback_value(latest, "velocity_raw")
        if position is None or velocity is None:
            raise JogSafetyError(
                "Catch feedback became unavailable during zero-error handoff."
            )
        tracking_error = float(target_raw) - position
        if start_position is None:
            start_position = position
        excursion = abs(position - start_position)
        overspeed_samples = (
            overspeed_samples + 1
            if abs(velocity) > HANDOFF_MAX_VELOCITY_RAW_S
            else 0
        )
        sample_count += 1
        _write_log(
            log_stream,
            "catch_handoff_sample",
            sample_index=sample_count,
            elapsed_s=monotonic() - started,
            target_raw=float(target_raw),
            position_raw=position,
            velocity_raw_s=velocity,
            tracking_error_raw=tracking_error,
            excursion_from_first_raw=excursion,
            effort0=_gripper_feedback_value(latest, "effort0"),
            overspeed_samples=overspeed_samples,
            independent_mit_feedback=(
                None
                if startup_guard is None
                else startup_guard._mit_feedback_summary()
            ),
            latest_vendor_mit_command=(
                None
                if startup_guard is None
                else startup_guard.latest_vendor_mit_command
            ),
        )
        if abs(velocity) > HANDOFF_EMERGENCY_VELOCITY_RAW_S:
            raise JogSafetyError(
                "Catch handoff emergency velocity exceeded "
                f"{HANDOFF_EMERGENCY_VELOCITY_RAW_S:.2f} raw/s."
            )
        if overspeed_samples >= HANDOFF_VELOCITY_CONSECUTIVE_SAMPLES:
            raise JogSafetyError(
                "Catch handoff velocity exceeded "
                f"{HANDOFF_MAX_VELOCITY_RAW_S:.2f} raw/s for "
                f"{HANDOFF_VELOCITY_CONSECUTIVE_SAMPLES} consecutive samples."
            )
        if abs(tracking_error) > HANDOFF_MAX_TRACKING_ERROR_RAW:
            raise JogSafetyError(
                "Catch handoff tracking error exceeded "
                f"{HANDOFF_MAX_TRACKING_ERROR_RAW:.2f} raw."
            )
        if excursion > HANDOFF_MAX_EXCURSION_RAW:
            raise JogSafetyError(
                "Catch handoff excursion exceeded "
                f"{HANDOFF_MAX_EXCURSION_RAW:.2f} raw."
            )
        sleep(HANDOFF_SAMPLE_PERIOD_S)
    if startup_guard is not None:
        startup_guard.check()
    assert latest is not None
    final_position = _gripper_feedback_value(latest, "position_raw")
    final_velocity = _gripper_feedback_value(latest, "velocity_raw")
    assert final_position is not None and final_velocity is not None
    final_tracking_error = float(target_raw) - final_position
    if abs(final_velocity) > HANDOFF_FINAL_MAX_VELOCITY_RAW_S:
        raise JogSafetyError(
            "Catch handoff did not settle below "
            f"{HANDOFF_FINAL_MAX_VELOCITY_RAW_S:.2f} raw/s."
        )
    if abs(final_tracking_error) > HANDOFF_FINAL_MAX_TRACKING_ERROR_RAW:
        raise JogSafetyError(
            "Catch handoff did not settle within "
            f"{HANDOFF_FINAL_MAX_TRACKING_ERROR_RAW:.2f} raw tracking error."
        )
    _write_log(
        log_stream,
        "catch_handoff_verified",
        elapsed_s=monotonic() - started,
        sample_count=sample_count,
        target_raw=float(target_raw),
        final_position_raw=final_position,
        final_velocity_raw_s=final_velocity,
        final_tracking_error_raw=final_tracking_error,
    )
    return latest


def _poll_stdin_key(stream: TextIO) -> str | None:
    ready, _, _ = select.select([stream], [], [], 0)
    return (stream.read(1) or None) if ready else None


def _finite_ik(
    ik_solver: Callable[[np.ndarray], Sequence[float]],
    target_pose: Sequence[float],
) -> list[float]:
    return [float(value) for value in ik_solver(np.asarray(target_pose, dtype=float))]


def _run_cartesian_command(
    session: VendorCatchGripperSession,
    arm: ArmJogContext,
    *,
    key: str,
    log_stream: TextIO,
) -> bool:
    before = _diagnostic(arm.controller)
    before, trajectory_start_joints, normalized = (
        _execute_joint_limit_normalization(
            arm.controller,
            before,
            max_velocity=arm.max_velocity,
            log_stream=log_stream,
            idle_pump=session.idle_pump,
        )
    )
    before_pose = _fk_pose(before, arm.fk_solver)
    if normalized:
        _user_print(
            "\n检测到关节零点边界偏差，已按不超过 0.20 rad/s 平滑归一；"
            "正在重新计算本次笛卡尔轨迹。"
        )
        _write_log(
            log_stream,
            "cartesian_replanned_after_joint_limit_normalization",
            key=key,
            measured_joints_rad=_joint_positions(before),
            trajectory_start_joints_rad=trajectory_start_joints,
            recalculated_pose_xyzrpy=before_pose,
        )
    try:
        if key == "h":
            target_pose = build_return_target(before_pose, arm.startup_pose)
            if _distance(before_pose[:3], target_pose[:3]) <= TARGET_TOLERANCE_M:
                _user_print("\n末端已经位于本次启动零点附近。")
                return False
            raw_solution = _finite_ik(arm.ik_solver, target_pose)
            ik_solution = validate_ik_solution(raw_solution, raw_solution)
            command_type = "return_to_start"
        else:
            axis, direction = KEY_DIRECTIONS[key]
            target_pose = build_cartesian_target(
                before_pose,
                arm.startup_pose,
                axis=axis,
                direction=direction,
            )
            ik_solution = validate_ik_solution(
                arm.ik_solver(np.asarray(target_pose, dtype=float)),
                _joint_positions(before),
            )
            command_type = "cartesian_step"
        _write_log(
            log_stream,
            "command_requested",
            key=key,
            command_type=command_type,
            target_pose_xyzrpy=target_pose,
            predicted_joints_rad=ik_solution,
            before=before,
        )
        after, after_pose, operator_stopped = _execute_interpolated_joint_move(
            arm.controller,
            start_joints=trajectory_start_joints,
            target_joints=ik_solution,
            start_pose=before_pose,
            target_pose=target_pose,
            fk_solver=arm.fk_solver,
            ik_solver=arm.ik_solver,
            motion_timeout=arm.motion_timeout,
            max_velocity=arm.max_velocity,
            log_stream=log_stream,
            idle_pump=session.idle_pump,
            on_key=lambda pressed: session.apply_key(pressed),
        )
    except JogSafetyError as exc:
        message = str(exc)
        if message.startswith("IK J") or "joint target rate" in message:
            _user_print(f"\n这一步笛卡尔未执行：{exc}")
            _write_log(log_stream, "cartesian_rejected", key=key, error=message)
            return False
        raise
    _user_print()
    _print_state(after, after_pose)
    return operator_stopped


def _print_gripper(session: VendorCatchGripperSession) -> None:
    measured = (
        None
        if session.latest_state is None
        else _gripper_feedback_value(session.latest_state, "position_raw")
    )
    measured_text = "不可用" if measured is None else f"{measured:.3f}"
    message = (
        f"夹爪：目标 raw={session.target_raw:.3f}，反馈 raw={measured_text}；"
        f"本次 Catch 端点={session.open_target_raw:.3f}（张开）/"
        f"{session.close_target_raw:.3f}（闭合）。"
    )
    if (
        measured is not None
        and session.physical_origin_raw is not None
        and session.travel_calibration is not None
    ):
        physical = session.physical_origin_raw + measured
        normalized = session.travel_calibration.normalized_position_raw(
            physical
        )
        message += (
            f" 物理绝对值={physical:.3f}，标定值={normalized:.3f}"
            "（0=闭合，最大值=张开）。"
        )
    _user_print(message)


def run_catch_jog(
    arm: ArmJogContext,
    *,
    initial_target_raw: float,
    open_target_raw: float,
    close_target_raw: float,
    open_rate_raw_s: float,
    close_rate_raw_s: float,
    refresh_hz: float,
    max_seconds: float,
    log_stream: TextIO,
    poll_key: Callable[[], str | None],
    calibration_path: Path | None = None,
    physical_origin_raw: float | None = None,
    command_feedback_offset_raw: float = 0.0,
    runtime_torque_slope_nm_per_raw: float | None = None,
    travel_calibration: GripperTravelCalibration | None = None,
    safety_check: Callable[[], None] | None = None,
    gripper_key_layout: str = "c-open",
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    if gripper_key_layout not in GRIPPER_KEY_LAYOUTS:
        raise ValueError(
            f"gripper_key_layout must be one of {sorted(GRIPPER_KEY_LAYOUTS)}."
        )
    session = VendorCatchGripperSession(
        arm.controller,
        initial_target_raw=initial_target_raw,
        open_target_raw=open_target_raw,
        close_target_raw=close_target_raw,
        open_rate_raw_s=open_rate_raw_s,
        close_rate_raw_s=close_rate_raw_s,
        refresh_hz=refresh_hz,
        max_arm_velocity=arm.max_velocity,
        log_stream=log_stream,
        physical_origin_raw=physical_origin_raw,
        travel_calibration=travel_calibration,
        command_feedback_offset_raw=command_feedback_offset_raw,
        runtime_torque_slope_nm_per_raw=runtime_torque_slope_nm_per_raw,
        monotonic=monotonic,
        sleep=sleep,
    )
    recorder = (
        None
        if calibration_path is None or physical_origin_raw is None
        else GripperCalibrationRecorder(
            path=calibration_path,
            interface=ALLOWED_INTERFACE,
            physical_origin_raw=physical_origin_raw,
            command_feedback_offset_raw=command_feedback_offset_raw,
            log_stream=log_stream,
        )
    )
    gripper_keys = (
        "按住 c/] 闭合，按住 o/[ 张开"
        if gripper_key_layout == "c-close"
        else "按住 c/[ 张开，按住 o/] 闭合"
    )
    calibration_keys = (
        "Z保存当前闭合零点，M保存最大张开点并写入标定文件；"
        if recorder is not None
        else "本次标定只读，Z/M已禁用；"
    )
    _user_print(
        "按键：W/S=X+/X-，A/D=Y+/Y-，R/F=Z+/Z-，H回启动零点；"
        f"{gripper_keys}，松键后保持；"
        f"{calibration_keys}"
        "p查看状态，q/Esc保护退出。"
    )
    try:
        while monotonic() - session.started_at < float(max_seconds):
            if safety_check is not None:
                safety_check()
            key = poll_key()
            if key is not None:
                lowered = key.lower()
                if key in QUIT_KEYS:
                    break
                if lowered == "p":
                    state = session.observe()
                    _print_state(state, _fk_pose(state, arm.fk_solver))
                    _print_gripper(session)
                elif lowered == "z":
                    if recorder is None:
                        _user_print("\n未配置夹爪标定文件，不能保存零点。")
                    else:
                        try:
                            physical = recorder.capture_closed_zero(session)
                        except ValueError as exc:
                            _user_print(f"\n闭合零点未保存：{exc}")
                        else:
                            _user_print(
                                "\n已暂存闭合零点："
                                f"物理绝对值 {physical:.4f} raw。"
                                "请用 C 张开到最大，静止后按 M。"
                            )
                elif lowered == "m":
                    if recorder is None:
                        _user_print("\n未配置夹爪标定文件，不能保存最大值。")
                    else:
                        try:
                            saved = recorder.capture_open_max_and_save(session)
                        except ValueError as exc:
                            _user_print(f"\n最大张开点未保存：{exc}")
                        else:
                            _user_print(
                                f"\n夹爪标定已保存：{recorder.path.resolve()}\n"
                                f"闭合=0，最大张开={saved.max_travel_raw:.4f} raw。"
                            )
                elif lowered in KEY_DIRECTIONS or lowered == "h":
                    if _run_cartesian_command(
                        session, arm, key=lowered, log_stream=log_stream
                    ):
                        break
                elif session.apply_key(
                    map_gripper_key(key, gripper_key_layout), monotonic()
                ):
                    break
            session.pump(monotonic(), wait=True)
            if safety_check is not None:
                safety_check()
        result = session.result()
        result["calibration_path"] = (
            None if calibration_path is None else str(calibration_path.resolve())
        )
        result["calibration_saved"] = (
            recorder is not None and recorder.saved_calibration is not None
        )
        _write_log(log_stream, "catch_jog_passed", **result)
        return result
    finally:
        try:
            session.observe(force_log=True)
        except Exception as exc:
            _write_log(
                log_stream,
                "catch_final_snapshot_failed",
                error=repr(exc),
            )
        try:
            backend.disable_motors(arm.controllers)
            _write_log(log_stream, "protection_requested")
        except Exception as exc:
            _write_log(log_stream, "protection_failed", error=repr(exc))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Vendor Catch gripper and Cartesian jog with a disabled torque-map "
            "preflight and independently monitored ESC 8 handoff."
        )
    )
    parser.add_argument("--interface", required=True)
    parser.add_argument("--sdk-path", default=None)
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument(
        "--gripper-calibration",
        type=Path,
        default=None,
        help=(
            "Two-point gripper calibration JSON. Defaults to the robot "
            "calibration directory; a missing file can be created with Z/M."
        ),
    )
    parser.add_argument("--enable-can-jog", action="store_true")
    parser.add_argument("--acknowledge-startup-esc-disable", action="store_true")
    parser.add_argument("--acknowledge-limited-feedback", action="store_true")
    parser.add_argument("--acknowledge-transition-grace-risk", action="store_true")
    parser.add_argument(
        "--gripper-key-layout",
        choices=sorted(GRIPPER_KEY_LAYOUTS),
        default="c-open",
        help="C/O key direction; c-close matches x5_gripper_sdk.",
    )
    parser.add_argument(
        "--read-only-gripper-calibration",
        action="store_true",
        help="Load calibration endpoints but disable Z/M writes during this session.",
    )
    parser.add_argument(
        "--open-target-raw", type=float, default=DEFAULT_OPEN_TARGET_RAW
    )
    parser.add_argument(
        "--close-target-raw", type=float, default=DEFAULT_CLOSE_TARGET_RAW
    )
    parser.add_argument(
        "--open-rate-raw-s", type=float, default=DEFAULT_OPEN_RATE_RAW_S
    )
    parser.add_argument(
        "--close-rate-raw-s", type=float, default=DEFAULT_CLOSE_RATE_RAW_S
    )
    parser.add_argument("--refresh-hz", type=float, default=DEFAULT_REFRESH_HZ)
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_JOG_SECONDS)
    parser.add_argument(
        "--motion-timeout", type=float, default=DEFAULT_MOTION_TIMEOUT_S
    )
    parser.add_argument(
        "--startup-speed-rad-s",
        type=float,
        default=CALIBRATED_STARTUP_MAX_COMMAND_VELOCITY_RAD_S,
        help="Speed request for the calibrated pre-Init reference motion.",
    )
    parser.add_argument(
        "--startup-timeout-s",
        type=float,
        default=CALIBRATED_STARTUP_TIMEOUT_S,
        help="Overall timeout for the calibrated pre-Init reference motion.",
    )
    parser.add_argument(
        "--max-velocity", type=float, default=MAX_MEASURED_VELOCITY_RAD_S
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.interface != ALLOWED_INTERFACE:
        raise ValueError(f"--interface must be exactly {ALLOWED_INTERFACE}.")
    if not args.enable_can_jog:
        raise ValueError("This experiment requires --enable-can-jog.")
    if not args.acknowledge_startup_esc_disable:
        raise ValueError("Requires --acknowledge-startup-esc-disable.")
    if not args.acknowledge_limited_feedback:
        raise ValueError("Requires --acknowledge-limited-feedback.")
    if not args.acknowledge_transition_grace_risk:
        raise ValueError("Requires --acknowledge-transition-grace-risk.")
    endpoints = (float(args.open_target_raw), float(args.close_target_raw))
    if not all(math.isfinite(value) for value in endpoints):
        raise ValueError("Catch endpoints must be finite.")
    if not all(GRIPPER_RAW_MIN <= value <= GRIPPER_RAW_MAX for value in endpoints):
        raise ValueError("Catch targets must both be in [0, 5] raw.")
    if math.isclose(endpoints[0], endpoints[1], abs_tol=1e-9):
        raise ValueError("Catch open and close targets must be different.")
    rates = (float(args.open_rate_raw_s), float(args.close_rate_raw_s))
    if not all(
        math.isfinite(value) and 0.01 <= value <= MAX_CATCH_RATE_RAW_S
        for value in rates
    ):
        raise ValueError(
            f"Catch rates must be in [0.01, {MAX_CATCH_RATE_RAW_S:.1f}] raw/s."
        )
    if not 20.0 <= float(args.refresh_hz) <= 50.0:
        raise ValueError("--refresh-hz must be in [20, 50] Hz.")
    if not 5.0 <= float(args.max_seconds) <= MAX_JOG_SECONDS:
        raise ValueError(f"--max-seconds must be in [5, {MAX_JOG_SECONDS}].")
    if not math.isfinite(float(args.motion_timeout)) or args.motion_timeout <= 0.0:
        raise ValueError("--motion-timeout must be positive.")
    if not math.isfinite(float(args.max_velocity)) or args.max_velocity <= 0.0:
        raise ValueError("--max-velocity must be positive.")
    startup_speed = float(
        getattr(
            args,
            "startup_speed_rad_s",
            CALIBRATED_STARTUP_MAX_COMMAND_VELOCITY_RAD_S,
        )
    )
    if not (
        math.isfinite(startup_speed)
        and CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S
        <= startup_speed
        <= CALIBRATED_STARTUP_COMMAND_VELOCITY_LIMIT_RAD_S
    ):
        raise ValueError(
            "--startup-speed-rad-s must be in "
            f"[{CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S:.2f}, "
            f"{CALIBRATED_STARTUP_COMMAND_VELOCITY_LIMIT_RAD_S:.2f}]."
        )
    startup_timeout = float(
        getattr(args, "startup_timeout_s", CALIBRATED_STARTUP_TIMEOUT_S)
    )
    if not math.isfinite(startup_timeout) or not 5.0 <= startup_timeout <= 180.0:
        raise ValueError("--startup-timeout-s must be in [5, 180].")


def _prepare_arm(
    args: argparse.Namespace,
    log_stream: TextIO,
    startup_guard: StartupEscDisableGuard,
) -> tuple[
    ArmJogContext,
    float,
    list[dict[str, Any]],
    dict[str, float],
    float,
]:
    sdk = backend.load_bimanual_sdk(sdk_path=args.sdk_path)
    vendor_fk_solver = getattr(sdk, "forward_kinematics", None)
    vendor_ik_solver = getattr(sdk, "inverse_kinematics", None)
    if not callable(vendor_fk_solver) or not callable(vendor_ik_solver):
        raise JogSafetyError("Vendor SDK does not expose forward/inverse kinematics.")
    tcp_kinematics = GripperTcpKinematics(
        vendor_fk_solver=vendor_fk_solver,
        vendor_ik_solver=vendor_ik_solver,
    )
    fk_solver = tcp_kinematics.forward_kinematics
    ik_solver = tcp_kinematics.inverse_kinematics
    controllers = backend.create_controllers(
        args.interface, [], [], [], [], [], 0, 1, sdk_path=args.sdk_path
    )
    try:
        if len(controllers) != 1:
            raise JogSafetyError(
                f"Expected exactly one controller, got {len(controllers)}."
            )
        controller = controllers[0]
        backend.disable_motors(controllers)
        backend.initialize(controllers)
        _user_print("等待厂家 Init 和夹爪反馈稳定……")
        initial_state, startup_pose = _warm_up_feedback(
            controller, fk_solver=fk_solver
        )
        _write_log(log_stream, "initial_state", state=initial_state)
        startup_guard.wait_for_vendor_init()
        startup_guard.require_stationary_mit_feedback(phase="vendor_init")
        _, post_init_sdk_target = _wait_for_gripper_settle(
            controller, log_stream=log_stream
        )
        physical_position, early_sdk_position, coordinate_error = (
            _validate_physical_catch_coordinate(initial_state, startup_guard)
        )
        _write_log(
            log_stream,
            "catch_physical_mit_coordinate_validated",
            independent_mit_position_rad=physical_position,
            early_sdk_position_raw=early_sdk_position,
            mit_minus_sdk_coordinate_error=coordinate_error,
            post_init_sdk_position_raw=post_init_sdk_target,
        )
        # The vendor Catch outer loop resets its own feedback coordinate during
        # Init. Keep its stable post-Init target for the disabled mapping phase;
        # the absolute MIT encoder position is observation, not a Catch target.
        initial_target = post_init_sdk_target
        controller.set_gripper_raw_position(initial_target)
        warm_state, warm_pose = _enter_and_warm_up_position_mode(
            controller,
            fk_solver=fk_solver,
            max_velocity=float(args.max_velocity),
            log_stream=log_stream,
        )
        # Map the vendor outer-loop response only while the independent socket
        # continues to revoke every ID-8 command with DISABLE. The fitted
        # zero-torque target becomes the monitored handoff target.
        controller.set_gripper_raw_position(initial_target)
        _write_log(
            log_stream,
            "catch_post_init_probe_target_reprimed",
            initial_target_raw=initial_target,
        )
        probe_results, probe_fit = _run_disabled_catch_torque_probe(
            controller,
            startup_guard,
            log_stream=log_stream,
        )
        initial_target = probe_fit["zero_torque_target_raw"]
        _print_state(warm_state, warm_pose)
        return (
            ArmJogContext(
                controllers=controllers,
                controller=controller,
                fk_solver=fk_solver,
                ik_solver=ik_solver,
                startup_pose=list(startup_pose),
                motion_timeout=float(args.motion_timeout),
                max_velocity=float(args.max_velocity),
            ),
            initial_target,
            probe_results,
            probe_fit,
            physical_position,
        )
    except BaseException:
        try:
            backend.close(controllers)
        except Exception as exc:
            _write_log(log_stream, "sdk_close_failed", error=repr(exc))
        raise


def run_integrated_session(
    argv: Sequence[str] | None = None,
    *,
    jog_runner: Callable[..., dict[str, Any]] | None = None,
    enable_control: bool = False,
) -> None:
    """启动安全交接流程，并把按键阶段交给调用方提供的控制循环。"""
    parser = _parser()
    args = parser.parse_args(argv)
    # Python SDK 调用方通过显式参数授权；旧命令行仍使用 --enable-can-jog。
    if enable_control:
        args.enable_can_jog = True
    try:
        _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    calibration_path = (
        _default_gripper_calibration_path(args.interface)
        if args.gripper_calibration is None
        else args.gripper_calibration.expanduser().resolve()
    )
    try:
        travel_calibration = load_gripper_calibration(
            calibration_path,
            interface=args.interface,
        )
    except ValueError as exc:
        parser.error(str(exc))
    if not sys.stdin.isatty():
        raise JogSafetyError("机械臂/夹爪联合控制需要在交互式终端中运行。")

    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    log_path = resolve_session_log_path(
        args.log, f"x5_2023_gripper_catch_jog_{args.interface}_{timestamp}.jsonl"
    )
    arm: ArmJogContext | None = None
    startup_guard: StartupEscDisableGuard | None = None
    with log_path.open("a", encoding="utf-8") as log_stream:
        _write_log(
            log_stream,
            "start",
            interface=args.interface,
            control_mode="vendor_catch_position",
            startup_direct_can_disable=True,
            runtime_direct_can=False,
            runtime_direct_can_torque_monitoring=True,
            startup_mit_feedback_can_id=GRIPPER_FEEDBACK_CAN_ID,
            startup_max_mit_segment_drift_rad=(
                STARTUP_MAX_MIT_SEGMENT_DRIFT_RAD
            ),
            startup_mode="disabled_map_then_monitored_enable",
            disabled_catch_probe_targets_raw=list(
                DISABLED_CATCH_PROBE_TARGETS_RAW
            ),
            disabled_catch_probe_samples=DISABLED_CATCH_PROBE_SAMPLES,
            vendor_catch_enable_locked=VENDOR_CATCH_ENABLE_LOCKED,
            handoff_direct_max_command_torque_nm=(
                HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM
            ),
            runtime_target_command_torque_nm=(
                RUNTIME_TARGET_COMMAND_TORQUE_NM
            ),
            cartesian_control_frame="gripper_tcp",
            link6_to_flange_offset_m=LINK6_TO_FLANGE_OFFSET_M,
            flange_to_gripper_tcp_offset_m=FLANGE_TO_GRIPPER_TCP_OFFSET_M,
            link6_to_gripper_tcp_offset_m=LINK6_TO_GRIPPER_TCP_OFFSET_M,
            keypress_jog_max_elapsed_s=KEYPRESS_JOG_MAX_ELAPSED_S,
            handoff_direct_max_velocity_rad_s=(
                HANDOFF_DIRECT_MAX_VELOCITY_RAD_S
            ),
            handoff_direct_max_excursion_rad=(
                HANDOFF_DIRECT_MAX_EXCURSION_RAD
            ),
            gripper_calibration_path=str(calibration_path),
            gripper_calibration_loaded=(travel_calibration is not None),
            calibrated_startup_from_any_position=(
                travel_calibration is not None
            ),
            calibrated_startup_vendor_prime=(
                travel_calibration is not None
            ),
            calibrated_startup_max_command_velocity_rad_s=(
                float(args.startup_speed_rad_s)
            ),
            calibrated_startup_timeout_s=float(args.startup_timeout_s),
            calibrated_startup_min_command_velocity_rad_s=(
                CALIBRATED_STARTUP_MIN_COMMAND_VELOCITY_RAD_S
            ),
            calibrated_startup_velocity_gain=(
                CALIBRATED_STARTUP_VELOCITY_GAIN
            ),
            calibrated_startup_velocity_control_gain=(
                CALIBRATED_STARTUP_VELOCITY_CONTROL_GAIN
            ),
            calibrated_startup_control_velocity_deadband_rad_s=(
                CALIBRATED_STARTUP_CONTROL_VELOCITY_DEADBAND_RAD_S
            ),
            calibrated_startup_active_reverse_braking=False,
            calibrated_startup_max_measured_velocity_rad_s=(
                CALIBRATED_STARTUP_MAX_MEASURED_VELOCITY_RAD_S
            ),
            calibrated_startup_position_tolerance_rad=(
                CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD
            ),
            calibrated_startup_max_standstill_torque_nm=(
                CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM
            ),
            calibrated_startup_max_feedforward_torque_nm=(
                CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM
            ),
            calibrated_startup_torque_ramp_nm_s=(
                CALIBRATED_STARTUP_TORQUE_RAMP_NM_S
            ),
            calibrated_startup_stall_arm_torque_nm=(
                CALIBRATED_STARTUP_STALL_ARM_TORQUE_NM
            ),
            gripper_calibration=(
                None
                if travel_calibration is None
                else travel_calibration.as_dict()
            ),
        )
        try:
            _user_print(f"接口：{args.interface}；日志：{log_path}")
            _user_print(
                "笛卡尔控制点：夹爪尖端 TCP；"
                f"法兰到 TCP {FLANGE_TO_GRIPPER_TCP_OFFSET_M * 100.0:.1f} cm，"
                f"SDK link6 到 TCP {LINK6_TO_GRIPPER_TCP_OFFSET_M * 100.0:.1f} cm，"
                "沿局部 +X 方向。"
            )
            _user_print(
                "启动阶段临时向 ID 8 发送 DISABLE，"
                "阻止厂家 Init 带动夹爪并检查独立电机反馈；"
                "先拟合 Catch 零力矩点，再进行受监控 ENABLE。"
            )
            _user_print(
                "ENABLE 后前 0.5 秒独立监控速度和位移；整个按键阶段持续"
                "监听厂家力矩：厂家命令超过"
                f" {HANDOFF_DIRECT_MAX_COMMAND_TORQUE_NM:.2f} Nm 会立即 DISABLE。"
            )
            _user_print(
                "按键目标按实时反馈和本次拟合斜率动态限幅，预计厂家命令"
                f"不超过 {RUNTIME_TARGET_COMMAND_TORQUE_NM:.2f} Nm；"
                "每个键盘事件最多折算 0.02 s。"
            )
            _user_print(
                "探测目标 raw："
                + ", ".join(
                    f"{target:.3f}"
                    for target in DISABLED_CATCH_PROBE_TARGETS_RAW
                )
            )
            if travel_calibration is None:
                _user_print(
                    f"尚无夹爪两点标定：{calibration_path}\n"
                    "本次可用 C/O 调节；闭合到零点并静止后按 Z，"
                    "再张开到最大并静止后按 M 保存。"
                )
            else:
                _user_print(
                    f"已读取夹爪标定：{calibration_path}\n"
                    f"闭合绝对值 {travel_calibration.closed_zero_physical_raw:.4f}，"
                    f"张开绝对值 {travel_calibration.open_max_physical_raw:.4f}，"
                    f"最大行程 {travel_calibration.max_travel_raw:.4f} raw。\n"
                    "可从标定行程内任意位置启动；确认后、厂家 Init 前会"
                    "自动低速移动到参考端点 "
                    f"{calibrated_startup_reference_raw(travel_calibration):.4f}，"
                    "软件目标速度不超过 "
                    f"{float(args.startup_speed_rad_s):.2f} "
                    "rad/s、静止起动力矩 "
                    f"{CALIBRATED_STARTUP_STANDSTILL_TORQUE_NM:.2f} Nm、"
                    "前馈硬上限 "
                    f"{CALIBRATED_STARTUP_MAX_FEEDFORWARD_TORQUE_NM:.2f} Nm，"
                    "启动位置容差 ±"
                    f"{CALIBRATED_STARTUP_POSITION_TOLERANCE_RAD:.2f} rad，"
                    "并独立监控实际速度。"
                )
            _user_print(f"输入 {MOTION_CONFIRMATION} 后按回车：")
            if sys.stdin.readline().strip() != MOTION_CONFIRMATION:
                _write_log(log_stream, "confirmation_rejected")
                _user_print("确认文字不匹配，未创建控制器。")
                return

            def stop_on_signal(signum: int, _frame: Any) -> None:
                raise KeyboardInterrupt(f"signal {signum}")

            descriptor = sys.stdin.fileno()
            original = termios.tcgetattr(descriptor)
            previous = signal.signal(signal.SIGTERM, stop_on_signal)
            with _filtered_vendor_output():
                try:
                    if travel_calibration is not None:
                        _user_print(
                            "正在以 DISABLE 守护临时启动厂家 SDK，"
                            "配置夹爪通信并读取当前位置……"
                        )
                        startup_guard = StartupEscDisableGuard(
                            open_socketcan(args.interface),
                            log_stream=log_stream,
                        )
                        startup_guard.start()
                        try:
                            _prime_gripper_esc_with_vendor_sdk(
                                args,
                                startup_guard,
                                log_stream=log_stream,
                            )
                        finally:
                            startup_guard.close_disabled()
                            startup_guard = None

                        _user_print(
                            "厂家临时发送线程已停止；正在从当前位置低速移动到"
                            "已标定启动参考端点……"
                        )
                        preposition_bus = open_socketcan(args.interface)
                        try:
                            preposition_result = preposition_calibrated_gripper(
                                preposition_bus,
                                travel_calibration,
                                log_stream=log_stream,
                                max_command_velocity_rad_s=float(
                                    args.startup_speed_rad_s
                                ),
                                timeout_s=float(args.startup_timeout_s),
                            )
                        finally:
                            preposition_bus.close()
                        if preposition_result["moved"]:
                            _user_print(
                                "已到启动参考端点："
                                f"{preposition_result['final_position_rad']:.4f} "
                                "rad。"
                            )
                        else:
                            _user_print("当前位置已在启动参考端点，无需移动。")

                    startup_guard = StartupEscDisableGuard(
                        open_socketcan(args.interface),
                        log_stream=log_stream,
                    )
                    startup_guard.start()
                    (
                        arm,
                        initial_target,
                        probe_results,
                        probe_fit,
                        physical_origin,
                    ) = _prepare_arm(
                        args, log_stream, startup_guard
                    )
                    startup_guard.enable_for_handoff()
                    handoff_state = _verify_zero_error_handoff(
                        arm.controller,
                        target_raw=initial_target,
                        log_stream=log_stream,
                        startup_guard=startup_guard,
                    )
                    handoff_position = _gripper_feedback_value(
                        handoff_state,
                        "position_raw",
                    )
                    if handoff_position is None:
                        raise JogSafetyError(
                            "Catch handoff position is unavailable for calibration."
                        )
                    command_feedback_offset = initial_target - handoff_position
                    open_target = float(args.open_target_raw)
                    close_target = float(args.close_target_raw)
                    if travel_calibration is not None:
                        open_target, close_target, transform = (
                            calibrated_catch_targets(
                                travel_calibration,
                                startup_physical_raw=physical_origin,
                                command_feedback_offset_raw=(
                                    command_feedback_offset
                                ),
                            )
                        )
                        _write_log(
                            log_stream,
                            "gripper_calibration_applied",
                            calibration_path=str(calibration_path),
                            startup_physical_raw=physical_origin,
                            command_feedback_offset_raw=command_feedback_offset,
                            **transform,
                        )
                        _user_print(
                            "标定端点已换算到本次 Catch 坐标："
                            f"O={open_target:.3f}，C={close_target:.3f} raw。"
                        )
                    startup_guard.complete_handoff()
                    tty.setcbreak(descriptor)
                    active_jog_runner = (
                        run_catch_jog if jog_runner is None else jog_runner
                    )
                    result = active_jog_runner(
                        arm,
                        initial_target_raw=initial_target,
                        open_target_raw=open_target,
                        close_target_raw=close_target,
                        open_rate_raw_s=float(args.open_rate_raw_s),
                        close_rate_raw_s=float(args.close_rate_raw_s),
                        refresh_hz=float(args.refresh_hz),
                        max_seconds=float(args.max_seconds),
                        log_stream=log_stream,
                        poll_key=lambda: _poll_stdin_key(sys.stdin),
                        calibration_path=(
                            None
                            if args.read_only_gripper_calibration
                            else calibration_path
                        ),
                        physical_origin_raw=physical_origin,
                        command_feedback_offset_raw=command_feedback_offset,
                        runtime_torque_slope_nm_per_raw=float(
                            probe_fit["slope_nm_per_raw"]
                        ),
                        travel_calibration=travel_calibration,
                        safety_check=startup_guard.check,
                        gripper_key_layout=args.gripper_key_layout,
                    )
                    result["runtime_direct_can_torque_monitoring"] = True
                    result["runtime_target_torque_limit_nm"] = (
                        RUNTIME_TARGET_COMMAND_TORQUE_NM
                    )
                    result["disabled_torque_map"] = probe_fit
                    result["disabled_torque_map_points"] = probe_results
                finally:
                    termios.tcsetattr(descriptor, termios.TCSADRAIN, original)
                    signal.signal(signal.SIGTERM, previous)
            _user_print(json.dumps(result, indent=2, ensure_ascii=False))
        except KeyboardInterrupt as exc:
            _write_log(log_stream, "interrupted", error=repr(exc))
            _user_print("\n收到中断，正在进入保护模式。")
        except JogSafetyError as exc:
            _write_log(log_stream, "safety_stop", error=repr(exc))
            _user_print(f"\n安全停止：{exc}")
            raise SystemExit(2) from None
        finally:
            if startup_guard is not None:
                startup_guard.close_disabled()
            if arm is not None:
                try:
                    backend.close(arm.controllers)
                except Exception as exc:
                    _write_log(log_stream, "sdk_close_failed", error=repr(exc))
            _write_log(log_stream, "closed")


def main(argv: Sequence[str] | None = None) -> None:
    """保留命令行兼容入口；新代码应调用 run_integrated_session。"""
    run_integrated_session(argv)


if __name__ == "__main__":
    main()
