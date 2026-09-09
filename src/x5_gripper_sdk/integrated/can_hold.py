"""Direct SocketCAN hold/nudge probe for one X5 gripper motor.

This tool never constructs vendor ``SingleArm`` and never calls ``set_catch``.
Preview mode only listens on can2.  Live mode enables one explicit motor ID,
holds the measured MIT position at vel=0, optionally slews a short close
nudge in the positive direction, then disables that motor.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import signal
import socket
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol, Sequence, TextIO

from .session_log import resolve_session_log_path
from .damiao_mit import (
    DISABLE_COMMAND,
    DM_J4310_LIMITS,
    ENABLE_COMMAND,
    MOTOR_PROFILES,
    STATUS_ENABLED,
    DamiaoMitFeedback,
    DamiaoMitLimits,
    is_system_command,
    looks_like_damiao_feedback,
    pack_mit_command,
    system_command_name,
    unpack_mit_command,
    unpack_mit_feedback,
)


ALLOWED_INTERFACE = "can2"
MOTION_CONFIRMATION = "1"
DEFAULT_OBSERVE_SECONDS = 2.0
DEFAULT_REFRESH_HZ = 50.0
RECV_POLL_SECONDS = 0.02
IDLE_LISTEN_SECONDS = 0.30
CLAIM_LISTEN_SECONDS = 0.20
MAX_DRAIN_FRAMES = 4096
MAX_RESIDUAL_COMMAND_KP = 1.0
RESIDUAL_HANDOFF_SECONDS = 0.05
FEEDBACK_WAIT_SECONDS = 0.50
FEEDBACK_WATCHDOG_SECONDS = 0.10
MAX_KP = 5.0
MAX_KD = 0.50
MAX_MOTOR_VELOCITY = 2.0
MAX_MOTOR_EXCURSION = 0.50
MAX_CONFIRM_DRIFT = 0.10
STABLE_VELOCITY = 0.05
MAX_NUDGE_CLOSE_RAD = 0.20
MAX_NUDGE_OPEN_RAD = 0.50
DEFAULT_NUDGE_SPEED = 0.15
MAX_NUDGE_SPEED = 0.25
MAX_NUDGE_VELOCITY = 0.50
MAX_TORQUE_NUDGE_VELOCITY = 2.00
MAX_NUDGE_TRACKING_ERROR = 0.12
MAX_NUDGE_WRONG_WAY = 0.05
MAX_NUDGE_OVERSHOOT = 0.08
MAX_POSITION_KP_ABS = 6.0
MAX_FF_TORQUE = 0.50
OURS_MIT_KEEP = 4
HYPOTHESIZED_ARM_CAN_IDS = frozenset(range(1, 8))
HYPOTHESIZED_X5_GRIPPER_CAN_ID = 8
HYPOTHESIZED_WRIST_CAN_IDS = frozenset({5, 6, 7})
CONFIRMED_WRIST_CAN_IDS = frozenset({7})
# Observed on can2 with the vendor SDK off: Damiao 5/6/7 plus EC-like 1/2/4.
KNOWN_IDLE_BUS_CAN_IDS = frozenset({0, 1, 2, 4, 5, 6, 7})

CAN_ERR_FLAG = 0x20000000
CAN_RTR_FLAG = 0x40000000
CAN_EFF_FLAG = 0x80000000
CAN_SFF_MASK = 0x7FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


class JogSafetyError(RuntimeError):
    """Raised when the direct-CAN probe refuses to send or stops a hold."""


class CanBus(Protocol):
    def send(self, can_id: int, data: bytes) -> None: ...

    def recv(self, timeout: float) -> tuple[int, bytes] | None: ...

    def close(self) -> None: ...


class SocketCanBus:
    """One Linux SocketCAN socket. Own-message loopback is disabled."""

    def __init__(self, sock: socket.socket):
        self._sock = sock

    def send(self, can_id: int, data: bytes) -> None:
        payload = bytes(data)
        if len(payload) > 8:
            raise ValueError("CAN payload cannot exceed 8 bytes.")
        dlc = len(payload)
        frame = struct.pack(
            CAN_FRAME_FMT,
            int(can_id) & CAN_SFF_MASK,
            dlc,
            payload.ljust(8, b"\x00"),
        )
        try:
            self._sock.send(frame)
        except OSError as exc:
            # 18:15 can2: 1 kHz Init overwrite plus Catch 200 Hz filled the
            # TX queue (ENOBUFS). Drop this frame; the next pump retries.
            if exc.errno == errno.ENOBUFS:
                return
            raise

    def recv(self, timeout: float) -> tuple[int, bytes] | None:
        self._sock.settimeout(max(float(timeout), 0.0))
        try:
            raw = self._sock.recv(CAN_FRAME_SIZE)
        except TimeoutError:
            return None
        except socket.timeout:
            return None
        except BlockingIOError:
            return None
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                return None
            raise
        if len(raw) < CAN_FRAME_SIZE:
            return None
        can_id, dlc, payload = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG):
            return None
        return int(can_id) & CAN_SFF_MASK, bytes(payload[: int(dlc)])

    def close(self) -> None:
        self._sock.close()


def open_socketcan(interface: str) -> SocketCanBus:
    if not hasattr(socket, "AF_CAN"):
        raise JogSafetyError(
            "This Python build has no AF_CAN; direct CAN requires Linux SocketCAN."
        )
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if hasattr(socket, "CAN_RAW_RECV_OWN_MSGS"):
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        sock.bind((interface,))
    except OSError as exc:
        raise JogSafetyError(f"Cannot bind SocketCAN {interface}: {exc}") from exc
    return SocketCanBus(sock)


def _can_id_int(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > CAN_SFF_MASK:
        raise argparse.ArgumentTypeError(
            f"CAN ID must be an 11-bit integer in [0, {CAN_SFF_MASK}]."
        )
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Listen on can2, or enable one Damiao MIT motor, hold its "
            "measured position, and optionally nudge it a short close step. "
            "Never constructs vendor SingleArm."
        )
    )
    parser.add_argument(
        "--interface",
        required=True,
        help=f"Exact SocketCAN name; this probe is limited to {ALLOWED_INTERFACE}.",
    )
    parser.add_argument(
        "--observe-seconds",
        type=float,
        default=DEFAULT_OBSERVE_SECONDS,
        help="Sniff duration, or live hold duration after enable.",
    )
    parser.add_argument("--refresh-hz", type=float, default=DEFAULT_REFRESH_HZ)
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="JSONL path. Default: app/logs/, keeping the 5 newest session logs.",
    )
    parser.add_argument("--enable-can-hold", action="store_true")
    parser.add_argument("--acknowledge-unverified-damiao-mit", action="store_true")
    parser.add_argument("--acknowledge-arm-not-commanded", action="store_true")
    parser.add_argument(
        "--acknowledge-possible-arm-motor-id",
        action="store_true",
        help="Required when --motor-can-id is in 1..7, the hypothesized arm IDs.",
    )
    parser.add_argument(
        "--acknowledge-confirmed-wrist-motor",
        action="store_true",
        help=(
            "Required when --motor-can-id is a visually confirmed wrist joint "
            "(currently 7 on this arm). Do not pass this to treat that ID as "
            "the gripper."
        ),
    )
    parser.add_argument(
        "--acknowledge-enabled-motor-traffic",
        action="store_true",
        help=(
            "Allow live hold while Damiao motors are still enabled and talking. "
            "The probe then disables only the chosen ID first."
        ),
    )
    parser.add_argument(
        "--acknowledge-residual-esc-mit",
        action="store_true",
        help=(
            "Allow live hold when the chosen ESC ID still has one repeated low-kp "
            "damping MIT after disable (the 5/6/7 leftover on this arm). Abort if "
            "that payload continues after this probe sends a hold command, or if "
            "kp is too high to treat as leftover damping."
        ),
    )
    parser.add_argument(
        "--motor-can-id",
        type=_can_id_int,
        default=None,
        help=(
            "ESC/CAN ID of the motor to enable and hold. Not guessed. "
            "Public ARX5 X5 configs use gripper ID 8; this 2023 external "
            "gripper is unverified."
        ),
    )
    parser.add_argument(
        "--feedback-can-id",
        type=_can_id_int,
        default=None,
        help="Optional MST_ID of feedback frames. If omitted, match payload motor id.",
    )
    parser.add_argument(
        "--motor-profile",
        choices=sorted(MOTOR_PROFILES),
        default=None,
        help="Damiao packing ranges. Live hold requires dm_j4310; no other profile is shipped.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        help=f"MIT kp. Required for live hold; 0 is damping-only. Capped at {MAX_KP}.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        help=f"MIT kd. Required for live hold; capped at {MAX_KD}.",
    )
    parser.add_argument(
        "--nudge-close-rad",
        type=float,
        default=None,
        help=(
            "After holding the measured position, slew this many MIT radians "
            f"in the close direction (position increases). Capped at "
            f"{MAX_NUDGE_CLOSE_RAD}. Mutually exclusive with --nudge-open-rad."
        ),
    )
    parser.add_argument(
        "--nudge-open-rad",
        type=float,
        default=None,
        help=(
            "Travel this many MIT encoder radians in the open direction "
            f"(position decreases). Capped at {MAX_NUDGE_OPEN_RAD}. "
            "Requires --kp 0 and negative --ff-torque; do not use position kp. "
            "Mutually exclusive with --nudge-close-rad."
        ),
    )
    parser.add_argument(
        "--nudge-speed",
        type=float,
        default=None,
        help=(
            "Position-slew close speed in MIT rad/s. Defaults to "
            f"{DEFAULT_NUDGE_SPEED}; capped at {MAX_NUDGE_SPEED}. "
            "Not used for torque open."
        ),
    )
    parser.add_argument(
        "--ff-torque",
        type=float,
        default=None,
        help=(
            "MIT torque feedforward in N·m. Vendor Catch on ID 8 used kp=0 "
            f"and about +0.13. Negative opens (encoder decreases). Capped at "
            f"±{MAX_FF_TORQUE}."
        ),
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.interface != ALLOWED_INTERFACE:
        raise ValueError(
            f"--interface must be exactly {ALLOWED_INTERFACE} for this CAN probe."
        )
    if any(char in args.interface for char in "^$*+?{}[]|()\\"):
        raise ValueError("--interface must be one exact SocketCAN name, not a pattern.")
    if not 10.0 <= float(args.refresh_hz) <= 100.0:
        raise ValueError("--refresh-hz must be in [10, 100] Hz.")
    if not args.enable_can_hold:
        if not 0.5 <= float(args.observe_seconds) <= 10.0:
            raise ValueError("--observe-seconds must be in [0.5, 10.0] for sniff mode.")
        return
    if not args.acknowledge_unverified_damiao_mit:
        raise ValueError(
            "--enable-can-hold also requires --acknowledge-unverified-damiao-mit."
        )
    if not args.acknowledge_arm_not_commanded:
        raise ValueError(
            "--enable-can-hold also requires --acknowledge-arm-not-commanded."
        )
    if args.motor_can_id is None:
        raise ValueError("--enable-can-hold requires an explicit --motor-can-id.")
    motor_id = int(args.motor_can_id)
    if motor_id < 1 or motor_id > 15:
        raise ValueError("--motor-can-id must be in [1, 15]; DaMiao feedback encodes 4 bits.")
    if motor_id in HYPOTHESIZED_ARM_CAN_IDS and not args.acknowledge_possible_arm_motor_id:
        raise ValueError(
            f"--motor-can-id {motor_id} is in hypothesized arm IDs 1..7; "
            "pass --acknowledge-possible-arm-motor-id only if this is the gripper."
        )
    if motor_id in CONFIRMED_WRIST_CAN_IDS and not args.acknowledge_confirmed_wrist_motor:
        raise ValueError(
            f"--motor-can-id {motor_id} is a confirmed wrist joint on this arm "
            "(17:01 close nudge moved the wrist, not the gripper). "
            "Do not use it as the gripper. Pass --acknowledge-confirmed-wrist-motor "
            "only if you intentionally want to command that wrist."
        )
    if not args.acknowledge_enabled_motor_traffic:
        raise ValueError(
            "--enable-can-hold also requires --acknowledge-enabled-motor-traffic "
            "on this arm: Damiao 5/6/7 stay enabled after the vendor SDK exits, "
            "so can2 will not go idle."
        )
    if not args.acknowledge_residual_esc_mit:
        raise ValueError(
            "--enable-can-hold also requires --acknowledge-residual-esc-mit "
            "on this arm: after disable, ESC IDs 5/6/7 still repeat a low-kp "
            "damping MIT. The probe will abort if that payload continues after "
            "the first hold command."
        )
    if args.motor_profile is None:
        raise ValueError(
            "--enable-can-hold requires --motor-profile dm_j4310; "
            "this probe will not guess packing ranges."
        )
    if args.kp is None or args.kd is None:
        raise ValueError(
            "--enable-can-hold requires explicit --kp and --kd; "
            "this probe will not guess MIT gains."
        )
    kp = float(args.kp)
    kd = float(args.kd)
    if not math.isfinite(kp) or kp < 0.0 or kp > MAX_KP:
        raise ValueError(f"--kp must be in [0, {MAX_KP}].")
    if not math.isfinite(kd) or kd < 0.0 or kd > MAX_KD:
        raise ValueError(f"--kd must be in [0, {MAX_KD}].")
    if not 0.1 <= float(args.observe_seconds) <= 2.0:
        raise ValueError("--observe-seconds must be in [0.1, 2.0] for live hold.")
    close_nudge = args.nudge_close_rad
    open_nudge = args.nudge_open_rad
    ff_torque = args.ff_torque
    if close_nudge is not None and open_nudge is not None:
        raise ValueError("Use only one of --nudge-close-rad and --nudge-open-rad.")
    if close_nudge is None and open_nudge is None:
        if args.nudge_speed is not None:
            raise ValueError(
                "--nudge-speed requires --nudge-close-rad or --nudge-open-rad."
            )
        if ff_torque is not None:
            raise ValueError(
                "--ff-torque requires --nudge-close-rad or --nudge-open-rad."
            )
        return
    if close_nudge is not None:
        nudge = float(close_nudge)
        if not math.isfinite(nudge) or nudge <= 0.0 or nudge > MAX_NUDGE_CLOSE_RAD:
            raise ValueError(
                f"--nudge-close-rad must be in (0, {MAX_NUDGE_CLOSE_RAD}]."
            )
        if kp > 0.0:
            if ff_torque is not None:
                raise ValueError("Position-slew close cannot be combined with --ff-torque.")
            speed = (
                DEFAULT_NUDGE_SPEED
                if args.nudge_speed is None
                else float(args.nudge_speed)
            )
            if not math.isfinite(speed) or speed <= 0.0 or speed > MAX_NUDGE_SPEED:
                raise ValueError(f"--nudge-speed must be in (0, {MAX_NUDGE_SPEED}].")
            return
        if ff_torque is None:
            raise ValueError(
                "A kp=0 close nudge requires --ff-torque > 0 "
                "(position kp slammed this gripper closed)."
            )
        torque = float(ff_torque)
        if not math.isfinite(torque) or torque <= 0.0 or torque > MAX_FF_TORQUE:
            raise ValueError(f"--ff-torque for close must be in (0, {MAX_FF_TORQUE}].")
        if args.nudge_speed is not None:
            raise ValueError("--nudge-speed does not apply to torque close.")
        return
    nudge = float(open_nudge)
    if not math.isfinite(nudge) or nudge <= 0.0 or nudge > MAX_NUDGE_OPEN_RAD:
        raise ValueError(f"--nudge-open-rad must be in (0, {MAX_NUDGE_OPEN_RAD}].")
    if kp > 0.0:
        raise ValueError(
            "Open cannot use --kp > 0: 17:14 position kp on ID 8 slammed "
            "more closed. Use --kp 0 and negative --ff-torque."
        )
    if ff_torque is None:
        raise ValueError(
            "Open requires --ff-torque < 0. Vendor Catch on ID 8 was kp=0 "
            "plus torque feedforward, not encoder position servo."
        )
    torque = float(ff_torque)
    if not math.isfinite(torque) or torque >= 0.0 or torque < -MAX_FF_TORQUE:
        raise ValueError(f"--ff-torque for open must be in [-{MAX_FF_TORQUE}, 0).")
    if args.nudge_speed is not None:
        raise ValueError("--nudge-speed does not apply to torque open.")


def _user_print(*values: Any, end: str = "\n") -> None:
    print(*values, file=sys.stderr, flush=True, end=end)


def _write_log(stream: TextIO, event: str, **values: Any) -> None:
    record = {
        "time": datetime.now().astimezone().isoformat(),
        "event": event,
        **values,
    }
    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    stream.flush()


def _decode_feedback(
    can_id: int,
    data: bytes,
    limits: DamiaoMitLimits,
) -> DamiaoMitFeedback | None:
    if not looks_like_damiao_feedback(data):
        return None
    try:
        return unpack_mit_feedback(can_id, data, limits)
    except ValueError:
        return None


def matching_feedback(
    can_id: int,
    data: bytes,
    *,
    motor_can_id: int,
    feedback_can_id: int | None,
    limits: DamiaoMitLimits,
) -> DamiaoMitFeedback | None:
    parsed = _decode_feedback(can_id, data, limits)
    if parsed is None:
        return None
    if parsed.motor_id != (int(motor_can_id) & 0xF):
        return None
    if feedback_can_id is not None and parsed.can_id != int(feedback_can_id):
        return None
    return parsed


def collect_frames(
    bus: CanBus,
    duration_s: float,
    *,
    recv_timeout_s: float = RECV_POLL_SECONDS,
) -> list[tuple[float, int, bytes]]:
    frames: list[tuple[float, int, bytes]] = []
    started = time.monotonic()
    while True:
        remaining = float(duration_s) - (time.monotonic() - started)
        if remaining <= 0.0:
            break
        received = bus.recv(min(recv_timeout_s, remaining))
        if received is None:
            continue
        frames.append((time.monotonic() - started, received[0], received[1]))
    return frames


def require_idle_bus(bus: CanBus, duration_s: float) -> None:
    frames = collect_frames(bus, duration_s)
    if frames:
        ids = sorted({can_id for _elapsed, can_id, _data in frames})
        raise JogSafetyError(
            "can2 already has traffic "
            f"({len(frames)} frames, IDs {ids}); another master such as "
            "SingleArm may own the bus. Stop the vendor SDK before live hold."
        )


def summarize_sniff(
    frames: Sequence[tuple[float, int, bytes]],
    limits: DamiaoMitLimits,
) -> list[dict[str, Any]]:
    grouped: dict[int, list[tuple[float, bytes]]] = defaultdict(list)
    for elapsed, can_id, data in frames:
        grouped[int(can_id)].append((elapsed, data))
    duration = 0.0
    if frames:
        duration = max(float(frames[-1][0]), 1e-6)
    summaries: list[dict[str, Any]] = []
    for can_id in sorted(grouped):
        items = grouped[can_id]
        last_data = items[-1][1]
        motor_counts: dict[str, int] = defaultdict(int)
        feedback_count = 0
        system_count = 0
        other_count = 0
        unique_payloads = {payload for _elapsed, payload in items}
        last_feedback: DamiaoMitFeedback | None = None
        for _elapsed, payload in items:
            command_name = system_command_name(payload)
            if command_name is not None:
                system_count += 1
                continue
            parsed = _decode_feedback(can_id, payload, limits)
            if parsed is None:
                other_count += 1
                continue
            feedback_count += 1
            motor_counts[str(parsed.motor_id)] += 1
            last_feedback = parsed
        summary: dict[str, Any] = {
            "can_id": can_id,
            "frame_count": len(items),
            "hz": round(len(items) / duration, 1),
            "unique_payloads": len(unique_payloads),
            "feedback_count": feedback_count,
            "system_command_count": system_count,
            "other_count": other_count,
            "last_hex": last_data.hex(),
        }
        if motor_counts:
            summary["payload_motor_id_counts"] = dict(sorted(motor_counts.items()))
        command_name = system_command_name(last_data)
        if command_name is not None:
            summary["last_kind"] = "system_command"
            summary["last_system_command"] = command_name
        elif last_feedback is not None and looks_like_damiao_feedback(last_data):
            summary["last_kind"] = "damiao_feedback"
            summary["payload_motor_id"] = last_feedback.motor_id
            summary["status"] = last_feedback.status
            summary["position"] = last_feedback.position
            summary["velocity"] = last_feedback.velocity
            summary["torque"] = last_feedback.torque
        else:
            summary["last_kind"] = "undecoded"
            if len(last_data) == 8 and not looks_like_damiao_feedback(last_data):
                try:
                    command = unpack_mit_command(last_data, limits)
                except ValueError:
                    command = None
                if command is not None:
                    summary["last_kind"] = (
                        "repeated_mit_command"
                        if len(unique_payloads) == 1
                        else "mit_command"
                    )
                    summary["command_position"] = command["position"]
                    summary["command_velocity"] = command["velocity"]
                    summary["command_kp"] = command["kp"]
                    summary["command_kd"] = command["kd"]
        summaries.append(summary)
    return summaries


def damiao_feedback_motors(
    frames: Sequence[tuple[float, int, bytes]],
    limits: DamiaoMitLimits,
) -> list[dict[str, Any]]:
    latest: dict[int, DamiaoMitFeedback] = {}
    counts: dict[int, int] = defaultdict(int)
    for _elapsed, can_id, data in frames:
        parsed = _decode_feedback(can_id, data, limits)
        if parsed is None:
            continue
        latest[parsed.motor_id] = parsed
        counts[parsed.motor_id] += 1
    motors: list[dict[str, Any]] = []
    for motor_id in sorted(latest):
        parsed = latest[motor_id]
        motors.append(
            {
                "motor_id": motor_id,
                "feedback_can_id": parsed.can_id,
                "frame_count": counts[motor_id],
                "status": parsed.status,
                "enabled": parsed.status == STATUS_ENABLED,
                "position": parsed.position,
                "velocity": parsed.velocity,
            }
        )
    return motors


def require_stationary_feedback(
    feedback: DamiaoMitFeedback,
    *,
    reference_position: float | None = None,
) -> float:
    if not math.isfinite(feedback.position) or not math.isfinite(feedback.velocity):
        raise JogSafetyError("Motor feedback is not finite.")
    if abs(feedback.velocity) > STABLE_VELOCITY:
        raise JogSafetyError(
            "Motor feedback is not stationary immediately before CAN hold."
        )
    if reference_position is not None:
        drift = abs(feedback.position - float(reference_position))
        if drift > MAX_CONFIRM_DRIFT:
            raise JogSafetyError(
                "Motor drifted from "
                f"{reference_position:.6f} to {feedback.position:.6f} "
                "while waiting for confirmation; refusing to hold the drifted position."
            )
    return float(feedback.position)


def is_esc_mit_command(can_id: int, data: bytes, motor_can_id: int) -> bool:
    if int(can_id) != int(motor_can_id):
        return False
    payload = bytes(data)
    if len(payload) != 8:
        return False
    if is_system_command(payload) or looks_like_damiao_feedback(payload):
        return False
    return True


def drain_queued_frames(bus: CanBus, *, max_frames: int = MAX_DRAIN_FRAMES) -> int:
    """Drop frames already sitting in the SocketCAN receive buffer."""
    drained = 0
    while drained < int(max_frames):
        received = bus.recv(0.0)
        if received is None:
            break
        drained += 1
    return drained


def summarize_mit_payloads(
    payloads: Sequence[bytes],
    limits: DamiaoMitLimits,
) -> list[dict[str, Any]]:
    counts: dict[bytes, int] = {}
    order: list[bytes] = []
    for raw in payloads:
        payload = bytes(raw)
        if payload not in counts:
            order.append(payload)
            counts[payload] = 0
        counts[payload] += 1
    summaries: list[dict[str, Any]] = []
    for payload in order:
        item: dict[str, Any] = {"hex": payload.hex(), "count": counts[payload]}
        try:
            item.update(unpack_mit_command(payload, limits))
        except ValueError:
            pass
        summaries.append(item)
    return summaries


def claim_motor(
    bus: CanBus,
    motor_can_id: int,
    *,
    listen_seconds: float,
    log_stream: TextIO,
    limits: DamiaoMitLimits = DM_J4310_LIMITS,
    allow_residual_esc_mit: bool = False,
    allow_sdk_sibling: bool = False,
    feedback_can_id: int | None = None,
) -> bytes | None:
    """Disable one ESC ID. Return leftover low-kp MIT, or abort if it looks live.

    ``allow_sdk_sibling`` is for the same process that already owns can2 through
    ``SingleArm``. Status 5 may keep writing Catch MIT on ID 8; unique payloads
    can change. Still refuse kp >= 1.0.
    """
    drained = drain_queued_frames(bus)
    bus.send(int(motor_can_id), DISABLE_COMMAND)
    frames = collect_frames(bus, listen_seconds)
    foreign = [
        bytes(frame[2])
        for frame in frames
        if is_esc_mit_command(frame[1], frame[2], motor_can_id)
    ]
    residual_summaries = summarize_mit_payloads(foreign, limits)
    last_status = None
    last_position = None
    for _elapsed, can_id, data in frames:
        parsed = matching_feedback(
            can_id,
            data,
            motor_can_id=motor_can_id,
            feedback_can_id=feedback_can_id,
            limits=limits,
        )
        if parsed is not None:
            last_status = parsed.status
            last_position = parsed.position
    _write_log(
        log_stream,
        "can_hold_claim_disable",
        motor_can_id=int(motor_can_id),
        drained_queued_frames=int(drained),
        residual_mit_count=len(foreign),
        residual_mit=residual_summaries,
        last_feedback_status=last_status,
        last_feedback_position=last_position,
    )
    if not foreign:
        return None
    unique = {bytes(payload) for payload in foreign}
    decoded = residual_summaries[0] if residual_summaries else {}
    kp = float(decoded.get("kp", math.inf))
    residual = next(iter(unique))
    peak_kp = 0.0
    for item in residual_summaries:
        item_kp = float(item.get("kp", 0.0))
        if math.isfinite(item_kp):
            peak_kp = max(peak_kp, item_kp)
    if allow_sdk_sibling:
        if peak_kp >= MAX_RESIDUAL_COMMAND_KP:
            raise JogSafetyError(
                f"After disable, CAN ID {motor_can_id} still has MIT-like frames "
                f"with kp={peak_kp:.2f}. Refusing to fight a position servo on "
                "the gripper while the vendor SDK also owns can2."
            )
        return residual
    low_kp_repeat = (
        len(unique) == 1
        and math.isfinite(kp)
        and kp < MAX_RESIDUAL_COMMAND_KP
    )
    if allow_residual_esc_mit and low_kp_repeat:
        return residual
    if not allow_residual_esc_mit:
        extra = (
            " If this is the leftover low-kp damping stream, pass "
            "--acknowledge-residual-esc-mit."
        )
    elif len(unique) != 1:
        extra = " Multiple MIT payloads after disable is a live commander."
    else:
        extra = (
            f" Residual kp={kp:.2f} is too stiff to treat as leftover damping."
        )
    raise JogSafetyError(
        f"After disable, CAN ID {motor_can_id} still has MIT-like frames "
        f"({len(foreign)} in {listen_seconds:.2f} s; drained {drained} "
        f"queued first; {residual_summaries}). Another commander is writing "
        f"this motor; refusing to fight it.{extra}"
    )


def _wait_for_matching_feedback(
    bus: CanBus,
    *,
    motor_can_id: int,
    feedback_can_id: int | None,
    limits: DamiaoMitLimits,
    timeout_s: float,
) -> DamiaoMitFeedback:
    deadline = time.monotonic() + float(timeout_s)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise JogSafetyError(
                f"No MIT feedback matched motor CAN ID {motor_can_id} "
                f"within {timeout_s:.2f} s after enable."
            )
        received = bus.recv(min(RECV_POLL_SECONDS, remaining))
        if received is None:
            continue
        parsed = matching_feedback(
            received[0],
            received[1],
            motor_can_id=motor_can_id,
            feedback_can_id=feedback_can_id,
            limits=limits,
        )
        if parsed is not None:
            return parsed


def commanded_nudge_position(
    start: float,
    *,
    delta: float,
    speed: float,
    elapsed: float,
) -> float:
    """Slew from start by signed ``delta`` (positive=close, negative=open)."""
    if float(delta) == 0.0:
        return float(start)
    traveled = min(max(float(elapsed), 0.0) * float(speed), abs(float(delta)))
    return float(start) + math.copysign(traveled, float(delta))


def _remember_our_mit(sent: list[bytes], payload: bytes) -> None:
    sent.append(bytes(payload))
    extra = len(sent) - OURS_MIT_KEEP
    if extra > 0:
        del sent[:extra]


def run_can_hold(
    bus: CanBus,
    *,
    motor_can_id: int,
    kp: float,
    kd: float,
    limits: DamiaoMitLimits,
    observe_seconds: float,
    refresh_hz: float,
    log_stream: TextIO,
    feedback_can_id: int | None = None,
    idle_listen_seconds: float = 0.0,
    claim_listen_seconds: float = 0.0,
    reference_position: float | None = None,
    allow_residual_esc_mit: bool = False,
    nudge_close_rad: float = 0.0,
    nudge_open_rad: float = 0.0,
    nudge_speed: float = DEFAULT_NUDGE_SPEED,
    ff_torque: float = 0.0,
) -> dict[str, Any]:
    """Enable one motor, hold, optionally nudge, then disable it."""
    residual_esc_mit: bytes | None = None
    if idle_listen_seconds > 0.0:
        require_idle_bus(bus, idle_listen_seconds)
    if claim_listen_seconds > 0.0:
        residual_esc_mit = claim_motor(
            bus,
            motor_can_id,
            listen_seconds=claim_listen_seconds,
            log_stream=log_stream,
            limits=limits,
            allow_residual_esc_mit=allow_residual_esc_mit,
            feedback_can_id=feedback_can_id,
        )
        if residual_esc_mit is not None:
            decoded = unpack_mit_command(residual_esc_mit, limits)
            _user_print(
                f"失能后 CAN ID {motor_can_id} 上仍有重复阻尼 MIT "
                f"(hex={residual_esc_mit.hex()}, kp={decoded['kp']:.2f}, "
                f"kd={decoded['kd']:.2f})。先发保持帧；若该旧 payload 在 "
                f"{RESIDUAL_HANDOFF_SECONDS:.2f} s 后仍出现则退出。"
            )
    period = 1.0 / float(refresh_hz)
    close_span = max(float(nudge_close_rad), 0.0)
    open_span = max(float(nudge_open_rad), 0.0)
    delta = close_span - open_span
    torque_cmd = float(ff_torque)
    use_torque = abs(torque_cmd) > 0.0 and float(kp) == 0.0
    speed = float(nudge_speed) if delta != 0.0 and not use_torque else DEFAULT_NUDGE_SPEED
    slew_seconds = (abs(delta) / speed) if delta != 0.0 and not use_torque else 0.0
    run_seconds = slew_seconds + float(observe_seconds)
    velocity_limit = (
        MAX_TORQUE_NUDGE_VELOCITY
        if use_torque
        else (MAX_NUDGE_VELOCITY if delta != 0.0 else MAX_MOTOR_VELOCITY)
    )
    our_mits: list[bytes] = []
    hold_command = None
    enabled = False
    samples: list[dict[str, Any]] = []
    max_velocity_seen = 0.0
    max_excursion_seen = 0.0
    max_tracking_error = 0.0
    try:
        bus.send(int(motor_can_id), ENABLE_COMMAND)
        enabled = True
        _write_log(log_stream, "can_hold_enable", motor_can_id=int(motor_can_id))
        before = _wait_for_matching_feedback(
            bus,
            motor_can_id=motor_can_id,
            feedback_can_id=feedback_can_id,
            limits=limits,
            timeout_s=FEEDBACK_WAIT_SECONDS,
        )
        hold_pos = require_stationary_feedback(
            before, reference_position=reference_position
        )
        if float(kp) > 0.0 and abs(hold_pos) > MAX_POSITION_KP_ABS:
            raise JogSafetyError(
                f"Refusing position kp={float(kp):.2f} at encoder "
                f"{hold_pos:.6f}: |pos| > {MAX_POSITION_KP_ABS:.1f} rad. "
                "On this gripper, kp>0 slams more closed (17:07 kp=5, "
                "17:14 kp=1). Use --kp 0 and --ff-torque."
            )
        target_pos = hold_pos + delta
        command_pos = hold_pos
        command_torque = torque_cmd if use_torque else 0.0
        torque_reached = False
        hold_command = pack_mit_command(
            command_pos, 0.0, kp, kd, command_torque, limits
        )
        _remember_our_mit(our_mits, hold_command)
        drain_queued_frames(bus)
        bus.send(int(motor_can_id), hold_command)
        hold_sent_at = time.monotonic()
        _write_log(
            log_stream,
            "can_hold_entered",
            motor_can_id=int(motor_can_id),
            feedback_can_id=feedback_can_id,
            kp=float(kp),
            kd=float(kd),
            pos=hold_pos,
            target_pos=target_pos,
            nudge_close_rad=close_span,
            nudge_open_rad=open_span,
            nudge_speed=speed if delta != 0.0 and not use_torque else 0.0,
            vel=0.0,
            torque=command_torque,
            ff_torque=torque_cmd,
            use_torque=use_torque,
            motor_profile=limits.name,
            hold_command_hex=hold_command.hex(),
            residual_esc_mit=(
                residual_esc_mit.hex() if residual_esc_mit is not None else None
            ),
            before={
                "can_id": before.can_id,
                "motor_id": before.motor_id,
                "status": before.status,
                "position": before.position,
                "velocity": before.velocity,
                "torque": before.torque,
            },
        )
        started = hold_sent_at
        last_refresh = started
        last_feedback_at = started
        last_feedback_hex: str | None = None
        latest = before
        while True:
            now = time.monotonic()
            elapsed = now - started
            if use_torque:
                command_pos = hold_pos
                measured_now = float(latest.position)
                if not torque_reached and (
                    (delta < 0.0 and measured_now <= target_pos)
                    or (delta > 0.0 and measured_now >= target_pos)
                ):
                    torque_reached = True
                command_torque = 0.0 if torque_reached else torque_cmd
            else:
                command_pos = commanded_nudge_position(
                    hold_pos, delta=delta, speed=speed, elapsed=elapsed
                )
                command_torque = 0.0
            if now - last_refresh >= period:
                hold_command = pack_mit_command(
                    command_pos, 0.0, kp, kd, command_torque, limits
                )
                _remember_our_mit(our_mits, hold_command)
                bus.send(int(motor_can_id), hold_command)
                last_refresh = now
            received = bus.recv(RECV_POLL_SECONDS)
            if received is not None:
                parsed = matching_feedback(
                    received[0],
                    received[1],
                    motor_can_id=motor_can_id,
                    feedback_can_id=feedback_can_id,
                    limits=limits,
                )
                if parsed is not None:
                    latest = parsed
                    last_feedback_at = now
                    last_feedback_hex = bytes(received[1]).hex()
                elif hold_command is not None and is_esc_mit_command(
                    received[0], received[1], motor_can_id
                ):
                    payload = bytes(received[1])
                    if payload not in our_mits:
                        if (
                            residual_esc_mit is not None
                            and payload == residual_esc_mit
                            and (now - hold_sent_at) <= RESIDUAL_HANDOFF_SECONDS
                        ):
                            pass
                        elif (
                            residual_esc_mit is not None
                            and payload == residual_esc_mit
                        ):
                            raise JogSafetyError(
                                f"Residual damping MIT continued on CAN ID "
                                f"{motor_can_id} after this probe sent a hold "
                                "command; another commander is still writing "
                                "this motor."
                            )
                        else:
                            raise JogSafetyError(
                                f"CAN ID {motor_can_id} received a MIT-like "
                                "frame that this probe did not send; another "
                                "commander is fighting the hold."
                            )
            if now - last_feedback_at > FEEDBACK_WATCHDOG_SECONDS:
                raise JogSafetyError(
                    "Lost motor feedback for more than "
                    f"{FEEDBACK_WATCHDOG_SECONDS:.2f} s; disabling."
                )
            velocity = abs(float(latest.velocity))
            measured = float(latest.position)
            excursion = abs(measured - hold_pos)
            tracking = abs(measured - command_pos)
            max_velocity_seen = max(max_velocity_seen, velocity)
            max_excursion_seen = max(max_excursion_seen, excursion)
            max_tracking_error = max(max_tracking_error, tracking)
            sample = {
                "elapsed_s": elapsed,
                "can_id": latest.can_id,
                "status": latest.status,
                "position": measured,
                "command_position": command_pos,
                "command_torque": command_torque,
                "velocity": float(latest.velocity),
                "torque": float(latest.torque),
                "excursion": excursion,
                "tracking_error": tracking,
            }
            samples.append(sample)
            _write_log(log_stream, "can_hold_sample", **sample)
            if not math.isfinite(velocity) or velocity > velocity_limit:
                raise JogSafetyError(
                    f"CAN hold exceeded {velocity_limit:.3f} rad/s on the motor "
                    f"(pos={measured:.6f}, vel={float(latest.velocity):.3f}, "
                    f"hex={last_feedback_hex})."
                )
            if delta > 0.0:
                if measured < hold_pos - MAX_NUDGE_WRONG_WAY:
                    raise JogSafetyError(
                        "Close nudge moved the motor open (position decreased) "
                        f"from {hold_pos:.6f} to {measured:.6f}; refusing."
                    )
                if measured > target_pos + MAX_NUDGE_OVERSHOOT:
                    raise JogSafetyError(
                        "Close nudge overshot the target by more than "
                        f"{MAX_NUDGE_OVERSHOOT:.3f} rad."
                    )
                if not use_torque and tracking > MAX_NUDGE_TRACKING_ERROR:
                    raise JogSafetyError(
                        "Close nudge tracking error exceeded "
                        f"{MAX_NUDGE_TRACKING_ERROR:.3f} rad."
                    )
            elif delta < 0.0:
                if measured > hold_pos + MAX_NUDGE_WRONG_WAY:
                    raise JogSafetyError(
                        "Open nudge moved the motor closed (position increased) "
                        f"from {hold_pos:.6f} to {measured:.6f}; refusing."
                    )
                if measured < target_pos - MAX_NUDGE_OVERSHOOT:
                    raise JogSafetyError(
                        "Open nudge overshot the target by more than "
                        f"{MAX_NUDGE_OVERSHOOT:.3f} rad."
                    )
                if not use_torque and tracking > MAX_NUDGE_TRACKING_ERROR:
                    raise JogSafetyError(
                        "Open nudge tracking error exceeded "
                        f"{MAX_NUDGE_TRACKING_ERROR:.3f} rad."
                    )
            elif not math.isfinite(excursion) or excursion > MAX_MOTOR_EXCURSION:
                raise JogSafetyError(
                    "CAN hold let the motor leave the measured position by more than "
                    f"{MAX_MOTOR_EXCURSION:.3f} rad."
                )
            if elapsed >= run_seconds:
                break
    finally:
        if enabled:
            bus.send(int(motor_can_id), DISABLE_COMMAND)
            _write_log(log_stream, "can_hold_disable", motor_can_id=int(motor_can_id))

    opened_toward_zero = any(
        float(sample["position"]) < hold_pos - 0.05 for sample in samples
    )
    final_position = float(samples[-1]["position"]) if samples else hold_pos
    moved_closed = bool(delta > 0.0 and final_position >= hold_pos + 0.5 * delta)
    moved_opened = bool(delta < 0.0 and final_position <= hold_pos + 0.5 * delta)
    sent_ids = {int(motor_can_id)}
    result = {
        "passed": True,
        "observe_seconds": observe_seconds,
        "sample_count": len(samples),
        "motor_can_id": int(motor_can_id),
        "feedback_can_id": int(samples[-1]["can_id"]) if samples else feedback_can_id,
        "kp": float(kp),
        "kd": float(kd),
        "hold_position": hold_pos,
        "target_position": target_pos,
        "nudge_close_rad": close_span,
        "nudge_open_rad": open_span,
        "nudge_speed": speed if delta != 0.0 and not use_torque else 0.0,
        "ff_torque": torque_cmd,
        "max_velocity": max_velocity_seen,
        "max_excursion": max_excursion_seen,
        "max_tracking_error": max_tracking_error,
        "opened_toward_zero": opened_toward_zero,
        "moved_closed": moved_closed,
        "moved_opened": moved_opened,
        "final_position": final_position,
        "commanded_can_ids": sorted(sent_ids),
        "vendor_sdk_used": False,
    }
    _write_log(log_stream, "can_hold_passed", **result)
    return result


def classify_sniff_ids(summaries: Sequence[dict[str, Any]]) -> dict[str, Any]:
    seen = {int(item["can_id"]) for item in summaries}
    new_ids = sorted(seen - KNOWN_IDLE_BUS_CAN_IDS)
    return {
        "seen_ids": sorted(seen),
        "known_idle_bus_ids": sorted(KNOWN_IDLE_BUS_CAN_IDS),
        "new_ids": new_ids,
        "missing_known_ids": sorted(KNOWN_IDLE_BUS_CAN_IDS - seen),
        "gripper_candidate_ids": new_ids,
    }


def run_sniff(
    bus: CanBus,
    *,
    observe_seconds: float,
    limits: DamiaoMitLimits,
    log_stream: TextIO,
) -> dict[str, Any]:
    frames = collect_frames(bus, observe_seconds)
    summaries = summarize_sniff(frames, limits)
    motors = damiao_feedback_motors(frames, limits)
    classified = classify_sniff_ids(summaries)
    result = {
        "passed": True,
        "observe_seconds": observe_seconds,
        "frame_count": len(frames),
        "ids": summaries,
        "damiao_feedback_motors": motors,
        "vendor_sdk_used": False,
        "sent_frames": False,
        **classified,
    }
    _write_log(log_stream, "can_sniff_complete", **result)
    return result


def main(argv: Sequence[str] | None = None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    limits = MOTOR_PROFILES.get(args.motor_profile, DM_J4310_LIMITS)
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    log_path = resolve_session_log_path(
        args.log,
        f"x5_2023_gripper_can_hold_{args.interface}_{timestamp}.jsonl",
    )

    bus: SocketCanBus | None = None
    with log_path.open("a", encoding="utf-8") as log_stream:
        _write_log(
            log_stream,
            "start",
            interface=args.interface,
            can_hold_enabled=bool(args.enable_can_hold),
            hold_measured_position_only=(
                args.nudge_close_rad is None and args.nudge_open_rad is None
            ),
            nudge_close_rad=args.nudge_close_rad,
            nudge_open_rad=args.nudge_open_rad,
            ff_torque=args.ff_torque,
            vendor_sdk_used=False,
            hypothesized_x5_gripper_can_id=HYPOTHESIZED_X5_GRIPPER_CAN_ID,
            hypothesized_arm_can_ids=sorted(HYPOTHESIZED_ARM_CAN_IDS),
            hypothesized_wrist_can_ids=sorted(HYPOTHESIZED_WRIST_CAN_IDS),
            confirmed_wrist_can_ids=sorted(CONFIRMED_WRIST_CAN_IDS),
            max_kp=MAX_KP,
            max_kd=MAX_KD,
            max_motor_velocity=MAX_MOTOR_VELOCITY,
            max_motor_excursion=MAX_MOTOR_EXCURSION,
            motor_profile=limits.name,
        )
        try:
            bus = open_socketcan(args.interface)
            _user_print(f"接口：{args.interface}；日志：{log_path}")
            _user_print("未创建厂家 SingleArm，也不会调用 set_catch。")
            if not args.enable_can_hold:
                _user_print(
                    f"只听 {args.observe_seconds:.1f} s，不发任何帧。"
                    "要找夹爪 ID：先让本探针听着，再在另一终端开厂家线性脚本；"
                    "Init/Catch 时多出来的 CAN ID 才是夹爪候选。"
                    "听完立刻关掉 SingleArm。不要对 5/6/7 做闭合步进。"
                )
                result = run_sniff(
                    bus,
                    observe_seconds=float(args.observe_seconds),
                    limits=limits,
                    log_stream=log_stream,
                )
                _user_print(json.dumps(result, indent=2, ensure_ascii=False))
                new_ids = result.get("new_ids") or []
                if new_ids:
                    _user_print(
                        "相对本机空闲总线多出来的 CAN ID："
                        + ", ".join(str(can_id) for can_id in new_ids)
                        + "。这些才是夹爪候选；下一步只对其中一个做保持，不要动 5/6/7。"
                    )
                else:
                    _user_print(
                        "没有出现空闲总线以外的 CAN ID。"
                        "夹爪在 SDK 关掉后不上报；请在厂家 Catch 运行时再听一次。"
                    )
                if result["frame_count"] == 0:
                    _user_print(
                        "没有收到帧。这在 SDK 未运行时是正常的。"
                        "公开 ARX5 X5 配置里夹爪 ESC ID 常为 8，本机外置夹爪尚未在总线上确认。"
                    )
                    return
                motors = result.get("damiao_feedback_motors") or []
                if motors:
                    enabled = [
                        motor["motor_id"]
                        for motor in motors
                        if motor.get("enabled")
                    ]
                    _user_print(
                        "达妙反馈电机号："
                        + ", ".join(
                            f"{motor['motor_id']}@"
                            f"can{motor['feedback_can_id']} "
                            f"pos={motor['position']:.3f}"
                            for motor in motors
                        )
                    )
                    if 8 not in {motor["motor_id"] for motor in motors}:
                        _user_print("未见到 ESC ID 8。不要用公开 X5 的夹爪号 8。")
                    if enabled:
                        _user_print(
                            "仍使能的电机："
                            + ", ".join(str(motor_id) for motor_id in enabled)
                            + "。本机没有厂家进程时这些流量也不会自己停。"
                            "要对其中一台做保持，加 --acknowledge-enabled-motor-traffic "
                            "和 --acknowledge-residual-esc-mit；"
                            "探针会抽空缓冲后失能该 ID。失能后若仍是同一条低 kp 阻尼 MIT，"
                            "会先发保持帧；旧 payload 还在就退出。不碰 5/6。"
                        )
                else:
                    _user_print(
                        "有总线流量，但没有可解码的达妙反馈。"
                        "1/2/4 很可能是另一类关节协议，不要把它们的 hex 当成夹爪角度。"
                    )
                return
            if not sys.stdin.isatty():
                raise JogSafetyError("Direct CAN hold requires an interactive terminal.")

            def stop_on_signal(signum: int, _frame: Any) -> None:
                raise KeyboardInterrupt(f"signal {signum}")

            previous_sigterm = signal.signal(signal.SIGTERM, stop_on_signal)
            try:
                close_span = (
                    0.0
                    if args.nudge_close_rad is None
                    else float(args.nudge_close_rad)
                )
                open_span = (
                    0.0
                    if args.nudge_open_rad is None
                    else float(args.nudge_open_rad)
                )
                speed = (
                    DEFAULT_NUDGE_SPEED
                    if args.nudge_speed is None
                    else float(args.nudge_speed)
                )
                ff_torque = (
                    0.0 if args.ff_torque is None else float(args.ff_torque)
                )
                _user_print(
                    "\n该探针只使能一个电机，并把当前 MIT 测量值作为起点（vel=0）。"
                )
                _user_print(
                    f"motor-can-id={int(args.motor_can_id)}，"
                    f"profile={limits.name}，kp={float(args.kp)}，kd={float(args.kd)}。"
                )
                if int(args.motor_can_id) in CONFIRMED_WRIST_CAN_IDS:
                    _user_print(
                        f"CAN ID {int(args.motor_can_id)} 已确认是腕，不是夹爪。"
                    )
                if close_span > 0.0 and abs(ff_torque) > 0.0:
                    _user_print(
                        f"闭合用前馈力矩 {ff_torque:+.3f} N·m，行程 "
                        f"{close_span:.3f} rad，最多 "
                        f"{float(args.observe_seconds):.2f} s。"
                        "位置增大是闭合。kp 必须为 0。若腕或臂在动，急停。"
                    )
                elif close_span > 0.0:
                    _user_print(
                        f"闭合步进 {close_span:.3f} rad（约 {math.degrees(close_span):.1f}°），"
                        f"速度 {speed:.3f} rad/s，到位后再保持 "
                        f"{float(args.observe_seconds):.2f} s。"
                        "位置增大是闭合。夹爪应略闭合；若腕或臂在动，急停。"
                    )
                elif open_span > 0.0:
                    _user_print(
                        f"张开用前馈力矩 {ff_torque:+.3f} N·m，行程 "
                        f"{open_span:.3f} rad（约 {math.degrees(open_span):.1f}°），"
                        f"最多 {float(args.observe_seconds):.2f} s。"
                        "位置减小是张开。kp=0，不会命令到 0。"
                        "夹爪应张开；若往更闭合冲或腕在动，急停。"
                    )
                else:
                    _user_print("只保持当前位置，不闭合也不张开。")
                _user_print("不会命令六轴。机械臂必须固定；通信丢失后该电机会卸力。")
                _user_print(
                    "先抽空 socket 里积着的旧帧，再只失能这一台。"
                    "若命令口上仍是同一条低 kp 阻尼 MIT，会尝试接管；"
                    "发出保持后该旧帧还在，或出现别的 MIT，就直接退出。"
                    "不会命令 5/6。"
                )
                _user_print("确认工作区清空、急停可触达。")
                _user_print(f"输入 {MOTION_CONFIRMATION} 后按回车：")
                if sys.stdin.readline().strip() != MOTION_CONFIRMATION:
                    _write_log(log_stream, "confirmation_rejected")
                    _user_print("确认文字不匹配，未使能电机。")
                    return
                result = run_can_hold(
                    bus,
                    motor_can_id=int(args.motor_can_id),
                    kp=float(args.kp),
                    kd=float(args.kd),
                    limits=limits,
                    observe_seconds=float(args.observe_seconds),
                    refresh_hz=float(args.refresh_hz),
                    log_stream=log_stream,
                    feedback_can_id=args.feedback_can_id,
                    idle_listen_seconds=0.0,
                    claim_listen_seconds=CLAIM_LISTEN_SECONDS,
                    allow_residual_esc_mit=bool(args.acknowledge_residual_esc_mit),
                    nudge_close_rad=close_span,
                    nudge_open_rad=open_span,
                    nudge_speed=speed,
                    ff_torque=ff_torque,
                )
            finally:
                signal.signal(signal.SIGTERM, previous_sigterm)
            _user_print(json.dumps(result, indent=2, ensure_ascii=False))
            if close_span > 0.0:
                if result.get("moved_closed"):
                    _user_print(
                        "位置已向闭合方向增大。请看夹爪是否略闭合；腕不该动。"
                    )
                else:
                    _user_print(
                        "位置没有明显跟着闭合命令走，可能卡住或 ID 不是夹爪。"
                    )
            elif open_span > 0.0:
                if result.get("moved_opened"):
                    _user_print(
                        "位置已向张开方向减小。请看夹爪是否张开；腕不该动。"
                    )
                else:
                    hold = float(result["hold_position"])
                    final = float(result["final_position"])
                    at_torque_cap = (
                        abs(float(result.get("ff_torque") or 0.0))
                        >= MAX_FF_TORQUE - 1e-9
                    )
                    if final < hold - 0.002:
                        if at_torque_cap:
                            _user_print(
                                "力矩已到硬限，编码器只小幅减小。不要重复同一命令，也不要加 kp。"
                                "若爪仍明显全闭，需要先提高 --ff-torque 上限再试。"
                            )
                        else:
                            _user_print(
                                "力矩方向对（编码器在减小），但行程不够。"
                                "可略增 |--ff-torque|，不要改符号、不要加 kp。"
                            )
                    elif final > hold + 0.002:
                        _user_print(
                            "编码器在增大，张开力矩符号可能反了。不要再加大负力矩。"
                        )
                    else:
                        _user_print(
                            "编码器几乎没动。可略增 |--ff-torque|，不要加 kp。"
                        )
            elif result.get("opened_toward_zero"):
                _user_print(
                    "注意：观察窗内电机仍向 0 方向离开保持点，说明保持增益不足或 ID 不是夹爪。"
                )
            else:
                _user_print(
                    "观察窗内电机未明显开向 0。请对照日志确认动的是夹爪而不是关节。"
                )
        except KeyboardInterrupt as exc:
            _write_log(log_stream, "interrupted", error=repr(exc))
            _user_print("\n收到中断，正在失能该电机。")
        except JogSafetyError as exc:
            _write_log(log_stream, "safety_stop", error=repr(exc))
            _user_print(f"\n安全停止：{exc}")
            raise SystemExit(2) from None
        except Exception as exc:
            _write_log(log_stream, "error", error=repr(exc))
            _user_print(f"\n未处理异常：{exc}")
            raise
        finally:
            if bus is not None:
                try:
                    bus.close()
                except OSError:
                    pass
            _write_log(log_stream, "closed")


if __name__ == "__main__":
    main()
